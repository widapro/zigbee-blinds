#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "os/os_mbuf.h"
#include "ble_ota.h"
#include "diagnostics.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_id.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "storage.h"

// 3 minutes in milliseconds
#define SAFETY_TIMER_PERIOD_MS (3 * 60 * 1000)
#define OTA_CONN_ITVL_MIN 6
#define OTA_CONN_ITVL_MAX 12
#define OTA_CONN_LATENCY 0
#define OTA_CONN_SUPERVISION_TIMEOUT 400
#define OTA_CONN_EVENT_LEN_MIN 0
#define OTA_CONN_EVENT_LEN_MAX 0
#define OTA_DATA_LEN_OCTETS 251
#define OTA_DATA_LEN_TIME_US 2120

static const char *TAG = "BLE_OTA";

// OTA Service UUID: 8e400001-f315-4f60-9fb8-838830daea50
static const ble_uuid128_t ota_service_uuid =
    BLE_UUID128_INIT(0x50, 0xea, 0xda, 0x30, 0x88, 0x83, 0xb8, 0x9f, 0x60, 0x4f,
                     0x15, 0xf3, 0x01, 0x00, 0x40, 0x8e);

// OTA Control Characteristic UUID: 8e400002-f315-4f60-9fb8-838830daea50
static const ble_uuid128_t ota_control_uuid =
    BLE_UUID128_INIT(0x50, 0xea, 0xda, 0x30, 0x88, 0x83, 0xb8, 0x9f, 0x60, 0x4f,
                     0x15, 0xf3, 0x02, 0x00, 0x40, 0x8e);

// OTA Data Characteristic UUID: 8e400003-f315-4f60-9fb8-838830daea50
static const ble_uuid128_t ota_data_uuid =
    BLE_UUID128_INIT(0x50, 0xea, 0xda, 0x30, 0x88, 0x83, 0xb8, 0x9f, 0x60, 0x4f,
                     0x15, 0xf3, 0x03, 0x00, 0x40, 0x8e);

static uint8_t ble_addr_type;
static TimerHandle_t safety_timer = NULL;
static esp_ota_handle_t update_handle = 0;
static const esp_partition_t *update_partition = NULL;
static bool update_in_progress = false;
static bool reboot_pending = false;

// Forward declarations
static void ble_app_advertise(void);

static int set_ota_ble_static_addr(void) {
  uint8_t mac[8];
  uint8_t addr[6];

  esp_read_mac(mac, ESP_MAC_IEEE802154);
  addr[0] = mac[2] ^ 0x4f;
  addr[1] = mac[3] ^ 0x54;
  addr[2] = mac[4] ^ 0x41;
  addr[3] = mac[5] ^ 0x32;
  addr[4] = mac[6] ^ 0x56;
  addr[5] = (mac[7] ^ 0x02) | 0xc0;

  int rc = ble_hs_id_set_rnd(addr);
  ESP_LOGI(TAG, "OTA BLE static random address set: rc=%d", rc);
  return rc;
}

static void request_fast_ble_params(uint16_t conn_handle) {
  struct ble_gap_upd_params params = {
      .itvl_min = OTA_CONN_ITVL_MIN,
      .itvl_max = OTA_CONN_ITVL_MAX,
      .latency = OTA_CONN_LATENCY,
      .supervision_timeout = OTA_CONN_SUPERVISION_TIMEOUT,
      .min_ce_len = OTA_CONN_EVENT_LEN_MIN,
      .max_ce_len = OTA_CONN_EVENT_LEN_MAX,
  };

  int rc = ble_gap_update_params(conn_handle, &params);
  ESP_LOGI(TAG, "Connection parameter update request: rc=%d", rc);

  rc = ble_gap_set_data_len(conn_handle, OTA_DATA_LEN_OCTETS,
                            OTA_DATA_LEN_TIME_US);
  ESP_LOGI(TAG, "Data length update request: rc=%d", rc);

  rc = ble_gap_set_prefered_le_phy(conn_handle, BLE_GAP_LE_PHY_2M_MASK,
                                   BLE_GAP_LE_PHY_2M_MASK,
                                   BLE_GAP_LE_PHY_CODED_ANY);
  ESP_LOGI(TAG, "2M PHY update request: rc=%d", rc);
}

// Safety timer callback - reboots device if OTA takes too long or is abandoned
static void safety_timer_callback(TimerHandle_t xTimer) {
  ESP_LOGW(TAG,
           "Safety timer expired! No OTA activity. Rebooting to Zigbee...");
  storage_save_pending_restart_cause(RESTART_CAUSE_OTA_TIMEOUT);
  esp_restart();
}

static void reset_safety_timer(void) {
  if (safety_timer) {
    xTimerReset(safety_timer, 0);
  }
}

static void stop_safety_timer(void) {
  if (safety_timer) {
    xTimerStop(safety_timer, 0);
  }
}

// An interrupted upload must release its OTA handle before another can begin.
static void abort_update(void) {
  if (update_in_progress) {
    esp_ota_abort(update_handle);
  }
  update_in_progress = false;
  update_handle = 0;
  update_partition = NULL;
}

// Return from the GATT callback before rebooting so the final write is acknowledged.
static void ota_restart_task(void *arg) {
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(1000));
  storage_save_pending_restart_cause(RESTART_CAUSE_OTA_SUCCESS);
  esp_restart();
}

// GATT Server Callbacks
static int device_write(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (reboot_pending) {
    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
  }

  // NimBLE can split one ATT value across several mbufs.
  uint8_t data[512];
  uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
  if (len == 0 || len > sizeof(data) ||
      os_mbuf_copydata(ctxt->om, 0, len, data) != 0) {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }

  // Data Characteristic (Firmware Chunks)
  if (ble_uuid_cmp(ctxt->chr->uuid, &ota_data_uuid.u) == 0) {
    diagnostics_mark_stage(DIAG_STAGE_BLE_OTA_WRITE);
    if (!update_in_progress) {
      ESP_LOGE(TAG, "Data received but update not started via control char");
      return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    reset_safety_timer(); // Keep alive

    esp_err_t err = esp_ota_write(update_handle, data, len);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error writing OTA data: %s", esp_err_to_name(err));
      abort_update();
      return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
  }

  // Control Characteristic (Start/End)
  if (ble_uuid_cmp(ctxt->chr->uuid, &ota_control_uuid.u) == 0) {
    if (len != 1)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t cmd = data[0];
    ESP_LOGI(TAG, "Control command received: %d", cmd);

    if (cmd == 0) { // Start OTA
      abort_update();
      ESP_LOGI(TAG, "Starting OTA update...");
      update_partition = esp_ota_get_next_update_partition(NULL);
      if (update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition found!");
        return BLE_ATT_ERR_UNLIKELY;
      }
      ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%lx",
               update_partition->subtype, update_partition->address);

      esp_err_t err =
          esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        return BLE_ATT_ERR_UNLIKELY;
      }
      update_in_progress = true;
      reset_safety_timer();

    } else if (cmd == 1) { // End OTA
      ESP_LOGI(TAG, "Finishing OTA update...");
      if (!update_in_progress)
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

      esp_err_t err = esp_ota_end(update_handle);
      // esp_ota_end consumes the handle even when validation fails.
      update_in_progress = false;
      update_handle = 0;
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return BLE_ATT_ERR_UNLIKELY;
      }

      err = esp_ota_set_boot_partition(update_partition);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s",
                 esp_err_to_name(err));
        return BLE_ATT_ERR_UNLIKELY;
      }

      ESP_LOGI(TAG, "OTA Success! Rebooting...");
      reboot_pending = true;
      if (xTaskCreate(ota_restart_task, "ota_restart", 2048, NULL, 1, NULL) != pdPASS) {
        reboot_pending = false;
        return BLE_ATT_ERR_INSUFFICIENT_RES;
      }
      stop_safety_timer();

    } else {
      return BLE_ATT_ERR_UNLIKELY; // Unknown command
    }
    return 0;
  }

  return BLE_ATT_ERR_UNLIKELY;
}

// Define the GATT service
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &ota_service_uuid.u,
     .characteristics =
         (struct ble_gatt_chr_def[]){
             {
                 .uuid = &ota_control_uuid.u,
                 .access_cb = device_write,
                 .flags = BLE_GATT_CHR_F_WRITE,
             },
             {
                 .uuid = &ota_data_uuid.u,
                 .access_cb = device_write,
                 .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
             },
             {0},
         }},
    {0},
};

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    ESP_LOGI(TAG, "BLE Client Connected: status=%d handle=%d",
             event->connect.status, event->connect.conn_handle);
    // Keep the safety window open while the browser negotiates the update.
    reset_safety_timer();
    if (event->connect.status == 0) {
      request_fast_ble_params(event->connect.conn_handle);
    } else {
      ble_app_advertise();
    }
    break;

  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "BLE Client Disconnected");
    abort_update();
    // Return to advertising and let the safety timer reboot if the update was
    // abandoned.
    reset_safety_timer();
    ble_app_advertise();
    break;

  case BLE_GAP_EVENT_CONN_UPDATE:
    ESP_LOGI(TAG, "Connection updated: status=%d handle=%d",
             event->conn_update.status, event->conn_update.conn_handle);
    break;

  case BLE_GAP_EVENT_MTU:
    ESP_LOGI(TAG, "MTU updated: handle=%d mtu=%d", event->mtu.conn_handle,
             event->mtu.value);
    break;

  case BLE_GAP_EVENT_DATA_LEN_CHG:
    ESP_LOGI(TAG, "Data length updated: handle=%d tx=%d/%dus rx=%d/%dus",
             event->data_len_chg.conn_handle, event->data_len_chg.max_tx_octets,
             event->data_len_chg.max_tx_time,
             event->data_len_chg.max_rx_octets,
             event->data_len_chg.max_rx_time);
    break;
  }
  return 0;
}

static void ble_app_advertise(void) {
  struct ble_hs_adv_fields fields;
  memset(&fields, 0, sizeof(fields));

  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  // Build unique name with last 4 hex digits of factory MAC (same as Zigbee
  // IEEE)
  uint8_t mac[8];
  esp_read_mac(mac, ESP_MAC_IEEE802154);
  static char name[20];
  snprintf(name, sizeof(name), "Blinds-%02X%02X", mac[6], mac[7]);

  fields.name = (uint8_t *)name;
  fields.name_len = strlen(name);
  fields.name_is_complete = 1;

  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "Error setting advertisement data: rc=%d", rc);
    return;
  }

  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                         ble_gap_event, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG, "Error starting advertisement: rc=%d", rc);
  } else {
    ESP_LOGI(TAG, "BLE Advertising started: %s", name);
  }
}

static void ble_host_task(void *param) {
  ESP_LOGI(TAG, "BLE Host Task Started");
  nimble_port_run();
  nimble_port_freertos_deinit();
}

static void on_sync(void) {
  int rc = set_ota_ble_static_addr();
  if (rc != 0) {
    ESP_LOGW(TAG, "Unable to set OTA BLE random address, using default");
  }
  ble_hs_util_ensure_addr(rc == 0 ? 1 : 0);
  ble_hs_id_infer_auto(0, &ble_addr_type);
  ble_app_advertise();
  ESP_LOGI(TAG, "BLE Advertising Started - Waiting for client...");
}

void start_ble_ota_mode(void) {
  diagnostics_mark_stage(DIAG_STAGE_BLE_OTA_START);
  ESP_LOGI(TAG, "Initializing Safety Timer (%d ms)...", SAFETY_TIMER_PERIOD_MS);
  safety_timer =
      xTimerCreate("ota_safety", pdMS_TO_TICKS(SAFETY_TIMER_PERIOD_MS), pdFALSE,
                   NULL, safety_timer_callback);
  if (safety_timer) {
    xTimerStart(safety_timer, 0);
  }

  ESP_LOGI(TAG, "Initializing NimBLE Stack...");
  ESP_ERROR_CHECK(nimble_port_init());

  ble_hs_cfg.sync_cb = on_sync;

  ble_gatts_count_cfg(gatt_svcs);
  ble_gatts_add_svcs(gatt_svcs);

  nimble_port_freertos_init(ble_host_task);
}
