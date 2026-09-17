/**
 * @file    main.cpp
 * @brief   ESP32 Communication Bridge — Bluetooth SPP + WiFi OTA → STM32.
 *
 * Architecture:
 *   - BT SPP server: accepts connections from phone apps, receives firmware
 *     files and text commands.
 *   - WiFi HTTP client: downloads firmware from a URL.
 *   - WiFi SoftAP + HTTP server: serves a phone-friendly firmware upload page.
 *   - MQTT client: publishes the sensor snapshot and accepts control commands
 *     on a public sandbox broker (broker.emqx.io), namespaced by MAC address.
 *   - UART link: communicates with STM32 bootloader/application using the
 *     shared protocol (115200 baud, framed, CRC-32).
 *   - Orchestrator: manages firmware staging (SPIFFS) and transfer state.
 *
 * Text commands over BT SPP:
 *   OTA <url>          Download firmware from URL, then transfer to STM32
 *   FW <ver>,<size>,<crc32>  Begin a Base64 Bluetooth firmware push
 *   DATA <offset>,<base64>   Append one acknowledged firmware block
 *   VERIFY             Verify the staged image size and CRC-32
 *   SEND               Transfer the staged firmware to STM32
 *   WIFI <ssid>,<pass> Configure WiFi credentials (NVS) and reconnect
 *   VERSION            Query STM32 firmware version
 *   STATUS             Show current bridge status
 *   RESET              Software reset the ESP32
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_spp_api.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"   /* esp_bt_gap_set_device_name (IDF 6) */

#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "mbedtls/base64.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "web_ota_page.h"

/*---------------------------------------------------------------------------
 * Shared protocol (C-compatible, included as extern "C")
 *---------------------------------------------------------------------------*/

extern "C" {
#include "../../shared/protocol.h"
}

/*---------------------------------------------------------------------------
 * Constants
 *---------------------------------------------------------------------------*/

static const char *TAG = "bridge";

/* UART to STM32 */
#define UART_STM32_NUM          UART_NUM_2
#define UART_STM32_TXD          17
#define UART_STM32_RXD          16
#define UART_STM32_RTS          UART_PIN_NO_CHANGE
#define UART_STM32_CTS          UART_PIN_NO_CHANGE
#define UART_STM32_BAUD         115200
#define UART_STM32_BUF_SIZE     2048

/* BT SPP */
#define SPP_SERVER_NAME         "STM32-OTA-Bridge"
#define SPP_TASK_STACK          8192
#define SPP_TASK_PRIO           5

/* OTA transfer */
#define OTA_CHUNK_SIZE          1024

/* SPIFFS firmware storage */
#define FW_FILE_PATH            "/spiffs/fw.bin"
#define FW_FILE_MAX_SIZE        (54 * 1024)  /* Max app size */
#define STM32_APP_BASE          0x08002000U
#define STM32_APP_END           0x0800F800U
#define STM32_RAM_BASE          0x20000000U
#define STM32_RAM_END           0x20005000U

/* WiFi OTA download buffer */
#define HTTP_DOWNLOAD_BUF_SIZE  4096

/* Web OTA server */
#define WEB_OTA_TASK_STACK      8192
#define WEB_OTA_TASK_PRIO       5
#define SENSOR_POLL_TASK_STACK  5120
#define SENSOR_POLL_TASK_PRIO   4
#define SENSOR_POLL_PERIOD_MS   1000U
#define SENSOR_CACHE_STALE_MS   5000U
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID            "STM32-OTA-Bridge"
#endif
#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD        "stm32ota"
#endif

/* WiFi credentials — override at compile time via -D WIFI_SSID/-D WIFI_PASSWORD,
 * or at runtime via the "WIFI <ssid>,<pass>" BT command (stored in NVS). */
#ifndef WIFI_SSID
#define WIFI_SSID     "YOUR_SSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_PASSWORD"
#endif

/* MQTT — public EMQX sandbox broker (no account, no TLS). Fine for a personal
 * demo device; migrating to a private broker later only needs a new URI,
 * since client ID and topics are already namespaced by MAC address. */
#ifndef MQTT_BROKER_URI
#define MQTT_BROKER_URI  "mqtt://broker.emqx.io:1883"
#endif

/*---------------------------------------------------------------------------
 * Global state
 *---------------------------------------------------------------------------*/

static uint32_t        g_spp_handle = 0;
static QueueHandle_t   g_spp_queue;       /* received SPP data (ptr + len) */

/* SPP data item carried through the queue. Keeps the real length so binary
 * firmware chunks with embedded NUL bytes survive intact. */
typedef struct {
    uint8_t *data;
    size_t   len;
} SppData_t;
static bool            g_fw_staged = false;
static bool            g_fw_receiving = false;
static uint32_t        g_fw_size = 0;
static uint32_t        g_fw_expected_size = 0;
static uint32_t        g_fw_version = 0;
static uint32_t        g_fw_crc32 = 0;
static uint32_t        g_fw_running_crc = 0;
static FILE           *g_fw_file = NULL;
static SemaphoreHandle_t g_ota_mutex = NULL;
static volatile bool   g_ota_running = false;
static volatile uint32_t g_ota_progress_sent = 0;
static volatile uint32_t g_ota_progress_total = 0;

typedef enum {
    WEB_OTA_IDLE = 0,
    WEB_OTA_UPLOADING,
    WEB_OTA_STAGED,
    WEB_OTA_TRANSFERRING,
    WEB_OTA_COMPLETE,
    WEB_OTA_FAILED
} WebOtaState_t;

static volatile WebOtaState_t g_web_ota_state = WEB_OTA_IDLE;
static httpd_handle_t g_http_server = NULL;
static SemaphoreHandle_t g_sensor_cache_mutex = NULL;
static SensorSnapshot_t g_sensor_cache = {};
static bool g_sensor_cache_valid = false;
static uint32_t g_sensor_cache_received_ms = 0;

/* MQTT — topics are namespaced with the last 3 MAC bytes so this device does
 * not collide with other clients on the shared public broker. */
static esp_mqtt_client_handle_t g_mqtt_client = NULL;
static volatile bool   g_mqtt_connected = false;
static char             g_mqtt_topic_sensors[48];
static char             g_mqtt_topic_control[48];

/* Bluetooth SPP is a byte stream, so a terminal command can arrive in
 * several ESP_SPP_DATA_IND_EVT callbacks. Accumulate it through CR/LF. */
static char             g_cmd_buf[256];
static size_t           g_cmd_len = 0;

/*---------------------------------------------------------------------------
 * Forward declarations
 *---------------------------------------------------------------------------*/

static void uart_stm32_init(void);
static void bt_spp_init(void);
static void wifi_init_sta(void);
static void wifi_connect(const char *ssid, const char *password);
static void web_server_start(void);
static void mqtt_init(void);
static void mqtt_publish_sensors(void);
static int format_sensor_json(char *json, size_t json_size);
static bool build_control_message(const char *body, char *msg, size_t msg_size);
static bool send_control_message(const char *msg, ProtoFrame_t *resp,
                                 const char **error_message);
static void sensor_poll_task(void *pv);
static void bt_recv_task(void *pv);
static void bt_spp_print(const char *msg);
static bool download_firmware_http(const char *url);
static bool transfer_to_stm32(void);
static bool stage_firmware_data(const uint8_t *data, size_t len);
static bool verify_staged_firmware(void);
static bool transfer_to_stm32_impl(void);

/*---------------------------------------------------------------------------
 * UART to STM32
 *---------------------------------------------------------------------------*/

static void uart_stm32_init(void) {
    uart_config_t uart_config = {};
    uart_config.baud_rate  = UART_STM32_BAUD;
    uart_config.data_bits  = UART_DATA_8_BITS;
    uart_config.parity     = UART_PARITY_DISABLE;
    uart_config.stop_bits  = UART_STOP_BITS_1;
    uart_config.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;

    ESP_ERROR_CHECK(uart_driver_install(UART_STM32_NUM,
                                         UART_STM32_BUF_SIZE,
                                         UART_STM32_BUF_SIZE,
                                         0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_STM32_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_STM32_NUM,
                                  UART_STM32_TXD, UART_STM32_RXD,
                                  UART_STM32_RTS, UART_STM32_CTS));
}

/**
 * @brief Send a protocol frame to STM32 over UART.
 */
static bool stm32_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    uint8_t buf[PROTO_MAX_FRAME];
    uint16_t total = proto_build_frame(buf, sizeof(buf), cmd, payload, len);
    if (total == 0) {
        ESP_LOGE(TAG, "Could not build UART frame cmd=0x%02X len=%u", cmd, len);
        return false;
    }

    int written = uart_write_bytes(UART_STM32_NUM, buf, total);
    if (written != total) {
        ESP_LOGE(TAG, "UART write short: cmd=0x%02X expected=%u got=%d",
                 cmd, total, written);
        return false;
    }

    return uart_wait_tx_done(UART_STM32_NUM, pdMS_TO_TICKS(3000)) == ESP_OK;
}

/**
 * @brief Wait for a specific command from STM32 with timeout.
 * @return true if matching frame received, false on timeout.
 */
static bool stm32_wait_cmd(uint8_t expected_cmd, ProtoFrame_t *out,
                            uint32_t timeout_ms) {
    ProtoParser_t parser;
    proto_parser_init(&parser);

    uint8_t byte;
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - start < timeout_ms) {
        int len = uart_read_bytes(UART_STM32_NUM, &byte, 1,
                                  pdMS_TO_TICKS(50));
        if (len > 0) {
            const ProtoFrame_t *f = proto_parser_feed(&parser, byte);
            if (f != NULL) {
                if (f->cmd == expected_cmd) {
                    if (out) memcpy(out, f, sizeof(ProtoFrame_t));
                    return true;
                }
                if (f->cmd == CMD_NAK) {
                    /* NAK received — caller should handle */
                    if (out) memcpy(out, f, sizeof(ProtoFrame_t));
                    return false;
                }
            }
        }
    }

    return false;
}

/*---------------------------------------------------------------------------
 * Sensor snapshot cache
 *---------------------------------------------------------------------------*/

static uint32_t bridge_uptime_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void sensor_poll_task(void *pv) {
    (void)pv;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t consecutive_failures = 0;

    for (;;) {
        SensorSnapshot_t snapshot = {};
        bool received = false;

        if (g_ota_mutex != NULL &&
            xSemaphoreTake(g_ota_mutex, 0) == pdTRUE) {
            ProtoFrame_t response;
            received = stm32_send_frame(CMD_GET_SENSOR_SNAPSHOT, NULL, 0) &&
                       stm32_wait_cmd(CMD_SENSOR_SNAPSHOT_RSP, &response,
                                      600) &&
                       response.len == sizeof(snapshot);
            if (received) {
                memcpy(&snapshot, response.payload, sizeof(snapshot));
            }
            xSemaphoreGive(g_ota_mutex);
        }

        if (received &&
            xSemaphoreTake(g_sensor_cache_mutex,
                           pdMS_TO_TICKS(50)) == pdTRUE) {
            g_sensor_cache = snapshot;
            g_sensor_cache_received_ms = bridge_uptime_ms();
            g_sensor_cache_valid = true;
            xSemaphoreGive(g_sensor_cache_mutex);
            mqtt_publish_sensors();

            if (consecutive_failures != 0U) {
                ESP_LOGI(TAG, "STM32 sensor stream recovered");
            }
            consecutive_failures = 0;
        } else {
            consecutive_failures++;
            if (consecutive_failures == 1U ||
                (consecutive_failures % 10U) == 0U) {
                ESP_LOGW(TAG, "STM32 sensor poll failed (%lu)",
                         consecutive_failures);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_POLL_PERIOD_MS));
    }
}

/*---------------------------------------------------------------------------
 * SPIFFS (firmware storage)
 *---------------------------------------------------------------------------*/

static void spiffs_init(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path              = "/spiffs",
        .partition_label        = "storage",
        .max_files              = 5,
        .format_if_mount_failed = true
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    size_t total = 0, used = 0;
    esp_spiffs_info(conf.partition_label, &total, &used);
    ESP_LOGI(TAG, "SPIFFS: total=%d, used=%d", total, used);
}

/*---------------------------------------------------------------------------
 * Firmware staging integrity
 *---------------------------------------------------------------------------*/

static void discard_incoming_firmware(void) {
    if (g_fw_file != NULL) {
        fclose(g_fw_file);
        g_fw_file = NULL;
    }
    g_fw_staged = false;
    g_fw_receiving = false;
    g_fw_size = 0;
    g_fw_expected_size = 0;
    g_fw_crc32 = 0;
    g_fw_running_crc = 0;
    unlink(FW_FILE_PATH);
}

static bool stage_firmware_data(const uint8_t *data, size_t len) {
    if (!g_fw_receiving || g_fw_file == NULL || g_fw_expected_size == 0 ||
        g_fw_size > g_fw_expected_size ||
        len > (size_t)(g_fw_expected_size - g_fw_size)) {
        ESP_LOGE(TAG, "Invalid firmware chunk: received=%u staged=%lu expected=%lu",
                 (unsigned)len, g_fw_size, g_fw_expected_size);
        discard_incoming_firmware();
        return false;
    }

    size_t written = fwrite(data, 1, len, g_fw_file);
    if (written != len) {
        ESP_LOGE(TAG, "SPIFFS write short: expected=%u got=%u",
                 (unsigned)len, (unsigned)written);
        discard_incoming_firmware();
        return false;
    }

    g_fw_running_crc = proto_crc32(data, len, g_fw_running_crc);
    g_fw_size += (uint32_t)len;
    if (g_fw_size == g_fw_expected_size) {
        if (fclose(g_fw_file) != 0) {
            g_fw_file = NULL;
            ESP_LOGE(TAG, "Failed to close staged firmware file");
            discard_incoming_firmware();
            return false;
        }
        g_fw_file = NULL;
        g_fw_receiving = false;
    }
    return true;
}

static bool verify_staged_firmware(void) {
    if (g_fw_receiving || g_fw_size == 0 ||
        g_fw_size != g_fw_expected_size) {
        ESP_LOGE(TAG, "Firmware stage is incomplete");
        return false;
    }

    struct stat st;
    if (stat(FW_FILE_PATH, &st) != 0 || st.st_size < 0 ||
        (uint32_t)st.st_size != g_fw_size) {
        ESP_LOGE(TAG, "Firmware file size mismatch");
        return false;
    }

    FILE *f = fopen(FW_FILE_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open staged firmware for verification");
        return false;
    }

    uint8_t buffer[256];
    uint32_t crc = 0U;
    uint32_t total = 0U;
    while (true) {
        size_t count = fread(buffer, 1, sizeof(buffer), f);
        if (count == 0) {
            break;
        }
        total += (uint32_t)count;
        crc = proto_crc32(buffer, count, crc);
    }
    bool read_ok = !ferror(f);
    fclose(f);

    if (!read_ok || total != g_fw_size || crc != g_fw_crc32) {
        ESP_LOGE(TAG, "Firmware verification failed: size=%lu/%lu crc=0x%08lX/0x%08lX",
                 total, g_fw_size, crc, g_fw_crc32);
        return false;
    }

    /* Reject a bootloader or unrelated binary before it reaches the STM32.
     * A valid Cortex-M application starts with its initial stack pointer and
     * a Thumb reset vector inside this project's application partition. */
    f = fopen(FW_FILE_PATH, "rb");
    if (!f) {
        return false;
    }
    uint32_t initial_sp = 0;
    uint32_t reset_vector = 0;
    bool vector_read_ok = fread(&initial_sp, 1, sizeof(initial_sp), f) == sizeof(initial_sp) &&
                          fread(&reset_vector, 1, sizeof(reset_vector), f) == sizeof(reset_vector);
    fclose(f);

    uint32_t reset_address = reset_vector & ~1U;
    if (!vector_read_ok ||
        initial_sp < STM32_RAM_BASE || initial_sp > STM32_RAM_END ||
        (reset_vector & 1U) == 0 ||
        reset_address < STM32_APP_BASE || reset_address >= STM32_APP_END) {
        ESP_LOGE(TAG, "Invalid application vectors: SP=0x%08lX reset=0x%08lX",
                 initial_sp, reset_vector);
        return false;
    }

    return true;
}

/*---------------------------------------------------------------------------
 * Bluetooth SPP
 *---------------------------------------------------------------------------*/

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "SPP init");
        ESP_ERROR_CHECK(esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE,
                                          ESP_SPP_ROLE_SLAVE,
                                          0, SPP_SERVER_NAME));
        break;

    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                                     ESP_BT_GENERAL_DISCOVERABLE));
            ESP_LOGI(TAG, "SPP server discoverable: %s", SPP_SERVER_NAME);
        } else {
            ESP_LOGE(TAG, "Failed to start SPP server: status=%d",
                     param->start.status);
        }
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "SPP client connected, handle=%u", param->srv_open.handle);
        g_spp_handle = param->srv_open.handle;
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "SPP closed, handle=%u", param->close.handle);
        g_spp_handle = 0;
        g_cmd_len = 0;
        if (g_fw_receiving) {
            ESP_LOGW(TAG, "Discarding incomplete firmware after SPP disconnect");
            g_fw_staged = false;
            g_fw_receiving = false;
            g_fw_size = 0;
            g_fw_expected_size = 0;
        }
        break;

    case ESP_SPP_DATA_IND_EVT:
        /* Forward received BT data to bt_recv_task via queue.
         * Carries length explicitly: binary firmware chunks may contain
         * embedded NUL bytes, so strlen() is not usable. */
        if (param->data_ind.len > 0) {
            uint8_t *copy = (uint8_t *)malloc(param->data_ind.len + 1);
            if (copy) {
                memcpy(copy, param->data_ind.data, param->data_ind.len);
                copy[param->data_ind.len] = '\0';
                SppData_t item = { .data = copy, .len = param->data_ind.len };
                if (xQueueSend(g_spp_queue, &item, 0) != pdPASS) {
                    ESP_LOGW(TAG, "SPP RX queue full; dropping %u bytes",
                             (unsigned)item.len);
                    free(copy);
                }
            }
        }
        break;

    default:
        break;
    }
}

static void bt_spp_init(void) {
    g_spp_queue = xQueueCreate(64, sizeof(SppData_t));

    /* sdkconfig selects Classic-only SPP, so BLE RAM is not needed. */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_callback));

    esp_spp_cfg_t spp_cfg = {};
    spp_cfg.mode = ESP_SPP_MODE_CB;
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));

    /* Set discoverable device name (IDF 6 API) */
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(SPP_SERVER_NAME));
}

/**
 * @brief Send data back to the connected SPP client.
 */
static void bt_spp_send(const uint8_t *data, size_t len) {
    if (g_spp_handle != 0) {
        esp_spp_write(g_spp_handle, len, (uint8_t *)data);
    }
}

/**
 * @brief Send a text string back to the SPP client.
 */
static void bt_spp_print(const char *msg) {
    bt_spp_send((const uint8_t *)msg, strlen(msg));
}

/*---------------------------------------------------------------------------
 * BT receive task — processes incoming data from phone
 *---------------------------------------------------------------------------*/

static void bt_recv_task(void *pv) {
    SppData_t item;

    for (;;) {
        if (xQueueReceive(g_spp_queue, &item, portMAX_DELAY) == pdPASS) {
            uint8_t *data = item.data;
            size_t   len  = item.len;

            /* Reassemble a terminal command. SPP does not preserve the
             * write boundaries of the PC serial terminal. */
            if (g_cmd_len > 0 ||
                (len > 0 && ((data[0] >= 'A' && data[0] <= 'Z') ||
                             (data[0] >= 'a' && data[0] <= 'z')))) {
                bool command_ready = false;

                for (size_t i = 0; i < len; i++) {
                    if (data[i] == '\r' || data[i] == '\n') {
                        if (g_cmd_len > 0) {
                            uint8_t *line = (uint8_t *)malloc(g_cmd_len + 1);
                            if (line) {
                                memcpy(line, g_cmd_buf, g_cmd_len);
                                line[g_cmd_len] = '\0';
                                free(data);
                                data = line;
                                len = g_cmd_len;
                                command_ready = true;
                            }
                            g_cmd_len = 0;
                        }
                        break;
                    }

                    if (data[i] < 0x20 || data[i] > 0x7E ||
                        g_cmd_len >= sizeof(g_cmd_buf) - 1) {
                        g_cmd_len = 0;
                        break;
                    }
                    g_cmd_buf[g_cmd_len++] = (char)data[i];
                }

                if (!command_ready) {
                    free(data);
                    continue;
                }
            }

            /* Simple text command detection: starts with ASCII letter */
            if (len > 0 && ((data[0] >= 'A' && data[0] <= 'Z') ||
                            (data[0] >= 'a' && data[0] <= 'z'))) {
                char cmd[256] = {0};
                size_t n = len < sizeof(cmd) - 1 ? len : sizeof(cmd) - 1;
                memcpy(cmd, data, n);
                cmd[n] = '\0';

                if (strncmp(cmd, "DATA ", 5) != 0 && strncmp(cmd, "data ", 5) != 0) {
                    ESP_LOGI(TAG, "BT cmd: %s", cmd);
                }

                if (strncmp(cmd, "OTA ", 4) == 0 || strncmp(cmd, "ota ", 4) == 0) {
                    /* Extract URL (http:// or https://) */
                    char *url = cmd + 4;
                    while (*url == ' ') url++;

                    bt_spp_print("STATUS: Downloading firmware...\r\n");

                    if (download_firmware_http(url)) {
                        bt_spp_print("STATUS: Download OK, transferring...\r\n");
                        if (transfer_to_stm32()) {
                            bt_spp_print("STATUS: OTA complete!\r\n");
                        } else {
                            bt_spp_print("STATUS: Transfer failed\r\n");
                        }
                    } else {
                        bt_spp_print("STATUS: Download failed\r\n");
                    }

                } else if (strncmp(cmd, "FW ", 3) == 0 || strncmp(cmd, "fw ", 3) == 0) {
                    /* Begin a Bluetooth firmware push. DATA carries Base64
                     * so each printable block can be offset-acknowledged and
                     * retried independently before final CRC verification. */
                    unsigned long ver = 0, size = 0, crc = 0;
                    if (sscanf(cmd + 3, "%lu,%lu,%lx", &ver, &size, &crc) == 3 &&
                        size > 0 && size <= FW_FILE_MAX_SIZE) {
                        discard_incoming_firmware();
                        g_fw_version = (uint32_t)ver;
                        g_fw_expected_size = (uint32_t)size;
                        g_fw_crc32   = (uint32_t)crc;
                        g_fw_running_crc = 0;
                        g_fw_file = fopen(FW_FILE_PATH, "wb");
                        if (g_fw_file != NULL) {
                            g_fw_receiving = true;

                            char msg[96];
                            snprintf(msg, sizeof(msg),
                                     "FW: ready for %lu bytes, send DATA blocks\r\n",
                                     g_fw_expected_size);
                            bt_spp_print(msg);
                        } else {
                            ESP_LOGE(TAG, "Cannot open staged firmware for writing");
                            discard_incoming_firmware();
                            bt_spp_print("FW: storage open failed\r\n");
                        }
                    } else {
                        bt_spp_print("FW: usage FW <version>,<size>,<crc32-hex>; max 55296 bytes\r\n");
                    }

                } else if (strncmp(cmd, "DATA ", 5) == 0 || strncmp(cmd, "data ", 5) == 0) {
                    char *offset_text = cmd + 5;
                    char *comma = strchr(offset_text, ',');
                    bool accepted = false;

                    if (comma != NULL && g_fw_expected_size > 0 && !g_fw_staged) {
                        *comma = '\0';
                        char *end = NULL;
                        unsigned long offset = strtoul(offset_text, &end, 10);
                        const char *encoded = comma + 1;
                        uint8_t decoded[192];
                        size_t decoded_len = 0;

                        int rc = mbedtls_base64_decode(
                            decoded, sizeof(decoded), &decoded_len,
                            (const unsigned char *)encoded, strlen(encoded));

                        if (end != offset_text && *end == '\0' && rc == 0 &&
                            decoded_len > 0 &&
                            offset <= g_fw_expected_size &&
                            decoded_len <= g_fw_expected_size - offset) {
                            if (offset == g_fw_size && g_fw_size < g_fw_expected_size) {
                                accepted = stage_firmware_data(decoded, decoded_len);
                            } else if (offset + decoded_len == g_fw_size) {
                                /* The previous ACK may have been lost. Do not
                                 * append the same block twice; ACK it again. */
                                accepted = true;
                            }
                        }
                    }

                    if (accepted) {
                        char msg[48];
                        snprintf(msg, sizeof(msg), "DATA: ACK %lu\r\n", g_fw_size);
                        bt_spp_print(msg);
                    } else {
                        char msg[64];
                        snprintf(msg, sizeof(msg), "DATA: NAK expected offset %lu\r\n",
                                 g_fw_size);
                        bt_spp_print(msg);
                    }

                } else if (strncmp(cmd, "VERIFY", 6) == 0 || strncmp(cmd, "verify", 6) == 0) {
                    if (!g_fw_receiving && g_fw_size == g_fw_expected_size) {
                        g_fw_staged = true;
                        if (g_fw_running_crc != g_fw_crc32) {
                            ESP_LOGE(TAG,
                                     "Firmware receive CRC failed: crc=0x%08lX/0x%08lX",
                                     g_fw_running_crc, g_fw_crc32);
                            g_fw_staged = false;
                            bt_spp_print("FW: CRC mismatch; restart with FW\r\n");
                        } else if (verify_staged_firmware()) {
                            char msg[96];
                            snprintf(msg, sizeof(msg),
                                     "FW: staged %lu bytes; send SEND to start OTA\r\n",
                                     g_fw_size);
                            bt_spp_print(msg);
                        } else {
                            g_fw_staged = false;
                            bt_spp_print("FW: CRC mismatch; restart with FW\r\n");
                        }
                    } else {
                        bt_spp_print("FW: incomplete; continue DATA blocks\r\n");
                    }

                } else if (strncmp(cmd, "SEND", 4) == 0 || strncmp(cmd, "send", 4) == 0) {
                    /* Transfer the staged (Bluetooth-pushed) firmware */
                    if (g_fw_staged) {
                        bt_spp_print("STATUS: Transferring...\r\n");
                        if (transfer_to_stm32()) {
                            bt_spp_print("STATUS: OTA complete!\r\n");
                        } else {
                            bt_spp_print("STATUS: Transfer failed\r\n");
                        }
                    } else {
                        bt_spp_print("STATUS: No verified firmware staged (use FW, DATA, VERIFY)\r\n");
                    }

                } else if (strncmp(cmd, "WIFI ", 5) == 0 || strncmp(cmd, "wifi ", 5) == 0) {
                    /* WIFI <ssid>,<password> — store to NVS and reconnect */
                    char ssid[33] = {0};
                    char pass[65] = {0};
                    if (sscanf(cmd + 5, "%32[^,],%64s", ssid, pass) == 2) {
                        nvs_handle_t h;
                        if (nvs_open("cfg", NVS_READWRITE, &h) == ESP_OK) {
                            nvs_set_str(h, "wifi_ssid", ssid);
                            nvs_set_str(h, "wifi_pass", pass);
                            nvs_commit(h);
                            nvs_close(h);
                        }
                        wifi_connect(ssid, pass);
                        bt_spp_print("WIFI: saved and reconnecting\r\n");
                    } else {
                        bt_spp_print("WIFI: usage WIFI <ssid>,<password>\r\n");
                    }

                } else if (strncmp(cmd, "VERSION", 7) == 0 || strncmp(cmd, "version", 7) == 0) {
                    /* Query STM32 version */
                    ProtoFrame_t resp;
                    bool ok = false;
                    if (xSemaphoreTake(g_ota_mutex,
                                       pdMS_TO_TICKS(750)) == pdTRUE) {
                        ok = stm32_send_frame(CMD_GET_STATUS, NULL, 0) &&
                             stm32_wait_cmd(CMD_STATUS_RSP, &resp, 1500);
                        xSemaphoreGive(g_ota_mutex);
                    }
                    if (ok) {
                        char buf[64];
                        uint32_t ver = 0;
                        if (resp.len >= 4) memcpy(&ver, resp.payload, 4);
                        snprintf(buf, sizeof(buf), "FW Version: %lu\r\n", ver);
                        bt_spp_print(buf);
                    } else {
                        bt_spp_print("VERSION: No response from STM32\r\n");
                    }

                } else if (strncmp(cmd, "STATUS", 6) == 0 || strncmp(cmd, "status", 6) == 0) {
                    wifi_ap_record_t ap_info;
                    bool wifi_connected = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
                    char buf[192];
                    snprintf(buf, sizeof(buf),
                             "Bridge Status:\r\n"
                             "  BT connected: %s\r\n"
                             "  WiFi connected: %s\r\n"
                             "  FW receiving: %s\r\n"
                             "  FW staged: %s\r\n"
                             "  Staged size: %lu / %lu bytes\r\n"
                             "  Staged version: %lu\r\n",
                             g_spp_handle ? "yes" : "no",
                             wifi_connected ? "yes" : "no",
                             g_fw_receiving ? "yes" : "no",
                             g_fw_staged ? "yes" : "no",
                             g_fw_size,
                             g_fw_expected_size,
                             g_fw_version);
                    bt_spp_print(buf);

                } else if (strncmp(cmd, "RESET", 5) == 0 || strncmp(cmd, "reset", 5) == 0) {
                    bt_spp_print("Resetting...\r\n");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_restart();

                } else if (strncmp(cmd, "RELAY", 5) == 0 || strncmp(cmd, "relay", 5) == 0 ||
                           strncmp(cmd, "AUTO", 4) == 0 || strncmp(cmd, "auto", 4) == 0 ||
                           strncmp(cmd, "MANUAL", 6) == 0 || strncmp(cmd, "manual", 6) == 0 ||
                           strncmp(cmd, "BUZZER", 6) == 0 || strncmp(cmd, "buzzer", 6) == 0) {
                    /* Relay control. Commands are canonicalized and passed to
                     * the STM32 as CMD_APP_MSG text; the reply is CMD_STATUS_RSP
                     * with {relay1, relay2, auto_mode, buzzer}. The mutex keeps this
                     * from interleaving with a Web OTA transfer. */
                    char msg[32];
                    bool recognized = true;

                    if (strncmp(cmd, "BUZZER", 6) == 0 || strncmp(cmd, "buzzer", 6) == 0) {
                        bool on;
                        if (strstr(cmd + 6, "ON") != NULL ||
                            strstr(cmd + 6, "on") != NULL) {
                            on = true;
                        } else if (strstr(cmd + 6, "OFF") != NULL ||
                                   strstr(cmd + 6, "off") != NULL) {
                            on = false;
                        } else {
                            recognized = false;
                        }
                        if (recognized) {
                            snprintf(msg, sizeof(msg), "BUZZER %s",
                                     on ? "ON" : "OFF");
                        }
                    } else if (strncmp(cmd, "MANUAL", 6) == 0 || strncmp(cmd, "manual", 6) == 0) {
                        strcpy(msg, "AUTO OFF");
                    } else if (strncmp(cmd, "AUTO", 4) == 0 || strncmp(cmd, "auto", 4) == 0) {
                        bool on = !(strstr(cmd + 4, "OFF") != NULL ||
                                    strstr(cmd + 4, "off") != NULL);
                        snprintf(msg, sizeof(msg), "AUTO %s", on ? "ON" : "OFF");
                    } else if (cmd[5] == '1' || cmd[5] == '2') {
                        bool on;
                        if (strstr(cmd + 6, "ON") != NULL ||
                            strstr(cmd + 6, "on") != NULL) {
                            on = true;
                        } else if (strstr(cmd + 6, "OFF") != NULL ||
                                   strstr(cmd + 6, "off") != NULL) {
                            on = false;
                        } else {
                            recognized = false;
                        }
                        if (recognized) {
                            snprintf(msg, sizeof(msg), "RELAY%c %s", cmd[5],
                                     on ? "ON" : "OFF");
                        }
                    } else if (cmd[5] == '\0' || cmd[5] == ' ') {
                        strcpy(msg, "RELAY");
                    } else {
                        recognized = false;
                    }

                    if (!recognized) {
                        bt_spp_print("Control usage: RELAY1 ON|OFF, RELAY2 ON|OFF, RELAY, AUTO ON|OFF, MANUAL, BUZZER ON|OFF\r\n");
                    } else if (xSemaphoreTake(g_ota_mutex,
                                              pdMS_TO_TICKS(750)) != pdTRUE) {
                        bt_spp_print("RELAY: OTA transfer busy, retry later\r\n");
                    } else {
                        ProtoFrame_t resp;
                        bool ok = stm32_send_frame(CMD_APP_MSG,
                                                   (const uint8_t *)msg,
                                                   strlen(msg)) &&
                                  stm32_wait_cmd(CMD_STATUS_RSP, &resp, 2000) &&
                                   resp.len >= 4;
                        xSemaphoreGive(g_ota_mutex);

                        if (ok) {
                            char rbuf[96];
                            snprintf(rbuf, sizeof(rbuf),
                                     "RELAY1: %s, RELAY2: %s, AUTO: %s, BUZZER: %s\r\n",
                                     resp.payload[0] ? "ON" : "OFF",
                                     resp.payload[1] ? "ON" : "OFF",
                                     resp.payload[2] ? "ON" : "OFF",
                                     resp.payload[3] ? "ON" : "OFF");
                            bt_spp_print(rbuf);
                        } else {
                            bt_spp_print("RELAY: no response from STM32\r\n");
                        }
                    }

                } else {
                    bt_spp_print("Unknown command. Commands: OTA <url>, FW <ver>,<size>,<crc>, DATA <offset>,<base64>, VERIFY, SEND, WIFI <ssid>,<pass>, VERSION, STATUS, RESET, RELAY1 ON|OFF, RELAY2 ON|OFF, RELAY, AUTO ON|OFF, MANUAL, BUZZER ON|OFF\r\n");
                }
            } else {
                bt_spp_print("FW: non-text data rejected; use Base64 DATA blocks\r\n");
            }

            free(data);
        }
    }
}

/*---------------------------------------------------------------------------
 * Web OTA server
 *---------------------------------------------------------------------------*/

static const char *web_ota_state_name(WebOtaState_t state) {
    switch (state) {
    case WEB_OTA_UPLOADING:    return "uploading";
    case WEB_OTA_STAGED:       return "staged";
    case WEB_OTA_TRANSFERRING: return "transferring";
    case WEB_OTA_COMPLETE:     return "complete";
    case WEB_OTA_FAILED:       return "failed";
    case WEB_OTA_IDLE:
    default:                   return "idle";
    }
}

static esp_err_t web_send_json(httpd_req_t *req, const char *json,
                               const char *status) {
    if (status != NULL) {
        httpd_resp_set_status(req, status);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t web_root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, WEB_OTA_PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t web_status_handler(httpd_req_t *req) {
    WebOtaState_t state = g_web_ota_state;
    char json[256];
    snprintf(json, sizeof(json),
             "{\"state\":\"%s\",\"staged_size\":%lu,\"version\":%lu,"
             "\"sent\":%lu,\"total\":%lu,\"message\":\"%s\"}",
             web_ota_state_name(state),
             g_fw_size,
             g_fw_version,
             g_ota_progress_sent,
             g_ota_progress_total,
             state == WEB_OTA_FAILED ? "OTA transfer failed" : "");
    return web_send_json(req, json, NULL);
}

static void web_format_centi(char *output, size_t output_size,
                             int32_t value) {
    const uint32_t magnitude = value < 0
        ? (uint32_t)(-(int64_t)value)
        : (uint32_t)value;
    snprintf(output, output_size, "%s%lu.%02lu",
             value < 0 ? "-" : "",
             (unsigned long)(magnitude / 100U),
             (unsigned long)(magnitude % 100U));
}

/**
 * @brief Format the cached sensor snapshot as JSON.
 *
 * Shared by the HTTP GET /api/sensors handler and the periodic MQTT publish,
 * so the wire format only needs to be defined once.
 *
 * @return Number of bytes written (excluding NUL), or -1 on overflow.
 */
static int format_sensor_json(char *json, size_t json_size) {
    SensorSnapshot_t snapshot = {};
    uint32_t received_ms = 0;
    bool cached = false;

    if (g_sensor_cache_mutex != NULL &&
        xSemaphoreTake(g_sensor_cache_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        snapshot = g_sensor_cache;
        received_ms = g_sensor_cache_received_ms;
        cached = g_sensor_cache_valid;
        xSemaphoreGive(g_sensor_cache_mutex);
    }

    const uint32_t age_ms = cached
        ? bridge_uptime_ms() - received_ms
        : 0U;
    const bool online = cached && age_ms <= SENSOR_CACHE_STALE_MS;
    const bool environment_valid =
        (snapshot.flags & SENSOR_FLAG_ENV_VALID) != 0U;
    const bool light_valid =
        (snapshot.flags & SENSOR_FLAG_LIGHT_VALID) != 0U;

    char temperature[16];
    char humidity[16];
    char pressure[16];
    char lux[12];
    char age[16];
    web_format_centi(temperature, sizeof(temperature),
                     snapshot.temperature_centi_c);
    web_format_centi(humidity, sizeof(humidity),
                     snapshot.humidity_centi_percent);
    web_format_centi(pressure, sizeof(pressure),
                     (int32_t)snapshot.pressure_pa);
    if (light_valid) {
        snprintf(lux, sizeof(lux), "%u", snapshot.light_lux);
    } else {
        strcpy(lux, "null");
    }
    if (cached) {
        snprintf(age, sizeof(age), "%lu", (unsigned long)age_ms);
    } else {
        strcpy(age, "null");
    }

    const int json_length = snprintf(
        json, json_size,
        "{\"online\":%s,\"mqtt_connected\":%s,\"age_ms\":%s,\"uptime_ms\":%lu,"
        "\"environment_valid\":%s,\"light_valid\":%s,"
        "\"temperature\":%s,\"humidity\":%s,\"pressure\":%s,"
        "\"lux\":%s,\"pir_ready\":%s,\"pir_warmed_up\":%s,"
        "\"pir\":%s,\"relay1\":%s,\"relay2\":%s,"
        "\"auto_mode\":%s,\"buzzer\":%s,\"ui_chinese\":%s,"
        "\"led_brightness\":%u,\"led_percent\":%u}",
        online ? "true" : "false",
        g_mqtt_connected ? "true" : "false",
        age,
        (unsigned long)snapshot.uptime_ms,
        environment_valid ? "true" : "false",
        light_valid ? "true" : "false",
        environment_valid ? temperature : "null",
        environment_valid ? humidity : "null",
        environment_valid ? pressure : "null",
        lux,
        (snapshot.flags & SENSOR_FLAG_PIR_READY) != 0U ? "true" : "false",
        (snapshot.flags & SENSOR_FLAG_PIR_WARMED_UP) != 0U ? "true" : "false",
        (snapshot.flags & SENSOR_FLAG_MOTION) != 0U ? "true" : "false",
        (snapshot.flags & SENSOR_FLAG_RELAY1_ON) != 0U ? "true" : "false",
        (snapshot.flags & SENSOR_FLAG_RELAY2_ON) != 0U ? "true" : "false",
        (snapshot.flags & SENSOR_FLAG_AUTO_MODE) != 0U ? "true" : "false",
        (snapshot.flags & SENSOR_FLAG_BUZZER_ON) != 0U ? "true" : "false",
        (snapshot.flags & SENSOR_FLAG_UI_CHINESE) != 0U ? "true" : "false",
        snapshot.led_brightness,
        snapshot.led_percent);
    return (json_length < 0 || json_length >= (int)json_size) ? -1 : json_length;
}

static esp_err_t web_sensors_handler(httpd_req_t *req) {
    char json[768];
    if (format_sensor_json(json, sizeof(json)) < 0) {
        return web_send_json(req, "{\"message\":\"Sensor JSON overflow\"}",
                             "500 Internal Server Error");
    }
    return web_send_json(req, json, NULL);
}

static bool web_parse_version(httpd_req_t *req, uint32_t *version) {
    char query[64] = {0};
    char value[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "version", value, sizeof(value)) != ESP_OK) {
        return false;
    }

    char *end = NULL;
    unsigned long parsed = strtoul(value, &end, 10);
    if (end == value || *end != '\0' || parsed == 0 || parsed > UINT32_MAX) {
        return false;
    }
    *version = (uint32_t)parsed;
    return true;
}

static esp_err_t web_upload_handler(httpd_req_t *req) {
    uint32_t version = 0;
    if (!web_parse_version(req, &version)) {
        return web_send_json(req, "{\"message\":\"Invalid firmware version\"}",
                             "400 Bad Request");
    }
    if (req->content_len <= 0 || req->content_len > FW_FILE_MAX_SIZE) {
        return web_send_json(req, "{\"message\":\"Firmware must be 1..55296 bytes\"}",
                             "413 Payload Too Large");
    }
    if (g_ota_running || g_fw_receiving ||
        g_web_ota_state == WEB_OTA_TRANSFERRING) {
        return web_send_json(req, "{\"message\":\"OTA bridge is busy\"}",
                             "409 Conflict");
    }

    discard_incoming_firmware();
    g_fw_version = version;
    g_fw_expected_size = (uint32_t)req->content_len;
    g_fw_running_crc = 0;
    g_fw_file = fopen(FW_FILE_PATH, "wb");
    if (g_fw_file == NULL) {
        g_web_ota_state = WEB_OTA_FAILED;
        return web_send_json(req, "{\"message\":\"Cannot open firmware storage\"}",
                             "500 Internal Server Error");
    }

    g_fw_receiving = true;
    g_web_ota_state = WEB_OTA_UPLOADING;
    g_ota_progress_sent = 0;
    g_ota_progress_total = g_fw_expected_size;

    uint8_t buffer[1024];
    int remaining = req->content_len;
    while (remaining > 0) {
        size_t wanted = remaining < (int)sizeof(buffer)
                        ? (size_t)remaining : sizeof(buffer);
        int received = httpd_req_recv(req, (char *)buffer, wanted);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (received <= 0 || !stage_firmware_data(buffer, (size_t)received)) {
            discard_incoming_firmware();
            g_web_ota_state = WEB_OTA_FAILED;
            return web_send_json(req, "{\"message\":\"Firmware upload interrupted\"}",
                                 "500 Internal Server Error");
        }
        remaining -= received;
        g_ota_progress_sent = g_fw_size;
    }

    g_fw_crc32 = g_fw_running_crc;
    g_fw_staged = true;
    if (!verify_staged_firmware()) {
        discard_incoming_firmware();
        g_web_ota_state = WEB_OTA_FAILED;
        return web_send_json(req, "{\"message\":\"Firmware verification failed\"}",
                             "422 Unprocessable Entity");
    }

    g_web_ota_state = WEB_OTA_STAGED;
    char json[128];
    snprintf(json, sizeof(json),
             "{\"message\":\"Firmware staged\",\"size\":%lu,\"crc32\":\"%08lX\"}",
             g_fw_size, g_fw_crc32);
    return web_send_json(req, json, NULL);
}

static void web_ota_task(void *pv) {
    (void)pv;
    bool success = transfer_to_stm32();
    g_web_ota_state = success ? WEB_OTA_COMPLETE : WEB_OTA_FAILED;
    vTaskDelete(NULL);
}

static esp_err_t web_start_handler(httpd_req_t *req) {
    if (!g_fw_staged || g_fw_receiving) {
        return web_send_json(req, "{\"message\":\"No verified firmware staged\"}",
                             "409 Conflict");
    }
    if (g_ota_running || g_web_ota_state == WEB_OTA_TRANSFERRING) {
        return web_send_json(req, "{\"message\":\"OTA transfer already running\"}",
                             "409 Conflict");
    }

    g_web_ota_state = WEB_OTA_TRANSFERRING;
    g_ota_progress_sent = 0;
    g_ota_progress_total = g_fw_size;
    if (xTaskCreate(web_ota_task, "web_ota", WEB_OTA_TASK_STACK, NULL,
                    WEB_OTA_TASK_PRIO, NULL) != pdPASS) {
        g_web_ota_state = WEB_OTA_FAILED;
        return web_send_json(req, "{\"message\":\"Cannot start OTA task\"}",
                             "500 Internal Server Error");
    }
    return web_send_json(req, "{\"message\":\"OTA started\"}", "202 Accepted");
}

/*---------------------------------------------------------------------------
 * Web control: POST /api/control
 *
 * Body is a tiny JSON object with one field, e.g.
 *   {"light":true} | {"relay1":true} | {"relay2":false} |
 *   {"buzzer":true} | {"light_auto":false} |
 *   {"brightness":50} | {"ui_chinese":true}
 * The command is forwarded to the STM32 through the same CMD_APP_MSG path
 * used by Bluetooth; the reply is the current relay/auto/buzzer state.
 *---------------------------------------------------------------------------*/

/**
 * @brief Parse a control JSON body into an STM32 text command.
 *
 * Shared by the HTTP POST /api/control handler and the MQTT control topic,
 * so the field-name-to-command mapping only needs to be defined once. Uses
 * cJSON (bundled with ESP-IDF) instead of ad-hoc substring scanning — a
 * hand-rolled scan previously had to check "light_auto" before "light" to
 * avoid the substring false-match; exact key lookup removes that footgun.
 *
 * @return true if a recognized field was found and msg was filled;
 *         false if the body is not valid JSON, or has no supported field.
 */
static bool build_control_message(const char *body, char *msg, size_t msg_size) {
    cJSON *root = cJSON_Parse(body);
    if (root == NULL) {
        return false;
    }

    bool ok = true;
    const cJSON *item;
    if ((item = cJSON_GetObjectItemCaseSensitive(root, "light_auto")) != NULL) {
        snprintf(msg, msg_size, "LIGHT %s",
                 cJSON_IsTrue(item) ? "AUTO" : "MANUAL");
    } else if ((item = cJSON_GetObjectItemCaseSensitive(root, "brightness")) != NULL) {
        if (!cJSON_IsNumber(item) || item->valuedouble < 1 ||
            item->valuedouble > 100) {
            ok = false;
        } else {
            snprintf(msg, msg_size, "LIGHT BRIGHTNESS %d",
                     (int)item->valuedouble);
        }
    } else if ((item = cJSON_GetObjectItemCaseSensitive(root, "ui_chinese")) != NULL) {
        snprintf(msg, msg_size, "LANG %s", cJSON_IsTrue(item) ? "ZH" : "EN");
    } else if ((item = cJSON_GetObjectItemCaseSensitive(root, "light")) != NULL) {
        snprintf(msg, msg_size, "RELAY2 %s", cJSON_IsTrue(item) ? "ON" : "OFF");
    } else if ((item = cJSON_GetObjectItemCaseSensitive(root, "relay1")) != NULL) {
        snprintf(msg, msg_size, "RELAY1 %s", cJSON_IsTrue(item) ? "ON" : "OFF");
    } else if ((item = cJSON_GetObjectItemCaseSensitive(root, "relay2")) != NULL) {
        snprintf(msg, msg_size, "RELAY2 %s", cJSON_IsTrue(item) ? "ON" : "OFF");
    } else if ((item = cJSON_GetObjectItemCaseSensitive(root, "buzzer")) != NULL) {
        snprintf(msg, msg_size, "BUZZER %s", cJSON_IsTrue(item) ? "ON" : "OFF");
    } else if ((item = cJSON_GetObjectItemCaseSensitive(root, "auto")) != NULL) {
        snprintf(msg, msg_size, "AUTO %s", cJSON_IsTrue(item) ? "ON" : "OFF");
    } else {
        ok = false;
    }

    cJSON_Delete(root);
    return ok;
}

/**
 * @brief Send a control command to the STM32 and wait for its status reply.
 *
 * Shared by the HTTP and MQTT control paths. Guards against overlapping an
 * in-progress OTA transfer, since both control and OTA share USART1.
 *
 * @param error_message  Optional; set to a short reason string on failure.
 */
static bool send_control_message(const char *msg, ProtoFrame_t *resp,
                                 const char **error_message) {
    if (g_ota_running || g_web_ota_state == WEB_OTA_TRANSFERRING) {
        if (error_message != NULL) *error_message = "OTA transfer busy";
        return false;
    }
    if (xSemaphoreTake(g_ota_mutex, pdMS_TO_TICKS(750)) != pdTRUE) {
        if (error_message != NULL) *error_message = "UART busy, retry later";
        return false;
    }

    bool ok = stm32_send_frame(CMD_APP_MSG, (const uint8_t *)msg,
                               strlen(msg)) &&
              stm32_wait_cmd(CMD_STATUS_RSP, resp, 2000) &&
              resp->len >= 4;
    xSemaphoreGive(g_ota_mutex);

    if (!ok && error_message != NULL) {
        *error_message = "No response from STM32";
    }
    return ok;
}

static esp_err_t web_control_handler(httpd_req_t *req) {
    char body[64];
    int remaining = req->content_len;
    int offset = 0;
    if (remaining <= 0 || remaining >= (int)sizeof(body)) {
        return web_send_json(req, "{\"message\":\"Empty control body\"}",
                             "400 Bad Request");
    }
    while (remaining > 0) {
        int received = httpd_req_recv(req, body + offset, (size_t)remaining);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (received <= 0) {
            return web_send_json(req, "{\"message\":\"Control body read error\"}",
                                 "400 Bad Request");
        }
        offset += received;
        remaining -= received;
    }
    body[offset] = '\0';
    ESP_LOGI(TAG, "control body: '%s' len=%d", body, offset);

    char msg[32];
    if (!build_control_message(body, msg, sizeof(msg))) {
        return web_send_json(req,
                             "{\"message\":\"Unsupported control field\"}",
                             "400 Bad Request");
    }
    ESP_LOGI(TAG, "control command: '%s'", msg);

    ProtoFrame_t resp;
    const char *error_message = NULL;
    if (!send_control_message(msg, &resp, &error_message)) {
        char json[96];
        snprintf(json, sizeof(json), "{\"message\":\"%s\"}", error_message);
        const bool busy = strcmp(error_message, "No response from STM32") != 0;
        return web_send_json(req, json,
                             busy ? "409 Conflict" : "502 Bad Gateway");
    }

    char json[192];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"relay1\":%s,\"relay2\":%s,"
             "\"auto_mode\":%s,\"buzzer\":%s,"
             "\"led_percent\":%u,\"ui_chinese\":%s}",
             resp.payload[0] ? "true" : "false",
             resp.payload[1] ? "true" : "false",
             resp.payload[2] ? "true" : "false",
             resp.payload[3] ? "true" : "false",
             resp.len >= 5 ? resp.payload[4] : 0U,
             resp.len >= 6 && resp.payload[5] ? "true" : "false");
    return web_send_json(req, json, NULL);
}

/*---------------------------------------------------------------------------
 * MQTT client — public EMQX sandbox broker
 *
 * Publishes the same JSON as GET /api/sensors on a topic namespaced by MAC
 * address, and accepts the same control fields as POST /api/control on a
 * matching subscribe topic. Both HTTP and MQTT funnel into the same
 * format_sensor_json()/build_control_message()/send_control_message() so the
 * wire format and STM32 command mapping are defined exactly once.
 *---------------------------------------------------------------------------*/

static void mqtt_handle_control(const char *payload, size_t len) {
    char body[64];
    if (len == 0 || len >= sizeof(body)) {
        ESP_LOGW(TAG, "MQTT control payload rejected: %u bytes", (unsigned)len);
        return;
    }
    memcpy(body, payload, len);
    body[len] = '\0';

    char msg[32];
    if (!build_control_message(body, msg, sizeof(msg))) {
        ESP_LOGW(TAG, "MQTT control payload not recognized: '%s'", body);
        return;
    }

    ProtoFrame_t resp;
    const char *error_message = NULL;
    if (!send_control_message(msg, &resp, &error_message)) {
        ESP_LOGW(TAG, "MQTT control '%s' failed: %s", msg, error_message);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    (void)handler_args;
    (void)base;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        g_mqtt_connected = true;
        esp_mqtt_client_subscribe(g_mqtt_client, g_mqtt_topic_control, 0);
        ESP_LOGI(TAG, "MQTT connected, subscribed to %s", g_mqtt_topic_control);
        break;
    case MQTT_EVENT_DISCONNECTED:
        g_mqtt_connected = false;
        ESP_LOGW(TAG, "MQTT disconnected");
        break;
    case MQTT_EVENT_DATA:
        mqtt_handle_control(event->data, (size_t)event->data_len);
        break;
    default:
        break;
    }
}

static void mqtt_init(void) {
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(g_mqtt_topic_sensors, sizeof(g_mqtt_topic_sensors),
             "envlink/%02x%02x%02x/sensors", mac[3], mac[4], mac[5]);
    snprintf(g_mqtt_topic_control, sizeof(g_mqtt_topic_control),
             "envlink/%02x%02x%02x/control", mac[3], mac[4], mac[5]);

    char client_id[32];
    snprintf(client_id, sizeof(client_id), "envlink-%02x%02x%02x",
             mac[3], mac[4], mac[5]);

    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri = MQTT_BROKER_URI;
    cfg.credentials.client_id = client_id;

    g_mqtt_client = esp_mqtt_client_init(&cfg);
    if (g_mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT client init failed");
        return;
    }
    esp_mqtt_client_register_event(g_mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(g_mqtt_client);
    ESP_LOGI(TAG, "MQTT starting: %s (sensors -> %s)",
             MQTT_BROKER_URI, g_mqtt_topic_sensors);
}

static void mqtt_publish_sensors(void) {
    if (!g_mqtt_connected || g_mqtt_client == NULL) {
        return;
    }
    char json[768];
    if (format_sensor_json(json, sizeof(json)) < 0) {
        return;
    }
    esp_mqtt_client_publish(g_mqtt_client, g_mqtt_topic_sensors, json, 0, 0, 0);
}

static void web_server_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6144;
    config.max_uri_handlers = 6;
    config.recv_wait_timeout = 10;

    if (httpd_start(&g_http_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Web OTA server");
        return;
    }

    const httpd_uri_t root = {
        .uri = "/", .method = HTTP_GET,
        .handler = web_root_handler, .user_ctx = NULL
    };
    const httpd_uri_t status = {
        .uri = "/api/status", .method = HTTP_GET,
        .handler = web_status_handler, .user_ctx = NULL
    };
    const httpd_uri_t sensors = {
        .uri = "/api/sensors", .method = HTTP_GET,
        .handler = web_sensors_handler, .user_ctx = NULL
    };
    const httpd_uri_t upload = {
        .uri = "/api/upload", .method = HTTP_POST,
        .handler = web_upload_handler, .user_ctx = NULL
    };
    const httpd_uri_t start = {
        .uri = "/api/start", .method = HTTP_POST,
        .handler = web_start_handler, .user_ctx = NULL
    };
    const httpd_uri_t control = {
        .uri = "/api/control", .method = HTTP_POST,
        .handler = web_control_handler, .user_ctx = NULL
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(g_http_server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(g_http_server, &status));
    ESP_ERROR_CHECK(httpd_register_uri_handler(g_http_server, &sensors));
    ESP_ERROR_CHECK(httpd_register_uri_handler(g_http_server, &upload));
    ESP_ERROR_CHECK(httpd_register_uri_handler(g_http_server, &start));
    ESP_ERROR_CHECK(httpd_register_uri_handler(g_http_server, &control));
    ESP_LOGI(TAG, "Web OTA ready: connect to %s and open http://192.168.4.1",
             WIFI_AP_SSID);
}

/*---------------------------------------------------------------------------
 * WiFi station + SoftAP
 *---------------------------------------------------------------------------*/

static void wifi_connect(const char *ssid, const char *password) {
    wifi_config_t wifi_cfg = {};
    strncpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, password, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_LOGI(TAG, "WiFi connecting to %s...", ssid);
    esp_wifi_connect();
}

static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_cfg = {};
    strncpy((char *)ap_cfg.ap.ssid, WIFI_AP_SSID, sizeof(ap_cfg.ap.ssid) - 1);
    strncpy((char *)ap_cfg.ap.password, WIFI_AP_PASSWORD,
            sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.ssid_len = strlen(WIFI_AP_SSID);
    ap_cfg.ap.channel = 1;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Credentials: runtime-configured via the "WIFI" BT command (NVS),
     * falling back to compile-time defaults above. */
    nvs_handle_t h;
    char ssid[33] = {0};
    char pass[65] = {0};
    if (nvs_open("cfg", NVS_READONLY, &h) == ESP_OK) {
        size_t slen = sizeof(ssid), plen = sizeof(pass);
        nvs_get_str(h, "wifi_ssid", ssid, &slen);
        nvs_get_str(h, "wifi_pass", pass, &plen);
        nvs_close(h);
    }
    if (ssid[0] == '\0') strncpy(ssid, WIFI_SSID, sizeof(ssid) - 1);
    if (pass[0] == '\0') strncpy(pass, WIFI_PASSWORD, sizeof(pass) - 1);

    if (strcmp(ssid, "YOUR_SSID") != 0 && ssid[0] != '\0') {
        wifi_connect(ssid, pass);
    } else {
        ESP_LOGI(TAG, "No station credentials; SoftAP remains available");
    }
}

/*---------------------------------------------------------------------------
 * Firmware version from URL filename
 *---------------------------------------------------------------------------*/

/**
 * @brief Parse the firmware version from a URL like ".../fw_v2.bin".
 *        Falls back to version 1 when the pattern is not found.
 */
static uint32_t parse_version_from_url(const char *url) {
    const char *name = strrchr(url, '/');
    name = (name != NULL) ? name + 1 : url;

    if (strncmp(name, "fw_v", 4) == 0) {
        return (uint32_t)strtoul(name + 4, NULL, 10);
    }
    return 1U;
}

static bool download_firmware_http(const char *url) {
    ESP_LOGI(TAG, "Downloading firmware from: %s", url);

    g_fw_staged = false;
    g_fw_receiving = false;
    g_fw_size = 0;
    g_fw_expected_size = 0;

    FILE *f = fopen(FW_FILE_PATH, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open firmware file for writing");
        return false;
    }

    esp_http_client_config_t http_cfg = {};
    http_cfg.url = url;
    http_cfg.timeout_ms = 30000;
    http_cfg.buffer_size = HTTP_DOWNLOAD_BUF_SIZE;

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    esp_err_t err = esp_http_client_open(client, 0);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        fclose(f);
        return false;
    }

    int content_length = esp_http_client_fetch_headers(client);
    if (content_length <= 0 || content_length > FW_FILE_MAX_SIZE) {
        ESP_LOGE(TAG, "Invalid content length: %d", content_length);
        esp_http_client_cleanup(client);
        fclose(f);
        return false;
    }

    uint8_t buf[HTTP_DOWNLOAD_BUF_SIZE];
    int total_read = 0;

    while (total_read < content_length) {
        int read_len = esp_http_client_read(client, (char *)buf, sizeof(buf));
        if (read_len <= 0) {
            ESP_LOGE(TAG, "HTTP read error at %d / %d", total_read, content_length);
            esp_http_client_cleanup(client);
            fclose(f);
            return false;
        }

        if (fwrite(buf, 1, read_len, f) != (size_t)read_len) {
            ESP_LOGE(TAG, "HTTP staging write failed at %d / %d",
                     total_read, content_length);
            esp_http_client_cleanup(client);
            fclose(f);
            return false;
        }
        total_read += read_len;

        ESP_LOGI(TAG, "Downloaded %d / %d bytes", total_read, content_length);
    }

    fclose(f);
    esp_http_client_cleanup(client);

    g_fw_size = (uint32_t)total_read;
    g_fw_expected_size = g_fw_size;

    /* Compute CRC-32 of downloaded file */
    f = fopen(FW_FILE_PATH, "rb");
    if (!f) return false;

    uint32_t crc = 0U;
    while (true) {
        size_t n = fread(buf, 1, sizeof(buf), f);
        if (n == 0) break;
        crc = proto_crc32(buf, n, crc);
    }
    fclose(f);
    g_fw_crc32 = crc;

    /* Derive version from the URL filename (fw_v<N>.bin); default 1. */
    g_fw_version = parse_version_from_url(url);

    g_fw_staged = true;

    ESP_LOGI(TAG, "Download complete: %d bytes, CRC32=0x%08lX",
             g_fw_size, g_fw_crc32);
    return true;
}

/*---------------------------------------------------------------------------
 * OTA Transfer to STM32
 *---------------------------------------------------------------------------*/

static bool transfer_to_stm32(void) {
    if (g_ota_mutex == NULL ||
        xSemaphoreTake(g_ota_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Another OTA transfer is already running");
        return false;
    }

    g_ota_running = true;
    bool success = transfer_to_stm32_impl();
    g_ota_running = false;
    xSemaphoreGive(g_ota_mutex);
    return success;
}

static bool transfer_to_stm32_impl(void) {
    if (!g_fw_staged || g_fw_receiving) {
        ESP_LOGE(TAG, "No firmware staged for transfer");
        return false;
    }

    if (!verify_staged_firmware()) {
        return false;
    }

    FILE *f = fopen(FW_FILE_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open firmware file for reading");
        return false;
    }

    ESP_LOGI(TAG, "Starting OTA transfer: size=%lu, CRC32=0x%08lX",
             g_fw_size, g_fw_crc32);

    /* Step 1: ask the running application to persist its OTA request and
     * reboot. CMD_OTA_READY makes the timing deterministic instead of relying
     * on a guessed reset delay.
     *
     * The payload carries version + image size. The size is what lets the
     * bootloader recompute the application region's CRC on the next boot and
     * refuse to jump into the half-written body a power loss would leave
     * behind. A 4-byte payload is still accepted by the application (size
     * treated as unknown) so an older bridge keeps working — but that mode
     * cannot detect a torn image. */
    uart_flush_input(UART_STM32_NUM);
    {
        uint8_t availability[8];
        memcpy(availability, &g_fw_version, 4);
        memcpy(availability + 4, &g_fw_size, 4);
        if (!stm32_send_frame(CMD_OTA_AVAILABLE, availability,
                              sizeof(availability))) {
            fclose(f);
            return false;
        }
    }

    ProtoFrame_t resp;
    if (!stm32_wait_cmd(CMD_OTA_READY, &resp, 2000) || resp.len < 4) {
        ESP_LOGE(TAG, "Application did not confirm OTA readiness");
        fclose(f);
        return false;
    }

    uint32_t ready_version = 0;
    memcpy(&ready_version, resp.payload, 4);
    if (ready_version != g_fw_version) {
        ESP_LOGE(TAG, "OTA readiness version mismatch: %lu != %lu",
                 ready_version, g_fw_version);
        fclose(f);
        return false;
    }

    /* The app sends READY, waits 50 ms, then resets. Give the bootloader time
     * to configure HSE/USART1; it will keep a 2 s OTA window open. */
    vTaskDelay(pdMS_TO_TICKS(150));
    uart_flush_input(UART_STM32_NUM);

    /* Step 2: start the bootloader transfer. */
    uint8_t begin_payload[12];
    memcpy(begin_payload,      &g_fw_size,   4);
    memcpy(begin_payload + 4,  &g_fw_version, 4);
    memcpy(begin_payload + 8,  &g_fw_crc32,  4);
    if (!stm32_send_frame(CMD_OTA_BEGIN, begin_payload, 12)) {
        fclose(f);
        return false;
    }

    /* Wait for OTA_BEGIN_ACK */
    if (!stm32_wait_cmd(CMD_OTA_BEGIN_ACK, &resp, 3000)) {
        ESP_LOGE(TAG, "No OTA_BEGIN_ACK from STM32");
        fclose(f);
        return false;
    }

    if (resp.len < 4) {
        ESP_LOGE(TAG, "Malformed OTA_BEGIN_ACK from STM32");
        fclose(f);
        return false;
    }

    uint32_t expected_seq = 0;
    memcpy(&expected_seq, resp.payload, 4);
    if (expected_seq != 0) {
        ESP_LOGE(TAG, "Bootloader expected unexpected first sequence: %lu",
                 expected_seq);
        fclose(f);
        return false;
    }

    /* Step 3: Send chunks */
    uint8_t chunk_payload[PROTO_MAX_PAYLOAD];
    uint32_t seq = 0;
    uint32_t bytes_sent = 0;

    while (bytes_sent < g_fw_size) {
        /* Prepare chunk header (seq) */
        memcpy(chunk_payload, &seq, 4);

        /* Read page data */
        size_t chunk_len = (g_fw_size - bytes_sent) < OTA_CHUNK_SIZE
                           ? (g_fw_size - bytes_sent)
                           : OTA_CHUNK_SIZE;

        size_t n = fread(chunk_payload + 4, 1, chunk_len, f);
        if (n != chunk_len) {
            ESP_LOGE(TAG, "File read error at seq=%lu", seq);
            fclose(f);
            return false;
        }

        /* Send chunk with retry */
        bool acked = false;
        for (int retry = 0; retry < OTA_MAX_RETRIES && !acked; retry++) {
            if (!stm32_send_frame(CMD_OTA_CHUNK, chunk_payload, 4 + n)) {
                ESP_LOGW(TAG, "Chunk %lu: UART write failed, retry %d",
                         seq, retry + 1);
                continue;
            }

            ProtoFrame_t ack_resp;
            if (stm32_wait_cmd(CMD_CHUNK_ACK, &ack_resp, OTA_CHUNK_TIMEOUT_MS) &&
                ack_resp.len >= 4) {
                uint32_t ack_seq;
                memcpy(&ack_seq, ack_resp.payload, 4);
                if (ack_seq == seq) {
                    acked = true;
                }
            } else {
                /* NAK/timeout already consumed inside stm32_wait_cmd; retry */
                ESP_LOGW(TAG, "Chunk %lu: retry %d", seq, retry + 1);
            }
        }

        if (!acked) {
            ESP_LOGE(TAG, "Chunk %lu failed after %d retries", seq, OTA_MAX_RETRIES);
            (void)stm32_send_frame(CMD_OTA_ABORT, NULL, 0);
            fclose(f);
            return false;
        }

        bytes_sent += n;
        g_ota_progress_sent = bytes_sent;
        seq++;

        if (seq % 10 == 0) {
            ESP_LOGI(TAG, "Progress: %lu / %lu bytes", bytes_sent, g_fw_size);
        }
    }
    fclose(f);

    /* Step 4: Send OTA_END and verify */
    if (!stm32_send_frame(CMD_OTA_END, NULL, 0)) {
        return false;
    }

    if (!stm32_wait_cmd(CMD_OTA_RESULT, &resp, 10000)) {
        ESP_LOGE(TAG, "No OTA_RESULT from STM32");
        return false;
    }

    uint32_t result_code = 0;
    if (resp.len >= 4) memcpy(&result_code, resp.payload, 4);

    if (result_code == OTA_RESULT_OK) {
        ESP_LOGI(TAG, "OTA transfer successful!");
        g_fw_staged = false;
        g_fw_size = 0;
        g_fw_expected_size = 0;
        g_fw_crc32 = 0;
        /* Delete the staged file */
        unlink(FW_FILE_PATH);
        return true;
    } else {
        ESP_LOGE(TAG, "OTA transfer failed: result=%lu", result_code);
        return false;
    }
}

/*---------------------------------------------------------------------------
 * Main
 *---------------------------------------------------------------------------*/

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== STM32 OTA Bridge starting ===");

    /* Init NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Init storage */
    spiffs_init();

    g_ota_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(g_ota_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    g_sensor_cache_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(g_sensor_cache_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    /* Init UART to STM32 */
    uart_stm32_init();

    /* Init Bluetooth SPP */
    bt_spp_init();

    /* Init WiFi (for OTA downloads) */
    wifi_init_sta();

    /* Start phone-friendly Web OTA page on the SoftAP. */
    web_server_start();

    /* Connect to the public MQTT sandbox; esp-mqtt reconnects on its own
     * once the WiFi station link comes up, so no ordering dependency here. */
    mqtt_init();

    /* Keep a non-blocking sensor snapshot cache for the Web dashboard. */
    BaseType_t sensor_task_created =
        xTaskCreate(sensor_poll_task, "sensor_poll", SENSOR_POLL_TASK_STACK,
                    NULL, SENSOR_POLL_TASK_PRIO, NULL);
    ESP_ERROR_CHECK(sensor_task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    /* Create BT receive task */
    xTaskCreate(bt_recv_task, "bt_recv", SPP_TASK_STACK, NULL, SPP_TASK_PRIO, NULL);

    ESP_LOGI(TAG, "Bridge ready. BT device: %s", SPP_SERVER_NAME);
    ESP_LOGI(TAG, "Connect via Bluetooth SPP and send commands.");

    /* Main loop — nothing to do, tasks handle everything */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "Bridge alive. BT=%s, FW=%s",
                 g_spp_handle ? "connected" : "idle",
                 g_fw_staged ? "staged" : "none");
    }
}
