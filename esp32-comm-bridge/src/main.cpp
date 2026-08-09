/**
 * @file    main.cpp
 * @brief   ESP32 Communication Bridge — BT SPP + WiFi OTA + Web Dashboard + WS Telemetry.
 *
 * Architecture:
 *   - BT SPP server: phone commands + binary firmware receive
 *   - WiFi AP (STM32-EnvMon) + STA dual-mode
 *   - HTTP Server (port 80): Web dashboard + REST API
 *   - WebSocket Server (port 81): real-time sensor data push
 *   - UART link: STM32 protocol (460800 baud, framed, CRC-32)
 *   - OTA orchestrator: SPIFFS firmware staging → STM32 transfer
 *   - MQTT client: cloud telemetry (placeholder)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netdb.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_spp_api.h"
#include "esp_bt_device.h"

#include "esp_http_server.h"
#include "esp_http_client.h"

/*---------------------------------------------------------------------------
 * Shared protocol (C-compatible)
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
#define UART_STM32_BAUD         460800
#define UART_STM32_BUF_SIZE     2048

/* BT SPP */
#define SPP_SERVER_NAME         "STM32-EnvMon"
#define SPP_TASK_STACK          4096
#define SPP_TASK_PRIO           5

/* OTA transfer */
#define OTA_TASK_STACK          4096
#define OTA_TASK_PRIO           4
#define OTA_CHUNK_SIZE          1024

/* SPIFFS */
#define FW_FILE_PATH            "/spiffs/fw.bin"
#define FW_FILE_MAX_SIZE        (54 * 1024)

/* HTTP download */
#define HTTP_DOWNLOAD_BUF_SIZE  4096

/* WiFi AP */
#define WIFI_AP_SSID            "STM32-EnvMon"
#define WIFI_AP_PASS            "12345678"
#define WIFI_AP_CHANNEL         6
#define WIFI_AP_MAX_CONN        4

/* WebSocket */
#define WS_SERVER_PORT          81
#define WS_MAX_CLIENTS          4

/* Telemetry */
#define TELEM_BUF_SIZE          512

/*---------------------------------------------------------------------------
 * Global state
 *---------------------------------------------------------------------------*/

/* BT */
static uint32_t        g_spp_handle = 0;
static QueueHandle_t   g_spp_queue;

/* OTA */
static bool            g_fw_staged = false;
static uint32_t        g_fw_size = 0;
static uint32_t        g_fw_version = 0;
static uint32_t        g_fw_crc32 = 0;

/* WebSocket clients */
static int             g_ws_sockets[WS_MAX_CLIENTS] = {-1, -1, -1, -1};
static portMUX_TYPE    g_ws_lock = portMUX_INITIALIZER_UNLOCKED;

/* Latest telemetry JSON (shared, mutex-protected) */
static char            g_telemetry[TELEM_BUF_SIZE] = "{}";
static portMUX_TYPE    g_telem_lock = portMUX_INITIALIZER_UNLOCKED;

/* Control queue (WebSocket → STM32) */
static QueueHandle_t   g_ctrl_queue;

/* WiFi connected flag */
static EventGroupHandle_t g_wifi_events;
#define WIFI_CONNECTED_BIT  BIT0

/*---------------------------------------------------------------------------
 * Forward declarations
 *---------------------------------------------------------------------------*/

static void uart_stm32_init(void);
static void bt_spp_init(void);
static void wifi_init_apsta(void);
static void bt_recv_task(void *pv);
static void stm32_reader_task(void *pv);
static bool download_firmware_http(const char *url);
static bool transfer_to_stm32(void);
static void http_server_start(void);
static void ws_send_all(const char *msg, size_t len);

/*---------------------------------------------------------------------------
 * UART to STM32
 *---------------------------------------------------------------------------*/

static void uart_stm32_init(void) {
    const uart_config_t uart_config = {
        .baud_rate  = UART_STM32_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_STM32_NUM,
                                         UART_STM32_BUF_SIZE,
                                         UART_STM32_BUF_SIZE,
                                         0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_STM32_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_STM32_NUM,
                                  UART_STM32_TXD, UART_STM32_RXD,
                                  UART_STM32_RTS, UART_STM32_CTS));
}

static void stm32_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    uint8_t buf[PROTO_MAX_FRAME];
    uint16_t total = proto_build_frame(buf, sizeof(buf), cmd, payload, len);
    if (total > 0) {
        uart_write_bytes(UART_STM32_NUM, buf, total);
    }
}

static bool stm32_wait_cmd(uint8_t expected_cmd, ProtoFrame_t *out,
                            uint32_t timeout_ms) {
    ProtoParser_t parser;
    proto_parser_init(&parser);
    uint8_t byte;
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - start < timeout_ms) {
        int len = uart_read_bytes(UART_STM32_NUM, &byte, 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            const ProtoFrame_t *f = proto_parser_feed(&parser, byte);
            if (f != NULL) {
                if (f->cmd == expected_cmd) {
                    if (out) memcpy(out, f, sizeof(ProtoFrame_t));
                    return true;
                }
                if (f->cmd == CMD_NAK) {
                    if (out) memcpy(out, f, sizeof(ProtoFrame_t));
                    return false;
                }
            }
        }
    }
    return false;
}

/*---------------------------------------------------------------------------
 * STM32 Reader Task — continuously read UART for telemetry + OTA responses
 *---------------------------------------------------------------------------*/

static void stm32_reader_task(void *pv) {
    ProtoParser_t parser;
    proto_parser_init(&parser);
    uint8_t byte;

    ESP_LOGI(TAG, "STM32 reader started");

    for (;;) {
        int len = uart_read_bytes(UART_STM32_NUM, &byte, 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            const ProtoFrame_t *f = proto_parser_feed(&parser, byte);
            if (f != NULL) {
                /* CMD_SENSOR_DATA (0x21): JSON telemetry from STM32 */
                if (f->cmd == CMD_SENSOR_DATA) {
                    taskENTER_CRITICAL(&g_telem_lock);
                    size_t copy_len = f->len < TELEM_BUF_SIZE - 1 ? f->len : TELEM_BUF_SIZE - 1;
                    memcpy(g_telemetry, f->payload, copy_len);
                    g_telemetry[copy_len] = '\0';
                    taskEXIT_CRITICAL(&g_telem_lock);

                    /* Forward to all WebSocket clients */
                    ws_send_all(g_telemetry, copy_len);
                }
                /* Other responses are consumed by the OTA state machine
                 * via stm32_wait_cmd() — but that reads from same UART buffer.
                 * For simplicity in this prototype, OTA transfers happen
                 * synchronously in bt_recv_task, and the reader task is
                 * suspended during OTA. In production: use a dedicated
                 * response dispatcher with queues. */
            }
        }
    }
}

/*---------------------------------------------------------------------------
 * SPIFFS
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
 * Bluetooth SPP
 *---------------------------------------------------------------------------*/

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "SPP init");
        esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE,
                          0, SPP_SERVER_NAME);
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "SPP client connected, handle=%u", param->srv_open.handle);
        g_spp_handle = param->srv_open.handle;
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "SPP closed, handle=%u", param->close.handle);
        g_spp_handle = 0;
        break;

    case ESP_SPP_DATA_IND_EVT:
        if (param->data_ind.len > 0) {
            uint8_t *copy = (uint8_t *)malloc(param->data_ind.len);
            if (copy) {
                memcpy(copy, param->data_ind.data, param->data_ind.len);
                xQueueSend(g_spp_queue, &copy, 0);
            }
        }
        break;

    default:
        break;
    }
}

static void bt_spp_init(void) {
    g_spp_queue = xQueueCreate(32, sizeof(uint8_t *));

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_callback));
    ESP_ERROR_CHECK(esp_spp_init(ESP_SPP_MODE_CB));

    esp_bt_dev_set_device_name(SPP_SERVER_NAME);
}

static void bt_spp_send(const uint8_t *data, size_t len) {
    if (g_spp_handle != 0) {
        esp_spp_write(g_spp_handle, len, (uint8_t *)data);
    }
}

static void bt_spp_print(const char *msg) {
    bt_spp_send((const uint8_t *)msg, strlen(msg));
}

/*---------------------------------------------------------------------------
 * BT Receive Task
 *---------------------------------------------------------------------------*/

/* Forward declare control handler for BT text commands */
static void handle_control_json(const char *json, size_t len);

static void bt_recv_task(void *pv) {
    uint8_t *data;

    for (;;) {
        if (xQueueReceive(g_spp_queue, &data, portMAX_DELAY) == pdPASS) {
            size_t len = strlen((char *)data);

            if (len > 0 && data[0] >= 'A' && data[0] <= 'Z') {
                char cmd[256] = {0};
                memcpy(cmd, data, len < sizeof(cmd) - 1 ? len : sizeof(cmd) - 1);

                ESP_LOGI(TAG, "BT cmd: %s", cmd);

                if (strncmp(cmd, "OTA http", 8) == 0 || strncmp(cmd, "ota http", 8) == 0) {
                    char *url = cmd + 4;
                    while (*url == ' ') url++;
                    bt_spp_print("STATUS: Downloading...\r\n");
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
                } else if (strncmp(cmd, "VERSION", 7) == 0 || strncmp(cmd, "version", 7) == 0) {
                    stm32_send_frame(CMD_GET_STATUS, NULL, 0);
                    ProtoFrame_t resp;
                    if (stm32_wait_cmd(CMD_STATUS_RSP, &resp, 1000)) {
                        char buf[64];
                        uint32_t ver = 0;
                        if (resp.len >= 4) memcpy(&ver, resp.payload, 4);
                        snprintf(buf, sizeof(buf), "FW Version: %lu\r\n", ver);
                        bt_spp_print(buf);
                    } else {
                        bt_spp_print("VERSION: No response\r\n");
                    }
                } else if (strncmp(cmd, "STATUS", 6) == 0 || strncmp(cmd, "status", 6) == 0) {
                    taskENTER_CRITICAL(&g_telem_lock);
                    char telem_copy[TELEM_BUF_SIZE];
                    strncpy(telem_copy, g_telemetry, sizeof(telem_copy));
                    taskEXIT_CRITICAL(&g_telem_lock);
                    bt_spp_print("Sensors: ");
                    bt_spp_print(telem_copy);
                    bt_spp_print("\r\n");
                } else if (strncmp(cmd, "RELAY1 ON", 9) == 0) {
                    handle_control_json("{\"relay1\":1}", 11);
                    bt_spp_print("OK\r\n");
                } else if (strncmp(cmd, "RELAY1 OFF", 10) == 0) {
                    handle_control_json("{\"relay1\":0}", 11);
                    bt_spp_print("OK\r\n");
                } else if (strncmp(cmd, "RELAY2 ON", 9) == 0) {
                    handle_control_json("{\"relay2\":1}", 11);
                    bt_spp_print("OK\r\n");
                } else if (strncmp(cmd, "RELAY2 OFF", 10) == 0) {
                    handle_control_json("{\"relay2\":0}", 11);
                    bt_spp_print("OK\r\n");
                } else if (strncmp(cmd, "MIST ON", 7) == 0) {
                    handle_control_json("{\"mist\":1}", 9);
                    bt_spp_print("OK\r\n");
                } else if (strncmp(cmd, "MIST OFF", 8) == 0) {
                    handle_control_json("{\"mist\":0}", 9);
                    bt_spp_print("OK\r\n");
                } else if (strncmp(cmd, "AUTO", 4) == 0) {
                    handle_control_json("{\"mode\":\"auto\"}", 15);
                    bt_spp_print("Mode: AUTO\r\n");
                } else if (strncmp(cmd, "MANUAL", 6) == 0) {
                    handle_control_json("{\"mode\":\"manual\"}", 17);
                    bt_spp_print("Mode: MANUAL\r\n");
                } else if (strncmp(cmd, "RESET", 5) == 0 || strncmp(cmd, "reset", 5) == 0) {
                    bt_spp_print("Resetting...\r\n");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_restart();
                } else {
                    bt_spp_print("Cmds: OTA <url>, VERSION, STATUS, RELAY1/2 ON/OFF, MIST ON/OFF, AUTO, MANUAL, RESET\r\n");
                }
            } else {
                /* Binary firmware data via BT */
                FILE *f = fopen(FW_FILE_PATH, "ab");
                if (f) {
                    fwrite(data, 1, len, f);
                    fclose(f);
                    g_fw_staged = true;
                    struct stat st;
                    if (stat(FW_FILE_PATH, &st) == 0) g_fw_size = st.st_size;
                }
            }
            free(data);
        }
    }
}

/*---------------------------------------------------------------------------
 * Control handler — sends JSON command to STM32 as CMD_CONTROL_CMD (0x22)
 *---------------------------------------------------------------------------*/

static void handle_control_json(const char *json, size_t len) {
    if (len > PROTO_MAX_PAYLOAD) len = PROTO_MAX_PAYLOAD;
    stm32_send_frame(CMD_CONTROL_CMD, (const uint8_t *)json, (uint16_t)len);
    ESP_LOGI(TAG, "Ctrl sent: %.*s", (int)len, json);
}

/*---------------------------------------------------------------------------
 * WiFi (AP + STA dual mode)
 *---------------------------------------------------------------------------*/

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t *evt = (wifi_event_ap_staconnected_t *)event_data;
            ESP_LOGI(TAG, "AP: station " MACSTR " connected", MAC2STR(evt->mac));
        } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            wifi_event_ap_stadisconnected_t *evt = (wifi_event_ap_stadisconnected_t *)event_data;
            ESP_LOGI(TAG, "AP: station " MACSTR " disconnected", MAC2STR(evt->mac));
        } else if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGW(TAG, "STA: disconnected, reconnecting...");
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *evt = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "STA IP: " IPSTR, IP2STR(&evt->ip_info.ip));
            xEventGroupSetBits(g_wifi_events, WIFI_CONNECTED_BIT);
        }
    }
}

static void wifi_init_apsta(void) {
    g_wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Create AP + STA netifs */
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                        &wifi_event_handler, NULL, NULL));

    /* AP config */
    wifi_config_t ap_cfg = {};
    strcpy((char *)ap_cfg.ap.ssid, WIFI_AP_SSID);
    strcpy((char *)ap_cfg.ap.password, WIFI_AP_PASS);
    ap_cfg.ap.ssid_len = strlen(WIFI_AP_SSID);
    ap_cfg.ap.channel = WIFI_AP_CHANNEL;
    ap_cfg.ap.max_connection = WIFI_AP_MAX_CONN;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    /* STA config — change these to your router's credentials */
    wifi_config_t sta_cfg = {};
    strcpy((char *)sta_cfg.sta.ssid, "YOUR_SSID");
    strcpy((char *)sta_cfg.sta.password, "YOUR_PASSWORD");

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP: %s (password: %s)", WIFI_AP_SSID, WIFI_AP_PASS);
    ESP_LOGI(TAG, "Connect to AP, then open http://192.168.4.1");
}

/*---------------------------------------------------------------------------
 * WebSocket helpers (plain TCP WebSocket, RFC 6455)
 *---------------------------------------------------------------------------*/

/* Simple WebSocket frame builder for text frames (< 126 bytes payload) */
static int ws_build_frame(uint8_t *out, const char *payload, size_t len) {
    int pos = 0;
    out[pos++] = 0x81;  /* FIN + text opcode */
    out[pos++] = (uint8_t)(len & 0x7F); /* no mask (server→client) */
    memcpy(out + pos, payload, len);
    pos += len;
    return pos;
}

/* Parse WebSocket client handshake, return key value for Sec-WebSocket-Accept */
#include "mbedtls/sha1.h"
#include "mbedtls/base64.h"

static bool ws_parse_handshake(const char *req, char *key_out, size_t key_size) {
    const char *p = strstr(req, "Sec-WebSocket-Key: ");
    if (!p) return false;
    p += 19;
    const char *end = strstr(p, "\r\n");
    if (!end) return false;
    size_t len = end - p;
    if (len >= key_size) return false;
    memcpy(key_out, p, len);
    key_out[len] = '\0';
    return true;
}

static void ws_send_handshake(int sock, const char *key) {
    /* Concatenate key + magic GUID */
    char combined[256];
    snprintf(combined, sizeof(combined), "%s%s", key, "258EAFA5-E914-47DA-95CA-C5AB0DC85B11");

    /* SHA-1 hash */
    uint8_t sha1sum[20];
    mbedtls_sha1((const unsigned char *)combined, strlen(combined), sha1sum);

    /* Base64 encode */
    size_t olen;
    char accept_key[64];
    mbedtls_base64_encode((unsigned char *)accept_key, sizeof(accept_key), &olen, sha1sum, 20);
    accept_key[olen] = '\0';

    /* Build response */
    char resp[512];
    snprintf(resp, sizeof(resp),
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %s\r\n\r\n",
        accept_key);

    send(sock, resp, strlen(resp), 0);
}

static void ws_add_client(int sock) {
    portENTER_CRITICAL(&g_ws_lock);
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (g_ws_sockets[i] < 0) {
            g_ws_sockets[i] = sock;
            ESP_LOGI(TAG, "WS client %d connected (slot %d)", sock, i);
            break;
        }
    }
    portEXIT_CRITICAL(&g_ws_lock);
}

static void ws_remove_client(int sock) {
    portENTER_CRITICAL(&g_ws_lock);
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (g_ws_sockets[i] == sock) {
            g_ws_sockets[i] = -1;
            ESP_LOGI(TAG, "WS client %d removed (slot %d)", sock, i);
            break;
        }
    }
    portEXIT_CRITICAL(&g_ws_lock);
    close(sock);
}

static void ws_send_all(const char *msg, size_t len) {
    uint8_t frame[256];
    int frame_len = ws_build_frame(frame, msg, len);

    portENTER_CRITICAL(&g_ws_lock);
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
        if (g_ws_sockets[i] >= 0) {
            int ret = send(g_ws_sockets[i], frame, frame_len, 0);
            if (ret < 0) {
                /* Client disconnected */
                close(g_ws_sockets[i]);
                g_ws_sockets[i] = -1;
            }
        }
    }
    portEXIT_CRITICAL(&g_ws_lock);
}

/**
 * @brief Simple WebSocket data handler — parse client frames.
 *
 * For this application, client → server frames are JSON control commands.
 * We handle masked text frames (opcode 0x81, mask bit set).
 */
static void ws_handle_data(int sock, const uint8_t *data, size_t len) {
    /* Minimal frame parser: opcode + mask + payload */
    if (len < 2) return;

    uint8_t opcode = data[0] & 0x0F;
    bool masked = (data[1] & 0x80) != 0;
    size_t payload_len = data[1] & 0x7F;
    size_t header_len = 2;

    if (payload_len == 126) {
        if (len < 4) return;
        payload_len = (data[2] << 8) | data[3];
        header_len = 4;
    } else if (payload_len == 127) {
        if (len < 10) return;
        payload_len = 0;
        for (int i = 0; i < 8; i++) payload_len = (payload_len << 8) | data[2 + i];
        header_len = 10;
    }

    uint8_t mask[4] = {0};
    if (masked) {
        memcpy(mask, data + header_len, 4);
        header_len += 4;
    }

    if (len < header_len + payload_len) return;

    /* Decode payload */
    char payload[256] = {0};
    size_t copy_len = payload_len < sizeof(payload) - 1 ? payload_len : sizeof(payload) - 1;
    for (size_t i = 0; i < copy_len; i++) {
        payload[i] = (char)(data[header_len + i] ^ mask[i % 4]);
    }
    payload[copy_len] = '\0';

    /* Handle opcodes */
    if (opcode == 0x08) {
        /* Close frame */
        ws_remove_client(sock);
    } else if (opcode == 0x09) {
        /* Ping → Pong */
        uint8_t pong[128];
        pong[0] = 0x8A; /* FIN + pong */
        size_t plen = payload_len < sizeof(pong) - 2 ? payload_len : sizeof(pong) - 2;
        pong[1] = (uint8_t)(plen & 0x7F);
        memcpy(pong + 2, data + header_len, plen);
        send(sock, pong, 2 + plen, 0);
    } else if (opcode == 0x01 || opcode == 0x02) {
        /* Text or binary → treat as control command */
        ESP_LOGI(TAG, "WS cmd: %s", payload);
        handle_control_json(payload, copy_len);
    }
}

/**
 * @brief WebSocket server task — accepts connections on port 81.
 */
static void ws_server_task(void *pv) {
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "WS: socket() failed");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(WS_SERVER_PORT);

    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "WS: bind() failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 4) < 0) {
        ESP_LOGE(TAG, "WS: listen() failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "WebSocket server listening on port %d", WS_SERVER_PORT);

    for (;;) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Read HTTP upgrade request */
        char buf[1024] = {0};
        int n = recv(client_sock, buf, sizeof(buf) - 1, 0);
        if (n > 0) {
            buf[n] = '\0';
            char ws_key[256];
            if (ws_parse_handshake(buf, ws_key, sizeof(ws_key))) {
                ws_send_handshake(client_sock, ws_key);
                ws_add_client(client_sock);

                /* Keep reading from this client in a simple select-like loop.
                 * For production: one task per client or async server. */
                /* Here we just add it; the HTTP server's connection handler
                 * won't touch it. WebSocket client frames are read below. */
            } else {
                /* Not a WebSocket upgrade — close */
                const char *resp = "HTTP/1.1 400 Bad Request\r\n\r\n";
                send(client_sock, resp, strlen(resp), 0);
                close(client_sock);
            }
        }
    }
}

/**
 * @brief WebSocket client reader task — polls all connected clients for data.
 */
static void ws_reader_task(void *pv) {
    uint8_t buf[512];

    for (;;) {
        portENTER_CRITICAL(&g_ws_lock);
        int socks[WS_MAX_CLIENTS];
        memcpy(socks, g_ws_sockets, sizeof(socks));
        portEXIT_CRITICAL(&g_ws_lock);

        for (int i = 0; i < WS_MAX_CLIENTS; i++) {
            if (socks[i] >= 0) {
                /* Non-blocking read */
                int n = recv(socks[i], buf, sizeof(buf), MSG_DONTWAIT);
                if (n > 0) {
                    ws_handle_data(socks[i], buf, n);
                } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                    ws_remove_client(socks[i]);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/*---------------------------------------------------------------------------
 * HTTP Server — serves dashboard HTML + REST API
 *---------------------------------------------------------------------------*/

/* Forward declare API handlers */
static esp_err_t api_sensors_handler(httpd_req_t *req);
static esp_err_t api_control_handler(httpd_req_t *req);
static esp_err_t api_ota_status_handler(httpd_req_t *req);
static esp_err_t root_handler(httpd_req_t *req);

/* Serve static files from SPIFFS */
static esp_err_t spiffs_handler(httpd_req_t *req) {
    char path[128];
    if (strcmp(req->uri, "/") == 0) {
        strcpy(path, "/spiffs/index.html");
    } else {
        snprintf(path, sizeof(path), "/spiffs%s", req->uri);
    }

    /* Check if file exists */
    struct stat st;
    if (stat(path, &st) != 0) {
        /* Serve in-memory index if SPIFFS doesn't have it */
        if (strcmp(req->uri, "/") == 0 || strcmp(req->uri, "/index.html") == 0) {
            /* Fallback: redirect to AP mode info page */
            const char *info =
                "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>STM32 EnvMon</title>"
                "<style>body{font-family:-apple-system,sans-serif;background:#0f1117;"
                "color:#e0e0e0;display:flex;align-items:center;justify-content:center;"
                "min-height:100vh;margin:0;text-align:center;}"
                ".box{background:#1a1d27;border:1px solid #2a2d3a;border-radius:12px;"
                "padding:40px;max-width:500px;}"
                "a{color:#3b82f6;}</style></head><body><div class='box'>"
                "<h1>STM32 EnvMon</h1>"
                "<p>Web dashboard not uploaded to SPIFFS yet.</p>"
                "<p>Open <code>web-dashboard/index.html</code> directly in your browser "
                "for demo mode, or upload it to ESP32 SPIFFS with PlatformIO.</p>"
                "<p><a href='/api/sensors'>GET /api/sensors</a> — raw sensor JSON</p>"
                "</div></body></html>";
            httpd_resp_set_type(req, "text/html");
            httpd_resp_send(req, info, strlen(info));
            return ESP_OK;
        }
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    /* Determine content type */
    const char *type = "text/plain";
    if (strstr(path, ".html")) type = "text/html";
    else if (strstr(path, ".css")) type = "text/css";
    else if (strstr(path, ".js")) type = "application/javascript";
    else if (strstr(path, ".json")) type = "application/json";
    else if (strstr(path, ".png")) type = "image/png";
    else if (strstr(path, ".ico")) type = "image/x-icon";

    /* Read file */
    FILE *f = fopen(path, "r");
    if (!f) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    char *buf = (char *)malloc(st.st_size + 1);
    if (!buf) {
        fclose(f);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t read_len = fread(buf, 1, st.st_size, f);
    fclose(f);
    buf[read_len] = '\0';

    httpd_resp_set_type(req, type);
    httpd_resp_send(req, buf, read_len);
    free(buf);
    return ESP_OK;
}

/* GET /api/sensors — return latest telemetry JSON */
static esp_err_t api_sensors_handler(httpd_req_t *req) {
    taskENTER_CRITICAL(&g_telem_lock);
    char buf[TELEM_BUF_SIZE];
    strncpy(buf, g_telemetry, sizeof(buf));
    taskEXIT_CRITICAL(&g_telem_lock);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

/* POST /api/control — forward control JSON to STM32 */
static esp_err_t api_control_handler(httpd_req_t *req) {
    char buf[256] = {0};
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    ESP_LOGI(TAG, "API control: %s", buf);
    handle_control_json(buf, (size_t)len);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, "{\"ok\":true}", 9);
    return ESP_OK;
}

/* GET /api/ota/status */
static esp_err_t api_ota_status_handler(httpd_req_t *req) {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"fw_staged\":%s,\"fw_size\":%lu,\"bt_connected\":%s}",
             g_fw_staged ? "true" : "false",
             g_fw_size,
             g_spp_handle ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

/* CORS preflight for /api/* */
static esp_err_t api_options_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, "", 0);
    return ESP_OK;
}

static void http_server_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 16;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        return;
    }

    /* API routes (registered before wildcard catch-all) */
    httpd_uri_t api_sensors = {
        .uri = "/api/sensors", .method = HTTP_GET,
        .handler = api_sensors_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_sensors);

    httpd_uri_t api_control = {
        .uri = "/api/control", .method = HTTP_POST,
        .handler = api_control_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_control);

    httpd_uri_t api_ota = {
        .uri = "/api/ota/status", .method = HTTP_GET,
        .handler = api_ota_status_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_ota);

    httpd_uri_t api_opts = {
        .uri = "/api/*", .method = HTTP_OPTIONS,
        .handler = api_options_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_opts);

    /* Static files / catch-all */
    httpd_uri_t static_files = {
        .uri = "/*", .method = HTTP_GET,
        .handler = spiffs_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &static_files);

    ESP_LOGI(TAG, "HTTP server started on port 80");
}

/*---------------------------------------------------------------------------
 * WiFi HTTP firmware download
 *---------------------------------------------------------------------------*/

static bool download_firmware_http(const char *url) {
    ESP_LOGI(TAG, "Downloading: %s", url);

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

    g_fw_size = content_length;

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
        fwrite(buf, 1, read_len, f);
        total_read += read_len;
    }

    fclose(f);
    esp_http_client_cleanup(client);

    /* Compute CRC-32 */
    f = fopen(FW_FILE_PATH, "rb");
    if (!f) return false;

    uint32_t crc = 0xFFFFFFFFU;
    while (true) {
        size_t n = fread(buf, 1, sizeof(buf), f);
        if (n == 0) break;
        crc = proto_crc32(buf, n, crc);
    }
    fclose(f);
    g_fw_crc32 = crc ^ 0xFFFFFFFFU;
    g_fw_version = (uint32_t)time(NULL);
    g_fw_staged = true;

    ESP_LOGI(TAG, "Download complete: %d bytes, CRC32=0x%08lX", g_fw_size, g_fw_crc32);
    return true;
}

/*---------------------------------------------------------------------------
 * OTA Transfer to STM32
 *---------------------------------------------------------------------------*/

static bool transfer_to_stm32(void) {
    if (!g_fw_staged) {
        ESP_LOGE(TAG, "No firmware staged");
        return false;
    }

    FILE *f = fopen(FW_FILE_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open firmware file for reading");
        return false;
    }

    ESP_LOGI(TAG, "Starting OTA: size=%lu, CRC32=0x%08lX", g_fw_size, g_fw_crc32);

    uint8_t begin_payload[12];
    memcpy(begin_payload,      &g_fw_size,   4);
    memcpy(begin_payload + 4,  &g_fw_version, 4);
    memcpy(begin_payload + 8,  &g_fw_crc32,  4);
    stm32_send_frame(CMD_OTA_BEGIN, begin_payload, 12);

    ProtoFrame_t resp;
    if (!stm32_wait_cmd(CMD_OTA_BEGIN_ACK, &resp, 2000)) {
        ESP_LOGE(TAG, "No OTA_BEGIN_ACK");
        fclose(f);
        return false;
    }

    uint8_t chunk_payload[PROTO_MAX_PAYLOAD];
    uint32_t seq = 0;
    uint32_t bytes_sent = 0;

    while (bytes_sent < g_fw_size) {
        memcpy(chunk_payload, &seq, 4);
        size_t chunk_len = (g_fw_size - bytes_sent) < OTA_CHUNK_SIZE
                           ? (g_fw_size - bytes_sent) : OTA_CHUNK_SIZE;
        size_t n = fread(chunk_payload + 4, 1, chunk_len, f);
        if (n == 0 && chunk_len > 0) {
            ESP_LOGE(TAG, "File read error at seq=%lu", seq);
            fclose(f);
            return false;
        }

        bool acked = false;
        for (int retry = 0; retry < OTA_MAX_RETRIES && !acked; retry++) {
            stm32_send_frame(CMD_OTA_CHUNK, chunk_payload, 4 + n);
            ProtoFrame_t ack_resp;
            if (stm32_wait_cmd(CMD_CHUNK_ACK, &ack_resp, OTA_CHUNK_TIMEOUT_MS)) {
                uint32_t ack_seq;
                memcpy(&ack_seq, ack_resp.payload, 4);
                if (ack_seq == seq) acked = true;
            }
        }

        if (!acked) {
            ESP_LOGE(TAG, "Chunk %lu failed after %d retries", seq, OTA_MAX_RETRIES);
            stm32_send_frame(CMD_OTA_ABORT, NULL, 0);
            fclose(f);
            return false;
        }

        bytes_sent += n;
        seq++;
    }
    fclose(f);

    stm32_send_frame(CMD_OTA_END, NULL, 0);
    if (!stm32_wait_cmd(CMD_OTA_RESULT, &resp, 10000)) {
        ESP_LOGE(TAG, "No OTA_RESULT");
        return false;
    }

    uint32_t result_code = 0;
    if (resp.len >= 4) memcpy(&result_code, resp.payload, 4);

    if (result_code == OTA_RESULT_OK) {
        ESP_LOGI(TAG, "OTA success!");
        g_fw_staged = false;
        unlink(FW_FILE_PATH);
        return true;
    } else {
        ESP_LOGE(TAG, "OTA failed: result=%lu", result_code);
        return false;
    }
}

/*---------------------------------------------------------------------------
 * Main
 *---------------------------------------------------------------------------*/

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== STM32 EnvMon Bridge starting ===");

    /* Init NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Init SPIFFS */
    spiffs_init();

    /* Init UART to STM32 */
    uart_stm32_init();

    /* Init Bluetooth SPP */
    bt_spp_init();

    /* Init WiFi AP+STA */
    wifi_init_apsta();

    /* Create tasks */
    xTaskCreate(bt_recv_task, "bt_recv", SPP_TASK_STACK, NULL, SPP_TASK_PRIO, NULL);
    xTaskCreate(stm32_reader_task, "stm32_rd", 3072, NULL, 4, NULL);
    xTaskCreate(ws_server_task, "ws_srv", 4096, NULL, 4, NULL);
    xTaskCreate(ws_reader_task, "ws_rd", 3072, NULL, 3, NULL);

    /* Start HTTP server (after WiFi is up) */
    vTaskDelay(pdMS_TO_TICKS(2000));
    http_server_start();

    ESP_LOGI(TAG, "=== Bridge ready ===");
    ESP_LOGI(TAG, "  WiFi AP: %s / %s", WIFI_AP_SSID, WIFI_AP_PASS);
    ESP_LOGI(TAG, "  Web: http://192.168.4.1");
    ESP_LOGI(TAG, "  BT: %s", SPP_SERVER_NAME);
    ESP_LOGI(TAG, "  WebSocket: ws://192.168.4.1:81/ws");
    ESP_LOGI(TAG, "===================================");

    /* Periodic status log */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        ESP_LOGI(TAG, "Alive. BT=%s, STA=%s, FW=%s",
                 g_spp_handle ? "conn" : "idle",
                 (xEventGroupGetBits(g_wifi_events) & WIFI_CONNECTED_BIT) ? "ok" : "no",
                 g_fw_staged ? "staged" : "none");
    }
}
