/**
 * @file    main.c
 * @brief   Application entry point and FreeRTOS task implementations.
 *
 * Runs from 0x08002000. Sets SCB->VTOR at startup to relocate the
 * interrupt vector table. Integrates FreeRTOS with the shared protocol
 * stack to communicate with the ESP32 co-processor.
 *
 * Tasks:
 *   vCommTask      — UART frame receive/send, protocol parsing
 *   vControlTask   — OTA trigger (writes config + resets), version reporting
 *   vAppTask       — Sensors, local UI, actuators, and WS2812B linkage
 *   vMonitorTask   — System health, stack/heap monitoring
 */

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include <string.h>

#include "../../../shared/protocol.h"
#include "../../../shared/ota_config.h"
#include "app_tasks.h"
#include "bh1750.h"
#include "uart_comm.h"
#include "cmd_handler.h"
#include "env_i2c.h"
#include "environment_sensor.h"
#include "ui_display.h"
#include "pir_sensor.h"
#include "buzzer.h"
#include "relay.h"
#include "rotary_encoder.h"
#include "back_button.h"
#include "ws2812b.h"

/*---------------------------------------------------------------------------
 * Pin definitions
 *---------------------------------------------------------------------------*/

#define LED_PORT        GPIOC
#define LED_PIN         GPIO_PIN_13

/*---------------------------------------------------------------------------
 * Communication handles
 *---------------------------------------------------------------------------*/

static QueueHandle_t        g_cmd_queue;       /* ProtoFrame_t*: comm → control */
static QueueHandle_t        g_data_queue;      /* Raw bytes: comm → app */
static EventGroupHandle_t   g_event_group;
static StreamBufferHandle_t g_rx_stream;
static bool                 g_display_ready;
static bool                 g_bh1750_ready;
static bool                 g_environment_ready;
static bool                 g_encoder_ready;
static bool                 g_back_button_ready;
static bool                 g_pir_ready;
static bool                 g_buzzer_ready;
static bool                 g_relay_ready;
static bool                 g_ws2812b_ready;
static volatile SensorSnapshot_t g_sensor_snapshot;
static bool                 g_light_auto_mode = true;
static uint8_t              g_light_manual_percent = 50U;
static uint8_t              g_light_brightness;
static bool                 g_ui_chinese = true;

/* The CMSIS startup file copies .data only. ota_config.c places the short
 * Flash erase/program wrappers in .ramfunc, so initialize that section before
 * the control task can write the OTA configuration page. */
extern uint8_t _siramfunc;
extern uint8_t _sramfunc;
extern uint8_t _eramfunc;

/* Event group bits */
#define EVENT_OTA_AVAILABLE  (1 << 0)
#define EVENT_CONNECTED      (1 << 1)
#define EVENT_ERROR          (1 << 2)

#define WS2812_UPDATE_MS             200U
#define WS2812_DARK_LUX              5U
#define WS2812_BRIGHT_LUX            1000U
#define WS2812_DARK_BRIGHTNESS       160U
#define WS2812_BRIGHT_BRIGHTNESS     1U
#define WS2812_FALLBACK_BRIGHTNESS   24U
#define WS2812_BRIGHTNESS_STEP       16U
#define WS2812_BRIGHTNESS_DEADBAND   2U
#define WS2812_REFRESH_MS             1000U
#define SENSOR_SNAPSHOT_UPDATE_MS    200U

/*---------------------------------------------------------------------------
 * System init
 *---------------------------------------------------------------------------*/

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    /* 8MHz HSE crystal → PLL ×8 → SYSCLK 64MHz. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL8;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        /* Clock configuration failure → halt with LED off. */
        while (1);
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;   /* PCLK1 = 32MHz (max 36MHz) */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;   /* PCLK2 = 64MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        while (1);
    }
}

static void ramfunc_init(void) {
    const uint8_t *src = &_siramfunc;
    uint8_t *dst = &_sramfunc;

    while (dst < &_eramfunc) {
        *dst++ = *src++;
    }
}

/*---------------------------------------------------------------------------
 * Independent watchdog (IWDG)
 *
 * Runs off the ~40kHz LSI, independent of SysTick and interrupts, so it
 * still fires if the CPU gets stuck in a disabled-interrupt loop (e.g.
 * vApplicationStackOverflowHook) or an unhandled fault. Register-level
 * (not HAL_IWDG_*) so it does not depend on HAL_IWDG_MODULE_ENABLED being
 * set in stm32f1xx_hal_conf.h.
 *
 * IWDG keeps counting across NVIC_SystemReset() (only a power-on reset
 * clears it), including the bootloader→application jump used for OTA, so
 * both binaries arm it with the same ~4s timeout and refresh it
 * independently — see bootloader/Core/Src/main.c.
 *---------------------------------------------------------------------------*/

static void iwdg_init(void) {
    IWDG->KR  = 0xCCCCU;   /* start the watchdog */
    IWDG->KR  = 0x5555U;   /* unlock PR/RLR for writing */
    IWDG->PR  = 4U;        /* /64 prescaler */
    IWDG->RLR = 2499U;     /* ~4.0s @ nominal 40kHz LSI */

    uint32_t guard = 100000U;
    while ((IWDG->SR != 0U) && (--guard != 0U)) {
        /* wait for the prescaler/reload update to latch */
    }

    IWDG->KR = 0xAAAAU;    /* load the counter from RLR */
}

static void iwdg_refresh(void) {
    IWDG->KR = 0xAAAAU;
}

static void system_init(void) {
    /* VTOR relocation — MUST be first */
    SCB->VTOR = APP_BASE;

    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); /* LED off */
}

/*---------------------------------------------------------------------------
 * vCommTask — UART protocol handler
 *---------------------------------------------------------------------------*/

static void vCommTask(void *pvParameters) {
    (void)pvParameters;
    ProtoParser_t parser;
    proto_parser_init(&parser);

    uint8_t rx_byte;
    size_t rx_count;

    for (;;) {
        /* Block waiting for UART bytes from the stream buffer */
        rx_count = xStreamBufferReceive(g_rx_stream, &rx_byte, 1,
                                        pdMS_TO_TICKS(100));

        if (rx_count > 0) {
            const ProtoFrame_t *f = proto_parser_feed(&parser, rx_byte);
            if (f != NULL) {
                /* Valid frame received — dispatch */
                cmd_handler_dispatch(f);
            }
        }
    }
}

/*---------------------------------------------------------------------------
 * vControlTask — OTA trigger and system control
 *---------------------------------------------------------------------------*/

static void vControlTask(void *pvParameters) {
    (void)pvParameters;

    /* Report current firmware version on startup */
    BootConfig_t cfg;
    if (ota_config_read(&cfg)) {
        uint32_t version = cfg.fw_version;
        cmd_handler_send_frame(CMD_STATUS_RSP, (uint8_t *)&version, 4);
    }

    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(
            g_event_group,
            EVENT_OTA_AVAILABLE,
            pdTRUE,   /* Clear on exit */
            pdFALSE,  /* Wait for any (not all) */
            portMAX_DELAY
        );

        if (bits & EVENT_OTA_AVAILABLE) {
            /* A new firmware version is available on the ESP32.
             * Read the version from the event group (passed via queue). */
            uint32_t pending_version;
            if (xQueueReceive(g_cmd_queue, &pending_version, 0) == pdPASS) {
                /* Write OTA request to config then reboot into bootloader */
                if (ota_config_request_update(pending_version, 0)) {
                    /* Confirm only after the config page is valid. The ESP32
                     * waits for this before it begins the bootloader transfer. */
                    cmd_handler_send_frame(CMD_OTA_READY,
                                           (const uint8_t *)&pending_version, 4);
                    /* Small delay so the confirmation fully leaves USART1. */
                    vTaskDelay(pdMS_TO_TICKS(50));
                    NVIC_SystemReset();
                }
            }
        }
    }
}

/*---------------------------------------------------------------------------
 * vAppTask — user application logic (placeholder)
 *---------------------------------------------------------------------------*/

#define PIR_WARMUP_MS       30000U
#define ENV_SAMPLE_MS       2000U
#define ENV_CONVERSION_MS   90U
#define UI_REFRESH_MS       100U

static void ui_status_build(UiStatus_t *status,
                            const EnvironmentReading_t *environment,
                            bool environment_valid,
                            uint16_t light_lux, bool light_valid,
                            bool motion_detected, bool pir_warmed_up,
                            uint8_t control_selected, bool control_editing,
                            uint32_t firmware_version);

static uint8_t light_brightness_to_percent(uint8_t brightness) {
    const uint32_t percent =
        ((uint32_t)brightness * 100U + (WS2812_DARK_BRIGHTNESS / 2U)) /
        WS2812_DARK_BRIGHTNESS;
    return (uint8_t)(percent > 100U ? 100U : percent);
}

static uint8_t light_percent_to_brightness(uint8_t percent) {
    if (percent < 1U) {
        percent = 1U;
    } else if (percent > 100U) {
        percent = 100U;
    }
    const uint32_t brightness =
        ((uint32_t)percent * WS2812_DARK_BRIGHTNESS + 50U) / 100U;
    return (uint8_t)(brightness < 1U ? 1U : brightness);
}

static void light_set_auto(bool enabled) {
    if (!enabled && g_light_auto_mode) {
        uint8_t current = light_brightness_to_percent(g_light_brightness);
        g_light_manual_percent = current < 1U ? 1U : current;
    }
    g_light_auto_mode = enabled;
    if (!enabled) {
        g_light_brightness =
            light_percent_to_brightness(g_light_manual_percent);
    }
}

static bool parse_percent(const char *text, uint8_t *value) {
    uint16_t parsed = 0U;
    bool digit_seen = false;
    while (*text == ' ') {
        text++;
    }
    while (*text >= '0' && *text <= '9') {
        parsed = (uint16_t)(parsed * 10U + (uint16_t)(*text - '0'));
        digit_seen = true;
        if (parsed > 100U) {
            return false;
        }
        text++;
    }
    if (!digit_seen || (*text != '\0' && *text != '\r' && *text != '\n') ||
        parsed < 1U) {
        return false;
    }
    *value = (uint8_t)parsed;
    return true;
}

/*---------------------------------------------------------------------------
 * Relay control commands (forwarded from ESP32 via CMD_APP_MSG)
 *
 * Canonical forms sent by the bridge:
 *   RELAY1 ON | RELAY1 OFF | RELAY2 ON | RELAY2 OFF | RELAY
 *   LIGHT AUTO | LIGHT MANUAL | LIGHT BRIGHTNESS 1..100
 *   LANG ZH | LANG EN | BUZZER ON | BUZZER OFF
 * Replies with {relay1, relay2, light_auto, buzzer, brightness, chinese}.
 *---------------------------------------------------------------------------*/

static void handle_control_command(const char *cmd) {
    bool handled = false;

    if (strncmp(cmd, "RELAY", 5U) == 0) {
        uint8_t channel = 0xFFU;  /* bare "RELAY" -> query only */
        if (cmd[5] == '1') {
            channel = 0U;
        } else if (cmd[5] == '2') {
            channel = 1U;
        }

        if (channel != 0xFFU) {
            if (strstr(cmd + 6, "ON") != NULL) {
                relay_set(channel, true);
            } else if (strstr(cmd + 6, "OFF") != NULL) {
                relay_set(channel, false);
            }
        }
        handled = true;
    } else if (strcmp(cmd, "LIGHT AUTO") == 0 ||
               strcmp(cmd, "AUTO ON") == 0) {
        light_set_auto(true);
        handled = true;
    } else if (strcmp(cmd, "LIGHT MANUAL") == 0 ||
               strcmp(cmd, "AUTO OFF") == 0 ||
               strcmp(cmd, "MANUAL") == 0) {
        light_set_auto(false);
        handled = true;
    } else if (strncmp(cmd, "LIGHT BRIGHTNESS ", 17U) == 0) {
        uint8_t percent;
        if (parse_percent(cmd + 17U, &percent)) {
            g_light_manual_percent = percent;
            if (!g_light_auto_mode) {
                g_light_brightness = light_percent_to_brightness(percent);
            }
            handled = true;
        }
    } else if (strcmp(cmd, "LANG ZH") == 0) {
        g_ui_chinese = true;
        handled = true;
    } else if (strcmp(cmd, "LANG EN") == 0) {
        g_ui_chinese = false;
        handled = true;
    } else if (strncmp(cmd, "BUZZER", 6U) == 0) {
        if (strstr(cmd + 6, "ON") != NULL) {
            buzzer_set(true);
        } else if (strstr(cmd + 6, "OFF") != NULL) {
            buzzer_set(false);
        }
        handled = true;
    }

    if (handled) {
        uint8_t status[6] = {
            relay_get_state(0U) ? 1U : 0U,
            relay_get_state(1U) ? 1U : 0U,
            g_light_auto_mode ? 1U : 0U,
            buzzer_get_state() ? 1U : 0U,
            light_brightness_to_percent(g_light_brightness),
            g_ui_chinese ? 1U : 0U
        };
        cmd_handler_send_frame(CMD_STATUS_RSP, status, sizeof(status));
    }
}

static uint8_t ws2812_target_brightness(uint16_t light_lux,
                                        bool light_valid) {
    if (!light_valid) {
        return WS2812_FALLBACK_BRIGHTNESS;
    }
    if (light_lux <= WS2812_DARK_LUX) {
        return WS2812_DARK_BRIGHTNESS;
    }
    if (light_lux >= WS2812_BRIGHT_LUX) {
        return WS2812_BRIGHT_BRIGHTNESS;
    }

    const uint32_t lux_range = WS2812_BRIGHT_LUX - WS2812_DARK_LUX;
    const uint32_t brightness_range =
        WS2812_DARK_BRIGHTNESS - WS2812_BRIGHT_BRIGHTNESS;
    const uint32_t reduction =
        ((uint32_t)(light_lux - WS2812_DARK_LUX) * brightness_range) /
        lux_range;
    return (uint8_t)(WS2812_DARK_BRIGHTNESS - reduction);
}

static uint8_t ws2812_smooth_brightness(uint8_t current, uint8_t target) {
    const uint8_t difference = current < target
                                   ? (uint8_t)(target - current)
                                   : (uint8_t)(current - target);
    if (difference <= WS2812_BRIGHTNESS_DEADBAND) {
        return current;
    }

    if (current < target) {
        return difference > WS2812_BRIGHTNESS_STEP
                   ? (uint8_t)(current + WS2812_BRIGHTNESS_STEP)
                   : target;
    }
    if (current > target) {
        return difference > WS2812_BRIGHTNESS_STEP
                   ? (uint8_t)(current - WS2812_BRIGHTNESS_STEP)
                   : target;
    }
    return current;
}

static void sensor_snapshot_publish(
    TickType_t now,
    const EnvironmentReading_t *environment,
    bool environment_valid,
    uint16_t light_lux,
    bool light_valid,
    bool motion_detected,
    bool pir_warmed_up,
    uint8_t led_brightness) {
    SensorSnapshot_t snapshot = {0};
    snapshot.uptime_ms = (uint32_t)(now * portTICK_PERIOD_MS);
    snapshot.temperature_centi_c = environment->temperature_centi_c;
    snapshot.humidity_centi_percent = environment->humidity_centi_percent;
    snapshot.pressure_pa = environment->pressure_pa;
    snapshot.light_lux = light_lux;
    const bool light_power_on =
        g_relay_ready && relay_get_state(RELAY_LIGHT_CHANNEL);
    const uint8_t effective_led_brightness =
        light_power_on ? led_brightness : 0U;
    snapshot.led_brightness = effective_led_brightness;

    snapshot.led_percent = light_brightness_to_percent(led_brightness);

    if (environment_valid) {
        snapshot.flags |= SENSOR_FLAG_ENV_VALID;
    }
    if (light_valid) {
        snapshot.flags |= SENSOR_FLAG_LIGHT_VALID;
    }
    if (g_pir_ready) {
        snapshot.flags |= SENSOR_FLAG_PIR_READY;
    }
    if (pir_warmed_up) {
        snapshot.flags |= SENSOR_FLAG_PIR_WARMED_UP;
    }
    if (motion_detected) {
        snapshot.flags |= SENSOR_FLAG_MOTION;
    }
    if (g_relay_ready && relay_get_state(0U)) {
        snapshot.flags |= SENSOR_FLAG_RELAY1_ON;
    }
    if (g_relay_ready && relay_get_state(1U)) {
        snapshot.flags |= SENSOR_FLAG_RELAY2_ON;
    }
    if (g_light_auto_mode) {
        snapshot.flags |= SENSOR_FLAG_AUTO_MODE;
    }
    if (g_buzzer_ready && buzzer_get_state()) {
        snapshot.flags |= SENSOR_FLAG_BUZZER_ON;
    }
    if (g_ui_chinese) {
        snapshot.flags |= SENSOR_FLAG_UI_CHINESE;
    }

    taskENTER_CRITICAL();
    g_sensor_snapshot = snapshot;
    taskEXIT_CRITICAL();
}

static void vAppTask(void *pvParameters) {
    (void)pvParameters;
    uint8_t data[64];
    UiPage_t page = UI_PAGE_MENU;
    uint8_t menu_selected = 0U;
    uint8_t control_selected = 0U;
    bool control_editing = false;
    EnvironmentReading_t environment = {0};
    bool environment_valid = false;
    uint16_t light_lux = 0U;
    bool light_valid = g_bh1750_ready && bh1750_read_lux(&light_lux);
    bool motion_detected = g_pir_ready && pir_sensor_motion_detected();
    bool pir_warmed_up = false;
    bool previous_button_pressed =
        g_encoder_ready && rotary_encoder_button_pressed();
    bool previous_back_pressed =
        g_back_button_ready && back_button_pressed();
    TickType_t last_light_read = 0U;
    TickType_t last_ws2812_update = 0U;
    TickType_t last_ws2812_refresh = 0U;
    TickType_t last_sensor_snapshot = 0U;
    TickType_t last_ui_refresh = 0U;
    g_light_brightness = ws2812_target_brightness(light_lux, light_valid);
    uint8_t ws2812_last_sent = 0U;
    bool ws2812_frame_sent = false;
    const TickType_t app_started_at = xTaskGetTickCount();
    TickType_t last_environment_start = app_started_at;
    TickType_t environment_started_at = app_started_at;
    bool environment_pending = g_environment_ready &&
                               environment_sensor_start_measurement();
    BootConfig_t boot_config;
    const uint32_t firmware_version = ota_config_read(&boot_config)
                                          ? boot_config.fw_version : 0U;
    UiStatus_t ui_status;

    ui_status_build(&ui_status, &environment, environment_valid,
                    light_lux, light_valid, motion_detected, pir_warmed_up,
                     control_selected, control_editing, firmware_version);
    if (g_display_ready) {
        ui_display_render(page, menu_selected, &ui_status, true);
    }

    sensor_snapshot_publish(app_started_at, &environment, environment_valid,
                            light_lux, light_valid, motion_detected,
                             pir_warmed_up, g_light_brightness);

    for (;;) {
        /* Receive control commands from ESP32 (BT serial bridge) */
        if (xQueueReceive(g_data_queue, data, pdMS_TO_TICKS(5)) == pdPASS) {
            handle_control_command((const char *)data);
        }

        const int16_t encoder_delta =
            g_encoder_ready ? rotary_encoder_get_delta() : 0;
        const bool button_pressed = g_encoder_ready &&
                                    rotary_encoder_button_pressed();
        const bool button_clicked = button_pressed &&
                                    !previous_button_pressed;
        previous_button_pressed = button_pressed;
        const bool back_pressed = g_back_button_ready &&
                                  back_button_pressed();
        const bool back_clicked = back_pressed && !previous_back_pressed;
        previous_back_pressed = back_pressed;
        const TickType_t now = xTaskGetTickCount();

        if (page == UI_PAGE_MENU && encoder_delta != 0) {
            int16_t remaining = encoder_delta;
            while (remaining < 0) {
                menu_selected = (menu_selected == 0U)
                    ? (UI_MENU_ITEM_COUNT - 1U)
                    : (uint8_t)(menu_selected - 1U);
                remaining++;
            }
            while (remaining > 0) {
                menu_selected = (uint8_t)((menu_selected + 1U) %
                                          UI_MENU_ITEM_COUNT);
                remaining--;
            }
        } else if (page == UI_PAGE_LIGHT && encoder_delta != 0) {
            if (control_editing && !g_light_auto_mode &&
                control_selected == 2U) {
                int16_t percent = (int16_t)g_light_manual_percent +
                                  encoder_delta * 5;
                if (percent < 1) {
                    percent = 1;
                } else if (percent > 100) {
                    percent = 100;
                }
                g_light_manual_percent = (uint8_t)percent;
                g_light_brightness = light_percent_to_brightness(
                    g_light_manual_percent);
            } else {
                int16_t item = (int16_t)control_selected + encoder_delta;
                while (item < 0) {
                    item += 3;
                }
                control_selected = (uint8_t)(item % 3);
            }
        }

        if (button_clicked) {
            if (page == UI_PAGE_MENU) {
                page = (UiPage_t)(menu_selected + 1U);
                control_selected = 0U;
                control_editing = false;
            } else if (page == UI_PAGE_LIGHT) {
                if (control_selected == 0U && g_relay_ready) {
                    relay_set(RELAY_LIGHT_CHANNEL,
                              !relay_get_state(RELAY_LIGHT_CHANNEL));
                } else if (control_selected == 1U) {
                    light_set_auto(!g_light_auto_mode);
                    control_editing = false;
                } else if (control_selected == 2U &&
                           !g_light_auto_mode) {
                    control_editing = !control_editing;
                }
            } else if (page == UI_PAGE_SYSTEM) {
                g_ui_chinese = !g_ui_chinese;
            }
        }
        if (back_clicked && page != UI_PAGE_MENU) {
            if (control_editing) {
                control_editing = false;
            } else {
                page = UI_PAGE_MENU;
            }
        }
        if (g_light_auto_mode && control_editing) {
            control_editing = false;
        }

        if ((now - last_light_read) >= pdMS_TO_TICKS(200)) {
            last_light_read = now;
            light_valid =
                g_bh1750_ready && bh1750_read_lux(&light_lux);
        }

        if (g_ws2812b_ready &&
            (now - last_ws2812_update) >=
                pdMS_TO_TICKS(WS2812_UPDATE_MS)) {
            last_ws2812_update = now;
            if (g_light_auto_mode) {
                const uint8_t target =
                    ws2812_target_brightness(light_lux, light_valid);
                g_light_brightness = ws2812_smooth_brightness(
                    g_light_brightness, target);
            } else {
                g_light_brightness = light_percent_to_brightness(
                    g_light_manual_percent);
            }

            const bool light_power_on =
                g_relay_ready && relay_get_state(RELAY_LIGHT_CHANNEL);
            if (!light_power_on) {
                /* Relay 2 removed strip power. Force a fresh frame after the
                 * next power-on even if the brightness value is unchanged. */
                ws2812_frame_sent = false;
            } else if (!ws2812_frame_sent ||
                       g_light_brightness != ws2812_last_sent ||
                       (now - last_ws2812_refresh) >=
                           pdMS_TO_TICKS(WS2812_REFRESH_MS)) {
                g_ws2812b_ready =
                    ws2812b_show_white(g_light_brightness);
                if (g_ws2812b_ready) {
                    ws2812_last_sent = g_light_brightness;
                    ws2812_frame_sent = true;
                    last_ws2812_refresh = now;
                }
            }
        }

        if (environment_pending &&
            (now - environment_started_at) >=
                pdMS_TO_TICKS(ENV_CONVERSION_MS)) {
            environment_valid = environment_sensor_read(&environment);
            environment_pending = false;

        } else if (!environment_pending &&
                   (now - last_environment_start) >=
                       pdMS_TO_TICKS(ENV_SAMPLE_MS)) {
            last_environment_start = now;
            environment_started_at = now;
            environment_pending =
                g_environment_ready &&
                environment_sensor_start_measurement();
            if (!environment_pending) {
                environment_valid = false;
            }
        }

        const bool current_motion =
            g_pir_ready && pir_sensor_motion_detected();
        const bool current_pir_warmed_up =
            (now - app_started_at) >= pdMS_TO_TICKS(PIR_WARMUP_MS);
        if (current_motion != motion_detected ||
            current_pir_warmed_up != pir_warmed_up) {
            motion_detected = current_motion;
            pir_warmed_up = current_pir_warmed_up;
        }

        if (g_display_ready &&
            (now - last_ui_refresh) >= pdMS_TO_TICKS(UI_REFRESH_MS)) {
            last_ui_refresh = now;
            ui_status_build(&ui_status, &environment, environment_valid,
                            light_lux, light_valid, motion_detected,
                            pir_warmed_up, control_selected, control_editing,
                            firmware_version);
            ui_display_render(page,
                              page == UI_PAGE_MENU ? menu_selected
                                                   : control_selected,
                              &ui_status, false);
        }

        if ((now - last_sensor_snapshot) >=
            pdMS_TO_TICKS(SENSOR_SNAPSHOT_UPDATE_MS)) {
            last_sensor_snapshot = now;
            sensor_snapshot_publish(now, &environment, environment_valid,
                                    light_lux, light_valid, motion_detected,
                                    pir_warmed_up, g_light_brightness);
        }
    }
}

static void ui_status_build(UiStatus_t *status,
                            const EnvironmentReading_t *environment,
                            bool environment_valid,
                            uint16_t light_lux, bool light_valid,
                            bool motion_detected, bool pir_warmed_up,
                            uint8_t control_selected, bool control_editing,
                            uint32_t firmware_version) {
    memset(status, 0, sizeof(*status));
    status->temperature_centi_c = environment->temperature_centi_c;
    status->humidity_centi_percent = environment->humidity_centi_percent;
    status->pressure_pa = environment->pressure_pa;
    status->light_lux = light_lux;
    status->firmware_version = firmware_version;
    status->environment_valid = environment_valid;
    status->light_valid = light_valid;
    status->pir_ready = g_pir_ready;
    status->pir_warmed_up = pir_warmed_up;
    status->motion_detected = motion_detected;
    status->light_power_on =
        g_relay_ready && relay_get_state(RELAY_LIGHT_CHANNEL);
    status->relay1_on =
        g_relay_ready && relay_get_state(RELAY_UNUSED_CHANNEL);
    status->buzzer_on = g_buzzer_ready && buzzer_get_state();
    status->led_percent = light_brightness_to_percent(g_light_brightness);
    status->control_selected = control_selected;
    status->light_auto_mode = g_light_auto_mode;
    status->control_editing = control_editing;
    status->ui_chinese = g_ui_chinese;
}

/*---------------------------------------------------------------------------
 * vMonitorTask — system health monitoring
 *---------------------------------------------------------------------------*/

static void vMonitorTask(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        /* Monitor stack high-water marks (debug builds only) */
        #if (configUSE_TRACE_FACILITY == 1)
        {
            /* Log or check task stack usage */
            /* UBaseType_t stack_free = uxTaskGetStackHighWaterMark(NULL); */
        }
        #endif

        /* Monitor heap */
        size_t free_heap = xPortGetFreeHeapSize();
        (void)free_heap; /* Can be logged via debug UART */

        iwdg_refresh();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*---------------------------------------------------------------------------
 * app_tasks_init — create all tasks and primitives
 *---------------------------------------------------------------------------*/

void app_tasks_init(void) {
    /* Create communication primitives */
    g_cmd_queue   = xQueueCreate(QUEUE_CMD_LENGTH, sizeof(uint32_t));
    g_data_queue  = xQueueCreate(QUEUE_DATA_LENGTH, 64);
    g_event_group = xEventGroupCreate();
    g_rx_stream   = uart_comm_get_rx_stream();

    configASSERT(g_cmd_queue != NULL);
    configASSERT(g_data_queue != NULL);
    configASSERT(g_event_group != NULL);
    configASSERT(g_rx_stream != NULL);

    /* Create tasks */
    BaseType_t ret;

    ret = xTaskCreate(vCommTask, "Comm", STACK_COMM_TASK, NULL,
                      PRIO_COMM_TASK, NULL);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vControlTask, "Control", STACK_CONTROL_TASK, NULL,
                      PRIO_CONTROL_TASK, NULL);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vAppTask, "App", STACK_APP_TASK, NULL,
                      PRIO_APP_TASK, NULL);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vMonitorTask, "Monitor", STACK_MONITOR_TASK, NULL,
                      PRIO_MONITOR_TASK, NULL);
    configASSERT(ret == pdPASS);
}

/*---------------------------------------------------------------------------
 * cmd_handler_dispatch — process received frames
 *---------------------------------------------------------------------------*/

void cmd_handler_dispatch(const ProtoFrame_t *f) {
    switch (f->cmd) {
    case CMD_OTA_AVAILABLE:
        if (f->len >= 4) {
            uint32_t version;
            memcpy(&version, f->payload, 4);
            xQueueSend(g_cmd_queue, &version, 0);
            xEventGroupSetBits(g_event_group, EVENT_OTA_AVAILABLE);
        }
        break;

    case CMD_APP_MSG:
        if (f->len > 0 && f->len <= 64) {
            /* Queue items are fixed-size; zero-fill the tail so the text
             * command is NUL-terminated for the parser in vAppTask. */
            uint8_t msg[64] = {0};
            memcpy(msg, f->payload, f->len);
            xQueueSend(g_data_queue, msg, 0);
        }
        break;

    case CMD_GET_STATUS:
        {
            BootConfig_t cfg;
            uint32_t version = 0;
            if (ota_config_read(&cfg)) {
                version = cfg.fw_version;
            }
            cmd_handler_send_frame(CMD_STATUS_RSP, (uint8_t *)&version, 4);
        }
        break;

    case CMD_GET_SENSOR_SNAPSHOT:
        {
            SensorSnapshot_t snapshot;
            taskENTER_CRITICAL();
            snapshot = g_sensor_snapshot;
            taskEXIT_CRITICAL();
            cmd_handler_send_frame(CMD_SENSOR_SNAPSHOT_RSP,
                                   (const uint8_t *)&snapshot,
                                   sizeof(snapshot));
        }
        break;

    case CMD_RESET:
        NVIC_SystemReset();
        break;

    default:
        /* Unknown command — ignore in application context */
        break;
    }
}

void cmd_handler_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    uint8_t buf[PROTO_MAX_FRAME];
    uint16_t total = proto_build_frame(buf, sizeof(buf), cmd, payload, len);
    if (total > 0) {
        /* Send over UART in task context (blocking, but the payload is small). */
        uart_comm_send(buf, total);
    }
}

/*---------------------------------------------------------------------------
 * main entry point
 *---------------------------------------------------------------------------*/

int main(void) {
    ramfunc_init();

    /* VTOR already set in system_init, but set it here too defensively */
    SCB->VTOR = APP_BASE;

    system_init();
    iwdg_init();

    /* Drive active-low relay inputs to the inactive level before the slower
     * sensor/display initialization. This reduces reset-time transients, but
     * stable module power and input biasing are still hardware requirements. */
    g_relay_ready = relay_init();

    /* Initialize USART1 (PA9/PA10) for ESP32 communication. */
    uart_comm_init(115200U);

    /* PB5: WS2812B DIN through timing-critical GPIO bit-bang.
     * PB6/PB7: OLED, BH1750, AHT20 and BMP280 on I2C1.
     * PB13/PB15: ST7789 SCK/MOSI on SPI2; PB12/PB14/PA8: CS/DC/RST.
     * PA6/PA7: encoder A/B; encoder C is tied to GND.
     * PA1: active-low confirm button. PA4: active-low back button.
     * PB0: HC-SR501 output. */
    const bool i2c_ready = env_i2c_init();
    g_display_ready = ui_display_init(i2c_ready);
    if (i2c_ready) {
        g_bh1750_ready = bh1750_init();
        g_environment_ready = environment_sensor_init();
    }
    g_encoder_ready = rotary_encoder_init();
    g_back_button_ready = back_button_init();
    g_pir_ready = pir_sensor_init();
    g_buzzer_ready = buzzer_init();
    g_ws2812b_ready = ws2812b_init();
    (void)g_buzzer_ready;

    /* Create FreeRTOS primitives and tasks */
    app_tasks_init();

    /* Start the scheduler — does not return */
    vTaskStartScheduler();

    /* Should never reach here */
    while (1);
    return 0;
}

/*---------------------------------------------------------------------------
 * FreeRTOS hooks
 *---------------------------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    /* Halt with LED on solid */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    __disable_irq();
    while (1);
}

void vApplicationMallocFailedHook(void) {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    __disable_irq();
    while (1);
}

/*---------------------------------------------------------------------------
 * FreeRTOS static allocation support (configSUPPORT_STATIC_ALLOCATION = 1)
 * The idle and timer tasks are created by the kernel; with static
 * allocation enabled it needs static TCB/stack buffers for them.
 *---------------------------------------------------------------------------*/

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
    static StaticTask_t xIdleTaskTCB;
    static StackType_t  uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t  uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

/*---------------------------------------------------------------------------
 * FreeRTOS interrupt handler aliases.
 * The STM32 startup vector table references SVC_Handler / PendSV_Handler,
 * but the FreeRTOS ARM_CM3 port implements them as vPortSVCHandler /
 * xPortPendSVHandler. Alias them so the vector table links correctly.
 * SysTick_Handler is defined above (calls xPortSysTickHandler).
 *---------------------------------------------------------------------------*/

void vPortSVCHandler(void);
void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

/* Thin wrappers — same pattern as CubeMX-generated stm32f1xx_it.c */
void SVC_Handler(void)    { vPortSVCHandler(); }
void PendSV_Handler(void) { xPortPendSVHandler(); }

/*---------------------------------------------------------------------------
 * SysTick handler — FreeRTOS tick
 *---------------------------------------------------------------------------*/

void SysTick_Handler(void) {
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}
