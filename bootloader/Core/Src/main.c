/**
 * @file    main.c
 * @brief   Bootloader entry point and OTA state machine.
 *
 * This runs from 0x08000000 (8KB). It is bare-metal — no RTOS.
 * Flash programming functions live in RAM (.ramfunc) to work around
 * the STM32F103 single-bank constraint.
 *
 * Boot decision flow:
 *   1. Check config area for BOOT_MODE_OTA flag
 *   2. Wait OTA_WINDOW_MS for CMD_OTA_BEGIN from ESP32
 *   3. If neither condition applies → jump to application
 */

#include "bootloader.h"
#include "stm32f1xx_hal.h"
#include "../../../shared/protocol.h"
#include "../../../shared/ota_config.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Hardware pin definitions (Blue Pill)
 *---------------------------------------------------------------------------*/

#define LED_PORT                GPIOC
#define LED_PIN                 GPIO_PIN_13    /* PC13 — onboard LED (active low) */

#define USART_BAUD              115200U

/*---------------------------------------------------------------------------
 * Static data (RAM only — bootloader is single-threaded)
 *---------------------------------------------------------------------------*/

static UART_HandleTypeDef    g_huart2;
static ProtoParser_t         g_parser;
static BootState_t           g_state = ST_BOOT;
static OtaContext_t           g_ota;
static uint8_t               g_frame_buf[PROTO_MAX_FRAME];
static uint32_t              g_ticks_at_boot;  /* HAL tick at reset */

/* Why this boot is opening an OTA window — reported to the host in
 * CMD_STATUS_RSP so it can tell "all good" from "send me firmware". */
static uint32_t              g_recovery_status = RECOVERY_STATUS_NORMAL;
static uint32_t              g_boot_attempts;

/* The vendor startup file initializes .data/.bss only. The linker places
 * flash-writing routines in .ramfunc, so copy that image explicitly before
 * any OTA flash operation can run. */
extern uint8_t _siramfunc;
extern uint8_t _sramfunc;
extern uint8_t _eramfunc;

/*---------------------------------------------------------------------------
 * Forward declarations
 *---------------------------------------------------------------------------*/

static void system_init(void);
static void SystemClock_Config(void);
static void ramfunc_init(void);
static void iwdg_init(void);
static void iwdg_refresh(void);
static void uart_init(void);
static void led_on(void);
static void led_off(void);
static void led_toggle(void);
static uint32_t elapsed_ms(void);
static void uart_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len);
static void uart_send_nak(uint32_t expected_seq, uint32_t err_code);
static void uart_send_chunk_ack(uint32_t seq);
static void process_ota_begin(const ProtoFrame_t *f);
static void process_ota_chunk(const ProtoFrame_t *f);
static void process_ota_end(const ProtoFrame_t *f);
static void process_ota_abort(void);
static uint32_t compute_image_crc(void);

/*---------------------------------------------------------------------------
 * Flash operations in RAM (.ramfunc)
 *---------------------------------------------------------------------------*/

/**
 * @brief Erase a single flash page. Runs from RAM.
 */
__attribute__((section(".ramfunc"), noinline))
static uint32_t flash_erase_page(uint32_t page_addr) {
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase   = FLASH_TYPEERASE_PAGES,
        .PageAddress = page_addr,
        .NbPages     = 1
    };
    uint32_t page_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    return (status == HAL_OK) ? 0 : page_error;
}

/**
 * @brief Program a halfword to flash. Runs from RAM.
 */
__attribute__((section(".ramfunc"), noinline))
static HAL_StatusTypeDef flash_program_halfword(uint32_t addr, uint16_t data) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, data);
}

/*---------------------------------------------------------------------------
 * Hardware init
 *---------------------------------------------------------------------------*/

static void ramfunc_init(void) {
    const uint8_t *src = &_siramfunc;
    uint8_t *dst = &_sramfunc;

    while (dst < &_eramfunc) {
        *dst++ = *src++;
    }
}

static void system_init(void) {
    HAL_Init();

    SystemClock_Config();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* LED PC13 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);
    led_off();

}

/*---------------------------------------------------------------------------
 * Independent watchdog (IWDG)
 *
 * Runs off the ~40kHz LSI, independent of SysTick and interrupts, so it
 * still fires if the CPU gets stuck in a disabled-interrupt loop or an
 * unhandled fault. Register-level (not HAL_IWDG_*) so it does not depend
 * on HAL_IWDG_MODULE_ENABLED being set in stm32f1xx_hal_conf.h.
 *
 * IWDG keeps counting across NVIC_SystemReset() (only a power-on reset
 * clears it), including the bootloader→application jump, so both binaries
 * arm it with the same ~4s timeout and refresh it independently.
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

/* 8MHz HSE crystal → PLL ×8 → SYSCLK 64MHz. */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL8;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1);
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        while (1);
    }
}

static void uart_init(void) {
    g_huart2.Instance          = USART1;
    g_huart2.Init.BaudRate     = USART_BAUD;
    g_huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    g_huart2.Init.StopBits     = UART_STOPBITS_1;
    g_huart2.Init.Parity       = UART_PARITY_NONE;
    g_huart2.Init.Mode         = UART_MODE_TX_RX;
    g_huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    g_huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&g_huart2);

    /* The bootloader receives synchronously in wait_for_frame(). Do not
     * enable RX interrupts: an ISR would consume DR before that polling
     * state machine sees the completed frame. */
}

/*---------------------------------------------------------------------------
 * LED helpers
 *---------------------------------------------------------------------------*/

static void led_on(void)  { HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); }
static void led_off(void) { HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); }
static void led_toggle(void) { HAL_GPIO_TogglePin(LED_PORT, LED_PIN); }

static uint32_t elapsed_ms(void) {
    return HAL_GetTick() - g_ticks_at_boot;
}

/*---------------------------------------------------------------------------
 * UART helpers
 *---------------------------------------------------------------------------*/

static void uart_send_byte(uint8_t byte) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = byte;
}

static void uart_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    uint16_t total = proto_build_frame(g_frame_buf, sizeof(g_frame_buf),
                                       cmd, payload, len);
    for (uint16_t i = 0; i < total; i++) {
        uart_send_byte(g_frame_buf[i]);
    }
}

static void uart_send_nak(uint32_t expected_seq, uint32_t err_code) {
    uint8_t payload[8];
    memcpy(payload, &expected_seq, 4);
    memcpy(payload + 4, &err_code, 4);
    uart_send_frame(CMD_NAK, payload, 8);
}

static void uart_send_chunk_ack(uint32_t seq) {
    uart_send_frame(CMD_CHUNK_ACK, (const uint8_t *)&seq, 4);
}

/*---------------------------------------------------------------------------
 * Application validation and jump
 *---------------------------------------------------------------------------*/

static bool app_is_valid(void) {
    uint32_t sp  = *(volatile uint32_t *)APP_BASE;
    uint32_t pc  = *(volatile uint32_t *)(APP_BASE + 4);

    /* SP must point into SRAM: 0x20000000 – 0x20005000 (20KB) */
    if (sp < 0x20000000U || sp > 0x20005000U) return false;

    /* PC must be within app flash region and have bit 0 set (Thumb) */
    if (pc < APP_BASE || pc > (APP_BASE + APP_SIZE)) return false;
    if ((pc & 1) == 0) return false;

    return true;
}

void bootloader_jump_to_app(void) {
    /* Disable interrupts before jump */
    __disable_irq();

    /* Disable SysTick */
    SysTick->CTRL = 0;

    /* Disable all peripheral interrupts */
    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
    }

    /* Clear pending interrupts */
    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    /* Reset USART1 to clean state for application */
    USART1->CR1 = 0;

    /* Set the vector table offset to application base */
    SCB->VTOR = APP_BASE;

    /* Set main stack pointer from application vector table */
    __set_MSP(*(volatile uint32_t *)APP_BASE);

    /* PRIMASK survives a function jump.  The application uses SysTick and
     * PendSV for FreeRTOS, so it must receive interrupts enabled. */
    __enable_irq();

    /* Jump to application reset handler */
    void (*app_reset)(void) = (void (*)(void))(*(volatile uint32_t *)(APP_BASE + 4));
    app_reset();

    /* Never reached */
    while (1);
}

void bootloader_reset(void) {
    __disable_irq();
    NVIC_SystemReset();
    while (1);
}

void bootloader_halt_error(uint32_t code) {
    __disable_irq();
    while (1) {
        /* Blink the error code on LED: N fast blinks, then pause */
        for (uint32_t i = 0; i < code; i++) {
            led_on();
            for (volatile uint32_t d = 0; d < 200000; d++);
            led_off();
            for (volatile uint32_t d = 0; d < 200000; d++);
        }
        /* Long pause between repetitions */
        for (volatile uint32_t d = 0; d < 2000000; d++);
    }
}

/*---------------------------------------------------------------------------
 * CRC over written image
 *---------------------------------------------------------------------------*/

static uint32_t compute_image_crc(void) {
    uint32_t crc = 0U;
    const uint8_t *addr = (const uint8_t *)APP_BASE;
    uint32_t remaining = g_ota.image_size;

    while (remaining >= 4) {
        uint32_t word = *(const volatile uint32_t *)addr;
        crc = proto_crc32((const uint8_t *)&word, 4, crc);
        addr += 4;
        remaining -= 4;
    }

    /* Trailing bytes */
    while (remaining > 0) {
        crc = proto_crc32(addr, 1, crc);
        addr++;
        remaining--;
    }

    return crc;
}

/**
 * @brief Recompute the CRC-32 of everything already sitting in the app region.
 *
 * Used at boot to tell a complete image apart from a half-transferred one. A
 * power loss in the middle of an OTA leaves the region part old, part new: the
 * vector table may still look valid, so `app_is_valid()` alone would happily
 * jump into a firmware that cannot run. Comparing against the CRC recorded when
 * the image was committed catches exactly that case.
 *
 * @param expected_size  Image size recorded in the config.
 * @param expected_crc   CRC-32 recorded in the config.
 */
static bool app_image_crc_ok(uint32_t expected_size, uint32_t expected_crc) {
    if (expected_size == 0U || expected_size > APP_SIZE) {
        /* No recorded size (pre-upgrade firmware) — nothing to verify against.
         * Treat as acceptable rather than refusing to boot a working device. */
        return true;
    }

    uint32_t crc = 0U;
    const uint8_t *addr = (const uint8_t *)APP_BASE;
    uint32_t remaining = expected_size;

    while (remaining >= 4U) {
        uint32_t word = *(const volatile uint32_t *)addr;
        crc = proto_crc32((const uint8_t *)&word, 4U, crc);
        addr += 4;
        remaining -= 4U;
    }
    while (remaining > 0U) {
        crc = proto_crc32(addr, 1U, crc);
        addr++;
        remaining--;
    }

    return crc == expected_crc;
}

/*---------------------------------------------------------------------------
 * OTA command handlers
 *---------------------------------------------------------------------------*/

static void process_ota_begin(const ProtoFrame_t *f) {
    if (f->len < 12) {
        uart_send_nak(0, ERR_UNKNOWN_CMD);
        return;
    }

    memcpy(&g_ota.image_size,  f->payload,      4);
    memcpy(&g_ota.version,     f->payload + 4,  4);
    memcpy(&g_ota.image_crc32, f->payload + 8,  4);

    /* Validate image fits in app region */
    if (g_ota.image_size == 0 || g_ota.image_size > APP_SIZE) {
        uart_send_nak(0, ERR_SIZE_TOO_LARGE);
        return;
    }

    g_ota.expected_seq  = 0;
    g_ota.bytes_written = 0;
    g_state = ST_OTA_ACTIVE;

    uart_send_frame(CMD_OTA_BEGIN_ACK, (const uint8_t *)&g_ota.expected_seq, 4);
}

static void process_ota_chunk(const ProtoFrame_t *f) {
    if (g_state != ST_OTA_ACTIVE) {
        uart_send_nak(0, ERR_BUSY);
        return;
    }

    if (f->len < 4) {
        uart_send_nak(g_ota.expected_seq, ERR_UNKNOWN_CMD);
        return;
    }

    uint32_t seq;
    memcpy(&seq, f->payload, 4);

    if (seq != g_ota.expected_seq) {
        uart_send_nak(g_ota.expected_seq, ERR_SEQ_MISMATCH);
        return;
    }

    uint32_t addr = APP_BASE + (seq * FLASH_PAGE_SIZE);
    uint16_t data_len = f->len - 4; /* payload after seq */

    if (data_len == 0 || data_len > FLASH_PAGE_SIZE ||
        g_ota.bytes_written + data_len > g_ota.image_size) {
        uart_send_nak(g_ota.expected_seq, ERR_SIZE_TOO_LARGE);
        return;
    }

    /* Program this page */
    HAL_FLASH_Unlock();

    /* Erase the target page first */
    if (flash_erase_page(addr) != 0) {
        HAL_FLASH_Lock();
        uart_send_nak(g_ota.expected_seq, ERR_FLASH_ERASE);
        return;
    }

    /* Write data as halfwords */
    const uint8_t *data = f->payload + 4;
    for (uint16_t i = 0; i < data_len; i += 2) {
        uint16_t hw;

        if (i + 1 < data_len) {
            hw = data[i] | ((uint16_t)data[i + 1] << 8);
        } else {
            /* Last odd byte: pad with 0xFF */
            hw = data[i] | 0xFF00;
        }

        if (flash_program_halfword(addr + i, hw) != HAL_OK) {
            HAL_FLASH_Lock();
            uart_send_nak(g_ota.expected_seq, ERR_FLASH_PROGRAM);
            return;
        }
    }

    HAL_FLASH_Lock();

    g_ota.bytes_written += data_len;
    g_ota.expected_seq++;

    uart_send_chunk_ack(seq);
}

static void process_ota_end(const ProtoFrame_t *f) {
    if (g_state != ST_OTA_ACTIVE) {
        uart_send_nak(0, ERR_BUSY);
        return;
    }

    g_state = ST_OTA_VERIFY;

    if (g_ota.bytes_written != g_ota.image_size) {
        uint32_t result_payload[2] = { OTA_RESULT_FAIL, g_ota.version };
        uart_send_frame(CMD_OTA_RESULT, (const uint8_t *)result_payload,
                        sizeof(result_payload));
        g_state = ST_ERROR;
        return;
    }

    /* Compute CRC-32 over the written image */
    uint32_t computed_crc = compute_image_crc();

    uint8_t result_payload[8];
    if (computed_crc == g_ota.image_crc32) {
        /* Success: mark config and jump */
        g_state = ST_OTA_DONE;

        if (!ota_config_mark_valid(g_ota.version, g_ota.image_size, g_ota.image_crc32)) {
            ota_config_mark_failed();
            uint32_t ok = OTA_RESULT_FAIL;
            memcpy(result_payload, &ok, 4);
            memcpy(result_payload + 4, &g_ota.version, 4);
            uart_send_frame(CMD_OTA_RESULT, result_payload, 8);
            bootloader_halt_error(3);
        }

        uint32_t ok = OTA_RESULT_OK;
        memcpy(result_payload, &ok, 4);
        memcpy(result_payload + 4, &g_ota.version, 4);
        uart_send_frame(CMD_OTA_RESULT, result_payload, 8);

        /* Brief delay so ESP32 can receive the ACK */
        HAL_Delay(100);

        /* Jump to the new application */
        bootloader_jump_to_app();
    } else {
        /* CRC mismatch */
        g_state = ST_ERROR;
        ota_config_mark_failed();

        uint32_t ok = OTA_RESULT_FAIL;
        memcpy(result_payload, &ok, 4);
        memcpy(result_payload + 4, &g_ota.version, 4);
        uart_send_frame(CMD_OTA_RESULT, result_payload, 8);

        bootloader_halt_error(4);
    }
}

static void process_ota_abort(void) {
    g_state = ST_ERROR;
    ota_config_mark_failed();
    uart_send_frame(CMD_NAK, NULL, 0);
    bootloader_reset();
}

/*---------------------------------------------------------------------------
 * OTA window: wait for UART bytes with timeout
 *---------------------------------------------------------------------------*/

/**
 * @brief Try to read a byte from USART1 (non-blocking).
 * @return true if a byte was read into *byte.
 */
static bool uart_read_byte_nonblock(uint8_t *byte) {
    if (USART1->SR & USART_SR_RXNE) {
        *byte = (uint8_t)(USART1->DR & 0xFF);
        return true;
    }
    return false;
}

/**
 * @brief Wait up to `timeout_ms` for a complete valid frame.
 * @return Pointer to frame if received, NULL on timeout.
 */
static const ProtoFrame_t *wait_for_frame(uint32_t timeout_ms) {
    uint32_t start = elapsed_ms();
    uint8_t byte;

    while (elapsed_ms() - start < timeout_ms) {
        iwdg_refresh();
        if (uart_read_byte_nonblock(&byte)) {
            const ProtoFrame_t *f = proto_parser_feed(&g_parser, byte);
            if (f != NULL) {
                return f;
            }
        }
    }

    return NULL;
}

/*---------------------------------------------------------------------------
 * Main bootloader loop
 *---------------------------------------------------------------------------*/

void bootloader_run(void) {
    g_ticks_at_boot = HAL_GetTick();
    proto_parser_init(&g_parser);
    memset(&g_ota, 0, sizeof(g_ota));

    /* --- Boot decision ---
     *
     * The bootloader decides in this order:
     *   1. count this boot (backup register; survives watchdog/HardFault reset)
     *   2. an explicit OTA request wins
     *   3. otherwise the app must be structurally valid AND match the CRC that
     *      was recorded when it was committed, AND have confirmed a previous
     *      boot. Anything less means a transfer was interrupted or the image
     *      crash-loops, either of which needs the host to re-send firmware.
     */

    BootConfig_t cfg;
    bool cfg_valid = ota_config_read(&cfg);

    (void)ota_config_record_boot_attempt(&g_boot_attempts);

    if (!app_is_valid()) {
        g_recovery_status = RECOVERY_STATUS_NO_APP;
    } else if (cfg_valid && cfg.boot_mode == BOOT_MODE_OTA) {
        /* The application itself asked for this update. */
        g_recovery_status = RECOVERY_STATUS_OTA_PENDING;
    } else if (!cfg_valid) {
        /* Structurally valid app but no readable config: there is no recorded
         * CRC to check and no boot history, so boot it and let the app rebuild
         * the config. This also covers a config page that failed to program. */
        g_recovery_status = RECOVERY_STATUS_NORMAL;
    } else if (!app_image_crc_ok(cfg.image_size, cfg.image_crc32)) {
        g_recovery_status = RECOVERY_STATUS_IMAGE_INVALID;
    } else if (cfg.boot_confirmed == 0U &&
               g_boot_attempts > BOOT_ATTEMPT_LIMIT) {
        /* The image is byte-correct but has never managed to confirm a boot,
         * BOOT_ATTEMPT_LIMIT times in a row: it runs and dies. */
        g_recovery_status = RECOVERY_STATUS_BOOT_UNCONFIRMED;
    } else {
        g_recovery_status = RECOVERY_STATUS_NORMAL;
    }

    g_state = ST_WAIT_HANDSHAKE;

    if (g_recovery_status == RECOVERY_STATUS_OTA_PENDING) {
        led_on(); /* Solid LED = OTA mode */
    } else if (g_recovery_status != RECOVERY_STATUS_NORMAL) {
        led_on(); /* Solid LED = needs firmware */
    }

    /* --- OTA handshake window --- */

    if (g_state == ST_WAIT_HANDSHAKE) {
        uint32_t window;

        if (g_recovery_status == RECOVERY_STATUS_OTA_PENDING) {
            window = 2000U;               /* 2s: the host already knows */
        } else if (g_recovery_status != RECOVERY_STATUS_NORMAL) {
            /* Long window so the host can notice and re-send firmware without
             * anyone touching the board. The device stays reachable here. */
            window = OTA_RECOVERY_WINDOW_MS;
        } else {
            window = OTA_WINDOW_MS;       /* 200ms passive window */
        }

        const ProtoFrame_t *f = wait_for_frame(window);

        if (f != NULL && f->cmd == CMD_OTA_BEGIN) {
            process_ota_begin(f);
        } else if (f != NULL && f->cmd == CMD_GET_STATUS) {
            /* Host is polling — report why we are still here, then keep
             * waiting. A host that sees a non-NORMAL status should re-send the
             * last known good image. */
            uart_send_frame(CMD_STATUS_RSP,
                            (const uint8_t *)&g_recovery_status, 4);
            if (g_recovery_status != RECOVERY_STATUS_NORMAL) {
                /* Already in recovery: keep the long window open. */
                f = wait_for_frame(OTA_RECOVERY_WINDOW_MS);
            } else {
                f = wait_for_frame(5000U);
            }
            if (f != NULL && f->cmd == CMD_OTA_BEGIN) {
                process_ota_begin(f);
            }
        } else if (g_state == ST_WAIT_HANDSHAKE) {
            /* Timeout — jump to app if valid, otherwise stay */
            /* Jump unless we know booting would fail. `OTA_PENDING` means the
             * host simply did not answer in time — the existing application is
             * still the best thing on this device, and it is what retries the
             * update, so hand over. Only a torn image, an image that never
             * confirms a boot, or no image at all justifies staying here. */
            if (app_is_valid()
                && g_recovery_status != RECOVERY_STATUS_IMAGE_INVALID
                && g_recovery_status != RECOVERY_STATUS_BOOT_UNCONFIRMED
                && g_recovery_status != RECOVERY_STATUS_NO_APP) {
                bootloader_jump_to_app();
            }

            /* Nothing bootable, or the host was told this device needs
             * firmware and did not deliver it. Stay reachable in maintenance
             * mode rather than jumping into an image known to be bad — that is
             * the difference between "needs a re-flash" and "crash-loops
             * forever". */
            g_state = ST_MAINTENANCE;
        }
    }

    /* --- OTA active loop --- */

    if (g_state == ST_OTA_ACTIVE) {
        /* Process chunks until done or timeout */
        uint32_t idle_start = elapsed_ms();

        while (g_state == ST_OTA_ACTIVE) {
            const ProtoFrame_t *f = wait_for_frame(OTA_CHUNK_TIMEOUT_MS);

            if (f == NULL) {
                /* 30s total inactivity timeout */
                if (elapsed_ms() - idle_start > 30000U) {
                    ota_config_mark_failed();
                    bootloader_reset();
                }
                continue;
            }

            idle_start = elapsed_ms(); /* Reset inactivity timer */

            switch (f->cmd) {
            case CMD_OTA_CHUNK:
                process_ota_chunk(f);
                break;
            case CMD_OTA_END:
                process_ota_end(f);
                break;
            case CMD_OTA_ABORT:
                process_ota_abort();
                break;
            default:
                uart_send_nak(g_ota.expected_seq, ERR_UNKNOWN_CMD);
                break;
            }
        }
    }

    /* --- Maintenance mode --- */

    if (g_state == ST_MAINTENANCE) {
        /* Stay in bootloader, accept manual commands */
        led_off();
        while (1) {
            /* Heartbeat blink */
            led_toggle();
            for (volatile uint32_t d = 0; d < 1000000; d++);

            /* Check for incoming frames */
            const ProtoFrame_t *f = wait_for_frame(500U);
            if (f != NULL) {
                switch (f->cmd) {
                case CMD_OTA_BEGIN:
                    process_ota_begin(f);
                    goto ota_active;
                    break;
                case CMD_GET_STATUS:
                    {
                        /* Report the real recovery reason, not a constant. This
                         * is the field the host polls to decide whether to
                         * re-send firmware; answering 0 here would make a device
                         * stuck in recovery look healthy forever. */
                        uart_send_frame(CMD_STATUS_RSP,
                                        (const uint8_t *)&g_recovery_status, 4);
                    }
                    break;
                case CMD_RESET:
                    bootloader_reset();
                    break;
                default:
                    break;
                }
            }
        }
ota_active:
        /* Fall through to the OTA active loop above */
        if (g_state == ST_OTA_ACTIVE) {
            uint32_t idle_start = elapsed_ms();
            while (g_state == ST_OTA_ACTIVE) {
                const ProtoFrame_t *f = wait_for_frame(OTA_CHUNK_TIMEOUT_MS);
                if (f == NULL) {
                    if (elapsed_ms() - idle_start > 30000U) {
                        ota_config_mark_failed();
                        bootloader_reset();
                    }
                    continue;
                }
                idle_start = elapsed_ms();

                switch (f->cmd) {
                case CMD_OTA_CHUNK: process_ota_chunk(f); break;
                case CMD_OTA_END:   process_ota_end(f);   break;
                case CMD_OTA_ABORT: process_ota_abort();  break;
                default: uart_send_nak(g_ota.expected_seq, ERR_UNKNOWN_CMD); break;
                }
            }
        }
    }

    /* Should not reach here — but if we do, jump to app or halt */
    if (app_is_valid()) {
        bootloader_jump_to_app();
    }
    bootloader_halt_error(1);
}

/*---------------------------------------------------------------------------
 * SysTick interrupt
 *---------------------------------------------------------------------------*/

/* HAL_Init() installs SysTick as the bootloader time base.  Without this
 * strong handler the startup file's weak default handler loops forever on
 * the first tick, so the OTA timeout can never reach the application jump. */
void SysTick_Handler(void) {
    HAL_IncTick();
}

/*---------------------------------------------------------------------------
 * HAL UART MSP init callback (called by HAL_UART_Init)
 *---------------------------------------------------------------------------*/

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        GPIO_InitTypeDef gpio = {0};

        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA9 = TX (alternate-function push-pull) */
        gpio.Pin       = GPIO_PIN_9;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &gpio);

        /* PA10 = RX (input floating) */
        gpio.Pin       = GPIO_PIN_10;
        gpio.Mode      = GPIO_MODE_INPUT;
        gpio.Pull      = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}

/*---------------------------------------------------------------------------
 * main entry point
 *---------------------------------------------------------------------------*/

int main(void) {
    ramfunc_init();
    system_init();
    iwdg_init();

    uart_init();
    proto_parser_init(&g_parser);

    /* Run the bootloader — does not return */
    bootloader_run();

    /* Unreachable */
    return 0;
}
