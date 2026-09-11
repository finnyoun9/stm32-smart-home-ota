/**
 * @file    main.c
 * @brief   Standalone RTC → STOP → wake probe (milestone M1).
 *
 * Bare-metal, no RTOS. Runs from 0x08002000 so the existing bootloader
 * boots it exactly like the real application — flashing this does not
 * touch the bootloader.
 *
 * Each cycle: arm the RTC alarm, enter STOP, wake through EXTI line 17,
 * restore the 64MHz clock, then report the measured interval over UART.
 * The point is to prove the wake path survives the ~4s IWDG for 100+
 * consecutive cycles before any of this goes near the main application.
 *
 * Wiring for the probe run:
 *   PA9  → USB-TTL RX (115200 8N1), GND common. RX is not used.
 *   ST-Link: SWDIO/SWCLK/GND only — do NOT connect its supply pin, it
 *   would bypass the current-measurement boundary.
 *
 * Scope note: the current drawn here is NOT comparable to the 57mA Normal
 * baseline in docs/low-power-eco-demo-plan.md. That baseline is the full
 * application with OLED/TFT/sensors initialised; this probe initialises
 * none of them. This answers "does the wake path work", not "does the
 * product save power".
 */

#include "stm32f1xx_hal.h"
#include "../../../shared/protocol.h"

/*---------------------------------------------------------------------------
 * Probe configuration
 *---------------------------------------------------------------------------*/

#define SLEEP_MS            1500U   /* 44% margin against the worst-case
                                     * ~2.67s IWDG (LSI at 60kHz) */
#define MIN_MARGIN_TICKS    8U      /* an RTC register write costs up to 3
                                     * RTCCLK cycles; never arm an alarm
                                     * closer than this or it is missed */
#define DELTA_TOLERANCE     1U      /* ±1 tick: CNT read vs alarm match race */

/* Backup-domain magic. Survives software/IWDG reset, lost on power-on
 * reset (Blue Pill has no VBAT battery), which is exactly the
 * discrimination we want: counters persist across an unexpected reset so
 * "100 consecutive cycles" cannot be faked by a silent restart. */
#define BKP_MAGIC           0x51C0U

/*---------------------------------------------------------------------------
 * State
 *---------------------------------------------------------------------------*/

static uint32_t g_rtc_hz;           /* 1024 (LSE) or 1000 (LSI) */
static uint32_t g_sleep_ticks;
static bool     g_rtc_is_lse;

/*---------------------------------------------------------------------------
 * System clock — byte-for-byte the same 8MHz HSE → PLL×8 → 64MHz as the
 * application and bootloader. Called again after every STOP: waking from
 * STOP leaves SYSCLK on HSI 8MHz with HSE and the PLL off.
 *
 * Re-entry is safe. HAL_RCC_OscConfig() only refuses to touch HSE while
 * HSE is the current SYSCLK or PLL source (stm32f1xx_hal_rcc.c:368); after
 * a STOP wake the source is HSI, so it takes the normal restart path.
 *---------------------------------------------------------------------------*/

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
    clk.APB1CLKDivider = RCC_HCLK_DIV2;   /* PCLK1 = 32MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;   /* PCLK2 = 64MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        while (1);
    }
}

/*---------------------------------------------------------------------------
 * IWDG — identical configuration to the application and bootloader, so a
 * pass here transfers directly to M2 without re-tuning the watchdog.
 *
 * IWDG runs off the LSI and keeps counting through STOP. LSI is specified
 * at 30–60kHz, so RLR=2499 is ~4.0s nominal but only ~2.67s worst case.
 * The sleep window is sized against 2.67s, not 4.0s.
 *---------------------------------------------------------------------------*/

static void iwdg_init(void) {
    IWDG->KR  = 0xCCCCU;
    IWDG->KR  = 0x5555U;
    IWDG->PR  = 4U;
    IWDG->RLR = 2499U;

    uint32_t guard = 100000U;
    while ((IWDG->SR != 0U) && (--guard != 0U)) {
        /* wait for the prescaler/reload update to latch */
    }

    IWDG->KR = 0xAAAAU;
}

static void iwdg_refresh(void) {
    IWDG->KR = 0xAAAAU;
}

/*---------------------------------------------------------------------------
 * UART — USART1 TX only, 115200 8N1. Plain text, not protocol frames:
 * this goes to a USB-TTL adapter, not to the ESP32.
 *---------------------------------------------------------------------------*/

static UART_HandleTypeDef g_huart;

static void uart_init(void) {
    g_huart.Instance          = USART1;
    g_huart.Init.BaudRate     = 115200U;
    g_huart.Init.WordLength   = UART_WORDLENGTH_8B;
    g_huart.Init.StopBits     = UART_STOPBITS_1;
    g_huart.Init.Parity       = UART_PARITY_NONE;
    g_huart.Init.Mode         = UART_MODE_TX;
    g_huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    g_huart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&g_huart);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = GPIO_PIN_9;
        gpio.Mode  = GPIO_MODE_AF_PP;
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}

static void uart_putc(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint8_t)c;
}

static void uart_puts(const char *s) {
    while (*s != '\0') {
        uart_putc(*s++);
    }
}

static void uart_put_u32(uint32_t v) {
    char buf[10];
    uint8_t n = 0;

    if (v == 0U) {
        uart_putc('0');
        return;
    }
    while (v > 0U) {
        buf[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (n > 0U) {
        uart_putc(buf[--n]);
    }
}

/**
 * @brief Block until the shift register is empty.
 *
 * TXE only means the data register accepted the byte. Entering STOP on TXE
 * stops the peripheral clock mid-frame and truncates the last character.
 */
static void uart_wait_tx_complete(void) {
    while (!(USART1->SR & USART_SR_TC));
}

/*---------------------------------------------------------------------------
 * Reset cause — read before anything clears it. Nothing else in this
 * project writes RCC->CSR, so these flags are trustworthy.
 *---------------------------------------------------------------------------*/

static void print_reset_cause(void) {
    const uint32_t csr = RCC->CSR;
    bool first = true;

    /* Report every flag that is set rather than picking one: a power-on
     * reset also raises PINRSTF, and collapsing that loses information. */
    static const struct {
        uint32_t mask;
        const char *name;
    } causes[] = {
        { RCC_CSR_LPWRRSTF, "LPWR" },
        { RCC_CSR_WWDGRSTF, "WWDG" },
        { RCC_CSR_IWDGRSTF, "IWDG" },
        { RCC_CSR_SFTRSTF,  "SFT"  },
        { RCC_CSR_PORRSTF,  "POR"  },
        { RCC_CSR_PINRSTF,  "PIN"  },
    };

    for (uint8_t i = 0; i < (sizeof(causes) / sizeof(causes[0])); i++) {
        if ((csr & causes[i].mask) != 0U) {
            if (!first) {
                uart_putc('|');
            }
            uart_puts(causes[i].name);
            first = false;
        }
    }
    if (first) {
        uart_puts("NONE");
    }

    RCC->CSR |= RCC_CSR_RMVF;
}

/*---------------------------------------------------------------------------
 * RTC (register level)
 *
 * The F1 RTC is a plain 32-bit up-counter, not the BCD calendar of later
 * families. HAL_RTC_SetAlarm_IT() wraps it in a fake RTC_TimeTypeDef and
 * converts back and forth, which is more code than writing ALRH/ALRL
 * directly — and this file already uses register-level IWDG for the same
 * reason.
 *
 * Only the alarm (ALR) can wake STOP, and it reaches the NVIC through
 * EXTI line 17. RTC_IRQn's Second/Overflow interrupts cannot wake STOP.
 * The F1 alarm is one-shot (stm32f1xx_hal_rtc.c:76), so it is re-armed
 * every cycle.
 *---------------------------------------------------------------------------*/

static void rtc_wait_operation_off(void) {
    while (!(RTC->CRL & RTC_CRL_RTOFF));
}

static void rtc_enter_config(void) {
    rtc_wait_operation_off();
    RTC->CRL |= RTC_CRL_CNF;
}

static void rtc_exit_config(void) {
    RTC->CRL &= ~RTC_CRL_CNF;
    rtc_wait_operation_off();
}

/**
 * @brief Read the 32-bit counter across its two 16-bit halves.
 *
 * CNTL wraps every 64s at 1024Hz. Re-read if the high half moved between
 * the two accesses.
 */
static uint32_t rtc_counter(void) {
    uint32_t high, low;

    do {
        high = RTC->CNTH & 0xFFFFU;
        low  = RTC->CNTL & 0xFFFFU;
    } while (high != (RTC->CNTH & 0xFFFFU));

    return (high << 16) | low;
}

static void rtc_set_alarm(uint32_t target) {
    rtc_enter_config();
    RTC->ALRH = (target >> 16) & 0xFFFFU;
    RTC->ALRL = target & 0xFFFFU;
    rtc_exit_config();
}

/**
 * @brief Bring up the RTC, preferring LSE.
 *
 * Returns with g_rtc_hz / g_sleep_ticks / g_rtc_is_lse set.
 *
 * A backup-domain reset would wipe the BKP cycle counters, so it only runs
 * on a genuinely cold start (magic absent). After a software or IWDG reset
 * the RTC is already configured and still running — leave it alone and
 * keep the counters.
 */
static void rtc_init(void) {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    PWR->CR |= PWR_CR_DBP;      /* unlock the backup domain for writes */

    if ((BKP->DR10 & 0xFFFFU) == BKP_MAGIC) {
        /* Warm restart: RTC already ticking from the previous boot. */
        g_rtc_is_lse = ((RCC->BDCR & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL_LSE);
    } else {
        RCC->BDCR |= RCC_BDCR_BDRST;
        RCC->BDCR &= ~RCC_BDCR_BDRST;

        /* Try LSE first. The IWDG is already armed and keeps counting here,
         * so refresh it inside the wait — a 32.768kHz crystal can take
         * hundreds of milliseconds to start. */
        RCC->BDCR |= RCC_BDCR_LSEON;

        const uint32_t start = HAL_GetTick();
        while (!(RCC->BDCR & RCC_BDCR_LSERDY)) {
            iwdg_refresh();
            if ((HAL_GetTick() - start) > LSE_STARTUP_TIMEOUT) {
                break;
            }
        }

        g_rtc_is_lse = ((RCC->BDCR & RCC_BDCR_LSERDY) != 0U);

        if (g_rtc_is_lse) {
            RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCSEL_LSE;
        } else {
            RCC->BDCR &= ~RCC_BDCR_LSEON;
            RCC->CSR |= RCC_CSR_LSION;
            while (!(RCC->CSR & RCC_CSR_LSIRDY)) {
                iwdg_refresh();
            }
            RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCSEL_LSI;
        }

        RCC->BDCR |= RCC_BDCR_RTCEN;

        /* Wait for the APB1 shadow registers to mirror the RTC domain. */
        RTC->CRL &= ~RTC_CRL_RSF;
        while (!(RTC->CRL & RTC_CRL_RSF));

        /* LSE 32768 / 32 = 1024Hz; LSI ~40000 / 40 = ~1000Hz nominal. */
        rtc_enter_config();
        RTC->PRLH = 0U;
        RTC->PRLL = g_rtc_is_lse ? 31U : 39U;
        rtc_exit_config();

        BKP->DR1  = 0U;   /* wake count  */
        BKP->DR2  = 0U;   /* boot count  */
        BKP->DR3  = 0U;   /* fail count  */
        BKP->DR10 = BKP_MAGIC;
    }

    g_rtc_hz      = g_rtc_is_lse ? 1024U : 1000U;
    g_sleep_ticks = (g_rtc_hz * SLEEP_MS) / 1000U;

    /* Route the alarm to the NVIC through EXTI line 17 — the only path
     * that wakes STOP. The line is internal; no AFIO_EXTICR needed. */
    EXTI->IMR  |= EXTI_IMR_MR17;
    EXTI->RTSR |= EXTI_RTSR_TR17;

    /* Clear anything stale so the first WFI is not skipped. */
    RTC->CRL &= ~RTC_CRL_ALRF;
    EXTI->PR  = EXTI_PR_PR17;

    rtc_enter_config();
    RTC->CRH |= RTC_CRH_ALRIE;
    rtc_exit_config();

    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
}

void RTC_Alarm_IRQHandler(void) {
    /* ALRF is rc_w0 and needs no config-mode unlock. Both this and the
     * EXTI pending bit must be cleared or the next WFI returns instantly. */
    RTC->CRL &= ~RTC_CRL_ALRF;
    EXTI->PR  = EXTI_PR_PR17;
}

/*---------------------------------------------------------------------------
 * Tick → milliseconds. Integer only: this part has no FPU, and the
 * project keeps all unit conversion in fixed point.
 *---------------------------------------------------------------------------*/

static uint32_t ticks_to_ms(uint32_t ticks) {
    return (ticks * 1000U) / g_rtc_hz;
}

/*---------------------------------------------------------------------------
 * main
 *---------------------------------------------------------------------------*/

int main(void) {
    SCB->VTOR = APP_BASE;

    HAL_Init();
    SystemClock_Config();

    /* Deliberately left alone: PC13. The onboard LED costs 2–3mA and would
     * pollute the current boundary, and the application has its heartbeat
     * disabled. Also deliberately not reconfiguring idle GPIOs to analog —
     * that would make the probe's pin state diverge from the application
     * and break comparability. Both belong to M2/M3, not here. */

    iwdg_init();
    uart_init();
    rtc_init();

    const uint32_t boots = BKP->DR2 + 1U;
    BKP->DR2 = boots;

    uint32_t wakes = BKP->DR1;
    uint32_t fails = BKP->DR3;

    uart_puts("\r\n[probe] boot rst=");
    print_reset_cause();
    uart_puts(" rtcsrc=");
    uart_puts(g_rtc_is_lse ? "LSE" : "LSI");
    uart_puts(" hz=");
    uart_put_u32(g_rtc_hz);
    uart_puts(" boots=");
    uart_put_u32(boots);
    uart_puts(" wakes=");
    uart_put_u32(wakes);
    uart_puts(" fails=");
    uart_put_u32(fails);
    uart_puts("\r\n");

    if (!g_rtc_is_lse) {
        uart_puts("[probe] WARN LSE did not start; LSI is uncalibrated "
                  "(30-60kHz) so act= is nominal only, not measured data\r\n");
    }

    /* Absolute-time scheduling: the target advances by a fixed step instead
     * of being computed from the counter each cycle. Wake latency, clock
     * restore and UART time therefore do not accumulate into the interval,
     * which is what makes a strict ±1 tick pass criterion meaningful. */
    uint32_t target    = rtc_counter() + g_sleep_ticks;
    uint32_t prev_wake = 0U;
    bool     have_prev = false;

    for (;;) {
        iwdg_refresh();

        rtc_set_alarm(target);
        uart_wait_tx_complete();

        HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

        /* Woken. SYSCLK is back on HSI 8MHz — restore 64MHz before doing
         * anything that depends on the clock (UART baud rate included). */
        SystemClock_Config();
        iwdg_refresh();

        const uint32_t wake = rtc_counter();

        wakes++;
        BKP->DR1 = wakes;

        uart_puts("[probe] n=");
        uart_put_u32(wakes);
        uart_puts(" req=");
        uart_put_u32(SLEEP_MS);
        uart_puts("ms");

        bool ok = true;

        if (have_prev) {
            const uint32_t delta = wake - prev_wake;
            uint32_t err = (delta > g_sleep_ticks) ? (delta - g_sleep_ticks)
                                                   : (g_sleep_ticks - delta);
            ok = (err <= DELTA_TOLERANCE);

            uart_puts(" d=");
            uart_put_u32(delta);
            uart_puts("t act=");
            uart_put_u32(ticks_to_ms(delta));
            uart_puts("ms");
        } else {
            /* First wake after this boot has no predecessor to measure
             * against. Not a failure, but not evidence either. */
            uart_puts(" d=- act=-");
        }

        prev_wake = wake;
        have_prev = true;

        /* Advance the schedule, then confirm the new target is still far
         * enough ahead. An RTC write costs up to 3 RTCCLK cycles, so an
         * alarm armed too close is simply missed — and because the match is
         * an exact CNT==ALR compare, a missed alarm means sleeping for 2^32
         * ticks. With ~5ms awake against a 1500ms window this should never
         * trigger; it is a safety net, and it reports itself if it does. */
        uint32_t next = target + g_sleep_ticks;
        const uint32_t now = rtc_counter();

        if ((int32_t)(next - now) < (int32_t)MIN_MARGIN_TICKS) {
            next = now + g_sleep_ticks;
            ok = false;
            uart_puts(" OVERRUN");
        }

        uart_puts(" awake=");
        uart_put_u32(ticks_to_ms(now - wake));
        uart_puts("ms ");

        if (ok) {
            uart_puts("OK");
        } else {
            uart_puts("FAIL");
            fails++;
            BKP->DR3 = fails;
        }
        uart_puts("\r\n");

        target = next;
    }
}
