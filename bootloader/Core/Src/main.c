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
 *
 * ===========================================================================
 * 中文导读：bootloader 是整台设备的"出题人"
 * ===========================================================================
 * 它只有 8KB,却决定了三件事:
 *   ① 这次上电跑谁     —— 应用(bootloader_run 的判定链)
 *   ② 怎么把新固件写进去 —— OTA 状态机(process_ota_*)
 *   ③ 写坏了怎么办     —— 恢复窗口 + 自动回滚上报
 *
 * ★注意上面那段 "Boot decision flow" 是简化版★,真实判定链有 7 步,
 * 见 bootloader_run() 第 590 行往下的注释。这里没写 CRC 复核、启动确认、
 * 恢复状态分级 —— 那些都是后来加的。
 *
 * 读这个文件的顺序建议:
 *   1. main()            (文件末尾, 只有 6 行)      —— 先看骨架
 *   2. bootloader_run()  (第 585 行起)              —— 核心决策链
 *   3. bootloader_jump_to_app() (第 265 行)         —— 面试必问的跳转
 *   4. process_ota_chunk() (第 415 行)              —— 真正写 Flash 的地方
 *   5. 其余按需
 *
 * 全文件是**单线程裸机**:没有 RTOS、没有任务、没有队列。所有并发风险
 * 都被"一次只做一件事"消掉了 —— 这是 bootloader 敢做这么简单的前提。
 * ===========================================================================
 */

#include "bootloader.h"
#include "stm32f1xx_hal.h"
#include "../../../shared/protocol.h"
#include "../../../shared/ota_config.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Hardware pin definitions (Blue Pill)
 *---------------------------------------------------------------------------*/
/* 中文：只有两个外设引脚 —— LED 和串口。
 * 为什么没有"强制进 bootloader"的按键引脚:早期版本读过 PB0,但 PB0 同时
 * 接的是 PIR 人体红外传感器,而它的空闲输出是低电平,导致**每次上电都停在
 * bootloader**。后来把这个读取删掉了 —— 这是"引脚复用冲突"的典型案例。
 * 现在进入维护模式只靠两条路:config 里的 OTA 标志,或 200ms 被动串口窗口。
 * 硬件兜底是 Blue Pill 自己的 BOOT0 跳帽(进 STM32 ROM bootloader)。 */

#define LED_PORT                GPIOC
#define LED_PIN                 GPIO_PIN_13    /* PC13 — onboard LED (active low) */
/* 中文：PC13 低电平点亮(active low)。所以 led_on() 写的是 RESET。 */

#define USART_BAUD              115200U

/*---------------------------------------------------------------------------
 * Static data (RAM only — bootloader is single-threaded)
 *---------------------------------------------------------------------------*/
/* 中文：全部是 static 全局,没有动态分配。
 * 注意 g_parser 一个就 1048 字节(内含 1028 字节的 payload 数组),
 * 在 20KB RAM 里占 5%。这是"协议用定长数组"的直接成本。 */

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
/* 中文：g_recovery_status 是"我为什么停在这里"的自述,由 CMD_RECOVERY_RSP
 * 发给 ESP32。ESP32 只有看到非 NORMAL 才会自动重推固件 —— 所以这个值判错
 * 会导致"把健康设备刷回旧版"(历史上真发生过,见 shared/protocol.h 里
 * 关于 CMD_STATUS_RSP 的说明)。 */

/* The vendor startup file initializes .data/.bss only. The linker places
 * flash-writing routines in .ramfunc, so copy that image explicitly before
 * any OTA flash operation can run. */
/* 中文：★这三个符号由链接脚本定义,不是变量声明★
 *   _siramfunc  .ramfunc 在 Flash 里的起始地址(LMA,源)
 *   _sramfunc   .ramfunc 在 RAM 里的起始地址(VMA,目标)
 *   _eramfunc   .ramfunc 在 RAM 里的结束地址
 * 厂商启动文件只负责拷 .data、清 .bss,**不管 .ramfunc** —— 所以必须
 * 自己写 ramfunc_init() 把机器码从 Flash 搬到 RAM。这是自己改链接脚本
 * 最容易漏的一步:漏了能编译能链接,但一擦 Flash 就死机。 */
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
/* 中文：★为什么这两个函数必须在 RAM 里执行(面试高频)★
 *
 * STM32F103 只有**一个 Flash Bank**,代码和数据在同一块 Flash。
 * 当 Flash 控制器正在擦除/写入时,CPU 从 Flash 取指令会被硬件挡停。
 * 如果"擦 Flash 的代码"本身也在 Flash 里,就等于在擦自己脚下的路 → 卡死。
 *
 * 解法:把这两个函数放进 .ramfunc 段。链接脚本里该段的 VMA 在 RAM、
 * LMA 在 Flash(> RAM AT > FLASH),启动时由 ramfunc_init() 拷过去。
 * `noinline` 防止编译器把它内联回调用方(内联了就又回到 Flash 里了)。
 *
 * 反过来说:如果芯片是双 Bank(F4/F7 部分型号),就不需要这个技巧。 */

/**
 * @brief Erase a single flash page. Runs from RAM.
 */
__attribute__((section(".ramfunc"), noinline))
static uint32_t flash_erase_page(uint32_t page_addr) {
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase   = FLASH_TYPEERASE_PAGES,
        .PageAddress = page_addr,
        .NbPages     = 1              /* 中文：一次 1 页 = 1KB,与分页粒度一致 */
    };
    uint32_t page_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    return (status == HAL_OK) ? 0 : page_error;   /* 中文：非 0 = 失败 */
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
    /* 中文：手写的 .ramfunc 拷贝循环。3 行,但少了它整个 OTA 就没法工作。
     * 为什么用 uint8_t* 逐字节拷:因为源码里没写对齐保证,逐字节最安全。 */
    const uint8_t *src = &_siramfunc;
    uint8_t *dst = &_sramfunc;

    while (dst < &_eramfunc) {
        *dst++ = *src++;
    }
}

static void system_init(void) {
    HAL_Init();              /* 中文：装 SysTick 时基、设 NVIC 优先级分组 */

    SystemClock_Config();    /* 中文：8MHz HSE → PLL×8 → 64MHz */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    /* 中文：★外设时钟默认是关的★ 不使能时钟就去操作那个外设,寄存器写不进去
     * 且不报错。这是新手最常见的"代码没错但没反应"的原因。 */

    /* LED PC13 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);
    led_off();               /* 中文：上电先灭灯,避免误导 */

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
    /* 中文：★为什么直接写寄存器而不调 HAL_IWDG_*★
     * 两个原因:① HAL 的 IWDG 模块需要 stm32f1xx_hal_conf.h 里打开
     * HAL_IWDG_MODULE_ENABLED,而 bootloader 想尽量少依赖配置;
     * ② IWDG 只有 4 个寄存器,写寄存器比调 HAL 更直观可控。
     * 这是"知道什么时候不该用 HAL"的一个例子。 */
    IWDG->KR  = 0xCCCCU;   /* 中文：0xCCCC = 启动看门狗(启动后不能停) */
    IWDG->KR  = 0x5555U;   /* 中文：0x5555 = 解锁 PR/RLR 才能写 */
    IWDG->PR  = 4U;        /* 中文：预分频 /64 */
    IWDG->RLR = 2499U;     /* 中文：重装值 → 约 4.0s @ 标称 40kHz LSI */

    uint32_t guard = 100000U;
    while ((IWDG->SR != 0U) && (--guard != 0U)) {
        /* 中文：等预分频/重装值真正生效。带 guard 计数是为了万一 LSI 没起振
         * 也不会死等在这。 */
    }

    IWDG->KR = 0xAAAAU;    /* 中文：0xAAAA = 喂狗(把计数器重装为 RLR) */
}

static void iwdg_refresh(void) {
    IWDG->KR = 0xAAAAU;
}
/* 中文：★看门狗在 bootloader 和 app 之间是连续的★
 * IWDG 在 NVIC_SystemReset() 之后继续计数(只有彻底断电才清零),所以
 * 两个固件用完全相同的超时参数(PR=4, RLR=2499),各自独立喂狗。
 * 如果两边参数不一致,跳转的瞬间就会一边喂一边超时 → 随机复位。
 * 另外:LSE 是最坏情况下 60kHz,那时超时只有约 2.67s —— 所以恢复窗口
 * 那 10 秒里必须持续喂狗,见 wait_for_frame()。 */

/* 8MHz HSE crystal → PLL ×8 → SYSCLK 64MHz. */
static void SystemClock_Config(void) {
    /* 中文：为什么必须自己配时钟:复位后芯片跑 HSI 8MHz(内部 RC)。
     * 应用和 bootloader 都需要 64MHz —— 因为 WS2812B 的位翻转时序是按
     * 64MHz 手工标定的,主频变了灯带就废了。所以两边时钟配置必须逐字一致。 */
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL8;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1);   /* 中文：晶振起不来 = 硬件问题,死等(此时还没配好看的灯) */
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;    /* 中文：HCLK = 64MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV2;      /* 中文：APB1 = 32MHz(上限 36MHz) */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;      /* 中文：APB2 = 64MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        while (1);
    }
    /* 中文：FLASH_LATENCY_2 = 64MHz 下 Flash 需要 2 个等待周期。
     * 忘了设 → 取指跟不上主频 → 随机跑飞。改主频必须同步改这个。 */
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
    /* 中文：★这段注释是一个真实的坑★
     * bootloader 用同步轮询 wait_for_frame() 收数据。如果开了 RX 中断,
     * ISR 会先把 USART1->DR 读走,轮询循环再去看 RXNE 标志时数据已经没了
     * → 永远收不到完整帧。
     * 教训:同一个外设不能同时用中断和轮询两种方式驱动,谁先读走谁赢。
     * 对比:application 侧(有 RTOS)才用 DMA+IDLE 中断 → StreamBuffer → 任务。 */
}

/*---------------------------------------------------------------------------
 * LED helpers
 *---------------------------------------------------------------------------*/
/* 中文：LED 是 bootloader 唯一的"输出设备"。正常启动时它基本不亮;
 * 停在 OTA 模式或维护模式时常亮;出错时用闪烁次数报错误码(见
 * bootloader_halt_error)。没有串口的时候,这是唯一的诊断手段。 */

static void led_on(void)  { HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); }
static void led_off(void) { HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); }
static void led_toggle(void) { HAL_GPIO_TogglePin(LED_PORT, LED_PIN); }

static uint32_t elapsed_ms(void) {
    /* 中文：★相对时间,不是绝对时间★ 用 "当前 tick 减去开机 tick"。
     * 为什么:如果直接比较绝对 tick,开机后约 49 天会溢出回绕,超时判断
     * 就会永久失效。无符号减法在溢出时仍然正确 —— 这是个经典技巧。 */
    return HAL_GetTick() - g_ticks_at_boot;
}

/*---------------------------------------------------------------------------
 * UART helpers
 *---------------------------------------------------------------------------*/
/* 中文：发送全部是**阻塞**的 —— 等 TXE(发送寄存器空)再写 DR,一个字节
 * 一个字节推。为什么不 DMA:DMA 要配通道、要处理完成中断,而 bootloader
 * 发的东西很短(ACK 4~8 字节),不值得。而且阻塞写代码简单到不会错。
 * 代价:发 8 字节 @115200 约 0.7ms,期间什么都干不了 —— 对 bootloader
 * 来说无所谓。 */

static void uart_send_byte(uint8_t byte) {
    while (!(USART1->SR & USART_SR_TXE));   /* 中文：等发送寄存器空 */
    USART1->DR = byte;
}

static void uart_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    /* 中文：组帧用共享代码,发送用自己的阻塞循环。这样协议格式永远和
     * 应用/ESP32 一致,而发送方式按各自需求选。 */
    uint16_t total = proto_build_frame(g_frame_buf, sizeof(g_frame_buf),
                                       cmd, payload, len);
    for (uint16_t i = 0; i < total; i++) {
        uart_send_byte(g_frame_buf[i]);
    }
}

static void uart_send_nak(uint32_t expected_seq, uint32_t err_code) {
    /* 中文：★NAK 的 8 字节 = 期望序号 + 错误码★
     * 期望序号是 Go-Back-N 的思路:接收方用"我下次想收什么"代替"我收到了
     * 什么"。好处是当 ACK 丢了、对端重发时,bootloader 回一个"我期望第 N
     * 块",ESP32 立刻就知道其实前面已经成功了,不用盲目重发。
     * 错误码按"能不能重试"分类,让 ESP32 能自动决策。 */
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
/* 中文：★"固件是不是好的"有两层标准,这里是第一层★
 * 第一层:结构检查(app_is_valid)—— 只看向量表前两个字,极快。
 * 第二层:内容检查(app_image_crc_ok)—— 重算整个 app 区,慢但能抓"半新半旧"。
 * 为什么两层都要:传输中途掉电时,向量表所在那页可能还是旧的且完全合法,
 * 第一层会说 OK,跳进去必崩。详见 app_image_crc_ok 上面那段注释。 */

static bool app_is_valid(void) {
    /* 中文：Cortex-M 的向量表前两个字有固定含义:
     *   偏移 0:初始 MSP(栈顶地址)
     *   偏移 4:Reset_Handler 地址(Thumb 代码地址,最低位必须是 1) */
    uint32_t sp  = *(volatile uint32_t *)APP_BASE;
    uint32_t pc  = *(volatile uint32_t *)(APP_BASE + 4);

    /* SP must point into SRAM: 0x20000000 – 0x20005000 (20KB) */
    if (sp < 0x20000000U || sp > 0x20005000U) return false;

    /* PC must be within app flash region and have bit 0 set (Thumb) */
    if (pc < APP_BASE || pc > (APP_BASE + APP_SIZE)) return false;
    if ((pc & 1) == 0) return false;
    /* 中文：★最后这一位检查是 Cortex-M 的硬性规定★
     * Thumb 指令集的函数地址最低位必须是 1(表示"这是 Thumb 代码")。
     * 如果这里是 0,说明该位置根本没有有效的 Reset_Handler —— 常见于
     * Flash 被擦成 0xFF 或 0x00 的情况。这是最省事的"空片检测"。 */

    return true;
}

void bootloader_jump_to_app(void) {
    /* 中文：★这是面试最爱问的一段。九步,每一步都有理由★
     * 核心问题:bootloader 和应用是两个独立编译、独立链接的程序,共享
     * 同一颗芯片。跳转时 CPU 的**全局状态**必须被重置成"应用期望的样子"。
     * 全局状态包括:中断开关(PRIMASK)、NVIC 的使能与挂起位、SysTick、
     * 外设配置、向量表位置(VTOR)、栈指针(MSP)。
     * 任何一项没清干净,应用都会以最难查的方式出问题。 */

    /* Disable interrupts before jump */
    __disable_irq();     /* 中文：① 关全局中断,防止跳转过程被中断打断 */

    /* Disable SysTick */
    SysTick->CTRL = 0;   /* 中文：② 关 SysTick —— 它是 bootloader 的时基 */

    /* Disable all peripheral interrupts */
    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
    }
    /* 中文：③ 关掉所有外设中断。8 个 32 位寄存器 × 32 = 256 个中断源,
     * 覆盖 Cortex-M3 的全部 IRQ。不清的话应用会继承 bootloader 开的中断。 */

    /* Clear pending interrupts */
    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    /* 中文：④ 清掉所有"已挂起但还没被服务"的中断请求。
     * ★这一步比③更容易被忽略★ 光关掉中断不解决"已经 pending"的那个:
     * 应用一开中断,那个悬挂的请求立刻触发,而它的处理函数可能假设
     * 自己已经初始化过了 → 直接崩。 */

    /* Reset USART1 to clean state for application */
    USART1->CR1 = 0;
    /* 中文：⑤ 把串口关掉。bootloader 用 115200 配过它,应用要重新配。
     * 不复位的话应用可能继承一个半配置状态(比如还在收上一个字节)。 */

    /* Set the vector table offset to application base */
    SCB->VTOR = APP_BASE;
    /* 中文：⑥ ★重定位中断向量表★
     * 复位后 VTOR = 0x08000000(bootloader 的向量表)。应用的中断处理函数
     * 地址在 0x08002000 开始的向量表里(由它的链接脚本决定)。
     * 不改 VTOR,应用一旦触发中断,CPU 就去 0x08000000+偏移 取地址 ——
     * 拿到的是 bootloader 的处理函数,而 bootloader 里根本没有那些函数
     * → 跑飞或 HardFault。 */

    /* Set main stack pointer from application vector table */
    __set_MSP(*(volatile uint32_t *)APP_BASE);
    /* 中文：⑦ ★从应用的向量表重设主栈指针★
     * 关键认知:**栈指针是 CPU 的全局硬件状态,函数跳转不会改变它。**
     * app_reset() 一进去就可能压栈(保存寄存器、调子函数),如果 MSP 还
     * 指着 bootloader 的栈,应用就在 bootloader 的栈空间上工作。
     * 为什么是"从向量表读"而不是"写死 0x20005000":两边栈顶由各自的链接
     * 脚本声明,可能不同。bootloader 不该替应用假设 —— 规范做法是读
     * 向量表第一个字,这也正是硬件复位时会做的事。 */

    /* PRIMASK survives a function jump.  The application uses SysTick and
     * PendSV for FreeRTOS, so it must receive interrupts enabled. */
    __enable_irq();
    /* 中文：⑧ ★最容易漏的一条★
     * __disable_irq() 设的是 PRIMASK 寄存器,它是 CPU 全局状态,
     * **函数调用/返回不会恢复它**。所以如果关了就跳,应用起来时中断还是
     * 关的 → FreeRTOS 的 SysTick 永远不来 → 调度器永远不切换任务 →
     * 系统"看起来启动了但什么都不干",而且不报任何错。这种 bug 极难查。 */

    /* Jump to application reset handler */
    void (*app_reset)(void) = (void (*)(void))(*(volatile uint32_t *)(APP_BASE + 4));
    app_reset();
    /* 中文：⑨ 把向量表第 2 个字(Reset_Handler 地址)当函数指针调用。
     * 注意这里**不是** 普通的函数调用语义:没有返回地址可回,app_reset()
     * 会一直跑下去。所以这行之后的东西永远不会执行。 */

    /* Never reached */
    while (1);
}

void bootloader_reset(void) {
    __disable_irq();
    NVIC_SystemReset();   /* 中文：软复位。★注意 IWDG 不会被它清零★ */
    while (1);
}

void bootloader_halt_error(uint32_t code) {
    /* 中文：★没有串口时的最后诊断手段:用 LED 闪 N 次表示错误码 N★
     * 配合 __disable_irq() 死循环,状态被彻底冻结 —— 便于用示波器/逻辑
     * 分析仪抓 PC13 的波形来数闪烁次数。比"什么都不做"强得多。
     * 注意这里刻意不喂看门狗,所以几秒后会被 IWDG 复位重来。 */
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
    /* 中文：对刚写完的 app 区算 CRC-32,用来和 OTA_BEGIN 里声明的值比对。
     * 算法上按 4 字节一批读、剩余不足 4 字节再逐字节 —— 比逐字节快,
     * 又不用管尾部对齐。注意用的是可累加的 proto_crc32(把上一次结果喂回去),
     * 所以不需要把 39KB 全放进内存。 */
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
    /* 中文：★★★ 这是整个"掉电不变砖"机制的第二层防线 ★★★
     *
     * 场景:传输到一半掉电。此时 app 区是"前几页是新固件、后面还是旧固件"。
     * 向量表所在的第 0 页可能还是旧的、而且完全合法 → app_is_valid() 说 OK
     * → 跳进去必然 HardFault 或跑飞。
     *
     * 这一层用 config 里记录的 image_size/image_crc32 重算整个 app 区,
     * 一比对就知道"这镜像不是完整的那一份"。
     *
     * 它能工作的前提:应用在**传输开始之前**就把期望大小写进了 config
     * (ota_config_request_update 的 image_size 参数,来自 CMD_OTA_AVAILABLE
     * 扩到 8 字节后新增的字段)。 */
    if (expected_size == 0U || expected_size > APP_SIZE) {
        /* No recorded size (pre-upgrade firmware) — nothing to verify against.
         * Treat as acceptable rather than refusing to boot a working device. */
        /* 中文：★"宁可放过,不可错杀"★
         * size 为 0 说明是升级前的老固件,没有可比对的基准。这时有两个选择:
         * 拒绝启动(把一台可能完全正常的设备锁死),或者放行(可能启动坏固件)。
         * 代码选放行 —— 这个取舍在 bootloader 里反复出现,是本文件的一条
         * 设计原则。 */
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

    return crc == expected_crc;   /* 中文：不匹配 = "半新半旧",不能跳 */
}

/*---------------------------------------------------------------------------
 * OTA command handlers
 *---------------------------------------------------------------------------*/

static void process_ota_begin(const ProtoFrame_t *f) {
    /* 中文：OTA_BEGIN 的 payload 是 12 字节:镜像大小 + 版本号 + CRC-32。
     * 这三个值后面全都用得上:
     *   image_size  → 校验总长度、防越界、事后复核
     *   version     → 提交成功后写进 config,成为 fw_version
     *   image_crc32 → 全部传完后比对整镜像 */
    if (f->len < 12) {
        uart_send_nak(0, ERR_UNKNOWN_CMD);
        return;
    }

    memcpy(&g_ota.image_size,  f->payload,      4);
    memcpy(&g_ota.version,     f->payload + 4,  4);
    memcpy(&g_ota.image_crc32, f->payload + 8,  4);

    /* Validate image fits in app region */
    if (g_ota.image_size == 0 || g_ota.image_size > APP_SIZE) {
        /* 中文：★第一道防线:声明的大小必须在 app 分区以内★
         * 不查的话,对端(或一个 bug)可以声明 1MB,把整个 Flash 写穿。 */
        uart_send_nak(0, ERR_SIZE_TOO_LARGE);
        return;
    }

    g_ota.expected_seq  = 0;
    g_ota.bytes_written = 0;
    g_state = ST_OTA_ACTIVE;

    uart_send_frame(CMD_OTA_BEGIN_ACK, (const uint8_t *)&g_ota.expected_seq, 4);
    /* 中文：BEGIN_ACK 里带的是"期望的第一个序号"(0)。
     * ESP32 收到这个才算握手完成,然后开始发第 0 块。 */
}

static void process_ota_chunk(const ProtoFrame_t *f) {
    /* 中文：★这是真正写 Flash 的地方,也是最需要防御性编程的地方★
     * 每一道检查都对应一种真实可能的坏输入。 */

    if (g_state != ST_OTA_ACTIVE) {
        uart_send_nak(0, ERR_BUSY);   /* 中文：没在升级流程里,不收数据块 */
        return;
    }

    if (f->len < 4) {
        uart_send_nak(g_ota.expected_seq, ERR_UNKNOWN_CMD);  /* 连序号都没有 */
        return;
    }

    uint32_t seq;
    memcpy(&seq, f->payload, 4);

    if (seq != g_ota.expected_seq) {
        /* 中文：★序号必须严格连续★ 这就是"停等协议"的核心检查。
         * 不连续说明:要么对端漏发了,要么上一个 ACK 丢了导致对端重发。
         * 回 NAK 时带上**我期望的序号**,对端据此就能判断该从哪里续 ——
         * 如果它重发的那块其实已经写进去了,它会发现自己已经领先,
         * 于是跳过。这就是 Go-Back-N 的好处。 */
        uart_send_nak(g_ota.expected_seq, ERR_SEQ_MISMATCH);
        return;
    }

    uint32_t addr = APP_BASE + (seq * FLASH_PAGE_SIZE);
    /* 中文：★地址换算把 Flash 几何写进了协议★
     * 序号 × 1KB 页大小 = 偏移。简单、无需传地址,但换到页大小不同的芯片
     * (比如 F4 的扇区大小不均)就必须改协议。这是明确记下的技术债。 */
    uint16_t data_len = f->len - 4; /* payload after seq */

    if (data_len == 0 || data_len > FLASH_PAGE_SIZE ||
        g_ota.bytes_written + data_len > g_ota.image_size) {
        /* 中文：★第二道防线:三个边界一起查★
         * ① data_len 不能是 0(空块没意义)
         * ② 不能超过一页(否则会写到下一块的地盘)
         * ③ 累计写入不能超过 BEGIN 时声明的大小
         * 第③条尤其重要:没有它,对端多发几块就能越过 app 分区边界,
         * 把 config 区甚至 bootloader 自己覆盖掉。 */
        uart_send_nak(g_ota.expected_seq, ERR_SIZE_TOO_LARGE);
        return;
    }

    /* Program this page */
    HAL_FLASH_Unlock();   /* 中文：解锁 Flash 控制器 */

    /* Erase the target page first */
    if (flash_erase_page(addr) != 0) {
        /* 中文：★擦除失败立刻停,不继续写★
         * 因为擦不干净就往里写,得到的是"新旧位混在一起"的数据,
         * 比直接失败更糟 —— 它会通过长度检查但过不了 CRC,排查起来更难。 */
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
            /* 中文：小端拼装 —— 和协议里多字节整数的字节序保持一致 */
        } else {
            /* Last odd byte: pad with 0xFF */
            hw = data[i] | 0xFF00;
            /* 中文：★为什么补 0xFF 而不是 0x00★
             * Flash 编程的物理特性:每个 bit 只能从 1 变成 0,不能反过来。
             * 要把"只有一个字节有效"凑成一个半字,补的那一半必须是全 1,
             * 也就是 0xFF,这样它等于"什么都没写",将来还能再改。
             * 补 0x00 就等于真的把那一半写成 0,再也擦不回来了(除非整页擦)。 */
        }

        if (flash_program_halfword(addr + i, hw) != HAL_OK) {
            HAL_FLASH_Lock();
            uart_send_nak(g_ota.expected_seq, ERR_FLASH_PROGRAM);
            return;
        }
    }

    HAL_FLASH_Lock();

    g_ota.bytes_written += data_len;
    g_ota.expected_seq++;     /* 中文：只有全部成功才推进序号 */
    /* 中文：★注意顺序★ 先写 Flash、都成功了、再推进序号。
     * 如果先推进序号再写,写失败了序号还是对的,对端就会以为这块成功了
     * → 静默丢数据。 */

    uart_send_chunk_ack(seq);
    /* 中文：回的是"刚写好的序号",而不是"下一个期望的序号"。
     * 两种约定都行,但要和 ESP32 侧一致 —— 这是协议的细节契约。 */
}

static void process_ota_end(const ProtoFrame_t *f) {
    /* 中文：★这是"提交"时刻:所有数据都写完了,现在决定这个镜像算不算数★
     * 两道验证:
     *   ① 总长度对不对(bytes_written == image_size)
     *   ② 整镜像 CRC 对不对
     * 两个都过才写 config、才跳转。任一不过都绝不跳进这个镜像。
     * (注意:这个函数没有用到参数 f —— CMD_OTA_END 的 payload 是空的,
     *  所有需要的信息都在 g_ota 里。) */
    if (g_state != ST_OTA_ACTIVE) {
        uart_send_nak(0, ERR_BUSY);
        return;
    }

    g_state = ST_OTA_VERIFY;

    if (g_ota.bytes_written != g_ota.image_size) {
        /* 中文：★第一道:长度对不上说明中间丢了块★
         * 注意这里为什么不能只看最后一块的 ACK —— 因为 ACK 可能丢,
         * 对端以为成功但实际没写。所以以"累计写入字节数"为准。 */
        uint32_t result_payload[2] = { OTA_RESULT_FAIL, g_ota.version };
        uart_send_frame(CMD_OTA_RESULT, (const uint8_t *)result_payload,
                        sizeof(result_payload));
        g_state = ST_ERROR;
        return;
    }

    /* Compute CRC-32 over the written image */
    uint32_t computed_crc = compute_image_crc();
    /* 中文：★第二道:重算整镜像 CRC,这是最终的真相★
     * 逐块的 ACK 只证明"每一块都写进去了",不证明"整体是对的"。
     * 只有把整块 Flash 读回来重算一遍,才算真的验证。 */

    uint8_t result_payload[8];
    if (computed_crc == g_ota.image_crc32) {
        /* Success: mark config and jump */
        g_state = ST_OTA_DONE;

        if (!ota_config_mark_valid(g_ota.version, g_ota.image_size, g_ota.image_crc32)) {
            /* 中文：★CRC 对了但配置写不进去 —— 最尴尬的情况★
             * 数据是对的,但"记录"没落盘。此时绝对不能跳:跳了之后
             * bootloader 下次上电读不到有效配置,就不知道这个镜像好不好。
             * 所以标记失败 + 报错 + 停机闪灯。 */
            ota_config_mark_failed();
            uint32_t ok = OTA_RESULT_FAIL;
            memcpy(result_payload, &ok, 4);
            memcpy(result_payload + 4, &g_ota.version, 4);
            uart_send_frame(CMD_OTA_RESULT, result_payload, 8);
            bootloader_halt_error(3);   /* 中文：闪 3 次 = 配置写入失败 */
        }

        uint32_t ok = OTA_RESULT_OK;
        memcpy(result_payload, &ok, 4);
        memcpy(result_payload + 4, &g_ota.version, 4);
        uart_send_frame(CMD_OTA_RESULT, result_payload, 8);

        /* Brief delay so ESP32 can receive the ACK */
        HAL_Delay(100);
        /* 中文：★为什么必须等 100ms★
         * uart_send_frame 是阻塞发送,但只保证"写进了移位寄存器",不保证
         * "对端收全了"。如果立刻跳进应用,应用的第一件事会重新初始化
         * USART1(CR1=0),把还没发完的字节截断 —— ESP32 就收不到这个
         * 关键的 OTA_RESULT,会以为升级失败。 */
        /* 中文：注意这里写进 config 的 boot_confirmed 是 0(见
         * ota_config_mark_valid),要等应用启动 3 秒后自己报平安才算数。 */

        /* Jump to the new application */
        bootloader_jump_to_app();
    } else {
        /* CRC mismatch */
        g_state = ST_ERROR;
        ota_config_mark_failed();   /* 中文：把失败记进 config,供事后诊断 */

        uint32_t ok = OTA_RESULT_FAIL;
        memcpy(result_payload, &ok, 4);
        memcpy(result_payload + 4, &g_ota.version, 4);
        uart_send_frame(CMD_OTA_RESULT, result_payload, 8);

        bootloader_halt_error(4);   /* 中文：闪 4 次 = 整镜像 CRC 不匹配 */
        /* 中文：★注意这里没有跳转★ app 区现在是一堆坏数据,跳进去必崩。
         * 而且此时 config 里的 image_crc32 还是旧值,下次上电 bootloader
         * 重算会发现不匹配 → 判 IMAGE_INVALID → 开 10 秒恢复窗口等重推。
         * 这就是"失败但不砖"的完整闭环。 */
    }
}

static void process_ota_abort(void) {
    /* 中文：主机主动放弃升级。标记失败然后复位 —— 复位后 config 里
     * 的 boot_mode 还是 OTA,但 update_status=FAILED,app 区是半截数据,
     * 于是 bootloader 会判 IMAGE_INVALID 并开恢复窗口。 */
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
    /* 中文：直接读 SR 寄存器的 RXNE 位,不调 HAL。
     * 非阻塞的含义:有字节就读走并返回 true,没有就立刻返回 false,
     * 绝不等待 —— 因为外层是一个带超时的轮询循环。 */
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
    /* 中文：★这是整个 bootloader 里唯一"等"的地方★
     * 结构是一个"忙等 + 喂狗"的轮询循环。三个要点:
     *
     * ① iwdg_refresh() 必须在这个循环里
     *    IWDG 超时约 4 秒,而恢复窗口是 10 秒。如果循环里不喂狗,
     *    还没等到主机重推固件,看门狗先把设备复位了 —— 表现为"在恢复窗口
     *    里反复重启",极难查。
     *
     * ② 解析是逐字节流式做的
     *    每读到一个字节就喂给共享的状态机。凑齐一帧需要很多次循环,
     *    中间任何时刻超时都会丢掉半截帧(解析器下次收到 0xA5 会自己重置)。
     *
     * ③ 返回值是指向解析器内部内存的指针
     *    调用方必须在下次 feed 之前用掉,不能存起来 —— 见 protocol.h 的说明。
     */
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
    /* =======================================================================
     * 中文：★ 函数全景图 —— 先看这张图，再往下读 ★
     * =======================================================================
     *
     * 这个函数是整份 bootloader 的心脏，分四个阶段：
     *
     *   ① 判定   读配置 + 记录本次启动 + 七步判定 → 得出 g_recovery_status
     *   ② 窗口   按恢复状态选一段"等主机开口"的时间（200ms / 2s / 10s），
     *             在这段时间里轮询串口等 CMD_OTA_BEGIN
     *   ③ 收固件 收到 OTA_BEGIN 后才进入：逐块接收 → 擦写 → 整镜像 CRC 校验
     *   ④ 维护   判定为"镜像坏了但主机没来救"时进入：死循环 + 心跳灯 +
     *             继续应答主机查询，等一次手动或自动重推
     *
     * 每个阶段在代码里都有中英双语的段落标题，用编辑器搜索这些标题即可跳转：
     *     "Boot decision"          → 第 ① 阶段
     *     "OTA handshake window"   → 第 ② 阶段
     *     "OTA active loop"        → 第 ③ 阶段
     *     "Maintenance mode"       → 第 ④ 阶段
     *
     * ★ 重要：这个函数【没有任何正常返回路径】★
     *   四个出口全部不返回，所以不要指望"读到函数末尾就结束了"：
     *     bootloader_jump_to_app()   跳到应用，永不回来
     *     bootloader_reset()         软复位（IWDG 计数器不清零）
     *     bootloader_halt_error(N)   关中断 + 闪 N 次灯 + 死循环
     *     维护模式的 while(1)         死循环
     *
     * -----------------------------------------------------------------------
     * 中文：★ 用五个真实场景走一遍同一段代码 ★
     * -----------------------------------------------------------------------
     * 同一份代码，五种处境，走五条完全不同的路。
     * 想快速理解这个函数，照着下面这张表对着代码走一遍最快：
     *
     *  场景                  判定结果            窗口    窗口内发生什么       最终去处
     *  ──────────────────────────────────────────────────────────────────────────
     *  A 正常上电            NORMAL              200ms   什么都没发生         跳进应用
     *                                                                        （启动只慢 200ms）
     *
     *  B 应用请求升级        OTA_PENDING         2s      ESP32 发 OTA_BEGIN   进 OTA 接收循环
     *                   （应用提前把标志                                       传完跳进新应用
     *                     写进 config 了）
     *
     *  C 传输中途掉电        IMAGE_INVALID       10s     ESP32 轮询发现问题   重推成功后
     *                   （app 区半新半旧）               并自动重推           新镜像正常跑起来
     *
     *  D 新固件启动即崩      前 3 次 NORMAL      200ms   前 3 次都跳进坏镜像   第 4 次起
     *                      第 4 次起            然后    然后崩（BKP 计数+1） 自动回滚到
     *                      BOOT_UNCONFIRMED     10s                          golden 镜像
     *
     *  E 镜像坏 + 主机不应答  IMAGE_INVALID       10s     超时，且判定不安全   进维护模式
     *                                                                      （设备还活着、
     *                                                                        串口还能连）
     *
     * ★ 场景 D 是整套机制最精彩的地方 ★
     *   同一个固件连续崩 3 次之后，第 4 次上电时 bootloader 就"学会"了：
     *   它不再信任这个镜像，改开 10 秒窗口等救援。三者分工是：
     *     看门狗      负责发现崩溃
     *     BKP 寄存器  负责记住崩溃（零 Flash 磨损）
     *     长窗口      负责把设备交还给能修它的主机（ESP32）
     *
     * ★ 场景 E 和"变砖"的区别 ★
     *   变砖 = 跳进坏固件 → 启动即崩 → 无限复位循环 → 连串口都没机会连上
     *   维护 = 停在 bootloader → 串口活着 → 能被查询、能被重刷
     *   这就是"需要重新刷"和"永远崩溃重启"的区别。
     *
     * -----------------------------------------------------------------------
     * 中文：为什么这里的注释不写代码行号
     * -----------------------------------------------------------------------
     * 行号会随任何一次编辑漂移，写死的行号很快就会变成错的（比不写更糟）。
     * 所以本文件用上面那些段落标题定位，而不是用行号。
     * ======================================================================= */

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
    /* 中文：★整个 bootloader 的心脏。判定顺序不是随便排的★
     *
     * 七个分支,每一个都对应一种"这台设备现在的处境":
     *
     *   1. 结构检查不过            → NO_APP            （没东西可跑）
     *   2. 应用主动要求升级         → OTA_PENDING       （正常升级流程）
     *   3. 配置读不出来             → NORMAL            （放行,见下面说明）
     *   4. 配置在但镜像 CRC 不符    → IMAGE_INVALID     （传到一半掉电）
     *   5. 字节对但从没成功启动过   → BOOT_UNCONFIRMED  （启动就崩）
     *   6. 其余                     → NORMAL            （一切正常）
     *
     * 顺序上的两个关键决定:
     *   · 第 2 条排在 CRC 检查之前:应用既然明确要求升级,当前镜像
     *     好不好已经不重要了。
     *   · 第 3 条也排在 CRC 检查之前:因为没有配置就**没有可比对的基准**,
     *     只能放行(宁可放过,不可错杀 —— 本文件反复出现的这条原则)。
     */

    BootConfig_t cfg;
    bool cfg_valid = ota_config_read(&cfg);

    (void)ota_config_record_boot_attempt(&g_boot_attempts);
    /* 中文：★这一句在判断之前,顺序很重要★
     * "这次启动了"是既成事实,不管后面判成什么都要记一笔。
     * 它只写 BKP 备份寄存器,不碰 Flash(零磨损),所以崩溃循环也不会
     * 磨损配置页。返回值 g_boot_attempts 用于第 5 条判断。 */

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
        /* 中文：★"半新半旧"检测★ 用 config 记录的 size/CRC 重算整个 app 区。
         * 这一步比 app_is_valid() 慢得多(要读 39KB),但只有它能抓住
         * "传输中途掉电"这种情况。 */
        g_recovery_status = RECOVERY_STATUS_IMAGE_INVALID;
    } else if (cfg.boot_confirmed == 0U &&
               g_boot_attempts > BOOT_ATTEMPT_LIMIT) {
        /* The image is byte-correct but has never managed to confirm a boot,
         * BOOT_ATTEMPT_LIMIT times in a row: it runs and dies. */
        /* 中文：★"字节全对但跑不起来"★
         * boot_confirmed 为 0 = 应用从来没成功报过平安。
         * 连续超过 3 次(每次都会让 BKP 计数 +1),就判定它启动即崩。
         * 注意判据是 ">" 而不是 ">=",即第 4 次上电才判定 —— 给偶发
         * 供电抖动留了余地。 */
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
    /* 中文：★LED 是给现场的人看的唯一信号★
     * 常亮 = "我停在 bootloader,而且原因不是正常上电"。
     * 正常启动时 LED 全程不亮,所以用户看到灯常亮就知道出事了。 */

    /* --- OTA handshake window --- */
    /* 中文：★三档窗口,按"这台设备现在的处境"选★
     * 窗口的作用:给主机一个机会在这段时间内发 CMD_OTA_BEGIN 进来。
     * 窗口越长,正常启动越慢 —— 所以只在需要救援时才拉长。 */

    if (g_state == ST_WAIT_HANDSHAKE) {
        uint32_t window;

        if (g_recovery_status == RECOVERY_STATUS_OTA_PENDING) {
            window = 2000U;               /* 2s: the host already knows */
            /* 中文：应用主动请求升级 → 主机(ESP32)已经知道要升级了,
             * 2 秒足够它发 OTA_BEGIN。 */
        } else if (g_recovery_status != RECOVERY_STATUS_NORMAL) {
            /* Long window so the host can notice and re-send firmware without
             * anyone touching the board. The device stays reachable here. */
            window = OTA_RECOVERY_WINDOW_MS;   /* 10 秒 */
            /* 中文：★这是"自动回滚"能成立的关键★
             * 设备明确广播"我坏了",并且保持可触达 10 秒,让 ESP32 的
             * 轮询能发现并自动重推上一版固件 —— 全程不需要人去按复位。 */
        } else {
            window = OTA_WINDOW_MS;       /* 200ms passive window */
            /* 中文：正常上电只等 200ms。因为"要升级"是通过 config 标志
             * 提前告诉它的,不靠这个窗口。 */
        }

        const ProtoFrame_t *f = wait_for_frame(window);

        if (f != NULL && f->cmd == CMD_OTA_BEGIN) {
            process_ota_begin(f);
        } else if (f != NULL && f->cmd == CMD_RECOVERY_CHECK) {
            /* The host asks "who is running, and do you need firmware?".
             * Answering from the bootloader means the application is not
             * running, which is itself information. Uses its own command
             * rather than CMD_STATUS_RSP, where the application answers with a
             * firmware version number — the two would collide (app v1 vs
             * RECOVERY_STATUS_OTA_PENDING). */
            /* 中文：★这条分支的存在本身就是一次事故的产物★
             * 原来 bootloader 和 app 共用 CMD_STATUS_RSP,但一个回恢复状态、
             * 一个回版本号,数值 1 撞车 → 健康设备被误判成需要恢复。
             * 现在恢复状态走专用命令 CMD_RECOVERY_RSP(0x89)。
             * 能让 bootloader 回答这条问题,本身就说明"应用没在跑"。 */
            uart_send_frame(CMD_RECOVERY_RSP,
                            (const uint8_t *)&g_recovery_status, 4);
            /* Keep the recovery window open for the host to act on it. */
            f = wait_for_frame(OTA_RECOVERY_WINDOW_MS);
            /* 中文：★回答完之后再等 10 秒★ —— 因为主机拿到"我需要固件"
             * 之后要重新发起传输,那需要时间。 */
            if (f != NULL && f->cmd == CMD_OTA_BEGIN) {
                process_ota_begin(f);
            }
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
            /* 中文：★★★ 整份代码里最重要的一条取舍 ★★★
             *
             * 超时之后只在**确知启动会失败**的三种情况下才留在 bootloader:
             *   IMAGE_INVALID / BOOT_UNCONFIRMED / NO_APP
             *
             * OTA_PENDING 超时**照样跳进应用**。理由:主机没应答不该让设备
             * 变砖。旧固件仍然是这台设备上最好的东西 —— 而且它自己会重试
             * 升级。留在 bootloader 反而让一台本来正常的设备彻底不可用。
             *
             * 面试话术:"主机没应答不该让设备变砖。" */
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
            /* 中文：★"需要重新刷"和"永远崩溃重启"是两回事★
             * 停在维护模式,设备还活着、还能被串口够到;跳进坏镜像则是无限
             * 崩溃循环,连诊断的机会都没有。 */
        }
    }

    /* --- OTA active loop --- */
    /* 中文：★接收循环。两个超时概念不要混★
     *   OTA_CHUNK_TIMEOUT_MS(2s)  —— 等**下一帧**最多等多久
     *   30000U(30s)               —— 整次传输**累计空闲**多久算彻底失联
     * 前者是"每次等待"的上限,后者是"整体放弃"的判据。只要还在收数据,
     * idle_start 就会被刷新,所以一次正常传输(几十秒)不会误触发 30 秒超时。
     * 这个"两个超时"的模式在网络协议里很常见(TCP 的 keepalive 同理)。 */

    if (g_state == ST_OTA_ACTIVE) {
        /* Process chunks until done or timeout */
        uint32_t idle_start = elapsed_ms();

        while (g_state == ST_OTA_ACTIVE) {
            const ProtoFrame_t *f = wait_for_frame(OTA_CHUNK_TIMEOUT_MS);

            if (f == NULL) {
                /* 30s total inactivity timeout */
                if (elapsed_ms() - idle_start > 30000U) {
                    /* 中文：★主机彻底失联 → 标记失败并复位★
                     * 复位后 config 里 update_status=FAILED、app 区是半截数据,
                     * bootloader 会判 IMAGE_INVALID,开 10 秒恢复窗口等重推。
                     * 也就是"自己把自己送进可救援状态"。 */
                    ota_config_mark_failed();
                    bootloader_reset();
                }
                continue;
            }

            idle_start = elapsed_ms(); /* Reset inactivity timer */
            /* 中文：收到任何一帧都刷新空闲计时器 —— 说明主机还活着 */

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
            /* 中文：★注意 process_ota_end 成功时会直接跳进应用,不会返回★
             * 所以循环条件 g_state == ST_OTA_ACTIVE 在那种情况下永远不会
             * 再被求值。这是"状态被函数内部改掉 + 不返回"的常见模式,
             * 读代码时要留意:循环的退出路径不止一条。 */
        }
    }

    /* --- Maintenance mode --- */
    /* 中文：★维护模式 = "我知道我坏了,而且我保持可被够到"★
     * 这里是一个死循环,但要区分它和"变砖":
     *   · 变砖 = 跳进坏固件 → 启动即崩 → 无限复位循环 → 串口都没机会连
     *   · 维护模式 = 停在 bootloader → 串口活着 → 能被查询、能被重刷
     * 这就是"需要重新刷"和"永远崩溃重启"的区别。
     * 心跳闪灯是给人看的现场信号。 */

    if (g_state == ST_MAINTENANCE) {
        /* Stay in bootloader, accept manual commands */
        led_off();
        while (1) {
            /* Heartbeat blink */
            led_toggle();
            for (volatile uint32_t d = 0; d < 1000000; d++);
            /* 中文：用空循环延时而不是 HAL_Delay —— 因为这里刻意不依赖
             * SysTick 中断的语义,而且 volatile 防止编译器把整个循环优化掉。 */

            /* Check for incoming frames */
            const ProtoFrame_t *f = wait_for_frame(500U);
            if (f != NULL) {
                switch (f->cmd) {
                case CMD_OTA_BEGIN:
                    process_ota_begin(f);
                    goto ota_active;
                    /* 中文：★这里用 goto 跳到循环外的 OTA 接收循环★
                     * 因为引导流程已经在前面走完了,现在要"半路插进" OTA 流程。
                     * 用 goto 而不是重构出函数,是嵌入式里常见的省事做法 ——
                     * 代价是可读性,好处是不用动栈、不用考虑状态传递。 */
                    break;
                case CMD_RECOVERY_CHECK:
                    /* Bootloader is running: the application is not. Report
                     * why, so the host knows whether to re-send firmware. */
                    uart_send_frame(CMD_RECOVERY_RSP,
                                    (const uint8_t *)&g_recovery_status, 4);
                    break;
                case CMD_GET_STATUS:
                    {
                        /* Report the real recovery reason, not a constant. This
                         * is the field the host polls to decide whether to
                         * re-send firmware; answering 0 here would make a device
                         * stuck in recovery look healthy forever. */
                        /* 中文：★这段注释记录了一个已修的真实缺陷★
                         * 原来这里硬编码回 0(NORMAL),结果一台卡在恢复里的
                         * 设备会永远"看起来健康",ESP32 就永远不会重推固件
                         * —— 自动恢复机制静默失效。现在回的是真实状态。 */
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
        /* 中文：★注意这是复制粘贴出来的第二份接收循环★
         * 和上面那段逻辑一样。这种重复是实现上的妥协(goto 不能跳进
         * if 块中间),读的时候知道"这两段是一样的"就行,不用分别理解。 */
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
    /* 中文：兜底分支。理论上走不到(上面每条路都会 jump / halt / 死循环),
     * 但保留它有两个意义:① 编译器不会因为"函数没有返回值"而报警;
     * ② 万一以后有人改出漏洞,行为是"尽量启动"而不是"莫名其妙停住"。 */
    if (app_is_valid()) {
        bootloader_jump_to_app();
    }
    bootloader_halt_error(1);   /* 中文：闪 1 次 = 无路可走 */
}

/*---------------------------------------------------------------------------
 * SysTick interrupt
 *---------------------------------------------------------------------------*/

/* HAL_Init() installs SysTick as the bootloader time base.  Without this
 * strong handler the startup file's weak default handler loops forever on
 * the first tick, so the OTA timeout can never reach the application jump. */
/* 中文：★这段注释讲的是链接期的"弱符号覆盖"机制★
 * 启动文件(startup_stm32f103xb.s)为每个中断向量都提供了一个**弱定义**
 * 的默认处理函数,内容是死循环。你在 C 里定义一个同名函数,链接器就会
 * 用你的覆盖它 —— 这叫 weak symbol override。
 *
 * 为什么必须覆盖 SysTick:HAL_GetTick() 依赖 SysTick 中断去累加 tick 计数。
 * 不覆盖的话,第一次 tick 就掉进那个死循环,elapsed_ms() 永远不增长,
 * 于是所有超时判断全部失效 —— 现象是"bootloader 卡死,既不跳转也不报错"。
 * 这类"弱符号没覆盖"的 bug 不报编译错,很难查。 */
void SysTick_Handler(void) {
    HAL_IncTick();
}

/*---------------------------------------------------------------------------
 * HAL UART MSP init callback (called by HAL_UART_Init)
 *---------------------------------------------------------------------------*/
/* 中文：MSP = MCU Support Package。HAL_UART_Init() 内部会回调这个函数,
 * 让你提供"这颗芯片上这个外设需要哪些引脚、哪个时钟、哪个 DMA 通道"。
 * 这是 HAL 的分层设计:**通用逻辑在 HAL 里,芯片相关配置回调给用户**。
 * 本项目里它做两件事:开 GPIOA 时钟、把 PA9/PA10 配成串口功能。 */

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        GPIO_InitTypeDef gpio = {0};

        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA9 = TX (alternate-function push-pull) */
        gpio.Pin       = GPIO_PIN_9;
        gpio.Mode      = GPIO_MODE_AF_PP;    /* 中文：交给外设控制(不是普通 GPIO) */
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &gpio);

        /* PA10 = RX (input floating) */
        gpio.Pin       = GPIO_PIN_10;
        gpio.Mode      = GPIO_MODE_INPUT;
        gpio.Pull      = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio);
        /* 中文：TX 用复用推挽(芯片要驱动这根线),
         * RX 用浮空输入(芯片只负责听)。方向搞反就收发不了。
         * 对比 application 侧:pins 一样,但那里还额外配了 DMA 通道。 */
    }
}

/*---------------------------------------------------------------------------
 * main entry point
 *---------------------------------------------------------------------------*/

int main(void) {
    ramfunc_init();
    /* 中文：① 必须最先 —— 后面任何擦 Flash 的调用都依赖它已经把
     * flash_erase_page / flash_program_halfword 拷到 RAM 了。 */
    system_init();
    /* 中文：② HAL_Init(装 SysTick 时基、设优先级分组)+ 64MHz 时钟 + LED + 外设时钟 */
    iwdg_init();
    /* 中文：③ 开看门狗(约 4s)。放在时钟之后 —— 虽然 IWDG 跑独立 LSI,
     * 但寄存器写序列需要前面的初始化稳定下来。 */

    uart_init();
    /* 中文：④ 串口 115200。注意它刻意不开 RX 中断(见 uart_init 的注释)。 */
    proto_parser_init(&g_parser);
    /* 中文：⑤ 清解析器。其实第 585 行的 bootloader_run() 里又清了一次 ——
     * 冗余但无害,属于防御性重复。 */

    /* Run the bootloader — does not return */
    bootloader_run();
    /* 中文：⑥ 进主流程。它要么跳进应用、要么复位、要么死循环 —— 总之不返回。 */

    /* Unreachable */
    return 0;
}
