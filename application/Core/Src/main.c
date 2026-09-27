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
 *
 * ===========================================================================
 * 中文导读：这是 STM32 应用固件的主文件（1114 行）
 * ===========================================================================
 * 它是 bootloader 跳过来之后运行的那个"真正的产品固件"。
 * 和 bootloader 最大的区别：**这里有 RTOS**。
 *
 * 整份文件按功能分成 8 块，建议一块一块读（每块都能独立看懂）：
 *
 *   块 1  文件头 + 全局对象 + 硬件初始化        ← 本注释往下到 system_init 结束
 *   块 2  vCommTask + cmd_handler_dispatch     收帧、分发命令
 *   块 3  vControlTask                         启动确认、触发 OTA
 *   块 4  vAppTask 的循环骨架                  一个周期里按顺序做什么
 *   块 5  vAppTask 的传感器采样 + 快照发布
 *   块 6  handle_control_command + 灯光映射     文本命令怎么变成继电器动作
 *   块 7  vMonitorTask + 诊断快照              喂狗、采栈水位
 *   块 8  app_tasks_init + 钩子 + main()       把上面这些组装起来
 *
 * 读的顺序建议：先块 1（硬件怎么起来的）→ 块 8（任务怎么建起来的）
 * → 再回头逐个看块 2/3/4/7（每个任务干什么）。
 *
 * ★ 一条必须先建立的概念：这是**多任务**程序 ★
 * bootloader 是"一条线跑到底"，而这里有 4 个任务在同时"看似并行"地跑。
 * 它们之间靠队列、事件组、临界区通信。读代码时时刻问自己：
 *   "这段代码跑在哪个任务里？它和谁共享数据？"
 * 想清楚这两点，这个文件的复杂度就消掉一半。
 *
 * 用编辑器搜索这些标题可以快速跳转：
 *   "块 1" … "块 8"（本文件的中文分块标记）
 * ===========================================================================
 */

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include <string.h>

/* 中文：include 分三组看，正好对应三个来源 —— */
#include "../../../shared/protocol.h"      /* 组 1：本项目共享层（通信契约） */
#include "../../../shared/ota_config.h"
#include "app_tasks.h"                     /* 组 2：本项目自己的模块 */
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
#include "ws2812b.h"                       /* 组 3：FreeRTOS 内核（上面 17-21 行） */

/*---------------------------------------------------------------------------
 * Pin definitions
 *---------------------------------------------------------------------------*/

#define LED_PORT        GPIOC
#define LED_PIN         GPIO_PIN_13
/* 中文：PC13 板载 LED。应用里它有两个用途：
 *   ① 正常运行时不亮（心跳任务已被移除，见 CHANGELOG）
 *   ② 栈溢出 / 内存分配失败时**常亮**，表示"我挂死了"（见文件末尾的钩子函数）
 * 所以"LED 常亮 = 出事了"是这个项目里统一的现场信号。 */

/*---------------------------------------------------------------------------
 * Communication handles
 *---------------------------------------------------------------------------*/
/* 中文：块 1 的第二部分 —— 全局对象。
 *
 * 这些变量在**所有任务之间共享**，是理解这个文件的关键。
 * 按用途分成四类（下面每段都有说明）：
 *   ① 任务间通信对象（队列 / 事件组 / 流缓冲）
 *   ② 外设初始化是否成功的标志（g_xxx_ready）
 *   ③ 跨任务的共享状态（快照、灯光、语言）
 *   ④ 诊断数据
 *
 * ★ 阅读约定：看到 g_ 开头的变量就是"全局状态"，要立刻问
 *   "谁写它？谁读它？会不会同时？" ★
 */

/**
 * @brief An OTA offer received from the ESP32 (CMD_OTA_AVAILABLE).
 *
 * Carries the image size as well as the version so the application can record
 * the expected size in the config *before* the transfer starts. Without it the
 * bootloader would have no size to check the image CRC against, and a transfer
 * interrupted by a power loss would be jumped into as if it were complete.
 */
typedef struct {
    uint32_t version;
    uint32_t size;
} OtaRequest_t;
/* 中文：这个 8 字节结构体就是 CMD_OTA_AVAILABLE 的 payload 在应用侧的形态。
 * 为什么要带 size：**在传输开始之前**就把"我期望收多大"写进 Flash 配置，
 * 这样传输中途掉电后，bootloader 才能重算 app 区 CRC、识别出"传了一半"。
 * 少了 size，那套掉电不变砖的机制就不成立。 */

static QueueHandle_t        g_cmd_queue;       /* OtaRequest_t: comm → control */
static QueueHandle_t        g_data_queue;      /* Raw bytes: comm → app */
static EventGroupHandle_t   g_event_group;
static StreamBufferHandle_t g_rx_stream;
/* 中文：① 任务间通信对象。四个各自的用法（详解见块 2 / 块 4）：
 *   g_cmd_queue   队列：Comm 任务 → Control 任务，传 OTA 请求（8 条容量）
 *   g_data_queue  队列：Comm 任务 → App 任务，传文本控制命令（16 条 × 64 字节）
 *   g_event_group 事件组：Comm → Control，只发"有固件要升级"这个信号，不带数据
 *   g_rx_stream   流缓冲：UART 中断 → Comm 任务，传原始字节流
 * 为什么要分这么多种：**因为语义不同**。
 *   要"每条都处理" → 队列；只是"通知一下" → 事件组；是字节流 → 流缓冲。
 *   （选了哪种机制，就等于选了哪种语义，这是设计决策不是随手挑的。） */

static bool                 g_display_ready;
static bool                 g_bh1750_ready;
static bool                 g_environment_ready;
static bool                 g_encoder_ready;
static bool                 g_back_button_ready;
static bool                 g_pir_ready;
static bool                 g_buzzer_ready;
static bool                 g_relay_ready;
static bool                 g_ws2812b_ready;
/* 中文：② 外设初始化标志。为什么需要它们：**某个传感器没插好，不该让整个
 * 设备起不来**。初始化失败就把对应的标志置 false，后面用到它的地方先查标志
 * 再决定做不做 —— 这叫"优雅降级"。例如 BH1750 没接好，灯带就不自动调光，
 * 但继电器、显示屏、OTA 全都照常工作。 */

static volatile SensorSnapshot_t g_sensor_snapshot;
static bool                 g_light_auto_mode = true;
static uint8_t              g_light_manual_percent = 50U;
static uint8_t              g_light_brightness;
static bool                 g_ui_chinese = true;
/* 中文：③ 跨任务的共享状态。
 *   g_sensor_snapshot  18 字节设备状态快照。App 任务写、Comm 任务读，
 *                      用临界区保护（volatile 只是告诉编译器"别优化掉"，
 *                      它**不**提供原子性，真正靠的是临界区）。
 *   g_light_auto_mode  灯带是否自动调光（由 BH1750 光照决定）
 *   g_light_manual_percent  手动模式下的亮度百分比
 *   g_light_brightness      实际写到灯带的通道值 0..255
 *   g_ui_chinese       界面语言（TFT 和网页共享这一个状态）
 * 注意这几个都是 App 任务在改 —— 呼应"执行器只有一个 owner"的设计。 */

/* 运行时诊断缓存：vMonitorTask 周期采样，CMD_DIAG_SNAPSHOT 直接读取，
 * 避免在命令处理路径里遍历任务列表。句柄顺序与 DIAG_TASK_IDX_* 一致。 */
static TaskHandle_t         g_diag_task_handles[DIAG_TASK_COUNT];
static DiagSnapshot_t       g_diag_snapshot;
/* 中文：④ 诊断数据（这段中文注释是项目原有的）。
 * 为什么要缓存而不是"收到命令时现采"：遍历 FreeRTOS 任务列表需要一块
 * 临时数组（4 个任务就要 100 多字节），而本应用 RAM 已用 97.3%。
 * 所以改成 Monitor 任务每 5 秒采一次存起来，命令处理路径只做一次 22 字节拷贝。 */

/* The CMSIS startup file copies .data only. ota_config.c places the short
 * Flash erase/program wrappers in .ramfunc, so initialize that section before
 * the control task can write the OTA configuration page. */
/* 中文：★和 bootloader 里一模一样的机制★
 * 这三个符号由链接脚本定义，不是变量：
 *   _siramfunc  .ramfunc 段在 Flash 里的源地址（LMA）
 *   _sramfunc   .ramfunc 段在 RAM 里的目标地址（VMA）
 *   _eramfunc   .ramfunc 段在 RAM 里的结束地址
 * 厂商启动文件只拷 .data、清 .bss，**不管 .ramfunc** —— 所以应用也必须
 * 自己写 ramfunc_init() 搬一次。原因同 bootloader：F103 单 Bank，
 * 擦 Flash 时 CPU 不能从 Flash 取指令。
 * "before the control task can write the OTA configuration page" 这句是
 * 关键：Control 任务是唯一会写 Flash 的应用任务，所以必须在它启动前搬好。 */
extern uint8_t _siramfunc;
extern uint8_t _sramfunc;
extern uint8_t _eramfunc;

/* Event group bits */
#define EVENT_OTA_AVAILABLE  (1 << 0)
#define EVENT_CONNECTED      (1 << 1)
#define EVENT_ERROR          (1 << 2)
/* 中文：事件组的三个位（用位掩码表示）。
 * 本项目只用了 EVENT_OTA_AVAILABLE 这一个：
 *   Comm 任务收到 CMD_OTA_AVAILABLE → xEventGroupSetBits(事件位)
 *   Control 任务 → xEventGroupWaitBits(等这个位, 等到就清零)
 * 用 1 位表示一个"事件已发生"的布尔量，一个 32 位变量能装 32 个事件。
 * EVENT_CONNECTED / EVENT_ERROR 是预留位，目前没有代码去设置或等待它们。 */

/* How long the application must run before it considers itself healthy and
 * confirms the boot. Until it does, the bootloader treats the image as
 * unverified: a firmware that crashes on startup never gets here, so its boot
 * attempt count keeps climbing across resets and recovery is triggered. */
#define BOOT_CONFIRM_DELAY_MS  3000U
/* 中文：★这个 3000 和 bootloader 的 BOOT_ATTEMPT_LIMIT(3) 是一对契约★
 * 应用起来后等 3 秒才"报平安"。如果固件在启动阶段就崩（传感器初始化、
 * 显示初始化、驱动有 bug），它永远走不到那一行 → 上电次数一直累加
 * → 累计到 3 次，bootloader 就判定"这个镜像就算字节全对也是坏的"。
 * 3 秒这个值是在"等太久拖慢正常启动"和"太快会误判慢启动的固件"之间取的折中。 */

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
#define DIAG_SNAPSHOT_UPDATE_MS      5000U
/* 中文：各任务的"节拍"常量（单位都是毫秒）。集中放在文件顶部是为了
 * 调参方便 —— 想改刷新率只改这里，不用去函数体里翻。
 * 读法提示：
 *   WS2812_*    灯带的更新周期和各种阈值（详见块 6）
 *   *_SNAPSHOT_*_MS  快照的采样周期
 * 注意 SENSOR 快照 200ms 一次、DIAG 快照 5 秒一次 —— 越"重"的数据采得越慢。 */

/*---------------------------------------------------------------------------
 * System init
 *---------------------------------------------------------------------------*/
/* 中文：块 1 的第三部分 —— 硬件初始化。
 * 四个函数的调用顺序在 main() 里（见块 8）：
 *   ramfunc_init() → system_init() → iwdg_init() → app_tasks_init()
 */

static void SystemClock_Config(void) {
    /* 中文：把主频从"复位后的 HSI 8MHz"拉到"HSE 8MHz × PLL ×8 = 64MHz"。
     * 为什么必须要 64MHz：WS2812B 的位翻转时序是**按 64MHz 手工标定**的
     * （靠空循环的圈数来凑 0.3µs / 0.9µs 的高电平时间）。主频一变，
     * 灯带的时序就全错了。所以 bootloader 和应用用完全相同的配置。 */
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    /* 中文：开电源控制器的时钟。这不是配置时钟本身，而是"允许访问电源相关
     * 寄存器"——很多时钟/低功耗配置的前置条件。 */

    /* 8MHz HSE crystal → PLL ×8 → SYSCLK 64MHz. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL8;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        /* Clock configuration failure → halt with LED off. */
        while (1);
        /* 中文：★晶振起不来 = 板级硬件问题，这里死等★
         * 注意此时 LED 还没初始化，所以现场看到的是"灯不亮、板子没反应"。
         * 这个死循环没有喂狗，所以看门狗会在约 4 秒后复位它，然后
         * 再次走到这里 —— 表现为"每 4 秒重启一次的砖"。
         * 排查方向永远是先量 8MHz 晶振有没有起振（示波器/频率计）。 */
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* 中文：HCLK = 64MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV2;   /* PCLK1 = 32MHz (max 36MHz) */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;   /* PCLK2 = 64MHz */
    /* 中文：为什么 APB1 要除以 2：STM32F1 的 APB1 总线**上限是 36MHz**。
     * 直接把 64MHz 挂上去会超频，表现为外设行为随机异常。
     * 所以 APB1 上的外设（定时器、USART2/3 等）实际跑 32MHz。
     * APB2 上限 72MHz，所以 64MHz 可以直通。 */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        while (1);
    }
    /* 中文：FLASH_LATENCY_2 = 64MHz 下 Flash 需要 2 个等待周期。
     * Flash 的读取速度跟不上 64MHz，插等待周期让 CPU 等一下。
     * **改主频必须同步改这个值**，忘了改会随机跑飞、且难以复现。 */
}

static void ramfunc_init(void) {
    /* 中文：3 行手写的段拷贝。和 bootloader 里那个一模一样。
     * 为什么必须手写：标准启动文件只管 .data（从 Flash 拷到 RAM）和
     * .bss（清零），**不知道 .ramfunc 这个自定义段存在**。
     * 漏了这一步的症状：平时一切正常，一开始 OTA 就挂死
     * —— 因为擦 Flash 的代码本身还在 Flash 里，取指被硬件挡停。 */
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
    /* 中文：★看门狗。四个寄存器，全是固定值，背下来即可★
     * 关键认知：**看门狗是"独立"的** —— 它用一个自己的低速振荡器（LSI），
     * 不依赖主时钟、不依赖中断。所以即使程序卡在关中断的死循环里，
     * 看门狗依然会计时并复位 —— 这正是它存在的意义。
     *
     * 下面第 169-171 行的英文注释讲了一件很重要的事：
     *   IWDG 跨 NVIC_SystemReset() 继续计数（只有彻底断电才清零），
     *   包括 bootloader → application 的那次跳转。
     * 所以两个固件必须用**完全相同的参数**，各自独立喂狗。
     * 参数不一致的后果：跳转瞬间一边在喂、另一边已经超时 → 随机复位。 */
    IWDG->KR  = 0xCCCCU;   /* start the watchdog */
    IWDG->KR  = 0x5555U;   /* unlock PR/RLR for writing */
    IWDG->PR  = 4U;        /* /64 prescaler */
    IWDG->RLR = 2499U;     /* ~4.0s @ nominal 40kHz LSI */
    /* 中文：0xCCCC 启动、0x5555 解锁、0xAAAA 喂狗 —— 这三个魔数是 ST 定的。
     * 超时时间 = RLR × 预分频 / LSI 频率：
     *   标称 40kHz：2499 × 64 / 40000 = 4.0s
     *   最坏 60kHz：2499 × 64 / 60000 = 2.67s   ← 设计时按这个算余量
     * LSI 的精度很差（数据手册给的是 30~60kHz），所以**永远按最坏值算**。 */

    uint32_t guard = 100000U;
    while ((IWDG->SR != 0U) && (--guard != 0U)) {
        /* wait for the prescaler/reload update to latch */
    }
    /* 中文：写完 PR/RLR 要等它们真正生效（IWDG->SR 变 0）。
     * 加 guard 计数是防御性的：万一 LSI 没起振，也不会死等在这。 */

    IWDG->KR = 0xAAAAU;    /* load the counter from RLR */
    /* 中文：喂一次狗，把计数器装载为 RLR。从这一刻开始倒计时。 */
}

static void iwdg_refresh(void) {
    IWDG->KR = 0xAAAAU;
}
/* 中文：喂狗。★谁会调用它？★
 * 只有 vMonitorTask（块 7），每秒一次。
 * 这意味着：**只要 Monitor 任务能每秒跑上一次，设备就不会复位**。
 * 反过来说，如果 Monitor 被饿死超过约 2.67 秒，设备就会重启 ——
 * 看门狗在这里起到了"调度器还活着吗"的检测作用。 */

static void system_init(void) {
    /* VTOR relocation — MUST be first */
    SCB->VTOR = APP_BASE;
    /* 中文：★★★ 必须是第一条语句 ★★★
     * 复位后 VTOR 指向 0x08000000（bootloader 的向量表）。应用的中断处理
     * 函数在 0x08002000 开始的向量表里。不重定位的话，应用一旦触发任何
     * 中断，CPU 就会去 bootloader 的向量表取地址 —— 拿到的是 bootloader
     * 的处理函数，而那里没有应用需要的函数 → 跑飞。
     *
     * 为什么 bootloader 跳转前已经设过了，这里还要再设一次：
     *   ① 不依赖别人的正确性（万一 bootloader 改坏了）
     *   ② 开发时用 ST-Link 直接烧 application（不经过 bootloader）也能跑
     * 这叫"防御性重复"—— 代价是一次寄存器写，收益是不依赖外部条件。 */

    HAL_Init();
    /* 中文：HAL 层初始化：装 SysTick 时基（1ms）、设 NVIC 优先级分组。
     * 必须在使用任何 HAL_Delay / HAL_GetTick 之前调用。 */

    SystemClock_Config();
    /* 中文：拉到 64MHz。注意是在 HAL_Init 之后 ——
     * HAL_Init 里的 SysTick 是按复位后的 8MHz 算的，改主频后 HAL 会重算。 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* 中文：开 GPIOC 时钟。★外设时钟默认是关的★，不开就去操作寄存器
     * 会静默无效（不报错、也没反应），这是嵌入式最常见的新手坑。 */

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); /* LED off */
    /* 中文：PC13 是低电平点亮，所以写 SET（高）＝ 灭灯。
     * 上电先灭灯是刻意的：这样"LED 亮着"就一定是异常信号
     * （栈溢出/内存不足的钩子、或看门狗前的最后状态）。 */
}

/*---------------------------------------------------------------------------
 * vCommTask — UART protocol handler
 *---------------------------------------------------------------------------*/
/* 中文：★ 块 2 之一：通信任务（4 个任务里最短的一个，只有 26 行）★
 *
 * 它的职责只有两件事：
 *   ① 从流缓冲里取字节，喂给协议解析器
 *   ② 解析出一整帧后，交给 cmd_handler_dispatch() 去分发
 *
 * ★ 关键设计：中断里只搬数据，解析放在任务里 ★
 *   UART 中断（USART1_IRQHandler）只做一件事：算出 DMA 写到哪了，
 *   把新到的那一段字节塞进 StreamBuffer，然后退出。
 *   **它不解析协议、不判断帧。**
 * 为什么：中断必须尽可能短 —— 中断期间同优先级和更低优先级的任务都跑不了。
 *   而这个任务可以慢慢解析，晚几毫秒没人受影响。
 * 这个模式有正式名字：中断延迟处理（deferred interrupt processing），
 * 也叫"上半部 / 下半部"（top half / bottom half）。
 *
 * ★ 优先级：这是全系统优先级最高的任务（3）★
 *   因为串口是唯一的输入通道，晚处理就会丢数据。
 *   正因为它优先级最高，**它绝不能干耗时的活** —— 这是理解块 2 后半段
 *   （cmd_handler_dispatch）为什么只转发不执行的关键。 */

static void vCommTask(void *pvParameters) {
    (void)pvParameters;
    /* 中文：解析器放在任务的**栈上**（不是全局）。
     * ★记住这个数字：sizeof(ProtoParser_t) = 1048 字节★
     * 它就是"为什么 Comm 任务的栈要开到 768 字（3072 字节）"的主要原因。 */

    ProtoParser_t parser;
    proto_parser_init(&parser);
    /* 中文：初始化为"等待同步字"状态。发一个字节喂一次，状态机自己推进。 */

    uint8_t rx_byte;
    size_t rx_count;

    /* The DMA may have filled part of its buffer before this task first ran,
     * and the IDLE interrupt only fires on a new burst edge. Drain once so
     * nothing that arrived during startup is stranded. */
    uart_comm_drain_rx();
    /* 中文：★这一句解决一个时序竞态★
     * 任务真正开始跑之前，DMA 可能已经收了一部分字节进缓冲区。
     * 但 IDLE 中断只在"线路由忙变空闲"的那个边沿触发 ——
     * 如果那批数据在中断使能前就到齐了，中断就永远不会来，
     * 那批字节就会一直躺在 DMA 缓冲里没人管。
     * 所以进循环前主动 drain 一次，把可能已经到达的数据捞出来。 */

    for (;;) {
        rx_count = xStreamBufferReceive(g_rx_stream, &rx_byte, 1,
                                        pdMS_TO_TICKS(100));
        /* 中文：从流缓冲里取 **1 个字节**，最多等 100ms。
         * 为什么一次只取 1 字节：因为协议解析器就是"逐字节推进的状态机"，
         * 一次给一个字节、问一次"凑齐了吗"，是它的使用方式。
         * 为什么要有 100ms 超时：没有数据时任务会阻塞在这里（把 CPU 让给
         * 别的任务），最多 100ms 醒一次再等。循环里没有别的事，
         * 其实用 portMAX_DELAY 也可以 —— 属于可优化但不影响正确性的细节。 */

        if (rx_count > 0) {
            const ProtoFrame_t *f = proto_parser_feed(&parser, rx_byte);
            if (f != NULL) {
                /* Valid frame received — dispatch */
                cmd_handler_dispatch(f);
                /* 中文：★注意这个指针的生命周期★
                 * f 指向 parser 内部的内存，**下一次 feed 就会被覆盖**。
                 * 所以必须立刻用掉（这里传进 dispatch 就用了），不能存起来
                 * 等以后再读 —— 这是 protocol.h 里写明的接口约定。
                 *
                 * 另外注意：绝大多数次 feed 返回的都是 NULL（还没凑齐一帧），
                 * 只有 CRC 校验通过的那一次才返回非 NULL。
                 * 所以这里不是"判断有没有出错"，而是"判断凑齐了没有"。 */
            }
        }
    }
}

/*---------------------------------------------------------------------------
 * vControlTask — OTA trigger and system control
 *---------------------------------------------------------------------------*/
/* 中文：★ 块 3：控制任务（优先级 2，栈 512 字 = 2048 字节）★
 *
 * 它只干两件事，而且干完第一件才进入第二件：
 *
 *   阶段一（只跑一次，启动时）
 *     ① 上报固件版本
 *     ② 等 3 秒 → 向 bootloader"报平安"
 *
 *   阶段二（常驻循环）
 *     ③ 阻塞等"有固件要升级"的通知
 *     ④ 把升级请求写进 Flash 配置 → 通知 ESP32 → 复位自己
 *
 * ★ 它是整个应用里【唯一会写 Flash】的地方 ★
 *   别的任务都不碰 Flash。所有固件区的擦写在 bootloader 里做，
 *   应用只写"配置区"这一小块，而且只在这一处。
 *   这样划分的好处：应用正常运行期间完全不用操心 Flash 的擦写时序
 *   和 .ramfunc 约束，出问题的面小很多。
 *
 * ★ 为什么"报平安"放在这个任务而不是 App 任务 ★
 *   它的优先级（2）比 App 任务（1）高。所以哪怕 App 任务里的
 *   UI 初始化卡住了，这里的 3 秒延时到点后照样会被调度 ——
 *   确认能按时发出去。
 *   （反过来说，这也意味着"报平安"只证明了调度器和 Control 任务正常，
 *    并不证明整个应用健康 —— 这是这套机制的一个已知边界。） */

static void vControlTask(void *pvParameters) {
    (void)pvParameters;

    /* Report current firmware version on startup */
    /* 中文：④-1 上报版本。这是"我起来了"的第一声招呼。
     * 注意 bootloader 也有一条 CMD_STATUS_RSP —— 但那一条回的是"恢复状态"，
     * 应用这一条回的是"固件版本号"。历史上两者撞过车，见 protocol.h。 */
    BootConfig_t cfg;
    if (ota_config_read(&cfg)) {
        uint32_t version = cfg.fw_version;
        cmd_handler_send_frame(CMD_STATUS_RSP, (uint8_t *)&version, 4);
    }

    /* Boot confirmation.
     *
     * The bootloader has already counted this boot. Waiting a few seconds
     * before confirming means a firmware that dies during startup — sensor
     * init, UI init, a bad driver — never reaches this point, so its attempt
     * count keeps rising across the resets the watchdog causes, and the
     * bootloader eventually stops trusting the image and asks the host to
     * re-send firmware. A healthy boot costs exactly one config write. */
    /* 中文：★④-2 报平安 —— 整个掉电回滚机制的"发令枪"★
     *
     * 时序是这样的（和 bootloader 侧配合）：
     *   上电 → bootloader 把 BKP 里的"未确认启动次数" +1 → 跳进应用
     *        → 应用跑到这里，等 3 秒
     *        → 如果一切正常，调 confirm_boot() 把计数清零、boot_confirmed 置 1
     *
     * 如果固件在启动阶段就崩（传感器初始化、UI 初始化、某个驱动有 bug），
     * 它**永远走不到这一行** → 看门狗复位 → 计数继续累加
     * → 累加到 3 次，bootloader 就判定"这个镜像就算是字节全对也是坏的"。
     *
     * 为什么要等 3 秒而不是立刻报：太早报的话，一个"初始化要 5 秒才崩"的
     * 固件也会被误判为好。3 秒是在"等太久拖慢启动"和"太快会漏掉慢崩的固件"
     * 之间的折中。
     *
     * 成本：健康启动只写一次 Flash（confirm 内部有优化，已确认过就不写）。 */
    vTaskDelay(pdMS_TO_TICKS(BOOT_CONFIRM_DELAY_MS));
    if (!ota_config_confirm_boot()) {
        cmd_handler_send_frame(CMD_NAK, NULL, 0);
        /* 中文：写配置失败 → 回 NAK 告诉 ESP32 出问题了。
         * 注意此时**不回** CMD_OTA_COMMITTED —— 因为"我确认不了自己"，
         * 那就不能被发现成"已知良好版本"。 */
    } else if (ota_config_read(&cfg)) {
        /* Tell the bridge which version actually booted and confirmed. The
         * bridge keeps a "golden" image of the last version known to run; this
         * message is what lets it adopt the newly installed version instead of
         * still treating the previous one as the rollback target. It is only
         * sent after confirmation, so a version that never boots is never
         * recorded as good. */
        /* 中文：★★★ CMD_OTA_COMMITTED 的深层含义：它是"golden 提升"的信号 ★★★
         *
         * ESP32 在 SPIFFS 里保留一份"上一版已确认能跑"的固件（叫 golden）。
         * 只有当它收到这条 CMD_OTA_COMMITTED（带版本号 N）时，
         * 才会把版本 N 提升为**新的回滚目标**。
         *
         * 为什么这个顺序很关键：
         *   "能启动并确认" 这个事实，是**由新固件自己证明的**。
         *   一个启动就崩的版本永远发不出这条消息 → 永远不会被记录成"已知良好"
         *   → 所以回滚目标永远是**真正验证过**的版本，而不是"刚写进去的"版本。
         *
         * 一句话：**回滚目标的可信度由目标自己证明。** */
        cmd_handler_send_frame(CMD_OTA_COMMITTED,
                               (const uint8_t *)&cfg.fw_version, 4);
    }

    for (;;) {
        /* 中文：⑤ 阶段二：常驻循环。等"有固件要升级"这个通知。
         * 平时它就阻塞在这里（portMAX_DELAY = 无限等），不占 CPU。 */

        EventBits_t bits = xEventGroupWaitBits(
            g_event_group,
            EVENT_OTA_AVAILABLE,     /* 等哪一位 */
            pdTRUE,   /* Clear on exit */
            pdFALSE,  /* Wait for any (not all) */
            portMAX_DELAY
        );
        /* 中文：四个参数的含义：
         *   EVENT_OTA_AVAILABLE  等这一位被置 1
         *   pdTRUE               等到后**自动清零**（不然下次会立刻又通过）
         *   pdFALSE              只需任意一位满足（本项只有一位，区别不大）
         *   portMAX_DELAY        无限等，直到事件发生
         * 为什么用事件组而不是直接等队列：事件组传的是"发生了什么"（通知），
         * 队列传的是"具体数据"。两者语义不同。
         * 诚实说明：现在只有这一条链路，所以单用队列也能实现；保留事件组的
         * 好处是将来加第二个事件类型时不用改等待逻辑，而且预留了
         * EVENT_CONNECTED / EVENT_ERROR 两个位。 */

        if (bits & EVENT_OTA_AVAILABLE) {
            /* A new firmware version is available on the ESP32. */
            OtaRequest_t request;
            if (xQueueReceive(g_cmd_queue, &request, 0) == pdPASS) {
                /* 中文：⑥ 取出具体内容（版本号 + 镜像大小）。
                 * 这里超时给 0 是安全的 —— 因为事件位刚被设置，
                 * 队列里必然已经有对应数据（Comm 任务是先发队列再设位）。 */

                /* Write OTA request to config then reboot into bootloader.
                 *
                 * The expected image size is recorded here, before the transfer
                 * starts. That is what lets the bootloader recompute the image
                 * CRC after an interrupted transfer and refuse to boot a body
                 * that is half old and half new. */
                /* 中文：★★★ ⑦ 这是应用侧最关键的一步 ★★★
                 * ota_config_request_update() 会把 boot_mode 置成 BOOT_MODE_OTA
                 * 并把 request.size 一起写进 Flash 配置页。
                 *
                 * 为什么必须在**传输开始之前**就把 size 落盘：
                 *   如果传输中途掉电，app 区会变成"一半新一半旧"。
                 *   bootloader 下次上电时靠这个 size 重算 app 区 CRC，
                 *   才能识别出"这不是一个完整的镜像"，从而拒绝跳进去。
                 *   没有它，那套"掉电不变砖"的机制就不成立。
                 *
                 * 注意这个函数内部会擦写 Flash —— 用的是 .ramfunc 里的
                 * 擦写函数，所以 ramfunc_init() 必须在任务启动之前跑完
                 * （见块 8 的 main()）。 */
                if (ota_config_request_update(request.version, request.size)) {
                    /* Confirm only after the config page is valid. The ESP32
                     * waits for this before it begins the bootloader transfer. */
                    /* 中文：★⑧ CMD_OTA_READY 是一个【同步点】，不是普通回包★
                     *
                     * 完整的一次 OTA 是这样对话的：
                     *
                     *   ESP32                                  STM32 应用
                     *     │──── CMD_OTA_AVAILABLE(版本,大小) ──→│
                     *     │                                        │ 写 Flash 配置
                     *     │←──── CMD_OTA_READY(版本) ────────────│
                     *     │  ★收到这个才开始等 bootloader★         │ 等 50ms
                     *     │                                        │ NVIC_SystemReset()
                     *     │                                        ▼
                     *     │                                  (bootloader 接管)
                     *     │──── CMD_OTA_BEGIN ──────────────────→ bootloader
                     *
                     * 为什么 ESP32 必须等到 READY：如果它一收到 AVAILABLE
                     * 就开始发 chunk，而这边配置还没写完就复位了 ——
                     * 发出去的数据没人收，全丢。
                     * 这条回包就是"配置已落盘，我随时可以复位，你可以开始了"。 */
                    cmd_handler_send_frame(CMD_OTA_READY,
                                           (const uint8_t *)&request.version, 4);
                    /* Small delay so the confirmation fully leaves USART1. */
                    /* 中文：★⑨ 这 50ms 是必须的，不是随手加的 ★
                     * uart_comm_send() 是阻塞发送，但它只保证"字节写进了
                     * 移位寄存器"，**不保证对端已经收全**。
                     * 如果紧接着就 NVIC_SystemReset()：
                     *   复位会让 USART1 回到默认状态，正在发的字节被截断
                     *   → ESP32 收不到 CMD_OTA_READY → 它会一直等到超时
                     *   → 升级失败，而且日志上看不出原因。
                     * 50ms @115200 能发约 576 字节，发这个 8 字节的帧
                     * 绰绰有余。 */
                    vTaskDelay(pdMS_TO_TICKS(50));
                    NVIC_SystemReset();
                    /* 中文：⑩ 软复位。复位后 bootloader 会读到
                     * boot_mode == BOOT_MODE_OTA，于是打开 OTA 握手窗口
                     * 而不是直接跳回应用 —— 整个链条在这里交接。 */
                }
            }
        }
    }
}
/* 中文：★ 顺手算一笔栈账（回答"你怎么定任务栈大小"）★
 * Control 任务的栈是 512 字 = 2048 字节。它最深路径上的大额局部变量：
 *   本函数：        BootConfig_t cfg      = 56 字节
 *                  OtaRequest_t request  =  8 字节
 *   被调函数：      cmd_handler_send_frame 里的 buf[1036] = 1036 字节
 *   ────────────────────────────────────────────────────────
 *   合计约 1100 字节 + 上下文保存 ≈ 1200 字节 / 2048 字节（余量约 40%）
 *
 * 注意那个 1036 字节的发送缓冲区 —— **它出现在每一个调用
 * cmd_handler_send_frame 的任务的栈预算里**（Comm、Control 都算了它）。
 * 这类"被多个任务共用的函数"的栈开销，算栈大小时不能漏。 */



/*---------------------------------------------------------------------------
 * vAppTask — user application logic (placeholder)
 *---------------------------------------------------------------------------*/
/* 中文：★ 块 4：应用任务 —— 全项目最大的一个任务（约 250 行）★
 *
 * 它一个人干了五件事：传感器采集、双屏 UI、旋钮/按键输入、PIR、
 * 执行器控制（继电器/蜂鸣器/灯带）。
 *
 * 【为什么全塞进一个任务，而不是每个外设一个任务？】
 *   ① RAM 不够。每个任务至少要 512 字（2KB）栈，20KB 的芯片开不起 8 个任务。
 *   ② 这些工作都是"低频、非阻塞"的，放一起没有实时性冲突。
 *      光照 200ms 一次、环境 2s 一次、UI 100ms 一次 —— 谁都不用抢。
 *   ③ **它们共享状态**。比如 g_light_brightness 同时被 UI 显示和灯带驱动用。
 *      放在一个任务里天然串行执行，**一把锁都不需要**。
 *
 * 【它是"执行器唯一的 owner"】
 *   全项目只有这个任务会调用 relay_set() / buzzer / ws2812b_show_white()。
 *   网络命令、旋钮、按键想改执行器，都得把请求汇到这里（见块 2/块 6）。
 *
 * 【它内部的调度方式很特别，是理解它的关键】
 *   它**不是**"一个循环做一件事"，而是"一个循环按时间戳检查很多件事"。
 *   看下面的 for(;;) 循环开头那段注释。
 *
 * 块 5 讲传感器和快照，块 6 讲控制命令和灯光，这里只讲循环骨架。 */

#define PIR_WARMUP_MS       30000U
#define ENV_SAMPLE_MS       2000U
#define ENV_CONVERSION_MS   90U
#define UI_REFRESH_MS       100U
/* 中文：这个任务自己用到的四个节拍常量（单位 ms）：
 *   PIR_WARMUP_MS      人体红外模块上电后要预热 30 秒才可信
 *   ENV_SAMPLE_MS      温湿度气压每 2 秒采一次
 *   ENV_CONVERSION_MS  ★转换等待 90ms★ —— 温湿度传感器收到"开始测量"后
 *                      需要约 90ms 才能出结果，这段时间不能阻塞，见块 5
 *   UI_REFRESH_MS      屏幕每 100ms 刷一次（人眼看起来就是实时的） */

static void ui_status_build(UiStatus_t *status,
                            const EnvironmentReading_t *environment,
                            bool environment_valid,
                            uint16_t light_lux, bool light_valid,
                            bool motion_detected, bool pir_warmed_up,
                            uint8_t control_selected, bool control_editing,
                            uint32_t firmware_version);

/*===========================================================================
 * 中文：★ 块 6：灯光映射 + 控制命令执行（注意：文件里排在块 5 前面）★
 *===========================================================================
 * 这段包含三类东西：
 *   ① 亮度换算：percent（0..100，给人看的） ↔ brightness（0..255，给灯带的）
 *   ② handle_control_command：把 ESP32 发来的文本命令变成继电器/蜂鸣器动作
 *   ③ 灯带亮度算法：光照反比映射 + 平滑过渡
 *
 * ★ 一个必须记住的换算关系 ★
 *   灯带的"暗"对应 brightness 160，不是 255 —— 因为 WS2812B 全白满亮度
 *   每颗要 60mA，15 颗就是 0.9A，电源带不动。所以把上限压在 160。
 *   于是 percent 100% ↔ brightness 160，而不是 255。
 *   下面 light_*_to_* 两个函数干的就是这件事。 */

static uint8_t light_brightness_to_percent(uint8_t brightness) {
    /* 中文：brightness → percent（给 UI 显示用）。
     * 注意 + (160/2) 是在做"四舍五入"，C 的整数除法是截断的，
     * 不加这半格 159 会算成 99%。这类"整数除法要手动补半"是嵌入式的常规操作。 */
    const uint32_t percent =
        ((uint32_t)brightness * 100U + (WS2812_DARK_BRIGHTNESS / 2U)) /
        WS2812_DARK_BRIGHTNESS;
    return (uint8_t)(percent > 100U ? 100U : percent);
}

static uint8_t light_percent_to_brightness(uint8_t percent) {
    /* 中文：percent → brightness（给灯带用）。先夹到 1..100，
     * 因为"0%"等于关灯，而关灯是靠继电器断 VCC 做的，不是靠亮度 0。 */
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
    /* 中文：切换自动/手动模式。★一个体贴的细节★
     * 从"自动"切到"手动"时，先把当前实际亮度换算成百分比存起来 ——
     * 这样用户切到手动时，灯带的亮度是**连贯的**，不会突然跳到默认的 50%。 */
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
    /* 中文：手写的十进制解析（不用 atoi/sscanf）。
     * 为什么手写：atoi 遇到非法输入不报错（"abc" 返回 0），
     * 而这里必须严格拒绝非法输入 —— 因为解析的是"控制灯带亮度"的命令，
     * 静默接受错误输入意味着灯会乱亮。
     * 逐条校验：跳过前导空格 → 只接受数字 → 累加时超 100 立即失败 →
     * 末尾必须是结束符或换行 → 不能是 0。
     * ★这是嵌入式解析用户输入的标准姿势：宁可拒绝，不可猜。★ */
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
    /* 中文：★ 网络命令的执行点：文本 → 硬件动作 ★
     * 这是块 2 里"只转发不执行"的落点。它跑在 vAppTask 里，
     * 所以是全项目**唯一**会调 relay_set / buzzer_set 的地方之一。
     *
     * 支持的命令（桥那边定义，这里解析）：
     *   RELAY1 ON|OFF · RELAY2 ON|OFF · RELAY（只查询）
     *   LIGHT AUTO|MANUAL · LIGHT BRIGHTNESS 1..100
     *   LANG ZH|EN · BUZZER ON|OFF
     *
     * ★ 为什么用 strncmp/strstr 这种土办法而不是正经的字符串库 ★
     *   因为在 20KB RAM 上不值得引入一个解析框架。命令集固定就这几种，
     *   前缀匹配足够，而且代码量最小。代价是"命令名不能互相是前缀"
     *   （比如 "LIGHT" 和 "LIGHT BRIGHTNESS" 就必须先判更长的那个）。 */

    bool handled = false;

    if (strncmp(cmd, "RELAY", 5U) == 0) {
        uint8_t channel = 0xFFU;  /* bare "RELAY" -> query only */
        /* 中文：0xFF 当"没指定通道"的哨兵值 —— 于是 "RELAY" 单独一条
         * 就成了"只查询不改状态"。一个小技巧省掉一个分支。 */
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
            /* 中文：注意先判 "ON" 再判 "OFF" —— 因为 "OFF" 里不含 "ON"
             * 所以这里没有子串陷阱；但反过来写就会有问题。
             * 这类"顺序敏感的字符串匹配"是土办法的固有代价。 */
        }
        handled = true;
    } else if (strcmp(cmd, "LIGHT AUTO") == 0 ||
               strcmp(cmd, "AUTO ON") == 0) {
        light_set_auto(true);
        handled = true;
        /* 中文：一个动作接受多个别名（LIGHT AUTO / AUTO ON），
         * 是为了兼容早期蓝牙命令和后来的 Web 命令两套叫法。 */
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
            /* 中文：★自动模式下只记数值、不立刻生效★
             * 因为自动模式的亮度由光照决定，用户设的百分比只是
             * "切到手动时该用多少"的备忘。 */
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
        /* 中文：★执行完立刻回一条 6 字节状态★
         * 内容依次是：继电器1、继电器2、自动模式、蜂鸣器、亮度百分比、语言。
         * 为什么要有这个回执：Web/TFT 的按钮需要立刻反映真实状态，
         * 而不是等下一轮传感器轮询（那是 1 秒一次）。
         * 这叫"命令回执"，和"周期上报"是两条不同的路径。 */
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
    /* 中文：★ 自动调光的核心算法：光照【反比】映射 ★
     * 环境越暗 → 灯越亮；环境越亮 → 灯越暗。
     *   光照 ≤ 5 lux    → 亮度 160（最亮，晚上）
     *   光照 ≥ 1000 lux → 亮度 1  （最暗，白天）
     *   中间线性插值
     * 为什么是反比而不是开关：晚上开灯不是为了照明，是为了"补光"，
     * 环境光越弱补得越多，这样视觉上亮度感是恒定的。
     *
     * ★ 传感器失效怎么办：返回一个固定值（24），而不是乱调 ★
     * 这是"优雅降级"—— BH1750 没插好时，灯带就是个固定的弱光，
     * 而不是随垃圾数据疯狂闪烁。 */
    if (!light_valid) {
        return WS2812_FALLBACK_BRIGHTNESS;
    }
    if (light_lux <= WS2812_DARK_LUX) {
        return WS2812_DARK_BRIGHTNESS;
    }
    if (light_lux >= WS2812_BRIGHT_LUX) {
        return WS2812_BRIGHT_BRIGHTNESS;
    }

    /* 中文：线性插值的定点写法（没有浮点）。
     * reduction = (当前光照 - 最低光照) × 亮度跨度 / 光照跨度
     * 最终亮度 = 最亮 - reduction
     * 先乘后除是为了保住精度（先除会把小数丢掉）。 */
    const uint32_t lux_range = WS2812_BRIGHT_LUX - WS2812_DARK_LUX;
    const uint32_t brightness_range =
        WS2812_DARK_BRIGHTNESS - WS2812_BRIGHT_BRIGHTNESS;
    const uint32_t reduction =
        ((uint32_t)(light_lux - WS2812_DARK_LUX) * brightness_range) /
        lux_range;
    return (uint8_t)(WS2812_DARK_BRIGHTNESS - reduction);
}

static uint8_t ws2812_smooth_brightness(uint8_t current, uint8_t target) {
    /* 中文：★ 平滑过渡 + 死区，防止灯带"抖" ★
     * 两个保护：
     *   死区(DEADBAND=2)：目标值和当前值差 2 以内就不动 ——
     *     光照传感器本身有噪声，不设死区灯带会一直微闪。
     *   步长(STEP=16)：每次最多变 16 级 —— 光照突变时灯带是"渐变"过去的，
     *     而不是"啪"一下跳变。
     * 这两个参数合起来的效果：看起来像"人眼适应的过程"。
     *
     * ★ 为什么这个平滑很重要（不只是好看）★
     *   每发一帧灯带数据要关中断约 0.5ms。如果每次光照微小波动都发帧，
     *   中断会被频繁关闭，串口和调度都会受影响。
     *   平滑 + 死区把"发帧次数"压到很低 —— 这是**性能优化**，不只是美观。 */
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

/*===========================================================================
 * 中文：★ 块 5：发布 18 字节状态快照 ★
 *===========================================================================
 * vAppTask 每 200ms 调一次这个函数，把"设备现在什么状态"打包成
 * SensorSnapshot_t（18 字节），存进 g_sensor_snapshot。
 * ESP32 每秒来问一次（CMD_GET_SENSOR_SNAPSHOT），把这份数据转成 JSON 喂网页。
 *
 * ★ 三个设计点 ★
 *   ① 用【位标志】而不是 10 个 bool —— 一个 uint16 装 10 个布尔量，省 8 字节
 *   ② 灯带亮度要乘以"继电器是否通电" —— 灯带断电时亮度应该报 0，
 *      不然网页上会显示"亮度 60%"但灯实际是灭的
 *   ③ 最后用临界区写入 —— 因为读它的是另一个任务（Comm） */

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
    /* 中文：★ 这个变量清单看着吓人，其实只有 5 类，按类看就清楚了 ★
     * 下面按类加注释，不要逐行背。
     * 重点看最后那几个 TickType_t —— 它们是"时间戳轮询"的全部秘密。 */

    (void)pvParameters;
    uint8_t data[64];
    /* 中文：① UI 状态：当前在哪一页、菜单选中第几项、控制页选中第几项、
     * 是否正在编辑（旋钮转动是在"选项间移动"还是"改数值"）。 */
    UiPage_t page = UI_PAGE_MENU;
    uint8_t menu_selected = 0U;
    uint8_t control_selected = 0U;
    bool control_editing = false;

    /* 中文：② 各传感器的当前读数 + 是否有效。
     * 注意 light_valid / motion_detected 初始化时就先读一次，
     * 而不是等第一次循环 —— 这样开机瞬间显示的就是真值，不是 0。 */
    EnvironmentReading_t environment = {0};
    bool environment_valid = false;
    uint16_t light_lux = 0U;
    bool light_valid = g_bh1750_ready && bh1750_read_lux(&light_lux);
    bool motion_detected = g_pir_ready && pir_sensor_motion_detected();
    bool pir_warmed_up = false;

    /* 中文：③ 按键的"上一次状态"。
     * ★这是软件消抖 + 边沿检测的标准写法★
     * 循环每 5ms 跑一次，如果只判断"按键现在是否按下"，
     * 一次按下的 100ms 会被识别成 20 次点击。
     * 所以记住上一次的状态，只有"上次没按、这次按了"才算一次点击。 */
    bool previous_button_pressed =
        g_encoder_ready && rotary_encoder_button_pressed();
    bool previous_back_pressed =
        g_back_button_ready && back_button_pressed();

    /* 中文：④ ★★★ 五个"上次做某事的时间戳" —— 这是本任务的核心机制 ★★★
     * 每一个对应一件周期性的事：
     *   last_light_read       上次读光照       （每 200ms）
     *   last_ws2812_update    上次更新灯带     （每 200ms）
     *   last_ws2812_refresh   上次强制重发灯带 （每 1s，见块 6）
     *   last_sensor_snapshot  上次发布快照     （每 200ms）
     *   last_ui_refresh       上次刷屏         （每 100ms）
     * 循环里统一用这个模式判断"到点了没"：
     *     if ((now - last_X) >= pdMS_TO_TICKS(周期)) { last_X = now; ...干活... }
     * 认熟这个模式，下面 200 行代码会一下子变得很好读。 */
    TickType_t last_light_read = 0U;
    TickType_t last_ws2812_update = 0U;
    TickType_t last_ws2812_refresh = 0U;
    TickType_t last_sensor_snapshot = 0U;
    TickType_t last_ui_refresh = 0U;

    /* 中文：⑤ 灯带相关状态：当前亮度、上次真正发出去的亮度值、
     * 以及"是否发过帧"。最后这个标志是为了处理"继电器断电后灯带丢失状态"
     * 的情况（见块 6）。 */
    g_light_brightness = ws2812_target_brightness(light_lux, light_valid);
    uint8_t ws2812_last_sent = 0U;
    bool ws2812_frame_sent = false;

    /* 中文：⑥ 环境传感器的"发起-等待-读取"三态机所需的三个时间点。
     * 这个机制比较特别，块 5 专门讲。这里只要知道它不是"读一次就完"，
     * 而是"发起 → 等 90ms → 读"的异步过程。 */
    const TickType_t app_started_at = xTaskGetTickCount();
    TickType_t last_environment_start = app_started_at;
    TickType_t environment_started_at = app_started_at;
    bool environment_pending = g_environment_ready &&
                               environment_sensor_start_measurement();

    /* 中文：⑦ 固件版本号只在启动时读一次 Flash，之后一直用这个常量。
     * 为什么不每次循环读：读 Flash 是有成本的，而且版本号运行时不会变。 */
    BootConfig_t boot_config;
    const uint32_t firmware_version = ota_config_read(&boot_config)
                                          ? boot_config.fw_version : 0U;
    UiStatus_t ui_status;

    /* 中文：下面两块是"进循环之前的第一次干活"：
     * 先把 UI 状态组装好、刷一次屏，再发布一次快照 ——
     * 这样设备一上电屏幕上就有内容，不会空白等到 100ms 后。 */
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
        /* ===================================================================
         * 中文：★ 循环骨架 —— 一个循环里按固定顺序做 9 件事 ★
         * ===================================================================
         *
         * 记住这个大顺序，200 行的循环就变成 9 个小块：
         *
         *   ① 取控制命令      每轮最多等 5ms ★这就是这个任务的"节拍器"★
         *   ② 读输入          旋钮转了多少、确认键、返回键
         *   ③ 处理输入        翻菜单、改亮度、开关灯、切语言、返回
         *   ④ 读光照          每 200ms 一次
         *   ⑤ 更新灯带        每 200ms 一次
         *   ⑥ 采环境数据      发起 → 等 90ms → 读（三态机，每 2s 一轮）
         *   ⑦ 读 PIR          人体红外 + 30 秒预热判断
         *   ⑧ 刷屏            每 100ms 一次
         *   ⑨ 发布快照        每 200ms 一次
         *
         * ★ 关键：循环周期是多少？答案是"约 5ms" ★
         *   第①步那个 xQueueReceive 的超时是 5ms —— 没有命令时，
         *   任务就在这里阻塞 5ms然后醒一次。所以循环大约每 5ms 转一圈。
         *
         * ★ 那些"每 200ms / 每 2s"是怎么实现的？★
         *   **不是用 vTaskDelay 睡到那个时刻**，而是每一圈都问一句
         *   "距离上次做这件事，过了 200ms 了吗？"
         *       if ((now - last_light_read) >= pdMS_TO_TICKS(200)) { ... }
         *   这个模式在本文件里出现 5 次。
         *
         * ★ 为什么不干脆给每件事开一个任务、用 vTaskDelay 各睡各的？★
         *   ① RAM 不够（每个任务最少 2KB 栈）
         *   ② 它们要共享状态（比如 g_light_brightness），放一个任务里天然串行
         *   ③ 5ms 的轮询精度对光照/UI/传感器完全够用，没必要更精确
         *   这在思路上其实还是"超级循环（superloop）"—— 只不过跑在
         *   一个 RTOS 任务里，外面还有其他 3 个任务在并行。
         * =================================================================== */

        /* 中文：★① 取控制命令（来自 ESP32 的网络命令）★
         * 这是块 2 里 Comm 任务投递过来的"RELAY2 ON"这类文本。
         * 5ms 超时 = 这个任务的节拍器（见上面的说明）。 */
        /* Receive control commands from ESP32 (BT serial bridge) */
        if (xQueueReceive(g_data_queue, data, pdMS_TO_TICKS(5)) == pdPASS) {
            handle_control_command((const char *)data);
            /* 中文：真正的执行在这里（块 6 讲）。
             * 注意：网络命令不在 Comm 任务里执行，绕一圈到这里才执行 ——
             * 因为执行器的 owner 是这一个任务。 */
        }

        /* 中文：★② 读输入（旋钮 + 两个按键）★
         * 注意这里只"读"不"处理"，处理在第③步。
         * 分开的好处：下面处理逻辑要判断很多条件，先把原始输入取好，
         * 逻辑部分就不会被"调驱动"的代码打断。 */
        const int16_t encoder_delta =
            g_encoder_ready ? rotary_encoder_get_delta() : 0;
        const bool button_pressed = g_encoder_ready &&
                                    rotary_encoder_button_pressed();
        const bool button_clicked = button_pressed &&
                                    !previous_button_pressed;
        previous_button_pressed = button_pressed;
        /* 中文：★`button_clicked` 就是"边沿检测"★
         * "这次按下 且 上次没按" → 一次点击。
         * 没有这个判断的话，按住 100ms 会被 5ms 的循环识别成 20 次点击。 */
        const bool back_pressed = g_back_button_ready &&
                                  back_button_pressed();
        const bool back_clicked = back_pressed && !previous_back_pressed;
        previous_back_pressed = back_pressed;
        const TickType_t now = xTaskGetTickCount();
        /* 中文：★`now` 取一次，整圈都用它★
         * 而不是每处都调一次 xTaskGetTickCount()。
         * 好处：① 省时间；② **同一圈里所有判断用的是同一个时刻**，
         * 不会出现"前后两次读到的 tick 不一样"导致的边界怪现象。 */

        /* 中文：★③ 处理输入 —— 把上一步读到的输入变成动作★
         * 这一大段（到"读光照"之前）没有硬件调用，全是纯逻辑：
         *   旋钮转 → 移动选中项 / 改亮度数值
         *   确认键 → 进页面 / 开关灯 / 切换自动手动 / 切中英文
         *   返回键 → 退编辑 / 回主菜单
         * 因为它不碰硬件、只改几个局部变量，所以写得再长也不影响实时性。 */
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

        /* 中文：★④ 每 200ms 读一次光照（BH1750）★
         * 光照是灯带自动调光的输入源，所以 200ms 一次足够快。
         * 注意 light_valid 的写法：传感器没插好（g_bh1750_ready 为假）时
         * 直接置 false，后面灯带就退化成"固定亮度"而不是乱调 —— 优雅降级。 */
        if ((now - last_light_read) >= pdMS_TO_TICKS(200)) {
            last_light_read = now;
            light_valid =
                g_bh1750_ready && bh1750_read_lux(&light_lux);
        }

        /* 中文：★⑤ 每 200ms 更新一次灯带★
         * 这块逻辑看着长，其实只有三件事：
         *   1) 算出目标亮度（自动模式：按光照反比映射；手动模式：用百分比）
         *   2) 平滑过渡，避免亮度突变
         *   3) **只在亮度真的变了才发帧** —— 因为发一帧要关中断约 0.5ms
         * 详细逻辑见块 6。这里只要知道它是"定时 + 变化才动"的模式。 */
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

        /* 中文：★⑥ 环境传感器：不是"读一次"，而是"发起 → 等待 → 读取"★
         * 温湿度传感器（AHT20）收到"开始测量"指令后，需要约 90ms 才能出结果。
         * 这段时间**不能阻塞**（阻塞了灯带、UI、按键就都停了）。
         * 所以拆成跨循环的三态：
         *   第 N 圈：  发起测量，environment_pending = true
         *   中间几圈： 什么都不做（pending 还是 true）
         *   第 N+k 圈：过了 90ms，读取结果，pending = false
         *   再过 2 秒，回到第一步
         * 这是"非阻塞延时"的典型写法 —— 想睡 90ms 但不想占着 CPU。
         * 详细逻辑见块 5。 */
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

        /* 中文：★⑦ PIR 人体红外 + 30 秒预热★
         * HC-SR501 上电后约 30 秒输出才稳定，所以前 30 秒只显示 WARMUP、
         * 不当作有效的人体检测。预热判断是"开机到现在过了 30 秒吗"。 */

        const bool current_motion =
            g_pir_ready && pir_sensor_motion_detected();
        const bool current_pir_warmed_up =
            (now - app_started_at) >= pdMS_TO_TICKS(PIR_WARMUP_MS);
        if (current_motion != motion_detected ||
            current_pir_warmed_up != pir_warmed_up) {
            motion_detected = current_motion;
            pir_warmed_up = current_pir_warmed_up;
        }

        /* 中文：★⑧ 每 100ms 刷一次屏★
         * 两个动作：先把各种状态组装成 UiStatus_t（ui_status_build），
         * 再交给显示层去画（ui_display_render）。
         * 为什么 100ms 而不是每圈 5ms：屏幕刷新有成本（I2C 写 OLED、
         * SPI 写 TFT），100ms 人眼已经是"实时"了，没必要浪费 CPU。 */
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

        /* 中文：★⑨ 每 200ms 发布一次快照（给 ESP32 的 Web 仪表盘用）★
         * 快照有 18 字节，涵盖所有传感器读数和执行器状态。
         * ESP32 每秒来问一次（CMD_GET_SENSOR_SNAPSHOT），
         * 而这边每 200ms 更新一次 —— 所以 ESP32 拿到的永远是 200ms 内的新鲜数据。
         * 注意写入用临界区保护（见 sensor_snapshot_publish），
         * 因为读它的是另一个任务（Comm）。 */
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
    /* 中文：★ 块 7：监控任务（优先级最低的那个，但责任最大）★
     *
     * 它只做两件事，每秒一次：
     *   ① iwdg_refresh() —— 喂狗。★它是全项目唯一喂狗的地方★
     *      所以"看门狗会不会复位"等价于"这个任务还能不能每秒跑上一次"。
     *   ② 每 5 秒采一次运行时余量（栈水位 + heap 余量），存进缓存。
     *
     * ★ 为什么采样周期是 5 秒，而循环周期是 1 秒？★
     *   因为看门狗超时约 4 秒。如果整个循环改成 5 秒转一圈，
     *   喂狗间隔就超过看门狗超时了 —— 设备会先被自己复位。
     *   所以**循环周期由看门狗决定（1 秒），采样周期独立控制（5 秒）**。
     *   这是一个很容易搞错的地方：优化性能时不能随便把这个 vTaskDelay 调大。
     *
     * ★ 为什么采样和应答解耦 ★
     *   命令处理路径（Comm 任务）只做一次 22 字节结构体拷贝，
     *   不去遍历任务列表 —— 遍历需要一块临时数组，RAM 已经 97% 了。 */

    (void)pvParameters;
    TickType_t last_diag_snapshot = 0U;
    bool diag_sampled = false;
    uint8_t sample_seq = 0U;

    for (;;) {
        const TickType_t now = xTaskGetTickCount();

        /* 每 5s 采样一次运行时余量，写进 static 缓存供 CMD_DIAG_SNAPSHOT 读取。
         * 全程无动态分配，采样本身也不阻塞（句柄在创建任务时已保存）。 */
        if (!diag_sampled ||
            (now - last_diag_snapshot) >=
                pdMS_TO_TICKS(DIAG_SNAPSHOT_UPDATE_MS)) {
            diag_sampled = true;
            last_diag_snapshot = now;

            DiagSnapshot_t sample;
            sample.uptime_ms = (uint32_t)(now * portTICK_PERIOD_MS);
            sample.free_heap_bytes = (uint32_t)xPortGetFreeHeapSize();
            sample.min_ever_free_heap_bytes =
                (uint32_t)xPortGetMinimumEverFreeHeapSize();

            /* 高水位 = 历史最小剩余栈，单位 word；句柄为空说明任务未创建 */
            for (uint8_t i = 0U; i < DIAG_TASK_COUNT; i++) {
                sample.stack_high_water[i] =
                    (g_diag_task_handles[i] != NULL)
                        ? (uint16_t)uxTaskGetStackHighWaterMark(
                              g_diag_task_handles[i])
                        : 0xFFFFU;
            }

            sample.task_count = (uint8_t)DIAG_TASK_COUNT;
            sample.sample_seq = ++sample_seq;

            taskENTER_CRITICAL();
            g_diag_snapshot = sample;
            taskEXIT_CRITICAL();
        }

        iwdg_refresh();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*---------------------------------------------------------------------------
 * app_tasks_init — create all tasks and primitives
 *---------------------------------------------------------------------------*/

void app_tasks_init(void) {
    /* 中文：★ 块 8：把前面四块"组装"起来 ★
     * 这个函数只做两件事：建通信对象、建 4 个任务。
     * 它由 main() 在启动调度器**之前**调用 —— 这一点很重要：
     * 任务是在调度器启动后才开始跑的，所以这里建对象的顺序不影响竞态。 */

    /* Create communication primitives */
    /* 中文：① 建四个通信对象（用法见块 1 的说明）。
     * 注意 g_rx_stream 不是这里创建的 —— 它由 uart_comm_init() 用
     * **静态**方式创建（xStreamBufferCreateStatic），这里只是取句柄。
     * 为什么流缓冲用静态而队列用动态：流缓冲在中断里被使用，
     * 静态分配能保证它在任何时刻都有效，不依赖堆的状态。 */
    g_cmd_queue   = xQueueCreate(QUEUE_CMD_LENGTH, sizeof(OtaRequest_t));
    g_data_queue  = xQueueCreate(QUEUE_DATA_LENGTH, 64);
    g_event_group = xEventGroupCreate();
    g_rx_stream   = uart_comm_get_rx_stream();

    configASSERT(g_cmd_queue != NULL);
    configASSERT(g_data_queue != NULL);
    configASSERT(g_event_group != NULL);
    configASSERT(g_rx_stream != NULL);
    /* 中文：★configASSERT 在 FreeRTOSConfig.h 里被定义成
     * "关中断 + 死循环"（不打印任何东西）★
     * 所以这四个断言一旦失败，设备会**静默卡死**、LED 也不亮。
     * 这是该项目已知的不足：没有日志通道，出错时看不到原因。
     * 改进方向见文件末尾钩子函数的注释。 */

    /* Create tasks — 句柄同时交给 vMonitorTask 做栈水位采集 */
    /* 中文：② 建四个任务。注意每个 xTaskCreate 的最后一个参数
     * 直接把任务句柄写进 g_diag_task_handles[] ——
     * 这样 vMonitorTask 拿得到句柄去查栈水位，不用遍历任务列表。
     * 参数顺序：函数指针、名字、栈大小(字)、参数、优先级、句柄输出。
     * 优先级和栈大小都定义在 app_tasks.h 里，集中管理。 */
    BaseType_t ret;

    ret = xTaskCreate(vCommTask, "Comm", STACK_COMM_TASK, NULL,
                      PRIO_COMM_TASK, &g_diag_task_handles[DIAG_TASK_IDX_COMM]);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vControlTask, "Control", STACK_CONTROL_TASK, NULL,
                      PRIO_CONTROL_TASK,
                      &g_diag_task_handles[DIAG_TASK_IDX_CONTROL]);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vAppTask, "App", STACK_APP_TASK, NULL,
                      PRIO_APP_TASK, &g_diag_task_handles[DIAG_TASK_IDX_APP]);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vMonitorTask, "Monitor", STACK_MONITOR_TASK, NULL,
                      PRIO_MONITOR_TASK,
                      &g_diag_task_handles[DIAG_TASK_IDX_MONITOR]);
    configASSERT(ret == pdPASS);
}

/*---------------------------------------------------------------------------
 * cmd_handler_dispatch — process received frames
 *---------------------------------------------------------------------------*/
/* 中文：★ 块 2 之二：命令分发表（协议层的"路由器"）★
 *
 * 【这个函数跑在哪个任务里？】
 *   答案：**vCommTask**（刚才那个优先级最高的任务）。
 *   这一点是理解整个文件最重要的一条线索，因为它决定了下面每个 case
 *   能做什么、不能做什么。
 *
 * 【它的设计原则：只"翻译 + 转发"，不"执行"】
 *   看下面几处对比就懂了：
 *     CMD_OTA_AVAILABLE  → 不在这里启动升级，只是丢进队列 + 设个事件位
 *     CMD_APP_MSG        → 不在这里操作继电器，只是把文本丢进队列
 *   真正的"执行"发生在别的任务里：
 *     OTA 升级       → vControlTask（块 3）
 *     继电器/蜂鸣器/灯带 → vAppTask（块 4/6）
 *
 * 【为什么必须这样分？两个理由】
 *   ① 它优先级最高（3）。让它干耗时的活（写 Flash、刷屏、发灯带时序）
 *      会长时间霸占 CPU → 串口收包被耽误 → **丢帧**。
 *   ② 业务会碰到共享状态（继电器状态、UI 状态）。如果通信任务也能改，
 *      就会和 App 任务产生竞态。**把执行器的 owner 收敛到一个任务里，
 *      比到处加锁简单得多。**
 *
 * 【面试话术】"我的通信任务只做解析和路由，不执行业务。因为它优先级最高，
 *   而且业务涉及共享状态 —— 让一个任务独占执行器、其他任务用队列提请求，
 *   就不需要锁了。" */

void cmd_handler_dispatch(const ProtoFrame_t *f) {
    /* 中文：一个 switch 就是一张分发表。f->cmd 是 1 字节命令字，
     * 下面每个 case 对应 protocol.h 里定义的一个命令 ID。 */

    switch (f->cmd) {
    case CMD_OTA_AVAILABLE:
        /* 中文：ESP32 说"我这边有新固件了"。
         * payload 8 字节 = 版本号(4) + 镜像大小(4)。 */
        if (f->len >= 4) {
            OtaRequest_t request = {0};
            memcpy(&request.version, f->payload, 4);
            if (f->len >= 8) {
                memcpy(&request.size, f->payload + 4, 4);
            }
            /* 中文：★这两行的写法是"向后兼容的字段追加"★
             * 先判 len >= 4 读版本号，再判 len >= 8 读大小。
             * 所以：旧固件收到 8 字节也不会出错（忽略多的 4 字节）；
             *       新固件收到 4 字节也能降级工作（size 保持 0）。
             * 这就是"协议演进不需要版本号"的具体做法 ——
             * **加字段只能追加在尾部，靠长度判断要不要解析**。 */
            xQueueSend(g_cmd_queue, &request, 0);
            xEventGroupSetBits(g_event_group, EVENT_OTA_AVAILABLE);
            /* 中文：★转发给 vControlTask 的两种机制，一次用全了★
             *   队列     —— 送"数据"（版本 + 大小）
             *   事件组   —— 送"通知"（有升级请求了，去处理）
             * 为什么不用一个队列搞定：因为 vControlTask 大部分时间阻塞在
             * 事件组上，收到通知后再去取队列。两者语义不同，分开更清楚。
             * 超时参数 0 = 不等待。队列满就丢 —— 命令是幂等的，丢了重发即可。 */
        }
        break;

    case CMD_RECOVERY_CHECK:
        {
            /* The application is running, so by definition it does not need
             * firmware re-sent. Answering this command at all is the signal:
             * when the bootloader owns the link it answers CMD_RECOVERY_RSP
             * instead, and never reaches here. */
            /* 中文：★这段英文注释很精妙，值得单独说★
             * ESP32 发这条命令是在问"现在谁在跑？你需要固件吗？"
             * 关键洞察：**能走到这里回答，这件事本身就是答案。**
             *   应用在跑 → 说明不需要重推固件 → 回 NORMAL
             *   如果 bootloader 在跑 → 它会回 RECOVERY_RSP，根本到不了这个函数
             * 所以这里不需要任何判断，直接回 NORMAL 就是正确的。 */
            uint32_t status = RECOVERY_STATUS_NORMAL;
            cmd_handler_send_frame(CMD_RECOVERY_RSP,
                                   (const uint8_t *)&status, 4);
        }
        break;

    case CMD_APP_MSG:
        /* 中文：★控制命令（"RELAY2 ON"这类文本）—— 注意这里不执行！★ */
        if (f->len > 0 && f->len <= 64) {
            /* Queue items are fixed-size; zero-fill the tail so the text
             * command is NUL-terminated for the parser in vAppTask. */
            uint8_t msg[64] = {0};
            /* 中文：★为什么要这么绕一下★
             * 队列的每个元素大小是固定的 64 字节（创建时就定死了）。
             * 但收到的文本可能只有 9 个字节（"RELAY2 ON"）。
             * 所以：先准备一个全 0 的 64 字节数组 → 把实际内容拷进去
             * → 剩下的部分天然就是 '\0'。
             * **这样 App 任务那边收到的字符串一定是 NUL 结尾的**，
             * 可以直接当 C 字符串用（strcmp / 逐字符解析都不会读越界）。
             * 不这么做的话，App 任务读到第 10 个字节之后就是内存垃圾。 */
            memcpy(msg, f->payload, f->len);
            xQueueSend(g_data_queue, msg, 0);
            /* 中文：丢给 vAppTask 去执行（块 6 的 handle_control_command）。
             * 这就是"单一 owner + 消息传递"：通信任务只管把请求投递过去，
             * 谁拥有继电器谁去动它。 */
        }
        break;

    case CMD_GET_STATUS:
        {
            /* 中文：问固件版本号。去 Flash 配置区读出来回过去。 */
            BootConfig_t cfg;
            uint32_t version = 0;
            if (ota_config_read(&cfg)) {
                version = cfg.fw_version;
            }
            cmd_handler_send_frame(CMD_STATUS_RSP, (uint8_t *)&version, 4);
            /* 中文：读不出配置就回 0。注意这是"应用"回的版本号，
             * 和 bootloader 用同一个命令 ID 回"恢复状态"是两回事 ——
             * 那正是历史上出过事故的地方（见 shared/protocol.h 的说明）。 */
        }
        break;

    case CMD_GET_SENSOR_SNAPSHOT:
        {
            /* 中文：★ESP32 每秒问一次"设备现在什么状态"★
             * 18 字节快照由 vAppTask 每 200ms 更新一次。
             * 这里的写法是"临界区里整体拷贝"： */
            SensorSnapshot_t snapshot;
            taskENTER_CRITICAL();
            snapshot = g_sensor_snapshot;
            taskEXIT_CRITICAL();
            /* 中文：★为什么用临界区而不是队列★
             * 因为这是"最新值语义"：消费者要的永远是"现在是什么样"，
             * 旧的那份没有价值。
             * 用队列会积压一串过期数据（读到 5 秒前的温度），
             * 而且要多占 18 字节 × N 的缓冲 —— 在 RAM 只剩几百字节时不可接受。
             * 临界区拷贝则是零额外内存 + 天然最新值。
             * 注意 18 字节的赋值不是原子的，所以必须包在临界区里，
             * 否则可能读到"一半新一半旧"的快照。 */
            cmd_handler_send_frame(CMD_SENSOR_SNAPSHOT_RSP,
                                   (const uint8_t *)&snapshot,
                                   sizeof(snapshot));
        }
        break;

    case CMD_DIAG_SNAPSHOT:
        {
            /* 请求 payload 为空；直接返回 vMonitorTask 缓存的上一次采样 */
            DiagSnapshot_t diag;
            taskENTER_CRITICAL();
            diag = g_diag_snapshot;
            taskEXIT_CRITICAL();
            cmd_handler_send_frame(CMD_DIAG_SNAPSHOT_RSP,
                                   (const uint8_t *)&diag,
                                   sizeof(diag));
            /* 中文：和上面快照完全一样的模式。区别只是数据来源：
             * 这份是 vMonitorTask 每 5 秒采一次的栈水位 / heap 余量。
             * 采样和应答解耦 —— 命令处理路径只做一次 22 字节拷贝，
             * 不去遍历 FreeRTOS 任务列表（那需要临时数组，RAM 不够）。 */
        }
        break;

    case CMD_RESET:
        NVIC_SystemReset();
        /* 中文：软复位。不返回。 */
        break;

    default:
        /* Unknown command — ignore in application context */
        break;
        /* 中文：★未知命令在应用里是"静默忽略"，注意和 bootloader 的区别★
         *   bootloader 收到未知命令 → 回 NAK + ERR_UNKNOWN_CMD
         *   应用      收到未知命令 → 什么都不做
         * 为什么不一样：
         *   bootloader 处于升级流程中，对端在等 ACK，**必须明确告知失败**，
         *   否则 ESP32 会一直等到超时。
         *   应用是常态运行，对端发来不认识的东西多半是版本不匹配或噪声，
         *   回 NAK 反而可能触发对端不必要的重试。
         * 这个"同一个协议在两端的错误处理策略不同"是有意为之，不是遗漏。 */
    }
}

void cmd_handler_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    /* 中文：★组帧 + 阻塞发送。注意这个 1036 字节的栈上缓冲区★
     * PROTO_MAX_FRAME = 1036 字节，直接开在**栈上**。
     * 加上 vCommTask 栈上那个 1048 字节的解析器，
     * 最深调用路径上的局部变量合计约 2084 字节 ——
     * **这就是 Comm 任务栈为什么开 768 字（3072 字节）的答案。**
     * 任务栈大小不是拍脑袋，是算最深调用路径上的局部变量总和，再留余量。 */
    uint8_t buf[PROTO_MAX_FRAME];
    uint16_t total = proto_build_frame(buf, sizeof(buf), cmd, payload, len);
    if (total > 0) {
        /* Send over UART in task context (blocking, but the payload is small). */
        uart_comm_send(buf, total);
        /* 中文：阻塞式一个字节一个字节推出去。发 26 字节 @115200 约 2.3ms，
         * 对 Comm 任务来说可以接受（而且它优先级最高，没人能抢它）。
         * 注意必须检查 total > 0 —— 返回 0 表示"没组装成功"（payload 超长
         * 或缓冲区不够），不检查就等于把"没发出去"当成"发出去了"。 */
    }
}

/*---------------------------------------------------------------------------
 * main entry point
 *---------------------------------------------------------------------------*/

int main(void) {
    /* 中文：★ 块 8 之二：main() —— 上电到调度器启动的全部顺序 ★
     *
     * 顺序不是随便排的，每一步都有依赖：
     *   ① ramfunc_init()   必须最先 —— 后面写 Flash 要用 RAM 里的函数
     *   ② SCB->VTOR        必须在任何中断可能发生之前
     *   ③ system_init()    HAL + 64MHz 时钟 + LED
     *   ④ iwdg_init()      开看门狗（放在时钟后，寄存器写序列才稳）
     *   ⑤ relay_init()     ★刻意排在传感器/显示之前★
     *   ⑥ uart_comm_init() 串口（含 DMA + IDLE 中断）
     *   ⑦ 各传感器/显示/输入初始化（慢，几十毫秒）
     *   ⑧ app_tasks_init() 建队列和任务（此时还没人跑）
     *   ⑨ vTaskStartScheduler() 交棒给 FreeRTOS，永不返回
     *
     * ⑤ 为什么继电器要提前：继电器是低电平触发，上电瞬间引脚状态不确定
     * 可能让继电器"啪"一下。先把它驱动到"断开"状态，再去初始化那些
     * 慢的传感器 —— 减少复位期间的误动作。
     * （注释里也老实写了：这是"减少瞬态"，不是根治；稳定的模块供电
     *   和输入偏置仍然是硬件要求。） */

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
/* 中文：★ 块 8 之三：FreeRTOS 钩子 —— 出事了怎么办 ★
 *
 * 两个钩子都是同一个套路：**点亮 LED + 关中断 + 死循环**。
 *   vApplicationStackOverflowHook  任务栈溢出（方法 2：检查栈末尾的填充字）
 *   vApplicationMallocFailedHook   堆分配失败（14KB heap 用光了）
 *
 * ★ 为什么是"静默挂死"而不是打印日志 ★
 *   这个应用的串口被协议帧占满了，没有留给日志的通道。要加 printf
 *   得额外实现一个重定向 + 占栈空间 —— 在 RAM 只剩几百字节时是奢侈的。
 *
 * ★ 代价（已知不足，面试主动说出来）★
 *   任何断言/钩子触发时，你**看不到是哪个任务、什么原因**。
 *   只能靠"LED 常亮"知道"挂死了"。
 *   改进方向（零成本、也不占串口）：
 *     ① 像 bootloader 的 bootloader_halt_error(N) 那样，
 *        把 pcTaskName 或失败原因编码成 **LED 闪烁次数**
 *     ② 或者留一个小的错误环形缓冲区，重启后通过诊断命令读出来
 *   注意这个死循环**不喂狗**，所以约 4 秒后看门狗会复位设备 ——
 *   于是表现为"LED 亮一下又重启"的循环。 */

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
/* 中文：★ 这三行包装是"让 FreeRTOS 接管三个内核异常"的接线 ★
 *
 * 背景：STM32 的启动文件里已经写好了一张中断向量表，
 * 表里第三项是 SVC_Handler、第四项是 PendSV_Handler、最后是 SysTick_Handler。
 * 而 FreeRTOS 的 Cortex-M3 移植层把自己的实现命名成
 * vPortSVCHandler / xPortPendSVHandler / xPortSysTickHandler。
 * 名字对不上，链接器就找不到 —— 所以要么改启动文件，要么写个包装函数。
 * 这里选了包装（和 CubeMX 生成的 stm32f1xx_it.c 是同一个套路）。
 *
 * ★ 这三个异常为什么必须归 FreeRTOS 管 ★
 *   SVC      —— 启动第一个任务时用（从特权态切到任务上下文）
 *   PendSV   —— **任务上下文切换**就靠它（设为最低优先级，保证不打断别人）
 *   SysTick  —— 系统时基，每次 tick 决定要不要切换任务
 * 任何一个没接上，FreeRTOS 都跑不起来。
 *
 * ★ 注意最后那个 SysTick_Handler 里有个判断 ★
 *   它先调 HAL_IncTick()（HAL 的毫秒计时），再判断"调度器启动了吗" ——
 *   只有启动后才调 xPortSysTickHandler()。
 *   为什么：调度器启动前的 SysTick 只用来给 HAL_Delay 计时，
 *   那时调 FreeRTOS 的 tick 处理会访问还没初始化的内核数据结构 → 崩。 */

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
