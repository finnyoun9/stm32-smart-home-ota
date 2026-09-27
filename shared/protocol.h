/**
 * @file    protocol.h
 * @brief   Shared UART protocol definitions for STM32 ↔ ESP32 communication.
 *
 * Used identically by: bootloader, application, ESP32, and PC test tools.
 *
 * ===========================================================================
 * 中文导读：这个文件是整台设备的「通信宪法」
 * ===========================================================================
 * 它只定两件事：
 *   1. 「字节怎么摆」——帧格式、结构体的内存布局、字节序；
 *   2. 「数字代表什么」——命令编号、错误码、状态枚举、超时常量。
 *
 * 它被编进下面这些目标（这就是「共享」的含义）：
 *   bootloader/Core/Src/main.c      → STM32 引导程序
 *   application/Core/Src/main.c     → STM32 应用
 *   esp32-comm-bridge/src/main.cpp  → ESP32 网关（走 shared_protocol 组件）
 *   tools 目录下的 xxx_test.c        → PC 上的主机测试（gcc 直接编）
 *
 * 为什么必须共享：这四边对同一个字节的解释必须逐位一致。任何一边改了这个
 * 文件而另一边没跟上，通信会「静默出错」——不崩溃、不报错，只是行为不对，
 * 这种 bug 最难查。所以这里的每个数字都是契约，改之前先想另外三边怎么办。
 *
 * 本文件零依赖：只用 stdint/stddef/stdbool，不 include 任何 HAL 或 FreeRTOS。
 * 这正是它能被 PC 上的主机测试直接编译的原因（见文件末尾的 CRC 段落）。
 * ===========================================================================
 *
 * Frame format (little-endian on the wire) / 帧格式（网络上按小端传输）:
 *   [SYNC 0xA5] [CMD 1B] [LEN_L 1B] [LEN_H 1B] [PAYLOAD LEN bytes] [CRC32 4B LE]
 *
 * Max frame: 4 + 1028 + 4 = 1036 bytes.
 */

#ifndef SHARED_PROTOCOL_H
#define SHARED_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * Constants
 *---------------------------------------------------------------------------*/

/* 中文：帧的四个组成部分。收包时解析器按这个顺序逐字节推进。
 *
 *   ┌──────┬─────┬───────┬───────┬──────────────┬──────────┐
 *   │ 0xA5 │ CMD │ LEN_L │ LEN_H │ payload      │ CRC32    │
 *   │ 1B   │ 1B  │ 1B    │ 1B    │ LEN 字节     │ 4B 小端  │
 *   └──────┴─────┴───────┴───────┴──────────────┴──────────┘
 *     同步字  命令  长度低  长度高   真正的数据     校验
 *
 * 逐字对照源码里的实现（shared/protocol.c 的 proto_build_frame）：
 *   buf[0] = 0xA5                buf[1] = cmd
 *   buf[2] = len & 0xFF          buf[3] = len >> 8
 *   buf[4..] = payload           buf[末尾4B] = CRC32，低字节在前
 *
 * 注意 LEN 是「小端」：先低字节后高字节。上面那句 "little-endian on the
 * wire" 说的就是这件事。（原文曾写成 big-endian，与实现不符，已更正。） */
#define PROTO_SYNC_BYTE         0xA5U
#define PROTO_MAX_PAYLOAD       1028U /* OTA chunk: sequence (4) + page data (1024) */
#define PROTO_HEADER_SIZE       4U    /* SYNC + CMD + LEN(2) */
#define PROTO_CRC_SIZE          4U    /* CRC32 LE */
#define PROTO_MAX_FRAME         (PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD + PROTO_CRC_SIZE)
/* 中文：PROTO_MAX_PAYLOAD = 1028 = 4 字节序号 + 1024 字节数据。
 * 为什么是 1028 而不是 1024：STM32F103 的 Flash 一页正好 1KB，所以 OTA 传输
 * 天然以 1KB 为粒度；但每个 chunk 还要带上「我是第几块」的 4 字节序号，
 * 于是 payload 变成 1028。这一条把「Flash 的物理页大小」写进了协议语义里，
 * 是优点（简单：addr = APP_BASE + seq*1024）也是负债（换页大小不同的芯片要改协议）。 */
/* 中文：PROTO_MAX_FRAME = 4 + 1028 + 4 = 1036。收包缓冲区必须 ≥ 这个数，
 * 否则环形缓冲回绕时会截断一帧。见 application/Core/Inc/uart_comm.h 的
 * UART_RX_BUF_SIZE = 1152。 */

/*---------------------------------------------------------------------------
 * 中文：Flash 分区（把一块 64KB 的 Flash 切成三个用途固定的区域）
 *
 *   0x0800 0000 ┌──────────────────────┐
 *               │ Bootloader   8KB     │ 上电先跑它；它决定接下来跑谁
 *   0x0800 2000 ├──────────────────────┤
 *               │ Application  54KB    │ 真正的业务固件（FreeRTOS 四任务）
 *   0x0800 F800 ├──────────────────────┤
 *               │ Config       2KB     │ 两份配置，各占一页，ping-pong 轮换
 *   0x0801 0000 └──────────────────────┘ （64KB 结束）
 *
 * 为什么必须"约定"而不是各写各的：同一个地址被四处同时依赖——
 *   ① bootloader 跳转的目标地址（bootloader_jump_to_app）
 *   ② application 自己的链接起始地址（application/STM32F103C8TX_APP.ld）
 *   ③ OTA 时 bootloader 擦写的目标地址（process_ota_chunk）
 *   ④ ESP32 校验上传镜像是否合法的地址范围（0x08002000..0x0800F800）
 * 任何一处和这里不一致，OTA 就会把固件写到错误的地方。
 *---------------------------------------------------------------------------*/

/* Flash layout (shared so bootloader and app agree) */
/* #ifndef guards: STM32 HAL already defines FLASH_BASE/FLASH_PAGE_SIZE */
#ifndef FLASH_BASE
#define FLASH_BASE              0x08000000U
#endif
#ifndef FLASH_SIZE
#define FLASH_SIZE              0x00010000U   /* 64KB */
#endif
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE         1024U         /* 1KB per page on F103C8 */
#endif

/* 中文：F103C8 的 Flash 只能整页擦除，一页 = 1KB。这是本项目很多设计的
 * 根源：OTA 以 1KB 为单位传、配置要分两页放、擦了就不能只改一半。 */

#define BOOTLOADER_BASE         0x08000000U
#define BOOTLOADER_SIZE         0x00002000U   /* 8KB, pages 0-7 */
#define BOOTLOADER_PAGES        8U
/* 中文：8KB 是硬约束（链接脚本同样写死 8K）。超了就直接链接失败。 */

#define APP_BASE                0x08002000U
#define APP_SIZE                0x0000D800U   /* 54KB, pages 8-61 */
#define APP_PAGES               54U
/* 中文：应用从 0x08002000 开始。所以应用启动第一件事必须是
 * SCB->VTOR = APP_BASE（重定位中断向量表），否则一进中断就跑飞。 */

#define CONFIG_BASE             0x0800F800U   /* page 62 */
#define CONFIG_SIZE             0x00000800U   /* 2KB, pages 62-63 */
#define CONFIG_PAGE             62U
/* 中文：配置区给了 2KB = 2 页，但 BootConfig_t 只有 56 字节。
 * 为什么要浪费整个第二页？见下面的 ping-pong 说明。 */

/* The config region holds two identical copies, one per page, written
 * alternately (ping-pong). This is what makes a config update atomic: a single
 * flash page can only be erased as a whole, so two copies inside one page would
 * both vanish during the erase. With one copy per page, the page that is not
 * being written always still holds the previous valid config, and
 * BootConfig_t.config_seq decides which copy is newer. */
/* 中文：这一段是整个"掉电不丢配置"机制的核心，值得逐句读懂。
 *
 * 问题：Flash 只能「整页擦除」，不能只改 56 个字节。而"擦除"和"写入"之间
 * 存在一个时间窗口，这时候掉电，那一页就是空的。
 *
 * 错误做法：把两份 56 字节的配置放在同一页的两个偏移，写入时切换偏移。
 *   → 切回第一个偏移时必须擦整页 → 两份同时消失 → 中间掉电全丢。
 *
 * 正确做法（本项目）：两份配置各占一整页（62 页和 63 页）。
 *   写新版时，永远擦「不是最新那份」所在的那一页。
 *   → 任何时刻，至少有一页是完整有效的。
 *   → 掉电后读出来发现新版 CRC 不对，就用旧的那份，设备照常启动。
 *
 * 谁是"最新"？由 BootConfig_t.config_seq 决定：两份都有效时，序号大的赢。
 * 这就是"ping-pong"（乒乓）：写入在两页之间来回跳。
 *
 * 完整实现在 shared/ota_config.c 的 next_write_page()。 */
#define CONFIG_PAGE_A           62U
#define CONFIG_PAGE_B           63U
#define CONFIG_RECORD_SIZE      64U           /* padded record per page */

/* OTA window: bootloader waits this long (ms) for OTA_BEGIN after reset */
#define OTA_WINDOW_MS           200U
/* 中文：正常上电时，bootloader 只等 200ms。因为正常启动不该被拖慢，
 * 而"要升级"这件事是通过 config 标志提前告诉它的，不靠这个窗口。 */

/* Window used when the bootloader has decided the host must re-send firmware
 * (partial/invalid image, or an application that never confirmed its boot). */
#define OTA_RECOVERY_WINDOW_MS  10000U
/* 中文：一旦判定"应用是坏的"，窗口从 200ms 拉长到 10 秒。
 * 目的：让 ESP32 有足够时间发现并自动重推固件，不需要人手去按复位。
 * 这就是"设备停在可救援状态"而不是"变砖"的区别。 */

/* Consecutive unconfirmed boots before the bootloader declares the image bad
 * and opens the long recovery window. */
#define BOOT_ATTEMPT_LIMIT      3U
/* 中文：应用启动 3 秒后必须主动"报平安"（ota_config_confirm_boot）。
 * 连续 3 次上电都没报平安（说明它在启动阶段就崩了），bootloader 就
 * 认定这个镜像是坏的。为什么是 3 次而不是 1 次：给偶发故障留余地，
 * 一次上电失败可能是供电抖动，不是固件问题。 */

/* Per-chunk ACK timeout (ms) — ESP32 side.
 * Sized for the original 9600-baud bring-up link (~1.1s per 1 KiB chunk);
 * generous headroom at the current 115200 baud, safe to tighten later. */
#define OTA_CHUNK_TIMEOUT_MS    2000U
/* Max retries per chunk */
#define OTA_MAX_RETRIES         3U
/* 中文：这两个常量是 ESP32 侧「发一块、等一个 ACK、超时就重发」的参数。
 * 注意注释里那句诚实的话：2000ms 是按当年 9600 波特率算的，现在跑 115200
 * 已经绰绰有余 —— 但作者选择保留保守值，而不是顺手改小。这类"知道可以优化
 * 但先不动"的判断，面试里可以讲。 */

/*---------------------------------------------------------------------------
 * Command IDs
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * 中文：命令表 —— 这就是这套协议对外暴露的「API 清单」
 *
 * CMD 那 1 个字节 = HTTP 里的 URL 路径。它决定了：
 *   · 这帧要干什么
 *   · payload 里装的是什么结构
 *   · 对方该回哪个 ID
 *
 * 编号不是随手排的，分了三个区间 + 一个方向位：
 *
 *   主机(ESP32) → 从机(STM32)：
 *     0x10 – 0x1F   OTA / 固件升级流程
 *     0x20          APP_MSG：文本命令透传（RELAY2 ON、LIGHT AUTO…）
 *     0x30 – 0x3F   查询类：状态、传感器快照、诊断、恢复检查
 *
 *   从机(STM32) → 主机(ESP32)：
 *     0x80 – 0x8F   全部响应。约定「响应 = 请求 ID + 0x80 附近」不是
 *                   严格算术关系，但整体落在 0x8x 区间，一眼能看出方向。
 *
 * 为什么要分区间而不是从 1 开始顺序编号：新增命令时不容易撞车，
 * 而且抓包时看到 0x8x 就知道是回包。tools/recovery_protocol_test.c 里
 * 有一个用例专门断言"所有 ID 互不重复"——因为复制粘贴改错 ID 是很现实的
 * 失误，而重复 ID 会在真机上静默走错分支。
 *---------------------------------------------------------------------------*/

/* Master → Slave (ESP32 → STM32) */
#define CMD_OTA_BEGIN           0x10U
#define CMD_OTA_CHUNK           0x11U
#define CMD_OTA_END             0x12U
#define CMD_OTA_ABORT           0x13U
#define CMD_OTA_AVAILABLE       0x14U  /* "update ready, please reboot" */
#define CMD_APP_MSG             0x20U  /* passthrough data to application */
#define CMD_GET_STATUS          0x30U
#define CMD_RESET               0x31U
#define CMD_GET_SENSOR_SNAPSHOT 0x32U  /* request latest app sensor state */
#define CMD_DIAG_SNAPSHOT       0x33U  /* request cached runtime diagnostics */
#define CMD_RECOVERY_CHECK      0x34U  /* "who is running, and do you need firmware?" */
#define CMD_OTA_COMMITTED       0x35U  /* app: "version N booted and confirmed" */
/* 中文逐条：
 *   0x10 OTA_BEGIN      主机说"开始传固件"，payload 带版本/大小/CRC
 *   0x11 OTA_CHUNK      一块数据，payload = 4 字节序号 + 最多 1024 字节
 *   0x12 OTA_END        传完了，请 bootloader 做整镜像 CRC 校验
 *   0x13 OTA_ABORT      中途放弃
 *   0x14 OTA_AVAILABLE  "我这边有新固件了，请重启进 bootloader"
 *                       ↑ 注意：这帧是发给"正在运行的应用"的，不是 bootloader
 *   0x20 APP_MSG        透传文本命令，应用侧解析（RELAY2 ON 之类）
 *   0x30 GET_STATUS     问固件版本
 *   0x31 RESET          要求重新枚举/复位
 *   0x32 GET_SENSOR_SNAPSHOT  问 18 字节环境快照（Web 仪表盘每秒问一次）
 *   0x33 DIAG_SNAPSHOT  问 22 字节运行诊断（栈水位/heap 余量）
 *   0x34 RECOVERY_CHECK 问"现在谁在跑？你需要固件吗？"
 *                       ↑ 这条是给 bootloader 答的，见下面 RSP 的说明
 *   0x35 OTA_COMMITTED  应用启动成功后的"报平安 + 报版本号" */

/* Slave → Master (STM32 → ESP32) */
#define CMD_OTA_BEGIN_ACK       0x81U
#define CMD_CHUNK_ACK           0x82U
#define CMD_NAK                 0x83U
#define CMD_OTA_RESULT          0x84U
#define CMD_STATUS_RSP          0x85U
#define CMD_OTA_READY           0x86U  /* app saved OTA request and will reboot */
#define CMD_SENSOR_SNAPSHOT_RSP 0x87U
#define CMD_DIAG_SNAPSHOT_RSP   0x88U
#define CMD_RECOVERY_RSP        0x89U  /* RECOVERY_STATUS_* payload (4 bytes LE) */
/* 中文逐条：
 *   0x81 BEGIN_ACK    bootloader 说"收到，可以开始发数据了"
 *   0x82 CHUNK_ACK    某一块写好了；ESP32 每发一块都等这个，超时就重发
 *   0x83 NAK          出错，payload 带错误码（见下面 ERR_*）
 *   0x84 OTA_RESULT   整镜像校验结果（成功/失败）
 *   0x85 STATUS_RSP   ← 历史坑位，见下方专门说明
 *   0x86 OTA_READY    应用说"配置已写盘，我马上复位"，ESP32 收到后才等 bootloader
 *   0x87 SENSOR_SNAPSHOT_RSP  18 字节快照
 *   0x88 DIAG_SNAPSHOT_RSP    22 字节诊断
 *   0x89 RECOVERY_RSP 恢复状态（RECOVERY_STATUS_*）
 *
 * ★ 关于 0x85 的历史事故（面试可讲）：
 *   最初 bootloader 和 app 共用 CMD_STATUS_RSP 回包，但语义不同——
 *     bootloader 回的是 RECOVERY_STATUS_*（1 = OTA_PENDING）
 *     app        回的是固件版本号（1、2、3…）
 *   于是「固件版本 1」被 ESP32 误判成「这台设备需要恢复」，
 *   结果把一个健康的设备自动刷回了旧固件。
 *   修法：恢复状态改用专用命令 0x89，两者不再共用通道。
 *   tools/recovery_protocol_test.c 的用例 02 专门钉住这个回归。
 *   教训：同一个命令 ID 承载两种语义 = 迟早出事，哪怕数值看起来巧合。 */

/* DiagSnapshot_t.stack_high_water[] index order (fixed across all builds) */
#define DIAG_TASK_COUNT         4U
#define DIAG_TASK_IDX_COMM      0U
#define DIAG_TASK_IDX_CONTROL   1U
#define DIAG_TASK_IDX_APP       2U
#define DIAG_TASK_IDX_MONITOR   3U

/* SensorSnapshot_t.flags */
#define SENSOR_FLAG_ENV_VALID     (1U << 0)
#define SENSOR_FLAG_LIGHT_VALID   (1U << 1)
#define SENSOR_FLAG_PIR_READY     (1U << 2)
#define SENSOR_FLAG_PIR_WARMED_UP (1U << 3)
#define SENSOR_FLAG_MOTION        (1U << 4)
#define SENSOR_FLAG_RELAY1_ON     (1U << 5)
#define SENSOR_FLAG_RELAY2_ON     (1U << 6)
#define SENSOR_FLAG_AUTO_MODE     (1U << 7)
#define SENSOR_FLAG_BUZZER_ON     (1U << 8)
#define SENSOR_FLAG_UI_CHINESE    (1U << 9)

/*---------------------------------------------------------------------------
 * Error codes (in NAK / RSP_ERROR payloads)
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * 中文：错误码 —— NAK 或 RSP_ERROR 的 payload 里会带这个数字
 *
 * 为什么要分这么细，而不是只回一句"失败了"：
 * 主机（ESP32）需要据此决定「能不能重试」。比如 ERR_SEQ_MISMATCH 重发同一块
 * 就行，ERR_IMAGE_CRC 说明整个镜像坏了必须重传，ERR_SIZE_TOO_LARGE 则是
 * 参数错、重试多少次都没用。错误码是"可编程的失败原因"。
 *---------------------------------------------------------------------------*/

#define ERR_NONE                0x00000000U
#define ERR_FRAME_CRC           0x00000001U
#define ERR_SEQ_MISMATCH        0x00000002U
#define ERR_FLASH_ERASE         0x00000003U
#define ERR_FLASH_PROGRAM       0x00000004U
#define ERR_FLASH_VERIFY        0x00000005U
#define ERR_IMAGE_CRC           0x00000006U
#define ERR_SIZE_TOO_LARGE      0x00000007U
#define ERR_UNKNOWN_CMD         0x00000008U
#define ERR_TIMEOUT             0x00000009U
#define ERR_BUSY                0x0000000AU

/*---------------------------------------------------------------------------
 * OTA result codes (in OTA_RESULT payload)
 *---------------------------------------------------------------------------*/

#define OTA_RESULT_OK           0x00000000U
#define OTA_RESULT_FAIL         0x00000001U

/*---------------------------------------------------------------------------
 * Boot modes
 *---------------------------------------------------------------------------*/

#define BOOT_MODE_APP           0x00000000U
#define BOOT_MODE_OTA           0x00000001U
/* 中文：BootConfig_t.boot_mode 的两个取值，存在 Flash 里。
 * BOOT_MODE_OTA 是「应用主动写下的升级请求」——应用写完就复位，
 * bootloader 上电读到它，于是打开 OTA 握手窗口而不是直接跳回应用。
 * 这是应用的唯一一种"触碰 Flash"的操作（其余 Flash 操作全在 bootloader）。 */

/*---------------------------------------------------------------------------
 * Update status
 *---------------------------------------------------------------------------*/

#define UPDATE_STATUS_NONE      0x00000000U
#define UPDATE_STATUS_OK        0x00000001U
#define UPDATE_STATUS_FAILED    0x00000002U
#define UPDATE_STATUS_IN_PROGRESS 0x00000003U
/* 中文：这次升级"到哪一步了"，写进 Flash，掉电也不丢。
 * 应用在开始传之前就把 IN_PROGRESS 落盘 —— 这样 bootloader 事后知道
 * "上次升级没走完"，可以据此判断镜像可能是半新半旧的。 */

/*---------------------------------------------------------------------------
 * Recovery status — the bootloader's own reason for opening the OTA window.
 *
 * Reported in CMD_STATUS_RSP so the host can tell "all good, this was just the
 * passive boot window" apart from "this device needs firmware re-sent".
 * ---------------------------------------------------------------------------*/
/* 中文：这是 bootloader 对"我为什么还停在这里"的自述。
 *
 * 场景：ESP32 的传感器轮询连续失败，说明 STM32 不在正常运行。于是它发
 * CMD_RECOVERY_CHECK(0x34) 问一句"谁在跑？你需要固件吗？"。bootloader
 * 用 CMD_RECOVERY_RSP(0x89) 回下面这五个值之一。
 *
 * 为什么要区分这么细：只有 NORMAL 之外的值才触发"自动重推固件"。
 * 如果分不清"正常启动窗口"和"镜像坏了"，就会把健康设备刷回旧版本
 * （历史上真的这么错过一次，见命令表里 0x85 的事故说明）。 */

#define RECOVERY_STATUS_NORMAL           0x00000000U /* app valid + boot confirmed */
#define RECOVERY_STATUS_OTA_PENDING      0x00000001U /* app asked for an update */
#define RECOVERY_STATUS_IMAGE_INVALID    0x00000002U /* image missing or CRC mismatch */
#define RECOVERY_STATUS_BOOT_UNCONFIRMED 0x00000003U /* app never proved it runs */
#define RECOVERY_STATUS_NO_APP           0x00000004U /* nothing bootable in flash */
/* 中文：
 *   0 NORMAL           一切正常，这次只是常规的上电被动窗口
 *   1 OTA_PENDING      应用自己要求升级（正常流程，不该触发自动回滚）
 *   2 IMAGE_INVALID    镜像校验不过 —— 多半是传输途中掉电，写到一半
 *   3 BOOT_UNCONFIRMED 镜像字节是对的，但连续 3 次都没能报平安 = 启动就崩
 *   4 NO_APP           Flash 里根本没有可启动的镜像
 * 只有 2/3/4 会让 ESP32 自动重推 golden 镜像。 */

/*---------------------------------------------------------------------------
 * Config struct magic
 *---------------------------------------------------------------------------*/

#define BOOT_CONFIG_MAGIC       0x424F4F54U  /* 'BOOT' */

/*---------------------------------------------------------------------------
 * Data structures
 *---------------------------------------------------------------------------*/
/* 中文：什么是「结构体布局」，以及为什么要专门约定它
 *
 * 本协议把结构体**直接当 payload 发**（不是 JSON、不是逐字段编码）。
 * 也就是把 `SensorSnapshot_t` 那块内存原样塞进帧里传走。这样做的代价是：
 * 两端的编译结果必须逐字节一致，具体包括三件事——
 *
 *   ① 字段顺序与宽度：编译器按声明顺序排布。所以不能随便调字段位置。
 *   ② 对齐填充：编译器默认会在字段之间插入 padding 让访问更快。
 *      `#pragma pack(1)` 强制取消所有填充，让布局完全由字段自己决定。
 *   ③ 字节序：多字节整数是低字节在前还是高字节在前。
 *      本项目两端（STM32F1 和 ESP32）都是小端，所以没做转换。
 *
 * 换编译器、换芯片、开不同优化等级，都可能改变布局 —— 于是用
 * `_Static_assert(sizeof(...) == N)` 把尺寸焊死（见本段末尾）。
 * 一旦哪天布局变了，编译期直接报错，而不是等到真机上数据错位。
 *
 * 收益：比 JSON 省几个数量级的 CPU 和内存（F103 没有 FPU，也没有余量跑
 * 序列化库）。代价：没有兼容性——加字段只能追加在尾部，且两边必须同时升级。
 * 本项目没有协议版本协商，这是明确记下的技术债。
 *---------------------------------------------------------------------------*/

#pragma pack(push, 1)

/**
 * @brief Boot configuration, stored as two ping-pong copies in the config pages.
 *
 * Written only on OTA transitions and boot confirmation. Must be byte-identical
 * across bootloader and application compiles — hence the static assertion on
 * its size further down.
 *
 * `config_seq` is the ping-pong tiebreaker: `ota_config_write()` writes to the
 * page that does *not* hold the newest valid copy, with `config_seq`
 * incremented. A reader picks the valid copy with the highest sequence, so a
 * power loss during a write leaves the other copy intact and authoritative.
 *
 * `boot_confirmed` is the durable half of the boot-verification contract:
 *   ota_config_mark_valid(): set 0 — a freshly committed image has not run yet
 *   application:             set 1 once it is demonstrably healthy
 * The live half is a counter in the RTC backup registers, incremented by the
 * bootloader on every boot and zeroed by the application when it confirms. It
 * lives in the backup domain so a crash loop costs no flash erase cycles, and
 * is lost on power-on — which is correct, since a power cycle is a fresh start.
 * After BOOT_ATTEMPT_LIMIT consecutive unconfirmed boots the bootloader
 * declares the image bad and opens the long recovery window.
 */
typedef struct {
    uint32_t magic;             /* BOOT_CONFIG_MAGIC */
    uint32_t boot_mode;         /* BOOT_MODE_APP or BOOT_MODE_OTA */
    uint32_t fw_version;        /* currently installed firmware version */
    uint32_t pending_version;   /* version being delivered via OTA */
    uint32_t image_size;        /* size of current firmware image in bytes */
    uint32_t image_crc32;       /* CRC-32 of current firmware image */
    uint32_t update_status;     /* UPDATE_STATUS_* */
    uint32_t config_seq;        /* ping-pong sequence; higher wins */
    uint32_t boot_confirmed;    /* 1 after the app proved it can run */
    uint32_t boot_attempts;     /* consecutive unconfirmed boots */
    uint32_t reserved[3];       /* future use */
    uint32_t cfg_crc32;         /* CRC-32 of all preceding fields */
} BootConfig_t;
/* 中文：这是整个掉电恢复机制的「账本」，56 字节，存在 Flash 里。
 * 逐字段：
 *   magic           固定 0x424F4F54（'BOOT'）。用来判断"这一页里到底有没有配置"。
 *   boot_mode       下次上电该跑谁：BOOT_MODE_APP（正常跳应用）
 *                   还是 BOOT_MODE_OTA（应用请求了升级，开 OTA 窗口）。
 *   fw_version      当前已安装的固件版本号（就是 VERSION 命令回的那个数）。
 *   pending_version 正在通过 OTA 传的版本号。
 *   image_size      ★关键★ 当前镜像的字节数。传输"开始之前"就写进去了。
 *                   作用：bootloader 上电时用这个长度重算 app 区 CRC，
 *                   从而识别出"传了一半"的镜像，不跳进必然崩溃的固件。
 *   image_crc32     ★关键★ 提交镜像时记录的 CRC-32，同上，用于事后复核。
 *   update_status   这次升级进行到哪一步了。
 *   config_seq      ★关键★ ping-pong 的裁决者：两份配置都有效时，序号大的赢。
 *   boot_confirmed  应用是否已经证明过自己能跑。新提交的镜像是 0，
 *                   应用启动 3 秒后自己置 1。bootloader 靠它判断"这个
 *                   镜像到底能不能用"。
 *   boot_attempts   连续多少次上电没报平安（持久化的备份值，实时值在 BKP 寄存器）。
 *   reserved[3]     预留。★注意：结构体尺寸被 _Static_assert 钉死为 56 字节，
 *                   所以加字段只能往 reserved 里塞或追加在 cfg_crc32 之前，
 *                   不能随便插在中间（会让所有已发布的固件读错）。
 *   cfg_crc32       前 13 个字段的 CRC-32（即"除自己以外"的全部内容）。
 *                   读的时候先验它，验不过就当这一份不存在。 */

/**
 * @brief Fixed-point application state returned to the ESP32.
 *
 * Multi-byte values use the native little-endian representation shared by
 * the current STM32F1 and ESP32 targets. Validity and actuator state live in
 * flags so the payload stays compact and does not require software floats.
 */
typedef struct {
    uint32_t uptime_ms;
    int16_t  temperature_centi_c;
    uint16_t humidity_centi_percent;
    uint32_t pressure_pa;
    uint16_t light_lux;
    uint16_t flags;
    uint8_t  led_brightness;    /* raw WS2812B channel value, 0..255 */
    uint8_t  led_percent;       /* 0..100 relative to configured max */
} SensorSnapshot_t;
/* 中文：18 字节的「设备当前状态快照」，ESP32 每秒问一次，喂给网页。
 * 逐字段（注意全部是整数，没有 float —— F103 没有浮点单元）：
 *   uptime_ms               开机至今毫秒数
 *   temperature_centi_c     温度 ×100（2630 = 26.30 ℃）—— 用"厘"做单位避免小数
 *   humidity_centi_percent  湿度 ×100（6100 = 61.00 %RH）
 *   pressure_pa             气压，单位帕（101240 = 1012.40 hPa）
 *   light_lux               光照度，整数 lux
 *   flags                   16 位打包的布尔量，见 SENSOR_FLAG_* 宏
 *   led_brightness          WS2812B 原始通道值 0..255
 *   led_percent             灯带亮度百分比 0..100
 * 18 = 4+2+2+4+2+2+1+1，正好没有对齐填充。 */

/* SensorSnapshot_t.flags 的位定义：一个 16 位整数装 10 个布尔值。
 * 为什么不用 10 个 bool：每个 bool 在 C 里也占 1 字节，10 个就是 10 字节，
 * 而这里总共只花 2 字节。嵌入式里这种"位打包"是常规操作。 */

/**
 * @brief Cached runtime observability state returned to the ESP32.
 *
 * Sampled by vMonitorTask (every DIAG_SNAPSHOT_UPDATE_MS) and copied out on
 * CMD_DIAG_SNAPSHOT, so answering a request never walks the task lists.
 * Stack figures are FreeRTOS "high water mark" values: the smallest number of
 * *words* that ever remained free on that task's stack, so a smaller number
 * means less headroom. 0xFFFF means "never sampled / task not created".
 *
 * Little-endian like SensorSnapshot_t; no floats.
 */
typedef struct {
    uint32_t uptime_ms;                 /* since scheduler start */
    uint32_t free_heap_bytes;           /* xPortGetFreeHeapSize() */
    uint32_t min_ever_free_heap_bytes;  /* xPortGetMinimumEverFreeHeapSize() */
    uint16_t stack_high_water[DIAG_TASK_COUNT]; /* words left, DIAG_TASK_IDX_* */
    uint8_t  task_count;                /* entries valid in stack_high_water */
    uint8_t  sample_seq;                /* +1 per sample; detects a stale cache */
} DiagSnapshot_t;
/* 中文：22 字节的运行诊断快照，回答"这块芯片还剩多少余量"。
 *   free_heap_bytes           当前剩余 FreeRTOS 堆
 *   min_ever_free_heap_bytes  历史最小剩余堆（启动至今）—— 比"当前值"更有意义，
 *                             因为峰值过后当前值会立刻回落，看不出曾经吃紧
 *   stack_high_water[4]       四个任务各自的栈「历史最小剩余」（单位：字，1 字 = 4 字节）
 *                             ★数字越小越危险★。0xFFFF 表示该任务没创建
 *   sample_seq                采样序号。若两次读取它没变，说明 vMonitorTask
 *                             卡住了 —— 此时里面的 heap/栈数字都是过期值
 * 采样和应答是解耦的：vMonitorTask 每 5 秒采一次存进缓存，收到命令只做一次
 * 22 字节拷贝。为什么不即采即答：遍历任务列表要临时数组，在 97% RAM 占用
 * 下不可接受。详见 docs/observability.md。 */

/* Statically verify sizes */
/* 中文：把三个结构体的尺寸焊死在编译期。
 * 这是"结构体直传"这个方案的保险丝：任何一次改动（加字段、换编译器、
 * 改对齐设置、换芯片）只要让布局变了，编译立刻失败，而不是等到真机上
 * 两端字段错位、读到一堆垃圾数。 */
#if defined(__cplusplus)
static_assert(sizeof(BootConfig_t) == 56, "BootConfig_t size mismatch");
static_assert(sizeof(SensorSnapshot_t) == 18,
              "SensorSnapshot_t size mismatch");
static_assert(sizeof(DiagSnapshot_t) == 22, "DiagSnapshot_t size mismatch");
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(BootConfig_t) == 56, "BootConfig_t size mismatch");
_Static_assert(sizeof(SensorSnapshot_t) == 18,
               "SensorSnapshot_t size mismatch");
_Static_assert(sizeof(DiagSnapshot_t) == 22,
               "DiagSnapshot_t size mismatch");
#endif
/* 中文：为什么这里写了两份（static_assert / _Static_assert）：
 * 这个头文件要被 C（STM32 固件、PC 主机测试）和 C++（ESP32 的 main.cpp）
 * 同时包含，两种语言的静态断言关键字不一样，所以按语言分支写。 */

#pragma pack(pop)

/*---------------------------------------------------------------------------
 * CRC-32
 *---------------------------------------------------------------------------*/
/* 中文：CRC 是什么、这里用的是哪一种
 *
 * CRC 是"循环冗余校验"：把一段数据算成一个固定长度的数字（这里 32 位）。
 * 发之前算一次、收到再算一次，两边不一致就说明数据在路上被改坏了。
 * 它跟"求和校验"的区别：CRC 能检出字节顺序被调换、连续多个 bit 翻转这类
 * 错误，检出率高得多，而计算量远小于哈希（MD5/SHA）。
 *
 * ★ "CRC-32" 不是一种算法，是一族。 ★ 参数不同结果完全不同：
 *
 *   变体                              特点                          "123456789" 的结果
 *   CRC-32/ISO-HDLC（本项目用的）     反射，init/xorout 全 0xFF      0xCBF43926
 *   CRC-32/BZIP2                     不反射，init/xorout 全 0xFF    0xFC891918
 *   CRC-32/MPEG-2                    不反射，xorout = 0             0x0376E6E7
 *   CRC-32C / Castagnoli             换多项式 0x1EDC6F41            0xE3069283
 *
 * 本项目用的是最通用的那个 —— Ethernet / zlib / PKZIP 都用它，所以它常被
 * 直接叫作"标准 CRC-32"。上面的英文注释写的也是它：
 *   polynomial 0x04C11DB7（正常形式）= 0xEDB88320（反射形式）
 *   init = 0xFFFFFFFF, final xor = 0xFFFFFFFF, reflected in/out
 *
 * 面试里如果只答"CRC-32"，正确的答法是先说明是哪个变体、参数是什么。
 *
 * 为什么自己写而不用现成库：这段代码要在 STM32 bootloader、STM32 应用、
 * ESP32、PC 四处跑，必须逐位一致；而 STM32F103 上没有 zlib，也没有硬件
 * CRC 外设（F4/F7 才有）。实现只有 20 行 + 1KB 查表，不构成负担。
 * 反过来说：如果芯片有 CRC 外设、且只有一端算，就应该用硬件。 */

/**
 * @brief Compute CRC-32 (Ethernet polynomial 0x04C11DB7).
 *
 * init=0xFFFFFFFF, final xor=0xFFFFFFFF, reflected in/out.
 * This implementation is shared across STM32, ESP32, and PC tools.
 *
 * @param data   Input buffer.
 * @param len    Length in bytes.
 * @param crc    Running finalized CRC (use 0 for the first block, then feed
 *                each returned value into the next call).
 * @return       Standard IEEE CRC-32 (polynomial 0xEDB88320) for all data
 *                processed so far.
 */
uint32_t proto_crc32(const uint8_t *data, size_t len, uint32_t crc);
/* 中文：注意 crc 参数支持"接着上次继续算"——传 0 表示从头开始，
 * 传上一次的返回值表示把新的一段接在后面。OTA 校验整镜像时就是这么
 * 一块一块累加出来的，不用把 39KB 全读进内存。 */

/**
 * @brief One-shot CRC-32 over a buffer.
 */
static inline uint32_t proto_crc32_buf(const uint8_t *data, size_t len) {
    return proto_crc32(data, len, 0U);
}
/* 中文：一次性算完的便捷包装，绝大多数场景用这个。 */

/*---------------------------------------------------------------------------
 * Frame parser (re-entrant, no dynamic allocation)
 *---------------------------------------------------------------------------*/
/* 中文：解析器 = 一台"逐字节推进的状态机"
 *
 * 为什么需要状态机：串口给你的是一串字节，**没有"包"的概念**。
 * 你无法"收一整帧再解析"，因为不知道该收多少 —— 除非先解析出长度字段。
 * 所以做法是：来一个字节，喂进去，状态往前走一格；走到最后一步才发现
 * "哦，这是一帧完整的、CRC 也对的数据"。
 *
 * 这也解释了为什么返回值 NULL 不代表出错：绝大多数时候 NULL 只是
 * "还没凑齐一帧"。真正出错（CRC 不对）时也是返回 NULL，且帧被静默丢弃 ——
 * 可靠性不在这里保证，而在上层的"序号 ACK + 超时重传"。
 *
 * 九个状态的推进顺序（正常帧）：
 *   SYNC   → 等待 0xA5。不是 0xA5 就丢掉继续等（这就是"重新同步"）
 *   CMD    → 收命令字
 *   LEN_LO → 收长度低字节
 *   LEN_HI → 收长度高字节；★在这里就检查长度是否合法★，越界立即回到 SYNC
 *   DATA   → 收 payload，收够 len 个字节才前进
 *   CRC0..CRC3 → 收 4 字节 CRC（小端），收完做比对
 *   ↑ 比对通过才返回帧指针；不通过就静默丢弃，回到 SYNC
 *
 * 为什么在 LEN 阶段就拒绝超长：如果等收完再判断，中间这段就已经把
 * 1028 字节的 payload 数组写越界了。先验证、后写入 —— 这是防缓冲区溢出的
 * 标准顺序。tools/protocol_boundary_test.c 里有专门用 0xFFFF 长度做攻击的用例。 */

/**
 * @brief Frame parser states.
 */
typedef enum {
    FRAME_STATE_SYNC = 0,
    FRAME_STATE_CMD,
    FRAME_STATE_LEN_LO,
    FRAME_STATE_LEN_HI,
    FRAME_STATE_DATA,
    FRAME_STATE_CRC0,
    FRAME_STATE_CRC1,
    FRAME_STATE_CRC2,
    FRAME_STATE_CRC3
} FrameState_t;

/**
 * @brief Decoded frame passed to the caller.
 */
typedef struct {
    uint8_t  cmd;                       /* command byte */
    uint16_t len;                       /* payload length */
    uint8_t  payload[PROTO_MAX_PAYLOAD];/* payload data */
} ProtoFrame_t;
/* 中文：★RAM 成本提醒★ 这个结构体含一个 1028 字节的数组，所以
 * sizeof(ProtoFrame_t) = 1032 字节。下一行的 ProtoParser_t 里又内嵌了一个，
 * 于是 sizeof(ProtoParser_t) = 1048 字节 —— 在只有 20KB RAM 的 F103 上，
 * 一个解析器实例就占掉 5%。bootloader 和 app 各有一个全局实例。
 * 这是"协议用定长数组而不是动态分配"的代价，也是嵌入式里必须算清的账。 */

/**
 * @brief Frame parser context (opaque; caller allocates as static/global).
 */
typedef struct {
    FrameState_t state;
    uint16_t     payload_idx;
    uint32_t     rx_crc;
    uint32_t     calc_crc;
    ProtoFrame_t frame;
} ProtoParser_t;
/* 中文：解析器的全部状态。注意它没有任何动态分配 —— 调用方给一块静态
 * 内存就能跑，这是嵌入式协议栈的常规要求（堆碎片不可接受）。
 *   state       当前走到哪一步
 *   payload_idx 已经收了多少字节 payload
 *   rx_crc      从帧里读出来的 CRC（收在最后 4 字节）
 *   calc_crc    自己算出来的 CRC（边收边算）
 *   frame       解析结果，也就是 proto_parser_feed 返回的那个指针指向的地方
 * 所谓"re-entrant"（可重入）指：可以有多个独立实例互不干扰，不是全局变量。 */

/**
 * @brief Initialize (or reset) a parser instance.
 */
void proto_parser_init(ProtoParser_t *p);
/* 中文：清空状态。每次开始收新帧前、或者判定失步时调用。
 * 注意 feed() 内部收到 0xA5 时也会自己调一次它 —— 这就是"重新同步"。 */

/**
 * @brief Feed one byte into the parser.
 *
 * @param p    Parser instance.
 * @param byte Incoming byte.
 * @return     Pointer to completed frame on success, NULL otherwise.
 *             The pointer is valid until the next proto_parser_feed() call.
 */
const ProtoFrame_t *proto_parser_feed(ProtoParser_t *p, uint8_t byte);
/* 中文：★最后那句注释是本项目最容易被误用的接口约定★
 * 返回的指针指向 p->frame，也就是解析器自己的内存。所以：
 *   · 想看帧内容，必须在下一次调用 feed() 之前拷走或处理完；
 *   · 不能把它存起来，等以后再读 —— 那时内容已经被下一帧覆盖了。
 * 调用范例（application/Core/Src/main.c 的 vCommTask）：
 *     if (f != NULL) cmd_handler_dispatch(f);   // 立刻用掉，不保存
 * 主机测试里的写法是"记住见过的最后一个非 NULL 指针"，只在循环结束后用，
 * 中间不再喂新的字节 —— 这样才安全。 */

/*---------------------------------------------------------------------------
 * Frame building (sender side)
 *---------------------------------------------------------------------------*/
/* 中文：组帧 = 解析的反向操作：把 cmd + payload 拼成上面那个
 * [0xA5][CMD][LEN_L][LEN_H][payload][CRC32] 字节序列。
 * 发送侧走的就是这条路（bootloader、应用、ESP32、PC 工具都在用）。 */

/**
 * @brief Build a protocol frame into a caller-provided buffer.
 *
 * @param buf       Output buffer (must be at least PROTO_MAX_FRAME bytes).
 * @param buf_size  Size of output buffer.
 * @param cmd       Command byte.
 * @param payload   Payload data (may be NULL if len == 0).
 * @param len       Payload length.
 * @return          Number of bytes written to buf, or 0 on error.
 */
uint16_t proto_build_frame(uint8_t *buf, uint16_t buf_size,
                           uint8_t cmd, const uint8_t *payload, uint16_t len);
/* 中文：★返回值语义要记住：成功返回"写入了多少字节"，失败返回 0★
 * 返回 0 只有两种原因：payload 超过 1028 字节，或者目标缓冲区装不下。
 * 调用方必须检查 —— 不检查就等于把"没发出去"当成了"发出去了"。
 * 缓冲区由调用方提供（而不是内部静态分配），这样同一份代码在 PC 上
 * 可以用栈上的大数组，在 MCU 上可以用固定全局数组，各取所需。 */

#ifdef __cplusplus
}
#endif

#endif /* SHARED_PROTOCOL_H */
