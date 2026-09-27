/**
 * @file    ota_config.h
 * @brief   Flash-based OTA configuration read/write utilities.
 *
 * Shared by bootloader and application. The config lives in the last two flash
 * pages (62 and 63). It is stored as two identical copies, one per page,
 * written alternately — see ota_config.c for why a single page cannot hold two
 * atomic copies. On STM32F103 flash writes are halfword-at-a-time and a page
 * must be erased (to 0xFF) before it can be programmed.
 *
 * ===========================================================================
 * 中文导读：这是「跨复位、跨掉电的状态存档」接口
 * ===========================================================================
 * 解决的问题：STM32 复位后 RAM 全部清空，那"我刚才是要升级吗""上次那个
 * 固件到底能不能跑"这类信息存哪？答案：存进 Flash 最后两页（62/63 页），
 * 由 bootloader 和 application 两个固件共同读写。
 *
 * 注意它的"共享"范围和 protocol.h 不一样：
 *   protocol.h   编进 4 个目标（两个 STM32 固件 + ESP32 + PC 主机测试）
 *   ota_config   只编进 2 个 STM32 固件
 * 因为本文件调用 STM32 HAL 的 Flash 接口，ESP32 和 PC 上根本没有。
 * esp32-comm-bridge/platformio.ini 里明确写着 "ota_config.c is intentionally
 * NOT built on ESP32"。所以 shared 目录里其实混着两种可移植性不同的东西，
 * 读代码时要分清。
 *
 * 三条硬约束（来自 STM32F103 的物理特性，不是设计偏好）：
 *   ① 只能整页擦除（1KB），不能只改 56 个字节
 *   ② 写入按"半字"（2 字节）进行，不能按字节写
 *   ③ 擦写时代码不能从 Flash 取指 → 擦写函数必须放 RAM（.ramfunc）
 * ===========================================================================
 */

#ifndef SHARED_OTA_CONFIG_H
#define SHARED_OTA_CONFIG_H

#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * API
 *---------------------------------------------------------------------------*/
/* 中文：对外一共 9 个函数，按用途分三组 ——
 *   ① 基础读写：read / write / prepare
 *   ② OTA 状态迁移：request_update / mark_valid / mark_failed
 *      （注意这三个不是简单 setter，内部都是"读出整个结构体 → 改字段 →
 *        写回去"，因为它们要保留其它字段的现值）
 *   ③ 启动确认：record_boot_attempt / clear_attempts / confirm_boot
 *      （跨复位的"报平安"握手，见文件末尾）
 * 调用方只有 bootloader 和 application，没有第三方。 */

/**
 * @brief Read the newest valid boot configuration.
 *
 * Reads both copies, validates magic + internal CRC on each, and returns the
 * one with the higher config_seq. A copy that fails validation is ignored, so
 * a power loss during a write cannot lose the previous configuration.
 *
 * @param cfg  Pointer to caller-allocated BootConfig_t.
 * @return     true if at least one valid copy exists.
 */
bool ota_config_read(BootConfig_t *cfg);
/* 中文：★所有读取的唯一入口，自动选"较新的那份有效配置"★
 * 逻辑：两份都读出来，各自验 magic + 内部 CRC，然后比 config_seq，大的赢。
 *
 * 返回 false 表示两份都不合法（从没写过配置，或配置区被擦坏）。
 *
 * 关键性质：写入过程中掉电 → 一定是"新的那份不合法、旧的那份完好"，
 * 所以这个函数在掉电后依然能返回一个可用的旧配置，而不是失败。 */

/**
 * @brief Read state and write the next config into the alternate page.
 *
 * Never erases the page holding the newest valid copy, so the write is
 * atomic with respect to power loss. Blocks during erase (~40ms/page on F103).
 * Call from a task, not an ISR.
 *
 * On success `cfg` receives the sequence number that was written.
 *
 * @param cfg  Config to write; magic and cfg_crc32 are set by this call.
 * @return     true on success.
 */
bool ota_config_write(BootConfig_t *cfg);
/* 中文：★三句必须记住的话★
 * ① "Never erases the page holding the newest valid copy"
 *    —— 写新版时永远擦"不是最新那份"所在的那一页，所以任何瞬间都有一份
 *    完整的旧配置躺在那里。这就是"掉电原子性"的全部来源。
 * ② "Blocks during erase (~40ms/page on F103). Call from a task, not an ISR."
 *    —— 擦一页要 40 毫秒，中断里绝对不能干这个。函数内部也不加锁，
 *    调用方要保证不会有两个任务同时写。
 * ③ 函数会顺便把 cfg->magic 和 cfg->cfg_crc32 填好，调用方不用管。 */

/**
 * @brief Prepare a BootConfig_t for writing: set magic and compute CRC.
 */
void ota_config_prepare(BootConfig_t *cfg);
/* 中文：只做两件事——填 magic、算 cfg_crc32。write() 内部会调用它，
 * 一般不需要单独调用。 */

/**
 * @brief Mark OTA request: sets boot_mode=BOOT_MODE_OTA and writes.
 *
 * Application calls this before NVIC_SystemReset().
 *
 * @param pending_version  Firmware version expected from OTA.
 * @param image_size       Expected image size in bytes (0 = unknown).
 * @return                 true on success.
 */
bool ota_config_request_update(uint32_t pending_version, uint32_t image_size);
/* 中文：★应用侧触发升级的唯一动作★
 * 应用写完这个，紧接着就 NVIC_SystemReset()。上电后 bootloader 读到
 * BOOT_MODE_OTA，于是打开 OTA 握手窗口，而不是直接跳回应用。
 *
 * image_size 这个参数很关键：它是"我期望收到多大的固件"，在传输**开始之前**
 * 就落盘了。有了它，bootloader 才能在传输中途掉电后重算 app 区 CRC，
 * 识别出"传了一半"的镜像，从而拒绝跳进必然崩溃的固件。 */

/**
 * @brief Mark the current firmware as valid (bootloader calls on OTA success).
 *
 * @param version     New firmware version.
 * @param image_size  Image size in bytes (>0 enables boot-time CRC checking).
 * @param image_crc   CRC-32 of the written image.
 * @return            true on success.
 */
bool ota_config_mark_valid(uint32_t version, uint32_t image_size, uint32_t image_crc);
/* 中文：bootloader 在整镜像 CRC 校验通过后调用。
 * 注意它同时把 boot_confirmed 清成 0 —— 意思是"这个镜像字节上是对的，
 * 但它还没证明自己能跑"。要等应用起来后自己调 confirm_boot() 才算数。 */

/**
 * @brief Mark the update as failed (bootloader calls on error).
 */
bool ota_config_mark_failed(void);
/* 中文：升级失败时记一笔，供事后诊断"上次升级为什么失败"。 */

/*---------------------------------------------------------------------------
 * Boot verification (cross-reset handshake)
 *
 * bootloader: record_boot_attempt() on every boot, clear_attempts() once the
 *             application is known good or a fresh image was committed.
 * application: confirm_boot() once it is demonstrably healthy.
 * ---------------------------------------------------------------------------*/
/* 中文：★这一段是"这个固件到底能不能跑"的判定机制，也是自动回滚的基础★
 *
 * 完整握手时序（一次上电）：
 *   ① bootloader 每次上电都调 record_boot_attempt()，把计数 +1
 *   ② 应用如果成功起来了，启动 3 秒后调 confirm_boot()，把计数清零
 *   ③ 如果应用在启动阶段就崩（看门狗复位），它永远走不到第②步，计数一直涨；
 *      涨到 BOOT_ATTEMPT_LIMIT(3) 次，bootloader 就判定"这个镜像就算字节
 *      全对，也是个跑不起来的坏镜像"，转而打开 10 秒恢复窗口等主机重推。
 *
 * 为什么实时计数放 BKP 备份寄存器而不是 Flash：
 *   崩溃循环会反复复位，每次都写 Flash 的话，配置页在约 1 万次擦写后报废
 *   —— 而崩溃循环恰恰是最容易触发这种写法的场景。
 *   BKP 由复位域保持（看门狗 / HardFault / NVIC_SystemReset 都不清），
 *   所以计数零成本。代价是掉电会丢 —— 但掉电本来就是全新开始，正好。 */

/**
 * @brief Note that this boot has started and report how many consecutive boots
 *        have failed to confirm so far.
 *
 * Persisted in the RTC backup registers rather than flash, so it survives a
 * watchdog reset without costing flash erase cycles; the durable count lives in
 * BootConfig_t.boot_attempts and is only written when the count crosses a
 * threshold or is reset.
 *
 * @param out_attempts  Receives the attempt count for this boot (1-based).
 * @return              true if out_attempts was filled.
 */
bool ota_config_record_boot_attempt(uint32_t *out_attempts);
/* 中文：bootloader 每次上电做的第一件事。
 * out_attempts 给出"这是连续第几次没能确认的启动"（从 1 开始）。
 * ★它本身不写 Flash★ —— 只动 BKP 寄存器，所以崩溃循环不会磨损 Flash。 */

/**
 * @brief Reset the consecutive failed-boot counter.
 *
 * Called by the bootloader (after a successful commit or on a confirmed boot)
 * and by the application (via ota_config_confirm_boot()).
 */
bool ota_config_clear_attempts(void);
/* 中文：把计数清零（BKP 里的实时值和 Flash 里的持久值都清）。 */

/**
 * @brief Application-side boot confirmation.
 *
 * Sets boot_confirmed=1 and clears the failed-boot counter. Until this runs,
 * the bootloader treats the next reset as evidence the image is bad.
 */
bool ota_config_confirm_boot(void);
/* 中文：★应用侧的"报平安"★ 由 vControlTask 在上电 3 秒后调用。
 * 3 秒这个延迟是刻意的：如果固件在初始化阶段（传感器、UI、驱动）就死，
 * 它根本走不到这一行，于是计数继续累积，最终被判为坏镜像。
 * 内部有个小优化：如果已经确认过、计数本来就是 0，就直接返回不写 Flash，
 * 避免每次上电白耗一次擦写寿命。 */

#ifdef __cplusplus
}
#endif

#endif /* SHARED_OTA_CONFIG_H */
