/**
 * @file    bootloader.h
 * @brief   Bootloader main definitions, state machine, and app jump logic.
 *
 * ===========================================================================
 * 中文导读：bootloader 的"类型契约"
 * ===========================================================================
 * 只有两样东西:
 *   ① BootState_t  —— 状态机的 7 个状态(main.c 里用 g_state 变量承载)
 *   ② OtaContext_t —— 一次升级过程中的上下文(要传的信息)
 * 外加 4 个对外函数声明。
 *
 * 注意:这个头文件**不被 application 包含**。bootloader 和 application 是
 * 两个独立编译、互不引用符号的固件,它们之间只通过两样东西通信:
 *   · Flash 里的 config 区(shared/ota_config.h 的 BootConfig_t)
 *   · UART 上的帧协议(shared/protocol.h)
 * 这就是"两个固件"和"一个固件的两个模块"的本质区别 —— 面试可以聊。
 * ===========================================================================
 */

#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include <stdint.h>
#include <stdbool.h>

/*---------------------------------------------------------------------------
 * Bootloader states
 *---------------------------------------------------------------------------*/
/* 中文：状态机的 7 个状态。g_state 在 main.c 里承载它。
 * 主要流转:
 *   ST_BOOT → ST_WAIT_HANDSHAKE → (收到 OTA_BEGIN) → ST_OTA_ACTIVE
 *                                → ST_OTA_VERIFY → ST_OTA_DONE → 跳转
 *                               → (超时且不可启动) → ST_MAINTENANCE
 *                               → (任何严重错误)   → ST_ERROR(停机闪灯)
 * 注意 ST_OTA_DONE 和 ST_ERROR 之后都不会再回到别的状态 —— 要么跳转、
 * 要么复位、要么死循环。 */

typedef enum {
    ST_BOOT = 0,          /* Decide what to do on reset */
    ST_WAIT_HANDSHAKE,    /* Waiting for OTA_BEGIN from ESP32 */
    ST_OTA_ACTIVE,        /* Receiving chunks, erasing + programming flash */
    ST_OTA_VERIFY,        /* Computing CRC-32 over written image */
    ST_OTA_DONE,          /* Config updated, ready to jump */
    ST_ERROR,             /* Non-recoverable error — halt with blink code */
    ST_MAINTENANCE        /* Forced bootloader entry (BOOT0 pin) */
} BootState_t;
/* 中文：★最后那个注释 (BOOT0 pin) 是过期的,别被误导★
 * 早期版本通过读 PB0 引脚来"强制进 bootloader"。但 PB0 同时接的是
 * PIR 人体红外传感器,而它的空闲输出是低电平 → **每次上电都停在 bootloader**。
 * 后来把这个读取删掉了。
 * 现在进入 ST_MAINTENANCE 的唯一路径是:启动判定认为镜像不可用。
 * 硬件兜底是 Blue Pill 板子自己的 BOOT0 跳帽(进 STM32 内置 ROM
 * bootloader),那和这个状态无关。 */

/*---------------------------------------------------------------------------
 * OTA context (passed through the state machine)
 *---------------------------------------------------------------------------*/
/* 中文：一次升级过程中的"行李"。在收到 CMD_OTA_BEGIN 时被填满,
 * 然后在每个 chunk 的处理中累积。
 * 为什么要有 bytes_written 和 expected_seq 两个游标:
 *   expected_seq   —— 协议层:我下一块想收第几号(用于查重、查乱序)
 *   bytes_written  —— 数据层:我实际写进去多少字节(用于最终长度校验)
 * 两者分开,是因为"收到第 N 块"和"累计写入 N KB"在出错时可能不一致,
 * 而最终提交时必须用字节数说话。 */

typedef struct {
    uint32_t image_size;        /* Total image size from OTA_BEGIN */
    uint32_t image_crc32;       /* Expected CRC-32 from OTA_BEGIN */
    uint32_t version;           /* Firmware version from OTA_BEGIN */
    uint32_t expected_seq;      /* Next expected chunk sequence number */
    uint32_t bytes_written;     /* Total bytes programmed so far */
} OtaContext_t;

/*---------------------------------------------------------------------------
 * Bootloader entry
 *---------------------------------------------------------------------------*/

/**
 * @brief Main bootloader loop. Called from main() after HAL init.
 *        Does not return — either jumps to app or halts.
 */
void bootloader_run(void);

/**
 * @brief Jump to the application at APP_BASE.
 *
 * Validates the application stack pointer, sets VTOR, and transfers control.
 * Does NOT return.
 */
void bootloader_jump_to_app(void) __attribute__((noreturn));
/* 中文：★noreturn 不是装饰,是给编译器的重要信息★
 * 它告诉编译器"这个函数不会返回",于是:
 *   · 不会为"返回后继续执行"生成代码
 *   · 调用它的分支后面不需要再 return
 *   · 优化器可以做更好的寄存器分配
 * 如果函数实际会返回但标了 noreturn,行为未定义 —— 所以标注必须诚实。 */

/**
 * @brief Software reset via NVIC.
 */
void bootloader_reset(void) __attribute__((noreturn));
/* 中文：NVIC_SystemReset() —— 软复位。★关键:IWDG 不会被它清零★
 * (只有彻底断电才清)所以复位后看门狗还在跑,新的一轮必须在 4 秒内喂上。 */

/**
 * @brief Halt with an error blink code on the LED.
 *
 * @param code   Blink pattern identifier. Does not return.
 */
void bootloader_halt_error(uint32_t code) __attribute__((noreturn));
/* 中文：用 LED 闪 code 次来报错误码。已知的错误码:
 *   1 = bootloader_run 走到无路可走的兜底分支
 *   3 = 整镜像校验通过,但配置写不进 Flash
 *   4 = 整镜像 CRC 不匹配
 * 这是没有串口时的最后诊断手段 —— 用示波器抓 PC13 数闪烁次数即可。 */

#endif /* BOOTLOADER_H */
