/**
 * @file    cmd_handler.h
 * @brief   Command handler — dispatches received protocol frames.
 */

#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "../../../shared/protocol.h"

/**
 * @brief Handle a received protocol frame.
 *
 * Called from the comm task. Dispatches based on command:
 *   - CMD_OTA_AVAILABLE → notifies control task → triggers OTA
 *   - CMD_APP_MSG → forwards to application task
 *   - CMD_GET_STATUS → responds with current firmware version
 *   - CMD_GET_SENSOR_SNAPSHOT → returns the latest fixed-point app state
 *   - CMD_DIAG_SNAPSHOT → returns the cached stack/heap runtime diagnostics
 *
 * @param f  Validated protocol frame.
 */
void cmd_handler_dispatch(const ProtoFrame_t *f);

/**
 * @brief Build and send a protocol frame over UART.
 */
void cmd_handler_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len);

#endif /* CMD_HANDLER_H */
