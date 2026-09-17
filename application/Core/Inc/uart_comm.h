/**
 * @file    uart_comm.h
 * @brief   UART communication driver for the application.
 *
 * RX uses DMA1 Channel 5 (USART1_RX) in circular mode. The USART IDLE
 * interrupt marks "this burst is complete"; the comm task drains the
 * new bytes into a FreeRTOS stream buffer. Protocol parsing always runs
 * in task context, never in the ISR.
 *
 * Rationale: a per-byte RXNE interrupt at 115200 baud fires ~11.5k times
 * per second, all of it spent entering and leaving an ISR to move one
 * byte. DMA + IDLE reduces that to one interrupt per burst (<= 1 frame).
 */

#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "stream_buffer.h"

/*---------------------------------------------------------------------------
 * Constants
 *---------------------------------------------------------------------------*/

/* Circular DMA landing buffer. Sized to hold one maximum-length protocol
 * frame (1032 B) plus margin. RAM budget on this part is the binding
 * constraint, so this is deliberately the smallest value that cannot split a
 * frame across the ring boundary. At 115200 baud (10 bits/byte) 1152 B is
 * ~100ms of headroom; the IDLE ISR drains it once per burst. */
#define UART_RX_BUF_SIZE        1152U

/* Stream buffer the comm task reads from. Must be >= UART_RX_BUF_SIZE so a
 * full burst is always handed over in one xStreamBufferSendFromISR; a short
 * write would silently drop bytes instead of applying backpressure. */
#define UART_RX_STREAM_SIZE     1152U

/*---------------------------------------------------------------------------
 * API
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialise USART1 (PA9=TX, PA10=RX) at the configured baud rate.
 *        Sets up DMA + IDLE burst reception feeding a stream buffer.
 */
void uart_comm_init(uint32_t baud_rate);

/**
 * @brief Get the stream buffer handle for the comm task to receive from.
 */
StreamBufferHandle_t uart_comm_get_rx_stream(void);

/**
 * @brief Copy bytes received by DMA since the last call into the stream
 *        buffer. Called from the IDLE ISR (and once at task start to pick
 *        up anything that arrived before the task first ran).
 */
void uart_comm_drain_rx(void);

/**
 * @brief Send a raw byte over USART1 (blocking).
 */
void uart_comm_send_byte(uint8_t byte);

/**
 * @brief Send a buffer over USART1 (blocking).
 */
void uart_comm_send(const uint8_t *data, uint16_t len);

#endif /* UART_COMM_H */
