/**
 * @file    uart_comm.c
 * @brief   UART communication driver implementation.
 *
 * USART1 (PA9=TX → ESP32 RX, PA10=RX ← ESP32 TX).
 *
 * RX path:
 *   USART1_RX --DMA1_Channel5 (circular)--> g_dma_rx_buf[UART_RX_BUF_SIZE]
 *   USART IDLE interrupt  --> uart_comm_drain_rx() --> StreamBuffer
 *   vCommTask             <-- xStreamBufferReceive() <-- parser
 *
 * Why DMA + IDLE instead of a per-byte RXNE interrupt: at 115200 baud a
 * continuous stream generates ~11,520 interrupts/second, each one moving a
 * single byte. With DMA the byte movement costs no CPU at all, and a burst
 * (typically one whole protocol frame) costs exactly one interrupt.
 *
 * The ISR does not parse and does not touch the parser: it only computes how
 * far the DMA head advanced and copies that span into the stream buffer.
 *
 * TX remains blocking. At these data rates the TX path is not the bottleneck,
 * and OTA chunks are paced by the peer's per-chunk ACK anyway.
 */

#include "uart_comm.h"
#include "stm32f1xx_hal.h"

/*---------------------------------------------------------------------------
 * Static data
 *---------------------------------------------------------------------------*/

static UART_HandleTypeDef    g_huart;
static DMA_HandleTypeDef     g_hdma_usart_rx;
static StreamBufferHandle_t  g_rx_stream;
static StaticStreamBuffer_t  g_rx_stream_struct;
static uint8_t               g_rx_stream_buf[UART_RX_STREAM_SIZE + 1];

/* Circular DMA destination. The DMA writes here forever; `g_dma_tail` tracks
 * how much of it the ISR has already published to the stream buffer. */
static uint8_t               g_dma_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t     g_dma_tail;

/*---------------------------------------------------------------------------
 * Initialization
 *---------------------------------------------------------------------------*/

void uart_comm_init(uint32_t baud_rate) {
    g_rx_stream = xStreamBufferCreateStatic(
        UART_RX_STREAM_SIZE,
        1,                          /* Trigger level: 1 byte */
        g_rx_stream_buf,
        &g_rx_stream_struct
    );

    g_dma_tail = 0U;

    /* USART1 */
    g_huart.Instance          = USART1;
    g_huart.Init.BaudRate     = baud_rate;
    g_huart.Init.WordLength   = UART_WORDLENGTH_8B;
    g_huart.Init.StopBits     = UART_STOPBITS_1;
    g_huart.Init.Parity       = UART_PARITY_NONE;
    g_huart.Init.Mode         = UART_MODE_TX_RX;
    g_huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    g_huart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&g_huart);

    HAL_UART_Receive_DMA(&g_huart, g_dma_rx_buf, UART_RX_BUF_SIZE);

    /* IDLE shares the USART1 vector. It must be at or below the FreeRTOS
     * syscall threshold because the drain path calls xStreamBufferSendFromISR.
     * Clearing IDLE first also drops any stale flag raised before init. */
    __HAL_UART_CLEAR_IDLEFLAG(&g_huart);
    HAL_NVIC_SetPriority(USART1_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(&g_huart, UART_IT_IDLE);
}

StreamBufferHandle_t uart_comm_get_rx_stream(void) {
    return g_rx_stream;
}

/*---------------------------------------------------------------------------
 * RX drain (DMA head → stream buffer)
 *---------------------------------------------------------------------------*/

void uart_comm_drain_rx(void) {
    /* How many bytes has the DMA written since the last drain? The channel
     * counter starts at UART_RX_BUF_SIZE and counts down to 0 as it writes,
     * then reloads; so SIZE - counter is exactly the head index in
     * 0..SIZE-1. Both head and g_dma_tail live in that same range, which is
     * all the wrap-around logic below needs — the old comment here claimed a
     * power-of-two size was required, which was never true. */
    uint16_t head = (uint16_t)(UART_RX_BUF_SIZE
                               - __HAL_DMA_GET_COUNTER(&g_hdma_usart_rx));

    if (head == g_dma_tail) {
        return;
    }

    if (head > g_dma_tail) {
        (void)xStreamBufferSendFromISR(g_rx_stream, &g_dma_rx_buf[g_dma_tail],
                                       (size_t)(head - g_dma_tail), NULL);
    } else {
        /* Wrapped past the end of the buffer: publish the tail of the buffer
         * first, then the head of it. */
        (void)xStreamBufferSendFromISR(g_rx_stream, &g_dma_rx_buf[g_dma_tail],
                                       (size_t)(UART_RX_BUF_SIZE - g_dma_tail), NULL);
        (void)xStreamBufferSendFromISR(g_rx_stream, &g_dma_rx_buf[0],
                                       (size_t)head, NULL);
    }

    g_dma_tail = head;
}

/*---------------------------------------------------------------------------
 * TX (blocking)
 *---------------------------------------------------------------------------*/

void uart_comm_send_byte(uint8_t byte) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = byte;
}

void uart_comm_send(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uart_comm_send_byte(data[i]);
    }
}

/*---------------------------------------------------------------------------
 * USART1 interrupt handler
 *
 * Handles exactly one thing: "a burst ended" (IDLE). Errors are cleared so a
 * noise-induced ORE/FE/NE/PE cannot wedge the receiver; the protocol's CRC
 * and sequence checks are what actually recover the stream.
 *---------------------------------------------------------------------------*/

void USART1_IRQHandler(void) {
    /* IDLE: the line went quiet, so the DMA has a complete burst ready. */
    if (__HAL_UART_GET_FLAG(&g_huart, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&g_huart);
        uart_comm_drain_rx();
    }

    /* Overrun / framing / noise: read DR to clear. A dropped burst is
     * recovered by the protocol layer, not here. */
    if (__HAL_UART_GET_FLAG(&g_huart, UART_FLAG_ORE) != RESET) {
        (void)USART1->DR;
    }
}

/*---------------------------------------------------------------------------
 * HAL MSP callbacks
 *
 * The HAL calls these when the UART needs its GPIO, clocks, and DMA channel.
 *---------------------------------------------------------------------------*/

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) {
        return;
    }

    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* PA9 = TX (Alternate Function Push-Pull) */
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

    /* USART1_RX is hardwired to DMA1 Channel 5 on STM32F1. */
    g_hdma_usart_rx.Instance                 = DMA1_Channel5;
    g_hdma_usart_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    g_hdma_usart_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    g_hdma_usart_rx.Init.MemInc              = DMA_MINC_ENABLE;
    g_hdma_usart_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_hdma_usart_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    g_hdma_usart_rx.Init.Mode                = DMA_CIRCULAR;
    g_hdma_usart_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&g_hdma_usart_rx);

    __HAL_LINKDMA(huart, hdmarx, g_hdma_usart_rx);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) {
        return;
    }

    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

    HAL_DMA_DeInit(huart->hdmarx);
    __HAL_RCC_DMA1_CLK_DISABLE();
}
