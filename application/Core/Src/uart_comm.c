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
 *
 * ===========================================================================
 * 中文导读：这是全项目并发设计最核心的一个文件（207 行）
 * ===========================================================================
 * 它解决的问题：**串口数据来了，怎么在不丢数据的前提下，把它交给任务去处理。**
 *
 * 三层结构，数据从下往上走：
 *
 *   ① 硬件层   USART1 收到字节 → DMA 自动搬进环形缓冲（CPU 完全不参与）
 *   ② 中断层   线路一安静（IDLE），中断把"新到的这一段"交给 StreamBuffer
 *   ③ 任务层   vCommTask 从 StreamBuffer 取字节 → 喂协议解析器
 *
 * ★ 三个关键数字（读这个文件前先记住）★
 *   UART_RX_BUF_SIZE    = 1152 字节  DMA 环形落地缓冲
 *   UART_RX_STREAM_SIZE = 1152 字节  StreamBuffer
 *   PROTO_MAX_FRAME     = 1036 字节  单帧最大长度
 * 两个缓冲都必须 ≥ 单帧最大长度，否则环形回绕会把一帧切成两半。
 * 1152 是"装得下一整帧 + 一点余量"的最小值 —— 因为 RAM 只剩几百字节。
 *
 * ★ 这个文件最该理解的一段 ★
 *   uart_comm_drain_rx() 里的 head/tail 环形指针算法。
 *   它就是"环形缓冲区"这个概念的教科书式实现，面试常问。
 * ===========================================================================
 */

#include "uart_comm.h"
#include "stm32f1xx_hal.h"

/*---------------------------------------------------------------------------
 * Static data
 *---------------------------------------------------------------------------*/
/* 中文：★ 为什么这两块缓冲都要开成 static ★
 *   g_dma_rx_buf   DMA 的落点。DMA 是**硬件**在写，
 *                  地址必须长期有效且固定 —— 不能是函数里的局部变量。
 *   g_rx_stream_buf  StreamBuffer 的存储。因为**中断里**要往它写，
 *                  所以用静态创建（xStreamBufferCreateStatic），
 *                  不依赖堆分配的状态。
 * 注意 g_rx_stream_buf 多开 1 字节（+1）：FreeRTOS 的静态流缓冲
 * 需要的存储比配置长度多 1 字节，这是内核的实现细节。
 *
 * g_dma_tail 为什么是 volatile：它被中断和任务共同访问，
 * 加 volatile 是告诉编译器"这个变量会被外部改变，别缓存到寄存器里"。 */

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
    /* 中文：① 先建 StreamBuffer（静态方式，不用堆）。
     * "触发等级 = 1 字节"的含义：只要缓冲里有 1 个字节，
     * 等待在这个缓冲上的任务就会被唤醒。设大一点可以"攒够 N 个字节再唤醒"，
     * 减少任务切换次数 —— 但这里解析器是逐字节处理的，设 1 最简单。 */
    g_rx_stream = xStreamBufferCreateStatic(
        UART_RX_STREAM_SIZE,
        1,                          /* Trigger level: 1 byte */
        g_rx_stream_buf,
        &g_rx_stream_struct
    );

    g_dma_tail = 0U;

    /* USART1 */
    /* 中文：② 配串口参数：115200 8N1（8 数据位、无校验、1 停止位）。
     * 这三个参数必须和 ESP32 侧完全一致，否则收到的是乱码。 */
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
    /* 中文：③ ★启动 DMA 接收★ 这一句之后，DMA 就开始"永远"把收到的字节
     * 往 g_dma_rx_buf 里写，写满了从头再来（环形模式，在 MspInit 里配的
     * DMA_CIRCULAR）。**全程不需要 CPU 参与，也不需要再调用第二次。**
     *
     * 对比一下如果没有 DMA 会怎样：每收到 1 个字节触发 1 次中断，
     * 115200 波特率下每秒约 11520 次中断。每次中断要保存现场、
     * 读寄存器、存内存、恢复现场 —— CPU 大量时间耗在"搬一个字节"上。
     * 用 DMA 之后：搬字节零 CPU 成本，一整个 burst 只花 1 次中断。 */

    /* IDLE shares the USART1 vector. It must be at or below the FreeRTOS
     * syscall threshold because the drain path calls xStreamBufferSendFromISR.
     * Clearing IDLE first also drops any stale flag raised before init. */
    /* 中文：④ ★★ 这一段是"能不能用 FreeRTOS API"的红线 ★★
     *
     * IDLE 中断的处理函数里会调 xStreamBufferSendFromISR()。
     * FreeRTOS 规定：**凡是调用 FromISR 类 API 的中断，
     * 其 NVIC 优先级数值必须 ≥ configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY**（本项目 = 5）。
     *
     * 原因：FreeRTOS 进临界区时写 BASEPRI = 0x50，只屏蔽优先级数值 ≥ 5 的中断。
     * 优先级 0–4 的中断能"穿透"临界区，在内核正在改就绪链表时打断它
     * → 数据结构损坏 → 随机 HardFault。
     *
     * 这里把 USART1 设成**正好等于 5**：
     *   既满足红线（≥ 5），又是允许范围内最高的优先级（数值越小优先级越高）
     *   → 串口收包响应最快。
     *
     * 先清一次 IDLE 标志：丢掉初始化前可能残留的旧标志，
     * 否则中断会立刻触发一次、发布一段垃圾数据。 */
    __HAL_UART_CLEAR_IDLEFLAG(&g_huart);
    HAL_NVIC_SetPriority(USART1_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(&g_huart, UART_IT_IDLE);
    /* 中文：⑤ 只使能 IDLE 中断，**不使能 RXNE 中断**。
     * 数据搬运交给 DMA，CPU 只在"一整段收完"时被打扰一次。 */
}

StreamBufferHandle_t uart_comm_get_rx_stream(void) {
    return g_rx_stream;
}

/*---------------------------------------------------------------------------
 * RX drain (DMA head → stream buffer)
 *---------------------------------------------------------------------------*/
/* 中文：★★★ 这个函数是整个文件的核心：环形缓冲区的 head/tail 算法 ★★★
 *
 * 【先理解两个指针】
 *   head —— DMA 写到哪了。**由硬件决定**，我们只能读出来。
 *   tail —— 我上次发布到哪了。**由我们记着**（就是 g_dma_tail）。
 *   两者都在 0..SIZE-1 范围内循环。
 *
 * 【head 怎么算出来】
 *   DMA 的 CNDTR 寄存器在环形模式下从 SIZE 往下数到 0，然后自动重装。
 *   所以：head = SIZE - CNDTR
 *     刚开始 CNDTR = 1152 → head = 0
 *     写了 100 字节    → CNDTR = 1052 → head = 100
 *     绕回一圈        → CNDTR = 1152 → head = 0
 *
 * 【怎么知道"新到了多少字节"】看 head 和 tail 的关系：
 *
 *   情况一：head > tail（没绕圈）
 *     ┌──────────────┬─────────┬────────┐
 *     │ 已发布       │ 新数据  │ 未使用 │
 *     └──────────────┴─────────┴────────┘
 *     0            tail       head     SIZE
 *     → 发一段：[tail, head)
 *
 *   情况二：head < tail（DMA 绕回缓冲开头了）
 *     ┌─────────┬──────────────┬────────┐
 *     │ 新数据  │ 已发布       │        │
 *     └─────────┴──────────────┴────────┘
 *     0        head            tail    SIZE
 *     → **发两段**：先 [tail, SIZE)，再 [0, head)
 *     这就是为什么会有两次 SendFromISR —— 不是发了两条消息，
 *     而是把"环绕的一整段"拆成物理上连续的两块来描述。
 *
 * 【为什么这个函数可以在中断里安全调用】
 *   它只用 head（硬件写）和 tail（只有这里改），不做任何解析、
 *   不调阻塞 API，所以执行时间是常数级的、很短的。
 *   真正的处理全在 vCommTask 里。 */

void uart_comm_drain_rx(void) {
    /* How many bytes has the DMA written since the last drain? The channel
     * counter starts at UART_RX_BUF_SIZE and counts down to 0 as it writes,
     * then reloads; so SIZE - counter is exactly the head index in
     * 0..SIZE-1. Both head and g_dma_tail live in that same range, which is
     * all the wrap-around logic below needs — the old comment here claimed a
     * power-of-two size was required, which was never true. */
    /* 中文：★末尾那句注释是一个已纠正的错误认知★
     * 曾经以为缓冲区大小必须是 2 的幂（很多环形缓冲实现确实要求这个，
     * 为了能用位与代替取模）。但这里根本不做取模，只比大小，
     * 所以 1152 这种非 2 的幂完全没问题。 */
    uint16_t head = (uint16_t)(UART_RX_BUF_SIZE
                               - __HAL_DMA_GET_COUNTER(&g_hdma_usart_rx));

    if (head == g_dma_tail) {
        return;   /* 中文：没有新数据。IDLE 中断可能在没收到数据时也触发。 */
    }

    if (head > g_dma_tail) {
        /* 中文：情况一：没绕圈，发一段。 */
        (void)xStreamBufferSendFromISR(g_rx_stream, &g_dma_rx_buf[g_dma_tail],
                                       (size_t)(head - g_dma_tail), NULL);
    } else {
        /* Wrapped past the end of the buffer: publish the tail of the buffer
         * first, then the head of it. */
        /* 中文：情况二：绕圈了，发两段。注意顺序 —— 必须先发靠近尾部的那段
         * （时间上更早到达），再发开头那段。顺序反了数据就乱了。 */
        (void)xStreamBufferSendFromISR(g_rx_stream, &g_dma_rx_buf[g_dma_tail],
                                       (size_t)(UART_RX_BUF_SIZE - g_dma_tail), NULL);
        (void)xStreamBufferSendFromISR(g_rx_stream, &g_dma_rx_buf[0],
                                       (size_t)head, NULL);
    }

    g_dma_tail = head;
    /* 中文：★最后才更新 tail，而且必须放在所有发送之后★
     * 如果先更新 tail 再发送，而发送途中又来了一个中断，就会漏数据。
     * "先干活，最后再推进游标"是这类算法的通用要求。 */
}
/* 中文：★ 一个容易忽略的点：这里没有加锁 ★
 * 因为调用者只有中断（USART1_IRQHandler）和任务启动时那一次 drain。
 * StreamBuffer 的 FromISR 版本本身就是中断安全的。
 * 但这也意味着：**如果将来有人从任务里高频调用它，就要重新考虑并发。** */

/*---------------------------------------------------------------------------
 * TX (blocking)
 *---------------------------------------------------------------------------*/
/* 中文：发送侧刻意**保持阻塞**（不搞 DMA）。
 * 理由（文件头注释里也写了）：
 *   ① 在这个速率下发送不是瓶颈 —— 一帧最多 1036 字节，
 *      发完约 90ms，而 OTA 本来就被对端"逐块 ACK"卡着节奏。
 *   ② 阻塞代码简单到不会错。DMA 发送要处理"发完了"中断、
 *      要防止两次发送撞车，复杂度换来的是这里根本不需要的性能。
 * ★ 这是一个"知道可以优化但选择不优化"的典型判断 —— 面试可以讲。 */

void uart_comm_send_byte(uint8_t byte) {
    while (!(USART1->SR & USART_SR_TXE));
    /* 中文：等 TXE（发送数据寄存器空）标志。这是直接读寄存器，
     * 没走 HAL —— 因为就这一行，包一层没有意义。 */
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
/* 中文：★★★ 这是"ISR 只搬数据"原则的落地，也是这个文件的灵魂 ★★★
 *
 * 它只做一件事：**发现"这一波数据收完了"，然后通知上面把它取走。**
 * 不解析、不判断帧、不认识任何命令。
 *
 * 【IDLE 中断是什么意思】
 *   不是"空闲了"这么简单 —— 它的硬件含义是"接收线路**由忙变静**"。
 *   所以它触发时，DMA 缓冲里一定躺着**一整段连续到达的数据**（一个 burst）。
 *   注意：**一个 burst ≠ 一帧**。可能是一帧，也可能是半帧，或者两帧半。
 *   划帧是上面协议解析器的活，不是这里。
 *
 * 【为什么中断里还要处理错误标志】
 *   ORE（溢出）这类错误如果不清，接收器会"卡住"再也收不到数据。
 *   处理办法很土但有效：读一下 DR 就清除。
 *   然后呢？丢掉的那个 burst 谁来补？**协议层** ——
 *   CRC 校验会发现帧不完整，上层的序号 ACK + 超时重传会补回来。
 *   这就是分层的价值：硬件层只负责"别卡死"，可靠性交给上层。 */

void USART1_IRQHandler(void) {
    /* IDLE: the line went quiet, so the DMA has a complete burst ready. */
    if (__HAL_UART_GET_FLAG(&g_huart, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&g_huart);
        uart_comm_drain_rx();
        /* 中文：★注意顺序：先清标志，再干活★
         * 如果先干活再清标志，干活期间新到的数据造成的标志会被误清。 */
    }

    /* Overrun / framing / noise: read DR to clear. A dropped burst is
     * recovered by the protocol layer, not here. */
    if (__HAL_UART_GET_FLAG(&g_huart, UART_FLAG_ORE) != RESET) {
        (void)USART1->DR;
        /* 中文：读 DR 是为了清除 ORE 标志（硬件行为）。读到的值直接丢掉。 */
    }
}

/*---------------------------------------------------------------------------
 * HAL MSP callbacks
 *
 * The HAL calls these when the UART needs its GPIO, clocks, and DMA channel.
 *---------------------------------------------------------------------------*/
/* 中文：MSP = MCU Support Package。HAL_UART_Init() 内部会回调它，
 * 让你提供"这颗芯片上这个外设要哪些引脚、哪些时钟、哪个 DMA 通道"。
 * 通用逻辑在 HAL 里，芯片相关配置回调给用户 —— 这是 HAL 的分层方式。 */

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) {
        return;
    }

    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    /* 中文：★三个时钟都要开★ GPIOA（引脚）、USART1（串口本身）、
     * DMA1（搬运通道）。少开任何一个，现象都是"完全没反应且不报错"。 */

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
    /* 中文：TX 用"复用推挽"（芯片要主动驱动这根线），
     * RX 用"浮空输入"（芯片只负责听）。方向配反了收发都不通。 */

    /* USART1_RX is hardwired to DMA1 Channel 5 on STM32F1. */
    /* 中文：★USART1 的接收只能走 DMA1 Channel 5 —— 这是芯片内部硬连线的，
     * 不是可选项★ 换别的通道根本不会触发。这类"查数据手册才知道的固定映射"
     * 是 STM32 的典型特征，也是面试可能问的细节。 */
    g_hdma_usart_rx.Instance                 = DMA1_Channel5;
    g_hdma_usart_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    /* 中文：方向 = 外设 → 内存（串口寄存器 → 我们的缓冲区） */
    g_hdma_usart_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    /* 中文：外设地址不自增 —— 永远是 USART1->DR 这一个寄存器 */
    g_hdma_usart_rx.Init.MemInc              = DMA_MINC_ENABLE;
    /* 中文：内存地址**要**自增 —— 每搬一个字节，写进缓冲的下一个位置 */
    g_hdma_usart_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_hdma_usart_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    /* 中文：两边都按字节搬（不是半字/字）—— 串口就是一个字节一个字节的 */
    g_hdma_usart_rx.Init.Mode                = DMA_CIRCULAR;
    /* 中文：★环形模式★ 写满 1152 字节后自动回到开头继续写，永不停止。
     * 这是整个设计的前提：DMA 自己转圈，我们通过 head/tail 读它。 */
    g_hdma_usart_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&g_hdma_usart_rx);

    __HAL_LINKDMA(huart, hdmarx, g_hdma_usart_rx);
    /* 中文：★把 DMA 句柄挂到 UART 句柄上★
     * 之后 HAL_UART_Receive_DMA() 才知道该用哪个 DMA 通道。
     * 漏了这一句，Receive_DMA 会返回错误、收不到任何数据。 */
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
/* 中文：MspDeInit 在本项目里其实**不会被调用**（没有人反初始化串口）。
 * 保留它是 HAL 的约定 —— 实现了 Init 就该实现对应的 DeInit，
 * 否则将来有人调 HAL_UART_DeInit() 会走到一个空的默认实现上。 */
