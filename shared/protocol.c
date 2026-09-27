/**
 * @file    protocol.c
 * @brief   Shared protocol implementation — CRC-32, frame parser, frame builder.
 *
 * Compiles into bootloader, application, and (with small adaption) ESP32 / PC tools.
 *
 * ===========================================================================
 * 中文导读：这个文件是 protocol.h 契约的「唯一实现」
 * ===========================================================================
 * 只有 232 行，分三块，互相独立：
 *   ① proto_crc32()        给数据算校验值（查表法）
 *   ② proto_parser_feed()  把字节流还原成一帧（九状态机）
 *   ③ proto_build_frame()  把一帧拼成字节流（组帧）
 *
 * 这三块**完全不碰硬件**：没有 HAL、没有 FreeRTOS、没有寄存器。
 * 所以同一份源码被编进四个地方：
 *   STM32 bootloader（arm-none-eabi-gcc）
 *   STM32 应用（arm-none-eabi-gcc）
 *   ESP32 网关（xtensa-esp32-elf-gcc，走 shared_protocol 组件）
 *   PC 主机测试（clang/gcc，在 tools 目录下直接编）
 * 这不是巧合，是刻意的分层 —— 把不依赖硬件的逻辑切出来，就能在电脑上
 * 跑自动化测试，接进 CI，不用插板子。这是本项目最该会讲的一条设计。
 * ===========================================================================
 */

#include "protocol.h"

/*---------------------------------------------------------------------------
 * CRC-32 lookup table (polynomial 0x04C11DB7, reflected)
 *---------------------------------------------------------------------------*/
/* 中文：★ 256 项表是怎么来的，怎么用的 ★
 *
 * 【为什么是 256 项】
 * 表的大小不由 CRC 的宽度（32 位）决定，而由「一次处理几位」决定。
 * 这里一次吃 1 个字节 = 8 位，一个字节只有 256 种取值，
 * 所以为这 256 种可能各预计算一个结果 → 256 项。
 *   · 一次吃 4 位（半字节）→ 16 项表（省 Flash，慢一倍）
 *   · 一次吃 16 位        → 65536 项表（不现实）
 * 32 位只影响每项占几个字节：每项 4 字节，于是 256 × 4 = 1KB。
 *
 * 【表怎么生成的】等价于下面这段（原本是离线算好写死在这里的）：
 *     for (int i = 0; i < 256; i++) {
 *         uint32_t c = i;
 *         for (int k = 0; k < 8; k++)
 *             c = (c & 1) ? (c >> 1) ^ 0xEDB88320UL : (c >> 1);
 *         table[i] = c;
 *     }
 * 含义：把「字节值 i 单独喂进 CRC 后对余数的贡献」预先算出来存好。
 * 类比九九乘法表：7×8 可以连加 8 次算，也可以直接查表。
 *
 * 【表怎么用的】见 proto_crc32() 里那一行：
 *     crc = crc32_table[(uint8_t)(crc ^ *data++)] ^ (crc >> 8);
 *   第一步  crc ^ byte     把新字节异或进 CRC 的低 8 位
 *   第二步  拿这 8 位当索引查表，得到"这 8 位会造成什么影响"
 *   第三步  crc >> 8       右移 8 位，腾位置给下一个字节
 *
 * 【为什么是"反射"形式】硬件做串行 CRC 时数据从最低位先进移位寄存器。
 * 为了软件结果跟硬件一致，整个算法都翻过来：多项式用 0x04C11DB7 的位反转
 * 形式 0xEDB88320，表生成用右移，更新用 >> 8。网上抄来的 CRC 代码经常
 * "算出来跟别人不一样"，八成就是反射/非反射混了。
 *
 * 【这段表曾经出过事】早期这份表和 ESP32 侧的副本里各有 4 个常量抄错，
 * 而经典测试向量 "123456789" 恰好没走到那几个表项，所以测试全绿、代码是错的。
 * 现在 tools/protocol_smoke_test.c 额外用 0x00..0xFF 全字节向量把 256 个
 * 表项全覆盖。这就是"测试通过 ≠ 正确，只等于覆盖到的那部分正确"。 */

static const uint32_t crc32_table[256] = {
    0x00000000U, 0x77073096U, 0xEE0E612CU, 0x990951BAU,
    0x076DC419U, 0x706AF48FU, 0xE963A535U, 0x9E6495A3U,
    0x0EDB8832U, 0x79DCB8A4U, 0xE0D5E91EU, 0x97D2D988U,
    0x09B64C2BU, 0x7EB17CBDU, 0xE7B82D07U, 0x90BF1D91U,
    0x1DB71064U, 0x6AB020F2U, 0xF3B97148U, 0x84BE41DEU,
    0x1ADAD47DU, 0x6DDDE4EBU, 0xF4D4B551U, 0x83D385C7U,
    0x136C9856U, 0x646BA8C0U, 0xFD62F97AU, 0x8A65C9ECU,
    0x14015C4FU, 0x63066CD9U, 0xFA0F3D63U, 0x8D080DF5U,
    0x3B6E20C8U, 0x4C69105EU, 0xD56041E4U, 0xA2677172U,
    0x3C03E4D1U, 0x4B04D447U, 0xD20D85FDU, 0xA50AB56BU,
    0x35B5A8FAU, 0x42B2986CU, 0xDBBBC9D6U, 0xACBCF940U,
    0x32D86CE3U, 0x45DF5C75U, 0xDCD60DCFU, 0xABD13D59U,
    0x26D930ACU, 0x51DE003AU, 0xC8D75180U, 0xBFD06116U,
    0x21B4F4B5U, 0x56B3C423U, 0xCFBA9599U, 0xB8BDA50FU,
    0x2802B89EU, 0x5F058808U, 0xC60CD9B2U, 0xB10BE924U,
    0x2F6F7C87U, 0x58684C11U, 0xC1611DABU, 0xB6662D3DU,
    0x76DC4190U, 0x01DB7106U, 0x98D220BCU, 0xEFD5102AU,
    0x71B18589U, 0x06B6B51FU, 0x9FBFE4A5U, 0xE8B8D433U,
    0x7807C9A2U, 0x0F00F934U, 0x9609A88EU, 0xE10E9818U,
    0x7F6A0DBBU, 0x086D3D2DU, 0x91646C97U, 0xE6635C01U,
    0x6B6B51F4U, 0x1C6C6162U, 0x856530D8U, 0xF262004EU,
    0x6C0695EDU, 0x1B01A57BU, 0x8208F4C1U, 0xF50FC457U,
    0x65B0D9C6U, 0x12B7E950U, 0x8BBEB8EAU, 0xFCB9887CU,
    0x62DD1DDFU, 0x15DA2D49U, 0x8CD37CF3U, 0xFBD44C65U,
    0x4DB26158U, 0x3AB551CEU, 0xA3BC0074U, 0xD4BB30E2U,
    0x4ADFA541U, 0x3DD895D7U, 0xA4D1C46DU, 0xD3D6F4FBU,
    0x4369E96AU, 0x346ED9FCU, 0xAD678846U, 0xDA60B8D0U,
    0x44042D73U, 0x33031DE5U, 0xAA0A4C5FU, 0xDD0D7CC9U,
    0x5005713CU, 0x270241AAU, 0xBE0B1010U, 0xC90C2086U,
    0x5768B525U, 0x206F85B3U, 0xB966D409U, 0xCE61E49FU,
    0x5EDEF90EU, 0x29D9C998U, 0xB0D09822U, 0xC7D7A8B4U,
    0x59B33D17U, 0x2EB40D81U, 0xB7BD5C3BU, 0xC0BA6CADU,
    0xEDB88320U, 0x9ABFB3B6U, 0x03B6E20CU, 0x74B1D29AU,
    0xEAD54739U, 0x9DD277AFU, 0x04DB2615U, 0x73DC1683U,
    0xE3630B12U, 0x94643B84U, 0x0D6D6A3EU, 0x7A6A5AA8U,
    0xE40ECF0BU, 0x9309FF9DU, 0x0A00AE27U, 0x7D079EB1U,
    0xF00F9344U, 0x8708A3D2U, 0x1E01F268U, 0x6906C2FEU,
    0xF762575DU, 0x806567CBU, 0x196C3671U, 0x6E6B06E7U,
    0xFED41B76U, 0x89D32BE0U, 0x10DA7A5AU, 0x67DD4ACCU,
    0xF9B9DF6FU, 0x8EBEEFF9U, 0x17B7BE43U, 0x60B08ED5U,
    0xD6D6A3E8U, 0xA1D1937EU, 0x38D8C2C4U, 0x4FDFF252U,
    0xD1BB67F1U, 0xA6BC5767U, 0x3FB506DDU, 0x48B2364BU,
    0xD80D2BDAU, 0xAF0A1B4CU, 0x36034AF6U, 0x41047A60U,
    0xDF60EFC3U, 0xA867DF55U, 0x316E8EEFU, 0x4669BE79U,
    0xCB61B38CU, 0xBC66831AU, 0x256FD2A0U, 0x5268E236U,
    0xCC0C7795U, 0xBB0B4703U, 0x220216B9U, 0x5505262FU,
    0xC5BA3BBEU, 0xB2BD0B28U, 0x2BB45A92U, 0x5CB36A04U,
    0xC2D7FFA7U, 0xB5D0CF31U, 0x2CD99E8BU, 0x5BDEAE1DU,
    0x9B64C2B0U, 0xEC63F226U, 0x756AA39CU, 0x026D930AU,
    0x9C0906A9U, 0xEB0E363FU, 0x72076785U, 0x05005713U,
    0x95BF4A82U, 0xE2B87A14U, 0x7BB12BAEU, 0x0CB61B38U,
    0x92D28E9BU, 0xE5D5BE0DU, 0x7CDCEFB7U, 0x0BDBDF21U,
    0x86D3D2D4U, 0xF1D4E242U, 0x68DDB3F8U, 0x1FDA836EU,
    0x81BE16CDU, 0xF6B9265BU, 0x6FB077E1U, 0x18B74777U,
    0x88085AE6U, 0xFF0F6A70U, 0x66063BCAU, 0x11010B5CU,
    0x8F659EFFU, 0xF862AE69U, 0x616BFFD3U, 0x166CCF45U,
    0xA00AE278U, 0xD70DD2EEU, 0x4E048354U, 0x3903B3C2U,
    0xA7672661U, 0xD06016F7U, 0x4969474DU, 0x3E6E77DBU,
    0xAED16A4AU, 0xD9D65ADCU, 0x40DF0B66U, 0x37D83BF0U,
    0xA9BCAE53U, 0xDEBB9EC5U, 0x47B2CF7FU, 0x30B5FFE9U,
    0xBDBDF21CU, 0xCABAC28AU, 0x53B39330U, 0x24B4A3A6U,
    0xBAD03605U, 0xCDD70693U, 0x54DE5729U, 0x23D967BFU,
    0xB3667A2EU, 0xC4614AB8U, 0x5D681B02U, 0x2A6F2B94U,
    0xB40BBE37U, 0xC30C8EA1U, 0x5A05DF1BU, 0x2D02EF8DU
};

uint32_t proto_crc32(const uint8_t *data, size_t len, uint32_t crc) {
    /* 中文：进函数先把 CRC 取反，出去前再取反 —— 这两步是"标准 CRC-32"
     * 的参数之一（init 和 final xor 都是 0xFFFFFFFF）。
     * 为什么要这样"取反进、取反出"：这样设计出来的 CRC 有两个好性质 ——
     * ① 数据前面补任意多个 0 不影响结果；② 检测数据开头丢失的 0 更敏感。
     * ★关键点★ 这种写法让函数可以"接着上次继续算"：传 0 表示从头开始，
     * 传上一次的返回值就把新数据接在后面。OTA 校验整个固件就是这么一块
     * 一块累加出来的，不用把 39KB 全读进内存。 */
    crc ^= 0xFFFFFFFFU;
    while (len--) {
        crc = crc32_table[(uint8_t)(crc ^ *data++)] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFU;
}

/*---------------------------------------------------------------------------
 * Frame parser
 *---------------------------------------------------------------------------*/

void proto_parser_init(ProtoParser_t *p) {
    p->state       = FRAME_STATE_SYNC;
    p->payload_idx = 0;
    p->rx_crc      = 0;
    /* The parser keeps the unfinalized IEEE CRC accumulator internally.
     * proto_crc32_buf() exposes the matching finalized value to callers. */
    p->calc_crc    = 0xFFFFFFFFU;
    p->frame.cmd   = 0;
    p->frame.len   = 0;
}
/* 中文：★注意 calc_crc 初值不是 0，而是 0xFFFFFFFF★
 * 因为解析器内部保存的是"还没做最终取反"的中间值（对应 proto_crc32() 里
 * 第一步 crc ^= 0xFFFFFFFFU 之后的状态）。校验比对时才补上最终取反，见
 * FRAME_STATE_CRC3 里的 final_crc。这个细节搞错，CRC 就会永远对不上。 */

static void parser_update_crc(ProtoParser_t *p, uint8_t byte) {
    p->calc_crc = crc32_table[(uint8_t)(p->calc_crc ^ byte)] ^ (p->calc_crc >> 8);
}
/* 中文：边收边算 —— 每进一个字节就更新一次累加器，所以不需要先把整帧
 * 缓存下来再算 CRC。这就是"流式解析"省内存的地方。 */

const ProtoFrame_t *proto_parser_feed(ProtoParser_t *p, uint8_t byte) {
    switch (p->state) {

    case FRAME_STATE_SYNC:
        if (byte == PROTO_SYNC_BYTE) {
            proto_parser_init(p);
            p->state = FRAME_STATE_CMD;
            parser_update_crc(p, byte);
        }
        /* else: stay in SYNC, skip byte */
        break;
        /* 中文：这就是"重新同步"。不是 0xA5 的字节直接丢掉 —— 所以线上
         * 多出来的垃圾数据（比如上电瞬间的噪声）会被自动跳过，不需要
         * 额外的握手。注意进来先调一次 init()，把上一次的半截帧彻底清空。 */

    case FRAME_STATE_CMD:
        p->frame.cmd = byte;
        parser_update_crc(p, byte);
        p->state = FRAME_STATE_LEN_LO;
        break;
        /* 中文：命令字原样收下，不在这里校验合法性。未知命令由更上层的
         * cmd_handler_dispatch() 回 NAK + ERR_UNKNOWN_CMD。分层原则：
         * 解析器只管"字节结构对不对"，不管"语义认不认识"。 */

    case FRAME_STATE_LEN_LO:
        p->frame.len = byte;
        parser_update_crc(p, byte);
        p->state = FRAME_STATE_LEN_HI;
        break;

    case FRAME_STATE_LEN_HI:
        p->frame.len |= ((uint16_t)byte << 8);
        parser_update_crc(p, byte);

        if (p->frame.len > PROTO_MAX_PAYLOAD) {
            /* Invalid length — abort frame, resync */
            p->state = FRAME_STATE_SYNC;
            break;
        }

        if (p->frame.len == 0) {
            p->state = FRAME_STATE_CRC0;
        } else {
            p->payload_idx = 0;
            p->state = FRAME_STATE_DATA;
        }
        break;
        /* 中文：★安全关键的一步★ 长度拼完（低字节在前 = 小端）立刻检查，
         * 超长马上回到 SYNC，绝不进入 DATA 状态。
         * 为什么在这里查而不是收完再查：如果先收再查，payload 数组早就被
         * 写越界了 —— 那是一个可被远程触发的内存破坏漏洞。
         * 先验证、后写入，顺序不能反。测试用例：tools/protocol_boundary_test.c
         * 里用 0xFFFF 长度专门打这个点，断言不会越界写。
         * 另外注意 len == 0 的处理：没有 payload 就直接跳到收 CRC，
         * 不能进 DATA（否则 payload_idx >= 0 立刻成立，会多读字节）。 */

    case FRAME_STATE_DATA:
        p->frame.payload[p->payload_idx++] = byte;
        parser_update_crc(p, byte);

        if (p->payload_idx >= p->frame.len) {
            p->state = FRAME_STATE_CRC0;
        }
        break;
        /* 中文：payload 收够 len 个才前进。因为前面已经保证 len ≤ 1028，
         * 这里写 payload[payload_idx] 不可能越界。 */

    case FRAME_STATE_CRC0:
        p->rx_crc  = byte;
        p->state   = FRAME_STATE_CRC1;
        break;

    case FRAME_STATE_CRC1:
        p->rx_crc |= ((uint32_t)byte << 8);
        p->state   = FRAME_STATE_CRC2;
        break;

    case FRAME_STATE_CRC2:
        p->rx_crc |= ((uint32_t)byte << 16);
        p->state   = FRAME_STATE_CRC3;
        break;
        /* 中文：CRC 是 4 字节小端，所以第一个收到的字节放最低位。
         * 这里正好是上面组帧时"低字节先写"的逆过程。 */

    case FRAME_STATE_CRC3: {
        /* Case body braced so this compiles as C and C++ (the local below
         * must not be jumped over across case labels in C++). */
        p->rx_crc |= ((uint32_t)byte << 24);

        /* proto_build_frame() writes a standard finalized IEEE CRC-32. */
        uint32_t final_crc = p->calc_crc ^ 0xFFFFFFFFU;

        p->state = FRAME_STATE_SYNC; /* ready for next frame */

        if (p->rx_crc == final_crc) {
            return &p->frame; /* valid frame */
        }
        /* CRC mismatch — frame silently dropped */
        break;
    }
        /* 中文：★整个解析器只有这一个地方会返回非 NULL★
         * 必须收完第 4 个 CRC 字节才能比对 —— 这就是为什么 CRC 放在帧尾，
         * 也是为什么前面所有次调用都返回 NULL。换句话说：
         *   返回 NULL ≠ 出错，只表示"还没凑齐一帧"或"这帧坏了"。
         * 校验失败时静默丢弃、不重传：链路层只负责"认出边界、扔掉坏帧"，
         * 可靠性由上层（OTA 的序号 ACK + 超时重试）保证。两层职责清晰。
         * 上面那句 C++ 兼容注释也很实在：C++ 不允许 case 标签跨过变量
         * 初始化，所以整个 case 体要加花括号。 */

    default:
        p->state = FRAME_STATE_SYNC;
        break;
    }

    return NULL;
}

/*---------------------------------------------------------------------------
 * Frame builder
 *---------------------------------------------------------------------------*/
/* 中文：组帧 = 解析的反向操作。把 cmd + payload 拼成
 *   [0xA5][CMD][LEN_L][LEN_H][payload][CRC32 小端]
 * 输出缓冲区由调用方提供（buf/buf_size），函数自己不分配内存 —— 这样
 * PC 上可以用栈数组，MCU 上用固定全局数组，同一份代码两边都能跑。 */

uint16_t proto_build_frame(uint8_t *buf, uint16_t buf_size,
                           uint8_t cmd, const uint8_t *payload, uint16_t len) {
    /* 中文：两道前置检查，任一不过就返回 0（"什么都没写"）。
     * 调用方必须检查返回值 —— 不检查就等于把"没发出去"当成"发出去了"。 */
    if (len > PROTO_MAX_PAYLOAD) return 0;

    uint16_t total = PROTO_HEADER_SIZE + len + PROTO_CRC_SIZE;
    if (total > buf_size) return 0;

    /* Header */
    uint16_t idx = 0;
    buf[idx++] = PROTO_SYNC_BYTE;
    buf[idx++] = cmd;
    buf[idx++] = (uint8_t)(len & 0xFF);        /* 中文：长度低字节在前 */
    buf[idx++] = (uint8_t)((len >> 8) & 0xFF); /* 中文：长度高字节在后 → 小端 */

    /* Payload */
    if (len > 0 && payload != NULL) {
        for (uint16_t i = 0; i < len; i++) {
            buf[idx++] = payload[i];
        }
    }

    /* CRC-32 (little-endian) over header + payload */
    /* 中文：★CRC 覆盖范围 = 头部 4 字节 + payload，不包含 CRC 自己★
     * 这是唯一合理的做法：算 CRC 时还不知道 CRC 是多少。
     * 收包侧（parser_update_crc）正好对应 —— 从 0xA5 开始一路累加，
     * 到进入 CRC0 状态前停手，覆盖的字节完全一致。两边任何一边多算或
     * 少算一个字节，所有帧都会校验失败，而且很难看出原因。 */
    uint32_t crc = proto_crc32_buf(buf, idx);
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 16) & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 24) & 0xFF);

    return idx;
}
/* 中文：返回值是实际写入的字节数（= 4 + len + 4），调用方拿它当"要发多少
 * 字节给串口"的长度用。 */
