#include "protocol.h"
#include <string.h>

uint8_t Protocol_CalcCRC(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

uint16_t Protocol_Pack(uint8_t *buf, uint8_t type, const void *payload, uint8_t pld_len)
{
    if (buf == NULL) return 0;
    if (pld_len > 250) return 0;  /* 长度字段是 1 字节，留出余量 */
    
    buf[0] = FRAME_HEADER_1;
    buf[1] = FRAME_HEADER_2;
    buf[2] = pld_len + 2;          /* LEN = TYPE(1) + PAYLOAD(N) + CRC(1) */
    buf[3] = type;
    
    if (payload != NULL && pld_len > 0) {
        memcpy(&buf[4], payload, pld_len);
    }
    
    /* CRC 覆盖 TYPE + PAYLOAD */
    buf[4 + pld_len] = Protocol_CalcCRC(&buf[3], pld_len + 1);
    
    return 5 + pld_len;  /* 总长度 = 2(头) + 1(LEN) + 1(TYPE) + N(载荷) + 1(CRC) */
}
