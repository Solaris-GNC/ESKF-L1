#include "vn_logfmt.h"

#include <string.h>

uint16_t vn_crc16(const uint8_t *p, size_t n, uint16_t crc)
{
    for (size_t i = 0; i < n; i++) {
        crc = (uint16_t)(crc ^ (uint16_t)((uint16_t)p[i] << 8));
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t vn_frame(uint8_t *out, size_t cap, uint8_t type, const void *payload, uint16_t len)
{
    size_t n = (size_t)len + VN_FRAME_OVERHEAD;
    if (n > cap) return 0;
    out[0] = VN_SYNC0;
    out[1] = VN_SYNC1;
    out[2] = (uint8_t)(len & 0xFFu);
    out[3] = (uint8_t)(len >> 8);
    out[4] = type;
    if (len) memcpy(out + 5, payload, len);
    uint16_t crc = vn_crc16(out + 4, (size_t)len + 1u, 0xFFFFu);
    out[5 + len] = (uint8_t)(crc & 0xFFu);
    out[6 + len] = (uint8_t)(crc >> 8);
    return n;
}

void vn_strfill(char *dst, size_t n, const char *src)
{
    if (n == 0) return;
    memset(dst, 0, n);
    if (src) {
        size_t l = strlen(src);
        if (l > n - 1) l = n - 1;
        memcpy(dst, src, l);
    }
}
