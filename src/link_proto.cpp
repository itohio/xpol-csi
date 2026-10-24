#include "link_proto.h"
#include <string.h>

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

size_t link_frame(uint8_t* out, MsgType type, const void* payload, uint16_t payload_len) {
    const uint16_t body = payload_len + 1;  // type + payload
    out[0] = 0x7E;
    out[1] = (uint8_t)(body & 0xFF);
    out[2] = (uint8_t)(body >> 8);
    out[3] = (uint8_t)type;
    if (payload_len) memcpy(out + 4, payload, payload_len);
    uint16_t crc = crc16_ccitt(out + 3, body);
    out[4 + payload_len] = (uint8_t)(crc & 0xFF);
    out[5 + payload_len] = (uint8_t)(crc >> 8);
    return 6 + payload_len;
}

void LinkParser::feed(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t c = data[i];
        switch (st_) {
        case St::SYNC:
            if (c == 0x7E) { st_ = St::LEN0; }
            break;
        case St::LEN0:
            need_ = c; st_ = St::LEN1; break;
        case St::LEN1:
            need_ |= (uint16_t)c << 8;
            if (need_ == 0 || need_ > sizeof(buf_) - 2) { st_ = St::SYNC; break; }
            got_ = 0; st_ = St::BODY; break;
        case St::BODY:
            buf_[got_++] = c;
            if (got_ == need_ + 2) {  // body + crc
                uint16_t rx = buf_[need_] | ((uint16_t)buf_[need_ + 1] << 8);
                if (rx == crc16_ccitt(buf_, need_)) {
                    MsgType t = (MsgType)buf_[0];
                    handler_(t, buf_ + 1, need_ - 1);
                }
                st_ = St::SYNC;
            }
            break;
        }
    }
}
