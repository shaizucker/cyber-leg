#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace slcan {
struct Frame {
    uint32_t id = 0;
    uint8_t len = 0;
    uint8_t data[8] = {};
    bool extended = false;
    bool remote = false;
};

inline int hex(char c) {
    if (c >= '0' && c <= '9') return c-'0';
    if (c >= 'A' && c <= 'F') return c-'A'+10;
    if (c >= 'a' && c <= 'f') return c-'a'+10;
    return -1;
}

inline bool parse(const char *line, Frame &frame) {
    const char kind = line[0];
    if (kind != 't' && kind != 'T' && kind != 'r' && kind != 'R') return false;
    Frame candidate;
    candidate.extended = kind == 'T' || kind == 'R';
    candidate.remote = kind == 'r' || kind == 'R';
    const size_t digits = candidate.extended ? 8 : 3;
    const size_t size = strlen(line);
    if (size < digits+2) return false;
    for (size_t i=1; i<=digits; ++i) {
        const int nibble = hex(line[i]);
        if (nibble < 0) return false;
        candidate.id = (candidate.id<<4) | nibble;
    }
    if (candidate.id > (candidate.extended ? 0x1FFFFFFFu : 0x7FFu)) return false;
    const int length = hex(line[digits+1]);
    if (length < 0 || length > 8) return false;
    candidate.len = length;
    if (size != digits+2+(candidate.remote ? 0 : 2*candidate.len)) return false;
    for (size_t i=0; !candidate.remote && i<candidate.len; ++i) {
        const int high = hex(line[digits+2+2*i]), low = hex(line[digits+3+2*i]);
        if (high < 0 || low < 0) return false;
        candidate.data[i] = (high<<4) | low;
    }
    frame = candidate;
    return true;
}

inline size_t format(const Frame &frame, char (&output)[32]) {
    static const char digits[] = "0123456789ABCDEF";
    size_t index = 0;
    output[index++] = frame.remote ? (frame.extended ? 'R' : 'r') : (frame.extended ? 'T' : 't');
    const unsigned width = frame.extended ? 8 : 3;
    for (unsigned i=width; i>0; --i) output[index++] = digits[(frame.id>>((i-1)*4))&15];
    output[index++] = digits[frame.len];
    if (!frame.remote) for (unsigned i=0; i<frame.len; ++i) {
        output[index++] = digits[frame.data[i]>>4];
        output[index++] = digits[frame.data[i]&15];
    }
    output[index++] = '\r';
    output[index] = '\0';
    return index;
}
}
