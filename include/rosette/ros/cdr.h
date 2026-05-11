#pragma once
/*
 * ros/cdr.h — Rosette CDR writer/reader support for generated interface headers.
 *
 * This header is the Rosette-owned home for the inline CDR helpers used by
 * generated `.msg` / `.srv` headers. It intentionally exposes a small,
 * generator-friendly API built around basic CDR encoding and decoding with a
 * 4-byte encapsulation header at the start of the top-level payload.
 *
 * The API remains C-style because the generated headers are designed to be
 * lightweight and easy to include from production code. The surrounding
 * generator output may later include C++ integration such as
 * `milkros::message_traits`, but the serialization helpers themselves stay
 * simple and POD-friendly.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
uint32_t rtps_platform_get_ms(void);
#ifdef __cplusplus
}
#endif

/* Encapsulation header written at the start of every DATA payload */
#define CDR_ENCAP_LE  { 0x00, 0x01, 0x00, 0x00 }
#define CDR_ENCAP_BE  { 0x00, 0x00, 0x00, 0x00 }
#define CDR_ENCAP_LEN 4

typedef struct {
    uint8_t* buf;
    size_t   cap;
    size_t   pos;
    bool     overflow;
} cdr_writer_t;

typedef struct {
    const uint8_t* buf;
    size_t         len;
    size_t         pos;
    bool           error;
    bool           little_endian;
} cdr_reader_t;

/* -----------------------------------------------------------------------
 * Alignment helpers
 * --------------------------------------------------------------------- */
static inline size_t cdr_align(size_t pos, size_t alignment) {
    return (pos + alignment - 1) & ~(alignment - 1);
}

/* ======================================================================
 * WRITER
 * ==================================================================== */

static inline void cdr_writer_init(cdr_writer_t* w, uint8_t* buf, size_t cap) {
    w->buf = buf;
    w->cap = cap;
    w->pos = 0;
    w->overflow = false;
    static const uint8_t enc[] = CDR_ENCAP_LE;
    memcpy(buf, enc, CDR_ENCAP_LEN);
    w->pos = CDR_ENCAP_LEN;
}

static inline void cdr_write_pad(cdr_writer_t* w, size_t alignment) {
    size_t aligned = cdr_align(w->pos, alignment);
    while (w->pos < aligned) {
        if (w->pos < w->cap) w->buf[w->pos] = 0;
        else w->overflow = true;
        w->pos++;
    }
}

static inline void cdr_write_u8(cdr_writer_t* w, uint8_t v) {
    if (w->pos + 1 <= w->cap) w->buf[w->pos] = v;
    else w->overflow = true;
    w->pos++;
}

static inline void cdr_write_bool(cdr_writer_t* w, bool v) {
    cdr_write_u8(w, v ? 1 : 0);
}

static inline void cdr_write_u16(cdr_writer_t* w, uint16_t v) {
    cdr_write_pad(w, 2);
    if (w->pos + 2 <= w->cap) {
        w->buf[w->pos + 0] = (uint8_t)(v);
        w->buf[w->pos + 1] = (uint8_t)(v >> 8);
    } else {
        w->overflow = true;
    }
    w->pos += 2;
}

static inline void cdr_write_i32(cdr_writer_t* w, int32_t v) {
    cdr_write_pad(w, 4);
    if (w->pos + 4 <= w->cap) {
        uint32_t u = (uint32_t)v;
        w->buf[w->pos + 0] = (uint8_t)(u);
        w->buf[w->pos + 1] = (uint8_t)(u >> 8);
        w->buf[w->pos + 2] = (uint8_t)(u >> 16);
        w->buf[w->pos + 3] = (uint8_t)(u >> 24);
    } else {
        w->overflow = true;
    }
    w->pos += 4;
}

static inline void cdr_write_u32(cdr_writer_t* w, uint32_t v) {
    cdr_write_i32(w, (int32_t)v);
}

static inline void cdr_write_f32(cdr_writer_t* w, float v) {
    uint32_t u;
    memcpy(&u, &v, 4);
    cdr_write_u32(w, u);
}

static inline void cdr_write_f64(cdr_writer_t* w, double v) {
    uint64_t u;
    cdr_write_pad(w, 4);
    memcpy(&u, &v, 8);
    if (w->pos + 8 <= w->cap) {
        for (int i = 0; i < 8; i++) {
            w->buf[w->pos + i] = (uint8_t)(u >> (8 * i));
        }
    } else {
        w->overflow = true;
    }
    w->pos += 8;
}

static inline void cdr_write_u64(cdr_writer_t* w, uint64_t v) {
    cdr_write_pad(w, 4);
    if (w->pos + 8 <= w->cap) {
        for (int i = 0; i < 8; i++) {
            w->buf[w->pos + i] = (uint8_t)(v >> (8 * i));
        }
    } else {
        w->overflow = true;
    }
    w->pos += 8;
}

static inline void cdr_write_string(cdr_writer_t* w, const char* s) {
    uint32_t slen = s ? (uint32_t)strlen(s) + 1 : 1;
    cdr_write_u32(w, slen);
    if (w->pos + slen <= w->cap) {
        if (s) memcpy(w->buf + w->pos, s, slen);
        else w->buf[w->pos] = 0;
    } else {
        w->overflow = true;
    }
    w->pos += slen;
}

static inline void cdr_write_bytes(cdr_writer_t* w,
                                   const uint8_t* data,
                                   size_t len) {
    if (w->pos + len <= w->cap) memcpy(w->buf + w->pos, data, len);
    else w->overflow = true;
    w->pos += len;
}

static inline size_t cdr_writer_size(const cdr_writer_t* w) {
    return w->pos;
}

/* ======================================================================
 * READER
 * ==================================================================== */

static inline bool cdr_reader_init(cdr_reader_t* r,
                                   const uint8_t* buf,
                                   size_t len) {
    r->buf = buf;
    r->len = len;
    r->pos = 0;
    r->error = false;
    if (len < CDR_ENCAP_LEN) {
        r->error = true;
        return false;
    }
    if (buf[0] != 0x00) {
        r->error = true;
        return false;
    }
    r->little_endian = (buf[1] == 0x01);
    r->pos = CDR_ENCAP_LEN;
    return true;
}

static inline void cdr_read_pad(cdr_reader_t* r, size_t alignment) {
    r->pos = cdr_align(r->pos, alignment);
}

static inline uint8_t cdr_read_u8(cdr_reader_t* r) {
    if (r->pos + 1 > r->len) {
        r->error = true;
        return 0;
    }
    return r->buf[r->pos++];
}

static inline bool cdr_read_bool(cdr_reader_t* r) {
    return cdr_read_u8(r) != 0;
}

static inline uint16_t cdr_read_u16(cdr_reader_t* r) {
    cdr_read_pad(r, 2);
    if (r->pos + 2 > r->len) {
        r->error = true;
        return 0;
    }
    uint16_t v;
    if (r->little_endian) {
        v = (uint16_t)(r->buf[r->pos] |
                       ((uint16_t)r->buf[r->pos + 1] << 8));
    } else {
        v = (uint16_t)(r->buf[r->pos + 1] |
                       ((uint16_t)r->buf[r->pos] << 8));
    }
    r->pos += 2;
    return v;
}

static inline uint32_t cdr_read_u32(cdr_reader_t* r) {
    cdr_read_pad(r, 4);
    if (r->pos + 4 > r->len) {
        r->error = true;
        return 0;
    }
    uint32_t v;
    if (r->little_endian) {
        v = (uint32_t)r->buf[r->pos]
          | ((uint32_t)r->buf[r->pos + 1] << 8)
          | ((uint32_t)r->buf[r->pos + 2] << 16)
          | ((uint32_t)r->buf[r->pos + 3] << 24);
    } else {
        v = ((uint32_t)r->buf[r->pos] << 24)
          | ((uint32_t)r->buf[r->pos + 1] << 16)
          | ((uint32_t)r->buf[r->pos + 2] << 8)
          | (uint32_t)r->buf[r->pos + 3];
    }
    r->pos += 4;
    return v;
}

static inline int32_t cdr_read_i32(cdr_reader_t* r) {
    return (int32_t)cdr_read_u32(r);
}

static inline float cdr_read_f32(cdr_reader_t* r) {
    uint32_t u = cdr_read_u32(r);
    float v;
    memcpy(&v, &u, 4);
    return v;
}

static inline double cdr_read_f64(cdr_reader_t* r) {
    cdr_read_pad(r, 4);
    if (r->pos + 8 > r->len) {
        r->error = true;
        return 0.0;
    }
    uint64_t u = 0;
    for (int i = 0; i < 8; i++) {
        if (r->little_endian) u |= ((uint64_t)r->buf[r->pos + i] << (8 * i));
        else u |= ((uint64_t)r->buf[r->pos + i] << (56 - 8 * i));
    }
    double v;
    memcpy(&v, &u, 8);
    r->pos += 8;
    return v;
}

static inline uint64_t cdr_read_u64(cdr_reader_t* r) {
    cdr_read_pad(r, 4);
    if (r->pos + 8 > r->len) {
        r->error = true;
        return 0;
    }
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) {
        if (r->little_endian) v |= ((uint64_t)r->buf[r->pos + i] << (8 * i));
        else v |= ((uint64_t)r->buf[r->pos + i] << (56 - 8 * i));
    }
    r->pos += 8;
    return v;
}

static inline int64_t cdr_read_i64(cdr_reader_t* r) {
    return (int64_t)cdr_read_u64(r);
}

static inline size_t cdr_read_string(cdr_reader_t* r,
                                     char* dst,
                                     size_t dst_cap) {
    uint32_t slen = cdr_read_u32(r);
    if (r->error || slen == 0) return 0;
    if (r->pos + slen > r->len) {
        r->error = true;
        return 0;
    }
    size_t copy = (slen < dst_cap) ? slen : dst_cap - 1;
    memcpy(dst, r->buf + r->pos, copy);
    dst[copy] = '\0';
    r->pos += slen;
    return copy > 0 ? copy - 1 : 0;
}

static inline void cdr_skip(cdr_reader_t* r, size_t n) {
    if (r->pos + n > r->len) {
        r->error = true;
        r->pos = r->len;
    } else {
        r->pos += n;
    }
}

static inline size_t cdr_reader_remaining(const cdr_reader_t* r) {
    return (r->pos <= r->len) ? r->len - r->pos : 0;
}

/* ======================================================================
 * BODY variants
 * ==================================================================== */

static inline void cdr_writer_init_body(cdr_writer_t* w,
                                        uint8_t* buf,
                                        size_t cap) {
    w->buf = buf;
    w->cap = cap;
    w->pos = 0;
    w->overflow = false;
}

static inline void cdr_writer_sub(cdr_writer_t* sub,
                                  const cdr_writer_t* parent) {
    sub->buf = parent->buf + parent->pos;
    sub->cap = (parent->pos <= parent->cap) ? parent->cap - parent->pos : 0;
    sub->pos = 0;
    sub->overflow = false;
}

static inline void cdr_writer_commit(cdr_writer_t* parent,
                                     const cdr_writer_t* sub) {
    parent->pos += sub->pos;
    if (sub->overflow) parent->overflow = true;
}

static inline void cdr_reader_init_body(cdr_reader_t* r,
                                        const uint8_t* buf,
                                        size_t len,
                                        bool little_endian) {
    r->buf = buf;
    r->len = len;
    r->pos = 0;
    r->error = false;
    r->little_endian = little_endian;
}

/* -----------------------------------------------------------------------
 * Time helpers used by generated builtin_interfaces/Time and std_msgs/Header
 * --------------------------------------------------------------------- */

static inline void cdr_time_now(int32_t* sec, uint32_t* nanosec) {
    const uint32_t now_ms = rtps_platform_get_ms();
    if (sec) *sec = (int32_t)(now_ms / 1000u);
    if (nanosec) *nanosec = (now_ms % 1000u) * 1000000u;
}
