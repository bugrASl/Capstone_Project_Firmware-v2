/**
 *  @file       cpcu_json.h
 *  @brief      Minimal JSON serializer for cpcu_ws (v2.4.0)
 *  @author     bugrASl
 *
 *  A tiny stream-style JSON writer. Symmetrical with the hand-rolled
 *  parser in cpcu_config.c — same project, same style, no external
 *  deps. Scope is intentionally tiny: produce valid JSON for objects
 *  containing strings, numbers (int/float), booleans, nested objects
 *  and arrays. No streaming reader, no validation, no escaping of
 *  arbitrary Unicode (we only ever emit ASCII anyway).
 *
 *  Usage:
 *      char buf[4096];
 *      JW jw;
 *      jw_init(&jw, buf, sizeof(buf));
 *      jw_obj_begin(&jw);
 *        jw_kv_int(&jw, "system_state", 1);
 *        jw_kv_str(&jw, "gesture", "rest");
 *        jw_kv_arr_f32(&jw, "rms", rms_array, 8);
 *      jw_obj_end(&jw);
 *      // jw.len is the bytes written; jw.overflow is true on truncation
 *
 *  Design choices:
 *    - Single linear buffer, no malloc. Caller owns memory.
 *    - Tracks a depth-stack of "is this the first child?" flags so
 *      commas between siblings get inserted correctly without the
 *      caller worrying about it.
 *    - On overflow, sets jw.overflow=true and stops writing. The
 *      output up to that point is incomplete JSON; the caller must
 *      check overflow before sending.
 *    - Floats serialize as %g with 6 sig figs — fine for dashboard
 *      display, not for archival precision.
 */

#ifndef CPCU_JSON_H
#define CPCU_JSON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define JW_MAX_DEPTH       8

typedef struct {
    char    *buf;
    size_t   cap;
    size_t   len;
    bool     overflow;
    /* Stack of "are we at first-child position in the current container".
     * On every container open we push true; after the first child write
     * we set top to false so the next sibling prepends a comma. */
    bool     first_at_depth[JW_MAX_DEPTH];
    int      depth;
} JW;

void  jw_init(JW *jw, char *buf, size_t cap);

/* Containers. Each open/close is balanced. */
void  jw_obj_begin(JW *jw);
void  jw_obj_end(JW *jw);
void  jw_arr_begin(JW *jw);
void  jw_arr_end(JW *jw);

/* Scalar values inside an array (no key). */
void  jw_int(JW *jw, long long v);
void  jw_u32(JW *jw, uint32_t v);
void  jw_f32(JW *jw, float v);
void  jw_str(JW *jw, const char *s);
void  jw_bool(JW *jw, bool v);

/* Key + value in a single call (only valid inside an object). */
void  jw_kv_int  (JW *jw, const char *k, long long v);
void  jw_kv_u32  (JW *jw, const char *k, uint32_t v);
void  jw_kv_u64  (JW *jw, const char *k, uint64_t v);
void  jw_kv_f32  (JW *jw, const char *k, float v);
void  jw_kv_str  (JW *jw, const char *k, const char *v);
void  jw_kv_bool (JW *jw, const char *k, bool v);

/* Convenience: emit an array of primitives under a key. */
void  jw_kv_arr_f32 (JW *jw, const char *k, const float    *vals, size_t n);
void  jw_kv_arr_u16 (JW *jw, const char *k, const uint16_t *vals, size_t n);
void  jw_kv_arr_i16 (JW *jw, const char *k, const int16_t  *vals, size_t n);
void  jw_kv_arr_u32 (JW *jw, const char *k, const uint32_t *vals, size_t n);

/* Sub-object/sub-array under a key. The caller is responsible for
 * matching close calls. */
void  jw_kv_obj_begin (JW *jw, const char *k);
void  jw_kv_arr_begin (JW *jw, const char *k);

#endif /* CPCU_JSON_H */
