/**
 * @file bindings/micropython/rosette_mpy_schema.cpp
 * @brief Runtime `.msg` schema parser and generic CDR codec implementation unit.
 *
 * This unit parses a subset of ROS `.msg` syntax into compact field
 * descriptors and uses those descriptors to encode Python dictionaries into
 * CDR as well as decode CDR back into Python dictionaries.
 *
 * The schema layer is separate from transport and backend entity lifecycle so
 * new message-field support can be added without changing how contexts or
 * sockets are managed.
 *
 * Supported `.msg` field shapes
 * ------------------------------
 *   primitive_type  field_name         (scalar)
 *   primitive_type[] field_name        (unbounded dynamic array → Python list)
 *   primitive_type[N] field_name       (fixed array of N elements → Python list)
 *   Package/Name    field_name         (nested message → Python dict/Message)
 *   Package/Name[]  field_name         (dynamic array of nested messages)
 *   Package/Name[N] field_name         (fixed array of nested messages)
 *
 * Nested schema objects are resolved lazily: the field stores the full_name
 * qstr and the schema pointer is filled in the first time encode or decode
 * needs it.  This avoids ordering constraints between schema() calls in Python.
 */

#include "rosette_mpy_backend.hpp"

extern "C" {
#include "py/objlist.h"
#include "py/runtime.h"
}

#include "rosette/ros/cdr.h"

#include <limits.h>
#include <string.h>

namespace rosette::bindings::micropython {

namespace {

// ============================================================
// Text-span helpers (parser building blocks)
// ============================================================

struct TextSpan {
    const char* data = nullptr;
    size_t length = 0u;
};

TextSpan Trim(TextSpan span) noexcept
{
    while (span.length != 0u &&
           (span.data[0] == ' ' || span.data[0] == '\t' || span.data[0] == '\r')) {
        ++span.data;
        --span.length;
    }
    while (span.length != 0u) {
        const char c = span.data[span.length - 1u];
        if (c != ' ' && c != '\t' && c != '\r') break;
        --span.length;
    }
    return span;
}

bool SpanEquals(TextSpan span, const char* token) noexcept
{
    return token != nullptr && strlen(token) == span.length &&
           strncmp(span.data, token, span.length) == 0;
}

// ============================================================
// Primitive type token → field kind
// ============================================================

/**
 * @brief Resolves a bare primitive type token (no brackets) to a field kind.
 *
 * @param[in]  type_token Type name span, already stripped of array brackets.
 * @param[out] kind       Receives the matching ROSETTE_MPY_FIELD_* constant.
 * @return true when the token is a recognised primitive.
 */
bool ResolvePrimitiveKind(TextSpan type_token, uint8_t* kind) noexcept
{
    if (SpanEquals(type_token, "bool"))    { *kind = ROSETTE_MPY_FIELD_BOOL;    return true; }
    if (SpanEquals(type_token, "int8"))    { *kind = ROSETTE_MPY_FIELD_INT8;    return true; }
    if (SpanEquals(type_token, "uint8") ||
        SpanEquals(type_token, "byte"))    { *kind = ROSETTE_MPY_FIELD_UINT8;   return true; }
    if (SpanEquals(type_token, "int16"))   { *kind = ROSETTE_MPY_FIELD_INT16;   return true; }
    if (SpanEquals(type_token, "uint16"))  { *kind = ROSETTE_MPY_FIELD_UINT16;  return true; }
    if (SpanEquals(type_token, "int32"))   { *kind = ROSETTE_MPY_FIELD_INT32;   return true; }
    if (SpanEquals(type_token, "uint32"))  { *kind = ROSETTE_MPY_FIELD_UINT32;  return true; }
    if (SpanEquals(type_token, "int64"))   { *kind = ROSETTE_MPY_FIELD_INT64;   return true; }
    if (SpanEquals(type_token, "uint64"))  { *kind = ROSETTE_MPY_FIELD_UINT64;  return true; }
    if (SpanEquals(type_token, "float32")) { *kind = ROSETTE_MPY_FIELD_FLOAT32; return true; }
    if (SpanEquals(type_token, "float64")) { *kind = ROSETTE_MPY_FIELD_FLOAT64; return true; }
    if (SpanEquals(type_token, "string"))  { *kind = ROSETTE_MPY_FIELD_STRING;  return true; }
    return false;
}

/**
 * @brief Returns whether a type token looks like a nested-message reference.
 *
 * ROS convention: package-qualified names contain a '/' (e.g. std_msgs/Header).
 * Unqualified names starting with an upper-case letter are treated the same
 * way (e.g. Header within the same package).  Both are accepted here so that
 * schemas can reference sibling message types.
 *
 * @param[in] type_token Type token span to test.
 * @return true when the token should be treated as a nested message name.
 */
bool LooksLikeNestedMessage(TextSpan type_token) noexcept
{
    if (type_token.length == 0u) return false;
    if (static_cast<const char*>(
            memchr(type_token.data, '/', type_token.length)) != nullptr) {
        return true;
    }
    return (type_token.data[0] >= 'A' && type_token.data[0] <= 'Z');
}

// ============================================================
// Default Python value per field kind
// ============================================================

mp_obj_t DefaultFieldValue(uint8_t kind)
{
    switch (kind) {
    case ROSETTE_MPY_FIELD_BOOL:
        return mp_const_false;
    case ROSETTE_MPY_FIELD_FLOAT32:
    case ROSETTE_MPY_FIELD_FLOAT64:
        return mp_obj_new_float(0.0);
    case ROSETTE_MPY_FIELD_STRING:
        return mp_obj_new_str("", 0);
    case ROSETTE_MPY_FIELD_NESTED_MESSAGE:
        return mp_obj_new_dict(0);
    case ROSETTE_MPY_FIELD_DYNAMIC_ARRAY:
    case ROSETTE_MPY_FIELD_FIXED_ARRAY:
        return mp_obj_new_list(0, nullptr);
    default:
        return MP_OBJ_NEW_SMALL_INT(0);
    }
}

// ============================================================
// Line counter for two-pass parser
// ============================================================

size_t CountFieldLines(const char* text) noexcept
{
    size_t count = 0u;
    const char* line = text;
    while (line != nullptr && *line != '\0') {
        const char* end = strchr(line, '\n');
        const size_t length = end == nullptr ? strlen(line)
                                             : static_cast<size_t>(end - line);
        TextSpan span = Trim({line, length});
        const char* comment = static_cast<const char*>(
            memchr(span.data, '#', span.length));
        if (comment != nullptr) {
            span.length = static_cast<size_t>(comment - span.data);
            span = Trim(span);
        }
        if (span.length != 0u &&
            memchr(span.data, '=', span.length) == nullptr) {
            ++count;
        }
        line = end == nullptr ? nullptr : end + 1;
    }
    return count;
}

// ============================================================
// Full .msg parser — fills schema->fields[]
// ============================================================

/**
 * @brief Parses one `.msg` text block into schema field descriptors.
 *
 * Each supported declaration has one of these shapes:
 *   <primitive>          <name>
 *   <primitive>[]        <name>   (dynamic array)
 *   <primitive>[N]       <name>   (fixed array of N elements)
 *   <Package/Name>       <name>
 *   <Package/Name>[]     <name>
 *   <Package/Name>[N]    <name>
 *
 * Comments, blank lines, and constant definitions are skipped.  Fields with
 * unrecognised types raise ValueError so the Python caller sees the problem at
 * schema construction time rather than at the first publish or recv call.
 *
 * @param[in,out] schema Schema object that receives the parsed descriptors.
 * @param[in]     text   Null-terminated `.msg` source.
 */
void ParseMessageText(mp_rosette_schema_obj_t* schema, const char* text)
{
    if (schema == nullptr || text == nullptr) return;

    const size_t field_count = CountFieldLines(text);
    if (field_count == 0u) return;

    schema->fields = m_new(rosette_mpy_schema_field_t, field_count);
    schema->field_count = field_count;

    size_t index = 0u;
    const char* line = text;
    while (line != nullptr && *line != '\0' && index < field_count) {
        const char* end = strchr(line, '\n');
        const size_t length = end == nullptr ? strlen(line)
                                             : static_cast<size_t>(end - line);
        TextSpan span = Trim({line, length});
        const char* comment = static_cast<const char*>(
            memchr(span.data, '#', span.length));
        if (comment != nullptr) {
            span.length = static_cast<size_t>(comment - span.data);
            span = Trim(span);
        }
        if (span.length == 0u ||
            memchr(span.data, '=', span.length) != nullptr) {
            line = end == nullptr ? nullptr : end + 1;
            continue;
        }

        // Split on whitespace into type token and name token.
        size_t split = 0u;
        while (split < span.length &&
               span.data[split] != ' ' && span.data[split] != '\t') {
            ++split;
        }
        TextSpan type_token = Trim({span.data, split});
        TextSpan name_token = Trim({span.data + split, span.length - split});
        if (type_token.length == 0u || name_token.length == 0u) {
            mp_raise_ValueError(MP_ERROR_TEXT("invalid .msg field declaration"));
        }

        // Detect and strip array suffix: [] or [N].
        uint8_t outer_kind = 0u;     // FIXED_ARRAY, DYNAMIC_ARRAY, or 0 for scalar
        uint16_t element_count = 0u;

        // Find the first '[' in the type token.
        const char* bracket =
            static_cast<const char*>(memchr(type_token.data, '[', type_token.length));
        if (bracket != nullptr) {
            size_t base_len = static_cast<size_t>(bracket - type_token.data);
            const char* close =
                static_cast<const char*>(memchr(bracket, ']', type_token.length - base_len));
            if (close == nullptr) {
                mp_raise_ValueError(MP_ERROR_TEXT(".msg array type missing closing ']'"));
            }
            const char* count_start = bracket + 1;
            size_t count_len = static_cast<size_t>(close - count_start);
            if (count_len == 0u) {
                outer_kind = ROSETTE_MPY_FIELD_DYNAMIC_ARRAY;
                element_count = 0u;
            } else {
                // Parse the integer count.  strtoul is not always available on
                // embedded targets so we implement a minimal decimal parser.
                unsigned long n = 0u;
                for (size_t ci = 0u; ci < count_len; ++ci) {
                    const char ch = count_start[ci];
                    if (ch < '0' || ch > '9') {
                        mp_raise_ValueError(MP_ERROR_TEXT(".msg array count must be decimal integer"));
                    }
                    n = n * 10u + static_cast<unsigned long>(ch - '0');
                    if (n > UINT16_MAX) {
                        mp_raise_ValueError(MP_ERROR_TEXT(".msg fixed array count too large"));
                    }
                }
                outer_kind = ROSETTE_MPY_FIELD_FIXED_ARRAY;
                element_count = static_cast<uint16_t>(n);
            }
            type_token.length = base_len;
            type_token = Trim(type_token);
        }

        // Resolve element kind.
        uint8_t element_kind = ROSETTE_MPY_FIELD_BOOL;
        qstr nested_full_name = 0;
        if (!ResolvePrimitiveKind(type_token, &element_kind)) {
            if (!LooksLikeNestedMessage(type_token)) {
                mp_raise_ValueError(MP_ERROR_TEXT("unsupported .msg field type"));
            }
            element_kind = ROSETTE_MPY_FIELD_NESTED_MESSAGE;
            // Intern the nested type name so we can resolve it later.
            nested_full_name = qstr_from_strn(type_token.data, type_token.length);
        }

        rosette_mpy_schema_field_t& f = schema->fields[index];
        f = {};
        f.name = qstr_from_strn(name_token.data, name_token.length);
        f.element_kind = element_kind;
        f.nested_full_name = nested_full_name;
        f.nested_schema = nullptr;  // resolved lazily on first codec use

        if (outer_kind != 0u) {
            f.kind = outer_kind;
            f.element_count = element_count;
        } else {
            f.kind = element_kind;
            f.element_count = 0u;
        }

        ++index;
        line = end == nullptr ? nullptr : end + 1;
    }

    // Trim field_count to the number actually parsed (should match, but guard).
    schema->field_count = index;
}

// ============================================================
// Static-registry helpers (kept for pre-compiled known types)
// ============================================================

uint8_t ResolveMessageKind(const char* full_name) noexcept
{
    if (full_name == nullptr) return ROSETTE_MPY_MESSAGE_UNSUPPORTED;
    if (strcmp(full_name, "std_msgs/Float32") == 0)
        return ROSETTE_MPY_MESSAGE_STD_MSGS_FLOAT32;
    return ROSETTE_MPY_MESSAGE_UNSUPPORTED;
}

void SplitMessageName(const char* full_name,
                      qstr* package_name,
                      qstr* short_name)
{
    const char* slash = full_name == nullptr ? nullptr : strchr(full_name, '/');
    if (slash == nullptr) {
        *package_name = qstr_from_str("");
        *short_name   = qstr_from_str(full_name == nullptr ? "" : full_name);
        return;
    }
    *package_name = qstr_from_strn(full_name, static_cast<size_t>(slash - full_name));
    *short_name   = qstr_from_str(slash + 1);
}

// ============================================================
// Nested-schema resolver (lazy, called from codec paths)
// ============================================================

/**
 * @brief Looks up a Schema object registered in the module's schema registry.
 *
 * The module keeps no registry of its own; instead Python code is expected to
 * pass a schema object or the codec raises at the point of first use so the
 * user sees a clear error.  When a nested schema was constructed before the
 * parent, `field.nested_schema` is already set.
 *
 * This function is intentionally kept simple: it iterates the MicroPython
 * globals dict looking for an object whose `full_name` qstr matches.
 * If a match is not found it raises AttributeError so the caller gets a
 * Python-visible error rather than a silent null-pointer dereference.
 *
 * @param[in,out] field Field descriptor whose nested_schema pointer may be null.
 */
void ResolveNestedSchema(rosette_mpy_schema_field_t* field)
{
    if (field->nested_schema != nullptr) return;
    if (field->nested_full_name == 0) {
        mp_raise_msg(&mp_type_AttributeError,
                     MP_ERROR_TEXT("nested .msg field has no type name"));
    }

    // Walk the caller's local/global scope looking for a Schema object whose
    // full_name matches.  This covers the common pattern:
    //   header_schema = rosette.schema("std_msgs/Header", "...")
    //   pose_schema   = rosette.schema("...", "std_msgs/Header header\n...")
    mp_obj_t globals = mp_globals_get();
    mp_map_t* map = mp_obj_dict_get_map(globals);
    for (size_t i = 0u; i < map->alloc; ++i) {
        if (mp_map_slot_is_filled(map, i)) {
            mp_obj_t val = map->table[i].value;
            if (mp_obj_is_type(val,
                    reinterpret_cast<const mp_obj_type_t*>(&rosette_schema_type))) {
                auto* s = static_cast<mp_rosette_schema_obj_t*>(MP_OBJ_TO_PTR(val));
                if (s->full_name == field->nested_full_name) {
                    field->nested_schema = s;
                    return;
                }
            }
        }
    }
    mp_raise_msg_varg(&mp_type_AttributeError,
                      MP_ERROR_TEXT("nested schema '%q' not found in globals"),
                      field->nested_full_name);
}

// ============================================================
// CDR encoder: Python value → CDR bytes
// ============================================================

// Forward declaration for mutual recursion (nested message arrays).
void EncodeCdrFromSchema(cdr_writer_t* w,
                         const mp_rosette_schema_obj_t* schema,
                         mp_obj_t values);

/**
 * @brief Writes one scalar field value into a CDR writer.
 *
 * @param[in,out] w     CDR writer positioned at the next field slot.
 * @param[in]     kind  ROSETTE_MPY_FIELD_* scalar kind.
 * @param[in]     value Python object holding the field value.
 */
void EncodeScalar(cdr_writer_t* w, uint8_t kind, mp_obj_t value)
{
    switch (kind) {
    case ROSETTE_MPY_FIELD_BOOL:
        cdr_write_bool(w, mp_obj_is_true(value));
        break;
    case ROSETTE_MPY_FIELD_INT8:
    case ROSETTE_MPY_FIELD_UINT8:
        cdr_write_u8(w, static_cast<uint8_t>(mp_obj_get_int(value)));
        break;
    case ROSETTE_MPY_FIELD_INT16:
    case ROSETTE_MPY_FIELD_UINT16:
        cdr_write_u16(w, static_cast<uint16_t>(mp_obj_get_int(value)));
        break;
    case ROSETTE_MPY_FIELD_INT32:
    case ROSETTE_MPY_FIELD_UINT32:
        cdr_write_u32(w, static_cast<uint32_t>(mp_obj_get_int(value)));
        break;
    case ROSETTE_MPY_FIELD_INT64:
    case ROSETTE_MPY_FIELD_UINT64:
        cdr_write_u64(w, static_cast<uint64_t>(mp_obj_get_int(value)));
        break;
    case ROSETTE_MPY_FIELD_FLOAT32:
        cdr_write_f32(w, static_cast<float>(mp_obj_get_float(value)));
        break;
    case ROSETTE_MPY_FIELD_FLOAT64:
        cdr_write_f64(w, mp_obj_get_float(value));
        break;
    case ROSETTE_MPY_FIELD_STRING: {
        size_t len = 0u;
        const char* s = mp_obj_str_get_data(value, &len);
        cdr_write_string(w, s);
        break;
    }
    default:
        break;
    }
}

/**
 * @brief Encodes a Python message value into CDR using schema field descriptors.
 *
 * `values` may be a Schema-created Message object, a plain dictionary, or (for
 * single-field schemas) a bare scalar accepted for convenience.  The encoder
 * writes each field in declaration order, recursing into nested messages.
 *
 * @param[in,out] w      CDR writer.  The caller is responsible for the
 *                       encapsulation header; sub-message encoding uses
 *                       cdr_writer_sub / cdr_writer_commit pairs.
 * @param[in]     schema Parsed field descriptors for this message level.
 * @param[in]     values Python dict, Message, or scalar payload.
 */
void EncodeCdrFromSchema(cdr_writer_t* w,
                         const mp_rosette_schema_obj_t* schema,
                         mp_obj_t values)
{
    for (size_t i = 0u; i < schema->field_count; ++i) {
        // Cast away const: ResolveNestedSchema may fill nested_schema pointer.
        rosette_mpy_schema_field_t* field =
            const_cast<rosette_mpy_schema_field_t*>(&schema->fields[i]);

        // Fetch value from dict or use default.
        mp_obj_t value = mp_const_none;
        if (mp_obj_is_dict_or_ordereddict(values)) {
            mp_map_t* map = mp_obj_dict_get_map(values);
            mp_map_elem_t* elem =
                mp_map_lookup(map, MP_OBJ_NEW_QSTR(field->name), MP_MAP_LOOKUP);
            value = (elem != nullptr) ? elem->value : DefaultFieldValue(field->kind);
        } else if (schema->field_count == 1u) {
            // Bare scalar convenience for single-field messages.
            value = values;
        } else {
            value = DefaultFieldValue(field->kind);
        }

        if (field->kind == ROSETTE_MPY_FIELD_NESTED_MESSAGE) {
            ResolveNestedSchema(field);
            EncodeCdrFromSchema(w, field->nested_schema, value);

        } else if (field->kind == ROSETTE_MPY_FIELD_FIXED_ARRAY ||
                   field->kind == ROSETTE_MPY_FIELD_DYNAMIC_ARRAY) {

            // Collect elements from the Python list (or tuple).
            mp_obj_t* items = nullptr;
            size_t     item_count = 0u;
            if (mp_obj_is_type(value, &mp_type_list) ||
                mp_obj_is_type(value, &mp_type_tuple)) {
                mp_obj_get_array(value, &item_count, &items);
            }

            if (field->kind == ROSETTE_MPY_FIELD_DYNAMIC_ARRAY) {
                // Dynamic arrays are length-prefixed.
                cdr_write_u32(w, static_cast<uint32_t>(item_count));
            }

            const size_t expected =
                (field->kind == ROSETTE_MPY_FIELD_FIXED_ARRAY)
                    ? static_cast<size_t>(field->element_count)
                    : item_count;

            for (size_t ei = 0u; ei < expected; ++ei) {
                mp_obj_t elem =
                    (ei < item_count) ? items[ei] : DefaultFieldValue(field->element_kind);

                if (field->element_kind == ROSETTE_MPY_FIELD_NESTED_MESSAGE) {
                    ResolveNestedSchema(field);
                    EncodeCdrFromSchema(w, field->nested_schema, elem);
                } else {
                    EncodeScalar(w, field->element_kind, elem);
                }
            }

        } else {
            EncodeScalar(w, field->kind, value);
        }
    }
}

// ============================================================
// CDR decoder: CDR bytes → Python dict
// ============================================================

// Forward declaration for mutual recursion.
mp_obj_t DecodeCdrToSchema(cdr_reader_t* r,
                           const mp_rosette_schema_obj_t* schema);

/**
 * @brief Reads one scalar field value from a CDR reader.
 *
 * @param[in,out] r    CDR reader positioned at the next field.
 * @param[in]     kind ROSETTE_MPY_FIELD_* scalar kind.
 * @return Python object for the decoded value.
 */
mp_obj_t DecodeScalar(cdr_reader_t* r, uint8_t kind)
{
    switch (kind) {
    case ROSETTE_MPY_FIELD_BOOL:
        return cdr_read_bool(r) ? mp_const_true : mp_const_false;
    case ROSETTE_MPY_FIELD_INT8:
        return MP_OBJ_NEW_SMALL_INT(static_cast<int8_t>(cdr_read_u8(r)));
    case ROSETTE_MPY_FIELD_UINT8:
        return MP_OBJ_NEW_SMALL_INT(cdr_read_u8(r));
    case ROSETTE_MPY_FIELD_INT16:
        return MP_OBJ_NEW_SMALL_INT(static_cast<int16_t>(cdr_read_u16(r)));
    case ROSETTE_MPY_FIELD_UINT16:
        return MP_OBJ_NEW_SMALL_INT(cdr_read_u16(r));
    case ROSETTE_MPY_FIELD_INT32:
        return mp_obj_new_int(cdr_read_i32(r));
    case ROSETTE_MPY_FIELD_UINT32:
        return mp_obj_new_int_from_uint(cdr_read_u32(r));
    case ROSETTE_MPY_FIELD_INT64:
        return mp_obj_new_int_from_ll(cdr_read_i64(r));
    case ROSETTE_MPY_FIELD_UINT64:
        return mp_obj_new_int_from_ull(cdr_read_u64(r));
    case ROSETTE_MPY_FIELD_FLOAT32:
        return mp_obj_new_float(static_cast<mp_float_t>(cdr_read_f32(r)));
    case ROSETTE_MPY_FIELD_FLOAT64:
        return mp_obj_new_float(static_cast<mp_float_t>(cdr_read_f64(r)));
    case ROSETTE_MPY_FIELD_STRING: {
        // CDR string: uint32 length (including NUL), then bytes.
        uint32_t slen = cdr_read_u32(r);
        if (r->error || slen == 0u) return mp_obj_new_str("", 0);
        if (r->pos + slen > r->len) { r->error = true; return mp_obj_new_str("", 0); }
        // slen includes the trailing NUL byte; exclude it from the Python str.
        const char* s = reinterpret_cast<const char*>(r->buf + r->pos);
        size_t str_len = (slen > 0u) ? slen - 1u : 0u;
        r->pos += slen;
        return mp_obj_new_str(s, str_len);
    }
    default:
        return MP_OBJ_NEW_SMALL_INT(0);
    }
}

/**
 * @brief Decodes one CDR payload into a Python dict driven by schema descriptors.
 *
 * The returned dict uses qstr keys that match the schema field names, so
 * callers can wrap it in a Message object (via Schema.__call__) or use it
 * directly as a plain dict.
 *
 * @param[in,out] r      CDR reader positioned just after the encapsulation
 *                       header (or at the start of a nested sub-message body).
 * @param[in]     schema Parsed field descriptors for this message level.
 * @return Python dict containing all decoded fields.
 */
mp_obj_t DecodeCdrToSchema(cdr_reader_t* r,
                           const mp_rosette_schema_obj_t* schema)
{
    mp_obj_t dict = mp_obj_new_dict(static_cast<size_t>(schema->field_count));

    for (size_t i = 0u; i < schema->field_count && !r->error; ++i) {
        rosette_mpy_schema_field_t* field =
            const_cast<rosette_mpy_schema_field_t*>(&schema->fields[i]);

        mp_obj_t value = mp_const_none;

        if (field->kind == ROSETTE_MPY_FIELD_NESTED_MESSAGE) {
            ResolveNestedSchema(field);
            value = DecodeCdrToSchema(r, field->nested_schema);

        } else if (field->kind == ROSETTE_MPY_FIELD_FIXED_ARRAY ||
                   field->kind == ROSETTE_MPY_FIELD_DYNAMIC_ARRAY) {

            size_t count = 0u;
            if (field->kind == ROSETTE_MPY_FIELD_DYNAMIC_ARRAY) {
                count = static_cast<size_t>(cdr_read_u32(r));
            } else {
                count = static_cast<size_t>(field->element_count);
            }

            mp_obj_t list = mp_obj_new_list(0, nullptr);
            for (size_t ei = 0u; ei < count && !r->error; ++ei) {
                mp_obj_t elem = mp_const_none;
                if (field->element_kind == ROSETTE_MPY_FIELD_NESTED_MESSAGE) {
                    ResolveNestedSchema(field);
                    elem = DecodeCdrToSchema(r, field->nested_schema);
                } else {
                    elem = DecodeScalar(r, field->element_kind);
                }
                mp_obj_list_append(list, elem);
            }
            value = list;

        } else {
            value = DecodeScalar(r, field->kind);
        }

        mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(field->name), value);
    }

    return dict;
}

// ============================================================
// Schema Python object callbacks
// ============================================================

void SchemaPrint(const mp_print_t* print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    const auto* self = static_cast<mp_rosette_schema_obj_t*>(MP_OBJ_TO_PTR(self_in));
    mp_printf(print,
              "Schema(name=%q, fields=%u, dynamic=%s)",
              self->full_name,
              static_cast<unsigned>(self->field_count),
              self->is_dynamic ? "True" : "False");
}

/**
 * @brief Builds a plain dict from a schema call.
 *
 * Positional arguments map to fields in declaration order. Keyword arguments
 * map by name. Missing fields receive type-appropriate default values.
 * Returns a plain Python dict — the encode path accepts dicts directly.
 */
mp_obj_t SchemaCall(mp_obj_t self_in,
                    size_t n_args,
                    size_t n_kw,
                    const mp_obj_t* args)
{
    auto* schema = static_cast<mp_rosette_schema_obj_t*>(MP_OBJ_TO_PTR(self_in));
    if (n_args > schema->field_count) {
        mp_raise_TypeError(MP_ERROR_TEXT("too many positional fields"));
    }

    mp_obj_t dict = mp_obj_new_dict(schema->field_count);

    for (size_t i = 0; i < schema->field_count; ++i) {
        mp_obj_dict_store(dict,
                          MP_OBJ_NEW_QSTR(schema->fields[i].name),
                          DefaultFieldValue(schema->fields[i].kind));
    }
    for (size_t i = 0; i < n_args; ++i) {
        mp_obj_dict_store(dict,
                          MP_OBJ_NEW_QSTR(schema->fields[i].name),
                          args[i]);
    }
    for (size_t i = 0; i < n_kw; ++i) {
        mp_obj_t key   = args[n_args + (2u * i)];
        mp_obj_t value = args[n_args + (2u * i) + 1u];
        const qstr field_name = mp_obj_str_get_qstr(key);
        bool known = false;
        for (size_t f = 0; f < schema->field_count; ++f) {
            if (schema->fields[f].name == field_name) { known = true; break; }
        }
        if (!known) {
            mp_raise_msg(&mp_type_TypeError, MP_ERROR_TEXT("unknown message field"));
        }
        mp_obj_dict_store(dict, key, value);
    }
    return dict;
}

mp_obj_t SchemaMakeNew(const mp_obj_type_t* type,
                       size_t n_args, size_t n_kw,
                       const mp_obj_t* all_args)
{
    (void)type; (void)n_args; (void)n_kw; (void)all_args;
    mp_raise_TypeError(MP_ERROR_TEXT("Schema objects are created by rosette.schema()"));
}

}  // namespace

// ============================================================
// Public codec entry points (called from pubsub and backend)
// ============================================================

/**
 * @brief Encodes a Python message value into a caller-provided CDR buffer.
 *
 * The buffer must be large enough to hold the encoded message.  The encoder
 * writes the 4-byte CDR encapsulation header followed by the field sequence.
 * Returns the number of bytes written, or 0 on overflow.
 *
 * @param[in]  schema     Schema descriptor for the message to encode.
 * @param[in]  payload    Python dict, Message, or bare scalar.
 * @param[out] buf        Destination byte buffer.
 * @param[in]  buf_size   Capacity of `buf` in bytes.
 * @return Number of bytes written (>= 4), or 0 when the buffer overflowed.
 */
size_t SchemaCdrEncode(const mp_rosette_schema_obj_t* schema,
                       mp_obj_t payload,
                       uint8_t* buf,
                       size_t buf_size)
{
    cdr_writer_t w{};
    cdr_writer_init(&w, buf, buf_size);
    EncodeCdrFromSchema(&w, schema, payload);
    if (w.overflow) return 0u;
    return cdr_writer_size(&w);
}

/**
 * @brief Decodes one CDR payload into a Python Message object.
 *
 * Reads the 4-byte encapsulation header to determine endianness, then decodes
 * each schema field in order into a new Message backed by the schema.
 *
 * @param[in] schema  Schema descriptor for the expected message type.
 * @param[in] buf     Raw CDR bytes (including encapsulation header).
 * @param[in] length  Number of valid bytes in `buf`.
 * @return New Message object, or None when the payload is malformed.
 */
mp_obj_t SchemaCdrDecode(const mp_rosette_schema_obj_t* schema,
                         const uint8_t* buf,
                         size_t length)
{
    cdr_reader_t r{};
    if (!cdr_reader_init(&r, buf, length)) {
        return mp_const_none;
    }
    mp_obj_t dict = DecodeCdrToSchema(&r, schema);
    if (r.error) {
        return mp_const_none;
    }
    return dict;
}

}  // namespace rosette::bindings::micropython

// ============================================================
// Module-level rosette.schema() factory
// ============================================================

extern "C" {

/**
 * @brief Creates a Schema object from an optional name and `.msg` text.
 *
 * Calling conventions accepted:
 *   rosette.schema("pkg/Name", "field_type field_name\n...")
 *   rosette.schema("pkg/Name")          # statically registered types only
 *   rosette.schema("field_type field_name\n...")  # anonymous dynamic schema
 *
 * When the name matches a statically-linked ROSIDL type the schema is marked
 * non-dynamic (is_dynamic=False) and carries hard-coded field descriptors so
 * the backend can use the native generated-message publish path.  All other
 * schemas are marked dynamic (is_dynamic=True) and use the generic CDR codec
 * with rcl_publish_serialized_message.
 */
mp_obj_t rosette_schema_make_from_text(size_t n_args,
                                       const mp_obj_t* args)
{
    const char* first = mp_obj_str_get_str(args[0]);
    const bool first_looks_like_msg =
        strchr(first, '\n') != nullptr || strchr(first, ' ') != nullptr ||
        strchr(first, '\t') != nullptr;
    const char* full_name    = first_looks_like_msg ? "" : first;
    const char* message_text = nullptr;
    if (first_looks_like_msg) {
        message_text = first;
    } else if (n_args >= 2) {
        message_text = mp_obj_str_get_str(args[1]);
    }

    auto* self = mp_obj_malloc(
        mp_rosette_schema_obj_t,
        reinterpret_cast<const mp_obj_type_t*>(&rosette_schema_type));
    self->full_name = qstr_from_str(full_name);
    rosette::bindings::micropython::SplitMessageName(
        full_name, &self->package_name, &self->short_name);
    self->fields              = nullptr;
    self->field_count         = 0u;
    self->max_serialized_size = 0u;
    self->is_dynamic          = true;
    self->message_kind =
        rosette::bindings::micropython::ResolveMessageKind(full_name);

    // Statically-registered types get hard-coded field descriptors so the
    // existing native-publish path continues to work without a .msg text.
    if (self->message_kind == ROSETTE_MPY_MESSAGE_STD_MSGS_FLOAT32) {
        self->is_dynamic   = false;
        self->fields       = m_new(rosette_mpy_schema_field_t, 1);
        self->fields[0]    = {};
        self->fields[0].name         = MP_QSTR_data;
        self->fields[0].kind         = ROSETTE_MPY_FIELD_FLOAT32;
        self->fields[0].element_kind = ROSETTE_MPY_FIELD_FLOAT32;
        self->field_count            = 1u;
        self->max_serialized_size    = sizeof(float) + 4u;  // + CDR header

    } else if (message_text != nullptr) {
        // Generic dynamic schema parsed from .msg text.
        rosette::bindings::micropython::ParseMessageText(self, message_text);
        self->is_dynamic = true;
    }

    return MP_OBJ_FROM_PTR(self);
}

const mp_obj_full_type_t rosette_schema_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_NONE,
    MP_QSTR_Schema,
    1,  // make_new
    2,  // print
    3,  // call
    0,  // unary_op
    0,  // binary_op
    0,  // attr
    0,  // subscr
    0,  // iter
    0,  // buffer
    0,  // protocol
    0,  // parent
    0,  // locals_dict
    {
        reinterpret_cast<const void*>(rosette::bindings::micropython::SchemaMakeNew),
        reinterpret_cast<const void*>(rosette::bindings::micropython::SchemaPrint),
        reinterpret_cast<const void*>(rosette::bindings::micropython::SchemaCall),
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
    },
};

}  // extern "C"
