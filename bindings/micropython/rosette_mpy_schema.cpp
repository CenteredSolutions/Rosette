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
 */

#include "rosette_mpy_backend.hpp"

extern "C" {
#include "py/runtime.h"
}

#include <cstring>

namespace rosette::bindings::micropython {

namespace {

struct TextSpan {
    const char* data = nullptr;
    size_t length = 0u;
};

/**
 * @brief Trims ASCII whitespace from both ends of a text span.
 *
 * The `.msg` grammar used by ROS interface files is line-oriented. The parser
 * only needs simple ASCII trimming around type and field tokens before it
 * decides whether a line is a comment, constant, or field declaration.
 *
 * @param[in] span Source text span to trim.
 * @return Trimmed subspan that still points into the original string.
 */
TextSpan Trim(TextSpan span) noexcept
{
    while (span.length != 0u &&
           (span.data[0] == ' ' || span.data[0] == '\t' ||
            span.data[0] == '\r')) {
        ++span.data;
        --span.length;
    }
    while (span.length != 0u) {
        const char c = span.data[span.length - 1u];
        if (c != ' ' && c != '\t' && c != '\r') {
            break;
        }
        --span.length;
    }
    return span;
}

/**
 * @brief Returns whether a span equals one ASCII token.
 *
 * @param[in] span Parsed token span.
 * @param[in] token Null-terminated token literal to compare against.
 * @return `true` when both texts contain the same bytes.
 */
bool SpanEquals(TextSpan span, const char* token) noexcept
{
    return token != nullptr && std::strlen(token) == span.length &&
           std::strncmp(span.data, token, span.length) == 0;
}

/**
 * @brief Converts a ROS primitive type token into a field kind.
 *
 * The first parser slice handles scalar primitive fields and strings. Array
 * and nested message support can layer on top of the same descriptor once the
 * generated-message registry supports more native conversion targets.
 *
 * @param[in] type_token Field type token from a `.msg` line.
 * @param[out] kind Field kind consumed by message construction and codecs.
 * @return `true` when the token names a supported primitive field type.
 */
bool ResolveFieldKind(TextSpan type_token, uint8_t* kind) noexcept
{
    if (SpanEquals(type_token, "bool")) {
        *kind = ROSETTE_MPY_FIELD_BOOL;
    } else if (SpanEquals(type_token, "int8")) {
        *kind = ROSETTE_MPY_FIELD_INT8;
    } else if (SpanEquals(type_token, "uint8") || SpanEquals(type_token, "byte")) {
        *kind = ROSETTE_MPY_FIELD_UINT8;
    } else if (SpanEquals(type_token, "int16")) {
        *kind = ROSETTE_MPY_FIELD_INT16;
    } else if (SpanEquals(type_token, "uint16")) {
        *kind = ROSETTE_MPY_FIELD_UINT16;
    } else if (SpanEquals(type_token, "int32")) {
        *kind = ROSETTE_MPY_FIELD_INT32;
    } else if (SpanEquals(type_token, "uint32")) {
        *kind = ROSETTE_MPY_FIELD_UINT32;
    } else if (SpanEquals(type_token, "int64")) {
        *kind = ROSETTE_MPY_FIELD_INT64;
    } else if (SpanEquals(type_token, "uint64")) {
        *kind = ROSETTE_MPY_FIELD_UINT64;
    } else if (SpanEquals(type_token, "float32")) {
        *kind = ROSETTE_MPY_FIELD_FLOAT32;
    } else if (SpanEquals(type_token, "float64")) {
        *kind = ROSETTE_MPY_FIELD_FLOAT64;
    } else if (SpanEquals(type_token, "string")) {
        *kind = ROSETTE_MPY_FIELD_STRING;
    } else {
        return false;
    }
    return true;
}

/**
 * @brief Returns the default Python value for one parsed field kind.
 *
 * Message construction fills every declared field so attribute access and
 * backend conversion never need to distinguish missing values from explicit
 * zero-like values.
 *
 * @param[in] kind Parsed field kind.
 * @return Default MicroPython object for the field.
 */
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
    default:
        return MP_OBJ_NEW_SMALL_INT(0);
    }
}

/**
 * @brief Counts parseable field declarations in one `.msg` text block.
 *
 * The parser ignores blank lines, comments, and constants. Unsupported field
 * types are counted here and rejected during the fill pass so the error points
 * at schema construction rather than deferred publication.
 *
 * @param[in] text Null-terminated `.msg` source text.
 * @return Number of non-comment field declaration lines.
 */
size_t CountFieldLines(const char* text) noexcept
{
    size_t count = 0u;
    const char* line = text;
    while (line != nullptr && *line != '\0') {
        const char* end = std::strchr(line, '\n');
        const size_t length = end == nullptr
                              ? std::strlen(line)
                              : static_cast<size_t>(end - line);
        TextSpan span = Trim({line, length});
        const char* comment = static_cast<const char*>(
            std::memchr(span.data, '#', span.length));
        if (comment != nullptr) {
            span.length = static_cast<size_t>(comment - span.data);
            span = Trim(span);
        }
        if (span.length != 0u &&
            std::memchr(span.data, '=', span.length) == nullptr) {
            ++count;
        }
        line = end == nullptr ? nullptr : end + 1;
    }
    return count;
}

/**
 * @brief Parses `.msg` text into schema field descriptors.
 *
 * Each supported declaration has the shape `<primitive-type> <field-name>`.
 * Comments, blank lines, and constants are skipped. The descriptors are stored
 * in declaration order, which also defines positional construction order for
 * `Schema(...)`.
 *
 * @param[in,out] schema Schema object that receives parsed descriptors.
 * @param[in] text Null-terminated `.msg` source text.
 */
void ParseMessageText(mp_rosette_schema_obj_t* schema, const char* text)
{
    if (schema == nullptr || text == nullptr) {
        return;
    }

    const size_t field_count = CountFieldLines(text);
    if (field_count == 0u) {
        return;
    }

    schema->fields = m_new(rosette_mpy_schema_field_t, field_count);
    schema->field_count = field_count;

    size_t index = 0u;
    const char* line = text;
    while (line != nullptr && *line != '\0' && index < field_count) {
        const char* end = std::strchr(line, '\n');
        const size_t length = end == nullptr
                              ? std::strlen(line)
                              : static_cast<size_t>(end - line);
        TextSpan span = Trim({line, length});
        const char* comment = static_cast<const char*>(
            std::memchr(span.data, '#', span.length));
        if (comment != nullptr) {
            span.length = static_cast<size_t>(comment - span.data);
            span = Trim(span);
        }
        if (span.length == 0u ||
            std::memchr(span.data, '=', span.length) != nullptr) {
            line = end == nullptr ? nullptr : end + 1;
            continue;
        }

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

        uint8_t kind = ROSETTE_MPY_FIELD_BOOL;
        if (!ResolveFieldKind(type_token, &kind)) {
            mp_raise_ValueError(MP_ERROR_TEXT("unsupported .msg field type"));
        }

        schema->fields[index] = {};
        schema->fields[index].name = qstr_from_strn(name_token.data, name_token.length);
        schema->fields[index].kind = kind;
        schema->fields[index].element_kind = kind;
        ++index;
        line = end == nullptr ? nullptr : end + 1;
    }
}

/**
 * @brief Resolves the message kind supported by the generated backend registry.
 *
 * The MicroPython binding can only publish or subscribe to message types whose
 * generated ROSIDL type support is linked into `libmicroros.a`. This registry
 * starts with `std_msgs/Float32` because it provides the shortest useful path
 * for numeric telemetry examples such as MCU temperature.
 *
 * @param[in] full_name Normalized ROS message name such as `std_msgs/Float32`.
 * @return Binding message-kind identifier used by publisher and subscription
 * factories.
 */
uint8_t ResolveMessageKind(const char* full_name) noexcept
{
    if (full_name == nullptr) {
        return ROSETTE_MPY_MESSAGE_UNSUPPORTED;
    }
    if (std::strcmp(full_name, "std_msgs/Float32") == 0) {
        return ROSETTE_MPY_MESSAGE_STD_MSGS_FLOAT32;
    }
    return ROSETTE_MPY_MESSAGE_UNSUPPORTED;
}

/**
 * @brief Splits a ROS message name into package and short-name qstrs.
 *
 * The schema object stores both the full type name and its display pieces so
 * later diagnostics can report unsupported generated-message types clearly.
 * Names without a slash are accepted as unsupported schemas rather than being
 * rejected at construction time.
 *
 * @param[in] full_name Full ROS message name from Python.
 * @param[out] package_name Package component before the slash.
 * @param[out] short_name Message component after the slash.
 */
void SplitMessageName(const char* full_name,
                      qstr* package_name,
                      qstr* short_name)
{
    const char* slash = full_name == nullptr ? nullptr : std::strchr(full_name, '/');
    if (slash == nullptr) {
        *package_name = qstr_from_str("");
        *short_name = qstr_from_str(full_name == nullptr ? "" : full_name);
        return;
    }

    *package_name = qstr_from_strn(full_name, static_cast<size_t>(slash - full_name));
    *short_name = qstr_from_str(slash + 1);
}

/**
 * @brief Displays a compact representation of a runtime schema object.
 *
 * @param[in] print MicroPython print destination.
 * @param[in] self_in Schema object to display.
 * @param[in] kind Print behavior requested by MicroPython.
 */
void SchemaPrint(const mp_print_t* print,
                 mp_obj_t self_in,
                 mp_print_kind_t kind)
{
    (void)kind;
    const auto* self = static_cast<mp_rosette_schema_obj_t*>(MP_OBJ_TO_PTR(self_in));
    mp_printf(print,
              "Schema(name=%q, fields=%u)",
              self->full_name,
              static_cast<unsigned>(self->field_count));
}

/**
 * @brief Displays one schema-created message instance.
 *
 * @param[in] print MicroPython print destination.
 * @param[in] self_in Message object to display.
 * @param[in] kind Print behavior requested by MicroPython.
 */
void MessagePrint(const mp_print_t* print,
                  mp_obj_t self_in,
                  mp_print_kind_t kind)
{
    (void)kind;
    const auto* self = static_cast<mp_rosette_message_obj_t*>(MP_OBJ_TO_PTR(self_in));
    const auto* schema =
        static_cast<mp_rosette_schema_obj_t*>(MP_OBJ_TO_PTR(self->schema_obj));
    mp_printf(print, "Message(schema=%q, values=", schema->full_name);
    mp_obj_print_helper(print, self->values_obj, PRINT_REPR);
    mp_printf(print, ")");
}

/**
 * @brief Loads or stores a field value on a schema-created message.
 *
 * Attribute access delegates to the message's field dictionary after checking
 * the parsed schema. Unknown attributes are ignored so MicroPython can raise
 * its normal `AttributeError`.
 *
 * @param[in,out] self_in Message object whose field should be accessed.
 * @param[in] attr Field qstr requested by the runtime.
 * @param[in,out] dest MicroPython attribute operation slots.
 */
void MessageAttr(mp_obj_t self_in, qstr attr, mp_obj_t* dest)
{
    auto* self = static_cast<mp_rosette_message_obj_t*>(MP_OBJ_TO_PTR(self_in));
    auto* schema =
        static_cast<mp_rosette_schema_obj_t*>(MP_OBJ_TO_PTR(self->schema_obj));
    for (size_t i = 0; i < schema->field_count; ++i) {
        if (schema->fields[i].name != attr) {
            continue;
        }
        mp_obj_t key = MP_OBJ_NEW_QSTR(attr);
        if (dest[0] == MP_OBJ_NULL) {
            dest[0] = mp_obj_dict_get(self->values_obj, key);
        } else {
            mp_obj_dict_store(self->values_obj, key, dest[1]);
            dest[0] = MP_OBJ_NULL;
        }
        return;
    }
}

/**
 * @brief Builds one message object from a schema call.
 *
 * Positional arguments map to fields in declaration order. Keyword arguments
 * map by field name. Any field not supplied by the caller receives the
 * default value associated with its field kind.
 *
 * @param[in] self_in Schema object being called.
 * @param[in] n_args Positional argument count.
 * @param[in] n_kw Keyword argument pair count.
 * @param[in] args Positional values followed by keyword key/value pairs.
 * @return New message instance backed by the schema.
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

    auto* message = mp_obj_malloc(
        mp_rosette_message_obj_t,
        reinterpret_cast<const mp_obj_type_t*>(&rosette_message_type));
    message->schema_obj = self_in;
    message->values_obj = mp_obj_new_dict(schema->field_count);

    for (size_t i = 0; i < schema->field_count; ++i) {
        mp_obj_dict_store(message->values_obj,
                          MP_OBJ_NEW_QSTR(schema->fields[i].name),
                          DefaultFieldValue(schema->fields[i].kind));
    }
    for (size_t i = 0; i < n_args; ++i) {
        mp_obj_dict_store(message->values_obj,
                          MP_OBJ_NEW_QSTR(schema->fields[i].name),
                          args[i]);
    }
    for (size_t i = 0; i < n_kw; ++i) {
        mp_obj_t key = args[n_args + (2u * i)];
        mp_obj_t value = args[n_args + (2u * i) + 1u];
        const qstr field_name = mp_obj_str_get_qstr(key);
        bool known = false;
        for (size_t field = 0; field < schema->field_count; ++field) {
            if (schema->fields[field].name == field_name) {
                known = true;
                break;
            }
        }
        if (!known) {
            mp_raise_msg(&mp_type_TypeError, MP_ERROR_TEXT("unknown message field"));
        }
        mp_obj_dict_store(message->values_obj, key, value);
    }
    return MP_OBJ_FROM_PTR(message);
}

/**
 * @brief Rejects direct Python `Schema(...)` construction.
 *
 * Schemas are created through the module-level `rosette.schema(...)` helper so
 * the parser can normalize the message name and the `.msg` text together.
 */
mp_obj_t SchemaMakeNew(const mp_obj_type_t* type,
                       size_t n_args,
                       size_t n_kw,
                       const mp_obj_t* all_args)
{
    (void)type;
    (void)n_args;
    (void)n_kw;
    (void)all_args;
    mp_raise_TypeError(MP_ERROR_TEXT("Schema objects are created by rosette.schema()"));
}

}  // namespace

}  // namespace rosette::bindings::micropython

extern "C" {

/**
 * @brief Creates a minimal runtime schema object from `.msg` text.
 *
 * The initial compile-and-smoke path keeps schema parsing deliberately tiny:
 * it records the fully qualified name and retains no parsed field descriptors
 * yet. That shape is enough to let the higher-level object model compile and
 * gives the next implementation step one stable place to extend.
 *
 * @param[in] n_args Number of positional arguments.
 * @param[in] args Positional argument array.
 * @return Newly allocated `Schema` object.
 */
mp_obj_t rosette_schema_make_from_text(size_t n_args,
                                       const mp_obj_t* args)
{
    const char* first = mp_obj_str_get_str(args[0]);
    const bool first_looks_like_msg =
        std::strchr(first, '\n') != nullptr || std::strchr(first, ' ') != nullptr ||
        std::strchr(first, '\t') != nullptr;
    const char* full_name = first_looks_like_msg ? "" : first;
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
    self->fields = nullptr;
    self->field_count = 0u;
    self->max_serialized_size = 0u;
    self->message_kind =
        rosette::bindings::micropython::ResolveMessageKind(full_name);

    if (self->message_kind == ROSETTE_MPY_MESSAGE_STD_MSGS_FLOAT32) {
        self->fields = m_new(rosette_mpy_schema_field_t, 1);
        self->fields[0] = {};
        self->fields[0].name = MP_QSTR_data;
        self->fields[0].kind = ROSETTE_MPY_FIELD_FLOAT32;
        self->fields[0].element_kind = ROSETTE_MPY_FIELD_FLOAT32;
        self->field_count = 1u;
        self->max_serialized_size = sizeof(float);
    } else if (message_text != nullptr) {
        rosette::bindings::micropython::ParseMessageText(self, message_text);
    }

    return MP_OBJ_FROM_PTR(self);
}

const mp_obj_full_type_t rosette_message_type = {
    { &mp_type_type },
    MP_TYPE_FLAG_NONE,
    MP_QSTR_Message,
    0,  // make_new
    1,  // print
    0,  // call
    0,  // unary_op
    0,  // binary_op
    2,  // attr
    0,  // subscr
    0,  // iter
    0,  // buffer
    0,  // protocol
    0,  // parent
    0,  // locals_dict
    {
        reinterpret_cast<const void*>(rosette::bindings::micropython::MessagePrint),
        reinterpret_cast<const void*>(rosette::bindings::micropython::MessageAttr),
        nullptr,
        nullptr,
        nullptr,
    },
};

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
