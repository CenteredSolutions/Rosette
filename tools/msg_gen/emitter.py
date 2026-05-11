from __future__ import annotations

from dataclasses import dataclass

from .model import (
    ConstantDef,
    FieldDef,
    MessageDef,
    PRIMITIVES,
    SIGNED_CAST,
    ServiceDef,
    TypeToken,
)


@dataclass(frozen=True)
class GeneratorOptions:
    max_string_len: int = 256
    max_dynamic_len: int = 64


def _camel_to_upper(name: str) -> str:
    out: list[str] = []
    for index, char in enumerate(name):
        if index and char.isupper() and (name[index - 1].islower() or name[index - 1].isdigit()):
            out.append("_")
        out.append(char.upper())
    return "".join(out)


def _guard(stem: str) -> str:
    package, name = stem.split("__", 1)
    return f"ROSETTE_GENERATED_{package.upper()}_{_camel_to_upper(name)}_H"


def _macro_prefix(package: str, name: str) -> str:
    return f"{package.upper()}_{_camel_to_upper(name)}"


def _struct_name(package: str, name: str) -> str:
    return f"{package}_{name}_t"


def _include_name(token: TypeToken, local_package: str) -> str:
    package = token.package or local_package
    return f"{package}__{token.name}.h"


def _dds_message_type_name(package: str, name: str) -> str:
    return f"{package}::msg::dds_::{name}_"


def _dds_service_type_name(package: str, name: str, suffix: str) -> str:
    return f"{package}::srv::dds_::{name}_{suffix}_"


def _collect_includes(message: MessageDef) -> list[str]:
    seen: set[str] = set()
    includes: list[str] = []
    for field in message.fields:
        token = field.type_token
        if token.is_primitive and not token.is_nested:
            continue
        if token.package is None and token.name in {"string", "wstring"}:
            continue
        include = _include_name(token, message.package)
        if include not in seen:
            seen.add(include)
            includes.append(include)
    return includes


def _is_builtin_time(message: MessageDef) -> bool:
    return message.package == "builtin_interfaces" and message.name == "Time"


def _is_std_header(message: MessageDef) -> bool:
    return message.package == "std_msgs" and message.name == "Header"


def _element_c_type(field: FieldDef, local_package: str) -> str:
    token = field.type_token
    if token.package is None and token.name in {"string", "wstring"}:
        return "char"
    if token.package is None and token.name in PRIMITIVES:
        return PRIMITIVES[token.name][0]
    return _struct_name(token.package or local_package, token.name)


def _slice_type(message: MessageDef, field: FieldDef) -> str:
    return f"{message.package}_{message.name}_{field.name}_slice_t"


def _max_expr_for_token(token: TypeToken, local_package: str, options: GeneratorOptions) -> str:
    if token.package is None and token.name in {"string", "wstring"}:
        limit = token.bounded_string + 1 if token.bounded_string else options.max_string_len
        return f"(4u + {limit}u + 3u)"

    if token.package is None and token.name in PRIMITIVES:
        size = PRIMITIVES[token.name][3]
        item = f"{size}u"
    else:
        package = token.package or local_package
        item = f"{_macro_prefix(package, token.name)}_MAX_BODY_SERIALIZED_SIZE"

    if token.array_kind == "scalar":
        return f"(3u + {item})"
    if token.array_kind == "fixed":
        return f"({token.array_bound}u * (3u + {item}))"
    count = token.array_bound if token.array_kind == "bounded" else options.max_dynamic_len
    return f"(4u + ({count}u * (3u + {item})))"


def _emit_field_decl(lines: list[str], message: MessageDef, field: FieldDef, options: GeneratorOptions) -> None:
    token = field.type_token
    ctype = _element_c_type(field, message.package)
    comment = f"  /* {field.comment} */" if field.comment else ""
    if token.bounded_string:
        lines.append(f"    char {field.name}[{token.bounded_string + 1}];{comment}")
        return
    if token.package is None and token.name in {"string", "wstring"}:
        if token.array_kind == "fixed":
            lines.append(
                f"    char {field.name}[{token.array_bound}][{options.max_string_len}];{comment}"
            )
            return
        if token.array_kind == "bounded":
            lines.append(
                f"    char {field.name}[{token.array_bound}][{options.max_string_len}];{comment}"
            )
            lines.append(f"    uint32_t {field.name}_length;")
            return
        if token.array_kind == "dynamic":
            lines.append(f"    {_slice_type(message, field)} {field.name};{comment}")
            return
        lines.append(
            f"    char {field.name}[{options.max_string_len}];"
            f"  /* truncated at {options.max_string_len - 1} chars */{comment}"
        )
        return
    if token.array_kind == "fixed":
        lines.append(f"    {ctype} {field.name}[{token.array_bound}];{comment}")
        return
    if token.array_kind == "dynamic":
        lines.append(f"    {_slice_type(message, field)} {field.name};{comment}")
        return
    if token.array_kind == "bounded":
        lines.append(f"    {ctype} {field.name}[{token.array_bound}];{comment}")
        lines.append(f"    uint32_t {field.name}_length;")
        return
    lines.append(f"    {ctype} {field.name};{comment}")


def _write_primitive(field: FieldDef, accessor: str) -> str:
    fn = PRIMITIVES[field.type_name][1]
    cast = SIGNED_CAST.get(field.type_name, "")
    return f"{fn}(_w, {cast}{accessor});"


def _read_primitive(field: FieldDef, accessor: str) -> str:
    fn = PRIMITIVES[field.type_name][2]
    ctype = PRIMITIVES[field.type_name][0]
    if field.type_name in SIGNED_CAST:
        return f"{accessor} = ({ctype}){fn}(_r);"
    return f"{accessor} = {fn}(_r);"


def _emit_encode_field(lines: list[str], message: MessageDef, field: FieldDef, options: GeneratorOptions) -> None:
    token = field.type_token
    accessor = f"_msg->{field.name}"
    if token.bounded_string or (token.package is None and token.name in {"string", "wstring"}):
        lines.append(f"    cdr_write_string(_w, {accessor});")
        return
    if token.package is None and token.name in PRIMITIVES and token.array_kind == "scalar":
        lines.append(f"    {_write_primitive(field, accessor)}")
        return
    if token.array_kind in {"dynamic", "bounded"}:
        length_expr = f"{accessor}.length" if token.array_kind == "dynamic" else f"_msg->{field.name}_length"
        lines.append(f"    cdr_write_u32(_w, {length_expr});")
    if token.array_kind == "scalar":
        if token.package is None and token.name in PRIMITIVES:
            lines.append(f"    {_write_primitive(field, accessor)}")
        else:
            nested = _struct_name(token.package or message.package, token.name).removesuffix("_t")
            lines.append(f"    encode_{nested}_body(_w, &{accessor});")
        return

    count_expr = (
        f"{accessor}.length"
        if token.array_kind == "dynamic"
        else (f"_msg->{field.name}_length" if token.array_kind == "bounded" else f"{token.array_bound}u")
    )
    lines.append(f"    for (uint32_t _i_{field.name} = 0; _i_{field.name} < {count_expr}; ++_i_{field.name}) {{")
    element = (
        f"{accessor}.data[_i_{field.name}]"
        if token.array_kind == "dynamic"
        else f"{accessor}[_i_{field.name}]"
    )
    if token.package is None and token.name in PRIMITIVES:
        pseudo = FieldDef(type_token=TypeToken(None, token.name), name=field.name)
        lines.append(f"        {_write_primitive(pseudo, element)}")
    elif token.package is None and token.name in {"string", "wstring"}:
        lines.append(f"        cdr_write_string(_w, {element});")
    else:
        nested = _struct_name(token.package or message.package, token.name).removesuffix("_t")
        lines.append(f"        encode_{nested}_body(_w, &{element});")
    lines.append("    }")


def _emit_decode_field(lines: list[str], message: MessageDef, field: FieldDef, options: GeneratorOptions) -> None:
    token = field.type_token
    accessor = f"_msg->{field.name}"
    if token.bounded_string:
        lines.append(f"    cdr_read_string(_r, {accessor}, {token.bounded_string + 1});")
        return
    if token.package is None and token.name in {"string", "wstring"}:
        lines.append(f"    cdr_read_string(_r, {accessor}, {options.max_string_len});")
        return
    if token.package is None and token.name in PRIMITIVES and token.array_kind == "scalar":
        lines.append(f"    {_read_primitive(field, accessor)}")
        return
    if token.array_kind in {"dynamic", "bounded"}:
        lines.append(f"    uint32_t _count_{field.name} = cdr_read_u32(_r);")
        if token.array_kind == "dynamic":
            lines.append(f"    {accessor}.length = 0u;")
            lines.append(f"    for (uint32_t _i_{field.name} = 0; _i_{field.name} < _count_{field.name}; ++_i_{field.name}) {{")
            target = f"{accessor}.data[_i_{field.name}]"
        else:
            lines.append(
                f"    _msg->{field.name}_length = (_count_{field.name} <= {token.array_bound}u) ? "
                f"_count_{field.name} : {token.array_bound}u;"
            )
            lines.append(f"    for (uint32_t _i_{field.name} = 0; _i_{field.name} < _count_{field.name}; ++_i_{field.name}) {{")
            target = f"{accessor}[_i_{field.name}]"
        if token.package is None and token.name in PRIMITIVES:
            pseudo = FieldDef(type_token=TypeToken(None, token.name), name=field.name)
            if token.array_kind == "dynamic":
                lines.append(f"        if (_i_{field.name} < {accessor}.capacity) {{")
                lines.append(f"            {_read_primitive(pseudo, target)}")
                lines.append(f"            {accessor}.length = _i_{field.name} + 1u;")
                lines.append("        } else {")
                lines.append(f"            (void){PRIMITIVES[token.name][2]}(_r);")
                lines.append("        }")
            else:
                lines.append(f"        if (_i_{field.name} < {token.array_bound}u) {{")
                lines.append(f"            {_read_primitive(pseudo, target)}")
                lines.append("        } else {")
                lines.append(f"            (void){PRIMITIVES[token.name][2]}(_r);")
                lines.append("        }")
        elif token.package is None and token.name in {"string", "wstring"}:
            if token.array_kind == "dynamic":
                lines.append(f"        if (_i_{field.name} < {accessor}.capacity) {{")
                lines.append(f"            cdr_read_string(_r, {target}, {options.max_string_len});")
                lines.append(f"            {accessor}.length = _i_{field.name} + 1u;")
                lines.append("        } else {")
                lines.append(f"            char _discard[{options.max_string_len}];")
                lines.append(f"            cdr_read_string(_r, _discard, {options.max_string_len});")
                lines.append("        }")
            else:
                lines.append(f"        if (_i_{field.name} < {token.array_bound}u) cdr_read_string(_r, {target}, {options.max_string_len});")
                lines.append("        else {")
                lines.append(f"            char _discard[{options.max_string_len}];")
                lines.append(f"            cdr_read_string(_r, _discard, {options.max_string_len});")
                lines.append("        }")
        else:
            nested = _struct_name(token.package or message.package, token.name).removesuffix("_t")
            if token.array_kind == "dynamic":
                lines.append(f"        if (_i_{field.name} < {accessor}.capacity) {{")
                lines.append(f"            decode_{nested}_body(_r, &{target});")
                lines.append(f"            {accessor}.length = _i_{field.name} + 1u;")
                lines.append("        } else {")
                lines.append(f"            {_struct_name(token.package or message.package, token.name)} _discard{{}};")
                lines.append(f"            decode_{nested}_body(_r, &_discard);")
                lines.append("        }")
            else:
                lines.append(f"        if (_i_{field.name} < {token.array_bound}u) decode_{nested}_body(_r, &{target});")
                lines.append(f"        else {{ {_struct_name(token.package or message.package, token.name)} _discard{{}}; decode_{nested}_body(_r, &_discard); }}")
        lines.append("    }")
        return

    if token.array_kind == "fixed":
        lines.append(f"    for (uint32_t _i_{field.name} = 0; _i_{field.name} < {token.array_bound}u; ++_i_{field.name}) {{")
        target = f"{accessor}[_i_{field.name}]"
        if token.package is None and token.name in PRIMITIVES:
            pseudo = FieldDef(type_token=TypeToken(None, token.name), name=field.name)
            lines.append(f"        {_read_primitive(pseudo, target)}")
        else:
            nested = _struct_name(token.package or message.package, token.name).removesuffix("_t")
            lines.append(f"        decode_{nested}_body(_r, &{target});")
        lines.append("    }")
        return

    nested = _struct_name(token.package or message.package, token.name).removesuffix("_t")
    lines.append(f"    decode_{nested}_body(_r, &{accessor});")


def _emit_constants(lines: list[str], message: MessageDef) -> None:
    if not message.constants:
        return
    lines.append("")
    lines.append("/* Message constants */")
    for constant in message.constants:
        lines.append(
            f"#define {_macro_prefix(message.package, message.name)}_{constant.name.upper()} ({constant.value})"
        )


def _emit_message_header(lines: list[str], message: MessageDef, options: GeneratorOptions) -> None:
    includes = _collect_includes(message)
    stem = message.stem
    guard = _guard(stem)
    macro = _macro_prefix(message.package, message.name)
    struct_name = _struct_name(message.package, message.name)

    lines.extend([
        "#pragma once",
        f"#ifndef {guard}",
        f"#define {guard}",
        "",
        "#include <stdbool.h>",
        "#include <stddef.h>",
        "#include <stdint.h>",
        '#include "rosette/ros/cdr.h"',
        "",
    ])
    for include in includes:
        lines.append(f'#include "{include}"')
    if includes:
        lines.append("")

    lines.append(f'#define {macro}_TYPE_NAME "{_dds_message_type_name(message.package, message.name)}"')
    lines.append(f'#define {macro}_TOPIC_PREFIX "rt/"')
    body_exprs = [_max_expr_for_token(field.type_token, message.package, options) for field in message.fields]
    body_expr = " + ".join(body_exprs) if body_exprs else "0u"
    lines.append(f"#define {macro}_MAX_BODY_SERIALIZED_SIZE ({body_expr})")
    lines.append(f"#define {macro}_MAX_SERIALIZED_SIZE (4u + {macro}_MAX_BODY_SERIALIZED_SIZE)")
    _emit_constants(lines, message)

    dynamic_fields = [field for field in message.fields if field.type_token.array_kind == "dynamic"]
    if dynamic_fields:
        lines.append("")
        for field in dynamic_fields:
            lines.append("typedef struct {")
            token = field.type_token
            if token.package is None and token.name in {"string", "wstring"}:
                lines.append(f"    char (*data)[{options.max_string_len}];")
            else:
                lines.append(f"    {_element_c_type(field, message.package)} *data;")
            lines.append("    uint32_t length;")
            lines.append("    uint32_t capacity;")
            lines.append(f"}} {_slice_type(message, field)};")

    lines.extend(["", "typedef struct {"])
    for field in message.fields:
        _emit_field_decl(lines, message, field, options)
    lines.extend([f"}} {struct_name};", ""])

    if _is_builtin_time(message):
        lines.extend([
            f"static inline {struct_name} builtin_interfaces_Time_now(void) {{",
            f"    {struct_name} _msg{{}};",
            "    cdr_time_now(&_msg.sec, &_msg.nanosec);",
            "    return _msg;",
            "}",
            "",
            f"static inline bool builtin_interfaces_Time_is_zero(const {struct_name}* _msg) {{",
            "    return _msg->sec == 0 && _msg->nanosec == 0u;",
            "}",
            "",
        ])
    elif _is_std_header(message):
        lines.extend([
            f"static inline bool std_msgs_Header_has_stamp(const {struct_name}* _msg) {{",
            "    return _msg->stamp.sec != 0 || _msg->stamp.nanosec != 0u;",
            "}",
            "",
            f"static inline void std_msgs_Header_apply_default_stamp({struct_name}* _msg) {{",
            "    if (!std_msgs_Header_has_stamp(_msg)) {",
            "        _msg->stamp = builtin_interfaces_Time_now();",
            "    }",
            "}",
            "",
            f"static inline {struct_name} std_msgs_Header_make(const char* _frame_id) {{",
            f"    {struct_name} _msg{{}};",
            "    std_msgs_Header_apply_default_stamp(&_msg);",
            "    if (_frame_id) {",
            "        size_t _i = 0u;",
            "        for (; _frame_id[_i] != '\\0' && _i + 1u < sizeof(_msg.frame_id); ++_i) {",
            "            _msg.frame_id[_i] = _frame_id[_i];",
            "        }",
            "        _msg.frame_id[_i] = '\\0';",
            "    }",
            "    return _msg;",
            "}",
            "",
        ])

    short = struct_name.removesuffix("_t")
    lines.extend([
        f"static inline void encode_{short}_body(cdr_writer_t* _w, const {struct_name}* _msg) {{"
    ])
    if _is_builtin_time(message):
        lines.extend([
            f"    const {struct_name} _resolved = builtin_interfaces_Time_is_zero(_msg) ? builtin_interfaces_Time_now() : *_msg;",
            "    const auto* _src = &_resolved;" if False else "",
        ])
        lines.pop()
    if _is_std_header(message):
        lines.extend([
            f"    {struct_name} _resolved = *_msg;",
            "    std_msgs_Header_apply_default_stamp(&_resolved);",
            f"    const {struct_name}* _src = &_resolved;",
        ])
    for field in message.fields:
        if _is_builtin_time(message):
            if field.name == "sec":
                lines.append("    cdr_write_i32(_w, _resolved.sec);")
            elif field.name == "nanosec":
                lines.append("    cdr_write_u32(_w, _resolved.nanosec);")
        elif _is_std_header(message):
            saved = FieldDef(field.type_token, field.name, field.comment)
            if field.name == "stamp":
                token = saved.type_token
                nested = _struct_name(token.package or message.package, token.name).removesuffix("_t")
                lines.append(f"    encode_{nested}_body(_w, &_src->{field.name});")
            elif field.name == "frame_id":
                lines.append(f"    cdr_write_string(_w, _src->{field.name});")
            else:
                _emit_encode_field(lines, message, field, options)
        else:
            _emit_encode_field(lines, message, field, options)
    lines.extend([
        "}",
        "",
        f"static inline size_t encode_{short}(uint8_t* _buf, size_t _cap, const {struct_name}* _msg) {{",
        "    cdr_writer_t _w;",
        "    cdr_writer_init(&_w, _buf, _cap);",
        f"    encode_{short}_body(&_w, _msg);",
        "    return _w.overflow ? 0u : cdr_writer_size(&_w);",
        "}",
        "",
        f"static inline void decode_{short}_body(cdr_reader_t* _r, {struct_name}* _msg) {{",
    ])
    for field in message.fields:
        _emit_decode_field(lines, message, field, options)
    lines.extend([
        "}",
        "",
        f"static inline bool decode_{short}(const uint8_t* _buf, size_t _len, {struct_name}* _msg) {{",
        "    cdr_reader_t _r;",
        "    if (!cdr_reader_init(&_r, _buf, _len)) return false;",
        f"    decode_{short}_body(&_r, _msg);",
        "    return !_r.error;",
        "}",
        "",
        "#ifdef __cplusplus",
        '#include "rosette/ros/message_traits.hpp"',
        "namespace milkros {",
        f"template<> struct message_traits<{struct_name}> {{",
        f"    static constexpr size_t max_serialized_size = {macro}_MAX_SERIALIZED_SIZE;",
        f'    static const char* dds_type_name() noexcept {{ return {macro}_TYPE_NAME; }}',
        (
            f"    static {struct_name} make_message() noexcept {{ return builtin_interfaces_Time_now(); }}"
            if _is_builtin_time(message)
            else (
                f"    static {struct_name} make_message() noexcept {{ {struct_name} _msg{{}}; std_msgs_Header_apply_default_stamp(&_msg); return _msg; }}"
                if _is_std_header(message)
                else f"    static {struct_name} make_message() noexcept {{ return {struct_name}{{}}; }}"
            )
        ),
        f"    static size_t encode(uint8_t* dst, size_t cap, const {struct_name}& message) noexcept {{ return encode_{short}(dst, cap, &message); }}",
        f"    static bool decode(const uint8_t* src, size_t len, {struct_name}& message) noexcept {{ return decode_{short}(src, len, &message); }}",
        "};",
        f"template<> inline constexpr bool has_message_traits_v<{struct_name}> = true;",
        "} // namespace milkros",
        "#endif",
        "",
        f"#endif /* {guard} */",
    ])


def generate_message_header(message: MessageDef, options: GeneratorOptions | None = None) -> str:
    opts = options or GeneratorOptions()
    lines: list[str] = []
    _emit_message_header(lines, message, opts)
    return "\n".join(lines) + "\n"


def generate_service_header(service: ServiceDef, options: GeneratorOptions | None = None) -> str:
    opts = options or GeneratorOptions()
    request_code = generate_message_header(service.request, opts).splitlines()
    response_code = generate_message_header(service.response, opts).splitlines()

    guard = _guard(service.stem + "_srv")
    macro = _macro_prefix(service.package, service.name)
    request_struct = _struct_name(service.package, f"{service.name}_Request")
    response_struct = _struct_name(service.package, f"{service.name}_Response")

    lines = [
        "#pragma once",
        f"#ifndef {guard}",
        f"#define {guard}",
        "",
        "#include <stddef.h>",
        "#include <stdint.h>",
        '#include "rosette/ros/cdr.h"',
        "",
        f'#define {macro}_SERVICE_REQUEST_TYPE_NAME "{_dds_service_type_name(service.package, service.name, "Request")}"',
        f'#define {macro}_SERVICE_RESPONSE_TYPE_NAME "{_dds_service_type_name(service.package, service.name, "Response")}"',
        "",
    ]
    lines.extend(request_code[3:-1])
    lines.append("")
    lines.extend(response_code[3:-1])
    lines.extend([
        "",
        "#ifdef __cplusplus",
        '#include "rosette/ros/service.hpp"',
        f"struct {service.package}_{service.name} {{}};",
        "namespace milkros {",
        f"template<> struct service_traits<{service.package}_{service.name}> {{",
        f"    using Request = {request_struct};",
        f"    using Response = {response_struct};",
        f'    static const char* request_type_name() noexcept {{ return {macro}_SERVICE_REQUEST_TYPE_NAME; }}',
        f'    static const char* response_type_name() noexcept {{ return {macro}_SERVICE_RESPONSE_TYPE_NAME; }}',
        f"    static constexpr size_t max_request_serialized_size = {_macro_prefix(service.package, service.request.name)}_MAX_SERIALIZED_SIZE;",
        f"    static constexpr size_t max_response_serialized_size = {_macro_prefix(service.package, service.response.name)}_MAX_SERIALIZED_SIZE;",
        f"    static Request make_request() noexcept {{ return Request{{}}; }}",
        f"    static Response make_response() noexcept {{ return Response{{}}; }}",
        f"    static size_t encode_request(uint8_t* dst, size_t cap, const Request& msg) noexcept {{ return encode_{service.package}_{service.name}_Request(dst, cap, &msg); }}",
        f"    static bool decode_request(const uint8_t* src, size_t len, Request& msg) noexcept {{ return decode_{service.package}_{service.name}_Request(src, len, &msg); }}",
        f"    static size_t encode_response(uint8_t* dst, size_t cap, const Response& msg) noexcept {{ return encode_{service.package}_{service.name}_Response(dst, cap, &msg); }}",
        f"    static bool decode_response(const uint8_t* src, size_t len, Response& msg) noexcept {{ return decode_{service.package}_{service.name}_Response(src, len, &msg); }}",
        "};",
        f"template<> inline constexpr bool has_service_traits_v<{service.package}_{service.name}> = true;",
        "} // namespace milkros",
        "#endif",
        "",
        f"#endif /* {guard} */",
    ])
    return "\n".join(lines) + "\n"
