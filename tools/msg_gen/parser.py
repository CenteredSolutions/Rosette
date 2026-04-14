from __future__ import annotations

import re
from pathlib import Path

from .model import ConstantDef, FieldDef, MessageDef, ServiceDef, TypeToken


TYPE_RE = re.compile(
    r"^(?:(?P<pkg>[A-Za-z_][A-Za-z0-9_]*)/)?(?P<name>[A-Za-z_][A-Za-z0-9_]*)$"
)
ARRAY_RE = re.compile(
    r"^(?P<base>.+?)(?:\[(?P<kind><=(?P<bounded>\d+)|(?P<fixed>\d+))?\])?$"
)
BOUNDED_STRING_RE = re.compile(r"^(?P<name>w?string)<=(?P<bound>\d+)$")


def parse_type_token(token: str) -> TypeToken:
    bounded_string = BOUNDED_STRING_RE.match(token)
    if bounded_string:
        return TypeToken(
            package=None,
            name=bounded_string.group("name"),
            bounded_string=int(bounded_string.group("bound")),
        )

    array = ARRAY_RE.match(token)
    if not array:
        raise ValueError(f"Invalid type token: {token}")

    base = array.group("base")
    type_match = TYPE_RE.match(base)
    if not type_match:
        raise ValueError(f"Invalid type token: {token}")

    array_kind = "scalar"
    array_bound = 0
    kind = array.group("kind")
    if token.endswith("[]"):
        array_kind = "dynamic"
    elif kind:
        if kind.startswith("<="):
            array_kind = "bounded"
            array_bound = int(array.group("bounded"))
        else:
            array_kind = "fixed"
            array_bound = int(array.group("fixed"))

    return TypeToken(
        package=type_match.group("pkg"),
        name=type_match.group("name"),
        array_kind=array_kind,
        array_bound=array_bound,
    )


def _infer_package_and_name(path: Path) -> tuple[str, str]:
    if "__" in path.stem:
        return tuple(path.stem.split("__", 1))  # type: ignore[return-value]
    return path.parent.name, path.stem


def _strip_comment(line: str) -> tuple[str, str]:
    if "#" not in line:
        return line.strip(), ""
    content, comment = line.split("#", 1)
    return content.strip(), comment.strip()


def _parse_message_lines(package: str, name: str, lines: list[str], source: Path) -> MessageDef:
    message = MessageDef(package=package, name=name, source_file=source)
    for raw_line in lines:
        line, comment = _strip_comment(raw_line)
        if not line:
            continue

        if "=" in line:
            parts = line.split(None, 1)
            if len(parts) != 2 or "=" not in parts[1]:
                continue
            type_name = parts[0].strip()
            const_name, value = parts[1].split("=", 1)
            message.constants.append(
                ConstantDef(type_name=type_name, name=const_name.strip(), value=value.strip())
            )
            continue

        pieces = line.split()
        if len(pieces) != 2:
            continue

        type_token = parse_type_token(pieces[0])
        message.fields.append(FieldDef(type_token=type_token, name=pieces[1], comment=comment))

    return message


def parse_message_file(path: Path) -> MessageDef:
    package, name = _infer_package_and_name(path)
    lines = path.read_text(encoding="utf-8").splitlines()
    return _parse_message_lines(package, name, lines, path)


def parse_service_file(path: Path) -> ServiceDef:
    package, name = _infer_package_and_name(path)
    lines = path.read_text(encoding="utf-8").splitlines()

    parts: list[list[str]] = [[]]
    for line in lines:
        if line.strip() == "---":
            parts.append([])
            continue
        parts[-1].append(line)

    if len(parts) != 2:
        raise ValueError(f"{path} must contain exactly one '---' separator")

    request = _parse_message_lines(package, f"{name}_Request", parts[0], path)
    response = _parse_message_lines(package, f"{name}_Response", parts[1], path)
    return ServiceDef(
        package=package,
        name=name,
        request=request,
        response=response,
        source_file=path,
    )
