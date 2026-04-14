from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


PRIMITIVES: dict[str, tuple[str, str, str, int]] = {
    "bool": ("bool", "cdr_write_bool", "cdr_read_bool", 1),
    "byte": ("uint8_t", "cdr_write_u8", "cdr_read_u8", 1),
    "char": ("uint8_t", "cdr_write_u8", "cdr_read_u8", 1),
    "int8": ("int8_t", "cdr_write_u8", "cdr_read_u8", 1),
    "uint8": ("uint8_t", "cdr_write_u8", "cdr_read_u8", 1),
    "int16": ("int16_t", "cdr_write_u16", "cdr_read_u16", 2),
    "uint16": ("uint16_t", "cdr_write_u16", "cdr_read_u16", 2),
    "int32": ("int32_t", "cdr_write_i32", "cdr_read_i32", 4),
    "uint32": ("uint32_t", "cdr_write_u32", "cdr_read_u32", 4),
    "int64": ("int64_t", "cdr_write_u64", "cdr_read_i64", 8),
    "uint64": ("uint64_t", "cdr_write_u64", "cdr_read_u64", 8),
    "float32": ("float", "cdr_write_f32", "cdr_read_f32", 4),
    "float64": ("double", "cdr_write_f64", "cdr_read_f64", 8),
    "string": ("char", "", "", 0),
    "wstring": ("char", "", "", 0),
}

SIGNED_CAST: dict[str, str] = {
    "int8": "(uint8_t)",
    "int16": "(uint16_t)",
    "int64": "(uint64_t)",
}


@dataclass(frozen=True)
class TypeToken:
    package: Optional[str]
    name: str
    array_kind: str = "scalar"  # scalar, fixed, dynamic, bounded
    array_bound: int = 0
    bounded_string: int = 0

    @property
    def is_primitive(self) -> bool:
        return self.package is None and self.name in PRIMITIVES

    @property
    def is_string(self) -> bool:
        return self.package is None and self.name in {"string", "wstring"}

    @property
    def is_nested(self) -> bool:
        return not self.is_primitive or (
            self.package is not None and self.name not in PRIMITIVES
        )


@dataclass
class FieldDef:
    type_token: TypeToken
    name: str
    comment: str = ""

    @property
    def package(self) -> Optional[str]:
        return self.type_token.package

    @property
    def type_name(self) -> str:
        return self.type_token.name


@dataclass
class ConstantDef:
    type_name: str
    name: str
    value: str


@dataclass
class MessageDef:
    package: str
    name: str
    fields: list[FieldDef] = field(default_factory=list)
    constants: list[ConstantDef] = field(default_factory=list)
    source_file: Optional[Path] = None

    @property
    def key(self) -> str:
        return f"{self.package}/{self.name}"

    @property
    def stem(self) -> str:
        return f"{self.package}__{self.name}"


@dataclass
class ServiceDef:
    package: str
    name: str
    request: MessageDef
    response: MessageDef
    source_file: Optional[Path] = None

    @property
    def key(self) -> str:
        return f"{self.package}/{self.name}"

    @property
    def stem(self) -> str:
        return f"{self.package}__{self.name}"
