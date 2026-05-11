from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .emitter import GeneratorOptions, generate_message_header, generate_service_header
from .parser import parse_message_file, parse_service_file


def _collect_inputs(paths: list[str], recursive: bool) -> list[Path]:
    files: list[Path] = []
    for raw in paths:
        path = Path(raw)
        if path.is_file():
            files.append(path)
            continue
        if not path.exists():
            raise FileNotFoundError(path)
        pattern = "**/*" if recursive else "*"
        files.extend(sorted(p for p in path.glob(pattern) if p.is_file() and p.suffix in {".msg", ".srv"}))
    return sorted(dict.fromkeys(files))


def _write_output(path: Path, content: str, verbose: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    if verbose:
        print(path)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate Rosette headers from .msg and .srv files")
    parser.add_argument("paths", nargs="+", help=".msg/.srv files or directories")
    parser.add_argument("-o", "--output-dir", type=Path, help="Directory for generated headers")
    parser.add_argument("-r", "--recursive", action="store_true", help="Search directories recursively")
    parser.add_argument("--dry-run", action="store_true", help="Print generated output instead of writing files")
    parser.add_argument("-v", "--verbose", action="store_true", help="Print generated file paths")
    parser.add_argument("--max-string-len", type=int, default=256, help="Capacity for unbounded strings")
    parser.add_argument("--max-dynamic-len", type=int, default=64, help="Assumed bound for unbounded arrays")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    options = GeneratorOptions(
        max_string_len=args.max_string_len,
        max_dynamic_len=args.max_dynamic_len,
    )

    files = _collect_inputs(args.paths, args.recursive)
    if not files:
        print("No interface files found", file=sys.stderr)
        return 1

    for path in files:
        if path.suffix == ".msg":
            definition = parse_message_file(path)
            content = generate_message_header(definition, options)
            name = f"{definition.stem}.h"
        elif path.suffix == ".srv":
            definition = parse_service_file(path)
            content = generate_service_header(definition, options)
            name = f"{definition.stem}.h"
        else:
            continue

        if args.dry_run or args.output_dir is None:
            if len(files) > 1:
                print(f"// {name}")
            print(content, end="")
            continue

        _write_output(args.output_dir / name, content, args.verbose)

    return 0
