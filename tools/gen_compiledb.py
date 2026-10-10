#!/usr/bin/env python3
"""Gộp compile DB native (env lint) + esp32 đã sanitize cho clangd/zed host.

clangd/zed chạy clang host nên không parse được flag cross-compiler của ESP-IDF
(-mlongcalls, -specs=..., ...). Script:
  - bỏ các flag cross-compiler host clang không hiểu;
  - gộp hợp các `-I` của toàn bộ DB esp32 (mỗi file IDF chỉ liệt kê một phần
    include dir của nó) để header IDF resolve;
  - thêm include `machine/endian.h` của IDF (components/linux) — tránh kéo newlib
    xung đột glibc host;
rồi ghi lại `firmware/compile_commands.json`.
"""

from __future__ import annotations

import json
import os
import shlex
import sys

DROP_FLAGS = {
    "-mlongcalls",
    "-fno-shrink-wrap",
    "-fno-tree-switch-conversion",
    "-fstrict-volatile-bitfields",
    "-fno-jump-tables",
}


def _expand(args: list[str]) -> list[str]:
    out: list[str] = []
    for arg in args:
        if arg.startswith("@"):
            with open(arg[1:], encoding="utf-8") as response:
                out.extend(shlex.split(response.read()))
        else:
            out.append(arg)
    return out


def _is_esp32(entry: dict) -> bool:
    return "src/esp32/" in entry["file"].replace(os.sep, "/")


def _idf_include_union(entries: list[dict]) -> list[str]:
    seen: dict[str, None] = {}
    for entry in entries:
        for arg in shlex.split(entry["command"]):
            if arg.startswith("-I") and len(arg) > 2:
                seen.setdefault(arg[2:], None)
    return list(seen)


def _host_include_overrides(entries: list[dict]) -> list[str]:
    args: list[str] = []
    # 1. components/linux/include (host-compat: machine/endian.h, sys/cdefs.h).
    linux_include = ""
    for entry in entries:
        for token in shlex.split(entry["command"]):
            if not token.startswith("-I"):
                continue
            path = token[2:]
            marker = path.find("/components/")
            if marker == -1:
                continue
            candidate = os.path.join(path[:marker], "components", "linux", "include")
            if os.path.isfile(os.path.join(candidate, "machine", "endian.h")):
                linux_include = candidate
                break
        if linux_include:
            break
    if linux_include:
        args += ["-isystem", linux_include]
    return args


def _sanitize(entry: dict, shared_includes: list[str], host_overrides: list[str]) -> dict:
    kept = [
        a
        for a in _expand(shlex.split(entry["command"]))
        if a not in DROP_FLAGS and not a.startswith("-specs=")
    ]
    extra: list[str] = list(host_overrides)
    extra += [f"-I{p}" for p in shared_includes]
    return {
        "directory": entry["directory"],
        "file": entry["file"],
        "arguments": kept[:1] + extra + kept[1:],
    }


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    firmware = os.path.join(root, "firmware")
    native_path = os.path.join(firmware, "compile_commands.json")
    esp32_path = os.path.join(firmware, ".pio", "build", "esp32s3", "compile_commands.json")

    with open(native_path, encoding="utf-8") as handle:
        native = json.load(handle)

    esp32_all: list[dict] = []
    if os.path.isfile(esp32_path):
        with open(esp32_path, encoding="utf-8") as handle:
            esp32_all = json.load(handle)
    esp32_src = [e for e in esp32_all if _is_esp32(e) and e["file"].endswith(".cpp")]
    if not esp32_src:
        print("compiledb: không có entry esp32 (chưa build esp32s3?) — giữ native-only.")
        return

    shared = _idf_include_union(esp32_all)
    overrides = _host_include_overrides(esp32_all)
    if not overrides:
        print("compiledb: không tìm thấy host include (machine/endian.h) cho clang", file=sys.stderr)

    merged = [e for e in native if not _is_esp32(e)]
    merged += [_sanitize(e, shared, overrides) for e in esp32_src]

    with open(native_path, "w", encoding="utf-8") as handle:
        json.dump(merged, handle, indent=4)
    print(f"compiledb: {len(merged) - len(esp32_src)} native + {len(esp32_src)} esp32 = {len(merged)} entries")
    print(f"compiledb: {len(shared)} include dir IDF dùng chung, host={' '.join(overrides)}")


if __name__ == "__main__":
    main()
