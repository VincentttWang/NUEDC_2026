#!/usr/bin/env python3
"""检查 MDK 5 uvprojx 与 MDK 6 cproject 的可构建文件列表是否同步。"""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
MDK5_DIR = REPO_ROOT / "project" / "keil"
MDK6_DIR = MDK5_DIR
MDK5_PROJECT = MDK5_DIR / "NUEDC2025_MSPM0G3507.uvprojx"
MDK6_PROJECT = MDK6_DIR / "NUEDC2025_MSPM0G3507.cproject.yml"
BUILD_SUFFIXES = {".c", ".s", ".a", ".lib"}


def normalize(base: Path, value: str) -> str:
    path = Path(value.replace("\\", "/"))
    resolved = (base / path).resolve() if not path.is_absolute() else path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def read_mdk5_files() -> set[str]:
    root = ET.parse(MDK5_PROJECT).getroot()
    result = set()
    for node in root.findall(".//FilePath"):
        if node.text and Path(node.text).suffix.lower() in BUILD_SUFFIXES:
            result.add(normalize(MDK5_DIR, node.text))
    return result


def read_mdk6_files() -> set[str]:
    result = set()
    file_line = re.compile(r"^\s*-\s+file:\s+(.+?)\s*$")
    for line in MDK6_PROJECT.read_text(encoding="utf-8").splitlines():
        match = file_line.match(line)
        if not match:
            continue
        value = match.group(1).strip("'\"")
        if Path(value).suffix.lower() in BUILD_SUFFIXES:
            result.add(normalize(MDK6_DIR, value))
    return result


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")

    mdk5 = read_mdk5_files()
    mdk6 = read_mdk6_files()
    only_mdk5 = sorted(mdk5 - mdk6)
    only_mdk6 = sorted(mdk6 - mdk5)

    if only_mdk5:
        print("仅 MDK 5 包含：")
        print("\n".join(f"  {path}" for path in only_mdk5))
    if only_mdk6:
        print("仅 MDK 6 包含：")
        print("\n".join(f"  {path}" for path in only_mdk6))
    if only_mdk5 or only_mdk6:
        return 1

    print(f"Keil 工程文件列表已同步：{len(mdk5)} 个编译/链接输入。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
