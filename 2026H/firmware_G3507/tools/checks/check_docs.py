#!/usr/bin/env python3
"""Check repository Markdown links and known obsolete command paths."""

from __future__ import annotations

import re
import subprocess
import sys
from functools import lru_cache
from pathlib import Path
from urllib.parse import unquote


ROOT = Path(__file__).resolve().parents[2]
SKIP_PARTS = {".git", "third_party", "build", "out", "Objects"}
LINK_RE = re.compile(r"!?(?:\[[^\]]*\])\(([^)]+)\)")
REPO_PATH_RE = re.compile(
    r"`((?:app|board|bsp|core|docs|k230|middleware|project|tools)/"
    r"[A-Za-z0-9_./-]+)`"
)
LEGACY_PATHS = (
    "tools/check_keil_project_sync.py",
    "tools/speed_pid_viz.py",
    "tools/straight_test_viz.py",
    "tools/canmv_mcp_capture.mjs",
    "tools/canmv_mcp_run.mjs",
    "tools/k230_remote.py",
    "tools/k230_tool.py",
    "tools/k230_video_viewer.py",
    "tools/probes/",
)


@lru_cache(maxsize=None)
def is_gitignored(repository_path: str) -> bool:
    """Whether a path is deliberately absent because .gitignore excludes it.

    Build outputs such as project/keil/Objects/ are documented on purpose but
    never exist in a fresh clone. Failing on those trains readers to ignore this
    check, which defeats it -- so treat "missing but gitignored" as expected.
    """
    try:
        completed = subprocess.run(
            ["git", "check-ignore", "-q", "--", repository_path],
            cwd=ROOT,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        return False          # 没有 git 就退回严格模式，宁可误报也不漏报
    return completed.returncode == 0


def markdown_files() -> list[Path]:
    return sorted(
        path
        for path in ROOT.rglob("*.md")
        if not SKIP_PARTS.intersection(path.relative_to(ROOT).parts)
    )


def link_target(raw_target: str) -> str:
    target = raw_target.strip()
    if target.startswith("<"):
        closing = target.find(">")
        return target[1:closing] if closing >= 0 else target[1:]
    return target.split(maxsplit=1)[0]


def main() -> int:
    failures: list[str] = []
    files = markdown_files()

    for document in files:
        text = document.read_text(encoding="utf-8")
        relative_document = document.relative_to(ROOT)

        for line_number, line in enumerate(text.splitlines(), 1):
            if relative_document.as_posix() != "docs/changelog.md":
                for legacy_path in LEGACY_PATHS:
                    if legacy_path in line:
                        failures.append(
                            f"{relative_document}:{line_number}: obsolete path {legacy_path}"
                        )

                for repository_path in REPO_PATH_RE.findall(line):
                    candidate = ROOT / repository_path
                    module_sources = (
                        candidate.with_suffix(".c"),
                        candidate.with_suffix(".h"),
                    )
                    if (
                        not candidate.exists()
                        and not any(source.exists() for source in module_sources)
                        and not is_gitignored(repository_path)
                    ):
                        failures.append(
                            f"{relative_document}:{line_number}: missing repository path "
                            f"{repository_path}"
                        )

            for match in LINK_RE.finditer(line):
                target = unquote(link_target(match.group(1)))
                if not target or target.startswith("#"):
                    continue
                if re.match(r"^[a-zA-Z][a-zA-Z0-9+.-]*:", target):
                    continue

                path_text = target.split("#", 1)[0].split("?", 1)[0]
                if not path_text:
                    continue
                candidate = (
                    ROOT / path_text.lstrip("/")
                    if path_text.startswith("/")
                    else document.parent / path_text
                )
                if not candidate.exists():
                    failures.append(
                        f"{relative_document}:{line_number}: missing link target {target}"
                    )

    if failures:
        print("Documentation check failed:")
        for failure in failures:
            print(f"  {failure}")
        return 1

    print(f"Documentation check passed: {len(files)} Markdown files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
