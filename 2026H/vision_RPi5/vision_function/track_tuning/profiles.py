"""Versioned, Raspberry-Pi-owned track-tuning profiles."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile
from typing import Mapping

from .parameters import PARAMETERS


PROFILE_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class TuningProfile:
    name: str
    values: dict[str, int]
    saved_at: str


def default_profile_directory() -> Path:
    configured = os.environ.get("XDG_CONFIG_HOME")
    root = Path(configured).expanduser() if configured else Path.home() / ".config"
    return root / "nuedc-track-tuning" / "profiles"


def _normalize_values(values: Mapping[str, int]) -> dict[str, int]:
    expected = {spec.key for spec in PARAMETERS}
    supplied = set(values)
    if supplied != expected:
        missing = sorted(expected - supplied)
        extra = sorted(supplied - expected)
        details = []
        if missing:
            details.append("缺少 " + "、".join(missing))
        if extra:
            details.append("未知 " + "、".join(extra))
        raise ValueError("配置参数集合不完整：" + "；".join(details))
    normalized = {key: int(value) for key, value in values.items()}
    invalid = [key for key, value in normalized.items() if not 0 <= value <= 0xFFFF]
    if invalid:
        raise ValueError("配置值超出 uint16：" + "、".join(sorted(invalid)))
    return normalized


def save_profile(path: Path | str, name: str, values: Mapping[str, int]) -> TuningProfile:
    target = Path(path)
    normalized = _normalize_values(values)
    saved_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    payload = {
        "schema_version": PROFILE_SCHEMA_VERSION,
        "name": str(name).strip() or target.stem,
        "saved_at": saved_at,
        "values": normalized,
    }
    target.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary_name = tempfile.mkstemp(
        prefix=target.name + ".", suffix=".tmp", dir=str(target.parent)
    )
    try:
        with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        os.replace(temporary_name, target)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return TuningProfile(payload["name"], normalized, saved_at)


def load_profile(path: Path | str) -> TuningProfile:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if payload.get("schema_version") != PROFILE_SCHEMA_VERSION:
        raise ValueError(
            f"不支持的配置版本：{payload.get('schema_version')!r}，当前版本为 {PROFILE_SCHEMA_VERSION}"
        )
    values = _normalize_values(payload.get("values", {}))
    name = str(payload.get("name", "")).strip()
    saved_at = str(payload.get("saved_at", "")).strip()
    if not name or not saved_at:
        raise ValueError("配置缺少 name 或 saved_at")
    return TuningProfile(name, values, saved_at)
