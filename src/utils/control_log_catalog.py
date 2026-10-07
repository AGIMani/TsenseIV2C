"""Local control-log path and bounded session summaries for the data viewer."""

from __future__ import annotations

from datetime import datetime
import json
import os
from pathlib import Path
from typing import Any

from config_path import LOGS_DIR, PROJECT_ROOT


PARENT_PATH_FILE = PROJECT_ROOT / "configs" / "control_log_parent.txt"
MAX_LINE_BYTES = 2 * 1024 * 1024
MAX_TAIL_BYTES = 2 * 1024 * 1024


def control_log_parent() -> Path:
    """Read the editable parent directory; never create it while listing logs."""
    if PARENT_PATH_FILE.is_file():
        lines = [line.strip().strip('"') for line in PARENT_PATH_FILE.read_text(encoding="utf-8-sig").splitlines()
                 if line.strip() and not line.lstrip().startswith("#")]
        if len(lines) != 1:
            raise ValueError(f"日志父目录配置需要且仅需要一行路径：{PARENT_PATH_FILE}")
        parent = Path(lines[0]).expanduser()
        if not parent.is_absolute():
            raise ValueError(f"日志父目录必须是绝对路径：{PARENT_PATH_FILE}")
        return parent.resolve()
    return Path("E:/TsenseI_hand_logs") if os.name == "nt" else Path(LOGS_DIR) / "control_sessions"


def set_control_log_parent(value: str) -> Path:
    """Create model folders, then atomically save the chosen absolute parent."""
    if not isinstance(value, str) or not value.strip() or len(value) > 4096 or "\n" in value or "\r" in value:
        raise ValueError("请输入单行绝对目录路径")
    parent = Path(value.strip().strip('"')).expanduser()
    if not parent.is_absolute():
        raise ValueError("日志父目录必须是绝对路径")
    parent = parent.resolve()
    for model in ("A30", "A32", "V2"):
        (parent / model).mkdir(parents=True, exist_ok=True)
    PARENT_PATH_FILE.parent.mkdir(parents=True, exist_ok=True)
    temporary = PARENT_PATH_FILE.with_suffix(".txt.tmp")
    try:
        temporary.write_text(str(parent) + "\n", encoding="utf-8")
        temporary.replace(PARENT_PATH_FILE)
    finally:
        temporary.unlink(missing_ok=True)
    return parent


def _model_matches(model: str, value: str) -> bool:
    text = value.lower()
    return ("v3_0" in text or "a30" in text) if model == "A30" else \
        ("v3_2" in text or "a32" in text) if model == "A32" else \
        ("v2" in text) if model == "V2" else False


def _model_folder(model: str) -> str:
    if model not in ("A30", "A32", "V2"):
        raise ValueError("型号必须是 A30、A32 或 V2")
    return model


def _row(line: bytes) -> dict[str, Any] | None:
    if not line or len(line) > MAX_LINE_BYTES:
        return None
    try:
        value = json.loads(line)
    except (UnicodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _tail_row(stream, size: int) -> dict[str, Any] | None:
    stream.seek(max(0, size - MAX_TAIL_BYTES))
    chunk = stream.read(MAX_TAIL_BYTES)
    lines = chunk.splitlines()
    if size > MAX_TAIL_BYTES and lines:
        lines = lines[1:]
    for line in reversed(lines):
        row = _row(line)
        if row and isinstance(row.get("t"), (int, float)):
            return row
    return None


def _control_record(path: Path, model: str) -> dict[str, Any] | None:
    """Read only the header, first event and bounded tail of a large JSONL file."""
    with path.open("rb") as stream:
        header = _row(stream.readline(MAX_LINE_BYTES + 1))
        if not header or header.get("type") != "session" or header.get("schema", "").startswith("hand-diagnostic"):
            return None
        metadata = header.get("metadata") or {}
        if not isinstance(metadata, dict) or not _model_matches(model, str(metadata.get("model_id", ""))):
            return None
        side = metadata.get("hand_side")
        if side not in ("left", "right"):
            return None
        first = _row(stream.readline(MAX_LINE_BYTES + 1))
        last = _tail_row(stream, path.stat().st_size)
    started_at = header.get("started_at")
    if not isinstance(started_at, str):
        return None
    try:
        start_ms = datetime.fromisoformat(started_at.replace("Z", "+00:00")).timestamp() * 1000
    except ValueError:
        return None
    if not first or not last:
        return None
    first_t, last_t = first.get("t"), last.get("t")
    if not isinstance(first_t, (int, float)) or not isinstance(last_t, (int, float)):
        return None
    index_info = header.get("recording_part") or {}
    index = index_info.get("index", 1)
    count = index_info.get("count")
    if not isinstance(index, int) or index < 1 or index > 10000:
        return None
    if not isinstance(count, int) or count < 1 or count > 10000:
        count = None
    duration = last.get("duration_s") if last.get("type") == "session_end" else None
    end_t = max(float(last_t), float(duration) if isinstance(duration, (int, float)) else 0.0)
    part_start = start_ms + (0 if index == 1 else max(0.0, float(first_t)) * 1000)
    part_end = start_ms + max(0.0, end_t) * 1000
    if part_end < part_start:
        return None
    identity = header.get("session_id") or index_info.get("session_id") or started_at
    return {"id": f"log:{identity}:{model}:{side}", "model": model, "side": side,
            "name": str(metadata.get("hand_name") or "")[:80], "start": start_ms,
            "parts": [{"index": index, "count": count, "start": part_start, "end": part_end,
                       "closed": last.get("type") == "session_end", "filename": path.name}]}


def control_log_file(model: str, name: str) -> Path | None:
    folder_name = _model_folder(model)
    if not isinstance(name, str) or not name.startswith("hand_control_") or not name.endswith(".jsonl") or Path(name).name != name:
        return None
    folder = (control_log_parent() / folder_name).resolve()
    path = (folder / name).resolve()
    if not path.is_relative_to(folder) or not path.is_file():
        return None
    return path if _control_record(path, model) is not None else None


def control_log_catalog(model: str) -> dict[str, Any]:
    folder_name = _model_folder(model)
    parent = control_log_parent()
    folder = parent / folder_name
    records, files = [], []
    if folder.is_dir():
        for path in sorted(folder.glob("hand_control_*.jsonl"))[:10000]:
            if not path.resolve().is_relative_to(folder.resolve()) or not path.is_file():
                continue
            try:
                record = _control_record(path, model)
                if record:
                    records.append(record)
                    files.append({"name": path.name, "bytes": path.stat().st_size})
            except OSError:
                continue
    return {"parent": str(parent), "model": model, "records": records, "files": files}


def a30_file(name: str) -> Path | None:
    return control_log_file("A30", name)


def a30_catalog() -> dict[str, Any]:
    return control_log_catalog("A30")
