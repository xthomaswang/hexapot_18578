"""Helpers for recording browser-driven dance timelines."""

from __future__ import annotations

import json
import re
import threading
import time
from pathlib import Path
from typing import Any


ROBOT_ROOT = Path(__file__).resolve().parents[1]
DANCE_ENGINE_ROOT = ROBOT_ROOT / "dance_engine"
DANCE_STORAGE_ROOT = DANCE_ENGINE_ROOT / "storage"
MANUAL_STORAGE_ROOT = DANCE_STORAGE_ROOT / "manual"
AUTO_STORAGE_ROOT = DANCE_STORAGE_ROOT / "auto"


def _slugify(name: str) -> str:
    text = re.sub(r"[^a-zA-Z0-9_-]+", "_", name.strip())
    text = re.sub(r"_+", "_", text).strip("_").lower()
    return text[:48]


def _timestamp_label(timestamp: float) -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.localtime(timestamp))


def _recording_entry_from_payload(payload: dict[str, Any], *, path: Path) -> dict[str, Any]:
    return {
        "mode": str(payload.get("mode") or "manual"),
        "name": str(payload.get("name") or "untitled"),
        "directory_name": path.name,
        "path": str(path),
        "saved_at": payload.get("stopped_at"),
        "duration_sec": float(payload.get("duration_sec") or 0.0),
        "event_count": int(payload.get("event_count") or 0),
    }


class DanceRecordingManager:
    """Captures manual control changes and saves them as timeline JSON."""

    def __init__(self, storage_root: Path | None = None, *, max_recent: int = 8) -> None:
        self.storage_root = Path(storage_root or DANCE_STORAGE_ROOT)
        self.max_recent = max_recent
        self._lock = threading.Lock()
        self._active: dict[str, Any] | None = None
        self._last_saved: dict[str, Any] | None = None
        self._recent = self._load_recent()
        self.storage_root.mkdir(parents=True, exist_ok=True)
        (self.storage_root / "manual").mkdir(parents=True, exist_ok=True)
        (self.storage_root / "auto").mkdir(parents=True, exist_ok=True)

    def _load_recent(self) -> list[dict[str, Any]]:
        recent: list[dict[str, Any]] = []
        for mode in ("manual", "auto"):
            mode_root = self.storage_root / mode
            if not mode_root.exists():
                continue
            for recording_file in mode_root.glob("*/recording.json"):
                try:
                    payload = json.loads(recording_file.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError):
                    continue
                recent.append(
                    _recording_entry_from_payload(payload, path=recording_file.parent)
                )
        recent.sort(key=lambda item: float(item.get("saved_at") or 0.0), reverse=True)
        return recent[: self.max_recent]

    def _recording_snapshot_locked(self) -> dict[str, Any]:
        if self._active is None:
            return {
                "active": False,
                "mode": "manual",
                "name": "",
                "directory_name": "",
                "path": "",
                "started_at": None,
                "duration_sec": 0.0,
                "event_count": 0,
                "last_saved": self._last_saved,
                "recent": list(self._recent),
                "storage_roots": {
                    "manual": str(self.storage_root / "manual"),
                    "auto": str(self.storage_root / "auto"),
                },
            }

        started_at = float(self._active["started_at"])
        return {
            "active": True,
            "mode": self._active["mode"],
            "name": self._active["name"],
            "directory_name": self._active["directory_name"],
            "path": str(self._active["path"]),
            "started_at": started_at,
            "duration_sec": max(0.0, time.time() - started_at),
            "event_count": len(self._active["events"]),
            "last_saved": self._last_saved,
            "recent": list(self._recent),
            "storage_roots": {
                "manual": str(self.storage_root / "manual"),
                "auto": str(self.storage_root / "auto"),
            },
        }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._recording_snapshot_locked()

    def start(self, *, mode: str = "manual", name: str = "") -> dict[str, Any]:
        normalized_mode = str(mode or "manual").strip().lower()
        if normalized_mode != "manual":
            raise ValueError("only manual dance recording is available right now")
        with self._lock:
            if self._active is not None:
                raise RuntimeError("a dance recording is already running")

            now = time.time()
            slug = _slugify(name)
            directory_name = _timestamp_label(now)
            if slug:
                directory_name = f"{directory_name}_{slug}"
            session_path = self.storage_root / normalized_mode / directory_name
            suffix = 2
            while session_path.exists():
                session_path = self.storage_root / normalized_mode / f"{directory_name}_{suffix}"
                suffix += 1
            self._active = {
                "mode": normalized_mode,
                "name": name.strip() or "untitled",
                "directory_name": session_path.name,
                "path": session_path,
                "started_at": now,
                "events": [],
                "last_signature": None,
            }
            return self._recording_snapshot_locked()

    def _append_event_locked(self, snapshot: dict[str, Any], *, reason: str | None = None) -> None:
        if self._active is None:
            return
        started_at = float(self._active["started_at"])
        event = {
            "t": round(max(0.0, time.time() - started_at), 3),
            "active": bool(snapshot.get("active")),
            "mode": str(snapshot.get("mode") or "N"),
            "magnitude": int(snapshot.get("magnitude") or 0),
            "forward_cmd": int(snapshot.get("forward_cmd") or 0),
            "strafe_cmd": int(snapshot.get("strafe_cmd") or 0),
            "turn_cmd": int(snapshot.get("turn_cmd") or 0),
            "source": str(snapshot.get("source") or "web"),
        }
        if reason:
            event["reason"] = reason
        signature = (
            event["active"],
            event["mode"],
            event["magnitude"],
            event["forward_cmd"],
            event["strafe_cmd"],
            event["turn_cmd"],
        )
        if signature == self._active["last_signature"]:
            return
        self._active["events"].append(event)
        self._active["last_signature"] = signature

    def record_override(self, snapshot: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self._active is not None and self._active["mode"] == "manual":
                self._append_event_locked(snapshot)
            return self._recording_snapshot_locked()

    def record_clear(self, snapshot: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self._active is not None and self._active["mode"] == "manual":
                self._append_event_locked(snapshot, reason="clear")
            return self._recording_snapshot_locked()

    def stop(self) -> dict[str, Any]:
        with self._lock:
            if self._active is None:
                raise RuntimeError("no dance recording is currently running")

            stopped_at = time.time()
            inactive_signature = (False, "N", 0, 0, 0, 0)
            if self._active["last_signature"] != inactive_signature:
                self._append_event_locked(
                    {
                        "active": False,
                        "mode": "N",
                        "magnitude": 0,
                        "forward_cmd": 0,
                        "strafe_cmd": 0,
                        "turn_cmd": 0,
                        "source": "web",
                    },
                    reason="stop",
                )

            active = self._active
            session_path = Path(active["path"])
            session_path.mkdir(parents=True, exist_ok=False)
            payload = {
                "version": 1,
                "mode": active["mode"],
                "name": active["name"],
                "directory_name": active["directory_name"],
                "started_at": float(active["started_at"]),
                "stopped_at": stopped_at,
                "duration_sec": round(max(0.0, stopped_at - float(active["started_at"])), 3),
                "event_count": len(active["events"]),
                "events": list(active["events"]),
            }
            (session_path / "recording.json").write_text(
                json.dumps(payload, indent=2),
                encoding="utf-8",
            )

            self._last_saved = _recording_entry_from_payload(payload, path=session_path)
            self._recent.insert(0, self._last_saved)
            self._recent = self._recent[: self.max_recent]
            self._active = None
            return self._recording_snapshot_locked()
