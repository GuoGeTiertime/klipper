# XYZ nozzle detect — Klipper [xyzcal_detect] plugin + Service/HTTP/snapshot.
# Algorithm: xyzcal_nozzle_detector.py (frozen). Calib: xyzcal_calib.py.
#
# Copyright (C) 2026 TierTime / ScreenQML migration
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import annotations

import argparse
import os
import logging
import base64
import hashlib
import json
import math
import sys
import threading
import time
from dataclasses import asdict, dataclass
from enum import Enum
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qsl, urlencode, urlparse, urlunparse

import cv2

try:
    from http.server import ThreadingHTTPServer as _Server
except ImportError:  # pragma: no cover
    _Server = HTTPServer


# --- snapshot fetch (from webcam_detect) ---
try:
    import certifi
except ImportError:
    certifi = None

import ssl
import urllib.error
import urllib.parse
import urllib.request

import numpy as np


def fetch_snapshot(
    url,
    timeout=15.0,
    allow_insecure_certificate=False,
):
    """Download and decode one uncached JPEG/PNG snapshot."""
    parsed = urllib.parse.urlsplit(url)
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise ValueError("摄像头地址必须是有效的 http:// 或 https:// URL")
    separator = "&" if parsed.query else "?"
    fresh_url = "%s%s_nozzle_ts=%d" % (url, separator, time.time_ns())
    request = urllib.request.Request(
        fresh_url,
        headers={
            "User-Agent": "NozzleDetector-V6-R16/1.0",
            "Cache-Control": "no-cache",
            "Pragma": "no-cache",
        },
    )
    if allow_insecure_certificate:
        ssl_context = ssl._create_unverified_context()  # noqa: SLF001
    elif certifi is not None:
        ssl_context = ssl.create_default_context(cafile=certifi.where())
    else:
        ssl_context = ssl.create_default_context()
    try:
        with urllib.request.urlopen(
            request, timeout=timeout, context=ssl_context
        ) as response:
            payload = response.read()
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        reason = getattr(exc, "reason", exc)
        certificate_hint = ""
        if isinstance(reason, ssl.SSLCertVerificationError):
            certificate_hint = (
                "；请先执行 pip install -r requirements.txt。"
                "仅在确认地址为可信设备时才使用 --insecure"
            )
        raise RuntimeError("无法取得摄像头快照：%s%s" % (exc, certificate_hint)) from exc
    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("摄像头返回内容不是有效图片")
    return image


_ROOT = Path(__file__).resolve().parent
_EXTRAS_DIR = str(_ROOT)
if _EXTRAS_DIR not in sys.path:
    sys.path.insert(0, _EXTRAS_DIR)

import xyzcal_nozzle_detector as _nozzle_detector
from xyzcal_nozzle_detector import (
    CameraProfile,
    NozzleTracker,
    draw_detection,
    _normalize_detect_mode,
    _write_image,
)

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 18765
DEFAULT_SNAPSHOT = "http://127.0.0.1:8080/?action=snapshot"
MIN_CONFIDENCE_DEFAULT = 0.36
LAST_DETECT_JPG = _ROOT / "_xycal_last_detect.jpg"
# UI、启动脚本和服务必须使用同一个版本标识。旧常驻进程即使仍占用
# 18765，也不能再被新版 UI 当作可用服务。
SERVICE_VERSION = "screenqml-v5-integration-r3"
# Keep SERVICE_VERSION unchanged: it is the UI/server wire-protocol contract.
# ALGORITHM_VERSION identifies the detector implementation and is exposed by
# /health so a stale integrated process can be found without changing the UI.
ALGORITHM_VERSION = "v6-r16-a133-80x72-predict-r6-klippy"

_TRACKER_LOCK = threading.Lock()
# Module-level services keyed by stable snapshot URL (HTTP standalone / shared).
_SERVICES = {}


class NozzleDetectionService:
    """Camera + tracker layer; pure vision stays in detect_nozzle()."""

    def __init__(self, snapshot_url, camera_profile=None, min_confidence=None):
        self.snapshot_url = str(snapshot_url or DEFAULT_SNAPSHOT).strip()
        self.profile = camera_profile or CameraProfile()
        self.min_confidence = (
            float(min_confidence)
            if min_confidence is not None
            else MIN_CONFIDENCE_DEFAULT
        )
        self._lock = threading.Lock()
        self._tracker = None
        self._tracker_key = None

    def reset(self):
        with self._lock:
            self._tracker = None
            self._tracker_key = None

    def _flush_snapshots(self, url, count, tag_prefix, allow_insecure):
        for index in range(max(0, int(count))):
            fetch_snapshot(
                _cache_busted_url(url, "%s-%d" % (tag_prefix, index)),
                timeout=15.0,
                allow_insecure_certificate=allow_insecure,
            )
            time.sleep(0.12)

    def capture(self, url=None, *, fresh=False, flush_count=0, allow_insecure=True):
        snap_url = str(url or self.snapshot_url).strip()
        n = max(0, min(8, int(flush_count)))
        if fresh and n < 2:
            n = 2
        self._flush_snapshots(snap_url, n, "flush", allow_insecure)
        bgr = fetch_snapshot(
            _cache_busted_url(snap_url, "detect"),
            timeout=15.0,
            allow_insecure_certificate=allow_insecure,
        )
        return bgr, n

    def detect(
        self,
        *,
        url=None,
        expected_center=None,
        search_delta=None,
        radius_range=None,
        min_confidence=None,
        fresh_frame=False,
        flush_snapshot_count=0,
        mode=None,
        reset=False,
        search_width_px=80.0,
        search_height_px=72.0,
        allow_insecure=True,
        include_overlay=False,
    ):
        snap_url = str(url or self.snapshot_url).strip()
        if not snap_url:
            return _result({
                "ok": False,
                "error": "NO_URL",
                "cx_px": -1,
                "cy_px": -1,
                "radius_px": 0,
                "confidence": 0.0,
                "frame_w": 0,
                "frame_h": 0,
            })

        min_conf = (
            float(min_confidence)
            if min_confidence is not None
            else self.min_confidence
        )
        detect_mode = _normalize_detect_mode(mode, expected_center is not None)
        if search_delta is not None:
            dx, dy = float(search_delta[0]), float(search_delta[1])
            position_window_width = max(32.0, min(160.0, 2.0 * dx))
            position_window_height = max(32.0, min(144.0, 2.0 * dy))
        else:
            position_window_width = float(search_width_px)
            position_window_height = float(search_height_px)

        min_radius_px = None
        max_radius_px = None
        if radius_range is not None:
            min_radius_px = float(radius_range[0])
            max_radius_px = float(radius_range[1])

        def _tip_looks_stale_vs_expected(frame_w, frame_h, tip_xy, expect_xy):
            if expect_xy is None or tip_xy is None:
                return False
            if frame_w < 40 or frame_h < 40:
                return False
            mid = (0.5 * float(frame_w), 0.5 * float(frame_h))
            ex, ey = float(expect_xy[0]), float(expect_xy[1])
            tx, ty = float(tip_xy[0]), float(tip_xy[1])
            est_off = math.hypot(ex - mid[0], ey - mid[1])
            tip_off = math.hypot(tx - mid[0], ty - mid[1])
            d_est = math.hypot(tx - ex, ty - ey)
            return est_off >= 60.0 and tip_off < 0.5 * est_off and d_est > 40.0

        bgr = None
        tracker = None
        stale_flush_retries = 0
        total_flushed = 0
        try:
            with self._lock:
                if reset:
                    self._tracker = None
                    self._tracker_key = None
                bgr, flushed = self.capture(
                    snap_url,
                    fresh=bool(fresh_frame),
                    flush_count=int(flush_snapshot_count),
                    allow_insecure=allow_insecure,
                )
                total_flushed = flushed
                tracker_key = (
                    _tracker_source_key(snap_url),
                    round(min_conf, 4),
                    round(position_window_width, 2),
                    round(position_window_height, 2),
                    None if min_radius_px is None else round(min_radius_px, 3),
                    None if max_radius_px is None else round(max_radius_px, 3),
                )
                if self._tracker is None or self._tracker_key != tracker_key:
                    self._tracker = NozzleTracker(
                        min_confidence=min_conf,
                        strict_position_lock=True,
                        position_window_width_px=position_window_width,
                        position_window_height_px=position_window_height,
                        min_radius_px=min_radius_px,
                        max_radius_px=max_radius_px,
                        allow_fixed640_relocation=bool(
                            self.profile.allow_fixed640_relocation
                        ),
                    )
                    self._tracker_key = tracker_key
                tracker = self._tracker
                det = tracker.update(
                    bgr, expected_center=expected_center, mode=detect_mode
                )
                while stale_flush_retries < 1:
                    fh, fw = bgr.shape[:2]
                    tip_xy = (float(det.circle.x), float(det.circle.y))
                    if not _tip_looks_stale_vs_expected(
                        fw, fh, tip_xy, expected_center
                    ):
                        break
                    stale_flush_retries += 1
                    extra = 2
                    self._flush_snapshots(
                        snap_url, extra, "stale-%d" % stale_flush_retries, allow_insecure
                    )
                    total_flushed += extra
                    bgr = fetch_snapshot(
                        _cache_busted_url(
                            snap_url, "detect-stale-%d" % stale_flush_retries
                        ),
                        timeout=15.0,
                        allow_insecure_certificate=allow_insecure,
                    )
                    det = tracker.update(
                        bgr, expected_center=expected_center, mode=detect_mode
                    )
        except (RuntimeError, ValueError, OSError) as e:
            msg = str(e)
            err = (
                "FETCH_FAIL"
                if ("快照" in msg or "URL" in msg or "图片" in msg)
                else "NO_DETECT"
            )
            frame_h = 0 if bgr is None else int(bgr.shape[0])
            frame_w = 0 if bgr is None else int(bgr.shape[1])
            return _result({
                "ok": False,
                "error": err,
                "detail": msg,
                "cx_px": -1,
                "cy_px": -1,
                "radius_px": 0,
                "confidence": 0.0,
                "frame_w": frame_w,
                "frame_h": frame_h,
                "tracker_status": (
                    "unavailable" if tracker is None else tracker.last_status
                ),
                "follow_roi": (
                    None
                    if tracker is None or tracker.last_follow_roi is None
                    else [round(value, 2) for value in tracker.last_follow_roi]
                ),
                "allowed_roi": (
                    None
                    if tracker is None or tracker.last_allowed_roi is None
                    else [round(value, 2) for value in tracker.last_allowed_roi]
                ),
                "allowed_circle": (
                    None
                    if tracker is None or tracker.last_allowed_circle is None
                    else [round(value, 2) for value in tracker.last_allowed_circle]
                ),
                "reject_reason": msg,
                "mode": detect_mode,
            })

        h, w = bgr.shape[:2]
        conf = float(det.confidence)
        overlay_file = ""
        overlay_jpeg_b64 = ""
        if bool(include_overlay):
            try:
                marked = draw_detection(bgr, det)
                _write_image(LAST_DETECT_JPG, marked)
                overlay_file = str(LAST_DETECT_JPG.resolve())
                ok, enc = cv2.imencode(
                    ".jpg", marked, [int(cv2.IMWRITE_JPEG_QUALITY), 85]
                )
                if ok:
                    overlay_jpeg_b64 = base64.b64encode(enc.tobytes()).decode("ascii")
            except OSError:
                overlay_file = ""
                overlay_jpeg_b64 = ""

        return _result({
            "ok": conf >= min_conf,
            "cx_px": float(det.circle.x),
            "cy_px": float(det.circle.y),
            "radius_px": float(det.circle.radius),
            "confidence": conf,
            "frame_w": int(w),
            "frame_h": int(h),
            "overlay_file": overlay_file,
            "overlay_jpeg_b64": overlay_jpeg_b64,
            "flushed_snapshots": total_flushed,
            "stale_flush_retries": stale_flush_retries,
            "tracker_status": tracker.last_status,
            "follow_roi": (
                None
                if tracker.last_follow_roi is None
                else [round(value, 2) for value in tracker.last_follow_roi]
            ),
            "allowed_roi": (
                None
                if tracker.last_allowed_roi is None
                else [round(value, 2) for value in tracker.last_allowed_roi]
            ),
            "allowed_circle": (
                None
                if tracker.last_allowed_circle is None
                else [round(value, 2) for value in tracker.last_allowed_circle]
            ),
            "reject_reason": "",
            "mode": detect_mode,
        })


def get_shared_service(snapshot_url, min_confidence=None):
    key = _tracker_source_key(str(snapshot_url or DEFAULT_SNAPSHOT))
    with _TRACKER_LOCK:
        svc = _SERVICES.get(key)
        if svc is None:
            svc = NozzleDetectionService(
                snapshot_url, min_confidence=min_confidence
            )
            _SERVICES[key] = svc
        elif min_confidence is not None:
            svc.min_confidence = float(min_confidence)
        return svc


def _parse_body_detect_args(body, default_url):
    """Parse HTTP/GCode body into Service.detect kwargs (legacy aliases OK)."""
    url = str(body.get("url") or default_url or "").strip()
    min_conf = body.get("min_confidence")
    try:
        min_conf = float(min_conf) if min_conf is not None else MIN_CONFIDENCE_DEFAULT
    except (TypeError, ValueError):
        min_conf = MIN_CONFIDENCE_DEFAULT

    def _optional_positive_float(name):
        value = body.get(name)
        if value is None:
            return None
        parsed = float(value)
        if not math.isfinite(parsed) or parsed <= 0:
            raise ValueError("%s must be positive" % name)
        return parsed

    min_radius_px = _optional_positive_float("min_radius_px")
    max_radius_px = _optional_positive_float("max_radius_px")
    if (
        min_radius_px is not None
        and max_radius_px is not None
        and min_radius_px > max_radius_px
    ):
        raise ValueError("min_radius_px > max_radius_px")

    try:
        flush_snapshot_count = int(body.get("flush_snapshot_count") or 0)
    except (TypeError, ValueError):
        flush_snapshot_count = 0
    flush_snapshot_count = max(0, min(8, flush_snapshot_count))
    fresh_frame = bool(body.get("fresh_frame"))
    if fresh_frame and flush_snapshot_count < 2:
        flush_snapshot_count = 2

    allow_insecure = bool(body.get("insecure")) if "insecure" in body else True

    search_width = body.get("search_width_px")
    search_height = body.get("search_height_px")
    search_delta = None
    if body.get("search_delta_x") is not None or body.get("search_delta_y") is not None:
        try:
            dx = float(body.get("search_delta_x"))
            dy = float(body.get("search_delta_y"))
            if not math.isfinite(dx) or not math.isfinite(dy) or dx <= 0 or dy <= 0:
                raise ValueError
            search_delta = (dx, dy)
        except (TypeError, ValueError):
            raise ValueError("search_delta_x/y 必须是正的半宽/半高") from None

    if search_delta is None:
        if (search_width is None) != (search_height is None):
            raise ValueError("search_width_px 和 search_height_px 必须同时提供")
        if search_width is None:
            position_window_width = 80.0
            position_window_height = 72.0
        else:
            position_window_width = float(search_width)
            position_window_height = float(search_height)
            if (
                not math.isfinite(position_window_width)
                or not math.isfinite(position_window_height)
                or position_window_width < 32.0
                or position_window_height < 32.0
            ):
                raise ValueError("位置搜索窗口宽高必须是至少 32px 的有限数值")
            position_window_width = min(160.0, position_window_width)
            position_window_height = min(144.0, position_window_height)
    else:
        position_window_width = max(32.0, min(160.0, 2.0 * search_delta[0]))
        position_window_height = max(32.0, min(144.0, 2.0 * search_delta[1]))

    expected_x = body.get("expected_x")
    expected_y = body.get("expected_y")
    if (expected_x is None) != (expected_y is None):
        raise ValueError("expected_x 和 expected_y 必须同时提供")
    expected_center = None
    if expected_x is not None:
        expected_center = (float(expected_x), float(expected_y))
        if not all(math.isfinite(value) for value in expected_center):
            raise ValueError("expected_x/y 必须是有限像素坐标")

    radius_range = None
    if min_radius_px is not None or max_radius_px is not None:
        if min_radius_px is None or max_radius_px is None:
            # Allow one-sided override via Service/Tracker configured radii
            radius_range = (
                min_radius_px if min_radius_px is not None else 10.0,
                max_radius_px if max_radius_px is not None else 17.0,
            )
        else:
            radius_range = (min_radius_px, max_radius_px)

    mode = body.get("mode")
    return {
        "url": url,
        "expected_center": expected_center,
        "search_delta": search_delta,
        "radius_range": radius_range,
        "min_confidence": min_conf,
        "fresh_frame": fresh_frame,
        "flush_snapshot_count": flush_snapshot_count,
        "mode": mode,
        "reset": bool(body.get("reset_follow")),
        "search_width_px": position_window_width,
        "search_height_px": position_window_height,
        "allow_insecure": allow_insecure,
        "include_overlay": bool(body.get("include_overlay")),
    }


# Back-compat aliases (old module globals removed; keep names for importers).
_TRACKER = None
_TRACKER_KEY = None


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


DETECTOR_PATH = Path(_nozzle_detector.__file__).resolve()
WEBCAM_PATH = Path(__file__).resolve()
DETECTOR_SHA256 = _sha256_file(DETECTOR_PATH)
WEBCAM_SHA256 = _sha256_file(WEBCAM_PATH)


def _runtime_metadata() -> dict:
    """Return enough identity data to prove which backend is actually running."""
    return {
        "service": "xyzcal-detect",
        "service_version": SERVICE_VERSION,
        "algorithm": ALGORITHM_VERSION,
        "detector_sha256": DETECTOR_SHA256,
        "detect_module_sha256": WEBCAM_SHA256,
        "detector_path": str(DETECTOR_PATH),
        "opencv_version": str(cv2.__version__),
    }


def _result(payload: dict) -> dict:
    payload.update(_runtime_metadata())
    return payload


def _json_bytes(obj: dict) -> bytes:
    return json.dumps(obj, ensure_ascii=False).encode("utf-8")


def _tracker_source_key(url: str) -> str:
    """Keep one tracker across snapshot cache-buster query changes."""
    parsed = urlparse(url)
    stable_query = [
        (key, value)
        for key, value in parse_qsl(parsed.query, keep_blank_values=True)
        if key.lower()
        not in {"t", "_", "timestamp", "cachebuster", "_xycal_fetch"}
    ]
    return urlunparse(parsed._replace(query=urlencode(stable_query)))


def _cache_busted_url(url: str, tag: str) -> str:
    """Force a new HTTP snapshot request without changing tracker identity."""
    parsed = urlparse(url)
    query = [
        (key, value)
        for key, value in parse_qsl(parsed.query, keep_blank_values=True)
        if key.lower() != "_xycal_fetch"
    ]
    query.append(("_xycal_fetch", "%d-%s" % (time.time_ns(), tag)))
    return urlunparse(parsed._replace(query=urlencode(query)))


def _detect_payload(body: dict, default_url: str) -> dict:
    expected_version = str(body.get("expected_service_version") or "").strip()
    if expected_version and expected_version != SERVICE_VERSION:
        return _result({
            "ok": False,
            "error": "BACKEND_VERSION_MISMATCH",
            "detail": "UI expects %s, backend is %s"
            % (expected_version, SERVICE_VERSION),
            "cx_px": -1,
            "cy_px": -1,
            "radius_px": 0,
            "confidence": 0.0,
            "frame_w": 0,
            "frame_h": 0,
        })
    try:
        kwargs = _parse_body_detect_args(body, default_url)
    except ValueError as exc:
        msg = str(exc)
        err = "BAD_RADIUS_RANGE"
        if "search" in msg.lower():
            err = "BAD_SEARCH_SIZE"
        elif "expected" in msg.lower():
            err = "BAD_EXPECTED"
        return _result({
            "ok": False,
            "error": err,
            "detail": msg,
            "cx_px": -1,
            "cy_px": -1,
            "radius_px": 0,
            "confidence": 0.0,
            "frame_w": 0,
            "frame_h": 0,
            "follow_roi": None,
            "allowed_roi": None,
            "allowed_circle": None,
            "reject_reason": msg,
        })
    if not kwargs.get("url"):
        return _result({
            "ok": False,
            "error": "NO_URL",
            "cx_px": -1,
            "cy_px": -1,
            "radius_px": 0,
            "confidence": 0.0,
            "frame_w": 0,
            "frame_h": 0,
        })
    service = get_shared_service(kwargs["url"], kwargs.get("min_confidence"))
    return service.detect(**kwargs)



def make_handler(default_snapshot: str):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

        def _send(self, code: int, obj: dict):
            raw = _json_bytes(obj)
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(raw)

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self):
            path = urlparse(self.path).path.rstrip("/") or "/"
            if path in ("/health", "/"):
                health = _runtime_metadata()
                health.update({"ok": True, "source": "xyzcal_detect"})
                self._send(200, health)
                return
            if path == "/shutdown":
                def _stop():
                    try:
                        threading.Event().wait(0.2)
                        self.server.shutdown()
                    except Exception:
                        pass
                threading.Thread(target=_stop, daemon=True).start()
                self._send(200, {"ok": True, "stopping": True})
                return
            self._send(404, {"ok": False, "error": "NOT_FOUND"})

        def do_POST(self):
            path = urlparse(self.path).path.rstrip("/") or "/"
            if path != "/detect":
                self._send(404, {"ok": False, "error": "NOT_FOUND"})
                return
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                body = json.loads(raw.decode("utf-8") or "{}")
                if not isinstance(body, dict):
                    body = {}
            except (json.JSONDecodeError, UnicodeDecodeError):
                body = {}
            payload = _detect_payload(body, default_snapshot)
            self._send(200, payload)

    return Handler


def run_detect(body, default_url):
    """Run one detect; body is the same dict as POST /detect JSON."""
    if not isinstance(body, dict):
        body = {}
    return _detect_payload(body, default_url or DEFAULT_SNAPSHOT)


def create_http_server(host, port, default_snapshot):
    """Build ThreadingHTTPServer for /health and /detect."""
    handler = make_handler(default_snapshot or DEFAULT_SNAPSHOT)
    return _Server((host, int(port)), handler)



# --- Klipper plugin ---
# Center session Profile/Point (migrated from xyzcal_session; Phase 2 will scale by frame size)


class ToolType(str, Enum):
    MAIN = "MAIN"
    SECOND = "SECOND"


@dataclass
class XYZPixelPoint:
    x: float
    y: float
    z: float
    tool: ToolType
    pixel_x: Optional[float] = None
    pixel_y: Optional[float] = None
    detected: bool = False


@dataclass(frozen=True)
class NozzleDetectionProfile:
    snapshot_url: str
    target_pixel_x: float
    target_pixel_y: float
    search_delta_x: float
    search_delta_y: float
    radius_min_px: float
    radius_max_px: float
    calib_px_mm: float
    min_confidence: float
    fresh_frame_flush_count: int = 2


@dataclass
class DetectCallContext:
    """Per-call Detect params (not part of frozen Profile)."""

    expected_pixel_x: Optional[float] = None
    expected_pixel_y: Optional[float] = None
    mode: str = ""
    reset_tracker: bool = False


def _parse_tool(raw):
    s = str(raw or "MAIN").strip().upper()
    if s in ("SEC", "SECOND", "T1", "1"):
        return ToolType.SECOND
    return ToolType.MAIN


_PROFILE_STATUS_KEYS = (
    "snapshot_url",
    "target_pixel_x",
    "target_pixel_y",
    "search_delta_x",
    "search_delta_y",
    "radius_min_px",
    "radius_max_px",
    "calib_px_mm",
    "min_confidence",
    "fresh_frame_flush_count",
)


class XyCalDetect:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")

        self.snapshot_url = config.get(
            "snapshot_url", "http://127.0.0.1:8080/?action=snapshot"
        )
        self.listen_host = config.get("listen_host", "127.0.0.1")
        self.listen_port = config.getint("listen_port", 18765, minval=0, maxval=65535)
        self.min_confidence = config.getfloat(
            "min_confidence", 0.36, above=0.0, maxval=1.0
        )
        self.enable_http = config.getboolean("enable_http", True)
        self.nozzle_radius_px = config.getfloat(
            "nozzle_radius_px", 0.0, minval=0.0
        )
        self.nozzle_radius_tol = config.getfloat(
            "nozzle_radius_tol", 0.15, above=0.0, maxval=1.0
        )

        self._api = None
        self._service = None
        self._server = None
        self._http_thread = None
        self._last_result = {}
        self._import_error = None
        self._busy = False
        self._detect_seq = 0

        # Center session snapshot (filled on XYZCAL_CENTER success)
        self.profile = None  # type: Optional[NozzleDetectionProfile]
        self.point = None  # type: Optional[XYZPixelPoint]
        self.last_detect = DetectCallContext()
        self.session_ready = False
        self._session_message = ""

        try:
            self._api = sys.modules[__name__]
            self._service = NozzleDetectionService(
                self.snapshot_url, min_confidence=self.min_confidence
            )
        except Exception as exc:
            self._import_error = str(exc)
            logging.exception("xyzcal_detect: failed to import detect_api")

        self.gcode.register_command(
            "XYZCAL_DETECT",
            self.cmd_XYZCAL_DETECT,
            desc=self.cmd_XYZCAL_DETECT_help,
        )
        self.gcode.register_command(
            "XYZCAL_DETECT_STATUS",
            self.cmd_XYZCAL_DETECT_STATUS,
            desc=self.cmd_XYZCAL_DETECT_STATUS_help,
        )
        # Phase 2 short alias (same as XYZCAL_DETECT_STATUS)
        self.gcode.register_command(
            "XYZCAL_STATUS",
            self.cmd_XYZCAL_DETECT_STATUS,
            desc=self.cmd_XYZCAL_DETECT_STATUS_help,
        )
        self.gcode.register_command(
            "XYZCAL_SET_NOZZLE_RADIUS",
            self.cmd_XYZCAL_SET_NOZZLE_RADIUS,
            desc=self.cmd_XYZCAL_SET_NOZZLE_RADIUS_help,
        )
        self.gcode.register_command(
            "XYZCAL_PROFILE_DUMP",
            self.cmd_XYZCAL_PROFILE_DUMP,
            desc=self.cmd_XYZCAL_PROFILE_DUMP_help,
        )
        # Compat alias (was xyzcal_session)
        self.gcode.register_command(
            "XYZCAL_SESSION_DUMP",
            self.cmd_XYZCAL_PROFILE_DUMP,
            desc=self.cmd_XYZCAL_PROFILE_DUMP_help,
        )
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)

    def _handle_ready(self):
        if self._api is None:
            logging.error(
                "xyzcal_detect: OpenCV/detect_api unavailable: %s",
                self._import_error,
            )
            return
        if self._service is None:
            self._service = NozzleDetectionService(
                self.snapshot_url, min_confidence=self.min_confidence
            )
        if not self.enable_http or self.listen_port <= 0:
            logging.info(
                "xyzcal_detect: HTTP disabled (enable_http=%s listen_port=%s); "
                "XYZCAL_DETECT still available",
                self.enable_http,
                self.listen_port,
            )
            return
        try:
            self._server = create_http_server(
                self.listen_host, self.listen_port, self.snapshot_url
            )
        except Exception:
            logging.exception(
                "xyzcal_detect: cannot bind %s:%s (stop old xyzcal_detect_server first)",
                self.listen_host,
                self.listen_port,
            )
            self._server = None
            return

        def _serve():
            logging.info(
                "xyzcal_detect: HTTP on http://%s:%s snapshot=%s",
                self.listen_host,
                self.listen_port,
                self.snapshot_url,
            )
            try:
                self._server.serve_forever()
            except Exception:
                logging.exception("xyzcal_detect: HTTP server stopped with error")

        self._http_thread = threading.Thread(
            target=_serve, name="xyzcal_detect_http", daemon=True
        )
        self._http_thread.start()

    def _handle_disconnect(self):
        server = self._server
        self._server = None
        if server is None:
            return
        try:
            server.shutdown()
        except Exception:
            logging.exception("xyzcal_detect: HTTP shutdown failed")
        try:
            server.server_close()
        except Exception:
            pass

    def reset_tracker(self):
        if self._service is not None:
            self._service.reset()

    def calibrated_radius_band(self):
        """Return (min, max) from saved nozzle radius, or None if unset."""
        r = float(self.nozzle_radius_px or 0.0)
        if r <= 0.0:
            return None
        half = max(1.5, r * float(self.nozzle_radius_tol))
        return (max(5.0, r - half), min(60.0, r + half))

    def _inject_radius_band(self, body):
        if not isinstance(body, dict):
            return body
        if body.get("min_radius_px") is not None or body.get("max_radius_px") is not None:
            return body
        band = self.calibrated_radius_band()
        if band is None:
            return body
        body = dict(body)
        body["min_radius_px"] = band[0]
        body["max_radius_px"] = band[1]
        return body

    def detect_once(self, body):
        """Run one detect for other extras (xyzcal_calib). Updates last result/seq."""
        if self._api is None:
            return {
                "ok": False,
                "error": "DETECT_UNAVAILABLE",
                "detail": self._import_error or "import failed",
                "cx_px": -1,
                "cy_px": -1,
                "radius_px": 0,
                "confidence": 0.0,
            }
        if not isinstance(body, dict):
            body = {}
        body = self._inject_radius_band(body)
        eventtime = self.reactor.monotonic()
        result_box = []
        was_busy = self._busy
        self._busy = True

        def _work():
            try:
                kwargs = _parse_body_detect_args(body, self.snapshot_url)
                svc = self._service
                if svc is None:
                    svc = get_shared_service(
                        kwargs.get("url") or self.snapshot_url,
                        kwargs.get("min_confidence"),
                    )
                result_box.append(svc.detect(**kwargs))
            except Exception as exc:
                result_box.append(
                    {
                        "ok": False,
                        "error": "DETECT_EXCEPTION",
                        "detail": str(exc),
                        "cx_px": -1,
                        "cy_px": -1,
                        "radius_px": 0,
                        "confidence": 0.0,
                    }
                )

        th = threading.Thread(target=_work, name="xyzcal_detect_once")
        result = {"ok": False, "error": "NO_RESULT"}
        try:
            th.start()
            while th.is_alive():
                eventtime = self.reactor.pause(eventtime + 0.05)
            th.join(timeout=0.1)
            result = result_box[0] if result_box else {"ok": False, "error": "NO_RESULT"}
            self._last_result = result
            self._detect_seq += 1
        finally:
            self._busy = was_busy
        return result

    cmd_XYZCAL_DETECT_help = (
        "Run one nozzle tip detect (OpenCV worker). "
        "Params: URL= RESET_FOLLOW= FLUSH= FRESH= MODE= EXPECTED_X/Y= "
        "MIN_CONF= MIN_RADIUS= MAX_RADIUS= SEARCH_W= SEARCH_H= "
        "SEARCH_DX= SEARCH_DY="
    )

    def cmd_XYZCAL_DETECT(self, gcmd):
        if self._api is None:
            raise gcmd.error(
                "xyzcal_detect unavailable: %s (install opencv-python-headless)"
                % (self._import_error or "import failed")
            )
        url = gcmd.get("URL", self.snapshot_url)
        reset_follow = gcmd.get_int("RESET_FOLLOW", 0)
        flush_n = gcmd.get_int("FLUSH", 0, minval=0, maxval=8)
        fresh = gcmd.get_int("FRESH", 0, minval=0, maxval=1)
        mode = gcmd.get("MODE", None)
        expect_x = gcmd.get_float("EXPECTED_X", None)
        expect_y = gcmd.get_float("EXPECTED_Y", None)
        min_conf = gcmd.get_float(
            "MIN_CONF", self.min_confidence, above=0.0, maxval=1.0
        )
        search_w = gcmd.get_float("SEARCH_W", 80.0, above=8.0)
        search_h = gcmd.get_float("SEARCH_H", 72.0, above=8.0)
        search_dx = gcmd.get_float("SEARCH_DX", None, above=0.0)
        search_dy = gcmd.get_float("SEARCH_DY", None, above=0.0)
        min_radius = gcmd.get_float("MIN_RADIUS", None, above=0.0)
        max_radius = gcmd.get_float("MAX_RADIUS", None, above=0.0)
        body = {
            "url": url,
            "min_confidence": min_conf,
            "search_width_px": search_w,
            "search_height_px": search_h,
            "flush_snapshot_count": flush_n,
            "fresh_frame": bool(fresh),
            "reset_follow": bool(reset_follow),
            "insecure": True,
        }
        if mode:
            body["mode"] = str(mode).strip().lower()
        if search_dx is not None and search_dy is not None:
            body["search_delta_x"] = search_dx
            body["search_delta_y"] = search_dy
        if min_radius is not None:
            body["min_radius_px"] = min_radius
        if max_radius is not None:
            body["max_radius_px"] = max_radius
        body = self._inject_radius_band(body)
        if expect_x is not None and expect_y is not None:
            body["expected_x"] = expect_x
            body["expected_y"] = expect_y
        elif expect_x is not None or expect_y is not None:
            raise gcmd.error("EXPECTED_X and EXPECTED_Y must be set together")
        if (search_dx is None) != (search_dy is None):
            raise gcmd.error("SEARCH_DX and SEARCH_DY must be set together")

        self._busy = True
        try:
            result = self.detect_once(body)
        finally:
            self._busy = False
        ok = bool(result.get("ok"))
        cx = result.get("cx_px", -1)
        cy = result.get("cy_px", -1)
        r = result.get("radius_px", 0)
        conf = result.get("confidence", 0)
        gcmd.respond_info(
            "XYZCAL_DETECT ok=%s cx=%.1f cy=%.1f r=%.1f conf=%.3f seq=%d err=%s"
            % (
                ok,
                float(cx) if cx is not None else -1.0,
                float(cy) if cy is not None else -1.0,
                float(r) if r is not None else 0.0,
                float(conf) if conf is not None else 0.0,
                self._detect_seq,
                result.get("error") or result.get("reject_reason") or "",
            )
        )

    cmd_XYZCAL_DETECT_STATUS_help = (
        "Report last XYZCAL_DETECT result and HTTP bind (alias: XYZCAL_STATUS)"
    )

    def cmd_XYZCAL_DETECT_STATUS(self, gcmd):
        api_ok = self._api is not None
        http_on = self._server is not None
        r = self._last_result or {}
        gcmd.respond_info(
            "xyzcal_detect api=%s busy=%s seq=%s http=%s://%s:%s "
            "last_ok=%s last_cx=%s last_cy=%s last_r=%s conf=%s err=%s"
            % (
                api_ok,
                self._busy,
                self._detect_seq,
                "http" if http_on else "off",
                self.listen_host,
                self.listen_port,
                r.get("ok"),
                r.get("cx_px"),
                r.get("cy_px"),
                r.get("radius_px"),
                r.get("confidence"),
                r.get("error") or r.get("reject_reason") or "",
            )
        )
        if self._import_error:
            gcmd.respond_info("import_error: %s" % self._import_error)

    cmd_XYZCAL_SET_NOZZLE_RADIUS_help = (
        "Set calibrated nozzle rim radius (px). Params: R= TOL= (optional)"
    )

    def cmd_XYZCAL_SET_NOZZLE_RADIUS(self, gcmd):
        r = gcmd.get_float("R", None, above=0.0)
        if r is None:
            raise gcmd.error("XYZCAL_SET_NOZZLE_RADIUS requires R=")
        tol = gcmd.get_float("TOL", self.nozzle_radius_tol, above=0.0, maxval=1.0)
        self.nozzle_radius_px = float(r)
        self.nozzle_radius_tol = float(tol)
        band = self.calibrated_radius_band()
        gcmd.respond_info(
            "XYZCAL_SET_NOZZLE_RADIUS r=%.2f tol=%.3f band=%.2f..%.2f"
            % (self.nozzle_radius_px, self.nozzle_radius_tol, band[0], band[1])
        )

    def build_profile_from_center(self, calib, tool=ToolType.MAIN):
        """Fill Profile + Point from a successful Center run on calib."""
        url = str(self.snapshot_url or "")
        if not url:
            url = str(getattr(calib, "snapshot_url", "") or "")
        try:
            tol = float(self.nozzle_radius_tol or 0.15)
        except (TypeError, ValueError):
            tol = 0.15

        fw = float(getattr(calib, "_last_fw", 0) or 0)
        fh = float(getattr(calib, "_last_fh", 0) or 0)
        if fw < 40 or fh < 40:
            fw, fh = 640.0, 480.0

        # Match xyzcal_calib._detect hard window 80x72
        search_dx = 40.0
        search_dy = 36.0

        r = float(getattr(calib, "_last_r", 0) or 0)
        if r > 0.0:
            half = max(1.5, r * tol)
            r_min = max(5.0, r - half)
            r_max = min(60.0, r + half)
        else:
            r_min, r_max = 10.0, 17.0

        if getattr(calib, "_fity_vy_ok", False):
            vx = float(getattr(calib, "_fity_vy_x", 0) or 0)
            vy = float(getattr(calib, "_fity_vy_y", 0) or 0)
        else:
            vx = float(getattr(calib, "_vy_x", 0) or 0)
            vy = float(getattr(calib, "_vy_y", 0) or 0)
        calib_px_mm = math.hypot(vx, vy)
        if not math.isfinite(calib_px_mm) or calib_px_mm < 1.0:
            calib_px_mm = 0.0

        conf = float(getattr(calib, "min_confidence", self.min_confidence) or self.min_confidence)

        self.profile = NozzleDetectionProfile(
            snapshot_url=url,
            target_pixel_x=0.5 * fw,
            target_pixel_y=0.5 * fh,
            search_delta_x=search_dx,
            search_delta_y=search_dy,
            radius_min_px=r_min,
            radius_max_px=r_max,
            calib_px_mm=calib_px_mm,
            min_confidence=conf,
            fresh_frame_flush_count=2,
        )

        cx = float(getattr(calib, "_last_cx", -1) or -1)
        cy = float(getattr(calib, "_last_cy", -1) or -1)
        detected = cx >= 0.0 and cy >= 0.0 and r > 0.0
        thx = getattr(calib, "_toolhead_x", None)
        thy = getattr(calib, "_toolhead_y", None)
        thz = getattr(calib, "_toolhead_z", None)
        self.point = XYZPixelPoint(
            x=float(thx) if thx is not None else 0.0,
            y=float(thy) if thy is not None else 0.0,
            z=float(thz) if thz is not None else 0.0,
            tool=tool if isinstance(tool, ToolType) else _parse_tool(tool),
            pixel_x=cx if detected else None,
            pixel_y=cy if detected else None,
            detected=detected,
        )

        ex = getattr(calib, "_last_expect_x", None)
        ey = getattr(calib, "_last_expect_y", None)
        self.last_detect = DetectCallContext(
            expected_pixel_x=float(ex) if ex is not None else None,
            expected_pixel_y=float(ey) if ey is not None else None,
            mode="track",
            reset_tracker=False,
        )
        self.session_ready = True
        self._session_message = "center_ok"
        return self.profile, self.point

    def apply_radius_from_profile(self):
        """Push tip radius into runtime so later Detect uses the band."""
        if not self.session_ready or self.profile is None or self.point is None:
            return
        if not self.point.detected:
            return
        r_min = float(self.profile.radius_min_px)
        r_max = float(self.profile.radius_max_px)
        r = 0.5 * (r_min + r_max)
        calib = self.printer.lookup_object("xyzcal_calib", None)
        if calib is not None:
            tip_r = float(getattr(calib, "_last_r", 0) or 0)
            if tip_r > 0.0:
                r = tip_r
        self.nozzle_radius_px = float(r)
        # Center 后略放宽 tol，避免 tip±15% 过窄导致随后 track 拒检
        self.nozzle_radius_tol = max(float(self.nozzle_radius_tol or 0.15), 0.20)
        band = self.calibrated_radius_band()
        logging.info(
            "xyzcal_detect: profile nozzle_radius_px=%.2f band=%s tol=%.3f",
            r,
            band,
            self.nozzle_radius_tol,
        )

    def on_center_done(self, calib, gcmd, tool=ToolType.MAIN):
        tool_e = tool if isinstance(tool, ToolType) else _parse_tool(tool)
        self.build_profile_from_center(calib, tool=tool_e)
        self.apply_radius_from_profile()
        self.emit_profile(gcmd)

    def emit_profile(self, gcmd):
        if not self.session_ready or self.profile is None or self.point is None:
            gcmd.respond_info("XYZCAL_PROFILE ready=0 (run XYZCAL_CENTER first)")
            return
        p = self.profile
        pt = self.point
        d = self.last_detect
        gcmd.respond_info(
            "XYZCAL_PROFILE profile "
            "snapshot_url=%s "
            "target_pixel_x=%.1f target_pixel_y=%.1f "
            "search_delta_x=%.1f search_delta_y=%.1f "
            "radius_min_px=%.2f radius_max_px=%.2f "
            "calib_px_mm=%.3f "
            "min_confidence=%.3f "
            "fresh_frame_flush_count=%d"
            % (
                p.snapshot_url,
                p.target_pixel_x,
                p.target_pixel_y,
                p.search_delta_x,
                p.search_delta_y,
                p.radius_min_px,
                p.radius_max_px,
                p.calib_px_mm,
                p.min_confidence,
                p.fresh_frame_flush_count,
            )
        )
        gcmd.respond_info(
            "XYZCAL_PROFILE point "
            "x=%.3f y=%.3f z=%.3f "
            "tool=%s "
            "pixel_x=%s pixel_y=%s "
            "detected=%d"
            % (
                pt.x,
                pt.y,
                pt.z,
                pt.tool.value,
                ("%.2f" % pt.pixel_x) if pt.pixel_x is not None else "nan",
                ("%.2f" % pt.pixel_y) if pt.pixel_y is not None else "nan",
                1 if pt.detected else 0,
            )
        )
        gcmd.respond_info(
            "XYZCAL_PROFILE detect "
            "expected_pixel_x=%s expected_pixel_y=%s "
            "mode=%s reset_tracker=%d"
            % (
                ("%.2f" % d.expected_pixel_x)
                if d.expected_pixel_x is not None
                else "nan",
                ("%.2f" % d.expected_pixel_y)
                if d.expected_pixel_y is not None
                else "nan",
                d.mode or "",
                1 if d.reset_tracker else 0,
            )
        )

    cmd_XYZCAL_PROFILE_DUMP_help = (
        "Dump last Center Profile + Point (alias: XYZCAL_SESSION_DUMP). No params."
    )

    def cmd_XYZCAL_PROFILE_DUMP(self, gcmd):
        self.emit_profile(gcmd)

    def _status_list(self, value):
        if value is None:
            return None
        if isinstance(value, (list, tuple)):
            out = []
            for item in value:
                try:
                    out.append(float(item))
                except (TypeError, ValueError):
                    return None
            return out
        return None

    def _profile_status(self):
        out = {
            "session_ready": bool(self.session_ready),
            "session_message": self._session_message or "",
            # Compat keys previously on xyzcal_session
            "ready": bool(self.session_ready),
            "message": self._session_message or "",
        }
        if self.profile is not None:
            for k, v in asdict(self.profile).items():
                out["profile_%s" % k] = v
        else:
            for k in _PROFILE_STATUS_KEYS:
                out["profile_%s" % k] = None if k == "snapshot_url" else 0
        if self.point is not None:
            out["point_x"] = self.point.x
            out["point_y"] = self.point.y
            out["point_z"] = self.point.z
            out["point_tool"] = self.point.tool.value
            out["point_pixel_x"] = self.point.pixel_x
            out["point_pixel_y"] = self.point.pixel_y
            out["point_detected"] = bool(self.point.detected)
        else:
            out.update(
                {
                    "point_x": 0.0,
                    "point_y": 0.0,
                    "point_z": 0.0,
                    "point_tool": "",
                    "point_pixel_x": None,
                    "point_pixel_y": None,
                    "point_detected": False,
                }
            )
        d = self.last_detect
        out["detect_expected_x"] = d.expected_pixel_x
        out["detect_expected_y"] = d.expected_pixel_y
        out["detect_mode"] = d.mode or ""
        out["detect_reset_tracker"] = bool(d.reset_tracker)
        return out

    def get_status(self, eventtime=None):
        r = self._last_result or {}
        # Moonraker objects/query：同时提供 last_* 与 HTTP 同名字段，方便屏端映射
        out = {
            "http_enabled": bool(self.enable_http and self.listen_port > 0),
            "listen_host": self.listen_host,
            "listen_port": self.listen_port,
            "snapshot_url": self.snapshot_url,
            "nozzle_radius_px": float(self.nozzle_radius_px or 0.0),
            "nozzle_radius_tol": float(self.nozzle_radius_tol or 0.15),
            "api_ready": self._api is not None,
            "import_error": self._import_error or "",
            "busy": bool(self._busy),
            "detect_seq": int(self._detect_seq),
            "last_ok": bool(r.get("ok")),
            "last_cx": r.get("cx_px", -1),
            "last_cy": r.get("cy_px", -1),
            "last_radius": r.get("radius_px", 0),
            "last_confidence": r.get("confidence", 0.0),
            "last_frame_w": r.get("frame_w", 0),
            "last_frame_h": r.get("frame_h", 0),
            "last_error": r.get("error") or "",
            "last_detail": r.get("detail") or "",
            "last_reject_reason": r.get("reject_reason") or "",
            "last_allowed_roi": self._status_list(r.get("allowed_roi")),
            "last_allowed_circle": self._status_list(r.get("allowed_circle")),
            # HTTP-shaped aliases (Phase 2 UI)
            "ok": bool(r.get("ok")),
            "cx_px": r.get("cx_px", -1),
            "cy_px": r.get("cy_px", -1),
            "radius_px": r.get("radius_px", 0),
            "confidence": r.get("confidence", 0.0),
            "frame_w": r.get("frame_w", 0),
            "frame_h": r.get("frame_h", 0),
            "error": r.get("error") or "",
            "detail": r.get("detail") or "",
            "reject_reason": r.get("reject_reason") or "",
            "allowed_roi": self._status_list(r.get("allowed_roi")),
            "allowed_circle": self._status_list(r.get("allowed_circle")),
            "algorithm": getattr(self._api, "ALGORITHM_VERSION", "")
            if self._api
            else "",
            "service_version": getattr(self._api, "SERVICE_VERSION", "")
            if self._api
            else "",
        }
        out.update(self._profile_status())
        return out


def load_config(config):
    return XyCalDetect(config)


def main():
    ap = argparse.ArgumentParser(description="XY calib nozzle detect HTTP server")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument(
        "--snapshot",
        default=DEFAULT_SNAPSHOT,
        help="fallback snapshot URL when POST body omits url (device-specific)",
    )
    args = ap.parse_args()

    server = create_http_server(args.host, args.port, args.snapshot)
    print(
        "xyzcal-detect (extras/xyzcal_detect) listening on http://%s:%d  fallback_snapshot=%s"
        % (args.host, args.port, args.snapshot),
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nbye", flush=True)
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
