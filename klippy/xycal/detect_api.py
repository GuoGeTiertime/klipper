#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""XY 标定喷嘴检测核心（供 Tierklipper [xycal_detect] 与独立 HTTP 共用）。

分层：
  detect_nozzle(image, …)     — 纯图像算法（无 URL / 无 Tracker）
  NozzleDetectionService      — 截图、fresh_frame、Tracker，再调 detect_nozzle

线协议：SERVICE_VERSION 不变；flush_snapshot_count / reset_follow 仍可用，
fresh_frame / mode / search_delta_* 为语义别名。
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import parse_qsl, urlencode, urlparse, urlunparse

import cv2

try:
    from http.server import ThreadingHTTPServer as _Server
except ImportError:  # pragma: no cover
    _Server = HTTPServer

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import nozzle_detector as _nozzle_detector  # noqa: E402
import webcam_detect as _webcam_detect  # noqa: E402
from nozzle_detector import (  # noqa: E402
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
            _webcam_detect.fetch_snapshot(
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
        bgr = _webcam_detect.fetch_snapshot(
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
                    bgr = _webcam_detect.fetch_snapshot(
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
                min_radius_px if min_radius_px is not None else 13.5,
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
WEBCAM_PATH = Path(_webcam_detect.__file__).resolve()
DETECTOR_SHA256 = _sha256_file(DETECTOR_PATH)
WEBCAM_SHA256 = _sha256_file(WEBCAM_PATH)


def _runtime_metadata() -> dict:
    """Return enough identity data to prove which backend is actually running."""
    return {
        "service": "xycal-detect",
        "service_version": SERVICE_VERSION,
        "algorithm": ALGORITHM_VERSION,
        "detector_sha256": DETECTOR_SHA256,
        "webcam_sha256": WEBCAM_SHA256,
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
                health.update({"ok": True, "source": "webcam_detect.detect_from_url"})
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
        "xycal-detect (klippy/xycal) listening on http://%s:%d  fallback_snapshot=%s"
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
