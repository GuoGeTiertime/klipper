#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""XY 标定喷嘴检测核心（供 Tierklipper [xycal_detect] 与独立 HTTP 共用）。

检测来源统一走 V6-R16-A133：fetch_snapshot
  → NozzleTracker 中心 80x72 硬窗、内切 r=36；expected 同尺寸平移。
本文件从 ScreenQML xycal_detect_server.py 迁入 klippy/xycal/；
SERVICE_VERSION 须与 ScreenQML 线协议保持一致。
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
from nozzle_detector import NozzleTracker, draw_detection, _write_image  # noqa: E402

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
    global _TRACKER, _TRACKER_KEY
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

    # 设备自己的 snapshot；空则用启动参数 --snapshot
    url = str(body.get("url") or default_url or "").strip()
    if not url:
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

    min_conf = body.get("min_confidence")
    try:
        min_conf = float(min_conf) if min_conf is not None else MIN_CONFIDENCE_DEFAULT
    except (TypeError, ValueError):
        min_conf = MIN_CONFIDENCE_DEFAULT

    def _optional_positive_float(name: str):
        value = body.get(name)
        if value is None:
            return None
        parsed = float(value)
        if not math.isfinite(parsed) or parsed <= 0:
            raise ValueError("%s must be positive" % name)
        return parsed

    try:
        min_radius_px = _optional_positive_float("min_radius_px")
        max_radius_px = _optional_positive_float("max_radius_px")
        if (
            min_radius_px is not None
            and max_radius_px is not None
            and min_radius_px > max_radius_px
        ):
            raise ValueError("min_radius_px > max_radius_px")
    except (TypeError, ValueError) as exc:
        return _result({
            "ok": False,
            "error": "BAD_RADIUS_RANGE",
            "detail": str(exc),
            "cx_px": -1,
            "cy_px": -1,
            "radius_px": 0,
            "confidence": 0.0,
            "frame_w": 0,
            "frame_h": 0,
        })

    try:
        flush_snapshot_count = int(body.get("flush_snapshot_count") or 0)
    except (TypeError, ValueError):
        flush_snapshot_count = 0
    flush_snapshot_count = max(0, min(8, flush_snapshot_count))

    # HTTPS 自签证书常见于穿透/板端；与 webcam_detect --insecure 一致可选
    allow_insecure = bool(body.get("insecure")) if "insecure" in body else True

    # R16-A133：固定裁剪框 80x72，真正有效区是其 r=36 最大内切圆。
    # ScreenQML：expected 为绝对硬范围；allow_fixed640_relocation=False，
    # 拒检不逃逸到金属面定位/背景圆。view_zoom 仅兼容读取，不改物理尺寸。
    try:
        view_zoom = float(body.get("view_zoom") or 1.0)
        if not math.isfinite(view_zoom) or view_zoom < 1.0:
            raise ValueError
    except (TypeError, ValueError):
        view_zoom = 1.0

    search_width = body.get("search_width_px")
    search_height = body.get("search_height_px")
    if (search_width is None) != (search_height is None):
        return _result({
            "ok": False,
            "error": "BAD_SEARCH_SIZE",
            "detail": "search_width_px 和 search_height_px 必须同时提供",
            "cx_px": -1,
            "cy_px": -1,
            "radius_px": 0,
            "confidence": 0.0,
            "frame_w": 0,
            "frame_h": 0,
            "follow_roi": None,
            "allowed_roi": None,
            "allowed_circle": None,
            "reject_reason": "位置搜索范围不完整",
        })
    try:
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
                raise ValueError
            position_window_width = min(160.0, position_window_width)
            position_window_height = min(144.0, position_window_height)
    except (TypeError, ValueError):
        return _result({
            "ok": False,
            "error": "BAD_SEARCH_SIZE",
            "detail": "位置搜索窗口宽高必须是至少 32px 的有限数值",
            "cx_px": -1,
            "cy_px": -1,
            "radius_px": 0,
            "confidence": 0.0,
            "frame_w": 0,
            "frame_h": 0,
            "follow_roi": None,
            "allowed_roi": None,
            "allowed_circle": None,
            "reject_reason": "位置搜索范围无效",
        })

    expected_x = body.get("expected_x")
    expected_y = body.get("expected_y")
    if (expected_x is None) != (expected_y is None):
        return _result({
            "ok": False,
            "error": "BAD_EXPECTED",
            "detail": "expected_x 和 expected_y 必须同时提供",
            "cx_px": -1,
            "cy_px": -1,
            "radius_px": 0,
            "confidence": 0.0,
            "frame_w": 0,
            "frame_h": 0,
            "follow_roi": None,
            "allowed_roi": None,
            "allowed_circle": None,
            "reject_reason": "expected 坐标不完整",
        })
    expected_center = None
    if expected_x is not None:
        try:
            expected_center = (float(expected_x), float(expected_y))
            if not all(math.isfinite(value) for value in expected_center):
                raise ValueError
        except (TypeError, ValueError):
            return _result({
                "ok": False,
                "error": "BAD_EXPECTED",
                "detail": "expected_x/y 必须是有限像素坐标",
                "cx_px": -1,
                "cy_px": -1,
                "radius_px": 0,
                "confidence": 0.0,
                "frame_w": 0,
                "frame_h": 0,
                "follow_roi": None,
                "allowed_roi": None,
                "allowed_circle": None,
                "reject_reason": "expected 坐标无效",
            })

    def _flush_snapshots(count, tag_prefix):
        for index in range(max(0, int(count))):
            _webcam_detect.fetch_snapshot(
                _cache_busted_url(url, "%s-%d" % (tag_prefix, index)),
                timeout=15.0,
                allow_insecure_certificate=allow_insecure,
            )
            time.sleep(0.12)

    def _tip_looks_stale_vs_expected(frame_w, frame_h, tip_xy, expect_xy):
        # expected 已明显离十字，tip 仍贴十字 → 多半是移动前旧帧。
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
    total_flushed = flush_snapshot_count
    try:
        with _TRACKER_LOCK:
            # GCode 的 M400 只说明机床到位，不代表 MJPEG/snapshot 缓冲已换帧。
            # 移动后丢弃若干张并稍等相机产生下一帧，再将最终帧交给 R16。
            _flush_snapshots(flush_snapshot_count, "flush")
            bgr = _webcam_detect.fetch_snapshot(
                _cache_busted_url(url, "detect"),
                timeout=15.0,
                allow_insecure_certificate=allow_insecure,
            )
            tracker_key = (
                _tracker_source_key(url),
                round(min_conf, 4),
                round(position_window_width, 2),
                round(position_window_height, 2),
                None if min_radius_px is None else round(min_radius_px, 3),
                None if max_radius_px is None else round(max_radius_px, 3),
            )
            if (
                _TRACKER is None
                or _TRACKER_KEY != tracker_key
                or bool(body.get("reset_follow"))
            ):
                _TRACKER = NozzleTracker(
                    min_confidence=min_conf,
                    strict_position_lock=True,
                    position_window_width_px=position_window_width,
                    position_window_height_px=position_window_height,
                    min_radius_px=min_radius_px,
                    max_radius_px=max_radius_px,
                    allow_fixed640_relocation=False,
                )
                _TRACKER_KEY = tracker_key
            tracker = _TRACKER
            det = tracker.update(bgr, expected_center=expected_center)
            # expected 远离中心但 tip 仍贴中心 → 再刷再检（最多 1 轮）
            while stale_flush_retries < 1:
                fh, fw = bgr.shape[:2]
                tip_xy = (float(det.circle.x), float(det.circle.y))
                if not _tip_looks_stale_vs_expected(fw, fh, tip_xy, expected_center):
                    break
                stale_flush_retries += 1
                extra = 2
                _flush_snapshots(extra, "stale-%d" % stale_flush_retries)
                total_flushed += extra
                bgr = _webcam_detect.fetch_snapshot(
                    _cache_busted_url(url, "detect-stale-%d" % stale_flush_retries),
                    timeout=15.0,
                    allow_insecure_certificate=allow_insecure,
                )
                det = tracker.update(bgr, expected_center=expected_center)
    except (RuntimeError, ValueError, OSError) as e:
        msg = str(e)
        err = "FETCH_FAIL" if ("快照" in msg or "URL" in msg or "图片" in msg) else "NO_DETECT"
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
            "tracker_status": "unavailable" if tracker is None else tracker.last_status,
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
        })

    h, w = bgr.shape[:2]
    conf = float(det.confidence)
    overlay_file = ""
    overlay_jpeg_b64 = ""
    # QML 用坐标自行画圈；A133 连续识别默认不再每帧写盘+JPEG+base64。
    if bool(body.get("include_overlay")):
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
    })


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
