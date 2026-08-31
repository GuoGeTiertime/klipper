#!/usr/bin/env python3
"""Fetch one printer-camera snapshot and detect the central nozzle orifice."""

from __future__ import annotations

import argparse
import json
import ssl
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np

try:
    import certifi
except ImportError:  # A configured operating-system CA store may still work.
    certifi = None

from nozzle_detector import (
    Detection,
    _open_result,
    _write_image,
    detect_nozzle,
    draw_detection,
)


DEFAULT_SNAPSHOT_URL = "https://7342904.tiertime.vip/webcam2/?action=snapshot"


def fetch_snapshot(
    url: str,
    timeout: float = 15.0,
    allow_insecure_certificate: bool = False,
) -> np.ndarray:
    """Download and decode one uncached JPEG/PNG snapshot."""
    parsed = urllib.parse.urlsplit(url)
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise ValueError("摄像头地址必须是有效的 http:// 或 https:// URL")
    separator = "&" if parsed.query else "?"
    fresh_url = f"{url}{separator}_nozzle_ts={time.time_ns()}"
    request = urllib.request.Request(
        fresh_url,
        headers={
            "User-Agent": "NozzleDetector-V6-R16/1.0",
            "Cache-Control": "no-cache",
            "Pragma": "no-cache",
        },
    )
    if allow_insecure_certificate:
        ssl_context = ssl._create_unverified_context()  # noqa: SLF001 - explicit camera opt-in
    elif certifi is not None:
        # Python installations on Windows, Linux and macOS do not always use
        # the same operating-system CA store.  An explicit Mozilla CA bundle
        # keeps the verified camera connection portable and fixes false
        # CERTIFICATE_VERIFY_FAILED errors without disabling TLS validation.
        ssl_context = ssl.create_default_context(cafile=certifi.where())
    else:
        ssl_context = ssl.create_default_context()
    try:
        with urllib.request.urlopen(
            request,
            timeout=timeout,
            context=ssl_context,
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
        raise RuntimeError(f"无法取得摄像头快照：{exc}{certificate_hint}") from exc
    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("摄像头返回内容不是有效图片")
    return image


def process_webcam(
    url: str,
    output_path: Path,
    original_path: Optional[Path] = None,
    timeout: float = 15.0,
    min_confidence: float = 0.36,
    roi_width_fraction: float = 0.50,
    roi_height_fraction: float = 0.50,
    fast_camera_mode: bool = True,
    fallback_general: bool = True,
    open_result: bool = True,
    allow_insecure_certificate: bool = False,
    adaptive_vertical_roi: bool = True,
    min_radius_px: Optional[float] = None,
    max_radius_px: Optional[float] = None,
    search_bounds: Optional[tuple[float, float, float, float]] = None,
) -> Detection:
    started = time.perf_counter()
    image = fetch_snapshot(
        url,
        timeout=timeout,
        allow_insecure_certificate=allow_insecure_certificate,
    )
    if original_path is not None:
        _write_image(original_path.expanduser().resolve(), image)
    detection = detect_nozzle(
        image,
        min_confidence=min_confidence,
        roi_width_fraction=roi_width_fraction,
        roi_height_fraction=roi_height_fraction,
        fast_camera_mode=fast_camera_mode,
        fallback_general=fallback_general,
        adaptive_vertical_roi=adaptive_vertical_roi,
        min_radius_px=min_radius_px,
        max_radius_px=max_radius_px,
        search_bounds=search_bounds,
    )
    output_path = output_path.expanduser().resolve()
    _write_image(output_path, draw_detection(image, detection))
    if open_result:
        _open_result(output_path)
    result = Detection(
        circle=detection.circle,
        confidence=detection.confidence,
        score=detection.score,
        output_path=output_path,
    )
    # Store elapsed time on the function for CLI reporting without changing
    # the stable Detection data structure used by existing callers.
    process_webcam.last_elapsed_ms = (time.perf_counter() - started) * 1000.0
    return result


process_webcam.last_elapsed_ms = 0.0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="抓取打印机摄像头快照，识别喷嘴本体外圆边沿。"
    )
    parser.add_argument("--url", default=DEFAULT_SNAPSHOT_URL, help="摄像头 snapshot URL")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("webcam_circle.jpg"),
        help="标注结果路径（默认：当前目录 webcam_circle.jpg）",
    )
    parser.add_argument("--save-original", type=Path, help="可选：同时保存本次原始快照")
    parser.add_argument("--timeout", type=float, default=15.0, help="网络超时秒数")
    parser.add_argument(
        "--insecure",
        action="store_true",
        help="允许摄像头使用无法验证的 HTTPS 证书（仅用于可信设备）",
    )
    parser.add_argument("--min-confidence", type=float, default=0.36)
    parser.add_argument("--roi-width", type=float, default=0.50)
    parser.add_argument("--roi-height", type=float, default=0.50)
    parser.add_argument("--general-mode", action="store_true")
    parser.add_argument("--no-fallback", action="store_true")
    parser.add_argument(
        "--fixed-center",
        action="store_true",
        help="固定使用几何中心 ROI，不随打印头上下位置选择窗口",
    )
    parser.add_argument(
        "--min-radius-px",
        type=float,
        help="可选：真机喷口最小半径（像素）",
    )
    parser.add_argument(
        "--max-radius-px",
        type=float,
        help="可选：真机喷口最大半径（像素）",
    )
    parser.add_argument("--expected-x", type=float, help="可选：喷口预期圆心 x（像素）")
    parser.add_argument("--expected-y", type=float, help="可选：喷口预期圆心 y（像素）")
    parser.add_argument("--search-width-px", type=float, help="硬搜索窗口宽度（像素）")
    parser.add_argument("--search-height-px", type=float, help="硬搜索窗口高度（像素）")
    parser.add_argument("--no-open", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        search_values = (
            args.expected_x,
            args.expected_y,
            args.search_width_px,
            args.search_height_px,
        )
        if any(value is not None for value in search_values) and not all(
            value is not None for value in search_values
        ):
            raise ValueError(
                "expected-x、expected-y、search-width-px、search-height-px 必须同时提供"
            )
        search_bounds = None
        if all(value is not None for value in search_values):
            assert args.search_width_px is not None
            assert args.search_height_px is not None
            if args.search_width_px <= 0.0 or args.search_height_px <= 0.0:
                raise ValueError("搜索窗口宽高必须为正数")
            search_bounds = (
                args.expected_x - 0.5 * args.search_width_px,
                args.expected_y - 0.5 * args.search_height_px,
                args.expected_x + 0.5 * args.search_width_px,
                args.expected_y + 0.5 * args.search_height_px,
            )
        detection = process_webcam(
            args.url,
            output_path=args.output,
            original_path=args.save_original,
            timeout=args.timeout,
            min_confidence=args.min_confidence,
            roi_width_fraction=args.roi_width,
            roi_height_fraction=args.roi_height,
            fast_camera_mode=not args.general_mode,
            fallback_general=not args.no_fallback,
            open_result=not args.no_open,
            allow_insecure_certificate=args.insecure,
            adaptive_vertical_roi=not args.fixed_center,
            min_radius_px=args.min_radius_px,
            max_radius_px=args.max_radius_px,
            search_bounds=search_bounds,
        )
    except (ValueError, OSError, RuntimeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2
    payload = {
        "x": round(detection.circle.x, 2),
        "y": round(detection.circle.y, 2),
        "radius": round(detection.circle.radius, 2),
        "confidence": round(detection.confidence, 4),
        "elapsed_ms": round(process_webcam.last_elapsed_ms, 1),
        "output": str(detection.output_path),
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
