#!/usr/bin/env python3
"""Detect exactly one nozzle/cavity outer-edge circle in an image (V6-R16).

V3 combines the tuned test-pack pipeline (tip/large arbitration, anti-drift
outer-edge fit, collage-ready batching) with the dark-cavity V2 modules
(high-resolution tip Hough, small-radius snap, soft-shadow aperture recovery).

Resolution and aspect ratio are not fixed: working size is capped by longest
side with aspect preserved; tip/large gates and Hough radii use short-side
ratios; outputs are mapped back to original-image pixels. V4 adds a central
1/2 x 1/2 camera ROI, a fast small-orifice candidate range, automatic general
search fallback, and dark-hole polishing for stable sub-pixel centres.
V5 adds complete-rim evidence for grey/red pin appearances and relaxes only
well-supported small-aperture gates so residue and background arcs cannot win.
V6 makes the dark square housing optional by adding multi-threshold metal-hex
faces and circle-in-hex nesting, plus deterministic specular-glare regression.
R5 replaces highlight-weighted final fitting with locally normalized opposite-
ray geometry so asymmetric lighting cannot pull the small-aperture circle.
R6 adds an automatic LED-fill branch: it locks the reflective hex face first,
then fits the bright circular nozzle rim from its dark-core / bright-ring /
metal-background radial topology.  The original no-fill pipeline is unchanged.
R7 adds candidate-centred metal-face containment, optional hard search bounds,
uniqueness checks, and a stateful lock for continuous frames so background rings
cannot replace an already locked nozzle.
R11 restores the required target to the complete outer nozzle edge.  The centre
hole is never required because material may hide it.  LED candidates are ranked
by diametrically opposed full-circle support; exact-six is preferred, while
3--6 visible metal-face sides may support a strongly closed circle when residue
or glare obscures the remaining edges.
R12 adds a proof-gated fast path for unambiguous LED frames.  It returns early
only when one exact-six candidate has exceptionally strong lower-quartile,
opposed, and sector-wide rim evidence and no strong candidate exists on another
face.  Every ambiguous/material/glare frame falls back to the complete R11
multi-start path unchanged.
R12.2 adds a calibrated 640x480 dual-nozzle profile.  Only the selected
central/right physical bay may output a circle, its final radius must be
13.5--17 px, and failure of the LED/hex proof is final rather than falling back
to the second nozzle or a background ring.
R15.1 keeps that small physical radius and uses a fixed circular position gate.
This A133 profile uses an 80x72 crop centred at (320,240), whose largest
inscribed circle (radius 36 px) is the true allowed region.  Afterwards the
same-size circle only translates with expected/follow and never expands.
The complete nozzle circle must fit inside it.  A strongly
closed concentric rim may track without a complete hex on every moving frame;
initial acquisition still requires 3--6 sided local metal support.
R16 keeps the fixed gate and 13.5--17 px final radius, but fixes integration when
the caller supplies no expected position.  If the current gate misses after a
carriage move, a bounded primary-work-area locator must first prove the local
3--6 sided metal face; the same rim is then proved again inside a translated
same-size gate.  A supplied expected centre remains authoritative and can never
escape to this relocation path.  Large predicted steps likewise require tight
agreement between local-rim and independent metal-face proofs.
R16-A133 keeps every final radius/closure/position gate but removes redundant
work: LAB and LED preparation are cropped to the physical context, angular
vectors are reused, clear proofs stop after one refined seed, and predicted
frames do not repeat the expensive acquisition-only multi-threshold face scan.
Weak, glare-covered or residue-covered acquisition frames automatically fall
back to the complete robust 3--6-sided R16 proof.
R16-A133.1 fixes long-running fixed-window recovery.  An expected-position miss
invalidates the stale visual lock immediately and latches that expected centre
for a small number of camera-delay frames.  Success establishes a fresh lock;
continued misses expire the latch and return to the original centre 80x72 gate.
The gate never expands and no background/global circle fallback is enabled.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
import platform
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

import cv2
import numpy as np

def _configure_embedded_opencv() -> None:
    """Avoid OpenCV oversubscription on small Linux ARM boards such as A133."""
    machine = platform.machine().lower()
    if not sys.platform.startswith("linux") or machine not in {
        "aarch64",
        "arm64",
        "armv7l",
        "armv8l",
    }:
        return
    try:
        thread_count = int(os.environ.get("NOZZLE_OPENCV_THREADS", "2"))
    except ValueError:
        thread_count = 2
    cv2.setNumThreads(max(1, min(4, thread_count)))
    cv2.setUseOptimized(True)
    try:
        cv2.ocl.setUseOpenCL(False)
    except (AttributeError, cv2.error):
        pass

_configure_embedded_opencv()

@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float

@dataclass(frozen=True)
class Detection:
    circle: Circle
    confidence: float
    score: float
    output_path: Optional[Path] = None


@dataclass(frozen=True)
class CameraProfile:
    """Per-camera defaults; not per-call knobs.

    ``default_search_delta`` is half-width / half-height in pixels
    (e.g. (40, 36) → 80×72 window).
    """

    frame_size: tuple[int, int] = (640, 480)
    allowed_bounds: Optional[tuple[float, float, float, float]] = (
        160.0,
        96.0,
        480.0,
        384.0,
    )
    default_search_delta: tuple[float, float] = (40.0, 36.0)
    # 默认先宽门控便于首检；设备校准后再收窄
    default_radius_range: tuple[float, float] = (10.0, 17.0)
    require_hex_lock: bool = True
    allow_fixed640_relocation: bool = False

    @classmethod
    def from_image(cls, image: np.ndarray) -> "CameraProfile":
        h, w = int(image.shape[0]), int(image.shape[1])
        if w == 640 and h == 480:
            return cls(
                frame_size=(640, 480),
                allowed_bounds=(160.0, 96.0, 480.0, 384.0),
                default_search_delta=(40.0, 36.0),
                default_radius_range=(10.0, 17.0),
                require_hex_lock=True,
                allow_fixed640_relocation=False,
            )
        if w == 1280 and h == 720:
            return cls(
                frame_size=(1280, 720),
                allowed_bounds=(320.0, 180.0, 960.0, 540.0),
                default_search_delta=(40.0, 36.0),
                default_radius_range=(18.0, 32.0),
                require_hex_lock=True,
                allow_fixed640_relocation=False,
            )
        return cls(
            frame_size=(w, h),
            allowed_bounds=None,
            default_search_delta=(40.0, 36.0),
            default_radius_range=(10.0, 17.0),
            require_hex_lock=False,
            allow_fixed640_relocation=True,
        )


def _normalize_detect_mode(mode: Optional[str], has_expected: bool) -> str:
    raw = str(mode or "").strip().lower()
    if raw in ("acquire", "track", "reacquire"):
        return raw
    # Legacy: expected ⇒ track, else acquire
    return "track" if has_expected else "acquire"


def _search_bounds_from_delta(
    expected_center: tuple[float, float],
    search_delta: tuple[float, float],
) -> tuple[float, float, float, float]:
    cx, cy = float(expected_center[0]), float(expected_center[1])
    dx, dy = float(search_delta[0]), float(search_delta[1])
    if not all(math.isfinite(v) for v in (cx, cy, dx, dy)) or dx <= 0 or dy <= 0:
        raise ValueError("search_delta 必须是正的半宽/半高")
    return (cx - dx, cy - dy, cx + dx, cy + dy)


class _HexLockError(RuntimeError):
    """Camera-mode candidate failed the local metal-face containment gate."""

@dataclass
class _PreparedImage:
    color: np.ndarray
    gray: np.ndarray
    saturation: np.ndarray
    smooth: np.ndarray
    grad_x: np.ndarray
    grad_y: np.ndarray
    grad_reference: float
    dynamic_range: float
    scale: float

@dataclass(frozen=True)
class _ScoredCircle:
    circle: Circle
    score: float
    contrast: float
    contrast_coverage: float
    edge_coverage: float
    edge_strength: float
    core_contrast: float
    core_level: float
    core_saturation: float

@dataclass(frozen=True)
class _HoughHypothesis:
    circle: Circle
    accumulator_threshold: int
    features: _ScoredCircle

@dataclass(frozen=True)
class _HexFace:
    x: float
    y: float
    size: float
    score: float

@dataclass(frozen=True)
class _LocalHexSupport:
    x: float
    y: float
    size: float
    score: float
    vertices: int
    containment_margin: float

@dataclass(frozen=True)
class _RimClosureSupport:
    """Angular evidence that a bright nozzle edge is a complete circle."""

    median_drop: float
    lower_drop: float
    coverage: float
    opposed_coverage: float
    sector_coverage: float

def _rank_scored(item: _ScoredCircle) -> float:
    return item.score + 0.80 * item.edge_coverage + 0.40 * item.contrast_coverage

def _read_image(path: Path) -> np.ndarray:
    """Read paths containing Chinese or other non-ASCII characters."""
    try:
        encoded = np.fromfile(str(path), dtype=np.uint8)
    except OSError as exc:
        raise ValueError(f"无法读取图片：{path}") from exc
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"不是受支持的图片或文件已损坏：{path}")
    return image

def _write_image(path: Path, image: np.ndarray) -> None:
    suffix = path.suffix.lower() or ".png"
    if suffix not in {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"}:
        suffix = ".png"
        path = path.with_suffix(suffix)
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        raise OSError(f"无法编码结果图片：{path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded.tofile(str(path))

def _prepare(image: np.ndarray, max_working_side: int = 1100) -> _PreparedImage:
    height, width = image.shape[:2]
    scale = min(1.0, max_working_side / float(max(height, width)))
    if scale < 1.0:
        working = cv2.resize(
            image,
            (max(1, round(width * scale)), max(1, round(height * scale))),
            interpolation=cv2.INTER_AREA,
        )
    else:
        working = image.copy()

    # L from Lab is less sensitive to red/blue colour changes than a raw BGR
    # channel and preserves the dark-cavity boundary well.
    gray = cv2.cvtColor(working, cv2.COLOR_BGR2LAB)[:, :, 0]
    saturation = cv2.cvtColor(working, cv2.COLOR_BGR2HSV)[:, :, 1]
    gray = cv2.GaussianBlur(gray, (0, 0), 0.8)
    smooth = cv2.GaussianBlur(gray, (0, 0), 1.35)
    grad_x = cv2.Scharr(smooth, cv2.CV_32F, 1, 0)
    grad_y = cv2.Scharr(smooth, cv2.CV_32F, 0, 1)
    magnitude = cv2.magnitude(grad_x, grad_y)
    grad_reference = max(20.0, float(np.percentile(magnitude, 88.0)))
    dynamic_range = max(25.0, float(np.percentile(gray, 92) - np.percentile(gray, 8)))
    return _PreparedImage(
        color=working,
        gray=gray,
        saturation=saturation,
        smooth=smooth,
        grad_x=grad_x,
        grad_y=grad_y,
        grad_reference=grad_reference,
        dynamic_range=dynamic_range,
        scale=scale,
    )

def _find_hex_faces(prepared: _PreparedImage) -> list[_HexFace]:
    """Find compact 4--8 sided metal-face hypotheses without housing cues."""
    gray = prepared.gray
    short_side = min(gray.shape)
    kernel_size = max(3, int(round(0.014 * short_side)) | 1)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (kernel_size, kernel_size),
    )
    variants: list[np.ndarray] = []
    for low, high in ((25, 75), (38, 105), (55, 140)):
        variants.append(cv2.Canny(prepared.smooth, low, high))
    for threshold in (55, 75, 95, 120, 145, 170, 195):
        variants.append(np.uint8(gray > threshold) * 255)

    raw: list[_HexFace] = []
    min_dimension = 0.14 * short_side
    max_dimension = 0.49 * short_side
    min_area = 0.008 * short_side * short_side
    max_area = 0.22 * short_side * short_side
    for index, mask in enumerate(variants):
        closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2 if index < 3 else 1)
        if index < 3:
            closed = cv2.dilate(closed, np.ones((3, 3), np.uint8), iterations=1)
        contours, _ = cv2.findContours(
            closed,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not min_area <= area <= max_area:
                continue
            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            if hull_area <= 1.0:
                continue
            x, y, width, height = cv2.boundingRect(hull)
            if not (
                min_dimension <= width <= max_dimension
                and min_dimension <= height <= max_dimension
                and 0.55 <= width / max(1.0, height) <= 1.80
            ):
                continue
            perimeter = float(cv2.arcLength(hull, True))
            polygon = cv2.approxPolyDP(hull, 0.045 * perimeter, True)
            vertices = len(polygon)
            if not 4 <= vertices <= 8:
                continue
            moments = cv2.moments(hull)
            if moments["m00"] <= 1e-6:
                continue
            centre_x = float(moments["m10"] / moments["m00"])
            centre_y = float(moments["m01"] / moments["m00"])
            solidity = float(np.clip(area / hull_area, 0.0, 1.0))
            polygon_term = float(np.clip(1.0 - abs(vertices - 6) / 4.0, 0.0, 1.0))
            aspect_term = float(np.clip(1.0 - abs(math.log(width / max(1.0, height))), 0.0, 1.0))
            size = float(max(width, height))
            score = 1.25 * polygon_term + 0.90 * aspect_term + 0.65 * solidity
            raw.append(_HexFace(centre_x, centre_y, size, score))

    raw.sort(key=lambda item: item.score, reverse=True)
    faces: list[_HexFace] = []
    separation = 0.045 * short_side
    for face in raw:
        if all(math.hypot(face.x - other.x, face.y - other.y) > separation for other in faces):
            faces.append(face)
    return faces[:80]

def _hex_nesting_score(circle: Circle, faces: Sequence[_HexFace]) -> float:
    """Return support for a small circle located near a metal hex centre."""
    best = 0.0
    for face in faces:
        radius_ratio = circle.radius / max(1.0, face.size)
        distance_ratio = math.hypot(circle.x - face.x, circle.y - face.y) / max(1.0, face.size)
        if not (0.075 <= radius_ratio <= 0.285 and distance_ratio <= 0.34):
            continue
        centre_term = float(np.clip(1.0 - distance_ratio / 0.34, 0.0, 1.0))
        scale_term = float(np.clip(1.0 - abs(radius_ratio - 0.16) / 0.14, 0.0, 1.0))
        best = max(best, face.score + 4.00 * centre_term + 0.55 * scale_term)
    return best

def _add_hough_candidates(gray: np.ndarray, output: list[Circle]) -> None:
    height, width = gray.shape
    short_side = min(height, width)
    min_radius = max(5, round(short_side * 0.007))
    ranges = (
        (min_radius, max(min_radius + 2, round(short_side * 0.11)), 17),
        (max(min_radius, round(short_side * 0.07)), round(short_side * 0.27), 26),
        (max(min_radius, round(short_side * 0.20)), round(short_side * 0.49), 38),
    )
    min_distance = max(14, round(short_side * 0.025))
    for low, high, accumulator_threshold in ranges:
        if high <= low:
            continue
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.25,
            minDist=min_distance,
            param1=105,
            param2=accumulator_threshold,
            minRadius=low,
            maxRadius=high,
        )
        if circles is None:
            continue
        for x, y, radius in circles[0, :350]:
            output.append(Circle(float(x), float(y), float(radius)))

def _add_high_resolution_tip_candidates(
    image: np.ndarray,
    working_scale: float,
    output: list[Circle],
    max_side: int = 1200,
) -> None:
    """Add small-tip candidates before aggressive working-image downscaling.

    Radii are relative to the short side of a longest-side-capped preview so
    arbitrary input resolutions and aspect ratios stay valid. Coordinates are
    mapped back into the common working coordinate system.
    """
    height, width = image.shape[:2]
    high_scale = min(1.0, max_side / float(max(height, width)))
    if high_scale < 1.0:
        high_resolution = cv2.resize(
            image,
            (max(1, round(width * high_scale)), max(1, round(height * high_scale))),
            interpolation=cv2.INTER_AREA,
        )
    else:
        high_resolution = image
    gray = cv2.cvtColor(high_resolution, cv2.COLOR_BGR2LAB)[:, :, 0]
    gray = cv2.GaussianBlur(gray, (0, 0), 1.2)
    short_side = min(gray.shape)
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.20,
        minDist=max(18, round(0.025 * short_side)),
        param1=105,
        param2=20,
        minRadius=max(6, round(0.007 * short_side)),
        maxRadius=max(8, round(0.065 * short_side)),
    )
    if circles is None:
        return
    map_scale = working_scale / high_scale
    for x, y, radius in circles[0, :250]:
        output.append(
            Circle(
                float(x) * map_scale,
                float(y) * map_scale,
                float(radius) * map_scale,
            )
        )

def _add_threshold_candidates(gray: np.ndarray, output: list[Circle]) -> None:
    height, width = gray.shape
    short_side = min(height, width)
    minimum_radius = max(5.0, short_side * 0.006)
    maximum_radius = short_side * 0.50
    thresholds = sorted(
        {
            int(np.percentile(gray, percentile))
            for percentile in (10, 18, 27, 36, 46, 56)
        }
    )
    otsu_value, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thresholds.append(int(otsu_value))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    for threshold in sorted(set(thresholds)):
        mask = np.where(gray <= threshold, 255, 0).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if len(contour) < 5:
                continue
            area = float(abs(cv2.contourArea(contour)))
            perimeter = float(cv2.arcLength(contour, True))
            if area < math.pi * minimum_radius * minimum_radius * 0.16 or perimeter <= 0:
                continue

            (x, y), enclosing_radius = cv2.minEnclosingCircle(contour)
            if not (minimum_radius <= enclosing_radius <= maximum_radius):
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            fill_ratio = area / max(1.0, math.pi * enclosing_radius * enclosing_radius)
            if circularity < 0.10 or fill_ratio < 0.12:
                continue

            ellipse = cv2.fitEllipse(contour)
            (ellipse_x, ellipse_y), (axis_a, axis_b), _ = ellipse
            major = max(axis_a, axis_b)
            minor = min(axis_a, axis_b)
            if major <= 0 or minor / major < 0.42:
                continue
            ellipse_radius = 0.25 * (axis_a + axis_b)
            if minimum_radius <= ellipse_radius <= maximum_radius:
                output.append(Circle(float(ellipse_x), float(ellipse_y), float(ellipse_radius)))
            output.append(Circle(float(x), float(y), float(enclosing_radius)))

        # Distance-transform peaks provide reliable centres for filled dark
        # disks even when their outline is broken by glare.
        distance = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
        local_maximum = distance >= cv2.dilate(distance, np.ones((7, 7), np.uint8))
        ys, xs = np.nonzero(local_maximum & (distance >= minimum_radius))
        if len(xs):
            radii = distance[ys, xs]
            for index in np.argsort(radii)[-30:]:
                radius = float(radii[index])
                if radius <= maximum_radius:
                    output.append(Circle(float(xs[index]), float(ys[index]), radius))

def _inside_image(circle: Circle, shape: tuple[int, int], margin_ratio: float = 0.025) -> bool:
    height, width = shape
    margin = max(2.0, circle.radius * margin_ratio)
    return (
        circle.radius >= 4.0
        and circle.x - circle.radius >= margin
        and circle.y - circle.radius >= margin
        and circle.x + circle.radius < width - margin
        and circle.y + circle.radius < height - margin
    )

def _deduplicate(circles: Iterable[Circle], shape: tuple[int, int]) -> list[Circle]:
    valid = [circle for circle in circles if _inside_image(circle, shape)]
    valid.sort(key=lambda item: item.radius, reverse=True)
    kept: list[Circle] = []
    for circle in valid:
        duplicate = False
        for other in kept:
            radius_scale = max(8.0, min(circle.radius, other.radius))
            centre_distance = math.hypot(circle.x - other.x, circle.y - other.y)
            if centre_distance < 0.08 * radius_scale + 2.0 and abs(circle.radius - other.radius) < 0.08 * radius_scale + 2.0:
                duplicate = True
                break
        if not duplicate:
            kept.append(circle)
    return kept

def _sample(image: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    map_x = xs.astype(np.float32).reshape(1, -1)
    map_y = ys.astype(np.float32).reshape(1, -1)
    return cv2.remap(
        image,
        map_x,
        map_y,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE,
    ).reshape(-1)

def _local_hex_support(
    gray: np.ndarray,
    circle: Circle,
    min_vertices: int = 6,
    max_vertices: int = 6,
    fast_probe: bool = False,
) -> Optional[_LocalHexSupport]:
    """Return local 3--6 sided metal-face support around ``circle``.

    This is deliberately a *local* test.  Reflections may split the metal face
    into several global contours, while a small crop centred on a true nozzle
    still contains enough straight sides to reconstruct its convex polygonal
    hull.  The default remains exact-six for backwards compatibility; the
    occlusion-tolerant caller may request 3--6 visible vertices.  In both cases
    the complete circle must be well inside the hull and have the calibrated
    nozzle/face scale, so a circular lamp or a remote black/white junction
    cannot win on circle evidence alone.
    """
    if not 3 <= min_vertices <= max_vertices <= 6:
        raise ValueError("local face vertices must satisfy 3 <= min <= max <= 6")
    height, width = gray.shape[:2]
    radius = float(circle.radius)
    if radius < 4.0 or not _inside_image(circle, (height, width), margin_ratio=0.0):
        return None

    crop_radius = int(round(4.8 * radius))
    centre_x_px = int(round(circle.x))
    centre_y_px = int(round(circle.y))
    x0 = max(0, centre_x_px - crop_radius)
    x1 = min(width, centre_x_px + crop_radius + 1)
    y0 = max(0, centre_y_px - crop_radius)
    y1 = min(height, centre_y_px + crop_radius + 1)
    patch = gray[y0:y1, x0:x1]
    if min(patch.shape) < max(24, int(round(5.0 * radius))):
        return None

    patch = cv2.GaussianBlur(patch, (0, 0), 0.8)
    variants: list[np.ndarray] = []
    canny_levels = (
        ((35, 90), (55, 135))
        if fast_probe
        else ((20, 60), (35, 90), (55, 135), (80, 180))
    )
    for low, high in canny_levels:
        variants.append(cv2.Canny(patch, low, high))

    percentiles = (30, 46, 62) if fast_probe else (22, 30, 38, 46, 54, 62, 70, 78)
    for percentile in percentiles:
        threshold = float(np.percentile(patch, percentile))
        variants.append(np.where(patch >= threshold, 255, 0).astype(np.uint8))
        variants.append(np.where(patch <= threshold, 255, 0).astype(np.uint8))

    block_size = min(31, (min(patch.shape) - 1) | 1)
    if not fast_probe and block_size >= 3:
        variants.append(
            cv2.adaptiveThreshold(
                patch,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                block_size,
                3,
            )
        )
        variants.append(
            cv2.adaptiveThreshold(
                patch,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV,
                block_size,
                3,
            )
        )

    centre = (float(circle.x - x0), float(circle.y - y0))
    close_size = max(3, int(round(0.20 * radius)) | 1)
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (close_size, close_size),
    )
    best: Optional[_LocalHexSupport] = None
    for variant in variants:
        for close_iterations in ((1,) if fast_probe else (1, 2)):
            closed = cv2.morphologyEx(
                variant,
                cv2.MORPH_CLOSE,
                close_kernel,
                iterations=close_iterations,
            )
            contours, _ = cv2.findContours(
                closed,
                cv2.RETR_LIST,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            for contour in contours:
                if len(contour) < 5:
                    continue
                hull = cv2.convexHull(contour)
                area = float(cv2.contourArea(hull))
                if area <= 0.0:
                    continue
                box_x, box_y, box_w, box_h = cv2.boundingRect(hull)
                face_size = float(max(box_w, box_h))
                if not 3.0 * radius <= face_size <= 8.0 * radius:
                    continue
                aspect = min(box_w, box_h) / max(1.0, float(max(box_w, box_h)))
                if aspect < 0.55:
                    continue

                containment = float(cv2.pointPolygonTest(hull, centre, True))
                if containment < 1.10 * radius:
                    continue
                perimeter = float(cv2.arcLength(hull, True))
                if perimeter <= 0.0:
                    continue

                polygon: Optional[np.ndarray] = None
                for epsilon_ratio in (0.025, 0.035, 0.045, 0.055, 0.070):
                    approximation = cv2.approxPolyDP(
                        hull,
                        epsilon_ratio * perimeter,
                        True,
                    )
                    if min_vertices <= len(approximation) <= max_vertices:
                        polygon = approximation
                        if len(approximation) == max_vertices:
                            break
                if polygon is None:
                    continue

                moments = cv2.moments(hull)
                if abs(moments["m00"]) < 1e-6:
                    continue
                face_x = float(moments["m10"] / moments["m00"])
                face_y = float(moments["m01"] / moments["m00"])
                centre_ratio = math.hypot(
                    centre[0] - face_x,
                    centre[1] - face_y,
                ) / max(1.0, face_size)
                radius_ratio = radius / max(1.0, face_size)
                # Perspective/cropping may expose a slightly smaller subset of
                # the same hex hull at a frame edge.  Keep the upper scale at
                # 0.205 (also the physical refinement limit) so a true outer
                # rim at 1920x1080 is not rejected by a 1 px contour change.
                if centre_ratio > 0.18 or not 0.105 <= radius_ratio <= 0.205:
                    continue

                scale_term = max(0.0, 1.0 - abs(radius_ratio - 0.15) / 0.10)
                aspect_term = max(
                    0.0,
                    1.0 - abs(math.log(box_w / max(1.0, float(box_h)))),
                )
                score = (
                    1.50
                    + aspect_term
                    + scale_term
                    + 1.50 * max(0.0, 1.0 - centre_ratio / 0.22)
                    - 0.18 * (6 - len(polygon))
                )
                support = _LocalHexSupport(
                    x=face_x + x0,
                    y=face_y + y0,
                    size=face_size,
                    score=float(score),
                    vertices=len(polygon),
                    containment_margin=containment,
                )
                if best is None or support.score > best.score:
                    best = support
    return best

def _robust_local_hex_support(
    gray: np.ndarray,
    circle: Circle,
) -> Optional[_LocalHexSupport]:
    """Stabilise the exact-six test against a one-pixel crop/threshold change.

    The physical circle being validated never changes.  A second probe merely
    enlarges the analysis crop by asking the contour extractor to use a 15%
    larger nominal radius; any returned hull is rechecked against the original
    circle's centre, radius, and full-containment margin.
    """
    candidates: list[_LocalHexSupport] = []
    for probe_radius in (circle.radius, 1.15 * circle.radius):
        support = _local_hex_support(
            gray,
            Circle(circle.x, circle.y, probe_radius),
        )
        if support is None:
            continue
        centre_ratio = math.hypot(
            circle.x - support.x,
            circle.y - support.y,
        ) / max(1.0, support.size)
        radius_ratio = circle.radius / max(1.0, support.size)
        if (
            centre_ratio <= 0.18
            and 0.105 <= radius_ratio <= 0.205
            and support.containment_margin >= 1.02 * circle.radius
        ):
            candidates.append(support)
    return max(candidates, key=lambda item: item.score) if candidates else None

def _robust_local_partial_face_support(
    gray: np.ndarray,
    circle: Circle,
) -> Optional[_LocalHexSupport]:
    """Accept 3--6 visible metal-face sides when residue hides the rest.

    This is intentionally only a geometric support provider.  LED candidate
    selection separately requires a strong, opposed, near-complete circular
    edge; therefore relaxing the polygon side count does not relax the actual
    nozzle-circle evidence or admit square/background arcs.
    """
    candidates: list[_LocalHexSupport] = []
    for probe_radius in (circle.radius, 1.15 * circle.radius):
        support = _local_hex_support(
            gray,
            Circle(circle.x, circle.y, probe_radius),
            min_vertices=3,
            max_vertices=6,
        )
        if support is None:
            continue
        centre_ratio = math.hypot(
            circle.x - support.x,
            circle.y - support.y,
        ) / max(1.0, support.size)
        radius_ratio = circle.radius / max(1.0, support.size)
        if (
            centre_ratio <= 0.16
            and 0.105 <= radius_ratio <= 0.205
            and support.containment_margin >= 1.04 * circle.radius
        ):
            candidates.append(support)
    return max(candidates, key=lambda item: item.score) if candidates else None

def _fast_local_partial_face_support(
    gray: np.ndarray,
    circle: Circle,
) -> Optional[_LocalHexSupport]:
    """Cheap first proof of the calibrated 3--6-sided nozzle metal face.

    Clean fixed-camera frames do not need the full 22-mask robust sweep.  Try
    eight representative edge/threshold masks and one closing pass first.
    Returning ``None`` is not a rejection: the caller then executes the full
    R16 exact/partial proof, so glare, residue and weak-focus recall are kept.
    """
    for probe_radius in (circle.radius, 1.15 * circle.radius):
        support = _local_hex_support(
            gray,
            Circle(circle.x, circle.y, probe_radius),
            min_vertices=3,
            max_vertices=6,
            fast_probe=True,
        )
        if support is None:
            continue
        centre_ratio = math.hypot(
            circle.x - support.x,
            circle.y - support.y,
        ) / max(1.0, support.size)
        radius_ratio = circle.radius / max(1.0, support.size)
        if (
            centre_ratio <= 0.16
            and 0.105 <= radius_ratio <= 0.205
            and support.containment_margin >= 1.04 * circle.radius
            and support.score >= 2.70
        ):
            return support
    return None

def _has_led_fill_lighting(image: np.ndarray) -> bool:
    """Recognise the broad shadow lift caused by the new white LED fill light.

    The decision uses only global exposure percentiles on a small preview.  It
    does not assume a nozzle position and deliberately requires both lifted
    shadows and a normally exposed highlight range, so the old dark-camera
    branch remains selected for all frozen no-fill frames.
    """
    if image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    height, width = image.shape[:2]
    scale = min(1.0, 360.0 / float(max(height, width)))
    if scale < 1.0:
        preview = cv2.resize(
            image,
            (max(1, round(width * scale)), max(1, round(height * scale))),
            interpolation=cv2.INTER_AREA,
        )
    else:
        preview = image
    lightness = cv2.cvtColor(preview, cv2.COLOR_BGR2LAB)[:, :, 0]
    p08, median, p92 = np.percentile(lightness, (8, 50, 92))
    broad_shadow_lift = p08 >= 24.0 and median >= 100.0 and p92 >= 188.0
    # The blue auxiliary light can coexist with a large genuinely black region
    # (single-nozzle image9), which keeps p08 low even though the nozzle area is
    # strongly illuminated.  Its high median and near-saturated upper range are
    # separated from every frozen no-fill TEST/live frame.
    strong_coloured_fill = median >= 155.0 and p92 >= 240.0
    return bool(broad_shadow_lift or strong_coloured_fill)

def _bright_rim_levels(
    gray: np.ndarray,
    circle: Circle,
    samples: int = 240,
) -> tuple[float, float, float]:
    """Return dark-core, bright-rim, and outside-metal median levels."""
    angles = np.linspace(
        0.0,
        2.0 * math.pi,
        samples,
        endpoint=False,
        dtype=np.float32,
    )
    cosines = np.cos(angles)
    sines = np.sin(angles)

    def ring(factor: float) -> np.ndarray:
        radius = circle.radius * factor
        return _sample(
            gray,
            circle.x + radius * cosines,
            circle.y + radius * sines,
        )

    core = float(
        np.median(np.concatenate([ring(factor) for factor in (0.0, 0.20, 0.40)]))
    )
    rim = float(
        np.median(np.concatenate([ring(factor) for factor in (0.65, 0.80, 0.92)]))
    )
    outside = float(
        np.median(np.concatenate([ring(factor) for factor in (1.08, 1.18, 1.30)]))
    )
    return core, rim, outside

_RIM_TRIG_CACHE: dict[int, tuple[np.ndarray, np.ndarray]] = {}

def _rim_trig(samples: int) -> tuple[np.ndarray, np.ndarray]:
    """Return immutable reusable angular vectors for hot-path rim sampling."""
    cached = _RIM_TRIG_CACHE.get(samples)
    if cached is not None:
        return cached
    angles = np.linspace(
        0.0,
        2.0 * math.pi,
        samples,
        endpoint=False,
        dtype=np.float32,
    )
    cached = (np.cos(angles), np.sin(angles))
    cached[0].setflags(write=False)
    cached[1].setflags(write=False)
    _RIM_TRIG_CACHE[samples] = cached
    return cached

def _bright_rim_objective(
    gray: np.ndarray,
    circles: Sequence[Circle],
    *,
    samples: int = 240,
) -> np.ndarray:
    """Vectorised score for the *outer* edge of an LED-lit circular rim.

    An exact circle keeps the bright-to-dark outward transition aligned over
    the complete circumference.  Median and lower-quantile terms prevent one
    saturated glare arc from pulling the centre toward itself.
    """
    if not circles:
        return np.empty(0, dtype=np.float32)
    # Coarse search may use fewer angular samples; final sub-pixel selection
    # and every acceptance proof retain the original 240/360-sample density.
    # This removes work only from hypotheses which are never returned.
    samples = int(np.clip(samples, 120, 240))
    base_cosines, base_sines = _rim_trig(samples)
    cosines = base_cosines[None, :]
    sines = base_sines[None, :]
    centres_x = np.asarray([circle.x for circle in circles], np.float32)[:, None]
    centres_y = np.asarray([circle.y for circle in circles], np.float32)[:, None]
    radii = np.asarray([circle.radius for circle in circles], np.float32)[:, None]

    def sample_at(sample_radii: np.ndarray) -> np.ndarray:
        return cv2.remap(
            gray,
            (centres_x + sample_radii * cosines).astype(np.float32),
            (centres_y + sample_radii * sines).astype(np.float32),
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE,
        )

    just_inside = np.median(
        np.stack([sample_at(radii * 0.89), sample_at(radii * 0.945)]),
        axis=0,
    )
    just_outside = np.median(
        np.stack([sample_at(radii * 1.055), sample_at(radii * 1.11)]),
        axis=0,
    )
    core = np.median(
        np.stack([sample_at(radii * factor) for factor in (0.0, 0.25, 0.45)]),
        axis=0,
    )
    outward_drop = just_inside - just_outside
    core_to_rim = just_inside - core
    clipped_drop = np.clip(outward_drop, -20.0, 120.0)
    return (
        0.75 * np.median(outward_drop, axis=1)
        + 0.20 * np.percentile(outward_drop, 35, axis=1)
        + 0.15 * np.mean(clipped_drop, axis=1)
        + 28.0 * np.mean(outward_drop > 8.0, axis=1)
        + 0.08 * np.median(core_to_rim, axis=1)
        + 8.0 * np.mean(core_to_rim > 12.0, axis=1)
    )

def _bright_rim_closure(
    gray: np.ndarray,
    circle: Circle,
    samples: int = 360,
) -> _RimClosureSupport:
    """Measure full-circle and diametrically opposed outer-edge support.

    A glare blob can produce a very large gradient on one arc.  A physical
    nozzle edge instead produces matching outward drops across many angular
    sectors and, crucially, on opposite rays.  These illumination-normalised
    ratios are used for candidate ranking and as a final anti-drift gate.
    """
    samples = max(120, int(samples // 24) * 24)
    cosines, sines = _rim_trig(samples)

    def ring(factor: float) -> np.ndarray:
        radius = factor * circle.radius
        return _sample(
            gray,
            circle.x + radius * cosines,
            circle.y + radius * sines,
        )

    just_inside = np.median(
        np.stack([ring(0.88), ring(0.94)]),
        axis=0,
    )
    just_outside = np.median(
        np.stack([ring(1.06), ring(1.12)]),
        axis=0,
    )
    outward_drop = just_inside - just_outside
    supported = outward_drop > 8.0
    opposed = supported & np.roll(supported, samples // 2)
    sector_size = samples // 24
    sector_support = np.asarray(
        [
            np.mean(supported[index * sector_size : (index + 1) * sector_size])
            for index in range(24)
        ],
        dtype=np.float32,
    )
    return _RimClosureSupport(
        median_drop=float(np.median(outward_drop)),
        lower_drop=float(np.percentile(outward_drop, 25)),
        coverage=float(np.mean(supported)),
        opposed_coverage=float(np.mean(opposed)),
        sector_coverage=float(np.mean(sector_support >= 0.35)),
    )

def _refine_bright_rim(
    gray: np.ndarray,
    initial: Circle,
    face_size: float,
) -> tuple[Circle, float]:
    """Jointly refine x/y/r without letting a one-sided highlight cause drift."""
    current = initial
    stages = (
        (9.0, 1.0, 8.0, 1.0),
        (1.2, 0.3, 1.2, 0.3),
    )
    for centre_span, centre_step, radius_span, radius_step in stages:
        circles = [
            Circle(
                current.x + float(dx),
                current.y + float(dy),
                current.radius + float(dr),
            )
            for dx in np.arange(-centre_span, centre_span + 0.01, centre_step)
            for dy in np.arange(-centre_span, centre_span + 0.01, centre_step)
            for dr in np.arange(-radius_span, radius_span + 0.01, radius_step)
            if 0.075 * face_size <= current.radius + dr <= 0.205 * face_size
        ]
        scores = _bright_rim_objective(gray, circles)
        if scores.size == 0:
            break
        winner = int(np.argmax(scores))
        current = circles[winner]
    best_score = float(_bright_rim_objective(gray, [current])[0])
    return current, best_score

def _refine_fixed_640_rim(
    gray: np.ndarray,
    initial: Circle,
    min_radius_px: float,
    max_radius_px: float,
) -> tuple[Circle, float]:
    """Fast sub-pixel refinement for the calibrated small 640px rim."""
    current = initial
    stages = (
        (5.0, 1.0, 6.0, 1.0),
        (0.8, 0.2, 0.8, 0.2),
    )
    for stage_index, (
        centre_span,
        centre_step,
        radius_span,
        radius_step,
    ) in enumerate(stages):
        circles = [
            Circle(
                current.x + float(dx),
                current.y + float(dy),
                current.radius + float(dr),
            )
            for dx in np.arange(-centre_span, centre_span + 0.01, centre_step)
            for dy in np.arange(-centre_span, centre_span + 0.01, centre_step)
            for dr in np.arange(-radius_span, radius_span + 0.01, radius_step)
            if min_radius_px <= current.radius + dr <= max_radius_px
        ]
        scores = _bright_rim_objective(
            gray,
            circles,
            samples=160 if stage_index == 0 else 240,
        )
        if scores.size == 0:
            break
        current = circles[int(np.argmax(scores))]
    return current, float(_bright_rim_objective(gray, [current])[0])

def _circle_fits_inside_region(
    circle: Circle,
    region: Optional[tuple[float, float, float]],
    *,
    tolerance: float = 0.75,
) -> bool:
    """Return whether the complete target circle fits in a circular ROI."""
    if region is None:
        return True
    region_x, region_y, region_radius = region
    return (
        math.hypot(circle.x - region_x, circle.y - region_y) + circle.radius
        <= region_radius + tolerance
    )

def _detect_fixed_640_center_rim(
    image: np.ndarray,
    *,
    min_confidence: float,
    allowed_x: tuple[float, float],
    allowed_y: tuple[float, float],
    min_radius_px: float,
    max_radius_px: float,
    expected_center: Optional[tuple[float, float]] = None,
    allowed_region_circle: Optional[tuple[float, float, float]] = None,
    require_face_support: bool = True,
) -> Optional[Detection]:
    """Find the small central nozzle rim in the calibrated 640x480 stream.

    During carriage motion the surrounding metal face can show anywhere from
    three to six usable sides, or no stable polygon at all, while the actual
    circular nozzle rim remains sharply closed.  This profile therefore starts
    from the physically calibrated small radius and ranks complete opposed
    circular evidence directly.  It never searches the far-left/far-right
    second-nozzle bays and never enlarges to the surrounding cavity boundary.
    """
    if image.shape[:2] != (480, 640):
        return None

    # A133 hot path: prepare only the physical context needed by the 80x72
    # hard gate.  Older R16 revisions converted and blurred all 307,200 frame
    # pixels before discarding everything outside that gate.  The largest
    # 3--6-sided support probe reaches 4.8 * 1.15 radii from a candidate, so a
    # 6*r padding preserves exactly the pixels used by the R16 proof while
    # avoiding full-frame LAB conversion on every request.
    full_height, full_width = image.shape[:2]
    context_padding = max(64, int(math.ceil(6.0 * max_radius_px)))
    context_x0 = max(0, int(math.floor(allowed_x[0])) - context_padding)
    context_y0 = max(0, int(math.floor(allowed_y[0])) - context_padding)
    context_x1 = min(
        full_width,
        int(math.ceil(allowed_x[1])) + context_padding,
    )
    context_y1 = min(
        full_height,
        int(math.ceil(allowed_y[1])) + context_padding,
    )
    context = image[context_y0:context_y1, context_x0:context_x1]
    if min(context.shape[:2]) < 64:
        return None

    allowed_x = (
        float(allowed_x[0]) - context_x0,
        float(allowed_x[1]) - context_x0,
    )
    allowed_y = (
        float(allowed_y[0]) - context_y0,
        float(allowed_y[1]) - context_y0,
    )
    if expected_center is not None:
        expected_center = (
            float(expected_center[0]) - context_x0,
            float(expected_center[1]) - context_y0,
        )
    if allowed_region_circle is not None:
        allowed_region_circle = (
            float(allowed_region_circle[0]) - context_x0,
            float(allowed_region_circle[1]) - context_y0,
            float(allowed_region_circle[2]),
        )

    raw_gray = cv2.cvtColor(context, cv2.COLOR_BGR2LAB)[:, :, 0]
    gray = cv2.GaussianBlur(raw_gray, (0, 0), 0.8)
    height, width = gray.shape
    margin = int(math.ceil(max_radius_px)) + 2
    x0 = max(margin, int(math.floor(allowed_x[0])))
    x1 = min(width - margin, int(math.ceil(allowed_x[1])))
    y0 = max(margin, int(math.floor(allowed_y[0])))
    y1 = min(height - margin, int(math.ceil(allowed_y[1])))
    if x1 - x0 < 32 or y1 - y0 < 32:
        return None

    patch = gray[y0:y1, x0:x1]
    # Proposal generation deliberately keeps the proven R15 lower seed scale.
    # R15.1 tightens the *final output* to 13.5 px, but using that value to
    # raise Hough's seed floor changed candidate ordering on a blurred motion
    # frame and moved the fitted centre.  Broad proposals are safe because
    # refinement and the final physical gate below still enforce 13.5--17 px.
    proposal_min_radius_px = min(11.5, min_radius_px)
    seed_minimum = max(5, int(math.floor(0.52 * proposal_min_radius_px)))
    seed_maximum = max(seed_minimum + 2, int(math.ceil(1.30 * max_radius_px)))
    candidates: list[Circle] = []
    for blur_sigma in (0.75, 1.35, 2.05):
        smooth = cv2.GaussianBlur(patch, (0, 0), blur_sigma)
        for accumulator_threshold in (8, 11):
            found = cv2.HoughCircles(
                smooth,
                cv2.HOUGH_GRADIENT,
                dp=1.0,
                minDist=7,
                param1=90,
                param2=accumulator_threshold,
                minRadius=seed_minimum,
                maxRadius=seed_maximum,
            )
            if found is None:
                continue
            candidates.extend(
                Circle(float(x + x0), float(y + y0), float(radius))
                for x, y, radius in found[0, :120]
            )
    # A clean white annulus with a dark centre is occasionally represented as
    # opposing arcs instead of a Hough circle.  Bright-component seeds recover
    # its outer boundary; the opposed-circle proof below still rejects glare.
    for threshold_level in (180, 195, 210, 225, 235, 245):
        _, binary = cv2.threshold(
            patch,
            threshold_level,
            255,
            cv2.THRESH_BINARY,
        )
        contours, _ = cv2.findContours(
            binary,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        for contour in contours:
            area = cv2.contourArea(contour)
            if not 20.0 <= area <= 1600.0:
                continue
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if not 0.45 * proposal_min_radius_px <= radius <= 1.42 * max_radius_px:
                continue
            candidates.append(
                Circle(float(x + x0), float(y + y0), float(radius))
            )
    candidates = _deduplicate(candidates, gray.shape)
    if allowed_region_circle is not None:
        # Keep only seeds whose centre still leaves room for at least the
        # minimum physical nozzle radius.  The refined circle is checked again
        # below with its actual radius.
        region_x, region_y, region_radius = allowed_region_circle
        candidates = [
            circle
            for circle in candidates
            if math.hypot(circle.x - region_x, circle.y - region_y)
            <= region_radius - min_radius_px + 0.75
        ]
    if not candidates:
        return None

    seed_scores = _bright_rim_objective(gray, candidates)
    seed_order = np.argsort(seed_scores)[::-1]
    selected: list[Circle] = []
    for index in seed_order:
        circle = candidates[int(index)]
        if not (
            allowed_x[0] <= circle.x <= allowed_x[1]
            and allowed_y[0] <= circle.y <= allowed_y[1]
        ):
            continue
        if any(
            math.hypot(circle.x - other.x, circle.y - other.y) <= 5.0
            and abs(circle.radius - other.radius) <= 3.0
            for other in selected
        ):
            continue
        selected.append(circle)
        # With an authoritative machine expected point the fixed 80x72 gate
        # can contain only the intended nozzle; two starts retain the ambiguity
        # check while avoiding one full sub-pixel refinement on A133.
        selected_limit = 2 if expected_center is not None else 3
        if len(selected) >= selected_limit:
            break
    if not selected:
        return None

    proven: list[
        tuple[float, Circle, float, _RimClosureSupport, Optional[_LocalHexSupport]]
    ] = []
    for initial in selected:
        refined, rim_score = _refine_fixed_640_rim(
            gray,
            initial,
            min_radius_px,
            max_radius_px,
        )
        if not (
            min_radius_px <= refined.radius <= max_radius_px
            and allowed_x[0] <= refined.x <= allowed_x[1]
            and allowed_y[0] <= refined.y <= allowed_y[1]
            and refined.radius + 1.0 <= refined.x <= width - refined.radius - 1.0
            and refined.radius + 1.0 <= refined.y <= height - refined.radius - 1.0
            and _circle_fits_inside_region(refined, allowed_region_circle)
        ):
            continue
        closure = _bright_rim_closure(gray, refined)
        if (
            rim_score < 92.0
            or closure.median_drop < 48.0
            or closure.lower_drop < 26.0
            or closure.opposed_coverage < 0.76
            or closure.sector_coverage < 0.86
        ):
            continue
        support: Optional[_LocalHexSupport] = None
        if require_face_support:
            support = _fast_local_partial_face_support(gray, refined)
            if support is None:
                support = _robust_local_hex_support(gray, refined)
            if support is None:
                support = _robust_local_partial_face_support(gray, refined)
        support_ok = bool(
            support is not None
            and 3 <= support.vertices <= 6
            and support.score >= 2.70
        )
        if require_face_support and not support_ok:
            continue
        if expected_center is not None:
            expected_distance = math.hypot(
                refined.x - expected_center[0],
                refined.y - expected_center[1],
            )
            expected_tolerance = max(
                12.0,
                min(
                    48.0,
                    0.32
                    * min(
                        allowed_x[1] - allowed_x[0],
                        allowed_y[1] - allowed_y[0],
                    ),
                ),
            )
            if expected_distance > expected_tolerance:
                continue
        else:
            expected_distance = 0.0
        support_bonus = 0.0
        if support_ok:
            assert support is not None
            support_bonus = 3.5 * max(0.0, support.score - 2.4)
        rank = (
            rim_score
            + 42.0 * closure.opposed_coverage
            + 16.0 * closure.sector_coverage
            + 0.12 * closure.median_drop
            + support_bonus
            - 0.10 * expected_distance
        )
        proven.append((rank, refined, rim_score, closure, support))
        if (
            (expected_center is not None or support_ok)
            and rim_score >= 145.0
            and closure.lower_drop >= 34.0
            and closure.opposed_coverage >= 0.88
            and closure.sector_coverage >= 0.92
        ):
            # The authoritative 80x72 location, physical radius and strong
            # opposed full circle already form an unambiguous R16 proof.  Do
            # not refine a second Hough representation of the same rim on the
            # A133 hot path.  Difficult/glare/material frames miss this gate
            # and retain the second start automatically.
            break
    if not proven:
        return None

    proven.sort(key=lambda item: item[0], reverse=True)
    _, circle, rim_score, closure, support = proven[0]
    # A second complete small rim inside the selected bay is ambiguous unless
    # the winner has a clear circular-evidence margin.  This prevents a random
    # black/white printer detail from replacing the physical nozzle.
    if len(proven) > 1:
        second = proven[1]
        different_object = math.hypot(
            circle.x - second[1].x,
            circle.y - second[1].y,
        ) > 2.8 * max(circle.radius, second[1].radius)
        if different_object and second[0] >= 0.94 * proven[0][0]:
            return None

    support_floor = 0.0
    if support is not None and 3 <= support.vertices <= 6:
        support_floor = 0.03 * float(np.clip(support.score - 2.4, 0.0, 2.0))
    confidence = float(
        np.clip(
            0.50
            + 0.30 * (rim_score - 92.0) / 100.0
            + 0.12 * (closure.opposed_coverage - 0.76) / 0.24
            + support_floor,
            min_confidence,
            0.995,
        )
    )
    full_circle = Circle(
        circle.x + context_x0,
        circle.y + context_y0,
        circle.radius,
    )
    return Detection(circle=full_circle, confidence=confidence, score=rim_score)

def _fit_coloured_outer_ring(
    gray: np.ndarray,
    initial: Circle,
) -> Optional[tuple[Circle, float]]:
    """Fit the outer silver-ring contour under saturated blue illumination.

    The cyan single-nozzle frames expose a narrow annulus whose visible outer
    boundary is sometimes elliptical or one-sided in brightness.  Fitting that
    boundary over many lightness thresholds is more geometric than moving the
    centre toward the brightest arc.  A high consensus score is required; weak
    or distorted contours (image9's adjacent bloom) leave the Hough seed intact.
    """
    height, width = gray.shape[:2]
    half_size = max(24, int(round(3.2 * initial.radius)))
    x0 = max(0, int(round(initial.x)) - half_size)
    x1 = min(width, int(round(initial.x)) + half_size + 1)
    y0 = max(0, int(round(initial.y)) - half_size)
    y1 = min(height, int(round(initial.y)) + half_size + 1)
    patch = gray[y0:y1, x0:x1]
    if min(patch.shape) < 32:
        return None

    thresholds = {
        int(round(float(level)))
        for level in np.percentile(patch, np.arange(45.0, 97.0, 3.0))
    }
    thresholds.update(range(45, 181, 8))
    local_x = initial.x - x0
    local_y = initial.y - y0
    candidates: list[tuple[float, Circle]] = []
    for threshold in sorted(thresholds):
        mask = np.where(patch >= threshold, 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_NONE,
        )
        for contour in contours:
            if len(contour) < 12:
                continue
            area = float(cv2.contourArea(contour))
            perimeter = float(cv2.arcLength(contour, True))
            if area < 20.0 or perimeter <= 0.0:
                continue
            (centre_x, centre_y), (axis_a, axis_b), _ = cv2.fitEllipse(contour)
            major_axis = max(axis_a, axis_b)
            minor_axis = min(axis_a, axis_b)
            if major_axis <= 0.0:
                continue
            axis_ratio = minor_axis / major_axis
            radius = 0.25 * (axis_a + axis_b)
            centre_shift = math.hypot(centre_x - local_x, centre_y - local_y)
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if (
                not 0.65 * initial.radius <= radius <= 1.45 * initial.radius
                or centre_shift > 0.65 * initial.radius
                or axis_ratio < 0.72
                or circularity < 0.45
            ):
                continue
            circle = Circle(
                float(centre_x + x0),
                float(centre_y + y0),
                float(radius),
            )
            core, ring, outside = _bright_rim_levels(gray, circle)
            dual_contrast = min(ring - core, ring - outside)
            score = (
                2.0 * axis_ratio
                + circularity
                - 1.5 * centre_shift / max(1.0, initial.radius)
                + 0.015 * dual_contrast
                - 0.020 * abs(radius - initial.radius)
            )
            candidates.append((float(score), circle))

    if not candidates:
        return None
    score, circle = max(candidates, key=lambda item: item[0])
    if score < 3.25:
        return None
    return circle, score

def _coloured_aperture_topology(
    gray: np.ndarray,
    circle: Circle,
    samples: int = 240,
) -> tuple[float, float, float, float]:
    """Measure the dark-core to bright-metal transition around a circle."""
    angles = np.linspace(
        0.0,
        2.0 * math.pi,
        samples,
        endpoint=False,
        dtype=np.float32,
    )
    cosines = np.cos(angles)
    sines = np.sin(angles)

    def ring(factor: float) -> np.ndarray:
        radius = factor * circle.radius
        return _sample(
            gray,
            circle.x + radius * cosines,
            circle.y + radius * sines,
        )

    inside = np.median(
        np.stack([ring(factor) for factor in (0.0, 0.35, 0.65, 0.82)]),
        axis=0,
    )
    outside = np.median(
        np.stack([ring(factor) for factor in (1.16, 1.32, 1.48)]),
        axis=0,
    )
    rise = outside - inside
    return (
        float(np.median(rise)),
        float(np.percentile(rise, 30)),
        float(np.mean(rise > 10.0)),
        float(np.median(inside)),
    )

def _snap_coloured_aperture_edge(
    gray: np.ndarray,
    component_circle: Circle,
    outer_ring: Circle,
) -> Circle:
    """Move a dark-component isophote out to the opposed physical edge."""
    angles = np.linspace(
        0.0,
        2.0 * math.pi,
        360,
        endpoint=False,
        dtype=np.float32,
    )
    cosines = np.cos(angles)
    sines = np.sin(angles)
    radial = np.linspace(
        0.92 * component_circle.radius,
        1.45 * component_circle.radius,
        121,
        dtype=np.float32,
    )
    profiles = cv2.remap(
        gray,
        (
            component_circle.x
            + cosines[:, None] * radial[None, :]
        ).astype(np.float32),
        (
            component_circle.y
            + sines[:, None] * radial[None, :]
        ).astype(np.float32),
        interpolation=cv2.INTER_CUBIC,
        borderMode=cv2.BORDER_REPLICATE,
    )
    profiles = cv2.GaussianBlur(profiles, (7, 1), 0)
    span = 3
    rises = profiles[:, 2 * span :] - profiles[:, :-2 * span]
    edge_radii = radial[span:-span]
    winner_indices = np.argmax(rises, axis=1)
    columns = np.arange(len(angles))
    distances = edge_radii[winner_indices]
    strengths = rises[columns, winner_indices]
    half = len(angles) // 2
    pair_strength = np.minimum(strengths[:half], strengths[half:])
    pair_radius = 0.5 * (distances[:half] + distances[half:])
    valid = pair_strength >= 7.0
    snapped_radius = 1.15 * component_circle.radius
    if float(np.mean(valid)) >= 0.30:
        median_radius = float(np.median(pair_radius[valid]))
        radius_mad = 1.4826 * float(
            np.median(np.abs(pair_radius[valid] - median_radius))
        )
        valid &= np.abs(pair_radius - median_radius) <= max(1.2, 2.8 * radius_mad)
        if float(np.mean(valid)) >= 0.25:
            weights = np.clip(pair_strength[valid], 1.0, 100.0)
            values = pair_radius[valid]
            order = np.argsort(values)
            ordered_values = values[order]
            ordered_weights = weights[order]
            cutoff = 0.5 * float(np.sum(ordered_weights))
            index = int(np.searchsorted(np.cumsum(ordered_weights), cutoff))
            snapped_radius = float(
                ordered_values[min(index, len(ordered_values) - 1)]
            )
    if snapped_radius < 1.10 * component_circle.radius:
        snapped_radius = 1.15 * component_circle.radius
    snapped_radius = float(
        np.clip(
            snapped_radius,
            1.05 * component_circle.radius,
            1.30 * component_circle.radius,
        )
    )
    snapped_radius = float(
        np.clip(
            snapped_radius,
            0.30 * outer_ring.radius,
            0.76 * outer_ring.radius,
        )
    )
    return Circle(component_circle.x, component_circle.y, snapped_radius)

def _fit_coloured_dark_aperture(
    gray: np.ndarray,
    outer_ring: Circle,
) -> Optional[tuple[Circle, float]]:
    """Fit the nozzle opening inside a saturated-blue silver annulus.

    The silver annulus is used only as a localisation anchor. Its outer contour
    blooms and smears with illumination, whereas the requested nozzle edge is
    the enclosed dark-aperture/metal boundary. Multi-threshold component
    consensus rejects masks that leak through the surrounding bright metal.
    """
    height, width = gray.shape
    half_size = max(20, int(round(1.65 * outer_ring.radius)))
    x0 = max(0, int(round(outer_ring.x)) - half_size)
    x1 = min(width, int(round(outer_ring.x)) + half_size + 1)
    y0 = max(0, int(round(outer_ring.y)) - half_size)
    y1 = min(height, int(round(outer_ring.y)) + half_size + 1)
    patch = gray[y0:y1, x0:x1]
    if min(patch.shape) < 24:
        return None

    local_x = outer_ring.x - x0
    local_y = outer_ring.y - y0
    seed_span = max(2, int(round(0.25 * outer_ring.radius)))
    seed_x0 = max(0, int(round(local_x)) - seed_span)
    seed_x1 = min(patch.shape[1], int(round(local_x)) + seed_span + 1)
    seed_y0 = max(0, int(round(local_y)) - seed_span)
    seed_y1 = min(patch.shape[0], int(round(local_y)) + seed_span + 1)
    seed_patch = patch[seed_y0:seed_y1, seed_x0:seed_x1]
    if seed_patch.size == 0:
        return None
    seed_index = int(np.argmin(seed_patch))
    seed_row, seed_column = np.unravel_index(seed_index, seed_patch.shape)
    seed_x = seed_x0 + int(seed_column)
    seed_y = seed_y0 + int(seed_row)

    thresholds = {
        int(round(float(level)))
        for level in np.percentile(patch, np.arange(8.0, 61.0, 3.0))
    }
    thresholds.update(range(25, 161, 8))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    candidates: list[tuple[float, Circle]] = []
    outer_area = math.pi * outer_ring.radius * outer_ring.radius
    for threshold in sorted(thresholds):
        mask = np.where(patch <= threshold, 255, 0).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        component_count, labels, statistics, _ = cv2.connectedComponentsWithStats(mask)
        if component_count <= 1:
            continue
        component_id = int(labels[seed_y, seed_x])
        if component_id <= 0:
            continue
        x, y, component_width, component_height, area = statistics[component_id]
        if (
            x <= 0
            or y <= 0
            or x + component_width >= patch.shape[1] - 1
            or y + component_height >= patch.shape[0] - 1
            or not 0.08 <= float(area) / max(1.0, outer_area) <= 0.72
        ):
            continue
        component = np.where(labels == component_id, 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(
            component,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE,
        )
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        if len(contour) < 8:
            continue
        (centre_x, centre_y), (axis_a, axis_b), _ = cv2.fitEllipse(contour)
        major_axis = max(axis_a, axis_b)
        minor_axis = min(axis_a, axis_b)
        if major_axis <= 0.0:
            continue
        axis_ratio = minor_axis / major_axis
        radius = 0.25 * (axis_a + axis_b)
        circle = Circle(
            float(centre_x + x0),
            float(centre_y + y0),
            float(radius),
        )
        centre_shift = math.hypot(
            circle.x - outer_ring.x,
            circle.y - outer_ring.y,
        )
        radius_ratio = circle.radius / max(1.0, outer_ring.radius)
        if (
            axis_ratio < 0.48
            or centre_shift > 0.35 * outer_ring.radius
            or not 0.28 <= radius_ratio <= 0.72
        ):
            continue
        median_rise, lower_rise, coverage, core_level = _coloured_aperture_topology(
            gray,
            circle,
        )
        if median_rise < 14.0 or lower_rise < -2.0 or coverage < 0.62:
            continue
        score = (
            1.8 * axis_ratio
            + 0.018 * median_rise
            + 0.010 * max(0.0, lower_rise)
            + 1.2 * coverage
            - 0.9 * centre_shift / max(1.0, outer_ring.radius)
            - 0.20 * abs(radius_ratio - 0.48)
            - 0.001 * core_level
        )
        candidates.append((float(score), circle))

    if len(candidates) < 4:
        return None
    values = np.asarray(
        [(circle.x, circle.y, circle.radius) for _, circle in candidates],
        dtype=np.float64,
    )
    median = np.median(values, axis=0)
    distances = np.sqrt(
        (values[:, 0] - median[0]) ** 2
        + (values[:, 1] - median[1]) ** 2
        + (values[:, 2] - median[2]) ** 2
    )
    distance_median = float(np.median(distances))
    keep = distances <= max(1.2, 2.8 * distance_median)
    if int(np.sum(keep)) < 4:
        return None
    fitted_values = np.median(values[keep], axis=0)
    fitted = Circle(*map(float, fitted_values))

    # Inner and outer boundaries are physically concentric. Average their two
    # independently measured centres, then limit any residual one-sided leak.
    fitted = Circle(
        0.5 * (fitted.x + outer_ring.x),
        0.5 * (fitted.y + outer_ring.y),
        fitted.radius,
    )
    dx = fitted.x - outer_ring.x
    dy = fitted.y - outer_ring.y
    shift = math.hypot(dx, dy)
    maximum_shift = 0.11 * outer_ring.radius
    if shift > maximum_shift:
        scale = maximum_shift / shift
        fitted = Circle(
            outer_ring.x + scale * dx,
            outer_ring.y + scale * dy,
            fitted.radius,
        )
    fitted = _snap_coloured_aperture_edge(gray, fitted, outer_ring)
    median_rise, lower_rise, coverage, _ = _coloured_aperture_topology(gray, fitted)
    radius_ratio = fitted.radius / max(1.0, outer_ring.radius)
    if (
        not 0.28 <= radius_ratio <= 0.72
        or median_rise < 14.0
        or lower_rise < -2.0
        or coverage < 0.62
    ):
        return None
    consensus = float(np.mean(keep))
    final_score = (
        0.020 * median_rise
        + 0.012 * max(0.0, lower_rise)
        + 1.3 * coverage
        + 0.5 * consensus
    )
    return fitted, final_score

def _detect_led_bright_rim(
    image: np.ndarray,
    min_confidence: float,
    allowed_x: tuple[float, float],
    allowed_y: tuple[float, float],
    min_radius_px: Optional[float],
    max_radius_px: Optional[float],
    lighting_reference: Optional[np.ndarray] = None,
) -> Optional[Detection]:
    """Detect the LED-lit silver nozzle rim through hex-constrained search."""
    if image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    prepared = _prepare(image)
    scale = prepared.scale
    faces = [
        face
        for face in _find_hex_faces(prepared)
        if face.score >= 1.70
        and allowed_x[0] <= face.x / scale <= allowed_x[1]
        and allowed_y[0] <= face.y / scale <= allowed_y[1]
    ][:16]
    if not faces:
        return None

    raw_gray = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)[:, :, 0]
    gray = cv2.GaussianBlur(raw_gray, (0, 0), 0.8)
    # A bounded A133 context must retain the original frame's lighting class;
    # otherwise a dark housing crop could make a cyan/LED frame look unlit and
    # silently select different R16 thresholds.  Use a small full-frame
    # photometric preview only for classification, never for circle search.
    if lighting_reference is None:
        lighting_gray = gray
        lighting_bgr = image
    else:
        ref_h, ref_w = lighting_reference.shape[:2]
        ref_scale = min(1.0, 360.0 / float(max(ref_h, ref_w)))
        if ref_scale < 1.0:
            lighting_bgr = cv2.resize(
                lighting_reference,
                (max(1, round(ref_w * ref_scale)), max(1, round(ref_h * ref_scale))),
                interpolation=cv2.INTER_AREA,
            )
        else:
            lighting_bgr = lighting_reference
        if lighting_bgr.shape[2] == 4:
            lighting_bgr = cv2.cvtColor(lighting_bgr, cv2.COLOR_BGRA2BGR)
        lighting_gray = cv2.cvtColor(lighting_bgr, cv2.COLOR_BGR2LAB)[:, :, 0]
    median_level, highlight_level = np.percentile(lighting_gray, (50, 92))
    strong_coloured_fill = bool(median_level >= 145.0 and highlight_level >= 240.0)
    median_saturation = float(
        np.median(cv2.cvtColor(lighting_bgr, cv2.COLOR_BGR2HSV)[:, :, 1])
    )
    blue_nozzle_fill = bool(strong_coloured_fill and median_saturation >= 75.0)
    fixed_640_rim_profile = bool(
        image.shape[1] == 640
        and image.shape[0] == 480
        and min_radius_px is not None
        and max_radius_px is not None
        and 11.0 <= min_radius_px <= 12.0
        and 16.5 <= max_radius_px <= 17.5
    )
    minimum_global_radius_ratio = 0.045 if strong_coloured_fill else 0.060
    minimum_seed_score = 42.0 if strong_coloured_fill else 55.0
    minimum_hex_score = 3.35 if strong_coloured_fill else 3.40
    hypotheses: list[tuple[float, Circle, float, float, float]] = []
    image_shape = gray.shape
    for face in faces:
        face_x = face.x / scale
        face_y = face.y / scale
        face_size = face.size / scale
        margin = int(round(0.30 * face_size))
        x0 = max(0, int(face_x) - margin)
        x1 = min(image_shape[1], int(face_x) + margin + 1)
        y0 = max(0, int(face_y) - margin)
        y1 = min(image_shape[0], int(face_y) + margin + 1)
        patch = gray[y0:y1, x0:x1]
        if min(patch.shape) < 16:
            continue

        candidates: list[Circle] = []
        automatic_low = max(3, round(0.045 * face_size))
        automatic_high = max(6, round(0.20 * face_size))
        if min_radius_px is not None:
            # A white-light Hough seed may start substantially inside the true
            # outer rim and grow during radial refinement.  Keep that seed
            # search permissive; the final refined circle is still checked
            # against the strict physical pixel range below.  Blue/cyan uses a
            # different outer-annulus topology and retains its original rule.
            seed_minimum = (
                1.25 * min_radius_px
                if blue_nozzle_fill
                else 0.60 * min_radius_px
            )
            automatic_low = max(automatic_low, int(math.floor(seed_minimum)))
        if max_radius_px is not None:
            seed_maximum = (
                2.80 * max_radius_px
                if blue_nozzle_fill
                else 1.20 * max_radius_px
            )
            automatic_high = min(automatic_high, int(math.ceil(seed_maximum)))
        if automatic_high <= automatic_low:
            continue
        for blur_sigma in (0.9, 1.8):
            smooth = cv2.GaussianBlur(patch, (0, 0), blur_sigma)
            for accumulator_threshold in (8, 12):
                found = cv2.HoughCircles(
                    smooth,
                    cv2.HOUGH_GRADIENT,
                    dp=1.0,
                    minDist=max(4, round(0.025 * face_size)),
                    param1=80,
                    param2=accumulator_threshold,
                    minRadius=automatic_low,
                    maxRadius=automatic_high,
                )
                if found is None:
                    continue
                candidates.extend(
                    Circle(float(x + x0), float(y + y0), float(radius))
                    for x, y, radius in found[0, :50]
                )
        candidates = _deduplicate(candidates, image_shape)
        for circle in candidates:
            distance_ratio = math.hypot(
                circle.x - face_x,
                circle.y - face_y,
            ) / max(1.0, face_size)
            radius_ratio = circle.radius / max(1.0, face_size)
            if (
                distance_ratio > 0.24
                or not minimum_global_radius_ratio <= radius_ratio <= 0.19
            ):
                continue
            core, rim, outside = _bright_rim_levels(gray, circle)
            coarse_score = (
                0.55 * (rim - core)
                + 0.45 * (rim - outside)
                - 40.0 * distance_ratio
                - 80.0 * abs(radius_ratio - 0.135)
                + 4.0 * face.score
            )
            hypotheses.append((coarse_score, circle, face_size, face_x, face_y))

    if not hypotheses:
        return None

    # R11 lock: rank seeds by opposed full-circle evidence before refinement.
    # Coarse brightness alone is unreliable because a one-sided glare patch can
    # outscore the true nozzle.  Exact-six remains preferred, while a seed with
    # very strong circular closure may enter refinement with only a partially
    # visible 3--6 sided face; the final result must pass both tests again.
    distinct_seeds: list[tuple[float, Circle, float, float, float]] = []
    for hypothesis in sorted(hypotheses, key=lambda item: item[0], reverse=True):
        _, circle, _, _, _ = hypothesis
        if any(
            math.hypot(circle.x - other.x, circle.y - other.y)
            <= 0.10 * min(circle.radius, other.radius) + 2.0
            and abs(circle.radius - other.radius)
            <= 0.10 * min(circle.radius, other.radius) + 2.0
            for _, other, _, _, _ in distinct_seeds
        ):
            continue
        distinct_seeds.append(hypothesis)
        if len(distinct_seeds) >= 24:
            break

    # R12 proof-gated fast path.  The full R11 seed/support/refinement pipeline
    # below is deliberately left intact as the fallback.  This path is only for
    # a visually unambiguous, exact-six, high-contrast rim.  In particular, the
    # lower-quartile and opposed gates exclude a one-sided glare arc, while the
    # different-face check prevents a fast jump to printer scenery.
    if distinct_seeds and not blue_nozzle_fill:
        if fixed_640_rim_profile:
            fixed_seed_scores = _bright_rim_objective(
                gray,
                [item[1] for item in distinct_seeds],
            )
            fast_seed_index = int(np.argmax(fixed_seed_scores))
            fast_seed_score = float(fixed_seed_scores[fast_seed_index])
            fast_seed_tuple = distinct_seeds[fast_seed_index]
        else:
            fast_seed_tuple = distinct_seeds[0]
        (
            fast_coarse_score,
            fast_initial,
            fast_face_size,
            fast_face_x,
            fast_face_y,
        ) = fast_seed_tuple
        if not fixed_640_rim_profile:
            fast_seed_score = float(_bright_rim_objective(gray, [fast_initial])[0])
        fast_seed_closure = _bright_rim_closure(gray, fast_initial)
        fast_seed_support = _robust_local_hex_support(gray, fast_initial)
        # Some live MJPEG frames preserve the complete physical rim but encode
        # its Hough seed a few pixels too small.  The unrefined radial closure
        # is then weak even though an exact, well-centred six-sided metal face
        # already proves that the seed belongs to the nozzle.  Permit one such
        # seed to enter refinement; the returned circle still has to pass the
        # unchanged, very strict full-rim + exact-six output proof below.
        fast_geometric_refinement_seed = bool(
            fast_coarse_score >= 0.85 * minimum_seed_score
            and fast_seed_support is not None
            and fast_seed_support.vertices == 6
            and fast_seed_support.score >= max(4.20, minimum_hex_score)
        )
        fixed_640_refinement_seed = bool(
            fixed_640_rim_profile
            and fast_seed_score >= 65.0
            and fast_seed_support is not None
            and fast_seed_support.vertices == 6
            and fast_seed_support.score >= 4.55
            and min_radius_px <= fast_initial.radius <= max_radius_px
        )
        fast_seed_proven = bool(
            fast_seed_support is not None
            and fast_seed_support.vertices == 6
            and fast_seed_support.score >= max(4.0, minimum_hex_score)
            and (
                (
                    fast_coarse_score >= minimum_seed_score
                    and fast_seed_score >= 175.0
                    and fast_seed_closure.lower_drop >= 65.0
                    and fast_seed_closure.opposed_coverage >= 0.70
                    and fast_seed_closure.sector_coverage >= 0.83
                )
                or fast_geometric_refinement_seed
                or fixed_640_refinement_seed
            )
        )
        if fast_seed_proven:
            fast_refined, fast_rim_score = _refine_bright_rim(
                gray,
                fast_initial,
                fast_face_size,
            )
            fast_closure = _bright_rim_closure(gray, fast_refined)
            fast_support = _robust_local_hex_support(gray, fast_refined)
            fast_radius_ok = bool(
                (min_radius_px is None or fast_refined.radius >= min_radius_px)
                and (max_radius_px is None or fast_refined.radius <= max_radius_px)
            )
            fast_refined_proven = bool(
                fast_rim_score >= (150.0 if fixed_640_rim_profile else 185.0)
                and fast_closure.median_drop
                >= (125.0 if fixed_640_rim_profile else 100.0)
                and fast_closure.lower_drop
                >= (70.0 if fixed_640_rim_profile else 70.0)
                and fast_closure.opposed_coverage
                >= (0.78 if fixed_640_rim_profile else 0.78)
                and fast_closure.sector_coverage
                >= (0.91 if fixed_640_rim_profile else 0.90)
                and fast_support is not None
                and fast_support.vertices == 6
                and fast_support.score >= max(4.0, minimum_hex_score)
                and fast_radius_ok
            )

            different_face_competitor = False
            if fast_refined_proven:
                alternate_items = []
                for alternate in distinct_seeds[1:]:
                    _, circle, face_size, face_x, face_y = alternate
                    same_face = (
                        math.hypot(face_x - fast_face_x, face_y - fast_face_y)
                        <= 0.18 * max(face_size, fast_face_size)
                        and max(face_size, fast_face_size)
                        / max(1.0, min(face_size, fast_face_size))
                        <= 1.35
                    )
                    if not same_face:
                        alternate_items.append(alternate)
                if alternate_items:
                    alternate_scores = _bright_rim_objective(
                        gray,
                        [item[1] for item in alternate_items],
                    )
                    for alternate, alternate_score in zip(
                        alternate_items,
                        alternate_scores,
                    ):
                        if alternate_score < 0.82 * fast_rim_score:
                            continue
                        alternate_closure = _bright_rim_closure(gray, alternate[1])
                        if (
                            alternate_closure.opposed_coverage >= 0.55
                            and alternate_closure.sector_coverage >= 0.75
                        ):
                            different_face_competitor = True
                            break

            if fast_refined_proven and not different_face_competitor:
                assert fast_support is not None
                confidence = float(
                    np.clip(
                        0.88 * (fast_rim_score - 70.0) / 125.0
                        + 0.12 * (fast_support.score - 3.40) / 1.40,
                        0.0,
                        0.995,
                    )
                )
                closure_floor = 0.36 + 0.42 * float(
                    np.clip(
                        (fast_closure.opposed_coverage - 0.42) / 0.58,
                        0.0,
                        1.0,
                    )
                )
                confidence = max(confidence, closure_floor)
                if strong_coloured_fill:
                    core_level, ring_level, outside_level = _bright_rim_levels(
                        gray,
                        fast_refined,
                    )
                    dual_contrast = min(
                        ring_level - core_level,
                        ring_level - outside_level,
                    )
                    coloured_floor = 0.38 + 0.24 * float(
                        np.clip((dual_contrast - 45.0) / 105.0, 0.0, 1.0)
                    )
                    confidence = max(confidence, coloured_floor)
                if confidence >= min_confidence:
                    return Detection(
                        circle=fast_refined,
                        confidence=confidence,
                        score=fast_rim_score,
                    )

    strict_seeds: list[
        tuple[
            float,
            Circle,
            float,
            Optional[_LocalHexSupport],
            float,
            float,
            float,
        ]
    ] = []
    for coarse_score, circle, face_size, face_x, face_y in distinct_seeds:
        seed_rim_score = float(_bright_rim_objective(gray, [circle])[0])
        seed_closure = _bright_rim_closure(gray, circle)
        circular_rescue = bool(
            seed_rim_score >= 32.0
            and seed_closure.opposed_coverage >= 0.28
            and seed_closure.sector_coverage >= 0.58
        )
        support = _robust_local_hex_support(gray, circle)
        if support is None:
            support = _robust_local_partial_face_support(gray, circle)
        support_ok = bool(
            support is not None
            and (
                (support.vertices == 6 and support.score >= minimum_hex_score)
                or (3 <= support.vertices <= 5 and support.score >= 2.75)
            )
        )
        # A displaced Hough circle can have weak unrefined contrast but still
        # sit securely inside the correct local metal face.  Let that geometric
        # neighbour enter refinement; it must still pass the strict refined
        # opposed-edge and 3--6-side gates below.  This supplies an independent
        # start for material/glare cases without lowering the output gate.
        geometric_neighbour_rescue = bool(
            support_ok
            and (
                (
                    seed_rim_score >= 22.0
                    and seed_closure.opposed_coverage >= 0.18
                    and seed_closure.sector_coverage >= 0.50
                )
                or (
                    coarse_score >= 0.55 * minimum_seed_score
                    and seed_closure.sector_coverage >= 0.40
                )
            )
        )
        if (
            coarse_score < minimum_seed_score
            and not circular_rescue
            and not geometric_neighbour_rescue
        ):
            continue
        # Residue may hide even the third local side in the unrefined Hough
        # crop.  A strongly closed seed may still be refined, but no output is
        # allowed unless the refined circle recovers 3--6 local sides.
        if not support_ok and not circular_rescue:
            continue
        support_bonus = 0.0 if support is None else 5.0 * (support.score - 2.75)
        strict_rank = (
            seed_rim_score
            + 48.0 * seed_closure.opposed_coverage
            + 12.0 * seed_closure.sector_coverage
            + 0.12 * coarse_score
            + support_bonus
        )
        strict_seeds.append(
            (
                strict_rank,
                circle,
                face_size,
                support,
                face_x,
                face_y,
                coarse_score,
            )
        )
    if not strict_seeds:
        return None

    # A single Hough seed can start on the glare side of the true rim.  Keep up
    # to four geometrically distinct starts on the same 3--6 sided metal face;
    # after refinement, opposed full-circle evidence chooses among them.  This
    # avoids both local-peak drift and the unsafe alternative of widening one
    # optimiser until it reaches a surrounding hex/housing arc.
    ranked_seeds = sorted(strict_seeds, key=lambda item: item[0], reverse=True)
    selected_seeds = []
    for seed in ranked_seeds:
        face_size = seed[2]
        face_x = seed[4]
        face_y = seed[5]
        same_face = [
            other
            for other in selected_seeds
            if math.hypot(face_x - other[4], face_y - other[5])
            <= 0.18 * max(face_size, other[2])
            and max(face_size, other[2]) / max(1.0, min(face_size, other[2]))
            <= 1.35
        ]
        if len(same_face) >= 4:
            continue
        if any(
            math.hypot(seed[1].x - other[1].x, seed[1].y - other[1].y)
            <= 0.18 * min(seed[1].radius, other[1].radius) + 2.0
            and abs(seed[1].radius - other[1].radius)
            <= 0.15 * min(seed[1].radius, other[1].radius) + 2.0
            for other in same_face
        ):
            continue
        selected_seeds.append(seed)
        if len(selected_seeds) >= 14:
            break

    # Re-run the polygon and opposed-edge gates afterwards so neither glare nor
    # material can pull the answer off-centre.
    refined_candidates: list[
        tuple[float, Circle, float, _LocalHexSupport]
    ] = []
    for (
        _,
        initial,
        face_size,
        initial_support,
        _,
        _,
        _,
    ) in selected_seeds:
        refined, rim_score = _refine_bright_rim(gray, initial, face_size)
        closure = _bright_rim_closure(gray, refined)
        if rim_score < 105.0:
            seed_rim_score = float(_bright_rim_objective(gray, [initial])[0])
            if strong_coloured_fill and seed_rim_score >= 80.0:
                # On a narrow cyan-lit annulus, the broad optimiser can be
                # attracted to a one-sided bloom.  The Hough seed itself is the
                # complete circular outer edge, so retain it when its strict
                # hex and dark/bright/dark topology are already sufficient.
                refined = initial
                rim_score = seed_rim_score
                closure = _bright_rim_closure(gray, refined)
            elif (
                rim_score >= 65.0
                and closure.median_drop >= 28.0
                and closure.opposed_coverage >= 0.45
                and closure.sector_coverage >= 0.70
            ):
                # A material-filled centre can lower the old dark-core/rim
                # score even when the physical outer circle is complete.
                pass
            else:
                continue
        if (
            closure.median_drop < 20.0
            or closure.opposed_coverage < 0.42
            or closure.sector_coverage < 0.67
        ):
            continue
        if not blue_nozzle_fill and min_radius_px is not None and refined.radius < min_radius_px:
            continue
        if not blue_nozzle_fill and max_radius_px is not None and refined.radius > max_radius_px:
            continue
        # The threshold-contour fitter is only for saturated blue/cyan light.
        # On neutral white fill it can follow a silver glare island (frame 22)
        # instead of the nozzle edge; opposed radial refinement is more stable.
        if blue_nozzle_fill:
            coloured_fit = _fit_coloured_outer_ring(raw_gray, refined)
            if coloured_fit is not None:
                contour_circle, contour_score = coloured_fit
                contour_closure = _bright_rim_closure(gray, contour_circle)
                if (
                    contour_closure.opposed_coverage
                    >= max(0.32, closure.opposed_coverage - 0.20)
                    and contour_closure.sector_coverage >= 0.62
                ):
                    refined = contour_circle
                    closure = contour_closure
                    rim_score = max(
                        rim_score,
                        80.0 + 12.0 * (contour_score - 3.25),
                    )
        if not blue_nozzle_fill and min_radius_px is not None and refined.radius < min_radius_px:
            continue
        if not blue_nozzle_fill and max_radius_px is not None and refined.radius > max_radius_px:
            continue
        support = _robust_local_hex_support(gray, refined)
        if support is None:
            support = _robust_local_partial_face_support(gray, refined)
        support_ok = bool(
            support is not None
            and (
                (support.vertices == 6 and support.score >= minimum_hex_score)
                or (3 <= support.vertices <= 5 and support.score >= 2.75)
            )
        )
        if not support_ok:
            # Local threshold contours can change when the refined crop moves by
            # one pixel, especially around the small blue-lit ring.  The seed
            # already proved an exact six-sided hull; accept that same hull only
            # when the refined circle is *conservatively* still inside it.
            if initial_support is None:
                continue
            shift = math.hypot(refined.x - initial.x, refined.y - initial.y)
            centre_ratio = math.hypot(
                refined.x - initial_support.x,
                refined.y - initial_support.y,
            ) / max(1.0, initial_support.size)
            radius_ratio = refined.radius / max(1.0, initial_support.size)
            remaining_margin = initial_support.containment_margin - shift
            if not (
                initial_support.vertices >= 3
                and centre_ratio <= 0.16
                and 0.105 <= radius_ratio <= 0.205
                and remaining_margin >= 1.04 * refined.radius
            ):
                continue
            support = initial_support
        rank = (
            rim_score
            + 48.0 * closure.opposed_coverage
            + 12.0 * closure.sector_coverage
            + 0.10 * max(0.0, closure.lower_drop)
            + 5.0 * (support.score - 2.75)
        )
        refined_candidates.append((rank, refined, rim_score, support))
    if not refined_candidates:
        return None

    refined_candidates.sort(key=lambda item: item[0], reverse=True)
    _, refined, rim_score, final_support = refined_candidates[0]

    # Candidates attached to the same metal face are alternative radii of one
    # physical object.  A similarly strong candidate on a *different* face is
    # ambiguous; refusing that frame is safer than jumping to printer scenery.
    for alternate_rank, alternate, _, alternate_support in refined_candidates[1:]:
        same_face = math.hypot(
            alternate_support.x - final_support.x,
            alternate_support.y - final_support.y,
        ) <= 0.28 * max(alternate_support.size, final_support.size)
        if not same_face and alternate_rank >= refined_candidates[0][0] - 7.0:
            return None

    # The required target is the complete *outer nozzle edge*.  The central
    # aperture may be filled with material or invisible and is never used as
    # the final output circle.
    output_circle = refined

    confidence = float(
        np.clip(
            0.88 * (rim_score - 70.0) / 125.0
            + 0.12 * (final_support.score - 3.40) / 1.40,
            0.0,
            0.995,
        )
    )
    final_closure = _bright_rim_closure(gray, refined)
    closure_floor = 0.36 + 0.42 * float(
        np.clip(
            (final_closure.opposed_coverage - 0.42) / 0.58,
            0.0,
            1.0,
        )
    )
    confidence = max(confidence, closure_floor)
    if strong_coloured_fill:
        core_level, ring_level, outside_level = _bright_rim_levels(gray, refined)
        dual_contrast = min(ring_level - core_level, ring_level - outside_level)
        coloured_floor = 0.38 + 0.24 * float(
            np.clip((dual_contrast - 45.0) / 105.0, 0.0, 1.0)
        )
        confidence = max(confidence, coloured_floor)
    if confidence < min_confidence:
        raise RuntimeError(
            f"LED 补光喷口外缘证据不足：confidence={confidence:.3f} < "
            f"{min_confidence:.3f}"
        )
    return Detection(circle=output_circle, confidence=confidence, score=rim_score)

def _detect_led_bright_rim_in_fixed_roi(
    image: np.ndarray,
    *,
    min_confidence: float,
    allowed_x: tuple[float, float],
    allowed_y: tuple[float, float],
    min_radius_px: float,
    max_radius_px: float,
    allowed_region_circle: Optional[tuple[float, float, float]],
) -> Optional[Detection]:
    """Run the unchanged R16 LED proof on a bounded physical context.

    The A133 bottleneck was not HTTP: ``_find_hex_faces`` and the R16 LED
    proof repeatedly prepared the whole 640x480 frame although the machine
    already supplied an absolute 80x72 gate.  Keep enough padding for the
    surrounding 3--6 sided metal face, run the same R16 proof on that context,
    then translate the proven outer-rim circle back to full-frame pixels.
    No thresholds or acceptance gates are relaxed.
    """
    height, width = image.shape[:2]
    padding = max(56, int(math.ceil(4.5 * float(max_radius_px))))
    crop_x0 = max(0, int(math.floor(allowed_x[0])) - padding)
    crop_y0 = max(0, int(math.floor(allowed_y[0])) - padding)
    crop_x1 = min(width, int(math.ceil(allowed_x[1])) + padding)
    crop_y1 = min(height, int(math.ceil(allowed_y[1])) + padding)
    if crop_x1 - crop_x0 < 64 or crop_y1 - crop_y0 < 64:
        return None

    context = image[crop_y0:crop_y1, crop_x0:crop_x1]
    local = _detect_led_bright_rim(
        context,
        min_confidence=min_confidence,
        allowed_x=(allowed_x[0] - crop_x0, allowed_x[1] - crop_x0),
        allowed_y=(allowed_y[0] - crop_y0, allowed_y[1] - crop_y0),
        min_radius_px=min_radius_px,
        max_radius_px=max_radius_px,
        lighting_reference=image,
    )
    if local is None:
        return None
    mapped = Detection(
        circle=Circle(
            local.circle.x + crop_x0,
            local.circle.y + crop_y0,
            local.circle.radius,
        ),
        confidence=local.confidence,
        score=local.score,
    )
    if not _circle_fits_inside_region(mapped.circle, allowed_region_circle):
        return None
    return mapped

def _circle_features(prepared: _PreparedImage, circle: Circle, samples: int = 240) -> _ScoredCircle:
    if not _inside_image(circle, prepared.gray.shape):
        return _ScoredCircle(circle, -1e9, -1.0, 0.0, 0.0, 0.0, -1.0, 255.0, 0.0)

    angle_count = int(np.clip(samples, 160, 480))
    angles = np.linspace(0.0, 2.0 * math.pi, angle_count, endpoint=False, dtype=np.float32)
    cosines = np.cos(angles)
    sines = np.sin(angles)

    def ring(radius_factor: float, source: np.ndarray = prepared.smooth) -> np.ndarray:
        radius = circle.radius * radius_factor
        return _sample(source, circle.x + radius * cosines, circle.y + radius * sines)

    inside = np.median(np.stack([ring(factor) for factor in (0.72, 0.79, 0.86, 0.91)]), axis=0)
    outside = np.median(np.stack([ring(factor) for factor in (1.08, 1.14, 1.20, 1.26)]), axis=0)
    outward_contrast = outside - inside
    contrast = float(np.median(outward_contrast)) / prepared.dynamic_range
    contrast_threshold = max(3.0, prepared.dynamic_range * 0.035)
    contrast_coverage = float(np.mean(outward_contrast > contrast_threshold))

    radial_gradients = []
    exact_radial_gradient = None
    for factor in (0.94, 0.98, 1.00, 1.02, 1.06):
        radius = circle.radius * factor
        xs = circle.x + radius * cosines
        ys = circle.y + radius * sines
        radial = _sample(prepared.grad_x, xs, ys) * cosines + _sample(prepared.grad_y, xs, ys) * sines
        radial_gradients.append(radial)
        if factor == 1.00:
            exact_radial_gradient = radial
    gradient_stack = np.stack(radial_gradients)
    strongest_gradient = np.max(gradient_stack, axis=0)
    edge_threshold = prepared.grad_reference * 0.20
    edge_coverage = float(np.mean(strongest_gradient > edge_threshold))
    positive_gradient = np.maximum(strongest_gradient, 0.0)
    edge_strength = float(np.median(positive_gradient)) / prepared.grad_reference
    exact_edge = float(np.mean(np.clip(exact_radial_gradient / prepared.grad_reference, 0.0, 2.5)))

    # Disc darkness uses several interior rings, not the centre, because the
    # centre may contain a bright pin or a red part.
    inner_level = float(np.median(np.concatenate([ring(factor) for factor in (0.48, 0.64, 0.78, 0.88)])))
    outer_level = float(np.median(np.concatenate([ring(factor) for factor in (1.10, 1.20, 1.30)])))
    darkness = (outer_level - inner_level) / prepared.dynamic_range

    # Real nozzles in both views have the same radial topology: a pin / red
    # core, a dark annulus, then the brighter metal outside.  A corner speck
    # generally has a dark centre and only gets brighter outwards.  This term
    # therefore removes tiny high-gradient false positives without adding an
    # image-centre prior.
    core_level = float(np.median(np.concatenate([ring(factor) for factor in (0.0, 0.10, 0.20, 0.30)])))
    core_contrast = (core_level - inner_level) / prepared.dynamic_range
    core_saturation = float(
        np.median(
            np.concatenate(
                [ring(factor, prepared.saturation) for factor in (0.0, 0.10, 0.20, 0.30)]
            )
        )
    ) / 255.0

    # Full-circle evidence matters more than a single strong arc. This is the
    # main protection against dark image corners and straight housing edges.
    excessive_core_penalty = 3.2 * max(0.0, core_contrast - 0.82)
    score = (
        2.25 * float(np.clip(contrast, -0.5, 1.5))
        + 1.35 * contrast_coverage
        + 1.55 * edge_coverage
        + 0.85 * float(np.clip(edge_strength, 0.0, 2.0))
        + 0.50 * float(np.clip(exact_edge, 0.0, 1.5))
        + 0.70 * float(np.clip(darkness, -0.5, 1.2))
        + 1.45 * float(np.clip(core_contrast, -0.7, 1.2))
        - excessive_core_penalty
    )
    return _ScoredCircle(
        circle=circle,
        score=score,
        contrast=contrast,
        contrast_coverage=contrast_coverage,
        edge_coverage=edge_coverage,
        edge_strength=edge_strength,
        core_contrast=core_contrast,
        core_level=core_level,
        core_saturation=core_saturation,
    )

def _dark_disc_seed(prepared: _PreparedImage, initial: Circle) -> Circle:
    """Recentre on the dark disc before edge fitting to reduce left/right drift."""
    height, width = prepared.smooth.shape
    radius = initial.radius
    y0 = max(0, int(math.floor(initial.y - radius)))
    y1 = min(height, int(math.ceil(initial.y + radius)) + 1)
    x0 = max(0, int(math.floor(initial.x - radius)))
    x1 = min(width, int(math.ceil(initial.x + radius)) + 1)
    if y1 <= y0 or x1 <= x0:
        return initial

    ys, xs = np.mgrid[y0:y1, x0:x1]
    dist = np.hypot(xs - initial.x, ys - initial.y)
    disc = dist <= radius * 0.92
    if int(np.count_nonzero(disc)) < 40:
        return initial

    patch = prepared.smooth[y0:y1, x0:x1]
    # Inverse brightness weights pull the seed toward the dark cavity interior.
    weights = np.clip(255.0 - patch.astype(np.float64), 8.0, None)
    weights = np.where(disc, weights, 0.0)
    total = float(np.sum(weights))
    if total <= 1e-6:
        return initial
    cx = float(np.sum(xs * weights) / total)
    cy = float(np.sum(ys * weights) / total)
    if not _inside_image(Circle(cx, cy, radius), prepared.gray.shape):
        return initial
    return Circle(cx, cy, radius)

def _refine_circle(
    prepared: _PreparedImage,
    initial: Circle,
    max_radius_change: float = 0.07,
) -> _ScoredCircle:
    current = initial
    best = _circle_features(prepared, current, samples=300)
    minimum_radius = initial.radius * (1.0 - max_radius_change)
    maximum_radius = initial.radius * (1.0 + max_radius_change)

    # Alternating radius and centre search is stable on low-resolution/noisy
    # screenshots and avoids the centre bias of a raw enclosing contour.
    for iteration in range(4):
        radius_span = current.radius * (0.065 if iteration == 0 else 0.028)
        radius_offsets = np.linspace(-radius_span, radius_span, 11)
        for offset in radius_offsets:
            trial_radius = current.radius + float(offset)
            if trial_radius < max(4.0, minimum_radius) or trial_radius > maximum_radius:
                continue
            trial = Circle(current.x, current.y, trial_radius)
            scored = _circle_features(prepared, trial, samples=300)
            if scored.score > best.score:
                best = scored
                current = trial

        step = max(0.45, current.radius * (0.055 if iteration == 0 else 0.022))
        origin = current
        for dy in (-step, 0.0, step):
            for dx in (-step, 0.0, step):
                trial = Circle(origin.x + dx, origin.y + dy, current.radius)
                scored = _circle_features(prepared, trial, samples=300)
                if scored.score > best.score:
                    best = scored
                    current = trial
        current = best.circle

    return _circle_features(prepared, current, samples=480)

def _fit_dark_blob_circle(
    prepared: _PreparedImage,
    initial: Circle,
) -> Optional[Circle]:
    """Fit the outer contour of the dark cavity blob around the candidate."""
    height, width = prepared.gray.shape
    pad = max(8, int(initial.radius * 0.35))
    x0 = max(0, int(math.floor(initial.x - initial.radius - pad)))
    x1 = min(width, int(math.ceil(initial.x + initial.radius + pad)) + 1)
    y0 = max(0, int(math.floor(initial.y - initial.radius - pad)))
    y1 = min(height, int(math.ceil(initial.y + initial.radius + pad)) + 1)
    if x1 - x0 < 20 or y1 - y0 < 20:
        return None

    roi = prepared.gray[y0:y1, x0:x1]
    cx_local = initial.x - x0
    cy_local = initial.y - y0
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    best_circle: Optional[Circle] = None
    best_score = -1e18

    for percentile in (28, 36, 44, 52):
        thr = float(np.percentile(roi, percentile))
        mask = (roi <= thr).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        num, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if num <= 1:
            continue

        best_label = 0
        best_rank = -1e18
        min_area = math.pi * (initial.radius * 0.30) ** 2
        max_area = math.pi * (initial.radius * 1.50) ** 2
        for label in range(1, num):
            area = float(stats[label, cv2.CC_STAT_AREA])
            if area < min_area or area > max_area:
                continue
            lx, ly = float(centroids[label, 0]), float(centroids[label, 1])
            centre_dist = math.hypot(lx - cx_local, ly - cy_local)
            if centre_dist > 0.50 * initial.radius:
                continue
            rank = area - 2.5 * centre_dist * initial.radius
            iy = int(np.clip(round(cy_local), 0, labels.shape[0] - 1))
            ix = int(np.clip(round(cx_local), 0, labels.shape[1] - 1))
            if labels[iy, ix] == label:
                rank += area
            if rank > best_rank:
                best_rank = rank
                best_label = label
        if best_label == 0:
            continue

        component = np.where(labels == best_label, 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        if len(contour) < 40:
            continue
        (ex, ey), (axis_a, axis_b), _ = cv2.fitEllipse(contour)
        major = max(axis_a, axis_b)
        minor = min(axis_a, axis_b)
        if major <= 0 or minor / major < 0.70:
            continue
        radius = 0.25 * (axis_a + axis_b)
        fitted = Circle(float(ex + x0), float(ey + y0), float(radius))
        if not _inside_image(fitted, prepared.gray.shape, margin_ratio=0.008):
            continue
        if abs(fitted.radius - initial.radius) > 0.45 * initial.radius:
            continue
        if math.hypot(fitted.x - initial.x, fitted.y - initial.y) > 0.40 * initial.radius:
            continue
        scored = _circle_features(prepared, fitted, samples=240)
        rank_score = scored.score + 0.7 * scored.edge_coverage + 0.3 * scored.contrast_coverage
        if rank_score > best_score:
            best_score = rank_score
            best_circle = fitted
    return best_circle

def _fit_cavity_by_core(
    prepared: _PreparedImage,
    initial: Circle,
) -> Optional[_ScoredCircle]:
    """Lock centre on the bright pin, then sweep radius onto the outer rim."""
    core = _bright_core_seed(prepared, initial)
    shift = math.hypot(core.x - initial.x, core.y - initial.y)
    # Only trust this path when a distinct luminous core is present.
    probe = _circle_features(prepared, Circle(core.x, core.y, initial.radius), samples=240)
    if probe.core_contrast < 0.18 and probe.core_saturation < 0.14:
        return None
    if shift > 0.28 * initial.radius:
        return None

    best: Optional[_ScoredCircle] = None
    radius_factors = np.linspace(0.78, 1.28, 29)
    for factor in radius_factors:
        trial = Circle(core.x, core.y, initial.radius * float(factor))
        if not _inside_image(trial, prepared.gray.shape, margin_ratio=0.01):
            continue
        scored = _circle_features(prepared, trial, samples=320)
        rank = (
            scored.score
            + 0.55 * scored.edge_coverage
            + 0.35 * scored.contrast_coverage
            + 0.25 * float(np.clip(scored.contrast, 0.0, 1.0))
        )
        if best is None or rank > (
            best.score
            + 0.55 * best.edge_coverage
            + 0.35 * best.contrast_coverage
            + 0.25 * float(np.clip(best.contrast, 0.0, 1.0))
        ):
            best = scored

    if best is None:
        return None

    # Micro-adjust centre around the core while keeping the chosen radius.
    current = best.circle
    step = max(0.6, current.radius * 0.012)
    for dy in (-step, 0.0, step):
        for dx in (-step, 0.0, step):
            trial = Circle(core.x + dx, core.y + dy, current.radius)
            if not _inside_image(trial, prepared.gray.shape, margin_ratio=0.01):
                continue
            scored = _circle_features(prepared, trial, samples=360)
            if scored.score + 0.45 * scored.edge_coverage > best.score + 0.45 * best.edge_coverage:
                best = scored
                current = trial

    locked = _radius_from_outer_peaks(prepared, Circle(best.circle.x, best.circle.y, best.circle.radius))
    if locked is not None:
        locked_score = _circle_features(prepared, locked, samples=480)
        if locked_score.score + 0.4 * locked_score.edge_coverage >= best.score + 0.4 * best.edge_coverage - 0.05:
            best = locked_score
    return best

def _fit_outer_edge(prepared: _PreparedImage, initial: Circle) -> _ScoredCircle:
    """Robustly fit a circle to outward-facing radial edge points.

    Hough/contour candidates are intentionally redundant and can have a
    slightly biased centre.  Sampling along rays, followed by iterative
    outlier rejection, makes the final geometry much less sensitive to glare,
    broken arcs and threshold choice.
    """
    seeded = _dark_disc_seed(prepared, initial)
    # For bright-core cavities, pull the seed toward the luminous pin so the
    # outer-rim search stays concentric with the true aperture.
    seeded = _bright_core_seed(prepared, seeded)
    working = seeded
    angle_count = 420
    angles = np.linspace(0.0, 2.0 * math.pi, angle_count, endpoint=False, dtype=np.float32)
    cosines = np.cos(angles)
    sines = np.sin(angles)
    factors = np.linspace(0.70, 1.22, 65, dtype=np.float32)

    fitted = working
    for _round in range(3):
        radial_gradients = []
        for factor in factors:
            radius = working.radius * float(factor)
            xs = working.x + radius * cosines
            ys = working.y + radius * sines
            radial_gradients.append(
                _sample(prepared.grad_x, xs, ys) * cosines
                + _sample(prepared.grad_y, xs, ys) * sines
            )

        gradient_stack = np.stack(radial_gradients)
        threshold = prepared.grad_reference * 0.16
        # Prefer the first strong outward gradient after mid-ring. Absolute
        # outermost peaks latch onto background clutter and inflate/drift.
        above = gradient_stack >= threshold
        has_peak = np.any(above, axis=0)
        peak_indices = np.where(
            has_peak,
            np.argmax(above, axis=0),
            np.argmax(gradient_stack, axis=0),
        )
        peak_strengths = gradient_stack[peak_indices, np.arange(angle_count)]
        peak_factors = factors[peak_indices]
        supported = has_peak & (peak_strengths > threshold * 0.85)
        if float(np.mean(supported)) < 0.38:
            break

        points = np.column_stack(
            (
                working.x + working.radius * peak_factors[supported] * cosines[supported],
                working.y + working.radius * peak_factors[supported] * sines[supported],
            )
        ).astype(np.float64)

        candidate = working
        for _ in range(5):
            if len(points) < 70:
                break
            design = np.column_stack((2.0 * points[:, 0], 2.0 * points[:, 1], np.ones(len(points))))
            target = np.sum(points * points, axis=1)
            centre_x, centre_y, constant = np.linalg.lstsq(design, target, rcond=None)[0]
            radius_squared = constant + centre_x * centre_x + centre_y * centre_y
            if radius_squared <= 0:
                break
            radius = math.sqrt(float(radius_squared))
            candidate = Circle(float(centre_x), float(centre_y), radius)
            residuals = np.abs(
                np.hypot(points[:, 0] - candidate.x, points[:, 1] - candidate.y) - candidate.radius
            )
            median_residual = float(np.median(residuals))
            keep = residuals <= max(1.2, 2.2 * median_residual)
            if int(np.count_nonzero(keep)) < 70:
                break
            points = points[keep]
        working = candidate
        fitted = candidate

    centre_shift = math.hypot(fitted.x - initial.x, fitted.y - initial.y)
    if (
        centre_shift > 0.25 * initial.radius
        or not 0.78 * initial.radius <= fitted.radius <= 1.28 * initial.radius
        or not _inside_image(fitted, prepared.gray.shape)
    ):
        fitted = seeded if _inside_image(seeded, prepared.gray.shape) else initial

    # Keep bright-core concentric radius polish when the seed is trustworthy.
    core_locked = _radius_from_outer_peaks(prepared, seeded)
    cavity_locked = _fit_cavity_by_core(prepared, initial)
    blob = _fit_dark_blob_circle(prepared, seeded)
    candidates = [initial, fitted]
    if core_locked is not None:
        candidates.append(core_locked)
    if blob is not None:
        candidates.append(blob)

    scored = [_circle_features(prepared, circle, samples=480) for circle in candidates]
    if cavity_locked is not None:
        scored.append(cavity_locked)

    def rank(item: _ScoredCircle) -> float:
        # Dark-blob fits are geometrically closer to the visible outer rim; give
        # them a mild prior when photometric evidence stays competitive.
        bonus = 0.0
        if blob is not None and abs(item.circle.radius - blob.radius) < 0.04 * blob.radius:
            if math.hypot(item.circle.x - blob.x, item.circle.y - blob.y) < 0.04 * blob.radius:
                bonus = 0.55
        return item.score + 0.60 * item.edge_coverage + 0.30 * item.contrast_coverage + bonus

    best = max(scored, key=rank)

    # Re-anchor radius to the first dark-to-bright rim while freezing centre.
    outer = _radius_from_outer_peaks(
        prepared,
        Circle(best.circle.x, best.circle.y, best.circle.radius),
        factor_low=0.82,
        factor_high=1.20,
        prefer_percentile=55,
    )
    if outer is not None:
        outer_score = _circle_features(prepared, outer, samples=480)
        if rank(outer_score) >= rank(best) - 0.85:
            best = outer_score

    # Keep centre nearly fixed during polish so photometric search cannot reintroduce drift.
    polished = _refine_circle(prepared, best.circle, max_radius_change=0.025)
    centre_shift = math.hypot(polished.circle.x - best.circle.x, polished.circle.y - best.circle.y)
    if centre_shift > 0.02 * best.circle.radius:
        radius_only = Circle(best.circle.x, best.circle.y, polished.circle.radius)
        polished = _circle_features(prepared, radius_only, samples=480)
    # Never allow polish to shrink away from a verified outer rim.
    if polished.circle.radius < best.circle.radius * 0.97:
        polished = _circle_features(
            prepared,
            Circle(best.circle.x, best.circle.y, best.circle.radius),
            samples=480,
        )
    return polished if rank(polished) >= rank(best) - 0.08 else best

def _bright_core_seed(prepared: _PreparedImage, initial: Circle) -> Circle:
    """If a luminous pin exists, use it to stabilise the cavity centre."""
    height, width = prepared.smooth.shape
    radius = initial.radius
    y0 = max(0, int(math.floor(initial.y - radius * 0.55)))
    y1 = min(height, int(math.ceil(initial.y + radius * 0.55)) + 1)
    x0 = max(0, int(math.floor(initial.x - radius * 0.55)))
    x1 = min(width, int(math.ceil(initial.x + radius * 0.55)) + 1)
    if y1 <= y0 or x1 <= x0:
        return initial

    ys, xs = np.mgrid[y0:y1, x0:x1]
    dist = np.hypot(xs - initial.x, ys - initial.y)
    disc = dist <= radius * 0.45
    if int(np.count_nonzero(disc)) < 25:
        return initial
    patch = prepared.smooth[y0:y1, x0:x1].astype(np.float64)
    values = patch[disc]
    bright_floor = float(np.percentile(values, 88))
    local_mean = float(np.mean(values))
    if bright_floor < local_mean + max(12.0, prepared.dynamic_range * 0.12):
        return initial
    core = disc & (patch >= bright_floor)
    if int(np.count_nonzero(core)) < 8:
        return initial
    cx = float(np.mean(xs[core]))
    cy = float(np.mean(ys[core]))
    if math.hypot(cx - initial.x, cy - initial.y) > 0.22 * radius:
        return initial
    seeded = Circle(cx, cy, radius)
    return seeded if _inside_image(seeded, prepared.gray.shape) else initial

def _radius_from_outer_peaks(
    prepared: _PreparedImage,
    centre: Circle,
    factor_low: float = 0.78,
    factor_high: float = 1.22,
    prefer_percentile: float = 50,
) -> Optional[Circle]:
    """Estimate rim radius using the first strong outward gradient after mid-ring.

    Using the absolute outermost peak over a wide radial range latches onto
    background clutter.  For cavity/nozzle rims the desired edge is the first
    dark-to-bright transition beyond the dark annulus.
    """
    angle_count = 360
    angles = np.linspace(0.0, 2.0 * math.pi, angle_count, endpoint=False, dtype=np.float32)
    cosines = np.cos(angles)
    sines = np.sin(angles)
    factors = np.linspace(factor_low, factor_high, 70, dtype=np.float32)
    stack = []
    for factor in factors:
        radius = centre.radius * float(factor)
        xs = centre.x + radius * cosines
        ys = centre.y + radius * sines
        stack.append(
            _sample(prepared.grad_x, xs, ys) * cosines
            + _sample(prepared.grad_y, xs, ys) * sines
        )
    gradient_stack = np.stack(stack)
    threshold = prepared.grad_reference * 0.18
    above = gradient_stack >= threshold
    if float(np.mean(np.any(above, axis=0))) < 0.40:
        return None
    # First peak beyond mid-ring, not the farthest clutter edge.
    first_indices = np.where(
        np.any(above, axis=0),
        np.argmax(above, axis=0),
        np.argmax(gradient_stack, axis=0),
    )
    supported = np.any(above, axis=0)
    radii = centre.radius * factors[first_indices][supported]
    if len(radii) < 80:
        return None
    # Reject rays whose first peak is an extreme outlier before aggregating.
    median_radius = float(np.median(radii))
    kept = radii[np.abs(radii - median_radius) <= max(3.0, 0.12 * median_radius)]
    if len(kept) < 60:
        kept = radii
    radius = float(np.percentile(kept, prefer_percentile))
    fitted = Circle(centre.x, centre.y, radius)
    return fitted if _inside_image(fitted, prepared.gray.shape) else None

def _fit_rim_points(prepared: _PreparedImage, initial: Circle) -> Optional[Circle]:
    angle_count = 360
    angles = np.linspace(0.0, 2.0 * math.pi, angle_count, endpoint=False, dtype=np.float32)
    cosines = np.cos(angles)
    sines = np.sin(angles)
    factors = np.linspace(0.84, 1.16, 49, dtype=np.float32)
    stack = []
    for factor in factors:
        radius = initial.radius * float(factor)
        xs = initial.x + radius * cosines
        ys = initial.y + radius * sines
        stack.append(
            _sample(prepared.grad_x, xs, ys) * cosines
            + _sample(prepared.grad_y, xs, ys) * sines
        )
    gradient_stack = np.stack(stack)
    threshold = prepared.grad_reference * 0.18
    above = gradient_stack >= threshold
    if float(np.mean(np.any(above, axis=0))) < 0.45:
        return None
    first_indices = np.where(
        np.any(above, axis=0),
        np.argmax(above, axis=0),
        np.argmax(gradient_stack, axis=0),
    )
    supported = np.any(above, axis=0)
    points = np.column_stack(
        (
            initial.x + initial.radius * factors[first_indices][supported] * cosines[supported],
            initial.y + initial.radius * factors[first_indices][supported] * sines[supported],
        )
    ).astype(np.float64)
    fitted = initial
    for _ in range(5):
        if len(points) < 80:
            return None
        design = np.column_stack((2.0 * points[:, 0], 2.0 * points[:, 1], np.ones(len(points))))
        target = np.sum(points * points, axis=1)
        centre_x, centre_y, constant = np.linalg.lstsq(design, target, rcond=None)[0]
        radius_squared = constant + centre_x * centre_x + centre_y * centre_y
        if radius_squared <= 0:
            return None
        candidate = Circle(float(centre_x), float(centre_y), math.sqrt(float(radius_squared)))
        if math.hypot(candidate.x - initial.x, candidate.y - initial.y) > 0.08 * initial.radius:
            candidate = Circle(initial.x, initial.y, candidate.radius)
        residuals = np.abs(
            np.hypot(points[:, 0] - candidate.x, points[:, 1] - candidate.y) - candidate.radius
        )
        median_residual = float(np.median(residuals))
        keep = residuals <= max(1.2, 2.0 * median_residual)
        if int(np.count_nonzero(keep)) < 80:
            break
        points = points[keep]
        fitted = candidate
    if not _inside_image(fitted, prepared.gray.shape, margin_ratio=0.008):
        return None
    if not 0.88 * initial.radius <= fitted.radius <= 1.12 * initial.radius:
        return None
    return fitted

def _refine_small_radius(prepared: _PreparedImage, initial: Circle) -> _ScoredCircle:
    """Snap a small tip circle to its exact outward edge.

    Keeps the refined centre fixed and selects radius from the
    full-circumference exact-radius outward gradient. Search bounds are
    relative to the current radius so arbitrary image scales remain valid.
    """
    angle_count = 480
    angles = np.linspace(0.0, 2.0 * math.pi, angle_count, endpoint=False, dtype=np.float32)
    cosines = np.cos(angles)
    sines = np.sin(angles)
    radii = np.linspace(0.80 * initial.radius, 1.18 * initial.radius, 61)
    objectives: list[float] = []
    for radius in radii:
        xs = initial.x + float(radius) * cosines
        ys = initial.y + float(radius) * sines
        radial = (
            _sample(prepared.grad_x, xs, ys) * cosines
            + _sample(prepared.grad_y, xs, ys) * sines
        )
        normalized = np.clip(radial / prepared.grad_reference, 0.0, 8.0)
        coverage = float(np.mean(normalized > 0.20))
        objectives.append(float(np.mean(normalized)) * (0.70 + 0.30 * coverage))

    best_index = int(np.argmax(objectives))
    radius = float(radii[best_index])
    if 0 < best_index < len(radii) - 1:
        left, middle, right = objectives[best_index - 1 : best_index + 2]
        denominator = left - 2.0 * middle + right
        if abs(denominator) > 1e-8:
            offset = float(np.clip(0.5 * (left - right) / denominator, -1.0, 1.0))
            radius += offset * float(radii[1] - radii[0])

    snapped = Circle(initial.x, initial.y, radius)
    if abs(snapped.radius - initial.radius) > 0.16 * initial.radius:
        return _circle_features(prepared, initial, samples=480)
    return _circle_features(prepared, snapped, samples=480)

def _fit_small_visible_rim(
    prepared: _PreparedImage,
    preliminary: _ScoredCircle,
) -> _ScoredCircle:
    """Fit the physical small-aperture rim with illumination-normalized pairs.

    Absolute gradient strength is deliberately not used to choose the final
    edge: a saturated metal highlight can be many times stronger than the
    opposite, normally illuminated rim and would pull a least-squares circle
    toward that highlight.  Instead, each radial profile is normalized by its
    own dark-inside and bright-outside levels.  The physical edge is the 50%
    transition of that local profile.

    Opposite rays are then paired.  Their average fixes radius while their
    difference fixes the centre projection, so no single glare arc or glue
    blob can decide both centre and radius.  Three nearby transition levels
    must agree before the correction is accepted; otherwise the already
    validated preliminary circle is retained.
    """
    initial = preliminary.circle
    if preliminary.contrast_coverage < 0.85 or preliminary.edge_coverage < 0.85:
        return preliminary

    angle_count = 720
    half_count = angle_count // 2
    angles = np.linspace(
        0.0,
        2.0 * math.pi,
        angle_count,
        endpoint=False,
        dtype=np.float32,
    )
    cosines = np.cos(angles)
    sines = np.sin(angles)
    factors = np.linspace(0.68, 1.32, 161, dtype=np.float32)
    profiles = np.stack(
        [
            _sample(
                prepared.smooth,
                initial.x + initial.radius * float(factor) * cosines,
                initial.y + initial.radius * float(factor) * sines,
            )
            for factor in factors
        ]
    ).astype(np.float64)
    # Only blur radially. Angular smoothing would spread a small glare arc into
    # neighbouring rays and give it more geometric influence than it deserves.
    profiles = cv2.GaussianBlur(profiles, (1, 0), 1.15)
    inside_band = (factors >= 0.72) & (factors <= 0.84)
    outside_band = (factors >= 1.16) & (factors <= 1.28)
    inside = np.median(profiles[inside_band], axis=0)
    outside = np.median(profiles[outside_band], axis=0)
    transition = outside - inside
    normalized = (profiles - inside[None, :]) / np.maximum(
        transition[None, :],
        1e-6,
    )
    radial_derivative = np.gradient(profiles, axis=0)
    noise = np.std(profiles[inside_band], axis=0) + 1.0
    minimum_transition = max(4.0, prepared.dynamic_range * 0.045)
    search_indices = np.flatnonzero((factors >= 0.84) & (factors <= 1.16))

    def weighted_median(values: np.ndarray, weights: np.ndarray) -> float:
        order = np.argsort(values)
        ordered_values = values[order]
        ordered_weights = weights[order]
        cumulative = np.cumsum(ordered_weights)
        index = int(np.searchsorted(cumulative, 0.5 * cumulative[-1]))
        return float(ordered_values[min(index, len(ordered_values) - 1)])

    def robust_projection_fit(
        design: np.ndarray,
        target: np.ndarray,
        base_weights: np.ndarray,
    ) -> np.ndarray:
        weights = base_weights.astype(np.float64).copy()
        result = np.zeros(design.shape[1], dtype=np.float64)
        for _ in range(8):
            root = np.sqrt(np.maximum(weights, 1e-6))
            result = np.linalg.lstsq(
                design * root[:, None],
                target * root,
                rcond=None,
            )[0]
            residual = target - design @ result
            scale = max(
                0.35,
                1.4826
                * float(np.median(np.abs(residual - np.median(residual)))),
            )
            huber = np.minimum(
                1.0,
                (1.5 * scale) / np.maximum(np.abs(residual), 1e-6),
            )
            weights = base_weights * huber
        return result

    def fit_transition_fraction(fraction: float) -> Optional[Circle]:
        local = normalized[search_indices]
        above = local >= fraction
        has_crossing = np.any(above, axis=0)
        first = np.argmax(above, axis=0)
        upper_indices = search_indices[first]
        lower_indices = np.maximum(upper_indices - 1, 0)
        columns = np.arange(angle_count)
        lower_values = normalized[lower_indices, columns]
        upper_values = normalized[upper_indices, columns]
        interpolation = np.clip(
            (fraction - lower_values)
            / np.maximum(upper_values - lower_values, 1e-6),
            0.0,
            1.0,
        )
        ray_factors = factors[lower_indices] + interpolation * (
            factors[upper_indices] - factors[lower_indices]
        )
        ray_radii = initial.radius * ray_factors
        slopes = radial_derivative[upper_indices, columns]
        supported = (
            has_crossing
            & (transition >= minimum_transition)
            & (slopes >= 0.45)
            & (transition / noise >= 2.0)
        )

        pair_supported = supported[:half_count] & supported[half_count:]
        pair_radius = 0.5 * (
            ray_radii[:half_count] + ray_radii[half_count:]
        )
        pair_projection = 0.5 * (
            ray_radii[:half_count] - ray_radii[half_count:]
        )
        pair_transition = np.minimum(
            transition[:half_count],
            transition[half_count:],
        )
        pair_slope = np.minimum(slopes[:half_count], slopes[half_count:])
        pair_weights = np.clip(
            pair_transition / max(prepared.dynamic_range, 1.0),
            0.05,
            1.0,
        ) * np.clip(pair_slope / 8.0, 0.1, 1.0)
        valid = (
            pair_supported
            & (pair_radius >= 0.86 * initial.radius)
            & (pair_radius <= 1.14 * initial.radius)
        )
        if float(np.mean(valid)) < 0.36:
            return None
        median_radius = weighted_median(pair_radius[valid], pair_weights[valid])
        radius_mad = 1.4826 * float(
            np.median(np.abs(pair_radius[valid] - median_radius))
        )
        valid &= np.abs(pair_radius - median_radius) <= max(1.2, 2.8 * radius_mad)
        if float(np.mean(valid)) < 0.32:
            return None

        design = np.column_stack(
            (cosines[:half_count][valid], sines[:half_count][valid])
        ).astype(np.float64)
        geometry = design.T @ (pair_weights[valid, None] * design)
        eigenvalues = np.linalg.eigvalsh(geometry)
        if float(eigenvalues[0] / max(eigenvalues[-1], 1e-8)) < 0.10:
            return None
        correction = robust_projection_fit(
            design,
            pair_projection[valid],
            pair_weights[valid],
        )
        radius = weighted_median(pair_radius[valid], pair_weights[valid])
        return Circle(
            initial.x + float(correction[0]),
            initial.y + float(correction[1]),
            radius,
        )

    fits = [
        fit_transition_fraction(fraction)
        for fraction in (0.42, 0.50, 0.58)
    ]
    if any(item is None for item in fits):
        return preliminary
    fitted_circles = [item for item in fits if item is not None]
    centre_span = max(
        math.hypot(first.x - second.x, first.y - second.y)
        for first in fitted_circles
        for second in fitted_circles
    )
    radius_span = (
        max(circle.radius for circle in fitted_circles)
        - min(circle.radius for circle in fitted_circles)
    )
    if (
        centre_span > max(1.6, 0.065 * initial.radius)
        or radius_span > max(2.0, 0.10 * initial.radius)
    ):
        return preliminary
    fitted = fitted_circles[1]
    centre_shift = math.hypot(fitted.x - initial.x, fitted.y - initial.y)
    if (
        centre_shift > 0.16 * initial.radius
        or not 0.86 * initial.radius <= fitted.radius <= 1.14 * initial.radius
        or not _inside_image(fitted, prepared.gray.shape, margin_ratio=0.008)
    ):
        return preliminary

    scored = _circle_features(prepared, fitted, samples=480)
    if (
        scored.contrast_coverage >= 0.90
        and scored.edge_coverage >= 0.90
        and _rank_scored(scored) >= _rank_scored(preliminary) - 0.55
    ):
        return scored
    return preliminary

def _polish_small_dark_orifice(
    prepared: _PreparedImage,
    preliminary: _ScoredCircle,
) -> _ScoredCircle:
    """Stabilize a dark live-camera orifice using its filled dark component.

    Hough and radial scores can move by a couple of pixels when highlights on
    the surrounding metal change. The filled low-luminance component is much
    more stable: it fixes the centre, while the final 360-degree gradient snap
    restores the physical outer rim instead of retaining a threshold-dependent
    component radius.
    """
    if (
        preliminary.core_contrast >= 0.04
        or preliminary.contrast_coverage < 0.90
        or preliminary.edge_coverage < 0.90
    ):
        return preliminary

    blob = _fit_dark_blob_circle(prepared, preliminary.circle)
    if blob is None:
        return preliminary
    if (
        math.hypot(blob.x - preliminary.circle.x, blob.y - preliminary.circle.y)
        > 0.16 * preliminary.circle.radius
        or not 0.78 * preliminary.circle.radius
        <= blob.radius
        <= 1.08 * preliminary.circle.radius
    ):
        return preliminary

    seeded = Circle(blob.x, blob.y, 1.02 * blob.radius)
    polished = _refine_small_radius(prepared, seeded)
    if (
        polished.contrast_coverage >= 0.92
        and polished.edge_coverage >= 0.92
        and _rank_scored(polished) >= _rank_scored(preliminary) - 0.30
    ):
        return polished
    return preliminary

def _large_hough_hypotheses(
    prepared: _PreparedImage,
    anchor: Circle,
    thresholds: Sequence[int],
    radius_tolerance: float,
    use_short_side: bool = False,
) -> list[_HoughHypothesis]:
    """Collect location-independent weak-arc hypotheses near a winner."""
    scale_side = min(prepared.gray.shape) if use_short_side else max(prepared.gray.shape)
    hough_image = cv2.GaussianBlur(prepared.gray, (7, 7), 0)
    hypotheses: list[_HoughHypothesis] = []
    seen: set[tuple[float, float, float]] = set()
    for threshold in thresholds:
        circles = cv2.HoughCircles(
            hough_image,
            cv2.HOUGH_GRADIENT,
            dp=1.30,
            minDist=max(14, round(0.030 * scale_side)),
            param1=100,
            param2=threshold,
            minRadius=max(5, round(0.070 * scale_side)),
            maxRadius=round(0.490 * scale_side),
        )
        if circles is None:
            continue
        for x, y, radius in circles[0, :180]:
            circle = Circle(float(x), float(y), float(radius))
            if (
                math.hypot(circle.x - anchor.x, circle.y - anchor.y) > 0.25 * anchor.radius
                or abs(circle.radius - anchor.radius) > radius_tolerance * anchor.radius
                or not _inside_image(circle, prepared.gray.shape)
            ):
                continue
            key = (round(circle.x, 1), round(circle.y, 1), round(circle.radius, 1))
            if key in seen:
                continue
            seen.add(key)
            hypotheses.append(
                _HoughHypothesis(
                    circle=circle,
                    accumulator_threshold=threshold,
                    features=_circle_features(prepared, circle, samples=360),
                )
            )
    return hypotheses

def _correct_large_outer_geometry(
    prepared: _PreparedImage,
    winner: _ScoredCircle,
) -> _ScoredCircle:
    """Correct large-circle bias caused by glare, glue and broken outer arcs.

    Branching uses radius/short_side ratios so arbitrary resolutions stay valid.
    """
    circle = winner.circle
    short_side = min(prepared.gray.shape)
    relative_radius = circle.radius / float(short_side)

    if relative_radius >= 0.36 and winner.core_saturation < 0.32:
        hypotheses = _large_hough_hypotheses(
            prepared,
            circle,
            thresholds=(18, 22, 26),
            radius_tolerance=0.12,
        )
        reliable = [
            item
            for item in hypotheses
            if item.features.score >= 3.20
            and item.features.core_contrast > 0.10
            and item.features.contrast_coverage >= 0.60
            and item.features.edge_coverage >= 0.60
        ]
        if len(reliable) >= 4:
            xs = np.asarray([item.circle.x for item in reliable], dtype=np.float64)
            ys = np.asarray([item.circle.y for item in reliable], dtype=np.float64)
            radii = np.asarray([item.circle.radius for item in reliable], dtype=np.float64)
            consensus = Circle(float(np.mean(xs)), float(np.mean(ys)), float(np.mean(radii)))
            consensus_shift = math.hypot(consensus.x - circle.x, consensus.y - circle.y)
            if consensus_shift < 0.05 * circle.radius:
                consensus = Circle(
                    0.5 * (consensus.x + circle.x),
                    0.5 * (consensus.y + circle.y),
                    consensus.radius,
                )
            if (
                np.std(radii) <= 0.07 * circle.radius
                and consensus_shift <= 0.12 * circle.radius
                and _inside_image(consensus, prepared.gray.shape)
            ):
                scored = _circle_features(prepared, consensus, samples=480)
                if scored.score >= 3.40:
                    return scored

    if 0.18 <= relative_radius < 0.36 and winner.core_saturation < 0.32:
        hypotheses = _large_hough_hypotheses(
            prepared,
            circle,
            thresholds=(46, 40, 34),
            radius_tolerance=0.10,
        )
        reliable = [
            item
            for item in hypotheses
            if item.accumulator_threshold >= 40
            and item.features.score >= 3.20
            and item.features.core_contrast > 0.10
            and item.features.contrast_coverage >= 0.60
            and item.features.edge_coverage >= 0.55
        ]
        if reliable:
            strongest_threshold = max(item.accumulator_threshold for item in reliable)
            strongest = [
                item for item in reliable if item.accumulator_threshold == strongest_threshold
            ]
            selected = min(
                strongest,
                key=lambda item: (
                    abs(item.circle.radius - circle.radius),
                    -item.features.score,
                ),
            )
            return _circle_features(prepared, selected.circle, samples=480)

    if 0.18 <= relative_radius < 0.36 and winner.core_saturation >= 0.32:
        hypotheses = _large_hough_hypotheses(
            prepared,
            circle,
            thresholds=(30, 34, 40, 46),
            radius_tolerance=0.10,
        )
        complete = [
            item
            for item in hypotheses
            if item.features.score >= winner.score - 0.12
            and item.features.contrast_coverage >= 0.95
            and item.features.edge_coverage >= 0.95
        ]
        if complete:
            selected = max(complete, key=lambda item: item.features.score)
            return _circle_features(prepared, selected.circle, samples=480)

    if (
        0.08 <= relative_radius < 0.18
        and winner.core_saturation < 0.32
        and winner.core_contrast > 0.75
    ):
        hypotheses = _large_hough_hypotheses(
            prepared,
            circle,
            thresholds=(18,),
            radius_tolerance=0.16,
            use_short_side=True,
        )
        outer = [
            item
            for item in hypotheses
            if item.circle.radius >= 1.07 * circle.radius
            and item.features.score >= winner.score - 0.15
            and item.features.contrast_coverage >= 0.75
            and item.features.edge_coverage >= 0.70
        ]
        if outer:
            selected = max(outer, key=lambda item: item.features.score)
            dx = selected.circle.x - circle.x
            dy = selected.circle.y - circle.y
            corrected_x = circle.x if abs(dx) < 0.50 * abs(dy) else circle.x + 0.50 * dx
            corrected_y = circle.y if abs(dy) < 0.50 * abs(dx) else circle.y + 0.50 * dy
            corrected = Circle(corrected_x, corrected_y, selected.circle.radius)
            if _inside_image(corrected, prepared.gray.shape):
                return _circle_features(prepared, corrected, samples=480)

    return winner

def _fit_dark_aperture_edge(
    prepared: _PreparedImage,
    preliminary: _ScoredCircle,
) -> _ScoredCircle:
    """Recover the actual dark-aperture rim from a larger soft shadow."""
    initial = preliminary.circle
    short_side = min(prepared.gray.shape)
    if (
        initial.radius < 0.08 * short_side
        or preliminary.core_saturation >= 0.32
        or preliminary.core_contrast <= 0.12
    ):
        return preliminary

    height, width = prepared.gray.shape
    yy, xx = np.ogrid[:height, :width]
    disc = (xx - initial.x) ** 2 + (yy - initial.y) ** 2 <= initial.radius**2
    levels = prepared.gray[disc]
    if levels.size < 100:
        return preliminary
    thresholds = sorted(
        {
            int(round(float(np.percentile(levels, percentile))))
            for percentile in np.arange(46.0, 73.0, 2.0)
        }
    )

    angles = np.linspace(0.0, 2.0 * math.pi, 720, endpoint=False)
    probe_x = np.clip(
        np.rint(initial.x + 0.55 * initial.radius * np.cos(angles)).astype(np.int32),
        0,
        width - 1,
    )
    probe_y = np.clip(
        np.rint(initial.y + 0.55 * initial.radius * np.sin(angles)).astype(np.int32),
        0,
        height - 1,
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    candidates: list[_ScoredCircle] = []

    for threshold in thresholds:
        mask = np.where(prepared.gray <= threshold, 255, 0).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        component_count, labels, statistics, _ = cv2.connectedComponentsWithStats(mask)
        if component_count <= 1:
            continue

        probe_labels = labels[probe_y, probe_x]
        probe_labels = probe_labels[probe_labels > 0]
        if probe_labels.size == 0:
            continue
        label_ids, label_counts = np.unique(probe_labels, return_counts=True)
        component_id = int(label_ids[int(np.argmax(label_counts))])
        x, y, component_width, component_height, _ = statistics[component_id]
        if (
            x <= 1
            or y <= 1
            or x + component_width >= width - 1
            or y + component_height >= height - 1
        ):
            continue

        component = np.where(labels == component_id, 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(
            component,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE,
        )
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        if len(contour) < 20:
            continue
        (centre_x, centre_y), (axis_a, axis_b), _ = cv2.fitEllipse(contour)
        major_axis = max(axis_a, axis_b)
        minor_axis = min(axis_a, axis_b)
        if major_axis <= 0 or minor_axis / major_axis < 0.78:
            continue

        radius = 0.25 * (axis_a + axis_b)
        circle = Circle(float(centre_x), float(centre_y), float(radius))
        if (
            not 0.65 * initial.radius <= circle.radius <= 0.93 * initial.radius
            or math.hypot(circle.x - initial.x, circle.y - initial.y)
            > 0.25 * initial.radius
            or not _inside_image(circle, prepared.gray.shape)
        ):
            continue

        features = _circle_features(prepared, circle, samples=480)
        if (
            features.score >= 4.0
            and features.score >= preliminary.score - 0.45
            and features.contrast_coverage >= 0.85
            and features.edge_coverage >= 0.72
        ):
            candidates.append(features)

    if not candidates:
        return preliminary
    contour_winner = max(candidates, key=lambda item: item.score)
    # Light polish only: avoid re-entering the full anti-drift stack here.
    return _circle_features(prepared, contour_winner.circle, samples=480)

def _confidence(scored: _ScoredCircle) -> float:
    # A deliberately conservative confidence: weak partial arcs should remain
    # below the default acceptance threshold.
    # Support both bright-pin tips and bottom-up dark orifice holes.
    bright_core_term = np.clip(scored.core_contrast / 0.16, 0.0, 1.0)
    dark_core_term = np.clip((-scored.core_contrast + 0.02) / 0.12, 0.0, 1.0)
    raw = (
        0.34 * np.clip(scored.contrast / 0.18, 0.0, 1.0)
        + 0.30 * np.clip((scored.contrast_coverage - 0.28) / 0.55, 0.0, 1.0)
        + 0.28 * np.clip((scored.edge_coverage - 0.22) / 0.55, 0.0, 1.0)
        + 0.08 * np.clip(scored.edge_strength / 0.75, 0.0, 1.0)
        + 0.12 * max(float(bright_core_term), float(dark_core_term))
    )
    bright_topology_gate = np.clip((scored.core_contrast - 0.015) / 0.060, 0.0, 1.0)
    # Mildly dark / flat cores still count when the bright rim is almost complete.
    dark_topology_gate = float(
        np.clip((-scored.core_contrast + 0.025) / 0.070, 0.0, 1.0)
        * np.clip((scored.contrast_coverage - 0.70) / 0.20, 0.0, 1.0)
        * np.clip((scored.edge_coverage - 0.70) / 0.20, 0.0, 1.0)
    )
    if (
        scored.core_contrast < 0.05
        and scored.contrast_coverage >= 0.92
        and scored.edge_coverage >= 0.92
    ):
        dark_topology_gate = max(dark_topology_gate, 0.90)
    radial_topology_gate = max(float(bright_topology_gate), dark_topology_gate)
    coloured_core_gate = np.clip((scored.core_saturation - 0.12) / 0.06, 0.0, 1.0)
    strong_core_gate = np.clip((scored.core_contrast - 0.30) / 0.25, 0.0, 1.0)
    dark_orifice_gate = float(
        np.clip((-scored.core_contrast + 0.025) / 0.070, 0.0, 1.0)
        * np.clip((scored.edge_coverage - 0.70) / 0.20, 0.0, 1.0)
    )
    if (
        scored.core_contrast < 0.05
        and scored.contrast_coverage >= 0.92
        and scored.edge_coverage >= 0.92
    ):
        dark_orifice_gate = max(dark_orifice_gate, 0.90)
    appearance_gate = max(
        float(coloured_core_gate),
        float(strong_core_gate),
        dark_orifice_gate,
    )
    return float(np.clip((raw / 1.12) * radial_topology_gate * appearance_gate, 0.0, 1.0))

def _detect_nozzle_in_frame(
    image: np.ndarray,
    min_confidence: float = 0.36,
    target_radius_ratio: Optional[tuple[float, float]] = None,
) -> Detection:
    """Return the single best nozzle circle or raise ``RuntimeError``.

    ``image`` must be a BGR OpenCV image. Coordinates in the returned circle
    refer to the original image, not the internal working scale.
    """
    if image is None or image.ndim != 3 or image.shape[2] not in (3, 4):
        raise ValueError("image 必须是 OpenCV BGR/BGRA 彩色图像")
    if image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    prepared = _prepare(image)
    candidates: list[Circle] = []
    _add_high_resolution_tip_candidates(image, prepared.scale, candidates)
    _add_hough_candidates(prepared.smooth, candidates)
    _add_threshold_candidates(prepared.gray, candidates)
    candidates = _deduplicate(candidates, prepared.gray.shape)
    short_side = min(prepared.gray.shape)
    if target_radius_ratio is not None:
        low_ratio, high_ratio = target_radius_ratio
        candidates = [
            circle
            for circle in candidates
            if low_ratio * short_side <= circle.radius <= high_ratio * short_side
        ]
        tip_radius_limit = max(0.085 * short_side, high_ratio * short_side + 1.0)
    else:
        tip_radius_limit = 0.085 * short_side
    if not candidates:
        raise RuntimeError("未找到完整的圆形喷口候选（已排除贴边背景）")

    coarse = [_circle_features(prepared, circle, samples=200) for circle in candidates]
    coarse.sort(key=lambda item: item.score, reverse=True)
    bright_reference = max(1.0, float(np.percentile(prepared.gray, 92)))
    hex_faces = _find_hex_faces(prepared) if target_radius_ratio is not None else []

    # Tip-camera apertures are tiny and nested inside a larger black holder.
    # Residual glue / glare can slightly weaken ring contrast, so gates are a
    # bit softer than before; coloured or bright cores still reject glare dots.
    # Bottom-up views often show a dark orifice hole (no bright pin); keep those.
    bright_tips = [
        item
        for item in coarse
        if item.circle.radius <= tip_radius_limit
        and 0.04 <= item.core_contrast <= 0.90
        and item.core_level <= 0.80 * bright_reference
        and item.contrast >= 0.075
        and item.contrast_coverage >= 0.74
        and item.edge_coverage >= 0.72
        and (item.core_saturation >= 0.12 or item.core_contrast >= 0.18)
    ]
    dark_tips = [
        item
        for item in coarse
        if item.circle.radius <= tip_radius_limit
        and item.core_contrast < 0.04
        and item.core_level <= 0.45 * bright_reference
        and item.contrast >= 0.10
        and item.contrast_coverage >= 0.80
        and item.edge_coverage >= 0.78
    ]
    tiny_nozzles = bright_tips + [item for item in dark_tips if item not in bright_tips]
    strict_tip_support: dict[Circle, _LocalHexSupport] = {}
    if target_radius_ratio is not None:
        # Strong glare can make a few photometric samples miss a soft tip gate
        # (TEST-02 right-side glare is the frozen example).  Six-sided physical
        # containment may rescue such a candidate, but only when its circular
        # edge evidence is already substantial; hex geometry alone is never
        # allowed to turn a weak background arc into a nozzle.
        for item in coarse[:32]:
            if (
                item.circle.radius <= tip_radius_limit
                and item.score >= 3.20
                and item.contrast >= 0.065
                and item.contrast_coverage >= 0.68
                and item.edge_coverage >= 0.68
            ):
                support = _local_hex_support(prepared.gray, item.circle)
                if support is not None and support.score >= 3.40:
                    strict_tip_support[item.circle] = support
                    if item not in tiny_nozzles:
                        tiny_nozzles.append(item)
    # If the global best score is already a plausible tip, prefer it over the
    # black housing even when a couple of soft gates barely miss.
    top = coarse[0]
    if (
        not tiny_nozzles
        and top.circle.radius <= tip_radius_limit
        and top.core_level <= 0.80 * bright_reference
        and top.contrast_coverage >= 0.80
        and top.edge_coverage >= 0.78
        and (
            (top.core_contrast >= 0.04 and (top.core_saturation >= 0.10 or top.core_contrast >= 0.15))
            or (top.core_contrast < 0.04 and top.core_level <= 0.45 * bright_reference and top.contrast >= 0.10)
        )
    ):
        tiny_nozzles = [top]

    if tiny_nozzles and hex_faces:
        hex_nested_tips = [
            item
            for item in tiny_nozzles
            if _hex_nesting_score(item.circle, hex_faces) > 0.0
        ]
        base_tip_rank = lambda item: (
            item.score + 0.35 * item.edge_coverage + 0.25 * item.contrast_coverage
        )
        if (
            hex_nested_tips
            and max(map(base_tip_rank, hex_nested_tips))
            >= max(map(base_tip_rank, tiny_nozzles)) - 0.10
        ):
            tiny_nozzles = hex_nested_tips

    def _tip_rank(item: _ScoredCircle) -> float:
        nesting = _hex_nesting_score(item.circle, hex_faces) if hex_faces else 0.0
        return (
            item.score
            + 0.35 * item.edge_coverage
            + 0.25 * item.contrast_coverage
            + 0.55 * nesting
        )

    def _orifice_rank(item: _ScoredCircle) -> float:
        return item.score + 0.40 * item.edge_coverage + 0.20 * item.contrast_coverage

    def _nested_in(inner: _ScoredCircle, outer: _ScoredCircle) -> bool:
        return (
            math.hypot(
                inner.circle.x - outer.circle.x,
                inner.circle.y - outer.circle.y,
            )
            <= 0.55 * outer.circle.radius
            and inner.circle.radius <= 0.35 * outer.circle.radius
        )

    # A strong mid/large dark orifice with a luminous core (like 11(8)) must beat
    # tiny glare dots on residue. Prefer tips nested inside that cavity; only keep
    # non-nested tips when they clearly outscore the orifice. Do not wipe nested
    # tips just because some distant false tip ranked higher.
    strong_orifice = [
        item
        for item in coarse
        if item.circle.radius > tip_radius_limit
        and item.core_contrast >= 0.30
        and item.edge_coverage >= 0.72
        and item.contrast_coverage >= 0.80
        and item.contrast >= 0.10
    ]
    if tiny_nozzles and strong_orifice:
        best_orifice = max(strong_orifice, key=_orifice_rank)
        nested_tips = [
            item for item in tiny_nozzles if any(_nested_in(item, orifice) for orifice in strong_orifice)
        ]
        competitive_tips = [
            item for item in tiny_nozzles if item.score >= best_orifice.score + 0.45
        ]
        tiny_nozzles = nested_tips or competitive_tips

    # Reject isolated false tips (wires / background arcs) when a much stronger
    # mid/large cavity exists elsewhere; keep tips nested in that cavity.
    if tiny_nozzles:
        competing_large = [
            item
            for item in coarse
            if item.circle.radius > tip_radius_limit
            and item.core_contrast > 0.0
            and item.contrast_coverage >= 0.75
            and item.edge_coverage >= 0.55
        ]
        if competing_large:
            best_large = max(competing_large, key=_orifice_rank)
            tiny_nozzles = [
                item
                for item in tiny_nozzles
                if _nested_in(item, best_large) or item.score >= best_large.score - 0.60
            ]

    if target_radius_ratio is not None:
        if not tiny_nozzles:
            raise _HexLockError("未找到六边形金属面内的喷口候选")
        geometrically_locked: list[_ScoredCircle] = []
        for item in sorted(tiny_nozzles, key=_tip_rank, reverse=True)[:24]:
            support = strict_tip_support.get(item.circle)
            if support is None:
                support = _local_hex_support(prepared.gray, item.circle)
            if support is None or support.score < 3.40:
                continue
            strict_tip_support[item.circle] = support
            geometrically_locked.append(item)
        if not geometrically_locked:
            raise _HexLockError(
                "圆形候选没有完整包含在六边形金属面内；为防止跳到背景，未输出"
            )
        tiny_nozzles = geometrically_locked

    if tiny_nozzles:
        initial_winner = max(tiny_nozzles, key=_tip_rank)
    else:
        # Choose the best-scoring member of the concentric family. Blindly
        # preferring the outermost circle inflated radii and caused centre drift.
        large = [
            item
            for item in coarse
            if item.circle.radius > 0.065 * short_side and item.core_contrast > 0.0
        ]
        if not large:
            large = [item for item in coarse if item.core_contrast > -0.05] or coarse
        anchor = max(
            large,
            key=lambda item: item.score + 0.40 * item.edge_coverage + 0.20 * item.contrast_coverage,
        )

        # Only keep expansions that actually improve the photometric score.
        if anchor.core_contrast > 0.45 and anchor.circle.radius < 0.40 * short_side:
            for factor in (1.06, 1.12, 1.18):
                expanded = Circle(
                    anchor.circle.x,
                    anchor.circle.y,
                    anchor.circle.radius * factor,
                )
                if _inside_image(expanded, prepared.gray.shape):
                    scored = _circle_features(prepared, expanded, samples=240)
                    if scored.score >= anchor.score - 0.35 and scored.edge_coverage >= 0.55:
                        large.append(scored)
        family = [
            item
            for item in large
            if item.score >= anchor.score - 1.05
            and math.hypot(
                item.circle.x - anchor.circle.x,
                item.circle.y - anchor.circle.y,
            )
            <= 0.30 * max(item.circle.radius, anchor.circle.radius)
        ] or [anchor]
        initial_winner = max(
            family,
            key=lambda item: item.score + 0.45 * item.edge_coverage + 0.15 * item.contrast_coverage,
        )

    if tiny_nozzles:
        winner = _refine_circle(prepared, initial_winner.circle, max_radius_change=0.12)
        tip_core = _bright_core_seed(prepared, winner.circle)
        if math.hypot(tip_core.x - winner.circle.x, tip_core.y - winner.circle.y) <= 0.35 * winner.circle.radius:
            tip_peak = _radius_from_outer_peaks(
                prepared,
                Circle(tip_core.x, tip_core.y, winner.circle.radius),
                factor_low=0.70,
                factor_high=1.35,
                prefer_percentile=55,
            )
            tip_radius = tip_peak.radius if tip_peak is not None else winner.circle.radius
            tip_trial = Circle(tip_core.x, tip_core.y, tip_radius)
            tip_score = _circle_features(prepared, tip_trial, samples=480)
            if tip_score.score + 0.25 * tip_score.edge_coverage >= winner.score + 0.25 * winner.edge_coverage - 0.15:
                winner = tip_score
        else:
            tip_outer = _radius_from_outer_peaks(prepared, winner.circle)
            if tip_outer is not None:
                tip_score = _circle_features(prepared, tip_outer, samples=480)
                if tip_score.score + 0.3 * tip_score.edge_coverage >= winner.score + 0.3 * winner.edge_coverage - 0.08:
                    winner = tip_score
        snapped = _refine_small_radius(prepared, winner.circle)
        if _rank_scored(snapped) >= _rank_scored(winner) - 0.35:
            winner = snapped
        winner = _polish_small_dark_orifice(prepared, winner)
        winner = _fit_small_visible_rim(prepared, winner)
        if target_radius_ratio is not None:
            final_support = _local_hex_support(prepared.gray, winner.circle)
            if final_support is None or final_support.score < 3.40:
                raise _HexLockError(
                    "圆心/半径精修后离开六边形物理约束；为防止反光拉偏，未输出"
                )
    else:
        winner = _fit_outer_edge(prepared, initial_winner.circle)
        # Prefer a dark-cavity ellipse fit for geometry.  The luminous pin is a
        # useful object check but is often above the true outer-rim centre under
        # perspective, so it must not fully override the rim fit.
        blob = _fit_dark_blob_circle(prepared, winner.circle)
        if blob is not None:
            core = _bright_core_seed(prepared, winner.circle)
            core_dist = math.hypot(core.x - blob.x, core.y - blob.y)
            if core_dist <= 0.28 * max(blob.radius, winner.circle.radius):
                # Soft blend: mostly blob centre, slight pull toward the pin.
                blend = 0.18 if (
                    initial_winner.core_contrast >= 0.20 or initial_winner.core_saturation >= 0.14
                ) else 0.0
                cx = (1.0 - blend) * blob.x + blend * core.x
                cy = (1.0 - blend) * blob.y + blend * core.y
                best_forced: Optional[_ScoredCircle] = None
                for scale in (0.96, 0.98, 1.00, 1.02, 1.04, 1.06):
                    trial = Circle(cx, cy, blob.radius * scale)
                    if not _inside_image(trial, prepared.gray.shape, margin_ratio=0.008):
                        continue
                    scored = _circle_features(prepared, trial, samples=400)
                    rank = _rank_scored(scored)
                    if best_forced is None or rank > _rank_scored(best_forced):
                        best_forced = scored
                if best_forced is not None and best_forced.edge_coverage >= 0.55:
                    # Accept blob-based geometry unless it is clearly worse.
                    if _rank_scored(best_forced) >= _rank_scored(winner) - 0.90:
                        winner = best_forced
                        polished = _fit_rim_points(prepared, winner.circle)
                        if polished is not None:
                            polished_score = _circle_features(prepared, polished, samples=480)
                            if _rank_scored(polished_score) >= _rank_scored(winner) - 0.35:
                                winner = polished_score
        # V2 dark-cavity modules: glare consensus then soft-shadow shrink-in.
        baseline = winner
        corrected = _correct_large_outer_geometry(prepared, winner)
        if _rank_scored(corrected) >= _rank_scored(baseline) - 0.90:
            winner = corrected
        aperture = _fit_dark_aperture_edge(prepared, winner)
        if _rank_scored(aperture) >= _rank_scored(winner) - 0.90:
            winner = aperture
    confidence = _confidence(winner)
    # In the real printer feed, the central pin can look grey, black or red as
    # illumination changes.  Requiring a particular core colour rejected clear
    # apertures even when the physical rim was complete.  Promote only a very
    # strong 360-degree small-rim fit; partial arcs and large housing circles do
    # not qualify for this evidence path.
    if (
        tiny_nozzles
        and winner.circle.radius <= tip_radius_limit
        and winner.score >= 5.50
        and winner.contrast >= 0.12
        and winner.contrast_coverage >= 0.96
        and winner.edge_coverage >= 0.96
    ):
        rim_strength = float(np.clip((winner.contrast - 0.12) / 0.24, 0.0, 1.0))
        confidence = max(confidence, 0.58 + 0.32 * rim_strength)
    if confidence < min_confidence:
        raise RuntimeError(
            f"没有足够可靠的喷口圆（confidence={confidence:.3f}，"
            f"阈值={min_confidence:.3f}）；为避免误检，未绘制结果"
        )

    inverse_scale = 1.0 / prepared.scale
    result_circle = Circle(
        winner.circle.x * inverse_scale,
        winner.circle.y * inverse_scale,
        winner.circle.radius * inverse_scale,
    )
    return Detection(circle=result_circle, confidence=confidence, score=winner.score)

def _center_roi_bounds(
    shape: tuple[int, int],
    width_fraction: float,
    height_fraction: float,
) -> tuple[int, int, int, int]:
    """Return integer bounds for a centred fractional ROI."""
    height, width = shape
    if not 0.10 <= width_fraction <= 1.0:
        raise ValueError("roi_width_fraction 必须在 0.10 到 1.0 之间")
    if not 0.10 <= height_fraction <= 1.0:
        raise ValueError("roi_height_fraction 必须在 0.10 到 1.0 之间")
    roi_width = max(1, int(round(width * width_fraction)))
    roi_height = max(1, int(round(height * height_fraction)))
    x0 = (width - roi_width) // 2
    y0 = (height - roi_height) // 2
    return x0, y0, x0 + roi_width, y0 + roi_height

def _metal_hex_support(
    prepared: _PreparedImage,
    circle: Circle,
) -> tuple[float, float, float]:
    """Score the reflective metal annulus around a small aperture.

    A real nozzle hole is surrounded by a textured six-sided metal face.  Its
    luminance varies strongly because of machining marks and specular glare,
    while unrelated circular arcs usually lack that concentric textured band.
    The score deliberately does not require a black outer housing.
    """
    height, width = prepared.gray.shape
    radius = circle.radius
    margin = 4.25 * radius
    x0 = max(0, int(math.floor(circle.x - margin)))
    x1 = min(width, int(math.ceil(circle.x + margin + 1)))
    y0 = max(0, int(math.floor(circle.y - margin)))
    y1 = min(height, int(math.ceil(circle.y + margin + 1)))
    yy, xx = np.ogrid[y0:y1, x0:x1]
    distance = np.sqrt((xx - circle.x) ** 2 + (yy - circle.y) ** 2)
    patch = prepared.gray[y0:y1, x0:x1]
    metal_mask = (distance >= 1.20 * radius) & (distance <= 3.15 * radius)
    outer_mask = (distance >= 3.30 * radius) & (distance <= 4.20 * radius)
    metal = patch[metal_mask]
    outer = patch[outer_mask]
    if metal.size < 40 or outer.size < 24:
        return 0.0, 0.0, 0.0

    texture = float(np.std(metal) / prepared.dynamic_range)
    outer_drop = float((np.mean(metal) - np.mean(outer)) / prepared.dynamic_range)
    texture_term = float(np.clip((texture - 0.24) / 0.18, 0.0, 1.0))
    boundary_term = float(np.clip((outer_drop - 0.02) / 0.20, 0.0, 1.0))
    support = 2.00 * texture_term + 0.80 * boundary_term
    return support, texture, outer_drop

def _prelocate_hex_aperture_y(
    image: np.ndarray,
    x0: int,
    x1: int,
) -> Optional[float]:
    """Locate the aperture height from circle-plus-metal evidence at low cost."""
    strip = image[:, x0:x1]
    if strip.shape[2] == 4:
        strip = cv2.cvtColor(strip, cv2.COLOR_BGRA2BGR)
    # Half scale is intentionally fixed for camera-sized frames: the frozen
    # aperture/hex thresholds below were calibrated in that normalized space.
    scale = min(0.50, 520.0 / max(strip.shape[:2]))
    if scale < 0.999:
        thumbnail = cv2.resize(
            strip,
            (max(1, round(strip.shape[1] * scale)), max(1, round(strip.shape[0] * scale))),
            interpolation=cv2.INTER_AREA,
        )
    else:
        thumbnail = strip
        scale = 1.0

    prepared = _prepare(thumbnail, max_working_side=600)
    candidates: list[Circle] = []
    _add_hough_candidates(prepared.smooth, candidates)
    _add_threshold_candidates(prepared.gray, candidates)
    candidates = _deduplicate(candidates, prepared.gray.shape)
    short_side = min(prepared.gray.shape)
    candidates = [
        circle
        for circle in candidates
        if 0.0175 * short_side <= circle.radius <= 0.052 * short_side
    ]
    if not candidates:
        return None

    bright_reference = max(1.0, float(np.percentile(prepared.gray, 92)))
    hex_faces = _find_hex_faces(prepared)
    ranked: list[tuple[float, float, float, _ScoredCircle]] = []
    for circle in candidates:
        features = _circle_features(prepared, circle, samples=160)
        if (
            features.core_level > 0.90 * bright_reference
            or features.contrast < 0.015
            or features.contrast_coverage < 0.50
            or features.edge_coverage < 0.60
        ):
            continue
        metal_support, texture, _ = _metal_hex_support(prepared, circle)
        if metal_support < 0.25 or texture < 0.16:
            continue
        nesting = _hex_nesting_score(circle, hex_faces)
        rank = features.score + metal_support + 0.70 * nesting
        ranked.append((rank, metal_support, nesting, features))
    if not ranked:
        return None

    nested = [item for item in ranked if item[2] > 0.0]
    if nested and max(item[0] for item in nested) >= max(item[0] for item in ranked) - 0.10:
        ranked = nested

    rank, metal_support, _, winner = max(ranked, key=lambda item: item[0])
    if rank < 4.20 or metal_support < 0.25:
        return None
    return float(winner.circle.y / scale)

def _camera_roi_bounds(
    image: np.ndarray,
    width_fraction: float,
    height_fraction: float,
    adaptive_vertical: bool,
) -> tuple[int, int, int, int]:
    """Choose one compact camera ROI while keeping corner clutter excluded.

    The expensive circle search always sees exactly ``width_fraction`` by
    ``height_fraction`` of the frame.  A printer head can move vertically, so
    live mode cheaply compares upper/centre/lower placements of that same-size
    window. A dominant dark housing is optional evidence; otherwise a small
    circle nested inside a reflective polygonal metal face selects the height.
    Horizontal placement stays centred, excluding side-wall reflections.
    """
    x0, y0, x1, y1 = _center_roi_bounds(
        image.shape[:2],
        width_fraction,
        height_fraction,
    )
    if not adaptive_vertical or height_fraction >= 0.999:
        return x0, y0, x1, y1

    height = image.shape[0]
    roi_height = y1 - y0
    starts = sorted({0, y0, height - roi_height})
    if len(starts) == 1:
        return x0, y0, x1, y1

    if image.shape[2] == 4:
        gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    strip = gray[:, x0:x1]
    cutoff = float(np.clip(np.percentile(strip, 40), 45.0, 100.0))

    def hardware_darkness(start: int) -> float:
        patch = strip[start : start + roi_height]
        dark_fraction = float(np.mean(patch < cutoff))
        dark_depth = float(
            np.mean(np.clip(cutoff - patch.astype(np.float32), 0.0, None)) / cutoff
        )
        return 0.72 * dark_fraction + 0.28 * dark_depth

    darkness = [(hardware_darkness(start), start) for start in starts]
    darkness.sort(reverse=True)
    # A clearly dominant dark band is a useful optional housing cue and is
    # especially stable under global exposure changes.  When it is absent or
    # ambiguous, locate the circle inside the reflective hex instead.
    if darkness[0][0] >= 0.42 and darkness[0][0] - darkness[1][0] >= 0.045:
        best_y = darkness[0][1]
        return x0, best_y, x1, best_y + roi_height

    aperture_y = _prelocate_hex_aperture_y(image, x0, x1)
    if aperture_y is not None:
        best_y = min(
            starts,
            key=lambda start: abs(aperture_y - (start + 0.5 * roi_height)),
        )
        return x0, best_y, x1, best_y + roi_height

    best_y = darkness[0][1]
    return x0, best_y, x1, best_y + roi_height

def detect_nozzle(
    image: np.ndarray,
    min_confidence: float = 0.36,
    roi_width_fraction: float = 0.50,
    roi_height_fraction: float = 0.50,
    fast_camera_mode: bool = True,
    fallback_general: bool = True,
    adaptive_vertical_roi: bool = True,
    min_radius_px: Optional[float] = None,
    max_radius_px: Optional[float] = None,
    search_bounds: Optional[tuple[float, float, float, float]] = None,
    expected_center: Optional[tuple[float, float]] = None,
    search_circle: Optional[tuple[float, float, float]] = None,
    *,
    search_delta: Optional[tuple[float, float]] = None,
    radius_range: Optional[tuple[float, float]] = None,
    mode: Optional[str] = None,
) -> Detection:
    """Detect one nozzle in an already-captured image (no camera I/O).

    Preferred call knobs: ``expected_center``, ``search_delta`` (half-width /
    half-height), ``radius_range``, ``min_confidence``, ``mode`` in
    ``acquire|track|reacquire``.

    Deprecated transition knobs (still accepted): ``search_bounds``,
    ``search_circle``, ``roi_width_fraction``, ``roi_height_fraction``,
    ``fast_camera_mode``, ``fallback_general``, ``adaptive_vertical_roi``,
    ``min_radius_px`` / ``max_radius_px``. Prefer ``search_delta`` /
    ``radius_range`` / ``CameraProfile`` instead.
    """
    if image is None or image.ndim != 3 or image.shape[2] not in (3, 4):
        raise ValueError("image 必须是 OpenCV BGR/BGRA 彩色图像")

    if radius_range is not None:
        if len(radius_range) != 2:
            raise ValueError("radius_range 必须是 (rmin, rmax)")
        if min_radius_px is None:
            min_radius_px = float(radius_range[0])
        if max_radius_px is None:
            max_radius_px = float(radius_range[1])

    for name, value in (("min_radius_px", min_radius_px), ("max_radius_px", max_radius_px)):
        if value is not None and (not math.isfinite(value) or value <= 0.0):
            raise ValueError(f"{name} 必须是正数")
    if min_radius_px is not None and max_radius_px is not None and min_radius_px > max_radius_px:
        raise ValueError("min_radius_px 不能大于 max_radius_px")
    explicit_expected_center = expected_center is not None
    detect_mode = _normalize_detect_mode(mode, explicit_expected_center)
    if detect_mode == "track" and not explicit_expected_center:
        raise ValueError("mode=track 需要 expected_center")
    if expected_center is not None and (
        len(expected_center) != 2
        or not all(math.isfinite(float(value)) for value in expected_center)
    ):
        raise ValueError("expected_center 必须是两个有限像素坐标")
    if search_delta is not None and search_bounds is None and expected_center is not None:
        profile = CameraProfile.from_image(image)
        delta = search_delta
        if delta is None:
            delta = profile.default_search_delta
        if detect_mode == "reacquire":
            delta = (float(delta[0]) * 1.5, float(delta[1]) * 1.5)
        search_bounds = _search_bounds_from_delta(
            (float(expected_center[0]), float(expected_center[1])),
            (float(delta[0]), float(delta[1])),
        )
    if search_circle is not None:
        if (
            len(search_circle) != 3
            or not all(math.isfinite(float(value)) for value in search_circle)
            or float(search_circle[2]) <= 0.0
        ):
            raise ValueError("search_circle 必须是有限数值 (cx, cy, radius)，且半径为正")
        search_circle = tuple(map(float, search_circle))

    # acquire/reacquire: strict face proof; track: lighter local proof
    require_face_support = detect_mode != "track"

    hard_bounds: Optional[tuple[int, int, int, int]] = None
    if search_bounds is not None:
        if len(search_bounds) != 4 or not all(math.isfinite(float(v)) for v in search_bounds):
            raise ValueError("search_bounds 必须是有限数值 (x0, y0, x1, y1)")
        raw_x0, raw_y0, raw_x1, raw_y1 = map(float, search_bounds)
        if raw_x1 <= raw_x0 or raw_y1 <= raw_y0:
            raise ValueError("search_bounds 必须满足 x1>x0 且 y1>y0")
        height, width = image.shape[:2]
        hard_x0 = max(0, min(width - 1, int(math.floor(raw_x0))))
        hard_y0 = max(0, min(height - 1, int(math.floor(raw_y0))))
        hard_x1 = max(hard_x0 + 1, min(width, int(math.ceil(raw_x1))))
        hard_y1 = max(hard_y0 + 1, min(height, int(math.ceil(raw_y1))))
        if hard_x1 - hard_x0 < 32 or hard_y1 - hard_y0 < 32:
            raise ValueError("search_bounds 与图像相交区域过小（宽高至少 32px）")
        hard_bounds = (hard_x0, hard_y0, hard_x1, hard_y1)

    # LED fill changes the visible target from a blurred dark aperture into a
    # sharp silver rim.  Detect it before darkness-based vertical ROI selection
    # so a lit housing cannot send the search to the wrong upper/lower band.
    centre_x0, centre_y0, centre_x1, centre_y1 = _center_roi_bounds(
        image.shape[:2],
        roi_width_fraction,
        roi_height_fraction,
    )

    # Work out the physical radius gate before the LED branch as well.  R12
    # previously calculated these automatic bounds only after LED detection,
    # so a large glare ring could bypass the camera calibration entirely.
    calibration_short_side = float(
        min(
            image.shape[1] * roi_width_fraction,
            image.shape[0] * roi_height_fraction,
        )
    )
    automatic_min = 0.038 * calibration_short_side if fast_camera_mode else None
    automatic_max = 0.088 * calibration_short_side if fast_camera_mode else None

    # The deployed printer stream is fixed at 1280x720.  Its verified nozzle
    # outer edge is 20.9--24.9 px in the supplied live frames.  Keep a generous
    # 18--32 px physical band, and keep acquisition inside the agreed middle
    # half of the frame.  This rejects the ~77 px glare ring in the reported
    # failure and background printer parts; general mode or explicit bounds
    # remain available for a different camera/profile.
    fixed_1280_profile = bool(
        fast_camera_mode
        and hard_bounds is None
        and image.shape[1] == 1280
        and image.shape[0] == 720
    )
    fixed_640_profile = bool(
        fast_camera_mode
        and image.shape[1] == 640
        and image.shape[0] == 480
    )
    if fixed_1280_profile:
        automatic_min = 18.0
        automatic_max = 32.0
    elif fixed_640_profile:
        # 默认先宽（10--17）保证多数机能首检；确认R后再收窄。
        automatic_min = 10.0
        automatic_max = 17.0

    active_min_radius = (
        float(min_radius_px) if min_radius_px is not None else automatic_min
    )
    active_max_radius = (
        float(max_radius_px) if max_radius_px is not None else automatic_max
    )
    if fixed_640_profile:
        # Explicit caller band (device-calibrated) is authoritative; only soft
        # safety clamps apply. Default profile uses 10--17 px.
        if min_radius_px is not None and max_radius_px is not None:
            active_min_radius = max(5.0, min(60.0, float(active_min_radius)))
            active_max_radius = max(5.0, min(60.0, float(active_max_radius)))
        else:
            active_min_radius = max(10.0, float(active_min_radius))
            active_max_radius = min(17.0, float(active_max_radius))
        if active_min_radius > active_max_radius:
            raise RuntimeError("640x480 喷嘴跟踪半径范围不相交")

    # Preserve the proven multi-resolution LED behaviour.  Automatic pixel
    # calibration is enforced in the new fixed 1280x720 profile; other image
    # sizes receive only bounds explicitly supplied by the caller, exactly as
    # in R12.  This avoids over-constraining the separate cyan-annulus model.
    calibrated_camera_profile = fixed_1280_profile or fixed_640_profile
    led_min_radius = active_min_radius if calibrated_camera_profile else min_radius_px
    led_max_radius = active_max_radius if calibrated_camera_profile else max_radius_px

    fixed_640_allowed_x: Optional[tuple[float, float]] = None
    fixed_640_allowed_y: Optional[tuple[float, float]] = None
    if fixed_640_profile:
        fixed_640_allowed_x = (0.25 * image.shape[1], 0.75 * image.shape[1])
        fixed_640_allowed_y = (0.20 * image.shape[0], 0.80 * image.shape[0])
        if hard_bounds is not None:
            fixed_640_allowed_x = (
                max(fixed_640_allowed_x[0], float(hard_bounds[0])),
                min(fixed_640_allowed_x[1], float(hard_bounds[2])),
            )
            fixed_640_allowed_y = (
                max(fixed_640_allowed_y[0], float(hard_bounds[1])),
                min(fixed_640_allowed_y[1], float(hard_bounds[3])),
            )
        if (
            fixed_640_allowed_x[1] <= fixed_640_allowed_x[0]
            or fixed_640_allowed_y[1] <= fixed_640_allowed_y[0]
        ):
            raise RuntimeError("指定搜索区域不在当前 640x480 喷嘴活动区内")
        assert led_min_radius is not None and led_max_radius is not None
        direct_rim = _detect_fixed_640_center_rim(
            image,
            min_confidence=min_confidence,
            allowed_x=fixed_640_allowed_x,
            allowed_y=fixed_640_allowed_y,
            min_radius_px=float(led_min_radius),
            max_radius_px=float(led_max_radius),
            # A hard rectangle limits where a centre may occur; its geometric
            # centre is not automatically the expected nozzle position.
            # Treating it as such rejected valid circles near a ROI edge.
            expected_center=expected_center,
            allowed_region_circle=search_circle,
            # mode=track → lighter local proof while following; acquire /
            # reacquire keep full metal-face proof even if expected is set.
            # mode=track → lighter local proof while following; acquire /
            # reacquire keep full metal-face proof even if expected is set.
            require_face_support=require_face_support,
        )
        if direct_rim is not None and _circle_fits_inside_region(
            direct_rim.circle,
            search_circle,
        ):
            return direct_rim

    if _has_led_fill_lighting(image):
        if fixed_640_profile:
            # The selected nozzle occupies the central/right carriage bay.
            # Recognition starts after the selected nozzle is moved into the
            # calibrated centre work band; the second nozzle stays outside it.
            assert fixed_640_allowed_x is not None
            assert fixed_640_allowed_y is not None
            allowed_x = fixed_640_allowed_x
            allowed_y = fixed_640_allowed_y
        elif fixed_1280_profile:
            allowed_x = (float(centre_x0), float(centre_x1))
            allowed_y = (float(centre_y0), float(centre_y1))
        else:
            allowed_x = (
                (0.0, float(image.shape[1]))
                if adaptive_vertical_roi
                else (float(centre_x0), float(centre_x1))
            )
            allowed_y = (
                (0.0, float(image.shape[0]))
                if adaptive_vertical_roi
                else (float(centre_y0), float(centre_y1))
            )
        if hard_bounds is not None:
            allowed_x = (
                max(allowed_x[0], float(hard_bounds[0])),
                min(allowed_x[1], float(hard_bounds[2])),
            )
            allowed_y = (
                max(allowed_y[0], float(hard_bounds[1])),
                min(allowed_y[1], float(hard_bounds[3])),
            )
            if allowed_x[1] <= allowed_x[0] or allowed_y[1] <= allowed_y[0]:
                raise RuntimeError("指定搜索区域不在当前摄像头有效 ROI 内")
        if (
            fixed_640_profile
            and hard_bounds is not None
            and led_min_radius is not None
            and led_max_radius is not None
        ):
            # A133 fast path: same R16 face+rim proof, but prepare only the
            # physical context surrounding the non-expanding 80x72 gate.
            led_detection = _detect_led_bright_rim_in_fixed_roi(
                image,
                min_confidence=min_confidence,
                allowed_x=allowed_x,
                allowed_y=allowed_y,
                min_radius_px=float(led_min_radius),
                max_radius_px=float(led_max_radius),
                allowed_region_circle=search_circle,
            )
        else:
            led_detection = _detect_led_bright_rim(
                image,
                min_confidence=min_confidence,
                allowed_x=allowed_x,
                allowed_y=allowed_y,
                min_radius_px=led_min_radius,
                max_radius_px=led_max_radius,
            )
        if led_detection is not None and _circle_fits_inside_region(
            led_detection.circle,
            search_circle,
        ):
            return led_detection
        if calibrated_camera_profile:
            # In the calibrated live stream, ordinary circle fallback is not a
            # safe substitute for the LED/hex proof: it can lock to a distant
            # black/white printer part of similar radius.  A missing strict
            # nozzle proof must be reported as a miss, never as a background
            # circle.  Continuous callers may hold their previous tracker lock.
            raise _HexLockError(
                "固定真机补光帧未通过喷嘴外圆/六边形物理锁；已禁止回退到第二喷嘴或背景圆"
            )

    if fixed_640_profile:
        x0 = int(round(0.25 * image.shape[1]))
        x1 = int(round(0.75 * image.shape[1]))
        y0 = int(round(0.20 * image.shape[0]))
        y1 = int(round(0.80 * image.shape[0]))
    elif hard_bounds is None:
        x0, y0, x1, y1 = _camera_roi_bounds(
            image,
            roi_width_fraction,
            roi_height_fraction,
            adaptive_vertical_roi,
        )
    else:
        x0, y0, x1, y1 = hard_bounds
    roi = image[y0:y1, x0:x1]
    roi_short_side = float(min(roi.shape[:2]))

    # Radius calibration follows frame resolution, not the requested crop size.
    # Otherwise tightening a positional ROI would accidentally shrink the
    # allowed nozzle radius and reject the same physical target.
    radius_range = None
    if active_min_radius is not None or active_max_radius is not None:
        low = active_min_radius if active_min_radius is not None else max(1.0, 0.004 * roi_short_side)
        high = active_max_radius if active_max_radius is not None else 0.49 * roi_short_side
        if low > high:
            raise ValueError(
                f"半径范围无效：{low:.2f}px > {high:.2f}px（当前 ROI）"
            )
        radius_range = (low / roi_short_side, high / roi_short_side)
    try:
        local = _detect_nozzle_in_frame(
            roi,
            min_confidence=min_confidence,
            target_radius_ratio=radius_range,
        )
    except _HexLockError:
        # The calibrated camera branch saw circle-like evidence but it was not
        # physically nested in the nozzle's six-sided metal face.  General-mode
        # fallback would undo the anti-jump guarantee and can select a housing
        # or background circle, so this safety rejection is final.
        raise
    except RuntimeError:
        if not (fast_camera_mode and fallback_general):
            raise
        local = _detect_nozzle_in_frame(
            roi,
            min_confidence=min_confidence,
            target_radius_ratio=None,
        )

    if active_min_radius is not None and local.circle.radius < active_min_radius:
        raise RuntimeError(
            f"候选圆半径 {local.circle.radius:.2f}px 小于允许下限 "
            f"{active_min_radius:.2f}px"
        )
    if active_max_radius is not None and local.circle.radius > active_max_radius:
        raise RuntimeError(
            f"候选圆半径 {local.circle.radius:.2f}px 大于允许上限 "
            f"{active_max_radius:.2f}px"
        )

    circle = Circle(
        local.circle.x + x0,
        local.circle.y + y0,
        local.circle.radius,
    )
    if not _circle_fits_inside_region(circle, search_circle):
        raise RuntimeError("候选喷嘴外圆未完整落入当前圆形识别区域")
    return Detection(circle=circle, confidence=local.confidence, score=local.score)

class NozzleTracker:
    """Stateful hard lock for a continuous printer-camera stream.

    The first fixed-640 frame uses the configured centre hard region.  Once
    locked, the same-size region follows the previous nozzle; an explicit
    expected centre moves it immediately and remains an absolute hard gate.
    Legacy callers may opt into relocation after a bounded 3--6-sided
    metal-face proof; ScreenQML disables relocation and treats the gate as
    absolute.  Call :meth:`reset` when a new sequence should
    discard its previous motion state.
    """

    def __init__(
        self,
        *,
        min_confidence: float = 0.36,
        roi_width_fraction: float = 0.50,
        roi_height_fraction: float = 0.50,
        fast_camera_mode: bool = True,
        fallback_general: bool = True,
        adaptive_vertical_roi: bool = True,
        min_radius_px: Optional[float] = None,
        max_radius_px: Optional[float] = None,
        search_radius_scale: float = 6.0,
        reacquire_frames: int = 3,
        hold_missing_frames: int = 2,
        strict_position_lock: bool = True,
        position_window_width_px: Optional[float] = None,
        position_window_height_px: Optional[float] = None,
        allow_fixed640_relocation: bool = True,
    ) -> None:
        if search_radius_scale < 5.0:
            raise ValueError("search_radius_scale 不能小于 5.0，否则可能裁掉六边形")
        if reacquire_frames < 2:
            raise ValueError("reacquire_frames 至少为 2")
        if hold_missing_frames < 0:
            raise ValueError("hold_missing_frames 不能为负数")
        for name, value in (
            ("position_window_width_px", position_window_width_px),
            ("position_window_height_px", position_window_height_px),
        ):
            if value is not None and (not math.isfinite(value) or value < 32.0):
                raise ValueError(f"{name} 必须为空或不小于 32px")
        self._options = {
            "min_confidence": min_confidence,
            "roi_width_fraction": roi_width_fraction,
            "roi_height_fraction": roi_height_fraction,
            "fast_camera_mode": fast_camera_mode,
            "fallback_general": fallback_general,
            "adaptive_vertical_roi": adaptive_vertical_roi,
        }
        self._configured_min_radius = min_radius_px
        self._configured_max_radius = max_radius_px
        self.search_radius_scale = float(search_radius_scale)
        self.reacquire_frames = int(reacquire_frames)
        self.hold_missing_frames = int(hold_missing_frames)
        self.strict_position_lock = bool(strict_position_lock)
        # UI/server mode can make the supplied 80x72 guide/follow window an
        # absolute machine-position gate.  In that mode a miss must never
        # escape to a metal face or a background circle elsewhere in frame.
        self.allow_fixed640_relocation = bool(allow_fixed640_relocation)
        self.position_window_width_px = (
            None
            if position_window_width_px is None
            else float(position_window_width_px)
        )
        self.position_window_height_px = (
            None
            if position_window_height_px is None
            else float(position_window_height_px)
        )
        self._stable: Optional[Detection] = None
        self._pending: Optional[Detection] = None
        self._pending_count = 0
        self._missing_count = 0
        # A carriage command may complete before the snapshot endpoint exposes
        # the new frame.  If an authoritative expected position misses, keep
        # that same hard gate briefly, but never continue predicting from the
        # stale visual circle.  The bounded latch also prevents a bad expected
        # value from trapping a long-running session forever.
        self._expected_recovery_center: Optional[tuple[float, float]] = None
        self._expected_recovery_left = 0
        self._expected_recovery_frames = max(2, self.reacquire_frames)
        self._history: deque[Detection] = deque(maxlen=3)
        self.last_status = "unlocked"
        self.last_follow_roi: Optional[tuple[float, float, float, float]] = None
        self.last_allowed_roi: Optional[tuple[float, float, float, float]] = None
        self.last_allowed_circle: Optional[tuple[float, float, float]] = None
        self.last_reject_reason = ""

    @property
    def locked(self) -> bool:
        return self._stable is not None

    @property
    def detection(self) -> Optional[Detection]:
        return self._stable

    def reset(self) -> None:
        self._stable = None
        self._pending = None
        self._pending_count = 0
        self._missing_count = 0
        self._expected_recovery_center = None
        self._expected_recovery_left = 0
        self._history.clear()
        self.last_status = "unlocked"
        self.last_follow_roi = None
        self.last_allowed_roi = None
        self.last_allowed_circle = None
        self.last_reject_reason = ""

    def _invalidate_visual_lock(self) -> None:
        """Drop stale circle motion state without widening the active gate."""
        self._stable = None
        self._pending = None
        self._pending_count = 0
        self._history.clear()

    def _clear_expected_recovery(self) -> None:
        self._expected_recovery_center = None
        self._expected_recovery_left = 0

    @staticmethod
    def _intersect_bounds(
        left: tuple[float, float, float, float],
        right: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        bounds = (
            max(left[0], right[0]),
            max(left[1], right[1]),
            min(left[2], right[2]),
            min(left[3], right[3]),
        )
        if bounds[2] - bounds[0] < 32.0 or bounds[3] - bounds[1] < 32.0:
            raise RuntimeError("位置搜索范围与 640x480 主喷嘴活动区不相交")
        return bounds

    def _strict_position_bounds(
        self,
        image: np.ndarray,
        expected_center: Optional[tuple[float, float]],
    ) -> Optional[tuple[float, float, float, float]]:
        """Return the hard per-frame position gate for the fixed camera.

        Initial acquisition is limited to the configured centre ROI.  Once a
        valid circle exists, the same gate follows the predicted circle.  An
        explicit machine/UI position takes priority and moves the gate there.
        The gate itself is never widened.  With no explicit expected centre,
        R16 may move the same gate only after an independent physical-face
        proof in the calibrated primary-nozzle work area.
        """
        if (
            not self.strict_position_lock
            or image.shape[:2] != (480, 640)
        ):
            return None

        physical = (160.0, 96.0, 480.0, 384.0)
        predicted = self._predicted_center()
        if expected_center is not None:
            centre_x, centre_y = map(float, expected_center)
        elif predicted is not None:
            centre_x, centre_y = predicted
        else:
            centre_x = 0.5 * image.shape[1]
            centre_y = 0.5 * image.shape[0]

        # The calibrated fixed-640 position gate is exactly 80x72.  This is
        # used unchanged for initial acquisition, explicit machine positions,
        # and visual follow unless the caller explicitly supplies another size.
        default_width = 80.0
        default_height = 72.0

        width = (
            default_width
            if self.position_window_width_px is None
            else self.position_window_width_px
        )
        height = (
            default_height
            if self.position_window_height_px is None
            else self.position_window_height_px
        )
        width = min(width, physical[2] - physical[0])
        height = min(height, physical[3] - physical[1])
        if expected_center is not None and not (
            physical[0] <= centre_x <= physical[2]
            and physical[1] <= centre_y <= physical[3]
        ):
            raise RuntimeError("给定喷嘴位置不在 640x480 主喷嘴活动区内")
        # The circular gate must follow the estimated nozzle itself.  Earlier
        # revisions forced the whole rectangle inside the narrower physical
        # centre range; near a corner that shifted a valid expected point by
        # more than 50 px and could exclude the actual rim.  Keep the supplied
        # centre exact and constrain only against the image boundary.  The
        # global locator is still limited to ``physical`` and every final rim
        # remains inside this same-size circular gate.
        half_width = 0.5 * width
        half_height = 0.5 * height
        if expected_center is None and predicted is not None:
            centre_x = float(np.clip(centre_x, physical[0], physical[2]))
            centre_y = float(np.clip(centre_y, physical[1], physical[3]))
        centre_x = float(
            np.clip(centre_x, half_width, image.shape[1] - half_width)
        )
        centre_y = float(
            np.clip(centre_y, half_height, image.shape[0] - half_height)
        )
        requested = (
            centre_x - half_width,
            centre_y - half_height,
            centre_x + half_width,
            centre_y + half_height,
        )
        return requested

    @staticmethod
    def _close(
        left: Detection,
        right: Detection,
        centre_ratio: float,
        radius_ratio: float,
    ) -> bool:
        reference_radius = max(4.0, 0.5 * (left.circle.radius + right.circle.radius))
        return (
            math.hypot(
                left.circle.x - right.circle.x,
                left.circle.y - right.circle.y,
            )
            <= max(5.0, centre_ratio * reference_radius)
            and abs(left.circle.radius - right.circle.radius)
            <= max(2.0, radius_ratio * reference_radius)
        )

    def _smoothed(self, latest: Detection) -> Detection:
        self._history.append(latest)
        circles = [item.circle for item in self._history]
        circle = Circle(
            float(np.median([item.x for item in circles])),
            float(np.median([item.y for item in circles])),
            float(np.median([item.radius for item in circles])),
        )
        return Detection(
            circle=circle,
            confidence=float(np.median([item.confidence for item in self._history])),
            score=float(np.median([item.score for item in self._history])),
        )

    def _accepted(self, latest: Detection, fixed_640: bool) -> Detection:
        if not fixed_640:
            return self._smoothed(latest)
        # Median smoothing is excellent for a stationary camera but visibly
        # lags a moving 640px carriage.  Keep raw accepted geometry while the
        # short history is used only to predict the next local window.
        self._history.append(latest)
        half_size = max(52.0, self.search_radius_scale * latest.circle.radius)
        follow_roi = (
            max(160.0, latest.circle.x - half_size),
            max(96.0, latest.circle.y - half_size),
            min(480.0, latest.circle.x + half_size),
            min(384.0, latest.circle.y + half_size),
        )
        if self.last_allowed_roi is not None:
            follow_roi = self._intersect_bounds(follow_roi, self.last_allowed_roi)
        self.last_follow_roi = follow_roi
        return latest

    def _predicted_center(self) -> Optional[tuple[float, float]]:
        if self._stable is None:
            return None
        if len(self._history) < 2:
            return (self._stable.circle.x, self._stable.circle.y)
        previous = self._history[-2].circle
        latest = self._history[-1].circle
        dx = float(np.clip(latest.x - previous.x, -80.0, 80.0))
        dy = float(np.clip(latest.y - previous.y, -80.0, 80.0))
        return (latest.x + dx, latest.y + dy)

    def _detect(
        self,
        image: np.ndarray,
        local_lock: bool,
        *,
        expansion: float = 1.0,
        expected_center: Optional[tuple[float, float]] = None,
        mode: Optional[str] = None,
    ) -> Detection:
        strict_bounds = self._strict_position_bounds(image, expected_center)
        self.last_allowed_roi = strict_bounds
        self.last_allowed_circle = (
            None
            if strict_bounds is None
            else (
                0.5 * (strict_bounds[0] + strict_bounds[2]),
                0.5 * (strict_bounds[1] + strict_bounds[3]),
                0.5
                * min(
                    strict_bounds[2] - strict_bounds[0],
                    strict_bounds[3] - strict_bounds[1],
                ),
            )
        )
        bounds = strict_bounds
        window_expected_center: Optional[tuple[float, float]] = None
        min_radius = self._configured_min_radius
        max_radius = self._configured_max_radius
        fixed_640 = image.shape[:2] == (480, 640)
        if local_lock and (self._stable is not None or expected_center is not None):
            radius = 15.0 if self._stable is None else self._stable.circle.radius
            predicted = self._predicted_center()
            if expected_center is not None:
                # A machine/UI position is authoritative.  Blending it with a
                # stale previous circle can move the search gate toward the
                # background, exactly when the carriage has just moved.
                expected_delta = (
                    0.0
                    if predicted is None
                    else math.hypot(
                        expected_center[0] - predicted[0],
                        expected_center[1] - predicted[1],
                    )
                )
                centre_x, centre_y = map(float, expected_center)
            elif predicted is not None:
                expected_delta = 0.0
                centre_x, centre_y = predicted
            else:
                expected_delta = 0.0
                assert self._stable is not None
                centre_x = self._stable.circle.x
                centre_y = self._stable.circle.y
            minimum_half_size = 52.0 if fixed_640 else 72.0
            half_size = max(
                minimum_half_size,
                self.search_radius_scale * radius,
            )
            half_size = expansion * half_size + min(36.0, 0.18 * expected_delta)
            local_bounds = (
                centre_x - half_size,
                centre_y - half_size,
                centre_x + half_size,
                centre_y + half_size,
            )
            if fixed_640:
                local_bounds = (
                    max(160.0, local_bounds[0]),
                    max(96.0, local_bounds[1]),
                    min(480.0, local_bounds[2]),
                    min(384.0, local_bounds[3]),
                )
                if (
                    local_bounds[2] - local_bounds[0] < 32.0
                    or local_bounds[3] - local_bounds[1] < 32.0
                ):
                    raise RuntimeError("expected/follow 搜索窗不在 640x480 主喷嘴硬范围内")
            if fixed_640 and strict_bounds is not None:
                # The supplied/remembered position rectangle is the active
                # same-size search range.  It translates with the approximate
                # centre but never grows because of motion distance.
                bounds = strict_bounds
            else:
                bounds = (
                    local_bounds
                    if strict_bounds is None
                    else self._intersect_bounds(local_bounds, strict_bounds)
                )
            window_expected_center = (centre_x, centre_y)
            if min_radius is None:
                min_radius = 0.82 * radius
            if max_radius is None:
                max_radius = 1.18 * radius
        elif strict_bounds is not None:
            # Initial/global fallback stays inside the active hard gate.  If a
            # position was supplied it remains the ranking centre as well.
            if expected_center is not None:
                window_expected_center = tuple(map(float, expected_center))
            elif self._stable is not None:
                window_expected_center = self._predicted_center()
        self.last_follow_roi = bounds
        detect_kwargs = dict(self._options)
        if mode is not None:
            detect_kwargs["mode"] = mode
        return detect_nozzle(
            image,
            min_radius_px=min_radius,
            max_radius_px=max_radius,
            search_bounds=bounds,
            expected_center=window_expected_center,
            search_circle=self.last_allowed_circle,
            **detect_kwargs,
        )

    def update(
        self,
        image: np.ndarray,
        expected_center: Optional[tuple[float, float]] = None,
        mode: Optional[str] = None,
    ) -> Detection:
        """Process one frame and return the current non-jumping lock."""
        if expected_center is not None and (
            len(expected_center) != 2
            or not all(math.isfinite(float(value)) for value in expected_center)
        ):
            raise ValueError("expected_center 必须是两个有限像素坐标")
        fixed_640 = image.shape[:2] == (480, 640)
        explicit_expected = expected_center is not None
        detect_mode = _normalize_detect_mode(mode, explicit_expected)
        recovery_expected = False
        if explicit_expected and fixed_640:
            assert expected_center is not None
            expected_center = tuple(map(float, expected_center))
            self._expected_recovery_center = expected_center
            self._expected_recovery_left = self._expected_recovery_frames
        elif fixed_640 and (
            self._expected_recovery_center is not None
            and self._expected_recovery_left > 0
        ):
            # The caller may send expected only on the movement request.  Keep
            # using that exact non-expanding 80x72 gate for a few fresh camera
            # frames instead of falling back to the old visual circle.
            expected_center = self._expected_recovery_center
            recovery_expected = True
        else:
            self._clear_expected_recovery()
        local_requested = self._stable is not None or expected_center is not None
        physical_face_reacquired = False
        try:
            candidate = self._detect(
                image,
                local_lock=local_requested,
                expected_center=expected_center,
                mode=detect_mode,
            )
        except RuntimeError as local_error:
            if fixed_640 and self.strict_position_lock:
                if expected_center is not None or not self.allow_fixed640_relocation:
                    # A supplied machine position is authoritative.  Never
                    # escape it to another nozzle or a background object.
                    self.last_reject_reason = str(local_error)
                    self._pending = None
                    self._pending_count = 0
                    self._missing_count += 1
                    if expected_center is not None:
                        # The machine position outranks a circle detected before
                        # the move.  Never allow the next no-expected frame to
                        # resume from that stale circle.
                        self._invalidate_visual_lock()
                        if recovery_expected:
                            self._expected_recovery_left -= 1
                            if self._expected_recovery_left <= 0:
                                self._clear_expected_recovery()
                        self.last_status = "rejected_expected"
                        if self._expected_recovery_center is None:
                            raise RuntimeError(
                                "预计位置连续拒检；旧圆锁和预计位置已清除，"
                                "下一帧回到中心 80x72 初始区域"
                            ) from local_error
                        raise RuntimeError(
                            "预计位置的固定识别范围内未找到喷嘴；"
                            "旧圆锁已清除，保留同尺寸位置窗作短时重试"
                        ) from local_error
                    if self._stable is None:
                        self.last_status = "rejected"
                        raise
                    if self._missing_count >= self._expected_recovery_frames:
                        # Missing expected transmission must not leave a
                        # continuous session permanently following old pixels.
                        self._invalidate_visual_lock()
                        self.last_status = "lost_position"
                        raise RuntimeError(
                            "连续固定窗拒检；旧圆锁已清除，"
                            "下一帧回到中心 80x72 初始区域"
                        ) from local_error
                    self.last_status = "rejected_missing"
                    raise RuntimeError(
                        "固定位置的识别范围内未找到喷嘴；"
                        "未扩大、未跳到背景、未输出旧圆"
                    ) from local_error

                # Optional legacy mode only.  ScreenQML's service disables it;
                # standalone callers may explicitly retain bounded *face*
                # relocation inside
                # the calibrated primary-nozzle work area.  This is not an
                # unconstrained circle fallback: the locator requires the
                # 3--6 sided local metal face and the 13.5--17 px physical rim,
                # then the result is proved a second time inside the translated
                # active circular gate.  Background circles cannot pass both.
                try:
                    locator = detect_nozzle(
                        image,
                        min_radius_px=self._configured_min_radius,
                        max_radius_px=self._configured_max_radius,
                        **self._options,
                    )
                    if locator.confidence < max(
                        0.70,
                        float(self._options["min_confidence"]),
                    ):
                        raise RuntimeError(
                            "主喷嘴物理区重定位证据不足，已拒绝弱候选"
                        )
                    candidate = self._detect(
                        image,
                        local_lock=True,
                        expected_center=(
                            locator.circle.x,
                            locator.circle.y,
                        ),
                    )
                    physical_face_reacquired = True
                except RuntimeError as relocation_error:
                    self.last_reject_reason = str(relocation_error)
                    self._pending = None
                    self._pending_count = 0
                    if self._stable is None:
                        self.last_status = "rejected"
                        raise
                    self._missing_count += 1
                    self.last_status = "rejected_missing"
                    raise RuntimeError(
                        "当前位置未通过圆形跟随及主喷嘴金属面重定位；未输出旧圆"
                    ) from relocation_error
            if not physical_face_reacquired:
                # Non-fixed resolutions retain the historical expand/general
                # fallback.  A successful fixed-640 physical-face relocation
                # has already been double-proved and must not be overwritten.
                try:
                    if not local_requested:
                        raise local_error
                    candidate = self._detect(
                        image,
                        local_lock=True,
                        expansion=1.8,
                        expected_center=expected_center,
                    )
                    self.last_status = "expanded"
                except RuntimeError:
                    try:
                        candidate = self._detect(image, local_lock=False)
                        self.last_status = (
                            "bounded_reacquire" if fixed_640 else "global_reacquire"
                        )
                    except RuntimeError as global_error:
                        self.last_reject_reason = str(global_error)
                        if self._stable is None:
                            self.last_status = "rejected"
                            raise
                        self._missing_count += 1
                        self._pending = None
                        self._pending_count = 0
                        if fixed_640:
                            self.last_status = "rejected_missing"
                            raise RuntimeError(
                                "当前帧局部窗、扩窗和位置硬范围兜底均未通过；未输出旧圆"
                            ) from global_error
                        if self._missing_count <= self.hold_missing_frames:
                            self.last_status = "held_missing"
                            return self._stable
                        self.last_status = "lost"
                        raise RuntimeError(
                            f"连续 {self._missing_count} 帧没有通过六边形硬约束的喷口；锁定已暂停"
                        ) from local_error

        self.last_reject_reason = ""
        self._missing_count = 0
        used_expected_gate = expected_center is not None
        # A successful explicit machine expected may be followed by one or
        # more fresh-frame verification calls without expected.  Do not clear
        # the exact gate immediately: the previous->current large carriage
        # displacement is not a velocity sample to extrapolate again.  Hold
        # this same non-expanding gate for a few frames while the visual
        # history fills with the new stationary centre.
        if recovery_expected:
            self._expected_recovery_left -= 1
            if self._expected_recovery_left <= 0:
                self._clear_expected_recovery()
        elif not explicit_expected:
            self._clear_expected_recovery()
        if self._stable is None:
            self._history.clear()
            self._stable = self._accepted(candidate, fixed_640)
            self.last_status = (
                "locked_physical_face"
                if physical_face_reacquired
                else (
                    "recovered_expected"
                    if recovery_expected
                    else ("locked_expected" if used_expected_gate else "locked")
                )
            )
            return self._stable

        if fixed_640 and physical_face_reacquired:
            self._pending = None
            self._pending_count = 0
            self._stable = self._accepted(candidate, fixed_640=True)
            self.last_status = "reacquired_physical_face"
            return self._stable

        if fixed_640 and expected_center is not None:
            self._pending = None
            self._pending_count = 0
            self._stable = self._accepted(candidate, fixed_640=True)
            self.last_status = (
                "tracked_expected" if explicit_expected else "recovered_expected"
            )
            return self._stable

        close_centre_ratio = 2.4 if fixed_640 else 0.30
        close_radius_ratio = 0.16 if fixed_640 else 0.11
        if self._close(
            self._stable,
            candidate,
            centre_ratio=close_centre_ratio,
            radius_ratio=close_radius_ratio,
        ):
            self._pending = None
            self._pending_count = 0
            self._stable = self._accepted(candidate, fixed_640)
            self.last_status = "tracked"
            return self._stable

        if fixed_640 and expected_center is None:
            # A large but correctly predicted carriage step may place the new
            # nozzle well beyond the old-circle jump threshold.  Confirm it
            # with an independent full primary-work-area metal-face proof.
            # Immediate acceptance requires both paths to agree tightly; a
            # lone background/glare circle still falls through to the normal
            # multi-frame rejection below.
            try:
                face_confirmation = detect_nozzle(
                    image,
                    min_radius_px=self._configured_min_radius,
                    max_radius_px=self._configured_max_radius,
                    **self._options,
                )
                confirmed_large_step = bool(
                    face_confirmation.confidence
                    >= max(0.70, float(self._options["min_confidence"]))
                    and math.hypot(
                        face_confirmation.circle.x - candidate.circle.x,
                        face_confirmation.circle.y - candidate.circle.y,
                    )
                    <= 2.5
                    and abs(
                        face_confirmation.circle.radius - candidate.circle.radius
                    )
                    <= 1.0
                )
            except RuntimeError:
                confirmed_large_step = False
            if confirmed_large_step:
                self._pending = None
                self._pending_count = 0
                self._stable = self._accepted(candidate, fixed_640=True)
                self.last_status = "tracked_face_confirmed"
                return self._stable

        if self._pending is not None and self._close(
            self._pending,
            candidate,
            centre_ratio=1.8 if fixed_640 else 0.35,
            radius_ratio=0.18 if fixed_640 else 0.13,
        ):
            self._pending_count += 1
            self._pending = candidate
        else:
            self._pending = candidate
            self._pending_count = 1

        if self._pending_count >= self.reacquire_frames:
            self._history.clear()
            self._stable = self._accepted(candidate, fixed_640)
            self._pending = None
            self._pending_count = 0
            self.last_status = "reacquired"
            return self._stable

        self.last_status = "rejected_jump" if fixed_640 else "held_jump"
        if fixed_640:
            self.last_reject_reason = "候选相对 follow 单帧跳变过大，等待一致重捕获"
            raise RuntimeError(self.last_reject_reason)
        return self._stable

def draw_detection(image: np.ndarray, detection: Detection) -> np.ndarray:
    result = image.copy()
    x = int(round(detection.circle.x))
    y = int(round(detection.circle.y))
    radius = int(round(detection.circle.radius))
    short_side = min(image.shape[:2])
    thickness = max(2, round(short_side / 450))
    cv2.circle(result, (x, y), radius, (0, 255, 0), thickness, cv2.LINE_AA)
    return result

def _default_output_path(input_path: Path) -> Path:
    suffix = input_path.suffix if input_path.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"} else ".png"
    return input_path.with_name(f"{input_path.stem}_circle{suffix}")

def _open_result(path: Path) -> bool:
    try:
        system = platform.system()
        if system == "Darwin":
            subprocess.Popen(["open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        elif system == "Windows":
            os.startfile(str(path))  # type: ignore[attr-defined]
        else:
            subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except (OSError, subprocess.SubprocessError):
        return False

def process_image(
    input_path: Path,
    output_path: Optional[Path] = None,
    min_confidence: float = 0.36,
    open_result: bool = True,
    roi_width_fraction: float = 0.50,
    roi_height_fraction: float = 0.50,
    fast_camera_mode: bool = True,
    fallback_general: bool = True,
    adaptive_vertical_roi: bool = True,
    min_radius_px: Optional[float] = None,
    max_radius_px: Optional[float] = None,
    search_bounds: Optional[tuple[float, float, float, float]] = None,
) -> Detection:
    input_path = input_path.expanduser().resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"图片不存在：{input_path}")
    output_path = (output_path or _default_output_path(input_path)).expanduser().resolve()
    if output_path == input_path:
        raise ValueError("输出路径不能覆盖原图")

    image = _read_image(input_path)
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
    annotated = draw_detection(image, detection)
    _write_image(output_path, annotated)
    if open_result:
        _open_result(output_path)
    return Detection(
        circle=detection.circle,
        confidence=detection.confidence,
        score=detection.score,
        output_path=output_path,
    )

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="识别一个喷嘴本体外圆边沿，输出坐标对应完整原图。"
    )
    parser.add_argument("image", type=Path, help="截图或图片路径")
    parser.add_argument("-o", "--output", type=Path, help="结果路径；默认保存到原图同目录")
    parser.add_argument(
        "--min-confidence",
        type=float,
        default=0.36,
        help="最低置信度，越高越保守（默认：0.36）",
    )
    parser.add_argument(
        "--roi-width",
        type=float,
        default=0.50,
        help="中心 ROI 宽度占原图比例（默认：0.50）",
    )
    parser.add_argument(
        "--roi-height",
        type=float,
        default=0.50,
        help="中心 ROI 高度占原图比例（默认：0.50）",
    )
    parser.add_argument(
        "--general-mode",
        action="store_true",
        help="关闭真机小喷嘴尺度快速通道，在 ROI 内执行完整多尺度搜索",
    )
    parser.add_argument(
        "--no-fallback",
        action="store_true",
        help="快速通道失败时不回退到 ROI 完整多尺度搜索",
    )
    parser.add_argument(
        "--fixed-center",
        action="store_true",
        help="固定使用几何中心 ROI，不随打印头上下位置选择窗口",
    )
    parser.add_argument(
        "--min-radius-px",
        type=float,
        help="可选：允许的最小半径（原图像素）",
    )
    parser.add_argument(
        "--max-radius-px",
        type=float,
        help="可选：允许的最大半径（原图像素）",
    )
    parser.add_argument("--expected-x", type=float, help="可选：喷口预期圆心 x（原图像素）")
    parser.add_argument("--expected-y", type=float, help="可选：喷口预期圆心 y（原图像素）")
    parser.add_argument(
        "--search-width-px",
        type=float,
        help="与 expected-x/y 同用：硬搜索窗口宽度（像素）",
    )
    parser.add_argument(
        "--search-height-px",
        type=float,
        help="与 expected-x/y 同用：硬搜索窗口高度（像素）",
    )
    parser.add_argument("--no-open", action="store_true", help="保存后不自动打开")
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
        detection = process_image(
            args.image,
            output_path=args.output,
            min_confidence=args.min_confidence,
            open_result=not args.no_open,
            roi_width_fraction=args.roi_width,
            roi_height_fraction=args.roi_height,
            fast_camera_mode=not args.general_mode,
            fallback_general=not args.no_fallback,
            adaptive_vertical_roi=not args.fixed_center,
            min_radius_px=args.min_radius_px,
            max_radius_px=args.max_radius_px,
            search_bounds=search_bounds,
        )
    except (FileNotFoundError, ValueError, OSError, RuntimeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2

    payload = {
        "x": round(detection.circle.x, 2),
        "y": round(detection.circle.y, 2),
        "radius": round(detection.circle.radius, 2),
        "confidence": round(detection.confidence, 4),
        "output": str(detection.output_path),
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
