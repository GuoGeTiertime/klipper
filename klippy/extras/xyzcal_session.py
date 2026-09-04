# XYZ calib session snapshot after Center.
# Holds NozzleDetectionProfile + XYZPixelPoint; filled on XYZCAL_CENTER success.
#
# Copyright (C) 2026 TierTime / ScreenQML migration
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import annotations

import logging
import math
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Optional


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


class XyzCalSession:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.profile = None  # type: Optional[NozzleDetectionProfile]
        self.point = None  # type: Optional[XYZPixelPoint]
        self.last_detect = DetectCallContext()
        self.ready = False
        self._message = ""
        self.gcode.register_command(
            "XYZCAL_SESSION_DUMP",
            self.cmd_XYZCAL_SESSION_DUMP,
            desc=self.cmd_XYZCAL_SESSION_DUMP_help,
        )

    def _detect_obj(self):
        return self.printer.lookup_object("xyzcal_detect", None)

    def build_from_center(self, calib, tool=ToolType.MAIN):
        """Fill Profile + Point from a successful Center run on calib."""
        det = self._detect_obj()
        url = ""
        tol = 0.15
        if det is not None:
            url = str(getattr(det, "snapshot_url", "") or "")
            try:
                tol = float(getattr(det, "nozzle_radius_tol", 0.15) or 0.15)
            except (TypeError, ValueError):
                tol = 0.15
        if not url:
            url = str(getattr(calib, "snapshot_url", "") or "")

        fw = float(getattr(calib, "_last_fw", 0) or 0)
        fh = float(getattr(calib, "_last_fh", 0) or 0)
        if fw < 40 or fh < 40:
            fw, fh = 640.0, 480.0
        target_x = 0.5 * fw
        target_y = 0.5 * fh

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

        conf = float(getattr(calib, "min_confidence", 0.35) or 0.35)

        self.profile = NozzleDetectionProfile(
            snapshot_url=url,
            target_pixel_x=target_x,
            target_pixel_y=target_y,
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
        self.ready = True
        self._message = "center_ok"
        return self.profile, self.point

    def apply_radius_to_detect(self):
        """Push tip radius into xyzcal_detect runtime so later Detect uses band."""
        if not self.ready or self.profile is None or self.point is None:
            return
        if not self.point.detected:
            return
        det = self._detect_obj()
        if det is None:
            return
        # Reconstruct r from band center when tip r stored on calib already applied
        r_min = float(self.profile.radius_min_px)
        r_max = float(self.profile.radius_max_px)
        r = 0.5 * (r_min + r_max)
        tip_r = None
        # Prefer true tip radius if band was built from it
        calib = self.printer.lookup_object("xyzcal_calib", None)
        if calib is not None:
            tip_r = float(getattr(calib, "_last_r", 0) or 0)
        if tip_r and tip_r > 0.0:
            r = tip_r
        det.nozzle_radius_px = float(r)
        band = det.calibrated_radius_band()
        logging.info(
            "xyzcal_session: set detect nozzle_radius_px=%.2f band=%s",
            r,
            band,
        )

    def on_center_done(self, calib, gcmd, tool=ToolType.MAIN):
        tool_e = tool if isinstance(tool, ToolType) else _parse_tool(tool)
        self.build_from_center(calib, tool=tool_e)
        self.apply_radius_to_detect()
        self.emit(gcmd)

    def emit(self, gcmd):
        if not self.ready or self.profile is None or self.point is None:
            gcmd.respond_info("XYZCAL_SESSION ready=0 (run XYZCAL_CENTER first)")
            return
        p = self.profile
        pt = self.point
        d = self.last_detect
        # Field names + order match NozzleDetectionProfile / XYZPixelPoint / DetectCallContext
        gcmd.respond_info(
            "XYZCAL_SESSION profile "
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
            "XYZCAL_SESSION point "
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
            "XYZCAL_SESSION detect "
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

    cmd_XYZCAL_SESSION_DUMP_help = (
        "Dump last Center session Profile + Point. No params."
    )

    def cmd_XYZCAL_SESSION_DUMP(self, gcmd):
        self.emit(gcmd)

    def get_status(self, eventtime=None):
        out = {
            "ready": bool(self.ready),
            "message": self._message or "",
        }
        if self.profile is not None:
            pd = asdict(self.profile)
            for k, v in pd.items():
                out["profile_%s" % k] = v
        else:
            for k in (
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
            ):
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


def load_config(config):
    return XyzCalSession(config)
