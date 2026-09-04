# Initial camera profile for dual-nozzle XYZ offset calibration.
#
# Copyright (C) 2026 TierTime
# This file may be distributed under the terms of the GNU GPLv3 license.

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class ToolType(str, Enum):
    MAIN = "MAIN"       # 主喷头
    SECOND = "SECOND"   # 副喷头


@dataclass
class XYZPixelPoint:
    x: float                              # 当前喷头的机器绝对 X 坐标，单位 mm
    y: float                              # 当前喷头的机器绝对 Y 坐标，单位 mm
    z: float                              # 当前喷头的机器绝对 Z 坐标，单位 mm
    tool: ToolType                        # 当前检测的是主喷头还是副喷头
    pixel_x: Optional[float] = None       # 喷嘴中心在完整图像中的 X 像素坐标
    pixel_y: Optional[float] = None       # 喷嘴中心在完整图像中的 Y 像素坐标
    detected: bool = False                # 本次喷嘴检测是否成功


@dataclass(frozen=True)
class NozzleDetectionProfile:
    image_width: int                      # 初始化图像宽度，单位 pixel
    image_height: int                     # 初始化图像高度，单位 pixel
    center_pixel_x: float                 # 图像中心 X，后续居中检测的目标位置
    center_pixel_y: float                 # 图像中心 Y，后续居中检测的目标位置
    search_delta_x: float                 # 期望位置左右两侧的搜索宽度，单位 pixel
    search_delta_y: float                 # 期望位置上下两侧的搜索宽度，单位 pixel
    radius_min_px: float                  # 可接受的喷嘴孔最小半径，单位 pixel
    radius_max_px: float                  # 可接受的喷嘴孔最大半径，单位 pixel
    min_confidence: float                 # 后续检测可接受的最低置信度
    fresh_frame_flush_count: int          # 后续取新帧前需要丢弃的缓存帧数


def _parse_tool(value):
    text = str(value or "MAIN").strip().upper()
    if text in ("SECOND", "SEC", "T1", "1"):
        return ToolType.SECOND
    if text in ("MAIN", "T0", "0"):
        return ToolType.MAIN
    raise ValueError("TOOL must be MAIN or SECOND")


class XYZOffsetCalibrator:
    """Build the immutable detection profile from one initial capture."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")

        # 这些是初始化策略，而不是相机地址；地址由 camera_capture 统一管理。
        self.search_tolerance = config.getfloat("search_tolerance", 3.0, above=1.5)    # 默认搜索区域为喷嘴直径的3倍
        self.radius_tolerance = config.getfloat("radius_tolerance", 0.30, above=0.0, maxval=1.0)
        self.min_confidence = config.getfloat("min_confidence", 0.36, minval=0.0, maxval=1.0)
        self.fresh_frame_flush_count = config.getint("fresh_frame_flush_count", 2, minval=0)

        self.profile = None
        self.initial_point = None
        self.initialized = False
        self.last_error = "not initialized"

        self.gcode.register_command(
            "XYZ_OFFSET_INIT",
            self.cmd_XYZ_OFFSET_INIT,
            desc=self.cmd_XYZ_OFFSET_INIT_help,
        )
        self.gcode.register_command(
            "XYZ_OFFSET_STATUS",
            self.cmd_XYZ_OFFSET_STATUS,
            desc=self.cmd_XYZ_OFFSET_STATUS_help,
        )

    def _current_position(self):
        toolhead = self.printer.lookup_object("toolhead")
        position = toolhead.get_position()
        return float(position[0]), float(position[1]), float(position[2])

    def initialize(self, tool=ToolType.MAIN):
        """Capture once, find the current nozzle, and build the profile."""
        tool = tool if isinstance(tool, ToolType) else _parse_tool(tool)
        camera = self.printer.lookup_object("camera_capture", None)
        if camera is None:
            raise RuntimeError("camera_capture is not defined or loaded")
        finder = self.printer.lookup_object("nozzle_finder", None)
        if finder is None:
            raise RuntimeError("nozzle_finder is not defined or loaded")

        x, y, z = self._current_position()
        point = XYZPixelPoint(x=x, y=y, z=z, tool=tool)
        self.profile = None
        self.initial_point = point
        self.initialized = False

        image = camera.capture()
        image_height, image_width = image.shape[:2]

        center_x = 0.5 * float(image_width)
        center_y = 0.5 * float(image_height)
        size = min(image_width, image_height)
        search_width = 0.5 * size  # 搜索区域完整宽度为图像短边的 1/2
        search_height = 0.5 * size    # 搜索区域完整高度为图像短边的 1/2
        expected_diameter = size * 0.05     # 默认喷嘴直径为图像尺寸的5%， 480高图形，喷嘴大小为24pixel.

        # 初始喷嘴已在画面中心附近，先按图像中心限定搜索区域。
        finder.configure(
            center_x,
            center_y,
            search_width,
            search_height,
            expected_diameter,
        )
        finder.set_diameter_scales(0.5, 2.0)  # 第一次用较大的直径范围进行搜索
        result = finder.execute()
        if not result.get("ok"):
            self.last_error = result.get("error", "DETECT_FAILED")
            return point, 0.0

        radius = float(result["radius"]) if "radius" in result else 0.0
        if radius <= 0.0:
            self.last_error = "INVALID_RADIUS"
            return point, 0.0
        radius_margin = max(2.0, radius * self.radius_tolerance)
        radius_min = max(1.0, radius - radius_margin)
        radius_max = radius + radius_margin
        search_delta_x = radius * self.search_tolerance
        search_delta_y = radius * self.search_tolerance
        point.pixel_x = float(result["cx"])
        point.pixel_y = float(result["cy"])
        point.detected = True

        # 首次宽范围搜索完成后，立即按实测结果收紧共享 Finder。
        finder.configure(
            center_x,
            center_y,
            2.0 * search_delta_x,
            2.0 * search_delta_y,
            2.0 * radius,
        )
        finder.set_diameter_scales(
            radius_min / radius,
            radius_max / radius,
        )

        self.profile = NozzleDetectionProfile(
            image_width=int(image_width),
            image_height=int(image_height),
            center_pixel_x=center_x,
            center_pixel_y=center_y,
            search_delta_x=search_delta_x,
            search_delta_y=search_delta_y,
            radius_min_px=radius_min,
            radius_max_px=radius_max,
            min_confidence=self.min_confidence,
            fresh_frame_flush_count=self.fresh_frame_flush_count,
        )
        self.initialized = True
        self.last_error = ""
        return point, radius

    cmd_XYZ_OFFSET_INIT_help = (
        "Capture and initialize the dual-nozzle detection profile. "
        "Params: TOOL=MAIN|SECOND"
    )

    def cmd_XYZ_OFFSET_INIT(self, gcmd):
        try:
            tool = _parse_tool(gcmd.get("TOOL", "MAIN"))
            point, radius = self.initialize(tool)
        except (RuntimeError, ValueError) as exc:
            self.last_error = str(exc)
            raise gcmd.error("XYZ_OFFSET_INIT failed: %s" % (exc,))
        except Exception as exc:
            self.last_error = str(exc)
            raise gcmd.error("XYZ_OFFSET_INIT failed: %s" % (exc,))

        if not point.detected:
            raise gcmd.error("XYZ_OFFSET_INIT failed: %s" % (self.last_error,))
        gcmd.respond_info(
            "XYZ_OFFSET_INIT ok=True tool=%s machine=(%.3f,%.3f,%.3f) "
            "pixel=(%.2f,%.2f), radius=%.2f"
            % (point.tool.value, point.x,  point.y, point.z, point.pixel_x, point.pixel_y, radius,)
        )

    cmd_XYZ_OFFSET_STATUS_help = "Report the initialized detection profile"

    def cmd_XYZ_OFFSET_STATUS(self, gcmd):
        if not self.initialized or self.profile is None:
            gcmd.respond_info(
                "XYZ_OFFSET_STATUS initialized=0 error=%s" % (self.last_error,)
            )
            return
        profile = self.profile
        point = self.initial_point
        gcmd.respond_info(
            "XYZ_OFFSET_STATUS initialized=1 image=%dx%d center=(%.2f,%.2f) "
            "search_delta=(%.2f,%.2f) radius=(%.2f,%.2f) "
            "min_confidence=%.3f flush_count=%d"
            % (
                profile.image_width,
                profile.image_height,
                profile.center_pixel_x,
                profile.center_pixel_y,
                profile.search_delta_x,
                profile.search_delta_y,
                profile.radius_min_px,
                profile.radius_max_px,
                profile.min_confidence,
                profile.fresh_frame_flush_count,
            )
        )
        gcmd.respond_info(
            "XYZ_OFFSET_STATUS point tool=%s machine=(%.3f,%.3f,%.3f) "
            "pixel=(%.2f,%.2f) detected=%d"
            % (
                point.tool.value,
                point.x,
                point.y,
                point.z,
                point.pixel_x,
                point.pixel_y,
                1 if point.detected else 0,
            )
        )

    def get_status(self, eventtime=None):
        profile = self.profile
        point = self.initial_point
        return {
            "initialized": self.initialized,
            "profile": None if profile is None else {
                "image_width": profile.image_width,
                "image_height": profile.image_height,
                "center_pixel_x": profile.center_pixel_x,
                "center_pixel_y": profile.center_pixel_y,
                "search_delta_x": profile.search_delta_x,
                "search_delta_y": profile.search_delta_y,
                "radius_min_px": profile.radius_min_px,
                "radius_max_px": profile.radius_max_px,
                "min_confidence": profile.min_confidence,
                "fresh_frame_flush_count": profile.fresh_frame_flush_count,
            },
            "initial_point": None if point is None else {
                "x": point.x,
                "y": point.y,
                "z": point.z,
                "tool": point.tool.value,
                "pixel_x": point.pixel_x,
                "pixel_y": point.pixel_y,
                "detected": point.detected,
            },
            "last_error": self.last_error,
        }


def load_config(config):
    return XYZOffsetCalibrator(config)
