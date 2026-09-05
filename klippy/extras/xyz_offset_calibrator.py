# Initial camera profile for dual-nozzle XYZ offset calibration.
#
# Copyright (C) 2026 TierTime
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import statistics

from dataclasses import dataclass, replace
from enum import Enum
from typing import Optional


Y_CENTER_DIFFERENCE_OFFSETS = (4.3, 4.4, 4.5, 4.6, 4.7)


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
    x_axis_maps_to_height: Optional[bool] = None  # X 轴是否对应图像高度方向
    pixels_per_mm: Optional[float] = None         # 坐标轴每移动 1 mm 对应的像素数
    x_axis_pixel_x_per_mm: Optional[float] = None  # 机器 +X 引起的 Pixel X 变化量
    x_axis_pixel_y_per_mm: Optional[float] = None  # 机器 +X 引起的 Pixel Y 变化量
    y_axis_pixel_x_per_mm: Optional[float] = None  # 机器 +Y 引起的 Pixel X 变化量
    y_axis_pixel_y_per_mm: Optional[float] = None  # 机器 +Y 引起的 Pixel Y 变化量


def _parse_tool(value):
    text = str(value or "MAIN").strip().upper()
    if text in ("SECOND", "SEC", "T1", "1"):
        return ToolType.SECOND
    if text in ("MAIN", "T0", "0"):
        return ToolType.MAIN
    raise ValueError("TOOL must be MAIN or SECOND")


class XYZOffsetCalibrator:
    """Initialize and calibrate the dual-nozzle camera profile."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")

        # 这些是初始化策略，而不是相机地址；地址由 camera_capture 统一管理。
        self.search_tolerance = config.getfloat("search_tolerance", 3.0, above=1.5)    # 默认搜索区域为喷嘴直径的3倍
        self.radius_tolerance = config.getfloat("radius_tolerance", 0.30, above=0.0, maxval=1.0)
        self.min_confidence = config.getfloat("min_confidence", 0.36, minval=0.0, maxval=1.0)
        self.fresh_frame_flush_count = config.getint("fresh_frame_flush_count", 2, minval=0)
        self.xy_probe_mm = config.getfloat("xy_probe_mm", 0.5, above=0.0)
        self.xy_reverse_mm = config.getfloat("xy_reverse_mm", 3.0, above=0.0)
        self.move_speed = config.getfloat("move_speed", 10.0, above=0.0)
        self.xy_settle_time = config.getfloat("xy_settle_time", 0.3, minval=0.0)
        self.xy_min_move_px = config.getfloat("xy_min_move_px", 3.0, above=0.0)
        self.xy_scale_tolerance = config.getfloat(
            "xy_scale_tolerance", 0.35, above=0.0, maxval=1.0
        )
        self.xy_orthogonality_tolerance = config.getfloat(
            "xy_orthogonality_tolerance", 0.35, minval=0.0, maxval=1.0
        )
        self.center_tolerance_px = config.getfloat(
            "center_tolerance_px", 0.2, above=0.0
        )
        self.center_max_iterations = config.getint(
            "center_max_iterations", 5, minval=1, maxval=5
        )
        self.center_coarse_tolerance_px = config.getfloat(
            "center_coarse_tolerance_px", 1.0, above=0.0
        )
        self.center_sample_distance_mm = config.getfloat(
            "center_sample_distance_mm", 1.0, above=0.0
        )
        self.secondary_tool_macro = config.get("secondary_tool_macro", "").strip()

        self.profile = None
        self.initial_point = None
        self.xy_points = []
        self.center_points = []
        self.center_sample_points = []
        self.center_point = None
        self.center_iterations = 0
        self.center_mode = 0
        self.center_average_pixel_x = None
        self.center_average_pixel_y = None
        self.current_tool = ToolType.MAIN
        self.tool_center_points = {}
        self.xy_offset_x = None
        self.xy_offset_y = None
        self.y_center_difference = None
        self.main_y_center_difference = None
        self.secondary_y_center_differences = []
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
        self.gcode.register_command(
            "XYZ_OFFSET_CALIBRATE_XY",
            self.cmd_XYZ_OFFSET_CALIBRATE_XY,
            desc=self.cmd_XYZ_OFFSET_CALIBRATE_XY_help,
        )
        self.gcode.register_command(
            "XYZ_OFFSET_CENTER",
            self.cmd_XYZ_OFFSET_CENTER,
            desc=self.cmd_XYZ_OFFSET_CENTER_help,
        )
        self.gcode.register_command(
            "XYZ_OFFSET_CALIBRATE_SECONDARY",
            self.cmd_XYZ_OFFSET_CALIBRATE_SECONDARY,
            desc=self.cmd_XYZ_OFFSET_CALIBRATE_SECONDARY_help,
        )
        self.gcode.register_command(
            "XYZ_OFFSET_MEASURE_Y_DIFF",
            self.cmd_XYZ_OFFSET_MEASURE_Y_DIFF,
            desc=self.cmd_XYZ_OFFSET_MEASURE_Y_DIFF_help,
        )

    def _current_position(self):
        toolhead = self.printer.lookup_object("toolhead")
        position = toolhead.get_position()
        return float(position[0]), float(position[1]), float(position[2])

    def _abs_move(self, coordinate, speed=None):
        """Move non-None XYZ coordinates to absolute machine positions."""
        if len(coordinate) != 3:
            raise ValueError("absolute move coordinate must contain X, Y, Z")
        move_speed = self.move_speed if speed is None else float(speed)
        if not math.isfinite(move_speed) or move_speed <= 0.0:
            raise ValueError("move speed must be positive")
        target = []
        for value in coordinate:
            if value is None:
                target.append(None)
                continue
            value = float(value)
            if not math.isfinite(value):
                raise ValueError("absolute move position must be finite")
            target.append(value)

        toolhead = self.printer.lookup_object("toolhead")
        toolhead.manual_move(target, move_speed)
        if self.xy_settle_time > 0.0:
            toolhead.dwell(self.xy_settle_time)
        toolhead.wait_moves()

    def _rel_move(self, delta, speed=None):
        """Move non-None XYZ values relative to current machine positions."""
        if len(delta) != 3:
            raise ValueError("relative move delta must contain X, Y, Z")
        toolhead = self.printer.lookup_object("toolhead")
        position = toolhead.get_position()
        coordinate = []
        for axis_index, value in enumerate(delta):
            if value is None:
                coordinate.append(None)
                continue
            value = float(value)
            if not math.isfinite(value):
                raise ValueError("relative move distance must be finite")
            coordinate.append(float(position[axis_index]) + value)
        self._abs_move(coordinate, speed)

    def _detect_at(self, expected_x, expected_y, tool):
        profile = self.profile
        finder = self.printer.lookup_object("nozzle_finder", None)
        if finder is None:
            raise RuntimeError("nozzle_finder is not defined or loaded")
        expected_radius = 0.5 * (
            profile.radius_min_px + profile.radius_max_px
        )
        finder.configure(
            expected_x,
            expected_y,
            2.0 * profile.search_delta_x,
            2.0 * profile.search_delta_y,
            2.0 * expected_radius,
        )
        finder.set_diameter_scales(
            profile.radius_min_px / expected_radius,
            profile.radius_max_px / expected_radius,
        )
        result = finder.execute()
        if not result.get("ok"):
            raise RuntimeError(
                "%s: %s"
                % (
                    result.get("error", "DETECT_FAILED"),
                    result.get("detail", ""),
                )
            )
        if (
            int(result.get("frame_width", 0)) != profile.image_width
            or int(result.get("frame_height", 0)) != profile.image_height
        ):
            raise RuntimeError("camera image size changed during XY calibration")
        x, y, z = self._current_position()
        return XYZPixelPoint(
            x=x,
            y=y,
            z=z,
            tool=tool,
            pixel_x=float(result["cx"]),
            pixel_y=float(result["cy"]),
            detected=True,
        )

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
        self.current_tool = tool
        self.tool_center_points = {}
        self.xy_offset_x = None
        self.xy_offset_y = None
        self.y_center_difference = None
        self.main_y_center_difference = None
        self.secondary_y_center_differences = []
        self.xy_points = [point]
        self.center_points = []
        self.center_sample_points = []
        self.center_point = None
        self.center_iterations = 0
        self.center_mode = 0
        self.center_average_pixel_x = None
        self.center_average_pixel_y = None
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

    def calibrate_xy(self):
        """Measure signed XY image vectors with two Y moves and one X move."""
        if not self.initialized or self.profile is None:
            raise RuntimeError("run XYZ_OFFSET_INIT before XY calibration")
        start = self.initial_point
        if start is None or not start.detected:
            raise RuntimeError("initial nozzle point is not available")
        if self.xy_reverse_mm <= self.xy_probe_mm:
            raise RuntimeError("xy_reverse_mm must be greater than xy_probe_mm")

        self.xy_points = [start]
        self._rel_move([None, self.xy_probe_mm, None])
        probe = self._detect_at(start.pixel_x, start.pixel_y, start.tool)
        self.xy_points.append(probe)

        probe_y_mm = probe.y - start.y
        if probe_y_mm <= 0.0:
            raise RuntimeError("first Y movement did not reach the requested direction")
        coarse_x = (probe.pixel_x - start.pixel_x) / probe_y_mm
        coarse_y = (probe.pixel_y - start.pixel_y) / probe_y_mm
        coarse_pixels_per_mm = math.hypot(coarse_x, coarse_y)
        if coarse_pixels_per_mm * probe_y_mm < self.xy_min_move_px:
            raise RuntimeError("first Y movement is too small in the image")
        coarse_y_maps_to_height = abs(coarse_y) >= abs(coarse_x)

        expected_x = probe.pixel_x - self.xy_reverse_mm * coarse_x
        expected_y = probe.pixel_y - self.xy_reverse_mm * coarse_y
        self._rel_move([None, -self.xy_reverse_mm, None])
        final = self._detect_at(expected_x, expected_y, start.tool)
        self.xy_points.append(final)

        total_y_mm = start.y - final.y
        if total_y_mm <= 0.0:
            raise RuntimeError("final Y movement did not cross the initial position")
        final_x = (start.pixel_x - final.pixel_x) / total_y_mm
        final_y = (start.pixel_y - final.pixel_y) / total_y_mm
        pixels_per_mm = math.hypot(final_x, final_y)
        if pixels_per_mm * total_y_mm < self.xy_min_move_px:
            raise RuntimeError("final Y movement is too small in the image")

        final_y_maps_to_height = abs(final_y) >= abs(final_x)
        if final_y_maps_to_height != coarse_y_maps_to_height:
            raise RuntimeError("Y image-axis mapping is inconsistent")
        dot = coarse_x * final_x + coarse_y * final_y
        if dot <= 0.0:
            raise RuntimeError("Y image direction is inconsistent")
        scale_error = abs(pixels_per_mm - coarse_pixels_per_mm) / pixels_per_mm
        if scale_error > self.xy_scale_tolerance:
            raise RuntimeError("coarse and final pixels/mm are inconsistent")

        self._rel_move([self.xy_probe_mm, None, None])
        x_probe = self._detect_at(
            final.pixel_x,
            final.pixel_y,
            start.tool,
        )
        self.xy_points.append(x_probe)

        probe_x_mm = x_probe.x - final.x
        if probe_x_mm <= 0.0:
            raise RuntimeError("X movement did not reach the requested direction")
        x_vector_x = (x_probe.pixel_x - final.pixel_x) / probe_x_mm
        x_vector_y = (x_probe.pixel_y - final.pixel_y) / probe_x_mm
        x_pixels_per_mm = math.hypot(x_vector_x, x_vector_y)
        if x_pixels_per_mm * probe_x_mm < self.xy_min_move_px:
            raise RuntimeError("X movement is too small in the image")

        x_maps_to_height = abs(x_vector_y) >= abs(x_vector_x)
        if x_maps_to_height == final_y_maps_to_height:
            raise RuntimeError("X and Y map to the same image axis")
        axis_scale_error = abs(x_pixels_per_mm - pixels_per_mm) / pixels_per_mm
        if axis_scale_error > self.xy_scale_tolerance:
            raise RuntimeError("X and Y pixels/mm are inconsistent")
        normalized_dot = abs(
            x_vector_x * final_x + x_vector_y * final_y
        ) / (x_pixels_per_mm * pixels_per_mm)
        if normalized_dot > self.xy_orthogonality_tolerance:
            raise RuntimeError("X and Y image vectors are not orthogonal")

        # 两轴独立测量并保留符号；允许镜像映射，行列式可正可负。
        determinant = x_vector_x * final_y - x_vector_y * final_x
        if not math.isfinite(determinant) or abs(determinant) < 1e-6:
            raise RuntimeError("XY image mapping is singular or invalid")

        pixels_per_mm = 0.5 * (x_pixels_per_mm + pixels_per_mm)
        self.profile = replace(
            self.profile,
            x_axis_maps_to_height=x_maps_to_height,
            pixels_per_mm=pixels_per_mm,
            x_axis_pixel_x_per_mm=x_vector_x,
            x_axis_pixel_y_per_mm=x_vector_y,
            y_axis_pixel_x_per_mm=final_x,
            y_axis_pixel_y_per_mm=final_y,
        )
        self.last_error = ""
        return self.profile

    def _xy_mapping(self):
        profile = self.profile
        if profile is None:
            raise RuntimeError("detection profile is not initialized")
        values = (
            profile.x_axis_pixel_x_per_mm,
            profile.x_axis_pixel_y_per_mm,
            profile.y_axis_pixel_x_per_mm,
            profile.y_axis_pixel_y_per_mm,
        )
        if any(value is None for value in values):
            raise RuntimeError("run XYZ_OFFSET_CALIBRATE_XY before centering")
        vx_x, vx_y, vy_x, vy_y = (float(value) for value in values)
        determinant = vx_x * vy_y - vx_y * vy_x
        if not math.isfinite(determinant) or abs(determinant) < 1e-6:
            raise RuntimeError("XY image mapping is singular or invalid")
        return vx_x, vx_y, vy_x, vy_y, determinant

    def _machine_delta_for_pixel_delta(self, pixel_x, pixel_y):
        vx_x, vx_y, vy_x, vy_y, determinant = self._xy_mapping()
        move_x = (vy_y * pixel_x - vy_x * pixel_y) / determinant
        move_y = (-vx_y * pixel_x + vx_x * pixel_y) / determinant
        if not math.isfinite(move_x) or not math.isfinite(move_y):
            raise RuntimeError("calculated XY center movement is invalid")
        return move_x, move_y

    def center_xy_iterative(self, tolerance_px=None):
        """Move the current nozzle to image center in at most five moves."""
        self._xy_mapping()
        if not self.xy_points:
            raise RuntimeError("no measured nozzle point is available")

        profile = self.profile
        tolerance = (
            self.center_tolerance_px
            if tolerance_px is None
            else float(tolerance_px)
        )
        if not math.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("center tolerance must be positive")
        reference = None
        if (
            self.center_point is not None
            and self.center_point.tool == self.current_tool
        ):
            reference = self.center_point
        if reference is None:
            for measured_point in reversed(self.xy_points):
                if measured_point.tool == self.current_tool:
                    reference = measured_point
                    break
        if reference is None:
            raise RuntimeError("no measured point for the current tool")
        current = self._detect_at(
            reference.pixel_x,
            reference.pixel_y,
            reference.tool,
        )
        self.center_points = [current]
        self.center_sample_points = []
        self.center_point = None
        self.center_iterations = 0
        self.center_mode = 0
        self.center_average_pixel_x = None
        self.center_average_pixel_y = None

        for iteration in range(self.center_max_iterations + 1):
            error_x = profile.center_pixel_x - current.pixel_x
            error_y = profile.center_pixel_y - current.pixel_y
            if (
                abs(error_x) <= tolerance
                and abs(error_y) <= tolerance
            ):
                self.center_point = current
                self.center_iterations = iteration
                self.xy_points.append(current)
                self.last_error = ""
                return current
            if iteration >= self.center_max_iterations:
                break

            move_x, move_y = self._machine_delta_for_pixel_delta(
                error_x,
                error_y,
            )
            self._rel_move([move_x, move_y, None])
            current = self._detect_at(
                profile.center_pixel_x,
                profile.center_pixel_y,
                reference.tool,
            )
            self.center_points.append(current)

        self.center_iterations = self.center_max_iterations
        raise RuntimeError(
            "nozzle did not reach image center within %d movements"
            % (self.center_max_iterations,)
        )

    def _sample_after_move(self, delta_x, delta_y, reference):
        profile = self.profile
        vx_x, vx_y, vy_x, vy_y, _ = self._xy_mapping()
        expected_x = reference.pixel_x + vx_x * delta_x + vy_x * delta_y
        expected_y = reference.pixel_y + vx_y * delta_x + vy_y * delta_y
        self._rel_move([delta_x, delta_y, None])
        return self._detect_at(
            expected_x,
            expected_y,
            reference.tool,
        )

    def center_xy_five_point(self):
        """Estimate image-center machine XY from center and four axial points."""
        coarse = self.center_xy_iterative(self.center_coarse_tolerance_px)
        distance = self.center_sample_distance_mm
        samples = [coarse]

        positive_x = self._sample_after_move(distance, 0.0, coarse)
        samples.append(positive_x)
        negative_x = self._sample_after_move(-2.0 * distance, 0.0, positive_x)
        samples.append(negative_x)

        self._rel_move([distance, None, None])
        positive_y = self._sample_after_move(0.0, distance, coarse)
        samples.append(positive_y)
        negative_y = self._sample_after_move(0.0, -2.0 * distance, positive_y)
        samples.append(negative_y)

        sample_count = float(len(samples))
        average_x = sum(point.x for point in samples) / sample_count
        average_y = sum(point.y for point in samples) / sample_count
        average_pixel_x = sum(point.pixel_x for point in samples) / sample_count
        average_pixel_y = sum(point.pixel_y for point in samples) / sample_count
        pixel_error_x = self.profile.center_pixel_x - average_pixel_x
        pixel_error_y = self.profile.center_pixel_y - average_pixel_y
        correction_x, correction_y = self._machine_delta_for_pixel_delta(
            pixel_error_x,
            pixel_error_y,
        )
        center_x = average_x + correction_x
        center_y = average_y + correction_y
        if not math.isfinite(center_x) or not math.isfinite(center_y):
            raise RuntimeError("five-point center estimate is invalid")

        self._abs_move([center_x, center_y, None])
        self.center_sample_points = samples
        self.center_average_pixel_x = average_pixel_x
        self.center_average_pixel_y = average_pixel_y
        self.center_point = XYZPixelPoint(
            x=center_x,
            y=center_y,
            z=coarse.z,
            tool=coarse.tool,
            pixel_x=self.profile.center_pixel_x,
            pixel_y=self.profile.center_pixel_y,
            detected=True,
        )
        self.center_mode = 2
        self.xy_points.append(self.center_point)
        self.last_error = ""
        return self.center_point

    def _save_tool_center(self, point):
        self.tool_center_points[point.tool] = point
        main = self.tool_center_points.get(ToolType.MAIN)
        secondary = self.tool_center_points.get(ToolType.SECOND)
        if main is not None and secondary is not None:
            self.xy_offset_x = secondary.x - main.x
            self.xy_offset_y = secondary.y - main.y

    def _center_by_mode(self, mode):
        mode = str(mode).strip().upper()
        if mode in ("1", "ITERATIVE", "CENTER"):
            point = self.center_xy_iterative()
            self.center_mode = 1
            return point
        if mode in ("2", "FIVE_POINT", "FIVE-POINT"):
            return self.center_xy_five_point()
        raise ValueError("MODE must be 1 or 2")

    def calibrate_secondary(self, mode="1"):
        if ToolType.MAIN not in self.tool_center_points:
            raise RuntimeError("center the main tool before calibrating secondary")
        if not self.secondary_tool_macro:
            raise RuntimeError("secondary_tool_macro is not configured")

        self.gcode.run_script_from_command(self.secondary_tool_macro)
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.wait_moves()
        self.current_tool = ToolType.SECOND
        self.center_point = None
        self.center_sample_points = []
        self.center_average_pixel_x = None
        self.center_average_pixel_y = None

        profile = self.profile
        secondary_start = self._detect_at(
            profile.center_pixel_x,
            profile.center_pixel_y,
            ToolType.SECOND,
        )
        self.xy_points.append(secondary_start)
        point = self._center_by_mode(mode)
        self._save_tool_center(point)
        return point

    def _calculate_y_center_difference(self, negative_points, positive_points):
        """Return the larger signed pixel-axis difference between both sides."""
        vy_x = float(self.profile.y_axis_pixel_x_per_mm)
        vy_y = float(self.profile.y_axis_pixel_y_per_mm)
        group_averages = []
        for points in (negative_points, positive_points):
            valid = [point for point in points if point.detected]
            if len(valid) < 3:
                raise RuntimeError("Y center difference needs at least 3 points per side")

            target_y = sum(point.y for point in valid) / float(len(valid))
            normalized = [
                (
                    point.pixel_x + vy_x * (target_y - point.y),
                    point.pixel_y + vy_y * (target_y - point.y),
                )
                for point in valid
            ]
            median_x = statistics.median(value[0] for value in normalized)
            median_y = statistics.median(value[1] for value in normalized)
            distances = [
                math.hypot(value[0] - median_x, value[1] - median_y)
                for value in normalized
            ]
            median_distance = statistics.median(distances)
            mad = statistics.median(
                abs(distance - median_distance) for distance in distances
            )
            limit = median_distance + max(0.75, 3.0 * 1.4826 * mad)
            filtered = [
                value for value, distance in zip(normalized, distances)
                if distance <= limit
            ]
            if len(filtered) < 3:
                raise RuntimeError("too many Y center difference outliers")
            group_averages.append(
                (
                    sum(value[0] for value in filtered) / float(len(filtered)),
                    sum(value[1] for value in filtered) / float(len(filtered)),
                )
            )
        difference_x = group_averages[0][0] - group_averages[1][0]
        difference_y = group_averages[0][1] - group_averages[1][1]
        if abs(difference_x) >= abs(difference_y):
            return difference_x
        return difference_y

    def measure_y_center_difference(self, tool, offsets=None):
        """Measure symmetric Y points at current Z and save their center difference."""
        tool = tool if isinstance(tool, ToolType) else _parse_tool(tool)
        if tool != self.current_tool:
            raise RuntimeError("requested tool is not the current tool")
        center = self.tool_center_points.get(tool)
        if center is None:
            raise RuntimeError("center the requested tool before Y difference measurement")
        self._xy_mapping()
        offsets = tuple(
            float(value)
            for value in (offsets or Y_CENTER_DIFFERENCE_OFFSETS)
        )
        if not offsets or any(value <= 0.0 for value in offsets):
            raise ValueError("Y difference offsets must be positive")

        _, _, measure_z = self._current_position()
        negative_points = []
        positive_points = []
        vy_x = float(self.profile.y_axis_pixel_x_per_mm)
        vy_y = float(self.profile.y_axis_pixel_y_per_mm)
        try:
            for side, output in ((-1.0, negative_points), (1.0, positive_points)):
                for offset in offsets:
                    delta_y = side * offset
                    self._abs_move([center.x, center.y + delta_y, measure_z])
                    output.append(
                        self._detect_at(
                            self.profile.center_pixel_x + vy_x * delta_y,
                            self.profile.center_pixel_y + vy_y * delta_y,
                            tool,
                        )
                    )
            difference = self._calculate_y_center_difference(
                negative_points,
                positive_points,
            )
        finally:
            self._abs_move([center.x, center.y, measure_z])

        self.y_center_difference = difference
        result = (measure_z, difference)
        if tool == ToolType.MAIN:
            self.main_y_center_difference = result
        else:
            self.secondary_y_center_differences.append(result)
        return difference

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

    cmd_XYZ_OFFSET_CALIBRATE_XY_help = (
        "Calibrate signed XY image vectors with two Y moves and one X move"
    )

    def cmd_XYZ_OFFSET_CALIBRATE_XY(self, gcmd):
        try:
            profile = self.calibrate_xy()
        except Exception as exc:
            self.last_error = str(exc)
            raise gcmd.error("XYZ_OFFSET_CALIBRATE_XY failed: %s" % (exc,))
        gcmd.respond_info(
            "XYZ_OFFSET_CALIBRATE_XY ok=True "
            "x_axis_maps_to_height=%d pixels_per_mm=%.4f "
            "vx=(%.4f,%.4f) vy=(%.4f,%.4f)"
            % (
                1 if profile.x_axis_maps_to_height else 0,
                profile.pixels_per_mm,
                profile.x_axis_pixel_x_per_mm,
                profile.x_axis_pixel_y_per_mm,
                profile.y_axis_pixel_x_per_mm,
                profile.y_axis_pixel_y_per_mm,
            )
        )

    cmd_XYZ_OFFSET_CENTER_help = (
        "Center the current nozzle. Params: MODE=1|2"
    )

    def cmd_XYZ_OFFSET_CENTER(self, gcmd):
        mode = str(gcmd.get("MODE", "1")).strip().upper()
        try:
            point = self._center_by_mode(mode)
            self._save_tool_center(point)
        except Exception as exc:
            self.last_error = str(exc)
            raise gcmd.error("XYZ_OFFSET_CENTER failed: %s" % (exc,))
        gcmd.respond_info(
            "XYZ_OFFSET_CENTER ok=True mode=%d iterations=%d "
            "machine=(%.4f,%.4f,%.4f) pixel=(%.3f,%.3f)"
            % (
                self.center_mode,
                self.center_iterations,
                point.x,
                point.y,
                point.z,
                point.pixel_x,
                point.pixel_y,
            )
        )

    cmd_XYZ_OFFSET_CALIBRATE_SECONDARY_help = (
        "Switch to and center the secondary tool. Params: MODE=1|2"
    )

    def cmd_XYZ_OFFSET_CALIBRATE_SECONDARY(self, gcmd):
        mode = str(gcmd.get("MODE", "1")).strip().upper()
        try:
            point = self.calibrate_secondary(mode)
        except Exception as exc:
            self.last_error = str(exc)
            raise gcmd.error(
                "XYZ_OFFSET_CALIBRATE_SECONDARY failed: %s" % (exc,)
            )
        gcmd.respond_info(
            "XYZ_OFFSET_CALIBRATE_SECONDARY ok=True mode=%d "
            "machine=(%.4f,%.4f,%.4f) xy_offset=(%.4f,%.4f)"
            % (
                self.center_mode,
                point.x,
                point.y,
                point.z,
                self.xy_offset_x,
                self.xy_offset_y,
            )
        )

    cmd_XYZ_OFFSET_MEASURE_Y_DIFF_help = (
        "Measure robust pixel difference at symmetric Y points. "
        "Params: TOOL=MAIN|SECOND COUNT=3..5"
    )

    def cmd_XYZ_OFFSET_MEASURE_Y_DIFF(self, gcmd):
        try:
            tool = _parse_tool(gcmd.get("TOOL", self.current_tool.value))
            count = gcmd.get_int(
                "COUNT",
                len(Y_CENTER_DIFFERENCE_OFFSETS),
                minval=3,
                maxval=len(Y_CENTER_DIFFERENCE_OFFSETS),
            )
            difference = self.measure_y_center_difference(
                tool,
                Y_CENTER_DIFFERENCE_OFFSETS[:count],
            )
            _, _, measure_z = self._current_position()
        except Exception as exc:
            self.last_error = str(exc)
            raise gcmd.error(
                "XYZ_OFFSET_MEASURE_Y_DIFF failed: %s" % (exc,)
            )
        gcmd.respond_info(
            "XYZ_OFFSET_MEASURE_Y_DIFF ok=True tool=%s z=%.4f "
            "count_per_side=%d center_difference_px=%.4f"
            % (tool.value, measure_z, count, difference)
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
        if profile.pixels_per_mm is not None:
            gcmd.respond_info(
                "XYZ_OFFSET_STATUS xy x_axis_maps_to_height=%d "
                "pixels_per_mm=%.4f"
                % (
                    1 if profile.x_axis_maps_to_height else 0,
                    profile.pixels_per_mm,
                )
            )
            gcmd.respond_info(
                "XYZ_OFFSET_STATUS vectors vx=(%.4f,%.4f) vy=(%.4f,%.4f)"
                % (
                    profile.x_axis_pixel_x_per_mm,
                    profile.x_axis_pixel_y_per_mm,
                    profile.y_axis_pixel_x_per_mm,
                    profile.y_axis_pixel_y_per_mm,
                )
            )
        if self.center_point is not None:
            gcmd.respond_info(
                "XYZ_OFFSET_STATUS center mode=%d iterations=%d "
                "machine=(%.4f,%.4f,%.4f) pixel=(%.3f,%.3f)"
                % (
                    self.center_mode,
                    self.center_iterations,
                    self.center_point.x,
                    self.center_point.y,
                    self.center_point.z,
                    self.center_point.pixel_x,
                    self.center_point.pixel_y,
                )
            )
            if self.center_mode == 2:
                gcmd.respond_info(
                    "XYZ_OFFSET_STATUS five_point average_pixel=(%.3f,%.3f)"
                    % (
                        self.center_average_pixel_x,
                        self.center_average_pixel_y,
                    )
                )
        if self.xy_offset_x is not None and self.xy_offset_y is not None:
            gcmd.respond_info(
                "XYZ_OFFSET_STATUS tool_offset secondary_minus_main=(%.4f,%.4f)"
                % (self.xy_offset_x, self.xy_offset_y)
            )
        if self.y_center_difference is not None:
            gcmd.respond_info(
                "XYZ_OFFSET_STATUS y_center_difference=%.4f"
                % (self.y_center_difference,)
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
                "x_axis_maps_to_height": profile.x_axis_maps_to_height,
                "pixels_per_mm": profile.pixels_per_mm,
                "x_axis_pixel_x_per_mm": profile.x_axis_pixel_x_per_mm,
                "x_axis_pixel_y_per_mm": profile.x_axis_pixel_y_per_mm,
                "y_axis_pixel_x_per_mm": profile.y_axis_pixel_x_per_mm,
                "y_axis_pixel_y_per_mm": profile.y_axis_pixel_y_per_mm,
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
            "xy_points": [
                {
                    "x": item.x,
                    "y": item.y,
                    "z": item.z,
                    "tool": item.tool.value,
                    "pixel_x": item.pixel_x,
                    "pixel_y": item.pixel_y,
                    "detected": item.detected,
                }
                for item in self.xy_points
            ],
            "center_point": None if self.center_point is None else {
                "x": self.center_point.x,
                "y": self.center_point.y,
                "z": self.center_point.z,
                "tool": self.center_point.tool.value,
                "pixel_x": self.center_point.pixel_x,
                "pixel_y": self.center_point.pixel_y,
                "detected": self.center_point.detected,
            },
            "center_iterations": self.center_iterations,
            "center_mode": self.center_mode,
            "center_average_pixel_x": self.center_average_pixel_x,
            "center_average_pixel_y": self.center_average_pixel_y,
            "center_points": [
                {
                    "x": item.x,
                    "y": item.y,
                    "z": item.z,
                    "tool": item.tool.value,
                    "pixel_x": item.pixel_x,
                    "pixel_y": item.pixel_y,
                    "detected": item.detected,
                }
                for item in self.center_points
            ],
            "center_sample_points": [
                {
                    "x": item.x,
                    "y": item.y,
                    "z": item.z,
                    "tool": item.tool.value,
                    "pixel_x": item.pixel_x,
                    "pixel_y": item.pixel_y,
                    "detected": item.detected,
                }
                for item in self.center_sample_points
            ],
            "tool_center_points": {
                tool.value: {
                    "x": item.x,
                    "y": item.y,
                    "z": item.z,
                    "pixel_x": item.pixel_x,
                    "pixel_y": item.pixel_y,
                    "detected": item.detected,
                }
                for tool, item in self.tool_center_points.items()
            },
            "xy_offset_x": self.xy_offset_x,
            "xy_offset_y": self.xy_offset_y,
            "y_center_difference": self.y_center_difference,
            "main_y_center_difference": self.main_y_center_difference,
            "secondary_y_center_differences": list(
                self.secondary_y_center_differences
            ),
            "last_error": self.last_error,
        }


def load_config(config):
    return XYZOffsetCalibrator(config)
