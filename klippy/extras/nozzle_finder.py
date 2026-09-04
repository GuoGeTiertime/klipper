# Nozzle finder based on a configured camera region.
#
# Copyright (C) 2026 TierTime
# This file may be distributed under the terms of the GNU GPLv3 license.

import math

import cv2
import numpy as np


class NozzleFinderError(Exception):
    pass


class NozzleFinder:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.center_x = config.getfloat("center_x", minval=0.0)
        self.center_y = config.getfloat("center_y", minval=0.0)
        self.region_width = config.getfloat("region_width", above=0.0)
        self.region_height = config.getfloat("region_height", above=0.0)
        self.expected_diameter = config.getfloat(
            "expected_diameter", above=0.0
        )

        self.gcode.register_command(
            "NOZZLE_FIND_CONFIG",
            self.cmd_NOZZLE_FIND_CONFIG,
            desc=self.cmd_NOZZLE_FIND_CONFIG_help,
        )
        self.gcode.register_command(
            "NOZZLE_FIND",
            self.cmd_NOZZLE_FIND,
            desc=self.cmd_NOZZLE_FIND_help,
        )

    def configure(
        self,
        center_x,
        center_y,
        region_width,
        region_height,
        expected_diameter,
    ):
        """Set the image search region and expected nozzle diameter in pixels."""
        values = (
            float(center_x),
            float(center_y),
            float(region_width),
            float(region_height),
            float(expected_diameter),
        )
        if values[0] < 0.0 or values[1] < 0.0:
            raise NozzleFinderError("search center must not be negative")
        if values[2] <= 0.0 or values[3] <= 0.0:
            raise NozzleFinderError("search region size must be positive")
        if values[4] <= 0.0:
            raise NozzleFinderError("expected nozzle diameter must be positive")
        (
            self.center_x,
            self.center_y,
            self.region_width,
            self.region_height,
            self.expected_diameter,
        ) = values

    def _crop_region(self, image):
        image_height, image_width = image.shape[:2]
        half_width = 0.5 * self.region_width
        half_height = 0.5 * self.region_height
        x0 = max(0, int(math.floor(self.center_x - half_width)))
        y0 = max(0, int(math.floor(self.center_y - half_height)))
        x1 = min(image_width, int(math.ceil(self.center_x + half_width)))
        y1 = min(image_height, int(math.ceil(self.center_y + half_height)))
        if x1 - x0 < 16 or y1 - y0 < 16:
            raise NozzleFinderError("configured search region is outside the image")
        return image[y0:y1, x0:x1], x0, y0

    def _circle_features(self, gray, x, y, radius):
        """Measure full-rim support and radial dark-cavity topology."""
        height, width = gray.shape
        sample_count = 144
        angles = np.linspace(0.0, 2.0 * math.pi, sample_count, endpoint=False)

        def sample_ring(scale):
            xs = np.clip(
                np.rint(x + radius * scale * np.cos(angles)).astype(np.int32),
                0,
                width - 1,
            )
            ys = np.clip(
                np.rint(y + radius * scale * np.sin(angles)).astype(np.int32),
                0,
                height - 1,
            )
            return gray[ys, xs].astype(np.float32)

        core = sample_ring(0.42)
        inner = sample_ring(0.72)
        edge_inside = sample_ring(0.90)
        edge_outside = sample_ring(1.10)
        outer = sample_ring(1.32)
        edge_delta = np.abs(edge_outside - edge_inside)
        edge_reference = max(6.0, float(np.percentile(edge_delta, 55.0)))
        edge_coverage = float(np.mean(edge_delta >= edge_reference))
        half = sample_count // 2
        opposed = np.minimum(edge_delta[:half], edge_delta[half:])
        opposed_coverage = float(np.mean(opposed >= edge_reference))
        sectors = np.array_split(edge_delta, 12)
        sector_coverage = float(
            np.mean([np.mean(sector) >= 0.65 * edge_reference for sector in sectors])
        )
        contrast = float(abs(np.median(outer) - np.median(inner)))
        core_contrast = float(np.median(outer) - np.median(core))
        score = (
            0.045 * float(np.median(edge_delta))
            + 2.0 * edge_coverage
            + 2.2 * opposed_coverage
            + 1.4 * sector_coverage
            + 0.018 * contrast
            + 0.012 * max(0.0, core_contrast)
        )
        return {
            "score": score,
            "edge_coverage": edge_coverage,
            "opposed_coverage": opposed_coverage,
            "sector_coverage": sector_coverage,
            "contrast": contrast,
            "core_contrast": core_contrast,
        }

    def _refine_circle(self, gray, initial):
        """Locally fit centre/radius using the complete radial rim."""
        x, y, radius = initial
        best = None
        center_steps = (-3.0, -1.5, 0.0, 1.5, 3.0)
        radius_steps = np.linspace(0.86, 1.14, 9)
        for dy in center_steps:
            for dx in center_steps:
                for scale in radius_steps:
                    trial_radius = radius * float(scale)
                    features = self._circle_features(
                        gray, x + dx, y + dy, trial_radius
                    )
                    rank = features["score"]
                    if best is None or rank > best[0]:
                        best = (
                            rank,
                            x + dx,
                            y + dy,
                            trial_radius,
                            features,
                        )
        return best[1], best[2], best[3], best[4]

    def _metal_face_support(self, gray, x, y, radius):
        """Check that the rim is locally nested in a larger metal face."""
        blurred = cv2.GaussianBlur(gray, (0, 0), 1.0)
        edges = cv2.Canny(blurred, 30, 100)
        contours, _ = cv2.findContours(
            edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )
        best = 0.0
        point = (float(x), float(y))
        for contour in contours:
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue
            polygon = cv2.approxPolyDP(contour, 0.045 * perimeter, True)
            if len(polygon) < 3 or len(polygon) > 8:
                continue
            if cv2.pointPolygonTest(polygon, point, False) < 0:
                continue
            area = float(abs(cv2.contourArea(polygon)))
            circle_area = math.pi * radius * radius
            ratio = area / max(1.0, circle_area)
            if 2.0 <= ratio <= 22.0:
                vertex_score = 1.0 - min(1.0, abs(len(polygon) - 6) / 4.0)
                best = max(best, 0.5 + 0.5 * vertex_score)
        return best

    def _find_candidates(self, gray):
        expected_radius = 0.5 * self.expected_diameter
        min_radius = max(3, int(round(expected_radius * 0.60)))
        max_radius = max(min_radius + 2, int(round(expected_radius * 1.40)))
        blurred = cv2.GaussianBlur(gray, (0, 0), 1.2)
        candidates = []

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.0,
            minDist=max(8.0, expected_radius),
            param1=80.0,
            param2=14.0,
            minRadius=min_radius,
            maxRadius=max_radius,
        )
        if circles is not None:
            for x, y, radius in circles[0]:
                candidates.append((float(x), float(y), float(radius)))

        edges = cv2.Canny(blurred, 35, 105)
        contours, _ = cv2.findContours(
            edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
        )
        for contour in contours:
            if len(contour) < 12:
                continue
            area = float(abs(cv2.contourArea(contour)))
            perimeter = float(cv2.arcLength(contour, True))
            if area <= 0.0 or perimeter <= 0.0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < 0.45:
                continue
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if min_radius <= radius <= max_radius:
                candidates.append((float(x), float(y), float(radius)))
        return candidates

    def execute(self, url=None):
        """Capture an image and return the best nozzle circle."""
        camera = self.printer.lookup_object("camera_capture", None)
        if camera is None:
            return {
                "ok": False,
                "error": "CAMERA_NOT_LOADED",
                "detail": "camera_capture is not loaded",
            }
        try:
            image = camera.capture(url)
        except Exception as exc:
            return {
                "ok": False,
                "error": "CAPTURE_FAILED",
                "detail": str(exc),
            }

        try:
            region, offset_x, offset_y = self._crop_region(image)
        except NozzleFinderError as exc:
            return {
                "ok": False,
                "error": "INVALID_SEARCH_REGION",
                "detail": str(exc),
            }
        gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
        candidates = self._find_candidates(gray)
        if not candidates:
            return {
                "ok": False,
                "error": "NO_CANDIDATE",
                "detail": "no nozzle circle found in search region",
            }

        region_center_x = self.center_x - offset_x
        region_center_y = self.center_y - offset_y
        expected_radius = 0.5 * self.expected_diameter
        refined = []
        for candidate in candidates:
            refined.append(self._refine_circle(gray, candidate))

        # Collapse near-identical Hough/contour seeds into one circle family.
        families = []
        for x, y, radius, features in refined:
            existing = None
            for family in families:
                if (
                    math.hypot(x - family[0], y - family[1])
                    <= 0.30 * max(radius, family[2])
                    and abs(radius - family[2]) <= 0.30 * max(radius, family[2])
                ):
                    existing = family
                    break
            if existing is None:
                families.append([x, y, radius, features])
            elif features["score"] > existing[3]["score"]:
                existing[:] = [x, y, radius, features]

        best = None
        for x, y, radius, features in families:
            position_error = math.hypot(
                x - region_center_x, y - region_center_y
            )
            radius_error = abs(radius - expected_radius) / expected_radius
            face_support = self._metal_face_support(gray, x, y, radius)
            score = (
                features["score"]
                + 1.4 * face_support
                - 0.055 * position_error
                - 1.8 * radius_error
            )
            if best is None or score > best[0]:
                best = (score, x, y, radius, features, face_support)

        score, x, y, radius, features, face_support = best
        if (
            features["opposed_coverage"] < 0.10
            or features["sector_coverage"] < 0.50
            or features["score"] < 4.0
        ):
            return {
                "ok": False,
                "error": "NO_RELIABLE_NOZZLE",
                "detail": "no reliable nozzle circle found",
            }
        return {
            "ok": True,
            "error": "",
            "cx": float(x + offset_x),
            "cy": float(y + offset_y),
            "radius": float(radius),
            "score": float(score),
            "candidate_count": len(candidates),
            "family_count": len(families),
            "edge_coverage": float(features["edge_coverage"]),
            "face_support": float(face_support),
            "frame_width": int(image.shape[1]),
            "frame_height": int(image.shape[0]),
        }

    cmd_NOZZLE_FIND_CONFIG_help = (
        "Configure nozzle search. Params: CENTER_X= CENTER_Y= WIDTH= HEIGHT= "
        "DIAMETER="
    )

    def cmd_NOZZLE_FIND_CONFIG(self, gcmd):
        self.configure(
            gcmd.get_float("CENTER_X", self.center_x, minval=0.0),
            gcmd.get_float("CENTER_Y", self.center_y, minval=0.0),
            gcmd.get_float("WIDTH", self.region_width, above=0.0),
            gcmd.get_float("HEIGHT", self.region_height, above=0.0),
            gcmd.get_float("DIAMETER", self.expected_diameter, above=0.0),
        )
        gcmd.respond_info(
            "NOZZLE_FIND_CONFIG center=(%.1f,%.1f) region=%.1fx%.1f "
            "diameter=%.1f"
            % (
                self.center_x,
                self.center_y,
                self.region_width,
                self.region_height,
                self.expected_diameter,
            )
        )

    cmd_NOZZLE_FIND_help = "Capture an image and locate the nozzle. Params: URL="

    def cmd_NOZZLE_FIND(self, gcmd):
        result = self.execute(gcmd.get("URL", None))
        if not result.get("ok"):
            gcmd.respond_info(
                "NOZZLE_FIND ok=False error=%s detail=%s"
                % (result.get("error", "UNKNOWN"), result.get("detail", ""))
            )
            return
        gcmd.respond_info(
            "NOZZLE_FIND ok=True cx=%.2f cy=%.2f radius=%.2f "
            "score=%.2f candidates=%d frame=%dx%d"
            % (
                result["cx"],
                result["cy"],
                result["radius"],
                result["score"],
                result["candidate_count"],
                result["frame_width"],
                result["frame_height"],
            )
        )


def load_config(config):
    return NozzleFinder(config)
