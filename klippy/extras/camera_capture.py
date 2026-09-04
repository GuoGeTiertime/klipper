# Generic HTTP camera snapshot capture for Klipper.
#
# Copyright (C) 2026 TierTime
# This file may be distributed under the terms of the GNU GPLv3 license.

import time
import urllib.error
import urllib.parse
import urllib.request

import cv2
import numpy as np


class CameraCaptureError(Exception):
    pass


class CameraCapture:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.url = config.get("url")
        self.timeout = config.getfloat("timeout", 15.0, above=0.0)

        self.gcode.register_command(
            "CAMERA_CAPTURE",
            self.cmd_CAMERA_CAPTURE,
            desc=self.cmd_CAMERA_CAPTURE_help,
        )

    def capture(self, url=None):
        """Fetch and decode a camera snapshot as an OpenCV BGR image."""
        snapshot_url = str(url or self.url).strip()
        parsed = urllib.parse.urlsplit(snapshot_url)
        if parsed.scheme not in ("http", "https") or not parsed.netloc:
            raise CameraCaptureError(
                "camera URL must be a valid http:// or https:// URL"
            )

        separator = "&" if parsed.query else "?"
        request_url = "%s%s_camera_ts=%d" % (
            snapshot_url,
            separator,
            time.time_ns(),
        )
        request = urllib.request.Request(
            request_url,
            headers={
                "User-Agent": "Klipper-CameraCapture/1.0",
                "Cache-Control": "no-cache",
                "Pragma": "no-cache",
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                payload = response.read()
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise CameraCaptureError("failed to fetch camera image: %s" % (exc,))

        if not payload:
            raise CameraCaptureError("camera returned an empty response")
        image = cv2.imdecode(
            np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            raise CameraCaptureError("camera response is not a valid image")
        return image

    cmd_CAMERA_CAPTURE_help = (
        "Capture one camera image and report its dimensions. Params: URL="
    )

    def cmd_CAMERA_CAPTURE(self, gcmd):
        url = gcmd.get("URL", None)
        started = time.monotonic()
        try:
            image = self.capture(url)
        except CameraCaptureError as exc:
            raise gcmd.error("CAMERA_CAPTURE failed: %s" % (exc,))

        height, width = image.shape[:2]
        channels = 1
        if image.ndim >= 3:
            channels = image.shape[2]
        elapsed_ms = (time.monotonic() - started) * 1000.0
        gcmd.respond_info(
            "CAMERA_CAPTURE ok=True width=%d height=%d channels=%d "
            "dtype=%s elapsed_ms=%.1f"
            % (width, height, channels, image.dtype, elapsed_ms)
        )


def load_config(config):
    return CameraCapture(config)
