# XY nozzle detect service hosted inside Klipper (Phase 1).
#
# Runs OpenCV detection in a background HTTP thread so the main reactor
# is not blocked. ScreenQML can keep posting to :18765; Fluidd can call
# XYCAL_DETECT for a one-shot tip report.
#
# Copyright (C) 2026 TierTime / ScreenQML migration
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import os
import sys
import threading

XYCAL_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "xycal")
if XYCAL_DIR not in sys.path:
    sys.path.insert(0, XYCAL_DIR)


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

        self._api = None
        self._server = None
        self._http_thread = None
        self._last_result = {}
        self._import_error = None

        try:
            import detect_api as api  # noqa: WPS433 — path inserted above
            self._api = api
        except Exception as exc:
            self._import_error = str(exc)
            logging.exception("xycal_detect: failed to import detect_api")

        self.gcode.register_command(
            "XYCAL_DETECT",
            self.cmd_XYCAL_DETECT,
            desc=self.cmd_XYCAL_DETECT_help,
        )
        self.gcode.register_command(
            "XYCAL_DETECT_STATUS",
            self.cmd_XYCAL_DETECT_STATUS,
            desc=self.cmd_XYCAL_DETECT_STATUS_help,
        )
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)

    def _handle_ready(self):
        if self._api is None:
            logging.error(
                "xycal_detect: OpenCV/detect_api unavailable: %s",
                self._import_error,
            )
            return
        if not self.enable_http or self.listen_port <= 0:
            logging.info(
                "xycal_detect: HTTP disabled (enable_http=%s listen_port=%s); "
                "XYCAL_DETECT still available",
                self.enable_http,
                self.listen_port,
            )
            return
        try:
            self._server = self._api.create_http_server(
                self.listen_host, self.listen_port, self.snapshot_url
            )
        except Exception:
            logging.exception(
                "xycal_detect: cannot bind %s:%s (stop old xycal_detect_server first)",
                self.listen_host,
                self.listen_port,
            )
            self._server = None
            return

        def _serve():
            logging.info(
                "xycal_detect: HTTP on http://%s:%s snapshot=%s",
                self.listen_host,
                self.listen_port,
                self.snapshot_url,
            )
            try:
                self._server.serve_forever()
            except Exception:
                logging.exception("xycal_detect: HTTP server stopped with error")

        self._http_thread = threading.Thread(
            target=_serve, name="xycal_detect_http", daemon=True
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
            logging.exception("xycal_detect: HTTP shutdown failed")
        try:
            server.server_close()
        except Exception:
            pass

    cmd_XYCAL_DETECT_help = "Run one nozzle tip detect (OpenCV worker)"

    def cmd_XYCAL_DETECT(self, gcmd):
        if self._api is None:
            raise gcmd.error(
                "xycal_detect unavailable: %s (install opencv-python-headless)"
                % (self._import_error or "import failed")
            )
        url = gcmd.get("URL", self.snapshot_url)
        reset_follow = gcmd.get_int("RESET_FOLLOW", 0)
        flush_n = gcmd.get_int("FLUSH", 0, minval=0, maxval=8)
        expect_x = gcmd.get_float("EXPECTED_X", None)
        expect_y = gcmd.get_float("EXPECTED_Y", None)
        body = {
            "url": url,
            "min_confidence": self.min_confidence,
            "search_width_px": 80,
            "search_height_px": 72,
            "flush_snapshot_count": flush_n,
            "reset_follow": bool(reset_follow),
            "insecure": True,
        }
        if expect_x is not None and expect_y is not None:
            body["expected_x"] = expect_x
            body["expected_y"] = expect_y
        elif expect_x is not None or expect_y is not None:
            raise gcmd.error("EXPECTED_X and EXPECTED_Y must be set together")

        # Detect runs in worker path of detect_api (HTTP fetch + OpenCV).
        # Pause reactor briefly so other events can run if detect is slow.
        eventtime = self.reactor.monotonic()
        result_box = []

        def _work():
            try:
                result_box.append(self._api.run_detect(body, self.snapshot_url))
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

        th = threading.Thread(target=_work, name="xycal_detect_once")
        th.start()
        while th.is_alive():
            eventtime = self.reactor.pause(eventtime + 0.05)
        th.join(timeout=0.1)
        result = result_box[0] if result_box else {"ok": False, "error": "NO_RESULT"}
        self._last_result = result
        ok = bool(result.get("ok"))
        cx = result.get("cx_px", -1)
        cy = result.get("cy_px", -1)
        r = result.get("radius_px", 0)
        conf = result.get("confidence", 0)
        gcmd.respond_info(
            "XYCAL_DETECT ok=%s cx=%.1f cy=%.1f r=%.1f conf=%.3f err=%s"
            % (
                ok,
                float(cx) if cx is not None else -1.0,
                float(cy) if cy is not None else -1.0,
                float(r) if r is not None else 0.0,
                float(conf) if conf is not None else 0.0,
                result.get("error") or result.get("reject_reason") or "",
            )
        )

    cmd_XYCAL_DETECT_STATUS_help = "Report last XYCAL_DETECT result and HTTP bind"

    def cmd_XYCAL_DETECT_STATUS(self, gcmd):
        api_ok = self._api is not None
        http_on = self._server is not None
        gcmd.respond_info(
            "xycal_detect api=%s http=%s://%s:%s last_ok=%s last_cx=%s last_cy=%s"
            % (
                api_ok,
                "http" if http_on else "off",
                self.listen_host,
                self.listen_port,
                self._last_result.get("ok"),
                self._last_result.get("cx_px"),
                self._last_result.get("cy_px"),
            )
        )
        if self._import_error:
            gcmd.respond_info("import_error: %s" % self._import_error)

    def get_status(self, eventtime=None):
        r = self._last_result or {}
        return {
            "http_enabled": bool(self.enable_http and self.listen_port > 0),
            "listen_host": self.listen_host,
            "listen_port": self.listen_port,
            "snapshot_url": self.snapshot_url,
            "api_ready": self._api is not None,
            "import_error": self._import_error or "",
            "last_ok": bool(r.get("ok")),
            "last_cx": r.get("cx_px", -1),
            "last_cy": r.get("cy_px", -1),
            "last_radius": r.get("radius_px", 0),
            "last_confidence": r.get("confidence", 0.0),
            "algorithm": getattr(self._api, "ALGORITHM_VERSION", "")
            if self._api
            else "",
            "service_version": getattr(self._api, "SERVICE_VERSION", "")
            if self._api
            else "",
        }


def load_config(config):
    return XyCalDetect(config)
