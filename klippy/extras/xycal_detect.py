# XY nozzle detect service hosted inside Klipper (Phase 1 + Phase 2).
#
# Phase 1: background HTTP :18765 (same contract as xycal_detect_server).
# Phase 2: ScreenQML can drive Detect via XYCAL_DETECT GCode and read
#          results from printer object status (Moonraker objects/query).
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
        self._service = None
        self._server = None
        self._http_thread = None
        self._last_result = {}
        self._import_error = None
        self._busy = False
        self._detect_seq = 0

        try:
            import detect_api as api  # noqa: WPS433 — path inserted above
            self._api = api
            self._service = api.NozzleDetectionService(
                self.snapshot_url, min_confidence=self.min_confidence
            )
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
        # Phase 2 short alias (same as XYCAL_DETECT_STATUS)
        self.gcode.register_command(
            "XYCAL_STATUS",
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
        if self._service is None:
            self._service = self._api.NozzleDetectionService(
                self.snapshot_url, min_confidence=self.min_confidence
            )
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

    def reset_tracker(self):
        if self._service is not None:
            self._service.reset()

    def detect_once(self, body):
        """Run one detect for other extras (xycal_calib). Updates last result/seq."""
        if self._api is None:
            return {
                "ok": False,
                "error": "DETECT_UNAVAILABLE",
                "detail": self._import_error or "import failed",
                "cx_px": -1,
                "cy_px": -1,
                "radius_px": 0,
                "confidence": 0.0,
            }
        if not isinstance(body, dict):
            body = {}
        eventtime = self.reactor.monotonic()
        result_box = []
        was_busy = self._busy
        self._busy = True

        def _work():
            try:
                kwargs = self._api._parse_body_detect_args(body, self.snapshot_url)
                svc = self._service
                if svc is None:
                    svc = self._api.get_shared_service(
                        kwargs.get("url") or self.snapshot_url,
                        kwargs.get("min_confidence"),
                    )
                result_box.append(svc.detect(**kwargs))
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
        result = {"ok": False, "error": "NO_RESULT"}
        try:
            th.start()
            while th.is_alive():
                eventtime = self.reactor.pause(eventtime + 0.05)
            th.join(timeout=0.1)
            result = result_box[0] if result_box else {"ok": False, "error": "NO_RESULT"}
            self._last_result = result
            self._detect_seq += 1
        finally:
            self._busy = was_busy
        return result

    cmd_XYCAL_DETECT_help = (
        "Run one nozzle tip detect (OpenCV worker). "
        "Params: URL= RESET_FOLLOW= FLUSH= FRESH= MODE= EXPECTED_X/Y= "
        "MIN_CONF= MIN_RADIUS= MAX_RADIUS= SEARCH_W= SEARCH_H= "
        "SEARCH_DX= SEARCH_DY="
    )

    def cmd_XYCAL_DETECT(self, gcmd):
        if self._api is None:
            raise gcmd.error(
                "xycal_detect unavailable: %s (install opencv-python-headless)"
                % (self._import_error or "import failed")
            )
        url = gcmd.get("URL", self.snapshot_url)
        reset_follow = gcmd.get_int("RESET_FOLLOW", 0)
        flush_n = gcmd.get_int("FLUSH", 0, minval=0, maxval=8)
        fresh = gcmd.get_int("FRESH", 0, minval=0, maxval=1)
        mode = gcmd.get("MODE", None)
        expect_x = gcmd.get_float("EXPECTED_X", None)
        expect_y = gcmd.get_float("EXPECTED_Y", None)
        min_conf = gcmd.get_float(
            "MIN_CONF", self.min_confidence, above=0.0, maxval=1.0
        )
        search_w = gcmd.get_float("SEARCH_W", 80.0, above=8.0)
        search_h = gcmd.get_float("SEARCH_H", 72.0, above=8.0)
        search_dx = gcmd.get_float("SEARCH_DX", None, above=0.0)
        search_dy = gcmd.get_float("SEARCH_DY", None, above=0.0)
        min_radius = gcmd.get_float("MIN_RADIUS", None, above=0.0)
        max_radius = gcmd.get_float("MAX_RADIUS", None, above=0.0)
        body = {
            "url": url,
            "min_confidence": min_conf,
            "search_width_px": search_w,
            "search_height_px": search_h,
            "flush_snapshot_count": flush_n,
            "fresh_frame": bool(fresh),
            "reset_follow": bool(reset_follow),
            "insecure": True,
        }
        if mode:
            body["mode"] = str(mode).strip().lower()
        if search_dx is not None and search_dy is not None:
            body["search_delta_x"] = search_dx
            body["search_delta_y"] = search_dy
        if min_radius is not None:
            body["min_radius_px"] = min_radius
        if max_radius is not None:
            body["max_radius_px"] = max_radius
        if expect_x is not None and expect_y is not None:
            body["expected_x"] = expect_x
            body["expected_y"] = expect_y
        elif expect_x is not None or expect_y is not None:
            raise gcmd.error("EXPECTED_X and EXPECTED_Y must be set together")
        if (search_dx is None) != (search_dy is None):
            raise gcmd.error("SEARCH_DX and SEARCH_DY must be set together")

        self._busy = True
        try:
            result = self.detect_once(body)
        finally:
            self._busy = False
        ok = bool(result.get("ok"))
        cx = result.get("cx_px", -1)
        cy = result.get("cy_px", -1)
        r = result.get("radius_px", 0)
        conf = result.get("confidence", 0)
        gcmd.respond_info(
            "XYCAL_DETECT ok=%s cx=%.1f cy=%.1f r=%.1f conf=%.3f seq=%d err=%s"
            % (
                ok,
                float(cx) if cx is not None else -1.0,
                float(cy) if cy is not None else -1.0,
                float(r) if r is not None else 0.0,
                float(conf) if conf is not None else 0.0,
                self._detect_seq,
                result.get("error") or result.get("reject_reason") or "",
            )
        )

    cmd_XYCAL_DETECT_STATUS_help = (
        "Report last XYCAL_DETECT result and HTTP bind (alias: XYCAL_STATUS)"
    )

    def cmd_XYCAL_DETECT_STATUS(self, gcmd):
        api_ok = self._api is not None
        http_on = self._server is not None
        r = self._last_result or {}
        gcmd.respond_info(
            "xycal_detect api=%s busy=%s seq=%s http=%s://%s:%s "
            "last_ok=%s last_cx=%s last_cy=%s last_r=%s conf=%s err=%s"
            % (
                api_ok,
                self._busy,
                self._detect_seq,
                "http" if http_on else "off",
                self.listen_host,
                self.listen_port,
                r.get("ok"),
                r.get("cx_px"),
                r.get("cy_px"),
                r.get("radius_px"),
                r.get("confidence"),
                r.get("error") or r.get("reject_reason") or "",
            )
        )
        if self._import_error:
            gcmd.respond_info("import_error: %s" % self._import_error)

    def _status_list(self, value):
        if value is None:
            return None
        if isinstance(value, (list, tuple)):
            out = []
            for item in value:
                try:
                    out.append(float(item))
                except (TypeError, ValueError):
                    return None
            return out
        return None

    def get_status(self, eventtime=None):
        r = self._last_result or {}
        # Moonraker objects/query：同时提供 last_* 与 HTTP 同名字段，方便屏端映射
        return {
            "http_enabled": bool(self.enable_http and self.listen_port > 0),
            "listen_host": self.listen_host,
            "listen_port": self.listen_port,
            "snapshot_url": self.snapshot_url,
            "api_ready": self._api is not None,
            "import_error": self._import_error or "",
            "busy": bool(self._busy),
            "detect_seq": int(self._detect_seq),
            "last_ok": bool(r.get("ok")),
            "last_cx": r.get("cx_px", -1),
            "last_cy": r.get("cy_px", -1),
            "last_radius": r.get("radius_px", 0),
            "last_confidence": r.get("confidence", 0.0),
            "last_frame_w": r.get("frame_w", 0),
            "last_frame_h": r.get("frame_h", 0),
            "last_error": r.get("error") or "",
            "last_detail": r.get("detail") or "",
            "last_reject_reason": r.get("reject_reason") or "",
            "last_allowed_roi": self._status_list(r.get("allowed_roi")),
            "last_allowed_circle": self._status_list(r.get("allowed_circle")),
            # HTTP-shaped aliases (Phase 2 UI)
            "ok": bool(r.get("ok")),
            "cx_px": r.get("cx_px", -1),
            "cy_px": r.get("cy_px", -1),
            "radius_px": r.get("radius_px", 0),
            "confidence": r.get("confidence", 0.0),
            "frame_w": r.get("frame_w", 0),
            "frame_h": r.get("frame_h", 0),
            "error": r.get("error") or "",
            "detail": r.get("detail") or "",
            "reject_reason": r.get("reject_reason") or "",
            "allowed_roi": self._status_list(r.get("allowed_roi")),
            "allowed_circle": self._status_list(r.get("allowed_circle")),
            "algorithm": getattr(self._api, "ALGORITHM_VERSION", "")
            if self._api
            else "",
            "service_version": getattr(self._api, "SERVICE_VERSION", "")
            if self._api
            else "",
        }


def load_config(config):
    return XyCalDetect(config)
