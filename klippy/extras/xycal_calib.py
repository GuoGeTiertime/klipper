# XY calib orchestration on Klipper host (Phase 3 step 3: Center).
#
# Macro = motion (UI_MOVE). Python = probe matrix + px→mm correct loop.
# Screen still uses its own Center until step 4 wires the button here.
#
# Copyright (C) 2026 TierTime / ScreenQML migration
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import math


class XyCalCalib:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.probe_mm = config.getfloat("probe_mm", 0.5, above=0.1, maxval=3.0)
        self.tol_px = config.getfloat("tol_px", 0.5, above=0.0)
        self.max_iter = config.getint("max_iter", 12, minval=1, maxval=40)
        self.min_move_px = config.getfloat("min_move_px", 2.5, above=0.5)
        self.jog_speed = config.getint("jog_speed", 30, minval=1, maxval=200)
        self.min_confidence = config.getfloat(
            "min_confidence", 0.35, above=0.0, maxval=1.0
        )
        self.snapshot_url = config.get("snapshot_url", "")

        self._busy = False
        self._cancel = False
        self._phase = "idle"
        self._message = ""
        self._error = ""
        self._last_cx = -1.0
        self._last_cy = -1.0
        self._last_r = 0.0
        self._vx_x = 0.0
        self._vx_y = 0.0
        self._vy_x = 0.0
        self._vy_y = 0.0
        self._matrix_ok = False

        self.gcode.register_command(
            "XYCAL_CENTER", self.cmd_XYCAL_CENTER, desc=self.cmd_XYCAL_CENTER_help
        )
        self.gcode.register_command(
            "XYCAL_CANCEL", self.cmd_XYCAL_CANCEL, desc=self.cmd_XYCAL_CANCEL_help
        )
        self.gcode.register_command(
            "XYCAL_CALIB_STATUS",
            self.cmd_XYCAL_CALIB_STATUS,
            desc=self.cmd_XYCAL_CALIB_STATUS_help,
        )

    def _detect_obj(self):
        return self.printer.lookup_object("xycal_detect", None)

    def _snap_url(self, gcmd):
        url = gcmd.get("URL", None)
        if url:
            return url
        if self.snapshot_url:
            return self.snapshot_url
        det = self._detect_obj()
        if det is not None:
            return det.snapshot_url
        return "http://127.0.0.1/webcam2/?action=snapshot"

    def _set_phase(self, phase, msg=""):
        self._phase = phase
        if msg:
            self._message = msg

    def _check_cancel(self, gcmd):
        if self._cancel:
            raise gcmd.error("XYCAL_CENTER cancelled")

    def _ui_move(self, axis, length_mm):
        axis = str(axis).upper()
        script = "UI_MOVE %s=1 LEN=%.3f SPEED=%d" % (
            axis,
            float(length_mm),
            int(self.jog_speed),
        )
        self.gcode.run_script_from_command(script)

    def _detect(self, gcmd, reset_follow=False, flush=0, expected=None):
        det = self._detect_obj()
        if det is None:
            raise gcmd.error("xycal_detect not loaded; include [xycal_detect] first")
        body = {
            "url": self._snap_url(gcmd),
            "min_confidence": self.min_confidence,
            "search_width_px": 80,
            "search_height_px": 72,
            "flush_snapshot_count": int(flush),
            "reset_follow": bool(reset_follow),
            "insecure": True,
        }
        if expected is not None:
            body["expected_x"] = float(expected[0])
            body["expected_y"] = float(expected[1])
        result = det.detect_once(body)
        self._last_cx = float(result.get("cx_px", -1) or -1)
        self._last_cy = float(result.get("cy_px", -1) or -1)
        self._last_r = float(result.get("radius_px", 0) or 0)
        return result

    def _looks_like_tip(self, result):
        if not result or not result.get("ok"):
            return False
        fw = float(result.get("frame_w") or 0)
        fh = float(result.get("frame_h") or 0)
        r = float(result.get("radius_px") or 0)
        cx = float(result.get("cx_px", -1))
        cy = float(result.get("cy_px", -1))
        if fw < 40 or fh < 40 or r <= 0 or cx < 0 or cy < 0:
            return False
        return r <= 0.055 * min(fw, fh)

    def _matrix_ok_vals(self, vxx, vxy, vyx, vyy):
        ax = math.sqrt(vxx * vxx + vxy * vxy)
        ay = math.sqrt(vyx * vyx + vyy * vyy)
        det = vxx * vyy - vyx * vxy
        return (
            math.isfinite(det)
            and abs(det) >= 5.0
            and ax >= 4.0
            and ay >= 4.0
            and ax <= 200.0
            and ay <= 200.0
        )

    def _apply_probe(self, axis, cx0, cy0, cx1, cy1, length_mm):
        dcx = cx1 - cx0
        dcy = cy1 - cy0
        main = max(abs(dcx), abs(dcy))
        if main < self.min_move_px:
            return False, "Δpx=%.1f too small" % main
        L = float(length_mm)
        if L < 0.001:
            return False, "bad probe length"
        if axis == "x":
            self._vx_x = dcx / L
            self._vx_y = dcy / L
        else:
            self._vy_x = dcx / L
            self._vy_y = dcy / L
        return True, ""

    def _correct_step(self, result):
        cx = float(result.get("cx_px"))
        cy = float(result.get("cy_px"))
        fw = float(result.get("frame_w") or 0)
        fh = float(result.get("frame_h") or 0)
        if fw < 40 or fh < 40:
            return None, "bad frame"
        ex = cx - fw * 0.5
        ey = cy - fh * 0.5
        err_main = max(abs(ex), abs(ey))
        if err_main <= self.tol_px:
            return (0.0, 0.0, err_main, True), ""
        det = self._vx_x * self._vy_y - self._vy_x * self._vx_y
        d_x = (-ex * self._vy_y + ey * self._vy_x) / det
        d_y = (ex * self._vx_y - ey * self._vx_x) / det
        if not (math.isfinite(d_x) and math.isfinite(d_y)):
            return None, "bad solve"
        if abs(d_x) < 0.01 and abs(d_y) < 0.01:
            return None, "stuck e=%.1f,%.1f" % (ex, ey)
        return (d_x, d_y, err_main, False), ""

    cmd_XYCAL_CENTER_help = (
        "Host Center: probe X/Y matrix, undo, fine correct. "
        "Params: URL= PROBE_MM= TOL_PX= MAX_ITER="
    )

    def cmd_XYCAL_CENTER(self, gcmd):
        if self._busy:
            raise gcmd.error("xycal_calib busy (%s)" % self._phase)
        toolhead = self.printer.lookup_object("toolhead")
        if "xy" not in toolhead.get_status(self.reactor.monotonic()).get(
            "homed_axes", ""
        ):
            # homed_axes may be 'xyz' string
            st = toolhead.get_status(self.reactor.monotonic())
            homed = str(st.get("homed_axes", ""))
            if "x" not in homed or "y" not in homed:
                raise gcmd.error("XYCAL_CENTER: home XY first")

        probe_mm = gcmd.get_float("PROBE_MM", self.probe_mm, above=0.1, maxval=3.0)
        # short bootstrap like screen when cold
        probe_mm = max(0.35, min(0.6, float(probe_mm)))
        tol_px = gcmd.get_float("TOL_PX", self.tol_px, above=0.0)
        max_iter = gcmd.get_int("MAX_ITER", self.max_iter, minval=1, maxval=40)

        self._busy = True
        self._cancel = False
        self._error = ""
        self._matrix_ok = False
        try:
            self._set_phase("need_tip", "need tip")
            gcmd.respond_info("XYCAL_CENTER: detect tip…")
            tip = self._detect(gcmd, reset_follow=True, flush=1)
            self._check_cancel(gcmd)
            if not self._looks_like_tip(tip):
                raise gcmd.error(
                    "XYCAL_CENTER: no tip (Detect tip in view first) err=%s"
                    % (tip.get("error") or tip.get("reject_reason") or "")
                )

            cx0 = float(tip["cx_px"])
            cy0 = float(tip["cy_px"])

            self._set_phase("probe_x", "probe +X")
            gcmd.respond_info("XYCAL_CENTER: probe X +%.2f" % probe_mm)
            self._ui_move("X", probe_mm)
            self._check_cancel(gcmd)
            tip_x = self._detect(gcmd, flush=2)
            if not self._looks_like_tip(tip_x):
                raise gcmd.error("XYCAL_CENTER: lost tip after +X")
            ok, detail = self._apply_probe(
                "x", cx0, cy0, float(tip_x["cx_px"]), float(tip_x["cy_px"]), probe_mm
            )
            if not ok:
                raise gcmd.error("XYCAL_CENTER Probe X: %s" % detail)
            cx0 = float(tip_x["cx_px"])
            cy0 = float(tip_x["cy_px"])

            self._set_phase("probe_y", "probe +Y")
            gcmd.respond_info("XYCAL_CENTER: probe Y +%.2f" % probe_mm)
            self._ui_move("Y", probe_mm)
            self._check_cancel(gcmd)
            tip_y = self._detect(gcmd, flush=2)
            if not self._looks_like_tip(tip_y):
                raise gcmd.error("XYCAL_CENTER: lost tip after +Y")
            ok, detail = self._apply_probe(
                "y", cx0, cy0, float(tip_y["cx_px"]), float(tip_y["cy_px"]), probe_mm
            )
            if not ok:
                raise gcmd.error("XYCAL_CENTER Probe Y: %s" % detail)

            if not self._matrix_ok_vals(
                self._vx_x, self._vx_y, self._vy_x, self._vy_y
            ):
                raise gcmd.error("XYCAL_CENTER: probe matrix invalid (det/scale)")
            self._matrix_ok = True

            self._set_phase("probe_undo", "undo probe")
            gcmd.respond_info("XYCAL_CENTER: undo probe")
            self._ui_move("X", -probe_mm)
            self._ui_move("Y", -probe_mm)
            self._check_cancel(gcmd)
            # 回退后 tip 已回起点附近；必须清 follow，否则仍在 Probe 偏移窗里搜 → lost tip
            tip = self._detect(gcmd, reset_follow=True, flush=3)
            if not self._looks_like_tip(tip):
                tip = self._detect(gcmd, reset_follow=True, flush=2)
            if not self._looks_like_tip(tip):
                raise gcmd.error(
                    "XYCAL_CENTER: lost tip after undo (%s)"
                    % (tip.get("error") or tip.get("reject_reason") or tip.get("detail") or "no tip")
                )

            last_err = None
            saved_tol = self.tol_px
            self.tol_px = tol_px
            try:
                for i in range(max_iter):
                    self._check_cancel(gcmd)
                    self._set_phase("correct", "correct %d/%d" % (i + 1, max_iter))
                    step, detail = self._correct_step(tip)
                    if step is None:
                        raise gcmd.error("XYCAL_CENTER: %s" % detail)
                    d_x, d_y, err_main, done = step
                    if done:
                        self._set_phase("done", "Centered e=%.2f" % err_main)
                        gcmd.respond_info(
                            "XYCAL_CENTER ok=True cx=%.1f cy=%.1f e=%.2f"
                            % (self._last_cx, self._last_cy, err_main)
                        )
                        return
                    if last_err is not None and err_main > last_err + max(
                        8.0, 0.25 * last_err
                    ):
                        raise gcmd.error(
                            "XYCAL_CENTER: dir wrong (e grew %.1f→%.1f)"
                            % (last_err, err_main)
                        )
                    last_err = err_main
                    gcmd.respond_info(
                        "XYCAL_CENTER correct dX=%.3f dY=%.3f e=%.1f"
                        % (d_x, d_y, err_main)
                    )
                    if abs(d_x) >= 0.01:
                        self._ui_move("X", d_x)
                    if abs(d_y) >= 0.01:
                        self._ui_move("Y", d_y)
                    tip = self._detect(
                        gcmd,
                        flush=2,
                        expected=(self._last_cx, self._last_cy),
                    )
                    if not self._looks_like_tip(tip):
                        tip = self._detect(gcmd, flush=1, reset_follow=False)
                    if not self._looks_like_tip(tip):
                        raise gcmd.error("XYCAL_CENTER: lost tip during correct")

                raise gcmd.error(
                    "XYCAL_CENTER: not within tol after %d iters (last e~%.1f)"
                    % (max_iter, last_err if last_err is not None else -1)
                )
            finally:
                self.tol_px = saved_tol
        except Exception as exc:
            self._error = str(exc)
            self._set_phase("error", self._error)
            logging.exception("XYCAL_CENTER failed")
            raise
        finally:
            self._busy = False
            if self._phase not in ("done", "error"):
                self._phase = "idle"

    cmd_XYCAL_CANCEL_help = "Cancel in-progress XYCAL_CENTER (checked between steps)"

    def cmd_XYCAL_CANCEL(self, gcmd):
        self._cancel = True
        gcmd.respond_info("XYCAL_CANCEL requested")

    cmd_XYCAL_CALIB_STATUS_help = "Report xycal_calib phase / last tip"

    def cmd_XYCAL_CALIB_STATUS(self, gcmd):
        gcmd.respond_info(
            "xycal_calib busy=%s phase=%s matrix=%s cx=%.1f cy=%.1f msg=%s err=%s"
            % (
                self._busy,
                self._phase,
                self._matrix_ok,
                self._last_cx,
                self._last_cy,
                self._message,
                self._error,
            )
        )

    def get_status(self, eventtime=None):
        return {
            "busy": bool(self._busy),
            "phase": self._phase,
            "message": self._message,
            "error": self._error,
            "matrix_ok": bool(self._matrix_ok),
            "cx_px": self._last_cx,
            "cy_px": self._last_cy,
            "radius_px": self._last_r,
            "vx_x": self._vx_x,
            "vx_y": self._vx_y,
            "vy_x": self._vy_x,
            "vy_y": self._vy_y,
        }


def load_config(config):
    return XyCalCalib(config)
