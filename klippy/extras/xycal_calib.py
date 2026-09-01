# XY calib orchestration on Klipper host (Center + FitY).
#
# Macro = motion (UI_MOVE). Python = probe matrix / FitY scan / px→mm correct.
# Screen wires buttons in later steps (center_backend / fity_backend).
#
# Copyright (C) 2026 TierTime / ScreenQML migration
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import math


# Same Y offsets as ScreenQML fitYOffsetsMm
FITY_OFFSETS_MM = (
    4.3, 4.4, 4.5, 4.6, 4.7,
    -4.3, -4.4, -4.5, -4.6, -4.7,
)


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
        self.span_mm = config.getfloat("span_mm", 4.5, above=1.0, maxval=8.0)
        self.snapshot_url = config.get("snapshot_url", "")
        self.resid_floor_px = config.getfloat("resid_floor_px", 8.0, above=0.0)
        self.resid_sigma_k = config.getfloat("resid_sigma_k", 2.5, above=0.0)
        self.zgap_axis = config.get("zgap_axis", "Y").upper()
        self.zgap_shift_mm = config.getfloat("zgap_shift_mm", 5.0, above=0.1)
        self.zgap_z_plus_mm = config.getfloat("zgap_z_plus_mm", 5.0, above=0.1)
        self.zgap_z_step_mm = config.getfloat("zgap_z_step_mm", 0.05, above=0.01)
        self.zgap_match_tol_px = config.getfloat(
            "zgap_match_tol_px", 1.0, above=0.0
        )
        self.zgap_dr_tol_px = config.getfloat("zgap_dr_tol_px", 2.0, above=0.0)
        self.zgap_max_iter = config.getint("zgap_max_iter", 120, minval=1, maxval=300)
        self.z_min_mm = config.getfloat("z_min_mm", 60.0, above=0.0)
        self.dcx_t1_fallback = config.getfloat("dcx_t1_fallback", 3.8)

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
        self._fity_vy_x = 0.0
        self._fity_vy_y = 0.0
        self._fity_vy_ok = False
        self._fity_index = 0
        self._fity_prev_mm = 0.0
        self._fity_samples = []
        self._fity_report = ""
        self._fity_dcx45 = None
        self._fity_dcx45_raw = None
        self._fity_tip0_cx = -1.0
        self._fity_tip0_cy = -1.0
        self._fity_span_pos_cx = None
        self._fity_span_pos_cy = None
        self._ghost_cx = -1.0
        self._ghost_cy = -1.0
        self._ghost_r = 0.0
        self._z_calib_z0 = None
        self._z_offset_dz = None
        self._dcx_main_dcx45 = None
        self._dcx_solved_offset = None
        self._dcx_report = ""
        self._dcx_rows = []
        self._auto_main_x = None
        self._auto_main_y = None
        self._auto_main_z = None
        self._auto_sec_x = None
        self._auto_sec_y = None
        self._auto_sec_z = None
        self._auto_ox = None
        self._auto_oy = None

        self.gcode.register_command(
            "XYCAL_CENTER", self.cmd_XYCAL_CENTER, desc=self.cmd_XYCAL_CENTER_help
        )
        self.gcode.register_command(
            "XYCAL_FITY", self.cmd_XYCAL_FITY, desc=self.cmd_XYCAL_FITY_help
        )
        self.gcode.register_command(
            "XYCAL_CANCEL", self.cmd_XYCAL_CANCEL, desc=self.cmd_XYCAL_CANCEL_help
        )
        self.gcode.register_command(
            "XYCAL_Z", self.cmd_XYCAL_Z, desc=self.cmd_XYCAL_Z_help
        )
        self.gcode.register_command(
            "XYCAL_DCXZ", self.cmd_XYCAL_DCXZ, desc=self.cmd_XYCAL_DCXZ_help
        )
        self.gcode.register_command(
            "XYCAL_AUTO", self.cmd_XYCAL_AUTO, desc=self.cmd_XYCAL_AUTO_help
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

    def _check_cancel(self, gcmd, label="XYCAL"):
        if self._cancel:
            raise gcmd.error("%s cancelled" % label)

    def _require_xy_homed(self, gcmd, label):
        toolhead = self.printer.lookup_object("toolhead")
        st = toolhead.get_status(self.reactor.monotonic())
        homed = str(st.get("homed_axes", ""))
        if "x" not in homed or "y" not in homed:
            raise gcmd.error("%s: home XY first" % label)

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

    def _toolhead_xyz(self):
        toolhead = self.printer.lookup_object("toolhead")
        pos = toolhead.get_position()
        return float(pos[0]), float(pos[1]), float(pos[2])

    def _home_xy_if_needed(self, gcmd, label="XYCAL"):
        toolhead = self.printer.lookup_object("toolhead")
        st = toolhead.get_status(self.reactor.monotonic())
        homed = str(st.get("homed_axes", ""))
        if "x" in homed and "y" in homed:
            return
        self._set_phase("home", "homing XY")
        gcmd.respond_info("%s: G28 X Y" % label)
        self.gcode.run_script_from_command("G28 X Y")
        self._check_cancel(gcmd, label)

    def _xycal_goto(self, gcmd, site, x, y, z):
        site = str(site).upper()
        if site.startswith("SEC"):
            site = "SECOND"
        else:
            site = "MAIN"
        script = "XYCAL_GOTO SITE=%s X=%.3f Y=%.3f Z=%.3f" % (
            site,
            float(x),
            float(y),
            float(z),
        )
        self.gcode.run_script_from_command(script)
        self._check_cancel(gcmd)

    def _xycal_move_z(self, gcmd, z_abs):
        script = "XYCAL_MOVE_Z Z=%.3f" % float(z_abs)
        self.gcode.run_script_from_command(script)
        self._check_cancel(gcmd)

    def _read_save_var_float(self, name, default):
        sv = self.printer.lookup_object("save_variables", None)
        if sv is None:
            return float(default)
        val = sv.allVariables.get(name)
        if val is None:
            return float(default)
        try:
            return float(val)
        except (TypeError, ValueError):
            return float(default)

    def _read_t1_offset_z(self, gcmd, default=None):
        if default is None:
            default = self.dcx_t1_fallback
        if gcmd.get("T1_REF", None) is not None:
            return gcmd.get_float("T1_REF", default)
        return self._read_save_var_float("t1_offset_z", default)

    def _circle_match(self, ghost_cx, ghost_cy, ghost_r, result, tol_px, dr_tol):
        if not result or not self._looks_like_tip(result):
            return False
        cx = float(result.get("cx_px"))
        cy = float(result.get("cy_px"))
        r = float(result.get("radius_px") or 0)
        err = max(abs(cx - ghost_cx), abs(cy - ghost_cy))
        dr = abs(r - ghost_r) if ghost_r > 0 and r > 0 else 0.0
        return err <= float(tol_px) and dr <= float(dr_tol)

    def _save_tool1_offset_z(self, gcmd, dz, dual_init=False):
        dz_f = round(float(dz) * 100.0) / 100.0
        dz_str = "%.2f" % dz_f
        self.gcode.run_script_from_command(
            "SET_DUAL_TOOL_OFFSET TOOL=1 Z=%s" % dz_str
        )
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=t1_offset_z VALUE=%s" % dz_str
        )
        if dual_init:
            self.gcode.run_script_from_command("DUAL_TOOL_INIT")
            self.gcode.run_script_from_command("TOOL_STATUS")
        return dz_f

    def _save_tool1_offsets_xyz(self, gcmd, ox, oy, oz):
        ox_f = round(float(ox) * 100.0) / 100.0
        oy_f = round(float(oy) * 100.0) / 100.0
        oz_f = round(float(oz) * 100.0) / 100.0
        self.gcode.run_script_from_command(
            "SET_DUAL_TOOL_OFFSET TOOL=1 X=%.2f Y=%.2f Z=%.2f"
            % (ox_f, oy_f, oz_f)
        )
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=t1_offset_x VALUE=%.2f" % ox_f
        )
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=t1_offset_y VALUE=%.2f" % oy_f
        )
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=t1_offset_z VALUE=%.2f" % oz_f
        )
        self.gcode.run_script_from_command("DUAL_TOOL_INIT")
        self.gcode.run_script_from_command("TOOL_STATUS")
        return ox_f, oy_f, oz_f

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

    def _span_vector_ok(self, vx, vy, ref_x, ref_y):
        speed = math.sqrt(vx * vx + vy * vy)
        ref = math.sqrt(ref_x * ref_x + ref_y * ref_y)
        if not (math.isfinite(speed) and math.isfinite(ref)) or ref < 1e-6:
            return False
        cos_dir = (vx * ref_x + vy * ref_y) / max(1e-6, speed * ref)
        ratio = speed / max(1e-6, ref)
        return (
            speed >= 4.0
            and speed <= 100.0
            and cos_dir >= 0.80
            and ratio >= 0.55
            and ratio <= 1.80
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

    def _run_long_y_span(self, gcmd, tip):
        """After short probe undo: Y+span / Y-span / home → FitY vy."""
        span = float(self.span_mm)
        cx0 = float(tip["cx_px"])
        cy0 = float(tip["cy_px"])
        ref_x, ref_y = self._vy_x, self._vy_y

        self._set_phase("probe_span_pos", "span Y+%.1f" % span)
        gcmd.respond_info("XYCAL_CENTER: span Y +%.1f" % span)
        self._ui_move("Y", span)
        self._check_cancel(gcmd)
        tip_pos = self._detect(gcmd, flush=2, expected=(
            cx0 + span * ref_x, cy0 + span * ref_y
        ))
        if not self._looks_like_tip(tip_pos):
            tip_pos = self._detect(gcmd, flush=2)
        if not self._looks_like_tip(tip_pos):
            self._ui_move("Y", -span)
            raise gcmd.error("XYCAL_CENTER: no tip at Y+%.1f" % span)
        vx = (float(tip_pos["cx_px"]) - cx0) / span
        vy = (float(tip_pos["cy_px"]) - cy0) / span
        if not self._span_vector_ok(vx, vy, ref_x, ref_y):
            self._ui_move("Y", -span)
            raise gcmd.error("XYCAL_CENTER: +span vector conflicts with short probe")
        pos_cx = float(tip_pos["cx_px"])
        pos_cy = float(tip_pos["cy_px"])
        self._vy_x, self._vy_y = vx, vy

        self._set_phase("probe_span_neg", "span Y-%.1f" % span)
        gcmd.respond_info("XYCAL_CENTER: span Y +%.1f → -%.1f" % (span, span))
        self._ui_move("Y", -2.0 * span)
        self._check_cancel(gcmd)
        tip_neg = self._detect(gcmd, flush=2, expected=(
            pos_cx - 2.0 * span * self._vy_x,
            pos_cy - 2.0 * span * self._vy_y,
        ))
        if not self._looks_like_tip(tip_neg):
            tip_neg = self._detect(gcmd, flush=2)
        if not self._looks_like_tip(tip_neg):
            self._ui_move("Y", span)
            raise gcmd.error("XYCAL_CENTER: no tip at Y-%.1f" % span)
        vx = (pos_cx - float(tip_neg["cx_px"])) / (2.0 * span)
        vy = (pos_cy - float(tip_neg["cy_px"])) / (2.0 * span)
        if not self._span_vector_ok(vx, vy, self._vy_x, self._vy_y):
            self._ui_move("Y", span)
            raise gcmd.error("XYCAL_CENTER: ±span vector inconsistent")
        self._vy_x, self._vy_y = vx, vy

        self._set_phase("probe_span_home", "span return")
        gcmd.respond_info("XYCAL_CENTER: return from Y-%.1f" % span)
        self._ui_move("Y", span)
        self._check_cancel(gcmd)
        tip_home = self._detect(gcmd, reset_follow=True, flush=3)
        if not self._looks_like_tip(tip_home):
            tip_home = self._detect(gcmd, reset_follow=True, flush=2)
        if not self._looks_like_tip(tip_home):
            raise gcmd.error("XYCAL_CENTER: no tip after ±span return")
        home_err = max(
            abs(float(tip_home["cx_px"]) - cx0),
            abs(float(tip_home["cy_px"]) - cy0),
        )
        if not math.isfinite(home_err) or home_err > 4.0:
            raise gcmd.error(
                "XYCAL_CENTER: ±span return error %.1fpx" % home_err
            )
        self._fity_vy_x = self._vy_x
        self._fity_vy_y = self._vy_y
        self._fity_vy_ok = True
        gcmd.respond_info(
            "XYCAL_CENTER span vy=(%.3f,%.3f) px/mm homeErr=%.2f"
            % (self._fity_vy_x, self._fity_vy_y, home_err)
        )
        return tip_home

    cmd_XYCAL_CENTER_help = (
        "Host Center: probe X/Y, ±span FitY vy, fine correct. "
        "Params: URL= PROBE_MM= TOL_PX= MAX_ITER= SPAN_MM="
    )

    def cmd_XYCAL_CENTER(self, gcmd):
        if self._busy:
            raise gcmd.error("xycal_calib busy (%s)" % self._phase)
        self._require_xy_homed(gcmd, "XYCAL_CENTER")
        self._busy = True
        self._cancel = False
        self._error = ""
        self._matrix_ok = False
        self._fity_vy_ok = False
        try:
            self.run_center(gcmd)
        except Exception as exc:
            self._error = str(exc)
            self._set_phase("error", self._error)
            logging.exception("XYCAL_CENTER failed")
            raise
        finally:
            self._busy = False
            if self._phase not in ("done", "error"):
                self._phase = "idle"

    def run_center(self, gcmd):
        probe_mm = gcmd.get_float("PROBE_MM", self.probe_mm, above=0.1, maxval=3.0)
        probe_mm = max(0.35, min(0.6, float(probe_mm)))
        tol_px = gcmd.get_float("TOL_PX", self.tol_px, above=0.0)
        max_iter = gcmd.get_int("MAX_ITER", self.max_iter, minval=1, maxval=40)
        span_ov = gcmd.get_float("SPAN_MM", self.span_mm, above=1.0, maxval=8.0)
        saved_span = self.span_mm
        self.span_mm = float(span_ov)
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
            tip = self._detect(gcmd, reset_follow=True, flush=3)
            if not self._looks_like_tip(tip):
                tip = self._detect(gcmd, reset_follow=True, flush=2)
            if not self._looks_like_tip(tip):
                raise gcmd.error(
                    "XYCAL_CENTER: lost tip after undo (%s)"
                    % (tip.get("error") or tip.get("reject_reason") or tip.get("detail") or "no tip")
                )

            tip = self._run_long_y_span(gcmd, tip)

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
                            "XYCAL_CENTER ok=True cx=%.1f cy=%.1f e=%.2f fity_vy_ok=%s"
                            % (
                                self._last_cx,
                                self._last_cy,
                                err_main,
                                self._fity_vy_ok,
                            )
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
        finally:
            self.span_mm = saved_span

    # ---- FitY ----
    @staticmethod
    def _lin_fit_cx(pts):
        n = len(pts)
        if n < 2:
            return False, 0.0, 0.0
        sum_m = sum_c = sum_mm = sum_mc = 0.0
        for p in pts:
            m = float(p["mm"])
            c = float(p["cx"])
            sum_m += m
            sum_c += c
            sum_mm += m * m
            sum_mc += m * c
        den = n * sum_mm - sum_m * sum_m
        if abs(den) < 1e-9:
            return False, 0.0, 0.0
        b = (n * sum_mc - sum_m * sum_c) / den
        a = (sum_c - b * sum_m) / n
        return True, a, b

    def _arm_fit_reject(self, pts):
        dropped = []
        if not pts or len(pts) < 2:
            return False, 0.0, 0.0, [], dropped
        ok0, a0, b0 = self._lin_fit_cx(pts)
        if not ok0:
            return False, 0.0, 0.0, [], dropped
        resid = []
        for p in pts:
            pred = a0 + b0 * float(p["mm"])
            resid.append(abs(float(p["cx"]) - pred))
        sorted_r = sorted(resid)
        med = sorted_r[len(sorted_r) // 2]
        abs_dev = sorted(abs(r - med) for r in resid)
        mad = abs_dev[len(abs_dev) // 2]
        sigma = 1.4826 * mad
        if not (sigma > 0.5):
            sigma = med if med > 0 else 1.0
        thr = max(float(self.resid_floor_px), float(self.resid_sigma_k) * sigma, 18.0)
        kept = []
        for i, p in enumerate(pts):
            if resid[i] > thr:
                dropped.append(dict(p))
            else:
                kept.append(p)
        if len(kept) < 2:
            return False, 0.0, 0.0, kept, dropped
        ok1, a1, b1 = self._lin_fit_cx(kept)
        if not ok1:
            return False, 0.0, 0.0, kept, dropped
        return True, a1, b1, kept, dropped

    def _compute_dcx45(self, samples):
        pos = [s for s in samples if float(s["mm"]) > 0]
        neg = [s for s in samples if float(s["mm"]) < 0]
        pok, pa, pb, _, pd = self._arm_fit_reject(pos)
        nok, na, nb, _, nd = self._arm_fit_reject(neg)
        dropped = list(pd) + list(nd)
        if not pok or not nok:
            return False, None, dropped
        x_pos = pa + pb * 4.5
        x_neg = na + nb * (-4.5)
        return True, x_neg - x_pos, dropped

    @staticmethod
    def _raw_dcx45(samples):
        cx_pos = cx_neg = None
        for s in samples:
            mm = float(s["mm"])
            if abs(mm - 4.5) < 0.001:
                cx_pos = float(s["cx"])
            if abs(mm + 4.5) < 0.001:
                cx_neg = float(s["cx"])
        if cx_pos is None or cx_neg is None:
            return False, None
        return True, cx_neg - cx_pos

    def _rebuild_fity_report(self):
        lines = []
        for i, s in enumerate(self._fity_samples):
            mm = float(s["mm"])
            lines.append(
                "[%d] Y%s%.1fmm  (cx,cy)=(%.2f, %.2f) px"
                % (
                    i + 1,
                    "+" if mm >= 0 else "",
                    mm,
                    float(s["cx"]),
                    float(s["cy"]),
                )
            )
        if self._fity_dcx45 is not None:
            lines.append("=======")
            lines.append(
                "实测Δcx@±4.5  %.2f px  (cx(-4.5)-cx(+4.5))" % self._fity_dcx45
            )
        self._fity_report = "\n".join(lines)

    def _fity_estimate(self, mm):
        return (
            self._fity_tip0_cx + float(mm) * self._fity_vy_x,
            self._fity_tip0_cy + float(mm) * self._fity_vy_y,
        )

    def _fity_tip_far_from_est(self, mm, cx, cy, fw, fh):
        ex, ey = self._fity_estimate(mm)
        d_est = math.sqrt((cx - ex) ** 2 + (cy - ey) ** 2)
        if not (d_est > 40):
            return False
        mid_x, mid_y = fw * 0.5, fh * 0.5
        est_off = math.sqrt((ex - mid_x) ** 2 + (ey - mid_y) ** 2)
        tip_off = math.sqrt((cx - mid_x) ** 2 + (cy - mid_y) ** 2)
        return est_off >= 60 and tip_off < 0.5 * est_off

    def _fity_refine_at_span(self, mm, cx, cy):
        m = float(mm)
        if abs(abs(m) - 4.5) > 0.011:
            return True
        if m > 0:
            vx = (cx - self._fity_tip0_cx) / m
            vy = (cy - self._fity_tip0_cy) / m
            tag = "+4.5-long"
        else:
            if self._fity_span_pos_cx is None or self._fity_span_pos_cy is None:
                return False
            vx = (self._fity_span_pos_cx - cx) / 9.0
            vy = (self._fity_span_pos_cy - cy) / 9.0
            tag = "plus-minus-4.5"
        if not self._span_vector_ok(vx, vy, self._fity_vy_x, self._fity_vy_y):
            logging.warning(
                "FitY span vector rejected tag=%s v=(%s,%s)", tag, vx, vy
            )
            return False
        self._fity_vy_x, self._fity_vy_y = vx, vy
        self._fity_vy_ok = True
        if m > 0:
            self._fity_span_pos_cx = cx
            self._fity_span_pos_cy = cy
        logging.info(
            "FitY span calibrated tag=%s vy=(%.3f,%.3f)", tag, vx, vy
        )
        return True

    def _fity_home_y(self):
        back = -float(self._fity_prev_mm)
        if abs(back) >= 0.01:
            self._ui_move("Y", back)
            self._fity_prev_mm = 0.0

    cmd_XYCAL_FITY_help = (
        "Host FitY Y-scan ±4.3..±4.7. Needs prior Center span (fity_vy). "
        "Params: URL="
    )

    def cmd_XYCAL_FITY(self, gcmd):
        if self._busy:
            raise gcmd.error("xycal_calib busy (%s)" % self._phase)
        self._require_xy_homed(gcmd, "XYCAL_FITY")
        if not self._fity_vy_ok:
            raise gcmd.error(
                "XYCAL_FITY: run XYCAL_CENTER first (need ±span FitY vy)"
            )
        self._busy = True
        self._cancel = False
        self._error = ""
        try:
            self.run_fity(gcmd)
        except Exception as exc:
            try:
                self._fity_home_y()
            except Exception:
                logging.exception("XYCAL_FITY home after error")
            self._error = str(exc)
            self._set_phase("error", self._error)
            self._rebuild_fity_report()
            logging.exception("XYCAL_FITY failed")
            raise
        finally:
            self._busy = False
            if self._phase not in ("done", "error"):
                self._phase = "idle"

    def run_fity(self, gcmd):
        self._fity_samples = []
        self._fity_report = ""
        self._fity_dcx45 = None
        self._fity_dcx45_raw = None
        self._fity_index = 0
        self._fity_prev_mm = 0.0
        self._fity_span_pos_cx = None
        self._fity_span_pos_cy = None
        try:
            self._set_phase("fity_tip", "FitY need tip")
            tip = self._detect(gcmd, reset_follow=True, flush=2)
            self._check_cancel(gcmd, "XYCAL_FITY")
            if not self._looks_like_tip(tip):
                raise gcmd.error("XYCAL_FITY: fresh centered tip required")
            self._fity_tip0_cx = float(tip["cx_px"])
            self._fity_tip0_cy = float(tip["cy_px"])
            fw = float(tip.get("frame_w") or 640)
            fh = float(tip.get("frame_h") or 480)
            gcmd.respond_info(
                "XYCAL_FITY start tip=(%.1f,%.1f) vy=(%.3f,%.3f)"
                % (
                    self._fity_tip0_cx,
                    self._fity_tip0_cy,
                    self._fity_vy_x,
                    self._fity_vy_y,
                )
            )

            n_off = len(FITY_OFFSETS_MM)
            for idx, target in enumerate(FITY_OFFSETS_MM):
                self._check_cancel(gcmd, "XYCAL_FITY")
                self._fity_index = idx
                delta = float(target) - float(self._fity_prev_mm)
                self._set_phase(
                    "fity_move",
                    "Y%s%.1f (%d/%d)"
                    % ("+" if target >= 0 else "", target, idx + 1, n_off),
                )
                if abs(delta) >= 0.001:
                    self._ui_move("Y", delta)
                flush_n = 2 if abs(delta) >= 2.0 or abs(abs(target) - 4.3) < 0.05 else 1
                est = self._fity_estimate(target)
                tip_pt = None
                for attempt in range(6):
                    self._check_cancel(gcmd, "XYCAL_FITY")
                    tip_pt = self._detect(
                        gcmd, flush=flush_n if attempt == 0 else 1, expected=est
                    )
                    if not self._looks_like_tip(tip_pt):
                        tip_pt = self._detect(gcmd, flush=1)
                    if not self._looks_like_tip(tip_pt):
                        continue
                    cx = float(tip_pt["cx_px"])
                    cy = float(tip_pt["cy_px"])
                    fw = float(tip_pt.get("frame_w") or fw)
                    fh = float(tip_pt.get("frame_h") or fh)
                    if abs(target) >= 4.0 and (
                        abs(self._fity_prev_mm) < 0.01
                        or len(self._fity_samples) == 0
                    ):
                        if self._fity_tip_far_from_est(target, cx, cy, fw, fh):
                            flush_n = 2
                            continue
                    # adjacent stale: almost no motion vs last same-arm 0.1mm
                    if self._fity_samples:
                        prev = self._fity_samples[-1]
                        pmm = float(prev["mm"])
                        if (
                            target * pmm > 0
                            and abs(target - pmm) <= 0.15
                            and math.sqrt(
                                (cx - float(prev["cx"])) ** 2
                                + (cy - float(prev["cy"])) ** 2
                            )
                            < 1.5
                            and attempt < 1
                        ):
                            continue
                    break
                else:
                    self._fity_home_y()
                    raise gcmd.error(
                        "XYCAL_FITY: Y%s%.1f same ROI failed"
                        % ("+" if target >= 0 else "", target)
                    )

                cx = float(tip_pt["cx_px"])
                cy = float(tip_pt["cy_px"])
                r = float(tip_pt.get("radius_px") or 0)
                if not self._fity_refine_at_span(target, cx, cy):
                    self._fity_home_y()
                    raise gcmd.error(
                        "XYCAL_FITY: ±4.5 vector check failed at Y%s%.1f"
                        % ("+" if target >= 0 else "", target)
                    )
                self._fity_prev_mm = float(target)
                self._fity_samples.append(
                    {"mm": float(target), "cx": cx, "cy": cy, "r": r}
                )
                self._rebuild_fity_report()
                gcmd.respond_info(
                    "XYCAL_FITY [%d/%d] Y%s%.1f cx=%.1f cy=%.1f"
                    % (
                        idx + 1,
                        n_off,
                        "+" if target >= 0 else "",
                        target,
                        cx,
                        cy,
                    )
                )

            raw_ok, raw_dcx = self._raw_dcx45(self._fity_samples)
            fit_ok, fit_dcx, _dropped = self._compute_dcx45(self._fity_samples)
            self._fity_dcx45_raw = raw_dcx if raw_ok else None
            # Standalone FitY on screen uses measured ±4.5; expose that as primary
            self._fity_dcx45 = raw_dcx if raw_ok else (fit_dcx if fit_ok else None)
            self._rebuild_fity_report()
            self._fity_home_y()
            tip_end = self._detect(gcmd, reset_follow=True, flush=2)
            if self._looks_like_tip(tip_end):
                pass
            self._set_phase(
                "done",
                "FitY done n=%d dcx45=%s"
                % (
                    len(self._fity_samples),
                    ("%.2f" % self._fity_dcx45) if self._fity_dcx45 is not None else "?",
                ),
            )
            gcmd.respond_info(
                "XYCAL_FITY ok=True n=%d dcx45=%s fit=%s raw=%s"
                % (
                    len(self._fity_samples),
                    self._fity_dcx45,
                    fit_dcx if fit_ok else None,
                    raw_dcx if raw_ok else None,
                )
            )
        finally:
            pass

    cmd_XYCAL_Z_help = (
        "Host Z gap: main/second center, ghost match, Z descend. "
        "Params: URL= MAIN_X/Y/Z SEC_X/Y/Z SHIFT_AXIS SHIFT_MM "
        "Z_PLUS Z_STEP MATCH_TOL DR_TOL WRITE="
    )

    def cmd_XYCAL_Z(self, gcmd):
        if self._busy:
            raise gcmd.error("xycal_calib busy (%s)" % self._phase)
        self._require_xy_homed(gcmd, "XYCAL_Z")
        self._busy = True
        self._cancel = False
        self._error = ""
        self._ghost_cx = -1.0
        self._ghost_cy = -1.0
        self._ghost_r = 0.0
        self._z_calib_z0 = None
        self._z_offset_dz = None
        try:
            self._run_z_calib(gcmd)
        except Exception as exc:
            self._error = str(exc)
            self._set_phase("error", self._error)
            logging.exception("XYCAL_Z failed")
            raise
        finally:
            self._busy = False
            if self._phase not in ("done", "error"):
                self._phase = "idle"

    def _run_z_calib(self, gcmd):
        mx = gcmd.get_float("MAIN_X", None)
        my = gcmd.get_float("MAIN_Y", None)
        mz = gcmd.get_float("MAIN_Z", None)
        sx = gcmd.get_float("SEC_X", None)
        sy = gcmd.get_float("SEC_Y", None)
        sz = gcmd.get_float("SEC_Z", None)
        for name, val in (
            ("MAIN_X", mx),
            ("MAIN_Y", my),
            ("MAIN_Z", mz),
            ("SEC_X", sx),
            ("SEC_Y", sy),
            ("SEC_Z", sz),
        ):
            if val is None:
                raise gcmd.error("XYCAL_Z: need %s" % name)
        shift_axis = gcmd.get("SHIFT_AXIS", self.zgap_axis).upper()
        if shift_axis not in ("X", "Y"):
            shift_axis = "Y"
        shift_mm = gcmd.get_float("SHIFT_MM", self.zgap_shift_mm, above=0.1)
        z_plus = gcmd.get_float("Z_PLUS", self.zgap_z_plus_mm, above=0.1)
        z_step = gcmd.get_float("Z_STEP", self.zgap_z_step_mm, above=0.01)
        match_tol = gcmd.get_float("MATCH_TOL", self.zgap_match_tol_px, above=0.0)
        dr_tol = gcmd.get_float("DR_TOL", self.zgap_dr_tol_px, above=0.0)
        max_iter = gcmd.get_int("MAX_ITER", self.zgap_max_iter, minval=1, maxval=300)
        write = gcmd.get_int("WRITE", 0, minval=0, maxval=1) != 0

        self._set_phase("z_goto_main", "Z go main")
        gcmd.respond_info("XYCAL_Z: goto main")
        self._xycal_goto(gcmd, "MAIN", mx, my, mz)
        self._matrix_ok = False
        self._fity_vy_ok = False
        self.run_center(gcmd)
        self._check_cancel(gcmd, "XYCAL_Z")

        self._set_phase("z_shift_main", "Z shift main")
        self._ui_move(shift_axis, shift_mm)
        self._check_cancel(gcmd, "XYCAL_Z")
        tip = self._detect(gcmd, flush=2)
        if not self._looks_like_tip(tip):
            tip = self._detect(gcmd, flush=2, reset_follow=True)
        if not self._looks_like_tip(tip):
            raise gcmd.error("XYCAL_Z: no tip after main shift")
        self._ghost_cx = float(tip["cx_px"])
        self._ghost_cy = float(tip["cy_px"])
        self._ghost_r = float(tip.get("radius_px") or 8.0)
        _, _, z0 = self._toolhead_xyz()
        self._z_calib_z0 = float(z0)
        gcmd.respond_info(
            "XYCAL_Z: ghost (%.1f,%.1f) r=%.1f Z0=%.3f"
            % (self._ghost_cx, self._ghost_cy, self._ghost_r, self._z_calib_z0)
        )

        self._set_phase("z_goto_second", "Z go 2nd")
        self._xycal_goto(gcmd, "SECOND", sx, sy, sz)
        self._matrix_ok = False
        self._fity_vy_ok = False
        self.run_center(gcmd)
        self._check_cancel(gcmd, "XYCAL_Z")

        self._set_phase("z_shift_second", "Z shift 2nd")
        self._ui_move(shift_axis, shift_mm)
        self._check_cancel(gcmd, "XYCAL_Z")
        tip2 = self._detect(gcmd, flush=2)
        if not self._looks_like_tip(tip2):
            tip2 = self._detect(gcmd, flush=2, reset_follow=True)
        if not self._looks_like_tip(tip2):
            raise gcmd.error("XYCAL_Z: no tip after 2nd shift")

        if self._circle_match(
            self._ghost_cx, self._ghost_cy, self._ghost_r, tip2, match_tol, dr_tol
        ):
            _, _, z_now = self._toolhead_xyz()
            self._z_offset_dz = float(z_now) - float(self._z_calib_z0)
            gcmd.respond_info("XYCAL_Z: matched without Z adjust dz=%.3f" % self._z_offset_dz)
        else:
            target_z = float(self._z_calib_z0) + float(z_plus)
            self._set_phase("z_jump", "Z jump Z0+%.1f" % z_plus)
            self._xycal_move_z(gcmd, target_z)
            tip2 = self._detect(gcmd, flush=2)
            if not self._looks_like_tip(tip2):
                tip2 = self._detect(gcmd, flush=2, reset_follow=True)
            if self._circle_match(
                self._ghost_cx, self._ghost_cy, self._ghost_r, tip2, match_tol, dr_tol
            ):
                _, _, z_now = self._toolhead_xyz()
                self._z_offset_dz = float(z_now) - float(self._z_calib_z0)
            else:
                floor_z = float(self._z_calib_z0) - float(z_plus)
                for _ in range(max_iter):
                    self._check_cancel(gcmd, "XYCAL_Z")
                    _, _, z_now = self._toolhead_xyz()
                    if z_now <= floor_z + 0.001:
                        raise gcmd.error("XYCAL_Z: no match within Z0±%.1f" % z_plus)
                    step = float(z_step)
                    err = max(
                        abs(float(tip2.get("cx_px", 0)) - self._ghost_cx),
                        abs(float(tip2.get("cy_px", 0)) - self._ghost_cy),
                    )
                    if err > 20:
                        step = min(0.12, step * 2)
                    elif err > 8:
                        step = min(0.08, step * 1.5)
                    room = float(z_now) - floor_z
                    if step > room:
                        step = max(0.01, room)
                    self._set_phase("z_descend", "Z −%.2f e=%.1f" % (step, err))
                    self._ui_move("Z", -step)
                    tip2 = self._detect(gcmd, flush=2)
                    if not self._looks_like_tip(tip2):
                        tip2 = self._detect(gcmd, flush=1)
                    if self._circle_match(
                        self._ghost_cx,
                        self._ghost_cy,
                        self._ghost_r,
                        tip2,
                        match_tol,
                        dr_tol,
                    ):
                        _, _, z_now = self._toolhead_xyz()
                        self._z_offset_dz = float(z_now) - float(self._z_calib_z0)
                        break
                else:
                    raise gcmd.error("XYCAL_Z: match timeout")

        if self._z_offset_dz is None:
            raise gcmd.error("XYCAL_Z: dz not computed")
        dz = round(float(self._z_offset_dz) * 100.0) / 100.0
        self._z_offset_dz = dz
        if write:
            self._save_tool1_offset_z(gcmd, dz, dual_init=False)
        self._set_phase("done", "Z dz=%.2f" % dz)
        gcmd.respond_info("XYCAL_Z ok=True dz=%.2f write=%s" % (dz, write))

    @staticmethod
    def _dcx_zero_cross_off(pts):
        best_off = None
        best_cost = float("inf")
        for i in range(len(pts) - 1):
            o1 = float(pts[i]["off"])
            o2 = float(pts[i + 1]["off"])
            v1 = float(pts[i]["vs"])
            v2 = float(pts[i + 1]["vs"])
            if abs(o2 - o1) < 1e-9:
                continue
            if v1 == 0:
                return o1
            if v2 == 0:
                return o2
            if v1 * v2 > 0:
                continue
            off = o1 + (0.0 - v1) / (v2 - v1) * (o2 - o1)
            cost = max(abs(v1), abs(v2))
            if cost < best_cost:
                best_cost = cost
                best_off = off
        return best_off

    def _dcx_solve_offset(self, rows, main_dcx):
        pts = []
        for r in rows:
            if not r.get("ok"):
                continue
            dcx = r.get("dcx")
            if dcx is None:
                continue
            off = float(r["off"])
            vs = float(dcx) - float(main_dcx)
            pts.append({"off": off, "dcx": float(dcx), "vs": vs})
        if not pts:
            return None
        pts.sort(key=lambda p: p["off"])
        off_star = self._dcx_zero_cross_off(pts)
        if off_star is None:
            best = min(pts, key=lambda p: abs(p["vs"]))
            off_star = best["off"]
        return round(float(off_star) * 100.0) / 100.0

    def _dcx_rebuild_report(self, main_dcx, rows):
        lines = []
        lines.append("main dcx45=%.2f" % float(main_dcx))
        for r in rows:
            tag = "[%s]" % r.get("pass", "?")
            if r.get("ok"):
                lines.append(
                    "%s off=%+.2f z=%.2f dcx=%.2f"
                    % (
                        tag,
                        float(r["off"]),
                        float(r.get("z", 0)),
                        float(r["dcx"]),
                    )
                )
            else:
                lines.append(
                    "%s off=%+.2f FAIL %s"
                    % (tag, float(r["off"]), r.get("reason", ""))
                )
        if self._dcx_solved_offset is not None:
            lines.append("offset*=%.2f" % float(self._dcx_solved_offset))
        self._dcx_report = "\n".join(lines)

    def _run_dcxz_scan_pass(
        self, gcmd, main_x, main_y, main_z, sec_x, sec_y,
        off_hi, off_lo, step, pass_tag, rows, need_main_fity,
    ):
        z_min = gcmd.get_float("Z_MIN", self.z_min_mm, above=0.0)
        main_dcx = self._dcx_main_dcx45
        if need_main_fity:
            self._set_phase("dcx_main", "dcx main FitY")
            self._xycal_goto(gcmd, "MAIN", main_x, main_y, main_z)
            self._matrix_ok = False
            self._fity_vy_ok = False
            self.run_center(gcmd)
            self.run_fity(gcmd)
            main_dcx = self._fity_dcx45
            if main_dcx is None:
                fit_ok, fit_dcx, _ = self._compute_dcx45(self._fity_samples)
                main_dcx = fit_dcx if fit_ok else None
            if main_dcx is None:
                raise gcmd.error("XYCAL_DCXZ: main FitY dcx45 failed")
            self._dcx_main_dcx45 = float(main_dcx)
            gcmd.respond_info("XYCAL_DCXZ: main dcx45=%.2f" % self._dcx_main_dcx45)

        off = float(off_hi)
        off_lo = float(off_lo)
        step = float(step)
        while off >= off_lo - 1e-6:
            self._check_cancel(gcmd, "XYCAL_DCXZ")
            z_abs = round((float(main_z) + off) * 1000.0) / 1000.0
            if z_abs < z_min - 1e-6:
                rows.append(
                    {
                        "off": off,
                        "z": z_abs,
                        "ok": False,
                        "dcx": None,
                        "pass": pass_tag,
                        "reason": "z<min",
                    }
                )
                off = round((off - step) * 1000.0) / 1000.0
                continue
            self._set_phase(
                "dcx_scan",
                "dcx off=%+.2f z=%.2f" % (off, z_abs),
            )
            self._xycal_goto(gcmd, "SECOND", sec_x, sec_y, z_abs)
            self._matrix_ok = False
            self._fity_vy_ok = False
            try:
                self.run_center(gcmd)
                self.run_fity(gcmd)
                dcx = self._fity_dcx45
                if dcx is None:
                    fit_ok, fit_dcx, _ = self._compute_dcx45(self._fity_samples)
                    dcx = fit_dcx if fit_ok else None
                ok = dcx is not None
                rows.append(
                    {
                        "off": off,
                        "z": z_abs,
                        "ok": ok,
                        "dcx": float(dcx) if ok else None,
                        "pass": pass_tag,
                        "reason": "" if ok else "fity",
                    }
                )
                if ok:
                    gcmd.respond_info(
                        "XYCAL_DCXZ [%s] off=%+.2f dcx=%.2f vs=%.2f"
                        % (
                            pass_tag,
                            off,
                            float(dcx),
                            float(dcx) - float(self._dcx_main_dcx45),
                        )
                    )
            except Exception as exc:
                rows.append(
                    {
                        "off": off,
                        "z": z_abs,
                        "ok": False,
                        "dcx": None,
                        "pass": pass_tag,
                        "reason": str(exc),
                    }
                )
            off = round((off - step) * 1000.0) / 1000.0
        return self._dcx_main_dcx45

    cmd_XYCAL_DCXZ_help = (
        "Host ΔcxZ scan: main FitY + second Z sweep. "
        "Params: URL= MAIN_X/Y/Z SEC_X/Y T1_REF OFF_HI OFF_LO STEP "
        "Z_MIN AUTO WRITE="
    )

    def cmd_XYCAL_DCXZ(self, gcmd):
        if self._busy:
            raise gcmd.error("xycal_calib busy (%s)" % self._phase)
        self._require_xy_homed(gcmd, "XYCAL_DCXZ")
        self._busy = True
        self._cancel = False
        self._error = ""
        self._dcx_rows = []
        self._dcx_main_dcx45 = None
        self._dcx_solved_offset = None
        self._dcx_report = ""
        try:
            self._run_dcxz(gcmd)
        except Exception as exc:
            self._error = str(exc)
            self._set_phase("error", self._error)
            logging.exception("XYCAL_DCXZ failed")
            raise
        finally:
            self._busy = False
            if self._phase not in ("done", "error"):
                self._phase = "idle"

    def _run_dcxz(self, gcmd, write_override=None, auto_override=None):
        mx = gcmd.get_float("MAIN_X", None)
        my = gcmd.get_float("MAIN_Y", None)
        mz = gcmd.get_float("MAIN_Z", None)
        sx = gcmd.get_float("SEC_X", None)
        sy = gcmd.get_float("SEC_Y", None)
        for name, val in (
            ("MAIN_X", mx),
            ("MAIN_Y", my),
            ("MAIN_Z", mz),
            ("SEC_X", sx),
            ("SEC_Y", sy),
        ):
            if val is None:
                raise gcmd.error("XYCAL_DCXZ: need %s" % name)
        t1_ref = self._read_t1_offset_z(gcmd)
        if auto_override is not None:
            auto = bool(auto_override)
        else:
            auto = gcmd.get_int("AUTO", 0, minval=0, maxval=1) != 0
        if write_override is not None:
            write = bool(write_override)
        else:
            write = gcmd.get_int("WRITE", 0, minval=0, maxval=1) != 0
        z_min = gcmd.get_float("Z_MIN", self.z_min_mm, above=0.0)
        if mz < z_min - 1e-6:
            raise gcmd.error("XYCAL_DCXZ: MAIN_Z=%.2f < Z_MIN" % mz)

        rows = []
        if auto:
            off_hi = round((t1_ref + 1.0) * 1000.0) / 1000.0
            off_lo = round((t1_ref - 1.0) * 1000.0) / 1000.0
            step = 0.5
            gate_lo = round((t1_ref - 1.25) * 1000.0) / 1000.0
            gate_hi = round((t1_ref + 1.25) * 1000.0) / 1000.0
        else:
            off_hi = gcmd.get_float("OFF_HI", t1_ref + 1.0)
            off_lo = gcmd.get_float("OFF_LO", t1_ref - 1.0)
            step = gcmd.get_float("STEP", 0.5, above=0.001)
            gate_lo = off_lo
            gate_hi = off_hi
        if not (off_hi > off_lo and step > 0):
            raise gcmd.error("XYCAL_DCXZ: bad OFF_HI/OFF_LO/STEP")

        self._run_dcxz_scan_pass(
            gcmd, mx, my, mz, sx, sy, off_hi, off_lo, step, "coarse", rows, True
        )
        solved = self._dcx_solve_offset(rows, self._dcx_main_dcx45)
        if auto and solved is not None:
            approx = solved
            if approx > t1_ref + 1.0 + 0.01:
                approx = round((t1_ref + 1.0) * 1000.0) / 1000.0
            if approx < t1_ref - 1.0 - 0.01:
                approx = round((t1_ref - 1.0) * 1000.0) / 1000.0
            fine_hi = round((approx + 0.25) * 1000.0) / 1000.0
            fine_lo = round((approx - 0.25) * 1000.0) / 1000.0
            if (float(mz) + fine_lo) >= z_min - 1e-6:
                fine_rows = []
                self._run_dcxz_scan_pass(
                    gcmd,
                    mx,
                    my,
                    mz,
                    sx,
                    sy,
                    fine_hi,
                    fine_lo,
                    0.1,
                    "fine",
                    fine_rows,
                    False,
                )
                fine_sol = self._dcx_solve_offset(fine_rows, self._dcx_main_dcx45)
                if fine_sol is not None and len(fine_rows) >= 2:
                    rows = fine_rows
                    solved = fine_sol
                    gate_lo = fine_lo
                    gate_hi = fine_hi

        self._dcx_rows = list(rows)
        if solved is None:
            raise gcmd.error("XYCAL_DCXZ: solve failed")
        if solved < gate_lo - 0.01 or solved > gate_hi + 0.01:
            raise gcmd.error(
                "XYCAL_DCXZ: offset*=%.2f out of gate [%.2f..%.2f]"
                % (solved, gate_lo, gate_hi)
            )
        self._dcx_solved_offset = float(solved)
        self._dcx_rebuild_report(self._dcx_main_dcx45, rows)
        if write:
            self._save_tool1_offset_z(gcmd, solved, dual_init=True)
        self._set_phase("done", "dcx offset=%.2f" % solved)
        gcmd.respond_info(
            "XYCAL_DCXZ ok=True offset=%.2f main_dcx=%.2f write=%s"
            % (solved, self._dcx_main_dcx45, write)
        )

    cmd_XYCAL_AUTO_help = (
        "Host one-shot auto: home, center main/2nd, DCXZ, apply XY+Z. "
        "Params: URL= MAIN_X/Y/Z SEC_X/Y/Z WRITE=1 AUTO=1"
    )

    def cmd_XYCAL_AUTO(self, gcmd):
        if self._busy:
            raise gcmd.error("xycal_calib busy (%s)" % self._phase)
        self._busy = True
        self._cancel = False
        self._error = ""
        try:
            self._run_auto(gcmd)
        except Exception as exc:
            self._error = str(exc)
            self._set_phase("error", self._error)
            logging.exception("XYCAL_AUTO failed")
            raise
        finally:
            self._busy = False
            if self._phase not in ("done", "error"):
                self._phase = "idle"

    def _run_auto(self, gcmd):
        mx = gcmd.get_float("MAIN_X", None)
        my = gcmd.get_float("MAIN_Y", None)
        mz = gcmd.get_float("MAIN_Z", None)
        sx = gcmd.get_float("SEC_X", None)
        sy = gcmd.get_float("SEC_Y", None)
        sz = gcmd.get_float("SEC_Z", mz if mz is not None else 0.0)
        for name, val in (
            ("MAIN_X", mx),
            ("MAIN_Y", my),
            ("MAIN_Z", mz),
            ("SEC_X", sx),
            ("SEC_Y", sy),
            ("SEC_Z", sz),
        ):
            if val is None:
                raise gcmd.error("XYCAL_AUTO: need %s" % name)
        write = gcmd.get_int("WRITE", 1, minval=0, maxval=1) != 0

        self._home_xy_if_needed(gcmd, "XYCAL_AUTO")
        self._set_phase("auto_main", "auto main center")
        self._xycal_goto(gcmd, "MAIN", mx, my, mz)
        self._matrix_ok = False
        self._fity_vy_ok = False
        self.run_center(gcmd)
        mx_r, my_r, mz_r = self._toolhead_xyz()
        self._auto_main_x = mx_r
        self._auto_main_y = my_r
        self._auto_main_z = mz_r

        self._set_phase("auto_sec", "auto 2nd center")
        self._xycal_goto(gcmd, "SECOND", sx, sy, sz)
        self._matrix_ok = False
        self._fity_vy_ok = False
        self.run_center(gcmd)
        sx_r, sy_r, sz_r = self._toolhead_xyz()
        self._auto_sec_x = sx_r
        self._auto_sec_y = sy_r
        self._auto_sec_z = sz_r
        self._auto_ox = round((sx_r - mx_r) * 100.0) / 100.0
        self._auto_oy = round((sy_r - my_r) * 100.0) / 100.0

        self._dcx_rows = []
        self._dcx_main_dcx45 = None
        self._dcx_solved_offset = None
        self._set_phase("auto_dcxz", "auto DCXZ")
        self._run_dcxz(gcmd, write_override=False, auto_override=True)
        dz = self._dcx_solved_offset
        if dz is None:
            raise gcmd.error("XYCAL_AUTO: DCXZ solve failed")

        if write:
            self._save_tool1_offsets_xyz(gcmd, self._auto_ox, self._auto_oy, dz)
        self._set_phase("auto_return", "auto return main")
        self._xycal_goto(gcmd, "MAIN", mx, my, mz)
        self._set_phase(
            "done",
            "Auto X=%.2f Y=%.2f Z=%.2f" % (self._auto_ox, self._auto_oy, dz),
        )
        gcmd.respond_info(
            "XYCAL_AUTO ok=True ox=%.2f oy=%.2f oz=%.2f write=%s"
            % (self._auto_ox, self._auto_oy, dz, write)
        )

    cmd_XYCAL_CANCEL_help = (
        "Cancel in-progress XYCAL_CENTER/FITY/Z/DCXZ/AUTO"
    )

    def cmd_XYCAL_CANCEL(self, gcmd):
        self._cancel = True
        gcmd.respond_info("XYCAL_CANCEL requested")

    cmd_XYCAL_CALIB_STATUS_help = "Report xycal_calib phase / last tip / FitY"

    def cmd_XYCAL_CALIB_STATUS(self, gcmd):
        gcmd.respond_info(
            "xycal_calib busy=%s phase=%s matrix=%s fity_vy=%s cx=%.1f cy=%.1f "
            "fity_n=%d dcx45=%s msg=%s err=%s"
            % (
                self._busy,
                self._phase,
                self._matrix_ok,
                self._fity_vy_ok,
                self._last_cx,
                self._last_cy,
                len(self._fity_samples),
                self._fity_dcx45,
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
            "fity_vy_ok": bool(self._fity_vy_ok),
            "fity_vy_x": self._fity_vy_x,
            "fity_vy_y": self._fity_vy_y,
            "fity_index": int(self._fity_index),
            "fity_n": len(self._fity_samples),
            "fity_report": self._fity_report,
            "fity_dcx45": self._fity_dcx45,
            "fity_dcx45_raw": self._fity_dcx45_raw,
            "fity_samples": list(self._fity_samples),
            "ghost_cx": self._ghost_cx,
            "ghost_cy": self._ghost_cy,
            "ghost_r": self._ghost_r,
            "z_calib_z0": self._z_calib_z0,
            "z_offset_dz": self._z_offset_dz,
            "dcx_main_dcx45": self._dcx_main_dcx45,
            "dcx_solved_offset": self._dcx_solved_offset,
            "dcx_report": self._dcx_report,
            "dcx_rows": list(self._dcx_rows),
            "auto_main_x": self._auto_main_x,
            "auto_main_y": self._auto_main_y,
            "auto_main_z": self._auto_main_z,
            "auto_sec_x": self._auto_sec_x,
            "auto_sec_y": self._auto_sec_y,
            "auto_sec_z": self._auto_sec_z,
            "ox": self._auto_ox,
            "oy": self._auto_oy,
        }


def load_config(config):
    return XyCalCalib(config)
