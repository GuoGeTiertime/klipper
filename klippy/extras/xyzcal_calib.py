# XYZ calib orchestration on Klipper host (Center + FitY).
#
# Macro = motion (UI_MOVE / XYZCAL_Z_POINT). Python = detect + math + probe-style Z scan.
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

PM45_SPAN_MM = 4.5

DEFAULT_Z_GHOST_COARSE = (5.0, 4.0, 3.0, 2.0, 1.0, 0.0, -1.0, -2.0, -3.0, -4.0, -5.0)
DEFAULT_Z_GHOST_FINE = (0.5, 0.25, -0.25, -0.5)


def _cfg_float_list(config, option, default=()):
    if config.get(option, None) is None:
        return [float(x) for x in default]
    # Prefer getfloatlist with comma-only sep. Space must NOT be a separator:
    # "1, 2" + seps=(","," ") becomes ["1","","2"] → parse error.
    return list(config.getfloatlist(option, sep=",", count=None))

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
        self.zgap_match_tol_px = config.getfloat(
            "zgap_match_tol_px", 1.0, above=0.0
        )
        self.zgap_dr_tol_px = config.getfloat("zgap_dr_tol_px", 2.0, above=0.0)
        self.z_min_mm = config.getfloat("z_min_mm", 60.0, above=0.0)
        self.dcx_t1_fallback = config.getfloat("dcx_t1_fallback", 3.8)
        self.dcxz_measure_mode = config.get(
            "dcxz_measure_mode", "full_fity"
        ).lower().strip()
        self.dcxz_coarse_offsets = _cfg_float_list(
            config, "dcxz_coarse_offsets", ()
        )
        self.dcxz_fine_span = config.getfloat("dcxz_fine_span", 0.25, above=0.0)
        self.dcxz_fine_step = config.getfloat(
            "dcxz_fine_step", 0.1, above=0.001
        )
        self.z_ghost_coarse = _cfg_float_list(
            config, "z_ghost_coarse", DEFAULT_Z_GHOST_COARSE
        )
        self.z_ghost_fine = _cfg_float_list(
            config, "z_ghost_fine", DEFAULT_Z_GHOST_FINE
        )
        self._busy = False
        self._cancel = False
        self._phase = "idle"
        self._message = ""
        self._error = ""
        self._last_cx = -1.0
        self._last_cy = -1.0
        self._last_r = 0.0
        self._last_fw = 0.0
        self._last_fh = 0.0
        self._last_allowed_roi = None
        self._last_allowed_circle = None
        self._last_expect_x = None
        self._last_expect_y = None
        self._toolhead_x = None
        self._toolhead_y = None
        self._toolhead_z = None
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
        self._dcx_main_dcx45_raw = None
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
            "XYZCAL_CENTER", self.cmd_XYZCAL_CENTER, desc=self.cmd_XYZCAL_CENTER_help
        )
        self.gcode.register_command(
            "XYZCAL_FITY", self.cmd_XYZCAL_FITY, desc=self.cmd_XYZCAL_FITY_help
        )
        self.gcode.register_command(
            "XYZCAL_CANCEL", self.cmd_XYZCAL_CANCEL, desc=self.cmd_XYZCAL_CANCEL_help
        )
        self.gcode.register_command(
            "XYZCAL_Z", self.cmd_XYZCAL_Z, desc=self.cmd_XYZCAL_Z_help
        )
        self.gcode.register_command(
            "XYZCAL_DCXZ", self.cmd_XYZCAL_DCXZ, desc=self.cmd_XYZCAL_DCXZ_help
        )
        self.gcode.register_command(
            "XYZCAL_AUTO", self.cmd_XYZCAL_AUTO, desc=self.cmd_XYZCAL_AUTO_help
        )
        self.gcode.register_command(
            "XYZCAL_CALIB_STATUS",
            self.cmd_XYZCAL_CALIB_STATUS,
            desc=self.cmd_XYZCAL_CALIB_STATUS_help,
        )

    def _detect_obj(self):
        return self.printer.lookup_object("xyzcal_detect", None)

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

    def _abs_move(self, coord):
        # coord: [x, y, z] 绝对机床坐标, None 表示该轴不动
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.manual_move(coord, float(self.jog_speed))
        toolhead.dwell(0.3)
        toolhead.wait_moves()

    def _rel_move(self, axis, delta_mm):
        # 单轴增量: 目标 = 当前机床坐标 + delta_mm
        idx = "XYZ".index(str(axis).upper())
        toolhead = self.printer.lookup_object("toolhead")
        pos = toolhead.get_position()
        coord = [None, None, None]
        coord[idx] = pos[idx] + float(delta_mm)
        self._abs_move(coord)

    def _detect(
        self,
        gcmd,
        reset_follow=False,
        flush=0,
        expected=None,
        fresh_frame=False,
        mode=None,
    ):
        det = self._detect_obj()
        if det is None:
            raise gcmd.error("xyzcal_detect not loaded; include [xyzcal_detect] first")
        flush_n = int(flush)
        # flush>=2 historically meant motion-fresh frames
        if flush_n >= 2:
            fresh_frame = True
        if fresh_frame and flush_n < 2:
            flush_n = 2
        if getattr(self, "_need_detect_reset", False):
            reset_follow = True
            self._need_detect_reset = False
            if not mode:
                mode = "acquire"
        body = {
            "url": self._snap_url(gcmd),
            "min_confidence": self.min_confidence,
            "search_width_px": 80,
            "search_height_px": 72,
            "flush_snapshot_count": flush_n,
            "fresh_frame": bool(fresh_frame),
            "reset_follow": bool(reset_follow),
            "insecure": True,
        }
        if mode:
            body["mode"] = str(mode)
        elif expected is not None:
            body["mode"] = "track"
        elif reset_follow:
            body["mode"] = "acquire"
        if expected is not None:
            body["expected_x"] = float(expected[0])
            body["expected_y"] = float(expected[1])
            self._last_expect_x = float(expected[0])
            self._last_expect_y = float(expected[1])
        elif reset_follow:
            self._last_expect_x = None
            self._last_expect_y = None
        result = det.detect_once(body)
        self._last_cx = float(result.get("cx_px", -1) or -1)
        self._last_cy = float(result.get("cy_px", -1) or -1)
        self._last_r = float(result.get("radius_px", 0) or 0)
        self._last_fw = float(result.get("frame_w", 0) or 0)
        self._last_fh = float(result.get("frame_h", 0) or 0)
        roi = result.get("allowed_roi")
        if isinstance(roi, (list, tuple)) and len(roi) >= 4:
            try:
                self._last_allowed_roi = [float(v) for v in roi[:4]]
            except (TypeError, ValueError):
                self._last_allowed_roi = None
        else:
            self._last_allowed_roi = None
        circ = result.get("allowed_circle")
        if isinstance(circ, (list, tuple)) and len(circ) >= 3:
            try:
                self._last_allowed_circle = [float(v) for v in circ[:3]]
            except (TypeError, ValueError):
                self._last_allowed_circle = None
        else:
            self._last_allowed_circle = None
        if self._looks_like_tip(result) and self._last_cx >= 0 and self._last_cy >= 0:
            self._last_expect_x = float(self._last_cx)
            self._last_expect_y = float(self._last_cy)
        return result

    def _detect_tip(
        self,
        gcmd,
        *,
        expected=None,
        after_motion=False,
        mode=None,
    ):
        """Detect tip; on miss, reacquire with tracker reset."""
        use_mode = mode
        if use_mode is None:
            use_mode = "track" if expected is not None else "acquire"
        tip = self._detect(
            gcmd,
            fresh_frame=bool(after_motion),
            flush=2 if after_motion else 0,
            expected=expected,
            mode=use_mode,
        )
        if self._looks_like_tip(tip):
            return tip
        return self._detect(
            gcmd,
            reset_follow=True,
            fresh_frame=True,
            flush=2,
            mode="reacquire",
        )

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

    def _remember_toolhead(self):
        x, y, z = self._toolhead_xyz()
        self._toolhead_x = x
        self._toolhead_y = y
        self._toolhead_z = z
        return x, y, z

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
        self._z_assert_min(gcmd, z, "goto Z")
        site = str(site).upper()
        if site.startswith("SEC"):
            site = "SECOND"
        else:
            site = "MAIN"
        prev = getattr(self, "_last_goto_site", None)
        script = "XYZCAL_GOTO SITE=%s X=%.3f Y=%.3f Z=%.3f" % (
            site,
            float(x),
            float(y),
            float(z),
        )
        self.gcode.run_script_from_command(script)
        self._check_cancel(gcmd)
        if prev is not None and prev != site:
            self._need_detect_reset = True
        self._last_goto_site = site

    def _z_min_mm(self, gcmd):
        return gcmd.get_float("Z_MIN", self.z_min_mm, above=0.0)

    def _z_assert_min(self, gcmd, z, where, label="XYCAL"):
        z_min = self._z_min_mm(gcmd)
        zf = float(z)
        if zf < z_min - 1e-6:
            raise gcmd.error(
                "%s: %s Z=%.2f < Z_MIN %.1f" % (label, where, zf, z_min)
            )
        return zf

    def _z_ladder_try_ok(self, gcmd, z_try, label="XYZCAL_Z"):
        return float(z_try) >= self._z_min_mm(gcmd) - 1e-6

    def _run_z_point_macro(self, gcmd, site, x, y, z, shift=0.0, label="XYZCAL_Z"):
        self._z_assert_min(gcmd, z, "goto Z", label)
        script = (
            "XYZCAL_Z_POINT SITE=%s X=%.3f Y=%.3f Z=%.3f SHIFT=%.3f SPEED=%d"
            % (
                str(site).upper(),
                float(x),
                float(y),
                float(z),
                float(shift),
                int(self.jog_speed),
            )
        )
        self.gcode.run_script_from_command(script)
        self._check_cancel(gcmd)

    def _dcxz_measure_pm45(self, gcmd, tip0=None):
        """Return {dcx, cx_pos, cy_pos, cx_neg, cy_neg} or None.

        Y±4.5 用 FitY vy（或短探针 vy）预测 expected，搜索窗跟随喷嘴。
        tip0: 当前 Z 点已检出的 tip；缺省则用 _last_cx/cy。
        """
        span = float(PM45_SPAN_MM)
        if tip0 and self._looks_like_tip(tip0):
            cx0 = float(tip0["cx_px"])
            cy0 = float(tip0["cy_px"])
        else:
            cx0 = float(self._last_cx)
            cy0 = float(self._last_cy)
        if self._fity_vy_ok:
            vx, vy = float(self._fity_vy_x), float(self._fity_vy_y)
        elif self._matrix_ok:
            vx, vy = float(self._vy_x), float(self._vy_y)
        else:
            vx = vy = None
        def _est(d_mm):
            if vx is None or not (cx0 >= 0 and cy0 >= 0):
                return None
            return (cx0 + float(d_mm) * vx, cy0 + float(d_mm) * vy)

        self._rel_move("Y", span)
        self._check_cancel(gcmd, "XYZCAL_DCXZ")
        tip_pos = self._detect_tip(gcmd, after_motion=True, expected=_est(span))
        if not self._looks_like_tip(tip_pos):
            return None
        cx_pos = float(tip_pos["cx_px"])
        cy_pos = float(tip_pos["cy_px"])
        self._rel_move("Y", -2.0 * span)
        self._check_cancel(gcmd, "XYZCAL_DCXZ")
        # 从 +span 再走到 -span：相对 tip0 为 -span；相对 tip_pos 为 -2*span
        tip_neg = self._detect_tip(
            gcmd,
            after_motion=True,
            expected=_est(-span)
            if _est(-span) is not None
            else (
                (cx_pos - 2.0 * span * vx, cy_pos - 2.0 * span * vy)
                if vx is not None
                else None
            ),
        )
        if not self._looks_like_tip(tip_neg):
            self._rel_move("Y", span)
            return None
        cx_neg = float(tip_neg["cx_px"])
        cy_neg = float(tip_neg["cy_px"])
        self._rel_move("Y", span)
        self._check_cancel(gcmd, "XYZCAL_DCXZ")
        return {
            "dcx": cx_neg - cx_pos,
            "cx_pos": cx_pos,
            "cy_pos": cy_pos,
            "cx_neg": cx_neg,
            "cy_neg": cy_neg,
        }

    def _dcxz_offsets_from_range(self, off_hi, off_lo, step):
        off = round(float(off_hi) * 1000.0) / 1000.0
        off_lo = float(off_lo)
        step = float(step)
        out = []
        while off >= off_lo - 1e-6:
            out.append(round(off * 1000.0) / 1000.0)
            off = round((off - step) * 1000.0) / 1000.0
        return out

    def _dcxz_offsets_for_pass(self, gcmd, t1_ref, auto, off_hi, off_lo, step):
        if self.dcxz_coarse_offsets:
            return list(self.dcxz_coarse_offsets)
        if auto:
            return self._dcxz_offsets_from_range(
                t1_ref + 1.0, t1_ref - 1.0, 0.5
            )
        return self._dcxz_offsets_from_range(off_hi, off_lo, step)

    def _dcxz_fine_offsets(self, center_off):
        span = float(self.dcxz_fine_span)
        step = float(self.dcxz_fine_step)
        hi = round((float(center_off) + span) * 1000.0) / 1000.0
        lo = round((float(center_off) - span) * 1000.0) / 1000.0
        return self._dcxz_offsets_from_range(hi, lo, step)

    def _run_dcxz_main_baseline(self, gcmd, main_x, main_y, main_z, skip_center=False):
        self._set_phase("dcx_main", "dcx main FitY")
        self._xycal_goto(gcmd, "MAIN", main_x, main_y, main_z)
        if skip_center and self._fity_vy_ok:
            gcmd.respond_info(
                "XYZCAL_DCXZ: reuse prior Center FitY vy=(%.3f,%.3f); skip re-center"
                % (self._fity_vy_x, self._fity_vy_y)
            )
        else:
            self._matrix_ok = False
            self._fity_vy_ok = False
            self.run_center(gcmd)
        self.run_fity(gcmd)
        main_dcx = self._fity_dcx45
        if main_dcx is None:
            fit_ok, fit_dcx, _ = self._compute_dcx45(self._fity_samples)
            main_dcx = fit_dcx if fit_ok else None
        if main_dcx is None:
            raise gcmd.error("XYZCAL_DCXZ: main FitY dcx45 failed")
        self._dcx_main_dcx45 = float(main_dcx)
        self._dcx_main_dcx45_raw = (
            float(self._fity_dcx45_raw)
            if self._fity_dcx45_raw is not None
            else None
        )
        gcmd.respond_info(
            "XYZCAL_DCXZ: main dcx45=%.2f raw=%s"
            % (
                self._dcx_main_dcx45,
                (
                    ("%.2f" % self._dcx_main_dcx45_raw)
                    if self._dcx_main_dcx45_raw is not None
                    else "?"
                ),
            )
        )
        return self._dcx_main_dcx45


    def _dcxz_scan_offsets(
        self, gcmd, main_z, sec_x, sec_y, offsets, pass_tag, rows, phase_label, measure_cb
    ):
        """Shared Z-offset scan loop; measure_cb(gcmd, off, z_abs) -> row dict."""
        z_min = gcmd.get_float("Z_MIN", self.z_min_mm, above=0.0)
        n = len(offsets)
        for i, off in enumerate(offsets):
            self._check_cancel(gcmd, "XYZCAL_DCXZ")
            off = float(off)
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
                self._dcx_rebuild_report(self._dcx_main_dcx45 or 0.0, rows)
                continue
            self._set_phase(
                "dcx_scan",
                "%s off=%+.2f z=%.2f (%d/%d)"
                % (phase_label, off, z_abs, i + 1, n),
            )
            self._run_z_point_macro(gcmd, "SECOND", sec_x, sec_y, z_abs, 0.0)
            try:
                row = measure_cb(gcmd, off, z_abs)
                if not isinstance(row, dict):
                    row = {
                        "off": off,
                        "z": z_abs,
                        "ok": False,
                        "dcx": None,
                        "pass": pass_tag,
                        "reason": "bad measure",
                    }
                else:
                    row.setdefault("off", off)
                    row.setdefault("z", z_abs)
                    row.setdefault("pass", pass_tag)
                rows.append(row)
                if row.get("ok") and row.get("dcx") is not None:
                    gcmd.respond_info(
                        "XYZCAL_DCXZ [%s] off=%+.2f dcx=%.2f vs=%.2f"
                        % (
                            pass_tag,
                            off,
                            float(row["dcx"]),
                            float(row["dcx"]) - float(self._dcx_main_dcx45 or 0.0),
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
            self._dcx_rebuild_report(self._dcx_main_dcx45 or 0.0, rows)

    def _run_dcxz_scan_pm45(
        self, gcmd, main_z, sec_x, sec_y, offsets, pass_tag, rows
    ):
        def _measure(gcmd, off, z_abs):
            tip0 = self._detect_tip(gcmd, after_motion=True)
            if not self._looks_like_tip(tip0):
                return {
                    "off": off,
                    "z": z_abs,
                    "ok": False,
                    "dcx": None,
                    "pass": pass_tag,
                    "reason": "no tip",
                }
            m = self._dcxz_measure_pm45(gcmd, tip0=tip0)
            ok = m is not None
            dcx = float(m["dcx"]) if ok else None
            row = {
                "off": off,
                "z": z_abs,
                "ok": ok,
                "dcx": dcx,
                "pass": pass_tag,
                "reason": "" if ok else "pm45",
            }
            if ok:
                row["cx_pos"] = float(m["cx_pos"])
                row["cy_pos"] = float(m["cy_pos"])
                row["cx_neg"] = float(m["cx_neg"])
                row["cy_neg"] = float(m["cy_neg"])
                logging.info(
                    "DCXZ coarse pm45 off=%+.2f z=%.2f "
                    "+4.5=(%.2f,%.2f) -4.5=(%.2f,%.2f) dcx=%.2f"
                    % (
                        off,
                        z_abs,
                        float(m["cx_pos"]),
                        float(m["cy_pos"]),
                        float(m["cx_neg"]),
                        float(m["cy_neg"]),
                        float(dcx),
                    )
                )
            return row

        self._dcxz_scan_offsets(
            gcmd, main_z, sec_x, sec_y, offsets, pass_tag, rows, "dcx pm45", _measure
        )

    def _z_detect_after_shift(self, gcmd, shift_axis, shift_mm, label="XYZCAL_Z"):
        self._rel_move(shift_axis, shift_mm)
        self._check_cancel(gcmd, label)
        tip = self._detect_tip(gcmd, after_motion=True)
        return tip

    def _z_ghost_ladder_match(
        self, gcmd, sx, sy, shift_axis, shift_mm, match_tol, dr_tol
    ):
        z0 = float(self._z_calib_z0)
        z_min = self._z_min_mm(gcmd)
        matched_z = None
        coarse_ladder = [float(d) for d in self.z_ghost_coarse]
        n = len(coarse_ladder)
        tried_coarse = 0
        for i, delta in enumerate(coarse_ladder):
            z_try = round((z0 + delta) * 1000.0) / 1000.0
            if not self._z_ladder_try_ok(gcmd, z_try, "XYZCAL_Z"):
                gcmd.respond_info(
                    "XYZCAL_Z: skip ladder coarse z=%.2f (< Z_MIN %.1f)"
                    % (z_try, z_min)
                )
                continue
            tried_coarse += 1
            self._check_cancel(gcmd, "XYZCAL_Z")
            self._set_phase(
                "z_ladder",
                "Z coarse z=%.2f (%d/%d)" % (z_try, i + 1, n),
            )
            self._run_z_point_macro(gcmd, "SECOND", sx, sy, z_try, 0.0)
            tip2 = self._z_detect_after_shift(gcmd, shift_axis, shift_mm)
            if not self._looks_like_tip(tip2):
                continue
            if self._circle_match(
                self._ghost_cx,
                self._ghost_cy,
                self._ghost_r,
                tip2,
                match_tol,
                dr_tol,
            ):
                matched_z = z_try
                gcmd.respond_info("XYZCAL_Z: ladder coarse match z=%.3f" % z_try)
                break
        if tried_coarse == 0:
            raise gcmd.error(
                "XYZCAL_Z: ladder coarse all below Z_MIN %.1f (Z0=%.2f)"
                % (z_min, z0)
            )
        if matched_z is None:
            return None
        fine_base = matched_z
        fine_ladder = [
            round((fine_base + float(d)) * 1000.0) / 1000.0
            for d in self.z_ghost_fine
        ]
        n_f = len(fine_ladder)
        best_z = matched_z
        for i, z_try in enumerate(fine_ladder):
            if not self._z_ladder_try_ok(gcmd, z_try, "XYZCAL_Z"):
                gcmd.respond_info(
                    "XYZCAL_Z: skip ladder fine z=%.2f (< Z_MIN %.1f)"
                    % (z_try, z_min)
                )
                continue
            self._check_cancel(gcmd, "XYZCAL_Z")
            self._set_phase(
                "z_ladder",
                "Z fine z=%.2f (%d/%d)" % (z_try, i + 1, n_f),
            )
            self._run_z_point_macro(gcmd, "SECOND", sx, sy, z_try, 0.0)
            tip2 = self._z_detect_after_shift(gcmd, shift_axis, shift_mm)
            if not self._looks_like_tip(tip2):
                continue
            if self._circle_match(
                self._ghost_cx,
                self._ghost_cy,
                self._ghost_r,
                tip2,
                match_tol,
                dr_tol,
            ):
                best_z = z_try
        return float(best_z) - z0

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
        init_flag = 1 if dual_init else 0
        self.gcode.run_script_from_command(
            "XYZCAL_WRITE_OFFSET Z=%.2f INIT=%d" % (dz_f, init_flag)
        )
        return dz_f

    def _save_tool1_offsets_xyz(self, gcmd, ox, oy, oz):
        ox_f = round(float(ox) * 100.0) / 100.0
        oy_f = round(float(oy) * 100.0) / 100.0
        oz_f = round(float(oz) * 100.0) / 100.0
        self.gcode.run_script_from_command(
            "XYZCAL_WRITE_OFFSET X=%.2f Y=%.2f Z=%.2f INIT=1"
            % (ox_f, oy_f, oz_f)
        )
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
        gcmd.respond_info("XYZCAL_CENTER: span Y +%.1f" % span)
        self._rel_move("Y", span)
        self._check_cancel(gcmd)
        tip_pos = self._detect_tip(
            gcmd,
            after_motion=True,
            expected=(cx0 + span * ref_x, cy0 + span * ref_y),
        )
        if not self._looks_like_tip(tip_pos):
            self._rel_move("Y", -span)
            raise gcmd.error("XYZCAL_CENTER: no tip at Y+%.1f" % span)
        vx = (float(tip_pos["cx_px"]) - cx0) / span
        vy = (float(tip_pos["cy_px"]) - cy0) / span
        if not self._span_vector_ok(vx, vy, ref_x, ref_y):
            self._rel_move("Y", -span)
            raise gcmd.error("XYZCAL_CENTER: +span vector conflicts with short probe")
        pos_cx = float(tip_pos["cx_px"])
        pos_cy = float(tip_pos["cy_px"])
        self._vy_x, self._vy_y = vx, vy

        self._set_phase("probe_span_neg", "span Y-%.1f" % span)
        gcmd.respond_info("XYZCAL_CENTER: span Y +%.1f → -%.1f" % (span, span))
        self._rel_move("Y", -2.0 * span)
        self._check_cancel(gcmd)
        tip_neg = self._detect_tip(
            gcmd,
            after_motion=True,
            expected=(
                pos_cx - 2.0 * span * self._vy_x,
                pos_cy - 2.0 * span * self._vy_y,
            ),
        )
        if not self._looks_like_tip(tip_neg):
            self._rel_move("Y", span)
            raise gcmd.error("XYZCAL_CENTER: no tip at Y-%.1f" % span)
        vx = (pos_cx - float(tip_neg["cx_px"])) / (2.0 * span)
        vy = (pos_cy - float(tip_neg["cy_px"])) / (2.0 * span)
        if not self._span_vector_ok(vx, vy, self._vy_x, self._vy_y):
            self._rel_move("Y", span)
            raise gcmd.error("XYZCAL_CENTER: ±span vector inconsistent")
        self._vy_x, self._vy_y = vx, vy

        self._set_phase("probe_span_home", "span return")
        gcmd.respond_info("XYZCAL_CENTER: return from Y-%.1f" % span)
        self._rel_move("Y", span)
        self._check_cancel(gcmd)
        tip_home = self._detect(
            gcmd, reset_follow=True, fresh_frame=True, mode="acquire"
        )
        if not self._looks_like_tip(tip_home):
            tip_home = self._detect(
                gcmd, reset_follow=True, fresh_frame=True, mode="reacquire"
            )
        if not self._looks_like_tip(tip_home):
            raise gcmd.error("XYZCAL_CENTER: no tip after ±span return")
        home_err = max(
            abs(float(tip_home["cx_px"]) - cx0),
            abs(float(tip_home["cy_px"]) - cy0),
        )
        if not math.isfinite(home_err) or home_err > 4.0:
            raise gcmd.error(
                "XYZCAL_CENTER: ±span return error %.1fpx" % home_err
            )
        self._fity_vy_x = self._vy_x
        self._fity_vy_y = self._vy_y
        self._fity_vy_ok = True
        gcmd.respond_info(
            "XYZCAL_CENTER span vy=(%.3f,%.3f) px/mm homeErr=%.2f"
            % (self._fity_vy_x, self._fity_vy_y, home_err)
        )
        return tip_home

    cmd_XYZCAL_CENTER_help = (
        "Host Center: probe X/Y, ±span FitY vy, fine correct. "
        "Params: URL= PROBE_MM= TOL_PX= MAX_ITER= SPAN_MM= TOOL=MAIN|SECOND"
    )

    def _emit_center_session(self, gcmd):
        session = self.printer.lookup_object("xyzcal_session", None)
        if session is None:
            return
        tool_raw = gcmd.get("TOOL", "MAIN")
        try:
            session.on_center_done(self, gcmd, tool=tool_raw)
        except Exception:
            logging.exception("xyzcal_session: on_center_done failed")

    def cmd_XYZCAL_CENTER(self, gcmd):
        if self._busy:
            raise gcmd.error("xyzcal_calib busy (%s)" % self._phase)
        self._require_xy_homed(gcmd, "XYZCAL_CENTER")
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
            logging.exception("XYZCAL_CENTER failed")
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
            gcmd.respond_info("XYZCAL_CENTER: detect tip…")
            tip = self._detect(gcmd, reset_follow=True, fresh_frame=True, mode="acquire")
            self._check_cancel(gcmd)
            if not self._looks_like_tip(tip):
                raise gcmd.error(
                    "XYZCAL_CENTER: no tip (Detect tip in view first) err=%s"
                    % (tip.get("error") or tip.get("reject_reason") or "")
                )

            cx0 = float(tip["cx_px"])
            cy0 = float(tip["cy_px"])

            self._set_phase("probe_x", "probe +X")
            gcmd.respond_info("XYZCAL_CENTER: probe X +%.2f" % probe_mm)
            self._rel_move("X", probe_mm)
            self._check_cancel(gcmd)
            # track 必须带 expected；探针前尚无矩阵，用上一 tip 作窗心（0.6mm≈14px 仍落在 80×72 内）
            tip_x = self._detect_tip(
                gcmd,
                after_motion=True,
                expected=(cx0, cy0),
                mode="track",
            )
            if not self._looks_like_tip(tip_x):
                raise gcmd.error("XYZCAL_CENTER: lost tip after +X")
            ok, detail = self._apply_probe(
                "x", cx0, cy0, float(tip_x["cx_px"]), float(tip_x["cy_px"]), probe_mm
            )
            if not ok:
                raise gcmd.error("XYZCAL_CENTER Probe X: %s" % detail)
            cx0 = float(tip_x["cx_px"])
            cy0 = float(tip_x["cy_px"])

            self._set_phase("probe_y", "probe +Y")
            gcmd.respond_info("XYZCAL_CENTER: probe Y +%.2f" % probe_mm)
            self._rel_move("Y", probe_mm)
            self._check_cancel(gcmd)
            tip_y = self._detect_tip(
                gcmd,
                after_motion=True,
                expected=(cx0, cy0),
                mode="track",
            )
            if not self._looks_like_tip(tip_y):
                raise gcmd.error("XYZCAL_CENTER: lost tip after +Y")
            ok, detail = self._apply_probe(
                "y", cx0, cy0, float(tip_y["cx_px"]), float(tip_y["cy_px"]), probe_mm
            )
            if not ok:
                raise gcmd.error("XYZCAL_CENTER Probe Y: %s" % detail)

            if not self._matrix_ok_vals(
                self._vx_x, self._vx_y, self._vy_x, self._vy_y
            ):
                raise gcmd.error("XYZCAL_CENTER: probe matrix invalid (det/scale)")
            self._matrix_ok = True

            self._set_phase("probe_undo", "undo probe")
            gcmd.respond_info("XYZCAL_CENTER: undo probe")
            self._rel_move("X", -probe_mm)
            self._rel_move("Y", -probe_mm)
            self._check_cancel(gcmd)
            undo_ex = (
                float(tip_y["cx_px"])
                - probe_mm * float(self._vx_x)
                - probe_mm * float(self._vy_x),
                float(tip_y["cy_px"])
                - probe_mm * float(self._vx_y)
                - probe_mm * float(self._vy_y),
            )
            tip = self._detect_tip(
                gcmd,
                after_motion=True,
                expected=undo_ex,
                mode="track",
            )
            if not self._looks_like_tip(tip):
                raise gcmd.error(
                    "XYZCAL_CENTER: lost tip after undo (%s)"
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
                        raise gcmd.error("XYZCAL_CENTER: %s" % detail)
                    d_x, d_y, err_main, done = step
                    if done:
                        self._remember_toolhead()
                        self._set_phase("done", "Centered e=%.2f" % err_main)
                        gcmd.respond_info(
                            "XYZCAL_CENTER ok=True cx=%.1f cy=%.1f e=%.2f "
                            "xyz=(%.3f,%.3f,%.3f) fity_vy_ok=%s"
                            % (
                                self._last_cx,
                                self._last_cy,
                                err_main,
                                self._toolhead_x,
                                self._toolhead_y,
                                self._toolhead_z,
                                self._fity_vy_ok,
                            )
                        )
                        self._emit_center_session(gcmd)
                        return
                    if last_err is not None and err_main > last_err + max(
                        8.0, 0.25 * last_err
                    ):
                        raise gcmd.error(
                            "XYZCAL_CENTER: dir wrong (e grew %.1f→%.1f)"
                            % (last_err, err_main)
                        )
                    last_err = err_main
                    gcmd.respond_info(
                        "XYZCAL_CENTER correct dX=%.3f dY=%.3f e=%.1f"
                        % (d_x, d_y, err_main)
                    )
                    if abs(d_x) >= 0.01:
                        self._rel_move("X", d_x)
                    if abs(d_y) >= 0.01:
                        self._rel_move("Y", d_y)
                    tip = self._detect(
                        gcmd,
                        fresh_frame=True,
                        mode="track",
                        expected=(self._last_cx, self._last_cy),
                    )
                    if not self._looks_like_tip(tip):
                        tip = self._detect(gcmd, fresh_frame=True, flush=1, mode="track")
                    if not self._looks_like_tip(tip):
                        raise gcmd.error("XYZCAL_CENTER: lost tip during correct")

                raise gcmd.error(
                    "XYZCAL_CENTER: not within tol after %d iters (last e~%.1f)"
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
        # 点位由 run_fity / DCXZ measure 写 log；此处只组 UI 摘要
        lines = []
        if self._fity_dcx45 is not None:
            lines.append(
                "【拟合】Δcx@±4.5 = %.2f px  (pos/neg 臂线性拟合)"
                % self._fity_dcx45
            )
        if self._fity_dcx45_raw is not None:
            lines.append(
                "【直接差】Δcx@±4.5 = %.2f px  (cx(-4.5)-cx(+4.5))"
                % self._fity_dcx45_raw
            )
        self._fity_report = "\n".join(lines)

    def _fity_estimate(self, mm):
        return (
            self._fity_tip0_cx + float(mm) * self._fity_vy_x,
            self._fity_tip0_cy + float(mm) * self._fity_vy_y,
        )

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
            self._rel_move("Y", back)
            self._fity_prev_mm = 0.0

    cmd_XYZCAL_FITY_help = (
        "Host FitY Y-scan ±4.3..±4.7. Needs prior Center span (fity_vy). "
        "Params: URL="
    )

    def cmd_XYZCAL_FITY(self, gcmd):
        if self._busy:
            raise gcmd.error("xyzcal_calib busy (%s)" % self._phase)
        self._require_xy_homed(gcmd, "XYZCAL_FITY")
        if not self._fity_vy_ok:
            raise gcmd.error(
                "XYZCAL_FITY: run XYZCAL_CENTER first (need ±span FitY vy)"
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
                logging.exception("XYZCAL_FITY home after error")
            self._error = str(exc)
            self._set_phase("error", self._error)
            self._rebuild_fity_report()
            logging.exception("XYZCAL_FITY failed")
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
        self._set_phase("fity_tip", "FitY need tip")
        tip = self._detect(
            gcmd, reset_follow=True, fresh_frame=True, mode="acquire"
        )
        self._check_cancel(gcmd, "XYZCAL_FITY")
        if not self._looks_like_tip(tip):
            tip = self._detect_tip(gcmd, after_motion=True, mode="reacquire")
        self._check_cancel(gcmd, "XYZCAL_FITY")
        if not self._looks_like_tip(tip):
            raise gcmd.error("XYZCAL_FITY: fresh centered tip required")
        self._fity_tip0_cx = float(tip["cx_px"])
        self._fity_tip0_cy = float(tip["cy_px"])
        fw = float(tip.get("frame_w") or 640)
        fh = float(tip.get("frame_h") or 480)
        gcmd.respond_info(
            "XYZCAL_FITY start tip=(%.1f,%.1f) vy=(%.3f,%.3f)"
            % (
                self._fity_tip0_cx,
                self._fity_tip0_cy,
                self._fity_vy_x,
                self._fity_vy_y,
            )
        )

        n_off = len(FITY_OFFSETS_MM)
        for idx, target in enumerate(FITY_OFFSETS_MM):
            self._check_cancel(gcmd, "XYZCAL_FITY")
            self._fity_index = idx
            delta = float(target) - float(self._fity_prev_mm)
            self._set_phase(
                "fity_move",
                "Y%s%.1f (%d/%d)"
                % ("+" if target >= 0 else "", target, idx + 1, n_off),
            )
            if abs(delta) >= 0.001:
                self._rel_move("Y", delta)
            est = self._fity_estimate(target)
            tip_pt = None
            for attempt in range(3):
                self._check_cancel(gcmd, "XYZCAL_FITY")
                tip_pt = self._detect_tip(
                    gcmd,
                    expected=est,
                    after_motion=(attempt == 0 and abs(delta) >= 0.001),
                    mode="track",
                )
                if not self._looks_like_tip(tip_pt):
                    continue
                cx = float(tip_pt["cx_px"])
                cy = float(tip_pt["cy_px"])
                fw = float(tip_pt.get("frame_w") or fw)
                fh = float(tip_pt.get("frame_h") or fh)
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
                    "XYZCAL_FITY: Y%s%.1f same ROI failed"
                    % ("+" if target >= 0 else "", target)
                )

            cx = float(tip_pt["cx_px"])
            cy = float(tip_pt["cy_px"])
            r = float(tip_pt.get("radius_px") or 0)
            if not self._fity_refine_at_span(target, cx, cy):
                self._fity_home_y()
                raise gcmd.error(
                    "XYZCAL_FITY: ±4.5 vector check failed at Y%s%.1f"
                    % ("+" if target >= 0 else "", target)
                )
            self._fity_prev_mm = float(target)
            self._fity_samples.append(
                {"mm": float(target), "cx": cx, "cy": cy, "r": r}
            )
            self._rebuild_fity_report()
            gcmd.respond_info(
                "XYZCAL_FITY [%d/%d] Y%s%.1f cx=%.1f cy=%.1f"
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
        # DCXZ/Auto 与屏端：主结果只用两臂线性拟合 Δcx@±4.5
        self._fity_dcx45 = fit_dcx if fit_ok else (raw_dcx if raw_ok else None)
        self._rebuild_fity_report()
        for i, s in enumerate(self._fity_samples):
            logging.info(
                "FitY point [%d] Y%+.1fmm (cx,cy)=(%.2f, %.2f) px"
                % (i + 1, float(s["mm"]), float(s["cx"]), float(s["cy"]))
            )
        self._fity_home_y()
        self._detect(gcmd, reset_follow=True, fresh_frame=True, mode="acquire")
        self._set_phase(
            "done",
            "FitY done n=%d dcx45=%s"
            % (
                len(self._fity_samples),
                ("%.2f" % self._fity_dcx45) if self._fity_dcx45 is not None else "?",
            ),
        )
        gcmd.respond_info(
            "XYZCAL_FITY ok=True n=%d dcx45=%s fit=%s raw=%s"
            % (
                len(self._fity_samples),
                self._fity_dcx45,
                fit_dcx if fit_ok else None,
                raw_dcx if raw_ok else None,
            )
        )

    cmd_XYZCAL_Z_help = (
        "Host Z gap: ghost match + fixed Z ladder. "
        "Params: URL= MAIN_X/Y/Z SEC_X/Y/Z SHIFT_AXIS SHIFT_MM "
        "MATCH_TOL DR_TOL Z_MIN WRITE="
    )

    def cmd_XYZCAL_Z(self, gcmd):
        if self._busy:
            raise gcmd.error("xyzcal_calib busy (%s)" % self._phase)
        self._require_xy_homed(gcmd, "XYZCAL_Z")
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
            logging.exception("XYZCAL_Z failed")
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
                raise gcmd.error("XYZCAL_Z: need %s" % name)
        shift_axis = gcmd.get("SHIFT_AXIS", self.zgap_axis).upper()
        if shift_axis not in ("X", "Y"):
            shift_axis = "Y"
        shift_mm = gcmd.get_float("SHIFT_MM", self.zgap_shift_mm, above=0.1)
        match_tol = gcmd.get_float("MATCH_TOL", self.zgap_match_tol_px, above=0.0)
        dr_tol = gcmd.get_float("DR_TOL", self.zgap_dr_tol_px, above=0.0)
        write = gcmd.get_int("WRITE", 0, minval=0, maxval=1) != 0
        z_min = self._z_min_mm(gcmd)
        self._z_assert_min(gcmd, mz, "MAIN_Z", "XYZCAL_Z")
        self._z_assert_min(gcmd, sz, "SEC_Z", "XYZCAL_Z")
        gcmd.respond_info("XYZCAL_Z: Z_MIN=%.1f" % z_min)

        self._set_phase("z_goto_main", "Z go main")
        gcmd.respond_info("XYZCAL_Z: goto main")
        self._xycal_goto(gcmd, "MAIN", mx, my, mz)
        self._matrix_ok = False
        self._fity_vy_ok = False
        self.run_center(gcmd)
        self._check_cancel(gcmd, "XYZCAL_Z")

        self._set_phase("z_shift_main", "Z shift main")
        self._rel_move(shift_axis, shift_mm)
        self._check_cancel(gcmd, "XYZCAL_Z")
        tip = self._detect_tip(gcmd, after_motion=True)
        if not self._looks_like_tip(tip):
            raise gcmd.error("XYZCAL_Z: no tip after main shift")
        self._ghost_cx = float(tip["cx_px"])
        self._ghost_cy = float(tip["cy_px"])
        self._ghost_r = float(tip.get("radius_px") or 8.0)
        _, _, z0 = self._toolhead_xyz()
        self._z_calib_z0 = float(z0)
        self._z_assert_min(gcmd, self._z_calib_z0, "Z0 ghost", "XYZCAL_Z")
        gcmd.respond_info(
            "XYZCAL_Z: ghost (%.1f,%.1f) r=%.1f Z0=%.3f"
            % (self._ghost_cx, self._ghost_cy, self._ghost_r, self._z_calib_z0)
        )

        self._set_phase("z_goto_second", "Z go 2nd")
        self._xycal_goto(gcmd, "SECOND", sx, sy, sz)
        self._matrix_ok = False
        self._fity_vy_ok = False
        self.run_center(gcmd)
        self._check_cancel(gcmd, "XYZCAL_Z")

        self._set_phase("z_shift_second", "Z shift 2nd")
        self._rel_move(shift_axis, shift_mm)
        self._check_cancel(gcmd, "XYZCAL_Z")
        tip2 = self._detect_tip(gcmd, after_motion=True)
        if not self._looks_like_tip(tip2):
            raise gcmd.error("XYZCAL_Z: no tip after 2nd shift")

        if self._circle_match(
            self._ghost_cx, self._ghost_cy, self._ghost_r, tip2, match_tol, dr_tol
        ):
            _, _, z_now = self._toolhead_xyz()
            self._z_offset_dz = float(z_now) - float(self._z_calib_z0)
            gcmd.respond_info("XYZCAL_Z: matched without Z adjust dz=%.3f" % self._z_offset_dz)
        else:
            self._z_offset_dz = self._z_ghost_ladder_match(
                gcmd, sx, sy, shift_axis, shift_mm, match_tol, dr_tol
            )
            if self._z_offset_dz is None:
                raise gcmd.error("XYZCAL_Z: ladder no ghost match")

        if self._z_offset_dz is None:
            raise gcmd.error("XYZCAL_Z: dz not computed")
        dz = round(float(self._z_offset_dz) * 100.0) / 100.0
        self._z_offset_dz = dz
        if write:
            self._save_tool1_offset_z(gcmd, dz, dual_init=False)
        self._set_phase("done", "Z dz=%.2f" % dz)
        gcmd.respond_info("XYZCAL_Z ok=True dz=%.2f write=%s" % (dz, write))

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
        # DCXZ 进行中清掉嵌套 FitY 的 UI 摘要，避免与本报告叠两份
        self._fity_report = ""
        lines = []
        lines.append("【主拟合】Δcx@±4.5 = %.2f px" % float(main_dcx))
        if self._dcx_main_dcx45_raw is not None:
            lines.append(
                "【主直接差】Δcx@±4.5 = %.2f px  (cx(-4.5)-cx(+4.5))"
                % float(self._dcx_main_dcx45_raw)
            )
        last_pass = None
        for r in rows:
            pass_tag = str(r.get("pass", "?") or "?")
            if pass_tag != last_pass:
                last_pass = pass_tag
                if pass_tag == "coarse":
                    lines.append("----【副粗直接差】----")
                elif pass_tag == "fine":
                    lines.append("----【副细拟合】----")
                else:
                    lines.append("----[%s]----" % pass_tag)
            off = float(r.get("off", 0))
            z = float(r.get("z", 0))
            if r.get("ok"):
                vs = float(r["dcx"]) - float(main_dcx)
                lines.append(
                    "off=%+.2f z=%.2f dcx=%.2f vs主=%+.2f"
                    % (off, z, float(r["dcx"]), vs)
                )
            else:
                lines.append(
                    "off=%+.2f z=%.2f FAIL %s"
                    % (off, z, r.get("reason") or "")
                )
        if self._dcx_solved_offset is not None:
            lines.append("【结果】offset* = %.2f" % float(self._dcx_solved_offset))
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
                raise gcmd.error("XYZCAL_DCXZ: main FitY dcx45 failed")
            self._dcx_main_dcx45 = float(main_dcx)
            self._dcx_main_dcx45_raw = (
                float(self._fity_dcx45_raw)
                if self._fity_dcx45_raw is not None
                else None
            )
            gcmd.respond_info("XYZCAL_DCXZ: main dcx45=%.2f" % self._dcx_main_dcx45)

        off = float(off_hi)
        off_lo = float(off_lo)
        step = float(step)
        while off >= off_lo - 1e-6:
            self._check_cancel(gcmd, "XYZCAL_DCXZ")
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
                row = {
                    "off": off,
                    "z": z_abs,
                    "ok": ok,
                    "dcx": float(dcx) if ok else None,
                    "pass": pass_tag,
                    "reason": "" if ok else "fity",
                }
                if ok:
                    row["fity_dcx45"] = self._fity_dcx45
                    row["fity_dcx45_raw"] = self._fity_dcx45_raw
                    row["fity_samples"] = [
                        {
                            "mm": float(s.get("mm", 0)),
                            "cx": float(s.get("cx", 0)),
                            "cy": float(s.get("cy", 0)),
                            "r": float(s.get("r", 0) or 0),
                        }
                        for s in self._fity_samples
                    ]
                    gcmd.respond_info(
                        "XYZCAL_DCXZ [%s] off=%+.2f dcx=%.2f vs=%.2f"
                        % (
                            pass_tag,
                            off,
                            float(dcx),
                            float(dcx) - float(self._dcx_main_dcx45),
                        )
                    )
                rows.append(row)
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

    def _run_dcxz_scan_fity(
        self, gcmd, main_z, sec_x, sec_y, offsets, pass_tag, rows, vy_x, vy_y
    ):
        """Per-Z FitY (10 pts) without Center; reuse Center-span FitY vy."""
        def _measure(gcmd, off, z_abs):
            self._fity_vy_ok = True
            self._fity_vy_x = float(vy_x)
            self._fity_vy_y = float(vy_y)
            self.run_fity(gcmd)
            dcx = self._fity_dcx45
            if dcx is None:
                fit_ok, fit_dcx, _ = self._compute_dcx45(self._fity_samples)
                dcx = fit_dcx if fit_ok else None
            ok = dcx is not None
            row = {
                "off": off,
                "z": z_abs,
                "ok": ok,
                "dcx": float(dcx) if ok else None,
                "pass": pass_tag,
                "reason": "" if ok else "fity",
            }
            if ok and self._fity_samples:
                row["fity_dcx45"] = self._fity_dcx45
                row["fity_dcx45_raw"] = self._fity_dcx45_raw
                row["fity_samples"] = [
                    {
                        "mm": float(s.get("mm", 0)),
                        "cx": float(s.get("cx", 0)),
                        "cy": float(s.get("cy", 0)),
                        "r": float(s.get("r", 0) or 0),
                    }
                    for s in self._fity_samples
                ]
                logging.info(
                    "DCXZ fine FitY off=%+.2f z=%.2f n=%d dcx=%.2f"
                    % (off, z_abs, len(row["fity_samples"]), float(dcx))
                )
            return row

        self._dcxz_scan_offsets(
            gcmd, main_z, sec_x, sec_y, offsets, pass_tag, rows, "dcx fity10", _measure
        )

    cmd_XYZCAL_DCXZ_help = (
        "Host ΔcxZ scan: main FitY baseline + Z sweep (pm45 or full_fity). "
        "Params: URL= MAIN_X/Y/Z SEC_X/Y T1_REF OFF_HI OFF_LO STEP "
        "Z_MIN AUTO WRITE=  cfg: dcxz_measure_mode"
    )

    def cmd_XYZCAL_DCXZ(self, gcmd):
        if self._busy:
            raise gcmd.error("xyzcal_calib busy (%s)" % self._phase)
        self._require_xy_homed(gcmd, "XYZCAL_DCXZ")
        self._busy = True
        self._cancel = False
        self._error = ""
        self._dcx_rows = []
        self._dcx_main_dcx45 = None
        self._dcx_main_dcx45_raw = None
        self._dcx_solved_offset = None
        self._dcx_report = ""
        try:
            self._run_dcxz(gcmd)
        except Exception as exc:
            self._error = str(exc)
            self._set_phase("error", self._error)
            logging.exception("XYZCAL_DCXZ failed")
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
                raise gcmd.error("XYZCAL_DCXZ: need %s" % name)
        t1_ref = self._read_t1_offset_z(gcmd)
        if auto_override is not None:
            auto = bool(auto_override)
        else:
            auto = gcmd.get_int("AUTO", 0, minval=0, maxval=1) != 0
        if write_override is not None:
            write = bool(write_override)
        else:
            write = gcmd.get_int("WRITE", 0, minval=0, maxval=1) != 0
        # Auto 流水线：用已居中精修坐标，避免回粗定位后再二次 Center
        if auto and self._auto_main_x is not None:
            mx = float(self._auto_main_x)
            my = float(self._auto_main_y)
            if self._auto_main_z is not None:
                mz = float(self._auto_main_z)
        if auto and self._auto_sec_x is not None:
            sx = float(self._auto_sec_x)
            sy = float(self._auto_sec_y)
        z_min = gcmd.get_float("Z_MIN", self.z_min_mm, above=0.0)
        if mz < z_min - 1e-6:
            raise gcmd.error("XYZCAL_DCXZ: MAIN_Z=%.2f < Z_MIN" % mz)

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
            raise gcmd.error("XYZCAL_DCXZ: bad OFF_HI/OFF_LO/STEP")

        if auto:
            # Auto：主喷 FitY 仍 10 点；副喷粗测 pm45(±各1)，细测 FitY10(±各5)，均不再 Center
            gcmd.respond_info(
                "XYZCAL_DCXZ: AUTO hybrid sec=pm45 coarse + fity10 fine "
                "(main FitY10; no per-Z Center)"
            )
            coarse_offs = self._dcxz_offsets_for_pass(
                gcmd, t1_ref, auto, off_hi, off_lo, step
            )
            self._run_dcxz_main_baseline(
                gcmd, mx, my, mz, skip_center=bool(self._fity_vy_ok)
            )
            vy_x = float(self._fity_vy_x)
            vy_y = float(self._fity_vy_y)
            self._run_dcxz_scan_pm45(
                gcmd, mz, sx, sy, coarse_offs, "coarse", rows
            )
            solved = self._dcx_solve_offset(rows, self._dcx_main_dcx45)
            if solved is not None:
                approx = solved
                if approx > t1_ref + 1.0 + 0.01:
                    approx = round((t1_ref + 1.0) * 1000.0) / 1000.0
                if approx < t1_ref - 1.0 - 0.01:
                    approx = round((t1_ref - 1.0) * 1000.0) / 1000.0
                fine_offs = self._dcxz_fine_offsets(approx)
                if (float(mz) + min(fine_offs)) >= z_min - 1e-6:
                    # 细测追加进同一 rows，报告始终保留粗测，不会被细测盖掉
                    n_before_fine = len(rows)
                    self._run_dcxz_scan_fity(
                        gcmd,
                        mz,
                        sx,
                        sy,
                        fine_offs,
                        "fine",
                        rows,
                        vy_x,
                        vy_y,
                    )
                    fine_only = [
                        r for r in rows[n_before_fine:]
                        if str(r.get("pass", "")) == "fine"
                    ]
                    fine_sol = self._dcx_solve_offset(
                        fine_only, self._dcx_main_dcx45
                    )
                    if fine_sol is not None and len(fine_only) >= 2:
                        solved = fine_sol
                        gate_lo = round(
                            (approx - self.dcxz_fine_span) * 1000.0
                        ) / 1000.0
                        gate_hi = round(
                            (approx + self.dcxz_fine_span) * 1000.0
                        ) / 1000.0
        else:
            use_pm45 = self.dcxz_measure_mode not in (
                "full_fity",
                "legacy",
                "full",
            )
            if use_pm45:
                coarse_offs = self._dcxz_offsets_for_pass(
                    gcmd, t1_ref, auto, off_hi, off_lo, step
                )
                self._run_dcxz_main_baseline(
                    gcmd, mx, my, mz, skip_center=False
                )
                self._run_dcxz_scan_pm45(
                    gcmd, mz, sx, sy, coarse_offs, "coarse", rows
                )
                solved = self._dcx_solve_offset(rows, self._dcx_main_dcx45)
            else:
                self._run_dcxz_scan_pass(
                    gcmd,
                    mx,
                    my,
                    mz,
                    sx,
                    sy,
                    off_hi,
                    off_lo,
                    step,
                    "coarse",
                    rows,
                    True,
                )
                solved = self._dcx_solve_offset(rows, self._dcx_main_dcx45)

        self._dcx_rows = list(rows)
        main_dcx = self._dcx_main_dcx45 if self._dcx_main_dcx45 is not None else 0.0
        self._dcx_rebuild_report(main_dcx, rows)
        if solved is None:
            raise gcmd.error(
                "XYZCAL_DCXZ: solve failed\n%s" % (self._dcx_report or "no rows")
            )
        if solved < gate_lo - 0.01 or solved > gate_hi + 0.01:
            raise gcmd.error(
                "XYZCAL_DCXZ: offset*=%.2f out of gate [%.2f..%.2f]\n%s"
                % (solved, gate_lo, gate_hi, self._dcx_report or "")
            )
        self._dcx_solved_offset = float(solved)
        self._dcx_rebuild_report(main_dcx, rows)
        # 结束后只留 DCXZ 摘要；清掉嵌套 FitY 的 10 点/拟合摘要，避免 UI 叠显示
        self._fity_report = ""
        if write:
            self._save_tool1_offset_z(gcmd, solved, dual_init=True)
        self._set_phase("done", "dcx offset=%.2f" % solved)
        gcmd.respond_info(
            "XYZCAL_DCXZ ok=True offset=%.2f main_dcx=%.2f write=%s"
            % (solved, self._dcx_main_dcx45, write)
        )

    cmd_XYZCAL_AUTO_help = (
        "Host one-shot auto: home, center main/2nd, DCXZ, apply XY+Z. "
        "Params: URL= MAIN_X/Y/Z SEC_X/Y/Z WRITE=1 AUTO=1"
    )

    def cmd_XYZCAL_AUTO(self, gcmd):
        if self._busy:
            raise gcmd.error("xyzcal_calib busy (%s)" % self._phase)
        self._busy = True
        self._cancel = False
        self._error = ""
        try:
            self._run_auto(gcmd)
        except Exception as exc:
            self._error = str(exc)
            self._set_phase("error", self._error)
            logging.exception("XYZCAL_AUTO failed")
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
                raise gcmd.error("XYZCAL_AUTO: need %s" % name)
        write = gcmd.get_int("WRITE", 1, minval=0, maxval=1) != 0

        self._z_assert_min(gcmd, mz, "MAIN_Z", "XYZCAL_AUTO")
        self._z_assert_min(gcmd, sz, "SEC_Z", "XYZCAL_AUTO")

        self._home_xy_if_needed(gcmd, "XYZCAL_AUTO")
        self._set_phase("auto_main", "auto main center")
        self._xycal_goto(gcmd, "MAIN", mx, my, mz)
        self._matrix_ok = False
        self._fity_vy_ok = False
        self.run_center(gcmd)
        mx_r, my_r, mz_r = self._toolhead_xyz()
        self._auto_main_x = mx_r
        self._auto_main_y = my_r
        self._auto_main_z = mz_r
        # 保留主喷头 ±span 标定的 FitY vy，供后续 DCXZ 基线复用（副喷头 Center 会改矩阵）
        main_fity_ok = bool(self._fity_vy_ok)
        main_fity_vx = float(self._fity_vy_x)
        main_fity_vy = float(self._fity_vy_y)

        self._set_phase("auto_sec", "auto 2nd center")
        self._xycal_goto(gcmd, "SECOND", sx, sy, sz)
        self._matrix_ok = False
        self.run_center(gcmd)
        sx_r, sy_r, sz_r = self._toolhead_xyz()
        self._auto_sec_x = sx_r
        self._auto_sec_y = sy_r
        self._auto_sec_z = sz_r
        self._auto_ox = round((sx_r - mx_r) * 100.0) / 100.0
        self._auto_oy = round((sy_r - my_r) * 100.0) / 100.0
        if main_fity_ok:
            self._fity_vy_ok = True
            self._fity_vy_x = main_fity_vx
            self._fity_vy_y = main_fity_vy

        self._dcx_rows = []
        self._dcx_main_dcx45 = None
        self._dcx_main_dcx45_raw = None
        self._dcx_solved_offset = None
        self._set_phase("auto_dcxz", "auto DCXZ")
        self._run_dcxz(gcmd, write_override=False, auto_override=True)
        dz = self._dcx_solved_offset
        if dz is None:
            raise gcmd.error("XYZCAL_AUTO: DCXZ solve failed")

        if write:
            self._save_tool1_offsets_xyz(gcmd, self._auto_ox, self._auto_oy, dz)
        self._set_phase("auto_return", "auto return main")
        ret_x = self._auto_main_x if self._auto_main_x is not None else mx
        ret_y = self._auto_main_y if self._auto_main_y is not None else my
        ret_z = self._auto_main_z if self._auto_main_z is not None else mz
        self._xycal_goto(gcmd, "MAIN", ret_x, ret_y, ret_z)
        self._set_phase(
            "done",
            "Auto X=%.2f Y=%.2f Z=%.2f" % (self._auto_ox, self._auto_oy, dz),
        )
        gcmd.respond_info(
            "XYZCAL_AUTO ok=True ox=%.2f oy=%.2f oz=%.2f write=%s"
            % (self._auto_ox, self._auto_oy, dz, write)
        )

    cmd_XYZCAL_CANCEL_help = (
        "Cancel in-progress XYZCAL_CENTER/FITY/Z/DCXZ/AUTO"
    )

    def cmd_XYZCAL_CANCEL(self, gcmd):
        self._cancel = True
        gcmd.respond_info("XYZCAL_CANCEL requested")

    cmd_XYZCAL_CALIB_STATUS_help = "Report xyzcal_calib phase / last tip / FitY"

    def cmd_XYZCAL_CALIB_STATUS(self, gcmd):
        gcmd.respond_info(
            "xyzcal_calib busy=%s phase=%s matrix=%s fity_vy=%s cx=%.1f cy=%.1f "
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
            "frame_w": self._last_fw,
            "frame_h": self._last_fh,
            "allowed_roi": self._last_allowed_roi,
            "allowed_circle": self._last_allowed_circle,
            "expected_x": self._last_expect_x,
            "expected_y": self._last_expect_y,
            "toolhead_x": self._toolhead_x,
            "toolhead_y": self._toolhead_y,
            "toolhead_z": self._toolhead_z,
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
            "dcxz_measure_mode": self.dcxz_measure_mode,
        }


def load_config(config):
    return XyCalCalib(config)
