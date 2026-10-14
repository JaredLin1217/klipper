# Runtime probe offsets and bounded T0 zero calibration for Vulcan600.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import configparser
import copy
import math
import os
import tempfile

from . import probe as native_probe


# The probe module is shared by the CN and TW configuration exports.  Keep
# one executable module, but localize every user-facing message from the
# active configuration's probe macro names.  The mapping is intentionally
# character based because the module emits formatted status and error text.
_CN_TO_TW = str.maketrans({
    "个": "個", "为": "為", "于": "於", "产": "產", "从": "從",
    "会": "會", "关": "關", "内": "內", "准": "準", "动": "動",
    "参": "參", "发": "發", "号": "號", "后": "後", "启": "啟",
    "喷": "噴", "围": "圍", "备": "備", "复": "復", "对": "對",
    "并": "並", "开": "開", "异": "異", "归": "歸", "态": "態",
    "执": "執", "换": "換", "数": "數", "断": "斷", "无": "無",
    "时": "時", "显": "顯", "暂": "暫", "标": "標", "档": "檔",
    "检": "檢", "测": "測", "点": "點", "状": "狀", "独": "獨",
    "确": "確", "组": "組", "结": "結", "网": "網", "范": "範",
    "视": "視", "触": "觸", "认": "認", "讯": "訊", "设": "設",
    "试": "試", "请": "請", "读": "讀", "败": "敗", "资": "資",
    "轴": "軸", "载": "載", "达": "達", "过": "過", "须": "須",
    "键": "鍵", "闭": "閉", "间": "間", "稳": "穩", "释": "釋",
    "针": "針", "头": "頭",
})

# Cover the remaining unambiguous Simplified-to-Traditional characters used
# by the shared module's user-facing messages.  Keep this independent of the
# locale catalog because the module is installed outside the copied config.
_CN_TO_TW.update(str.maketrans({
    "与": "與", "专": "專", "两": "兩", "个": "個", "义": "義",
    "于": "於", "产": "產", "仅": "僅", "仓": "倉", "仪": "儀",
    "会": "會", "体": "體", "侦": "偵", "侧": "側", "偿": "償",
    "储": "儲", "关": "關", "内": "內", "写": "寫", "准": "準",
    "凭": "憑", "则": "則", "刚": "剛", "删": "刪", "别": "別",
    "务": "務", "动": "動", "单": "單", "却": "卻", "厂": "廠",
    "历": "歷", "参": "參", "双": "雙", "发": "發", "变": "變",
    "号": "號", "后": "後", "启": "啟", "喷": "噴", "围": "圍",
    "处": "處", "备": "備", "够": "夠", "实": "實", "宽": "寬",
    "对": "對", "将": "將", "层": "層", "带": "帶", "幂": "冪",
    "应": "應", "开": "開", "弹": "彈", "强": "強", "归": "歸",
    "录": "錄", "态": "態", "户": "戶", "执": "執", "护": "護",
    "拟": "擬", "拦": "攔", "择": "擇", "挤": "擠", "换": "換",
    "据": "據", "数": "數", "断": "斷", "无": "無", "旧": "舊",
    "时": "時", "显": "顯", "暂": "暫", "机": "機", "条": "條",
    "来": "來", "极": "極", "构": "構", "标": "標", "栏": "欄",
    "档": "檔", "检": "檢", "槛": "檻", "毕": "畢", "没": "沒",
    "测": "測", "温": "溫", "满": "滿", "灯": "燈", "点": "點",
    "热": "熱", "状": "狀", "独": "獨", "电": "電", "画": "畫",
    "监": "監", "盖": "蓋", "码": "碼", "确": "確", "称": "稱",
    "类": "類", "紧": "緊", "线": "線", "组": "組", "终": "終",
    "结": "結", "给": "給", "统": "統", "继": "繼", "绪": "緒",
    "续": "續", "缩": "縮", "网": "網", "范": "範", "虚": "虛",
    "补": "補", "装": "裝", "视": "視", "触": "觸", "计": "計",
    "订": "訂", "认": "認", "议": "議", "讯": "訊", "记": "記",
    "许": "許", "设": "設", "证": "證", "试": "試", "询": "詢",
    "该": "該", "误": "誤", "请": "請", "读": "讀", "调": "調",
    "败": "敗", "资": "資", "转": "轉", "软": "軟", "轴": "軸",
    "载": "載", "辅": "輔", "辑": "輯", "输": "輸", "边": "邊",
    "达": "達", "过": "過", "运": "運", "这": "這", "进": "進",
    "连": "連", "选": "選", "逻": "邏", "针": "針", "钮": "鈕",
    "锁": "鎖", "错": "錯", "键": "鍵", "长": "長", "门": "門",
    "闭": "閉", "间": "間", "阵": "陣", "际": "際", "韧": "韌",
    "页": "頁", "项": "項", "顺": "順", "须": "須", "预": "預",
    "频": "頻", "额": "額", "风": "風",
}))


class VulcanProbe:
    KEYS = ("probe_x_offset", "probe_y_offset", "probe_z_offset")

    def __init__(self, config):
        self.printer = config.get_printer()
        self.locale = self._detect_locale(config)
        self.gcode = self.printer.lookup_object("gcode")
        self.probe = self.printer.load_object(config, "probe")
        self.saved = self.printer.load_object(config, "save_variables")
        self.offset_helper = self.probe.probe_offsets
        self.calibrating = False
        self.meshing = False
        self.last_mesh_calibration = None
        self.last_calibration = None
        self.last_speed_test = None
        self.calibration_speed = config.getfloat(
            "calibration_probe_speed", 0.1, above=0., maxval=5.)
        self.calibration_lift_speed = config.getfloat(
            "calibration_lift_speed", 1., above=0., maxval=5.)
        self.calibration_tolerance = config.getfloat(
            "calibration_tolerance", 0.01, above=0., maxval=0.01)
        if not all(math.isfinite(value) for value in (
                self.calibration_speed, self.calibration_lift_speed,
                self.calibration_tolerance)):
            raise config.error(self._t('探针校准速度及差值上限必须是有限数值。'))
        self.virtual_endstop = self.printer.lookup_object("pins").chips["probe"]
        mesh = config.getsection("bed_mesh")
        self.move_z = mesh.getfloat("horizontal_move_z", 5., note_valid=False)
        self.mesh_min = mesh.getfloatlist("mesh_min", count=2, note_valid=False)
        self.mesh_max = mesh.getfloatlist("mesh_max", count=2, note_valid=False)
        self.axis_limits = []
        for axis in ("x", "y"):
            section = config.getsection("stepper_" + axis)
            self.axis_limits.append((
                section.getfloat("position_min", 0., note_valid=False),
                section.getfloat("position_max", note_valid=False)))
        z_pin = config.getsection("stepper_z").get("endstop_pin",
                                                   note_valid=False)
        if "probe:z_virtual_endstop" in z_pin:
            raise config.error(self._t('即时探针偏移功能需要保留独立的 Z 轴归零开关。'))
        defaults = self.probe.get_offsets()
        values = tuple(self.saved.allVariables.get(key, default)
                       for key, default in zip(self.KEYS, defaults))
        values = self._validate(values, config.error)
        if any(key not in self.saved.allVariables for key in self.KEYS):
            try:
                self._persist(values)
            except Exception as exc:
                raise config.error(self._t('无法初始化探针偏移值：%s' % (exc,)))
        self._apply(values)
        self.gcode.register_command(
            "_VULCAN_SET_PROBE_OFFSET", self.cmd_SET_PROBE_OFFSET,
            desc=self._t('保存并立即套用探针 X／Y／Z 偏移'))
        self.gcode.register_command(
            "_VULCAN_CALIBRATE_PROBE_ZERO", self.cmd_CALIBRATE_PROBE_ZERO,
            desc=self._t('以 T0 零点为基准，从 5 至 0 mm 自动校准并保存探针 Z 偏移'))
        self.gcode.register_command(
            "_VULCAN_BED_MESH_CALIBRATE", self.cmd_BED_MESH_CALIBRATE,
            desc=self._t('一键自动量测、收回探针并保存床网格'))
        # Result prompts are configuration macros.  Keep this module focused
        # on probing, persistence and safety cleanup; bed_leveling.cfg owns
        # the locale-specific Mainsail presentation.
        self.printer.register_event_handler(
            "klippy:connect", self._register_result_macros)
        # Keep native calibration formulas and SAVE_CONFIG behaviour intact.
        # Persist the native result so startup cannot restore an older offset.
        helper = self.probe.cmd_helper
        self.native_calibrate_finalize = helper.probe_calibrate_finalize
        helper.probe_calibrate_finalize = self._calibrate_finalize
        self.native_apply_z = self.gcode.register_command(
            "Z_OFFSET_APPLY_PROBE", None)
        if self.native_apply_z is None:
            raise config.error(self._t('找不到原生 Z_OFFSET_APPLY_PROBE 指令。'))
        self.gcode.register_command(
            "Z_OFFSET_APPLY_PROBE", self.cmd_Z_OFFSET_APPLY_PROBE,
            desc=helper.cmd_Z_OFFSET_APPLY_PROBE_help)

    def _register_result_macros(self):
        # These three configuration macros only render result/error/close
        # messages.  Permit them after an MCU shutdown, without enabling any
        # probing or motion command while the printer is not ready.
        for name in ("_VULCAN_SHOW_RESULT", "_VULCAN_RAISE_RESULT_ERROR",
                     "_VULCAN_CLOSE_RESULT"):
            macro = self.printer.lookup_object("gcode_macro " + name, None)
            if macro is not None:
                self.gcode.register_command(name, None)
                self.gcode.register_command(
                    name, macro.cmd, when_not_ready=True, desc=macro.cmd_desc)

    @staticmethod
    def _detect_locale(config):
        """Detect the locale from an explicit option or paired macro names."""
        explicit = config.get("locale", None)
        if explicit:
            locale = str(explicit).strip().upper()
            if locale in ("CN", "TW"):
                return locale
        fileconfig = getattr(config, "fileconfig", None)
        if fileconfig is not None:
            if fileconfig.has_section("gcode_macro 探針_伸出"):
                return "TW"
            if fileconfig.has_section("gcode_macro 探针_伸出"):
                return "CN"
        return "CN"

    def _t(self, text):
        text = str(text)
        return text.translate(_CN_TO_TW) if self.locale == "TW" else text

    def _command_error(self, text):
        return self.printer.command_error(self._t(text))

    def _localize_gcmd(self, gcmd):
        """Keep parser errors localized; result presentation is in config."""
        if getattr(gcmd, "_vulcan_locale_wrapped", False):
            return gcmd
        error = gcmd.error

        def localized_error(message):
            return error(self._t(message))

        gcmd.error = localized_error
        gcmd._vulcan_locale_wrapped = True
        return gcmd

    def _run_probe_macro(self, action):
        names = {
            "deploy": ('探针_伸出', '探針_伸出'),
            "stow": ('探针_收回', '探針_收回'),
        }.get(action)
        if names is None:
            raise self._command_error('未知的探针动作：%s' % action)
        handlers = getattr(self.gcode, "gcode_handlers", {})
        for name in names:
            if name in handlers:
                self.gcode.run_script_from_command(name + "\nM400")
                return
        raise self._command_error(
            '找不到探针%s宏（CN/TW）' % ('伸出' if action == "deploy" else '收回'))

    def _validate(self, values, error):
        if any(isinstance(value, bool) or
               not isinstance(value, (int, float)) for value in values):
            raise error(self._t('探针 X／Y／Z 偏移必须是有效数字。'))
        values = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in values):
            raise error(self._t('探针偏移不可使用 NaN 或无限大。'))
        if not 0. <= values[2] < self.move_z:
            raise error(self._t(
                '探针 Z 偏移必须大于等于 0 且小于量测移动高度 %.3f mm。'
                % (self.move_z,)))
        for i, axis in enumerate(("X", "Y")):
            low, high = self.axis_limits[i]
            if (self.mesh_min[i] - values[i] < low or
                    self.mesh_max[i] - values[i] > high):
                raise error(self._t(
                    '探针 %s 偏移会使目前网格量测位置超出行程。' % (axis,)))
        return values

    def _check_idle(self, error):
        if getattr(self, "meshing", False):
            raise error('自动床网格校准正在执行，请等待完成。')
        if self.calibrating:
            raise error('探针零点校准正在执行，请等待完成。')
        eventtime = self.printer.get_reactor().monotonic()
        print_stats = self.printer.lookup_object("print_stats", None)
        pause = self.printer.lookup_object("pause_resume", None)
        if ((print_stats is not None and
             print_stats.get_status(eventtime)["state"]
             in ("printing", "paused"))
                or (pause is not None and
                    pause.get_status(eventtime)["is_paused"])):
            raise error('列印或暂停期间不能修改探针偏移。')
        virtual_sd = self.printer.lookup_object("virtual_sdcard", None)
        if (virtual_sd is not None and
                virtual_sd.get_status(eventtime)["is_active"]):
            raise error('列印档案正在执行，不能修改探针偏移。')
        manual = self.printer.lookup_object("manual_probe", None)
        if (self.probe.probe_session.hw_probe_session is not None or
                (manual is not None and
                 manual.get_status(eventtime)["is_active"])):
            raise error('量测或校正期间不能修改探针偏移。')

    def _persist(self, values):
        variables = dict(self.saved.allVariables)
        variables.update(zip(self.KEYS, values))
        if variables == self.saved.allVariables:
            return
        parser = configparser.ConfigParser()
        parser.add_section("Variables")
        for key, value in sorted(variables.items()):
            parser.set("Variables", key, repr(value))

        def write_atomic():
            path = self.saved.filename
            directory = os.path.dirname(os.path.abspath(path))
            mode = os.stat(path).st_mode & 0o777
            staging = None
            try:
                with tempfile.NamedTemporaryFile(
                        mode="w", encoding="utf-8", dir=directory,
                        prefix=".probe-offset-", suffix=".pending",
                        delete=False) as output:
                    staging = output.name
                    os.fchmod(output.fileno(), mode)
                    parser.write(output)
                    output.flush()
                    os.fsync(output.fileno())
                os.replace(staging, path)
                staging = None
            finally:
                if staging is not None and os.path.exists(staging):
                    os.unlink(staging)

        # The existing save_variables worker yields the Klipper reactor
        # during IO.
        self.saved.executor.submit(write_atomic)
        self.saved.allVariables = variables

    def _apply(self, values):
        self.offset_helper.x_offset, self.offset_helper.y_offset,\
            self.offset_helper.z_offset = values
        self.virtual_endstop.position_endstop = values[2]

    def cmd_SET_PROBE_OFFSET(self, gcmd):
        self._localize_gcmd(gcmd)
        supplied = gcmd.get_command_parameters()
        unknown = set(supplied) - {"X", "Y", "Z"}
        if unknown:
            raise gcmd.error('不支援的探针参数：%s' % (", ".join(sorted(unknown)),))
        if not supplied:
            # The public bed_leveling macro owns the human-readable status
            # output.  An empty setter call is therefore a no-op so that
            # blank X/Y/Z fields keep their existing values.
            return
        self._check_idle(gcmd.error)
        values = list(self.probe.get_offsets())
        for i, axis in enumerate(("X", "Y", "Z")):
            if axis in supplied:
                try:
                    values[i] = float(supplied[axis])
                except (TypeError, ValueError):
                    raise gcmd.error('探针 %s 偏移必须是有效数字。' % (axis,))
        values = self._validate(values, gcmd.error)
        try:
            self._persist(values)
        except Exception as exc:
            raise gcmd.error('探针偏移保存失败，原设定未更动：%s' % (exc,))
        self._apply(values)
        pending = self.printer.lookup_object("configfile").get_status(None)
        pending_probe = pending["save_config_pending_items"].get("probe") or {}
        if "z_offset" in pending_probe:
            self.printer.lookup_object("configfile").set(
                "probe", "z_offset", "%.3f" % (values[2],))

    def _calibrate_finalize(self, result):
        if result is not None:
            self._check_idle(self._command_error)
            helper = self.probe.cmd_helper
            position, offsets = helper.probe_calibrate_info
            z_offset = float("%.3f" % (
                offsets[2] - result.bed_z + position.bed_z,))
            values = self._validate(
                self.probe.get_offsets()[:2] + (z_offset,),
                self._command_error)
            try:
                self._persist(values)
            except Exception as exc:
                raise self._command_error(
                    '探针校正值保存失败：%s' % (exc,))
        self.native_calibrate_finalize(result)

    def cmd_Z_OFFSET_APPLY_PROBE(self, gcmd):
        self._localize_gcmd(gcmd)
        self._check_idle(gcmd.error)
        offset = self.printer.lookup_object("gcode_move").get_status(
        )["homing_origin"].z
        if offset != 0.:
            values = self._validate(
                self.probe.get_offsets()[:2] +
                (float("%.3f" % (self.probe.get_offsets()[2] - offset,)),),
                gcmd.error)
            try:
                self._persist(values)
            except Exception as exc:
                raise gcmd.error('探针校正值保存失败：%s' % (exc,))
        self.native_apply_z(gcmd)

    def _query_pin(self, toolhead):
        helper = self.probe.cmd_helper
        state = helper.query_endstop(toolhead.get_last_move_time())
        helper.last_state = state
        return bool(state)

    def _t1_z_offset(self):
        # The probe is deployed by selecting T1.  toolhead.manual_move() uses
        # machine coordinates, while all saved probe and mesh results are
        # reported in the T0 reference frame.  Keep the conversion in one
        # place for both calibration operations.
        try:
            t1_z = float(self.saved.allVariables.get("t1_z", 0.0))
        except (TypeError, ValueError):
            raise self._command_error(
                "T1 Z 偏移不是有效数值，已取消探针操作。")
        if not math.isfinite(t1_z):
            raise self._command_error(
                "T1 Z 偏移不是有限数值，已取消探针操作。")
        return t1_z

    def _measure_zero_window(self, gcmd, toolhead, probe_speed, tolerance):
        self._localize_gcmd(gcmd)
        # The native homing helper reports machine-frame positions. The T1
        # compensation is removed below so the saved result stays T0-relative.
        helper = self.probe.mcu_probe.homing_helper
        previous_floor = helper.z_min_position
        probe_cmd = self.gcode.create_gcode_command(
            "_VULCAN_CALIBRATE_PROBE_ZERO", "", {
                "SAMPLES": "1", "PROBE_SPEED": str(probe_speed),
                "LIFT_SPEED": str(self.calibration_lift_speed)})
        measurements = []
        t1_z = self._t1_z_offset()
        # The active T1 tool changes the machine-frame height by t1_z.  Start
        # at the requested T0 reference Z=5 plus that compensation, then
        # convert the trigger position back to the T0 frame before checking or
        # saving it.  This prevents a valid probe from being seen as already
        # triggered when the saved T1 Z compensation is non-zero.
        start_z = max(5.0, 5.0 + t1_z)
        limits = toolhead.get_status(
            self.printer.get_reactor().monotonic())
        low_z = limits["axis_minimum"][2]
        high_z = limits["axis_maximum"][2]
        if start_z < low_z or start_z > high_z:
            raise self._command_error(
                "探针零点校准起始高度超出 Z 轴行程。")
        helper.z_min_position = 0.
        try:
            for sample in range(3):
                toolhead.manual_move([None, None, start_z],
                                     self.calibration_lift_speed)
                toolhead.wait_moves()
                if self._query_pin(toolhead):
                    raise gcmd.error(
                        '探针在 T0 基准 Z=5 mm 已触发，已取消校准并保留原偏移。')
                try:
                    result = native_probe.run_single_probe(
                        self.probe, probe_cmd)
                except self.printer.command_error as exc:
                    if "No trigger on probe" in str(exc):
                        raise gcmd.error(
                            '探针在 T0 基准 Z=5 至 0 mm 内未触发；'
                            '已停止下降并保留原偏移。')
                    raise
                raw_trigger_z = result.test_z
                trigger_z = raw_trigger_z - t1_z
                if (not math.isfinite(raw_trigger_z)
                        or not math.isfinite(trigger_z)
                        or not 0. <= trigger_z < 5.):
                    raise gcmd.error(
                        '探针触发高度不在 T0 基准的 0 至 5 mm 范围内。')
                toolhead.wait_moves()
                if not self._query_pin(toolhead):
                    raise gcmd.error('触发后的探针讯号不稳定，已保留原偏移。')
                measurements.append(trigger_z)
                self.last_calibration = dict(self.last_calibration, **{
                    "current_round_heights": list(measurements),
                    "current_sample": sample + 1})
        finally:
            helper.z_min_position = previous_floor
            # Native run_single_probe does not end a session on failure.
            if self.probe.probe_session.hw_probe_session is not None:
                self.probe.probe_session.end_probe_session()
        spread = max(measurements) - min(measurements)
        if tolerance is not None and spread > tolerance + 1.e-9:
            raise gcmd.error(
                '三次探针量测差异 %.9f mm 超过 %.6f mm，已保留原偏移。'
                % (spread, tolerance))
        toolhead.manual_move([None, None, start_z],
                             self.calibration_lift_speed)
        toolhead.wait_moves()
        if self._query_pin(toolhead):
            raise gcmd.error('探针退回 T0 基准 Z=5 mm 后未释放，已保留原偏移。')
        return measurements

    def _finish_zero_motion(self, toolhead):
        # Lift before the T1 -> T0 switch so both the probe and T0 nozzle clear
        # the bed. The public stow macro retains the displayed XYZ position.
        if toolhead.get_extruder().get_name() == "extruder1":
            clear_z = max(20., toolhead.get_position()[2])
            toolhead.manual_move([None, None, clear_z], 20.)
            toolhead.wait_moves()
            self._run_probe_macro("stow")
        self.gcode.run_script_from_command("G28\nM400")
        self._verify_t0_frame(toolhead)

    def _verify_t0_frame(self, toolhead):
        status = toolhead.get_status(self.printer.get_reactor().monotonic())
        move = self.printer.lookup_object("gcode_move").get_status()
        if (status["homed_axes"] != "xyz"
                or status["extruder"] != "extruder"
                or any(abs(v) > 0.000001 for v in move["homing_origin"][:3])
                or any(abs(move["position"][i] - move["gcode_position"][i])
                       > 0.000001 for i in range(3))):
            raise self._command_error(
                'T0 归零座标未正确建立，已取消探针校准。')

    def cmd_CALIBRATE_PROBE_ZERO(self, gcmd):
        self._localize_gcmd(gcmd)
        # Reset before parsing, so a rejected request cannot expose an old
        # successful result or claim an offset was saved by this attempt.
        self.last_calibration = {
            "mode": "calibration", "completed": False, "saved": False,
            "error": None, "rounds": [], "trigger_heights": []}
        self.last_speed_test = None
        raise_on_error = True
        try:
            raise_on_error = gcmd.get_int(
                "RAISE_ON_ERROR", 1, minval=0, maxval=1)
            self._calibrate_probe_zero(gcmd)
        except Exception as exc:
            self._record_calibration_error(exc)
            if raise_on_error:
                raise

    def _record_calibration_error(self, error):
        """Publish a structured result for the configuration UI macro."""
        result = dict(self.last_calibration or {})
        result.update({
            "completed": False,
            "saved": bool(result.get("saved", False)),
            "error": " ".join(str(error).split()),
        })
        self.last_calibration = result
        if self.last_speed_test is not None and not self.last_speed_test.get(
                "completed", False):
            self.last_speed_test = dict(self.last_speed_test,
                                        error=result["error"])

    def _calibrate_probe_zero(self, gcmd):
        self._localize_gcmd(gcmd)
        supplied = gcmd.get_command_parameters()
        unknown = set(supplied) - {
            "PROBE_SPEED", "ROUNDS", "SAVE", "RAISE_ON_ERROR"}
        if unknown:
            raise gcmd.error('不支援的校准参数：%s' % (
                ", ".join(sorted(unknown)),))
        probe_speed = gcmd.get_float(
            "PROBE_SPEED", self.calibration_speed, above=0., maxval=5.)
        if not math.isfinite(probe_speed):
            raise gcmd.error('探针校准速度必须是有限数值。')
        rounds = gcmd.get_int("ROUNDS", 1, minval=1, maxval=5)
        save = gcmd.get_int("SAVE", 1, minval=0, maxval=1)
        self.last_calibration = dict(self.last_calibration, **{
            "mode": "calibration" if save else "accuracy",
            "save_requested": bool(save),
            "probe_speed": probe_speed, "requested_rounds": rounds,
            "tolerance": self.calibration_tolerance,
        })
        self._check_idle(gcmd.error)
        toolhead = self.printer.lookup_object("toolhead")
        move = self.printer.lookup_object("gcode_move")
        mesh = self.printer.lookup_object("bed_mesh")
        thermal = self.printer.lookup_object("z_thermal_adjust", None)
        settings = move.get_status()
        mesh_before = mesh.z_mesh
        thermal_before = (thermal.capture_state()
                          if thermal is not None else None)
        offsets = self.probe.get_offsets()
        target_xy = (300. - offsets[0], 250. - offsets[1])
        for i, value in enumerate(target_xy):
            if not self.axis_limits[i][0] <= value <= self.axis_limits[i][1]:
                raise gcmd.error('平台中心的探针位置超出行程，已取消校准。')
        homing_helper = getattr(self.probe.mcu_probe, "homing_helper", None)
        if (homing_helper is None
                or not hasattr(homing_helper, "z_min_position")):
            raise gcmd.error('目前的探针模组不支援限制校准下降范围。')
        self.calibrating = True
        failure = None
        measurements = []
        round_results = []
        if not save:
            self.last_speed_test = {
                "probe_speed": probe_speed,
                "lift_speed": self.calibration_lift_speed,
                "tolerance": self.calibration_tolerance,
                "requested_rounds": rounds, "rounds": round_results,
                "saved": False, "completed": False}
        homed_for_calibration = False
        try:
            # Establish the T0 machine reference with no old mesh or applied
            # thermal bias; G28 must preserve this disabled state.
            mesh.set_mesh(None)
            if thermal is not None:
                self.gcode.run_script_from_command(
                    "SET_Z_THERMAL_ADJUST ENABLE=0")
            self.gcode.run_script_from_command(
                "M220 S100\nM221 S100\nG28\nM400")
            self._verify_t0_frame(toolhead)
            homed_for_calibration = True
            # Existing mesh and thermal compensation must not bias the fixed
            # T0 reference. Restore both after the final home.
            mesh.set_mesh(None)
            if thermal is not None:
                self.gcode.run_script_from_command(
                    "SET_Z_THERMAL_ADJUST ENABLE=0")
            self._run_probe_macro("deploy")
            if toolhead.get_extruder().get_name() != "extruder1":
                raise gcmd.error('探针未切换至 T1，已取消校准。')
            toolhead.manual_move([None, None, 20.], 20.)
            toolhead.manual_move(list(target_xy) + [None], 50.)
            toolhead.wait_moves()
            for round_number in range(1, rounds + 1):
                self.last_calibration = dict(self.last_calibration, **{
                    "current_round": round_number,
                    "current_round_heights": [], "current_sample": 0})
                positions = self._measure_zero_window(
                    gcmd, toolhead, probe_speed,
                    self.calibration_tolerance if save else None)
                measurements.extend(positions)
                spread = max(positions) - min(positions)
                passed = spread <= self.calibration_tolerance + 1.e-9
                round_results.append({
                    "round": round_number, "trigger_heights": positions,
                    "spread": spread, "mean": sum(positions) / 3.,
                    "passed": passed})
                self.last_calibration = dict(self.last_calibration, **{
                    "rounds": list(round_results),
                    "trigger_heights": list(measurements)})
            values = self._validate(
                offsets[:2] + (round(sum(measurements) /
                                    len(measurements), 6),), gcmd.error)
        except Exception as exc:
            failure = str(exc)
        finally:
            try:
                if homed_for_calibration and not self.printer.is_shutdown():
                    status = toolhead.get_status(
                        self.printer.get_reactor().monotonic())
                    if status["homed_axes"] != "xyz":
                        raise gcmd.error(
                            '校准中断后轴已失去归零，未执行自动返回。')
                    self._finish_zero_motion(toolhead)
            except Exception as exc:
                failure = ('%s；返回失败：%s' % (failure, exc)
                           if failure else '校准返回失败：%s' % (exc,))
            try:
                if not self.printer.is_shutdown():
                    mesh.set_mesh(mesh_before)
                    if thermal is not None:
                        thermal.restore_state(thermal_before)
                    self.gcode.run_script_from_command(
                        "M220 S%.6f\nM221 S%.6f\n%s\n%s\nG1 F%.6f"
                        % (settings["speed_factor"] * 100.,
                           settings["extrude_factor"] * 100.,
                           "G90" if settings["absolute_coordinates"] else "G91",
                           "M82" if settings["absolute_extrude"] else "M83",
                           settings["speed"]))
            except Exception as exc:
                failure = ('%s；状态恢复失败：%s' % (failure, exc)
                           if failure else '校准状态恢复失败：%s' % (exc,))
            self.calibrating = False
        if failure is not None:
            if not save:
                self.last_speed_test["error"] = failure
            raise gcmd.error('探针零点校准失败，原偏移未更动：%s' % (failure,))
        if not save:
            self.last_speed_test.update({
                "completed": True,
                "all_passed": all(r["passed"] for r in round_results),
                "overall_spread": max(measurements) - min(measurements),
                "max_round_spread": max(r["spread"] for r in round_results)})
            self.last_calibration = dict(self.last_calibration, **{
                "completed": True, "saved": False, "error": None,
                "rounds": round_results,
                "all_passed": self.last_speed_test["all_passed"],
                "overall_spread": self.last_speed_test["overall_spread"],
                "max_round_spread": self.last_speed_test["max_round_spread"]})
            return
        # Do not update persistent/runtime values until all probing and the
        # final T0 home have succeeded. Only probe_z_offset is changed.
        try:
            self._persist(values)
        except Exception as exc:
            raise gcmd.error('探针零点保存失败，原偏移未更动：%s' % (exc,))
        self._apply(values)
        self.last_calibration = dict(self.last_calibration, **{
            "saved": True, "previous_z_offset": offsets[2],
            "z_offset": values[2]})
        pending = self.printer.lookup_object("configfile").get_status(None)
        pending_probe = pending["save_config_pending_items"].get("probe") or {}
        if "z_offset" in pending_probe:
            self.printer.lookup_object("configfile").set(
                "probe", "z_offset", "%.6f" % (values[2],))
        self.last_calibration = {
            "mode": "calibration", "completed": True, "saved": True,
            "error": None, "reference_tool": "T0", "start_z": 5.,
            "minimum_z": 0.,
            "sample_count": len(measurements), "trigger_heights": measurements,
            "probe_speed": probe_speed, "rounds": round_results,
            "spread": max(measurements) - min(measurements),
            "previous_z_offset": offsets[2], "z_offset": values[2]}

    def _restore_mesh_modes(self, settings, thermal, thermal_before):
        if thermal is not None:
            thermal.restore_state(thermal_before)
        self.gcode.run_script_from_command(
            "M220 S%.6f\nM221 S%.6f\n%s\n%s\nG1 F%.6f"
            % (settings["speed_factor"] * 100.,
               settings["extrude_factor"] * 100.,
               "G90" if settings["absolute_coordinates"] else "G91",
               "M82" if settings["absolute_extrude"] else "M83",
               settings["speed"]))

    def _finish_mesh_motion(self, toolhead):
        # Machine coordinates are independent of the active tool's UI offset.
        clear_z = max(20., toolhead.get_position()[2])
        toolhead.manual_move([None, None, clear_z], 20.)
        toolhead.wait_moves()
        if toolhead.get_extruder().get_name() == "extruder1":
            # Native probing can leave an endpoint a few microns beyond an
            # axis boundary. The public stow macro must retain its strict
            # displayed-position checks, so stage the automatic mesh return
            # inside the bed before asking it to retain that position.
            toolhead.manual_move([300., 250., None], 50.)
            toolhead.wait_moves()
            self._run_probe_macro("stow")
        self._verify_t0_frame(toolhead)
        toolhead.manual_move([None, None, max(20., toolhead.get_position()[2])],
                             20.)
        toolhead.manual_move([300., 250., None], 50.)
        toolhead.wait_moves()
        if self._query_pin(toolhead):
            raise self._command_error('探针收回后讯号仍为触发，请检查探针。')

    def _save_mesh_config(self, configfile):
        # This device's SAVE_CONFIG writes in place without restarting.
        # Verify the native autosave block before clearing our pending entry.
        path = self.printer.get_start_args()["config_file"]
        with open(path, encoding="utf-8") as config_input:
            before = config_input.read()
        autosave = configfile.autosave
        expected = dict(autosave.fileconfig.items("bed_mesh default"))
        try:
            self.gcode.run_script_from_command("SAVE_CONFIG")
            with open(path, encoding="utf-8") as config_input:
                written = config_input.read()
            unused, saved_data = autosave._find_autosave_data(written)
            parsed = configparser.ConfigParser(interpolation=None)
            parsed.read_string(saved_data)
            if (not parsed.has_section("bed_mesh default") or any(
                    " ".join(parsed.get("bed_mesh default", key).split())
                    != " ".join(value.split())
                    for key, value in expected.items())):
                raise self._command_error('保存后读回的网格资料不一致。')
        except Exception:
            # Protect printer.cfg if its existing in-place writer fails.
            staging = None
            try:
                with tempfile.NamedTemporaryFile(
                        mode="w", encoding="utf-8",
                        dir=os.path.dirname(os.path.abspath(path)),
                        prefix=".bed-mesh-", suffix=".pending",
                        delete=False) as output:
                    staging = output.name
                    os.fchmod(output.fileno(), os.stat(path).st_mode & 0o777)
                    output.write(before)
                    output.flush()
                    os.fsync(output.fileno())
                os.replace(staging, path)
                staging = None
            finally:
                if staging is not None and os.path.exists(staging):
                    os.unlink(staging)
            raise
        pending = dict(autosave.status_save_pending)
        if pending.get("bed_mesh default") == expected:
            del pending["bed_mesh default"]
            autosave.status_save_pending = pending
            autosave.save_config_pending = bool(pending)


    def _measure_mesh_with_settle(self, toolhead, script):
        averaging = self.probe.probe_session
        native_sample = averaging._probe
        def settled_sample(gcmd):
            toolhead.wait_moves()
            toolhead.dwell(0.5)
            return native_sample(gcmd)
        # Add settling only for this operation; retain native averaging/retries.
        averaging._probe = settled_sample
        try:
            self.gcode.run_script_from_command(script)
        finally:
            averaging._probe = native_sample


    def cmd_BED_MESH_CALIBRATE(self, gcmd):
        self._localize_gcmd(gcmd)
        self.last_mesh_calibration = {
            "completed": False, "saved": False, "profile": "default",
            "error": None,
        }
        raise_on_error = True
        try:
            raise_on_error = gcmd.get_int(
                "RAISE_ON_ERROR", 1, minval=0, maxval=1)
            self._run_mesh_calibration(gcmd)
        except Exception as exc:
            self._record_mesh_error(exc)
            if raise_on_error:
                raise

    def _record_mesh_error(self, error):
        """Publish a structured mesh result for the configuration UI macro."""
        result = dict(self.last_mesh_calibration or {})
        result.update({
            "completed": False,
            "error": " ".join(str(error).split()),
        })
        self.last_mesh_calibration = result

    def _home_mesh_return(self, toolhead, mesh, new_mesh, settings,
                          thermal, thermal_before):
        # The final home runs only after the measured mesh is safely persisted.
        mesh.set_mesh(None)
        if thermal is not None:
            self.gcode.run_script_from_command("SET_Z_THERMAL_ADJUST ENABLE=0")
        try:
            self.gcode.run_script_from_command("G28\nM400")
            # Retain disabled compensation until the T0 frame and final
            # position have been verified, then restore the caller's mode.
            if thermal is not None:
                self.gcode.run_script_from_command(
                    "SET_Z_THERMAL_ADJUST ENABLE=0")
            self._verify_t0_frame(toolhead)
            toolhead.manual_move([300., 250., None], 50.)
            toolhead.wait_moves()
            position = toolhead.get_position()
            if abs(position[0] - 300.) > 0.01 or abs(position[1] - 250.) > 0.01:
                raise self._command_error('最后归零后未正确返回 X300、Y250。')
        finally:
            if not self.printer.is_shutdown():
                mesh.set_mesh(new_mesh)
                self._restore_mesh_modes(settings, thermal, thermal_before)

    def _run_mesh_calibration(self, gcmd):
        self._localize_gcmd(gcmd)
        supplied = set(gcmd.get_command_parameters()) - {"RAISE_ON_ERROR"}
        if supplied:
            raise gcmd.error('一键床网格校准不接受参数，会使用目前已保存的探针设定。')
        self._check_idle(gcmd.error)
        toolhead = self.printer.lookup_object("toolhead")
        mesh = self.printer.lookup_object("bed_mesh")
        configfile = self.printer.lookup_object("configfile")
        if configfile.get_status(None)["save_config_pending_items"]:
            raise gcmd.error('尚有其他未保存的校准设定，请先完成保存再量测网格。')
        thermal = self.printer.lookup_object("z_thermal_adjust", None)
        settings = self.printer.lookup_object("gcode_move").get_status()
        thermal_before = (thermal.capture_state()
                          if thermal is not None else None)
        offsets = self._validate(self.probe.get_offsets(), gcmd.error)
        t1_z = self._t1_z_offset()
        mesh_probe_z = offsets[2] + t1_z
        if not math.isfinite(mesh_probe_z):
            raise gcmd.error('探针与 T1 Z 偏移合并后不是有限数值。')
        homing_helper = getattr(self.probe.mcu_probe, "homing_helper", None)
        if (homing_helper is None
                or not hasattr(homing_helper, "z_min_position")):
            raise gcmd.error('目前的探针模组不支援限制量测下降范围。')
        autosave = configfile.autosave
        snapshot = (mesh.z_mesh, copy.deepcopy(mesh.pmgr.profiles),
                    copy.deepcopy(autosave.fileconfig),
                    copy.deepcopy(autosave.status_save_pending),
                    autosave.save_config_pending)
        previous_floor = homing_helper.z_min_position
        # Native bed_mesh subtracts probe.z_offset from the machine-frame
        # trigger position.  While T1 is active, that position also includes
        # the saved T1 Z tool offset, so use the effective T1 value only for
        # the native mesh run and restore the persistent T0 value afterwards.
        # Keep a true 5 mm probe-tip clearance in the T0 reference frame.
        clearance = mesh_probe_z + self.move_z
        self.meshing = True
        self.last_mesh_calibration = {
            "completed": False, "saved": False, "profile": "default",
            "error": None,
            "probe_offsets": list(offsets),
            "probe_speed": self.calibration_speed,
            "effective_mesh_probe_z_offset": mesh_probe_z,
            "t1_z_offset": t1_z,
            "samples": 3, "tolerance": self.calibration_tolerance,
            "settle_time": 0.5, "sample_retries": 2,
            "tip_clearance": self.move_z, "horizontal_move_z": clearance}
        start = self.printer.get_reactor().monotonic()
        failure = None
        homed = False
        new_mesh = None
        try:
            mesh.set_mesh(None)
            if thermal is not None:
                self.gcode.run_script_from_command(
                    "SET_Z_THERMAL_ADJUST ENABLE=0")
            self.gcode.run_script_from_command(
                "M220 S100\nM221 S100\nG28\nM400")
            self._verify_t0_frame(toolhead)
            homed = True
            # Keep the measurement frame uncompensated after homing as well.
            if thermal is not None:
                self.gcode.run_script_from_command(
                    "SET_Z_THERMAL_ADJUST ENABLE=0")
            self._run_probe_macro("deploy")
            if toolhead.get_extruder().get_name() != "extruder1":
                raise gcmd.error('探针未切换至 T1，已取消网格校准。')
            toolhead.manual_move([None, None, clearance], 20.)
            toolhead.wait_moves()
            if self._query_pin(toolhead):
                raise gcmd.error('探针在安全移动高度已触发，已取消网格校准。')
            # An absent trigger cannot move below the calibrated T0 Z=0.
            homing_helper.z_min_position = 0.
            original_probe_offsets = self.probe.get_offsets()
            self.offset_helper.z_offset = mesh_probe_z
            try:
                self._measure_mesh_with_settle(toolhead,
                    "BED_MESH_CALIBRATE METHOD=automatic "
                    "PROFILE=default ADAPTIVE=0 "
                    "HORIZONTAL_MOVE_Z=%.6f PROBE_SPEED=%.6f LIFT_SPEED=%.6f "
                    "SAMPLES=3 SAMPLES_RESULT=average SAMPLE_RETRACT_DIST=1 "
                    "SAMPLES_TOLERANCE=%.6f SAMPLES_TOLERANCE_RETRIES=2"
                    % (clearance, self.calibration_speed,
                       self.calibration_lift_speed, self.calibration_tolerance))
            finally:
                # Keep the configured probe offset T0-relative after the
                # native callback has converted every measured point.
                (self.offset_helper.x_offset,
                 self.offset_helper.y_offset,
                 self.offset_helper.z_offset) = original_probe_offsets
            new_mesh = mesh.z_mesh
            if new_mesh is None or new_mesh.get_profile_name() != "default":
                raise gcmd.error('原生网格量测未产生完整的 default 网格。')
            points = new_mesh.get_probed_matrix()
            if (not points or any(not math.isfinite(z)
                                  for row in points for z in row)):
                raise gcmd.error('网格量测结果无效，未保存。')
            self.last_mesh_calibration = dict(self.last_mesh_calibration, **{
                "point_count": sum(map(len, points)),
                "minimum": min(z for row in points for z in row),
                "maximum": max(z for row in points for z in row)})
        except Exception as exc:
            failure = str(exc)
        finally:
            homing_helper.z_min_position = previous_floor
            try:
                if self.probe.probe_session.hw_probe_session is not None:
                    self.probe.probe_session.end_probe_session()
                if not self.printer.is_shutdown():
                    # Disable mesh compensation during the physical tool switch.
                    mesh.set_mesh(None)
                    if homed:
                        if toolhead.get_status(
                                self.printer.get_reactor().monotonic()
                                )["homed_axes"] != "xyz":
                            raise gcmd.error('量测中断后轴已失去归零，未执行返回。')
                        self._finish_mesh_motion(toolhead)
            except Exception as exc:
                failure = ('%s；安全返回失败：%s' % (failure, exc)
                           if failure else '安全返回失败：%s' % (exc,))
            try:
                if not self.printer.is_shutdown():
                    self._restore_mesh_modes(settings, thermal, thermal_before)
            except Exception as exc:
                failure = ('%s；状态恢复失败：%s' % (failure, exc)
                           if failure else '状态恢复失败：%s' % (exc,))
        committed = False
        try:
            if failure is not None:
                raise gcmd.error(failure)
            mesh.set_mesh(new_mesh)
            self._save_mesh_config(configfile)
            committed = True
            self.last_mesh_calibration = dict(
                self.last_mesh_calibration, saved=True)
            self._home_mesh_return(toolhead, mesh, new_mesh, settings,
                                   thermal, thermal_before)
            self.last_mesh_calibration = dict(self.last_mesh_calibration, **{
                "completed": True, "saved": True,
                "elapsed": self.printer.get_reactor().monotonic() - start})
        except Exception as exc:
            failure = str(exc)
            if not committed:
                mesh.pmgr.profiles = snapshot[1]
                autosave.fileconfig = snapshot[2]
                autosave.status_save_pending = snapshot[3]
                autosave.save_config_pending = snapshot[4]
            if not self.printer.is_shutdown():
                mesh.set_mesh(new_mesh if committed else snapshot[0])
            self.last_mesh_calibration = dict(self.last_mesh_calibration,
                                              error=failure)
        finally:
            self.meshing = False
        if failure is not None:
            raise gcmd.error('自动床网格校准失败，%s：%s' % (
                '网格已保存但最后归零或返回未完成' if committed else '原网格未更动',
                failure))

    def get_status(self, eventtime):
        current = self.probe.get_offsets()
        saved = tuple(self.saved.allVariables[key] for key in self.KEYS)
        return {"x_offset": current[0], "y_offset": current[1],
                "z_offset": current[2], "saved_offsets": dict(zip(
                    ("x_offset", "y_offset", "z_offset"), saved)),
                "saved_matches_current": current == saved,
                "horizontal_move_z": self.move_z, "reference_tool": "T0",
                "calibrating": self.calibrating,
                "meshing": self.meshing,
                "last_mesh_calibration": self.last_mesh_calibration,
                "last_calibration": self.last_calibration,
                "calibration_probe_speed": self.calibration_speed,
                "calibration_lift_speed": self.calibration_lift_speed,
                "calibration_tolerance": self.calibration_tolerance,
                "last_speed_test": self.last_speed_test}

def load_config(config):
    return VulcanProbe(config)
