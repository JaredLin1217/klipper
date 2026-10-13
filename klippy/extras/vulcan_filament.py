# Vulcan600 job-aware filament protection.
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import re

TOOLS = ("extruder", "extruder1")
ALIASES = {"T0": 0, "0": 0, "EXTRUDER": 0,
           "T1": 1, "1": 1, "EXTRUDER1": 1}
BOTH = ("BOTH", "ALL", "T0/T1")

def scan_file(path):
    """Read G-code in an IO worker; count tools that actually extrude."""
    used, selected, first = set(), set(), None
    tool, relative, last_e = 0, False, 0.
    with open(path, encoding="utf-8", errors="replace") as stream:
        for line in stream:
            code = line.split(";", 1)[0].strip().upper()
            if not code:
                continue
            words = code.split()
            cmd = words[0]
            if cmd in ("T0", "T1"):
                tool = int(cmd[1])
                selected.add(tool)
            elif cmd == "ACTIVATE_EXTRUDER":
                match = re.search(r"EXTRUDER=(EXTRUDER1|EXTRUDER)\b", code)
                if match:
                    tool = ALIASES[match.group(1)]
                    selected.add(tool)
            elif cmd in ("M82", "M83"):
                relative = cmd == "M83"
            elif cmd in ("G0", "G1", "G2", "G3", "G92"):
                match = re.search(r"\bE\s*([-+]?\d*\.?\d+)", code)
                if match:
                    value = float(match.group(1))
                    delta = value if relative else value - last_e
                    if cmd != "G92" and delta > 0.:
                        used.add(tool)
                        if first is None:
                            first = tool
                    if cmd == "G92" or not relative:
                        last_e = value
    used = used or selected or {0}
    return sorted(used), first if first is not None else min(used)

class VulcanFilament:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.interval = config.getfloat("check_interval", .25,
                                        minval=.1, maxval=1.)
        self.executor = self.printer.load_object(
            config, "aio_executor").allocate_executor("vulcan_filament")
        self.sensors = [
            config.get("sensor_t0",
                       "filament_switch_sensor filament_sensor_T0"),
            config.get("sensor_t1",
                       "filament_switch_sensor filament_sensor_T1")]
        self.reset()
        self.homing = False
        self.busy = False
        self.queued = False
        commands = {
            "BEGIN": self.cmd_BEGIN, "CHECK": self.cmd_CHECK,
            "HOLD": self.cmd_HOLD, "SWITCH": self.cmd_SWITCH,
            "HOME": self.cmd_HOME, "START": self.cmd_START,
            "RESUME": self.cmd_RESUME, "RESET": self.cmd_RESET}
        for name, handler in commands.items():
            self.gcode.register_command(
                "_VULCAN_FILAMENT_" + name, handler,
                during_temperature_wait=(name in ("HOLD", "RESET")))
        self.printer.register_event_handler("klippy:ready", self._ready)
        self.printer.register_event_handler("virtual_sdcard:reset_file",
                                            self.reset)
    def reset(self, *args):
        self.armed = False
        self.preflight = False
        self.tools = set()
        self.first_tool = 0
        self.filename = None
        self.pending_tool = None
        self.pending_start = None
        self.held_wait = None
        self.blocked = set()
        self.no_park_pause = False
        self.resuming = False
    def _ready(self):
        self.reactor.register_timer(self._poll,
                                    self.reactor.monotonic()+self.interval)
    def _status(self, name):
        obj = self.printer.lookup_object(name, None)
        return {} if obj is None else obj.get_status(self.reactor.monotonic())
    def _macro(self, primary, fallback):
        """Resolve a localized macro name from the active config."""
        for name in (primary, fallback):
            if self.printer.lookup_object(
                    "gcode_macro " + name, None) is not None:
                return name
        return primary
    def _macro_status(self, primary, fallback):
        return self._status("gcode_macro " + self._macro(primary, fallback))
    def _run_macro(self, primary, fallback, suffix=""):
        self._run(self._macro(primary, fallback) + suffix)
    def _active_tool(self):
        name = self._status("toolhead").get("extruder", "extruder")
        return 1 if name == "extruder1" else 0
    def _job_active(self):
        return (self._status("virtual_sdcard").get("is_active", False)
                or self._status("print_stats").get("state")
                in ("printing", "paused")
                or self.pending_start is not None)
    def _monitor_tools(self):
        if self.preflight or self.pending_start is not None:
            return set(self.tools)
        if self.pending_tool is not None:
            return {self.pending_tool}
        active = self._active_tool()
        # Full G28 selects T0, even before a T1-only file selects its nozzle.
        return {active if active in self.tools else self.first_tool}
    def _paused(self):
        return self._status("pause_resume").get("is_paused", False)
    def _run(self, script):
        self.gcode.run_script_from_command(script)
    def _select(self, value, automatic):
        value = str(value).upper()
        if value in ("", "AUTO"):
            return set(automatic)
        if value in BOTH:
            return {0, 1}
        if value in ALIASES:
            return {ALIASES[value]}
        raise self.gcode.error("未知的耗材工具：%s" % value)
    def _flag(self, tool, value):
        name = "need_filament_t%d" % tool
        old = self._status("save_variables").get("variables", {}).get(name, 0)
        if int(old) != value:
            self._run("SAVE_VARIABLE VARIABLE=%s VALUE=%d" % (name, value))
    def _missing(self, tools, clear=False):
        missing = set()
        for tool in sorted(tools):
            sensor = self._status(self.sensors[tool])
            if (not sensor or (sensor.get("enabled", False)
                               and not sensor.get("filament_detected", False))):
                missing.add(tool)
            elif clear:
                self._flag(tool, 0)
        return missing
    def _begin(self, value="AUTO"):
        sd = self._status("virtual_sdcard")
        path = sd.get("file_path")
        inferred, first = {self._active_tool()}, self._active_tool()
        if path:
            try:
                inferred_list, first = self.executor.submit(scan_file, path)
                inferred = set(inferred_list)
            except (OSError, ValueError) as exc:
                raise self.gcode.error("無法確認列印檔案的噴頭使用範圍：%s" % exc)
        explicit = self._select(value, inferred)
        # A loaded print's G-code is authoritative, including T1-only jobs.
        # A legacy TOOL parameter must not add a channel the file never uses.
        self.tools = inferred if path else explicit
        if (not path and len(explicit) == 1
                and str(value).upper() not in ("AUTO", "")):
            first = next(iter(explicit))
        self.first_tool = first
        self.filename = path
        self.armed = True
        self.preflight = True
        passed = self._check(self.tools, preflight=True)
        self.preflight = not passed
        return passed
    def _prompt(self, missing):
        names = "/".join("T%d" % tool for tool in sorted(missing))
        confirm = self._macro("提示_确认", "提示_確認")
        self._run('RESPOND TYPE=command MSG="action:prompt_begin 斷料警告"')
        self._run('RESPOND TYPE=command MSG="action:prompt_text %s '
                  '無耗材或感測器不存在；請完成進料，'
                  '或手動關閉該感測器後再恢復列印。"' % names)
        self._run('RESPOND TYPE=command MSG="action:prompt_footer_button '
                  '確認|%s|primary"' % confirm)
        self._run('RESPOND TYPE=command MSG="action:prompt_show"')
    def _hold(self, missing, preflight=False):
        if not missing:
            return
        for tool in missing:
            self._flag(tool, 1)
        newly_blocked = missing != self.blocked
        self.blocked = set(missing)
        waiting = self.gcode.active_temperature_wait is not None
        if not self._paused() or (self.resuming and waiting):
            pause = self.printer.lookup_object("pause_resume")
            was_sd_paused = (pause.sd_paused and self.resuming
                             and self.pending_start is not None)
            sd = self._status("virtual_sdcard")
            active = self._active_tool()
            temp = self._status(TOOLS[active]).get("target", 0.)
            self._run("SET_GCODE_VARIABLE MACRO=RESUME "
                      "VARIABLE=last_extruder_temp "
                      "VALUE=\"{'restore': True, 'temp': %s}\"" % temp)
            self.no_park_pause = preflight or waiting
            if waiting:
                sensor, target = (sd.get("temperature_wait_sensor"),
                                  sd.get("temperature_wait_target"))
                if not sensor or target is None:
                    raise self.gcode.error("無法安全暫停未知的溫度等待")
                self.held_wait = (sensor, target)
                # Only the waiting heater is stopped; no motion in this lane.
                if not self._paused():
                    pause.send_pause_command()
                    self._run("PAUSE_BASE")
                self._run("SET_HEATER_TEMPERATURE HEATER=%s TARGET=0" % sensor)
            elif preflight:
                self._run("PAUSE_BASE")
            else:
                self._run("PAUSE")
            if was_sd_paused:
                pause.sd_paused = True
        if newly_blocked:
            self._prompt(missing)
    def _check(self, tools, preflight=False):
        missing = self._missing(tools, clear=True)
        if missing:
            self._hold(missing, preflight)
            return False
        return True
    def _poll(self, eventtime):
        if (self.armed and self._job_active() and not self.homing
            and (not self._paused() or self.resuming)
            and (not self.busy
                 or self.gcode.active_temperature_wait is not None)):
            if self._missing(self._monitor_tools()) and not self.queued:
                self.queued = True
                self.reactor.register_callback(self._dispatch_hold)
        return eventtime + self.interval
    def _dispatch_hold(self, eventtime):
        try:
            # Re-read live enabled/detected state AFTER obtaining dispatch.
            self.gcode.run_script("_VULCAN_FILAMENT_HOLD")
        except self.gcode.error:
            logging.exception("Vulcan filament protection")
        finally:
            self.queued = False
    def cmd_HOLD(self, gcmd):
        if (self.armed and self._job_active() and not self.homing
            and (not self._paused() or self.resuming)):
            self._check(self._monitor_tools(),
                        preflight=(self.preflight
                                   or self.pending_start is not None))
    def cmd_BEGIN(self, gcmd):
        self.pending_tool = self.pending_start = self.held_wait = None
        self.blocked = set()
        self._begin(gcmd.get("TOOL", gcmd.get("T", "AUTO")))
    def cmd_CHECK(self, gcmd):
        event = gcmd.get_int("EVENT", 0)
        if not self._job_active() or self.homing:
            return
        if not self.armed:
            self._begin()
        requested = self._select(gcmd.get("T", "AUTO"),
                                 {self._active_tool()})
        requested &= self._monitor_tools()
        if requested and not self._paused():
            self._check(requested)
    def cmd_SWITCH(self, gcmd):
        target = next(iter(self._select(gcmd.get("TOOL"),
                                       {self._active_tool()})))
        if self._job_active() and not self.homing and not self._paused():
            if not self.armed and not self._begin():
                self.pending_tool = target
                self.tools.add(target)
                return
            self.tools.add(target)
            if not self._check({target}, preflight=True):
                self.pending_tool = target
                return
        self._run_macro("_工具切换", "_工具切換", " TOOL=" + TOOLS[target])
    def cmd_HOME(self, gcmd):
        old = self.homing
        self.homing = True
        try:
            axes = gcmd.get("AXES", "XYZ").upper()
            if not axes or any(axis not in "XYZ" for axis in axes):
                raise gcmd.error("未知的歸零軸：%s" % axes)
            self._run("_VULCAN_HOME " + " ".join(
                "%s=0" % axis for axis in axes))
        finally:
            self.homing = old
    def cmd_START(self, gcmd):
        if self._macro_status("_冷模式状态", "_冷模式狀態").get("enabled", False):
            raise gcmd.error("冷模式仍為啟用狀態，請先停用再開始列印")
        reset = self._macro("_重置取消列印保护", "_重置取消列印保護")
        park = self._macro("_暂停取消停靠", "_暫停取消停靠")
        self._run(reset)
        self._run("SET_GCODE_VARIABLE MACRO=%s "
                  "VARIABLE=has_parked VALUE=0" % park)
        self.pending_start = {
            "bed": gcmd.get_float("BED", 60., minval=0.),
            "extruder": gcmd.get_float("EXTRUDER", 200., minval=0.),
            "stage": 0, "prime_index": 0}
        if self._begin(gcmd.get("TOOL", gcmd.get("T", "AUTO"))):
            self._continue_start()
    def _continue_start(self):
        start = self.pending_start
        self.busy = True
        try:
            while start is self.pending_start and start["stage"] < 5:
                if self._paused() or not self._check(self.tools,
                                                    preflight=True):
                    return
                stage = start["stage"]
                if stage == 0:
                    self._run("G90\nM83\nG28")
                elif stage == 1:
                    self._run_macro("_工具切换", "_工具切換",
                                    " TOOL=" + TOOLS[self.first_tool])
                    if self.printer.lookup_object("bed_mesh", None):
                        self._run("BED_MESH_PROFILE LOAD=default")
                    self._run("M104 S%s\nM140 S%s" % (
                        start["extruder"], start["bed"]))
                elif stage == 2:
                    self._run("M190 S%s" % start["bed"])
                elif stage == 3:
                    self._run("M109 S%s" % start["extruder"])
                else:
                    # Each macro phase rechecks LIVE state before extrusion.
                    lines = ("G1 Z5 F600", "G1 X5 Y5 F9000",
                             "G1 Z0.3 F600", "G1 X120 E12 F1200",
                             "G1 Z2 F600")
                    while start["prime_index"] < len(lines):
                        if self._paused() or not self._check(
                                self.tools, preflight=True):
                            return
                        self._run(lines[start["prime_index"]])
                        start["prime_index"] += 1
                if self._paused():
                    return
                start["stage"] += 1
            if start is self.pending_start:
                self.pending_start = None
                self.no_park_pause = False
        finally:
            self.busy = False
    def _restore_wait(self):
        held = self.held_wait
        if held is None:
            return True
        sensor, target = held
        self.held_wait = None
        self._run("SET_HEATER_TEMPERATURE HEATER=%s TARGET=%s" % held)
        if target > 0.:
            if sensor == "heater_bed":
                self._run("M190 S%s" % target)
            elif sensor in TOOLS:
                self._run("M109 T%d S%s" % (TOOLS.index(sensor), target))
            else:
                raise self.gcode.error("不支援的溫度等待感測器：%s" % sensor)
        return (self.held_wait is None
                and not self._missing(self._monitor_tools()))
    def cmd_RESUME(self, gcmd):
        if not self._paused():
            gcmd.respond_info("目前沒有暫停中的列印")
            return
        needed = self._monitor_tools() if self.armed else {self._active_tool()}
        if not self._check(needed, preflight=self.no_park_pause):
            return
        pause = self.printer.lookup_object("pause_resume")
        self.resuming = True
        try:
            if not self._restore_wait() or not self._check(
                    needed, preflight=self.no_park_pause):
                return
            if self.pending_start is not None:
                # Start resumes at its saved phase, not at the old park state.
                self._run("CLEAR_PAUSE")
                self._continue_start()
                if self.pending_start is not None or self._paused():
                    return
                pause.send_resume_command()
            else:
                resume = self._status("gcode_macro RESUME")
                client = self._macro_status("_用户端设定", "_用戶端設定")
                last = resume.get("last_extruder_temp", {})
                active = TOOLS[self._active_tool()]
                if (not self.no_park_pause and last.get("restore")
                    and float(last.get("temp", 0)) > 0
                    and (resume.get("idle_state")
                         or self._status("idle_timeout").get("state") == "Idle"
                         or not self._status(active).get(
                             "can_extrude", False))):
                    self._run("M109 S%s" % last["temp"])
                if not self._check(needed,
                                   preflight=self.no_park_pause):
                    return
                if not self.no_park_pause:
                    if not self._status(active).get("can_extrude", False):
                        gcmd.respond_info("恢復失敗：請先將目前噴頭加熱至可擠出溫度")
                        return
                    timeout = resume.get("restore_idle_timeout", 0)
                    if timeout > 0:
                        self._run("SET_IDLE_TIMEOUT TIMEOUT=%s" % timeout)
                    user = client.get("user_resume_macro", "")
                    if user:
                        self._run(user)
                    if not self._check(needed):
                        return
                    self._run_macro("_用户端挤出", "_用戶端擠出")
                velocity = gcmd.get_float("VELOCITY",
                                          client.get("speed_move", 50.),
                                          above=0.)
                # Dispatch lock prevents the SD next line racing this switch.
                self._run("RESUME_BASE VELOCITY=%s" % velocity)
                if self.pending_tool is not None:
                    target = self.pending_tool
                    self._run_macro("_工具切换", "_工具切換", " TOOL=" + TOOLS[target])
                    self.pending_tool = None
            self.blocked = set()
            self.preflight = False
            self.no_park_pause = False
            park = self._macro("_暂停取消停靠", "_暫停取消停靠")
            self._run("SET_GCODE_VARIABLE MACRO=%s "
                      "VARIABLE=has_parked VALUE=0" % park)
            self._run("SET_GCODE_VARIABLE MACRO=RESUME "
                      "VARIABLE=idle_state VALUE=False")
            self._run(self._macro("状态_列印中", "狀態_列印中"))
        finally:
            self.resuming = False
    def cmd_RESET(self, gcmd):
        self.reset()
    def get_status(self, eventtime):
        return {"armed": self.armed, "required_tools": sorted(self.tools),
                "monitored_tools": (sorted(self._monitor_tools())
                                    if self.armed else []),
                "preflight_active": self.preflight,
                "first_tool": self.first_tool, "file_path": self.filename,
                "blocked_tools": sorted(self.blocked),
                "pending_tool": self.pending_tool,
                "start_stage": (None if self.pending_start is None
                                else self.pending_start["stage"]),
                "held_temperature_wait": self.held_wait,
                "homing": self.homing}

def load_config(config):
    return VulcanFilament(config)
