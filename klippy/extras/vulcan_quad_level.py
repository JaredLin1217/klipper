# VULCAN1200 four-motor gantry leveling with explicit machine motor mapping.
# This is intentionally separate from quad_gantry_level: the machine's
# physical order is Z2, Z3, Z1, Z rather than Klipper's native Z, Z1, Z2, Z3.
import logging

from . import probe


class _ZAdjustHelper:
    """Safely move independent Z steppers while preserving toolhead state."""
    def __init__(self, printer, motor_names, config_name,
                 adjust_speed, settle_time):
        self.printer = printer
        self.motor_names = motor_names
        self.config_name = config_name
        self.adjust_speed = adjust_speed
        self.settle_time = settle_time
        self.z_steppers = []
        printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        available = {s.get_name(): s for s in kin.get_steppers()
                     if s.is_active_axis("z")}
        missing = [name for name in self.motor_names if name not in available]
        if missing:
            raise self.printer.config_error(
                "%s missing Z steppers: %s" %
                (self.config_name, ", ".join(missing)))
        if len(set(self.motor_names)) != 4:
            raise self.printer.config_error(
                "%s requires four distinct Z steppers" % self.config_name)
        self.z_steppers = [available[name] for name in self.motor_names]

    def adjust(self, adjustments, speed):
        if len(self.z_steppers) != 4:
            raise self.printer.command_error(
                "%s is not connected to four Z steppers" % self.config_name)
        toolhead = self.printer.lookup_object("toolhead")
        gcode = self.printer.lookup_object("gcode")
        curpos = toolhead.get_position()
        gcode.respond_info(
            "Z motor adjustments: %s" % " ".join(
                "%s=%+.6f" % (s.get_name(), a)
                for s, a in zip(self.z_steppers, adjustments)))

        # Match Klipper's quad_gantry_level mechanism: detach all Z trapqs,
        # move each motor in sorted order, then restore the common trapq.
        toolhead.flush_step_generation()
        for stepper in self.z_steppers:
            stepper.set_trapq(None)
        positions = [(-adjustment, stepper)
                     for adjustment, stepper in zip(adjustments,
                                                   self.z_steppers)]
        positions.sort(key=lambda item: item[0])
        first_offset, _ = positions[0]
        z_low = curpos[2] - first_offset
        try:
            for step_index in range(len(positions) - 1):
                stepper_offset, stepper = positions[step_index]
                next_offset, _ = positions[step_index + 1]
                toolhead.flush_step_generation()
                stepper.set_trapq(toolhead.get_trapq())
                curpos[2] = z_low + next_offset
                toolhead.move(curpos, speed)
                toolhead.set_position(curpos)
                if self.settle_time:
                    toolhead.dwell(self.settle_time)
            last_offset, last_stepper = positions[-1]
            toolhead.flush_step_generation()
            last_stepper.set_trapq(toolhead.get_trapq())
            curpos[2] += first_offset
            toolhead.set_position(curpos)
            if self.settle_time:
                toolhead.dwell(self.settle_time)
        except Exception:
            logging.exception("VULCAN_QUAD_LEVEL Z adjustment")
            toolhead.flush_step_generation()
            for stepper in self.z_steppers:
                stepper.set_trapq(toolhead.get_trapq())
            raise


class VulcanQuadLevel:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.speed = config.getfloat("speed", 50., above=0.)
        self.horizontal_move_z = config.getfloat(
            "horizontal_move_z", 30., above=0.)
        self.max_adjust = config.getfloat("max_adjust", 2., above=0.)
        self.tolerance = config.getfloat(
            "retry_tolerance", .03, minval=0.)
        self.max_retries = config.getint("retries", 4, minval=0, maxval=30)
        self.adjust_speed = config.getfloat("adjust_speed", 3., above=0.)
        self.settle_time = config.getfloat("settle_time", .2, minval=0.)
        self.samples = config.getint("samples", 3, minval=1)
        self.samples_result = config.get("samples_result", "median")
        self.sample_retract_dist = config.getfloat(
            "sample_retract_dist", 2., above=0.)
        self.motor_names = [config.get("motor_z2"), config.get("motor_z3"),
                            config.get("motor_z1"), config.get("motor_z")]
        self.points = [config.getfloatlist("point_z2", count=2),
                       config.getfloatlist("point_z3", count=2),
                       config.getfloatlist("point_z1", count=2),
                       config.getfloatlist("point_z", count=2)]
        self.probe_helper = probe.ProbePointsHelper(
            config, self._probe_finalize, default_points=self.points)
        self.probe_helper.minimum_points(4)
        self.z_helper = _ZAdjustHelper(
            self.printer, self.motor_names, config.get_name(),
            self.adjust_speed, self.settle_time)
        self.retry_count = 0
        self.previous_spread = None
        self.increasing_spread = 0
        self.success_streak = 0
        self.gcode.register_command(
            "VULCAN_QUAD_LEVEL", self.cmd_VULCAN_QUAD_LEVEL,
            desc="Level the VULCAN1200 gantry with four mapped Z motors")

    def cmd_VULCAN_QUAD_LEVEL(self, gcmd):
        if self.printer.lookup_object("bed_mesh", None) is not None:
            # The user macro clears the mesh; this guard also makes the raw
            # command safe when called directly.
            self.printer.lookup_object("bed_mesh").set_mesh(None)
        self.retry_count = 0
        self.previous_spread = None
        self.increasing_spread = 0
        self.success_streak = 0
        # Supply this section's sampling defaults to ProbePointsHelper while
        # still allowing an operator to override them on the G-code command.
        params = dict(gcmd.get_command_parameters())
        params.setdefault("SAMPLES", str(self.samples))
        params.setdefault("SAMPLES_RESULT", self.samples_result)
        params.setdefault("SAMPLE_RETRACT_DIST", str(self.sample_retract_dist))
        params.setdefault("HORIZONTAL_MOVE_Z", str(self.horizontal_move_z))
        probe_gcmd = self.gcode.create_gcode_command(
            gcmd.get_command(), gcmd.get_commandline(), params)
        self.probe_helper.start_probe(probe_gcmd)

    def _probe_finalize(self, positions):
        if len(positions) != 4:
            raise self.gcode.error("VULCAN_QUAD_LEVEL requires four results")
        # Convert contact Z to gantry-relative height, matching Klipper's
        # quad_gantry_level convention.  This keeps the requested
        # adjustment = average - measured_value sign correct for a moving
        # gantry (and is equivalent to raw_bed_z - average_raw_bed_z).
        raw_values = [position.bed_z for position in positions]
        values = [self.horizontal_move_z - value for value in raw_values]
        average = sum(values) / 4.
        spread = max(values) - min(values)
        self.gcode.respond_info(
            "VULCAN 四轴探针结果：" + " ".join(
                "%s@(%.1f,%.1f) bed_z=%+.6f gantry_z=%+.6f" %
                (motor, point[0], point[1], raw_value, value)
                for motor, point, raw_value, value in
                zip(self.motor_names, self.points, raw_values, values)))
        self.gcode.respond_info(
            "平均值=%+.6f mm，当前误差范围=%.6f mm，容许值=%.6f mm" %
            (average, spread, self.tolerance))
        # A noisy probe can briefly cross the tolerance boundary.  Do not
        # keep driving the gantry when the measured range is worsening over
        # consecutive rounds; stop and preserve the evidence for inspection.
        prior_spread = self.previous_spread
        if prior_spread is not None:
            if spread > prior_spread + 1e-6:
                self.increasing_spread += 1
            elif self.increasing_spread:
                self.increasing_spread -= 1
        self.previous_spread = spread
        if self.increasing_spread > 1:
            raise self.gcode.error(
                "VULCAN_QUAD_LEVEL range increased in consecutive rounds "
                "(%.6f -> %.6f mm); adjustment stopped" %
                (prior_spread, spread))
        if spread <= self.tolerance:
            self.success_streak += 1
            if self.success_streak >= 2:
                self.gcode.respond_info("VULCAN 四轴平台调平完成")
                return "done"
            self.gcode.respond_info(
                "本轮已达容差，进行第 2 次连续确认")
            return "retry"
        self.success_streak = 0

        # The four probe points are directly below their mapped Z motors.
        # Use the measured error directly.  A learned scalar gain mixes
        # probe noise with cross-coupling and can amplify the next correction.
        adjustments = [average - value for value in values]
        required = max(abs(adjustment) for adjustment in adjustments)
        if self.retry_count >= self.max_retries:
            raise self.gcode.error(
                "VULCAN_QUAD_LEVEL exceeded %d retries; final range %.6f mm"
                % (self.max_retries, spread))
        # Keep each physical move within max_adjust.  A large initial tilt is
        # corrected in proportional stages instead of aborting before the
        # first useful adjustment.
        scale = min(1., self.max_adjust / required) if required else 1.
        applied = [adjustment * scale for adjustment in adjustments]
        if scale < 1.:
            self.gcode.respond_info(
                "首轮调整需求 %.6f mm 超过单轮上限 %.6f mm，"
                "按 %.3f 比例分段调整" %
                (required, self.max_adjust, scale))
        self.z_helper.adjust(applied, self.adjust_speed)
        self.retry_count += 1
        self.gcode.respond_info(
            "四轴已调整，开始第 %d 次复测" % (self.retry_count + 1))
        return "retry"


def load_config(config):
    return VulcanQuadLevel(config)
