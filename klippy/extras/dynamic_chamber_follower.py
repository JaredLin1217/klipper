#=====================================================================================================================
# dynamic_chamber_follower.py
#
# Clean refactor version for Klipper
#
# Purpose:
#   Use one Klipper native PID heater as master chamber heater.
#   Use three PWM output_pin channels as follower heating outputs.
#   Follower outputs track the master PID demand and apply different power limits by Z stage.
#
# Hardware model:
#   chamber   = master PID heater, sensor PA0, heater output PC5
#   chamber1  = follower1 PWM output, PB1
#   chamber2  = follower2 PWM output, PB0
#   chamber3  = follower3 PWM output, PE8
#
# Config section:
#   [dynamic_chamber_follower]
#
# G-code:
#   SET_DYNAMIC_CHAMBER TARGET=<temp> [ENABLE=1]
#   STOP_DYNAMIC_CHAMBER
#   QUERY_DYNAMIC_CHAMBER
#   UPDATE_DYNAMIC_CHAMBER
#   SET_DYNAMIC_CHAMBER_STAGE STAGE=<1|2|3|4>
#   CLEAR_DYNAMIC_CHAMBER_STAGE
#
# Important:
#   This is process control, not a hardware safety device.
#   Each chamber heater zone still needs fuse / breaker / over-temperature protection.
#=====================================================================================================================

import logging


class DynamicChamberFollower:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")

        #-------------------------------------------------------------------------------------------------------------
        # Clean naming
        #-------------------------------------------------------------------------------------------------------------
        self.master_heater_name = config.get("master_heater", "chamber")

        self.follower1_pin = config.get("follower1_pin", "chamber1")
        self.follower2_pin = config.get("follower2_pin", "chamber2")
        self.follower3_pin = config.get("follower3_pin", "chamber3")

        self.update_interval = config.getfloat(
            "update_interval", 1.0, above=0.05
        )

        # If true, the module will send console messages when stage changes.
        self.report_stage_change = config.getboolean("report_stage_change", True)

        # Minimum PWM difference before writing follower pins again.
        self.min_output_change = config.getfloat(
            "min_output_change", 0.005, minval=0.0, maxval=1.0
        )

        #-------------------------------------------------------------------------------------------------------------
        # Z thresholds
        #-------------------------------------------------------------------------------------------------------------
        self.z_stage_1 = config.getfloat("z_stage_1", 220.0)
        self.z_stage_2 = config.getfloat("z_stage_2", 470.0)
        self.z_stage_3 = config.getfloat("z_stage_3", 720.0)

        if not (self.z_stage_1 < self.z_stage_2 < self.z_stage_3):
            raise config.error(
                "dynamic_chamber_follower: z_stage_1 < z_stage_2 < z_stage_3 is required"
            )

        #-------------------------------------------------------------------------------------------------------------
        # Stage power table
        #
        # Actual logic:
        #   master heater:
        #       module updates master heater max_power by current stage.
        #
        #   followers:
        #       follower output = normalized_master_pid_demand * stage_follower_power
        #
        # normalized_master_pid_demand:
        #   master actual pwm / current master max_power
        #-------------------------------------------------------------------------------------------------------------

        # Stage 1: Z < z_stage_1
        self.stage1_master_power = config.getfloat(
            "stage1_master_power", 0.80, minval=0.0, maxval=1.0
        )
        self.stage1_follower1_power = config.getfloat(
            "stage1_follower1_power", 0.00, minval=0.0, maxval=1.0
        )
        self.stage1_follower2_power = config.getfloat(
            "stage1_follower2_power", 0.00, minval=0.0, maxval=1.0
        )
        self.stage1_follower3_power = config.getfloat(
            "stage1_follower3_power", 0.00, minval=0.0, maxval=1.0
        )

        # Stage 2: z_stage_1 <= Z < z_stage_2
        self.stage2_master_power = config.getfloat(
            "stage2_master_power", 0.60, minval=0.0, maxval=1.0
        )
        self.stage2_follower1_power = config.getfloat(
            "stage2_follower1_power", 0.80, minval=0.0, maxval=1.0
        )
        self.stage2_follower2_power = config.getfloat(
            "stage2_follower2_power", 0.00, minval=0.0, maxval=1.0
        )
        self.stage2_follower3_power = config.getfloat(
            "stage2_follower3_power", 0.00, minval=0.0, maxval=1.0
        )

        # Stage 3: z_stage_2 <= Z < z_stage_3
        self.stage3_master_power = config.getfloat(
            "stage3_master_power", 0.40, minval=0.0, maxval=1.0
        )
        self.stage3_follower1_power = config.getfloat(
            "stage3_follower1_power", 0.60, minval=0.0, maxval=1.0
        )
        self.stage3_follower2_power = config.getfloat(
            "stage3_follower2_power", 0.80, minval=0.0, maxval=1.0
        )
        self.stage3_follower3_power = config.getfloat(
            "stage3_follower3_power", 0.00, minval=0.0, maxval=1.0
        )

        # Stage 4: Z >= z_stage_3
        self.stage4_master_power = config.getfloat(
            "stage4_master_power", 0.20, minval=0.0, maxval=1.0
        )
        self.stage4_follower1_power = config.getfloat(
            "stage4_follower1_power", 0.40, minval=0.0, maxval=1.0
        )
        self.stage4_follower2_power = config.getfloat(
            "stage4_follower2_power", 0.60, minval=0.0, maxval=1.0
        )
        self.stage4_follower3_power = config.getfloat(
            "stage4_follower3_power", 0.80, minval=0.0, maxval=1.0
        )

        #-------------------------------------------------------------------------------------------------------------
        # Runtime state
        #-------------------------------------------------------------------------------------------------------------
        self.enabled = False
        self.target = 0.0

        self.master_heater = None
        self.toolhead = None
        self.timer = None

        self.current_stage = None
        self.current_master_power = None

        # Manual stage override:
        #   None     = AUTO by Z
        #   "stage1" = forced stage 1
        #   "stage2" = forced stage 2
        #   "stage3" = forced stage 3
        #   "stage4" = forced stage 4
        self.manual_stage = None

        self.last_follower1_value = None
        self.last_follower2_value = None
        self.last_follower3_value = None

        self.last_master_pwm = 0.0
        self.last_normalized_demand = 0.0

        #-------------------------------------------------------------------------------------------------------------
        # Register G-code commands
        #-------------------------------------------------------------------------------------------------------------
        self.gcode.register_command(
            "SET_DYNAMIC_CHAMBER",
            self.cmd_SET_DYNAMIC_CHAMBER,
            desc="Enable dynamic chamber follower and set chamber target"
        )

        self.gcode.register_command(
            "STOP_DYNAMIC_CHAMBER",
            self.cmd_STOP_DYNAMIC_CHAMBER,
            desc="Stop dynamic chamber follower and turn off chamber outputs"
        )

        self.gcode.register_command(
            "QUERY_DYNAMIC_CHAMBER",
            self.cmd_QUERY_DYNAMIC_CHAMBER,
            desc="Query dynamic chamber follower status"
        )

        self.gcode.register_command(
            "UPDATE_DYNAMIC_CHAMBER",
            self.cmd_UPDATE_DYNAMIC_CHAMBER,
            desc="Force one dynamic chamber follower update"
        )

        self.gcode.register_command(
            "SET_DYNAMIC_CHAMBER_STAGE",
            self.cmd_SET_DYNAMIC_CHAMBER_STAGE,
            desc="Manually force dynamic chamber stage"
        )

        self.gcode.register_command(
            "CLEAR_DYNAMIC_CHAMBER_STAGE",
            self.cmd_CLEAR_DYNAMIC_CHAMBER_STAGE,
            desc="Clear manual dynamic chamber stage override"
        )

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)

    #-----------------------------------------------------------------------------------------------------------------
    # Klipper lifecycle
    #-----------------------------------------------------------------------------------------------------------------

    def _handle_ready(self):
        heaters = self.printer.lookup_object("heaters")
        self.master_heater = heaters.lookup_heater(self.master_heater_name)
        self.toolhead = self.printer.lookup_object("toolhead")

        self.timer = self.reactor.register_timer(
            self._timer_event,
            self.reactor.NEVER
        )

        logging.info(
            "DynamicChamberFollower ready: master_heater=%s follower1=%s follower2=%s follower3=%s",
            self.master_heater_name,
            self.follower1_pin,
            self.follower2_pin,
            self.follower3_pin
        )

    def _handle_shutdown(self):
        try:
            self.enabled = False
            self._write_followers(0.0, 0.0, 0.0, force=True)
        except Exception:
            logging.exception("DynamicChamberFollower shutdown handler failed")

    #-----------------------------------------------------------------------------------------------------------------
    # G-code commands
    #-----------------------------------------------------------------------------------------------------------------

    def cmd_SET_DYNAMIC_CHAMBER(self, gcmd):
        target = gcmd.get_float("TARGET", None, minval=0.0, maxval=220.0)
        enable = gcmd.get_int("ENABLE", 1, minval=0, maxval=1)

        if target is not None:
            self.target = target

        if enable:
            self.enabled = True
            self._set_master_target(self.target)
            self.reactor.update_timer(self.timer, self.reactor.NOW)

            gcmd.respond_info(
                "Dynamic chamber enabled: target=%.2f manual_stage=%s"
                % (
                    self.target,
                    self.manual_stage if self.manual_stage is not None else "AUTO"
                )
            )
        else:
            self.enabled = False
            self._stop_all_outputs()
            self.reactor.update_timer(self.timer, self.reactor.NEVER)

            gcmd.respond_info("Dynamic chamber disabled")

    def cmd_STOP_DYNAMIC_CHAMBER(self, gcmd):
        self.enabled = False
        self.target = 0.0
        self.manual_stage = None
        self._stop_all_outputs()
        self.reactor.update_timer(self.timer, self.reactor.NEVER)

        gcmd.respond_info(
            "Dynamic chamber stopped: master target=0, followers=0, manual_stage cleared"
        )

    def cmd_QUERY_DYNAMIC_CHAMBER(self, gcmd):
        z = self._get_z_position()
        stage = self._get_stage(z)
        powers = self._get_stage_powers(stage)

        master_pwm = self._get_master_pwm()
        normalized_demand = self._normalize_master_pwm(
            master_pwm,
            powers["master"]
        )

        follower1_value = self._clamp(normalized_demand * powers["follower1"])
        follower2_value = self._clamp(normalized_demand * powers["follower2"])
        follower3_value = self._clamp(normalized_demand * powers["follower3"])

        gcmd.respond_info(
            "Dynamic chamber: enabled=%s target=%.2f z=%.3f stage=%s mode=%s "
            "master_power=%.3f master_pwm=%.3f normalized=%.3f "
            "follower1_power=%.3f follower1_value=%.3f "
            "follower2_power=%.3f follower2_value=%.3f "
            "follower3_power=%.3f follower3_value=%.3f"
            % (
                self.enabled,
                self.target,
                z,
                stage,
                self.manual_stage if self.manual_stage is not None else "AUTO",
                powers["master"],
                master_pwm,
                normalized_demand,
                powers["follower1"],
                follower1_value,
                powers["follower2"],
                follower2_value,
                powers["follower3"],
                follower3_value,
            )
        )

    def cmd_UPDATE_DYNAMIC_CHAMBER(self, gcmd):
        self._update_outputs(force=True)
        gcmd.respond_info("Dynamic chamber manual update completed")

    def cmd_SET_DYNAMIC_CHAMBER_STAGE(self, gcmd):
        stage_num = gcmd.get_int("STAGE", minval=1, maxval=4)
        self.manual_stage = "stage%d" % stage_num

        self._update_outputs(force=True)

        gcmd.respond_info(
            "Dynamic chamber manual stage enabled: %s" % self.manual_stage
        )

    def cmd_CLEAR_DYNAMIC_CHAMBER_STAGE(self, gcmd):
        self.manual_stage = None

        self._update_outputs(force=True)

        gcmd.respond_info(
            "Dynamic chamber manual stage cleared: AUTO Z mode enabled"
        )

    #-----------------------------------------------------------------------------------------------------------------
    # Timer
    #-----------------------------------------------------------------------------------------------------------------

    def _timer_event(self, eventtime):
        if not self.enabled:
            return self.reactor.NEVER

        try:
            self._update_outputs(force=False)
        except Exception:
            logging.exception("DynamicChamberFollower update failed")

        return eventtime + self.update_interval

    #-----------------------------------------------------------------------------------------------------------------
    # Main control
    #-----------------------------------------------------------------------------------------------------------------

    def _update_outputs(self, force=False):
        z = self._get_z_position()
        stage = self._get_stage(z)
        powers = self._get_stage_powers(stage)

        # Change master heater max_power when stage changes or forced update.
        if force or self.current_master_power != powers["master"]:
            self._set_master_max_power(powers["master"])
            self.current_master_power = powers["master"]

        # Keep target active while enabled.
        if self.enabled and self.target > 0.0:
            self._set_master_target(self.target)

        # Read master PID output.
        master_pwm = self._get_master_pwm()
        normalized_demand = self._normalize_master_pwm(
            master_pwm,
            powers["master"]
        )

        follower1_value = self._clamp(normalized_demand * powers["follower1"])
        follower2_value = self._clamp(normalized_demand * powers["follower2"])
        follower3_value = self._clamp(normalized_demand * powers["follower3"])

        self._write_followers(
            follower1_value,
            follower2_value,
            follower3_value,
            force=force
        )

        if stage != self.current_stage:
            logging.info(
                "Dynamic chamber stage changed: z=%.3f stage=%s mode=%s "
                "master_power=%.3f follower1_power=%.3f follower2_power=%.3f follower3_power=%.3f",
                z,
                stage,
                self.manual_stage if self.manual_stage is not None else "AUTO",
                powers["master"],
                powers["follower1"],
                powers["follower2"],
                powers["follower3"]
            )

            if self.report_stage_change:
                self.gcode.respond_info(
                    "Dynamic chamber stage changed: Z=%.2f stage=%s mode=%s "
                    "master=%.2f follower1=%.2f follower2=%.2f follower3=%.2f"
                    % (
                        z,
                        stage,
                        self.manual_stage if self.manual_stage is not None else "AUTO",
                        powers["master"],
                        powers["follower1"],
                        powers["follower2"],
                        powers["follower3"]
                    )
                )

        self.current_stage = stage
        self.last_master_pwm = master_pwm
        self.last_normalized_demand = normalized_demand

    #-----------------------------------------------------------------------------------------------------------------
    # Stage table
    #-----------------------------------------------------------------------------------------------------------------

    def _get_stage(self, z):
        if self.manual_stage is not None:
            return self.manual_stage

        if z < self.z_stage_1:
            return "stage1"
        if z < self.z_stage_2:
            return "stage2"
        if z < self.z_stage_3:
            return "stage3"
        return "stage4"

    def _get_stage_powers(self, stage):
        if stage == "stage1":
            return {
                "master": self.stage1_master_power,
                "follower1": self.stage1_follower1_power,
                "follower2": self.stage1_follower2_power,
                "follower3": self.stage1_follower3_power,
            }

        if stage == "stage2":
            return {
                "master": self.stage2_master_power,
                "follower1": self.stage2_follower1_power,
                "follower2": self.stage2_follower2_power,
                "follower3": self.stage2_follower3_power,
            }

        if stage == "stage3":
            return {
                "master": self.stage3_master_power,
                "follower1": self.stage3_follower1_power,
                "follower2": self.stage3_follower2_power,
                "follower3": self.stage3_follower3_power,
            }

        if stage == "stage4":
            return {
                "master": self.stage4_master_power,
                "follower1": self.stage4_follower1_power,
                "follower2": self.stage4_follower2_power,
                "follower3": self.stage4_follower3_power,
            }

        # Defensive fallback. This should never happen.
        return {
            "master": 0.0,
            "follower1": 0.0,
            "follower2": 0.0,
            "follower3": 0.0,
        }

    #-----------------------------------------------------------------------------------------------------------------
    # Master heater helpers
    #-----------------------------------------------------------------------------------------------------------------

    def _set_master_target(self, target):
        self.gcode.run_script_from_command(
            "SET_HEATER_TEMPERATURE HEATER=%s TARGET=%.3f"
            % (self.master_heater_name, target)
        )

    def _get_master_pwm(self):
        if self.master_heater is None:
            return 0.0

        # In common Klipper heater implementations, last_pwm_value stores the last heater PWM value.
        # This is an internal Klipper attribute, so this module should be tested after Klipper updates.
        pwm = getattr(self.master_heater, "last_pwm_value", 0.0)

        try:
            return self._clamp(float(pwm))
        except Exception:
            return 0.0

    def _normalize_master_pwm(self, master_pwm, master_power):
        if master_power <= 0.0:
            return 0.0
        return self._clamp(master_pwm / master_power)

    def _set_master_max_power(self, max_power):
        if self.master_heater is None:
            return

        max_power = self._clamp(max_power)
        old_power = getattr(self.master_heater, "max_power", None)

        self.master_heater.max_power = max_power

        if hasattr(self.master_heater, "min_pwm_change"):
            self.master_heater.min_pwm_change = max_power * 0.05

        control = getattr(self.master_heater, "control", None)

        if control is not None and hasattr(control, "heater_max_power"):
            control.heater_max_power = max_power

        if control is not None:
            ki = getattr(control, "Ki", 0.0)

            if hasattr(control, "temp_integ_max"):
                if ki:
                    control.temp_integ_max = max_power / ki
                else:
                    control.temp_integ_max = 0.0

            if hasattr(control, "prev_temp_integ") and hasattr(control, "temp_integ_max"):
                control.prev_temp_integ = max(
                    0.0,
                    min(control.prev_temp_integ, control.temp_integ_max)
                )

        if hasattr(self.master_heater, "last_pwm_value"):
            if self.master_heater.last_pwm_value > max_power:
                self.master_heater.last_pwm_value = max_power

        logging.info(
            "Dynamic chamber master max_power changed: heater=%s old=%s new=%.3f",
            self.master_heater_name,
            old_power,
            max_power
        )

    #-----------------------------------------------------------------------------------------------------------------
    # Follower helpers
    #-----------------------------------------------------------------------------------------------------------------

    def _write_followers(self, follower1_value, follower2_value, follower3_value, force=False):
        follower1_value = self._clamp(follower1_value)
        follower2_value = self._clamp(follower2_value)
        follower3_value = self._clamp(follower3_value)

        if force or self._need_write(follower1_value, self.last_follower1_value):
            self._set_pin(self.follower1_pin, follower1_value)
            self.last_follower1_value = follower1_value

        if force or self._need_write(follower2_value, self.last_follower2_value):
            self._set_pin(self.follower2_pin, follower2_value)
            self.last_follower2_value = follower2_value

        if force or self._need_write(follower3_value, self.last_follower3_value):
            self._set_pin(self.follower3_pin, follower3_value)
            self.last_follower3_value = follower3_value

    def _need_write(self, new_value, old_value):
        if old_value is None:
            return True
        return abs(new_value - old_value) >= self.min_output_change

    def _set_pin(self, pin_name, value):
        self.gcode.run_script_from_command(
            "SET_PIN PIN=%s VALUE=%.5f" % (pin_name, self._clamp(value))
        )

    #-----------------------------------------------------------------------------------------------------------------
    # Stop / status helpers
    #-----------------------------------------------------------------------------------------------------------------

    def _stop_all_outputs(self):
        try:
            self._set_master_target(0.0)
        except Exception:
            logging.exception("DynamicChamberFollower: unable to stop master target")

        try:
            self._set_master_max_power(0.0)
        except Exception:
            logging.exception("DynamicChamberFollower: unable to set master max_power to zero")

        try:
            self._write_followers(0.0, 0.0, 0.0, force=True)
        except Exception:
            logging.exception("DynamicChamberFollower: unable to stop follower outputs")

        self.current_stage = None
        self.current_master_power = None
        self.last_master_pwm = 0.0
        self.last_normalized_demand = 0.0

    def _get_z_position(self):
        if self.toolhead is None:
            return 0.0

        try:
            return float(self.toolhead.get_position()[2])
        except Exception:
            logging.exception("DynamicChamberFollower: unable to read Z position")
            return 0.0

    def _clamp(self, value):
        try:
            value = float(value)
        except Exception:
            return 0.0

        if value < 0.0:
            return 0.0
        if value > 1.0:
            return 1.0
        return value


def load_config(config):
    return DynamicChamberFollower(config)
