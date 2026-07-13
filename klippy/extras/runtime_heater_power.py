# runtime_heater_power.py
#
# Add runtime G-code command:
#   SET_HEATER_MAX_POWER HEATER=<heater_name> MAX_POWER=<0.0~1.0>
#
# Example:
#   SET_HEATER_MAX_POWER HEATER=heater_bed MAX_POWER=0.50
#
# This is a custom Klipper extension.
# It modifies heater.max_power and the current control algorithm's
# heater_max_power at runtime.

import logging


class RuntimeHeaterPower:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "SET_HEATER_MAX_POWER",
            self.cmd_SET_HEATER_MAX_POWER,
            desc="Set heater max_power at runtime"
        )

    def cmd_SET_HEATER_MAX_POWER(self, gcmd):
        heater_name = gcmd.get("HEATER")
        max_power = gcmd.get_float("MAX_POWER", minval=0.0, maxval=1.0)

        heaters = self.printer.lookup_object("heaters")
        heater = heaters.lookup_heater(heater_name)

        old_power = getattr(heater, "max_power", None)

        # Update main heater object
        heater.max_power = max_power

        # Klipper heater.py uses min_pwm_change = max_power * 0.05
        # Keep the same behavior after runtime change.
        if hasattr(heater, "min_pwm_change"):
            heater.min_pwm_change = max_power * 0.05

        # Update current control algorithm object.
        # Both ControlBangBang and ControlPID store heater_max_power internally.
        control = getattr(heater, "control", None)
        if control is not None and hasattr(control, "heater_max_power"):
            control.heater_max_power = max_power

        # PID mode also stores temp_integ_max = heater_max_power / Ki.
        # If we do not update this, old integral limit may remain.
        if control is not None:
            ki = getattr(control, "Ki", 0.0)
            if hasattr(control, "temp_integ_max"):
                if ki:
                    control.temp_integ_max = max_power / ki
                else:
                    control.temp_integ_max = 0.0

            # Clamp previous integral if it exists.
            if hasattr(control, "prev_temp_integ") and hasattr(control, "temp_integ_max"):
                control.prev_temp_integ = max(
                    0.0,
                    min(control.prev_temp_integ, control.temp_integ_max)
                )

        # If current cached PWM is higher than new limit, force next control cycle lower.
        # This does not directly write PWM here; it makes the next heater update obey the limit.
        if hasattr(heater, "last_pwm_value") and heater.last_pwm_value > max_power:
            heater.last_pwm_value = max_power

        logging.info(
            "Runtime heater max_power changed: heater=%s old=%s new=%.3f",
            heater_name,
            old_power,
            max_power
        )

        gcmd.respond_info(
            "SET_HEATER_MAX_POWER: HEATER=%s old=%s new=%.3f"
            % (
                heater_name,
                "unknown" if old_power is None else "%.3f" % old_power,
                max_power
            )
        )


def load_config(config):
    return RuntimeHeaterPower(config)
