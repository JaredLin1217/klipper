# Z Thermal Adjust
#
# Copyright (C) 2022  Robert Pazdzior <robertp@norbital.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Adjusts Z position in real-time using one or more temperature sources to
# compensate for thermal expansion of the printer.

import math
import threading

KELVIN_TO_CELSIUS = -273.15
REPORT_TIME = 0.300


class ZThermalSource:
    def __init__(self, config, default_smooth_time, adjuster=None):
        self.printer = config.get_printer()
        self.adjuster = adjuster
        self.name = config.get_name().split(None, 1)[1]
        self.sensor_name = config.get('sensor')
        self.temp_coeff = config.getfloat(
            'temp_coeff', default=0., minval=-1., maxval=1.)
        self.smooth_time = config.getfloat(
            'smooth_time', default_smooth_time, above=0.)
        self.config_ref_temperature = config.getfloat(
            'reference_temperature', None, minval=KELVIN_TO_CELSIUS)
        self.ref_temperature = self.config_ref_temperature
        self.ref_temp_override = False

        self.sensor = None
        self.last_temp = None
        self.smoothed_temp = None
        self.last_temp_time = None
        self.measured_min = None
        self.measured_max = None
        self.available = False

    def handle_connect(self):
        self.sensor = self.printer.lookup_object(self.sensor_name)
        if self.sensor is self.adjuster:
            raise self.printer.config_error(
                "z_thermal_adjust source '%s' cannot reference "
                "z_thermal_adjust itself" % (self.name,))
        if not hasattr(self.sensor, 'get_status'):
            raise self.printer.config_error(
                "z_thermal_adjust source '%s': sensor '%s' does not "
                "have a status" % (self.name, self.sensor_name))
        status = self.sensor.get_status(
            self.printer.get_reactor().monotonic())
        if 'temperature' not in status:
            raise self.printer.config_error(
                "z_thermal_adjust source '%s': sensor '%s' does not "
                "report a temperature" % (self.name, self.sensor_name))

    def update_temperature(self, eventtime):
        status = self.sensor.get_status(eventtime)
        temperature = status.get('temperature')
        try:
            temperature = float(temperature)
        except (TypeError, ValueError):
            self.available = False
            return False
        if not math.isfinite(temperature):
            self.available = False
            return False

        self.available = True
        self.last_temp = temperature
        if self.smoothed_temp is None:
            self.smoothed_temp = temperature
        else:
            time_diff = max(eventtime - self.last_temp_time, 0.)
            adj_time = min(time_diff / self.smooth_time, 1.)
            temp_diff = temperature - self.smoothed_temp
            self.smoothed_temp += temp_diff * adj_time
        self.last_temp_time = eventtime

        if self.measured_min is None:
            self.measured_min = self.measured_max = self.smoothed_temp
        else:
            self.measured_min = min(self.measured_min, self.smoothed_temp)
            self.measured_max = max(self.measured_max, self.smoothed_temp)
        return True

    def handle_z_homing(self):
        if self.config_ref_temperature is None:
            if self.smoothed_temp is not None:
                self.ref_temperature = self.smoothed_temp
        else:
            self.ref_temperature = self.config_ref_temperature
        self.ref_temp_override = False

    def set_temp_coeff(self, temp_coeff):
        self.temp_coeff = temp_coeff

    def set_ref_temperature(self, ref_temperature):
        self.ref_temperature = ref_temperature
        self.ref_temp_override = True

    def get_contribution(self):
        if (not self.available or self.smoothed_temp is None
                or self.ref_temperature is None):
            return None
        delta_t = self.smoothed_temp - self.ref_temperature
        return -1. * self.temp_coeff * delta_t

    def get_status(self, eventtime=None):
        contribution = self.get_contribution()
        delta_t = None
        if self.smoothed_temp is not None and self.ref_temperature is not None:
            delta_t = self.smoothed_temp - self.ref_temperature
        return {
            'sensor': self.sensor_name,
            'available': self.available,
            'temperature': self.smoothed_temp,
            'raw_temperature': self.last_temp,
            'measured_min_temp': self.measured_min,
            'measured_max_temp': self.measured_max,
            'temp_coeff': self.temp_coeff,
            'reference_temperature': self.ref_temperature,
            'reference_is_manual': self.ref_temp_override,
            'delta_temperature': delta_t,
            'contribution': contribution,
        }


class ZThermalAdjuster:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.lock = threading.Lock()

        # Common configuration
        self.smooth_time = config.getfloat('smooth_time', 2., above=0.)
        self.off_above_z = config.getfloat(
            'z_adjust_off_above', 99999999.)
        self.max_z_adjust_mm = config.getfloat(
            'max_z_adjustment', 99999999., minval=0.)

        # A sensor_type in the main section selects the original
        # single-sensor configuration. Otherwise named source sections are
        # used, for example [z_thermal_adjust bed].
        self.legacy_mode = config.get('sensor_type', None) is not None
        self.sources = {}
        self.temperature_update_timer = None

        # Legacy single-sensor state
        self.temp_coeff = None
        self.sensor = None
        self.last_temp = 0.
        self.measured_min = self.measured_max = 0.
        self.smoothed_temp = 0.
        self.last_temp_time = 0.
        self.ref_temperature = 0.
        self.ref_temp_override = False

        # Model and Z transformation state
        self.model_ready = self.legacy_mode
        self.unclamped_z_adjust_mm = 0.
        self.target_z_adjust_mm = 0.
        self.limit_active = False
        self.z_adjust_mm = 0.
        self.last_z_adjust_mm = 0.
        self.adjust_enable = True
        self.last_position = [0., 0., 0., 0.]
        self.next_transform = None

        if self.legacy_mode:
            self._setup_legacy_sensor(config)
        else:
            self.temperature_update_timer = self.reactor.register_timer(
                self._temperature_update_event)
            self.printer.register_event_handler(
                'klippy:ready', self.handle_ready)

        # Register printer events
        self.printer.register_event_handler(
            'klippy:connect', self.handle_connect)
        self.printer.register_event_handler(
            'homing:home_rails_end', self.handle_homing_move_end)

        # Register gcode commands
        self.gcode.register_command(
            'SET_Z_THERMAL_ADJUST',
            self.cmd_SET_Z_THERMAL_ADJUST,
            desc=self.cmd_SET_Z_THERMAL_ADJUST_help)

    def _setup_legacy_sensor(self, config):
        self.temp_coeff = config.getfloat(
            'temp_coeff', minval=-1., maxval=1., default=0.)
        self.inv_smooth_time = 1. / self.smooth_time
        self.min_temp = config.getfloat(
            'min_temp', minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        pheaters = self.printer.load_object(config, 'heaters')
        self.sensor = pheaters.setup_sensor(config)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        pheaters.register_sensor(config, self)

    def add_source(self, config):
        if self.legacy_mode:
            raise config.error(
                "Cannot combine sensor_type in [z_thermal_adjust] with "
                "a [%s] source section" % (config.get_name(),))
        source = ZThermalSource(config, self.smooth_time, self)
        if source.name in self.sources:
            raise config.error(
                "Duplicate z_thermal_adjust source '%s'" % (source.name,))
        self.sources[source.name] = source
        return source

    def handle_connect(self):
        'Called after all printer objects are instantiated'
        self.toolhead = self.printer.lookup_object('toolhead')
        gcode_move = self.printer.lookup_object('gcode_move')

        # Register move transformation
        self.next_transform = gcode_move.set_move_transform(self, force=True)

        # Pull Z step distance for minimum adjustment increment
        kin = self.toolhead.get_kinematics()
        steppers = [s.get_name() for s in kin.get_steppers()]
        z_stepper = kin.get_steppers()[steppers.index('stepper_z')]
        self.z_step_dist = z_stepper.get_step_dist()

        if not self.legacy_mode:
            if not self.sources:
                raise self.printer.config_error(
                    "[z_thermal_adjust] requires sensor_type or at least one "
                    "[z_thermal_adjust <name>] source")
            for source in self.sources.values():
                source.handle_connect()

    def handle_ready(self):
        # Delay the first read because some status-based temperature sensors
        # are not initialized immediately when Klipper becomes ready.
        self.reactor.update_timer(
            self.temperature_update_timer,
            self.reactor.monotonic() + 1.)

    def _limit_adjust(self, adjust):
        limited_adjust = max(
            -self.max_z_adjust_mm,
            min(self.max_z_adjust_mm, adjust))
        self.limit_active = limited_adjust != adjust
        return limited_adjust

    def _update_legacy_model_locked(self):
        delta_t = self.smoothed_temp - self.ref_temperature
        self.unclamped_z_adjust_mm = -1. * self.temp_coeff * delta_t
        self.target_z_adjust_mm = self._limit_adjust(
            self.unclamped_z_adjust_mm)

    def _update_multi_model_locked(self):
        contributions = [
            source.get_contribution() for source in self.sources.values()]
        if not contributions or any(c is None for c in contributions):
            self.model_ready = False
            return
        self.model_ready = True
        self.unclamped_z_adjust_mm = sum(contributions)
        self.target_z_adjust_mm = self._limit_adjust(
            self.unclamped_z_adjust_mm)

    def _update_multi_temperatures(self, eventtime):
        with self.lock:
            for source in self.sources.values():
                source.update_temperature(eventtime)
            self._update_multi_model_locked()

    def _temperature_update_event(self, eventtime):
        self._update_multi_temperatures(eventtime)
        return eventtime + REPORT_TIME

    def get_status(self, eventtime):
        if self.legacy_mode:
            with self.lock:
                return {
                    'mode': 'single_sensor',
                    'enabled': self.adjust_enable,
                    'temperature': self.smoothed_temp,
                    'measured_min_temp': round(self.measured_min, 2),
                    'measured_max_temp': round(self.measured_max, 2),
                    'current_z_adjust': self.z_adjust_mm,
                    'target_z_adjust': self.target_z_adjust_mm,
                    'unclamped_z_adjust': self.unclamped_z_adjust_mm,
                    'limit_active': self.limit_active,
                    'z_adjust_ref_temperature': self.ref_temperature,
                }
        with self.lock:
            source_status = {
                name: source.get_status()
                for name, source in self.sources.items()
            }
            return {
                'mode': 'multi_source',
                'enabled': self.adjust_enable,
                'model_ready': self.model_ready,
                'temperature': None,
                'measured_min_temp': None,
                'measured_max_temp': None,
                'current_z_adjust': self.z_adjust_mm,
                'target_z_adjust': self.target_z_adjust_mm,
                'unclamped_z_adjust': self.unclamped_z_adjust_mm,
                'limit_active': self.limit_active,
                'z_adjust_ref_temperature': None,
                'sources': source_status,
            }

    def handle_homing_move_end(self, homing_state, rails):
        'Set reference temperature after Z homing.'
        if 2 not in homing_state.get_axes():
            return
        if self.legacy_mode:
            with self.lock:
                self.ref_temperature = self.smoothed_temp
                self.ref_temp_override = False
                self._update_legacy_model_locked()
        else:
            eventtime = self.reactor.monotonic()
            self._update_multi_temperatures(eventtime)
            with self.lock:
                for source in self.sources.values():
                    source.handle_z_homing()
                self._update_multi_model_locked()
        self.z_adjust_mm = 0.

    def calc_adjust(self, pos):
        'Z adjustment calculation'
        if pos[2] < self.off_above_z:
            if self.legacy_mode:
                with self.lock:
                    self._update_legacy_model_locked()
            else:
                with self.lock:
                    self._update_multi_model_locked()

            # Don't apply adjustments smaller than step distance
            if (self.model_ready
                    and abs(self.target_z_adjust_mm - self.z_adjust_mm)
                    > self.z_step_dist):
                self.z_adjust_mm = self.target_z_adjust_mm

        # Apply Z adjustment
        new_z = pos[2] + self.z_adjust_mm
        self.last_z_adjust_mm = self.z_adjust_mm
        return [pos[0], pos[1], new_z] + pos[3:]

    def calc_unadjust(self, pos):
        'Remove Z adjustment'
        unadjusted_z = pos[2] - self.z_adjust_mm
        return [pos[0], pos[1], unadjusted_z] + pos[3:]

    def get_position(self):
        position = self.calc_unadjust(self.next_transform.get_position())
        self.last_position = self.calc_adjust(position)
        return position

    def move(self, newpos, speed):
        # don't apply to extrude only moves or when disabled
        if (newpos[0:2] == self.last_position[0:2]
                or not self.adjust_enable):
            z = newpos[2] + self.last_z_adjust_mm
            adjusted_pos = [newpos[0], newpos[1], z, newpos[3]]
            self.next_transform.move(adjusted_pos, speed)
        else:
            adjusted_pos = self.calc_adjust(newpos)
            self.next_transform.move(adjusted_pos, speed)
        self.last_position[:] = newpos

    def temperature_callback(self, read_time, temp):
        'Called every time the legacy Z adjust thermistor is read'
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_temp += temp_diff * adj_time
            self.measured_min = min(self.measured_min, self.smoothed_temp)
            self.measured_max = max(self.measured_max, self.smoothed_temp)
            self._update_legacy_model_locked()

    def get_temp(self, eventtime):
        return self.smoothed_temp, 0.

    def stats(self, eventtime):
        if self.legacy_mode:
            return False, 'z_thermal_adjust: temp=%.1f' % (
                self.smoothed_temp,)
        with self.lock:
            source_stats = []
            for name, source in self.sources.items():
                temperature = source.smoothed_temp
                if temperature is None:
                    source_stats.append('%s=unavailable' % (name,))
                else:
                    source_stats.append('%s=%.1f' % (name, temperature))
            source_stats.append('adjust=%.4f' % (self.z_adjust_mm,))
        return False, 'z_thermal_adjust: ' + ' '.join(source_stats)

    def _set_enable(self, enable):
        if enable is None or enable == self.adjust_enable:
            return
        self.adjust_enable = bool(enable)
        gcode_move = self.printer.lookup_object('gcode_move')
        gcode_move.reset_last_position()

    def _cmd_set_legacy(self, gcmd, enable, coeff, ref_temp, source_name):
        if source_name is not None:
            raise gcmd.error(
                "SOURCE is only valid with named z_thermal_adjust sources")
        if ref_temp is not None:
            with self.lock:
                self.ref_temperature = ref_temp
                self.ref_temp_override = True
        if coeff is not None:
            with self.lock:
                self.temp_coeff = coeff
        with self.lock:
            self._update_legacy_model_locked()
        self._set_enable(enable)

        state = '1 (enabled)' if self.adjust_enable else '0 (disabled)'
        override = ' (manual)' if self.ref_temp_override else ''
        msg = (
            "mode: single_sensor\n"
            "enable: %s\n"
            "temp_coeff: %f mm/degC\n"
            "ref_temp: %.2f degC%s\n"
            "-------------------\n"
            "Current Z temp: %.2f degC\n"
            "Target Z adjustment: %.4f mm\n"
            "Applied Z adjustment: %.4f mm"
            % (state, self.temp_coeff, self.ref_temperature, override,
               self.smoothed_temp, self.target_z_adjust_mm,
               self.z_adjust_mm))
        gcmd.respond_info(msg)

    def _cmd_set_multi(self, gcmd, enable, coeff, ref_temp, source_name):
        if (coeff is not None or ref_temp is not None) and source_name is None:
            raise gcmd.error(
                "SOURCE is required when setting TEMP_COEFF or REF_TEMP in "
                "multi-source mode")
        with self.lock:
            if source_name is not None:
                source = self.sources.get(source_name)
                if source is None:
                    raise gcmd.error(
                        "Unknown z_thermal_adjust source '%s'"
                        % (source_name,))
                if coeff is not None:
                    source.set_temp_coeff(coeff)
                if ref_temp is not None:
                    source.set_ref_temperature(ref_temp)
            self._update_multi_model_locked()
        self._set_enable(enable)

        state = '1 (enabled)' if self.adjust_enable else '0 (disabled)'
        lines = [
            'mode: multi_source',
            'enable: %s' % (state,),
        ]
        with self.lock:
            for name, source in self.sources.items():
                if source_name is not None and name != source_name:
                    continue
                status = source.get_status()
                temperature = status['temperature']
                ref_temperature = status['reference_temperature']
                delta_t = status['delta_temperature']
                contribution = status['contribution']
                lines.extend([
                    '-------------------',
                    'source: %s' % (name,),
                    'sensor: %s' % (status['sensor'],),
                    'temp_coeff: %.6f mm/degC'
                    % (status['temp_coeff'],),
                    'temperature: %s degC'
                    % ('unavailable' if temperature is None
                       else '%.2f' % (temperature,)),
                    'ref_temp: %s degC%s'
                    % ('unset' if ref_temperature is None
                       else '%.2f' % (ref_temperature,),
                       ' (manual)' if status['reference_is_manual'] else ''),
                    'delta_temp: %s degC'
                    % ('unavailable' if delta_t is None
                       else '%.2f' % (delta_t,)),
                    'contribution: %s mm'
                    % ('unavailable' if contribution is None
                       else '%.4f' % (contribution,)),
                ])
            lines.extend([
                '===================',
                'Target Z adjustment: %.4f mm'
                % (self.target_z_adjust_mm,),
                'Applied Z adjustment: %.4f mm' % (self.z_adjust_mm,),
                'Limit active: %s' % (self.limit_active,),
            ])
        gcmd.respond_info('\n'.join(lines))

    def cmd_SET_Z_THERMAL_ADJUST(self, gcmd):
        enable = gcmd.get_int('ENABLE', None, minval=0, maxval=1)
        coeff = gcmd.get_float(
            'TEMP_COEFF', None, minval=-1., maxval=1.)
        ref_temp = gcmd.get_float(
            'REF_TEMP', None, minval=KELVIN_TO_CELSIUS)
        source_name = gcmd.get('SOURCE', None)

        if self.legacy_mode:
            self._cmd_set_legacy(
                gcmd, enable, coeff, ref_temp, source_name)
        else:
            self._cmd_set_multi(
                gcmd, enable, coeff, ref_temp, source_name)

    cmd_SET_Z_THERMAL_ADJUST_help = 'Set/query Z Thermal Adjust parameters.'


def load_config(config):
    return ZThermalAdjuster(config)


def load_config_prefix(config):
    printer = config.get_printer()
    adjuster = printer.load_object(config, 'z_thermal_adjust')
    return adjuster.add_source(config)
