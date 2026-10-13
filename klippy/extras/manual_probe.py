# Helper script for manual z height probing
#
# Copyright (C) 2019-2026  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, bisect, collections, math

# Main probe results tuple.  The probe estimates that if the toolhead
# is commanded to xy position [bed_x, bed_y] and then descends, the
# nozzle will contact the bed at a toolhead z position of bed_z.  The
# probe test itself was performed while the toolhead was at xyz
# position [test_x, test_y, test_z].  All coordinates are relative to
# the frame (the coordinate system used in the config file).
ProbeResult = collections.namedtuple('probe_result', [
    'bed_x', 'bed_y', 'bed_z', 'test_x', 'test_y', 'test_z'])

# Helper to create a ProbeResult from a test position and probe offsets
def create_probe_result(test_pos, offsets=(0., 0., 0.)):
    x_offset, y_offset, z_offset = offsets
    return ProbeResult(
        test_pos[0]+x_offset, test_pos[1]+y_offset, test_pos[2]-z_offset,
        test_pos[0], test_pos[1], test_pos[2])

# Helper to lookup the Z stepper config section
def lookup_z_endstop_config(config):
    if config.has_section('stepper_z'):
        return config.getsection('stepper_z')
    for cconfig in config.get_prefix_sections('carriage '):
        carriage_name = cconfig.get_name().split()[-1].strip()
        axis_name = cconfig.get('axis', carriage_name, note_valid=False)
        if axis_name == 'z':
            return cconfig
    return None

class ManualProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.load_object(config, "gcode_move")
        self.gcode.register_command('MANUAL_PROBE', self.cmd_MANUAL_PROBE,
                                    desc=self.cmd_MANUAL_PROBE_help)
        # Endstop value for cartesian printers with separate Z axis
        zconfig = lookup_z_endstop_config(config)
        if zconfig is not None:
            self.z_position_endstop = zconfig.getfloat('position_endstop', None,
                                                       note_valid=False)
            self.z_endstop_config_name = zconfig.get_name()
        else:
            self.z_position_endstop = self.z_endstop_config_name = None
        # Endstop values for linear delta printers with vertical A,B,C towers
        a_tower_config = config.getsection('stepper_a')
        self.a_position_endstop = a_tower_config.getfloat('position_endstop',
                                                          None,
                                                          note_valid=False)
        b_tower_config = config.getsection('stepper_b')
        self.b_position_endstop = b_tower_config.getfloat('position_endstop',
                                                          None,
                                                          note_valid=False)
        c_tower_config = config.getsection('stepper_c')
        self.c_position_endstop = c_tower_config.getfloat('position_endstop',
                                                          None,
                                                          note_valid=False)
        # Conditionally register appropriate commands depending on printer
        # Cartestian printers with separate Z Axis
        if self.z_position_endstop is not None:
            self.gcode.register_command(
                'Z_ENDSTOP_CALIBRATE', self.cmd_Z_ENDSTOP_CALIBRATE,
                desc=self.cmd_Z_ENDSTOP_CALIBRATE_help)
            self.gcode.register_command(
                'Z_OFFSET_APPLY_ENDSTOP',
                self.cmd_Z_OFFSET_APPLY_ENDSTOP,
                desc=self.cmd_Z_OFFSET_APPLY_ENDSTOP_help)
        # Linear delta printers with A,B,C towers
        if 'delta' == config.getsection('printer').get('kinematics'):
            self.gcode.register_command(
                'Z_OFFSET_APPLY_ENDSTOP',
                self.cmd_Z_OFFSET_APPLY_DELTA_ENDSTOPS,
                desc=self.cmd_Z_OFFSET_APPLY_ENDSTOP_help)
        self.reset_status()
        self.pending_z_offset = None
        self.last_z_offset_save = {'saved': False, 'applied': False,
                                   'deferred': False, 'error': ''}
        if (self.z_position_endstop is not None and
                config.getsection('printer').get('kinematics') in
                ('cartesian', 'corexy')):
            aio = self.printer.load_object(config, 'aio_executor')
            self.z_save_executor = aio.allocate_executor('z_offset_save')
            self.gcode.register_command('_Z_OFFSET_SAVE_ENDSTOP',
                                        self.cmd_Z_OFFSET_SAVE_ENDSTOP,
                                        desc='Persist the Z endstop without restart')
            self.gcode.register_command('_Z_OFFSET_HOME_SAVED',
                                        self.cmd_Z_OFFSET_HOME_SAVED,
                                        desc='Home after saving only when no print is active')
            self.printer.register_event_handler('klippy:connect',
                                                self._register_z_result_macros)
    def _register_z_result_macros(self):
        for name in ('_Z_OFFSET_SHOW_RESULT', '_Z_OFFSET_RAISE_ERROR',
                     '_Z_OFFSET_CLOSE_RESULT'):
            macro = self.printer.lookup_object('gcode_macro ' + name, None)
            if macro is not None:
                self.gcode.register_command(name, None)
                self.gcode.register_command(name, macro.cmd,
                                            when_not_ready=True,
                                            desc=macro.cmd_desc)
    def _get_z_rail(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for rail in kin.rails:
            if rail.get_name() == self.z_endstop_config_name:
                if any(hasattr(es, 'get_position_endstop')
                       for es, name in rail.get_endstops()):
                    raise self.printer.command_error(
                        'Z offset saving requires an independent Z endstop')
                return rail
        raise self.printer.command_error('Unable to find the Z endstop rail')
    def _print_active(self):
        eventtime = self.printer.get_reactor().monotonic()
        stats = self.printer.lookup_object('print_stats', None)
        pause = self.printer.lookup_object('pause_resume', None)
        sd = self.printer.lookup_object('virtual_sdcard', None)
        return bool((stats is not None and stats.get_status(eventtime)['state']
                     in ('printing', 'paused')) or
                    (pause is not None and pause.is_paused) or
                    (sd is not None and sd.is_active()))
    def _tool_z_offset(self):
        tool = self.printer.lookup_object('toolhead').get_extruder().get_name()
        if tool == 'extruder':
            return 0.
        saved = self.printer.lookup_object('save_variables', None)
        if tool != 'extruder1' or saved is None or 't1_z' not in saved.allVariables:
            raise self.printer.command_error('Unable to identify the tool Z offset')
        value = float(saved.allVariables['t1_z'])
        if not math.isfinite(value):
            raise self.printer.command_error('Tool Z offset must be finite')
        return value
    def cmd_Z_OFFSET_SAVE_ENDSTOP(self, gcmd):
        raise_error = gcmd.get_int('RAISE_ON_ERROR', 1, minval=0, maxval=1)
        self.last_z_offset_save = {'saved': False, 'applied': False,
                                  'deferred': False, 'error': ''}
        try:
            if self.status['is_active']:
                raise gcmd.error('Finish manual probing before saving a Z offset')
            probe = self.printer.lookup_object('vulcan_probe', None)
            if probe is not None and (probe.calibrating or probe.meshing):
                raise gcmd.error('Finish probe calibration before saving a Z offset')
            rail = self._get_z_rail()
            origin = self.gcode_move.get_status()['homing_origin'].z
            offset = origin - self._tool_z_offset()
            value = float('%.3f' % (self.z_position_endstop - offset,))
            low, high = rail.get_range()
            if not math.isfinite(offset) or not math.isfinite(value) or not low <= value <= high:
                raise gcmd.error('Saved Z endstop must be finite and inside the axis range')
            self.printer.lookup_object('configfile').save_z_endstop(
                self.z_endstop_config_name, '%.3f' % (value,), self.z_save_executor)
            # The active baseline stays unchanged until Z homing. Repeated saves
            # therefore overwrite the target instead of accumulating the offset.
            self.pending_z_offset = value
            self.last_z_offset_save.update(
                saved=True, deferred=self._print_active(), position_endstop=value,
                saved_offset=self.z_position_endstop - value)
        except Exception as exc:
            self.last_z_offset_save['error'] = str(exc)
            if raise_error:
                raise gcmd.error(str(exc))
    def cmd_Z_OFFSET_HOME_SAVED(self, gcmd):
        result = self.last_z_offset_save
        if not result['saved'] or result['error']:
            return
        # Check again after disk IO; the UI's earlier state is not authority.
        result['deferred'] = self._print_active()
        if result['deferred']:
            return
        try:
            self.gcode.run_script_from_command('G28')
        except Exception as exc:
            result['error'] = str(exc)
    def prepare_z_offset_homing(self, axes):
        if 2 not in axes or self.pending_z_offset is None:
            return None
        rail = self._get_z_rail()
        origin = self.gcode_move.get_status()['homing_origin'].z
        if not math.isfinite(origin):
            raise self.printer.command_error('Active Z offset must be finite')
        state = (rail, rail.position_endstop, self.z_position_endstop, origin)
        value = self.pending_z_offset
        delta = self.z_position_endstop - value
        # Remove only the persisted component. Keep tool compensation and any
        # additional, unsaved babystep; MOVE=0 never moves the running print.
        self._set_z_origin(origin - delta)
        rail.position_endstop = self.z_position_endstop = value
        return state
    def _set_z_origin(self, value):
        # Call the existing coordinate setter directly so failure recovery also
        # works after an MCU shutdown. MOVE=0 never requests hardware motion.
        command = self.gcode.create_gcode_command('SET_GCODE_OFFSET', '',
                         {'Z': '%.9f' % (value,), 'MOVE': '0'})
        self.gcode_move.cmd_SET_GCODE_OFFSET(command)
    def finish_z_offset_homing(self, state, success):
        if state is None:
            return
        rail, endstop, baseline, origin = state
        if not success:
            rail.position_endstop, self.z_position_endstop = endstop, baseline
            self._set_z_origin(origin)
            self.last_z_offset_save['applied'] = False
            return
        self.pending_z_offset = None
        self.last_z_offset_save.update(applied=True, deferred=False)
        self.printer.lookup_object('configfile').note_z_endstop_applied(
            self.z_endstop_config_name, '%.3f' % (self.z_position_endstop,))
    def manual_probe_finalize(self, mpresult):
        if mpresult is not None:
            self.gcode.respond_info("Z position is %.3f" % (mpresult.bed_z,))
    def reset_status(self):
        self.status = {
            'is_active': False,
            'z_position': None,
            'z_position_lower': None,
            'z_position_upper': None
        }
    def get_status(self, eventtime):
        status = dict(self.status)
        status['z_offset_save'] = dict(self.last_z_offset_save)
        status['pending_z_endstop'] = self.pending_z_offset
        status['active_z_endstop'] = self.z_position_endstop
        return status
    cmd_MANUAL_PROBE_help = "Start manual probe helper script"
    def cmd_MANUAL_PROBE(self, gcmd):
        ManualProbeHelper(self.printer, gcmd, self.manual_probe_finalize)
    def z_endstop_finalize(self, mpresult):
        if mpresult is None:
            return
        z_pos = self.z_position_endstop - mpresult.bed_z
        self.gcode.respond_info(
            "%s: position_endstop: %.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % (
                self.z_endstop_config_name, z_pos,))
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.z_endstop_config_name, 'position_endstop',
                       "%.3f" % (z_pos,))
    cmd_Z_ENDSTOP_CALIBRATE_help = "Calibrate a Z endstop"
    def cmd_Z_ENDSTOP_CALIBRATE(self, gcmd):
        ManualProbeHelper(self.printer, gcmd, self.z_endstop_finalize)
    def cmd_Z_OFFSET_APPLY_ENDSTOP(self,gcmd):
        offset = self.gcode_move.get_status()['homing_origin'].z
        configfile = self.printer.lookup_object('configfile')
        if offset == 0:
            self.gcode.respond_info("Nothing to do: Z Offset is 0")
        else:
            new_calibrate = self.z_position_endstop - offset
            self.gcode.respond_info(
                "%s: position_endstop: %.3f\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with the above and restart the printer." % (
                    self.z_endstop_config_name, new_calibrate))
            configfile.set(self.z_endstop_config_name, 'position_endstop',
                           "%.3f" % (new_calibrate,))
    def cmd_Z_OFFSET_APPLY_DELTA_ENDSTOPS(self,gcmd):
        offset = self.gcode_move.get_status()['homing_origin'].z
        configfile = self.printer.lookup_object('configfile')
        if offset == 0:
            self.gcode.respond_info("Nothing to do: Z Offset is 0")
        else:
            new_a_calibrate = self.a_position_endstop - offset
            new_b_calibrate = self.b_position_endstop - offset
            new_c_calibrate = self.c_position_endstop - offset
            self.gcode.respond_info(
                "stepper_a: position_endstop: %.3f\n"
                "stepper_b: position_endstop: %.3f\n"
                "stepper_c: position_endstop: %.3f\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with the above and restart the printer." % (new_a_calibrate,
                                                             new_b_calibrate,
                                                             new_c_calibrate))
            configfile.set('stepper_a', 'position_endstop',
                "%.3f" % (new_a_calibrate,))
            configfile.set('stepper_b', 'position_endstop',
                "%.3f" % (new_b_calibrate,))
            configfile.set('stepper_c', 'position_endstop',
                "%.3f" % (new_c_calibrate,))
    cmd_Z_OFFSET_APPLY_ENDSTOP_help = "Adjust the z endstop_position"

# Verify that a manual probe isn't already in progress
def verify_no_manual_probe(printer):
    gcode = printer.lookup_object('gcode')
    try:
        gcode.register_command('ACCEPT', 'dummy')
    except printer.config_error as e:
        raise gcode.error(
            "Already in a manual Z probe. Use ABORT to abort it.")
    gcode.register_command('ACCEPT', None)

Z_BOB_MINIMUM = 0.500
BISECT_MAX = 0.200

# Helper script to determine a Z height
class ManualProbeHelper:
    def __init__(self, printer, gcmd, finalize_callback):
        self.printer = printer
        self.finalize_callback = finalize_callback
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.manual_probe = self.printer.lookup_object('manual_probe')
        self.speed = gcmd.get_float("SPEED", 5.)
        self.past_positions = []
        self.last_toolhead_pos = self.last_kinematics_pos = None
        # Register commands
        verify_no_manual_probe(printer)
        self.gcode.register_command('ACCEPT', self.cmd_ACCEPT,
                                    desc=self.cmd_ACCEPT_help)
        self.gcode.register_command('NEXT', self.cmd_ACCEPT)
        self.gcode.register_command('ABORT', self.cmd_ABORT,
                                    desc=self.cmd_ABORT_help)
        self.gcode.register_command('TESTZ', self.cmd_TESTZ,
                                    desc=self.cmd_TESTZ_help)
        self.gcode.respond_info(
            "Starting manual Z probe. Use TESTZ to adjust position.\n"
            "Finish with ACCEPT or ABORT command.")
        self.start_position = self.toolhead.get_position()
        self.report_z_status()
    def get_kinematics_pos(self):
        toolhead_pos = self.toolhead.get_position()
        if toolhead_pos == self.last_toolhead_pos:
            return self.last_kinematics_pos
        self.toolhead.flush_step_generation()
        kin = self.toolhead.get_kinematics()
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in kin.get_steppers()}
        kin_pos = kin.calc_position(kin_spos)
        self.last_toolhead_pos = toolhead_pos
        self.last_kinematics_pos = kin_pos
        return kin_pos
    def move_z(self, z_pos):
        curpos = self.toolhead.get_position()
        try:
            z_bob_pos = z_pos + Z_BOB_MINIMUM
            if curpos[2] < z_bob_pos:
                self.toolhead.manual_move([None, None, z_bob_pos], self.speed)
            self.toolhead.manual_move([None, None, z_pos], self.speed)
        except self.printer.command_error as e:
            self.finalize(False)
            raise
    def report_z_status(self, warn_no_change=False, prev_pos=None):
        # Get position
        kin_pos = self.get_kinematics_pos()
        z_pos = kin_pos[2]
        if warn_no_change and z_pos == prev_pos:
            self.gcode.respond_info(
                "WARNING: No change in position (reached stepper resolution)")
        # Find recent positions that were tested
        pp = self.past_positions
        next_pos = bisect.bisect_left(pp, z_pos)
        prev_pos = next_pos - 1
        if next_pos < len(pp) and pp[next_pos] == z_pos:
            next_pos += 1
        prev_pos_val = next_pos_val = None
        prev_str = next_str = "??????"
        if prev_pos >= 0:
            prev_pos_val = pp[prev_pos]
            prev_str = "%.3f" % (prev_pos_val,)
        if next_pos < len(pp):
            next_pos_val = pp[next_pos]
            next_str = "%.3f" % (next_pos_val,)
        self.manual_probe.status = {
            'is_active': True,
            'z_position': z_pos,
            'z_position_lower': prev_pos_val,
            'z_position_upper': next_pos_val,
        }
        # Find recent positions
        self.gcode.respond_info("Z position: %s --> %.3f <-- %s"
                                % (prev_str, z_pos, next_str))
    cmd_ACCEPT_help = "Accept the current Z position"
    def cmd_ACCEPT(self, gcmd):
        pos = self.toolhead.get_position()
        start_pos = self.start_position
        if pos[:2] != start_pos[:2] or pos[2] >= start_pos[2]:
            gcmd.respond_info(
                "Manual probe failed! Use TESTZ commands to position the\n"
                "nozzle prior to running ACCEPT.")
            self.finalize(False)
            return
        self.finalize(True)
    cmd_ABORT_help = "Abort manual Z probing tool"
    def cmd_ABORT(self, gcmd):
        self.finalize(False)
    cmd_TESTZ_help = "Move to new Z height"
    def cmd_TESTZ(self, gcmd):
        # Store current position for later reference
        kin_pos = self.get_kinematics_pos()
        z_pos = kin_pos[2]
        pp = self.past_positions
        insert_pos = bisect.bisect_left(pp, z_pos)
        if insert_pos >= len(pp) or pp[insert_pos] != z_pos:
            pp.insert(insert_pos, z_pos)
        # Determine next position to move to
        req = gcmd.get("Z")
        if req in ('+', '++'):
            check_z = 9999999999999.9
            if insert_pos < len(self.past_positions) - 1:
                check_z = self.past_positions[insert_pos + 1]
            if req == '+':
                check_z = (check_z + z_pos) / 2.
            next_z_pos = min(check_z, z_pos + BISECT_MAX)
        elif req in ('-', '--'):
            check_z = -9999999999999.9
            if insert_pos > 0:
                check_z = self.past_positions[insert_pos - 1]
            if req == '-':
                check_z = (check_z + z_pos) / 2.
            next_z_pos = max(check_z, z_pos - BISECT_MAX)
        else:
            next_z_pos = z_pos + gcmd.get_float("Z")
        # Move to given position and report it
        self.move_z(next_z_pos)
        self.report_z_status(next_z_pos != z_pos, z_pos)
    def finalize(self, success):
        self.manual_probe.reset_status()
        self.gcode.register_command('ACCEPT', None)
        self.gcode.register_command('NEXT', None)
        self.gcode.register_command('ABORT', None)
        self.gcode.register_command('TESTZ', None)
        mpresult = None
        if success:
            kin_pos = self.get_kinematics_pos()
            mpresult = create_probe_result(kin_pos)
        self.finalize_callback(mpresult)

def load_config(config):
    return ManualProbe(config)
