"""Coordinate the active nozzle frame, mesh lookup and saved T1 geometry."""
import configparser
import copy
import math
import os
import tempfile


class VulcanToolOffsets:
    KEYS = ('t1_x', 't1_y', 't1_z')

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.saved = self.printer.load_object(config, 'save_variables')
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.locale = ('TW' if config.fileconfig.has_section(
            'gcode_macro 後噴頭_偏移設定') else 'CN')
        self.last_result = dict(saved=False, applied=False, deferred=False,
                                error='')
        self.gcode.register_command('_VULCAN_TOOL_MESH', self.cmd_TOOL_MESH)
        # Klipper's command lexer on x600hd splits command names at digits.
        # Keep the private command name alphabetic so ``T1`` is not truncated.
        self.gcode.register_command('_VULCAN_SET_REAR_OFFSET',
                                    self.cmd_SET_T1_OFFSET)

    def _t(self, tw, cn):
        return tw if self.locale == 'TW' else cn

    def _head(self):
        return self.printer.lookup_object('toolhead')

    def _active_t1(self):
        return self._head().get_extruder().get_name() == 'extruder1'

    def sync_mesh(self):
        mesh = self.printer.lookup_object('bed_mesh', None)
        if mesh is None or mesh.z_mesh is None:
            return
        origin = self.gcode_move.homing_position
        offsets = origin[:3] if self._active_t1() else (0., 0., 0.)
        if not all(math.isfinite(value) for value in offsets):
            raise self.printer.command_error('Nonfinite active tool offset')
        # Bed mesh consumes carriage coordinates. Undo the nozzle's G-code
        # offsets for XY lookup and for the height used by mesh fade.
        command = self.gcode.create_gcode_command('BED_MESH_OFFSET', '',
            {key: str(-value) for key, value in zip(('X', 'Y', 'ZFADE'), offsets)})
        mesh.cmd_BED_MESH_OFFSET(command)

    def cmd_TOOL_MESH(self, gcmd):
        self.sync_mesh()

    def _check_calibration(self):
        probe = self.printer.lookup_object('vulcan_probe', None)
        manual = self.printer.lookup_object('manual_probe', None)
        if ((probe is not None and (probe.calibrating or probe.meshing))
                or (manual is not None and manual.get_status(None)['is_active'])):
            raise self.printer.command_error(self._t(
                '校準期間不能修改 T1 偏移。', '校准期间不能修改 T1 偏移。'))

    def _persist(self, values):
        variables = dict(self.saved.allVariables)
        variables.update(zip(self.KEYS, values))
        if variables == self.saved.allVariables:
            return
        parser = configparser.ConfigParser()
        parser.add_section('Variables')
        for key, value in sorted(variables.items()):
            parser.set('Variables', key, repr(value))

        def write_atomic():
            path = os.path.realpath(self.saved.filename)
            directory = os.path.dirname(os.path.abspath(path))
            mode = os.stat(path).st_mode & 0o777
            staging = None
            try:
                with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                        dir=directory, prefix='.tool-offset-', suffix='.pending',
                        delete=False) as output:
                    staging = output.name
                    os.chmod(output.name, mode)
                    parser.write(output)
                    output.flush()
                    os.fsync(output.fileno())
                os.replace(staging, path)
                staging = None
            finally:
                if staging is not None and os.path.exists(staging):
                    os.unlink(staging)

        # Reuse the native save_variables worker, keeping disk IO off the
        # Klipper reactor. Publish the dictionary only after replacement.
        self.saved.executor.submit(write_atomic)
        self.saved.allVariables = variables

    def _homed(self):
        now = self.printer.get_reactor().monotonic()
        return all(axis in self._head().get_status(now)['homed_axes']
                   for axis in 'xyz')

    def _check_target(self, target):
        now = self.printer.get_reactor().monotonic()
        status = self._head().get_status(now)
        low, high = status['axis_minimum'], status['axis_maximum']
        for axis in (0, 1):
            if not low[axis] <= target[axis] <= high[axis]:
                raise self.printer.command_error('T1 offset target outside axis range')
        # A conservative envelope also covers thermal and mesh transforms.
        zlow = zhigh = target[2]
        thermal = self.printer.lookup_object('z_thermal_adjust', None)
        if thermal is not None:
            margin = max(abs(thermal.z_adjust_mm), thermal.max_z_adjust_mm)
            zlow -= margin
            zhigh += margin
        mesh = self.printer.lookup_object('bed_mesh', None)
        if mesh is not None and mesh.z_mesh is not None:
            lower, upper = mesh.z_mesh.get_z_range()
            zlow += min(0., lower, mesh.fade_target)
            zhigh += max(0., upper, mesh.fade_target)
        if not low[2] <= zlow <= zhigh <= high[2]:
            raise self.printer.command_error('T1 offset target outside safe Z range')

    def _apply(self, values):
        move = self.gcode_move
        old = {key: list(getattr(move, key)) for key in
               ('base_position', 'homing_position', 'last_position')}
        target = list(move.last_position)
        for axis, value in enumerate(values):
            target[axis] += value - old['homing_position'][axis]
        self._check_target(target)
        mesh = self.printer.lookup_object('bed_mesh', None)
        mesh_state = ((mesh.z_mesh, list(mesh.z_mesh.mesh_offsets), mesh.tool_offset)
                      if mesh is not None and mesh.z_mesh is not None else None)
        thermal = self.printer.lookup_object('z_thermal_adjust', None)
        thermal_state = thermal.capture_state() if thermal is not None else None
        try:
            command = self.gcode.create_gcode_command('SET_GCODE_OFFSET', '',
                dict(zip('XYZ', map(str, values)), MOVE='0'))
            move.cmd_SET_GCODE_OFFSET(command)
            self.sync_mesh()
            move.move_with_transform(target, 5.)
            move.last_position[:] = target
        except Exception:
            for key, value in old.items():
                getattr(move, key)[:] = value
            if mesh_state is not None and mesh.z_mesh is mesh_state[0]:
                mesh.z_mesh.set_mesh_offsets(mesh_state[1])
                mesh.tool_offset = mesh_state[2]
            if thermal_state is not None:
                thermal.restore_state(thermal_state)
            move.reset_last_position()
            raise

    def cmd_SET_T1_OFFSET(self, gcmd):
        self.last_result = dict(saved=False, applied=False, deferred=False,
                                error='')
        try:
            if set(gcmd.get_command_parameters()) - set('XYZ'):
                raise gcmd.error('T1 offset accepts only X, Y and Z')
            self._check_calibration()
            values = tuple(gcmd.get_float(axis, float(
                self.saved.allVariables.get(key, 0.)))
                for axis, key in zip('XYZ', self.KEYS))
            if not all(math.isfinite(value) for value in values):
                raise gcmd.error('T1 offsets must be finite')
            limits = self._head().get_status(
                self.printer.get_reactor().monotonic())
            if any(abs(value) > limits['axis_maximum'][axis]
                   - limits['axis_minimum'][axis]
                   for axis, value in enumerate(values)):
                raise gcmd.error('T1 offset exceeds axis travel')
            if self._active_t1() and self._homed():
                target = list(self.gcode_move.last_position)
                for axis, value in enumerate(values):
                    target[axis] += value - self.gcode_move.homing_position[axis]
                self._check_target(target)
            self._persist(values)
            self.last_result.update(saved=True, offsets=list(values))
            # The worker yields: inspect live tool/homing state again after IO.
            self._check_calibration()
            if self._active_t1() and self._homed():
                self._apply(values)
                self.last_result['applied'] = True
                message = self._t('T1 偏移已儲存並套用。', 'T1 偏移已保存并套用。')
            else:
                self.last_result['deferred'] = True
                message = self._t(
                    'T1 偏移已儲存，尚未套用；歸零後於 T1 啟用時套用，未移動。',
                    'T1 偏移已保存，尚未套用；归零后于 T1 启用时套用，未移动。')
            gcmd.respond_info(message + ' X=%.3f Y=%.3f Z=%.3f' % values)
        except Exception as exc:
            self.last_result['error'] = str(exc)
            if self.last_result['saved']:
                self.last_result['deferred'] = True
                gcmd.respond_info(self._t('T1 偏移已儲存，但未套用。',
                                         'T1 偏移已保存，但未套用。'))
            raise gcmd.error(str(exc))

    def get_status(self, eventtime):
        return {'last_offset_update': copy.deepcopy(self.last_result)}


def load_config(config):
    return VulcanToolOffsets(config)
