# Unit tests for responsive temperature waits
import io
import os
import sys
import unittest


KLIPPER_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                            '..'))
KLIPPY_DIR = os.path.join(KLIPPER_ROOT, 'klippy')
if KLIPPER_ROOT not in sys.path:
    sys.path.insert(0, KLIPPER_ROOT)

from klippy.extras import heaters, virtual_sdcard

if KLIPPY_DIR not in sys.path:
    sys.path.append(KLIPPY_DIR)
import gcode
import reactor


class FakeCommandError(Exception):
    pass


class FakeTemplate:
    def render(self):
        return ""


class FakePrintStats:
    def __init__(self):
        self.state = "standby"
        self.pause_count = 0

    def set_current_file(self, filename):
        self.state = "standby"

    def note_start(self):
        self.state = "printing"

    def note_pause(self):
        self.pause_count += 1
        self.state = "paused"

    def note_complete(self):
        self.state = "complete"

    def note_error(self, message):
        self.state = "error"

    def note_cancel(self):
        self.state = "cancelled"

    def reset(self):
        self.state = "standby"


class DeterministicReactor(reactor.SelectReactor):
    """Run real Klipper greenlets and mutexes against a virtual clock."""
    def __init__(self):
        reactor.SelectReactor.__init__(self)
        self.now = 0.
        self.monotonic = lambda: self.now

    def _dispatch_loop(self):
        while self._process:
            if self._next_timer >= self.NEVER:
                raise RuntimeError("Deterministic reactor has no next timer")
            self.now = max(self.now, self._next_timer)
            self._check_timers(self.now, False)


class FakeDispatchPrinter:
    command_error = gcode.CommandError

    def __init__(self, test_reactor):
        self.reactor = test_reactor
        self.event_handlers = {}
        self.events = []
        self.shutdown = False
        self.shutdown_messages = []
        self.objects = {}

    def get_reactor(self):
        return self.reactor

    def get_start_args(self):
        return {}

    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)

    def config_error(self, message):
        return gcode.CommandError(message)

    def send_event(self, event, *args):
        self.events.append((event, args))

    def invoke_shutdown(self, message):
        self.shutdown = True
        self.shutdown_messages.append(message)

    def is_shutdown(self):
        return self.shutdown

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


class NamedStringIO(io.StringIO):
    name = "temperature_waiting.gcode"


class FakeLiveHeater(heaters.Heater):
    def __init__(self, temperature, tolerance=3., name='extruder'):
        self.temperature = temperature
        self.tolerance = tolerance
        self.name = name

    def get_temp(self, eventtime):
        return (self.temperature['actual'], self.temperature['target'])

    def get_temperature_wait_tolerance(self):
        return self.tolerance

    def get_name(self):
        return self.name

    def set_temp(self, target):
        self.temperature['target'] = target


class UniversalWaitHarness(unittest.TestCase):
    def setUp(self):
        self.reactor = DeterministicReactor()
        self.printer = FakeDispatchPrinter(self.reactor)
        self.gcode = gcode.GCodeDispatch(self.printer)
        self.gcode._handle_ready()
        self.trace = []
        self.temperature = {'actual': 20., 'target': 100.}
        self.heater = FakeLiveHeater(self.temperature)
        self.other_heater_target = 60.
        self.virtual_sd = None
        self.mid_wait = None

        self.gcode.register_command('MARK', self.cmd_MARK)
        self.gcode.register_command('MOVE', self.cmd_MOVE)
        self.gcode.register_command(
            'SET_HEATER_TEMPERATURE', self.cmd_SET_HEATER_TEMPERATURE,
            during_temperature_wait=True)
        self.gcode.register_command(
            'TURN_OFF_HEATERS', self.cmd_TURN_OFF_HEATERS,
            during_temperature_wait=True)
        self.gcode.register_command('M109', self.cmd_M109)
        self.gcode.register_command('INNER_WAIT', self.cmd_INNER_WAIT)
        self.gcode.register_command('OUTER_WAIT', self.cmd_OUTER_WAIT)
        self.gcode.register_command('CANCEL_BASE', self.cmd_CANCEL_BASE)
        self.gcode.register_command('CANCEL_PRINT', self.cmd_CANCEL_PRINT)

    def tearDown(self):
        self.reactor.end()
        self.reactor.finalize()

    def cmd_MARK(self, gcmd):
        self.trace.append(gcmd.get('NAME'))

    def cmd_MOVE(self, gcmd):
        self.trace.append("move:" + gcmd.get('NAME'))

    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        target = gcmd.get_float('TARGET')
        self.temperature['target'] = target
        self.trace.append(('set_target', target))

    def cmd_TURN_OFF_HEATERS(self, gcmd):
        self.temperature['target'] = 0.
        self.other_heater_target = 0.
        self.trace.append('turn_off_heaters')

    def cmd_M109(self, gcmd):
        target = gcmd.get_float('S', self.temperature['target'])
        self.temperature['target'] = target
        self.trace.append(('wait', target))
        target_wait = heaters.HeaterTargetWait(self.heater)
        if self.virtual_sd is not None:
            self.virtual_sd.begin_temperature_wait(
                'extruder', target_wait.check_ready,
                target_wait.get_target)
        else:
            self.gcode.wait_for_temperature(
                target_wait.check_ready, .25)

    def cmd_INNER_WAIT(self, gcmd):
        self.gcode.run_script_from_command(
            "MARK NAME=inner_before\n"
            "M109 S100\n"
            "MARK NAME=inner_after")

    def cmd_OUTER_WAIT(self, gcmd):
        self.gcode.run_script_from_command(
            "MARK NAME=outer_before\n"
            "INNER_WAIT\n"
            "MARK NAME=outer_after")

    def cmd_CANCEL_BASE(self, gcmd):
        self.trace.append('cancel_base')
        self.other_heater_target = 0.
        if self.virtual_sd is not None:
            self.virtual_sd.do_cancel()

    def cmd_CANCEL_PRINT(self, gcmd):
        # Model a user macro that wraps Klipper's renamed base command.
        self.gcode.run_script_from_command(
            "MARK NAME=cancel_before\n"
            "CANCEL_BASE\n"
            "MARK NAME=cancel_after")

    def schedule(self, waketime, callback):
        self.reactor.register_callback(callback, waketime)

    def run_until(self, waketime=2.):
        watchdog = []

        def stop(eventtime):
            watchdog.append(eventtime)
            self.reactor.end()

        self.schedule(waketime, stop)
        self.reactor.run()
        self.assertTrue(watchdog)

    def set_target(self, target, actual=None):
        if actual is not None:
            self.temperature['actual'] = actual
        self.gcode.run_script(
            "SET_HEATER_TEMPERATURE HEATER=extruder TARGET=%s" % (target,))

    def build_virtual_sd(self, gcode_text):
        vsd = virtual_sdcard.VirtualSD.__new__(virtual_sdcard.VirtualSD)
        stats = FakePrintStats()
        vsd.current_file = NamedStringIO(gcode_text)
        vsd.file_position = 0
        vsd.file_size = len(gcode_text.encode())
        vsd.cancelable_temperature_wait = True
        vsd.temperature_wait_check_interval = .25
        vsd.temperature_wait = None
        vsd.print_stats = stats
        vsd.reactor = self.reactor
        vsd.must_pause_work = False
        vsd.cmd_from_sd = False
        vsd.next_file_position = 0
        vsd.work_timer = None
        vsd.on_error_gcode = FakeTemplate()
        vsd.gcode = self.gcode
        vsd.printer = self.printer
        self.virtual_sd = vsd
        self.printer.objects['virtual_sdcard'] = vsd
        return vsd, stats


class UniversalTemperatureWaitingTest(UniversalWaitHarness):
    def test_console_nested_macro_preserves_all_continuations(self):
        root_result = []
        root_return_time = []

        def start(eventtime):
            result = self.gcode.run_script(
                "OUTER_WAIT\nMARK NAME=top_after")
            root_result.append(result)
            self.trace.append('root_return')
            root_return_time.append(self.reactor.monotonic())

        def inspect_wait(eventtime):
            self.mid_wait = list(self.trace)

        self.schedule(0., start)
        self.schedule(.1, inspect_wait)
        self.schedule(.2, lambda eventtime: self.set_target(60., actual=60.))
        self.run_until()

        self.assertEqual(
            ['outer_before', 'inner_before', ('wait', 100.)],
            self.mid_wait)
        self.assertEqual([
            'outer_before', 'inner_before', ('wait', 100.),
            ('set_target', 60.), 'inner_after', 'outer_after',
            'top_after', 'root_return'], self.trace)
        self.assertEqual([None], root_result)
        self.assertEqual([.2], root_return_time)
        self.assertFalse(self.printer.shutdown)

    def test_unsafe_console_command_waits_behind_owner(self):
        unsafe_result = []

        def start(eventtime):
            self.gcode.run_script(
                "MARK NAME=owner_before\n"
                "M109 S100\n"
                "MARK NAME=owner_after")
            self.trace.append('owner_return')

        def unsafe(eventtime):
            self.trace.append('unsafe_call')
            unsafe_result.append(self.gcode.run_script(
                "MOVE NAME=unsafe"))
            self.trace.append('unsafe_return')

        self.schedule(0., start)
        self.schedule(.1, unsafe)
        self.schedule(.2, lambda eventtime: self.set_target(60., actual=60.))
        self.run_until()

        self.assertLess(self.trace.index('owner_after'),
                        self.trace.index('move:unsafe'))
        self.assertLess(self.trace.index('owner_return'),
                        self.trace.index('move:unsafe'))
        self.assertEqual([None], unsafe_result)

    def test_mixed_control_script_does_not_let_move_bypass_wait(self):
        def start(eventtime):
            self.gcode.run_script(
                "M109 S100\nMARK NAME=owner_after")

        def mixed_script(eventtime):
            self.trace.append('mixed_call')
            self.gcode.run_script(
                "SET_HEATER_TEMPERATURE HEATER=extruder TARGET=40\n"
                "MOVE NAME=injected")
            self.trace.append('mixed_return')

        self.schedule(0., start)
        self.schedule(.1, mixed_script)
        self.schedule(.2, lambda eventtime: self.set_target(20., actual=20.))
        self.run_until()

        self.assertLess(self.trace.index('owner_after'),
                        self.trace.index(('set_target', 40.)))
        self.assertLess(self.trace.index(('set_target', 40.)),
                        self.trace.index('move:injected'))

    def test_live_target_raise_does_not_finish_at_old_target(self):
        self.temperature['actual'] = 90.
        snapshots = []

        def start(eventtime):
            self.gcode.run_script(
                "M109 S100\nMARK NAME=owner_after")

        def raise_target(eventtime):
            self.temperature['actual'] = 100.
            self.set_target(120.)

        def inspect(eventtime):
            snapshots.append(list(self.trace))

        def reach_new_target(eventtime):
            self.set_target(120., actual=120.)

        self.schedule(0., start)
        self.schedule(.1, raise_target)
        self.schedule(.2, inspect)
        self.schedule(.3, reach_new_target)
        self.run_until()

        self.assertNotIn('owner_after', snapshots[0])
        self.assertIn('owner_after', self.trace)

    def test_live_target_lower_waits_for_cooling_tolerance(self):
        self.temperature['actual'] = 150.
        snapshots = []

        def start(eventtime):
            self.gcode.run_script(
                "M109 S200\nMARK NAME=owner_after")

        self.schedule(0., start)
        self.schedule(.1, lambda eventtime: self.set_target(100.))
        self.schedule(.2, lambda eventtime:
                      snapshots.append(list(self.trace)))
        self.schedule(.3, lambda eventtime:
                      self.set_target(100., actual=103.))
        self.run_until()

        self.assertNotIn('owner_after', snapshots[0])
        self.assertIn('owner_after', self.trace)

    def test_cooling_completion_uses_periodic_sensor_check(self):
        self.temperature['actual'] = 150.
        return_time = []

        def start(eventtime):
            self.gcode.run_script(
                "M109 S100\nMARK NAME=owner_after")
            return_time.append(self.reactor.monotonic())

        self.schedule(0., start)
        # A sensor update does not issue G-Code or explicitly wake the wait.
        self.schedule(.1, lambda eventtime:
                      self.temperature.__setitem__('actual', 90.))
        self.run_until()

        self.assertIn('owner_after', self.trace)
        self.assertEqual([.25], return_time)

    def test_rechecks_target_after_mutex_queue_setter(self):
        self.temperature['actual'] = 100.
        snapshots = []

        def racy_wait(gcmd):
            self.temperature['target'] = 100.
            target_wait = heaters.HeaterTargetWait(self.heater)
            # Keep the mutex while the higher-target setter enters its queue.
            self.reactor.pause(.1)
            self.gcode.wait_for_temperature(
                target_wait.check_ready, .25)

        self.gcode.register_command('RACY_WAIT', racy_wait)

        def start(eventtime):
            self.gcode.run_script(
                "RACY_WAIT\nMARK NAME=owner_after")

        self.schedule(0., start)
        self.schedule(.05, lambda eventtime: self.set_target(120.))
        self.schedule(.2, lambda eventtime:
                      snapshots.append(list(self.trace)))
        self.schedule(.3, lambda eventtime:
                      self.set_target(120., actual=120.))
        self.run_until()

        self.assertNotIn('owner_after', snapshots[0])
        self.assertIn('owner_after', self.trace)

    def test_rechecks_lower_target_after_mutex_queue_setter(self):
        self.temperature['actual'] = 100.
        snapshots = []

        def racy_wait(gcmd):
            self.temperature['target'] = 100.
            target_wait = heaters.HeaterTargetWait(self.heater)
            self.reactor.pause(.1)
            self.gcode.wait_for_temperature(
                target_wait.check_ready, .25)

        self.gcode.register_command('RACY_COOL_WAIT', racy_wait)

        def start(eventtime):
            self.gcode.run_script(
                "RACY_COOL_WAIT\nMARK NAME=owner_after")

        self.schedule(0., start)
        self.schedule(.05, lambda eventtime: self.set_target(80.))
        self.schedule(.2, lambda eventtime:
                      snapshots.append(list(self.trace)))
        self.schedule(.3, lambda eventtime:
                      self.set_target(80., actual=83.))
        self.run_until()

        self.assertNotIn('owner_after', snapshots[0])
        self.assertIn('owner_after', self.trace)

    def test_turn_off_heaters_releases_wait_and_continues_script(self):
        owner_result = []

        def start(eventtime):
            owner_result.append(self.gcode.run_script(
                "M109 S100\nMARK NAME=owner_after"))

        self.schedule(0., start)
        self.schedule(.1, lambda eventtime:
                      self.gcode.run_script("TURN_OFF_HEATERS"))
        self.run_until()

        self.assertIn('turn_off_heaters', self.trace)
        self.assertIn('owner_after', self.trace)
        self.assertEqual([None], owner_result)
        self.assertEqual(0., self.other_heater_target)

    def test_zero_target_continues_nested_macro_stack(self):
        owner_result = []

        def start(eventtime):
            owner_result.append(self.gcode.run_script(
                "OUTER_WAIT\nMARK NAME=top_after"))

        self.schedule(0., start)
        self.schedule(.1, lambda eventtime: self.set_target(0.))
        self.run_until()

        self.assertIn('inner_after', self.trace)
        self.assertIn('outer_after', self.trace)
        self.assertIn('top_after', self.trace)
        self.assertNotIn('cancel_before', self.trace)
        self.assertNotIn('cancel_base', self.trace)
        self.assertEqual([None], owner_result)
        self.assertEqual(60., self.other_heater_target)

    def test_wait_error_discards_queued_unsafe_command(self):
        owner_errors = []
        unsafe_results = []

        def broken_wait(gcmd):
            def check_ready(eventtime):
                if eventtime >= .2:
                    raise gcmd.error('temperature sensor failed')
                return False
            self.gcode.wait_for_temperature(check_ready, .25)

        self.gcode.register_command('BROKEN_WAIT', broken_wait)

        def start(eventtime):
            try:
                self.gcode.run_script(
                    "BROKEN_WAIT\nMARK NAME=owner_after")
            except gcode.CommandError as error:
                owner_errors.append(str(error))

        def unsafe(eventtime):
            unsafe_results.append(self.gcode.run_script(
                "MOVE NAME=unsafe"))

        self.schedule(0., start)
        self.schedule(.1, unsafe)
        self.run_until()

        self.assertEqual(['temperature sensor failed'], owner_errors)
        self.assertNotIn('owner_after', self.trace)
        self.assertNotIn('move:unsafe', self.trace)
        self.assertEqual([self.gcode.SCRIPT_CANCELLED], unsafe_results)

    def test_cancel_unwinds_owner_macro_and_batch(self):
        owner_result = []

        def start(eventtime):
            owner_result.append(self.gcode.run_script(
                "OUTER_WAIT\nMARK NAME=top_after"))
            self.trace.append('owner_return')

        self.schedule(0., start)
        self.schedule(.1, lambda eventtime:
                      self.gcode.run_script("CANCEL_PRINT"))
        self.run_until()

        self.assertIn('cancel_before', self.trace)
        self.assertIn('cancel_base', self.trace)
        self.assertIn('cancel_after', self.trace)
        self.assertNotIn('inner_after', self.trace)
        self.assertNotIn('outer_after', self.trace)
        self.assertNotIn('top_after', self.trace)
        self.assertEqual([self.gcode.SCRIPT_CANCELLED], owner_result)
        self.assertFalse(self.printer.shutdown)

    def test_cancel_discards_unsafe_command_queued_during_wait(self):
        unsafe_result = []

        def start(eventtime):
            self.gcode.run_script(
                "M109 S100\nMARK NAME=owner_after")

        def unsafe(eventtime):
            unsafe_result.append(self.gcode.run_script(
                "MOVE NAME=unsafe"))
            self.trace.append('unsafe_return')

        self.schedule(0., start)
        self.schedule(.05, unsafe)
        self.schedule(.1, lambda eventtime:
                      self.gcode.run_script("CANCEL_PRINT"))
        self.run_until()

        self.assertNotIn('owner_after', self.trace)
        self.assertNotIn('move:unsafe', self.trace)
        self.assertEqual([self.gcode.SCRIPT_CANCELLED], unsafe_result)
        self.assertFalse(self.printer.shutdown)

    def test_cancel_preflight_wins_mutex_queue_race(self):
        owner_result = []

        def slow_setter(gcmd):
            self.temperature['target'] = 60.
            self.temperature['actual'] = 60.
            self.trace.append('slow_setter_wake')
            # Wake the owner while this control command still owns the mutex.
            # The owner will be first in the mutex queue.  CANCEL_PRINT must
            # mark the wait cancelled before it queues behind that owner.
            self.gcode._wake_temperature_wait()
            self.reactor.pause(.2)
            self.trace.append('slow_setter_return')

        self.gcode.register_command(
            'SLOW_SETTER', slow_setter, during_temperature_wait=True)

        def start(eventtime):
            owner_result.append(self.gcode.run_script(
                "M109 S100\nMARK NAME=owner_after"))
            self.trace.append('owner_return')

        self.schedule(0., start)
        self.schedule(.1, lambda eventtime:
                      self.gcode.run_script("SLOW_SETTER"))
        self.schedule(.15, lambda eventtime:
                      self.gcode.run_script("CANCEL_PRINT"))
        self.run_until()

        self.assertNotIn('owner_after', self.trace)
        self.assertEqual([self.gcode.SCRIPT_CANCELLED], owner_result)
        self.assertLess(self.trace.index('slow_setter_return'),
                        self.trace.index('cancel_before'))
        self.assertFalse(self.printer.shutdown)

    def test_wait_cannot_unlock_mutex_owned_by_another_greenlet(self):
        attempts = []

        def mutex_owner(eventtime):
            with self.gcode.get_mutex():
                self.trace.append('mutex_owner_start')
                self.reactor.pause(.2)
                self.trace.append('mutex_owner_end')

        def invalid_waiter(eventtime):
            try:
                self.gcode.wait_for_temperature(lambda now: False, .25)
            except gcode.CommandError as error:
                attempts.append(str(error))

        self.schedule(0., mutex_owner)
        self.schedule(.1, invalid_waiter)
        self.run_until()

        self.assertEqual(['Temperature wait requested outside G-Code dispatch'],
                         attempts)
        self.assertEqual(['mutex_owner_start', 'mutex_owner_end'], self.trace)
        self.assertIsNone(self.gcode.active_temperature_wait)

    def test_macro_override_does_not_inherit_safe_setter_permission(self):
        original = self.gcode.register_command(
            'SET_HEATER_TEMPERATURE', None)
        self.assertIsNotNone(original)

        def overridden_setter(gcmd):
            self.trace.append('overridden_setter')

        def core_setter(gcmd):
            self.temperature['target'] = 60.
            self.temperature['actual'] = 60.
            self.trace.append('core_setter')

        self.gcode.register_command(
            'SET_HEATER_TEMPERATURE', overridden_setter)
        self.gcode.register_command(
            'CORE_SETTER', core_setter, during_temperature_wait=True)

        def start(eventtime):
            self.gcode.run_script(
                "M109 S100\nMARK NAME=owner_after")

        self.schedule(0., start)
        self.schedule(.1, lambda eventtime:
                      self.gcode.run_script(
                          "SET_HEATER_TEMPERATURE HEATER=x TARGET=60"))
        self.schedule(.2, lambda eventtime:
                      self.gcode.run_script("CORE_SETTER"))
        self.run_until()

        self.assertLess(self.trace.index('owner_after'),
                        self.trace.index('overridden_setter'))
        self.assertLess(self.trace.index('core_setter'),
                        self.trace.index('owner_after'))

    def test_cancel_macro_error_still_cancels_virtual_sd_owner(self):
        vsd, stats = self.build_virtual_sd(
            "M109 S100\nMARK NAME=file_after\n")
        self.gcode.register_command('CANCEL_PRINT', None)
        cancel_errors = []

        def broken_cancel(gcmd):
            self.trace.append('broken_cancel')
            raise gcmd.error('cancel macro failed')

        self.gcode.register_command('CANCEL_PRINT', broken_cancel)

        def request_cancel(eventtime):
            try:
                self.gcode.run_script("CANCEL_PRINT")
            except gcode.CommandError as error:
                cancel_errors.append(str(error))

        vsd.work_timer = self.reactor.register_timer(vsd.work_handler, 0.)
        self.schedule(.1, request_cancel)
        self.run_until()

        self.assertEqual(['cancel macro failed'], cancel_errors)
        self.assertEqual('cancelled', stats.state)
        self.assertIsNone(vsd.current_file)
        self.assertNotIn('file_after', self.trace)
        self.assertFalse(self.printer.shutdown)

    def test_virtual_sd_nested_wait_keeps_printing_and_orders_file(self):
        vsd, stats = self.build_virtual_sd(
            "OUTER_WAIT\nMARK NAME=file_after\n")
        wait_status = []

        def inspect(eventtime):
            wait_status.append((stats.state, stats.pause_count,
                                vsd.get_status(eventtime), list(self.trace)))

        vsd.work_timer = self.reactor.register_timer(vsd.work_handler, 0.)
        self.schedule(.1, inspect)
        self.schedule(.2, lambda eventtime: self.set_target(60., actual=60.))
        self.run_until()

        state, pause_count, status, mid_trace = wait_status[0]
        self.assertEqual('printing', state)
        self.assertEqual(0, pause_count)
        self.assertTrue(status['is_active'])
        self.assertTrue(status['temperature_waiting'])
        self.assertEqual('extruder', status['temperature_wait_sensor'])
        self.assertEqual(100., status['temperature_wait_target'])
        self.assertNotIn('inner_after', mid_trace)
        self.assertNotIn('file_after', mid_trace)
        self.assertLess(self.trace.index('inner_after'),
                        self.trace.index('outer_after'))
        self.assertLess(self.trace.index('outer_after'),
                        self.trace.index('file_after'))
        self.assertEqual('complete', stats.state)
        self.assertEqual(0, stats.pause_count)

    def test_virtual_sd_cancel_preserves_cancelled_state(self):
        vsd, stats = self.build_virtual_sd(
            "M109 S100\nMARK NAME=file_after\n")

        vsd.work_timer = self.reactor.register_timer(vsd.work_handler, 0.)
        self.schedule(.1, lambda eventtime:
                      self.gcode.run_script("CANCEL_PRINT"))
        self.run_until()

        self.assertEqual('cancelled', stats.state)
        self.assertIsNone(vsd.current_file)
        self.assertIsNone(vsd.temperature_wait)
        self.assertNotIn('file_after', self.trace)
        self.assertFalse(self.printer.shutdown)

    def test_virtual_sd_zero_target_continues_file(self):
        vsd, stats = self.build_virtual_sd(
            "M109 S100\nMARK NAME=file_after\n")

        vsd.work_timer = self.reactor.register_timer(vsd.work_handler, 0.)
        self.schedule(.1, lambda eventtime: self.set_target(0.))
        self.run_until()

        self.assertEqual('complete', stats.state)
        self.assertIsNone(vsd.current_file)
        self.assertIsNone(vsd.temperature_wait)
        self.assertIn('file_after', self.trace)
        self.assertNotIn('cancel_before', self.trace)
        self.assertNotIn('cancel_base', self.trace)
        self.assertNotIn('cancel_after', self.trace)
        self.assertEqual(60., self.other_heater_target)
        self.assertFalse(self.printer.shutdown)

    def test_feature_disabled_does_not_register_wait(self):
        vsd, stats = self.build_virtual_sd("")
        vsd.cancelable_temperature_wait = False
        registered = vsd.begin_temperature_wait(
            'extruder', lambda eventtime: False,
            lambda eventtime: self.temperature['target'])
        self.assertFalse(registered)
        self.assertIsNone(vsd.temperature_wait)


class FakeWaitGCode:
    error = FakeCommandError

    def __init__(self, **params):
        self.params = params
        self.responses = []

    def respond_raw(self, message):
        self.responses.append(message)

    def get(self, name, default=None):
        return self.params.get(name, default)

    def get_int(self, name, default=None, minval=None, maxval=None):
        value = int(self.params.get(name, default))
        if minval is not None and value < minval:
            raise self.error("value below minimum")
        if maxval is not None and value > maxval:
            raise self.error("value above maximum")
        return value

    def get_float(self, name, default=None, minval=None, maxval=None,
                  above=None, below=None):
        value = float(self.params.get(name, default))
        if above is not None and value <= above:
            raise self.error("value must be above minimum")
        return value


class FakeVirtualSDWaitReceiver:
    def __init__(self):
        self.wait = None

    def begin_temperature_wait(self, sensor, check_ready, get_target):
        self.wait = (sensor, check_ready, get_target)
        return True


class FakeTemperatureWaitDispatch:
    def __init__(self):
        self.notify_count = 0
        self.responses = []

    def notify_temperature_wait(self):
        self.notify_count += 1

    def respond_raw(self, message):
        self.responses.append(message)

class FakeDisabledVirtualSD:
    def begin_temperature_wait(self, sensor, check_ready, get_target):
        return False


class FakeFallbackReactor:
    def __init__(self, on_pause=None):
        self.now = 0.
        self.pause_count = 0
        self.on_pause = on_pause

    def monotonic(self):
        return self.now

    def pause(self, waketime):
        self.now = waketime
        self.pause_count += 1
        if self.on_pause is not None:
            self.on_pause(self.pause_count)
        return self.now


class FakeFallbackToolhead:
    def __init__(self):
        self.flush_count = 0

    def get_last_move_time(self):
        self.flush_count += 1
        return 0.


class FakeFallbackPrinter:
    def __init__(self, reactor, dispatch, virtual_sd, toolhead):
        self.reactor = reactor
        self.objects = {
            'gcode': dispatch,
            'virtual_sdcard': virtual_sd,
            'toolhead': toolhead,
        }

    def get_start_args(self):
        return {}

    def get_reactor(self):
        return self.reactor

    def is_shutdown(self):
        return False

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


class FakeHeaterPrinter:
    def __init__(self, virtual_sd, objects):
        self.virtual_sd = virtual_sd
        self.objects = objects

    def get_start_args(self):
        return {}

    def lookup_object(self, name, default=None):
        if name == 'virtual_sdcard':
            return self.virtual_sd
        if name == 'toolhead':
            return self
        return self.objects.get(name, default)

    def get_last_move_time(self):
        return 0.


class HeaterTemperatureWaitingTest(unittest.TestCase):
    def build_fallback_wait(self, actual, target, tolerance=3.,
                            on_pause=None):
        temperature = {'actual': actual, 'target': target}
        heater = FakeLiveHeater(temperature, tolerance)
        reactor = FakeFallbackReactor(on_pause)
        dispatch = FakeTemperatureWaitDispatch()
        virtual_sd = FakeDisabledVirtualSD()
        toolhead = FakeFallbackToolhead()
        printer = FakeFallbackPrinter(
            reactor, dispatch, virtual_sd, toolhead)
        pheaters = heaters.PrinterHeaters.__new__(heaters.PrinterHeaters)
        pheaters.printer = printer
        pheaters.heaters = {'extruder': heater}
        pheaters._get_temp = lambda eventtime: "T:%.1f" % (
            temperature['actual'],)
        return (temperature, heater, reactor, dispatch, toolhead, pheaters)

    def build_follow_target_wait(self, actual=20., target=100., tolerance=3.):
        temperature = {'actual': actual, 'target': target}
        heater = heaters.Heater.__new__(heaters.Heater)
        heater.temperature_wait_tolerance = tolerance
        heater.get_temp = lambda eventtime: (temperature['actual'],
                                             temperature['target'])
        heater.set_temp = lambda value: temperature.__setitem__(
            'target', value)
        receiver = FakeVirtualSDWaitReceiver()
        dispatch = FakeTemperatureWaitDispatch()
        printer = FakeHeaterPrinter(
            receiver, {'gcode': dispatch})
        pheaters = heaters.PrinterHeaters.__new__(heaters.PrinterHeaters)
        pheaters.printer = printer
        pheaters.heaters = {'extruder': heater}

        pheaters.cmd_TEMPERATURE_WAIT(FakeWaitGCode(
            SENSOR='extruder', FOLLOW_TARGET=1))
        sensor, check_ready, get_target = receiver.wait
        return (temperature, dispatch, sensor, check_ready, get_target)

    def test_follow_target_registers_dynamic_heater_wait(self):
        temperature, dispatch, sensor, check_ready, get_target = \
            self.build_follow_target_wait()
        self.assertEqual('extruder', sensor)
        self.assertFalse(check_ready(0.))
        temperature['target'] = 50.
        self.assertEqual(50., get_target(0.))
        temperature['actual'] = 46.99
        self.assertFalse(check_ready(0.))
        temperature['actual'] = 47.
        self.assertTrue(check_ready(0.))

    def test_follow_target_waits_in_cooling_direction(self):
        temperature, dispatch, sensor, check_ready, get_target = \
            self.build_follow_target_wait(actual=150., target=100.)

        self.assertFalse(check_ready(0.))
        temperature['actual'] = 103.01
        self.assertFalse(check_ready(0.))
        temperature['actual'] = 103.
        self.assertTrue(check_ready(0.))

        # Crossing the whole tolerance band between polls still counts as
        # reaching the target in the selected cooling direction.
        temperature, dispatch, sensor, check_ready, get_target = \
            self.build_follow_target_wait(actual=150., target=100.)
        self.assertFalse(check_ready(0.))
        temperature['actual'] = 90.
        self.assertTrue(check_ready(0.))

    def test_follow_target_reselects_direction_on_each_target_change(self):
        temperature, dispatch, sensor, check_ready, get_target = \
            self.build_follow_target_wait(actual=20., target=100.)

        self.assertFalse(check_ready(0.))
        temperature.update(actual=110., target=80.)
        self.assertFalse(check_ready(0.))
        temperature.update(actual=100., target=120.)
        self.assertFalse(check_ready(0.))
        temperature['actual'] = 117.
        self.assertTrue(check_ready(0.))
        temperature.update(actual=120., target=110.)
        self.assertFalse(check_ready(0.))
        temperature['actual'] = 113.
        self.assertTrue(check_ready(0.))

    def test_follow_target_rechecks_drift_after_ready_sample(self):
        temperature, dispatch, sensor, check_ready, get_target = \
            self.build_follow_target_wait(actual=100., target=100.)

        self.assertTrue(check_ready(0.))
        temperature['actual'] = 110.
        self.assertFalse(check_ready(0.))
        temperature['actual'] = 103.
        self.assertTrue(check_ready(0.))

    def test_follow_target_uses_each_heater_tolerance(self):
        temperature, dispatch, sensor, check_ready, get_target = \
            self.build_follow_target_wait(
                actual=94.99, target=100., tolerance=5.)

        self.assertFalse(check_ready(0.))
        temperature['actual'] = 95.
        self.assertTrue(check_ready(0.))

    def test_zero_target_finishes_wait_without_cancelling(self):
        temperature, dispatch, sensor, check_ready, get_target = \
            self.build_follow_target_wait()
        temperature['target'] = 0.

        self.assertTrue(check_ready(0.))
        self.assertEqual(0, dispatch.notify_count)

    def test_native_temperature_wait_fallback_uses_cooling_tolerance(self):
        state = {}

        def on_pause(count):
            state['temperature']['actual'] = 90.

        result = self.build_fallback_wait(
            actual=150., target=100., on_pause=on_pause)
        temperature, heater, reactor, dispatch, toolhead, pheaters = result
        state['temperature'] = temperature

        pheaters._wait_for_temperature(heater)

        self.assertEqual(1, reactor.pause_count)
        self.assertEqual(90., temperature['actual'])
        self.assertEqual(1, len(dispatch.responses))

    def test_follow_target_fallback_uses_heating_tolerance(self):
        state = {}

        def on_pause(count):
            state['temperature']['actual'] = 97.

        result = self.build_fallback_wait(
            actual=20., target=100., on_pause=on_pause)
        temperature, heater, reactor, dispatch, toolhead, pheaters = result
        state['temperature'] = temperature
        gcmd = FakeWaitGCode(SENSOR='extruder', FOLLOW_TARGET=1)

        pheaters.cmd_TEMPERATURE_WAIT(gcmd)

        self.assertEqual(1, reactor.pause_count)
        self.assertEqual(97., temperature['actual'])
        self.assertEqual(1, len(gcmd.responses))

    def test_zero_target_fallback_continues_without_pause(self):
        result = self.build_fallback_wait(actual=200., target=0.)
        temperature, heater, reactor, dispatch, toolhead, pheaters = result
        gcmd = FakeWaitGCode(SENSOR='extruder', FOLLOW_TARGET=1)

        pheaters.cmd_TEMPERATURE_WAIT(gcmd)

        self.assertEqual(0, reactor.pause_count)
        self.assertEqual([], gcmd.responses)
        self.assertEqual(0., temperature['target'])

    def test_follow_target_rejects_non_heater_sensor(self):
        class FakeSensor:
            def get_temp(self, eventtime):
                return 20., 0.

        receiver = FakeVirtualSDWaitReceiver()
        sensor = FakeSensor()
        printer = FakeHeaterPrinter(receiver, {'temperature_sensor room':
                                                sensor})
        pheaters = heaters.PrinterHeaters.__new__(heaters.PrinterHeaters)
        pheaters.printer = printer
        pheaters.heaters = {}

        with self.assertRaises(FakeCommandError):
            pheaters.cmd_TEMPERATURE_WAIT(FakeWaitGCode(
                SENSOR='temperature_sensor room', FOLLOW_TARGET=1))


if __name__ == '__main__':
    unittest.main()
