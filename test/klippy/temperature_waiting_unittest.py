# Unit tests for virtual SD non-blocking temperature waits
import io
import os
import sys
import unittest


KLIPPER_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                            '..'))
if KLIPPER_ROOT not in sys.path:
    sys.path.insert(0, KLIPPER_ROOT)

from klippy.extras import heaters, virtual_sdcard


class FakeCommandError(Exception):
    pass


class FakeMutex:
    def test(self):
        return False


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


class FakePrinter:
    def send_event(self, event):
        pass


class FakeWaitGCode:
    error = FakeCommandError

    def __init__(self, **params):
        self.params = params

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


class FakeHeaterPrinter:
    def __init__(self, virtual_sd, objects):
        self.virtual_sd = virtual_sd
        self.objects = objects

    def get_start_args(self):
        return {}

    def lookup_object(self, name, default=None):
        if name == 'virtual_sdcard':
            return self.virtual_sd
        return self.objects.get(name, default)


class FakeReactor:
    NOW = 0.
    NEVER = 999999999.

    def __init__(self):
        self.now = 0.
        self.on_pause = None

    def monotonic(self):
        return self.now

    def pause(self, waketime):
        self.now = max(self.now, waketime)
        if self.on_pause is not None:
            self.on_pause(self.now)
        return self.now

    def unregister_timer(self, timer):
        pass


class NamedStringIO(io.StringIO):
    name = "temperature_waiting.gcode"


class FakeGCode:
    error = FakeCommandError

    def __init__(self, vsd, temperature):
        self.vsd = vsd
        self.temperature = temperature
        self.commands = []
        self.mutex = FakeMutex()

    def get_mutex(self):
        return self.mutex

    def run_script(self, script):
        command = script.strip()
        if not command:
            return
        self.commands.append(command)
        if command.startswith("M109"):
            self.vsd.begin_temperature_wait(
                "extruder",
                lambda eventtime: (self.temperature['target'] <= 0.
                                   or self.temperature['actual']
                                   >= self.temperature['target']),
                lambda eventtime: self.temperature['target'])

    def respond_raw(self, message):
        pass


def build_virtual_sd(gcode_text, temperature):
    vsd = virtual_sdcard.VirtualSD.__new__(virtual_sdcard.VirtualSD)
    reactor = FakeReactor()
    stats = FakePrintStats()
    vsd.current_file = NamedStringIO(gcode_text)
    vsd.file_position = 0
    vsd.file_size = len(gcode_text.encode())
    vsd.cancelable_temperature_wait = True
    vsd.temperature_wait_check_interval = .25
    vsd.temperature_wait = None
    vsd.active_file_command = None
    vsd.print_stats = stats
    vsd.reactor = reactor
    vsd.must_pause_work = False
    vsd.cmd_from_sd = False
    vsd.next_file_position = 0
    vsd.work_timer = object()
    vsd.on_error_gcode = FakeTemplate()
    vsd.gcode = FakeGCode(vsd, temperature)
    vsd.printer = FakePrinter()
    return vsd, reactor, stats


class VirtualSDTemperatureWaitingTest(unittest.TestCase):
    def test_wait_keeps_printing_and_follows_live_target(self):
        temperature = {'actual': 20., 'target': 100.}
        vsd, reactor, stats = build_virtual_sd("M109 S100\nAFTER\n",
                                               temperature)
        wait_statuses = []
        wait_ticks = [0]

        def update_temperature(eventtime):
            if vsd.temperature_wait is None:
                return
            wait_ticks[0] += 1
            if wait_ticks[0] == 1:
                temperature['target'] = 60.
                wait_statuses.append(vsd.get_status(eventtime))
            elif wait_ticks[0] == 2:
                self.assertNotIn("AFTER", vsd.gcode.commands)
                temperature['actual'] = 60.

        reactor.on_pause = update_temperature
        vsd.work_handler(reactor.monotonic())

        self.assertEqual(["M109 S100", "AFTER"], vsd.gcode.commands)
        self.assertEqual("complete", stats.state)
        self.assertEqual(0, stats.pause_count)
        self.assertIsNone(vsd.temperature_wait)
        self.assertTrue(wait_statuses[0]['temperature_waiting'])
        self.assertEqual("extruder",
                         wait_statuses[0]['temperature_wait_sensor'])
        self.assertEqual(60., wait_statuses[0]['temperature_wait_target'])
        self.assertTrue(wait_statuses[0]['is_active'])

    def test_feature_disabled_does_not_register_wait(self):
        temperature = {'actual': 20., 'target': 100.}
        vsd, reactor, stats = build_virtual_sd("", temperature)
        vsd.cancelable_temperature_wait = False
        vsd.cmd_from_sd = True
        vsd.active_file_command = 'M109'
        registered = vsd.begin_temperature_wait(
            "extruder", lambda eventtime: False,
            lambda eventtime: temperature['target'])
        self.assertFalse(registered)
        self.assertIsNone(vsd.temperature_wait)

    def test_real_pause_preserves_wait_until_resume(self):
        temperature = {'actual': 20., 'target': 100.}
        vsd, reactor, stats = build_virtual_sd("M109 S100\nAFTER\n",
                                               temperature)
        pause_requested = [False]

        def request_pause(eventtime):
            if vsd.temperature_wait is not None and not pause_requested[0]:
                pause_requested[0] = True
                vsd.must_pause_work = True

        reactor.on_pause = request_pause
        vsd.work_handler(reactor.monotonic())

        self.assertEqual(["M109 S100"], vsd.gcode.commands)
        self.assertEqual("paused", stats.state)
        self.assertEqual(1, stats.pause_count)
        self.assertIsNotNone(vsd.temperature_wait)

        temperature['actual'] = 100.
        reactor.on_pause = None
        vsd.must_pause_work = False
        vsd.work_timer = object()
        vsd.work_handler(reactor.monotonic())

        self.assertEqual(["M109 S100", "AFTER"], vsd.gcode.commands)
        self.assertEqual("complete", stats.state)
        self.assertIsNone(vsd.temperature_wait)

    def test_reset_clears_wait_state(self):
        temperature = {'actual': 20., 'target': 100.}
        vsd, reactor, stats = build_virtual_sd("", temperature)
        vsd.cmd_from_sd = True
        vsd.active_file_command = 'M190'
        self.assertTrue(vsd.begin_temperature_wait(
            "heater_bed", lambda eventtime: False,
            lambda eventtime: temperature['target']))
        vsd.cmd_from_sd = False
        vsd.work_timer = None
        vsd._reset_file()
        self.assertIsNone(vsd.temperature_wait)
        self.assertIsNone(vsd.current_file)

    def test_nested_macro_wait_is_not_deferred(self):
        temperature = {'actual': 20., 'target': 100.}
        vsd, reactor, stats = build_virtual_sd("", temperature)
        vsd.cmd_from_sd = True
        vsd.active_file_command = 'START_PRINT'
        registered = vsd.begin_temperature_wait(
            "heater_bed", lambda eventtime: False,
            lambda eventtime: temperature['target'])
        self.assertFalse(registered)
        self.assertIsNone(vsd.temperature_wait)

    def test_file_command_parser(self):
        temperature = {'actual': 20., 'target': 100.}
        vsd, reactor, stats = build_virtual_sd("", temperature)
        self.assertEqual('M190', vsd._get_file_command(
            '  N42 M190 S60*123 ; wait for bed'))
        self.assertEqual('TEMPERATURE_WAIT', vsd._get_file_command(
            'temperature_wait SENSOR=extruder FOLLOW_TARGET=1'))
        self.assertEqual('START_PRINT', vsd._get_file_command(
            'START_PRINT BED=60 EXTRUDER=220'))
        self.assertIsNone(vsd._get_file_command(' ; comment only'))


class HeaterTemperatureWaitingTest(unittest.TestCase):
    def test_follow_target_registers_dynamic_heater_wait(self):
        temperature = {'actual': 20., 'target': 100.}
        heater = heaters.Heater.__new__(heaters.Heater)
        heater.get_temp = lambda eventtime: (temperature['actual'],
                                             temperature['target'])
        receiver = FakeVirtualSDWaitReceiver()
        printer = FakeHeaterPrinter(receiver, {})
        pheaters = heaters.PrinterHeaters.__new__(heaters.PrinterHeaters)
        pheaters.printer = printer
        pheaters.heaters = {'extruder': heater}

        pheaters.cmd_TEMPERATURE_WAIT(FakeWaitGCode(
            SENSOR='extruder', FOLLOW_TARGET=1))

        sensor, check_ready, get_target = receiver.wait
        self.assertEqual('extruder', sensor)
        self.assertFalse(check_ready(0.))
        temperature['target'] = 50.
        self.assertEqual(50., get_target(0.))
        temperature['actual'] = 50.
        self.assertTrue(check_ready(0.))

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
