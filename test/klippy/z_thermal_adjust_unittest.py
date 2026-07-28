# Unit tests for multi-source Z thermal adjustment
import os
import sys
import unittest


KLIPPER_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                            '..'))
if KLIPPER_ROOT not in sys.path:
    sys.path.insert(0, KLIPPER_ROOT)

from klippy.extras import z_thermal_adjust


class FakeReactor:
    def __init__(self):
        self.now = 0.

    def monotonic(self):
        return self.now


class FakeSensor:
    def __init__(self, temperature):
        self.temperature = temperature

    def get_status(self, eventtime):
        return {'temperature': self.temperature}


class FakePrinter:
    def __init__(self, sensor):
        self.reactor = FakeReactor()
        self.objects = {'fake_sensor': sensor}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name):
        return self.objects[name]

    def config_error(self, message):
        return ValueError(message)


class FakeConfig:
    def __init__(self, printer, name, values):
        self.printer = printer
        self.name = name
        self.values = values

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.name

    def get(self, option, default=None):
        return self.values.get(option, default)

    def getfloat(self, option, default=None, **kwargs):
        value = self.values.get(option, default)
        if value is None:
            return None
        value = float(value)
        if 'minval' in kwargs and value < kwargs['minval']:
            raise ValueError(option)
        if 'maxval' in kwargs and value > kwargs['maxval']:
            raise ValueError(option)
        if 'above' in kwargs and value <= kwargs['above']:
            raise ValueError(option)
        return value


class FakeContributionSource:
    def __init__(self, contribution):
        self.contribution = contribution

    def get_contribution(self):
        return self.contribution


class ZThermalSourceTest(unittest.TestCase):
    def make_source(self, values, temperature=60.):
        sensor = FakeSensor(temperature)
        printer = FakePrinter(sensor)
        values = dict(values)
        values['sensor'] = 'fake_sensor'
        config = FakeConfig(
            printer, 'z_thermal_adjust test_source', values)
        source = z_thermal_adjust.ZThermalSource(config, 2.)
        source.handle_connect()
        return printer, sensor, source

    def test_fixed_reference_contribution_and_smoothing(self):
        printer, sensor, source = self.make_source({
            'temp_coeff': -0.004,
            'reference_temperature': 15.,
            'smooth_time': 2.,
        })
        self.assertTrue(source.update_temperature(0.))
        self.assertAlmostEqual(source.get_contribution(), 0.18)

        sensor.temperature = 80.
        self.assertTrue(source.update_temperature(1.))
        self.assertAlmostEqual(source.smoothed_temp, 70.)
        self.assertAlmostEqual(source.get_contribution(), 0.22)

        source.set_ref_temperature(20.)
        self.assertAlmostEqual(source.get_contribution(), 0.20)
        source.handle_z_homing()
        self.assertEqual(source.ref_temperature, 15.)
        self.assertFalse(source.ref_temp_override)

    def test_homing_reference(self):
        printer, sensor, source = self.make_source({
            'temp_coeff': -0.002,
        }, temperature=45.)
        source.update_temperature(0.)
        self.assertIsNone(source.get_contribution())
        source.handle_z_homing()
        self.assertEqual(source.ref_temperature, 45.)
        self.assertEqual(source.get_contribution(), 0.)

    def test_invalid_temperature_does_not_initialize_source(self):
        printer, sensor, source = self.make_source({
            'temp_coeff': -0.002,
            'reference_temperature': 20.,
        }, temperature=None)
        self.assertFalse(source.update_temperature(0.))
        self.assertIsNone(source.smoothed_temp)
        self.assertIsNone(source.get_contribution())

    def test_invalid_temperature_makes_initialized_source_unavailable(self):
        printer, sensor, source = self.make_source({
            'temp_coeff': -0.002,
            'reference_temperature': 20.,
        }, temperature=40.)
        self.assertTrue(source.update_temperature(0.))
        self.assertIsNotNone(source.get_contribution())
        sensor.temperature = None
        self.assertFalse(source.update_temperature(1.))
        self.assertFalse(source.available)
        self.assertIsNone(source.get_contribution())


class ZThermalModelTest(unittest.TestCase):
    def make_adjuster(self, contributions, limit):
        adjuster = z_thermal_adjust.ZThermalAdjuster.__new__(
            z_thermal_adjust.ZThermalAdjuster)
        adjuster.sources = {
            name: FakeContributionSource(value)
            for name, value in contributions.items()
        }
        adjuster.max_z_adjust_mm = limit
        adjuster.model_ready = False
        adjuster.unclamped_z_adjust_mm = 0.
        adjuster.target_z_adjust_mm = 0.
        adjuster.limit_active = False
        return adjuster

    def test_contributions_are_summed_then_limited(self):
        adjuster = self.make_adjuster({
            'bed': 0.42,
            'chamber': 0.12,
        }, 0.5)
        adjuster._update_multi_model_locked()
        self.assertTrue(adjuster.model_ready)
        self.assertAlmostEqual(adjuster.unclamped_z_adjust_mm, 0.54)
        self.assertAlmostEqual(adjuster.target_z_adjust_mm, 0.5)
        self.assertTrue(adjuster.limit_active)

    def test_opposite_contributions_cancel(self):
        adjuster = self.make_adjuster({
            'bed': 0.30,
            'chamber': -0.10,
        }, 0.5)
        adjuster._update_multi_model_locked()
        self.assertAlmostEqual(adjuster.target_z_adjust_mm, 0.20)
        self.assertFalse(adjuster.limit_active)

    def test_missing_source_value_marks_model_not_ready(self):
        adjuster = self.make_adjuster({
            'bed': 0.30,
            'chamber': None,
        }, 0.5)
        adjuster._update_multi_model_locked()
        self.assertFalse(adjuster.model_ready)


if __name__ == '__main__':
    unittest.main()
