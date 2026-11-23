import threading
import time
from unittest import mock

from openpilot.system.hardware.base import ThermalConfig, ThermalZone
from openpilot.system.hardware import sensor_sampler
from openpilot.system.hardware.sensor_sampler import SensorSampler, thermal_readers, thermal_temps


class StuckReader:
  def __init__(self, value):
    self.value = value
    self.stuck = False
    self.release = threading.Event()

  def __call__(self):
    if self.stuck:
      self.release.wait(10)
    return self.value


class TestSensorSampler:
  def test_stuck_read_does_not_block_get(self, monkeypatch):
    monkeypatch.setattr(sensor_sampler, "DT_HW", 0.01)
    reader = StuckReader(40.0)
    sensors = SensorSampler({"cpu0": reader})
    end_event = threading.Event()
    thread = threading.Thread(target=sensors.run, args=(end_event,), daemon=True)
    thread.start()
    try:
      time.sleep(0.05)
      reader.stuck = True
      time.sleep(0.05)
      reader.value = 90.0

      t = time.monotonic()
      values, age = sensors.get()
      assert time.monotonic() - t < 0.01
      assert values == {"cpu0": 40.0}

      time.sleep(0.2)
      _, later_age = sensors.get()
      assert later_age > age + 0.15

      reader.stuck = False
      reader.release.set()
      time.sleep(0.05)
      values, age = sensors.get()
      assert values == {"cpu0": 90.0}
      assert age < 0.05
    finally:
      reader.release.set()
      end_event.set()
      thread.join(1)

  def test_slow_read_logs_the_reader(self, monkeypatch):
    event = mock.Mock()
    monkeypatch.setattr(sensor_sampler.cloudlog, "event", event)
    monkeypatch.setattr(sensor_sampler, "SLOW_READ_S", 0.01)
    SensorSampler({"fast": lambda: 1, "slow": lambda: time.sleep(0.03) or 2})
    event.assert_called_once()
    assert list(event.call_args.kwargs["reads"]) == ["slow"]

  def test_failed_read_keeps_last_values(self, monkeypatch):
    monkeypatch.setattr(sensor_sampler, "DT_HW", 0.01)
    exception = mock.Mock()
    monkeypatch.setattr(sensor_sampler.cloudlog, "exception", exception)
    calls = []

    def flaky():
      calls.append(1)
      if len(calls) > 1:
        raise OSError("i2c timeout")
      return 50.0

    sensors = SensorSampler({"intake": flaky})
    end_event = threading.Event()
    thread = threading.Thread(target=sensors.run, args=(end_event,), daemon=True)
    thread.start()
    time.sleep(0.05)
    end_event.set()
    thread.join(1)

    values, age = sensors.get()
    assert values == {"intake": 50.0}
    assert age >= 0.04
    assert exception.called


def test_thermal_temps_match_get_msg_layout():
  config = ThermalConfig(cpu=[ThermalZone("cpu0"), ThermalZone("cpu1")], gpu=[ThermalZone("gpu0")],
                         memory=ThermalZone("ddr"), pmic=[ThermalZone("pm8998_tz")], intake=ThermalZone("intake"))
  readers = thermal_readers(config)
  assert set(readers) == {"cpu0", "cpu1", "gpu0", "ddr", "pm8998_tz", "intake"}

  values = {"cpu0": 60.0, "cpu1": 61.0, "gpu0": 55.0, "ddr": 50.0, "pm8998_tz": 45.0, "intake": 30.0}
  assert thermal_temps(config, values) == {
    "cpuTempC": [60.0, 61.0],
    "gpuTempC": [55.0],
    "memoryTempC": 50.0,
    "pmicTempC": [45.0],
    "intakeTempC": 30.0,
  }
