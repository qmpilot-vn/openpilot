import threading
import time
from collections.abc import Callable
from dataclasses import fields
from typing import Any

from openpilot.common.realtime import DT_HW
from openpilot.common.swaglog import cloudlog

SLOW_READ_S = 1.


def thermal_readers(thermal_config) -> dict[str, Callable[[], float]]:
  # one reader per zone, so a slow read is logged with the zone that blocked
  readers = {}
  for f in fields(thermal_config):
    v = getattr(thermal_config, f.name)
    for zone in (v if isinstance(v, list) else [v] if v is not None else []):
      readers[zone.name] = zone.read
  return readers


def thermal_temps(thermal_config, values: dict[str, Any]) -> dict[str, Any]:
  temps = {}
  for f in fields(thermal_config):
    v = getattr(thermal_config, f.name)
    if isinstance(v, list):
      temps[f.name + "TempC"] = [values[zone.name] for zone in v]
    elif v is not None:
      temps[f.name + "TempC"] = values[v.name]
  return temps


class SensorSampler:
  """Reads sysfs sensors on their own thread and keeps the latest values.

  A sensor read can block in the kernel for several seconds, longer than selfdrived
  tolerates a missing deviceState (5s), so the deviceState loop must never wait on one.
  """

  def __init__(self, readers: dict[str, Callable[[], Any]]):
    self._readers = readers
    self._lock = threading.Lock()
    self._values = self._read_all()
    self._ts = time.monotonic()

  def _read_all(self) -> dict[str, Any]:
    values, slow = {}, {}
    for name, read in self._readers.items():
      t = time.monotonic()
      values[name] = read()
      dt = time.monotonic() - t
      if dt > SLOW_READ_S:
        slow[name] = round(dt, 2)
    if slow:
      cloudlog.event("slow sensor read", reads=slow, error=True)
    return values

  def run(self, end_event: threading.Event) -> None:
    while not end_event.is_set():
      try:
        values = self._read_all()
      except Exception:
        cloudlog.exception("sensor read failed")
      else:
        with self._lock:
          self._values, self._ts = values, time.monotonic()
      end_event.wait(DT_HW)

  def get(self) -> tuple[dict[str, Any], float]:
    """Latest values and their age in seconds."""
    with self._lock:
      return self._values, time.monotonic() - self._ts
