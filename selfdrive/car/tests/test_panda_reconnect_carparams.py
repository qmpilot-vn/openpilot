from openpilot.selfdrive.car.card import PANDA_RECONNECT_REPUBLISH_S, panda_reconnect_window


def test_baseline_stamp_does_not_republish():
  prev, until, publish = panda_reconnect_window(10.0, 10.0, now=100.0, until=0.0)
  assert prev == 10.0
  assert until == 0.0
  assert not publish


def test_stamp_change_opens_a_republish_window():
  prev, until, publish = panda_reconnect_window(10.0, 20.0, now=100.0, until=0.0)
  assert prev == 20.0
  assert until == 100.0 + PANDA_RECONNECT_REPUBLISH_S
  assert publish


def test_stamp_appearing_opens_a_republish_window():
  prev, until, publish = panda_reconnect_window(None, 20.0, now=100.0, until=0.0)
  assert prev == 20.0
  assert publish
  assert until == 100.0 + PANDA_RECONNECT_REPUBLISH_S


def test_window_stays_open_and_then_closes():
  _, _, still = panda_reconnect_window(20.0, 20.0, now=109.0, until=110.0)
  _, _, closed = panda_reconnect_window(20.0, 20.0, now=110.0, until=110.0)
  assert still
  assert not closed


def test_stamp_disappearing_does_not_open_a_window():
  prev, until, publish = panda_reconnect_window(10.0, None, now=100.0, until=0.0)
  assert prev is None
  assert until == 0.0
  assert not publish
