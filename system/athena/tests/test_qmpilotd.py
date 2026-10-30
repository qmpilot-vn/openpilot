import openpilot.system.athena.qmpilotd as qmpilotd


class Resp:
  def __init__(self, status_code):
    self.status_code = status_code


def test_stats_body_uses_athena_getmessage():
  device_event = {
    "deviceState": {
      "started": True,
      "networkType": "wifi",
      "freeSpacePercent": 41.23,
      "maxTempC": 46.54,
    }
  }
  panda_event = {"pandaStates": [{"voltage": 12640}]}
  assert qmpilotd.build_stats(device_event, panda_event) == {
    "voltage_mv": 12640,
    "started": True,
    "network_type": "wifi",
    "free_space_percent": 41.2,
    "max_temp_c": 46.5,
  }
  assert "voltage_mv" not in qmpilotd.build_stats(device_event, {"pandaStates": []})


def test_read_stats_waits_until_both_messages_arrive():
  class SM:
    def __init__(self):
      self.seen = {"deviceState": False, "pandaStates": True}
      self.updated = False

    def update(self, timeout):
      self.updated = timeout == 1000

    def __getitem__(self, key):
      raise AssertionError(key)

  sm = SM()
  assert qmpilotd.read_stats(sm) is None
  assert sm.updated


def test_stats_post_uses_device_key(monkeypatch):
  posts = []

  def post(url, **kwargs):
    posts.append((url, kwargs))
    return Resp(200)

  monkeypatch.setattr(qmpilotd.requests, "post", post)
  resp = qmpilotd.post_stats(
    "https://qmpilot-connect.com",
    "test-key",
    "0123456789abcdef",
    {"voltage_mv": 12640, "started": True, "network_type": "wifi", "free_space_percent": 41.2, "max_temp_c": 46.5},
  )
  assert resp.status_code == 200
  assert posts[0][0] == "https://qmpilot-connect.com/v1/0123456789abcdef/stats"
  assert posts[0][1]["headers"]["X-Device-Key"] == "test-key"
  assert posts[0][1]["json"]["voltage_mv"] == 12640
