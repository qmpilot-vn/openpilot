#!/usr/bin/env python3
"""Live session to qmpilot-connect. Same job as athenad for comma connect.

Comma Connect reads voltage by calling getMessage("pandaStates") and
getMessage("deviceState") on the athena websocket. qmpilot-connect does
not serve that socket: /ws/v2/ is the website. It takes the same sample
on POST /v1/{dongle_id}/stats. This process only does that post.
"""
from __future__ import annotations

import fcntl
import os
import threading
import time

import requests

import cereal.messaging as messaging
from openpilot.common.api.comma_connect import get_qmpilot_api_host, get_qmpilot_api_key, qmpilot_backend_selected
from openpilot.common.params import Params
from openpilot.common.realtime import set_core_affinity
from openpilot.common.swaglog import cloudlog
from openpilot.system.athena.athenad import backoff

STATS_INTERVAL_S = 30.0
LOCK_PATH = "/data/tmp/qmpilotd.lock"
_NETWORK_NAMES = ("none", "wifi", "cell2G", "cell3G", "cell4G", "cell5G", "ethernet")


def build_stats(device_event: dict, panda_event: dict) -> dict:
  """Fields comma connect reads off getMessage, under the names qmpilot-connect stores."""
  device = device_event["deviceState"]
  pandas = panda_event.get("pandaStates") or []
  body = {
    "started": bool(device["started"]),
    "network_type": str(device["networkType"]),
    "free_space_percent": round(float(device["freeSpacePercent"]), 1),
    "max_temp_c": round(float(device["maxTempC"]), 1),
  }
  if len(pandas) > 0:
    body["voltage_mv"] = int(pandas[0]["voltage"])
  return body


def read_stats(sm) -> dict | None:
  """Same fields as build_stats, from the live deviceState and pandaStates sockets.

  getMessage opens a new socket and raises TimeoutError when the first second
  has no message, which is normal while manager is still starting.
  """
  sm.update(1000)
  if not sm.seen["deviceState"] or not sm.seen["pandaStates"]:
    return None
  device = sm["deviceState"]
  pandas = sm["pandaStates"]
  try:
    network_type = _NETWORK_NAMES[int(device.networkType.raw)]
  except (TypeError, ValueError, IndexError):
    network_type = "none"
  body = {
    "started": bool(device.started),
    "network_type": network_type,
    "free_space_percent": round(float(device.freeSpacePercent), 1),
    "max_temp_c": round(float(device.maxTempC), 1),
  }
  if len(pandas) > 0:
    body["voltage_mv"] = int(pandas[0].voltage)
  return body


def post_stats(api_host: str, api_key: str, dongle_id: str, body: dict, http=None):
  http = http or requests
  return http.post(
    f"{api_host}/v1/{dongle_id}/stats",
    headers={
      "Content-Type": "application/json",
      "X-Device-Key": api_key,
    },
    json=body,
    timeout=10,
  )


def _hold_lock():
  os.makedirs(os.path.dirname(LOCK_PATH), exist_ok=True)
  lock = open(LOCK_PATH, "a+")
  fcntl.flock(lock, fcntl.LOCK_EX)
  return lock


def main(exit_event: threading.Event | None = None) -> None:
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("failed to set core affinity")

  params = Params()
  dongle_id = params.get("DongleId")
  if dongle_id is None:
    cloudlog.info("qmpilotd missing dongle_id")
    raise Exception("qmpilotd can't start without dongle id")

  # One poster. A second process, from a later manager start, waits here.
  _hold_lock()
  sm = messaging.SubMaster(["deviceState", "pandaStates"])

  conn_retries = 0
  while exit_event is None or not exit_event.is_set():
    if not qmpilot_backend_selected():
      cloudlog.info("qmpilotd stopped; backend is comma connect")
      return

    api_host = get_qmpilot_api_host()
    api_key = get_qmpilot_api_key()
    if not api_host or not api_key:
      cloudlog.info("qmpilotd waiting for ApiHost and QmpilotApiKey")
      if exit_event is not None and exit_event.wait(STATS_INTERVAL_S):
        return
      if exit_event is None:
        time.sleep(STATS_INTERVAL_S)
      continue

    wait_s = STATS_INTERVAL_S
    try:
      body = read_stats(sm)
      if body is None:
        wait_s = 1.0
      elif body["network_type"] != "none" and "voltage_mv" in body:
        resp = post_stats(api_host, api_key, dongle_id, body)
        cloudlog.event("qmpilot_stats", status=resp.status_code, **body)
        if resp.status_code >= 400:
          raise Exception(f"qmpilot stats {resp.status_code}")
        conn_retries = 0
    except Exception:
      conn_retries += 1
      cloudlog.exception("qmpilotd.stats")
      wait_s = max(1, backoff(conn_retries))

    if exit_event is not None and exit_event.wait(wait_s):
      return
    if exit_event is None:
      time.sleep(wait_s)


if __name__ == "__main__":
  main()
