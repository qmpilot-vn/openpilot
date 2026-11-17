#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta
  rm -f /data/scons_cache/config.lock

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # /VERSION differs. The OS updater stays on "waiting for internet" until a
  # 2s request to openpilot.comma.ai succeeds. The images are on
  # commadist.azureedge.net, so flash once that host answers. The updater
  # screen is only for picking Wi-Fi, and it is stopped before a flash.
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    UPDATER="/usr/comma/updater"
    if [ ! -x "$UPDATER" ]; then
      UPDATER="$DIR/system/hardware/tici/updater"
    fi
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi

    sudo systemctl start systemd-timesyncd 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
      [ "$(date +%Y)" -ge 2025 ] && break
      sleep 1
    done

    agnos_images_reachable() {
      MANIFEST="$MANIFEST" python3 - << 'PY'
import json, os, sys, urllib.request
url = json.load(open(os.environ["MANIFEST"]))[0]["url"]
req = urllib.request.Request(url, headers={"Range": "bytes=0-0"})
try:
  with urllib.request.urlopen(req, timeout=20) as resp:
    resp.read(1)
except Exception:
  sys.exit(1)
PY
    }

    UPDATER_PID=""
    stop_updater() {
      if [ -n "$UPDATER_PID" ] && kill -0 "$UPDATER_PID" 2>/dev/null; then
        kill "$UPDATER_PID" 2>/dev/null || true
        wait "$UPDATER_PID" 2>/dev/null || true
      fi
      UPDATER_PID=""
    }

    while true; do
      if pgrep -f "agnos.py --swap" >/dev/null; then
        sleep 5
        continue
      fi
      if $AGNOS_PY --verify $MANIFEST; then
        stop_updater
        sudo reboot
      fi
      if agnos_images_reachable; then
        echo "AGNOS image host reachable, flashing inactive slot" >&2
        stop_updater
        if $AGNOS_PY --swap $MANIFEST; then
          sudo reboot
        fi
        sleep 10
        continue
      fi
      if [ -z "$UPDATER_PID" ] || ! kill -0 "$UPDATER_PID" 2>/dev/null; then
        echo "AGNOS image host unreachable, opening Wi-Fi screen" >&2
        setsid "$UPDATER" "$AGNOS_PY" "$MANIFEST" >/tmp/agnos_updater.log 2>&1 &
        UPDATER_PID=$!
      fi
      sleep 5
    done
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD"

  # submodule package symlink so `import jetlink` works on device
  ln -sfn jetlink_repo/jetlink jetlink

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # start manager
  cd system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
