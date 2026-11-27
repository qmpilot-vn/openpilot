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

  # Flash only an older AGNOS. A newer release stays on the device.
  device_ver=$(tr -d '[:space:]' < /VERSION)
  oldest=$(printf '%s\n%s\n' "$device_ver" "$AGNOS_VERSION" | sort -V | head -n 1)
  if [ "$device_ver" != "$AGNOS_VERSION" ] && [ "$oldest" = "$device_ver" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
  fi
}

# AGNOS 19.9 does not ship these. loggerd, pigeond, athenad, model manager, and qcomgpsd import them.
function ensure_device_python_deps {
  local py="/usr/local/venv/bin/python"
  local pip="/usr/local/venv/bin/pip"
  [ -x "$py" ] && [ -x "$pip" ] || return 0
  if "$py" - <<'PY'
import importlib
for name in ("xattr", "serial", "jsonrpc", "aiohttp", "crcmod"):
    importlib.import_module(name)
PY
  then
    return 0
  fi
  echo "installing device python packages"
  timeout 90 "$pip" install --disable-pip-version-check \
    'pyxattr==0.8.1' 'pyserial==3.5' 'json-rpc==1.15.0' 'aiohttp==3.14.5' 'crcmod==1.7' \
    || echo "python package install failed"
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
    ensure_device_python_deps
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
