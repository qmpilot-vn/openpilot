#!/usr/bin/env bash
# In-tree manual build of system/loggerd/loggerd using this tree's own
# compile_commands.json flags (larch64 / -DQCOM2 -D__TICI__ -mcpu=cortex-a57).
set -euo pipefail
cd /data/openpilot

echo "== compiling =="
python3 - <<'PY'
import json, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

SRCS = [
  "system/loggerd/loggerd.cc",
  "system/loggerd/logger.cc",
  "system/loggerd/video_writer.cc",
  "system/loggerd/zstd_writer.cc",
  "common/params.cc",
  "common/swaglog.cc",
  "common/util.cc",
  "msgq_repo/msgq/ipc.cc",
  "msgq_repo/msgq/event.cc",
  "msgq_repo/msgq/impl_msgq.cc",
  "msgq_repo/msgq/impl_fake.cc",
  "msgq_repo/msgq/msgq.cc",
  "cereal/gen/cpp/log.capnp.c++",
  "cereal/gen/cpp/car.capnp.c++",
  "cereal/gen/cpp/deprecated.capnp.c++",
  "cereal/gen/cpp/custom.capnp.c++",
  "third_party/json11/json11.cpp",
]

db = json.load(open("compile_commands.json"))
cmds = {os.path.relpath(e["file"], "/data/openpilot"): (e.get("command") or " ".join(e.get("arguments", []))) for e in db}

missing = [s for s in SRCS if s not in cmds]
if missing:
    sys.exit("no compile command for: %s" % missing)

def obj_of(src):
    return os.path.splitext(src)[0] + ".o"

todo = []
for s in SRCS:
    o = obj_of(s)
    if os.path.exists(o) and os.path.getmtime(o) > os.path.getmtime(s):
        print("up to date: %s" % o, flush=True)
    else:
        todo.append(s)

def build(src):
    print("CC %s" % src, flush=True)
    r = subprocess.run(cmds[src], shell=True, capture_output=True, text=True)
    return src, r.returncode, r.stdout + r.stderr

fail = False
with ThreadPoolExecutor(max_workers=os.cpu_count()) as ex:
    for src, rc, out in ex.map(build, todo):
        if rc != 0:
            fail = True
            print("FAILED %s\n%s" % (src, out), flush=True)
        elif out.strip():
            print("warnings %s:\n%s" % (src, out[:2000]), flush=True)
print("compile done", flush=True)
sys.exit(1 if fail else 0)
PY

OBJS=(
  system/loggerd/loggerd.o system/loggerd/logger.o system/loggerd/video_writer.o system/loggerd/zstd_writer.o
  common/params.o common/swaglog.o common/util.o
  msgq_repo/msgq/ipc.os msgq_repo/msgq/event.os msgq_repo/msgq/impl_msgq.os msgq_repo/msgq/impl_fake.os msgq_repo/msgq/msgq.os
  cereal/gen/cpp/log.capnp.o cereal/gen/cpp/car.capnp.o cereal/gen/cpp/deprecated.capnp.o cereal/gen/cpp/custom.capnp.o
  third_party/json11/json11.o
)
for o in "${OBJS[@]}"; do [ -f "$o" ] || { echo "MISSING OBJ $o"; exit 1; }; done

VENV=/usr/local/venv/lib/python3.12/site-packages
echo "== linking =="
clang++ -o /tmp/loggerd.new "${OBJS[@]}" \
  -L"$VENV/capnproto/install/lib" \
  -L"$VENV/zeromq/install/lib" \
  -L"$VENV/zstd/install/lib" \
  -L"$VENV/bzip2/install/lib" \
  -L"$VENV/ffmpeg/install/lib" \
  -L"$VENV/libyuv/install/lib" \
  -lcapnp -lkj -lzmq -lzstd -lbz2 \
  -lavformat -lavcodec -lavutil -lswresample -lx264 -lz \
  -lyuv -lpthread -ldl -lm

echo "== result =="
ls -la /tmp/loggerd.new
file /tmp/loggerd.new
