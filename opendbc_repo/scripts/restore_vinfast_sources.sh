#!/usr/bin/env bash
# Restore VinFast readable sources (after bytecode/safety packaging) from GitHub Contents API only.
# A Personal Access Token is always required — no local backup directory and no git checkout fallback.
#
# Token (first non-empty wins):
#   VINFAST_RESTORE_GITHUB_TOKEN, GITHUB_TOKEN, or GH_TOKEN
#
# Required:
#   VINFAST_RESTORE_REPO=owner/repo
#
# Optional:
#   VINFAST_RESTORE_REF=main          (branch, tag, or commit SHA)
#   VINFAST_RESTORE_CONTENT_PREFIX=opendbc_repo/opendbc/car/vinfast
#   VINFAST_RESTORE_SAFETY_PREFIX=opendbc_repo/opendbc/safety/modes
#
# Example:
#   export GITHUB_TOKEN=ghp_xxx
#   export VINFAST_RESTORE_REPO=myuser/openpilot
#   export VINFAST_RESTORE_REF=master
#   ./scripts/restore_vinfast_sources.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
VF="${REPO_ROOT}/opendbc_repo/opendbc/car/vinfast"
SAFETY_DIR="${REPO_ROOT}/opendbc_repo/opendbc/safety/modes"

MODULES=(
  carstate vinfastcan carcontroller radar_interface info_radar_interface values fingerprints interface
)
SAFETY_HEADERS=(
  vinfast.h vinfast_common.h vinfast_vf6.h
)

TOKEN="${VINFAST_RESTORE_GITHUB_TOKEN:-${GITHUB_TOKEN:-${GH_TOKEN:-}}}"
REPO="${VINFAST_RESTORE_REPO:?Set VINFAST_RESTORE_REPO=owner/repo}"
REF="${VINFAST_RESTORE_REF:-main}"
PREFIX="${VINFAST_RESTORE_CONTENT_PREFIX:-opendbc_repo/opendbc/car/vinfast}"
SAFETY_PREFIX="${VINFAST_RESTORE_SAFETY_PREFIX:-opendbc_repo/opendbc/safety/modes}"

if [[ -z "${TOKEN}" ]]; then
  echo "Restore requires a GitHub PAT: set VINFAST_RESTORE_GITHUB_TOKEN, GITHUB_TOKEN, or GH_TOKEN."
  exit 1
fi

MODULES_JSON="$(printf '%s\n' "${MODULES[@]}" | python3 -c 'import json,sys; print(json.dumps([l.strip() for l in sys.stdin if l.strip()]))')"
SAFETY_JSON="$(printf '%s\n' "${SAFETY_HEADERS[@]}" | python3 -c 'import json,sys; print(json.dumps([l.strip() for l in sys.stdin if l.strip()]))')"

echo "Downloading car modules from GitHub ${REPO} @ ${REF} (prefix ${PREFIX}/) …"
TOKEN="${TOKEN}" REPO="${REPO}" REF="${REF}" PREFIX="${PREFIX}" OUT_DIR="${VF}" FILES_JSON="${MODULES_JSON}" EXT=".py" python3 <<'PY'
import base64
import json
import os
import ssl
import sys
import urllib.error
import urllib.parse
import urllib.request

token = os.environ["TOKEN"]
owner_repo = os.environ["REPO"].strip().strip("/")
ref = os.environ["REF"]
prefix = os.environ["PREFIX"].strip().strip("/")
out_dir = os.environ["OUT_DIR"]
ext = os.environ["EXT"]
files = json.loads(os.environ["FILES_JSON"])

if "/" not in owner_repo:
  sys.exit("VINFAST_RESTORE_REPO must look like owner/repo")


def api_url(rel_path):
  return (
    "https://api.github.com/repos/"
    + owner_repo
    + "/contents/"
    + rel_path
    + "?ref="
    + urllib.parse.quote(ref)
  )


headers = {
  "Authorization": "Bearer " + token,
  "Accept": "application/vnd.github+json",
  "X-GitHub-Api-Version": "2022-11-28",
  "User-Agent": "restore-vinfast-sources",
}

ctx = ssl.create_default_context()

for m in files:
  rel = f"{prefix}/{m}{ext}"
  url = api_url(rel)
  req = urllib.request.Request(url, headers=headers)
  try:
    with urllib.request.urlopen(req, context=ctx, timeout=120) as resp:
      body = json.load(resp)
  except urllib.error.HTTPError as e:
    sys.stderr.buffer.write(e.read())
    sys.exit(f"GitHub HTTP {e.code} for {rel}")

  if isinstance(body, list):
    sys.exit(f"Expected a file at {rel}, got a directory listing")
  if body.get("encoding") != "base64" or "content" not in body:
    sys.exit(f"Unexpected JSON for {rel}")

  raw = base64.b64decode(body["content"].replace("\n", ""))
  out = os.path.join(out_dir, m + ext)
  with open(out, "wb") as f:
    f.write(raw)
  print("wrote", out)
PY

done

echo "Downloading safety headers from GitHub ${REPO} @ ${REF} (prefix ${SAFETY_PREFIX}/) …"
TOKEN="${TOKEN}" REPO="${REPO}" REF="${REF}" PREFIX="${SAFETY_PREFIX}" OUT_DIR="${SAFETY_DIR}" FILES_JSON="${SAFETY_JSON}" EXT="" python3 <<'PY'
import base64
import json
import os
import ssl
import sys
import urllib.error
import urllib.parse
import urllib.request

token = os.environ["TOKEN"]
owner_repo = os.environ["REPO"].strip().strip("/")
ref = os.environ["REF"]
prefix = os.environ["PREFIX"].strip().strip("/")
out_dir = os.environ["OUT_DIR"]
ext = os.environ["EXT"]
files = json.loads(os.environ["FILES_JSON"])

if "/" not in owner_repo:
  sys.exit("VINFAST_RESTORE_REPO must look like owner/repo")


def api_url(rel_path):
  return (
    "https://api.github.com/repos/"
    + owner_repo
    + "/contents/"
    + rel_path
    + "?ref="
    + urllib.parse.quote(ref)
  )


headers = {
  "Authorization": "Bearer " + token,
  "Accept": "application/vnd.github+json",
  "X-GitHub-Api-Version": "2022-11-28",
  "User-Agent": "restore-vinfast-sources",
}

ctx = ssl.create_default_context()

for m in files:
  rel = f"{prefix}/{m}{ext}"
  url = api_url(rel)
  req = urllib.request.Request(url, headers=headers)
  try:
    with urllib.request.urlopen(req, context=ctx, timeout=120) as resp:
      body = json.load(resp)
  except urllib.error.HTTPError as e:
    sys.stderr.buffer.write(e.read())
    sys.exit(f"GitHub HTTP {e.code} for {rel}")

  if isinstance(body, list):
    sys.exit(f"Expected a file at {rel}, got a directory listing")
  if body.get("encoding") != "base64" or "content" not in body:
    sys.exit(f"Unexpected JSON for {rel}")

  raw = base64.b64decode(body["content"].replace("\n", ""))
  out = os.path.join(out_dir, m + ext)
  with open(out, "wb") as f:
    f.write(raw)
  print("wrote", out)
PY

for m in "${MODULES[@]}"; do
  rm -f "${VF}/__pycache__/${m}."*.pyc 2>/dev/null || true
  rm -f "${VF}/__pycache__/${m}_impl."*.pyc 2>/dev/null || true
done

echo "Restored modules:" "${MODULES[*]}" "and safety:" "${SAFETY_HEADERS[*]}" "(GitHub PAT)"
