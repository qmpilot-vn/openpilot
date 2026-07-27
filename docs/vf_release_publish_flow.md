# VinFast release publish flow

How code moves from a private dev branch to a public Custom Software install URL.

## Lanes

| Lane | Repo | Branch | What lives here |
|------|------|--------|-----------------|
| Dev | `ronnguyen185/qmpilot` (private) | `vf6`, `vf8-alpha-c4`, … | Full `.py`, full `.dbc`, real `vinfast*.h`, tests |
| Release prep | same private repo | `vf-release-c4` | Packaged tip only (cache / `.so` or bytecode / stub / no tests) |
| Public ship | `qmpilot-vn/openpilot` (public) | `vf-release` (or `release`) | Mirror of the packaged tip — what users install |

```text
vf6 (dev, plaintext)
   │  merge / cherry-pick what you want to ship
   ▼
vf-release-c4 (private)  ← run package scripts here
   │  git push install HEAD:vf-release
   ▼
qmpilot-vn/openpilot  branch vf-release  (public)
   │
   ▼
user types:  qmpilot-vn/vf-release
```

**Important:** Custom Software does **not** clone `qmpilot-vn/release`.  
`installer.comma.ai/qmpilot-vn/<branch>` always clones:

`https://github.com/qmpilot-vn/openpilot.git` @ branch `<branch>`

So:

| User types | Installs |
|------------|----------|
| `qmpilot-vn/vf-release` | `qmpilot-vn/openpilot` @ `vf-release` |
| `qmpilot-vn/release` | `qmpilot-vn/openpilot` @ `release` |

---

## One-time remotes

```bash
# private work
git remote -v
# origin  → https://github.com/ronnguyen185/qmpilot.git

# public install target (add once)
git remote add install https://github.com/qmpilot-vn/openpilot.git
```

Do **not** push packaged releases only to `qmpilot-vn/release` if you want Custom Software — that repo name is not what the comma installer uses.

---

## Publish checklist (each release)

### 1) Finish development

Work on the private FW branch, e.g. `vf6`:

```bash
git checkout vf6
git pull origin vf6
# ... develop / test on car ...
git push origin vf6
```

### 2) Bring changes into release-prep

```bash
git fetch origin
git checkout vf-release-c4
git merge origin/vf6
# or: git cherry-pick <commits>
```

Merging may bring plaintext VinFast sources back. That is expected. Packaging is the next step.

### 3) Package (on C4 or matching Python ABI)

Use the release packaging path (bytecode/Cython + DBC cache + safety stub), for example:

```bash
./setup_vinfast.sh
# Prefer used-only DBC → *.dbc.cache, then strip plaintext *.dbc
# Keep safety stub in tree; real vinfast*.h only on flash machines
# Remove tests / examples / dump scripts from the tip you will publish
```

Target runtime: **C4 Python 3.12.x aarch64** for `.so` / `.pyc`.

Recommended release contents:

- VinFast car: shims + `*_impl.*.pyc` **or** stripped Cython `.so` (no plaintext logic `.py`)
- DBC: **used-only** messages/signals as `*.dbc.cache` (no full OEM `.dbc`, no `body_can` if unused)
- Safety: `vinfast_stub.h` in public tree; real headers gitignored / private
- No: `panda/examples/vinfast_*`, VinFast tests, docs dumps, `dump_vinfast_dbc_cache.py` on tip

### 4) Verify footprint

```bash
bash scripts/check_vinfast_footprint.sh
# Fail publish if plaintext vinfast_*.dbc, real vinfast.h, or source .py logic remain
```

### 5) Commit packaged tip (private)

```bash
git status
git add -A
git commit -m "Ship packaged VinFast build for public vf-release."
git push origin vf-release-c4
```

### 6) Mirror to public (one-way)

```bash
# Publish the packaged tip as the public branch users will type
git push install HEAD:vf-release

# Optional short name:
# git push install HEAD:release
```

### 7) User install

On device (Custom Software):

```text
qmpilot-vn/vf-release
```

Same as:

```text
https://installer.comma.ai/qmpilot-vn/vf-release
```

---

## Rules

1. **One-way only:** `vf-release-c4` → public `vf-release`. Do not develop on the public branch.
2. **Never push `vf6` to `install`.** That leaks full DBC / sources / headers.
3. **Package is the gate.** The tip you push must already be stripped.
4. **Same SHA is fine.** Public can point at the same commit as private `vf-release-c4`; secrecy is tree contents, not a separate history.
5. **Branch names:** no `/` in the public branch name (installer limitation).
6. **Panda firmware:** build/flash with real safety headers on a private machine; do not commit `main.elf` / real headers to the public tip.

---

## Optional: short branded URL

Not required. Comma already hosts the installer.

If you want `https://release.qmpilot.vn`:

1. Publish GitHub branch as above.
2. Cloudflare Worker (or R2) serving the ELF from  
   `https://installer.comma.ai/qmpilot-vn/vf-release`  
   with `Content-Type: application/octet-stream`.
3. Users type `https://release.qmpilot.vn`.

---

## Quick reference

```text
Dev:     origin/vf6
Prep:    origin/vf-release-c4     (packaged)
Public:  install/vf-release       (qmpilot-vn/openpilot)
URL:     qmpilot-vn/vf-release
```
