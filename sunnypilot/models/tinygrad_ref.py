import os

from openpilot.common.basedir import BASEDIR


def get_tinygrad_ref():
  repo_path = os.path.join(BASEDIR, "tinygrad_repo")
  git_path = os.path.join(repo_path, ".git")

  # tinygrad is vendored here with no .git, so the ref is recorded at vendor time instead.
  # Without this the ref reads as None and test_tinygrad_ref can no longer tell whether the
  # compiled model artifacts match the tinygrad in the tree.
  ref_path = os.path.join(repo_path, "TINYGRAD_REF")
  if not os.path.exists(git_path) and os.path.isfile(ref_path):
    with open(ref_path) as f:
      return f.read().strip()

  try:
    if os.path.isdir(git_path):
      git_dir = git_path
    else:
      with open(git_path) as f:
        line = f.read().strip()
      git_dir = os.path.join(repo_path, line[8:])
    with open(os.path.join(git_dir, "HEAD")) as f:
      ref = f.read().strip()
    if ref.startswith("ref:"):
      with open(os.path.join(git_dir, ref.split(" ", 1)[1])) as f:
        return f.read().strip()
    return ref
  except Exception as e:
    print(f"Error getting tinygrad_repo ref: {e}")
    return None


def main():
  current_ref = get_tinygrad_ref()
  if current_ref:
    print(current_ref)
  else:
    print("")


if __name__ == "__main__":
  main()
