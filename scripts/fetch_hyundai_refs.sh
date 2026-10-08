#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
repos=(hdr_client_driver hdr_description hdr_ros2_driver hdr_simulation_gz)
paths=()
fail() { echo "ERROR: $*" >&2; exit 1; }
[[ $(git rev-parse --show-toplevel) == "$ROOT" ]] || fail "Run from a Git checkout of AHEAD."

# Check all inputs before updating any repository. Never discard local work.
for repo in "${repos[@]}"; do
  path="external/hyundai_robotics/$repo"
  paths+=("$path")
  [[ $(git ls-files -s -- "$path") == 160000\ * ]] || fail "$path is not a tracked submodule."
  link="ros2_ws/src/$repo"
  [[ -L "$link" && $(readlink "$link") == "../../$path" ]] || fail "$link must be the tracked relative symlink to $path. Restore it with git restore -- $link."
  if [[ -e "$path/.git" ]]; then
    [[ $(git -C "$path" rev-parse --show-toplevel) == "$ROOT/$path" ]] || fail "Invalid repository at $path."
    [[ -z $(git -C "$path" status --porcelain --untracked-files=all) ]] || fail "Local changes in $path. Commit or back up your work before retrying."
    git -C "$path" submodule foreach --recursive 'test -z "$(git status --porcelain --untracked-files=all)"' || fail "Local changes in nested submodules of $path."
  elif [[ -d "$path" && -n $(ls -A "$path") ]]; then
    fail "$path is nonempty and not a Git repository. Move it aside before retrying."
  fi
done

# Explicit checkout overrides a local update=merge/rebase setting. No --remote,
# branch checkout, reset, clean, or force: the superproject index pins the SHAs.
git submodule sync --recursive -- "${paths[@]}"
git submodule update --init --recursive --checkout -- "${paths[@]}"
for repo in "${repos[@]}"; do
  path="external/hyundai_robotics/$repo"
  expected=$(git ls-files -s -- "$path" | awk '{print $2}')
  [[ $(git -C "$path" rev-parse HEAD) == "$expected" ]] || fail "Commit mismatch in $path."
  [[ -d "ros2_ws/src/$repo" ]] || fail "Broken ROS 2 symlink for $repo."
done
if git submodule status --recursive -- "${paths[@]}" | grep -q '^[+U-]'; then
  fail "A submodule does not match its recorded commit."
fi
echo "Hyundai references initialized at recorded commits; ROS 2 symlinks verified."
