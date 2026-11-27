#!/usr/bin/env bash
# Fill src/ with the official WRS 2025 simulation sources at the exact versions this
# workspace was built on, then add this repository's changes on top:
#   1. download each upstream repository at a pinned commit
#   2. apply the patches in patches/ (changes to existing upstream files)
#   3. copy overlay/ into src/ (new files: team Prasid robots, controllers, projects)
#   4. check that hairo-world-plugin is in place (it is not public; see README)
#
# Safe to run again: repositories that are already there are left alone and
# patches that are already applied are skipped.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$ROOT/src"

# name | path under src/ | repository | commit
SOURCES=(
  "Choreonoid|choreonoid|https://github.com/choreonoid/choreonoid.git|c272f4d942f0cf90d4d977fb81e3286ceda72e8d"
  "choreonoid_ros|choreonoid_ros|https://github.com/choreonoid/choreonoid_ros.git|34558e71af10240afa80e921a6d9db1858c84a4d"
  "choreonoid_joy2|choreonoid_joy2|https://github.com/wrs-sim/choreonoid_joy2.git|a3925ce2c32d882d062cc631522fcc68aa1e7bb0"
  "WRS-2025|choreonoid/ext/WRS2025|https://github.com/wrs-frei-simulation/WRS-2025.git|1f5433896a282448305892a7a369e166d75bb8e4"
  "WRS-Pre-2024|choreonoid/ext/WRS2024PRE|https://github.com/wrs-frei-simulation/WRS-Pre-2024.git|e953c13cced7f20f23e66bdff3f21d25294195df"
)

# path under src/ | patch file
PATCHES=(
  "choreonoid|choreonoid.patch"
  "choreonoid_ros|choreonoid_ros.patch"
  "choreonoid/ext/WRS2025|WRS2025.patch"
)

fetch_pinned() {
  local name=$1 dir=$2 url=$3 sha=$4
  if [ -d "$dir/.git" ]; then
    echo "  $name: already present, leaving it as it is"
    return
  fi
  echo "  $name: downloading ${sha:0:7}"
  mkdir -p "$dir"
  git -C "$dir" init -q
  git -C "$dir" remote add origin "$url"
  git -C "$dir" fetch -q --depth 1 origin "$sha"
  git -C "$dir" checkout -q FETCH_HEAD
}

echo "1/4 Official sources"
for entry in "${SOURCES[@]}"; do
  IFS='|' read -r name path url sha <<< "$entry"
  fetch_pinned "$name" "$SRC/$path" "$url" "$sha"
done

echo "2/4 Patches"
for entry in "${PATCHES[@]}"; do
  IFS='|' read -r path patch <<< "$entry"
  if git -C "$SRC/$path" apply --check "$ROOT/patches/$patch" 2> /dev/null; then
    git -C "$SRC/$path" apply --whitespace=nowarn "$ROOT/patches/$patch"
    echo "  $patch: applied"
  elif git -C "$SRC/$path" apply --check -R "$ROOT/patches/$patch" 2> /dev/null; then
    echo "  $patch: already applied"
  else
    echo "  $patch: does not apply to src/$path. Is that folder at the pinned commit?" >&2
    exit 1
  fi
done

echo "3/4 New files"
cp -a "$ROOT/overlay/." "$SRC/"
echo "  copied $(find "$ROOT/overlay" -type f | wc -l) files from overlay/"

echo "4/4 hairo-world-plugin"
if [ -f "$SRC/choreonoid/ext/hairo-world-plugin/CMakeLists.txt" ]; then
  echo "  found"
else
  echo "  MISSING: put the hairo-world-plugin folder from the organisers at"
  echo "  src/choreonoid/ext/hairo-world-plugin before you build (see README)."
fi

echo "Done. Next: set up the Python environment and build (README, steps 3 and 4)."
