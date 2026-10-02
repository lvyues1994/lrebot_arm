#!/usr/bin/env bash
# Download the pinned official MuJoCo release into .deps/ (ignored by git).
set -euo pipefail

version=3.8.0
sha256=2be88c6f92a06c3eaffdb47d3a6d3fbf159fbc057e9d272d592fb194e41fefab
archive="mujoco-${version}-linux-x86_64.tar.gz"
url="https://github.com/google-deepmind/mujoco/releases/download/${version}/${archive}"

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
deps="${root}/.deps"
mkdir -p "${deps}"

if [[ -f "${deps}/mujoco-${version}/lib/libmujoco.so" ]]; then
    echo "MuJoCo ${version} already present in ${deps}/mujoco-${version}"
    exit 0
fi

curl -fsSL -o "${deps}/${archive}" "${url}"
echo "${sha256}  ${deps}/${archive}" | sha256sum --check --quiet
tar -xzf "${deps}/${archive}" -C "${deps}"
echo "MuJoCo ${version} installed in ${deps}/mujoco-${version}"
