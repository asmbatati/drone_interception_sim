#!/usr/bin/env bash
#
# Copy drone_interception_sim's interception-specific assets into the PX4 tree.
#
# This package reuses uav_gz_sim's models (x500_d435, x3_uav) and airframes
# (4020, 4021), which uav_gz_sim/install.sh already copies into PX4. Run THIS
# script only for the *delta* this package adds (custom worlds / airframes).
# It assumes uav_gz_sim/install.sh has already been run.
#
# Required env var: PX4_DIR  (path to the PX4-Autopilot checkout)
set -euo pipefail

if [[ -z "${PX4_DIR:-}" ]]; then
  echo "ERROR: PX4_DIR is not set. export PX4_DIR=/path/to/PX4-Autopilot" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

GZ_MODELS_DIR="${PX4_DIR}/Tools/simulation/gz/models"
GZ_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gz/worlds"
AIRFRAMES_DIR="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes"

if [[ -d "${SCRIPT_DIR}/worlds" ]] && compgen -G "${SCRIPT_DIR}/worlds/*" > /dev/null; then
  echo "Copying worlds -> ${GZ_WORLDS_DIR}"
  mkdir -p "${GZ_WORLDS_DIR}"
  cp -r "${SCRIPT_DIR}/worlds/"* "${GZ_WORLDS_DIR}/"
fi

if [[ -d "${SCRIPT_DIR}/models" ]] && compgen -G "${SCRIPT_DIR}/models/*" > /dev/null; then
  echo "Copying models -> ${GZ_MODELS_DIR}"
  mkdir -p "${GZ_MODELS_DIR}"
  cp -r "${SCRIPT_DIR}/models/"* "${GZ_MODELS_DIR}/"
fi

if compgen -G "${SCRIPT_DIR}/config/px4/[0-9]*" > /dev/null; then
  echo "Copying custom airframes -> ${AIRFRAMES_DIR}"
  mkdir -p "${AIRFRAMES_DIR}"
  cp -r "${SCRIPT_DIR}/config/px4/"[0-9]* "${AIRFRAMES_DIR}/"
fi

echo "drone_interception_sim assets installed. Rebuild PX4 if airframes changed:"
echo "  (cd \"${PX4_DIR}\" && make px4_sitl)"
