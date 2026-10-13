#!/usr/bin/env bash

# Source this file to use the workspace-local PX4 / Gazebo Harmonic toolchain.
PX4_WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_AUTOPILOT_DIR="${PX4_WORKSPACE_DIR}/PX4-Autopilot"
PX4_GZ_ROOT="${PX4_AUTOPILOT_DIR}/.gz-harmonic/root"
PX4_GZ_LIB="${PX4_GZ_ROOT}/usr/lib/x86_64-linux-gnu"

if [[ ! -x "${PX4_AUTOPILOT_DIR}/.venv/bin/python" ]]; then
    echo "PX4 Python virtual environment is missing: ${PX4_AUTOPILOT_DIR}/.venv" >&2
    return 1 2>/dev/null || exit 1
fi

if [[ ! -x "${PX4_GZ_ROOT}/usr/bin/gz" ]]; then
    echo "Workspace-local Gazebo Harmonic is missing: ${PX4_GZ_ROOT}" >&2
    return 1 2>/dev/null || exit 1
fi

export VIRTUAL_ENV="${PX4_AUTOPILOT_DIR}/.venv"
export PATH="${VIRTUAL_ENV}/bin:${PX4_GZ_ROOT}/usr/bin:${PATH}"
unset PYTHONHOME
export PYTHONPATH=""

# Keep build cache writes inside the workspace (the session runtime dir may be read-only).
export CCACHE_DIR="${PX4_AUTOPILOT_DIR}/.ccache"
export CCACHE_TEMPDIR="${CCACHE_DIR}/tmp"
mkdir -p "${CCACHE_TEMPDIR}"

export CMAKE_PREFIX_PATH="${PX4_GZ_ROOT}/usr${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${PX4_GZ_LIB}:${PX4_GZ_LIB}/OGRE-2.3${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PKG_CONFIG_PATH="${PX4_GZ_LIB}/pkgconfig:${PX4_GZ_ROOT}/usr/share/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"

export GZ_CONFIG_PATH="${PX4_GZ_ROOT}/usr/share/gz"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_GZ_LIB}/gz-sim-8/plugins${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_PHYSICS_ENGINE_PATH="${PX4_GZ_LIB}/gz-physics-7/engine-plugins${GZ_SIM_PHYSICS_ENGINE_PATH:+:${GZ_SIM_PHYSICS_ENGINE_PATH}}"
export GZ_SIM_RENDER_ENGINE_PATH="${PX4_GZ_LIB}/gz-rendering-8/engine-plugins${GZ_SIM_RENDER_ENGINE_PATH:+:${GZ_SIM_RENDER_ENGINE_PATH}}"
export GZ_RENDERING_RESOURCE_PATH="${PX4_GZ_ROOT}/usr/share/gz/gz-rendering8${GZ_RENDERING_RESOURCE_PATH:+:${GZ_RENDERING_RESOURCE_PATH}}"
export GZ_GUI_PLUGIN_PATH="${PX4_GZ_LIB}/gz-gui-8/plugins:${PX4_GZ_LIB}/gz-sim-8/plugins/gui${GZ_GUI_PLUGIN_PATH:+:${GZ_GUI_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="${PX4_GZ_ROOT}/usr/share/gz/gz-sim8${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export QML2_IMPORT_PATH="${PX4_GZ_LIB}/gz-sim-8/plugins/gui${QML2_IMPORT_PATH:+:${QML2_IMPORT_PATH}}"
export PX4_GZ_SIM_RENDER_ENGINE="ogre"

