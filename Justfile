set shell := ["bash", "-e", "-o", "pipefail", "-c"]
set quiet

repo := justfile_directory()
vpy := repo + "/.venv/bin/python"
uv := env("UV_BIN", "uv")
system_config := repo + "/configs/system/vlai_l1.toml"
collection_config := repo + "/configs/collection/default.toml"

default:
    @just --list

# Development

check:
    {{ vpy }} -m pytest
    {{ vpy }} -m ruff check .
    {{ vpy }} -m ruff format --check .

# x_air SDK preparation and commissioning observation

sdk-build:
    {{ repo }}/scripts/build_xair.sh

sdk-verify:
    {{ vpy }} -m vlai_l1_runtime.cli verify-xair --config {{ system_config }}

sdk-describe side="left":
    {{ vpy }} -m vlai_l1_runtime.cli describe-xair \
        --config {{ system_config }} \
        --side "{{ side }}"

sdk-prepare output="build/xair-assets":
    {{ vpy }} -m vlai_l1_runtime.cli prepare-xair \
        --config {{ system_config }} \
        --output "{{ repo }}/{{ output }}"

sdk-observe samples="3000" timeout="15":
    sudo install -d -o "$(id -un)" -g "$(id -gn)" /run/vlai-l1
    {{ vpy }} -m vlai_l1_runtime.cli observe-xair \
        --config {{ system_config }} \
        --side bimanual \
        --samples "{{ samples }}" \
        --timeout "{{ timeout }}"

sdk-start side="left":
    {{ vpy }} -m vlai_l1_runtime.cli verify-xair --config {{ system_config }}
    sudo {{ vpy }} -m vlai_l1_runtime.cli run-xair \
        --config {{ system_config }} \
        --side "{{ side }}"

sdk-status:
    #!/usr/bin/env bash
    set -euo pipefail
    systemctl is-active xarm-teleop-left.service || true
    systemctl is-active xarm-teleop-right.service || true
    pgrep -af '[v]lai_l1_xair_sidecar|[u]nilateral_control' || true
    for interface in can0 can1 can2 can3; do
        echo "### ${interface}"
        ip -details -statistics link show "${interface}" | sed -n '1,18p'
        tc -s qdisc show dev "${interface}"
    done

stop:
    #!/usr/bin/env bash
    set -euo pipefail
    sudo pkill -INT -f '[v]lai_l1_runtime.cli run-xair' || true
    sudo pkill -INT -f '^{{ repo }}/build/xair-sidecar/vlai_l1_xair_sidecar ' || true
    sleep 2
    sudo /opt/xarm_teleop/disable_unilateral_pair.sh left_arm
    sudo /opt/xarm_teleop/disable_unilateral_pair.sh right_arm

# Cameras and collection

setup-camera:
    {{ uv }} sync --frozen --python 3.12 --extra camera

setup-dataset:
    {{ uv }} sync --frozen --python 3.12 --extra camera --extra dataset

camera-list:
    #!/usr/bin/env bash
    set -euo pipefail
    shopt -s nullglob
    {
        for path in /dev/v4l/by-id/*; do
            device="$(readlink -f "${path}")"
            udevadm info --query=property --name="${device}" \
                | sed -n 's/^ID_SERIAL_SHORT=//p'
        done
    } | sort -u

camera-check samples="30" timeout="0.25":
    {{ repo }}/scripts/camera_service.sh start
    {{ vpy }} -m vlai_l1_runtime.cli camera-check \
        --config {{ system_config }} \
        --samples "{{ samples }}" \
        --timeout "{{ timeout }}"

cameras action="start":
    {{ repo }}/scripts/camera_service.sh "{{ action }}"

panel:
    {{ vpy }} -m vlai_l1_runtime.cli panel --config {{ collection_config }}

reset:
    {{ vpy }} -m vlai_l1_runtime.cli reset --config {{ collection_config }}

collect experiment task:
    {{ repo }}/scripts/collect.sh \
        --config {{ collection_config }} \
        --experiment "{{ experiment }}" \
        --task "{{ task }}"

dataset-doctor experiment:
    {{ vpy }} -m vlai_l1_runtime.cli dataset-doctor \
        --config {{ collection_config }} \
        --experiment "{{ experiment }}"

export-v21 experiment:
    {{ vpy }} -m vlai_l1_runtime.cli export-v21 \
        --config {{ collection_config }} \
        --experiment "{{ experiment }}"
