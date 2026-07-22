set shell := ["bash", "-e", "-o", "pipefail", "-c"]
set quiet

repo := justfile_directory()
vpy := repo + "/.venv/bin/python"
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

sdk-status:
    #!/usr/bin/env bash
    set -euo pipefail
    systemctl is-active xarm-teleop-left.service || true
    systemctl is-active xarm-teleop-right.service || true
    pgrep -af '[v]lai_l1_xair_sidecar|[u]nilateral_control' || true
    for interface in can0 can1 can2 can3; do
        echo "### ${interface}"
        ip -details -statistics link show "${interface}" | sed -n '1,18p'
    done

sdk-stop:
    #!/usr/bin/env bash
    set -euo pipefail
    sudo pkill -INT -f '^{{ repo }}/build/xair-sidecar/vlai_l1_xair_sidecar ' || true
    sleep 2
    sudo /opt/xarm_teleop/disable_unilateral_pair.sh left_arm
    sudo /opt/xarm_teleop/disable_unilateral_pair.sh right_arm

# Cameras and collection

camera-list:
    #!/usr/bin/env bash
    set -euo pipefail
    for path in /sys/bus/usb/devices/*; do
        if [[ -r "${path}/idVendor" ]] \
            && [[ "$(<"${path}/idVendor")" == "8086" ]] \
            && [[ -r "${path}/idProduct" ]] \
            && [[ "$(<"${path}/idProduct")" == "0b5b" ]]; then
            printf '%s\n' "$(<"${path}/serial")"
        fi
    done

panel:
    {{ vpy }} -m vlai_l1_runtime.cli panel --config {{ collection_config }}

collect experiment task frames="300" decision="save":
    {{ vpy }} -m vlai_l1_runtime.cli collect \
        --config {{ collection_config }} \
        --experiment "{{ experiment }}" \
        --task "{{ task }}" \
        --frames "{{ frames }}" \
        --decision "{{ decision }}"

dataset-doctor experiment:
    {{ vpy }} -m vlai_l1_runtime.cli dataset-doctor \
        --config {{ collection_config }} \
        --experiment "{{ experiment }}"

export-v21 experiment:
    {{ vpy }} -m vlai_l1_runtime.cli export-v21 \
        --config {{ collection_config }} \
        --experiment "{{ experiment }}"
