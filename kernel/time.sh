#!/usr/bin/env bash
set -euo pipefail

QEMU="qemu-system-aarch64"
QEMU_ARGS=(
  -M raspi4b
  -cpu cortex-a53
  -m 2G
  -serial stdio
  -display none
  -kernel iotest.img
)

for opt in -O3 -O0; do
  for data_cache in true false; do
    for instruction_cache in true false; do
      case "${data_cache}" in
        true) data_label="on" ;;
        false) data_label="off" ;;
      esac
      case "${instruction_cache}" in
        true) instruction_label="on" ;;
        false) instruction_label="off" ;;
      esac

      log_file="time_${opt#-}_data-${data_label}_instr-${instruction_label}.log"

      printf 'Running %s -> %s\n' "${opt} data_cache=${data_cache} instruction_cache=${instruction_cache}" "${log_file}" >&2
      {
        echo "### opt=${opt} data_cache=${data_cache} instruction_cache=${instruction_cache}"
        echo "### $(date -Is)"
        make clean
        make OPT="${opt}" DATA_CACHE="${data_cache}" INSTRUCTION_CACHE="${instruction_cache}" PERF_TEST="true"
        "${QEMU}" "${QEMU_ARGS[@]}"
      } &> "${log_file}"
    done
  done
done
