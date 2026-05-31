#!/usr/bin/env bash
set -euo pipefail

# Springer-revision EMRMF experiment runner.
#
# This script runs the EMRMF experiment logger and writes:
#   1. one timestamped CSV under run_logs/ for every individual run,
#   2. one aggregate repeated-run CSV,
#   3. one final summary CSV with mean, standard deviation, and 95% CI.
#
# Override any variable below when needed, e.g.:
#   OUTPUT_DIR=results/revision_2 SESSION_ID=springer_trial_01 ./run_emrmf_experiments.sh

OUTPUT_DIR=${OUTPUT_DIR:-results/emrmf_revision}
SESSION_ID=${SESSION_ID:-$(date -u +%Y%m%dT%H%M%SZ)}
P_VALUES=${P_VALUES:-"1.0 2.0 3.0"}
GAMMA_VALUES=${GAMMA_VALUES:-"0.0 0.25 0.5 1.0"}
DELAYS=${DELAYS:-"0.5 2.0"}
PACKET_LOSSES=${PACKET_LOSSES:-"0.10 0.30"}
REPEATS=${REPEATS:-5}
SAMPLES_PER_RUN=${SAMPLES_PER_RUN:-60}
SEED=${SEED:-7}

read -r -a P_VALUE_ARGS <<< "${P_VALUES}"
read -r -a GAMMA_VALUE_ARGS <<< "${GAMMA_VALUES}"
read -r -a DELAY_ARGS <<< "${DELAYS}"
read -r -a PACKET_LOSS_ARGS <<< "${PACKET_LOSSES}"

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
LOGGER_PY="${SCRIPT_DIR}/src/emrmf_core/emrmf_core/experiment_logger.py"

python3 "${LOGGER_PY}" \
  --output-dir "${OUTPUT_DIR}" \
  --session-id "${SESSION_ID}" \
  --p-values "${P_VALUE_ARGS[@]}" \
  --gamma-values "${GAMMA_VALUE_ARGS[@]}" \
  --delays "${DELAY_ARGS[@]}" \
  --packet-losses "${PACKET_LOSS_ARGS[@]}" \
  --repeats "${REPEATS}" \
  --samples-per-run "${SAMPLES_PER_RUN}" \
  --seed "${SEED}"

SESSION_DIR="${OUTPUT_DIR}/${SESSION_ID}"
SUMMARY_CSV="${SESSION_DIR}/emrmf_experiment_summary_${SESSION_ID}.csv"
RESULTS_CSV="${SESSION_DIR}/emrmf_experiment_results_${SESSION_ID}.csv"
RUN_LOG_COUNT=$(find "${SESSION_DIR}/run_logs" -maxdepth 1 -type f -name '*.csv' | wc -l | tr -d ' ')

printf '\nEMRMF experiment files saved for Springer revision:\n'
printf '  Session directory: %s\n' "${SESSION_DIR}"
printf '  Timestamped run logs: %s/run_logs/ (%s CSV files)\n' "${SESSION_DIR}" "${RUN_LOG_COUNT}"
printf '  Aggregate run CSV: %s\n' "${RESULTS_CSV}"
printf '  Final summary CSV: %s\n' "${SUMMARY_CSV}"
