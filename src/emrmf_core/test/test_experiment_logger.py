import csv
from pathlib import Path

from emrmf_core.experiment_logger import (
    ABLATION_MODES,
    CONFIDENCE_95_Z,
    ExperimentConfiguration,
    ExperimentLogger,
    build_summary_rows,
    compute_theta,
)


def test_compute_theta_matches_emrmf_formula():
    theta = compute_theta(error_norm=0.25, tau_e=1.0, p=2.0, gamma=0.5, delta_t=2.0)
    expected = max(0.0, 1.0 - (0.25 / 1.0) ** 2.0) * 2.718281828459045 ** (-0.5 * 2.0)
    assert abs(theta - expected) < 1e-12


def test_compute_theta_clamps_spatial_term_to_zero():
    assert compute_theta(error_norm=2.0, tau_e=1.0, p=2.0, gamma=0.5, delta_t=2.0) == 0.0


def test_logger_writes_expected_repeated_rows_and_summary(tmp_path: Path):
    logger = ExperimentLogger(
        output_dir=tmp_path,
        p_values=(1.0,),
        gamma_values=(0.5,),
        delays=(0.5, 2.0),
        packet_losses=(0.1, 0.3),
        ablation_modes=ABLATION_MODES,
        repeats=5,
        tau_e=1.0,
        samples_per_run=8,
        seed=1,
        session_id="test-session",
    )

    results_path, summary_path, results, summary_rows = logger.run()

    configuration_count = len(ABLATION_MODES) * 1 * 1 * 2 * 2
    assert results_path.exists()
    assert summary_path.exists()
    assert len(results) == configuration_count * 5
    assert len(summary_rows) == configuration_count * 5
    run_logs = sorted((tmp_path / "test-session" / "run_logs").glob("*.csv"))
    assert len(run_logs) == len(results)
    assert all(result.run_log_file.startswith("run_logs/") for result in results)
    with run_logs[0].open(newline="", encoding="utf-8") as handle:
        run_log_rows = list(csv.DictReader(handle))
    assert len(run_log_rows) == 1
    assert run_log_rows[0]["session_id"] == "test-session"
    assert {result.run_id for result in results} == {1, 2, 3, 4, 5}
    assert {result.delay_s for result in results} == {0.5, 2.0}
    assert {result.packet_loss_rate for result in results} == {0.1, 0.3}


def test_summary_rows_include_95_percent_confidence_interval():
    base_config = ExperimentConfiguration(
        ablation_mode="full_emrmf",
        p=1.0,
        gamma=0.5,
        delay_s=0.5,
        packet_loss_rate=0.1,
    )
    logger = ExperimentLogger(
        output_dir=Path("unused"),
        p_values=(1.0,),
        gamma_values=(0.5,),
        delays=(0.5,),
        packet_losses=(0.1,),
        ablation_modes=("full_emrmf",),
        repeats=5,
        tau_e=1.0,
        samples_per_run=8,
        seed=2,
        session_id="summary-session",
    )
    results = [
        logger.simulator.run(
            base_config,
            run_id,
            session_id="summary-session",
            run_log_file=f"run_logs/run{run_id:02d}.csv",
            run_started_at_utc="2026-05-30T00:00:00Z",
        )
        for run_id in range(1, 6)
    ]
    rows = build_summary_rows(results)
    pose_row = next(row for row in rows if row.metric == "pose_rmse_m")

    assert pose_row.n == 5
    assert pose_row.ci95_low < pose_row.mean < pose_row.ci95_high
    expected_half_width = CONFIDENCE_95_Z * pose_row.std / (pose_row.n ** 0.5)
    assert abs((pose_row.ci95_high - pose_row.mean) - expected_half_width) < 1e-12
