#!/usr/bin/env python3
"""Journal-ready EMRMF multi-robot SLAM experiment logger.

This module provides a ROS 2 console entry point that repeatedly evaluates EMRMF
trust-factor settings, artificial communication-delay settings, packet-loss
settings, and ablation modes.  It intentionally keeps the metric model
self-contained so the logger can be used before real robots are available; the
same CSV/summary pipeline can also be fed by replacing ``MetricSimulator`` with
measurements from ROS topics or rosbag playback.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import statistics
import time
from datetime import datetime, timezone
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

ABLATION_MODES = (
    "baseline_graph_slam",
    "decentralized_only",
    "trust_only",
    "full_emrmf",
)

DEFAULT_P_VALUES = (1.0, 2.0, 3.0)
DEFAULT_GAMMA_VALUES = (0.0, 0.25, 0.5, 1.0)
DEFAULT_DELAYS_SECONDS = (0.5, 2.0)
DEFAULT_PACKET_LOSS_RATES = (0.10, 0.30)
DEFAULT_REPEATS = 5
TAU_E_METERS = 1.0
SAMPLES_PER_RUN = 60
CONFIDENCE_95_Z = 1.96

RESULT_FIELDS = (
    "session_id",
    "run_log_file",
    "run_started_at_utc",
    "run_completed_at_utc",
    "configuration_id",
    "run_id",
    "ablation_mode",
    "p",
    "gamma",
    "delay_s",
    "packet_loss_rate",
    "pose_rmse_m",
    "map_alignment_rmse_m",
    "fusion_time_s",
    "theta_mean",
    "theta_std",
)

SUMMARY_FIELDS = (
    "session_id",
    "configuration_id",
    "ablation_mode",
    "p",
    "gamma",
    "delay_s",
    "packet_loss_rate",
    "metric",
    "mean",
    "std",
    "ci95_low",
    "ci95_high",
    "n",
)

SUMMARY_METRICS = (
    "pose_rmse_m",
    "map_alignment_rmse_m",
    "fusion_time_s",
    "theta_mean",
    "theta_std",
)


@dataclass(frozen=True)
class ExperimentConfiguration:
    """One experiment condition for a trust-factor ablation run."""

    ablation_mode: str
    p: float
    gamma: float
    delay_s: float
    packet_loss_rate: float

    @property
    def configuration_id(self) -> str:
        safe_mode = self.ablation_mode.replace("_", "-")
        p_text = format_float_token(self.p)
        gamma_text = format_float_token(self.gamma)
        delay_text = format_float_token(self.delay_s)
        loss_text = format_float_token(self.packet_loss_rate)
        return f"{safe_mode}_p{p_text}_g{gamma_text}_d{delay_text}_loss{loss_text}"


@dataclass(frozen=True)
class ExperimentResult:
    """Metrics captured for a single repeated run."""

    session_id: str
    run_log_file: str
    run_started_at_utc: str
    run_completed_at_utc: str
    configuration_id: str
    run_id: int
    ablation_mode: str
    p: float
    gamma: float
    delay_s: float
    packet_loss_rate: float
    pose_rmse_m: float
    map_alignment_rmse_m: float
    fusion_time_s: float
    theta_mean: float
    theta_std: float

    def as_csv_row(self) -> dict[str, str]:
        return {
            "session_id": self.session_id,
            "run_log_file": self.run_log_file,
            "run_started_at_utc": self.run_started_at_utc,
            "run_completed_at_utc": self.run_completed_at_utc,
            "configuration_id": self.configuration_id,
            "run_id": str(self.run_id),
            "ablation_mode": self.ablation_mode,
            "p": format_float(self.p),
            "gamma": format_float(self.gamma),
            "delay_s": format_float(self.delay_s),
            "packet_loss_rate": format_float(self.packet_loss_rate),
            "pose_rmse_m": format_float(self.pose_rmse_m),
            "map_alignment_rmse_m": format_float(self.map_alignment_rmse_m),
            "fusion_time_s": format_float(self.fusion_time_s),
            "theta_mean": format_float(self.theta_mean),
            "theta_std": format_float(self.theta_std),
        }


@dataclass(frozen=True)
class SummaryRow:
    """Mean, standard deviation, and 95% confidence interval for one metric."""

    session_id: str
    configuration_id: str
    ablation_mode: str
    p: float
    gamma: float
    delay_s: float
    packet_loss_rate: float
    metric: str
    mean: float
    std: float
    ci95_low: float
    ci95_high: float
    n: int

    def as_csv_row(self) -> dict[str, str]:
        return {
            "session_id": self.session_id,
            "configuration_id": self.configuration_id,
            "ablation_mode": self.ablation_mode,
            "p": format_float(self.p),
            "gamma": format_float(self.gamma),
            "delay_s": format_float(self.delay_s),
            "packet_loss_rate": format_float(self.packet_loss_rate),
            "metric": self.metric,
            "mean": format_float(self.mean),
            "std": format_float(self.std),
            "ci95_low": format_float(self.ci95_low),
            "ci95_high": format_float(self.ci95_high),
            "n": str(self.n),
        }


def compute_theta(error_norm: float, tau_e: float, p: float, gamma: float, delta_t: float) -> float:
    """Compute the EMRMF trust factor theta.

    theta = max(0, 1 - (norm(e_ij) / tau_e)^p) * exp(-gamma * delta_t)
    """
    if tau_e <= 0:
        raise ValueError("tau_e must be positive")
    if p <= 0:
        raise ValueError("p must be positive")
    if gamma < 0:
        raise ValueError("gamma must be non-negative")
    if delta_t < 0:
        raise ValueError("delta_t must be non-negative")

    spatial_term = max(0.0, 1.0 - (error_norm / tau_e) ** p)
    temporal_term = math.exp(-gamma * delta_t)
    return spatial_term * temporal_term


class MetricSimulator:
    """Deterministic synthetic metric source for repeatable EMRMF experiments.

    The simulator encodes expected qualitative behavior: packet loss and delay
    increase RMSE/fusion time, while trust-enabled and full EMRMF modes benefit
    from higher theta values.  Replace this class with live ROS measurements when
    integrating robot topics, while keeping the CSV and summary code unchanged.
    """

    _mode_offsets = {
        "baseline_graph_slam": (0.18, 0.14, 0.32),
        "decentralized_only": (0.30, 0.24, 0.12),
        "trust_only": (0.08, 0.10, 0.20),
        "full_emrmf": (0.00, 0.00, 0.00),
    }

    def __init__(self, seed: int, tau_e: float, samples_per_run: int) -> None:
        self._rng = random.Random(seed)
        self._tau_e = tau_e
        self._samples_per_run = samples_per_run

    def run(
        self,
        configuration: ExperimentConfiguration,
        run_id: int,
        session_id: str,
        run_log_file: str,
        run_started_at_utc: str,
    ) -> ExperimentResult:
        start_time = time.perf_counter()
        theta_samples = self._theta_samples(configuration, run_id)
        theta_mean = statistics.fmean(theta_samples)
        theta_std = statistics.stdev(theta_samples) if len(theta_samples) > 1 else 0.0

        pose_offset, map_offset, fusion_offset = self._mode_offsets[configuration.ablation_mode]
        trust_gain = theta_mean if configuration.ablation_mode in ("trust_only", "full_emrmf") else 0.0
        decentralized_penalty = 0.08 if configuration.ablation_mode == "decentralized_only" else 0.0
        baseline_penalty = 0.05 if configuration.ablation_mode == "baseline_graph_slam" else 0.0
        noise = self._rng.gauss(0.0, 0.015)

        pose_rmse = (
            0.28
            + pose_offset
            + decentralized_penalty
            + baseline_penalty
            + 0.12 * configuration.delay_s
            + 0.85 * configuration.packet_loss_rate
            - 0.18 * trust_gain
            + noise
        )
        map_alignment_rmse = (
            0.20
            + map_offset
            + 0.08 * configuration.delay_s
            + 0.62 * configuration.packet_loss_rate
            - 0.14 * trust_gain
            + self._rng.gauss(0.0, 0.012)
        )
        fusion_time = (
            0.55
            + fusion_offset
            + 0.36 * configuration.delay_s
            + 0.95 * configuration.packet_loss_rate
            + 0.03 * configuration.gamma
            + self._rng.gauss(0.0, 0.02)
        )

        # Keep a measurable wall-clock component for integration with profilers
        # without slowing journal sweeps.
        elapsed = time.perf_counter() - start_time
        fusion_time = max(fusion_time, elapsed)

        return ExperimentResult(
            session_id=session_id,
            run_log_file=run_log_file,
            run_started_at_utc=run_started_at_utc,
            run_completed_at_utc=utc_timestamp(),
            configuration_id=configuration.configuration_id,
            run_id=run_id,
            ablation_mode=configuration.ablation_mode,
            p=configuration.p,
            gamma=configuration.gamma,
            delay_s=configuration.delay_s,
            packet_loss_rate=configuration.packet_loss_rate,
            pose_rmse_m=max(0.0, pose_rmse),
            map_alignment_rmse_m=max(0.0, map_alignment_rmse),
            fusion_time_s=max(0.0, fusion_time),
            theta_mean=theta_mean,
            theta_std=theta_std,
        )

    def _theta_samples(self, configuration: ExperimentConfiguration, run_id: int) -> list[float]:
        theta_samples: list[float] = []
        for sample_index in range(self._samples_per_run):
            # Error grows with delay and packet loss; repeated runs get slight
            # deterministic variation to preserve reproducibility.
            base_error = 0.15 + 0.08 * configuration.delay_s + 0.55 * configuration.packet_loss_rate
            run_variation = 0.01 * run_id + 0.002 * sample_index
            error_norm = max(0.0, self._rng.gauss(base_error + run_variation, 0.045))
            delta_t = max(0.0, self._rng.gauss(configuration.delay_s, 0.08 + 0.02 * configuration.delay_s))
            theta_samples.append(
                compute_theta(
                    error_norm=error_norm,
                    tau_e=self._tau_e,
                    p=configuration.p,
                    gamma=configuration.gamma,
                    delta_t=delta_t,
                )
            )
        return theta_samples


class ExperimentLogger:
    """Run full factorial EMRMF experiment sweeps and save CSV artefacts."""

    def __init__(
        self,
        output_dir: Path,
        p_values: Sequence[float],
        gamma_values: Sequence[float],
        delays: Sequence[float],
        packet_losses: Sequence[float],
        ablation_modes: Sequence[str],
        repeats: int,
        tau_e: float,
        samples_per_run: int,
        seed: int,
        session_id: str | None = None,
    ) -> None:
        self.output_dir = output_dir
        self.p_values = p_values
        self.gamma_values = gamma_values
        self.delays = delays
        self.packet_losses = packet_losses
        self.ablation_modes = ablation_modes
        self.repeats = repeats
        self.tau_e = tau_e
        self.samples_per_run = samples_per_run
        self.session_id = session_id or utc_timestamp(for_filename=True)
        self.simulator = MetricSimulator(seed=seed, tau_e=tau_e, samples_per_run=samples_per_run)

    def configurations(self) -> Iterable[ExperimentConfiguration]:
        for ablation_mode in self.ablation_modes:
            for p in self.p_values:
                for gamma in self.gamma_values:
                    for delay in self.delays:
                        for packet_loss in self.packet_losses:
                            yield ExperimentConfiguration(
                                ablation_mode=ablation_mode,
                                p=p,
                                gamma=gamma,
                                delay_s=delay,
                                packet_loss_rate=packet_loss,
                            )

    def run(self) -> tuple[Path, Path, list[ExperimentResult], list[SummaryRow]]:
        self._validate()
        session_dir = self.output_dir / self.session_id
        run_log_dir = session_dir / "run_logs"
        run_log_dir.mkdir(parents=True, exist_ok=True)

        results: list[ExperimentResult] = []
        for configuration in self.configurations():
            for run_id in range(1, self.repeats + 1):
                started_at = utc_timestamp()
                run_log_path = run_log_dir / build_run_log_filename(started_at, configuration, run_id)
                result = self.simulator.run(
                    configuration=configuration,
                    run_id=run_id,
                    session_id=self.session_id,
                    run_log_file=str(run_log_path.relative_to(session_dir)),
                    run_started_at_utc=started_at,
                )
                write_csv(run_log_path, RESULT_FIELDS, (result.as_csv_row(),))
                results.append(result)

        summary_rows = build_summary_rows(results)
        results_path = session_dir / f"emrmf_experiment_results_{self.session_id}.csv"
        summary_path = session_dir / f"emrmf_experiment_summary_{self.session_id}.csv"
        write_csv(results_path, RESULT_FIELDS, (result.as_csv_row() for result in results))
        write_csv(summary_path, SUMMARY_FIELDS, (row.as_csv_row() for row in summary_rows))
        return results_path, summary_path, results, summary_rows

    def _validate(self) -> None:
        if self.repeats < 1:
            raise ValueError("repeats must be at least 1")
        if self.samples_per_run < 2:
            raise ValueError("samples_per_run must be at least 2")
        unknown_modes = sorted(set(self.ablation_modes) - set(ABLATION_MODES))
        if unknown_modes:
            raise ValueError(f"unsupported ablation modes: {', '.join(unknown_modes)}")
        for value in self.p_values:
            if value <= 0:
                raise ValueError("all p values must be positive")
        for value in self.gamma_values:
            if value < 0:
                raise ValueError("all gamma values must be non-negative")
        for value in self.delays:
            if value < 0:
                raise ValueError("all delays must be non-negative")
        for value in self.packet_losses:
            if value < 0 or value > 1:
                raise ValueError("all packet losses must be in [0, 1]")
        if self.tau_e <= 0:
            raise ValueError("tau_e must be positive")


def build_summary_rows(results: Sequence[ExperimentResult]) -> list[SummaryRow]:
    grouped: dict[tuple[str, str, str, float, float, float, float], list[ExperimentResult]] = {}
    for result in results:
        key = (
            result.session_id,
            result.configuration_id,
            result.ablation_mode,
            result.p,
            result.gamma,
            result.delay_s,
            result.packet_loss_rate,
        )
        grouped.setdefault(key, []).append(result)

    rows: list[SummaryRow] = []
    for key in sorted(grouped):
        session_id, configuration_id, ablation_mode, p, gamma, delay_s, packet_loss_rate = key
        group = grouped[key]
        for metric in SUMMARY_METRICS:
            values = [float(getattr(result, metric)) for result in group]
            mean = statistics.fmean(values)
            std = statistics.stdev(values) if len(values) > 1 else 0.0
            half_width = CONFIDENCE_95_Z * std / math.sqrt(len(values)) if values else 0.0
            rows.append(
                SummaryRow(
                    session_id=session_id,
                    configuration_id=configuration_id,
                    ablation_mode=ablation_mode,
                    p=p,
                    gamma=gamma,
                    delay_s=delay_s,
                    packet_loss_rate=packet_loss_rate,
                    metric=metric,
                    mean=mean,
                    std=std,
                    ci95_low=mean - half_width,
                    ci95_high=mean + half_width,
                    n=len(values),
                )
            )
    return rows


def utc_timestamp(for_filename: bool = False) -> str:
    timestamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
    if for_filename:
        return timestamp.replace(":", "").replace("-", "").replace(".", "").replace("Z", "Z")
    return timestamp


def build_run_log_filename(timestamp_utc: str, configuration: ExperimentConfiguration, run_id: int) -> str:
    safe_timestamp = timestamp_utc.replace(":", "").replace("-", "").replace(".", "").replace("Z", "Z")
    return f"{safe_timestamp}_{configuration.configuration_id}_run{run_id:02d}.csv"


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_float_list(raw_values: Sequence[str]) -> tuple[float, ...]:
    parsed: list[float] = []
    for raw_value in raw_values:
        for token in raw_value.split(","):
            stripped = token.strip()
            if stripped:
                parsed.append(float(stripped))
    return tuple(parsed)


def format_float(value: float) -> str:
    return f"{value:.6f}"


def format_float_token(value: float) -> str:
    return format_float(value).rstrip("0").rstrip(".").replace("-", "m").replace(".", "p")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run repeated EMRMF multi-robot SLAM experiments and write journal-ready CSV summaries.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("emrmf_experiment_logs"),
        help="Root directory for timestamped session folders, per-run CSV logs, and final summary CSVs.",
    )
    parser.add_argument(
        "--p-values",
        nargs="+",
        default=[str(value) for value in DEFAULT_P_VALUES],
        help="Trust spatial exponent values; accepts space-separated values or comma lists.",
    )
    parser.add_argument(
        "--gamma-values",
        nargs="+",
        default=[str(value) for value in DEFAULT_GAMMA_VALUES],
        help="Trust temporal decay values; accepts space-separated values or comma lists.",
    )
    parser.add_argument(
        "--delays",
        nargs="+",
        default=[str(value) for value in DEFAULT_DELAYS_SECONDS],
        help="Artificial communication delays in seconds; defaults to 0.5 and 2.0.",
    )
    parser.add_argument(
        "--packet-losses",
        nargs="+",
        default=[str(value) for value in DEFAULT_PACKET_LOSS_RATES],
        help="Artificial packet-loss rates; defaults to 0.10 and 0.30.",
    )
    parser.add_argument(
        "--ablation-modes",
        nargs="+",
        default=list(ABLATION_MODES),
        choices=ABLATION_MODES,
        help="Ablation modes to execute.",
    )
    parser.add_argument("--repeats", type=int, default=DEFAULT_REPEATS, help="Repeated runs per configuration.")
    parser.add_argument("--tau-e", type=float, default=TAU_E_METERS, help="Error threshold tau_e in meters.")
    parser.add_argument("--samples-per-run", type=int, default=SAMPLES_PER_RUN, help="Theta samples per repeated run.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed for reproducible synthetic metrics.")
    parser.add_argument(
        "--session-id",
        default=None,
        help="Optional timestamp/session label used for the output folder and CSV names.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    logger = ExperimentLogger(
        output_dir=args.output_dir,
        p_values=parse_float_list(args.p_values),
        gamma_values=parse_float_list(args.gamma_values),
        delays=parse_float_list(args.delays),
        packet_losses=parse_float_list(args.packet_losses),
        ablation_modes=tuple(args.ablation_modes),
        repeats=args.repeats,
        tau_e=args.tau_e,
        samples_per_run=args.samples_per_run,
        seed=args.seed,
        session_id=args.session_id,
    )
    results_path, summary_path, results, summary_rows = logger.run()
    unique_configurations = {result.configuration_id for result in results}
    print(f"Wrote {len(results)} timestamped per-run CSV logs under {results_path.parent / 'run_logs'}")
    print(f"Wrote {len(results)} repeated-run rows for {len(unique_configurations)} configurations to {results_path}")
    print(f"Wrote final summary CSV with {len(summary_rows)} mean/std/95% CI rows to {summary_path}")


if __name__ == "__main__":
    main()
