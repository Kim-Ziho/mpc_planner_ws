#!/usr/bin/env python3
"""
세 가지 방법(STO-MPC, T-MPC++, LMPCC)의 실험 로그(.txt)를 집계해 표용 요약 값을 계산하는 스크립트.

- data/experiments/{파일명 접두사}-*_*.txt 파일을 모두 읽어 반복 실행 세그먼트를 자동 분리합니다.
- Safe, Dur.[s], 그리고 runtime_control_loop/runtime_optimization/runtime_guidance/processing_runtime/prm_runtime/homotopy_comparison_runtime 평균(분산)을 각 방법 행에 채워 넣을 값을 출력합니다.
- objective_0~4/best_planner_idx를 기반으로, 각 실험에서 성공적으로 사용된 guidance trajectory 수(평균/분산)와 guidance vs original planner 선택 비율도 계산합니다.
- 각 반복(idx)별 safe/duration/여러 runtime 및 objective 통계를 CSV로도 남깁니다.

파일명 접두사는 `METHOD_PREFIXES` 전역 상수에서 방법별로 간편하게 변경할 수 있습니다.
"""
from __future__ import annotations

import csv
import re
from dataclasses import dataclass
import math
from pathlib import Path
from statistics import mean, pvariance
from typing import Iterable, List, Sequence

from txt_to_csv import parse_dataset_file


ROOT = Path(__file__).resolve().parent
# 각 방법별 파일명 접두사. 필요에 따라 값을 수정하세요.
METHOD_PREFIXES = [
    ("LMPCC", "expLMPCC3"),
    ("T-MPC++", "expTMPC4"),
    ("STO-MPC(ours)", "exp3"),
]


@dataclass(frozen=True)
class RuntimeMetric:
    dataset: str
    table_label: str
    csv_prefix: str


RUNTIME_METRICS: list[RuntimeMetric] = [
    RuntimeMetric("runtime_control_loop", "Runtime[ms]", "runtime_ctrl"),
    RuntimeMetric("runtime_optimization", "Runtime Opt[ms]", "runtime_opt"),
    RuntimeMetric("runtime_guidance", "Runtime Guidance[ms]", "runtime_guidance"),
    RuntimeMetric("processing_runtime", "Processing Runtime[ms]", "processing"),
    RuntimeMetric("prm_runtime", "PRM Runtime[ms]", "prm"),
    RuntimeMetric("homotopy_comparison_runtime", "Homotopy Runtime[ms]", "homotopy"),
]

# 터미널 표시에 포함할 런타임 메트릭(Processing/PRM/Homotopy는 제외)
DISPLAY_RUNTIME_METRICS: list[RuntimeMetric] = [
    metric
    for metric in RUNTIME_METRICS
    if metric.table_label
    not in {"Processing Runtime[ms]", "PRM Runtime[ms]", "Homotopy Runtime[ms]"}
]

GUIDANCE_OBJECTIVE_KEYS = [f"objective_{idx}" for idx in range(4)]
ALL_OBJECTIVE_KEYS = GUIDANCE_OBJECTIVE_KEYS + ["objective_4"]
BEST_PLANNER_KEY = "best_planner_idx"
ORIGINAL_PLANNER_ID_KEY = "original_planner_id"


@dataclass
class RuntimeStats:
    mean_ms: float
    var_ms2: float


@dataclass
class RunRecord:
    method: str
    global_idx: int
    exp_id: int
    trial_idx: int
    file: str
    safe: int
    duration_s: float
    runtime_stats: dict[str, RuntimeStats]
    guidance_traj_count: int
    guidance_best_count: int
    original_best_count: int
    completed: int


def _extract_exp_id(path: Path, pattern: re.Pattern[str]) -> int:
    match = pattern.match(path.name)
    if not match:
        raise ValueError(f"파일명에서 exp id를 찾지 못했습니다: {path}")
    return int(match.group("exp_id"))


def _split_bounds(reset_values: Sequence[Sequence[float]]) -> List[int]:
    bounds = [0]
    for value in reset_values:
        bounds.append(int(value[0]))
    return bounds


def _safe_flag(collisions: Iterable[float]) -> int:
    return 0 if any(val > 0 for val in collisions) else 1


def load_runs_for_method(
    base: Path, method: str, file_prefix: str
) -> tuple[list[RunRecord], dict[str, list[float]]]:
    runs: list[RunRecord] = []
    aggregated_runtime_ms: dict[str, list[float]] = {metric.dataset: [] for metric in RUNTIME_METRICS}

    pattern = re.compile(rf"{re.escape(file_prefix)}-(?P<exp_id>\d+)_.*\.txt$")
    files = sorted(base.glob(f"{file_prefix}-*_*.txt"), key=lambda p: _extract_exp_id(p, pattern))
    for path in files:
        exp_id = _extract_exp_id(path, pattern)
        rows, _, _ = parse_dataset_file(path)

        resets = _split_bounds(rows["reset"])
        durations = [val[0] for val in rows["metric_duration"]]
        completions = [int(val[0]) for val in rows["metric_completed"]]
        collisions = [val[0] for val in rows["metric_collisions"]]
        runtime_series: dict[str, list[float]] = {}
        for metric in RUNTIME_METRICS:
            dataset_rows = rows.get(metric.dataset)
            if dataset_rows is None:
                runtime_series[metric.dataset] = []
            else:
                runtime_series[metric.dataset] = [val[0] * 1000.0 for val in dataset_rows]

        objective_series: dict[str, list[float]] = {}
        for key in ALL_OBJECTIVE_KEYS:
            dataset_rows = rows.get(key)
            objective_series[key] = [val[0] for val in dataset_rows] if dataset_rows else []

        best_idx_series = [int(val[0]) for val in rows.get(BEST_PLANNER_KEY, [])]
        original_id_series = [int(val[0]) for val in rows.get(ORIGINAL_PLANNER_ID_KEY, [])]

        if len(resets) - 1 != len(durations):
            raise ValueError(
                f"{path.name}: reset({len(resets)-1})와 metric_duration({len(durations)}) 개수가 다릅니다."
            )

        for metric in RUNTIME_METRICS:
            series = runtime_series.get(metric.dataset, [])
            if series and resets[-1] != len(series):
                raise ValueError(
                    f"{path.name}: reset 마지막 값({resets[-1]})과 {metric.dataset} 길이({len(series)})가 다릅니다."
                )
        for key, series in objective_series.items():
            if series and resets[-1] != len(series):
                raise ValueError(
                    f"{path.name}: reset 마지막 값({resets[-1]})과 {key} 길이({len(series)})가 다릅니다."
                )
        if best_idx_series and resets[-1] != len(best_idx_series):
            raise ValueError(
                f"{path.name}: reset 마지막 값({resets[-1]})과 {BEST_PLANNER_KEY} 길이({len(best_idx_series)})가 다릅니다."
            )
        if original_id_series and resets[-1] != len(original_id_series):
            raise ValueError(
                f"{path.name}: reset 마지막 값({resets[-1]})과 {ORIGINAL_PLANNER_ID_KEY} 길이({len(original_id_series)})가 다릅니다."
            )

        for trial_idx, (start, end, duration, completed) in enumerate(
            zip(resets[:-1], resets[1:], durations, completions), start=1
        ):
            collision_slice = collisions[start:end]

            safe = _safe_flag(collision_slice)
            runtime_stats: dict[str, RuntimeStats] = {}
            for metric in RUNTIME_METRICS:
                series = runtime_series[metric.dataset]
                segment = series[start:end] if series else []
                mean_val = mean(segment) if segment else float("nan")
                var_val = pvariance(segment) if len(segment) > 1 else 0.0
                runtime_stats[metric.dataset] = RuntimeStats(mean_val, var_val)
                aggregated_runtime_ms[metric.dataset].extend(segment)

            guidance_traj_count = 0
            for key in GUIDANCE_OBJECTIVE_KEYS:
                series = objective_series.get(key, [])
                if not series:
                    continue
                if any(val != -1.0 for val in series[start:end]):
                    guidance_traj_count += 1

            guidance_best = 0
            original_best = 0
            for idx in range(start, end):
                if idx >= len(best_idx_series):
                    continue
                best_idx = int(best_idx_series[idx])
                if best_idx < 0:
                    continue
                original_idx = (
                    int(original_id_series[idx]) if idx < len(original_id_series) else 4
                )
                if best_idx in range(len(GUIDANCE_OBJECTIVE_KEYS)):
                    guidance_best += 1
                elif best_idx == original_idx:
                    original_best += 1

            runs.append(
                RunRecord(
                    method=method,
                    global_idx=len(runs) + 1,
                    exp_id=exp_id,
                    trial_idx=trial_idx,
                    file=path.name,
                    safe=safe,
                    duration_s=duration,
                    runtime_stats=runtime_stats,
                    guidance_traj_count=guidance_traj_count,
                    guidance_best_count=guidance_best,
                    original_best_count=original_best,
                    completed=completed,
                )
            )
    return runs, aggregated_runtime_ms


def _csv_header() -> list[str]:
    return [
        "method",
        "run_idx",
        "exp_id",
        "file",
        "trial_idx_in_file",
        "safe",
        "duration_s",
        *[f"{metric.csv_prefix}_mean_ms" for metric in RUNTIME_METRICS],
        *[f"{metric.csv_prefix}_var_ms2" for metric in RUNTIME_METRICS],
        "guidance_traj_count",
        "guidance_best_count",
        "original_best_count",
        "completed_flag",
    ]


def write_run_csv(runs: Sequence[RunRecord], output: Path) -> None:
    with output.open("w", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(_csv_header())
        for r in runs:
            writer.writerow(
                [
                    r.method,
                    r.global_idx,
                    r.exp_id,
                    r.file,
                    r.trial_idx,
                    r.safe,
                    f"{r.duration_s:.3f}",
                    *[
                        f"{r.runtime_stats[metric.dataset].mean_ms:.6f}"
                        for metric in RUNTIME_METRICS
                    ],
                    *[
                        f"{r.runtime_stats[metric.dataset].var_ms2:.9f}"
                        for metric in RUNTIME_METRICS
                    ],
                    r.guidance_traj_count,
                    r.guidance_best_count,
                    r.original_best_count,
                    r.completed,
                ]
            )


def _summarize_runs(
    method: str,
    runs: Sequence[RunRecord],
    aggregated_runtime_ms: dict[str, Sequence[float]],
) -> dict[str, str]:
    total_runs = len(runs)
    if total_runs == 0:
        row = {"Method": method, "Dur.[s]": "", "Safe(%)": ""}
        for metric in RUNTIME_METRICS:
            row[metric.table_label] = ""
        row["Guidance Traj Count"] = ""
        row["Guidance vs Orig (%)"] = ""
        return row

    safe_rate = 100.0 * sum(r.safe for r in runs) / total_runs
    durations = [r.duration_s for r in runs]
    avg_duration = mean(durations)
    var_duration = pvariance(durations) if len(durations) > 1 else 0.0

    row = {
        "Method": method,
        "Dur.[s]": f"{avg_duration:.2f} ({var_duration:.6f})",
        "Safe(%)": f"{safe_rate:.1f}",
    }

    for metric in RUNTIME_METRICS:
        values = aggregated_runtime_ms.get(metric.dataset, [])
        if values:
            mean_val = mean(values)
            var_val = pvariance(values) if len(values) > 1 else 0.0
            row[metric.table_label] = f"{mean_val:.3f} ({var_val:.6f})"
        else:
            row[metric.table_label] = ""

    traj_counts = [r.guidance_traj_count for r in runs]
    traj_mean = mean(traj_counts)
    traj_var = pvariance(traj_counts) if len(traj_counts) > 1 else 0.0
    row["Guidance Traj Count"] = f"{traj_mean:.2f} ({traj_var:.6f})"

    total_guidance = sum(r.guidance_best_count for r in runs)
    total_original = sum(r.original_best_count for r in runs)
    total_considered = total_guidance + total_original
    if total_considered > 0:
        guidance_pct = 100.0 * total_guidance / total_considered
        original_pct = 100.0 - guidance_pct
        row["Guidance vs Orig (%)"] = f"{guidance_pct:.1f} / {original_pct:.1f}"
    else:
        row["Guidance vs Orig (%)"] = ""
    return row


def main() -> None:
    all_runs: list[RunRecord] = []
    table_rows: list[dict[str, str]] = []

    for method, prefix in METHOD_PREFIXES:
        runs, aggregated_runtime_ms = load_runs_for_method(ROOT, method, prefix)
        table_rows.append(_summarize_runs(method, runs, aggregated_runtime_ms))
        all_runs.extend(runs)

    print("=== 방법별 표 입력값 ===")
    for row in table_rows:
        details = [
            f"Dur.[s]={row['Dur.[s]']}",
            f"Safe(%)={row['Safe(%)']}",
        ]
        for metric in DISPLAY_RUNTIME_METRICS:
            details.append(f"{metric.table_label}={row.get(metric.table_label, '')}")
        details.append(f"Guidance Traj Count={row.get('Guidance Traj Count', '')}")
        details.append(f"Guidance vs Orig (%)={row.get('Guidance vs Orig (%)', '')}")
        print(f"{row['Method']}: " + ", ".join(details))

    print("\n요약 테이블:")
    table_columns: list[tuple[str, int]] = [
        ("Method", 14),
        ("Dur.[s]", 16),
        ("Safe(%)", 7),
    ]
    for metric in DISPLAY_RUNTIME_METRICS:
        width = max(len(metric.table_label), 21)
        table_columns.append((metric.table_label, width))
    table_columns.append(("Guidance Traj Count", 21))
    table_columns.append(("Guidance vs Orig (%)", 23))

    header = "| " + " | ".join(f"{name:<{width}}" for name, width in table_columns) + " |"
    separator = "| " + " | ".join("-" * width for _, width in table_columns) + " |"
    rows_str = [
        "| "
        + " | ".join(f"{row.get(name, ''):<{width}}" for name, width in table_columns)
        + " |"
        for row in table_rows
    ]
    print("\n".join([header, separator, *rows_str]))

    csv_path = ROOT / "run_metrics.csv"
    write_run_csv(all_runs, csv_path)
    print(f"\n개별 실험 런 통계 CSV: {csv_path}")
    print("컬럼: " + ", ".join(_csv_header()))

    print("\n방법별 샘플 상위 5개 런:")
    for method, _ in METHOD_PREFIXES:
        method_runs = [r for r in all_runs if r.method == method][:5]
        if not method_runs:
            print(f"- {method}: 데이터 없음")
            continue
        print(f"- {method}:")
        for record in method_runs:
            runtime_parts = []
            for metric in RUNTIME_METRICS:
                stats = record.runtime_stats.get(metric.dataset)
                if stats is None or math.isnan(stats.mean_ms):
                    continue
                runtime_parts.append(
                    f"{metric.csv_prefix}={stats.mean_ms:.3f}({stats.var_ms2:.6f}) ms"
                )
            runtime_summary = ", ".join(runtime_parts)
            best_total = record.guidance_best_count + record.original_best_count
            if best_total > 0:
                guidance_pct = 100.0 * record.guidance_best_count / best_total
                orig_pct = 100.0 - guidance_pct
                best_summary = f", guidance_vs_orig={guidance_pct:.1f}/{orig_pct:.1f}%"
            else:
                best_summary = ""
            print(
                f"  #{record.global_idx:03d} exp{record.exp_id:02d}-{record.trial_idx}: "
                f"safe={record.safe}, dur={record.duration_s:.2f}s, guidance_traj_count={record.guidance_traj_count}"
                + (f", {runtime_summary}" if runtime_summary else "")
                + best_summary
            )


if __name__ == "__main__":
    main()
