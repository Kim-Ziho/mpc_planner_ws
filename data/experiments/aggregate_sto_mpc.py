#!/usr/bin/env python3
"""
STO-MPC 실험 로그(.txt)를 집계해 표용 요약 값을 계산하는 스크립트.

- data/experiments/exp1-*_*.txt 파일을 모두 읽어 반복 실행 세그먼트를 자동 분리합니다.
- Safe, Dur.[s], Runtime[ms] 평균(분산)을 STO-MPC(ours) 행에 채워 넣을 값을 출력합니다.
- 각 반복(idx)별 safe/duration/runtime 통계를 CSV로도 남깁니다.
"""
from __future__ import annotations

import csv
import re
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, pvariance
from typing import Iterable, List, Sequence

from txt_to_csv import parse_dataset_file


ROOT = Path(__file__).resolve().parent
PATTERN = re.compile(r"exp1-(?P<exp_id>\d+)_.*\.txt$")


@dataclass
class RunRecord:
    global_idx: int
    exp_id: int
    trial_idx: int
    file: str
    safe: int
    duration_s: float
    runtime_mean_ms: float
    runtime_var_ms2: float
    completed: int


def _extract_exp_id(path: Path) -> int:
    match = PATTERN.match(path.name)
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


def load_runs(base: Path) -> tuple[list[RunRecord], list[float]]:
    runs: list[RunRecord] = []
    all_runtimes_ms: list[float] = []

    files = sorted(base.glob("exp1-*_*.txt"), key=_extract_exp_id)
    for path in files:
        exp_id = _extract_exp_id(path)
        rows, _, _ = parse_dataset_file(path)

        resets = _split_bounds(rows["reset"])
        durations = [val[0] for val in rows["metric_duration"]]
        completions = [int(val[0]) for val in rows["metric_completed"]]
        collisions = [val[0] for val in rows["metric_collisions"]]
        runtimes_ms = [val[0] * 1000.0 for val in rows["runtime_control_loop"]]

        if len(resets) - 1 != len(durations):
            raise ValueError(
                f"{path.name}: reset({len(resets)-1})와 metric_duration({len(durations)}) 개수가 다릅니다."
            )

        if resets[-1] != len(runtimes_ms):
            raise ValueError(
                f"{path.name}: reset 마지막 값({resets[-1]})과 runtime 길이({len(runtimes_ms)})가 다릅니다."
            )

        for trial_idx, (start, end, duration, completed) in enumerate(
            zip(resets[:-1], resets[1:], durations, completions), start=1
        ):
            runtime_slice = runtimes_ms[start:end]
            collision_slice = collisions[start:end]

            runtime_mean = mean(runtime_slice) if runtime_slice else float("nan")
            runtime_var = pvariance(runtime_slice) if len(runtime_slice) > 1 else 0.0
            safe = _safe_flag(collision_slice)

            runs.append(
                RunRecord(
                    global_idx=len(runs) + 1,
                    exp_id=exp_id,
                    trial_idx=trial_idx,
                    file=path.name,
                    safe=safe,
                    duration_s=duration,
                    runtime_mean_ms=runtime_mean,
                    runtime_var_ms2=runtime_var,
                    completed=completed,
                )
            )
            all_runtimes_ms.extend(runtime_slice)
    return runs, all_runtimes_ms


def write_run_csv(runs: Sequence[RunRecord], output: Path) -> None:
    with output.open("w", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(
            [
                "run_idx",
                "exp_id",
                "file",
                "trial_idx_in_file",
                "safe",
                "duration_s",
                "runtime_mean_ms",
                "runtime_var_ms2",
                "completed_flag",
            ]
        )
        for r in runs:
            writer.writerow(
                [
                    r.global_idx,
                    r.exp_id,
                    r.file,
                    r.trial_idx,
                    r.safe,
                    f"{r.duration_s:.3f}",
                    f"{r.runtime_mean_ms:.6f}",
                    f"{r.runtime_var_ms2:.9f}",
                    r.completed,
                ]
            )


def main() -> None:
    runs, all_runtimes_ms = load_runs(ROOT)
    total_runs = len(runs)
    safe_rate = 100.0 * sum(r.safe for r in runs) / total_runs if total_runs else 0.0
    durations = [r.duration_s for r in runs]
    avg_duration = mean(durations) if durations else 0.0
    var_duration = pvariance(durations) if len(durations) > 1 else 0.0
    runtime_mean = mean(all_runtimes_ms) if all_runtimes_ms else 0.0
    runtime_var = pvariance(all_runtimes_ms) if len(all_runtimes_ms) > 1 else 0.0

    summary_row = {
        "Method": "STO-MPC(ours)",
        "Dur.[s]": f"{avg_duration:.2f} ({var_duration:.6f})",
        "Safe(%)": f"{safe_rate:.1f}",
        "Runtime[ms]": f"{runtime_mean:.3f} ({runtime_var:.6f})",
    }

    print("=== STO-MPC 표 입력값 ===")
    print(f"Dur.[s]: {summary_row['Dur.[s]']}")
    print(f"Safe(%): {summary_row['Safe(%)']}")
    print(f"Runtime[ms]: {summary_row['Runtime[ms]']}")
    print()
    print("요약 테이블 (LMPCC/T-MPC++는 비워둠):")
    print(
        "\n".join(
            [
                "| Method         | Dur.[s]          | Safe(%) | Runtime[ms]           |",
                "|----------------|------------------|---------|-----------------------|",
                "| LMPCC          |                  |         |                       |",
                "| T-MPC++        |                  |         |                       |",
                f"| {summary_row['Method']:<14} | {summary_row['Dur.[s]']:<16} | {summary_row['Safe(%)']:<7} | {summary_row['Runtime[ms]']:<21} |",
            ]
        )
    )

    csv_path = ROOT / "sto_mpc_run_metrics.csv"
    write_run_csv(runs, csv_path)
    print(f"\n개별 실험 런 통계 CSV: {csv_path}")
    print("컬럼: run_idx, exp_id, file, trial_idx_in_file, safe, duration_s, runtime_mean_ms, runtime_var_ms2, completed_flag")

    print("\n샘플 상위 5개 런:")
    for record in runs[:5]:
        print(
            f"#{record.global_idx:03d} exp{record.exp_id:02d}-{record.trial_idx}: "
            f"safe={record.safe}, dur={record.duration_s:.2f}s, "
            f"runtime={record.runtime_mean_ms:.3f}({record.runtime_var_ms2:.6f}) ms"
        )


if __name__ == "__main__":
    main()
