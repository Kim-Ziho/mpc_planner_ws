#!/usr/bin/env python3
"""
ROS 네비게이션 실험을 자동 반복 실행하는 스크립트.

- roslaunch mpc_planner_rosnavigation ros1_rosnavigation.launch를 실행해 로그를 모니터링합니다.
- "Completed 5 experiments." 로그가 나오면 roslaunch 프로세스를 종료한 뒤
  data/experiments/set_next_iterations.py를 실행하여 다음 실험 설정을 준비하고 다시 roslaunch를 실행합니다.
- "MPC failed"가 5번 연속 나오거나 "process has died"가 나오면 준비 과정 없이 즉시 roslaunch를 재시작합니다.
"""
from __future__ import annotations

import argparse
import io
import os
import pty
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Literal, Optional, TextIO, Tuple

WORKSPACE = Path("/workspace")
SET_NEXT_SCRIPT = WORKSPACE / "data" / "experiments" / "set_next_iterations.py"
ROS_LAUNCH_CMD = (
    "source /workspace/devel/setup.bash && "
    "roslaunch mpc_planner_rosnavigation ros1_rosnavigation.launch"
)


Result = Literal["completed", "restart_no_prep", "exited"]
RESTART_DELAY_SEC = 0.0


def _stop_process(proc: subprocess.Popen[bytes]) -> None:
    """roslaunch를 깔끔하게 종료합니다."""
    if proc.poll() is not None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=10)
    except Exception:
        proc.kill()
        proc.wait(timeout=5)


def _run_set_next_iterations() -> None:
    """다음 실험 번호로 설정 파일을 갱신합니다."""
    completed = subprocess.run(
        ["python3", str(SET_NEXT_SCRIPT)],
        cwd=str(WORKSPACE),
        text=True,
        capture_output=True,
    )
    if completed.stdout:
        sys.stdout.write(completed.stdout)
    if completed.stderr:
        sys.stderr.write(completed.stderr)

    if completed.returncode != 0:
        raise RuntimeError(
            f"set_next_iterations.py 실행 실패 (code={completed.returncode})"
        )


def _kill_ros_processes() -> None:
    """남아 있을 수 있는 ros 관련 프로세스를 강제로 종료합니다."""
    patterns = [
        "roslaunch mpc_planner_rosnavigation",
        # "rosmaster",
        # "roscore",
        # "gazebo",
    ]
    for pattern in patterns:
        subprocess.run(
            ["bash", "-lc", f"pkill -9 -f '{pattern}' || true"],
            cwd=str(WORKSPACE),
            text=True,
        )


def _monitor_roslaunch(stream: TextIO) -> Result:
    """roslaunch 로그를 모니터링하며 재시작 조건을 판단합니다."""
    consecutive_mpc_failed = 0
    for line in stream:
        sys.stdout.write(line)
        sys.stdout.flush()

        if "Completed" in line:
            return "completed"

        if "process has died" in line:
            return "restart_no_prep"

        if "std::bad_alloc" in line:
            return "restart_no_prep"
        
        if "Timeout is enabled:" in line:
            return "restart_no_prep"

        # if "MPC failed" in line:
        #     consecutive_mpc_failed += 1
        #     if consecutive_mpc_failed >= 5:
        #         return "restart_no_prep"
        else:
            consecutive_mpc_failed = 0

    return "exited"


def _start_roslaunch() -> Tuple[subprocess.Popen[bytes], TextIO]:
    """roslaunch를 새로 실행합니다."""
    # PTY를 붙여 roslaunch가 실제 터미널처럼 stdout에 [INFO] 로그를 모두 남기도록 한다.
    master_fd, slave_fd = pty.openpty()
    proc = subprocess.Popen(
        ["bash", "-lc", ROS_LAUNCH_CMD],
        cwd=str(WORKSPACE),
        stdout=slave_fd,
        stderr=slave_fd,
        stdin=subprocess.DEVNULL,
        text=False,
        bufsize=0,
    )
    os.close(slave_fd)
    master_file = os.fdopen(master_fd, "rb", buffering=0)
    stream = io.TextIOWrapper(
        master_file,
        encoding="utf-8",
        errors="replace",
        line_buffering=True,
    )
    return proc, stream


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="roslaunch 반복 실행 자동화 (기본: 무한 반복)"
    )
    parser.add_argument(
        "-n",
        "--iterations",
        type=int,
        default=None,
        help="실험을 반복할 횟수 (미지정 시 무한 반복)",
    )
    parser.add_argument(
        "-c",
        "--completed-target",
        type=int,
        default=None,
        help='"Completed." 이벤트가 지정 횟수에 도달하면 종료',
    )
    return parser.parse_args()


def main(max_runs: Optional[int], completed_target: Optional[int]) -> int:
    """사용자는 Ctrl+C로 전체 자동화를 중단할 수 있습니다."""
    iteration = 1
    completed_count = 0
    try:
        while max_runs is None or iteration <= max_runs:
            print(f"\n[auto] roslaunch 시작 (loop #{iteration})")
            proc, ros_stdout = _start_roslaunch()
            result: Optional[Result] = None
            try:
                result = _monitor_roslaunch(ros_stdout)
            finally:
                try:
                    _stop_process(proc)
                finally:
                    ros_stdout.close()

            if result == "completed":
                print("[auto] Completed. 다음 실험을 준비합니다...")
                _run_set_next_iterations()
                _kill_ros_processes()
                completed_count += 1
                if completed_target is not None:
                    print(
                        f"[auto] 누적 completed 횟수: {completed_count}/{completed_target}"
                    )
                    if completed_count >= completed_target:
                        print("[auto] 지정된 completed 목표에 도달하여 종료합니다.")
                        break
                iteration += 1
                print(f"[auto] 재시작 대기 {RESTART_DELAY_SEC}초...")
                time.sleep(RESTART_DELAY_SEC)
                continue

            if result == "restart_no_prep":
                print("[auto] 오류 패턴 감지: 설정 변경 없이 roslaunch를 다시 시작합니다.")
                _kill_ros_processes()
                iteration += 1
                print(f"[auto] 재시작 대기 {RESTART_DELAY_SEC}초...")
                time.sleep(RESTART_DELAY_SEC)
                continue

            print("[auto] roslaunch가 종료되었습니다. 스크립트를 마칩니다.")
            break
        else:
            print(f"[auto] 지정한 반복 횟수({max_runs})를 완료했습니다.")
    except KeyboardInterrupt:
        print("\n[auto] 사용자 종료 신호를 받아 종료합니다.")
    return 0


if __name__ == "__main__":
    args = _parse_args()
    sys.exit(main(args.iterations, args.completed_target))
