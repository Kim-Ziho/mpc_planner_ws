#!/usr/bin/env python3
"""
Convenience script that bumps the experiment identifiers used by ROS navigation
launches. It increments both the recording name in settings.yaml and the
single_scenario value in the pedestrian simulator configuration so the next set
of runs uses the following five experiments.
"""
from pathlib import Path
from typing import Tuple
import re
import sys


SETTINGS_PATH = Path(
    "/workspace/src/mpc_planner/mpc_planner_rosnavigation/config/settings.yaml"
)
CONFIG_PATH = Path(
    "/workspace/src/pedestrian_simulator/config/configuration.yaml"
)


def _load_recording_info(path: Path) -> Tuple[str, int]:
    """settings.yaml에서 recording/file 접두사와 num_experiments 값을 읽어옵니다."""
    file_value = None
    num_experiments = None
    in_recording = False
    recording_indent = 0
    for raw_line in path.read_text().splitlines():
        line = raw_line.rstrip()
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue

        indent = len(line) - len(line.lstrip())
        if not in_recording:
            if stripped == "recording:":
                in_recording = True
                recording_indent = indent
            continue

        if indent <= recording_indent:
            break

        content = line.split("#", 1)[0].strip()
        if not content:
            continue

        if content.startswith("file:"):
            file_value = content.split(":", 1)[1].strip().strip('"\'')
        elif content.startswith("num_experiments:"):
            num_experiments = int(content.split(":", 1)[1].strip())

        if file_value is not None and num_experiments is not None:
            break

    if file_value is None or num_experiments is None:
        raise RuntimeError("settings.yaml에서 recording/file 또는 num_experiments를 찾을 수 없습니다.")

    if "-" not in file_value:
        raise RuntimeError(
            f"recording/file 값('{file_value}')에서 '-' 구분자를 찾을 수 없습니다."
        )
    exp_name = file_value.split("-", 1)[0]
    return exp_name, num_experiments


def _bump_value(path: Path, pattern: re.Pattern, increment: int) -> Tuple[int, int]:
    """Load file, increment the matched integer, and write the update back."""
    text = path.read_text()
    match = pattern.search(text)
    if not match:
        raise RuntimeError(f"Could not locate target pattern in {path}")

    current = int(match.group("value"))
    updated = current + increment

    new_text = (
        text[: match.start("value")] + str(updated) + text[match.end("value") :]
    )
    path.write_text(new_text)
    return current, updated


def main() -> None:
    exp_name, increment = _load_recording_info(SETTINGS_PATH)
    settings_pattern = re.compile(
        rf'(file:\s*{re.escape(exp_name)}-)(?P<value>\d+)'
    )
    config_pattern = re.compile(r'(single_scenario:\s*)(?P<value>-?\d+)')

    settings_old, settings_new = _bump_value(SETTINGS_PATH, settings_pattern, increment)
    config_old, config_new = _bump_value(CONFIG_PATH, config_pattern, increment)

    print(
        f"settings.yaml file: {exp_name}-{settings_old} -> {exp_name}-{settings_new}\n"
        f"configuration.yaml single_scenario: {config_old} -> {config_new}"
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # pragma: no cover - surfaced to CLI
        print(f"Failed to update experiment settings: {exc}", file=sys.stderr)
        sys.exit(1)
