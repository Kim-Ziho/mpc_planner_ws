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


INCREMENT = 5
SETTINGS_PATH = Path(
    "/workspace/src/mpc_planner/mpc_planner_rosnavigation/config/settings.yaml"
)
CONFIG_PATH = Path(
    "/workspace/src/pedestrian_simulator/config/configuration.yaml"
)


def _bump_value(path: Path, pattern: re.Pattern) -> Tuple[int, int]:
    """Load file, increment the matched integer, and write the update back."""
    text = path.read_text()
    match = pattern.search(text)
    if not match:
        raise RuntimeError(f"Could not locate target pattern in {path}")

    current = int(match.group("value"))
    updated = current + INCREMENT

    new_text = (
        text[: match.start("value")] + str(updated) + text[match.end("value") :]
    )
    path.write_text(new_text)
    return current, updated


def main() -> None:
    settings_pattern = re.compile(r'(file:\s*exp1-)(?P<value>\d+)')
    config_pattern = re.compile(r'(single_scenario:\s*)(?P<value>-?\d+)')

    settings_old, settings_new = _bump_value(SETTINGS_PATH, settings_pattern)
    config_old, config_new = _bump_value(CONFIG_PATH, config_pattern)

    print(
        f"settings.yaml file: exp1-{settings_old} -> exp1-{settings_new}\n"
        f"configuration.yaml single_scenario: {config_old} -> {config_new}"
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # pragma: no cover - surfaced to CLI
        print(f"Failed to update experiment settings: {exc}", file=sys.stderr)
        sys.exit(1)
