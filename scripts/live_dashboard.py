"""Stream telemetry from AeroStabilize-1D with Matplotlib (polls the CSV on each frame).

The simulator must flush the CSV after each row so the plot can read partial runs.

From repo root (with matplotlib and pandas available):
  python scripts/live_dashboard.py
  python scripts/live_dashboard.py build/flight_data.csv
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation

# Columns the dashboard expects; missing or unreadable data keeps the last good frame.
_REQUIRED = ("Time_s", "Altitude_m", "SensedAlt_m", "Target_m", "Thrust_N")


def _read_telemetry(path: Path) -> pd.DataFrame | None:
    if not path.is_file():
        return None
    try:
        if path.stat().st_size == 0:
            return None
    except OSError:
        return None
    try:
        df = pd.read_csv(path, on_bad_lines="skip")
    except (OSError, ValueError):
        return None
    if _REQUIRED[0] not in df.columns:
        return None
    return df


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live Matplotlib view of a growing AeroStabilize-1D telemetry CSV.",
    )
    parser.add_argument(
        "csv",
        nargs="?",
        type=Path,
        default=Path("flight_data.csv"),
        help="Path to the telemetry file (default: flight_data.csv in the working directory)",
    )
    parser.add_argument(
        "--interval-ms",
        type=int,
        default=100,
        help="FuncAnimation update interval in milliseconds (default: 100)",
    )
    args = parser.parse_args()
    path = args.csv
    if not path.is_absolute():
        path = Path(path).resolve()

    fig, (ax_t, ax_b) = plt.subplots(2, 1, sharex=True, figsize=(9, 6.5))
    (line_alt,) = ax_t.plot([], [], "b-", label="Altitude_m")
    (line_sen,) = ax_t.plot(
        [],
        [],
        color="tab:orange",
        alpha=0.7,
        label="SensedAlt_m",
    )
    (line_tar,) = ax_t.plot([], [], color="0.2", linestyle="--", label="Target_m")
    (line_thr,) = ax_b.plot([], [], "r-", label="Thrust_N")

    ax_t.set_ylabel("Altitude (m)")
    ax_t.set_title("AeroStabilize-1D — live telemetry")
    ax_t.grid(True, alpha=0.3)
    ax_t.legend(loc="upper right")

    ax_b.set_xlabel("Time (s)")
    ax_b.set_ylabel("Thrust (N)")
    ax_b.grid(True, alpha=0.3)
    ax_b.legend(loc="upper right")

    def on_frame(_frame: int) -> None:
        df = _read_telemetry(path)
        if df is None or df.empty or not all(c in df.columns for c in _REQUIRED):
            return
        t = df["Time_s"]
        if len(t) == 0:
            return
        line_alt.set_data(t, df["Altitude_m"])
        line_sen.set_data(t, df["SensedAlt_m"])
        line_tar.set_data(t, df["Target_m"])
        line_thr.set_data(t, df["Thrust_N"])
        ax_t.relim()
        ax_t.autoscale_view()
        ax_b.relim()
        ax_b.autoscale_view()

    # Keep a reference: some backends stop the loop if the animation is collected.
    _animation = FuncAnimation(
        fig, on_frame, interval=args.interval_ms, blit=False, cache_frame_data=False
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
