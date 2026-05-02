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
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation

# Every column used below must exist; partial or malformed rows are skipped by pandas instead of aborting.
_REQUIRED = (
    "Time_s",
    "TrueX",
    "TrueZ",
    "TargetX",
    "TargetZ",
    "SensedZ",
    "PitchDeg",
    "Thrust",
    "Torque",
)


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
    except (OSError, ValueError, PermissionError):
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

    fig, (ax_spatial, ax_alt, ax_act) = plt.subplots(
        3,
        1,
        figsize=(9, 10),
        layout="constrained",
        height_ratios=[1.15, 1.0, 1.0],
    )
    ax_torque = ax_act.twinx()

    (path_line,) = ax_spatial.plot([], [], "b-", linewidth=1.2, label="True path")
    (target_pt,) = ax_spatial.plot(
        [],
        [],
        linestyle="none",
        marker="x",
        color="red",
        markersize=11,
        markeredgewidth=2.2,
        label="Target",
    )
    (drone_seg,) = ax_spatial.plot([], [], "k-", linewidth=5.0, solid_capstyle="round", label="Body")

    (line_true_z,) = ax_alt.plot([], [], "b-", label="TrueZ")
    (line_tar_z,) = ax_alt.plot([], [], color="0.25", linestyle="--", label="TargetZ")
    (line_sen_z,) = ax_alt.plot([], [], color="tab:orange", alpha=0.85, label="SensedZ")

    (line_thr,) = ax_act.plot([], [], color="tab:red", label="Thrust")
    (line_tor,) = ax_torque.plot([], [], color="tab:blue", linestyle="-", label="Torque")

    ax_spatial.set_xlabel("TrueX (m)")
    ax_spatial.set_ylabel("TrueZ (m)")
    ax_spatial.set_title("AeroStabilize-1D — spatial (radar)")
    ax_spatial.grid(True, alpha=0.3)
    ax_spatial.legend(loc="upper right", fontsize=8)
    ax_spatial.set_aspect("equal", adjustable="datalim")

    ax_alt.set_ylabel("Altitude (m)")
    ax_alt.set_title("Altitude vs time")
    ax_alt.grid(True, alpha=0.3)
    ax_alt.legend(loc="upper right", fontsize=8)

    ax_act.set_xlabel("Time (s)")
    ax_act.set_ylabel("Thrust (N)", color="tab:red")
    ax_act.tick_params(axis="y", labelcolor="tab:red")
    ax_torque.set_ylabel("Torque (N·m)", color="tab:blue")
    ax_torque.tick_params(axis="y", labelcolor="tab:blue")
    ax_act.set_title("Actuators")
    ax_act.grid(True, alpha=0.3)

    h1, l1 = ax_act.get_legend_handles_labels()
    h2, l2 = ax_torque.get_legend_handles_labels()
    ax_act.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=8)

    ax_alt.sharex(ax_act)

    # Body segment half-length in meters; nose sits along thrust (+sin pitch, +cos pitch) per PhysicsEngine.
    body_half_m = 0.35

    def on_frame(_frame: int) -> None:
        df = _read_telemetry(path)
        if df is None or df.empty or not all(c in df.columns for c in _REQUIRED):
            return
        t = df["Time_s"]
        if len(t) == 0:
            return

        tx = df["TrueX"].to_numpy(dtype=float)
        tz = df["TrueZ"].to_numpy(dtype=float)
        path_line.set_data(tx, tz)

        row = df.iloc[-1]
        tgt_x, tgt_z = float(row["TargetX"]), float(row["TargetZ"])
        target_pt.set_data([tgt_x], [tgt_z])

        cx, cz = float(row["TrueX"]), float(row["TrueZ"])
        pitch_deg = float(row["PitchDeg"])
        pr = np.deg2rad(pitch_deg)
        dx = body_half_m * np.sin(pr)
        dz = body_half_m * np.cos(pr)
        drone_seg.set_data([cx - dx, cx + dx], [cz - dz, cz + dz])

        ax_spatial.relim()
        ax_spatial.autoscale_view()

        line_true_z.set_data(t, df["TrueZ"])
        line_tar_z.set_data(t, df["TargetZ"])
        line_sen_z.set_data(t, df["SensedZ"])
        ax_alt.relim()
        ax_alt.autoscale_view()

        line_thr.set_data(t, df["Thrust"])
        line_tor.set_data(t, df["Torque"])
        ax_act.relim()
        ax_act.autoscale_view()
        ax_torque.relim()
        ax_torque.autoscale_view()

    _animation = FuncAnimation(
        fig, on_frame, interval=args.interval_ms, blit=False, cache_frame_data=False
    )
    plt.show()


if __name__ == "__main__":
    main()
