"""Plot flight_data.csv produced by AeroStabilize-1D.

Usage (from repo root, after activating a venv and pip install -r requirements.txt):
  python scripts/plot_flight.py
  python scripts/plot_flight.py build/flight_data.csv

The default path is flight_data.csv in the current working directory (often your
build folder if you run the .exe from there).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    p = argparse.ArgumentParser(description="Plot telemetry CSV from AeroStabilize-1D.")
    p.add_argument(
        "csv",
        nargs="?",
        default="flight_data.csv",
        type=Path,
        help="Path to CSV (default: ./flight_data.csv)",
    )
    args = p.parse_args()
    path = args.csv.resolve()
    if not path.is_file():
        raise SystemExit(f"File not found: {path}")

    df = pd.read_csv(path)

    _, ax = plt.subplots(figsize=(9, 5))
    ax.plot(df["Time_s"], df["Altitude_m"], label="altitude", color="C0")
    ax.plot(df["Time_s"], df["Target_m"], "--", label="target", color="C1")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("AeroStabilize-1D — altitude vs time (disturbance rejection)")

    if "Disturbance_N" in df.columns and df["Disturbance_N"].abs().max() > 0:
        ax2 = ax.twinx()
        ax2.step(df["Time_s"], df["Disturbance_N"], where="post", color="C2", alpha=0.85, label="disturbance")
        ax2.set_ylabel("Disturbance (N)")
        ax2.axhline(0.0, color="gray", linewidth=0.8, linestyle=":")
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right")
    else:
        ax.legend()

    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
