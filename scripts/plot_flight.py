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

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(df["Time_s"], df["Altitude_m"], label="altitude", color="C0")
    ax.plot(df["Time_s"], df["Target_m"], "--", label="target", color="C1")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("AeroStabilize-1D — altitude vs time")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
