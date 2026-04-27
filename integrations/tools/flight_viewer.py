"""Live 2D flight-path viewer — the closest we get to "rendering" a GPS
test flight without Isaac Sim installed.

Reads sim_loop's ground-truth state file at ~10 Hz, plots the drone's
N/E trajectory in a top-down view alongside mission_demo's expected
waypoints (SW, SE, NE, NW corners + CENTER) so you can see the X-pattern
trace out as it flies.

Run as a fourth terminal alongside the sim stack:

  T1>  docker compose -f sitl/docker-compose.yml up -d
  T2>  python -m integrations.tools.sim_loop --motor-port 35000
  T3>  python -m integrations.tools.bf_gps_shim
  T4>  python -m integrations.tools.flight_viewer  &      # ← THIS
  T5>  python -m integrations.tools.mission_demo --bf-port 5762

The viewer also draws an altitude trace over time in a side panel and a
text overlay with current N/E/alt/yaw + the most recent mission phase
(if it can read mission_demo's stdout via the optional --mission-log).

What it's not: a 3D render, a full physics visualization, or a
PX4/QGC-style ground station. It's a fast feedback loop for "did the
drone go where mission_demo asked it to."
"""
from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle


DEFAULT_STATE_FILE = os.path.join(tempfile.gettempdir(), "bf_sim_state.txt")


@dataclass
class Waypoint:
    label: str
    n_m: float
    e_m: float


def build_corners(half: float) -> list[Waypoint]:
    """Match mission_demo's build_corners() — same labels and offsets so
    the plot lines up with the script's actual targets."""
    return [
        Waypoint("SW", -half, -half),
        Waypoint("SE", -half, +half),
        Waypoint("NE", +half, +half),
        Waypoint("NW", +half, -half),
        Waypoint("CENTER", 0.0, 0.0),
    ]


class FlightViewer:
    def __init__(self, state_file: str, half_size: float, max_history: int):
        self.state_file = state_file
        self.half = half_size
        self.max_history = max_history
        self.t_history: list[float] = []
        self.n_history: list[float] = []
        self.e_history: list[float] = []
        self.alt_history: list[float] = []
        self.yaw_history: list[float] = []
        self.t0 = time.monotonic()
        self.last_mtime = 0.0

        self.fig, (self.ax_xy, self.ax_alt) = plt.subplots(
            1, 2, figsize=(14, 7),
            gridspec_kw={"width_ratios": [3, 2]},
        )
        self.fig.canvas.manager.set_window_title(
            "Project_Beta_Ardu — flight_viewer"
        )
        self._draw_layout()

    def _draw_layout(self) -> None:
        # Top-down N/E plot.
        h = self.half
        self.ax_xy.set_xlim(-h * 1.4, h * 1.4)
        self.ax_xy.set_ylim(-h * 1.4, h * 1.4)
        self.ax_xy.set_aspect("equal")
        self.ax_xy.set_xlabel("East (m)")
        self.ax_xy.set_ylabel("North (m)")
        self.ax_xy.set_title("Top-down trajectory")
        self.ax_xy.grid(True, alpha=0.3)
        # Map bounding box.
        self.ax_xy.add_patch(Rectangle(
            (-h, -h), 2 * h, 2 * h,
            fill=False, edgecolor="gray", linestyle="--", alpha=0.5,
        ))
        # Waypoints.
        for w in build_corners(self.half):
            self.ax_xy.plot(w.e_m, w.n_m, "x", color="C1", markersize=12,
                            markeredgewidth=2)
            self.ax_xy.annotate(
                w.label, (w.e_m, w.n_m), textcoords="offset points",
                xytext=(8, 8), fontsize=9, color="C1",
            )
        # X-pattern path (SW → NE → SE → NW).
        x_pattern = [(-h, -h), (+h, +h), (+h, -h), (-h, +h), (0, 0)]
        xs = [e for n, e in x_pattern]
        ys = [n for n, e in x_pattern]
        self.ax_xy.plot(xs, ys, ":", color="C1", alpha=0.4, label="X-pattern")
        # Drone trajectory + current position (populated each frame).
        self.line_traj, = self.ax_xy.plot([], [], "-", color="C0",
                                          alpha=0.7, label="Trajectory")
        self.dot_drone, = self.ax_xy.plot([], [], "o", color="C0",
                                          markersize=10, label="Iris")
        # Heading indicator (small line out of dot).
        self.line_heading, = self.ax_xy.plot([], [], "-", color="C0",
                                             linewidth=2, alpha=0.8)
        self.ax_xy.legend(loc="upper left", fontsize=8)

        # Altitude over time.
        self.ax_alt.set_xlabel("Time (s)")
        self.ax_alt.set_ylabel("Altitude (m, AGL)")
        self.ax_alt.set_title("Altitude trace")
        self.ax_alt.grid(True, alpha=0.3)
        self.line_alt, = self.ax_alt.plot([], [], "-", color="C2")
        # Status text overlay.
        self.status = self.ax_alt.text(
            0.02, 0.98, "", transform=self.ax_alt.transAxes,
            verticalalignment="top", fontfamily="monospace", fontsize=9,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
        )

    def _read_state(self) -> tuple[float, float, float, float] | None:
        try:
            mtime = os.path.getmtime(self.state_file)
            if mtime == self.last_mtime:
                return None
            self.last_mtime = mtime
            with open(self.state_file, "r") as f:
                parts = f.read().strip().split()
            if len(parts) >= 4:
                return (float(parts[0]), float(parts[1]),
                        float(parts[2]), float(parts[3]))
        except (OSError, ValueError):
            pass
        return None

    def _update(self, _frame_idx: int):
        s = self._read_state()
        if s is not None:
            n, e, alt, yaw = s
            self.t_history.append(time.monotonic() - self.t0)
            self.n_history.append(n)
            self.e_history.append(e)
            self.alt_history.append(alt)
            self.yaw_history.append(yaw)
            # Trim to max_history points (keeps memory bounded over long runs).
            if len(self.t_history) > self.max_history:
                k = len(self.t_history) - self.max_history
                self.t_history = self.t_history[k:]
                self.n_history = self.n_history[k:]
                self.e_history = self.e_history[k:]
                self.alt_history = self.alt_history[k:]
                self.yaw_history = self.yaw_history[k:]

        if self.n_history:
            self.line_traj.set_data(self.e_history, self.n_history)
            self.dot_drone.set_data([self.e_history[-1]], [self.n_history[-1]])
            # 5 m heading vector
            import math
            yaw_rad = math.radians(self.yaw_history[-1])
            hn = self.n_history[-1] + 5.0 * math.cos(yaw_rad)
            he = self.e_history[-1] + 5.0 * math.sin(yaw_rad)
            self.line_heading.set_data(
                [self.e_history[-1], he], [self.n_history[-1], hn],
            )
            self.line_alt.set_data(self.t_history, self.alt_history)
            # Auto-scale altitude axis.
            if self.alt_history:
                lo = min(self.alt_history) - 2
                hi = max(self.alt_history) + 2
                if hi - lo < 5:
                    hi = lo + 5
                self.ax_alt.set_ylim(lo, hi)
            if self.t_history:
                self.ax_alt.set_xlim(0, max(30.0, self.t_history[-1] + 2))

            self.status.set_text(
                f"N={self.n_history[-1]:+7.2f}m\n"
                f"E={self.e_history[-1]:+7.2f}m\n"
                f"alt={self.alt_history[-1]:6.2f}m\n"
                f"yaw={self.yaw_history[-1]:+6.1f}°\n"
                f"t={self.t_history[-1]:6.1f}s"
            )
        else:
            self.status.set_text(
                f"Waiting for {self.state_file}\n"
                f"(start sim_loop in another terminal)"
            )
        return (self.line_traj, self.dot_drone, self.line_heading,
                self.line_alt, self.status)

    def run(self) -> None:
        # blit=False because the status text needs full redraws to update
        # cleanly; perf is fine at 10 Hz.
        anim = animation.FuncAnimation(
            self.fig, self._update, interval=100, blit=False,
            cache_frame_data=False,
        )
        plt.tight_layout()
        plt.show()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--state-file", default=DEFAULT_STATE_FILE,
                   help=f"Path sim_loop publishes to (default: {DEFAULT_STATE_FILE})")
    p.add_argument("--map-half-size", type=float, default=30.0,
                   help="Half-side of mission_demo's square map (m). "
                        "Match mission_demo's --map-half-size to align "
                        "the waypoint markers with the actual targets.")
    p.add_argument("--max-history", type=int, default=10000,
                   help="Max trajectory samples to keep (10 Hz × 1000 s).")
    args = p.parse_args()

    print(f"[viewer] reading {args.state_file}, "
          f"map half-size {args.map_half_size}m", flush=True)
    viewer = FlightViewer(args.state_file, args.map_half_size, args.max_history)
    viewer.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
