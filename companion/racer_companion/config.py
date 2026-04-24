"""Config loader. JSON file with companion / nav / safety / waypoint sections."""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

from .nav import NavTuning, Waypoint
from .safety import SafetyConfig


@dataclass
class CompanionConfig:
    serial_port: str = "/dev/ttyAMA0"
    baud: int = 115200
    loop_hz: int = 50
    telemetry_hz: int = 10
    aux_channel_index: int = 6
    aux_active_us_min: int = 1700
    climb_target_alt_m: float = 5.0
    arrival_dwell_s: float = 2.0
    log_path: str = ""


@dataclass
class FullConfig:
    companion: CompanionConfig
    nav: NavTuning
    safety: SafetyConfig
    waypoint: Waypoint


def load(path: str | Path) -> FullConfig:
    raw = json.loads(Path(path).read_text())
    return FullConfig(
        companion=CompanionConfig(**raw.get("companion", {})),
        nav=NavTuning(**raw.get("nav", {})),
        safety=SafetyConfig(**raw.get("safety", {})),
        waypoint=Waypoint(**raw["waypoint"]),
    )


def dump_example(path: str | Path) -> None:
    cfg = FullConfig(
        companion=CompanionConfig(),
        nav=NavTuning(),
        safety=SafetyConfig(),
        waypoint=Waypoint(lat_deg=37.7749, lon_deg=-122.4194, alt_m=5.0),
    )
    Path(path).write_text(json.dumps({
        "companion": asdict(cfg.companion),
        "nav": asdict(cfg.nav),
        "safety": asdict(cfg.safety),
        "waypoint": asdict(cfg.waypoint),
    }, indent=2))
