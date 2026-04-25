"""State machine for the companion.

States:
- IDLE: silent. RX values flow to BF unchanged.
- CLIMB: companion-active AUX high, climb to target altitude.
- TRANSIT: at altitude, fly to waypoint.
- HOLD: arrived. Centered sticks + hover throttle. Re-engages TRANSIT if
        the drone drifts past `arrival_radius_m * HOLD_RECHECK_HYSTERESIS`
        (wind drift).
- RELEASED: safety violated. Sticky until AUX cycled to low (operator confirms).

Companion sends MSP_SET_RAW_RC only in CLIMB/TRANSIT/HOLD (is_active()==True).
In any other state it stays silent and BF reverts to RX values within the
MSP-override timeout (~500ms on BF 4.5+). That timeout is the load-bearing
safety primitive — verify on your BF version (BF issue #13374).
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

# Multiplier on arrival_radius_m. If a drone in HOLD drifts past this it
# re-enters TRANSIT for active correction. >1.0 provides spatial hysteresis
# so a hovering drone doesn't ping-pong between HOLD and TRANSIT.
HOLD_RECHECK_HYSTERESIS = 1.5


class State(Enum):
    IDLE = auto()
    CLIMB = auto()
    TRANSIT = auto()
    HOLD = auto()
    RELEASED = auto()


@dataclass
class StateContext:
    state: State = State.IDLE
    arrival_dwell_start: float = 0.0
    last_transition: float = 0.0


def step(
    ctx: StateContext,
    *,
    now: float,
    aux_companion_active: bool,
    safety_ok: bool,
    current_alt_m: float,
    distance_to_target_m: float,
    climb_target_alt_m: float,
    arrival_radius_m: float,
    arrival_dwell_s: float,
) -> State:
    prev = ctx.state

    if prev == State.RELEASED:
        nxt = State.IDLE if not aux_companion_active else State.RELEASED
    elif not safety_ok:
        nxt = State.RELEASED
    elif not aux_companion_active:
        nxt = State.IDLE
    elif prev == State.IDLE:
        nxt = State.CLIMB
    elif prev == State.CLIMB:
        nxt = State.TRANSIT if current_alt_m >= climb_target_alt_m else State.CLIMB
    elif prev == State.TRANSIT:
        if distance_to_target_m < arrival_radius_m:
            if ctx.arrival_dwell_start == 0.0:
                ctx.arrival_dwell_start = now
                nxt = State.TRANSIT
            elif now - ctx.arrival_dwell_start >= arrival_dwell_s:
                nxt = State.HOLD
            else:
                nxt = State.TRANSIT
        else:
            ctx.arrival_dwell_start = 0.0
            nxt = State.TRANSIT
    elif prev == State.HOLD:
        # Wind drift: if we've drifted out of the hold disc, re-engage TRANSIT
        # for active correction. Hysteresis (>1× radius) prevents ping-pong.
        if distance_to_target_m > arrival_radius_m * HOLD_RECHECK_HYSTERESIS:
            ctx.arrival_dwell_start = 0.0
            nxt = State.TRANSIT
        else:
            nxt = State.HOLD
    else:
        nxt = State.HOLD

    if nxt != prev:
        ctx.last_transition = now
        if nxt != State.TRANSIT:
            ctx.arrival_dwell_start = 0.0
    ctx.state = nxt
    return nxt


def is_active(s: State) -> bool:
    return s in (State.CLIMB, State.TRANSIT, State.HOLD)
