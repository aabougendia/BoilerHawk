"""Mission state definitions and FSM transition rules.

Copyright 2026 BoilerHawk — MIT License.
"""

from enum import Enum, auto
from typing import Dict, Set


class MissionState(Enum):
    """All possible states of the mission manager FSM."""

    IDLE = auto()
    PREFLIGHT = auto()
    TAKEOFF = auto()
    EXECUTING = auto()
    PAUSED = auto()
    RTL = auto()
    LANDING = auto()
    EMERGENCY = auto()


# Valid transitions: state -> set of reachable states
VALID_TRANSITIONS: Dict[MissionState, Set[MissionState]] = {
    MissionState.IDLE: {MissionState.PREFLIGHT},
    MissionState.PREFLIGHT: {MissionState.TAKEOFF, MissionState.IDLE},
    MissionState.TAKEOFF: {MissionState.EXECUTING, MissionState.EMERGENCY},
    MissionState.EXECUTING: {
        MissionState.PAUSED,
        MissionState.RTL,
        MissionState.LANDING,
        MissionState.EMERGENCY,
    },
    MissionState.PAUSED: {
        MissionState.EXECUTING,
        MissionState.RTL,
        MissionState.LANDING,
        MissionState.EMERGENCY,
    },
    MissionState.RTL: {MissionState.LANDING, MissionState.EMERGENCY},
    MissionState.LANDING: {MissionState.IDLE, MissionState.EMERGENCY},
    MissionState.EMERGENCY: {MissionState.IDLE},
}


def is_valid_transition(from_state: MissionState, to_state: MissionState) -> bool:
    """Check whether a state transition is allowed.

    EMERGENCY can be entered from any state.
    """
    if to_state == MissionState.EMERGENCY:
        return True
    return to_state in VALID_TRANSITIONS.get(from_state, set())


def get_valid_transitions(state: MissionState) -> Set[MissionState]:
    """Return the set of states reachable from *state*."""
    reachable = VALID_TRANSITIONS.get(state, set()).copy()
    if state != MissionState.EMERGENCY:
        reachable.add(MissionState.EMERGENCY)
    return reachable
