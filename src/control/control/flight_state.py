"""
Flight state machine definition for BoilerHawk drone control.

Separated from control_node.py so it can be tested independently
on any platform (no ROS 2 / pymavlink dependency).
"""


class FlightState:
    """Enumeration of drone flight states."""
    IDLE = 'IDLE'
    SETTING_GUIDED = 'SETTING_GUIDED'
    ARMING = 'ARMING'
    TAKING_OFF = 'TAKING_OFF'
    HOVERING = 'HOVERING'
    FLYING = 'FLYING'
    LANDING = 'LANDING'
    LANDED = 'LANDED'
    EMERGENCY_HOLD = 'EMERGENCY_HOLD'

    # Valid transitions: from_state -> set of valid to_states
    TRANSITIONS = {
        IDLE:            {SETTING_GUIDED},
        SETTING_GUIDED:  {ARMING, IDLE},
        ARMING:          {TAKING_OFF, IDLE},
        TAKING_OFF:      {HOVERING, LANDING, IDLE},
        HOVERING:        {FLYING, LANDING, EMERGENCY_HOLD, IDLE},
        FLYING:          {HOVERING, LANDING, EMERGENCY_HOLD, IDLE},
        LANDING:         {LANDED, IDLE},
        LANDED:          {IDLE, SETTING_GUIDED},
        EMERGENCY_HOLD:  {HOVERING, FLYING, LANDING, IDLE},
    }

    @staticmethod
    def is_valid_transition(from_state: str, to_state: str) -> bool:
        """Check if a transition is valid."""
        valid = FlightState.TRANSITIONS.get(from_state, set())
        return to_state in valid
