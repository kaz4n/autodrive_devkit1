"""Deprecated compatibility shim.

The old mission manager / safety FSM has been removed from the nominal stack. Safety is now
handled directly inside autonomy_node.py and by the MPC failure fallback.
"""

from __future__ import annotations

from .models import ControlCommand


class SafetyMonitor:
    def __init__(self, *args, **kwargs) -> None:
        pass

    def precheck(self, *args, **kwargs):
        return None

    def postcheck(self, command: ControlCommand, hold_steering: float) -> ControlCommand:
        return command
