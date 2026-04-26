"""Compatibility shim.

Nominal planning and control now live together inside free_space_mpc.FreeSpaceMPC.
"""

from .free_space_mpc import FreeSpaceMPC

__all__ = ['FreeSpaceMPC']
