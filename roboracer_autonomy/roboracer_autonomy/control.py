"""Compatibility shim.

The old low-level controller has been retired. Nominal control is generated directly by the
free-space MPC solver.
"""

from .free_space_mpc import FreeSpaceMPC

__all__ = ['FreeSpaceMPC']
