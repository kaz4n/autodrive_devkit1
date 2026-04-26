from __future__ import annotations

from .autonomy_node_v2 import RoboRacerAutonomyNodeV2, main as _main_v2

RoboRacerAutonomyNode = RoboRacerAutonomyNodeV2


def main(args=None) -> None:
	_main_v2(args)
