"""The grasp types this hand can actually make.

One entry per way of arranging the fingers, each a spread pair plus how far it
may close. The closure ceilings come from reach_table.py rather than from
judgement: past them the fingers would run into each other, and the servo would
have to veto the goal. Staying inside the table means that never happens.

Nothing below the decision layer knows these exist. Swap this file for a learned
policy's output and the rest of the stack does not change.
"""

from dataclasses import dataclass
import math

from gripper_grasp.reach_table import FLEXION_LOWER, max_closure

JOINTS = 8
SPREAD_INDEX = 6      # A_1_Joint
SPREAD_MIDDLE = 7     # B_1_Joint


@dataclass(frozen=True)
class Grasp:
    key: str
    name: str
    index_spread: float        # degrees
    middle_spread: float
    note: str

    @property
    def closure_limit(self):
        """Smaller of the two fingers' limits: both have to fit."""
        return min(max_closure(self.index_spread), max_closure(self.middle_spread))

    def pose(self, closure):
        """Joint vector for this grasp closed by `closure` of its usable range."""
        fraction = min(max(closure, 0.0), 1.0) * self.closure_limit
        q = [fraction * lower for lower in FLEXION_LOWER]
        q.append(math.radians(self.index_spread))
        q.append(math.radians(self.middle_spread))
        return q


# Tripod and pinch were found geometrically and confirmed on the hardware; the
# tripod is not symmetric-looking on paper but ±52 is what actually puts the
# three tips evenly around the palm axis.
GRASPS = [
    Grasp('1', 'tripod', +52.0, -52.0,
          'three tips evenly spaced, for round objects'),
    Grasp('2', 'pinch', +90.0, -90.0,
          'index and middle together against the thumb, for rims'),
    Grasp('3', 'wide', 0.0, 0.0,
          'fingers apart, widest opening'),
]

BY_KEY = {g.key: g for g in GRASPS}


def open_pose(grasp):
    return grasp.pose(0.0)
