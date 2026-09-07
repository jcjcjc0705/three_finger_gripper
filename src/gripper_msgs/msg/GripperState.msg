# What the fingers are actually doing. Same joint order as GripperGoal.

uint8 IDLE=0
uint8 TRACKING=1
uint8 HOLDING=2
uint8 BLOCKED=3      # the goal needs a self-colliding pose; see reach

std_msgs/Header header

float32[8] position        # rad
float32[8] velocity        # rad/s
float32[8] current         # Present Current, motor register units

# True where the joint is pushing at its ceiling and not moving: contact, with
# an object, another finger or a hard stop. What that means is the caller's
# business -- deciding "did I grasp it" is a strategy question.
bool[8] stalled

uint8 status

# Holding because the goal aged out, not because it was reached. Both look like
# HOLDING, so without this a caller whose link dropped never learns it happened.
bool goal_stale

# How much of the distance still owed to the goal the collision model let
# through this cycle. It is not progress-so-far: once the fingers have advanced
# as far as they legally can this reads 0 while status stays BLOCKED, which is
# the signal that the goal itself is unreachable and needs rethinking.
float32 reach
