# What targeting_controller is doing and why. Published every control tick so
# a bag of a run explains itself without re-deriving the state machine from
# joint angles.

uint8 MODE_DISABLED=0
uint8 MODE_SEARCHING=1
uint8 MODE_TRACKING=2
uint8 MODE_LOCKED=3
uint8 MODE_FAULT=4

std_msgs/Header header

uint8 mode

# Whether a target arrived recently enough to act on. False in TRACKING for
# one tick is normal; false for longer is what drops back to SEARCHING.
bool target_fresh
bool locked

# What the controller asked the servos for, after limits, rate limiting and
# calibration offsets -- not the raw detector angle.
float32 requested_pan_deg
float32 requested_tilt_deg

# What the servos report back. Diverging from requested means the gimbal is
# slewing, stalled, or the bus is dropping writes.
float32 actual_pan_deg
float32 actual_tilt_deg

float32 detection_age_s

# Empty unless something is wrong. Set in FAULT, and also while the controller
# is held back for a recoverable reason (no camera intrinsics yet, say).
string fault_message
