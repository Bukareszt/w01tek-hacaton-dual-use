# One detection from the targeting detector, consumed by targeting_controller.
#
# The detector publishes this and nothing else -- it never commands a motor.
# The stamp is mandatory, not decorative: targeting_controller rejects any
# target older than target_timeout_s, and a zero stamp reads as infinitely
# stale, which is the safe way for it to fail.

std_msgs/Header header

bool detected
string class_name

# Bounding-box centre, normalized to -1..1 across the image: -1 is the left
# (top) edge, +1 the right (bottom) edge, 0 the centre of the image. Normalized
# rather than pixels so a detector that runs on a downscaled frame does not
# have to know the capture geometry to be understood.
float32 target_x
float32 target_y

# 0.0 when the detector has no depth estimate. Carried for a future range-aware
# aim; the controller does not use it today -- camera/laser parallax is
# absorbed by the calibrated pan_offset_deg/tilt_offset_deg instead.
float32 distance_m

float32 confidence
