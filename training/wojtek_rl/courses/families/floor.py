"""Floor-friction scenarios: geometry and speed stay nominal, only the
contact friction changes (applied to floor AND feet -- see
rollout.friction_geom_ids for why the floor alone would be a no-op).

Two directions off the model's mu=0.9. The slippery rows (0.4) are the
original pair. The sticky rows go the other way, because the robot's
failures on carpet and rough concrete were never measured in sim: 1.5 is
the realistic ceiling for rubber on such floors, 2.5 is a stress level. A
gait that shuffles or skates instead of lifting its feet is what breaks
when the foot can no longer slide, and a pivot is the motion that needs
sliding most -- hence the sticky spin row.
"""

from wojtek_rl.courses.families.geometry_paths import STRAIGHT_10M, circle
from wojtek_rl.courses.families.spin import SPIN_NOMINAL
from wojtek_rl.courses.spec import NOMINAL_SPEED as NOM
from wojtek_rl.courses.spec import (
    SLIPPERY_FRICTION,
    STICKY_FRICTION,
    STICKY_FRICTION_HI,
    Course,
    SpinCourse,
)

COURSES = [
    Course(
        "straight_slippery",
        f"straight on mu={SLIPPERY_FRICTION}",
        STRAIGHT_10M,
        (NOM,),
        friction=SLIPPERY_FRICTION,
    ),
    Course(
        "circle_r1_slippery",
        f"turning on mu={SLIPPERY_FRICTION}",
        circle(1.0),
        (NOM,),
        friction=SLIPPERY_FRICTION,
    ),
    Course(
        "straight_sticky",
        f"straight on mu={STICKY_FRICTION} (rough concrete, carpet)",
        STRAIGHT_10M,
        (NOM,),
        friction=STICKY_FRICTION,
    ),
    Course(
        "circle_r1_sticky",
        f"turning on mu={STICKY_FRICTION}",
        circle(1.0),
        (NOM,),
        friction=STICKY_FRICTION,
    ),
    Course(
        "straight_sticky_hi",
        f"straight on mu={STICKY_FRICTION_HI} (stress, past physical)",
        STRAIGHT_10M,
        (NOM,),
        friction=STICKY_FRICTION_HI,
    ),
    Course(
        "circle_r1_sticky_hi",
        f"turning on mu={STICKY_FRICTION_HI}",
        circle(1.0),
        (NOM,),
        friction=STICKY_FRICTION_HI,
    ),
    SpinCourse(
        "spin_left_sticky",
        f"pure spin CCW at {SPIN_NOMINAL} rad/s on mu={STICKY_FRICTION}",
        wz=+SPIN_NOMINAL,
        friction=STICKY_FRICTION,
    ),
]
