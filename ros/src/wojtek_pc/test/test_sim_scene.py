"""The simulation scene with its cast (config/scene_sim.xml).

Three things have to keep holding, and none of them is obvious from reading
the XML:

* the cast must not add a single number to the generalized position. The
  physics plugin publishes /sim/qpos and the camera node copies it into its
  own model; the two agree only because every body in here is static.
* the pictures the dock wears have to survive the trip through the staging
  directory, which is what the camera node and the plugin both hand to
  MuJoCo. Staging used to take the XMLs and leave everything else behind.
* the harbour the cast stands in is scenery. The bodies the robot is meant
  to bump into collide; the water, the quay slab and the boat must not, or
  the robot trips over the view. The drones fly, so they collide with
  nothing at all.

Run inside the dev container, where mujoco and the ament index exist:

    docker exec wojtek_robot python3 -m pytest /ros2_ws/src/wojtek_pc/test -q
"""

import re
import shutil
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

mujoco = pytest.importorskip("mujoco")

CONFIG = PKG / "config"
MESHES = PKG.parent / "wojtek_description" / "meshes"
# The cast the detector works on. Dock workers wear the same orange hi-vis
# uniform and a helmet; the unauthorized people wear plain dark clothes and
# no helmet. Telling one from the other is the VLM's job -- to a COCO
# detector every one of them is just "person".
WORKERS = (
    "actor_worker_1",
    "actor_worker_2",
    "actor_worker_3",
)
INTRUDERS = (
    "actor_intruder_1",
    "actor_intruder_2",
)
# Visual only, and off the ground.
DRONES = (
    "actor_drone_1",
    "actor_drone_2",
    "actor_drone_3",
)
ACTORS = WORKERS + INTRUDERS + DRONES
# Where each of them stands or hovers. These numbers are not private to the
# scene: the RAI places file (nav/config/places_sim.yaml) names the same
# spots, so dressing the world around the cast may not nudge one of them.
# People are placed on the floor, so only x and y are pinned here; a drone's
# height is part of its spot.
ACTOR_POS = {
    "actor_worker_1": (6.5, 0.6),
    "actor_worker_2": (6.2, 3.5),
    "actor_worker_3": (8.0, -2.4),
    "actor_intruder_1": (9.5, 2.6),
    "actor_intruder_2": (6.0, -4.2),
    "actor_drone_1": (7.8, 1.1, 1.9),
    "actor_drone_2": (10.0, 2.8, 2.5),
    "actor_drone_3": (7.3, -3.2, 1.65),
}
# The harbour around them. Solid is what the robot can walk into; visual is
# the view, and it has to stay out of the collision world.
DOCK_SOLID = (
    "dock_kerb",
    "dock_bollard_1",
    "dock_bollard_2",
    "dock_bollard_3",
    "dock_bollard_4",
    "dock_container_1",
    "dock_container_2",
    "dock_container_3",
    "dock_crate_1",
    "dock_crate_2",
    "dock_crate_3",
)
DOCK_VISUAL = (
    "dock_quay",
    "dock_water",
    "dock_wall",
    "dock_boat",
    "dock_crane",
)
DOCK_PICTURES = (
    "quay_concrete.png",
    "container_wall.png",
    "water.png",
    "crate.png",
)


@pytest.fixture(scope="module")
def staged(tmp_path_factory):
    """config/ copied with meshdir pointed at the source-tree meshes.

    The same shape of copy the camera node and the plugin make at run time,
    props directory included.
    """
    out = tmp_path_factory.mktemp("wojtek_scene")
    for entry in CONFIG.iterdir():
        if entry.is_dir():
            shutil.copytree(entry, out / entry.name)
        elif entry.suffix == ".xml":
            (out / entry.name).write_text(
                re.sub(r'meshdir="[^"]*"', f'meshdir="{MESHES}"', entry.read_text())
            )
    return out


def _load(staged, name):
    return mujoco.MjModel.from_xml_path(str(staged / name))


def _geoms(model, name):
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert body >= 0, name
    first = model.body_geomadr[body]
    return range(first, first + model.body_geomnum[body])


def test_the_cast_leaves_the_generalized_position_alone(staged):
    empty = _load(staged, "scene_mjx.xml")
    furnished = _load(staged, "scene_sim.xml")
    assert (furnished.nq, furnished.nv, furnished.nu) == (
        empty.nq, empty.nv, empty.nu
    )


def test_the_whole_cast_is_there(staged):
    model = _load(staged, "scene_sim.xml")
    for name in ACTORS:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0, name


def test_the_robot_can_walk_into_the_people(staged):
    """Workers and intruders stand on the quay, so each of them needs a geom
    that collides, or the robot walks straight through a person."""
    model = _load(staged, "scene_sim.xml")
    for name in WORKERS + INTRUDERS:
        assert any(model.geom_contype[g] for g in _geoms(model, name)), name


def test_the_drones_are_only_a_picture(staged):
    """A drone hovers, and nothing holds it up. One colliding geom on one of
    them and the robot bumps into something the camera says is in the air."""
    model = _load(staged, "scene_sim.xml")
    for name in DRONES:
        for g in _geoms(model, name):
            assert not model.geom_contype[g], (name, g)
            assert not model.geom_conaffinity[g], (name, g)


def test_the_cast_has_not_moved(staged):
    """The dock is dressing. Everyone keeps the spot the RAI places file
    (nav/config/places_sim.yaml) was measured against."""
    model = _load(staged, "scene_sim.xml")
    for name, pos in ACTOR_POS.items():
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert body >= 0, name
        here = tuple(model.body_pos[body])[:len(pos)]
        assert here == pytest.approx(pos, abs=1e-6), name


def test_the_whole_dock_is_there(staged):
    model = _load(staged, "scene_sim.xml")
    for name in DOCK_SOLID + DOCK_VISUAL:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0, name


def test_the_robot_can_walk_into_the_dock(staged):
    """The kerb, the bollards, the containers and the crates are obstacles."""
    model = _load(staged, "scene_sim.xml")
    for name in DOCK_SOLID:
        assert any(model.geom_contype[g] for g in _geoms(model, name)), name


def test_the_robot_cannot_walk_into_the_view(staged):
    """The water, the quay slab, the wall, the boat and the crane are pictures.
    One collidable geom among them and the robot trips over the scenery or
    falls through the floor it is already standing on."""
    model = _load(staged, "scene_sim.xml")
    for name in DOCK_VISUAL:
        for g in _geoms(model, name):
            assert not model.geom_contype[g], (name, g)
            assert not model.geom_conaffinity[g], (name, g)


def test_staging_brings_the_props_along():
    """The camera node's own staging has to carry props/, not just XMLs."""
    pytest.importorskip("rclpy")
    from wojtek_pc.sim_camera_node import _staged_scene

    scene = Path(_staged_scene(str(CONFIG / "scene_sim.xml")))
    props = scene.parent / "props"
    for picture in DOCK_PICTURES:
        assert (props / picture).is_file(), picture
    # And the staged copy still compiles, which is the whole point of it.
    assert mujoco.MjModel.from_xml_path(str(scene)).ngeom > 0
