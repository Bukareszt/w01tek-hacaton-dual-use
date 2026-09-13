"""The simulation scene with its props (config/scene_sim.xml).

Two things have to keep holding, and neither is obvious from reading the
XML:

* the props must not add a single number to the generalized position. The
  physics plugin publishes /sim/qpos and the camera node copies it into its
  own model; the two agree only because every prop is a static body.
* the pictures the signs and the dock wear have to survive the trip through
  the staging directory, which is what the camera node and the plugin both
  hand to MuJoCo. Staging used to take the XMLs and leave everything else
  behind.
* the harbour the props now stand in is scenery. The bodies the robot is
  meant to bump into collide; the water, the quay slab and the boat must
  not, or the robot trips over the view.

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
PROPS = (
    "prop_ball",
    "prop_hydrant",
    "prop_traffic_light",
    "prop_stop_sign",
    "prop_clock",
    "prop_person",
)
# Where each prop stands. These numbers are not private to the scene: the
# RAI places file, the panel's detection range table in ros/README.md and
# the follow experiment all name the same spots, so dressing the world
# around the props may not nudge one of them.
PROP_POS = {
    "prop_ball": (1.60, 0.58, 0.0),
    "prop_hydrant": (2.10, -1.30, 0.0),
    "prop_traffic_light": (0.90, -3.10, 0.0),
    "prop_stop_sign": (1.72, 2.46, 0.0),
    "prop_clock": (-2.35, 0.86, 0.0),
    "prop_person": (6.48, 0.57, 0.0),
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


def test_props_leave_the_generalized_position_alone(staged):
    empty = _load(staged, "scene_mjx.xml")
    furnished = _load(staged, "scene_sim.xml")
    assert (furnished.nq, furnished.nv, furnished.nu) == (
        empty.nq, empty.nv, empty.nu
    )


def test_every_prop_is_there(staged):
    model = _load(staged, "scene_sim.xml")
    for name in PROPS:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0, name


def test_the_signs_wear_their_pictures(staged):
    """A file texture that fails to load is a compile error, so getting this
    far is most of the test; the names say both pictures arrived."""
    model = _load(staged, "scene_sim.xml")
    names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TEXTURE, i)
        for i in range(model.ntex)
    }
    assert {"stop_face", "clock_face"} <= names


def test_the_robot_can_walk_into_the_props(staged):
    """Each prop needs a geom that collides, or the robot walks through it."""
    model = _load(staged, "scene_sim.xml")
    for name in PROPS:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        first = model.body_geomadr[body]
        geoms = range(first, first + model.body_geomnum[body])
        assert any(model.geom_contype[g] for g in geoms), name


def test_the_props_have_not_moved(staged):
    """The dock is dressing. Every prop keeps the spot the detector, the RAI
    places file and the follow experiment were measured against."""
    model = _load(staged, "scene_sim.xml")
    for name, pos in PROP_POS.items():
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert body >= 0, name
        assert tuple(model.body_pos[body]) == pytest.approx(pos, abs=1e-6), name


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
    for picture in ("stop_sign.png",) + DOCK_PICTURES:
        assert (props / picture).is_file(), picture
    # And the staged copy still compiles, which is the whole point of it.
    assert mujoco.MjModel.from_xml_path(str(scene)).ngeom > 0
