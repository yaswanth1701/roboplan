"""
Unit tests for RRT planners in RoboPlan.
"""

from pathlib import Path

import numpy as np
import pytest

from roboplan.core import JointConfiguration, Scene, computePathLength
from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan.rrt import PoseConstraint, RRTOptions, RRT


@pytest.fixture
def test_scene() -> Scene:
    roboplan_models_dir = get_package_models_dir()
    urdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [get_package_share_dir()]

    return Scene("test_scene", urdf_path, srdf_path, package_paths)


def test_plan(test_scene: Scene) -> None:
    # Ensure determinism in the test.
    test_scene.setRngSeed(286)

    options = RRTOptions()
    options.group_name = "arm"
    options.max_connection_distance = 1.0
    options.collision_check_step_size = 0.05

    rrt = RRT(test_scene, options)
    rrt.setRngSeed(1234)

    start = JointConfiguration()
    start.positions = test_scene.randomCollisionFreePositions()
    assert start.positions is not None

    goal = JointConfiguration()
    goal.positions = test_scene.randomCollisionFreePositions()
    assert goal.positions is not None

    path = rrt.plan(start, goal)
    assert path is not None
    print(path)


def test_plan_rrt_star(test_scene: Scene) -> None:
    # Plan the same problem with and without RRT*. RRT* keeps rewiring and optimizing,
    # so its path must be equal or shorter than plain RRT.

    # Ensure determinism in the test.
    test_scene.setRngSeed(286)

    start = JointConfiguration()
    start.positions = test_scene.randomCollisionFreePositions()
    assert start.positions is not None

    goal = JointConfiguration()
    goal.positions = test_scene.randomCollisionFreePositions()
    assert goal.positions is not None

    def plan_with(rrt_star: bool):
        options = RRTOptions()
        options.group_name = "arm"
        options.max_connection_distance = 1.0
        options.collision_check_step_size = 0.05
        options.rrt_star = rrt_star
        options.rewire_distance = 2.0
        # Disable fast_return so RRT* optimizes, and bound the search by a fixed node budget (not a
        # wall-clock time budget). A node budget makes both runs do exactly the same amount of work
        # for the same seed, so the comparison is deterministic and independent of machine load.
        options.fast_return = False
        options.max_nodes = 500

        rrt = RRT(test_scene, options)
        rrt.setRngSeed(1234)
        return rrt.plan(start, goal)

    star_path = plan_with(rrt_star=True)
    rrt_path = plan_with(rrt_star=False)
    assert star_path is not None
    assert rrt_path is not None
    print(star_path)

    # RRT* must never produce a longer path than plain RRT.
    star_length = computePathLength(test_scene, "arm", star_path)
    rrt_length = computePathLength(test_scene, "arm", rrt_path)
    assert star_length <= rrt_length


def test_pose_constraint_fields() -> None:
    # The PoseConstraint struct should be constructible and its fields should round-trip.
    constraint = PoseConstraint()
    assert np.all(np.isneginf(constraint.min))
    assert np.all(np.isposinf(constraint.max))
    np.testing.assert_allclose(constraint.frame, np.eye(4))

    constraint.link_name = "tool0"
    constraint.min = np.array([-0.1, -0.1, -0.1, -0.2, -0.2, -0.2])
    constraint.max = np.array([0.1, 0.1, 0.1, 0.2, 0.2, 0.2])
    frame = np.eye(4)
    frame[:3, 3] = [0.4, 0.0, 0.3]
    constraint.frame = frame
    constraint.tolerence = 1e-2

    assert constraint.link_name == "tool0"
    np.testing.assert_allclose(constraint.min, [-0.1, -0.1, -0.1, -0.2, -0.2, -0.2])
    np.testing.assert_allclose(constraint.max, [0.1, 0.1, 0.1, 0.2, 0.2, 0.2])
    np.testing.assert_allclose(constraint.frame, frame)
    assert constraint.tolerence == pytest.approx(1e-2)

    # The constraint should also attach to RRTOptions and round-trip there.
    options = RRTOptions(group_name="arm", rrt_connect=True, pose_constraint=constraint)
    assert options.pose_constraint is not None
    assert options.pose_constraint.link_name == "tool0"


def test_plan_with_pose_constraint(test_scene: Scene) -> None:
    # Plan with constraint-projection RRT enabled. Bounds are left unbounded so the projection
    # step runs (exercising the full constrained code path) while every configuration trivially
    # satisfies the constraint, keeping the test deterministic.
    test_scene.setRngSeed(286)

    options = RRTOptions()
    options.group_name = "arm"
    options.max_connection_distance = 1.0
    options.collision_check_step_size = 0.05
    # Constraint projection only runs for the RRT-Connect variant.
    options.rrt_connect = True
    options.pose_constraint = PoseConstraint(link_name="tool0")

    rrt = RRT(test_scene, options)
    rrt.setRngSeed(1234)

    # setPoseConstraint validates that the link exists in the model.
    with pytest.raises(Exception):
        rrt.setPoseConstraint(PoseConstraint(link_name="nonexistent_link"))
    rrt.setPoseConstraint(PoseConstraint(link_name="tool0"))

    start = JointConfiguration()
    start.positions = test_scene.randomCollisionFreePositions()
    assert start.positions is not None

    goal = JointConfiguration()
    goal.positions = test_scene.randomCollisionFreePositions()
    assert goal.positions is not None

    path = rrt.plan(start, goal)
    assert path is not None
