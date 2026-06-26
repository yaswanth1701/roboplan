from typing import Annotated

import numpy
from numpy.typing import NDArray
import roboplan.core._core_ext


class Node:
    """Defines a graph node for search-based planners."""

    def __init__(self, config: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], parent_id: int) -> None: ...

    @property
    def config(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The configuration (e.g., joint positions) of this node."""

    @property
    def parent_id(self) -> int:
        """The parent node ID."""

    @property
    def cost(self) -> float:
        """The cost-to-come from the tree root to this node (RRT* only)."""

class PoseConstraint:
    """Pose constraint on a link frame for constraint-projection RRT."""

    def __init__(self, link_name: str = '', min: Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')] = ..., max: Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')] = ..., frame: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')] = ..., tolerence: float = 0.001) -> None: ...

    @property
    def link_name(self) -> str:
        """The link to be constrained."""

    @link_name.setter
    def link_name(self, arg: str, /) -> None: ...

    @property
    def min(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]:
        """
        The minimum for each dimension of the pose (translation: xyz, orientation: roll, pitch, yaw as extrinsic XYZ Euler angles).
        """

    @min.setter
    def min(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')], /) -> None: ...

    @property
    def max(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]:
        """
        The maximum for each dimension of the pose (translation: xyz, orientation: roll, pitch, yaw as extrinsic XYZ Euler angles).
        """

    @max.setter
    def max(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')], /) -> None: ...

    @property
    def frame(self) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
        """
        Reference transform/frame for the constraint with respect to the base frame. The link pose is expressed relative to this frame before checking against min/max.
        """

    @frame.setter
    def frame(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], /) -> None: ...

    @property
    def tolerence(self) -> float:
        """The convergence tolerance for the constraint projection."""

    @tolerence.setter
    def tolerence(self, arg: float, /) -> None: ...

class RRTOptions:
    """Options struct for RRT planner."""

    def __init__(self, group_name: str = '', max_nodes: int = 1000, max_connection_distance: float = 3.0, collision_check_step_size: float = 0.05, collision_check_use_bisection: bool = False, goal_biasing_probability: float = 0.15, max_planning_time: float = 0.0, rrt_connect: bool = False, rrt_star: bool = False, rewire_distance: float = 5.0, fast_return: bool = True, pose_constraint: PoseConstraint | None = None) -> None: ...

    @property
    def group_name(self) -> str:
        """The joint group name to be used by the planner."""

    @group_name.setter
    def group_name(self, arg: str, /) -> None: ...

    @property
    def max_nodes(self) -> int:
        """The maximum number of nodes to sample."""

    @max_nodes.setter
    def max_nodes(self, arg: int, /) -> None: ...

    @property
    def max_connection_distance(self) -> float:
        """The maximum configuration distance between two nodes."""

    @max_connection_distance.setter
    def max_connection_distance(self, arg: float, /) -> None: ...

    @property
    def collision_check_step_size(self) -> float:
        """The configuration-space step size for collision checking along edges."""

    @collision_check_step_size.setter
    def collision_check_step_size(self, arg: float, /) -> None: ...

    @property
    def collision_check_use_bisection(self) -> bool:
        """
        If true, uses bisection instead of linear search for collision checking along edges.
        """

    @collision_check_use_bisection.setter
    def collision_check_use_bisection(self, arg: bool, /) -> None: ...

    @property
    def goal_biasing_probability(self) -> float:
        """The probability of sampling the goal node instead of a random node."""

    @goal_biasing_probability.setter
    def goal_biasing_probability(self, arg: float, /) -> None: ...

    @property
    def max_planning_time(self) -> float:
        """The maximum amount of time to allow for planning, in seconds."""

    @max_planning_time.setter
    def max_planning_time(self, arg: float, /) -> None: ...

    @property
    def rrt_connect(self) -> bool:
        """If true, use the RRT-Connect algorithm to grow the search trees."""

    @rrt_connect.setter
    def rrt_connect(self, arg: bool, /) -> None: ...

    @property
    def rrt_star(self) -> bool:
        """If true, use the RRT* algorithm to grow asymptotically optimal trees."""

    @rrt_star.setter
    def rrt_star(self, arg: bool, /) -> None: ...

    @property
    def rewire_distance(self) -> float:
        """
        The configuration-space radius used to find neighbors for RRT* rewiring.
        """

    @rewire_distance.setter
    def rewire_distance(self, arg: float, /) -> None: ...

    @property
    def fast_return(self) -> bool:
        """
        If true, return on the first path found; if false, plan until the budget is exhausted and return the lowest-cost path.
        """

    @fast_return.setter
    def fast_return(self, arg: bool, /) -> None: ...

    @property
    def pose_constraint(self) -> PoseConstraint | None:
        """
        Optional pose constraint on a link frame. Only used when `rrt_connect` is true, otherwise ignored.
        """

    @pose_constraint.setter
    def pose_constraint(self, arg: PoseConstraint | None) -> None: ...

class RRT:
    """
    Motion planner based on the Rapidly-exploring Random Tree (RRT) algorithm.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, options: RRTOptions) -> None: ...

    def plan(self, start: roboplan.core._core_ext.JointConfiguration, goal: roboplan.core._core_ext.JointConfiguration) -> roboplan.core._core_ext.JointPath:
        """Plan a path from start to goal."""

    def setRngSeed(self, seed: int) -> None:
        """Sets the seed for the random number generator (RNG)."""

    def setPoseConstraint(self, constraint: PoseConstraint) -> None:
        """Updates the pose constraint."""

    def getNodes(self) -> tuple[list[Node], list[Node]]:
        """
        Returns the start and goal trees' node vectors, for visualization purposes.
        """
