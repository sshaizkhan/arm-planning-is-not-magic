#!/usr/bin/env python3
"""
Demo 04: Collision-aware planning with geometric shapes.

Demonstrates:
1. Adding collision objects (cube, sphere, cylinder) around start/goal
2. Planning with collision checking
3. Logging collision detections
"""

import numpy as np

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.collision_manager import ShapeCollisionManager, Box, Sphere, Cylinder
from planners.ompl_rrt_connect import OMPLRRTConnectPlanner
from parameterization.toppra_parameterization import ToppraTimeParameterizer


def add_collision_objects_at_pose(collision_manager, robot, q, create_collision=True):
    """
    Add collision objects at or near a joint configuration.

    Args:
        collision_manager: ShapeCollisionManager instance
        robot: Robot model
        q: Joint configuration
        create_collision: If True, place obstacles to cause collisions
    """
    # Get end-effector position
    try:
        pose = robot.fk(q)
        if pose is None:
            return
        ee_pos = pose[:3, 3] if isinstance(pose, np.ndarray) else np.array(pose['translation'])
    except Exception:
        print(f"Warning: Could not compute FK for q={q}, skipping collision objects")
        return

    if create_collision:
        # Place obstacles that will intersect with the end-effector
        # Large box that contains the end-effector (centered at EE, so EE is inside)
        cube = Box(ee_pos, np.array([0.25, 0.25, 0.25]))
        collision_manager.add_shape(cube)
        print(f"  Added box centered at {ee_pos} (size: 0.25x0.25x0.25)")
        # Verify: box should contain ee_pos
        assert cube.check_point(ee_pos), f"Box should contain EE position {ee_pos}"

        # Large sphere that contains the end-effector
        sphere = Sphere(ee_pos, radius=0.15)
        collision_manager.add_shape(sphere)
        print(f"  Added sphere centered at {ee_pos} (radius: 0.15)")
        # Verify: sphere should contain ee_pos
        assert sphere.check_point(ee_pos), f"Sphere should contain EE position {ee_pos}"

        # Cylinder that intersects the end-effector
        cylinder = Cylinder(ee_pos, radius=0.12, height=0.3)
        collision_manager.add_shape(cylinder)
        print(f"  Added cylinder centered at {ee_pos} (radius: 0.12, height: 0.3)")
        # Verify: cylinder should contain ee_pos
        assert cylinder.check_point(ee_pos), f"Cylinder should contain EE position {ee_pos}"
    else:
        # Place obstacles offset from the end-effector (no collision)
        cube_center = ee_pos + np.array([0.3, 0.0, 0.0])
        cube = Box(cube_center, np.array([0.15, 0.15, 0.15]))
        collision_manager.add_shape(cube)
        print(f"  Added box at {cube_center} (offset, no collision)")

        sphere_center = ee_pos + np.array([-0.3, 0.0, 0.0])
        sphere = Sphere(sphere_center, radius=0.1)
        collision_manager.add_shape(sphere)
        print(f"  Added sphere at {sphere_center} (offset, no collision)")


def add_collision_objects_along_path(collision_manager, robot, path, num_obstacles=3):
    """
    Add collision objects along the path to create collisions.

    Args:
        collision_manager: ShapeCollisionManager instance
        robot: Robot model
        path: List of joint configurations
        num_obstacles: Number of obstacles to add along the path
    """
    if len(path) < 2:
        return

    # Sample waypoints along the path
    indices = np.linspace(len(path) // 4, 3 * len(path) // 4, num_obstacles, dtype=int)

    for idx in indices:
        if idx >= len(path):
            continue
        q = path[idx]
        try:
            pose = robot.fk(q)
            if pose is None:
                continue
            ee_pos = pose[:3, 3] if isinstance(pose, np.ndarray) else np.array(pose['translation'])

            # Add obstacle that will intersect
            obstacle = Sphere(ee_pos, radius=0.12)
            collision_manager.add_shape(obstacle)
            print(f"  Added obstacle at waypoint {idx}, EE position: {ee_pos}")
        except Exception:
            continue


def main():
    """Main demo function."""
    print("=" * 80)
    print("COLLISION-AWARE PLANNING DEMO")
    print("=" * 80)

    # Setup robot
    print("\n1. Setting up robot...")
    robot = UR5RobotModel(use_opw=True)
    state_space = JointStateSpace(robot)

    # Setup collision manager
    print("\n2. Setting up collision manager...")
    collision_manager = ShapeCollisionManager(robot)
    robot.in_collision = collision_manager.in_collision

    # Define start and goal
    print("\n3. Defining start and goal configurations...")
    q_start = np.array([0.0, 0.927747217, 0.0, 0.642, 0.0, 0.0])
    q_goal = np.array([0.5, -0.5, 0.3, -0.7, 0.2, 0.0])

    print(f"   Start: {q_start}")
    print(f"   Goal:  {q_goal}")

    # Add collision objects at start and goal to create collisions
    print("\n4. Adding collision objects...")
    print("   At start pose (will cause collision):")
    add_collision_objects_at_pose(collision_manager, robot, q_start, create_collision=True)
    print("   At goal pose (will cause collision):")
    add_collision_objects_at_pose(collision_manager, robot, q_goal, create_collision=True)

    # Check initial states
    print("\n5. Checking initial states for collisions...")
    collision_manager.clear_log()  # Start with clean log

    # Get EE positions for verification
    start_pose = robot.fk(q_start)
    start_ee_pos = start_pose[:3, 3] if isinstance(start_pose, np.ndarray) else np.array(start_pose['translation'])
    goal_pose = robot.fk(q_goal)
    goal_ee_pos = goal_pose[:3, 3] if isinstance(goal_pose, np.ndarray) else np.array(goal_pose['translation'])

    print(f"   Start EE position: {start_ee_pos}")
    print(f"   Goal EE position: {goal_ee_pos}")
    print(f"   Number of collision shapes: {len(collision_manager.shapes)}")

    # Manually verify shapes contain EE positions
    print("\n   Verifying shape intersections...")
    for i, shape in enumerate(collision_manager.shapes):
        start_inside = shape.check_point(start_ee_pos)
        goal_inside = shape.check_point(goal_ee_pos)
        print(f"     Shape {i} ({shape.get_type()}): start_inside={start_inside}, goal_inside={goal_inside}")

    start_collision = collision_manager.in_collision(q_start)
    if start_collision:
        print("\n   WARNING: Start configuration is in collision!")
        collision_log = collision_manager.get_collision_log()
        for entry in collision_log:
            print(f"     -> Collision with {entry['shape_type']} at EE position {entry['ee_position']}")
    else:
        print("\n   Start configuration is collision-free (this should not happen if obstacles are placed correctly)")

    # Check goal (clear log first to separate from start collisions)
    collision_manager.clear_log()
    goal_collision = collision_manager.in_collision(q_goal)
    if goal_collision:
        print("   WARNING: Goal configuration is in collision!")
        collision_log = collision_manager.get_collision_log()
        for entry in collision_log:
            print(f"     -> Collision with {entry['shape_type']} at EE position {entry['ee_position']}")
    else:
        print("   Goal configuration is collision-free (this should not happen if obstacles are placed correctly)")

    collision_manager.clear_log()

    # Plan path (first without obstacles blocking start/goal)
    print("\n6. Planning initial path (without obstacles at start/goal)...")
    # Temporarily remove obstacles at start/goal for planning
    temp_shapes = collision_manager.shapes.copy()
    collision_manager.clear_shapes()

    planner = OMPLRRTConnectPlanner(state_space, step_size=0.1)
    path = planner.plan(q_start, q_goal, timeout=5.0)

    if path is None:
        print("   No path found! Restoring obstacles and checking collisions...")
        collision_manager.shapes = temp_shapes
        print(f"   Total collisions detected: {len(collision_manager.get_collision_log())}")
        return

    print(f"   Path found with {len(path)} waypoints")

    # Add obstacles along the path to create collisions
    print("\n   Adding obstacles along path to create collisions...")
    add_collision_objects_along_path(collision_manager, robot, path, num_obstacles=3)

    # Restore all obstacles
    collision_manager.shapes = temp_shapes + collision_manager.shapes

    # Check path for collisions
    print("\n7. Checking path for collisions...")
    collision_manager.clear_log()  # Clear previous logs
    path_collisions = []
    for i, q in enumerate(path):
        if collision_manager.in_collision(q):
            path_collisions.append(i)
            # Get the latest collision entry
            collision_log = collision_manager.get_collision_log()
            if collision_log:
                latest = collision_log[-1]
                print(f"   -> Collision at waypoint {i}/{len(path)-1}: EE at {latest['ee_position']} collides with {latest['shape_type']}")

    if path_collisions:
        print(f"   WARNING: Path has {len(path_collisions)} collisions at waypoints: {path_collisions}")
    else:
        print("   Path is collision-free!")

    # Time-parameterize
    print("\n8. Time-parameterizing path...")
    dof = robot.dof()
    v_max = np.ones(dof) * 1.0
    a_max = np.ones(dof) * 2.0
    toppra = ToppraTimeParameterizer(v_max, a_max)

    try:
        time_stamps, trajectory = toppra.compute(path)
        print(f"   Trajectory duration: {time_stamps[-1]:.3f} s")
        print(f"   Number of samples: {len(time_stamps)}")
    except Exception as e:
        print(f"   Failed to time-parameterize: {e}")

    # Summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)
    total_collisions = len(collision_manager.get_collision_log())
    print(f"Total collisions detected: {total_collisions}")
    print(f"Collision objects in scene: {len(collision_manager.shapes)}")
    if total_collisions > 0:
        print("\nCollision breakdown by shape type:")
        shape_counts = {}
        for entry in collision_manager.get_collision_log():
            shape_type = entry['shape_type']
            shape_counts[shape_type] = shape_counts.get(shape_type, 0) + 1
        for shape_type, count in shape_counts.items():
            print(f"  {shape_type}: {count}")


if __name__ == "__main__":
    main()
