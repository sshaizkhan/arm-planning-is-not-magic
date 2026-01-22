#!/usr/bin/env python3
"""
Demo 06: PyBullet 3D Visualization with Obstacle Avoidance.

This demo shows how to visualize robot arm trajectories using PyBullet
for realistic 3D rendering with proper physics and lighting.

Features demonstrated:
- Proper UR5 model with correct kinematics
- Obstacle avoidance planning using PyBullet's collision detection
- Path preview (showing planned waypoints as a line before execution)
- Trajectory visualization with end-effector trail
- Interactive camera controls

Run with: python demos/06_pybullet_visualization.py
"""

import numpy as np
import time

from core.robot_model import UR5RobotModel
from core.state_space import JointStateSpace
from core.path_smoothing import smooth_path
from core.kinematics.opw import OPWKinematics
from core.kinematics.opw_parameters import OPWParameters
from planners.ompl_rrt_connect import OMPLRRTConnectPlanner
from parameterization.toppra_parameterization import ToppraTimeParameterizer

try:
    from visualization.pybullet_visualizer import PyBulletVisualizer
except ImportError:
    print("ERROR: PyBullet not installed. Install with: pip install pybullet")
    exit(1)


def quaternion_to_rotation_matrix(q):
    """
    Convert quaternion to 3x3 rotation matrix.

    Args:
        q: Quaternion as (w, x, y, z) or (x, y, z, w)

    Returns:
        3x3 rotation matrix
    """
    q = np.array(q)
    if len(q) != 4:
        raise ValueError("Quaternion must have 4 elements")

    # Normalize
    q = q / np.linalg.norm(q)

    # Handle both (w,x,y,z) and (x,y,z,w) formats
    if abs(q[0]) > 1.0 or (abs(q[0]) < 0.5 and abs(q[3]) > 0.5):
        # Likely (x,y,z,w) format
        x, y, z, w = q
    else:
        # Likely (w,x,y,z) format
        w, x, y, z = q

    # Convert to rotation matrix
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    return R


def euler_to_rotation_matrix(euler_angles):
    """
    Convert Euler angles (roll, pitch, yaw) to 3x3 rotation matrix.

    Args:
        euler_angles: (roll, pitch, yaw) in radians

    Returns:
        3x3 rotation matrix
    """
    roll, pitch, yaw = euler_angles
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr],
    ])
    return R


def pose_to_transform(position, orientation):
    """
    Convert position and orientation to 4x4 transformation matrix.

    Args:
        position: (x, y, z) position
        orientation: Can be:
            - Quaternion as (w, x, y, z) or (x, y, z, w)
            - Rotation matrix (3x3)
            - Euler angles (roll, pitch, yaw) in radians
            - None (identity rotation)

    Returns:
        4x4 transformation matrix
    """
    T = np.eye(4)
    T[:3, 3] = np.array(position)

    if orientation is None:
        return T

    orientation = np.array(orientation)

    if orientation.shape == (4,):
        # Quaternion
        T[:3, :3] = quaternion_to_rotation_matrix(orientation)
    elif orientation.shape == (3, 3):
        # Rotation matrix
        T[:3, :3] = orientation
    elif orientation.shape == (3,):
        # Euler angles (roll, pitch, yaw)
        T[:3, :3] = euler_to_rotation_matrix(orientation)
    else:
        raise ValueError(f"Invalid orientation shape: {orientation.shape}")

    return T


def solve_ik_for_pose(ik_solver, pose, reference_q=None, prefer_within_limits=True, collision_check=None):
    """
    Solve IK for a pose and select the best solution.

    Args:
        ik_solver: OPWKinematics instance
        pose: 4x4 transformation matrix or dict with 'translation' and 'rotation'
        reference_q: Reference joint configuration for selecting closest solution
        prefer_within_limits: If True, prefer solutions within joint limits
        collision_check: Optional collision checking function. If provided, filters out colliding solutions.

    Returns:
        Best joint configuration (6,) or None if no valid solution
    """
    # Convert pose to 4x4 matrix if needed
    if isinstance(pose, dict):
        T = pose_to_transform(pose['translation'], pose.get('rotation'))
    elif isinstance(pose, np.ndarray) and pose.shape == (4, 4):
        T = pose
    else:
        raise ValueError("Pose must be 4x4 matrix or dict with 'translation' and 'rotation'")

    # Get all IK solutions
    solutions = ik_solver.inverse_kinematics(T)

    # Filter valid (non-NaN) solutions
    valid_solutions = []
    for sol in solutions:
        if not np.any(np.isnan(sol)):
            valid_solutions.append(sol)

    if len(valid_solutions) == 0:
        return None

    # Filter by joint limits if requested
    if prefer_within_limits:
        limited_solutions = ik_solver.filter_joint_limits(valid_solutions)
        if len(limited_solutions) > 0:
            valid_solutions = limited_solutions

    # Filter by collision if collision checker provided
    if collision_check is not None:
        collision_free = [q for q in valid_solutions if not collision_check(q)]
        if len(collision_free) > 0:
            valid_solutions = collision_free
        # If all solutions are in collision, we'll still return one (closest to reference)
        # but warn the user

    # Select best solution (closest to reference if provided)
    if reference_q is not None:
        distances = [float(np.linalg.norm(q - reference_q)) for q in valid_solutions]
        best_idx = np.argmin(distances)
        best_sol = valid_solutions[best_idx]
    else:
        best_sol = valid_solutions[0]

    # Warn if the selected solution is in collision
    if collision_check is not None and collision_check(best_sol):
        print("      WARNING: Selected IK solution is in collision, but no collision-free solution found")

    return best_sol


def main():
    print("=" * 60)
    print("PYBULLET 3D VISUALIZATION WITH OBSTACLE AVOIDANCE")
    print("=" * 60)

    # ---------------------------
    # 1. Setup robot and IK solver
    # ---------------------------
    print("\n[1] Setting up robot and IK solver...")

    robot = UR5RobotModel()
    opw_params = OPWParameters.make_ur5()
    ik_solver = OPWKinematics(robot, opw_params)

    # Display joint limits being used
    lower, upper = robot.joint_limits()
    print("    Robot joint limits (rad):")
    joint_names = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
    for i, name in enumerate(joint_names):
        print(f"      {name:10s}: [{lower[i]:7.3f}, {upper[i]:7.3f}]")

    # Define initial start and goal configurations for obstacle placement
    # These will be used to determine obstacle positions, then we'll recompute from poses
    q_start_init = np.array([0.0, -np.pi/4, np.pi/3, -np.pi/2, np.pi/4, 0.0])
    q_goal_init = np.array([np.pi/2, -np.pi/3, np.pi/4, -np.pi/4, -np.pi/4, np.pi/2])

    # Get approximate EE positions using robot's FK for reference
    start_pose_init = robot.fk(q_start_init)
    goal_pose_init = robot.fk(q_goal_init)
    start_ee = start_pose_init[:3, 3]
    goal_ee = goal_pose_init[:3, 3]

    print(f"    Approximate start EE position: [{start_ee[0]:.3f}, {start_ee[1]:.3f}, {start_ee[2]:.3f}]")
    print(f"    Approximate goal EE position:  [{goal_ee[0]:.3f}, {goal_ee[1]:.3f}, {goal_ee[2]:.3f}]")

    # Randomly place obstacle in workspace
    # Define workspace bounds for obstacle placement
    workspace_x_range = (-0.4, 0.4)  # X bounds
    workspace_y_range = (-0.4, 0.4)  # Y bounds
    workspace_z_range = (0.2, 1.0)   # Z bounds (height)

    # Generate random obstacle position (different each run)
    obstacle_x = np.random.uniform(workspace_x_range[0], workspace_x_range[1])
    obstacle_y = np.random.uniform(workspace_y_range[0], workspace_y_range[1])
    obstacle_z = np.random.uniform(workspace_z_range[0], workspace_z_range[1])
    obstacle_center = np.array([obstacle_x, obstacle_y, obstacle_z])
    obstacle_size = (0.10, 0.10, 0.98)

    print(f"    Random obstacle center: [{obstacle_center[0]:.3f}, {obstacle_center[1]:.3f}, {obstacle_center[2]:.3f}]")

    # ---------------------------
    # 2. Create visualizer and add obstacles
    # ---------------------------
    print("\n[2] Starting PyBullet and adding obstacles...")

    viz = PyBulletVisualizer(
        gui=True,
        base_position=(0, 0, 0),
        camera_distance=1.8,
        camera_yaw=50,
        camera_pitch=-25,
    )

    # Add obstacle to PyBullet (returns body ID)
    obstacle_id = viz.add_box(
        center=tuple(obstacle_center),
        size=obstacle_size,
        color=(0.9, 0.2, 0.2, 0.7),
    )

    # Add floor obstacle to prevent robot from going under ground
    floor_height = 0.01  # Thickness of floor
    floor_size = (2.0, 2.0, floor_height)  # Large floor covering workspace
    floor_center = (0.0, 0.0, -floor_height / 2)  # Positioned at z=0 (slightly below)
    floor_id = viz.add_box(
        center=floor_center,
        size=floor_size,
        color=(0.5, 0.5, 0.5, 0.8),  # Gray floor
    )
    print(f"    Floor obstacle added at z={floor_center[2]:.3f}")

    # Add walls and ceiling to constrain workspace
    workspace_size = 1.2  # Workspace half-size (meters)
    workspace_height = 1.5  # Height of workspace (meters)
    wall_thickness = 0.02  # Thickness of walls

    # Create list to store all obstacle IDs
    obstacle_ids = [obstacle_id, floor_id]

    # Front wall (positive Y)
    front_wall_id = viz.add_box(
        center=(0.0, workspace_size, workspace_height / 2),
        size=(workspace_size * 2, wall_thickness, workspace_height),
        color=(0.4, 0.4, 0.6, 0.6),
    )
    obstacle_ids.append(front_wall_id)

    # Back wall (negative Y)
    back_wall_id = viz.add_box(
        center=(0.0, -workspace_size, workspace_height / 2),
        size=(workspace_size * 2, wall_thickness, workspace_height),
        color=(0.4, 0.4, 0.6, 0.6),
    )
    obstacle_ids.append(back_wall_id)

    # Left wall (negative X)
    left_wall_id = viz.add_box(
        center=(-workspace_size, 0.0, workspace_height / 2),
        size=(wall_thickness, workspace_size * 2, workspace_height),
        color=(0.4, 0.4, 0.6, 0.6),
    )
    obstacle_ids.append(left_wall_id)

    # Right wall (positive X)
    right_wall_id = viz.add_box(
        center=(workspace_size, 0.0, workspace_height / 2),
        size=(wall_thickness, workspace_size * 2, workspace_height),
        color=(0.4, 0.4, 0.6, 0.6),
    )
    obstacle_ids.append(right_wall_id)

    # Ceiling
    ceiling_id = viz.add_box(
        center=(0.0, 0.0, workspace_height),
        size=(workspace_size * 2, workspace_size * 2, wall_thickness),
        color=(0.4, 0.4, 0.6, 0.6),
    )
    obstacle_ids.append(ceiling_id)

    print(f"    Workspace enclosure added: {workspace_size*2:.1f}m x {workspace_size*2:.1f}m x {workspace_height:.1f}m")

    # ---------------------------
    # 3. Setup collision-aware planning using PyBullet
    # ---------------------------
    print("\n[3] Setting up collision-aware planning...")

    # Create collision checker that uses PyBullet (includes all obstacles)
    pybullet_collision_check = viz.create_collision_checker(obstacle_ids)

    # Create a custom collision manager that uses PyBullet's collision detection
    class PyBulletCollisionManager:
        def __init__(self, check_func):
            self._check = check_func

        def in_collision(self, q):
            return self._check(q)

    collision_manager = PyBulletCollisionManager(pybullet_collision_check)
    robot.in_collision = collision_manager.in_collision

    state_space = JointStateSpace(robot)

    # ---------------------------
    # 4. Compute start and goal states from poses
    # ---------------------------
    print("\n[4] Computing start and goal states from poses...")

    # Start with a known valid joint configuration (home position)
    # This ensures we have a valid starting state
    q_start = np.array([0.0, -np.pi/4, np.pi/3, -np.pi/2, np.pi/4, 0.0])

    # Get the start pose from the joint configuration using forward kinematics
    start_pose = robot.fk(q_start)
    start_position = start_pose[:3, 3]
    start_rotation = start_pose[:3, :3]

    print(f"    Start pose from FK: position={start_position}")

    # Generate random goal pose within workspace bounds
    # Workspace bounds for goal generation (similar to obstacle placement)
    workspace_x_range = (-0.4, 0.4)
    workspace_y_range = (-0.4, 0.4)
    workspace_z_range = (0.2, 1.0)

    q_goal = None
    goal_pose = None
    goal_position = None

    print("    Generating random goal pose...")
    max_attempts = 20  # Try up to 20 random positions

    for attempt in range(max_attempts):
        # Generate random goal position
        goal_x = np.random.uniform(workspace_x_range[0], workspace_x_range[1])
        goal_y = np.random.uniform(workspace_y_range[0], workspace_y_range[1])
        goal_z = np.random.uniform(workspace_z_range[0], workspace_z_range[1])
        candidate_position = np.array([goal_x, goal_y, goal_z])

        # Keep same orientation as start (or could randomize this too)
        candidate_rotation = start_rotation.copy()
        candidate_pose = pose_to_transform(candidate_position, candidate_rotation)

        if attempt < 3 or attempt % 5 == 0:  # Print first few and every 5th
            print(f"      Attempt {attempt+1}/{max_attempts}: position={candidate_position}")

        candidate_q = solve_ik_for_pose(
            ik_solver, candidate_pose,
            reference_q=q_start,
            prefer_within_limits=True,
            collision_check=pybullet_collision_check
        )

        if candidate_q is not None:
            # Verify it's not in collision
            if not pybullet_collision_check(candidate_q):
                q_goal = candidate_q
                goal_pose = candidate_pose
                goal_position = candidate_position
                print(f"      ✓ Found valid collision-free goal at position {goal_position} (attempt {attempt+1})")
                break

    if q_goal is None:
        print(f"ERROR: Could not find collision-free goal pose after {max_attempts} attempts!")
        print("      Try adjusting obstacle position or workspace bounds.")
        viz.close()
        return

    print(f"    Start joint angles: {q_start}")
    print(f"    Goal joint angles:  {q_goal}")

    # Verify poses match (optional check)
    goal_pose_actual = robot.fk(q_goal)
    if goal_pose is not None:
        goal_error = np.linalg.norm(goal_pose[:3, 3] - goal_pose_actual[:3, 3])
        print(f"    IK position error: goal={goal_error:.4f}m")
    else:
        print("    Goal pose verification skipped")

    # Verify start and goal states are valid
    print("    Validating start and goal states...")
    start_valid = state_space.is_valid(q_start)
    goal_valid = state_space.is_valid(q_goal)

    if not start_valid:
        print("ERROR: Start state is invalid!")
        print(f"      Joint angles: {q_start}")
        print(f"      In collision: {pybullet_collision_check(q_start)}")
        # Check joint limits
        lower, upper = robot.joint_limits()
        for i in range(len(q_start)):
            if q_start[i] < lower[i] or q_start[i] > upper[i]:
                print(f"      Joint {i} out of limits: {q_start[i]:.3f} not in [{lower[i]:.3f}, {upper[i]:.3f}]")
        viz.close()
        return

    if not goal_valid:
        print("ERROR: Goal state is invalid!")
        print(f"      Joint angles: {q_goal}")
        print(f"      In collision: {pybullet_collision_check(q_goal)}")
        # Check joint limits
        lower, upper = robot.joint_limits()
        for i in range(len(q_goal)):
            if q_goal[i] < lower[i] or q_goal[i] > upper[i]:
                print(f"      Joint {i} out of limits: {q_goal[i]:.3f} not in [{lower[i]:.3f}, {upper[i]:.3f}]")
        viz.close()
        return

    print("    Both start and goal states are valid!")

    # ---------------------------
    # 5. Plan path avoiding obstacles
    # ---------------------------
    print("\n[5] Planning collision-free path...")

    planner = OMPLRRTConnectPlanner(state_space, step_size=0.05)
    path = planner.plan(q_start, q_goal, timeout=10.0)

    if path is None:
        print("ERROR: No path found! Try adjusting obstacle position or size.")
        viz.close()
        return

    print(f"    Raw path found with {len(path)} waypoints")

    # ---------------------------
    # 6. Smooth the path
    # ---------------------------
    print("\n[6] Smoothing path...")

    # Apply shortcut + spline smoothing
    smoothed_path = smooth_path(
        path,
        collision_check=pybullet_collision_check,
        shortcut_iterations=100,  # Remove unnecessary waypoints
        spline_points=150,        # Smooth curve with 150 points
        step_size=0.02,
    )

    print(f"    Smoothed path: {len(smoothed_path)} points")

    # Verify smoothed path is collision-free
    collisions = sum(1 for q in smoothed_path if pybullet_collision_check(q))
    if collisions > 0:
        print(f"    WARNING: Smoothed path has {collisions} collisions, using original")
        smoothed_path = np.array(path)
    else:
        print("    Smoothed path verified collision-free!")

    # Use smoothed path for visualization and execution
    path = [smoothed_path[i] for i in range(len(smoothed_path))]

    # ---------------------------
    # 7. Time-parameterize path
    # ---------------------------
    print("\n[7] Time-parameterizing path...")

    dof = robot.dof()
    # UR5 realistic velocity limits (rad/s) - from URDF specifications
    # These are the actual maximum velocities for each joint as specified in the robot's URDF
    # Joints 1-3 (base, shoulder, elbow): ~4.0-4.5 rad/s
    # Joints 4-5 (wrist): ~8.2 rad/s
    # Joint 6 (end effector): ~12.2 rad/s
    v_max = np.array([4.5, 4.0, 4.5, 8.2, 8.2, 12.2])

    # UR5 realistic acceleration limits (rad/s^2)
    # Typical industrial robot acceleration: 2-5 rad/s² for safety and smoothness
    # Higher values (5-10) for faster but still safe operation
    # These limits ensure the trajectory is executable on a real UR5 robot
    a_max = np.ones(dof) * 5.0  # Realistic acceleration for real robot execution

    toppra = ToppraTimeParameterizer(v_max, a_max)
    time_stamps, trajectory = toppra.compute(path)
    print("    Trajectory computed with realistic robot limits:")
    print(f"      Duration: {time_stamps[-1]:.3f}s")
    print(f"      Samples: {len(time_stamps)}")
    print(f"      Velocity limits: {v_max} rad/s")
    print(f"      Acceleration limits: {a_max[0]:.1f} rad/s² (all joints)")

    # Validate trajectory respects joint limits
    print("\n    Validating trajectory joint limits...")
    lower, upper = robot.joint_limits()
    violations = []
    for i, q in enumerate(trajectory):
        for j in range(dof):
            if q[j] < lower[j] or q[j] > upper[j]:
                violations.append((i, j, q[j], lower[j], upper[j]))

    if violations:
        print(f"      WARNING: Found {len(violations)} joint limit violations!")
        for idx, joint, value, low, high in violations[:10]:  # Show first 10
            print(f"        Sample {idx}, Joint {joint}: {value:.3f} not in [{low:.3f}, {high:.3f}]")
        if len(violations) > 10:
            print(f"        ... and {len(violations) - 10} more violations")
    else:
        print(f"      ✓ All {len(trajectory)} trajectory samples respect joint limits")
        print(f"      Joint limits: {lower} to {upper}")

    # Check velocity and acceleration (if we can compute them)
    print("\n    Checking velocity/acceleration limits...")
    if len(trajectory) > 1:
        dt = np.diff(time_stamps)
        velocities = np.diff(trajectory, axis=0) / dt[:, np.newaxis]
        vel_violations = np.any(np.abs(velocities) > v_max, axis=1)
        if np.any(vel_violations):
            n_violations = np.sum(vel_violations)
            print(f"      WARNING: {n_violations} velocity limit violations found")
            max_vel = np.max(np.abs(velocities), axis=0)
            for j in range(dof):
                if max_vel[j] > v_max[j]:
                    print(f"        Joint {j}: max velocity {max_vel[j]:.3f} > limit {v_max[j]:.3f} rad/s")
        else:
            print(f"      ✓ All velocities within limits (max: {np.max(np.abs(velocities)):.3f} rad/s)")

        if len(velocities) > 1:
            accel = np.diff(velocities, axis=0) / dt[1:, np.newaxis]
            accel_violations = np.any(np.abs(accel) > a_max, axis=1)
            if np.any(accel_violations):
                n_violations = np.sum(accel_violations)
                print(f"      WARNING: {n_violations} acceleration limit violations found")
                max_accel = np.max(np.abs(accel), axis=0)
                for j in range(dof):
                    if max_accel[j] > a_max[j]:
                        print(f"        Joint {j}: max acceleration {max_accel[j]:.3f} > limit {a_max[j]:.3f} rad/s²")
            else:
                print(f"      ✓ All accelerations within limits (max: {np.max(np.abs(accel)):.3f} rad/s²)")

    # ---------------------------
    # 8. Visualize
    # ---------------------------
    print("\n[8] Visualizing...")
    print("    Controls:")
    print("    - Mouse: Rotate camera")
    print("    - Scroll: Zoom")
    print("    - Q: Quit during playback")

    # Show start configuration
    print("\n    Showing start configuration...")
    viz.visualize_configuration(q_start, duration=1.0)

    # Add markers for start and goal
    start_ee_pos, _ = viz.get_end_effector_pose()
    viz.add_marker(tuple(start_ee_pos), color=(0, 1, 0), size=0.04)  # Green = start

    viz.set_joint_positions(q_goal)
    goal_ee_pos, _ = viz.get_end_effector_pose()
    viz.add_marker(tuple(goal_ee_pos), color=(1, 0, 0), size=0.04)  # Red = goal

    viz.visualize_configuration(q_start, duration=0.5)

    # Show planned path
    print("\n    Showing planned path (orange) - notice it avoids the obstacle...")
    viz.visualize_path(
        np.array(path),
        color=(1.0, 0.5, 0.0),
        line_width=3,
        show_waypoints=True,
        waypoint_color=(1.0, 0.3, 0.0),
        waypoint_size=0.012,
    )
    time.sleep(3.0)

    # Show smooth trajectory
    print("    Showing smooth trajectory (cyan line)...")
    viz.visualize_trajectory_path(
        trajectory,
        color=(0.0, 0.8, 1.0),
        line_width=2,
        sample_every=10,
    )
    time.sleep(2.0)

    # Play trajectory at real-time speed (as it would execute on real robot)
    # The trajectory timing is computed with realistic velocity/acceleration limits
    # This matches what would be sent to the actual UR5 robot controller
    print(f"\n    Executing trajectory (real-time, {time_stamps[-1]:.2f}s duration)...")
    viz.visualize_trajectory(
        trajectory,
        time_stamps=time_stamps,
        real_time=True,  # Play at actual computed timing (real robot speed)
        speed=1.0,      # No speed multiplier - use computed trajectory timing
        show_ee_trail=True,
        trail_length=200,
    )

    # Hold final position
    print("\n    Trajectory complete. Holding final position...")
    print("    Press Q in PyBullet window or wait 5 seconds to exit.")
    viz.visualize_configuration(trajectory[-1], duration=5.0)

    viz.close()

    print("\n" + "=" * 60)
    print("VISUALIZATION COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    main()
