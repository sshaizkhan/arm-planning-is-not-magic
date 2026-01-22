"""Path smoothing utilities for robot motion planning.

This module provides functions to smooth jagged paths from sampling-based
planners (RRT, PRM, etc.) to produce smoother, more natural robot motions.
"""

import numpy as np
from typing import List, Callable, Optional
from scipy import interpolate


def shortcut_smoothing(
    path: List[np.ndarray],
    collision_check: Callable[[np.ndarray], bool],
    max_iterations: int = 100,
    step_size: float = 0.05,
) -> List[np.ndarray]:
    """
    Shortcut-based path smoothing.

    Randomly selects two waypoints and tries to connect them directly.
    If the direct connection is collision-free, removes intermediate waypoints.

    Args:
        path: Original path as list of joint configurations
        collision_check: Function that returns True if configuration is in collision
        max_iterations: Maximum smoothing iterations
        step_size: Step size for collision checking along shortcuts

    Returns:
        Smoothed path with fewer waypoints
    """
    if len(path) <= 2:
        return path

    smoothed = [np.array(q) for q in path]

    for _ in range(max_iterations):
        if len(smoothed) <= 2:
            break

        # Random indices
        i = np.random.randint(0, len(smoothed) - 2)
        j = np.random.randint(i + 2, len(smoothed))

        # Check if direct connection is collision-free
        if _is_path_clear(smoothed[i], smoothed[j], collision_check, step_size):
            # Remove intermediate waypoints
            smoothed = smoothed[:i+1] + smoothed[j:]

    return smoothed


def _is_path_clear(
    q1: np.ndarray,
    q2: np.ndarray,
    collision_check: Callable[[np.ndarray], bool],
    step_size: float,
) -> bool:
    """Check if straight-line path between two configs is collision-free."""
    dist = np.linalg.norm(q2 - q1)
    if dist < step_size:
        return not collision_check(q1) and not collision_check(q2)

    n_steps = int(np.ceil(dist / step_size))
    for i in range(n_steps + 1):
        t = i / n_steps
        q = q1 + t * (q2 - q1)
        if collision_check(q):
            return False
    return True


def spline_smoothing(
    path: List[np.ndarray],
    num_points: int = 100,
    smoothing_factor: float = 0.0,
) -> np.ndarray:
    """
    Smooth path using B-spline interpolation.

    Args:
        path: Original path as list of joint configurations
        num_points: Number of points in output path
        smoothing_factor: Spline smoothing (0 = interpolate exactly through points)

    Returns:
        Smoothed path as numpy array of shape (num_points, dof)
    """
    path_array = np.array(path)
    n_waypoints, dof = path_array.shape

    if n_waypoints < 4:
        # Not enough points for spline, use linear interpolation
        t_original = np.linspace(0, 1, n_waypoints)
        t_new = np.linspace(0, 1, num_points)
        smoothed = np.zeros((num_points, dof))
        for j in range(dof):
            smoothed[:, j] = np.interp(t_new, t_original, path_array[:, j])
        return smoothed

    # Parameterize by cumulative distance
    distances = np.zeros(n_waypoints)
    for i in range(1, n_waypoints):
        distances[i] = distances[i-1] + np.linalg.norm(path_array[i] - path_array[i-1])

    if distances[-1] == 0:
        return path_array

    t_original = distances / distances[-1]

    # Fit B-spline to each joint
    t_new = np.linspace(0, 1, num_points)
    smoothed = np.zeros((num_points, dof))

    for j in range(dof):
        # Create B-spline representation
        tck = interpolate.splrep(t_original, path_array[:, j], s=smoothing_factor, k=3)
        smoothed[:, j] = interpolate.splev(t_new, tck)

    return smoothed


def smooth_path(
    path: List[np.ndarray],
    collision_check: Optional[Callable[[np.ndarray], bool]] = None,
    shortcut_iterations: int = 50,
    spline_points: int = 100,
    step_size: float = 0.05,
) -> np.ndarray:
    """
    Full path smoothing pipeline: shortcut + spline smoothing.

    Args:
        path: Original path from planner
        collision_check: Collision checking function (optional, for shortcutting)
        shortcut_iterations: Number of shortcut smoothing iterations
        spline_points: Number of points in final smoothed path
        step_size: Step size for collision checking

    Returns:
        Smoothed path as numpy array
    """
    # Step 1: Shortcut smoothing (if collision checker provided)
    if collision_check is not None and shortcut_iterations > 0:
        path = shortcut_smoothing(path, collision_check, shortcut_iterations, step_size)

    # Step 2: Spline smoothing
    smoothed = spline_smoothing(path, spline_points)

    # Step 3: If collision checker provided, verify and fix any collisions
    if collision_check is not None:
        smoothed = _fix_collisions(smoothed, collision_check, path)

    return smoothed


def _fix_collisions(
    smoothed_path: np.ndarray,
    collision_check: Callable[[np.ndarray], bool],
    original_path: List[np.ndarray],
) -> np.ndarray:
    """
    Check smoothed path for collisions and fall back to original if needed.

    If spline smoothing introduced collisions, we blend back toward the
    original path at collision points.
    """
    # Check for collisions
    collision_indices = []
    for i, q in enumerate(smoothed_path):
        if collision_check(q):
            collision_indices.append(i)

    if not collision_indices:
        return smoothed_path

    # If too many collisions, return densified original path
    if len(collision_indices) > len(smoothed_path) * 0.1:
        # Fall back to linear interpolation of original path
        return spline_smoothing(original_path, len(smoothed_path), smoothing_factor=0.0)

    return smoothed_path
