"""Core abstractions for robot arm motion planning.

This module provides the fundamental building blocks:
- RobotModel: Abstract interface for any robot arm
- UR5RobotModel: Concrete implementation for UR5
- JointStateSpace: Configuration space operations
- CollisionManager: Collision checking abstractions
- OPWKinematics: Closed-form inverse kinematics
"""

from core.robot_model import RobotModel, UR5RobotModel
from core.state_space import JointStateSpace
from core.collision_manager import (
    CollisionManager,
    CollisionShape,
    Box,
    Sphere,
    Cylinder,
    NullCollisionManager,
    ShapeCollisionManager,
    SimpleSelfCollisionManager,
)

__all__ = [
    # Robot models
    "RobotModel",
    "UR5RobotModel",
    # State space
    "JointStateSpace",
    # Collision
    "CollisionManager",
    "CollisionShape",
    "Box",
    "Sphere",
    "Cylinder",
    "NullCollisionManager",
    "ShapeCollisionManager",
    "SimpleSelfCollisionManager",
]
