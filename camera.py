"""Camera helpers for PyBullet visualization."""

from __future__ import annotations

from models import CameraConfig, Vec3


def apply_camera(p, cam: CameraConfig) -> None:
    """Apply a camera configuration using :func:`pybullet.resetDebugVisualizerCamera`."""

    target: Vec3 = cam.target_m
    p.resetDebugVisualizerCamera(
        cameraDistance=cam.distance,
        cameraYaw=cam.yaw_deg,
        cameraPitch=cam.pitch_deg,
        cameraTargetPosition=target,
    )


__all__ = ["apply_camera"]
