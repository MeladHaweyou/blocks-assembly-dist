"""PyBullet simulation runner utilities."""
from __future__ import annotations

import time
from dataclasses import replace
from typing import List

import pybullet as p
import pybullet_data

from camera import apply_camera
from models import CameraConfig, GlobalConfig, ResolvedInstance, euler_deg_to_quat


def run_sequence(cfg: GlobalConfig, resolved: List[ResolvedInstance], stop_evt) -> None:
    """Simulate a sequence of resolved block instances."""

    connection_mode = p.GUI if cfg.gui else p.DIRECT
    cid = p.connect(connection_mode)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0.0, 0.0, -9.81)
        p.setTimeStep(1.0 / 240.0)
        try:
            p.setPhysicsEngineParameter(
                numSolverIterations=150,
                enableConeFriction=1,
                frictionAnchor=1,
                deterministicOverlappingPairs=1,
                enableSplitImpulse=1,
                splitImpulsePenetrationThreshold=-0.02,
                contactBreakingThreshold=1e-3,
                contactSlop=1e-4,
                numSubSteps=2,
            )
        except TypeError:
            # Older PyBullet builds may not accept all keyword arguments.
            p.setPhysicsEngineParameter(numSolverIterations=150)
            p.setPhysicsEngineParameter(numSubSteps=2)
        plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(
            plane_id,
            -1,
            lateralFriction=1.0,
            spinningFriction=0.0,
            rollingFriction=0.0,
            restitution=0.01,
        )

        camera_cfg = cfg.camera
        distance = camera_cfg.distance if camera_cfg.distance and camera_cfg.distance > 0 else 1.0
        camera_to_use: CameraConfig = replace(camera_cfg, distance=distance)
        apply_camera(p, camera_to_use)

        if cfg.gui and cfg.pre_spawn_delay_s > 0:
            end_time = time.time() + cfg.pre_spawn_delay_s
            while time.time() < end_time:
                if stop_evt.is_set():
                    return
                time.sleep(0.05)

        for resolved_inst in resolved:
            if stop_evt.is_set():
                break

            body_id = _spawn_block_instance(p, resolved_inst)

            if cfg.placement_mode == "gentle":
                _servo_place(
                    p,
                    body_id,
                    resolved_inst,
                    cfg,
                    stop_evt=stop_evt,
                )
            else:
                _drop_place(
                    p,
                    body_id,
                    resolved_inst,
                    cfg,
                    stop_evt=stop_evt,
                )

            delay = getattr(resolved_inst.instance, "delay_after_s", 0.0) or 0.0
            if delay > 0:
                _step_for(p, delay, stop_evt)

        _wait_to_settle_or_stop(p, stop_evt, timeout_s=5.0)
    finally:
        p.disconnect(cid)


def _step_for(p_module, seconds: float, stop_evt) -> None:
    if seconds <= 0:
        return
    end = time.time() + seconds
    while time.time() < end and not stop_evt.is_set():
        p_module.stepSimulation()
        time.sleep(1.0 / 240.0)


def _spawn_block_instance(p_module, resolved_inst: ResolvedInstance) -> int:
    bt = resolved_inst.block_type
    if (bt.shape or "box").lower() == "wedge":
        run = bt.wedge_run_x_m or bt.size_x_m
        width = bt.wedge_width_y_m or bt.size_y_m
        rise = bt.wedge_rise_z_m or bt.size_z_m
        rx = float(run); wy = float(width); rz = float(rise)
        # Right-triangular prism (slope along +X) with VERTICALLY-CENTERED vertices:
        # z spans [-rz/2, +rz/2], so posZ acts like a center height (consistent with boxes).
        z0 = -rz / 2.0
        z1 = +rz / 2.0
        verts = [
            (-rx/2, -wy/2, z0), ( rx/2, -wy/2, z0), (-rx/2, -wy/2, z1),
            (-rx/2,  wy/2, z0), ( rx/2,  wy/2, z0), (-rx/2,  wy/2, z1),
        ]
        shape_type = p_module.GEOM_CONVEX_HULL if hasattr(p_module, "GEOM_CONVEX_HULL") else p_module.GEOM_MESH
        col_shape = p_module.createCollisionShape(shape_type, vertices=verts)
        half_extents = [rx/2, wy/2, max(rz, 1e-6)/2]  # used for CCD/fallbacks as needed
    else:
        half_extents = [
            bt.size_x_m / 2.0,
            bt.size_y_m / 2.0,
            bt.size_z_m / 2.0,
        ]
        col_shape = p_module.createCollisionShape(p_module.GEOM_BOX, halfExtents=half_extents)
    visual_shape = -1
    body_id = p_module.createMultiBody(
        baseMass=resolved_inst.mass_kg,  # 0 mass -> static wedge support
        baseCollisionShapeIndex=col_shape,
        baseVisualShapeIndex=visual_shape,
    )

    quat = euler_deg_to_quat(resolved_inst.instance.rot_deg)
    p_module.resetBasePositionAndOrientation(body_id, resolved_inst.instance.pos_m, quat)
    r_min = 0.25 * max(min(half_extents), 1e-6)
    dyn_kwargs = dict(
        lateralFriction=resolved_inst.material.mu_slide,
        rollingFriction=resolved_inst.material.mu_roll,
        spinningFriction=resolved_inst.material.mu_spin,
        restitution=resolved_inst.material.restitution,
        frictionAnchor=1,
        linearDamping=0.01,
        angularDamping=0.02,
        ccdSweptSphereRadius=r_min,
        contactProcessingThreshold=0,
    )
    if resolved_inst.material.anisotropic_friction is not None:
        dyn_kwargs["anisotropicFriction"] = list(resolved_inst.material.anisotropic_friction)
    p_module.changeDynamics(body_id, -1, **dyn_kwargs)
    return body_id


def _servo_place(
    p_module,
    body_id: int,
    resolved_inst: ResolvedInstance,
    cfg: GlobalConfig,
    *,
    stop_evt,
) -> None:
    target_pos = resolved_inst.instance.pos_m
    orientation = euler_deg_to_quat(resolved_inst.instance.rot_deg)

    clearance = max(cfg.servo_clearance_m, cfg.micro_drop_height_m)
    approach_offset = clearance + max(cfg.micro_drop_height_m, 0.0)
    start_pos = (
        target_pos[0],
        target_pos[1],
        target_pos[2] + approach_offset,
    )
    hold_pos = (
        target_pos[0],
        target_pos[1],
        target_pos[2] + clearance,
    )
    p_module.resetBasePositionAndOrientation(body_id, start_pos, orientation)

    approach_steps = max(cfg.preview_hold_steps, 1)
    for step in range(approach_steps):
        if stop_evt.is_set():
            return
        alpha = (step + 1) / approach_steps
        interp_pos = tuple(
            start + (hold - start) * alpha for start, hold in zip(start_pos, hold_pos)
        )
        p_module.resetBasePositionAndOrientation(body_id, interp_pos, orientation)
        p_module.resetBaseVelocity(body_id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        p_module.stepSimulation()
        time.sleep(1.0 / 240.0)

    preview_steps = max(cfg.preview_hold_steps, 1)
    for _ in range(preview_steps):
        if stop_evt.is_set():
            return
        p_module.resetBasePositionAndOrientation(body_id, hold_pos, orientation)
        p_module.resetBaseVelocity(body_id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        p_module.stepSimulation()
        time.sleep(1.0 / 240.0)

    drop_steps = cfg.micro_drop_frames if cfg.micro_drop_frames > 0 else (1 if cfg.micro_drop_height_m > 0 else 0)
    p_module.resetBaseVelocity(body_id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    for _ in range(drop_steps):
        if stop_evt.is_set():
            return
        p_module.stepSimulation()
        time.sleep(1.0 / 240.0)

    settle_steps = max(cfg.settle_hold_steps, 0)
    for _ in range(settle_steps):
        if stop_evt.is_set():
            return
        p_module.resetBaseVelocity(body_id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        p_module.stepSimulation()
        time.sleep(1.0 / 240.0)

    if cfg.dwell_time_s > 0:
        _step_for(p_module, cfg.dwell_time_s, stop_evt)


def _drop_place(
    p_module,
    body_id: int,
    resolved_inst: ResolvedInstance,
    cfg: GlobalConfig,
    *,
    stop_evt,
) -> None:
    target_pos = resolved_inst.instance.pos_m
    orientation = euler_deg_to_quat(resolved_inst.instance.rot_deg)
    start_pos = (
        target_pos[0],
        target_pos[1],
        target_pos[2] + max(cfg.drop_distance_m, 0.0),
    )
    p_module.resetBasePositionAndOrientation(body_id, start_pos, orientation)
    p_module.resetBaseVelocity(body_id, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    for _ in range(240):
        if stop_evt.is_set():
            return
        p_module.stepSimulation()
        time.sleep(1.0 / 240.0)

    p_module.resetBasePositionAndOrientation(body_id, target_pos, orientation)
    if cfg.dwell_time_s > 0:
        _step_for(p_module, cfg.dwell_time_s, stop_evt)


def _wait_to_settle_or_stop(p_module, stop_evt, timeout_s: float) -> None:
    start_time = time.time()
    while (time.time() - start_time) < timeout_s:
        if stop_evt.is_set():
            return
        p_module.stepSimulation()
        time.sleep(1.0 / 240.0)


__all__ = ["run_sequence"]

