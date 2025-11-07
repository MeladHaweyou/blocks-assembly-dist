"""Validation helpers for assembly metadata and instance resolution."""

from __future__ import annotations

from typing import Dict, Iterable, List

from models import (
    AssemblyInstance,
    BlockType,
    GlobalConfig,
    Material,
    ResolvedInstance,
)


def apply_meta_to_config(meta: Dict[str, str], cfg: GlobalConfig) -> GlobalConfig:
    """Apply metadata overrides from the CSV to a :class:`GlobalConfig`."""

    lowered = {k.lower(): v for k, v in meta.items()}

    placement = lowered.get("placement_mode")
    if placement is not None:
        placement_l = placement.lower()
        if placement_l in ("gentle", "dynamic"):
            cfg.placement_mode = placement_l  # type: ignore[assignment]

    if "servo_clearance_mm" in lowered:
        cfg.servo_clearance_m = float(lowered["servo_clearance_mm"]) / 1000.0

    if "drop_distance_mm" in lowered:
        cfg.drop_distance_m = float(lowered["drop_distance_mm"]) / 1000.0

    if "preview_hold_steps" in lowered:
        cfg.preview_hold_steps = int(float(lowered["preview_hold_steps"]))

    if "micro_drop_height_mm" in lowered:
        cfg.micro_drop_height_m = float(lowered["micro_drop_height_mm"]) / 1000.0

    if "micro_drop_frames" in lowered:
        cfg.micro_drop_frames = int(float(lowered["micro_drop_frames"]))

    if "settle_hold_steps" in lowered:
        cfg.settle_hold_steps = int(float(lowered["settle_hold_steps"]))

    if "dwell_time_s" in lowered:
        cfg.dwell_time_s = float(lowered["dwell_time_s"])

    if "camera_distance" in lowered:
        cfg.camera.distance = float(lowered["camera_distance"])

    if "camera_yaw_deg" in lowered:
        cfg.camera.yaw_deg = float(lowered["camera_yaw_deg"])

    if "camera_pitch_deg" in lowered:
        cfg.camera.pitch_deg = float(lowered["camera_pitch_deg"])

    if all(key in lowered for key in ("camera_target_x", "camera_target_y", "camera_target_z")):
        cfg.camera.target_m = (
            float(lowered["camera_target_x"]),
            float(lowered["camera_target_y"]),
            float(lowered["camera_target_z"]),
        )

    return cfg


def _append_error(errors: List[str], message: str) -> None:
    if message:
        errors.append(message)


def validate_and_resolve(
    materials: Dict[str, Material],
    block_types: Dict[str, BlockType],
    assembly: Iterable[AssemblyInstance],
) -> List[ResolvedInstance]:
    """Validate referential integrity and resolve assembly instances.

    Raises
    ------
    ValueError
        If any validation rule fails. The error message aggregates all
        detected issues for easier correction by callers.
    """

    errors: List[str] = []

    # Allow zero-density for static supports (massless rigid bodies).
    # Still forbid negative densities.
    for name, mat in materials.items():
        if mat.density_kgm3 < 0:
            _append_error(errors, f"Material '{name}' has negative density.")

    for name, block in block_types.items():
        if block.material_name not in materials:
            _append_error(
                errors,
                f"Block type '{name}' references missing material '{block.material_name}'.",
            )
        if block.size_x_m <= 0 or block.size_y_m <= 0 or block.size_z_m <= 0:
            _append_error(errors, f"Block type '{name}' has non-positive size.")
        # New: wedge parameter checks
        if (block.shape or "box").lower() == "wedge":
            if (block.wedge_run_x_m is None) or (block.wedge_run_x_m <= 0):
                _append_error(errors, f"Wedge '{name}' requires positive wedge_run_x_m.")
            if (block.wedge_width_y_m is None) or (block.wedge_width_y_m <= 0):
                _append_error(errors, f"Wedge '{name}' requires positive wedge_width_y_m.")
            if (block.wedge_rise_z_m is None) or (block.wedge_rise_z_m <= 0):
                _append_error(errors, f"Wedge '{name}' requires positive wedge_rise_z_m.")

    seen_orders = set()
    instances: List[AssemblyInstance] = []
    for inst in assembly:
        instances.append(inst)
        if inst.block_type not in block_types:
            _append_error(
                errors,
                f"Assembly row order {inst.order} references unknown block_type '{inst.block_type}'.",
            )
        if inst.order in seen_orders:
            _append_error(errors, f"Duplicate 'order' value: {inst.order}.")
        seen_orders.add(inst.order)

    if errors:
        raise ValueError("\n".join(errors))

    resolved = [
        ResolvedInstance(
            instance=inst,
            block_type=block_types[inst.block_type],
            material=materials[block_types[inst.block_type].material_name],
        )
        for inst in sorted(instances, key=lambda a: a.order)
    ]

    return resolved


__all__ = ["apply_meta_to_config", "validate_and_resolve"]
