"""Core data models for block assembly simulations."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple, Literal

import math

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Material:
    """Physical description of a block material."""

    name: str
    density_kgm3: float
    mu_slide: float = 0.55
    mu_spin: float = 0.02
    mu_roll: float = 0.01
    restitution: float = 0.01
    anisotropic_friction: Optional[Tuple[float, float, float]] = None
    color_rgba: Optional[Tuple[float, float, float, float]] = None


@dataclass(frozen=True)
class BlockType:
    """Reusable block geometry tied to a material."""

    name: str  # a.k.a. block_type
    size_x_m: float
    size_y_m: float
    size_z_m: float
    material_name: str
    # New: geometric family (default 'box'); 'wedge' = right-triangular prism
    shape: str = "box"  # "box" | "wedge"
    # New: wedge-specific dimensions (run along X, width along Y, rise along Z)
    wedge_run_x_m: Optional[float] = None
    wedge_width_y_m: Optional[float] = None
    wedge_rise_z_m: Optional[float] = None
    color_rgba: Optional[Tuple[float, float, float, float]] = None

    @property
    def volume_m3(self) -> float:
        """Return the block volume in cubic meters (wedge uses 0.5 * run * width * rise)."""
        if (self.shape or "box").lower() == "wedge":
            run = self.wedge_run_x_m if self.wedge_run_x_m is not None else self.size_x_m
            width = self.wedge_width_y_m if self.wedge_width_y_m is not None else self.size_y_m
            rise = self.wedge_rise_z_m if self.wedge_rise_z_m is not None else self.size_z_m
            return 0.5 * float(run) * float(width) * float(rise)
        return self.size_x_m * self.size_y_m * self.size_z_m


@dataclass(frozen=True)
class AssemblyInstance:
    """A placed block within an assembly definition."""

    order: int
    block_type: str
    pos_m: Vec3
    rot_deg: Vec3 = (0.0, 0.0, 0.0)
    label: Optional[str] = None
    delay_after_s: float = 0.0


@dataclass
class CameraConfig:
    """Camera preset applied when initializing the PyBullet GUI."""

    distance: float = 1.0  # default dist=1.00
    yaw_deg: float = 45.0
    pitch_deg: float = -35.0
    target_m: Vec3 = (0.0, 0.0, 0.0)


PlacementMode = Literal["gentle", "dynamic"]


@dataclass
class GlobalConfig:
    """High-level simulation configuration shared across modules."""

    placement_mode: PlacementMode = "gentle"
    drop_distance_m: float = 0.001  # used when placement_mode == "dynamic"
    servo_clearance_m: float = 0.0003  # used when placement_mode == "gentle"
    preview_hold_steps: int = 60
    micro_drop_height_m: float = 0.00025
    micro_drop_frames: int = 12
    settle_hold_steps: int = 8
    dwell_time_s: float = 0.20
    gui: bool = True
    pre_spawn_delay_s: float = 2.0
    camera: CameraConfig = field(default_factory=CameraConfig)


@dataclass(frozen=True)
class ResolvedInstance:
    """Assembly instance with its BlockType and Material resolved."""

    instance: AssemblyInstance
    block_type: BlockType
    material: Material

    @property
    def mass_kg(self) -> float:
        return self.material.density_kgm3 * self.block_type.volume_m3


def block_mass_kg(block_type: BlockType, material: Material) -> float:
    """Compute the physical mass for a block type using its material density."""

    return material.density_kgm3 * block_type.volume_m3


def euler_deg_to_rad(rot_deg: Vec3) -> Vec3:
    """Convert an Euler rotation in degrees to radians."""

    rx, ry, rz = rot_deg
    return (math.radians(rx), math.radians(ry), math.radians(rz))


def euler_deg_to_quat(rot_deg: Vec3) -> Quat:
    """Convert XYZ Euler degrees (roll, pitch, yaw) into a quaternion."""

    roll, pitch, yaw = euler_deg_to_rad(rot_deg)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)


def resolve_instances(
    instances: Iterable[AssemblyInstance],
    block_types: Dict[str, BlockType],
    materials: Dict[str, Material],
) -> List[ResolvedInstance]:
    """Bind block type and material definitions to assembly instances."""

    resolved: List[ResolvedInstance] = []
    for inst in instances:
        block = block_types.get(inst.block_type)
        if block is None:
            raise KeyError(f"Unknown block_type '{inst.block_type}' for order {inst.order}.")
        material = materials.get(block.material_name)
        if material is None:
            raise KeyError(
                f"Unknown material '{block.material_name}' referenced by block '{block.name}'."
            )
        resolved.append(ResolvedInstance(instance=inst, block_type=block, material=material))
    return resolved


__all__ = [
    "AssemblyInstance",
    "BlockType",
    "CameraConfig",
    "GlobalConfig",
    "Material",
    "PlacementMode",
    "Quat",
    "ResolvedInstance",
    "Vec3",
    "block_mass_kg",
    "euler_deg_to_quat",
    "euler_deg_to_rad",
    "resolve_instances",
]
