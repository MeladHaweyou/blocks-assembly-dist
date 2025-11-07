"""Data model for block assembly definitions.

This module implements a sectioned CSV schema that separates
materials, reusable block types, and assembly instances while still
supporting the legacy single-section layout used by early versions of
the app.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

import math
import pandas as pd

from backend_bullet import BlockSpec

SECTION_COLUMN = "section"
MATERIAL_SECTION = "material"
BLOCK_TYPE_SECTION = "block_type"
ASSEMBLY_SECTION = "assembly"

SECTION_ORDER = [MATERIAL_SECTION, BLOCK_TYPE_SECTION, ASSEMBLY_SECTION]

# Canonical column order for the sectioned CSV. Additional columns are
# preserved when round-tripping through the editor but are ignored by
# the parser.
SECTIONED_COLUMNS = [
    SECTION_COLUMN,
    "id",
    "name",
    "material",
    "material_override",
    "density",
    "mu_slide",
    "mu_spin",
    "mu_roll",
    "size_x",
    "size_y",
    "size_z",
    "mass",
    "block_type",
    "order",
    "posX",
    "posY",
    "posZ",
    "rotX",
    "rotY",
    "rotZ",
    "drop_height",
]

DROP_CLAMP_MAX = 0.02


class AssemblyValidationError(ValueError):
    """Raised when the CSV contents cannot be converted to a data model."""

    def __init__(self, errors: Iterable[str]):
        self.messages = [str(e) for e in errors if str(e).strip()]
        super().__init__("\n".join(self.messages))

    def __bool__(self) -> bool:  # pragma: no cover - convenience only
        return bool(self.messages)


@dataclass(frozen=True)
class Material:
    material_id: str
    name: str
    density: float
    mu_slide: float
    mu_spin: float = 0.0
    mu_roll: float = 0.0
    restitution: float = 0.01
    anisotropic_friction: Optional[Tuple[float, float, float]] = None

    def friction_tuple(self) -> Tuple[float, float, float]:
        return (self.mu_slide, self.mu_spin, self.mu_roll)


@dataclass(frozen=True)
class BlockType:
    type_id: str
    name: str
    size_x: float
    size_y: float
    size_z: float
    material_id: str
    mass_override: Optional[float] = None

    def volume(self) -> float:
        return float(self.size_x) * float(self.size_y) * float(self.size_z)

    def mass(self, material: Material) -> float:
        if self.mass_override is not None and self.mass_override > 0.0:
            return float(self.mass_override)
        return float(material.density) * self.volume()


@dataclass(frozen=True)
class AssemblyBlock:
    name: str
    order: int
    block_type_id: str
    posX: float
    posY: float
    posZ: float
    rotX: float
    rotY: float
    rotZ: float
    drop_height: float
    material_override_id: Optional[str] = None


@dataclass
class AssemblyData:
    materials: Dict[str, Material] = field(default_factory=dict)
    block_types: Dict[str, BlockType] = field(default_factory=dict)
    assembly: List[AssemblyBlock] = field(default_factory=list)

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------
    @classmethod
    def from_dataframe(cls, df: pd.DataFrame) -> "AssemblyData":
        if SECTION_COLUMN not in df.columns:
            return cls._from_legacy(df)
        errors: List[str] = []
        materials: Dict[str, Material] = {}
        block_types: Dict[str, BlockType] = {}
        assembly_blocks: List[AssemblyBlock] = []

        # Normalise column labels for lookups.
        frame = df.copy()
        frame.columns = [str(c) for c in frame.columns]

        for row_idx, (_, row) in enumerate(frame.iterrows(), start=1):
            section_raw = row.get(SECTION_COLUMN)
            section = str(section_raw).strip().lower() if pd.notna(section_raw) else ""
            if not section:
                # Allow empty rows for editing convenience.
                if not row.drop(labels=[SECTION_COLUMN], errors="ignore").dropna().any():
                    continue
                errors.append(f"Row {row_idx}: missing '{SECTION_COLUMN}' value.")
                continue
            if section not in SECTION_ORDER:
                errors.append(f"Row {row_idx}: unsupported section '{section_raw}'.")
                continue
            if section == MATERIAL_SECTION:
                mat = _parse_material(row_idx, row, errors)
                if mat:
                    if mat.material_id in materials:
                        errors.append(
                            f"Row {row_idx}: duplicate material id '{mat.material_id}'."
                        )
                    else:
                        materials[mat.material_id] = mat
            elif section == BLOCK_TYPE_SECTION:
                bt = _parse_block_type(row_idx, row, errors)
                if bt:
                    if bt.type_id in block_types:
                        errors.append(
                            f"Row {row_idx}: duplicate block type id '{bt.type_id}'."
                        )
                    else:
                        block_types[bt.type_id] = bt
            elif section == ASSEMBLY_SECTION:
                blk = _parse_assembly_block(row_idx, row, errors)
                if blk:
                    assembly_blocks.append(blk)

        data = cls(materials=materials, block_types=block_types, assembly=assembly_blocks)
        errors.extend(data.validate())
        if errors:
            raise AssemblyValidationError(errors)
        return data

    @classmethod
    def _from_legacy(cls, df: pd.DataFrame) -> "AssemblyData":
        required = {
            "name",
            "order",
            "size_x",
            "size_y",
            "size_z",
            "mass",
            "mu_slide",
            "posX",
            "posY",
            "posZ",
            "rotX",
            "rotY",
            "rotZ",
        }
        missing = [c for c in required if c not in df.columns]
        if missing:
            raise AssemblyValidationError(
                [
                    "Legacy CSV is missing required columns: "
                    + ", ".join(sorted(missing))
                ]
            )
        materials: Dict[str, Material] = {}
        block_types: Dict[str, BlockType] = {}
        assembly_blocks: List[AssemblyBlock] = []
        errors: List[str] = []

        for idx, (_, row) in enumerate(df.iterrows(), start=1):
            try:
                name = str(row["name"]).strip()
                order = int(row["order"])
            except Exception:
                errors.append(f"Row {idx}: failed to parse name/order in legacy CSV.")
                continue
            mat_id = f"legacy_material_{idx}"
            try:
                sx = float(row["size_x"])
                sy = float(row["size_y"])
                sz = float(row["size_z"])
                vol = sx * sy * sz
                mass = float(row["mass"])
                density = mass / vol if vol > 0 else 0.0
            except Exception:
                errors.append(f"{name or f'Row {idx}'}: invalid size or mass values.")
                continue
            mu_slide = float(row.get("mu_slide", 0.6))
            mu_spin = float(row.get("mu_spin", 0.0))
            mu_roll = float(row.get("mu_roll", 0.0))
            materials[mat_id] = Material(
                material_id=mat_id,
                name=f"Legacy material {idx}",
                density=density if density > 0 else 750.0,
                mu_slide=mu_slide,
                mu_spin=mu_spin,
                mu_roll=mu_roll,
                restitution=0.01,
            )
            bt_id = f"legacy_block_type_{idx}"
            block_types[bt_id] = BlockType(
                type_id=bt_id,
                name=f"Legacy block {idx}",
                size_x=sx,
                size_y=sy,
                size_z=sz,
                material_id=mat_id,
                mass_override=mass,
            )
            drop_val = float(row.get("drop_height", 0.0) or 0.0)
            assembly_blocks.append(
                AssemblyBlock(
                    name=name or f"Block {idx}",
                    order=order,
                    block_type_id=bt_id,
                    posX=float(row["posX"]),
                    posY=float(row["posY"]),
                    posZ=float(row["posZ"]),
                    rotX=float(row["rotX"]),
                    rotY=float(row["rotY"]),
                    rotZ=float(row["rotZ"]),
                    drop_height=drop_val,
                )
            )
        data = cls(materials=materials, block_types=block_types, assembly=assembly_blocks)
        errors.extend(data.validate())
        if errors:
            raise AssemblyValidationError(errors)
        return data

    # ------------------------------------------------------------------
    # Validation helpers
    # ------------------------------------------------------------------
    def validate(self) -> List[str]:
        errors: List[str] = []
        seen_orders: Dict[int, str] = {}
        for blk in self.assembly:
            if blk.block_type_id not in self.block_types:
                errors.append(
                    f"Assembly block '{blk.name}' references unknown block type '{blk.block_type_id}'."
                )
            mat_id = blk.material_override_id or self.block_types.get(blk.block_type_id, BlockType("", "", 0, 0, 0, "")).material_id
            if mat_id and mat_id not in self.materials:
                errors.append(
                    f"Assembly block '{blk.name}' references unknown material '{mat_id}'."
                )
            if blk.order in seen_orders:
                errors.append(
                    f"Duplicate order {blk.order} for blocks '{blk.name}' and '{seen_orders[blk.order]}'."
                )
            else:
                seen_orders[blk.order] = blk.name
        for bt in self.block_types.values():
            if bt.material_id not in self.materials:
                errors.append(
                    f"Block type '{bt.type_id}' references unknown material '{bt.material_id}'."
                )
            if bt.size_x <= 0 or bt.size_y <= 0 or bt.size_z <= 0:
                errors.append(
                    f"Block type '{bt.type_id}' must have positive dimensions."
                )
        return errors

    # ------------------------------------------------------------------
    # Export helpers
    # ------------------------------------------------------------------
    def to_dataframe(self) -> pd.DataFrame:
        rows: List[Dict[str, object]] = []
        for mat in self.materials.values():
            rows.append(
                {
                    SECTION_COLUMN: MATERIAL_SECTION,
                    "id": mat.material_id,
                    "name": mat.name,
                    "density": mat.density,
                    "mu_slide": mat.mu_slide,
                    "mu_spin": mat.mu_spin,
                    "mu_roll": mat.mu_roll,
                    "restitution": mat.restitution,
                    "mu_slide_along": _format_anisotropic_component(mat, index=0),
                    "mu_slide_across": _format_anisotropic_component(mat, index=1),
                }
            )
        for bt in self.block_types.values():
            rows.append(
                {
                    SECTION_COLUMN: BLOCK_TYPE_SECTION,
                    "id": bt.type_id,
                    "name": bt.name,
                    "material": bt.material_id,
                    "size_x": bt.size_x,
                    "size_y": bt.size_y,
                    "size_z": bt.size_z,
                    "mass": bt.mass_override,
                }
            )
        for blk in sorted(self.assembly, key=lambda b: b.order):
            rows.append(
                {
                    SECTION_COLUMN: ASSEMBLY_SECTION,
                    "name": blk.name,
                    "order": blk.order,
                    "block_type": blk.block_type_id,
                    "material_override": blk.material_override_id,
                    "posX": blk.posX,
                    "posY": blk.posY,
                    "posZ": blk.posZ,
                    "rotX": blk.rotX,
                    "rotY": blk.rotY,
                    "rotZ": blk.rotZ,
                    "drop_height": blk.drop_height,
                }
            )
        df = pd.DataFrame(rows)
        for col in SECTIONED_COLUMNS:
            if col not in df:
                df[col] = pd.NA
        df = df[SECTIONED_COLUMNS + [c for c in df.columns if c not in SECTIONED_COLUMNS]]
        return df

    # ------------------------------------------------------------------
    # Simulation helpers
    # ------------------------------------------------------------------
    def to_block_specs(
        self,
        *,
        drop_height_override: Optional[float] = None,
    ) -> Tuple[List[BlockSpec], bool]:
        specs: List[BlockSpec] = []
        drop_clamped = False
        blocks = sorted(self.assembly, key=lambda b: b.order)
        for blk in blocks:
            block_type = self.block_types[blk.block_type_id]
            material = self.materials[
                blk.material_override_id or block_type.material_id
            ]
            drop_height = (
                float(drop_height_override)
                if drop_height_override is not None
                else float(blk.drop_height)
            )
            drop_height = max(0.0, float(drop_height))
            if drop_height > DROP_CLAMP_MAX:
                drop_clamped = True
                drop_height = DROP_CLAMP_MAX
            mass = block_type.mass(material)
            if not math.isfinite(mass) or mass <= 0:
                raise AssemblyValidationError(
                    [
                        f"Block '{blk.name}' results in non-positive mass. Check density/mass overrides."
                    ]
                )
            specs.append(
                BlockSpec(
                    name=blk.name,
                    order=blk.order,
                    size_x=float(block_type.size_x),
                    size_y=float(block_type.size_y),
                    size_z=float(block_type.size_z),
                    mass=float(mass),
                    mu_slide=float(material.mu_slide),
                    mu_spin=float(material.mu_spin),
                    mu_roll=float(material.mu_roll),
                    restitution=float(material.restitution),
                    anisotropic_friction=material.anisotropic_friction,
                    drop_height=float(drop_height),
                    posX=float(blk.posX),
                    posY=float(blk.posY),
                    posZ=float(blk.posZ),
                    rotX=float(blk.rotX),
                    rotY=float(blk.rotY),
                    rotZ=float(blk.rotZ),
                )
            )
        return specs, drop_clamped


# ----------------------------------------------------------------------
# Parsing helpers for sectioned CSV rows
# ----------------------------------------------------------------------

def _require_str(row_idx: int, value: object, field_name: str, errors: List[str]) -> Optional[str]:
    if pd.isna(value) or not str(value).strip():
        errors.append(f"Row {row_idx}: '{field_name}' is required.")
        return None
    return str(value).strip()


def _require_float(row_idx: int, value: object, field_name: str, errors: List[str], *, positive: bool = False) -> Optional[float]:
    if pd.isna(value):
        errors.append(f"Row {row_idx}: '{field_name}' is required.")
        return None
    try:
        val = float(value)
    except (TypeError, ValueError):
        errors.append(f"Row {row_idx}: '{field_name}' must be a number.")
        return None
    if positive and val <= 0:
        errors.append(f"Row {row_idx}: '{field_name}' must be positive.")
        return None
    return val


def _parse_material(row_idx: int, row: pd.Series, errors: List[str]) -> Optional[Material]:
    mat_id = _require_str(row_idx, row.get("id"), "id", errors)
    density = _require_float(row_idx, row.get("density"), "density", errors, positive=True)
    mu_slide = _require_float(row_idx, row.get("mu_slide", 0.6), "mu_slide", errors)
    mu_spin = _optional_float(row.get("mu_spin"), default=0.0)
    mu_roll = _optional_float(row.get("mu_roll"), default=0.0)
    restitution = _optional_float(row.get("restitution"), default=0.01)
    mu_along = _optional_float_or_none(row.get("mu_slide_along"))
    mu_across = _optional_float_or_none(row.get("mu_slide_across"))
    name = str(row.get("name") or mat_id or f"Material {row_idx}")
    if mat_id is None or density is None or mu_slide is None:
        return None
    anisotropic = None
    if (mu_along is not None or mu_across is not None) and mu_slide > 0:
        along = float(mu_along) if mu_along is not None else float(mu_slide)
        across = float(mu_across) if mu_across is not None else float(mu_slide)
        anisotropic = (
            float(along) / float(mu_slide),
            float(across) / float(mu_slide),
            1.0,
        )
    return Material(
        material_id=mat_id,
        name=name,
        density=float(density),
        mu_slide=float(mu_slide),
        mu_spin=float(mu_spin),
        mu_roll=float(mu_roll),
        restitution=float(restitution),
        anisotropic_friction=anisotropic,
    )


def _parse_block_type(row_idx: int, row: pd.Series, errors: List[str]) -> Optional[BlockType]:
    type_id = _require_str(row_idx, row.get("id"), "id", errors)
    material_id = _require_str(row_idx, row.get("material"), "material", errors)
    size_x = _require_float(row_idx, row.get("size_x"), "size_x", errors, positive=True)
    size_y = _require_float(row_idx, row.get("size_y"), "size_y", errors, positive=True)
    size_z = _require_float(row_idx, row.get("size_z"), "size_z", errors, positive=True)
    if None in (type_id, material_id, size_x, size_y, size_z):
        return None
    name = str(row.get("name") or type_id)
    mass_override = None
    mass_val = row.get("mass")
    if pd.notna(mass_val) and str(mass_val).strip():
        try:
            mass_override = float(mass_val)
        except (TypeError, ValueError):
            errors.append(f"Row {row_idx}: 'mass' must be numeric when provided.")
            mass_override = None
    return BlockType(
        type_id=type_id,
        name=name,
        size_x=float(size_x),
        size_y=float(size_y),
        size_z=float(size_z),
        material_id=material_id,
        mass_override=mass_override,
    )


def _parse_assembly_block(row_idx: int, row: pd.Series, errors: List[str]) -> Optional[AssemblyBlock]:
    name = _require_str(row_idx, row.get("name"), "name", errors)
    order_val = _require_float(row_idx, row.get("order"), "order", errors)
    block_type_id = _require_str(row_idx, row.get("block_type"), "block_type", errors)
    posX = _require_float(row_idx, row.get("posX"), "posX", errors)
    posY = _require_float(row_idx, row.get("posY"), "posY", errors)
    posZ = _require_float(row_idx, row.get("posZ"), "posZ", errors)
    rotX = _require_float(row_idx, row.get("rotX"), "rotX", errors)
    rotY = _require_float(row_idx, row.get("rotY"), "rotY", errors)
    rotZ = _require_float(row_idx, row.get("rotZ"), "rotZ", errors)
    drop_height = _optional_float(row.get("drop_height"), default=0.0)
    if None in (name, order_val, block_type_id, posX, posY, posZ, rotX, rotY, rotZ):
        return None
    material_override = row.get("material_override")
    material_id = None
    if pd.notna(material_override) and str(material_override).strip():
        material_id = str(material_override).strip()
    return AssemblyBlock(
        name=name,
        order=int(order_val),
        block_type_id=block_type_id,
        posX=float(posX),
        posY=float(posY),
        posZ=float(posZ),
        rotX=float(rotX),
        rotY=float(rotY),
        rotZ=float(rotZ),
        drop_height=float(drop_height),
        material_override_id=material_id,
    )


def _optional_float(value: object, *, default: float = 0.0) -> float:
    if pd.isna(value) or value is None or str(value).strip() == "":
        return float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _optional_float_or_none(value: object) -> Optional[float]:
    if pd.isna(value) or value is None:
        return None
    s = str(value).strip()
    if not s:
        return None
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _format_anisotropic_component(mat: Material, index: int) -> Optional[float]:
    if mat.anisotropic_friction is None:
        return None
    if not (0 <= index < len(mat.anisotropic_friction)):
        return None
    factor = mat.anisotropic_friction[index]
    if mat.mu_slide <= 0:
        return None
    value = mat.mu_slide * factor
    return float(value)


__all__ = [
    "AssemblyBlock",
    "AssemblyData",
    "AssemblyValidationError",
    "BlockType",
    "Material",
    "SECTION_COLUMN",
    "SECTIONED_COLUMNS",
    "DROP_CLAMP_MAX",
]
