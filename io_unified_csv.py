"""CSV import/export helpers for the unified sectioned schema.

This module reads and writes the consolidated CSV layout that stores
meta settings, materials, block types, and assembly instances in a
single file.
"""
from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

from models import AssemblyInstance, BlockType, Material

MetaMapping = Dict[str, str]


class CsvFormatError(ValueError):
    """Raised when the CSV contents are invalid."""


def _read_csv_rows(path: str) -> List[dict]:
    try:
        with open(path, newline="", encoding="utf-8") as handle:
            return list(csv.DictReader(handle))
    except FileNotFoundError as exc:  # pragma: no cover - surfaced to caller
        raise exc
    except OSError as exc:  # pragma: no cover - surfaced to caller
        raise OSError(f"Failed to read CSV '{path}': {exc}") from exc


def _get(row: dict, primary: str, *aliases: str) -> str:
    for key in (primary, *aliases):
        if key in row:
            value = row.get(key) or ""
            if value is not None:
                value = value.strip()
            if value:
                return value
    return ""


def _parse_float(
    row: dict,
    field: str,
    *,
    default: Optional[float] = 0.0,
    context: str,
    row_number: int,
    aliases: Iterable[str] = (),
) -> float:
    raw = _get(row, field, *aliases)
    if raw == "":
        if default is None:
            raise CsvFormatError(
                f"Missing value for '{field}' in {context} row {row_number}."
            )
        return float(default)
    try:
        return float(raw)
    except ValueError as exc:
        raise CsvFormatError(
            f"Invalid float for '{field}' in {context} row {row_number}: {raw!r}"
        ) from exc


def _parse_optional_float(
    row: dict,
    field: str,
    *,
    context: str,
    row_number: int,
    aliases: Iterable[str] = (),
) -> Optional[float]:
    raw = _get(row, field, *aliases)
    if raw == "":
        return None
    try:
        return float(raw)
    except ValueError as exc:
        raise CsvFormatError(
            f"Invalid float for '{field}' in {context} row {row_number}: {raw!r}"
        ) from exc


def _parse_int(
    row: dict,
    field: str,
    *,
    default: Optional[int] = None,
    context: str,
    row_number: int,
    aliases: Iterable[str] = (),
) -> int:
    raw = _get(row, field, *aliases)
    if raw == "":
        if default is None:
            raise CsvFormatError(
                f"Missing value for '{field}' in {context} row {row_number}."
            )
        return int(default)
    try:
        return int(float(raw))
    except ValueError as exc:
        raise CsvFormatError(
            f"Invalid integer for '{field}' in {context} row {row_number}: {raw!r}"
        ) from exc


def _parse_rgba(s: Optional[str], *, context: str, row_number: int) -> Optional[Tuple[float, float, float, float]]:
    if not s:
        return None
    cleaned = s.strip().replace(";", ",")
    parts = [part.strip() for part in cleaned.split(",") if part.strip()]
    if len(parts) != 4:
        raise CsvFormatError(
            f"Expected four components for color_rgba in {context} row {row_number}, got {len(parts)}"
        )
    try:
        rgba = tuple(float(p) for p in parts)
    except ValueError as exc:
        raise CsvFormatError(
            f"Non-numeric color component in {context} row {row_number}: {parts!r}"
        ) from exc
    return rgba  # type: ignore[return-value]


def load_unified_csv(
    path: str,
) -> Tuple[MetaMapping, Dict[str, Material], Dict[str, BlockType], List[AssemblyInstance]]:
    """Load a unified CSV file."""

    rows = _read_csv_rows(path)
    if not rows:
        return {}, {}, {}, []

    has_section = any((_get(r, "section") or "").strip() for r in rows)
    if not has_section:
        raise CsvFormatError(
            "Missing 'section' column; only the unified, sectioned CSV is supported."
        )

    meta: MetaMapping = {}
    materials: Dict[str, Material] = {}
    block_types: Dict[str, BlockType] = {}
    assembly: List[AssemblyInstance] = []

    for index, row in enumerate(rows, start=2):
        section = _get(row, "section").lower()
        if not section:
            continue
        if section == "meta":
            key = _get(row, "key")
            value = _get(row, "value")
            if key:
                meta[key] = value
            continue

        if section == "materials":
            name = _get(row, "material", "name")
            if not name:
                continue
            if name in materials:
                raise CsvFormatError(f"Duplicate material '{name}' in row {index}.")
            density = _parse_float(
                row,
                "density_kgm3",
                context="materials",
                row_number=index,
                default=None,
                aliases=("density",),
            )
            mu_slide = _parse_float(
                row,
                "mu_slide",
                context="materials",
                row_number=index,
                default=0.55,
            )
            mu_spin = _parse_float(
                row,
                "mu_spin",
                context="materials",
                row_number=index,
                default=0.02,
            )
            mu_roll = _parse_float(
                row,
                "mu_roll",
                context="materials",
                row_number=index,
                default=0.01,
            )
            restitution = _parse_float(
                row,
                "restitution",
                context="materials",
                row_number=index,
                default=0.01,
            )
            mu_along = _parse_optional_float(
                row,
                "mu_slide_along",
                context="materials",
                row_number=index,
            )
            mu_across = _parse_optional_float(
                row,
                "mu_slide_across",
                context="materials",
                row_number=index,
            )
            anisotropic = None
            if (mu_along is not None or mu_across is not None) and mu_slide > 0:
                along = mu_along if mu_along is not None else mu_slide
                across = mu_across if mu_across is not None else mu_slide
                anisotropic = (
                    float(along) / float(mu_slide),
                    float(across) / float(mu_slide),
                    1.0,
                )
            materials[name] = Material(
                name=name,
                density_kgm3=density,
                mu_slide=mu_slide,
                mu_spin=mu_spin,
                mu_roll=mu_roll,
                restitution=restitution,
                anisotropic_friction=anisotropic,
                color_rgba=_parse_rgba(
                    _get(row, "color_rgba"), context="materials", row_number=index
                ),
            )
            continue

        if section == "block_types":
            bt_name = _get(row, "block_type", "name")
            if not bt_name:
                continue
            if bt_name in block_types:
                raise CsvFormatError(f"Duplicate block_type '{bt_name}' in row {index}.")
            material_name = _get(row, "material_of_type", "material", "material_name")
            if not material_name:
                raise CsvFormatError(
                    f"Missing material reference for block_type '{bt_name}' in row {index}."
                )
            # New: parse shape and optional wedge params
            shape = (_get(row, "shape") or "box").lower()
            wedge_kwargs: dict = {}
            if shape == "wedge":
                wedge_kwargs["wedge_run_x_m"] = _parse_float(
                    row, "wedge_run_x_m", context="block_types", row_number=index, default=None,
                    aliases=("run_x_m", "wedge_run", "run_m"),
                )
                wedge_kwargs["wedge_width_y_m"] = _parse_float(
                    row, "wedge_width_y_m", context="block_types", row_number=index, default=None,
                    aliases=("width_y_m", "wedge_width", "width_m"),
                )
                wedge_kwargs["wedge_rise_z_m"] = _parse_float(
                    row, "wedge_rise_z_m", context="block_types", row_number=index, default=None,
                    aliases=("rise_z_m", "wedge_rise", "rise_m"),
                )
            block_types[bt_name] = BlockType(
                name=bt_name,
                size_x_m=_parse_float(
                    row,
                    "size_x_m",
                    context="block_types",
                    row_number=index,
                    default=None,
                    aliases=("size_x",),
                ),
                size_y_m=_parse_float(
                    row,
                    "size_y_m",
                    context="block_types",
                    row_number=index,
                    default=None,
                    aliases=("size_y",),
                ),
                size_z_m=_parse_float(
                    row,
                    "size_z_m",
                    context="block_types",
                    row_number=index,
                    default=None,
                    aliases=("size_z",),
                ),
                material_name=material_name,
                shape=shape,
                color_rgba=_parse_rgba(
                    _get(row, "color_rgba"), context="block_types", row_number=index
                ),
                **wedge_kwargs,
            )
            continue

        if section == "assembly":
            order = _parse_int(
                row,
                "order",
                context="assembly",
                row_number=index,
                default=len(assembly) + 1,
            )
            block_type = _get(row, "block_type")
            if not block_type:
                raise CsvFormatError(
                    f"Missing block_type reference in assembly row {index}."
                )
            pos = (
                _parse_float(
                    row,
                    "posX_m",
                    context="assembly",
                    row_number=index,
                    aliases=("posX",),
                ),
                _parse_float(
                    row,
                    "posY_m",
                    context="assembly",
                    row_number=index,
                    aliases=("posY",),
                ),
                _parse_float(
                    row,
                    "posZ_m",
                    context="assembly",
                    row_number=index,
                    aliases=("posZ",),
                ),
            )
            rot = (
                _parse_float(
                    row,
                    "rotX_deg",
                    context="assembly",
                    row_number=index,
                    aliases=("rotX",),
                ),
                _parse_float(
                    row,
                    "rotY_deg",
                    context="assembly",
                    row_number=index,
                    aliases=("rotY",),
                ),
                _parse_float(
                    row,
                    "rotZ_deg",
                    context="assembly",
                    row_number=index,
                    aliases=("rotZ",),
                ),
            )
            delay_after = _parse_float(
                row,
                "delay_after_s",
                context="assembly",
                row_number=index,
                default=0.0,
                aliases=("delay_s", "pause_s"),
            )
            assembly.append(
                AssemblyInstance(
                    order=order,
                    block_type=block_type,
                    pos_m=pos,
                    rot_deg=rot,
                    label=_get(row, "label", "name") or None,
                    delay_after_s=delay_after,
                )
            )
            continue

        raise CsvFormatError(f"Unknown section '{section}' in row {index}.")

    assembly.sort(key=lambda inst: inst.order)
    return meta, materials, block_types, assembly


def write_unified_csv(
    path: str,
    meta: MetaMapping,
    materials: Dict[str, Material],
    block_types: Dict[str, BlockType],
    assembly: List[AssemblyInstance],
) -> None:
    """Write the unified CSV format using the superset field layout."""

    fields = [
        "section",
        "key",
        "value",
        "material",
        "density_kgm3",
        "mu_slide",
        "mu_spin",
        "mu_roll",
        "restitution",
        "mu_slide_along",
        "mu_slide_across",
        "block_type",
        "size_x_m",
         "size_y_m",
         "size_z_m",
         "material_of_type",
        "shape",
        "wedge_run_x_m",
        "wedge_width_y_m",
        "wedge_rise_z_m",
         "color_rgba",
        "order",
        "label",
        "posX_m",
        "posY_m",
        "posZ_m",
        "rotX_deg",
        "rotY_deg",
        "rotZ_deg",
        "delay_after_s",
    ]

    path_obj = Path(path)
    path_obj.parent.mkdir(parents=True, exist_ok=True)

    with open(path_obj, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()

        for key, value in meta.items():
            writer.writerow({"section": "meta", "key": key, "value": value})

        for material in materials.values():
            row = {
                "section": "materials",
                "material": material.name,
                "density_kgm3": material.density_kgm3,
                "mu_slide": material.mu_slide,
                "mu_spin": material.mu_spin,
                "mu_roll": material.mu_roll,
                "restitution": material.restitution,
            }
            if material.anisotropic_friction is not None:
                row["mu_slide_along"] = material.mu_slide * material.anisotropic_friction[0]
                row["mu_slide_across"] = material.mu_slide * material.anisotropic_friction[1]
            if material.color_rgba is not None:
                row["color_rgba"] = ",".join(str(component) for component in material.color_rgba)
            writer.writerow(row)

        for block_type in block_types.values():
            row = {
                "section": "block_types",
                "block_type": block_type.name,
                "size_x_m": block_type.size_x_m,
                "size_y_m": block_type.size_y_m,
                "size_z_m": block_type.size_z_m,
                "material_of_type": block_type.material_name,
                "shape": (block_type.shape or "box"),
            }
            if (block_type.shape or "box").lower() == "wedge":
                row["wedge_run_x_m"] = block_type.wedge_run_x_m or block_type.size_x_m
                row["wedge_width_y_m"] = block_type.wedge_width_y_m or block_type.size_y_m
                row["wedge_rise_z_m"] = block_type.wedge_rise_z_m or block_type.size_z_m
            if block_type.color_rgba is not None:
                row["color_rgba"] = ",".join(str(component) for component in block_type.color_rgba)
            writer.writerow(row)

        for instance in assembly:
            row = {
                "section": "assembly",
                "order": instance.order,
                "block_type": instance.block_type,
                "label": instance.label or "",
                "posX_m": instance.pos_m[0],
                "posY_m": instance.pos_m[1],
                "posZ_m": instance.pos_m[2],
                "rotX_deg": instance.rot_deg[0],
                "rotY_deg": instance.rot_deg[1],
                "rotZ_deg": instance.rot_deg[2],
                "delay_after_s": instance.delay_after_s,
            }
            writer.writerow(row)


__all__ = [
    "CsvFormatError",
    "load_unified_csv",
    "write_unified_csv",
]

