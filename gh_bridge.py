"""Utilities for bridging Rhino/Grasshopper data with the Streamlit app schema."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, MutableMapping, Sequence

import pandas as pd

DEFAULT_DENSITY = 750.0  # kg/m^3
DEFAULT_DROP_HEIGHT = 0.01  # meters


@dataclass(frozen=True)
class BlockType:
    """Catalog entry for a block type."""

    type_id: int
    label: str
    size_x: float
    size_y: float
    size_z: float
    density: float = DEFAULT_DENSITY
    mu_slide: float = 0.6
    mu_spin: float = 0.0
    mu_roll: float = 0.0

    @property
    def footprint(self) -> tuple[float, float]:
        return (self.size_x, self.size_y)


BLOCK_CATALOG: Dict[int, BlockType] = {
    1: BlockType(1, "1×1×1 cell", 0.038, 0.038, 0.019),
    2: BlockType(2, "1×2 beam", 0.076, 0.038, 0.019),
    3: BlockType(3, "1×4 beam", 0.152, 0.038, 0.019),
    4: BlockType(4, "2×2 slab", 0.076, 0.076, 0.019),
}

LENGTH_UNIT_SCALES: Dict[str, float] = {
    "m": 1.0,
    "meter": 1.0,
    "meters": 1.0,
    "metre": 1.0,
    "metres": 1.0,
    "cm": 0.01,
    "centimeter": 0.01,
    "centimeters": 0.01,
    "centimetre": 0.01,
    "centimetres": 0.01,
    "mm": 0.001,
    "millimeter": 0.001,
    "millimeters": 0.001,
    "millimetre": 0.001,
    "millimetres": 0.001,
}

REQUIRED_COLUMNS = [
    "name",
    "order",
    "type_id",
    "size_x",
    "size_y",
    "size_z",
    "mass",
    "mu_slide",
    "mu_spin",
    "mu_roll",
    "drop_height",
    "posX",
    "posY",
    "posZ",
    "rotX",
    "rotY",
    "rotZ",
]


def compute_mass(density: float, size_x: float, size_y: float, size_z: float) -> float:
    """Return block mass from density and dimensions."""

    return float(density) * float(size_x) * float(size_y) * float(size_z)


def _unit_scale(label: str | None, default: float = 1.0) -> float:
    if not label:
        return default
    key = label.strip().lower()
    return LENGTH_UNIT_SCALES.get(key, default)


def _normalize_sequence(values: Sequence[Any], scale: float) -> list[float]:
    return [float(v) * scale for v in values]


def _ensure_iterable_dict(data: Any) -> MutableMapping[str, Any]:
    if isinstance(data, MutableMapping):
        return data  # type: ignore[return-value]
    raise TypeError("Grasshopper JSON must contain mapping objects for blocks")


def _classify_block_type(size_x: float, size_y: float, size_z: float) -> BlockType:
    sx = float(size_x)
    sy = float(size_y)
    sz = float(size_z)
    best: BlockType | None = None
    best_err = float("inf")
    for block in BLOCK_CATALOG.values():
        for orient in ((block.size_x, block.size_y), (block.size_y, block.size_x)):
            dx = orient[0] - sx
            dy = orient[1] - sy
            dz = block.size_z - sz
            err = dx * dx + dy * dy + dz * dz
            if err < best_err:
                best_err = err
                best = block
    if best is None:
        raise ValueError("Block catalog is empty; cannot classify block size")
    return best


def apply_physical_defaults(
    df: pd.DataFrame,
    *,
    density: float = DEFAULT_DENSITY,
    drop_height: float = DEFAULT_DROP_HEIGHT,
) -> pd.DataFrame:
    """Ensure catalog-backed physical properties are filled in."""

    result = df.copy()
    if "mu_slide" not in result:
        result["mu_slide"] = 0.6
    else:
        result["mu_slide"] = result["mu_slide"].fillna(0.6)
    for key in ("mu_spin", "mu_roll"):
        if key not in result:
            result[key] = 0.0
        else:
            result[key] = result[key].fillna(0.0)
    if "drop_height" not in result:
        result["drop_height"] = drop_height
    else:
        result["drop_height"] = result["drop_height"].fillna(drop_height)

    if "density" in result:
        density_series = result["density"].astype(float)
    else:
        density_series = float(density)

    result["mass"] = (
        result["size_x"].astype(float)
        * result["size_y"].astype(float)
        * result["size_z"].astype(float)
        * density_series
    )

    type_ids = []
    for _, row in result.iterrows():
        block = _classify_block_type(row["size_x"], row["size_y"], row["size_z"])
        type_ids.append(block.type_id)
    result["type_id"] = type_ids

    return result


def _parse_json_source(source: Any) -> Mapping[str, Any]:
    if isinstance(source, (str, Path)):
        path = Path(source)
        if isinstance(source, str) and not path.exists():
            return json.loads(source)
        text = path.read_text(encoding="utf-8")
        return json.loads(text)
    if isinstance(source, bytes):
        return json.loads(source.decode("utf-8"))
    if hasattr(source, "read"):
        data = source.read()
        if isinstance(data, bytes):
            return json.loads(data.decode("utf-8"))
        return json.loads(str(data))
    if hasattr(source, "getvalue"):
        data = source.getvalue()
        if isinstance(data, bytes):
            return json.loads(data.decode("utf-8"))
        return json.loads(str(data))
    if isinstance(source, Mapping):
        return source
    raise TypeError("Unsupported source type for GH JSON")


def load_blocks_from_gh_json(source: Any) -> pd.DataFrame:
    """
    Parse a Rhino/Grasshopper JSON export and return a DataFrame in the app schema.

    Expected JSON shape:
    {
        "units": {"length": "mm"},
        "defaults": {...},
        "blocks": [
            {
                "name": "B1",
                "order": 1,
                "type_id": 2,
                "size": [76, 38, 19],
                "position": [0, 0, 9.5],
                "rotation": [0, 0, 0]
            },
            ...
        ]
    }
    """

    payload = _parse_json_source(source)
    blocks = payload.get("blocks")
    if not isinstance(blocks, Sequence):
        raise ValueError("Grasshopper JSON must contain a 'blocks' array")

    units = payload.get("units", {})
    if not isinstance(units, Mapping):
        units = {}
    defaults = payload.get("defaults", {})
    if not isinstance(defaults, Mapping):
        defaults = {}

    length_scale = _unit_scale(str(units.get("length", "m")), 1.0)
    size_scale = _unit_scale(str(units.get("size", units.get("length", "m"))), length_scale)
    position_scale = _unit_scale(str(units.get("position", units.get("length", "m"))), length_scale)
    drop_scale = _unit_scale(str(units.get("drop_height", units.get("length", "m"))), length_scale)

    rows: list[Dict[str, Any]] = []
    for idx, entry in enumerate(blocks, start=1):
        block = _ensure_iterable_dict(entry)
        name = str(block.get("name") or f"Block {idx}")
        order_val = block.get("order") or block.get("sequence") or block.get("index")
        order = int(order_val) if order_val is not None else idx

        raw_size = block.get("size") or block.get("dimensions")
        if raw_size is None:
            raise ValueError(f"Block {idx} missing 'size' field")
        size_vals = _normalize_sequence(raw_size, size_scale)
        if len(size_vals) != 3:
            raise ValueError("Block size must have three components")

        raw_pos = (
            block.get("position")
            or block.get("center")
            or block.get("centroid")
            or block.get("pos")
        )
        if raw_pos is not None:
            pos_vals = _normalize_sequence(raw_pos, position_scale)
            if len(pos_vals) != 3:
                raise ValueError("Block position must have three components")
        else:
            layer = block.get("layer")
            cell = block.get("cell")
            if layer is None or cell is None:
                raise ValueError(
                    f"Block {idx} must define 'position' or both 'layer' and 'cell'"
                )
            cx, cy = (float(cell[0]), float(cell[1]))
            pitch = 0.038
            pos_vals = [
                (cx - 3.0) * pitch,
                (cy - 3.0) * pitch,
                (float(layer) + 0.5) * size_vals[2],
            ]

        rot_vals = block.get("rotation") or block.get("angles") or block.get("rot") or [0.0, 0.0, 0.0]
        rot = [float(v) for v in rot_vals]
        while len(rot) < 3:
            rot.append(0.0)
        rot = rot[:3]

        type_id = block.get("type_id") or defaults.get("type_id")
        if type_id is not None:
            try:
                type_id = int(type_id)
            except (TypeError, ValueError):
                type_id = None
        density = float(block.get("density", defaults.get("density", DEFAULT_DENSITY)))

        catalog_entry = BLOCK_CATALOG.get(type_id) if isinstance(type_id, int) else None
        if catalog_entry is None:
            catalog_entry = _classify_block_type(*size_vals)
            type_id = catalog_entry.type_id
        mu_slide = float(block.get("mu_slide", catalog_entry.mu_slide))
        mu_spin = float(block.get("mu_spin", catalog_entry.mu_spin))
        mu_roll = float(block.get("mu_roll", catalog_entry.mu_roll))
        raw_drop = block.get("drop_height")
        if raw_drop is None:
            raw_drop = defaults.get("drop_height")
            if raw_drop is None:
                drop = DEFAULT_DROP_HEIGHT
            else:
                drop = float(raw_drop) * drop_scale
        else:
            drop = float(raw_drop) * drop_scale

        rows.append(
            {
                "name": name,
                "order": order,
                "type_id": type_id,
                "size_x": size_vals[0],
                "size_y": size_vals[1],
                "size_z": size_vals[2],
                "mu_slide": mu_slide,
                "mu_spin": mu_spin,
                "mu_roll": mu_roll,
                "drop_height": drop,
                "posX": pos_vals[0],
                "posY": pos_vals[1],
                "posZ": pos_vals[2],
                "rotX": rot[0],
                "rotY": rot[1],
                "rotZ": rot[2],
                "density": density,
            }
        )

    df = pd.DataFrame(rows)
    df = apply_physical_defaults(df)
    if "density" in df:
        density_series = df["density"].astype(float)
        df["mass"] = (
            df["size_x"].astype(float)
            * df["size_y"].astype(float)
            * df["size_z"].astype(float)
            * density_series
        )
        df = df.drop(columns=["density"])

    ordered_cols = [col for col in REQUIRED_COLUMNS if col in df.columns]
    extra_cols = [col for col in df.columns if col not in ordered_cols]
    df = df[ordered_cols + extra_cols]
    return df.sort_values("order").reset_index(drop=True)
