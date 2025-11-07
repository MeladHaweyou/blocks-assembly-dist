"""Unit tests for the unified CSV I/O helpers."""
from __future__ import annotations

import csv
from pathlib import Path

import pytest

from io_unified_csv import CsvFormatError, load_unified_csv, write_unified_csv
from models import AssemblyInstance, BlockType, Material


FIELDS = [
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
    "color_rgba",
    "order",
    "label",
    "posX_m",
    "posY_m",
    "posZ_m",
    "rotX_deg",
    "rotY_deg",
    "rotZ_deg",
]


def test_write_and_load_roundtrip(tmp_path: Path) -> None:
    meta = {
        "length_units": "m",
        "placement_mode": "gentle",
        "camera_distance": "1.0",
    }

    materials = {
        "Foam": Material(name="Foam", density_kgm3=250.0, mu_slide=0.4),
    }

    block_types = {
        "FoamCube": BlockType(
            name="FoamCube",
            size_x_m=0.1,
            size_y_m=0.1,
            size_z_m=0.1,
            material_name="Foam",
        )
    }

    assembly = [
        AssemblyInstance(
            order=2,
            block_type="FoamCube",
            pos_m=(0.0, 0.1, 0.2),
            rot_deg=(10.0, 0.0, 0.0),
            label="top",
        ),
        AssemblyInstance(
            order=1,
            block_type="FoamCube",
            pos_m=(0.0, 0.0, 0.0),
            rot_deg=(0.0, 0.0, 0.0),
        ),
    ]

    csv_path = tmp_path / "assembly.csv"
    write_unified_csv(str(csv_path), meta, materials, block_types, assembly)

    loaded_meta, loaded_materials, loaded_block_types, loaded_assembly = load_unified_csv(str(csv_path))

    assert loaded_meta == meta
    assert loaded_materials == materials
    assert loaded_block_types == block_types
    assert [inst.order for inst in loaded_assembly] == [1, 2]
    assert loaded_assembly[1].label == "top"


def test_color_parsing_accepts_semicolon(tmp_path: Path) -> None:
    csv_path = tmp_path / "with_color.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerow(
            {
                "section": "materials",
                "material": "Foam",
                "density_kgm3": "100.0",
                "mu_slide": "0.5",
                "mu_spin": "0",
                "mu_roll": "0",
                "restitution": "0",
                "color_rgba": "0.1;0.2;0.3;0.4",
            }
        )
        writer.writerow(
            {
                "section": "block_types",
                "block_type": "Cube",
                "size_x_m": "0.1",
                "size_y_m": "0.1",
                "size_z_m": "0.1",
                "material_of_type": "Foam",
            }
        )
        writer.writerow({"section": "assembly", "block_type": "Cube", "order": "1"})

    _, materials, _, _ = load_unified_csv(str(csv_path))
    assert materials["Foam"].color_rgba == (0.1, 0.2, 0.3, 0.4)


def test_invalid_color_raises(tmp_path: Path) -> None:
    csv_path = tmp_path / "bad.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerow(
            {
                "section": "materials",
                "material": "Foam",
                "density_kgm3": "100.0",
                "mu_slide": "0.5",
                "mu_spin": "0",
                "mu_roll": "0",
                "restitution": "0",
                "color_rgba": "invalid",
            }
        )

    with pytest.raises(CsvFormatError):
        load_unified_csv(str(csv_path))

