"""Quick smoke test for the load → validate → run pipeline."""
from __future__ import annotations

import csv
import os
import tempfile
import threading

from io_unified_csv import load_unified_csv
from models import GlobalConfig
from runner import run_sequence
from validation import apply_meta_to_config, validate_and_resolve


_FIELDS = [
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


def _write_tmp_csv(path: str) -> None:
    rows = [
        {"section": "materials", "material": "MDF", "density_kgm3": "750"},
        {
            "section": "block_types",
            "block_type": "1x1",
            "size_x_m": "0.038",
            "size_y_m": "0.038",
            "size_z_m": "0.019",
            "material_of_type": "MDF",
        },
        {
            "section": "assembly",
            "order": "1",
            "block_type": "1x1",
            "posX_m": "0",
            "posY_m": "0",
            "posZ_m": "0.01",
        },
    ]
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def test_pipeline() -> None:
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_path = os.path.join(tmpdir, "scene.csv")
        _write_tmp_csv(csv_path)
        meta, materials, block_types, assembly = load_unified_csv(csv_path)
        cfg = apply_meta_to_config(
            meta, GlobalConfig(gui=False, pre_spawn_delay_s=0.0)
        )
        resolved = validate_and_resolve(materials, block_types, assembly)

        stop_evt = threading.Event()
        timer = threading.Timer(0.25, stop_evt.set)
        timer.start()
        try:
            run_sequence(cfg, resolved, stop_evt)
        finally:
            timer.cancel()
            stop_evt.set()


if __name__ == "__main__":
    test_pipeline()
    print("OK")
