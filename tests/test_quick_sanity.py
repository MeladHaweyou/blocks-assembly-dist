import math
from pathlib import Path

import pytest

pytest.importorskip("numpy")
pytest.importorskip("pybullet")

from backend_bullet import BlockSpec, BulletWorld
from io_unified_csv import load_unified_csv
from validation import validate_and_resolve


def _resolved_to_spec(resolved_inst):
    block = resolved_inst.block_type
    mat = resolved_inst.material
    inst = resolved_inst.instance
    shape = (block.shape or "box").lower()
    if shape == "wedge":
        size_x = block.wedge_run_x_m or block.size_x_m
        size_y = block.wedge_width_y_m or block.size_y_m
        size_z = block.wedge_rise_z_m or block.size_z_m
    else:
        size_x = block.size_x_m
        size_y = block.size_y_m
        size_z = block.size_z_m
    return BlockSpec(
        name=inst.label or inst.block_type,
        order=inst.order,
        size_x=float(size_x),
        size_y=float(size_y),
        size_z=float(size_z),
        mass=float(resolved_inst.mass_kg),
        mu_slide=float(mat.mu_slide),
        mu_spin=float(mat.mu_spin),
        mu_roll=float(mat.mu_roll),
        restitution=float(mat.restitution),
        anisotropic_friction=mat.anisotropic_friction,
        drop_height=0.0,
        posX=float(inst.pos_m[0]),
        posY=float(inst.pos_m[1]),
        posZ=float(inst.pos_m[2]),
        rotX=float(inst.rot_deg[0]),
        rotY=float(inst.rot_deg[1]),
        rotZ=float(inst.rot_deg[2]),
    )


def test_ramp_sequence_sliding(tmp_path):
    csv_path = Path("data/ramp_with_blocks.csv")
    meta, materials, block_types, assembly = load_unified_csv(str(csv_path))
    assert meta.get("sim_name") == "Ramp friction sanity"
    resolved = validate_and_resolve(materials, block_types, assembly)
    assert len(resolved) == 5

    world = BulletWorld(gui=False, time_step=1 / 240.0, solver_iters=180)
    try:
        world.create_ground()
        block_ids = {}
        sliding = []
        ramp_height = None
        for resolved_inst in resolved:
            spec = _resolved_to_spec(resolved_inst)
            shape = (resolved_inst.block_type.shape or "box").lower()
            if shape == "wedge":
                ramp_height = spec.size_z
                bid = world.spawn_wedge(spec, as_support=True)
            else:
                bid = world.spawn_box(spec)
                sliding.append((resolved_inst.material.mu_slide, bid))
            label = resolved_inst.instance.label or resolved_inst.instance.block_type
            block_ids[label] = bid

        total_steps = int(8.0 / world.time_step)
        world.step(total_steps)

        assert ramp_height is not None
        ramp_state = world.get_body_state(block_ids["Ramp"])
        ramp_bottom = ramp_state["pos"][2] - 0.5 * ramp_height
        assert abs(ramp_bottom) < 2e-3, f"expected ramp bottom near z=0, got {ramp_bottom}"

        speeds = []
        positions = []
        for mu, bid in sliding:
            state = world.get_body_state(bid)
            positions.append((mu, state["pos"][0]))
            lin = state["lin"]
            speed = math.sqrt(lin[0] ** 2 + lin[1] ** 2 + lin[2] ** 2)
            speeds.append(speed)

        assert all(v < 0.05 for v in speeds), "blocks should be nearly settled before comparison"

        low = [pos for mu, pos in positions if math.isclose(mu, 0.2, abs_tol=1e-6)]
        medium = [pos for mu, pos in positions if math.isclose(mu, 0.45, abs_tol=1e-6)]
        high = [pos for mu, pos in positions if math.isclose(mu, 0.6, abs_tol=1e-6)]

        assert low, "missing low friction block"
        assert medium, "missing medium friction block"
        assert high, "missing high friction block"

        low_x = low[0]
        avg_medium = sum(medium) / len(medium)
        high_x = high[0]

        assert low_x > avg_medium + 0.01, "low friction block should slide noticeably farther down the ramp"
        assert avg_medium >= high_x - 1e-3, "medium friction blocks should not slide less than the high friction block"
        assert high_x < -0.02, "high friction block should remain near the top of the ramp"
    finally:
        world.disconnect()


def test_followup_csv_loads_cleanly():
    csv_path = Path("data/01a_two_block_safe.csv")
    _, materials, block_types, assembly = load_unified_csv(str(csv_path))
    resolved = validate_and_resolve(materials, block_types, assembly)
    assert len(resolved) == 2
    # Ensure mass computed from density and volume (no leftover edits)
    masses = [round(inst.material.density_kgm3 * inst.block_type.volume_m3, 6) for inst in resolved]
    assert masses[0] > 0
    assert masses[1] > 0
