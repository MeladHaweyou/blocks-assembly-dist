import pytest

pytest.importorskip("numpy")
pytest.importorskip("pybullet")

from backend_bullet import BulletWorld, BlockSpec
from analysis import (
    assembly_com,
    project_point_to_ground_xy,
    support_polygon_from_ground_contacts,
    max_tilt_deg,
    slip_flag_from_contacts,
    stability_verdict,
)


def _run_cantilever(offset: float):
    size = 0.2
    world = BulletWorld(gui=False, time_step=1 / 240.0, solver_iters=120)
    try:
        world.create_ground()
        specs = [
            BlockSpec(
                name="base",
                order=1,
                size_x=size,
                size_y=size,
                size_z=size,
                mass=5.0,
                mu_slide=0.8,
                mu_spin=0.0,
                mu_roll=0.0,
                restitution=0.01,
                drop_height=0.005,
                posX=0.0,
                posY=0.0,
                posZ=size / 2,
                rotX=0.0,
                rotY=0.0,
                rotZ=0.0,
            ),
            BlockSpec(
                name="top",
                order=2,
                size_x=size,
                size_y=size,
                size_z=size,
                mass=5.0,
                mu_slide=0.8,
                mu_spin=0.0,
                mu_roll=0.0,
                restitution=0.01,
                drop_height=0.005,
                posX=offset,
                posY=0.0,
                posZ=1.5 * size,
                rotX=0.0,
                rotY=0.0,
                rotZ=0.0,
            ),
        ]
        added = []
        for spec in specs:
            bid = world.spawn_box(spec)
            added.append(bid)
            world.step_until_settled(added, v_th=2e-3, w_th=2e-3, dwell_s=0.5, max_s=4.0)
        bodies = added + ([world.ground_id] if world.ground_id is not None else [])
        contacts = world.get_contacts(bodies=bodies)
        hull = support_polygon_from_ground_contacts(world, contacts)
        com_w = assembly_com(world, added)
        com_xy = project_point_to_ground_xy(world, com_w)
        tilt = max_tilt_deg(world, added)
        slip = slip_flag_from_contacts(world, contacts, tangential_vel_th=2e-3)
        verdict = stability_verdict(com_xy, hull, tilt, slip)
        top_state = world.get_body_state(added[-1]) if added else {"pos": (0.0, 0.0, 0.0)}
        verdict["top_center_z"] = float(top_state["pos"][2])
        return verdict
    finally:
        world.disconnect()


def test_cantilever_verdict_changes_with_overhang():
    stable = _run_cantilever(offset=0.05)
    tipping = _run_cantilever(offset=0.18)
    assert stable["stable"]
    assert stable["top_center_z"] > 0.28
    assert tipping["top_center_z"] < 0.25
