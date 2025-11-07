import pytest

pytest.importorskip("numpy")
pytest.importorskip("pybullet")

from backend_bullet import BulletWorld, BlockSpec
from analysis import (
    center_of_pressure_xy,
    cop_margin_mm,
    support_polygon_from_ground_contacts,
    project_point_to_ground_xy,
    assembly_com,
)


def _tower(offset):
    w = BulletWorld(gui=False, time_step=1 / 240.0, solver_iters=180)
    try:
        w.create_ground()
        base = BlockSpec(
            name="base",
            order=1,
            size_x=0.2,
            size_y=0.2,
            size_z=0.1,
            mass=3.0,
            mu_slide=0.8,
            mu_spin=0.0,
            mu_roll=0.0,
            restitution=0.01,
            drop_height=0.003,
            posX=0.0,
            posY=0.0,
            posZ=0.05,
            rotX=0.0,
            rotY=0.0,
            rotZ=0.0,
        )
        top = BlockSpec(
            name="top",
            order=2,
            size_x=0.2,
            size_y=0.2,
            size_z=0.1,
            mass=1.0,
            mu_slide=0.8,
            mu_spin=0.0,
            mu_roll=0.0,
            restitution=0.01,
            drop_height=0.003,
            posX=offset,
            posY=0.0,
            posZ=0.15,
            rotX=0.0,
            rotY=0.0,
            rotZ=0.0,
        )
        ids = [w.spawn_box(base), w.spawn_box(top)]
        w.step_until_settled(ids, v_th=2e-3, w_th=2e-3, dwell_s=0.4, max_s=4.0)
        bodies = ids + [w.ground_id]
        cps = w.get_contacts(bodies=bodies)
        hull = support_polygon_from_ground_contacts(w, cps)
        cop, _ = center_of_pressure_xy(w, cps)
        com = project_point_to_ground_xy(w, assembly_com(w, ids))
        return cop_margin_mm(cop, hull), com
    finally:
        w.disconnect()


def test_cop_margin_shrinks_when_offset_grows():
    m1, _ = _tower(0.0)
    m2, _ = _tower(0.06)
    assert m2 <= m1
