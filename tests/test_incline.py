import math

import pytest

pytest.importorskip("numpy")
pytest.importorskip("pybullet")

from backend_bullet import BulletWorld, BlockSpec


def _run_incline(mu_slide: float, theta_deg: float) -> float:
    world = BulletWorld(gui=False, time_step=1 / 240.0, solver_iters=120)
    try:
        world.create_ground()
        world.set_incline(theta_deg, axis="x")
        spec = BlockSpec(
            name="incline",
            order=1,
            size_x=0.2,
            size_y=0.2,
            size_z=0.2,
            mass=1.0,
            mu_slide=mu_slide,
            mu_spin=mu_slide,
            mu_roll=mu_slide,
            restitution=0.01,
            drop_height=0.005,
            posX=0.0,
            posY=0.0,
            posZ=0.1,
            rotX=0.0,
            rotY=0.0,
            rotZ=0.0,
        )
        bid = world.spawn_box(spec)
        world.step_until_settled([bid], v_th=2e-3, w_th=2e-3, dwell_s=0.5, max_s=4.0)
        state = world.get_body_state(bid)
        return float(state["pos"][1])
    finally:
        world.disconnect()


def test_incline_slide_vs_stick():
    theta = 25.0
    assert math.tan(math.radians(theta)) > 0.4
    slid_pos = _run_incline(mu_slide=0.2, theta_deg=theta)
    stuck_pos = _run_incline(mu_slide=0.6, theta_deg=theta)
    assert abs(slid_pos) > 0.3, f"expected noticeable sliding displacement, got {slid_pos}"
    assert abs(stuck_pos) < 1.0, f"expected limited motion for high friction, got {stuck_pos}"
    assert abs(stuck_pos) < abs(slid_pos), "high-friction block should move less than low-friction block"
