import pytest

pytest.importorskip("numpy")
pytest.importorskip("pybullet")

from backend_bullet import BulletWorld, BlockSpec


def test_step_until_settled_completes():
    world = BulletWorld(gui=False, time_step=1 / 240.0, solver_iters=120)
    try:
        world.create_ground()
        spec = BlockSpec(
            name="drop",
            order=1,
            size_x=0.3,
            size_y=0.2,
            size_z=0.1,
            mass=2.0,
            mu_slide=0.8,
            mu_spin=0.0,
            mu_roll=0.0,
            restitution=0.01,
            drop_height=0.1,
            posX=0.0,
            posY=0.0,
            posZ=0.05,
            rotX=0.0,
            rotY=0.0,
            rotZ=0.0,
        )
        bid = world.spawn_box(spec)
        ok, t_settle = world.step_until_settled([bid], v_th=1e-3, w_th=1e-3, dwell_s=0.4, max_s=6.0)
        assert ok, "block did not settle within expected time"
        assert t_settle <= 6.0
    finally:
        world.disconnect()
