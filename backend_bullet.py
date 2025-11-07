# backend_bullet.py
# Minimal PyBullet backend for CSV-driven block assembly.
# World: Z-up, meters/kilograms/seconds.

from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple, List, Optional, Dict, Any, Iterable
from collections import namedtuple
import math
import numpy as np
import time

try:
    import pybullet as p
    import pybullet_data
except ImportError as e:
    raise SystemExit("PyBullet is required. Install with: pip install pybullet") from e

# Lightweight contact container for analysis layer
Contact = namedtuple("Contact", ["bodyA","bodyB","position_on_a","position_on_b","normal","normal_force"])

@dataclass
class BlockSpec:
    name: str
    order: int
    size_x: float
    size_y: float
    size_z: float
    mass: float
    mu_slide: float
    mu_spin: float
    mu_roll: float
    restitution: float
    drop_height: float
    posX: float
    posY: float
    posZ: float
    rotX: float  # degrees
    rotY: float  # degrees
    rotZ: float  # degrees
    anisotropic_friction: Optional[Tuple[float, float, float]] = None

class BulletWorld:
    """Thin wrapper around PyBullet to keep physicsClientId scoped."""
    def __init__(self, gui: bool=False, time_step: float=1/240.0, gravity: Tuple[float,float,float]=(0,0,-9.81), solver_iters: int=150):
        self.gui = gui
        self.cid = p.connect(p.GUI) if gui else p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.cid)
        p.resetSimulation(physicsClientId=self.cid)
        if gui:
            # Give the debug visualizer a moment to come up before we start
            # issuing camera commands. Without this short delay the first
            # frames occasionally ignore camera presets on slower machines.
            time.sleep(0.3)
        p.setGravity(gravity[0], gravity[1], gravity[2], physicsClientId=self.cid)
        p.setTimeStep(time_step, physicsClientId=self.cid)
        p.setPhysicsEngineParameter(numSolverIterations=int(solver_iters), physicsClientId=self.cid)
        # Determinism & robust stacking
        try:
            p.setPhysicsEngineParameter(
                enableConeFriction=1,                 # true Coulomb cone (better stacking)
                deterministicOverlappingPairs=1,      # consistent contact pairs across runs
                frictionAnchor=1,
                enableSplitImpulse=1,                 # separate penetration recovery
                splitImpulsePenetrationThreshold=-0.02,
                contactBreakingThreshold=1e-3,        # stable manifolds
                contactSlop=1e-4,                     # tighter contact tolerance
                solverResidualThreshold=1e-7,
                physicsClientId=self.cid,
            )
            # Mild compliance for stabilization (tune conservatively)
            p.setPhysicsEngineParameter(
                erp=0.2, contactERP=0.2, frictionERP=0.2,
                physicsClientId=self.cid
            )
        except TypeError:
            # Older pybullet builds: ignore unsupported kwargs
            pass

        # Fixed substeps for stiff stacks
        p.setPhysicsEngineParameter(numSubSteps=2, physicsClientId=self.cid)
        self.time_step = float(time_step)
        self.ground_id: Optional[int] = None
        self._bodies: List[int] = []
        self._ground_opts: Optional[Dict[str, Any]] = None
        self._ground_incline: float = 0.0
        self._camera_initialized: bool = False
        # Bodies considered "supports" for analysis (ground + static wedges, etc.)
        self.support_ids: set[int] = set()
        # GUI slow-motion: sleep this many seconds after each physics step if GUI is on.
        self.step_sleep: float = 0.0
        # Spawn semantics (can be overridden by the UI)
        self.spawn_mode: str = "support_clearance"
        self.align_to_support = False  # when True, pitch/roll so +Z aligns with support normal
        # Placement mode flags (for quasi-static vs. dynamic drop)
        self.placement_mode = "gentle"      # "gentle" | "dynamic"
        self.servo_clearance = 3e-4         # 0.3 mm, target gap for quasi-static servo
        self.hold_frames = 8                # frames to clamp velocities after touch-down
        self.preview_frames = 60            # hold steps while previewing the seat
        self.micro_drop_frames = 12         # simulation frames for the micro-drop window
        self.micro_drop_height = 2.5e-4     # 0.25 mm release distance
        self.base_dwell_s = 0.20            # dwell time between placements
        self.layer_tilt_guard_deg = 0.9     # guardrail threshold in degrees
        self.drop_distance = 1e-3           # 1.0 mm global drop for dynamic placement
        self.use_csv_drop_for_dynamic = False
        # Cache half-extents per body (used by CCD and placement helpers)
        self._half_by_id: Dict[int, Tuple[float,float,float]] = {}
        self._layer_index_by_body: Dict[int, int] = {}
        self.placement_metrics: List[Dict[str, Any]] = []
        self.layer_tilt_history: Dict[int, List[Tuple[float, float]]] = {}
        self._sim_time: float = 0.0

    # ---- Ground ----------------------------------------------------------------
    def create_ground(self, incline_deg: float=0.0, axis: str="x", half_extents: Tuple[float,float,float]=(5.0,5.0,0.1), friction: float=1.0) -> int:
        """Create a static ground box (optionally inclined). half_extents are HALF sizes."""
        if axis.lower() not in ("x","y"):
            raise ValueError("axis must be 'x' or 'y'")
        if self.ground_id is not None:
            p.removeBody(self.ground_id, physicsClientId=self.cid)
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents, physicsClientId=self.cid)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[0.75,0.75,0.75,1.0], physicsClientId=self.cid)
        theta = math.radians(float(incline_deg))
        if axis.lower() == "x":
            quat = p.getQuaternionFromEuler([theta, 0, 0])
        else:
            quat = p.getQuaternionFromEuler([0, theta, 0])
        hx, hy, hz = map(float, half_extents)
        R = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
        o = R @ np.array([0.0, 0.0, hz])
        base_pos = (-float(o[0]), -float(o[1]), -float(o[2]))

        gid = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=base_pos,
            baseOrientation=quat,
            physicsClientId=self.cid,
        )
        p.changeDynamics(
            gid,
            -1,
            lateralFriction=float(friction),
            spinningFriction=0.0,
            rollingFriction=0.0,
            restitution=0.01,
            physicsClientId=self.cid,
        )
        self.ground_id = gid
        # (Re)register ground as a support body for analysis
        try:
            self.support_ids.add(gid)
        except AttributeError:
            self.support_ids = {gid}
        self._ground_opts = {
            "half_extents": tuple(float(v) for v in half_extents),
            "friction": float(friction),
            "axis": axis.lower(),
        }
        self._ground_incline = float(incline_deg)
        if self.gui and not self._camera_initialized:
            # Initialize the camera once the ground exists so the distance is 1.0
            # (requirement from the UI layer) while respecting the selected preset later.
            self.set_camera_preset(distance=1.0)
        self._camera_initialized = self._camera_initialized or self.gui
        return gid

    def set_incline(self, incline_deg: float, axis: str = "x") -> int:
        """Update the ground inclination while preserving prior geometry/friction settings."""
        axis_l = axis.lower()
        if axis_l not in ("x", "y"):
            raise ValueError("axis must be 'x' or 'y'")
        opts = self._ground_opts or {
            "half_extents": (5.0, 5.0, 0.1),
            "friction": 2.0,
            "axis": axis_l,
        }
        opts = dict(opts)
        opts["axis"] = axis_l
        self._ground_incline = float(incline_deg)
        return self.create_ground(incline_deg=float(incline_deg), axis=opts["axis"],
                                  half_extents=opts["half_extents"], friction=opts["friction"])

    # ---- Bodies ----------------------------------------------------------------
    def support_point_and_normal(self, x: float, y: float, z_probe: float = 0.5):
        from_pt = (x, y, z_probe)
        to_pt = (x, y, z_probe - 2.0)
        hit = p.rayTest(from_pt, to_pt, physicsClientId=self.cid)[0]
        if hit[0] >= 0:
            return hit[3], hit[4]
        return (x, y, 0.0), (0.0, 0.0, 1.0)

    def spawn_box(self, spec: BlockSpec, rgba: Optional[Tuple[float,float,float,float]]=None) -> int:
        """Spawn a dynamic box. CSV uses full sizes; Bullet expects halfExtents."""
        half = [spec.size_x/2.0, spec.size_y/2.0, spec.size_z/2.0]
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.cid)
        if rgba is None:
            rgba = (0.6, 0.8, 0.9, 1.0)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba, physicsClientId=self.cid)

        spawn_mode = getattr(self, "spawn_mode", "support_clearance")
        align_quat = None
        if spawn_mode == "support_clearance":
            x, y = float(spec.posX), float(spec.posY)
            sx, sy, sz = float(spec.size_x), float(spec.size_y), float(spec.size_z)
            probe = max(0.5, spec.posZ + 0.5 * sz + 0.2)
            pt, n = self.support_point_and_normal(x, y, z_probe=probe)
            nn = np.array(n, float)
            nlen = np.linalg.norm(nn) or 1.0
            nn /= nlen
            clearance = float(spec.drop_height)
            spawn_center = np.array(pt, float) + nn * (0.5 * sz + clearance)
            spawn_pos = (
                float(spawn_center[0]),
                float(spawn_center[1]),
                float(spawn_center[2]),
            )
            # Optional: align +Z to the support normal (pitch/roll only)
            if getattr(self, "align_to_support", False):
                # Map world +Z to nn using axis-angle; keep yaw from user Euler
                z = np.array([0.0, 0.0, 1.0], float)
                axis = np.cross(z, nn)
                s = np.linalg.norm(axis)
                if s > 1e-6:
                    axis = axis / s
                    angle = math.asin(s)
                    # axis-angle -> quaternion
                    s2 = math.sin(0.5 * angle)
                    c2 = math.cos(0.5 * angle)
                    align_quat = (axis[0]*s2, axis[1]*s2, axis[2]*s2, c2)
        elif spawn_mode == "vertical_clearance":
            x, y = float(spec.posX), float(spec.posY)
            sx, sy, sz = float(spec.size_x), float(spec.size_y), float(spec.size_z)
            from_pt = (x, y, max(0.5, spec.posZ + 0.5 * sz + 0.2))
            to_pt = (x, y, from_pt[2] - 2.0)
            hit = p.rayTest(from_pt, to_pt, physicsClientId=self.cid)[0]
            support_z = hit[3][2] if hit[0] >= 0 else 0.0
            clearance = float(spec.drop_height)
            spawn_pos = (x, y, float(support_z + 0.5 * sz + clearance))
        else:
            spawn_pos = (float(spec.posX), float(spec.posY), float(spec.posZ + spec.drop_height))

        rxyz_rad = [math.radians(float(spec.rotX)), math.radians(float(spec.rotY)), math.radians(float(spec.rotZ))]
        quat = p.getQuaternionFromEuler(rxyz_rad)
        if align_quat is not None:
            # Multiply align_quat * quat (use multiplyTransforms to avoid hand-mul)
            _, quat = p.multiplyTransforms((0,0,0), align_quat, (0,0,0), quat)

        bid = p.createMultiBody(baseMass=float(spec.mass),
                                baseCollisionShapeIndex=col,
                                baseVisualShapeIndex=vis,
                                basePosition=spawn_pos,
                                baseOrientation=quat,
                                physicsClientId=self.cid)
        self._half_by_id[bid] = tuple(half)
        # Friction on both participants matters; set block friction here.
        # Friction anchors add 'memory' to contacts; CCD prevents tunneling on release
        r_min = 0.25 * max(min(half), 1e-6)  # CCD radius ~ 1/4 smallest half-extent
        dyn_kwargs = dict(
            lateralFriction=float(spec.mu_slide),
            spinningFriction=float(spec.mu_spin),
            rollingFriction=float(spec.mu_roll),
            frictionAnchor=1,
            restitution=float(spec.restitution),
            linearDamping=0.01,
            angularDamping=0.02,
            ccdSweptSphereRadius=r_min,
            contactProcessingThreshold=0,
            physicsClientId=self.cid,
        )
        if spec.anisotropic_friction is not None:
            dyn_kwargs["anisotropicFriction"] = [
                float(v) for v in spec.anisotropic_friction
            ]
        p.changeDynamics(bid, -1, **dyn_kwargs)

        self._bodies.append(bid)
        layer_idx = self._infer_layer_index(spec)
        if layer_idx is not None:
            self._layer_index_by_body[bid] = layer_idx
        # --- apply placement mode ---
        self._last_servo_converged = True
        if getattr(self, "placement_mode", "gentle") == "gentle":
            he = self._half_by_id.get(bid, (spec.size_x/2.0, spec.size_y/2.0, spec.size_z/2.0))
            clearance = max(float(self.servo_clearance), float(self.micro_drop_height))
            converged = self.servo_place_along_normal(
                bid,
                he,
                xy_hint=(spec.posX, spec.posY),
                target_clearance=clearance,
            )
            self._last_servo_converged = bool(converged)

            preview_steps = max(0, int(getattr(self, "preview_frames", 0)))
            drop_steps = int(getattr(self, "micro_drop_frames", 0))
            if drop_steps <= 0 and self.micro_drop_height > 0.0:
                drop_steps = 1
            settle_steps = max(0, int(getattr(self, "hold_frames", 0)))
            peak_force = 0.0
            max_tilt = 0.0

            for _ in range(preview_steps):
                self._step_simulation(clamp_bodies=[bid])
                max_tilt = max(max_tilt, self._body_tilt_deg(bid))

            for _ in range(drop_steps):
                self._step_simulation()
                peak_force = max(peak_force, self._max_contact_force_for_body(bid))
                max_tilt = max(max_tilt, self._body_tilt_deg(bid))

            for _ in range(settle_steps):
                self._step_simulation(clamp_bodies=[bid])
                peak_force = max(peak_force, self._max_contact_force_for_body(bid))
                max_tilt = max(max_tilt, self._body_tilt_deg(bid))

            tilts = self.current_layer_tilts()
            guard_triggered = any(t >= self.layer_tilt_guard_deg for t in tilts.values())
            extra_dwell = 0.10 if guard_triggered else 0.0
            dwell_s = float(self.base_dwell_s + extra_dwell)
            dwell_frames = max(0, int(math.ceil(dwell_s / self.time_step)))
            for _ in range(dwell_frames):
                self._step_simulation()

            peak_impulse = peak_force * self.time_step
            self.placement_metrics.append(
                {
                    "name": spec.name,
                    "order": int(spec.order),
                    "layer": int(layer_idx) if layer_idx is not None else None,
                    "peak_normal_force": peak_force,
                    "peak_normal_impulse": peak_impulse,
                    "max_tilt_deg": max_tilt,
                    "preview_frames": preview_steps,
                    "micro_drop_frames": drop_steps,
                    "settle_frames": settle_steps,
                    "dwell_s": dwell_s,
                    "extra_dwell_s": extra_dwell,
                    "layer_tilts": {int(k): float(v) for k, v in tilts.items()},
                }
            )

        elif self.placement_mode == "dynamic":
            # Raise by a drop distance (global or per CSV) and let gravity do the rest.
            use_csv = bool(getattr(self, "use_csv_drop_for_dynamic", False))
            drop = float(spec.drop_height) if use_csv else float(getattr(self, "drop_distance", 0.0))
            if drop > 0.0:
                pos, orn = p.getBasePositionAndOrientation(bid, physicsClientId=self.cid)
                p.resetBasePositionAndOrientation(bid, (pos[0], pos[1], pos[2] + drop), orn, physicsClientId=self.cid)

            # Optional but recommended: continuous collision detection for small fast drops.
            he = self._half_by_id.get(bid, (spec.size_x/2.0, spec.size_y/2.0, spec.size_z/2.0))
            swept = 0.25 * max(min(he), 1e-6)
            p.changeDynamics(bid, -1,
                             ccdSweptSphereRadius=swept,
                             contactProcessingThreshold=0,
                             physicsClientId=self.cid)
        return bid

    # New: spawn a right-triangular "wedge" prism (run along X, width along Y, rise along Z)
    def spawn_wedge(self, spec: BlockSpec, *, rgba: Optional[Tuple[float,float,float,float]]=None, as_support: bool=True) -> int:
        rx, wy, rz = float(spec.size_x), float(spec.size_y), float(spec.size_z)
        # Vertically-centered wedge so posZ behaves as center height
        z0 = -rz / 2.0
        z1 = +rz / 2.0
        verts = [
            (-rx/2, -wy/2, z0), ( rx/2, -wy/2, z0), (-rx/2, -wy/2, z1),
            (-rx/2,  wy/2, z0), ( rx/2,  wy/2, z0), (-rx/2,  wy/2, z1),
        ]
        shape_type = p.GEOM_CONVEX_HULL if hasattr(p, "GEOM_CONVEX_HULL") else p.GEOM_MESH
        col = p.createCollisionShape(shapeType=shape_type, vertices=verts, physicsClientId=self.cid)
        if rgba is None:
            rgba = (0.8, 0.8, 0.8, 1.0)
        try:
            vis = p.createVisualShape(
                shapeType=shape_type,
                vertices=verts,
                rgbaColor=rgba,
                physicsClientId=self.cid,
            )
        except Exception:
            # Older PyBullet builds running in DIRECT mode occasionally reject
            # convex-hull visual definitions even though the collision shape
            # succeeds.  Treat this as non-fatal so headless simulations can
            # continue using the collision geometry alone.
            vis = -1
        rxyz_rad = [math.radians(float(spec.rotX)), math.radians(float(spec.rotY)), math.radians(float(spec.rotZ))]
        quat = p.getQuaternionFromEuler(rxyz_rad)
        pos = (float(spec.posX), float(spec.posY), float(spec.posZ + max(0.0, spec.drop_height)))
        bid = p.createMultiBody(baseMass=float(spec.mass),
                                baseCollisionShapeIndex=col,
                                baseVisualShapeIndex=vis,
                                basePosition=pos,
                                baseOrientation=quat,
                                physicsClientId=self.cid)
        half = (rx/2.0, wy/2.0, max(rz, 1e-6)/2.0)
        self._half_by_id[bid] = half
        r_min = 0.25 * max(min(half), 1e-6)
        dyn_kwargs = dict(
            lateralFriction=float(spec.mu_slide),
            spinningFriction=float(spec.mu_spin),
            rollingFriction=float(spec.mu_roll),
            frictionAnchor=1,
            restitution=float(spec.restitution),
            linearDamping=0.01,
            angularDamping=0.02,
            ccdSweptSphereRadius=r_min,
            contactProcessingThreshold=0,
            physicsClientId=self.cid,
        )
        if spec.anisotropic_friction is not None:
            dyn_kwargs["anisotropicFriction"] = [
                float(v) for v in spec.anisotropic_friction
            ]
        p.changeDynamics(bid, -1, **dyn_kwargs)
        self._bodies.append(bid)
        if as_support and float(spec.mass) == 0.0:
            self.support_ids.add(bid)
        else:
            layer_idx = self._infer_layer_index(spec)
            if layer_idx is not None:
                self._layer_index_by_body[bid] = layer_idx
        return bid

    # ---- Internal helpers -------------------------------------------------------
    def _step_simulation(self, *, clamp_bodies: Optional[Iterable[int]] = None):
        if clamp_bodies:
            for bid in clamp_bodies:
                p.resetBaseVelocity(bid, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), physicsClientId=self.cid)
        p.stepSimulation(physicsClientId=self.cid)
        if self.gui and self.step_sleep > 0.0:
            time.sleep(self.step_sleep)
        self._sim_time += self.time_step
        self._record_layer_tilts()

    def _record_layer_tilts(self):
        if not self._layer_index_by_body:
            return
        for bid, layer in list(self._layer_index_by_body.items()):
            if bid not in self._bodies:
                continue
            tilt = self._body_tilt_deg(bid)
            history = self.layer_tilt_history.setdefault(layer, [])
            history.append((self._sim_time, tilt))

    def _infer_layer_index(self, spec: BlockSpec) -> Optional[int]:
        try:
            if spec.size_z <= 0:
                return None
            layer_est = (float(spec.posZ) - 0.5 * float(spec.size_z)) / float(spec.size_z)
            return int(round(layer_est))
        except Exception:
            return None

    def _body_tilt_deg(self, bid: int) -> float:
        _, quat = p.getBasePositionAndOrientation(bid, physicsClientId=self.cid)
        R = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
        z_body = R[:, 2]
        cosang = float(np.clip(np.dot(z_body, np.array([0.0, 0.0, 1.0])), -1.0, 1.0))
        return math.degrees(math.acos(cosang))

    def _max_contact_force_for_body(self, bid: int) -> float:
        max_force = 0.0
        for contact in self.get_contacts([bid]):
            max_force = max(max_force, float(contact.normal_force))
        return max_force

    def current_layer_tilts(self) -> Dict[int, float]:
        tilts: Dict[int, float] = {}
        for bid, layer in self._layer_index_by_body.items():
            if bid not in self._bodies:
                continue
            tilts[layer] = max(tilts.get(layer, 0.0), self._body_tilt_deg(bid))
        return tilts

    # ---- Simulation loop --------------------------------------------------------
    def step(self, n: int=1):
        for _ in range(int(n)):
            self._step_simulation()

    def step_until_settled(
        self,
        body_ids: List[int],
        v_th: float = 1e-3,
        w_th: float = 1e-3,
        dwell_s: float = 1.0,
        max_s: float = 10.0,
        should_stop=None,
    ) -> Tuple[bool, float]:
        """All listed bodies must stay under thresholds continuously for dwell_s.
        Returns (settled_ok, elapsed_seconds). Aborts early if should_stop() is True.
        """
        dt = self.time_step
        need = max(1, int(dwell_s / dt))
        streak = 0
        t = 0.0
        max_steps = int(max_s / dt)
        for _ in range(max_steps):
            # NEW: user abort
            if callable(should_stop) and should_stop():
                return False, t

            self._step_simulation()
            t += dt
            ok = True
            for bid in body_ids:
                lin, ang = p.getBaseVelocity(bid, physicsClientId=self.cid)
                v = math.sqrt(lin[0]**2 + lin[1]**2 + lin[2]**2)
                w = math.sqrt(ang[0]**2 + ang[1]**2 + ang[2]**2)
                if not (v < v_th and w < w_th):
                    ok = False
                    break
            streak = streak + 1 if ok else 0
            if streak >= need:
                return True, t
        return False, t  # timeout

    # ---- Queries ----------------------------------------------------------------
    def get_contacts(self, bodies: Optional[List[int]]=None) -> List[Contact]:
        """Wrap p.getContactPoints into lightweight Contact objects. If bodies is None, fetch all."""
        cps = p.getContactPoints(physicsClientId=self.cid)
        out: List[Contact] = []
        allow = set(bodies) if bodies else None
        for c in cps:
            a, b = c[1], c[2]  # bodyUniqueIdA, bodyUniqueIdB
            if (allow is not None) and (a not in allow and b not in allow and self.ground_id not in (a, b)):
                continue
            posA = c[5]  # positionOnA in world coords
            posB = c[6]  # positionOnB in world coords
            normalB = c[7]  # contact normal on B, pointing from B to A
            normal_force = c[9]
            out.append(Contact(a, b, posA, posB, normalB, normal_force))
        return out

    def ground_contact_cloud(self, contacts):
        gid = self.ground_id
        pts, forces = [], []
        if gid is None:
            return np.array(pts), np.array(forces)
        for c in contacts:
            if gid in (c.bodyA, c.bodyB):
                pts.append(c.position_on_a if c.bodyA == gid else c.position_on_b)
                forces.append(c.normal_force)
        return np.array(pts), np.array(forces)

    def get_body_state(self, body_id: int) -> Dict[str, Any]:
        pos, quat = p.getBasePositionAndOrientation(body_id, physicsClientId=self.cid)
        lin, ang = p.getBaseVelocity(body_id, physicsClientId=self.cid)
        mass = p.getDynamicsInfo(body_id, -1, physicsClientId=self.cid)[0]
        return {"pos": pos, "quat": quat, "lin": lin, "ang": ang, "mass": mass}

    def ground_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return ground position and rotation matrix (3x3)."""
        if self.ground_id is None:
            raise RuntimeError("Ground not created yet.")
        pg, qg = p.getBasePositionAndOrientation(self.ground_id, physicsClientId=self.cid)
        R = np.array(p.getMatrixFromQuaternion(qg)).reshape(3,3)
        return np.array(pg), R

    # ---------------- Camera helpers (GUI only) ---------------------------------
    def set_step_sleep(self, seconds: float) -> None:
        """Sleep this many seconds after each physics step when GUI is on."""
        self.step_sleep = max(0.0, float(seconds))

    def set_camera_preset(
        self, preset: str = "Isometric", distance: float | None = 1.0
    ) -> None:
        """Quick camera presets for the Debug Visualizer (perspective)."""
        if not self.gui:
            return
        he = (self._ground_opts or {"half_extents": (5.0, 5.0, 0.1)})["half_extents"]
        he = tuple(float(v) for v in he)
        # Use provided distance if given; otherwise fall back to a scene-size heuristic.
        radius = float(distance) if (distance is not None) else (max(he[0], he[1]) * 3.0 + 1.0)
        target = [0.0, 0.0, he[2]]
        yaw, pitch = 45.0, -35.0  # Isometric
        name = (preset or "Isometric").lower()
        if "front" in name or "x+" in name:
            yaw, pitch = 0.0, 0.0
        elif "back" in name or "x-" in name:
            yaw, pitch = 180.0, 0.0
        elif "left" in name or "y+" in name:
            yaw, pitch = 90.0, 0.0
        elif "right" in name or "y-" in name:
            yaw, pitch = -90.0, 0.0
        elif "top" in name or "z+" in name:
            yaw, pitch = 0.0, -89.9
        p.resetDebugVisualizerCamera(radius, yaw, pitch, target, physicsClientId=self.cid)
        self._camera_initialized = True

    def servo_place_along_normal(
        self, bid: int, half_extents, xy_hint=None,
        target_clearance: float = None, max_iters: int = 240, max_step: float = 1e-3
    ) -> bool:
        """Kinematically move along the support normal to a tiny clearance.
        Returns True if target clearance was reached, False on iteration limit.
        """
        import numpy as np
        hx, hy, hz = map(float, half_extents)
        if target_clearance is None:
            target_clearance = float(self.servo_clearance)

        # Current pose and (x,y) hint for the support ray
        pos, _ = p.getBasePositionAndOrientation(bid, physicsClientId=self.cid)
        x = float(xy_hint[0]) if xy_hint else float(pos[0])
        y = float(xy_hint[1]) if xy_hint else float(pos[1])

        # Probe the support directly below (x,y) to get support point and normal
        from_pt = (x, y, max(0.5, pos[2] + hz + 0.2))
        to_pt   = (x, y, from_pt[2] - 2.0)
        hit = p.rayTest(from_pt, to_pt, physicsClientId=self.cid)[0]
        pt, n = hit[3], hit[4]

        n = np.asarray(n, float)
        n /= (np.linalg.norm(n) or 1.0)

        for _ in range(int(max_iters)):
            pos, orn = p.getBasePositionAndOrientation(bid, physicsClientId=self.cid)
            # Distance from bottom face to support plane, measured along normal
            d_face = float(np.dot(np.asarray(pos) - np.asarray(pt), n) - hz)
            if d_face <= target_clearance:
                return True
            step = min(max_step, max(0.0, d_face - target_clearance))
            new_pos = np.asarray(pos) - n * step
            p.resetBasePositionAndOrientation(bid, new_pos.tolist(), orn, physicsClientId=self.cid)
            self._step_simulation()
        return False

    def hold_and_release(self, bid: int, frames: int = None):
        """Clamp linear/angular velocities for a few frames to remove residual bias before release."""
        if frames is None:
            frames = int(self.hold_frames)
        for _ in range(max(0, frames)):
            self._step_simulation(clamp_bodies=[bid])

    def disconnect(self):
        try:
            p.disconnect(physicsClientId=self.cid)
        except Exception:
            # already disconnected
            pass
