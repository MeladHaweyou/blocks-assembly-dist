# analysis.py
# Stability metrics and kinematic queries for the block assembly.
from __future__ import annotations
from typing import List, Tuple
import math
import numpy as np

try:
    import pybullet as p
except ImportError:
    # This module is imported by app.py only when running locally.
    pass

from geometry import convex_hull_xy, point_in_convex_polygon_with_margin

# ---------- COM & projections ---------------------------------------------------

def assembly_com(world, body_ids: List[int]) -> np.ndarray:
    """Mass-weighted COM in world coordinates."""
    total_m = 0.0
    com = np.zeros(3)
    pos_accum = []
    for bid in body_ids:
        mass = p.getDynamicsInfo(bid, -1, physicsClientId=world.cid)[0]
        pos, _ = p.getBasePositionAndOrientation(bid, physicsClientId=world.cid)
        total_m += mass
        com += mass * np.array(pos)
        pos_accum.append(np.array(pos))
    if total_m <= 0:
        if pos_accum:
            return np.mean(np.vstack(pos_accum), axis=0)
        return com
    return com / total_m

def project_point_to_ground_xy(world, point_w: np.ndarray) -> Tuple[float,float]:
    """Express a world point in ground frame and drop z -> return (x,y) in ground-local plane."""
    pg, Rg = world.ground_pose()
    # transform: local = Rg^T (p - pg)
    local = Rg.T.dot(point_w - pg)
    return float(local[0]), float(local[1])

# ---------- Support polygon from ground contacts -------------------------------

def support_polygon_from_ground_contacts(world, contacts, z_epsilon: float=3e-3):
    """Collect support contact points (ground + any world.support_ids), project to ground XY, return convex hull."""
    pg, Rg = world.ground_pose()
    pts_xy: list[tuple[float, float]] = []
    support_ids = set(getattr(world, "support_ids", set([getattr(world, "ground_id", -1)])))
    for c in contacts:
        a_is_sup = c.bodyA in support_ids
        b_is_sup = c.bodyB in support_ids
        if not (a_is_sup ^ b_is_sup):
            # ignore non-support contacts and support-support contacts
            continue
        pos_w = c.position_on_a if a_is_sup else c.position_on_b
        local = Rg.T.dot(np.array(pos_w) - pg)  # project into ground frame
        pts_xy.append((float(local[0]), float(local[1])))
    if not pts_xy:
        return []  # no support yet (mid-air)
    hull = convex_hull_xy(pts_xy)
    return hull  # list of (x,y) in ground frame

# ---------- Tilt & slip ---------------------------------------------------------

def max_tilt_deg(world, body_ids: List[int]) -> float:
    """Max angle between each body's local +Z and world +Z."""
    z_world = np.array([0,0,1.0])
    max_deg = 0.0
    for bid in body_ids:
        _, quat = p.getBasePositionAndOrientation(bid, physicsClientId=world.cid)
        R = np.array(p.getMatrixFromQuaternion(quat)).reshape(3,3)
        z_body = R[:,2]
        cosang = float(np.clip(z_world.dot(z_body), -1.0, 1.0))
        ang = math.degrees(math.acos(cosang))
        max_deg = max(max_deg, ang)
    return max_deg

def body_point_velocity(world, bid: int, pos_world) -> np.ndarray:
    """Velocity of a body point in world coordinates: v = v_com + ω × r."""
    pos_b, _ = p.getBasePositionAndOrientation(bid, physicsClientId=world.cid)
    v, w = p.getBaseVelocity(bid, physicsClientId=world.cid)
    r = np.array(pos_world) - np.array(pos_b)
    return np.array(v) + np.cross(np.array(w), r)

def slip_flag_from_contacts(world, contacts, tangential_vel_th: float=2e-3) -> bool:
    """Return True if any contact shows significant tangential relative speed."""
    for c in contacts:
        pa = np.array(c.position_on_a)
        pb = np.array(c.position_on_b)
        n = np.array(c.normal)
        if np.linalg.norm(n) < 1e-9:
            continue
        n = n / np.linalg.norm(n)
        va = body_point_velocity(world, c.bodyA, pa)
        vb = body_point_velocity(world, c.bodyB, pb)
        rel = va - vb
        vt = rel - (rel.dot(n)) * n  # tangential component
        if float(np.linalg.norm(vt)) > tangential_vel_th:
            return True
    return False


def center_of_pressure_xy(world, contacts):
    """Return (cop_xy, total_normal_force) using contacts against any support in world.support_ids, projected to ground XY."""
    pg, Rg = world.ground_pose()
    support_ids = set(getattr(world, "support_ids", set([getattr(world, "ground_id", -1)])))
    num = np.zeros(2)
    den = 0.0
    for c in contacts:
        a_is_sup = c.bodyA in support_ids
        b_is_sup = c.bodyB in support_ids
        if not (a_is_sup ^ b_is_sup):
            continue
        pos_w = c.position_on_a if a_is_sup else c.position_on_b
        fN = float(c.normal_force)
        local = Rg.T.dot(np.array(pos_w) - pg)  # project to ground-local XY
        num += fN * np.array([local[0], local[1]])
        den += fN
    if den <= 0.0:
        return None, 0.0
    cop = (float(num[0] / den), float(num[1] / den))
    return cop, float(den)


def cop_margin_mm(cop_xy, hull_xy) -> float:
    if (cop_xy is None) or (not hull_xy):
        return float("-inf")
    _, signed = point_in_convex_polygon_with_margin(cop_xy, hull_xy)
    return float(signed * 1000.0)


def max_tangential_speed(world, contacts) -> float:
    vmax = 0.0
    for c in contacts:
        pa = np.array(c.position_on_a)
        pb = np.array(c.position_on_b)
        n = np.array(c.normal)
        n = n / np.linalg.norm(n) if np.linalg.norm(n) > 1e-9 else n
        va = body_point_velocity(world, c.bodyA, pa)
        vb = body_point_velocity(world, c.bodyB, pb)
        rel = va - vb
        vt = rel - (rel.dot(n)) * n
        vmax = max(vmax, float(np.linalg.norm(vt)))
    return vmax

# ---------- Verdict -------------------------------------------------------------

def stability_verdict(com_xy, hull_xy, tilt_deg: float, slip_flag: bool,
                      margin_mm: float=5.0, max_tilt_deg: float=2.0):
    if not hull_xy:
        return {"stable": False, "reason": "no_support_contacts", "margin_mm": 0.0, "tilt_deg": float(tilt_deg), "slip": bool(slip_flag)}
    inside, signed_dist = point_in_convex_polygon_with_margin(com_xy, hull_xy)
    margin_signed_mm = float(signed_dist * 1000.0)
    margin_now = max(0.0, margin_signed_mm)
    ok = inside and (margin_now >= margin_mm) and (tilt_deg <= max_tilt_deg) and (not slip_flag)
    reason = "ok"
    if not inside:
        reason = "com_outside_support"
    elif margin_now < margin_mm:
        reason = "margin_below_threshold"
    elif tilt_deg > max_tilt_deg:
        reason = "tilt_exceeded"
    elif slip_flag:
        reason = "slip_detected"
    return {"stable": bool(ok),
            "reason": reason,
            "margin_mm": float(margin_now),
            "tilt_deg": float(tilt_deg),
            "slip": bool(slip_flag)}


def com_margin_mm(com_xy, hull_xy) -> float:
    """Return signed COM margin relative to the support polygon in millimetres."""
    if not hull_xy:
        return float("-inf")
    _, signed_dist = point_in_convex_polygon_with_margin(com_xy, hull_xy)
    return float(signed_dist * 1000.0)
