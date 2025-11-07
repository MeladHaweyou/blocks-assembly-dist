# geometry.py
# Small, dependency-free 2D geometry helpers (convex hull & distance).

from __future__ import annotations
from typing import List, Tuple
import math

Point = Tuple[float, float]

def convex_hull_xy(points: List[Point]) -> List[Point]:
    """Monotone chain convex hull. Returns points in CCW order, no duplicate last point."""
    pts = sorted(set((float(x), float(y)) for (x, y) in points))
    if len(pts) <= 1:
        return pts
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    return hull

def _point_in_convex_polygon(pt: Point, poly: List[Point], eps: float=1e-12) -> bool:
    """Check if point is inside (or on edge of) a convex polygon in CCW order."""
    if len(poly) < 3:
        # treat segment or point
        return _distance_point_to_polygon(pt, poly) <= eps
    sign = None
    for i in range(len(poly)):
        a = poly[i]
        b = poly[(i+1) % len(poly)]
        cross = (b[0]-a[0])*(pt[1]-a[1]) - (b[1]-a[1])*(pt[0]-a[0])
        if abs(cross) <= eps:
            continue  # on edge
        if sign is None:
            sign = cross > 0
        elif (cross > 0) != sign:
            return False
    return True

def _distance_point_to_segment(pt: Point, a: Point, b: Point) -> float:
    """Euclidean distance from point to segment AB."""
    (x, y) = pt
    (x1, y1) = a
    (x2, y2) = b
    dx = x2 - x1; dy = y2 - y1
    if dx == dy == 0.0:
        return math.hypot(x - x1, y - y1)
    t = ((x - x1)*dx + (y - y1)*dy) / (dx*dx + dy*dy)
    t = max(0.0, min(1.0, t))
    projx = x1 + t*dx; projy = y1 + t*dy
    return math.hypot(x - projx, y - projy)

def _distance_point_to_polygon(pt: Point, poly: List[Point]) -> float:
    if not poly:
        return float("inf")
    mind = float("inf")
    n = len(poly)
    if n == 1:
        return math.hypot(pt[0]-poly[0][0], pt[1]-poly[0][1])
    for i in range(n):
        a = poly[i]; b = poly[(i+1) % n]
        d = _distance_point_to_segment(pt, a, b)
        if d < mind:
            mind = d
    return mind

def point_in_convex_polygon_with_margin(pt: Point, poly: List[Point], eps: float=1e-12) -> Tuple[bool, float]:
    """Return (inside, signed_distance) where distance>0 if inside, <0 if outside, 0 on edge."""
    inside = _point_in_convex_polygon(pt, poly, eps=eps)
    d = _distance_point_to_polygon(pt, poly)
    return inside, (d if inside else -d)
