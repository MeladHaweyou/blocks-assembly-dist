# Rhino → Block Assembly Unified CSV exporter

import rhinoscriptsyntax as rs
import scriptcontext as sc
import Rhino
import csv, math, sys

# ------------ CONFIG ------------
MM_PER_M = 1000.0
ROUND_MM_STEP = 0.1      # round type key to nearest 0.1 mm
DENSITY_KG_M3 = 700.0
DEFAULT_MATERIAL = "WOOD"
PLACEMENT_MODE = "gentle"
SERVO_CLEARANCE_MM = 0.3

# Z: the lowest block bottom will be shifted to this world Z level (meters)
Z_OFFSET_M = 0.0

# XY: shift whole assembly near origin without changing internal distances.
#   "min": align overall bounding box min corner to (X_OFFSET_M, Y_OFFSET_M)
#   "center": move overall XY center to (X_OFFSET_M, Y_OFFSET_M)
XY_SHIFT_MODE = "min"    # "min" or "center"
X_OFFSET_M = 0.0
Y_OFFSET_M = 0.0

STRICT_CLOSED_ONLY = True
# --------------------------------

def unit_scale_to_m():
    doc = sc.doc or Rhino.RhinoDoc.ActiveDoc
    try:
        return Rhino.RhinoMath.UnitScale(doc.ModelUnitSystem, Rhino.UnitSystem.Meters)
    except:
        return 1.0

def is_solid_brep(b):
    try: return b and b.IsValid and b.IsSolid
    except: return False

def volume_centroid_or_bbox_center(b):
    try:
        mp = Rhino.Geometry.VolumeMassProperties.Compute(b)
        if mp: return mp.Centroid
    except: pass
    return b.GetBoundingBox(True).Center

def largest_planar_face_plane(brep):
    best_area = -1.0; best_pl = None
    for f in brep.Faces:
        try:
            ok, pl = f.TryGetPlane()
        except TypeError:
            ok, pl = f.TryGetPlane(sc.doc.ModelAbsoluteTolerance)
        if not ok: continue
        try:
            dup = f.DuplicateFace(False)
            amp = Rhino.Geometry.AreaMassProperties.Compute(dup)
            area = amp.Area if amp else 0.0
        except:
            area = 0.0
        if area > best_area:
            best_area = area; best_pl = pl
    return best_pl

def longest_edge_dir_in_plane(brep, plane, in_plane_dot_tol=0.15):
    if plane is None: return None
    n = plane.Normal
    best_len = 0.0; best_vec = None
    for e in brep.Edges:
        crv = e.EdgeCurve
        if not crv: continue
        t0,t1 = crv.Domain.T0, crv.Domain.T1
        p0 = crv.PointAt(t0); p1 = crv.PointAt(t1)
        v = p1 - p0; L = v.Length
        if L <= sc.doc.ModelAbsoluteTolerance: continue
        v.Unitize()
        if abs(Rhino.Geometry.Vector3d.Multiply(v, n)) > in_plane_dot_tol:
            continue
        if L > best_len:
            best_len = L; best_vec = v
    return best_vec

def orthonormal_axes(x_hint, z_axis):
    if not x_hint or x_hint.IsTiny() or not z_axis or z_axis.IsTiny(): return None
    x = Rhino.Geometry.Vector3d(x_hint); z = Rhino.Geometry.Vector3d(z_axis)
    x.Unitize(); z.Unitize()
    if abs(Rhino.Geometry.Vector3d.Multiply(x, z)) > 0.999: return None
    y = Rhino.Geometry.Vector3d.CrossProduct(z, x); y.Unitize()
    x = Rhino.Geometry.Vector3d.CrossProduct(y, z); x.Unitize()
    return x, y, z

def dims_along_axes(brep, origin, x, y, z):
    mins = [float("+inf")]*3; maxs = [float("-inf")]*3
    for vtx in brep.Vertices:
        p = vtx.Location
        v = p - origin
        u = Rhino.Geometry.Vector3d.Multiply(v, x)
        v2 = Rhino.Geometry.Vector3d.Multiply(v, y)
        w = Rhino.Geometry.Vector3d.Multiply(v, z)
        mins[0] = min(mins[0], u); maxs[0] = max(maxs[0], u)
        mins[1] = min(mins[1], v2); maxs[1] = max(maxs[1], v2)
        mins[2] = min(mins[2], w); maxs[2] = max(maxs[2], w)
    return abs(maxs[0]-mins[0]), abs(maxs[1]-mins[1]), abs(maxs[2]-mins[2])

def clamp(x,a,b): return a if x<a else (b if x>b else x)
def rad2deg(a): return a*180.0/math.pi

def euler_xyz_from_axes(x, y, z):
    R = ((x.X, y.X, z.X),
         (x.Y, y.Y, z.Y),
         (x.Z, y.Z, z.Z))
    r11,r12,r13 = R[0]; r21,r22,r23 = R[1]; r31,r32,r33 = R[2]
    pitch = math.asin(clamp(-r31,-1.0,1.0))
    cp = math.cos(pitch)
    if abs(cp) > 1e-8:
        roll = math.atan2(r32, r33)
        yaw  = math.atan2(r21, r11)
    else:
        roll = 0.0
        yaw  = math.atan2(-r12, r22)
    return rad2deg(roll), rad2deg(pitch), rad2deg(yaw)

def round_mm(v_m, step=ROUND_MM_STEP):
    mm = v_m * MM_PER_M
    return round(mm/step)*step

def type_key_from_sizes_m(sx_m, sy_m, sz_m):
    vals = sorted([sx_m, sy_m, sz_m], reverse=True)
    a,b,c = (round_mm(vals[0]), round_mm(vals[1]), round_mm(vals[2]))
    def fmt(mm):
        s = ("%g"%mm)
        return s.replace(".0","")
    name = "%sx%sx%s" % (fmt(a), fmt(b), fmt(c))
    return name, (vals[0], vals[1], vals[2])

def write_csv(path, block_types, instances):
    header = [
        "section","key","value","material","density_kgm3","mu_slide","mu_spin","mu_roll",
        "restitution","block_type","size_x_m","size_y_m","size_z_m","material_of_type","shape",
        "wedge_run_x_m","wedge_width_y_m","wedge_rise_z_m","color_rgba",
        "order","label","posX_m","posY_m","posZ_m","rotX_deg","rotY_deg","rotZ_deg","delay_after_s"
    ]
    if sys.version_info[0] >= 3:
        f = open(path, "w", newline="", encoding="utf-8")
    else:
        f = open(path, "wb")
    with f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerow(["meta","placement_mode",PLACEMENT_MODE] + [""]*(len(header)-3))
        w.writerow(["meta","servo_clearance_mm",SERVO_CLEARANCE_MM] + [""]*(len(header)-3))
        w.writerow(["materials","","",DEFAULT_MATERIAL,DENSITY_KG_M3,0.6,0,0,0] + [""]*(len(header)-9))
        for name in block_types["order"]:
            sx,sy,sz = block_types["sizes_m"][name]
            w.writerow(["block_types","","","","","","","","",
                        name, "%.6f"%sx, "%.6f"%sy, "%.6f"%sz,
                        DEFAULT_MATERIAL, "box",
                        "","","","", "", "", "", "", "", "", "", ""])
        for ins in instances:
            px,py,pz = ins["pos_m"]; rx,ry,rz = ins["rot_deg"]
            w.writerow(["assembly","","","","","","","","",
                        ins["block_type"], "", "", "", "", "",
                        "", "", "", ins["color_rgba"],
                        ins["order"], ins["label"],
                        "%.6f"%px, "%.6f"%py, "%.6f"%pz,
                        "%.6f"%rx, "%.6f"%ry, "%.6f"%rz, 0])

def main():
    scale_to_m = unit_scale_to_m()
    sel = rs.SelectedObjects() or rs.GetObjects(
        "Select closed polysurfaces (solids) to export",
        rs.filter.polysurface, preselect=True, select=True
    )
    if not sel:
        print("No objects selected."); return

    records = []
    notes = []

    # Track the overall geometric XY extents (axis-aligned world bbox)
    global_min_x_m = float("+inf")
    global_min_y_m = float("+inf")
    global_max_x_m = float("-inf")
    global_max_y_m = float("-inf")

    for oid in sel:
        ro = sc.doc.Objects.Find(oid)
        if ro is None:
            notes.append("%s: skipped (not found)"%str(oid)); continue
        geo = ro.Geometry
        brep = geo if isinstance(geo, Rhino.Geometry.Brep) else None
        if brep is None:
            try: brep = rs.coercebrep(oid)
            except: brep = None
        if not is_solid_brep(brep):
            if STRICT_CLOSED_ONLY:
                notes.append("%s: skipped (not a closed solid)"%str(oid)); continue

        pl = largest_planar_face_plane(brep)
        xhint = longest_edge_dir_in_plane(brep, pl)
        axes = orthonormal_axes(xhint, pl.Normal if pl else None)
        if axes is None:
            notes.append("%s: skipped (failed local frame)"%str(oid)); continue
        x_axis, y_axis, z_axis = axes

        c = volume_centroid_or_bbox_center(brep)
        sx_u, sy_u, sz_u = dims_along_axes(brep, c, x_axis, y_axis, z_axis)  # model units

        # Convert to meters
        sx_m, sy_m, sz_m = sx_u*scale_to_m, sy_u*scale_to_m, sz_u*scale_to_m
        if min(sx_m, sy_m, sz_m) <= 0.0:
            notes.append("%s: skipped (zero dimension)"%str(oid)); continue

        rx, ry, rz = euler_xyz_from_axes(x_axis, y_axis, z_axis)

        posX_m = float(c.X)*scale_to_m
        posY_m = float(c.Y)*scale_to_m
        posZ_m = float(c.Z)*scale_to_m

        # Update global XY extents using actual geometry bounding boxes (world-aligned)
        bb = brep.GetBoundingBox(True)
        global_min_x_m = min(global_min_x_m, bb.Min.X*scale_to_m)
        global_min_y_m = min(global_min_y_m, bb.Min.Y*scale_to_m)
        global_max_x_m = max(global_max_x_m, bb.Max.X*scale_to_m)
        global_max_y_m = max(global_max_y_m, bb.Max.Y*scale_to_m)

        try:
            col = ro.Attributes.DrawColor(sc.doc)
            color_rgba = "%d,%d,%d,255"%(col.R,col.G,col.B)
        except:
            color_rgba = "200,200,200,255"

        records.append({
            "id": str(oid),
            "pos_m": [posX_m, posY_m, posZ_m],
            "sizes_m_xyz": (sx_m, sy_m, sz_m),   # keep axis-aligned sizes for Z-bottom calc
            "rot_deg": (rx, ry, rz),
            "color_rgba": color_rgba
        })

    print("Selected: %d | valid solids: %d"%(len(sel), len(records)))
    if not records:
        if notes:
            print("Notes:"); 
            for n in notes: print(" -", n)
        print("No valid solids exported."); 
        return
    if notes:
        print("Notes / diagnostics:")
        for n in notes: print(" -", n)

    # Build unique block types and instances
    bt_sizes_m = {}
    bt_order = []
    instances = []
    min_bottom_z = float("+inf")

    for i, r in enumerate(records, 1):
        sx,sy,sz = r["sizes_m_xyz"]
        name, canon_sizes_m = type_key_from_sizes_m(sx, sy, sz)
        if name not in bt_sizes_m:
            bt_sizes_m[name] = canon_sizes_m
            bt_order.append(name)

        posX, posY, posZ = r["pos_m"]
        bottom_z = posZ - 0.5*sz
        if bottom_z < min_bottom_z:
            min_bottom_z = bottom_z

        instances.append({
            "order": i,
            "label": r["id"],
            "block_type": name,
            "pos_m": [posX, posY, posZ],   # will shift below
            "rot_deg": r["rot_deg"],
            "color_rgba": r["color_rgba"]
        })

    # Z shift: lowest bottom -> Z_OFFSET_M
    z_shift = (Z_OFFSET_M - min_bottom_z)
    for ins in instances:
        ins["pos_m"][2] += z_shift

    # XY shift: move whole assembly near (X_OFFSET_M, Y_OFFSET_M)
    if XY_SHIFT_MODE.lower() == "center":
        cx = 0.5*(global_min_x_m + global_max_x_m)
        cy = 0.5*(global_min_y_m + global_max_y_m)
        x_shift = X_OFFSET_M - cx
        y_shift = Y_OFFSET_M - cy
        xy_mode_desc = "center"
    else:  # "min"
        x_shift = X_OFFSET_M - global_min_x_m
        y_shift = Y_OFFSET_M - global_min_y_m
        xy_mode_desc = "min corner"
    for ins in instances:
        ins["pos_m"][0] += x_shift
        ins["pos_m"][1] += y_shift

    out_path = rs.SaveFileName("Save assembly CSV", "CSV (*.csv)|*.csv||", None, "assembly.csv")
    if not out_path:
        print("Export canceled."); return

    write_csv(out_path, {"sizes_m": bt_sizes_m, "order": bt_order}, instances)

    print("Wrote:", out_path)
    print("Exported %d instance(s) • %d block type(s)." % (len(instances), len(bt_order)))
    print("Applied Z shift of %.6f m (lowest bottom now at %.6f m)." % (z_shift, Z_OFFSET_M))
    print("Applied XY shift of (%.6f, %.6f) m using %s alignment to (%.6f, %.6f)." %
          (x_shift, y_shift, xy_mode_desc, X_OFFSET_M, Y_OFFSET_M))

if __name__ == "__main__":
    main()
