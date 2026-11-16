import struct
import math
import os
import zipfile
import numpy as np
import scipy.spatial

def normalize(v):
    l = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if l == 0:
        return (0.0, 0.0, 0.0)
    return (v[0]/l, v[1]/l, v[2]/l)

def compute_normal(p0, p1, p2):
    u = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    v = (p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2])
    nx = u[1] * v[2] - u[2] * v[1]
    ny = u[2] * v[0] - u[0] * v[2]
    nz = u[0] * v[1] - u[1] * v[0]
    return normalize((nx, ny, nz))

def write_binary_stl(triangles, filepath, header_title="Instant-Robot 3D Printable STL"):
    header = header_title.encode('ascii', errors='ignore')[:80].ljust(80, b' ')
    with open(filepath, 'wb') as f:
        f.write(header)
        f.write(struct.pack('<I', len(triangles)))
        for t in triangles:
            norm = compute_normal(t[0], t[1], t[2])
            f.write(struct.pack('<3f', *norm))
            for pt in t:
                f.write(struct.pack('<3f', float(pt[0]), float(pt[1]), float(pt[2])))
            f.write(struct.pack('<H', 0))

def verify_watertight(triangles, name="Part"):
    edge_counts = {}
    for tri in triangles:
        p0, p1, p2 = tri
        v0 = tuple(round(float(c), 4) for c in p0)
        v1 = tuple(round(float(c), 4) for c in p1)
        v2 = tuple(round(float(c), 4) for c in p2)
        edges = [(v0, v1), (v1, v2), (v2, v0)]
        for e in edges:
            edge_counts[e] = edge_counts.get(e, 0) + 1

    boundary_edges = 0
    for (u, v), count in edge_counts.items():
        opp_count = edge_counts.get((v, u), 0)
        if count != 1 or opp_count != 1:
            boundary_edges += 1

    is_watertight = (boundary_edges == 0)
    print(f"[{name}] {len(triangles)} triangles | Watertight & 2-manifold: {is_watertight} (boundary edges: {boundary_edges})")
    return is_watertight

def make_rounded_rect_2d(min_x, max_x, min_y, max_y, radius=6.0, n_corner_pts=6, round_top_only=False):
    pts = []
    if not round_top_only and radius > 0:
        cx, cy = max_x - radius, min_y + radius
        for i in range(n_corner_pts + 1):
            ang = -math.pi/2 + (math.pi/2) * (i / n_corner_pts)
            pts.append((cx + radius * math.cos(ang), cy + radius * math.sin(ang)))
    else:
        pts.append((max_x, min_y))
    if radius > 0:
        cx, cy = max_x - radius, max_y - radius
        for i in range(n_corner_pts + 1):
            ang = 0 + (math.pi/2) * (i / n_corner_pts)
            pts.append((cx + radius * math.cos(ang), cy + radius * math.sin(ang)))
    else:
        pts.append((max_x, max_y))
    if radius > 0:
        cx, cy = min_x + radius, max_y - radius
        for i in range(n_corner_pts + 1):
            ang = math.pi/2 + (math.pi/2) * (i / n_corner_pts)
            pts.append((cx + radius * math.cos(ang), cy + radius * math.sin(ang)))
    else:
        pts.append((min_x, max_y))
    if not round_top_only and radius > 0:
        cx, cy = min_x + radius, min_y + radius
        for i in range(n_corner_pts + 1):
            ang = math.pi + (math.pi/2) * (i / n_corner_pts)
            pts.append((cx + radius * math.cos(ang), cy + radius * math.sin(ang)))
    else:
        pts.append((min_x, min_y))

    cleaned = []
    for p in pts:
        if not cleaned or math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) > 0.01:
            cleaned.append(p)
    return cleaned

# ─────────────────────────────────────────────────────────────────────────────
# 1. Extended Base Plate (115x80x6mm) - Large Front Mascot Shelf + Counterbores
# ─────────────────────────────────────────────────────────────────────────────
def create_base_plate(
    min_x=-40.0, max_x=75.0, min_y=-40.0, max_y=40.0,
    thickness=6.0, cb_depth=3.2, corner_radius=6.0,
    through_r=1.9, cb_r=3.3, pitch=3.0, amp=0.22
):
    """
    115x80x6mm Extended Base Plate with:
    - 4 rear gusset mounting holes (with 3.2mm deep bottom counterbore pockets)
    - 2 front animal mascot mounting holes (with 3.2mm deep bottom counterbore pockets)
    - 4 rounded outer corners (radius 6mm)
    - Procedural tactile micro-texture on exposed top surfaces
    """
    hole_centers = [
        # Rear gusset holes
        (5.5, -18.0), (-9.5, -18.0),
        (5.5, 18.0),  (-9.5, 18.0),
        # Front animal mascot mounting holes
        (45.0, -20.0), (45.0, 20.0)
    ]
    z_bot = 0.0
    z_step = cb_depth
    z_top_val = thickness

    perim_2d = make_rounded_rect_2d(min_x, max_x, min_y, max_y, radius=corner_radius, n_corner_pts=6)

    dense_perim = []
    n_p = len(perim_2d)
    for i in range(n_p):
        p0 = perim_2d[i]
        p1 = perim_2d[(i+1)%n_p]
        d = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
        steps = max(1, int(round(d / 4.0)))
        for s in range(steps):
            t = s / steps
            dense_perim.append((p0[0] + t*(p1[0]-p0[0]), p0[1] + t*(p1[1]-p0[1])))

    n_hole_pts = 16

    pts_lower = list(dense_perim)
    cb_hole_indices = []
    for hx, hy in hole_centers:
        cur = []
        for i in range(n_hole_pts):
            ang = 2 * math.pi * i / n_hole_pts
            px = float(hx + cb_r * math.cos(ang))
            py = float(hy + cb_r * math.sin(ang))
            cur.append(len(pts_lower))
            pts_lower.append((px, py))
        cb_hole_indices.append(cur)

    xs = np.arange(min_x + pitch, max_x - pitch/2, pitch)
    ys = np.arange(min_y + pitch, max_y - pitch/2, pitch)
    for x in xs:
        for y in ys:
            dist = min(math.hypot(x-hx, y-hy) for hx, hy in hole_centers)
            if dist > cb_r + 2.0:
                pts_lower.append((float(x), float(y)))

    pts_lower_arr = np.array(pts_lower)
    del_lower = scipy.spatial.Delaunay(pts_lower_arr)

    valid_lower = []
    for simplex in del_lower.simplices:
        centroid = pts_lower_arr[simplex].mean(axis=0)
        inside = any(math.hypot(centroid[0]-hx, centroid[1]-hy) < cb_r * 0.999 for hx, hy in hole_centers)
        if not inside:
            valid_lower.append(simplex)

    pts_upper = list(dense_perim)
    through_hole_indices = []
    for hx, hy in hole_centers:
        cur = []
        for i in range(n_hole_pts):
            ang = 2 * math.pi * i / n_hole_pts
            px = float(hx + through_r * math.cos(ang))
            py = float(hy + through_r * math.sin(ang))
            cur.append(len(pts_upper))
            pts_upper.append((px, py))
        through_hole_indices.append(cur)

    for x in xs:
        for y in ys:
            dist = min(math.hypot(x-hx, y-hy) for hx, hy in hole_centers)
            if dist > through_r + 2.0:
                pts_upper.append((float(x), float(y)))

    pts_upper_arr = np.array(pts_upper)
    del_upper = scipy.spatial.Delaunay(pts_upper_arr)

    valid_upper = []
    for simplex in del_upper.simplices:
        centroid = pts_upper_arr[simplex].mean(axis=0)
        inside = any(math.hypot(centroid[0]-hx, centroid[1]-hy) < through_r * 0.999 for hx, hy in hole_centers)
        if not inside:
            valid_upper.append(simplex)

    triangles = []

    # Bottom cap (-Z)
    for s in valid_lower:
        p0 = (pts_lower_arr[s[0]][0], pts_lower_arr[s[0]][1], z_bot)
        p1 = (pts_lower_arr[s[1]][0], pts_lower_arr[s[1]][1], z_bot)
        p2 = (pts_lower_arr[s[2]][0], pts_lower_arr[s[2]][1], z_bot)
        cp = (p1[0]-p0[0])*(p2[1]-p0[1]) - (p1[1]-p0[1])*(p2[0]-p0[0])
        if cp < 0:
            triangles.append((p0, p1, p2))
        else:
            triangles.append((p0, p2, p1))

    # Top cap (+Z)
    z_top_coords = []
    for i, (x, y) in enumerate(pts_upper):
        if i < len(dense_perim):
            z_top_coords.append(z_top_val)
        else:
            dist_h = min(math.hypot(x-hx, y-hy) for hx, hy in hole_centers) - through_r
            if dist_h < 1.5 or (11.0 <= x <= 18.5) or amp == 0:
                z_top_coords.append(z_top_val)
            else:
                kx = math.sin((x - min_x) * math.pi / pitch)
                ky = math.sin((y - min_y) * math.pi / pitch)
                z_top_coords.append(z_top_val + amp * kx * ky)

    for s in valid_upper:
        p0 = (pts_upper_arr[s[0]][0], pts_upper_arr[s[0]][1], z_top_coords[s[0]])
        p1 = (pts_upper_arr[s[1]][0], pts_upper_arr[s[1]][1], z_top_coords[s[1]])
        p2 = (pts_upper_arr[s[2]][0], pts_upper_arr[s[2]][1], z_top_coords[s[2]])
        cp = (p1[0]-p0[0])*(p2[1]-p0[1]) - (p1[1]-p0[1])*(p2[0]-p0[0])
        if cp > 0:
            triangles.append((p0, p1, p2))
        else:
            triangles.append((p0, p2, p1))

    # Outer side walls
    n_dp = len(dense_perim)
    for i in range(n_dp):
        next_i = (i + 1) % n_dp
        b0 = (dense_perim[i][0], dense_perim[i][1], z_bot)
        b1 = (dense_perim[next_i][0], dense_perim[next_i][1], z_bot)
        t0 = (dense_perim[i][0], dense_perim[i][1], z_top_val)
        t1 = (dense_perim[next_i][0], dense_perim[next_i][1], z_top_val)
        triangles.append((b0, b1, t1))
        triangles.append((b0, t1, t0))

    # Lower counterbore hole walls
    for h_indices in cb_hole_indices:
        n_h = len(h_indices)
        for i in range(n_h):
            next_i = (i + 1) % n_h
            idx0, idx1 = h_indices[i], h_indices[next_i]
            b0 = (pts_lower[idx0][0], pts_lower[idx0][1], z_bot)
            b1 = (pts_lower[idx1][0], pts_lower[idx1][1], z_bot)
            t0 = (pts_lower[idx0][0], pts_lower[idx0][1], z_step)
            t1 = (pts_lower[idx1][0], pts_lower[idx1][1], z_step)
            triangles.append((b0, t1, b1))
            triangles.append((b0, t0, t1))

    # Internal counterbore shelf at z_step
    for h_idx in range(len(hole_centers)):
        cb_ind = cb_hole_indices[h_idx]
        th_ind = through_hole_indices[h_idx]
        for i in range(n_hole_pts):
            next_i = (i + 1) % n_hole_pts
            o0 = (pts_lower[cb_ind[i]][0], pts_lower[cb_ind[i]][1], z_step)
            o1 = (pts_lower[cb_ind[next_i]][0], pts_lower[cb_ind[next_i]][1], z_step)
            i0 = (pts_upper[th_ind[i]][0], pts_upper[th_ind[i]][1], z_step)
            i1 = (pts_upper[th_ind[next_i]][0], pts_upper[th_ind[next_i]][1], z_step)
            triangles.append((o1, o0, i0))
            triangles.append((o1, i0, i1))

    # Upper through-hole walls
    for h_indices in through_hole_indices:
        n_h = len(h_indices)
        for i in range(n_h):
            next_i = (i + 1) % n_h
            idx0, idx1 = h_indices[i], h_indices[next_i]
            b0 = (pts_upper[idx0][0], pts_upper[idx0][1], z_step)
            b1 = (pts_upper[idx1][0], pts_upper[idx1][1], z_step)
            t0 = (pts_upper[idx0][0], pts_upper[idx0][1], z_top_val)
            t1 = (pts_upper[idx1][0], pts_upper[idx1][1], z_top_val)
            triangles.append((b0, t1, b1))
            triangles.append((b0, t0, t1))

    return triangles

# ─────────────────────────────────────────────────────────────────────────────
# 2. Clean Upright Face Plate (52x4x95mm) - Front Counterbore + Rounded Top Corners
# ─────────────────────────────────────────────────────────────────────────────
def create_upright_face_plate(
    width=52.0, height=95.0, thickness=4.0,
    base_t=6.0, cb_depth=2.0, corner_radius=6.0,
    through_r=1.9, cb_r=3.3, pitch=3.0, amp=0.20
):
    min_x = 14.5
    max_x = 14.5 + thickness
    min_y, max_y = -width/2, width/2
    min_z = base_t
    max_z = base_t + height

    x_rear = min_x
    x_step = max_x - cb_depth
    x_front = max_x

    face_holes = [
        (-18.0, base_t + 15.0), (-18.0, base_t + 27.0),
        (18.0, base_t + 15.0),  (18.0, base_t + 27.0),
    ]

    perim_2d = make_rounded_rect_2d(min_y, max_y, min_z, max_z, radius=corner_radius, n_corner_pts=6, round_top_only=True)

    dense_perim = []
    n_p = len(perim_2d)
    for i in range(n_p):
        p0 = perim_2d[i]
        p1 = perim_2d[(i+1)%n_p]
        d = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
        steps = max(1, int(round(d / 4.0)))
        for s in range(steps):
            t = s / steps
            dense_perim.append((p0[0] + t*(p1[0]-p0[0]), p0[1] + t*(p1[1]-p0[1])))

    n_hole_pts = 16

    pts_rear = list(dense_perim)
    through_hole_indices = []
    for hy, hz in face_holes:
        cur = []
        for i in range(n_hole_pts):
            ang = 2 * math.pi * i / n_hole_pts
            py = float(hy + through_r * math.cos(ang))
            pz = float(hz + through_r * math.sin(ang))
            cur.append(len(pts_rear))
            pts_rear.append((py, pz))
        through_hole_indices.append(cur)

    ys = np.arange(min_y + pitch, max_y - pitch/2, pitch)
    zs = np.arange(min_z + pitch, max_z - pitch/2, pitch)
    for y in ys:
        for z in zs:
            dist = min(math.hypot(y-hy, z-hz) for hy, hz in face_holes)
            if dist > through_r + 2.0:
                pts_rear.append((float(y), float(z)))

    pts_rear_arr = np.array(pts_rear)
    del_rear = scipy.spatial.Delaunay(pts_rear_arr)

    valid_rear = []
    for simplex in del_rear.simplices:
        centroid = pts_rear_arr[simplex].mean(axis=0)
        inside = any(math.hypot(centroid[0]-hy, centroid[1]-hz) < through_r * 0.999 for hy, hz in face_holes)
        if not inside:
            valid_rear.append(simplex)

    pts_front = list(dense_perim)
    cb_hole_indices = []
    for hy, hz in face_holes:
        cur = []
        for i in range(n_hole_pts):
            ang = 2 * math.pi * i / n_hole_pts
            py = float(hy + cb_r * math.cos(ang))
            pz = float(hz + cb_r * math.sin(ang))
            cur.append(len(pts_front))
            pts_front.append((py, pz))
        cb_hole_indices.append(cur)

    for y in ys:
        for z in zs:
            dist = min(math.hypot(y-hy, z-hz) for hy, hz in face_holes)
            if dist > cb_r + 2.0:
                pts_front.append((float(y), float(z)))

    pts_front_arr = np.array(pts_front)
    del_front = scipy.spatial.Delaunay(pts_front_arr)

    valid_front = []
    for simplex in del_front.simplices:
        centroid = pts_front_arr[simplex].mean(axis=0)
        inside = any(math.hypot(centroid[0]-hy, centroid[1]-hz) < cb_r * 0.999 for hy, hz in face_holes)
        if not inside:
            valid_front.append(simplex)

    triangles = []

    # Rear cap (-X)
    for s in valid_rear:
        p0 = (x_rear, pts_rear_arr[s[0]][0], pts_rear_arr[s[0]][1])
        p1 = (x_rear, pts_rear_arr[s[1]][0], pts_rear_arr[s[1]][1])
        p2 = (x_rear, pts_rear_arr[s[2]][0], pts_rear_arr[s[2]][1])
        cp = (p1[1]-p0[1])*(p2[2]-p0[2]) - (p1[2]-p0[2])*(p2[1]-p0[1])
        if cp < 0:
            triangles.append((p0, p1, p2))
        else:
            triangles.append((p0, p2, p1))

    # Front cap (+X)
    x_front_coords = []
    for i, (y, z) in enumerate(pts_front):
        if i < len(dense_perim) or z >= min_z + 44.0 or amp == 0:
            x_front_coords.append(x_front)
        else:
            dist_h = min(math.hypot(y-hy, z-hz) for hy, hz in face_holes) - cb_r
            if dist_h < 1.5:
                x_front_coords.append(x_front)
            else:
                ky = math.sin((y - min_y) * math.pi / pitch)
                kz = math.sin((z - min_z) * math.pi / pitch)
                x_front_coords.append(x_front + amp * ky * kz)

    for s in valid_front:
        p0 = (x_front_coords[s[0]], pts_front_arr[s[0]][0], pts_front_arr[s[0]][1])
        p1 = (x_front_coords[s[1]], pts_front_arr[s[1]][0], pts_front_arr[s[1]][1])
        p2 = (x_front_coords[s[2]], pts_front_arr[s[2]][0], pts_front_arr[s[2]][1])
        cp = (p1[1]-p0[1])*(p2[2]-p0[2]) - (p1[2]-p0[2])*(p2[1]-p0[1])
        if cp > 0:
            triangles.append((p0, p1, p2))
        else:
            triangles.append((p0, p2, p1))

    # Outer perimeter walls
    n_dp = len(dense_perim)
    for i in range(n_dp):
        next_i = (i + 1) % n_dp
        b0 = (x_rear, dense_perim[i][0], dense_perim[i][1])
        b1 = (x_rear, dense_perim[next_i][0], dense_perim[next_i][1])
        t0 = (x_front, dense_perim[i][0], dense_perim[i][1])
        t1 = (x_front, dense_perim[next_i][0], dense_perim[next_i][1])
        triangles.append((b0, b1, t1))
        triangles.append((b0, t1, t0))

    # Rear through-hole walls
    for h_indices in through_hole_indices:
        n_h = len(h_indices)
        for i in range(n_h):
            next_i = (i + 1) % n_h
            idx0, idx1 = h_indices[i], h_indices[next_i]
            b0 = (x_rear, pts_rear[idx0][0], pts_rear[idx0][1])
            b1 = (x_rear, pts_rear[idx1][0], pts_rear[idx1][1])
            t0 = (x_step, pts_rear[idx0][0], pts_rear[idx0][1])
            t1 = (x_step, pts_rear[idx1][0], pts_rear[idx1][1])
            triangles.append((b0, t1, b1))
            triangles.append((b0, t0, t1))

    # Internal counterbore shelf at x_step
    for h_idx in range(len(face_holes)):
        cb_ind = cb_hole_indices[h_idx]
        th_ind = through_hole_indices[h_idx]
        for i in range(n_hole_pts):
            next_i = (i + 1) % n_hole_pts
            o0 = (x_step, pts_front[cb_ind[i]][0], pts_front[cb_ind[i]][1])
            o1 = (x_step, pts_front[cb_ind[next_i]][0], pts_front[cb_ind[next_i]][1])
            i0 = (x_step, pts_rear[th_ind[i]][0], pts_rear[th_ind[i]][1])
            i1 = (x_step, pts_rear[th_ind[next_i]][0], pts_rear[th_ind[next_i]][1])
            triangles.append((i0, o0, o1))
            triangles.append((i0, o1, i1))

    # Front counterbore hole walls
    for h_indices in cb_hole_indices:
        n_h = len(h_indices)
        for i in range(n_h):
            next_i = (i + 1) % n_h
            idx0, idx1 = h_indices[i], h_indices[next_i]
            b0 = (x_step, pts_front[idx0][0], pts_front[idx0][1])
            b1 = (x_step, pts_front[idx1][0], pts_front[idx1][1])
            t0 = (x_front, pts_front[idx0][0], pts_front[idx0][1])
            t1 = (x_front, pts_front[idx1][0], pts_front[idx1][1])
            triangles.append((b0, t1, b1))
            triangles.append((b0, t0, t1))

    return triangles

# ─────────────────────────────────────────────────────────────────────────────
# 3. Modular Gusset Bracket (Starts at Z = base_t)
# ─────────────────────────────────────────────────────────────────────────────
def create_single_side_gusset_bracket(leg=33.0, rib_t=3.5, flange_w=11.0, flange_t=3.5, base_t=6.0, hole_r=1.9):
    triangles = []
    x_front = 14.5
    x_rear = 14.5 - leg
    z_bot = base_t
    z_top = base_t + leg
    y_flange_start = rib_t
    y_flange_end = rib_t + flange_w
    y_hole_center = (y_flange_start + y_flange_end) / 2.0

    # 1. Bottom Side Flange
    bot_holes = [(5.5, y_hole_center), (-9.5, y_hole_center)]
    bot_flange = create_flat_plate_z(
        min_x=x_rear, max_x=x_front,
        min_y=y_flange_start, max_y=y_flange_end,
        min_z=z_bot, max_z=z_bot + flange_t,
        hole_centers=bot_holes,
        radius=hole_r,
        n_perim_divs=11
    )
    triangles.extend(bot_flange)

    # 2. Vertical Rear Side Flange
    rear_holes = [(y_hole_center, base_t + 15.0), (y_hole_center, base_t + 27.0)]
    rear_flange = create_flat_plate_x(
        min_x=x_front - flange_t, max_x=x_front,
        min_y=y_flange_start, max_y=y_flange_end,
        min_z=z_bot + flange_t, max_z=z_top,
        hole_centers=rear_holes,
        radius=hole_r,
        n_perim_divs=11
    )
    triangles.extend(rear_flange)

    # 3. 45° Triangular Central Web Rib
    p1 = (x_front, z_bot)
    p2 = (x_front, z_top)
    p3 = (x_rear, z_bot)
    y_min, y_max = 0.0, rib_t

    v = [
        (p1[0], y_min, p1[1]),
        (p2[0], y_min, p2[1]),
        (p3[0], y_min, p3[1]),
        (p1[0], y_max, p1[1]),
        (p2[0], y_max, p2[1]),
        (p3[0], y_max, p3[1]),
    ]
    rib_triangles = [
        (v[0], v[1], v[2]),
        (v[3], v[5], v[4]),
        (v[1], v[4], v[2]), (v[2], v[4], v[5]),
        (v[2], v[3], v[0]), (v[2], v[5], v[3]),
        (v[0], v[4], v[1]), (v[0], v[3], v[4]),
    ]
    triangles.extend(rib_triangles)
    return triangles

def create_flat_plate_z(min_x, max_x, min_y, max_y, min_z, max_z, hole_centers=None, radius=1.9, n_hole_pts=16, n_perim_divs=11):
    if hole_centers is None:
        hole_centers = []
    perimeter_loop = []
    for x in np.linspace(min_x, max_x, n_perim_divs)[:-1]:
        perimeter_loop.append((float(x), float(min_y)))
    for y in np.linspace(min_y, max_y, n_perim_divs)[:-1]:
        perimeter_loop.append((float(max_x), float(y)))
    for x in np.linspace(max_x, min_x, n_perim_divs)[:-1]:
        perimeter_loop.append((float(x), float(max_y)))
    for y in np.linspace(max_y, min_y, n_perim_divs)[:-1]:
        perimeter_loop.append((float(min_x), float(y)))

    pts_2d = list(perimeter_loop)
    hole_pt_indices = []
    for hx, hy in hole_centers:
        cur = []
        for i in range(n_hole_pts):
            ang = 2 * math.pi * i / n_hole_pts
            px = float(hx + radius * math.cos(ang))
            py = float(hy + radius * math.sin(ang))
            cur.append(len(pts_2d))
            pts_2d.append((px, py))
        hole_pt_indices.append(cur)

    pts_arr = np.array(pts_2d)
    delaunay = scipy.spatial.Delaunay(pts_arr)

    valid_simplices = []
    for simplex in delaunay.simplices:
        centroid = pts_arr[simplex].mean(axis=0)
        inside = any(math.hypot(centroid[0]-hx, centroid[1]-hy) < radius * 0.999 for hx, hy in hole_centers)
        if not inside:
            valid_simplices.append(simplex)

    triangles = []
    for s in valid_simplices:
        p0 = (pts_arr[s[0]][0], pts_arr[s[0]][1], max_z)
        p1 = (pts_arr[s[1]][0], pts_arr[s[1]][1], max_z)
        p2 = (pts_arr[s[2]][0], pts_arr[s[2]][1], max_z)
        cp = (p1[0]-p0[0])*(p2[1]-p0[1]) - (p1[1]-p0[1])*(p2[0]-p0[0])
        if cp > 0:
            triangles.append((p0, p1, p2))
        else:
            triangles.append((p0, p2, p1))

    for s in valid_simplices:
        p0 = (pts_arr[s[0]][0], pts_arr[s[0]][1], min_z)
        p1 = (pts_arr[s[1]][0], pts_arr[s[1]][1], min_z)
        p2 = (pts_arr[s[2]][0], pts_arr[s[2]][1], min_z)
        cp = (p1[0]-p0[0])*(p2[1]-p0[1]) - (p1[1]-p0[1])*(p2[0]-p0[0])
        if cp < 0:
            triangles.append((p0, p1, p2))
        else:
            triangles.append((p0, p2, p1))

    n_perim = len(perimeter_loop)
    for i in range(n_perim):
        next_i = (i + 1) % n_perim
        b0 = (pts_2d[i][0], pts_2d[i][1], min_z)
        b1 = (pts_2d[next_i][0], pts_2d[next_i][1], min_z)
        t0 = (pts_2d[i][0], pts_2d[i][1], max_z)
        t1 = (pts_2d[next_i][0], pts_2d[next_i][1], max_z)
        triangles.append((b0, b1, t1))
        triangles.append((b0, t1, t0))

    for h_indices in hole_pt_indices:
        n_h = len(h_indices)
        for i in range(n_h):
            next_i = (i + 1) % n_h
            idx0, idx1 = h_indices[i], h_indices[next_i]
            b0 = (pts_2d[idx0][0], pts_2d[idx0][1], min_z)
            b1 = (pts_2d[idx1][0], pts_2d[idx1][1], min_z)
            t0 = (pts_2d[idx0][0], pts_2d[idx0][1], max_z)
            t1 = (pts_2d[idx1][0], pts_2d[idx1][1], max_z)
            triangles.append((b0, t1, b1))
            triangles.append((b0, t0, t1))

    return triangles

def create_flat_plate_x(min_x, max_x, min_y, max_y, min_z, max_z, hole_centers=None, radius=1.9, n_hole_pts=16, n_perim_divs=11):
    triangles_z = create_flat_plate_z(
        min_x=min_y, max_x=max_y,
        min_y=min_z, max_y=max_z,
        min_z=min_x, max_z=max_x,
        hole_centers=hole_centers,
        radius=radius,
        n_hole_pts=n_hole_pts,
        n_perim_divs=n_perim_divs
    )
    triangles_x = []
    for tri in triangles_z:
        p0 = (tri[0][2], tri[0][0], tri[0][1])
        p1 = (tri[1][2], tri[1][0], tri[1][1])
        p2 = (tri[2][2], tri[2][0], tri[2][1])
        triangles_x.append((p0, p1, p2))
    return triangles_x

# ─────────────────────────────────────────────────────────────────────────────
# 4. Animal Mascot Parsers & Transformed Meshes
# ─────────────────────────────────────────────────────────────────────────────
def parse_obj_by_material(path):
    verts = []
    mat_faces = {}
    cur_mat = 'default'
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('v '):
                verts.append([float(x) for x in line.split()[1:4]])
            elif line.startswith('usemtl '):
                cur_mat = line.split()[1]
            elif line.startswith('f '):
                v_idx = [int(p.split('/')[0]) - 1 for p in line.split()[1:]]
                if len(v_idx) == 3:
                    mat_faces.setdefault(cur_mat, []).append(v_idx)
                elif len(v_idx) > 3:
                    for i in range(1, len(v_idx) - 1):
                        mat_faces.setdefault(cur_mat, []).append([v_idx[0], v_idx[i], v_idx[i+1]])
    return np.array(verts), mat_faces

def get_transformed_fox_data(z_base=6.0, center_x=45.0, center_y=0.0, height=42.0):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    obj_path = os.path.join(script_dir, 'assets/fox/model.obj')
    fox_v, fox_mf = parse_obj_by_material(obj_path)
    R_fox = np.array([[ 0,  0, -1], [ 1,  0,  0], [ 0,  1,  0]])
    v_rot = fox_v @ R_fox.T
    scale = height / (v_rot[:,2].max() - v_rot[:,2].min())
    v_trans = v_rot * scale
    v_trans[:,2] = v_trans[:,2] - v_trans[:,2].min() + z_base
    v_trans[:,0] = v_trans[:,0] - (v_trans[:,0].min() + v_trans[:,0].max())/2 + center_x
    v_trans[:,1] = v_trans[:,1] - (v_trans[:,1].min() + v_trans[:,1].max())/2 + center_y
    return v_trans, fox_mf

def get_transformed_rabbit_data(z_base=6.0, center_x=45.0, center_y=0.0, height=42.0):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    obj_path = os.path.join(script_dir, 'assets/rabbit/Rabbit_01(1).obj')
    rab_v, rab_mf = parse_obj_by_material(obj_path)
    R_rab = np.array([[-1,  0,  0], [ 0, -1,  0], [ 0,  0,  1]])
    v_rot = rab_v @ R_rab.T
    scale = height / (v_rot[:,2].max() - v_rot[:,2].min())
    v_trans = v_rot * scale
    v_trans[:,2] = v_trans[:,2] - v_trans[:,2].min() + z_base
    v_trans[:,0] = v_trans[:,0] - (v_trans[:,0].min() + v_trans[:,0].max())/2 + center_x
    v_trans[:,1] = v_trans[:,1] - (v_trans[:,1].min() + v_trans[:,1].max())/2 + center_y
    return v_trans, rab_mf

def create_attachment_plate(min_x=22.0, max_x=68.0, min_y=-28.0, max_y=28.0, min_z=0.0, max_z=3.5, hole_r=1.9):
    hole_centers = [(45.0, -20.0), (45.0, 20.0)]
    perim = make_rounded_rect_2d(min_x, max_x, min_y, max_y, radius=5.0)

    dense_perim = []
    n_p = len(perim)
    for i in range(n_p):
        p0 = perim[i]
        p1 = perim[(i+1)%n_p]
        d = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
        steps = max(1, int(round(d / 4.0)))
        for s in range(steps):
            t = s / steps
            dense_perim.append((p0[0] + t*(p1[0]-p0[0]), p0[1] + t*(p1[1]-p0[1])))

    n_h = 16
    pts_2d = list(dense_perim)
    hole_indices = []
    for hx, hy in hole_centers:
        cur = []
        for i in range(n_h):
            ang = 2 * math.pi * i / n_h
            cur.append(len(pts_2d))
            pts_2d.append((hx + hole_r * math.cos(ang), hy + hole_r * math.sin(ang)))
        hole_indices.append(cur)

    xs = np.arange(min_x + 3.0, max_x - 1.5, 3.0)
    ys = np.arange(min_y + 3.0, max_y - 1.5, 3.0)
    for x in xs:
        for y in ys:
            if min(math.hypot(x-hx, y-hy) for hx, hy in hole_centers) > hole_r + 2.0:
                pts_2d.append((float(x), float(y)))

    pts_arr = np.array(pts_2d)
    del_tri = scipy.spatial.Delaunay(pts_arr)
    valid = []
    for simplex in del_tri.simplices:
        centroid = pts_arr[simplex].mean(axis=0)
        if not any(math.hypot(centroid[0]-hx, centroid[1]-hy) < hole_r * 0.999 for hx, hy in hole_centers):
            valid.append(simplex)

    triangles = []
    for s in valid:
        p0 = (pts_arr[s[0]][0], pts_arr[s[0]][1], min_z)
        p1 = (pts_arr[s[1]][0], pts_arr[s[1]][1], min_z)
        p2 = (pts_arr[s[2]][0], pts_arr[s[2]][1], min_z)
        cp = (p1[0]-p0[0])*(p2[1]-p0[1]) - (p1[1]-p0[1])*(p2[0]-p0[0])
        if cp < 0:
            triangles.append((p0, p1, p2))
        else:
            triangles.append((p0, p2, p1))

    for s in valid:
        p0 = (pts_arr[s[0]][0], pts_arr[s[0]][1], max_z)
        p1 = (pts_arr[s[1]][0], pts_arr[s[1]][1], max_z)
        p2 = (pts_arr[s[2]][0], pts_arr[s[2]][1], max_z)
        cp = (p1[0]-p0[0])*(p2[1]-p0[1]) - (p1[1]-p0[1])*(p2[0]-p0[0])
        if cp > 0:
            triangles.append((p0, p1, p2))
        else:
            triangles.append((p0, p2, p1))

    n_dp = len(dense_perim)
    for i in range(n_dp):
        next_i = (i + 1) % n_dp
        b0 = (dense_perim[i][0], dense_perim[i][1], min_z)
        b1 = (dense_perim[next_i][0], dense_perim[next_i][1], min_z)
        t0 = (dense_perim[i][0], dense_perim[i][1], max_z)
        t1 = (dense_perim[next_i][0], dense_perim[next_i][1], max_z)
        triangles.append((b0, b1, t1))
        triangles.append((b0, t1, t0))

    for h_ind in hole_indices:
        for i in range(n_h):
            next_i = (i + 1) % n_h
            b0 = (pts_2d[h_ind[i]][0], pts_2d[h_ind[i]][1], min_z)
            b1 = (pts_2d[h_ind[next_i]][0], pts_2d[h_ind[next_i]][1], min_z)
            t0 = (pts_2d[h_ind[i]][0], pts_2d[h_ind[i]][1], max_z)
            t1 = (pts_2d[h_ind[next_i]][0], pts_2d[h_ind[next_i]][1], max_z)
            triangles.append((b0, t1, b1))
            triangles.append((b0, t0, t1))

    return triangles

def create_fox_stand_attachment():
    triangles = []
    # 1. Base plate with M3 mounting holes
    triangles.extend(create_attachment_plate(min_z=0.0, max_z=3.5))
    # 2. Fox on top of base plate
    v_fox, fox_mf = get_transformed_fox_data(z_base=3.5, center_x=45.0, center_y=0.0, height=42.0)
    for mat, faces in fox_mf.items():
        for f in faces:
            triangles.append((tuple(v_fox[f[0]]), tuple(v_fox[f[1]]), tuple(v_fox[f[2]])))
    return triangles

def create_rabbit_stand_attachment():
    triangles = []
    # 1. Base plate with M3 mounting holes
    triangles.extend(create_attachment_plate(min_z=0.0, max_z=3.5))
    # 2. Rabbit on top of base plate
    v_rab, rab_mf = get_transformed_rabbit_data(z_base=3.5, center_x=45.0, center_y=0.0, height=42.0)
    for mat, faces in rab_mf.items():
        for f in faces:
            triangles.append((tuple(v_rab[f[0]]), tuple(v_rab[f[1]]), tuple(v_rab[f[2]])))
    return triangles

# ─────────────────────────────────────────────────────────────────────────────
# 5. Monolithic Combined Stands (with Fox and Rabbit options)
# ─────────────────────────────────────────────────────────────────────────────
def create_monolithic_assembled_stand():
    triangles = []
    triangles.extend(create_base_plate())
    triangles.extend(create_upright_face_plate())
    gusset_raw = create_single_side_gusset_bracket()
    for tri in gusset_raw:
        p0 = (tri[0][0], -tri[0][1] - 9.0, tri[0][2])
        p1 = (tri[1][0], -tri[1][1] - 9.0, tri[1][2])
        p2 = (tri[2][0], -tri[2][1] - 9.0, tri[2][2])
        triangles.append((p0, p2, p1))
    for tri in gusset_raw:
        p0 = (tri[0][0], tri[0][1] + 9.0, tri[0][2])
        p1 = (tri[1][0], tri[1][1] + 9.0, tri[1][2])
        p2 = (tri[2][0], tri[2][1] + 9.0, tri[2][2])
        triangles.append((p0, p1, p2))
    return triangles

def create_monolithic_stand_with_fox():
    triangles = create_monolithic_assembled_stand()
    # 1. Attachment plate elevated on top of base plate (Z = 6.0 to 9.5 mm)
    triangles.extend(create_attachment_plate(min_z=6.0, max_z=9.5))
    # 2. Fox elevated on top of attachment plate (Z = 9.5 to 51.5 mm)
    v_fox, fox_mf = get_transformed_fox_data(z_base=9.5, center_x=45.0, center_y=0.0, height=42.0)
    for mat, faces in fox_mf.items():
        for f in faces:
            triangles.append((tuple(v_fox[f[0]]), tuple(v_fox[f[1]]), tuple(v_fox[f[2]])))
    return triangles

def create_monolithic_stand_with_rabbit():
    triangles = create_monolithic_assembled_stand()
    # 1. Attachment plate elevated on top of base plate (Z = 6.0 to 9.5 mm)
    triangles.extend(create_attachment_plate(min_z=6.0, max_z=9.5))
    # 2. Rabbit elevated on top of attachment plate (Z = 9.5 to 51.5 mm)
    v_rab, rab_mf = get_transformed_rabbit_data(z_base=9.5, center_x=45.0, center_y=0.0, height=42.0)
    for mat, faces in rab_mf.items():
        for f in faces:
            triangles.append((tuple(v_rab[f[0]]), tuple(v_rab[f[1]]), tuple(v_rab[f[2]])))
    return triangles

# ─────────────────────────────────────────────────────────────────────────────
# 6. MuJoCo Multi-Color Sub-Meshes
# ─────────────────────────────────────────────────────────────────────────────
def get_mujoco_animal_submeshes():
    v_fox, fox_mf = get_transformed_fox_data(z_base=6.0, center_x=45.0, center_y=0.0, height=42.0)
    fox_body_tris = [ (tuple(v_fox[f[0]]), tuple(v_fox[f[1]]), tuple(v_fox[f[2]])) for f in fox_mf.get('mat13', []) ]
    fox_white_tris = [ (tuple(v_fox[f[0]]), tuple(v_fox[f[1]]), tuple(v_fox[f[2]])) for f in fox_mf.get('mat21', []) ]
    fox_dark_tris = [ (tuple(v_fox[f[0]]), tuple(v_fox[f[1]]), tuple(v_fox[f[2]])) for f in fox_mf.get('mat20', []) ]

    v_rab, rab_mf = get_transformed_rabbit_data(z_base=6.0, center_x=45.0, center_y=0.0, height=42.0)
    rab_body_tris = [ (tuple(v_rab[f[0]]), tuple(v_rab[f[1]]), tuple(v_rab[f[2]])) for f in rab_mf.get('blinn6SG', []) ]
    rab_dark_tris = [ (tuple(v_rab[f[0]]), tuple(v_rab[f[1]]), tuple(v_rab[f[2]])) for f in rab_mf.get('blinn5SG', []) ]

    return {
        'fox_body.stl': (fox_body_tris, 'Fox Mascot Body (Orange)'),
        'fox_white.stl': (fox_white_tris, 'Fox Mascot Accent (White)'),
        'fox_dark.stl': (fox_dark_tris, 'Fox Mascot Features (Dark)'),
        'rabbit_body.stl': (rab_body_tris, 'Rabbit Mascot Body (White)'),
        'rabbit_dark.stl': (rab_dark_tris, 'Rabbit Mascot Features (Dark)'),
    }

def create_upright_structure():
    triangles = []
    triangles.extend(create_upright_face_plate())
    gusset_raw = create_single_side_gusset_bracket()
    for tri in gusset_raw:
        p0 = (tri[0][0], -tri[0][1] - 9.0, tri[0][2])
        p1 = (tri[1][0], -tri[1][1] - 9.0, tri[1][2])
        p2 = (tri[2][0], -tri[2][1] - 9.0, tri[2][2])
        triangles.append((p0, p2, p1))
    for tri in gusset_raw:
        p0 = (tri[0][0], tri[0][1] + 9.0, tri[0][2])
        p1 = (tri[1][0], tri[1][1] + 9.0, tri[1][2])
        p2 = (tri[2][0], tri[2][1] + 9.0, tri[2][2])
        triangles.append((p0, p1, p2))
    return triangles

def main():
    print("=== Generating Enhanced Stop Stand STLs with Fox & Rabbit Mascots ===")

    base_tris = create_base_plate()
    verify_watertight(base_tris, "Extended Base Plate")

    face_tris = create_upright_face_plate()
    verify_watertight(face_tris, "Counterbored Upright Face Plate")

    bracket_tris = create_single_side_gusset_bracket()
    verify_watertight(bracket_tris, "Gusset Bracket")

    upright_structure_tris = create_upright_structure()
    assembled_tris = create_monolithic_assembled_stand()
    fox_attach_tris = create_fox_stand_attachment()
    rab_attach_tris = create_rabbit_stand_attachment()
    fox_stand_tris = create_monolithic_stand_with_fox()
    rab_stand_tris = create_monolithic_stand_with_rabbit()

    dest_dirs = [
        '/Users/river/Fun/instant-robot/docs/public/models',
        '/Users/river/Fun/instant-robot/app/public/models',
        '/Users/river/Fun/instant-robot/app/public/docs/models',
        '/Users/river/Fun/instant-robot/app/public/models/so101/assets',
        '/Users/river/Fun/instant-robot/models/so101/assets',
    ]

    parts = [
        ('boundary_base_plate.stl', base_tris, 'Modular Stop Stand: Extended Base Plate (115x80x6mm)'),
        ('boundary_upright_face.stl', face_tris, 'Modular Stop Stand: Counterbored Upright Face Plate'),
        ('boundary_gusset_bracket.stl', bracket_tris, 'Modular Stop Stand: Gusset Bracket with Side Flange'),
        ('boundary_stand_upright.stl', upright_structure_tris, 'Modular Stop Stand: Upright Structure'),
        ('boundary_stop_stand.stl', assembled_tris, 'Modular Stop Stand: Assembled Full Model'),
        ('fox_stand_attachment.stl', fox_attach_tris, 'Fox Mascot Stand Attachment with M3 Screw Holes'),
        ('rabbit_stand_attachment.stl', rab_attach_tris, 'Rabbit Mascot Stand Attachment with M3 Screw Holes'),
        ('boundary_stop_stand_fox.stl', fox_stand_tris, 'Stop Stand with Fox Mascot (Monolithic)'),
        ('boundary_stop_stand_rabbit.stl', rab_stand_tris, 'Stop Stand with Rabbit Mascot (Monolithic)'),
    ]

    # Add MuJoCo multi-color sub-meshes
    sub_meshes = get_mujoco_animal_submeshes()
    for filename, (tri_list, title) in sub_meshes.items():
        parts.append((filename, tri_list, title))

    for d in dest_dirs:
        os.makedirs(d, exist_ok=True)
        for filename, tri_list, title in parts:
            filepath = os.path.join(d, filename)
            write_binary_stl(tri_list, filepath, title)

    for d in dest_dirs:
        zip_path = os.path.join(d, 'boundary_stand_stls.zip')
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
            for filename, _, _ in parts:
                fp = os.path.join(d, filename)
                if os.path.exists(fp):
                    zf.write(fp, arcname=filename)
        print(f"Updated ZIP bundle: {zip_path}")

    print("All models, mascot attachments, and ZIP bundles successfully written!")

if __name__ == '__main__':
    main()
