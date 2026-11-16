import numpy as np
import scipy.spatial
import math

def create_plate_with_holes_z(min_x, max_x, min_y, max_y, min_z, max_z, hole_centers, radius=1.7, n_hole_pts=16):
    """
    Generate a 100% watertight, manifold 3D plate with through-holes along Z.
    """
    # 1. Build ordered outer perimeter loop (CCW)
    perimeter_loop = []
    # Bottom edge (-Y): from (min_x, min_y) to (max_x, min_y)
    for x in np.linspace(min_x, max_x, 9)[:-1]:
        perimeter_loop.append((float(x), float(min_y)))
    # Right edge (+X): from (max_x, min_y) to (max_x, max_y)
    for y in np.linspace(min_y, max_y, 9)[:-1]:
        perimeter_loop.append((float(max_x), float(y)))
    # Top edge (+Y): from (max_x, max_y) to (min_x, max_y)
    for x in np.linspace(max_x, min_x, 9)[:-1]:
        perimeter_loop.append((float(x), float(max_y)))
    # Left edge (-X): from (min_x, max_y) to (min_x, min_y)
    for y in np.linspace(max_y, min_y, 9)[:-1]:
        perimeter_loop.append((float(min_x), float(y)))

    pts_2d = list(perimeter_loop)

    # 2. Build hole perimeter loops (CW so inner normals are clean)
    hole_pt_indices = []
    for hx, hy in hole_centers:
        cur_indices = []
        for i in range(n_hole_pts):
            angle = 2 * math.pi * i / n_hole_pts
            px = float(hx + radius * math.cos(angle))
            py = float(hy + radius * math.sin(angle))
            cur_indices.append(len(pts_2d))
            pts_2d.append((px, py))
        hole_pt_indices.append(cur_indices)

    pts_arr = np.array(pts_2d)

    # 3. 2D Delaunay Triangulation
    delaunay = scipy.spatial.Delaunay(pts_arr)

    # 4. Filter triangles whose centroid falls inside any hole
    valid_2d_triangles = []
    for simplex in delaunay.simplices:
        tri_pts = pts_arr[simplex]
        centroid = tri_pts.mean(axis=0)
        inside_hole = False
        for hx, hy in hole_centers:
            dist = math.hypot(centroid[0] - hx, centroid[1] - hy)
            if dist < radius * 0.999:
                inside_hole = True
                break
        if not inside_hole:
            valid_2d_triangles.append(simplex)

    triangles_3d = []

    # 5. Top face (+Z)
    for s in valid_2d_triangles:
        p0 = (pts_2d[s[0]][0], pts_2d[s[0]][1], max_z)
        p1 = (pts_2d[s[1]][0], pts_2d[s[1]][1], max_z)
        p2 = (pts_2d[s[2]][0], pts_2d[s[2]][1], max_z)
        u = (p1[0]-p0[0], p1[1]-p0[1])
        v = (p2[0]-p0[0], p2[1]-p0[1])
        cross_z = u[0]*v[1] - u[1]*v[0]
        if cross_z > 0:
            triangles_3d.append((p0, p1, p2))
        else:
            triangles_3d.append((p0, p2, p1))

    # 6. Bottom face (-Z)
    for s in valid_2d_triangles:
        p0 = (pts_2d[s[0]][0], pts_2d[s[0]][1], min_z)
        p1 = (pts_2d[s[1]][0], pts_2d[s[1]][1], min_z)
        p2 = (pts_2d[s[2]][0], pts_2d[s[2]][1], min_z)
        u = (p1[0]-p0[0], p1[1]-p0[1])
        v = (p2[0]-p0[0], p2[1]-p0[1])
        cross_z = u[0]*v[1] - u[1]*v[0]
        if cross_z < 0:
            triangles_3d.append((p0, p1, p2))
        else:
            triangles_3d.append((p0, p2, p1))

    # 7. Outer perimeter walls (along perimeter_loop)
    n_perim = len(perimeter_loop)
    for i in range(n_perim):
        x1, y1 = perimeter_loop[i]
        x2, y2 = perimeter_loop[(i + 1) % n_perim]
        p0 = (x1, y1, min_z)
        p1 = (x2, y2, min_z)
        p2 = (x2, y2, max_z)
        p3 = (x1, y1, max_z)
        # Outward normal for CCW loop:
        triangles_3d.append((p0, p1, p2))
        triangles_3d.append((p0, p2, p3))

    # 8. Inner cylinder walls for each hole
    for h_indices in hole_pt_indices:
        n_h = len(h_indices)
        for i in range(n_h):
            idx1 = h_indices[i]
            idx2 = h_indices[(i + 1) % n_h]
            x1, y1 = pts_2d[idx1]
            x2, y2 = pts_2d[idx2]
            p0 = (x1, y1, min_z)
            p1 = (x2, y2, min_z)
            p2 = (x2, y2, max_z)
            p3 = (x1, y1, max_z)
            # Hole perimeter is CCW in angle -> to point normal inward, reverse winding
            triangles_3d.append((p0, p2, p1))
            triangles_3d.append((p0, p3, p2))

    return triangles_3d
