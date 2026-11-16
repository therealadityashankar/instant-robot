import numpy as np
import scipy.spatial
import math

def verify_mesh(triangles):
    """
    Check if a triangle mesh is 100% 2-manifold and watertight:
    Every directed edge (u, v) must have exactly one opposite directed edge (v, u).
    """
    edge_counts = {}
    for tri in triangles:
        p0, p1, p2 = tri
        # Quantize vertices to avoid float precision mismatch
        v0 = tuple(round(c, 4) for c in p0)
        v1 = tuple(round(c, 4) for c in p1)
        v2 = tuple(round(c, 4) for c in p2)
        edges = [(v0, v1), (v1, v2), (v2, v0)]
        for e in edges:
            edge_counts[e] = edge_counts.get(e, 0) + 1

    boundary_edges = 0
    non_manifold_edges = 0
    for (u, v), count in edge_counts.items():
        opp_count = edge_counts.get((v, u), 0)
        if count != 1 or opp_count != 1:
            boundary_edges += 1
            if count > 1:
                non_manifold_edges += 1

    print(f"Total triangles: {len(triangles)}, Total directed edges: {len(edge_counts)}")
    print(f"Boundary/mismatched edges: {boundary_edges}, Non-manifold edges: {non_manifold_edges}")
    return boundary_edges == 0

from test_triangulation import create_plate_with_holes_z
tris = create_plate_with_holes_z(-40, 40, -40, 40, 0, 4, [(-8.75, 0), (-20, -17), (-20, 17)])
is_valid = verify_mesh(tris)
print("Mesh is 100% watertight & 2-manifold:", is_valid)
