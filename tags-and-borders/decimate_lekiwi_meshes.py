#!/usr/bin/env python3
"""Decimate oversized LeKiwi base STLs in-place.

The exported LeKiwi wheel meshes are ~314k triangles / 15.7 MB each — far too
heavy for a decorative part and large enough that MuJoCo's mesh loader rejects
them. These are visual-only geoms, so aggressive quadric decimation is fine.
Any base mesh above THRESHOLD triangles is reduced to at most TARGET triangles.
Idempotent: meshes already under THRESHOLD are left untouched.
"""

import os
import struct

import numpy as np

ASSETS = os.path.join(os.path.dirname(__file__), "..", "app", "public", "models", "lekiwi", "assets")
THRESHOLD = 60_000  # triangles above which we decimate
GRID = 80           # vertex-clustering grid resolution across the largest bbox dim


def read_binary_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        (ntri,) = struct.unpack("<I", f.read(4))
        data = np.frombuffer(f.read(), dtype=np.uint8)
    # each triangle: 12 floats (normal + 3 verts) = 48 bytes + 2 byte attr = 50
    tris = data.reshape(ntri, 50)
    verts = tris[:, 12:48].view("<f4").reshape(ntri * 3, 3)
    return verts.astype(np.float64)


def dedup(verts):
    # verts: (ntri*3, 3) -> unique vertices + (ntri,3) face indices
    faces = np.arange(len(verts)).reshape(-1, 3)
    uniq, inv = np.unique(np.round(verts, 6), axis=0, return_inverse=True)
    return uniq, inv[faces].astype(np.int64)


def cluster_decimate(pts, faces, grid):
    """Dependency-free vertex-clustering decimation: snap vertices to a grid,
    collapse each cell to its centroid, drop faces that become degenerate."""
    lo = pts.min(axis=0)
    span = pts.max(axis=0) - lo
    cell = span.max() / grid
    if cell <= 0:
        return pts, faces
    cid = np.floor((pts - lo) / cell).astype(np.int64)
    _, cluster_of, counts = np.unique(cid, axis=0, return_inverse=True, return_counts=True)
    # centroid of each cluster
    ncl = len(counts)
    sums = np.zeros((ncl, 3))
    np.add.at(sums, cluster_of, pts)
    reps = sums / counts[:, None]
    new_faces = cluster_of[faces]
    a, b, c = new_faces[:, 0], new_faces[:, 1], new_faces[:, 2]
    keep = (a != b) & (b != c) & (a != c)
    new_faces = new_faces[keep]
    new_faces = np.unique(np.sort(new_faces, axis=1), axis=0)
    return reps, new_faces


def write_binary_stl(path, points, faces):
    tri = points[faces]  # (F,3,3)
    n = len(faces)
    out = bytearray(80)
    out += struct.pack("<I", n)
    v1, v2, v3 = tri[:, 0], tri[:, 1], tri[:, 2]
    nrm = np.cross(v2 - v1, v3 - v1)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 0)
    body = np.zeros((n, 50), dtype=np.uint8)
    fbuf = np.concatenate([nrm, v1, v2, v3], axis=1).astype("<f4")
    body[:, :48] = fbuf.view(np.uint8).reshape(n, 48)
    out += body.tobytes()
    with open(path, "wb") as f:
        f.write(out)


def main():
    for fn in sorted(os.listdir(ASSETS)):
        if not fn.lower().endswith(".stl"):
            continue
        path = os.path.join(ASSETS, fn)
        with open(path, "rb") as f:
            f.seek(80)
            (ntri,) = struct.unpack("<I", f.read(4))
        if ntri <= THRESHOLD:
            continue
        verts = read_binary_stl(path)
        pts, faces = dedup(verts)
        pts2, faces2 = cluster_decimate(pts, faces, GRID)
        write_binary_stl(path, pts2.astype(np.float64), faces2.astype(np.int64))
        print(f"{fn}: {ntri} -> {len(faces2)} tris, {os.path.getsize(path)//1024} KB")
    print("done")


if __name__ == "__main__":
    main()
