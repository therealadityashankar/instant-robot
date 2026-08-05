#!/usr/bin/env python3
"""
Convert the LeKiwi URDF mobile-base into a MuJoCo-ready "base geometry" JSON
that can be injected under our own SO-101 arm model.

Pipeline:
  1. Download the LeKiwi URDF (and referenced base meshes) if missing.
  2. Parse the URDF link/joint tree, compose world transforms from the root.
  3. Exclude the arm sub-chain (everything descending from `Base_08q-v1`, i.e.
     WaveShare_Mounting_Plate_01d-v1 + the SO-101 arm links). Keep the base.
  4. Re-express every kept visual in the arm-mount frame
     (WaveShare_Mounting_Plate_01d-v1) and emit base.json.

Uses only the Python stdlib plus numpy. Idempotent / re-runnable.
"""

import json
import math
import os
import sys
import urllib.request
import xml.etree.ElementTree as ET

try:
    import numpy as np
except ImportError:  # pragma: no cover - numpy expected in this env
    np = None

# --------------------------------------------------------------------------
# Paths / constants
# --------------------------------------------------------------------------
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
RAW_BASE = "https://raw.githubusercontent.com/SIGRobotics-UIUC/LeKiwi/HEAD/URDF"
URDF_URL = RAW_BASE + "/LeKiwi.urdf"
MESH_URL_PREFIX = RAW_BASE + "/meshes/"

OUT_DIR = os.path.join(REPO_ROOT, "app", "public", "models", "lekiwi")
ASSET_DIR = os.path.join(OUT_DIR, "assets")
URDF_LOCAL = os.path.join(OUT_DIR, "LeKiwi.urdf")
OUT_JSON = os.path.join(OUT_DIR, "base.json")

MOUNT_FRAME = "WaveShare_Mounting_Plate_01d-v1"
# Root of the arm sub-chain to exclude. Every link descending from this link
# (its whole subtree, but NOT the link itself) is part of the SO-101 arm.
ARM_SUBTREE_ROOT = "Base_08q-v1"


# --------------------------------------------------------------------------
# 4x4 homogeneous transform helpers (numpy)
# --------------------------------------------------------------------------
def rpy_to_matrix(roll, pitch, yaw):
    """URDF fixed-axis XYZ (rpy) -> 3x3 rotation matrix: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def make_transform(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def matrix_to_quat(R):
    """3x3 rotation -> quaternion (w, x, y, z), normalized, scalar-positive."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    q /= np.linalg.norm(q)
    if q[0] < 0:  # scalar-positive convention
        q = -q
    return q


def rnd(v, sig=8):
    """Round a float to ~sig significant digits, normalizing -0.0 to 0.0."""
    v = float(v)
    if v == 0.0 or not math.isfinite(v):
        return 0.0
    d = sig - int(math.floor(math.log10(abs(v)))) - 1
    r = round(v, d)
    return 0.0 if r == 0.0 else r


def safe_name(s):
    out = []
    for ch in s:
        out.append(ch if (ch.isalnum() or ch == "_") else "_")
    name = "".join(out)
    if name and name[0].isdigit():
        name = "m_" + name
    return name


# --------------------------------------------------------------------------
# Download helpers
# --------------------------------------------------------------------------
def download(url, dest):
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    with urllib.request.urlopen(url) as r:
        data = r.read()
    with open(dest, "wb") as f:
        f.write(data)
    return data


def is_valid_stl(path):
    if not os.path.exists(path) or os.path.getsize(path) < 84:
        return False
    with open(path, "rb") as f:
        head = f.read(6)
    if head[:5].lower() == b"solid":  # ASCII STL
        return True
    # Binary STL: 80-byte header + uint32 triangle count; validate size.
    with open(path, "rb") as f:
        f.seek(80)
        import struct
        (ntri,) = struct.unpack("<I", f.read(4))
    expected = 84 + ntri * 50
    return os.path.getsize(path) == expected


def read_stl_vertices(path):
    """Return all triangle vertices of a binary STL as an (N*3, 3) array."""
    import struct

    with open(path, "rb") as f:
        f.read(80)
        (ntri,) = struct.unpack("<I", f.read(4))
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape(ntri, 50)
    return data[:, 12:48].view("<f4").reshape(ntri * 3, 3).astype(np.float64)


def ensure_urdf():
    if not os.path.exists(URDF_LOCAL):
        print(f"Downloading URDF -> {URDF_LOCAL}")
        download(URDF_URL, URDF_LOCAL)
    return URDF_LOCAL


# --------------------------------------------------------------------------
# URDF parsing
# --------------------------------------------------------------------------
def parse_vec(s, n=3):
    parts = [float(x) for x in s.split()]
    return parts[:n] if len(parts) >= n else parts + [0.0] * (n - len(parts))


def origin_of(elem):
    o = elem.find("origin") if elem is not None else None
    if o is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    xyz = parse_vec(o.get("xyz", "0 0 0"))
    rpy = parse_vec(o.get("rpy", "0 0 0"))
    return xyz, rpy


def main():
    ensure_urdf()
    root = ET.parse(URDF_LOCAL).getroot()

    links = {}  # name -> link element
    for l in root.findall("link"):
        links[l.get("name")] = l

    # joint tree
    children = {}  # parent -> list of (child, T_joint)
    parent_of = {}  # child -> parent
    joint_T = {}  # child -> T_joint
    for j in root.findall("joint"):
        p = j.find("parent").get("link")
        c = j.find("child").get("link")
        xyz, rpy = origin_of(j)
        T = make_transform(xyz, rpy)
        children.setdefault(p, []).append(c)
        parent_of[c] = p
        joint_T[c] = T

    # find root link (a link that is never a child)
    root_links = [n for n in links if n not in parent_of]
    assert len(root_links) == 1, f"expected 1 root, got {root_links}"
    root_link = root_links[0]

    # world transforms via BFS from root
    T_world = {root_link: np.eye(4)}
    stack = [root_link]
    while stack:
        cur = stack.pop()
        for ch in children.get(cur, []):
            T_world[ch] = T_world[cur] @ joint_T[ch]
            stack.append(ch)

    # arm subtree = ARM_SUBTREE_ROOT and everything below it. The root
    # (Base_08q-v1) is the arm-mount bracket, which our SO-101 arm model already
    # includes — keeping it here produces a visible "double base", so exclude it.
    arm_links = {ARM_SUBTREE_ROOT}
    stack = list(children.get(ARM_SUBTREE_ROOT, []))
    while stack:
        n = stack.pop()
        arm_links.add(n)
        stack.extend(children.get(n, []))

    kept = [n for n in links if n not in arm_links]
    # keep in URDF document order
    kept = [l.get("name") for l in root.findall("link") if l.get("name") in kept]

    T_mount = T_world[MOUNT_FRAME]
    T_mount_inv = np.linalg.inv(T_mount)

    meshes = []  # deduped by (file, scale)
    mesh_key_to_name = {}
    geoms = []
    geom_xforms = []  # (fname, scale, T) for the exact floor computation
    referenced_files = {}  # file -> url

    for name in kept:
        link = links[name]
        for vis in link.findall("visual"):
            geom = vis.find("geometry")
            mesh = geom.find("mesh") if geom is not None else None
            if mesh is None:
                continue
            fname = os.path.basename(mesh.get("filename"))
            scale = parse_vec(mesh.get("scale", "1 1 1"))
            scale = [rnd(s) for s in scale]

            vxyz, vrpy = origin_of(vis)
            T_vis = make_transform(vxyz, vrpy)
            T = T_mount_inv @ T_world[name] @ T_vis
            pos = [rnd(v) for v in T[:3, 3]]
            quat = [rnd(v) for v in matrix_to_quat(T[:3, :3])]

            key = (fname, tuple(scale))
            if key not in mesh_key_to_name:
                base = safe_name(os.path.splitext(fname)[0])
                mname = base
                i = 1
                existing = set(mesh_key_to_name.values())
                while mname in existing:
                    i += 1
                    mname = f"{base}_{i}"
                mesh_key_to_name[key] = mname
                meshes.append({"name": mname, "file": fname, "scale": list(scale)})
                referenced_files[fname] = MESH_URL_PREFIX + mesh.get("filename").split("meshes/")[-1]

            geoms.append({"mesh": mesh_key_to_name[key], "pos": pos, "quat": quat})
            geom_xforms.append((fname, scale, T))

    # download / verify meshes
    downloaded = 0
    for fname, url in referenced_files.items():
        dest = os.path.join(ASSET_DIR, fname)
        if not is_valid_stl(dest):
            print(f"Downloading mesh {fname}")
            download(url, dest)
            if not is_valid_stl(dest):
                raise RuntimeError(f"Downloaded mesh failed STL validation: {fname}")
        downloaded += 1

    # Exact floor: the base hangs from the mount in +Z (mount frame). The lowest
    # world point after the app's 180°-about-X flip is -max(vertex Z). Lifting
    # the arm by that max (baseDrop) sits the wheels exactly on the grid — using
    # vertex extents, not mesh origins, so round wheels don't sink below z=0.
    base_drop = 0.0
    for fname, scale, T in geom_xforms:
        verts = read_stl_vertices(os.path.join(ASSET_DIR, fname))  # (N,3)
        verts = verts * np.asarray(scale)
        world = (T[:3, :3] @ verts.T).T + T[:3, 3]
        base_drop = max(base_drop, float(world[:, 2].max()))

    out = {
        "mountFrame": MOUNT_FRAME,
        "baseDrop": rnd(base_drop),
        "meshes": meshes,
        "geoms": geoms,
    }
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(OUT_JSON, "w") as f:
        json.dump(out, f, indent=2)

    zs = [g["pos"][2] for g in geoms]
    total_bytes = sum(
        os.path.getsize(os.path.join(ASSET_DIR, f)) for f in referenced_files
    )
    print("---- LeKiwi base conversion ----")
    print(f"base links kept:     {len(kept)}")
    print(f"unique meshes:       {len(meshes)}")
    print(f"meshes downloaded:   {downloaded}")
    print(f"geoms:               {len(geoms)}")
    print(f"geom Z min/max:      {min(zs):.6f} / {max(zs):.6f}")
    print(f"total assets:        {len(referenced_files)} files, {total_bytes/1e6:.2f} MB")
    print(f"base.json:           {OUT_JSON}")
    print("kept links:")
    for n in kept:
        print("  -", n)


if __name__ == "__main__":
    if np is None:
        sys.exit("numpy required")
    main()
