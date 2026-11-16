import struct
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import os
import math

def read_stl(filepath):
    triangles = []
    with open(filepath, 'rb') as f:
        f.read(80) # header
        count = struct.unpack('<I', f.read(4))[0]
        for _ in range(count):
            f.read(12) # normal
            v1 = struct.unpack('<3f', f.read(12))
            v2 = struct.unpack('<3f', f.read(12))
            v3 = struct.unpack('<3f', f.read(12))
            f.read(2) # attr
            triangles.append([v1, v2, v3])
    return triangles

def plot_mesh(ax, triangles, color, alpha=0.85, edgecolor='#0f172a', linewidth=0.3):
    mesh = Poly3DCollection(triangles, alpha=alpha, edgecolor=edgecolor, linewidths=linewidth)
    mesh.set_facecolor(color)
    ax.add_collection3d(mesh)

def add_cylinder(ax, p1, p2, radius=1.65, color='#fbbf24', alpha=0.9, n_segs=12):
    """Draw a 3D cylinder representing an M3 bolt shaft."""
    v = np.array(p2) - np.array(p1)
    length = np.linalg.norm(v)
    if length == 0:
        return
    v_norm = v / length
    # Orthogonal vectors
    not_v = np.array([1, 0, 0]) if abs(v_norm[0]) < 0.8 else np.array([0, 1, 0])
    n1 = np.cross(v_norm, not_v)
    n1 /= np.linalg.norm(n1)
    n2 = np.cross(v_norm, n1)

    theta = np.linspace(0, 2*np.pi, n_segs)
    pts_bot = [np.array(p1) + radius*np.cos(t)*n1 + radius*np.sin(t)*n2 for t in theta]
    pts_top = [np.array(p2) + radius*np.cos(t)*n1 + radius*np.sin(t)*n2 for t in theta]

    tris = []
    for i in range(n_segs - 1):
        # Wall
        tris.append([pts_bot[i], pts_bot[i+1], pts_top[i+1]])
        tris.append([pts_bot[i], pts_top[i+1], pts_top[i]])
        # Caps
        tris.append([p1, pts_bot[i+1], pts_bot[i]])
        tris.append([p2, pts_top[i], pts_top[i+1]])

    mesh = Poly3DCollection(tris, alpha=alpha, edgecolor='#b45309', linewidths=0.2)
    mesh.set_facecolor(color)
    ax.add_collection3d(mesh)

def main():
    base_stl = '/Users/river/Fun/instant-robot/docs/public/models/boundary_base_plate.stl'
    face_stl = '/Users/river/Fun/instant-robot/docs/public/models/boundary_upright_face.stl'
    gusset_stl = '/Users/river/Fun/instant-robot/docs/public/models/boundary_gusset_bracket.stl'

    base_tris = read_stl(base_stl)
    face_tris = read_stl(face_stl)
    gusset_raw_tris = read_stl(gusset_stl)

    # Place Left Gusset: mirror Y and offset so flange center is at Y = -18.0mm
    gusset_left = []
    for tri in gusset_raw_tris:
        p0 = (tri[0][0], -tri[0][1] - 9.0, tri[0][2])
        p1 = (tri[1][0], -tri[1][1] - 9.0, tri[1][2])
        p2 = (tri[2][0], -tri[2][1] - 9.0, tri[2][2])
        gusset_left.append((p0, p2, p1))

    # Place Right Gusset: shift Y by +9.0mm so flange center is at Y = +18.0mm
    gusset_right = []
    for tri in gusset_raw_tris:
        p0 = (tri[0][0], tri[0][1] + 9.0, tri[0][2])
        p1 = (tri[1][0], tri[1][1] + 9.0, tri[1][2])
        p2 = (tri[2][0], tri[2][1] + 9.0, tri[2][2])
        gusset_right.append((p0, p1, p2))

    # M3 Bolt Shafts (demonstrating 100% concentric alignment):
    bolts = [
        # 2x Left Gusset Base Bolts (through base Z=0 to bracket flange Z=7.5)
        ((5.5, -18.0, -2.0), (5.5, -18.0, 9.5)),
        ((-9.5, -18.0, -2.0), (-9.5, -18.0, 9.5)),
        # 2x Right Gusset Base Bolts (through base Z=0 to bracket flange Z=7.5)
        ((5.5, 18.0, -2.0),  (5.5, 18.0, 9.5)),
        ((-9.5, 18.0, -2.0),  (-9.5, 18.0, 9.5)),
        # 2x Left Gusset Upright Bolts (through face X=18.0 to bracket flange X=11.0)
        ((20.0, -18.0, 19.0), (9.0, -18.0, 19.0)),
        ((20.0, -18.0, 31.0), (9.0, -18.0, 31.0)),
        # 2x Right Gusset Upright Bolts (through face X=18.0 to bracket flange X=11.0)
        ((20.0, 18.0, 19.0),  (9.0, 18.0, 19.0)),
        ((20.0, 18.0, 31.0),  (9.0, 18.0, 31.0)),
    ]

    # Create 4-view CAD inspection figure
    fig = plt.figure(figsize=(18, 14), facecolor='#0f172a')

    views = [
        ('1. Combined Perspective View (Full Assembly + M3 Bolts)', (28, -55), (-45, 45, -45, 45, -5, 105)),
        ('2. Top-Down Alignment (Z-Axis: Concentric Base & Flange Holes)', (90, -90), (-45, 45, -45, 45, -5, 105)),
        ('3. Side Profile (Y-Axis: Flush Mating Faces)', (0, -90), (-45, 45, -45, 45, -5, 105)),
        ('4. Rear Inspection View (X-Axis: Vertical Flange M3 Hole Alignment)', (20, 135), (-45, 45, -45, 45, -5, 105)),
    ]

    for i, (title, (elev, azim), limits) in enumerate(views):
        ax = fig.add_subplot(2, 2, i + 1, projection='3d')
        
        # Base Plate (Cyan)
        plot_mesh(ax, base_tris, '#38bdf8', alpha=0.75)
        # Upright Face Plate (Magenta)
        plot_mesh(ax, face_tris, '#ec4899', alpha=0.8)
        # Left Gusset Bracket (Orange)
        plot_mesh(ax, gusset_left, '#f59e0b', alpha=0.85)
        # Right Gusset Bracket (Amber)
        plot_mesh(ax, gusset_right, '#f59e0b', alpha=0.85)

        # M3 Bolt Cylinders (Gold)
        for p1, p2 in bolts:
            add_cylinder(ax, p1, p2, radius=1.65, color='#eab308', alpha=0.95)

        ax.set_xlim(limits[0], limits[1])
        ax.set_ylim(limits[2], limits[3])
        ax.set_zlim(limits[4], limits[5])
        ax.set_title(title, fontsize=12, fontweight='bold', color='#ffffff', pad=12)
        ax.set_xlabel('X (mm)', fontsize=9, color='#94a3b8')
        ax.set_ylabel('Y (mm)', fontsize=9, color='#94a3b8')
        ax.set_zlabel('Z (mm)', fontsize=9, color='#94a3b8')
        ax.tick_params(colors='#94a3b8', labelsize=8)
        ax.set_facecolor('#1e293b')
        ax.view_init(elev=elev, azim=azim)

    plt.tight_layout()

    out_img = '/Users/river/Fun/instant-robot/docs/public/images/modular_stand_combined_alignment_render.png'
    out_brain = '/Users/river/.gemini/antigravity-ide/brain/a3f0fb2b-2953-4610-8e0f-da789809cae9/modular_stand_combined_alignment_render.png'
    os.makedirs(os.path.dirname(out_img), exist_ok=True)
    plt.savefig(out_img, dpi=200, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.savefig(out_brain, dpi=200, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.close()

    # Also update modular parts overview render with the clean upright plate
    fig_parts = plt.figure(figsize=(16, 12), facecolor='#0f172a')
    parts_info = [
        (base_tris, '1. Base Plate (80x80x4mm, 4x Centered M3 Holes)', '#38bdf8', (-45, 45, -45, 45, 0, 10), (30, -50)),
        (face_tris, '2. Clean Upright Face Plate (52x95mm, 4x M3 Holes)', '#ec4899', (0, 30, -30, 30, 0, 105), (25, -60)),
        (gusset_raw_tris, '3. Modular Gusset Bracket (Single Side-Flange with Full M3 Holes)', '#f59e0b', (-25, 20, -5, 20, 0, 45), (25, -45)),
        (read_stl('/Users/river/Fun/instant-robot/docs/public/models/boundary_stop_stand.stl'), '4. Full Assembled Combined Stand (All Holes & Centered)', '#10b981', (-45, 45, -45, 45, 0, 105), (30, -50)),
    ]
    for i, (tris, title, color, limits, view_angle) in enumerate(parts_info):
        ax = fig_parts.add_subplot(2, 2, i + 1, projection='3d')
        plot_mesh(ax, tris, color, alpha=0.9)
        ax.set_xlim(limits[0], limits[1])
        ax.set_ylim(limits[2], limits[3])
        ax.set_zlim(limits[4], limits[5])
        ax.set_title(title, fontsize=12, fontweight='bold', color='#ffffff', pad=12)
        ax.set_xlabel('X (mm)', fontsize=9, color='#94a3b8')
        ax.set_ylabel('Y (mm)', fontsize=9, color='#94a3b8')
        ax.set_zlabel('Z (mm)', fontsize=9, color='#94a3b8')
        ax.tick_params(colors='#94a3b8', labelsize=8)
        ax.set_facecolor('#1e293b')
        ax.view_init(elev=view_angle[0], azim=view_angle[1])

    plt.tight_layout()
    out_parts = '/Users/river/Fun/instant-robot/docs/public/images/modular_stand_parts_render.png'
    out_parts_brain = '/Users/river/.gemini/antigravity-ide/brain/a3f0fb2b-2953-4610-8e0f-da789809cae9/modular_stand_parts_render.png'
    plt.savefig(out_parts, dpi=200, facecolor=fig_parts.get_facecolor(), edgecolor='none')
    plt.savefig(out_parts_brain, dpi=200, facecolor=fig_parts.get_facecolor(), edgecolor='none')
    plt.close()

    print(f"Saved combined alignment render to {out_img}")
    print(f"Updated parts render in {out_parts}")

if __name__ == '__main__':
    main()
