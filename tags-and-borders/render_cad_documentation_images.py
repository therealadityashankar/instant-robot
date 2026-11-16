import struct
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os

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

def plot_solid(ax, triangles, color, edgecolor='#0f172a', alpha=0.9):
    mesh = Poly3DCollection(triangles, alpha=alpha, edgecolor=edgecolor, linewidths=0.3)
    mesh.set_facecolor(color)
    ax.add_collection3d(mesh)

def render_all_views():
    os.makedirs('/Users/river/Fun/instant-robot/docs/public/images', exist_ok=True)
    os.makedirs('/Users/river/.gemini/antigravity-ide/brain/a3f0fb2b-2953-4610-8e0f-da789809cae9', exist_ok=True)

    base_path = '/Users/river/Fun/instant-robot/docs/public/models/boundary_base_plate.stl'
    face_path = '/Users/river/Fun/instant-robot/docs/public/models/boundary_upright_face.stl'
    gusset_path = '/Users/river/Fun/instant-robot/docs/public/models/boundary_gusset_bracket.stl'
    stand_fox_path = '/Users/river/Fun/instant-robot/docs/public/models/boundary_stop_stand_fox.stl'
    stand_rab_path = '/Users/river/Fun/instant-robot/docs/public/models/boundary_stop_stand_rabbit.stl'
    fox_attach_path = '/Users/river/Fun/instant-robot/docs/public/models/fox_stand_attachment.stl'
    rab_attach_path = '/Users/river/Fun/instant-robot/docs/public/models/rabbit_stand_attachment.stl'

    base_tris = read_stl(base_path)
    face_tris = read_stl(face_path)
    gusset_tris = read_stl(gusset_path)
    stand_fox_tris = read_stl(stand_fox_path)
    stand_rab_tris = read_stl(stand_rab_path)
    fox_attach_tris = read_stl(fox_attach_path)
    rab_attach_tris = read_stl(rab_attach_path)

    # ── Figure 1: 4-Panel Overview of Modular Parts & Mascots ──
    fig = plt.figure(figsize=(16, 12), facecolor='#0f172a')
    
    parts_info = [
        (base_tris, '1. Extended Base Plate (115x80x6mm, M3 Hole Grid)', '#38bdf8', (-45, 80, -45, 45, 0, 12), (30, -50)),
        (face_tris, '2. Upright Face (52x95mm, Front Counterbores)', '#ec4899', (10, 25, -30, 30, 0, 105), (25, -60)),
        (fox_attach_tris, '3. Fox Stand Attachment (M3 Screw Holes)', '#f97316', (20, 70, -30, 30, 0, 50), (25, -45)),
        (rab_attach_tris, '4. Rabbit Stand Attachment (M3 Screw Holes)', '#e2e8f0', (20, 70, -30, 30, 0, 50), (25, -45)),
    ]

    for i, (tris, title, color, limits, view_angle) in enumerate(parts_info):
        ax = fig.add_subplot(2, 2, i + 1, projection='3d')
        plot_solid(ax, tris, color)
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
    out1 = '/Users/river/Fun/instant-robot/docs/public/images/modular_stand_parts_render.png'
    out1_brain = '/Users/river/.gemini/antigravity-ide/brain/a3f0fb2b-2953-4610-8e0f-da789809cae9/modular_stand_parts_render.png'
    plt.savefig(out1, dpi=200, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.savefig(out1_brain, dpi=200, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.close()
    print(f"Saved {out1}")

    # ── Figure 2: Combined Stands with Fox & Rabbit Mascots ──
    fig2 = plt.figure(figsize=(16, 8), facecolor='#0f172a')
    
    # Panel 1: Stop Stand with Fox Mascot
    ax1 = fig2.add_subplot(1, 2, 1, projection='3d')
    plot_solid(ax1, stand_fox_tris, '#38bdf8', alpha=0.95)
    ax1.set_xlim(-45, 80)
    ax1.set_ylim(-45, 45)
    ax1.set_zlim(0, 105)
    ax1.set_title('Stop Stand with Fox Mascot (Z >= 6mm, Zero Overlap)', fontsize=12, fontweight='bold', color='#ffffff', pad=12)
    ax1.set_xlabel('X (mm)', fontsize=9, color='#94a3b8')
    ax1.set_ylabel('Y (mm)', fontsize=9, color='#94a3b8')
    ax1.set_zlabel('Z (mm)', fontsize=9, color='#94a3b8')
    ax1.tick_params(colors='#94a3b8', labelsize=8)
    ax1.set_facecolor('#1e293b')
    ax1.view_init(elev=28, azim=-55)

    # Panel 2: Stop Stand with Rabbit Mascot
    ax2 = fig2.add_subplot(1, 2, 2, projection='3d')
    plot_solid(ax2, stand_rab_tris, '#e2e8f0', alpha=0.95)
    ax2.set_xlim(-45, 80)
    ax2.set_ylim(-45, 45)
    ax2.set_zlim(0, 105)
    ax2.set_title('Stop Stand with Rabbit Mascot (Z >= 6mm, Zero Overlap)', fontsize=12, fontweight='bold', color='#ffffff', pad=12)
    ax2.set_xlabel('X (mm)', fontsize=9, color='#94a3b8')
    ax2.set_ylabel('Y (mm)', fontsize=9, color='#94a3b8')
    ax2.set_zlabel('Z (mm)', fontsize=9, color='#94a3b8')
    ax2.tick_params(colors='#94a3b8', labelsize=8)
    ax2.set_facecolor('#1e293b')
    ax2.view_init(elev=28, azim=-55)

    plt.tight_layout()
    out2 = '/Users/river/Fun/instant-robot/docs/public/images/modular_stand_combined_alignment_render.png'
    out2_brain = '/Users/river/.gemini/antigravity-ide/brain/a3f0fb2b-2953-4610-8e0f-da789809cae9/modular_stand_combined_alignment_render.png'
    plt.savefig(out2, dpi=200, facecolor=fig2.get_facecolor(), edgecolor='none')
    plt.savefig(out2_brain, dpi=200, facecolor=fig2.get_facecolor(), edgecolor='none')
    plt.close()
    print(f"Saved {out2}")

if __name__ == '__main__':
    render_all_views()
