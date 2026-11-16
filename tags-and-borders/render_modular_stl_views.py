import struct
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import os

def read_stl(filepath):
    triangles = []
    with open(filepath, 'rb') as f:
        header = f.read(80)
        count = struct.unpack('<I', f.read(4))[0]
        for _ in range(count):
            norm = struct.unpack('<3f', f.read(12))
            v1 = struct.unpack('<3f', f.read(12))
            v2 = struct.unpack('<3f', f.read(12))
            v3 = struct.unpack('<3f', f.read(12))
            attr = struct.unpack('<H', f.read(2))
            triangles.append([v1, v2, v3])
    return triangles

def plot_part(ax, triangles, title, color='#38bdf8'):
    mesh = Poly3DCollection(triangles, alpha=0.85, edgecolor='#0284c7', linewidths=0.5)
    mesh.set_facecolor(color)
    ax.add_collection3d(mesh)
    
    # Auto scale limits
    all_pts = [pt for tri in triangles for pt in tri]
    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    zs = [p[2] for p in all_pts]
    
    ax.set_xlim(min(xs) - 5, max(xs) + 5)
    ax.set_ylim(min(ys) - 5, max(ys) + 5)
    ax.set_zlim(min(zs) - 5, max(zs) + 5)
    
    ax.set_title(title, fontsize=11, fontweight='bold', color='#ffffff', pad=10)
    ax.set_xlabel('X (mm)', fontsize=9, color='#cbd5e1')
    ax.set_ylabel('Y (mm)', fontsize=9, color='#cbd5e1')
    ax.set_zlabel('Z (mm)', fontsize=9, color='#cbd5e1')
    ax.tick_params(colors='#94a3b8', labelsize=8)
    ax.set_facecolor('#1e293b')

def main():
    parts = [
        ('/Users/river/Fun/instant-robot/docs/public/models/boundary_base_plate.stl', '1. Base Plate (with M3 Hole Grid)', '#38bdf8', (30, -50)),
        ('/Users/river/Fun/instant-robot/docs/public/models/boundary_upright_face.stl', '2. Upright Face (with Bottom Foot & Holes)', '#ec4899', (25, -60)),
        ('/Users/river/Fun/instant-robot/docs/public/models/boundary_gusset_bracket.stl', '3. Modular Gusset Bracket (with M3 Flanges)', '#f59e0b', (25, -45)),
        ('/Users/river/Fun/instant-robot/docs/public/models/boundary_stop_stand.stl', '4. Complete Assembled Stand', '#10b981', (30, -50)),
    ]

    fig = plt.figure(figsize=(16, 12), facecolor='#0f172a')
    
    for i, (path, title, color, view_angle) in enumerate(parts):
        ax = fig.add_subplot(2, 2, i + 1, projection='3d')
        triangles = read_stl(path)
        plot_part(ax, triangles, title, color)
        ax.view_init(elev=view_angle[0], azim=view_angle[1])

    plt.tight_layout()
    out_img = '/Users/river/.gemini/antigravity-ide/brain/a3f0fb2b-2953-4610-8e0f-da789809cae9/modular_stl_parts_preview.png'
    plt.savefig(out_img, dpi=180, facecolor=fig.get_facecolor(), edgecolor='none')
    print(f"Saved modular parts rendering to {out_img}")

if __name__ == '__main__':
    main()
