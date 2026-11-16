import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def read_stl(filename):
    with open(filename, 'rb') as f:
        header = f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]
        triangles = []
        for _ in range(num_triangles):
            normal = struct.unpack('<3f', f.read(12))
            pts = [struct.unpack('<3f', f.read(12)) for _ in range(3)]
            attr = f.read(2)
            triangles.append(pts)
    return np.array(triangles)

def main():
    stl_path = '/Users/river/Fun/instant-robot/app/public/models/boundary_stop_stand.stl'
    triangles = read_stl(stl_path)
    print(f"Loaded {len(triangles)} triangles from {stl_path}")

    fig = plt.figure(figsize=(16, 12), dpi=150)
    fig.patch.set_facecolor('#1e293b')

    # View 1: 3D Isometric View from Back-Left (showing gusset clearly)
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.set_facecolor('#0f172a')
    poly1 = Poly3DCollection(triangles, facecolors='#38bdf8', edgecolors='#0284c7', alpha=0.85, linewidths=0.6)
    ax1.add_collection3d(poly1)
    ax1.view_init(elev=28, azim=-135) # Looking from behind
    ax1.set_title("1. Perspective View (From Rear/Side showing Gusset)", color='#f8fafc', fontsize=12, fontweight='bold')

    # View 2: Side Elevation View (X-Z plane showing exact right-angled isosceles triangle)
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    ax2.set_facecolor('#0f172a')
    poly2 = Poly3DCollection(triangles, facecolors='#38bdf8', edgecolors='#0284c7', alpha=0.85, linewidths=0.6)
    ax2.add_collection3d(poly2)
    ax2.view_init(elev=0, azim=-90) # Exact side elevation
    ax2.set_title("2. Side Elevation View (Profile showing 45° Gusset)", color='#f8fafc', fontsize=12, fontweight='bold')

    # View 3: Front Elevation View (showing thin upright face centered on base)
    ax3 = fig.add_subplot(2, 2, 3, projection='3d')
    ax3.set_facecolor('#0f172a')
    poly3 = Poly3DCollection(triangles, facecolors='#38bdf8', edgecolors='#0284c7', alpha=0.85, linewidths=0.6)
    ax3.add_collection3d(poly3)
    ax3.view_init(elev=10, azim=0) # Front view
    ax3.set_title("3. Front View (Upright Face Centered on Base)", color='#f8fafc', fontsize=12, fontweight='bold')

    # View 4: Top View (X-Y plane)
    ax4 = fig.add_subplot(2, 2, 4, projection='3d')
    ax4.set_facecolor('#0f172a')
    poly4 = Poly3DCollection(triangles, facecolors='#38bdf8', edgecolors='#0284c7', alpha=0.85, linewidths=0.6)
    ax4.add_collection3d(poly4)
    ax4.view_init(elev=90, azim=-90) # Top down
    ax4.set_title("4. Top View (Base 80x80, Centered Face, Rear Fin)", color='#f8fafc', fontsize=12, fontweight='bold')

    for ax in [ax1, ax2, ax3, ax4]:
        ax.set_xlim([-50, 50])
        ax.set_ylim([-50, 50])
        ax.set_zlim([0, 90])
        ax.set_xlabel('X (mm)', color='#94a3b8')
        ax.set_ylabel('Y (mm)', color='#94a3b8')
        ax.set_zlabel('Z (mm)', color='#94a3b8')
        ax.tick_params(colors='#94a3b8')
        # Grid styling
        ax.xaxis._axinfo["grid"]['color'] = "#334155"
        ax.yaxis._axinfo["grid"]['color'] = "#334155"
        ax.zaxis._axinfo["grid"]['color'] = "#334155"

    out_path = '/Users/river/.gemini/antigravity-ide/brain/a3f0fb2b-2953-4610-8e0f-da789809cae9/stl_multi_angle_views.png'
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, facecolor=fig.get_facecolor(), edgecolor='none')
    print(f"Multi-angle rendering saved to {out_path}")

if __name__ == '__main__':
    main()
