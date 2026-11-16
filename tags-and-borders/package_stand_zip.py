import zipfile
import os

def create_zip():
    models_dir = '/Users/river/Fun/instant-robot/docs/public/models'
    zip_path = os.path.join(models_dir, 'boundary_stand_stls.zip')
    
    files_to_zip = [
        ('boundary_base_plate.stl', '1_boundary_base_plate.stl'),
        ('boundary_upright_face.stl', '2_boundary_upright_face.stl'),
        ('boundary_gusset_bracket.stl', '3_boundary_gusset_bracket.stl'),
        ('boundary_stop_stand.stl', '4_boundary_stop_stand_combined_reference.stl'),
    ]
    
    readme_content = """Instant Robot - Modular Boundary Stop Stand (3D Printable STLs)
================================================================

Included Files:
1. 1_boundary_base_plate.stl               (80x80x4mm base plate with 4x centered M3 holes)
2. 2_boundary_upright_face.stl             (52x3.5x95mm clean vertical wall with 4x rear M3 holes)
3. 3_boundary_gusset_bracket.stl           (33x33mm single-side flange gusset bracket - PRINT 2X)
4. 4_boundary_stop_stand_combined_reference.stl (1-piece full assembled reference model)

Bill of Materials (BOM) for 1 Modular Stand:
- 1x Base Plate
- 1x Upright Face Plate
- 2x Gusset Brackets
- 8x M3x8mm or M3x10mm socket head cap screws (DIN 912)
- 8x M3 hex nuts (or threaded heat-set inserts)
- 1x Thin Vertical STOP Tag (Tag #240) from the Printable Tag Generator

Recommended 3D Print Settings:
- Material: PLA, PETG, or ABS
- Layer Height: 0.20 mm
- Wall / Perimeters: 3-4 loops
- Infill: 15% - 20% (Grid or Gyroid)
- Supports: NONE (all parts print flat on their largest faces)

Assembly:
1. Bolt the 2 gusset brackets to the base plate using 4x M3 screws and nuts from underneath.
2. Bolt the upright face plate to the rear flanges of the 2 gussets using 4x M3 screws and nuts.
3. Glue or stick the printed STOP tag #240 to the front face.
"""
    
    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        for orig_name, arcname in files_to_zip:
            orig_path = os.path.join(models_dir, orig_name)
            if os.path.exists(orig_path):
                zipf.write(orig_path, arcname=arcname)
        zipf.writestr('README.txt', readme_content)
        
    print(f"Created {zip_path}")
    
    # Sync to other folders
    dest_dirs = [
        '/Users/river/Fun/instant-robot/app/public/models',
        '/Users/river/Fun/instant-robot/app/public/docs/models',
    ]
    for d in dest_dirs:
        os.makedirs(d, exist_ok=True)
        dest_zip = os.path.join(d, 'boundary_stand_stls.zip')
        with open(zip_path, 'rb') as src, open(dest_zip, 'wb') as dst:
            dst.write(src.read())
        print(f"Synced to {dest_zip}")

if __name__ == '__main__':
    create_zip()
