---
title: 3D Printing the Stop Stands
description: Complete CAD models, STL downloads, and assembly instructions for the modular plug-and-play M3 stop stands.
---

The **Stop Stands** define the physical start and end limits of the linear station rack. They hold the standalone **Thin Vertical STOP Tags (`Tag #240`)** aligned flush at the top of the upright face, which the robot uses during exploration sweeps to detect corridor limits and reverse heading.

To eliminate failed full-model prints and allow quick repairs or modifications, the stand is designed as a **Modular Plug-and-Play M3 System**.

---

## 📐 Modular Stand Architecture

```
       Front View                             Side Profile (Dual Gussets)
  ┌──────────────┐                         │ Clean Upright Face (52x95mm)
  │   [ #240 ]   │                         │ (Tag aligned to top)
  │    [STOP]    │                       ▲ │ \ Dual Rear Isosceles Gussets
  │              │                     95mm│   \ (45°-45°-90° braces with side flanges)
  │              │                         │     \
  └──────────────┘                    ─────┴───────\───── Base Plate (80mm x 80mm)
  ════════════════ (Base)             ◄────── 80mm ──────►
```

---

## 📥 STL Model Downloads

<div style="background: rgba(56, 189, 248, 0.08); border: 1px solid rgba(56, 189, 248, 0.25); border-radius: 10px; padding: 1.25rem; margin-bottom: 1.5rem;">
  <h4 style="margin: 0 0 0.5rem 0; color: #38bdf8;">📦 1-Click All-in-One STL Bundle</h4>
  <p style="margin: 0 0 1rem 0; color: #cbd5e1; font-size: 0.95rem;">Download all modular STL files + assembled reference model + README instructions in a single ZIP archive:</p>
  <a
    href="/docs/models/boundary_stand_stls.zip"
    download="boundary_stand_stls.zip"
    style="display: inline-flex; align-items: center; gap: 0.5rem; background: #0284c7; color: #ffffff; padding: 0.65rem 1.25rem; border-radius: 6px; text-decoration: none; font-weight: 600; font-size: 0.95rem;"
  >
    📥 Download All STLs (.ZIP Bundle)
  </a>
</div>

### Individual Component Downloads

Print each modular component individually. The base plate features an **extended front shelf** with dedicated M3 screw holes to optionally mount the Low-Poly Fox or Rabbit mascot attachments:

| Component | Dimensions | Description | Download |
| :--- | :--- | :--- | :--- |
| **1. Extended Base Plate** | 115 × 80 × 6 mm | Rounded corners ($R=6\text{mm}$), **$3.2\text{mm}$ bottom counterbored pockets** (gusset + mascot holes), & tactile texture | [📥 Download Base Plate (STL)](/docs/models/boundary_base_plate.stl) |
| **2. Clean Upright Face Plate** | 52 × 4 × 95 mm | Vertical wall with **front counterbored pockets** (screw heads flush), **rounded top corners ($R=6\text{mm}$)**, square bottom edge, & top tag zone | [📥 Download Upright Face (STL)](/docs/models/boundary_upright_face.stl) |
| **3. Gusset Bracket** | 33 × 33 mm (45°) | Triangular stiffener with side base & back **3.8mm M3** flanges (print 2×) | [📥 Download Gusset Bracket (STL)](/docs/models/boundary_gusset_bracket.stl) |
| **4. Combined Reference Stand** | 115 × 80 × 101 mm | Full 1-piece assembled stand with dual counterbored faces, rounded corners, & textured surfaces | [📥 Download Assembled Stand (STL)](/docs/models/boundary_stop_stand.stl) |

---

### 🦊 Mascot Attachments (Optional Bolt-On Front Accessories)

Screw these low-poly animal mascots onto the front shelf of the base plate using 2× M3 screws from underneath:

| Mascot Attachment | Dimensions | Description | Download |
| :--- | :--- | :--- | :--- |
| **🦊 Fox Stand Attachment** | 46 × 56 × 45 mm | Low-Poly Fox on matching M3 mounting base plate | [📥 Download Fox Attachment (STL)](/docs/models/fox_stand_attachment.stl) |
| **🐰 Rabbit Stand Attachment** | 47 × 56 × 45 mm | Low-Poly Rabbit on matching M3 mounting base plate | [📥 Download Rabbit Attachment (STL)](/docs/models/rabbit_stand_attachment.stl) |
| **🦊 Complete Stand with Fox** | 115 × 80 × 101 mm | Full 1-piece monolithic stand with Fox mascot on front shelf | [📥 Download Stand + Fox (STL)](/docs/models/boundary_stop_stand_fox.stl) |
| **🐰 Complete Stand with Rabbit** | 115 × 80 × 101 mm | Full 1-piece monolithic stand with Rabbit mascot on front shelf | [📥 Download Stand + Rabbit (STL)](/docs/models/boundary_stop_stand_rabbit.stl) |

---

## 🔩 Hardware Bill of Materials (BOM)

For assembling one modular stop stand:

| Item | Quantity | Description |
| :--- | :--- | :--- |
| **M3×8mm or M3×10mm Screws** | 8 pcs | Socket head cap screws (DIN 912 / ISO 4762) — slides freely through 3.8mm holes |
| **M3 Hex Nuts** | 8 pcs | Standard M3 hex nuts (or M3 locknuts) |
| **Stop Stand Tag** | 1 pc | Printed from the [Printable Tag Generator](/docs/tags-and-navigation/tag-generator/) |

---

## 🎨 How to Get a Premium "Non-3D-Printed" Matte Texture

To make your parts look and feel like injection-molded / powder-coated engineering equipment rather than shiny 3D prints:

1. **Built-in Geometric Knurling (in the STLs)**:
   - The STL models feature a fine procedural **$0.22\text{ mm}$ diamond micro-relief** on the top surfaces. Slicers generate real micro-ridges that break up top-layer extrusion lines.
2. **Textured PEI Build Plate (Bottom Face)**:
   - Printing the flat base plate directly onto a **Textured Gold PEI Sheet** gives the bottom and outer perimeter a gorgeous fine-grain matte powder-coat finish.
3. **Fuzzy Skin Slicer Setting (Side Perimeters)**:
   - In **Bambu Studio / OrcaSlicer / PrusaSlicer / Cura**, enable:
     - **Fuzzy Skin**: `Contour` (or `Outside Walls Only`)
     - **Fuzzy Skin Thickness**: `0.15 mm – 0.20 mm`
     - **Fuzzy Skin Point Distance**: `0.30 mm – 0.40 mm`
   - *Result*: Completely hides all vertical layer lines and creates an ultra-grippy, non-slip matte texture.

---

## 🖨️ Recommended 3D Print Settings

| Parameter | Recommended Value | Notes |
| :--- | :--- | :--- |
| **Material** | PLA, PETG, or ABS | Matte PLA or Matte PETG looks stunning with texture |
| **Layer Height** | `0.20 mm` | Balances detail and fast print speed |
| **Wall / Perimeters** | `3 – 4 loops` | Ensures strong screw hole perimeters |
| **Top / Bottom Layers** | `4 layers` | Solid surface finish |
| **Infill** | `15% – 20%` | Gyroid or Grid infill |
| **Supports** | **None** | All modular parts print flat on their largest faces |
| **Bed Adhesion** | Skirt or Brim (optional) | Not typically needed on clean PEI sheets |

---

## 🛠️ Step-by-Step Assembly Guide

1. **Print the Components**:
   - 1× `boundary_base_plate.stl`
   - 1× `boundary_upright_face.stl`
   - 2× `boundary_gusset_bracket.stl`
2. **Mount the Dual Gusset Brackets to the Base**:
   - Place the Left Gusset Bracket at $Y = -15\text{ mm}$ and the Right Gusset Bracket at $Y = +15\text{ mm}$.
   - Drop M3 screws through the 3.8mm holes and secure to the Base Plate using M3 nuts from the bottom.
3. **Attach Upright Face Plate**:
   - Position the Upright Face Plate against the vertical front faces of the two gusset brackets.
   - Insert 4× M3 screws through the face plate holes into the gusset rear flanges and tighten.
4. **Apply Tag to Top of Stand**:
   - Generate and print the thin vertical STOP tag from the [Printable Tag Generator](/docs/tags-and-navigation/tag-generator/).
   - Stick or glue the tag aligned with the **top edge** of the upright front face.
5. **(Optional) Mount Fox or Rabbit Mascot**:
   - Align the 2× holes of `fox_stand_attachment.stl` or `rabbit_stand_attachment.stl` with the front shelf holes.
   - Secure from the bottom of the base plate using 2× M3×8mm screws.

---

## 📜 Third-Party Model Credits & Attributions

The optional 3D animal mascots are licensed under the Creative Commons Attribution 3.0 Unported ([CC-BY 3.0](https://creativecommons.org/licenses/by/3.0/)) license:

| Model | Author | License | Source |
| :--- | :--- | :--- | :--- |
| **Fox v2** | Jake Blakeley | [CC-BY 3.0](https://creativecommons.org/licenses/by/3.0/) | [Poly Pizza (0-NaVqxdfRu)](https://poly.pizza/m/0-NaVqxdfRu) |
| **Rabbit** | Poly by Google | [CC-BY 3.0](https://creativecommons.org/licenses/by/3.0/) | [Poly Pizza (dyeBDJxhDwP)](https://poly.pizza/m/dyeBDJxhDwP) |

*Modifications: Scaled, oriented, and mounted onto 3D-printable modular stop stand base attachments, and decomposed into segmented multi-color meshes for the MuJoCo simulator.*


