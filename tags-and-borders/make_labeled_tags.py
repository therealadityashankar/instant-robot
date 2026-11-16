"""make_labeled_tags.py — Generate labeled navigation tags for Instant Robot.

Each tag card has ArUco markers 200 and 201 on the LEFT and RIGHT, with a text
region in between. The top half of the text region is the LABEL (station
identity), the bottom half is a DESCRIPTION (extra context for Gemini AI).

    ┌──────────┬────────────────────┬──────────┐
    │          │                    │          │
    │  ArUco   │      LABEL        │  ArUco   │
    │   200    │                   │   201    │
    │          │────────────────────│          │
    │          │   description     │          │
    │          │     text          │          │
    └──────────┴────────────────────┴──────────┘

The app detects both markers, uses their known positions to compute a
homography for the text region between them, warps it flat, and reads it
with Tesseract.js (or sends the crop to Gemini).

IMPORTANT: print at 100% / no scaling, so the marker sizes match solvePnP.

Output: printables/nav_tags.pdf + printables/nav_tags.png (page-1 preview).

Usage:
    # Default set of labels
    uv run python make_labeled_tags.py

    # Custom labels  (label:description pairs)
    uv run python make_labeled_tags.py \\
        --tags "Apple:Red fruit on table" "Banana:Yellow curved fruit" \\
               "Basket:Drop zone for items" "Orange:Round citrus fruit"

    # Bigger marker
    uv run python make_labeled_tags.py --size 50
"""

import argparse
import os
import tempfile
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas as rl_canvas

DPI = 300
A4_W_MM, A4_H_MM = 210.0, 297.0
PAGE_MARGIN_MM = 10.0
GUTTER_MM = 10.0  # space between cards on the page

# Universal right anchor tag ID — always 200 on every station card.
RIGHT_ID = 200
DEFAULT_START_LEFT_ID = 201

# Card geometry relative to marker size
QUIET_MM = 3.0        # quiet-zone padding around each marker
TEXT_GAP_MM = 4.0      # gap between marker edge and text region
BORDER_MM = 0.6        # card outline thickness
CARD_PAD_MM = 3.0      # padding inside card border
CAPTION_H_MM = 8.0     # bottom caption height for large prominent tag ID numbers


def mm_to_px(v, dpi=DPI):
    return int(round(v / 25.4 * dpi))


def marker_img(tag_id: int, size_px: int, dict_name: str = "DICT_6X6_250") -> np.ndarray:
    d = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    return cv2.cvtColor(
        aruco.generateImageMarker(d, tag_id, size_px, borderBits=1),
        cv2.COLOR_GRAY2RGB,
    )


def _try_load_font(size: int, bold: bool = False):
    """Try to load a clean sans-serif font; fall back to PIL default."""
    if bold:
        candidates = [
            "/System/Library/Fonts/Helvetica.ttc",
            "/Library/Fonts/Arial Bold.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
            "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
            "C:/Windows/Fonts/arialbd.ttf",
        ]
    else:
        candidates = [
            "/System/Library/Fonts/Helvetica.ttc",
            "/Library/Fonts/Arial.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
            "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
            "C:/Windows/Fonts/arial.ttf",
        ]
    for p in candidates:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()


def render_text_block(
    text: str,
    width_px: int,
    height_px: int,
    max_font_size: int,
    bold: bool = False,
    bg=(255, 255, 255),
    fg=(0, 0, 0),
) -> np.ndarray:
    """Render text centered within a fixed-size bounding box, auto-scaling font size."""
    img = Image.new("RGB", (width_px, height_px), bg)
    draw = ImageDraw.Draw(img)

    # Word-wrap helper
    def wrap_text(t: str, f: ImageFont.ImageFont, max_w: int) -> list[str]:
        words = t.split()
        if not words:
            return [""]
        lines = []
        cur = words[0]
        for w in words[1:]:
            test = f"{cur} {w}"
            bbox = draw.textbbox((0, 0), test, font=f)
            if (bbox[2] - bbox[0]) <= max_w:
                cur = test
            else:
                lines.append(cur)
                cur = w
        lines.append(cur)
        return lines

    font_size = max_font_size
    lines = [text]
    pad = int(round(width_px * 0.04))
    usable_w = width_px - 2 * pad

    while font_size >= 8:
        font = _try_load_font(font_size, bold=bold)
        lines = wrap_text(text, font, usable_w)
        line_heights = []
        for line in lines:
            bb = draw.textbbox((0, 0), line or "A", font=font)
            line_heights.append(bb[3] - bb[1])
        total_h = sum(line_heights) + int(len(lines) * font_size * 0.25)
        if total_h <= height_px * 0.90:
            break
        font_size -= 1

    font = _try_load_font(max(font_size, 8), bold=bold)
    lines = wrap_text(text, font, usable_w)
    line_bboxes = [draw.textbbox((0, 0), line or "A", font=font) for line in lines]
    line_heights = [bb[3] - bb[1] for bb in line_bboxes]
    line_spacing = int(font_size * 0.25)
    total_h = sum(line_heights) + (len(lines) - 1) * line_spacing

    y_curr = (height_px - total_h) // 2
    for line, lh in zip(lines, line_heights):
        bb = draw.textbbox((0, 0), line, font=font)
        lw = bb[2] - bb[0]
        x = (width_px - lw) // 2
        draw.text((x, y_curr), line, fill=fg, font=font)
        y_curr += lh

    return np.array(img)


def build_card(label: str, description: str, marker_mm: float,
               left_id: int = 201, dict_name: str = "DICT_6X6_250", dpi: int = DPI) -> np.ndarray:
    """Build a single labeled nav-tag card (RGB, at `dpi`).

    Layout:
        [pad][quiet][unique left marker][quiet][gap][TEXT REGION][gap][quiet][anchor 200][quiet][pad]
        [generous space for large prominent ID #left_id and ID #200]
    """
    # Enforce 20-character maximum limit on label
    clean_label = label.strip()[:20].upper()

    marker_px = mm_to_px(marker_mm, dpi)
    quiet_px = mm_to_px(QUIET_MM, dpi)
    gap_px = mm_to_px(TEXT_GAP_MM, dpi)
    pad_px = mm_to_px(CARD_PAD_MM, dpi)
    caption_px = mm_to_px(CAPTION_H_MM, dpi)
    border_px = max(1, mm_to_px(BORDER_MM, dpi))

    # Text region width: at least as wide as a marker, looks good at ~1.3x marker
    text_w = max(marker_px, int(marker_px * 1.3))

    # Card dimensions
    card_w = (pad_px + quiet_px + marker_px + quiet_px + gap_px) * 2 + text_w
    card_h = pad_px + quiet_px + marker_px + quiet_px + caption_px + pad_px

    card = np.ones((card_h, card_w, 3), dtype=np.uint8) * 255

    # ── Left marker (Unique Station ID) ──
    mk_left = marker_img(left_id, marker_px, dict_name)
    lx = pad_px + quiet_px
    my = pad_px + quiet_px
    card[my:my + marker_px, lx:lx + marker_px] = mk_left

    # ── Right marker (Universal Anchor ID 200) ──
    mk_right = marker_img(RIGHT_ID, marker_px, dict_name)
    rx = card_w - pad_px - quiet_px - marker_px
    card[my:my + marker_px, rx:rx + marker_px] = mk_right

    # ── Text region (between the two markers) ──
    text_x = lx + marker_px + quiet_px + gap_px
    text_total_w = rx - quiet_px - gap_px - text_x
    if text_total_w < 10:
        text_total_w = text_w  # fallback
    text_h = marker_px  # same height as the markers

    # Split into label (top half) and description (bottom half)
    label_h = text_h // 2
    desc_h = text_h - label_h

    # Render label (bold, large, uppercase, max 20 chars)
    label_font_size = max(14, int(marker_px * 0.30))
    label_img = render_text_block(
        clean_label, text_total_w, label_h, label_font_size,
        bold=True, fg=(0, 0, 0),
    )
    card[my:my + label_h, text_x:text_x + text_total_w] = label_img

    # Separator line between label and description
    sep_y = my + label_h
    cv2.line(card, (text_x, sep_y), (text_x + text_total_w, sep_y),
             (180, 180, 180), max(1, dpi // 300))

    # Render description (smaller, grey-on-light-grey)
    desc_font_size = max(10, int(marker_px * 0.14))
    desc_img = render_text_block(
        description, text_total_w, desc_h, desc_font_size,
        bold=False, bg=(248, 248, 248), fg=(60, 60, 60),
    )
    card[sep_y + 1:my + text_h, text_x:text_x + text_total_w] = desc_img[:text_h - label_h - 1]

    # ── Bottom Captions: Prominent Large Tag IDs with Generous Padding ──
    caption_y = my + marker_px + mm_to_px(5.5, dpi)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.52 * (dpi / 300)
    thickness = max(1, int(1.5 * (dpi / 300)))

    # Left tag caption (Large & Bold)
    cv2.putText(
        card, f"ID #{left_id}",
        (lx + mm_to_px(1, dpi), caption_y),
        font, font_scale, (15, 23, 42), thickness, cv2.LINE_AA,
    )
    # Right tag caption
    cv2.putText(
        card, "ID #200 (END)",
        (rx + mm_to_px(0.5, dpi), caption_y),
        font, font_scale * 0.88, (100, 116, 139), thickness, cv2.LINE_AA,
    )

    # ── Card border ──
    cv2.rectangle(card, (0, 0), (card_w - 1, card_h - 1), (120, 120, 120), border_px)

    # Light vertical separator lines between markers and text
    cv2.line(card, (text_x - gap_px // 2, my), (text_x - gap_px // 2, my + marker_px),
             (210, 210, 210), 1)
    cv2.line(card, (text_x + text_total_w + gap_px // 2, my),
             (text_x + text_total_w + gap_px // 2, my + marker_px),
             (210, 210, 210), 1)

    return card


def build_pages(tags: list[tuple[str, str]], marker_mm: float,
                dict_name: str, start_id: int = DEFAULT_START_LEFT_ID,
                dpi: int = DPI) -> list[np.ndarray]:
    """Lay out cards on A4 pages with unique left station IDs."""
    a4w, a4h = mm_to_px(A4_W_MM, dpi), mm_to_px(A4_H_MM, dpi)
    margin = mm_to_px(PAGE_MARGIN_MM, dpi)
    gutter = mm_to_px(GUTTER_MM, dpi)
    title_h = mm_to_px(8, dpi)

    # Build a sample card to measure
    sample = build_card("X", "x", marker_mm, start_id, dict_name, dpi)
    cw, ch = sample.shape[1], sample.shape[0]

    cols = max(1, (a4w - 2 * margin + gutter) // (cw + gutter))
    rows = max(1, (a4h - margin - title_h - margin + gutter) // (ch + gutter))
    per_page = cols * rows

    pages = []
    font = cv2.FONT_HERSHEY_SIMPLEX
    for page_idx, start in enumerate(range(0, len(tags), per_page)):
        chunk = tags[start:start + per_page]
        page = np.ones((a4h, a4w, 3), dtype=np.uint8) * 255

        # Title
        cv2.putText(
            page,
            f"Labeled Nav Tags  -  {dict_name}  -  marker {marker_mm:.0f} mm  -  "
            f"Unique Left IDs + Right Anchor 200  -  PRINT AT 100%, NO SCALING",
            (margin, margin + mm_to_px(4, dpi)),
            font, 0.38, (30, 30, 30), 1, cv2.LINE_AA,
        )

        y_start = margin + title_h
        for k, (label, desc) in enumerate(chunk):
            left_id = start_id + start + k
            card = build_card(label, desc, marker_mm, left_id, dict_name, dpi)
            r, c = divmod(k, cols)
            x = margin + c * (cw + gutter)
            y = y_start + r * (ch + gutter)
            if y + ch > a4h - margin:
                break
            h_fit = min(ch, a4h - y)
            w_fit = min(cw, a4w - x)
            card_crop = card[:h_fit, :w_fit]
            page[y:y + card_crop.shape[0], x:x + card_crop.shape[1]] = card_crop

        # Scale bar (bottom-left)
        bx0, by = margin, a4h - mm_to_px(6, dpi)
        bx1 = bx0 + mm_to_px(50, dpi)
        tick = mm_to_px(2, dpi)
        cv2.line(page, (bx0, by), (bx1, by), (80, 80, 80), 2)
        cv2.line(page, (bx0, by - tick), (bx0, by + tick), (80, 80, 80), 2)
        cv2.line(page, (bx1, by - tick), (bx1, by + tick), (80, 80, 80), 2)
        cv2.putText(page, "50 mm", (bx0 + mm_to_px(3, dpi), by - mm_to_px(2, dpi)),
                    font, 0.4, (80, 80, 80), 1, cv2.LINE_AA)

        pages.append(page)

    print(f"Layout: {cols}x{rows} = {per_page} cards/page, {len(pages)} page(s)")
    return pages


def save_pdf(pages: list[np.ndarray], out_path: str, dpi: int = DPI):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    c = rl_canvas.Canvas(str(out_path), pagesize=A4)
    tmpfiles = []
    for page in pages:
        tmp = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
        tmp.close()
        Image.fromarray(page).save(tmp.name, format="PNG", dpi=(dpi, dpi))
        tmpfiles.append(tmp.name)
        c.drawImage(tmp.name, 0, 0, width=210 * mm, height=297 * mm)
        c.showPage()
    c.save()
    for t in tmpfiles:
        os.unlink(t)
    print(f"PDF saved: {out_path}  ({len(pages)} page(s))")
    png = out_path.with_suffix(".png")
    cv2.imwrite(str(png), cv2.cvtColor(pages[0], cv2.COLOR_RGB2BGR))
    print(f"PNG preview (page 1): {png}")


# ── Default tags ───────────────────────────────────────────────────────────────
DEFAULT_TAGS = [
    ("Apple",   "Red fruit, pick it up"),
    ("Banana",  "Yellow curved fruit"),
    ("Orange",  "Round citrus fruit"),
    ("Basket",  "Drop zone for items"),
    ("Block",   "Wooden jenga block"),
    ("Bottle",  "Water bottle, side tag"),
    ("Plant",   "Potted plant, water me"),
]


def parse_tag_spec(spec: str) -> tuple[str, str]:
    """Parse 'Label:Description' or just 'Label'."""
    if ":" in spec:
        label, desc = spec.split(":", 1)
        return label.strip(), desc.strip()
    return spec.strip(), ""


def main():
    ap = argparse.ArgumentParser(
        description="Generate labeled navigation tags for Instant Robot.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Each card has a unique ArUco marker on the left (e.g. 201, 202...) and fixed anchor marker 200 on the right flanking a text region.
The top half is the LABEL (station identity), the bottom half is a DESCRIPTION
(extra context for Gemini AI). The app reads the text via homography + OCR.

Examples:
  uv run python make_labeled_tags.py
  uv run python make_labeled_tags.py --tags "Apple:Red fruit" "Basket:Drop zone" --start-id 250
  uv run python make_labeled_tags.py --size 50
        """,
    )
    ap.add_argument(
        "--tags", type=str, nargs="+", default=None,
        help='Tag specs as "Label:Description" pairs. Default: built-in set of 7.',
    )
    ap.add_argument("--start-id", type=int, default=DEFAULT_START_LEFT_ID,
                    help="Starting unique ArUco ID for left marker (default: 201).")
    ap.add_argument("--size", type=float, default=30.0,
                    help="ArUco marker side (mm). Default: 30")
    ap.add_argument("--dict", type=str, default="DICT_6X6_250",
                    help="ArUco dictionary")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--out", type=str, default="printables/nav_tags.pdf")
    args = ap.parse_args()

    if args.tags:
        tags = [parse_tag_spec(s) for s in args.tags]
    else:
        tags = DEFAULT_TAGS

    if not hasattr(aruco, args.dict):
        raise SystemExit(f"Unknown dictionary '{args.dict}'.")

    print(f"Generating {len(tags)} labeled nav tags "
          f"(Unique Left IDs {args.start_id}..{args.start_id + len(tags) - 1} + Right Anchor {RIGHT_ID}, {args.size:.0f} mm):")
    for idx, (label, desc) in enumerate(tags):
        tag_id = args.start_id + idx
        print(f"  • ID #{tag_id}: {label}" + (f"  —  {desc}" if desc else ""))

    pages = build_pages(tags, args.size, args.dict, start_id=args.start_id, dpi=args.dpi)
    save_pdf(pages, args.out, dpi=args.dpi)


if __name__ == "__main__":
    main()


if __name__ == "__main__":
    main()
