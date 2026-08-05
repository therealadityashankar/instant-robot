"""Generate a printable A4 PDF of a standard OpenCV ChArUco board.

A ChArUco board (chessboard + ArUco markers) is the "normal" OpenCV calibration
board. This is an alternative to the custom bordered-ArUco board — see the blog
discussion. It uses a DIFFERENT dictionary (DICT_5X5_100) from the block tags
(DICT_6X6_250, IDs 100–199) so the two never collide.

Output: printables/charuco_board.pdf  (+ charuco_board.png preview)

Usage:
    python make_charuco_board.py
    python make_charuco_board.py --squares-x 7 --squares-y 5 --square-mm 25 --marker-mm 18
"""
import argparse
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas as rl_canvas
from PIL import Image

# Dictionary chosen to NOT overlap with the block tags (DICT_6X6_250).
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
DPI = 300


def mm_to_px(mm_val: float, dpi: int = DPI) -> int:
    return int(round(mm_val / 25.4 * dpi))


def make_board(squares_x, squares_y, square_mm, marker_mm):
    """Create the CharucoBoard object (metres) + a rendered image (px)."""
    # Board geometry is stored in metres so poses come out in metres later.
    board = aruco.CharucoBoard(
        (squares_x, squares_y),
        square_mm / 1000.0,
        marker_mm / 1000.0,
        ARUCO_DICT,
    )
    w_px = mm_to_px(squares_x * square_mm)
    h_px = mm_to_px(squares_y * square_mm)
    margin = mm_to_px(4)
    img = board.generateImage((w_px + 2 * margin, h_px + 2 * margin), marginSize=margin, borderBits=1)
    return board, img


def save_pdf(img: np.ndarray, out_path: Path, board_w_mm, board_h_mm,
             squares_x, squares_y, square_mm, marker_mm):
    pil_img = Image.fromarray(img)
    a4_w_mm, a4_h_mm = 210.0, 297.0

    # The rendered image includes a 4 mm margin on each side.
    img_w_mm = board_w_mm + 8
    img_h_mm = board_h_mm + 8
    x_off_mm = (a4_w_mm - img_w_mm) / 2
    y_off_mm = (a4_h_mm - img_h_mm) / 2 + 20.0  # nudge up to leave room for labels

    c = rl_canvas.Canvas(str(out_path), pagesize=A4)

    import tempfile, os
    tmp = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
    tmp.close()
    pil_img.save(tmp.name, format="PNG", dpi=(DPI, DPI))
    c.drawImage(tmp.name, x_off_mm * mm, y_off_mm * mm,
                width=img_w_mm * mm, height=img_h_mm * mm)
    os.unlink(tmp.name)

    c.setFont("Helvetica", 8)
    c.drawCentredString(
        a4_w_mm / 2 * mm, (y_off_mm - 6) * mm,
        f"ChArUco {squares_x}x{squares_y}  square={square_mm:.0f}mm marker={marker_mm:.0f}mm  "
        f"DICT_5X5_100  |  PRINT AT 100% — NO SCALING",
    )

    # 100 mm scale bar to verify print accuracy with a ruler.
    bar_x = x_off_mm * mm
    bar_y = (y_off_mm - 14) * mm
    c.setLineWidth(1.5)
    c.line(bar_x, bar_y, bar_x + 100 * mm, bar_y)
    c.line(bar_x, bar_y - 2 * mm, bar_x, bar_y + 2 * mm)
    c.line(bar_x + 100 * mm, bar_y - 2 * mm, bar_x + 100 * mm, bar_y + 2 * mm)
    c.setFont("Helvetica-Bold", 8)
    c.drawCentredString(bar_x + 50 * mm, bar_y + 3 * mm, "|<----------- 100 mm ----------->|")

    c.showPage()
    c.save()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--squares-x", type=int, default=7)
    ap.add_argument("--squares-y", type=int, default=5)
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--marker-mm", type=float, default=18.0)
    ap.add_argument("--out", type=str, default="printables/charuco_board.pdf")
    args = ap.parse_args()

    board, img = make_board(args.squares_x, args.squares_y, args.square_mm, args.marker_mm)
    board_w_mm = args.squares_x * args.square_mm
    board_h_mm = args.squares_y * args.square_mm

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    save_pdf(img, out, board_w_mm, board_h_mm,
             args.squares_x, args.squares_y, args.square_mm, args.marker_mm)
    png = out.with_suffix(".png")
    Image.fromarray(img).save(png)

    print(f"Board: {args.squares_x}x{args.squares_y} squares  "
          f"{board_w_mm:.0f}x{board_h_mm:.0f} mm  square={args.square_mm}mm marker={args.marker_mm}mm")
    print(f"Saved {out}")
    print(f"Saved {png}")


if __name__ == "__main__":
    main()
