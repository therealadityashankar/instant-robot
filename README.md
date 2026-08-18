# pick-101 - Aruco + IK controller

This repo lets one detect the location of jenga blocks and place jenga blocks appropriately via the use of inverse kinematics

some code was initially adapted from  forked from ggand0/pick-101 but, very little of that code still exists - except for the usage of dm_control, specifically the mapping within the src/ directory

## Installation

this package needs uv installed to be run properly, this can be downloaded from https://docs.astral.sh/uv/

```bash
git clone https://github.com/therealadityashankar/pick-101.git
cd pick-101
uv sync
```

---

## Running on the Real Robot

### Step 1, Print the board, block tags, and nav tags

Print everything in [`printables/`](printables/) — these are committed to the repo,
so you can print them directly without regenerating:

- [`printables/aruco_board.pdf`](printables/aruco_board.pdf) — the calibration board
- [`printables/bordered_tags.pdf`](printables/bordered_tags.pdf) — bordered block tags (IDs 100–150)
- [`printables/bordered_tags_20mm.pdf`](printables/bordered_tags_20mm.pdf) — 20 mm variant
- [`printables/nav_tags.pdf`](printables/nav_tags.pdf) — **labeled navigation tags**

**IMPORTANT: do not scale the pages when printing — print at 100% scale**, or the
tag geometry won't match the calibration.

To regenerate them from source instead:

```bash
uv run python tags-and-borders/make_aruco_board.py     # printables/aruco_board.pdf
uv run python tags-and-borders/make_jenga_tag.py       # printables/jenga_tag.pdf
uv run python tags-and-borders/make_bordered_tags.py   # printables/bordered_tags.pdf
uv run python tags-and-borders/make_labeled_tags.py    # printables/nav_tags.pdf
```

#### Creating custom labeled nav tags

Navigation tags identify stations by **printed text**, not by tag number. Each
card has ArUco markers **200** (left) and **201** (right) flanking a text region
with a **label** on top and a **description** below:

```
┌──────────┬────────────────────┬──────────┐
│          │                    │          │
│  ArUco   │      LABEL        │  ArUco   │
│   200    │                   │   201    │
│          │────────────────────│          │
│          │   description     │          │
│          │     text          │          │
└──────────┴────────────────────┴──────────┘
```

The app detects both markers, uses their known positions to warp the middle
region flat via `findHomography`, and reads the text with Tesseract.js (or sends
the crop to Gemini when an API key is configured).

**Generate the default set** (Apple, Banana, Orange, Basket, Block, Bottle, Plant):

```bash
uv run python tags-and-borders/make_labeled_tags.py
```

**Create your own labels** — pass `"Label:Description"` pairs:

```bash
uv run python tags-and-borders/make_labeled_tags.py \
    --tags "Coffee Mug:White ceramic mug" \
           "Keys:Car keys with red fob" \
           "Charger:USB-C phone charger"
```

**Options:**

| Flag | Default | Description |
|------|---------|-------------|
| `--tags` | built-in 7 | Space-separated `"Label:Description"` pairs |
| `--size` | `40` | ArUco marker side in mm |
| `--dict` | `DICT_6X6_250` | ArUco dictionary |
| `--dpi` | `300` | Output resolution |
| `--out` | `printables/nav_tags.pdf` | Output path |

**Tips for good labels:**
- Use **short, distinct words** — `"Apple"` not `"Red Delicious Apple"`
- Labels are printed in **ALL CAPS** automatically for better OCR
- Keep descriptions to 3–5 words — they are context for the AI, not documentation
- Use a marker size of **40 mm+** so the text region is readable by the camera
- **Laminate** the printed cards for durability

### Step 2, calibrate joints

Maps real robot joint readings to simulation joint angles. Produces `.calibration/joint_calibration.json`.

```bash
# full calibration
uv run python calibration/calibrate_joints_real.py --port /dev/tty.usbmodem5A680089441

# single joint calibration
uv run python calibration/calibrate_joints_real.py --port /dev/tty.usbmodem5A680089441 --joint wrist_roll
```

### Step 3, Calibrate block position detection

Corrects for camera angle so the detected block position matches its true location on the board.

```bash
uv run python calibration/calibrate_board.py --camera 0
```

1. press C to start calibration
2. place the jenga block in the appropriately marked position
3. press space to set the position
4. repeat for all 4 corners
5. After 4 corners, the script fits a per-axis linear correction and saves `.calibration/camera_calibration.npz` — `run_real_ik.py` and `visualize_irl_block.py` load it automatically, no copy-pasting needed

### Step 4, Run on a real robot

```bash
uv run python run_real_ik.py --port /dev/tty.usbmodem5A680089441
```

Video is saved to `real_ik_run.mp4`.

## Simulated scene: stations and props

The sim places pedestals around the robot's start point, each with a labeled
nav tag on the side turned back toward the origin. All tags use the same ArUco
pair (200 & 201) — the station identity comes from the **label text** printed
between them (read via homography + OCR). Each station carries something
different:

| Label | Prop | Object tag | Pick status |
|---|---|---|---|
| Block | jenga block | 101 | the pick under development |
| Apple | apple | 102 | tag lies flat — same geometry as the block |
| Banana | banana | 103 | tag lies flat — same geometry as the block |
| Bottle | water bottle | 104 | **to do** — see below |
| Plant | potted plant | — | the thing to be watered |
| Basket | drop basket | — | where picked items go |
| Orange | orange | 105 | tag lies flat — same geometry as the block |

Props are built from MuJoCo primitives rather than imported meshes. What a pick
needs from an object is somewhere to put the jaws and a tag to aim at, and a
primitive gives exact collision geometry for a few bytes where a mesh gives
approximate collision for megabytes.

### To do: picking up the water bottle

Every grasp so far comes down onto a tag lying **flat**, with the approach axis
along the tag's normal — straight down. The bottle is an upright cylinder with
its marker on the **curved side**, so its tag normal is horizontal: the approach
has to come in sideways, and the jaws close around a round cross-section rather
than a flat-sided one. That needs:

- an approach axis taken from the tag's measured normal instead of forced
  vertical (`topDownGrasp` currently pins it to straight down, which is right for
  everything else in the scene and wrong for this);
- a grasp width and roll suited to a cylinder, where there is no long axis to
  line the jaws up with — any diameter will do, so the roll constraint that lines
  the jaws up with a tag edge stops being meaningful;
- somewhere to put it down again, since the point of the bottle is watering the
  plant.

### To do: apple and banana as movable objects

Both are static geoms on their pedestals at the moment, so they can be found and
approached but not lifted. Making them free bodies with a `freejoint`, the way
the block already is, is what turns them into pick targets.
