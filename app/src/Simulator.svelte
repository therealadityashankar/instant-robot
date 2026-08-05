<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import {
    loadMujocoModule,
    mountModel,
    type ModelFiles,
    type Session,
  } from './lib/mujocoSession';
  import { IKSolver } from './lib/ik';
  import { MujocoRenderer } from './lib/mujocoRender';
  import { Robot } from './lib/robot';
  import { CALIBRATION_PLAN, simRadToServo, type JointCalibration } from './lib/joints';
  import {
    saveJointCalibration,
    loadJointCalibration,
    loadIntrinsics,
    saveRobotId,
    loadRobotId,
    saveArmOffset,
    loadArmOffset,
    saveBaseConfig,
    loadBaseConfig,
  } from './lib/storage';
  import { wheelSpeeds, bodyToPrimitives, PRIMITIVE_DIRS, CAL_PAIRS, DEFAULT_BASE_CONFIG, type BaseConfig } from './lib/lekiwiBase';
  import {
    buildBoardSceneXml,
    interiorToSim,
    SQUARE_MM,
    INSET_MM,
    BLOCK_HALF_Z,
    ROBOT_BASE_TIP_X,
  } from './lib/boardSim';
  import { ROBOTS, robotById, injectBase, type RobotId, type BaseManifest } from './lib/robots';
  import { injectShelves, shelfJointName, rodLocal, SHELF } from './lib/shelves';
  import { phaseTarget, DEFAULT_PICK, type PickPhase } from './lib/pick';
  import LoadingScreen from './LoadingScreen.svelte';
  import DriveTriad from './DriveTriad.svelte';
  import RotationControl from './RotationControl.svelte';
  import { loadCv, type Cv } from './lib/cv';
  import { boardTagCentres, BORDERED_IDS, type TagCentres } from './lib/board';
  import { createDetector, detectMarkers, type Detector } from './lib/homography';
  import { boardPoseFromTags, blockBoardXY } from './lib/detect3d';
  import { solvePnpMarkerPose } from './lib/pose';
  import type { Intrinsics } from './lib/charuco';

  const XML = 'so101_new_calib.xml';
  const ASSETS = [
    'waveshare_mounting_plate_so101_v2.stl',
    'sts3215_03a_v1.stl',
    'motor_holder_so101_base_v1.stl',
    'wrist_roll_follower_so101_v1.stl',
    'moving_jaw_so101_v1.stl',
    'base_motor_holder_so101_v1.stl',
    'upper_arm_so101_v1.stl',
    'wrist_roll_pitch_so101_v2.stl',
    'under_arm_so101_v1.stl',
    'rotation_pitch_so101_v1.stl',
    'motor_holder_so101_wrist_v1.stl',
    'sts3215_03a_no_horn_v1.stl',
    'base_so101_v2.stl',
  ];
  const ARM_DOFS = [0, 1, 2, 3, 4];
  // Extra 90° added to wrist_roll (dof 4) for the on-screen pose ONLY — the real
  // servo still gets the raw IK-solved angle.
  const WRIST_ROLL_OFFSET = Math.PI / 2;
  const GRASP_SITE = 'graspframe';

  let canvas: HTMLCanvasElement;
  let wrap: HTMLDivElement;

  let status = $state('Loading MuJoCo…');
  let ready = $state(false);
  let errorMsg = $state<string | null>(null);

  let target = $state<[number, number, number]>([0.25, 0, 0.2]);
  // Manual grasp orientation (best-effort, not a hard 6-DOF pose):
  //  top  → straight down · 45 → halfway · front → horizontal (toward target)
  //  none → don't constrain the approach at all, only fix the wrist roll.
  // Roll rotates the gripper about the approach axis (top/45/front) or fixes the
  // wrist_roll joint directly (none).
  let graspMode = $state<'top' | '45' | 'front' | 'none'>('top');
  let graspRollDeg = $state(0);
  // Which IK branch to prefer: 'top' clamps wrist_flex ≥ 0 (arm bends up), 'bottom'
  // clamps it ≤ 0. Resolves the two-solution ambiguity for a given target.
  let armBranch = $state<'auto' | 'top' | 'bottom'>('auto');
  const WRIST_FLEX_HI = 1.658; // wrist_flex actuator range ±1.658 rad
  // Rest / home pose (sim radians per arm joint) — arm swung to the right and
  // tucked upright, out of the way. Used by the "Rest" arm mode and auto-engaged
  // while the base is navigating (the EE target isn't pursued, so the real arm
  // doesn't flail as the base moves). [pan(+=right), lift(0=up), elbow(fold),
  // wrist_flex, wrist_roll].
  const HOME_POSE = [1.3, 0.0, 0.9, 0.0, 0.0];
  let armRest = $state(false); // park the arm at HOME instead of pursuing the target
  let ikOk = $state(true);
  let ikErrMm = $state(0);
  let jointAngles = $state<number[]>([0, 0, 0, 0, 0]);
  let gripperRange = $state<[number, number]>([-0.1745, 1.7453]); // [open, closed]

  // ── sim / real ─────────────────────────────────────────────────────────────
  let mode = $state<'sim' | 'real'>('sim');
  const robot = new Robot();
  let realConnected = $state(false);
  const savedJointCal = loadJointCalibration();
  let jointCal = $state<Record<string, JointCalibration> | null>(savedJointCal);
  let jointCalName = $state<string | null>(savedJointCal ? 'saved calibration' : null);
  let realError = $state<string | null>(null);
  let lastSent = 0;

  // ── pick + board detection ──────────────────────────────────────────────────
  let pickPhase = $state<PickPhase>('idle');
  let blockTag = $state(101);
  let gripperCmd = $state(-0.1745); // current gripper angle (rad)

  // Stage selector: '' = off (manual sliders), 'auto' = run the full sequence,
  // or a single phase to drive to and hold there.
  type Stage = '' | 'approach' | 'descend' | 'grasp' | 'lift';
  let stage = $state<Stage>('');
  // Last known sim position of the selected block (from detection).
  let pickBlock = $state<[number, number, number] | null>(null);
  // Configurable grasp offsets (m). Depth = descend/grasp height (can be negative).
  let graspDepth = $state(DEFAULT_PICK.graspZ);
  let graspX = $state(DEFAULT_PICK.graspXOffset);
  let graspY = $state(DEFAULT_PICK.graspYOffset);
  // Tag-centre → block-centre offset in the TAG's own frame (mm). Rotates with
  // the block, so it corrects for the tag not being at the block centre.
  let tagOffX = $state(0);
  let tagOffY = $state(0);
  // Constant rotation (degrees) between the block-tag frame and the block's
  // long axis. Adjust in 90° steps if the rendered block is turned relative to
  // the real one (see setBlockMocap).
  let blockYawOffsetDeg = $state(0);
  // Sim-only: position of the sample block to practice picking (sim metres).
  let simBlockX = $state(ROBOT_BASE_TIP_X + 0.15);
  let simBlockY = $state(0);
  let grabbed = false; // sim: block is currently held by the gripper
  // Finger-opening direction (deg) for the top-down grasp. Adjust to line the
  // gripper up with the block (0 and 180 are equivalent; 90 rotates it a quarter).
  let gripYawDeg = $state(0);
  let simBlockYawDeg = $state(0); // resting orientation of the sample block

  // Board perspective correction (from the Test-calibration tab's JSON).
  // Camera intrinsics (mandatory — from the ChArUco step). Reloaded when the
  // real-arm camera starts, so a fresh calibration is picked up without reload.
  let intrinsics = $state<Intrinsics | null>(loadIntrinsics());
  const TAG_MM = 16; // board border-tag size
  const BLOCK_MARKER_MM = 15; // block ArUco marker size (the black square, not the white border)
  // Onboard-camera shelf fiducial: a plain DICT_6X6_250 marker, ID outside the
  // board border range (0–63) and the object-tag range (100–199). Printed via
  // tags-and-borders/make_shelf_tag.py — SHELF_TAG_MM must match the printout.
  const SHELF_TAG_ID = 200;
  const SHELF_TAG_MM = 80; // printed black marker side (mm)
  let camW = 640;
  let camH = 480;

  // Latest detected block poses (interior mm X/Y + board-frame yaw) keyed by tag id.
  let detected = $state<Map<number, [number, number, number]>>(new Map());
  let detectMsg = $state<string | null>(null);

  // Camera detection plumbing (only runs in real mode once started).
  let cv: Cv | null = null;
  let detector: Detector | null = null;
  let video: HTMLVideoElement;
  let previewVideo = $state<HTMLVideoElement>();
  let camStream: MediaStream | null = null;
  let grabCtx: CanvasRenderingContext2D | null = null;
  let srcMat: any = null;
  let grayMat: any = null;
  let tagCentresMm: TagCentres = new Map();
  let lastDetect = 0;

  let session: Session | null = null;
  let solver: IKSolver | null = null;
  let renderer: MujocoRenderer | null = null;
  let mj: Awaited<ReturnType<typeof loadMujocoModule>> | null = null;

  // ── robot selection + mobile-base placement ─────────────────────────────────
  let selectedRobot = $state<RobotId>(loadRobotId() ?? 'so101');
  // Base position relative to the arm mount (m). Tuned live via the base mocap
  // body, so no remount is needed while dialing it in.
  const initOffset =
    loadArmOffset(selectedRobot) ?? robotById(selectedRobot).base?.defaultOffset ?? [0, 0, 0];
  // Whole-robot world pose (LeKiwi drives/turns around the sim).
  let robotX = $state(0);
  let robotY = $state(0);
  let robotYawDeg = $state(0);

  // Latest drive intent as amounts of the three primitive directions (forward,
  // back-left, back-right) plus rotation, each −1..1. applyBaseMotion turns it into
  // the sim pose; wheelSpeeds turns it into the real wheel command.
  let baseVel = { fwd: 0, bl: 0, br: 0, turn: 0 };
  const zeroBaseVel = () => { baseVel.fwd = 0; baseVel.bl = 0; baseVel.br = 0; baseVel.turn = 0; };

  // The drive triad sets which primitive directions are held.
  function setDrive(fwd: number, bl: number, br: number) {
    if (!hasBase || navigating || baseTesting) return; // auto-nav/calibration owns the base
    baseVel.fwd = fwd;
    baseVel.bl = bl;
    baseVel.br = br;
  }
  function turnRobot(direction: number) {
    if (!hasBase || navigating || baseTesting) return; // auto-nav/calibration owns the base
    baseVel.turn = direction;
  }

  // Integrate the drive intent into the sim robot pose (the sim is the ground truth
  // for what a command means; calibration makes the real base match it). Skipped
  // while navigating (open-loop; use "Reset MuJoCo position").
  function applyBaseMotion() {
    if (!hasBase || navigating) return;
    const D = PRIMITIVE_DIRS;
    // Sum the held primitives into a body velocity (vx forward, vy left).
    const vx = baseVel.fwd * D.forward[0] + baseVel.bl * D.backLeft[0] + baseVel.br * D.backRight[0];
    const vy = baseVel.fwd * D.forward[1] + baseVel.bl * D.backLeft[1] + baseVel.br * D.backRight[1];
    const wz = baseVel.turn;
    if (Math.abs(vx) < 1e-4 && Math.abs(vy) < 1e-4 && Math.abs(wz) < 1e-4) return;
    const STEP = 0.003; // m per frame at full deflection
    const ASTEP = 0.7; // deg per frame at full deflection
    const yaw = (robotYawDeg * Math.PI) / 180;
    const cs = Math.cos(yaw), sn = Math.sin(yaw);
    robotX += STEP * (cs * vx - sn * vy);
    robotY += STEP * (sn * vx + cs * vy);
    robotYawDeg += ASTEP * wz;
  }

  // ── LeKiwi base motors (real mode) ───────────────────────────────────────────
  // The 3 wheel servos share the arm's serial bus (IDs 7/8/9). On connect they're
  // switched to wheel mode; then each frame the sim base's motion (from the
  // joystick, rotation panel, or the shelf Drive stage) is converted to wheel
  // speeds and streamed — so the real base tracks whatever moves the sim base.
  let baseConfig = $state<BaseConfig>({ ...DEFAULT_BASE_CONFIG, ...(loadBaseConfig() ?? {}) });
  let baseConnected = $state(false);
  let baseError = $state<string | null>(null);
  let lastWheelSent = 0;
  let armSending = false; // an arm position write is in flight
  let wheelSending = false; // a wheel speed write is in flight
  // Onboard-camera navigation to the shelf tag.
  // Shelf tag in the camera frame: [x, y, z] metres + `sq` = how far off square-on
  // the tag face is (rad; 0 = viewing it head-on, sign = which way it's rotated).
  let shelfTagCam = $state<[number, number, number, number] | null>(null);
  let navigating = $state(false); // auto-driving toward the shelf tag
  let navStandoff = $state(0.3); // stop this far (m) from the tag
  let navSquareSign = $state(1); // flip if the square-up turns the wrong way
  // Simple test-and-flip direction calibration: pulse one body motion, watch the
  // real base vs the sim, flip that axis if they disagree.
  let baseTesting = $state(false);
  // Shelves (LeKiwi only): configurable count + independent per-drawer open amount.
  let shelfCount = $state(3);
  let shelfOpens = $state<number[]>([0, 0, 0]); // open amount (m) per drawer
  // Movable/adjustable shelf placement (world). Placed to the left, farther out.
  let shelfX = $state(-0.36);
  let shelfY = $state(0.6);
  let shelfYawDeg = $state(0);
  let shelfElevation = $state(0.15); // height of the unit's bottom off the floor
  // Manual, staged open/close so the behaviour can be stepped through and watched.
  // Position-only IK (no forced approach) + wrist_flex-up branch + roll 0, which
  // is the sequence that grabs the rod cleanly. The drawer slides under contact.
  let shelfSel = $state(0); // which drawer the stages act on
  type ShelfStage = '' | 'drive' | 'approach' | 'enter' | 'grip' | 'open' | 'close';
  let shelfStage = $state<ShelfStage>('');
  const SHELF_STANDOFF = 0.32; // m from arm base to the closed rod (Drive stage)
  const SHELF_APPROACH = 0.13; // m in front of the rod for the Approach stage
  let shelfQadr: number[] = []; // qpos address of each drawer slide joint
  let shelfMocapId = -1;
  let loadedShelfCount = 0; // shelf count baked into the current scene
  let baseOffX = $state(initOffset[0]);
  let baseOffY = $state(initOffset[1]);
  let baseOffZ = $state(initOffset[2]);
  // The arm's mount is rotated on the base within the base plane; this yaws it
  // about vertical (keeps it level) so it faces the expected direction.
  let armYawDeg = $state(-60);
  let hasBase = $state(false); // true once a mobile base is mounted in the scene
  let armLift = 0; // how far the arm is raised on the base (m) so wheels touch the floor
  // Mocap body indices, resolved by name each load (order-independent). -1 = absent.
  let blockMocapId = 0;
  let baseMocapId = -1;
  let armMocapId = -1;
  let raf = 0;

  // ── sim-mode physics (block is a real free body, arm driven by actuators) ────
  let physics = false; // current scene is the physics (sim-only) variant
  let loadedMode: 'sim' | 'real' | null = null; // which mode the scene was built for
  let graspSiteId = -1;
  let blockQadr = -1; // free-block qpos address (pos[3] + quat[4])
  let blockDadr = -1; // free-block dof address (lin[3] + ang[3])
  // Relative transform of the block in the grasp-site frame, captured on grab.
  let relPos: [number, number, number] = [0, 0, 0];
  let relQuat: [number, number, number, number] = [1, 0, 0, 0];

  /** Resolve a body's mocap index by name (-1 if the body is absent/non-mocap). */
  function mocapIdOf(name: string): number {
    if (!session) return -1;
    const bid = session.mj.mj_name2id(session.model, 1 /* mjOBJ_BODY */, name);
    if (bid < 0) return -1;
    return (session.model.body_mocapid as Int32Array)[bid];
  }

  function setBlockMocap(pos: [number, number, number], boardYaw = 0) {
    if (!session || blockMocapId < 0) return;
    const i = blockMocapId;
    const mp = session.data.mocap_pos as Float64Array;
    mp[i * 3] = pos[0];
    mp[i * 3 + 1] = pos[1];
    mp[i * 3 + 2] = pos[2];
    // Board-frame yaw → sim-frame yaw. interiorToSim maps board (x,y) → sim
    // (simX=-y, simY=-x), so a board-plane direction (cos,sin) becomes
    // (-sin,-cos) in sim; rotate the block's mocap quaternion about world Z.
    const simYaw =
      Math.atan2(-Math.cos(boardYaw), -Math.sin(boardYaw)) + (blockYawOffsetDeg * Math.PI) / 180;
    const mq = session.data.mocap_quat as Float64Array;
    mq[i * 4] = Math.cos(simYaw / 2);
    mq[i * 4 + 1] = 0;
    mq[i * 4 + 2] = 0;
    mq[i * 4 + 3] = Math.sin(simYaw / 2);
  }

  // Place the whole mobile robot in the world (both mocap bodies): the base is
  // planted at [robotX, robotY] with its wheels on the floor and yawed by
  // robotYawDeg; the arm sits on top, shifted by the x/y/z arm-on-base offset
  // (rotated with the robot). Lets the LeKiwi drive/turn around in the sim.
  function placeRobot() {
    if (!session) return;
    const psi = (robotYawDeg * Math.PI) / 180;
    const cs = Math.cos(psi), sn = Math.sin(psi);
    const yaw: number[] = [Math.cos(psi / 2), 0, 0, Math.sin(psi / 2)];
    const mp = session.data.mocap_pos as Float64Array;
    const mq = session.data.mocap_quat as Float64Array;
    if (baseMocapId >= 0) {
      const i = baseMocapId;
      mp[i * 3] = robotX;
      mp[i * 3 + 1] = robotY;
      mp[i * 3 + 2] = armLift;
      const bq = qMul(yaw, [0, 1, 0, 0]); // world yaw ∘ mount flip
      mq[i * 4] = bq[0];
      mq[i * 4 + 1] = bq[1];
      mq[i * 4 + 2] = bq[2];
      mq[i * 4 + 3] = bq[3];
    }
    if (armMocapId >= 0) {
      const j = armMocapId;
      const ox = baseOffX * cs - baseOffY * sn;
      const oy = baseOffX * sn + baseOffY * cs;
      mp[j * 3] = robotX + ox;
      mp[j * 3 + 1] = robotY + oy;
      mp[j * 3 + 2] = armLift + baseOffZ;
      // world yaw + mount rotation, both about vertical (arm stays level, just
      // rotated within the base plane).
      const ap = psi + (armYawDeg * Math.PI) / 180;
      mq[j * 4] = Math.cos(ap / 2);
      mq[j * 4 + 1] = 0;
      mq[j * 4 + 2] = 0;
      mq[j * 4 + 3] = Math.sin(ap / 2);
    }
  }

  // ── small quaternion / rotation helpers (row-major 3×3, quats w,x,y,z) ───────
  function rotVec(m: Float64Array, o: number, v: number[]): [number, number, number] {
    return [
      m[o] * v[0] + m[o + 1] * v[1] + m[o + 2] * v[2],
      m[o + 3] * v[0] + m[o + 4] * v[1] + m[o + 5] * v[2],
      m[o + 6] * v[0] + m[o + 7] * v[1] + m[o + 8] * v[2],
    ];
  }
  function rotVecT(m: Float64Array, o: number, v: number[]): [number, number, number] {
    return [
      m[o] * v[0] + m[o + 3] * v[1] + m[o + 6] * v[2],
      m[o + 1] * v[0] + m[o + 4] * v[1] + m[o + 7] * v[2],
      m[o + 2] * v[0] + m[o + 5] * v[1] + m[o + 8] * v[2],
    ];
  }
  function mat3ToQuat(m: Float64Array, o: number): [number, number, number, number] {
    const t = m[o] + m[o + 4] + m[o + 8];
    let w, x, y, z;
    if (t > 0) {
      const s = Math.sqrt(t + 1) * 2;
      w = 0.25 * s;
      x = (m[o + 7] - m[o + 5]) / s;
      y = (m[o + 2] - m[o + 6]) / s;
      z = (m[o + 3] - m[o + 1]) / s;
    } else if (m[o] > m[o + 4] && m[o] > m[o + 8]) {
      const s = Math.sqrt(1 + m[o] - m[o + 4] - m[o + 8]) * 2;
      w = (m[o + 7] - m[o + 5]) / s;
      x = 0.25 * s;
      y = (m[o + 1] + m[o + 3]) / s;
      z = (m[o + 2] + m[o + 6]) / s;
    } else if (m[o + 4] > m[o + 8]) {
      const s = Math.sqrt(1 + m[o + 4] - m[o] - m[o + 8]) * 2;
      w = (m[o + 2] - m[o + 6]) / s;
      x = (m[o + 1] + m[o + 3]) / s;
      y = 0.25 * s;
      z = (m[o + 5] + m[o + 7]) / s;
    } else {
      const s = Math.sqrt(1 + m[o + 8] - m[o] - m[o + 4]) * 2;
      w = (m[o + 3] - m[o + 1]) / s;
      x = (m[o + 2] + m[o + 6]) / s;
      y = (m[o + 5] + m[o + 7]) / s;
      z = 0.25 * s;
    }
    return [w, x, y, z];
  }
  /**
   * Gripper world rotation (row-major 3×3, columns = local x/y/z) from an
   * approach direction (pitch 90° = straight down, yaw = azimuth) + roll about
   * the approach axis. Local X is the approach the cone points along.
   */
  function rpyToRd(rollDeg: number, pitchDeg: number, yawDeg: number): number[] {
    const r = (rollDeg * Math.PI) / 180, p = (pitchDeg * Math.PI) / 180, y = (yawDeg * Math.PI) / 180;
    const a = [Math.cos(p) * Math.cos(y), Math.cos(p) * Math.sin(y), -Math.sin(p)];
    let up = Math.abs(a[2]) > 0.999 ? [1, 0, 0] : [0, 0, 1];
    const cross = (u: number[], v: number[]) => [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    const norm = (v: number[]) => { const n = Math.hypot(v[0], v[1], v[2]) || 1; return [v[0] / n, v[1] / n, v[2] / n]; };
    const y0 = norm(cross(up, a));
    const z0 = cross(a, y0);
    const cr = Math.cos(r), sr = Math.sin(r);
    const yv = [y0[0] * cr + z0[0] * sr, y0[1] * cr + z0[1] * sr, y0[2] * cr + z0[2] * sr];
    const zv = [-y0[0] * sr + z0[0] * cr, -y0[1] * sr + z0[1] * cr, -y0[2] * sr + z0[2] * cr];
    return [a[0], yv[0], zv[0], a[1], yv[1], zv[1], a[2], yv[2], zv[2]];
  }

  /** Best-effort grasp orientation for the manual target (top / 45 / front). */
  function graspRd(): number[] {
    // Approach azimuth: from the arm base toward the target (used by 45 & front).
    const bx = hasBase ? robotX : 0, by = hasBase ? robotY : 0;
    const dx = target[0] - bx, dy = target[1] - by;
    const yawDeg =
      Math.hypot(dx, dy) < 1e-4 ? robotYawDeg : (Math.atan2(dy, dx) * 180) / Math.PI;
    const pitch = graspMode === 'top' ? 90 : graspMode === '45' ? 45 : 0;
    return rpyToRd(graspRollDeg, pitch, yawDeg);
  }

  const qConj = (q: number[]): [number, number, number, number] => [q[0], -q[1], -q[2], -q[3]];
  function qMul(a: number[], b: number[]): [number, number, number, number] {
    return [
      a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ];
  }

  /** Read each drawer's actual (physics-driven) open amount for display. */
  function readShelfOpens() {
    if (!session || shelfQadr.length === 0) return;
    const q = session.data.qpos as Float64Array;
    for (let i = 0; i < shelfQadr.length; i++) shelfOpens[i] = q[shelfQadr[i]];
  }

  /** Set the shelf unit's live pose (mocap). */
  function placeShelf() {
    if (!session || shelfMocapId < 0) return;
    const i = shelfMocapId;
    const th = (shelfYawDeg * Math.PI) / 180;
    const mp = session.data.mocap_pos as Float64Array;
    mp[i * 3] = shelfX;
    mp[i * 3 + 1] = shelfY;
    mp[i * 3 + 2] = shelfElevation;
    const mq = session.data.mocap_quat as Float64Array;
    mq[i * 4] = Math.cos(th / 2);
    mq[i * 4 + 1] = 0;
    mq[i * 4 + 2] = 0;
    mq[i * 4 + 3] = Math.sin(th / 2);
  }

  /** Drawer `i` rod position in world (m) at `open`. */
  function rodWorld(i: number, open: number): [number, number, number] {
    const [lx, ly, lz] = rodLocal(i, open);
    const th = (shelfYawDeg * Math.PI) / 180;
    const c = Math.cos(th), s = Math.sin(th);
    return [shelfX + c * lx - s * ly, shelfY + s * lx + c * ly, shelfElevation + lz];
  }
  /** Shelf outward normal (world XY), pointing toward where the robot stands. */
  function shelfNormal(): [number, number] {
    const th = (shelfYawDeg * Math.PI) / 180;
    return [Math.sin(th), -Math.cos(th)];
  }

  /** Drawer `i`'s current (physics) open amount. */
  function drawerOpen(i: number): number {
    return session && shelfQadr[i] != null ? (session.data.qpos as Float64Array)[shelfQadr[i]] : 0;
  }

  // Arm target + gripper for a shelf stage (position-only; the loop handles the
  // free/branch/roll options). Mirrors the manual sequence:
  //   approach → hover in front of the rod, gripper open
  //   enter    → move onto the rod, still open
  //   grip     → same spot, close the gripper on the rod
  //   open     → drag the rod to the fully-open position (gripper closed)
  //   close    → push the rod back to the closed position (gripper closed)
  function shelfStageTarget(i: number, stage: ShelfStage): {
    target: [number, number, number];
    grip: number;
  } {
    const n = shelfNormal();
    const open = DEFAULT_PICK.gripperOpen, close = DEFAULT_PICK.gripperClose;
    const rod = rodWorld(i, drawerOpen(i));
    switch (stage) {
      case 'approach':
        return { target: [rod[0] + n[0] * SHELF_APPROACH, rod[1] + n[1] * SHELF_APPROACH, rod[2]], grip: open };
      case 'enter':
        return { target: rod, grip: open };
      case 'grip':
        return { target: rod, grip: close };
      case 'open':
        return { target: rodWorld(i, SHELF.openLimit), grip: close };
      case 'close':
        return { target: rodWorld(i, 0), grip: close };
      default:
        return { target: rod, grip: open };
    }
  }

  // Drive stage: ease the base to a standoff pose facing the drawer; carry the arm.
  function shelfDriveTarget(i: number): { target: [number, number, number]; grip: number } {
    const n = shelfNormal();
    const rc = rodWorld(i, 0);
    const tx = rc[0] + n[0] * SHELF_STANDOFF, ty = rc[1] + n[1] * SHELF_STANDOFF;
    const baseYaw = (Math.atan2(-n[1], -n[0]) * 180) / Math.PI;
    const dx = tx - robotX, dy = ty - robotY;
    let dyaw = baseYaw - robotYawDeg;
    while (dyaw > 180) dyaw -= 360;
    while (dyaw < -180) dyaw += 360;
    const sp = 0.004, sa = 1.2;
    robotX += Math.abs(dx) < sp ? dx : Math.sign(dx) * sp;
    robotY += Math.abs(dy) < sp ? dy : Math.sign(dy) * sp;
    robotYawDeg += Math.abs(dyaw) < sa ? dyaw : Math.sign(dyaw) * sa;
    const yaw = (robotYawDeg * Math.PI) / 180;
    return {
      target: [robotX + 0.18 * Math.cos(yaw), robotY + 0.18 * Math.sin(yaw), armLift + 0.12],
      grip: DEFAULT_PICK.gripperOpen,
    };
  }

  /** Reset the physics block to a resting pose at [x, y], rotated `yawDeg`. */
  function resetPhysicsBlock(x: number, y: number, yawDeg: number) {
    if (!session || blockQadr < 0) return;
    const h = (yawDeg * Math.PI) / 180 / 2;
    const q = session.data.qpos as Float64Array;
    q[blockQadr] = x;
    q[blockQadr + 1] = y;
    q[blockQadr + 2] = BLOCK_HALF_Z;
    q[blockQadr + 3] = Math.cos(h);
    q[blockQadr + 4] = 0;
    q[blockQadr + 5] = 0;
    q[blockQadr + 6] = Math.sin(h);
    const v = session.data.qvel as Float64Array;
    for (let k = 0; k < 6; k++) v[blockDadr + k] = 0;
    grabbed = false;
  }

  // Advance the physics sim one frame: drive the arm actuators toward the IK
  // solution (computed on a snapshot so it doesn't disturb the live state),
  // step, then hold the block rigidly if it's grasped.
  function stepPhysics(
    solveTarget: [number, number, number],
    Rd: number[],
    grip: number,
    opts: { free?: boolean; rollRad?: number; dofLimits?: Record<number, [number, number]> } = {},
  ) {
    if (!session || !solver || !mj) return;
    const d = session.data;
    const nq = session.model.nq as number;
    const nv = session.model.nv as number;
    const qsave = Float64Array.from((d.qpos as Float64Array).subarray(0, nq));
    const vsave = Float64Array.from((d.qvel as Float64Array).subarray(0, nv));
    const ctrl = d.ctrl as Float64Array;
    if (opts.free) {
      // "None": don't constrain the approach — fix the wrist roll, reach the
      // position with the other four joints. Often reaches where a full pose can't.
      const q = d.qpos as Float64Array;
      const roll = opts.rollRad ?? q[4];
      q[4] = roll;
      const res = solver.solve(solveTarget, { dofIndices: [0, 1, 2, 3], maxIters: 12, dofLimits: opts.dofLimits });
      ikOk = res.ok;
      ikErrMm = res.error * 1000;
      (d.qpos as Float64Array).set(qsave);
      (d.qvel as Float64Array).set(vsave);
      for (let i = 0; i < 4; i++) ctrl[i] = res.qpos[i];
      ctrl[4] = roll;
    } else {
      // Low orientation weight → best-effort: reach the position, approximate the
      // approach angle (a slight tilt is fine if it reaches better).
      const res = solver.solvePose(solveTarget, Rd, { dofIndices: ARM_DOFS, maxIters: 10, oriWeight: 0.25, dofLimits: opts.dofLimits });
      ikOk = res.ok;
      ikErrMm = res.error * 1000;
      (d.qpos as Float64Array).set(qsave);
      (d.qvel as Float64Array).set(vsave);
      for (let i = 0; i < 5; i++) ctrl[i] = res.qpos[i];
    }
    // Gripper: same open/close flip the renderer uses, so a "close" command
    // physically closes the model's fingers.
    ctrl[5] = gripperRange[0] + gripperRange[1] - grip;

    mj.mj_step(session.model, d);
    jointAngles = ARM_DOFS.map((i) => (d.qpos as Float64Array)[i]);

    updateGrab();
    readShelfOpens();
    mj.mj_forward(session.model, d);
  }

  // Grab/hold logic: latch when the closed gripper is at the block; while held,
  // pin the block to the grasp site at the captured relative transform.
  function updateGrab() {
    if (!session || blockQadr < 0) return;
    const d = session.data;
    const q = d.qpos as Float64Array;
    const sx = d.site_xpos as Float64Array;
    const sm = d.site_xmat as Float64Array;
    const so = graspSiteId * 3;
    const mo = graspSiteId * 9;
    const closing = stage === 'grasp' || stage === 'lift';
    if (!closing) {
      grabbed = false;
      return;
    }
    if (!grabbed) {
      // The grasp site sits ~ikZOffset above the fingers, so trigger on IK
      // "reached" (fingers at the block) rather than site-to-block distance.
      if (!ikOk) return;
      // Capture relative transform (block in grasp-site frame).
      relPos = rotVecT(sm, mo, [
        q[blockQadr] - sx[so],
        q[blockQadr + 1] - sx[so + 1],
        q[blockQadr + 2] - sx[so + 2],
      ]);
      const bq = [q[blockQadr + 3], q[blockQadr + 4], q[blockQadr + 5], q[blockQadr + 6]];
      relQuat = qMul(qConj(mat3ToQuat(sm, mo)), bq);
      grabbed = true;
    }
    // Hold: block = graspSite ∘ rel.
    const wp = rotVec(sm, mo, relPos);
    q[blockQadr] = sx[so] + wp[0];
    q[blockQadr + 1] = sx[so + 1] + wp[1];
    q[blockQadr + 2] = sx[so + 2] + wp[2];
    const wq = qMul(mat3ToQuat(sm, mo), relQuat);
    q[blockQadr + 3] = wq[0];
    q[blockQadr + 4] = wq[1];
    q[blockQadr + 5] = wq[2];
    q[blockQadr + 6] = wq[3];
    const v = d.qvel as Float64Array;
    for (let k = 0; k < 6; k++) v[blockDadr + k] = 0;
  }

  /**
   * Fetch + mount the selected robot's scene, replacing any current one. On
   * mobile robots the arm root is a mocap body, so the x/y/z placement offset
   * moves the arm LIVE (via setArmMocap) with no remount; the base stays planted.
   */
  async function loadScene(robotId: RobotId) {
    if (!mj) return;
    ready = false;
    status = 'Loading model…';
    renderer?.dispose();
    renderer = null;
    session?.dispose();
    session = null;

    const robot = robotById(robotId);
    let armXml = await (await fetch(`/models/so101/${XML}`)).text();
    const files: ModelFiles = {};
    for (const name of ASSETS) {
      const res = await fetch(`/models/so101/assets/${name}`);
      files[`assets/${name}`] = new Uint8Array(await res.arrayBuffer());
    }

    hasBase = false;
    armLift = 0;
    let manifest: BaseManifest | null = null;
    if (robot.base) {
      manifest = (await (
        await fetch(`/models/${robot.base.dir}/${robot.base.manifest}`)
      ).json()) as BaseManifest;
      // Raise the arm so the base (which hangs below the mount) rests its wheels
      // on the floor (z=0); board + block stay on the ground.
      armLift = manifest.baseDrop;
      // Make the arm root a mocap body so its placement can be tuned live.
      armXml = armXml.replace('<body name="base" pos="0 0 0"', '<body name="base" mocap="true" pos="0 0 0"');
    }

    // In sim-only mode the block is a real physics free body; in real mode it's
    // a mocap body mirroring detection.
    physics = mode === 'sim';
    loadedMode = mode;
    let sceneXml = buildBoardSceneXml(armXml, { physics });
    if (robot.base && manifest) {
      sceneXml = injectBase(sceneXml, manifest);
      for (const m of manifest.meshes) {
        const res = await fetch(`/models/${robot.base.dir}/assets/${m.file}`);
        files[`assets/${m.file}`] = new Uint8Array(await res.arrayBuffer());
      }
      hasBase = true;
      // Shelves are part of the mobile-robot scene (LeKiwi only).
      sceneXml = injectShelves(sceneXml, shelfCount);
      loadedShelfCount = shelfCount;
    }
    files['scene.xml'] = new TextEncoder().encode(sceneXml);

    session = mountModel(mj, files, 'scene.xml');
    solver = new IKSolver(mj, session.model, session.data, GRASP_SITE);
    blockMocapId = physics ? -1 : mocapIdOf('block');
    baseMocapId = hasBase ? mocapIdOf('lekiwi_base') : -1;
    armMocapId = hasBase ? mocapIdOf('base') : -1;
    graspSiteId = mj.mj_name2id(session.model, 6 /* mjOBJ_SITE */, GRASP_SITE);
    if (physics) {
      const jid = mj.mj_name2id(session.model, 3 /* mjOBJ_JOINT */, 'block_free');
      blockQadr = jid >= 0 ? (session.model.jnt_qposadr as Int32Array)[jid] : -1;
      blockDadr = jid >= 0 ? (session.model.jnt_dofadr as Int32Array)[jid] : -1;
      grabbed = false;
      resetPhysicsBlock(simBlockX, simBlockY, simBlockYawDeg);
    } else {
      blockQadr = -1;
      blockDadr = -1;
    }
    // Resolve drawer slide-joint addresses (LeKiwi shelves) + size the open array.
    shelfQadr = [];
    shelfStage = '';
    shelfMocapId = -1;
    if (hasBase) {
      shelfOpens = Array.from({ length: shelfCount }, (_, i) => shelfOpens[i] ?? 0);
      shelfMocapId = mocapIdOf('shelf_unit');
      for (let i = 0; i < shelfCount; i++) {
        const jid = mj.mj_name2id(session.model, 3 /* mjOBJ_JOINT */, shelfJointName(i));
        if (jid >= 0) shelfQadr.push((session.model.jnt_qposadr as Int32Array)[jid]);
      }
    }
    if (hasBase) {
      placeRobot();
      placeShelf();
      readShelfOpens();
    }
    session.forward();
    target = solver.sitePosition();

    // Gripper ctrlrange (joint index 5) for the gripper slider.
    const cr = Array.from(session.model.actuator_ctrlrange as ArrayLike<number>);
    if (cr[10] < cr[11]) gripperRange = [cr[10], cr[11]];

    renderer = new MujocoRenderer(canvas, mj, session.model, session.data);
    renderer.setTarget(target);
    fitCanvas();
    ready = true;
    status = 'Ready';
  }

  onMount(async () => {
    try {
      mj = await loadMujocoModule();
      await loadScene(selectedRobot);
      loop();
    } catch (e) {
      errorMsg = e instanceof Error ? e.message : String(e);
      status = 'Failed to load';
    }
  });

  async function onRobotChange(id: RobotId) {
    if (id === selectedRobot && session) return;
    selectedRobot = id;
    saveRobotId(id);
    const off = loadArmOffset(id) ?? robotById(id).base?.defaultOffset ?? [0, 0, 0];
    baseOffX = off[0];
    baseOffY = off[1];
    baseOffZ = off[2];
    try {
      cancelAnimationFrame(raf);
      await loadScene(id);
      loop();
    } catch (e) {
      errorMsg = e instanceof Error ? e.message : String(e);
      status = 'Failed to load';
    }
  }

  // Tune the arm-on-base placement + drive the robot around live (no remount).
  $effect(() => {
    const x = baseOffX, y = baseOffY, z = baseOffZ;
    const rx = robotX, ry = robotY, rψ = robotYawDeg, ayaw = armYawDeg;
    void rx; void ry; void rψ; void ayaw; // tracked so the robot re-places on change
    if (hasBase && session) {
      placeRobot();
      saveArmOffset(selectedRobot, [x, y, z]);
    }
  });

  function fitCanvas() {
    if (!renderer || !wrap) return;
    const w = wrap.clientWidth;
    const h = Math.round(w * 0.66);
    renderer.resize(w, h);
  }

  // Park the arm at HOME instead of chasing the EE target. In sim the position
  // actuators ease it there; in real mode qpos is set and streamed to the servos.
  function restArm() {
    if (!session || !mj || !renderer) return;
    const d = session.data;
    const q = d.qpos as Float64Array;
    jointAngles = HOME_POSE.slice();
    if (physics) {
      const ctrl = d.ctrl as Float64Array;
      for (let i = 0; i < 5; i++) ctrl[i] = HOME_POSE[i];
      ctrl[5] = gripperRange[0] + gripperRange[1] - gripperCmd;
      mj.mj_step(session.model, d);
      readShelfOpens();
      mj.mj_forward(session.model, d);
    } else {
      for (let i = 0; i < 5; i++) q[i] = HOME_POSE[i];
      const solvedRoll = q[4];
      q[4] = solvedRoll + WRIST_ROLL_OFFSET; // display-only wrist_roll fixup
      q[5] = gripperRange[0] + gripperRange[1] - gripperCmd;
      readShelfOpens();
      session.forward();
      q[4] = solvedRoll;
      q[5] = gripperCmd;
    }
    renderer.setTarget(target, undefined); // no active EE target while resting
    renderer.update();
  }

  function loop() {
    if (!solver || !renderer || !session) return;
    const q = session.data.qpos as Float64Array;

    // Rest mode (manual toggle, or auto while the base navigates): park the arm at
    // HOME and skip IK entirely, then run the usual detect/nav/stream tail.
    if (armRest || navigating) {
      restArm();
      maybeDetect();
      maybeNavigate();
      applyBaseMotion();
      maybeStreamToRobot();
      maybeStreamBase();
      raf = requestAnimationFrame(loop);
      return;
    }

    // Default (manual): position + a best-effort grasp orientation (top/45/front).
    let Rd = graspRd();
    let solveTarget = target;
    let grip = gripperCmd;
    let manual = true;
    let shelfActive = false;

    if (shelfStage !== '' && hasBase) {
      // Manual staged shelf open/close: position-only IK, wrist-up branch, roll 0.
      // Runs in both sim and real mode — in real mode the same targets drive the
      // arm servos (and the Drive stage's base motion drives the wheels).
      const r = shelfStage === 'drive' ? shelfDriveTarget(shelfSel) : shelfStageTarget(shelfSel, shelfStage);
      solveTarget = r.target;
      grip = r.grip;
      gripperCmd = grip;
      target = solveTarget;
      pickPhase = 'idle';
      manual = false;
      shelfActive = true;
    } else if (stage !== '' && pickBlock) {
      // A selected stage overrides the manual target with that phase's target,
      // top-down (fingers at gripYaw).
      const s = phaseTarget(pickBlock, stage, {
        ...DEFAULT_PICK,
        graspZ: graspDepth,
        graspXOffset: graspX,
        graspYOffset: graspY,
      });
      const phi = (gripYawDeg * Math.PI) / 180;
      Rd = [0, Math.cos(phi), Math.sin(phi), 0, Math.sin(phi), -Math.cos(phi), -1, 0, 0];
      solveTarget = s.target;
      grip = s.gripper;
      gripperCmd = s.gripper;
      pickPhase = stage;
      target = s.target;
      manual = false;
    } else if (stage === '') {
      pickPhase = 'idle';
    }

    // Position-only IK (approach free) for the shelf stages, and for the manual
    // "None" grasp mode. Wrist roll is fixed (0 for the shelf, the roll slider else).
    const free = shelfActive || (manual && graspMode === 'none');
    const rollRad = shelfActive ? 0 : (graspRollDeg * Math.PI) / 180;
    // Prefer an IK branch by clamping wrist_flex (dof 3): the shelf always uses the
    // wrist-up branch; otherwise follow the Direction control.
    const branch = shelfActive ? 'top' : armBranch;
    const dofLimits =
      branch === 'top'
        ? { 3: [0, WRIST_FLEX_HI] as [number, number] }
        : branch === 'bottom'
          ? { 3: [-WRIST_FLEX_HI, 0] as [number, number] }
          : undefined;

    if (physics) {
      // Sim-only: real dynamics — drive actuators to the IK solution, step, and
      // hold the block if grasped. The block falls/rests/can be pushed on its own.
      stepPhysics(solveTarget, Rd, grip, { free, rollRad, dofLimits });
      renderer.setTarget(solveTarget, free ? undefined : Rd);
      renderer.update();
    } else {
      // Kinematic path (real mode): solve IK, write the solution into qpos for
      // display, and stream the solved joints to the servos. Honors the same
      // free / orientation / branch choices as the physics path, so the grasp
      // modes and shelf stages behave identically to sim-only mode.
      let res;
      if (free) {
        q[4] = rollRad; // fix wrist roll; reach position with the other four joints
        res = solver.solve(solveTarget, { dofIndices: [0, 1, 2, 3], maxIters: 12, dofLimits });
      } else {
        res = solver.solvePose(solveTarget, Rd, { dofIndices: ARM_DOFS, maxIters: 10, oriWeight: 0.25, dofLimits });
      }
      ikOk = res.ok;
      ikErrMm = res.error * 1000;
      jointAngles = ARM_DOFS.map((i) => q[i]); // the solved joints (streamed as-is)

      // Display-only fixups (restored afterwards so they don't affect IK or the
      // streamed command): wrist_roll gets +90°, and the gripper open/close is
      // flipped because the model's gripper geometry renders inverted vs the real arm.
      const solvedRoll = q[4];
      q[4] = solvedRoll + WRIST_ROLL_OFFSET;
      q[5] = gripperRange[0] + gripperRange[1] - gripperCmd;
      readShelfOpens();
      session.forward();
      renderer.setTarget(solveTarget, free ? undefined : Rd);
      renderer.update();
      q[4] = solvedRoll;
      q[5] = gripperCmd;
    }

    maybeDetect();
    maybeNavigate();
    applyBaseMotion();
    maybeStreamToRobot();
    maybeStreamBase();
    raf = requestAnimationFrame(loop);
  }

  async function maybeStreamToRobot() {
    if (mode !== 'real' || !realConnected || !jointCal || armSending) return;
    const now = performance.now();
    if (now - lastSent < 66) return; // ~15 Hz
    lastSent = now;
    const targets = new Map<number, number>();
    // Arm joints from IK, gripper from the pick/gripper command.
    CALIBRATION_PLAN.forEach((def, i) => {
      const cal = jointCal![def.joint];
      if (!cal) return;
      const angle = i < 5 ? jointAngles[i] : gripperCmd;
      const servo = simRadToServo(angle, cal);
      if (servo != null) targets.set(def.servoId, servo);
    });
    if (targets.size) {
      // Guard against overlapping frames: skip until this write settles, so the
      // bus queue never backs up (the write is serialized in Robot regardless).
      armSending = true;
      try {
        await robot.syncWritePositions(targets);
      } catch (e) {
        realError = e instanceof Error ? e.message : String(e);
      } finally {
        armSending = false;
      }
    }
  }

  // ── Camera detection (real mode) ────────────────────────────────────────────
  async function startCamera() {
    if (camStream) return; // already acquired
    intrinsics = loadIntrinsics(); // pick up the latest calibration
    try {
      if (!cv) {
        cv = await loadCv();
        detector = createDetector(cv);
        tagCentresMm = boardTagCentres(SQUARE_MM, 16, 2, 10, 8);
      }
      camStream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: 'environment' },
      });
      video.srcObject = camStream;
      await video.play();
      if (previewVideo) {
        previewVideo.srcObject = camStream; // visible preview shares the stream
        previewVideo.play().catch(() => {});
      }
      const w = video.videoWidth || 640;
      const h = video.videoHeight || 480;
      camW = w;
      camH = h;
      const grab = document.createElement('canvas');
      grab.width = w;
      grab.height = h;
      grabCtx = grab.getContext('2d', { willReadFrequently: true });
      srcMat = new cv.Mat(h, w, cv.CV_8UC4);
      grayMat = new cv.Mat();
    } catch (e) {
      detectMsg = 'Camera error: ' + (e instanceof Error ? e.message : String(e));
    }
  }

  function stopCamera() {
    camStream?.getTracks().forEach((t) => t.stop());
    camStream = null;
    try {
      srcMat?.delete();
      grayMat?.delete();
    } catch {
      /* ignore */
    }
    srcMat = null;
    grayMat = null;
    grabCtx = null;
  }

  function maybeDetect() {
    if (mode !== 'real' || !cv || !detector || !grabCtx || !srcMat || !session || !intrinsics) return;
    const now = performance.now();
    if (now - lastDetect < 100) return; // ~10 Hz
    lastDetect = now;

    const { width, height } = grabCtx.canvas;
    grabCtx.drawImage(video, 0, 0, width, height);
    srcMat.data.set(grabCtx.getImageData(0, 0, width, height).data);
    cv.cvtColor(srcMat, grayMat, cv.COLOR_RGBA2GRAY);
    const corners = detectMarkers(cv, detector, grayMat);

    // Onboard-camera shelf fiducial: its pose in the camera frame (m). Independent
    // of the board — the onboard camera sees the shelf, not necessarily the board.
    const st = corners.get(SHELF_TAG_ID);
    if (st) {
      const mp = solvePnpMarkerPose(cv, st, SHELF_TAG_MM, camW, camH, intrinsics);
      if (mp) {
        // Marker's outward normal (its local +Z) in camera coords = 3rd column of R.
        // Square-on it points back at the camera ≈ (0,0,−1); its horizontal tilt is
        // how far off-square we're viewing the shelf face.
        const nx = mp.R[2], nz = mp.R[8];
        const sq = Math.atan2(nx, -nz);
        shelfTagCam = [mp.t[0] / 1000, mp.t[1] / 1000, mp.t[2] / 1000, sq];
      } else {
        shelfTagCam = null;
      }
    } else {
      shelfTagCam = null;
    }

    // Board pose (camera→board) from border tags, then each block via solvePnP.
    const pose = boardPoseFromTags(cv, corners, tagCentresMm, TAG_MM, intrinsics, camW, camH);
    const m = new Map<number, [number, number, number]>();
    if (pose) {
      for (const [id, c] of corners) {
        if (tagCentresMm.has(id) || !BORDERED_IDS.has(id)) continue;
        const b = blockBoardXY(cv, c, BLOCK_MARKER_MM, intrinsics, camW, camH, pose, [tagOffX, tagOffY]);
        // Store as interior mm (board mm minus the border inset) + board-frame yaw.
        if (b) m.set(id, [b.x - INSET_MM, b.y - INSET_MM, b.yaw]);
      }
    }
    detected = m;
    const d = m.get(blockTag);
    if (d) {
      pickBlock = interiorToSim(d[0], d[1]);
      setBlockMocap(pickBlock, d[2]);
    }
  }

  function onStageChange(next: Stage) {
    detectMsg = null;
    if (next === '') {
      stage = '';
      pickPhase = 'idle';
      return;
    }
    if (mode === 'sim') {
      // Practice pick: target the block's live position (it may have been nudged).
      if (physics && session && blockQadr >= 0) {
        const q = session.data.qpos as Float64Array;
        pickBlock = [q[blockQadr], q[blockQadr + 1], BLOCK_HALF_Z];
      } else {
        pickBlock = [simBlockX, simBlockY, BLOCK_HALF_Z];
      }
      stage = next;
      return;
    }
    // Real mode: need a camera-detected block position to target.
    const d = detected.get(blockTag);
    if (d) pickBlock = interiorToSim(d[0], d[1]);
    if (!pickBlock) {
      detectMsg = `Block ${blockTag} not detected on the board.`;
      stage = '';
      return;
    }
    stage = next;
  }

  async function connectRobot() {
    realError = null;
    try {
      await robot.connect();
      realConnected = true;
      await robot.setTorque(
        CALIBRATION_PLAN.map((j) => j.servoId),
        true,
      );
    } catch (e) {
      realError = e instanceof Error ? e.message : String(e);
    }
  }

  async function disconnectRobot() {
    if (baseConnected) await disconnectBase().catch(() => {});
    try {
      await robot.setTorque(CALIBRATION_PLAN.map((j) => j.servoId), false);
    } catch {
      /* ignore */
    }
    await robot.disconnect().catch(() => {});
    realConnected = false;
  }

  // ── Base motors ──────────────────────────────────────────────────────────────
  // Wheels share the arm bus, so require the arm to be connected first, then just
  // flip the three wheel servos into wheel (continuous-rotation) mode.
  async function connectBase() {
    baseError = null;
    if (!realConnected) {
      baseError = 'Connect the arm servos first (same bus).';
      return;
    }
    try {
      await robot.setWheelMode([...baseConfig.wheelIds]);
      baseConnected = true;
    } catch (e) {
      baseError = e instanceof Error ? e.message : String(e);
    }
  }

  async function disconnectBase() {
    try {
      await stopWheels();
    } catch {
      /* ignore */
    }
    baseConnected = false;
  }

  function stopWheels() {
    const speeds = new Map(baseConfig.wheelIds.map((id) => [id, 0] as [number, number]));
    return robot.syncWriteWheelSpeed(speeds);
  }

  function persistBaseConfig() {
    saveBaseConfig({ ...baseConfig });
  }

  // Reset the drive patterns to the defaults (keeps wheel IDs & speed).
  function resetBaseCalibration() {
    baseConfig.forward = [...DEFAULT_BASE_CONFIG.forward] as [number, number, number];
    baseConfig.backLeft = [...DEFAULT_BASE_CONFIG.backLeft] as [number, number, number];
    baseConfig.backRight = [...DEFAULT_BASE_CONFIG.backRight] as [number, number, number];
    baseConfig.rotate = [...DEFAULT_BASE_CONFIG.rotate] as [number, number, number];
    persistBaseConfig();
  }

  // Drive toward the shelf tag using the onboard camera. First rotate to FACE the
  // tag (turn until it's centred ahead), then drive forward to the standoff while
  // keeping it centred — so the robot orients itself before/while approaching.
  // Sets the base velocity (streamed by maybeStreamBase); the sim pose is left
  // untouched — use "Reset MuJoCo position" to snap it once parked.
  function maybeNavigate() {
    if (mode !== 'real' || !navigating) return;
    const c = shelfTagCam;
    if (!c) {
      zeroBaseVel(); // lost the tag → hold
      return;
    }
    const fErr = c[2] - navStandoff; // +ve: too far, drive forward
    const lat = c[0]; // camera +x: tag to the right
    const sq = c[3]; // off-square angle of the shelf face (0 = head-on)
    const zTol = 0.03, xTol = 0.025, sqTol = 0.06; // ~3.5° square tolerance
    if (Math.abs(fErr) < zTol && Math.abs(lat) < xTol && Math.abs(sq) < sqTol) {
      navigating = false;
      zeroBaseVel();
      stopWheels().catch(() => {});
      return;
    }
    const clamp = (v: number) => Math.max(-1, Math.min(1, v));
    // Desired body velocity: centre the tag (vy left), close the distance once
    // lined up (vx), square up to the shelf face (wz).
    const vx = Math.abs(lat) < 0.1 && Math.abs(sq) < 0.2 ? clamp(2.5 * fErr) : 0;
    const vy = clamp(-2.5 * lat); // tag on the right → move right (vy negative)
    const p = bodyToPrimitives(vx, vy); // spread across the 3 drive directions
    baseVel.fwd = p.fwd;
    baseVel.bl = p.bl;
    baseVel.br = p.br;
    baseVel.turn = clamp(1.2 * navSquareSign * sq);
  }

  // ── Direction calibration: drive a wheel pair, say which way it went ──────────
  // Each of the three pairs (one wheel idle) makes the base slide along one of its
  // three primitive directions; the user watches and labels it, storing that
  // pattern. A separate rotation test sets the spin pattern.
  const TEST_MS = 1600; // how long each calibration pulse runs
  function testPattern(pattern: number[]) {
    if (baseTesting) return;
    if (!baseConnected) {
      baseError = 'Connect the wheels first.';
      return;
    }
    baseError = null;
    baseTesting = true;
    const speeds = new Map(
      baseConfig.wheelIds.map((id, i) => [id, Math.round(pattern[i] * baseConfig.speed)] as [number, number]),
    );
    robot.syncWriteWheelSpeed(speeds).catch((e) => {
      baseError = e instanceof Error ? e.message : String(e);
    });
    window.setTimeout(() => {
      baseTesting = false;
      stopWheels().catch(() => {});
    }, TEST_MS);
  }
  const testPair = (i: number) => testPattern(CAL_PAIRS[i]);
  const testRotate = () => testPattern([1, 1, 1]);

  type PairDir = 'forward' | 'backward' | 'backLeft' | 'frontRight' | 'backRight' | 'frontLeft';
  function labelPair(i: number, dir: PairDir) {
    const p = CAL_PAIRS[i];
    const neg = [-p[0], -p[1], -p[2]] as [number, number, number];
    const pos = [...p] as [number, number, number];
    if (dir === 'forward') baseConfig.forward = pos;
    else if (dir === 'backward') baseConfig.forward = neg;
    else if (dir === 'backLeft') baseConfig.backLeft = pos;
    else if (dir === 'frontRight') baseConfig.backLeft = neg;
    else if (dir === 'backRight') baseConfig.backRight = pos;
    else baseConfig.backRight = neg;
    persistBaseConfig();
  }

  function labelRotate(ccw: boolean) {
    baseConfig.rotate = (ccw ? [1, 1, 1] : [-1, -1, -1]) as [number, number, number];
    persistBaseConfig();
  }

  function toggleNavigate() {
    baseError = null;
    navigating = !navigating;
    if (!navigating) {
      zeroBaseVel();
      stopWheels().catch(() => {});
    }
  }

  // Snap the sim robot to the standoff pose in front of the selected drawer — the
  // point the real robot was navigating to — correcting sim/reality divergence.
  function snapSimToStandoff() {
    const n = shelfNormal();
    const rc = rodWorld(shelfSel, 0);
    robotX = rc[0] + n[0] * SHELF_STANDOFF;
    robotY = rc[1] + n[1] * SHELF_STANDOFF;
    robotYawDeg = (Math.atan2(-n[1], -n[0]) * 180) / Math.PI;
    placeRobot();
  }

  async function maybeStreamBase() {
    if (mode !== 'real' || !baseConnected || wheelSending) return;
    const now = performance.now();
    if (now - lastWheelSent < 66) return; // ~15 Hz
    lastWheelSent = now;
    const speeds = wheelSpeeds(baseVel.fwd, baseVel.bl, baseVel.br, baseVel.turn, baseConfig);
    // Skip overlapping frames so the shared bus queue never backs up.
    wheelSending = true;
    try {
      await robot.syncWriteWheelSpeed(speeds);
    } catch (e) {
      baseError = e instanceof Error ? e.message : String(e);
    } finally {
      wheelSending = false;
    }
  }

  async function loadJointCal(e: Event) {
    const input = e.target as HTMLInputElement;
    const file = input.files?.[0];
    if (!file) return;
    try {
      jointCal = JSON.parse(await file.text());
      jointCalName = file.name;
      if (jointCal) saveJointCalibration(jointCal); // becomes the new default
    } catch (err) {
      realError = 'Bad calibration file: ' + (err instanceof Error ? err.message : String(err));
    }
    input.value = '';
  }


  const onResize = () => fitCanvas();
  $effect(() => {
    window.addEventListener('resize', onResize);
    return () => window.removeEventListener('resize', onResize);
  });

  // Attach the live stream to the visible preview whenever the element mounts
  // (covers entering real mode after the camera already started).
  $effect(() => {
    if (previewVideo && camStream) {
      previewVideo.srcObject = camStream;
      previewVideo.play().catch(() => {});
    }
  });

  // Start/stop the camera as real mode is entered/left.
  $effect(() => {
    if (mode === 'real') startCamera();
    else {
      stopCamera();
      stage = '';
      pickPhase = 'idle';
    }
  });

  // The block differs at compile time (physics free body in sim vs mocap in real),
  // so remount the scene when the mode changes.
  $effect(() => {
    const m = mode;
    if (!mj || loadedMode === null || loadedMode === m) return;
    stage = '';
    pickPhase = 'idle';
    cancelAnimationFrame(raf);
    loadScene(selectedRobot)
      .then(loop)
      .catch((e) => {
        errorMsg = e instanceof Error ? e.message : String(e);
        status = 'Failed to load';
      });
  });

  // Live-place the shelf unit when its pose changes.
  $effect(() => {
    const _ = [shelfX, shelfY, shelfYawDeg, shelfElevation];
    void _;
    if (hasBase && session && shelfMocapId >= 0) placeShelf();
  });

  // Remount to rebuild the shelves when their count changes (LeKiwi only).
  $effect(() => {
    const c = shelfCount;
    if (!mj || !hasBase || loadedMode === null || c === loadedShelfCount) return;
    cancelAnimationFrame(raf);
    loadScene(selectedRobot)
      .then(loop)
      .catch((e) => {
        errorMsg = e instanceof Error ? e.message : String(e);
        status = 'Failed to load';
      });
  });

  // Reposition the physics block when its target X/Y/angle changes (sim mode).
  $effect(() => {
    const x = simBlockX, y = simBlockY, yaw = simBlockYawDeg;
    if (physics && session && blockQadr >= 0 && stage === '') {
      resetPhysicsBlock(x, y, yaw);
      session.forward();
    }
  });

  onDestroy(() => {
    cancelAnimationFrame(raf);
    stopCamera();
    if (realConnected) disconnectRobot();
    solver?.dispose();
    renderer?.dispose();
    session?.dispose();
  });
</script>

<div class="sim">
  <div class="viewer" bind:this={wrap}>
    <div class="robotbar">
      <label for="robot">Robot</label>
      <select
        id="robot"
        value={selectedRobot}
        onchange={(e) => onRobotChange((e.currentTarget as HTMLSelectElement).value as RobotId)}
      >
        {#each ROBOTS as r}
          <option value={r.id}>{r.label}</option>
        {/each}
      </select>
      {#if hasBase}
        <span class="baseoff">
          <span>Arm on base (m)</span>
          <input type="number" step="0.005" bind:value={baseOffX} aria-label="base X" />
          <input type="number" step="0.005" bind:value={baseOffY} aria-label="base Y" />
          <input type="number" step="0.005" bind:value={baseOffZ} aria-label="base Z" />
        </span>
        <span class="baseoff">
          <span>Arm mount rotation (°)</span>
          <input type="number" step="5" bind:value={armYawDeg} aria-label="arm mount rotation" />
        </span>
        <span class="baseoff">
          <span>Move robot (m, °)</span>
          <input type="number" step="0.02" bind:value={robotX} aria-label="robot X" />
          <input type="number" step="0.02" bind:value={robotY} aria-label="robot Y" />
          <input type="number" step="15" bind:value={robotYawDeg} aria-label="robot yaw" />
        </span>
        <span class="baseoff">
          <span>Shelves</span>
          <input type="number" min="0" max="8" step="1" bind:value={shelfCount} aria-label="shelf count" />
        </span>
      {/if}
    </div>
    <canvas bind:this={canvas}></canvas>
    {#if hasBase}
      <div class="joyoverlay">
        <DriveTriad onmove={setDrive} />
        <RotationControl onrotate={turnRobot} />
      </div>
    {/if}
    {#if !ready && !errorMsg}<LoadingScreen message={status} />{/if}
    {#if errorMsg}<div class="overlay err">{errorMsg}</div>{/if}
  </div>

  <div class="panel">
    <div class="modeswitch">
      <button class:active={mode === 'sim'} onclick={() => (mode = 'sim')}>Sim only</button>
      <button class:active={mode === 'real'} onclick={() => (mode = 'real')}>Drive real arm</button>
    </div>

    <h2>Arm</h2>
    <div class="graspmode">
      <button class:active={!armRest} onclick={() => (armRest = false)}>Active (follow target)</button>
      <button class:active={armRest} onclick={() => (armRest = true)}>Rest (home / parked)</button>
    </div>
    <p class="hint">
      Rest parks the arm swung to the right and tucked away, ignoring the end-effector target.
      It also engages automatically while the base is navigating{navigating && !armRest ? ' (active now)' : ''},
      so the real arm doesn't flail as the robot drives.
    </p>

    <h2>End-effector target</h2>
    <div class="sliders">
      {#each ['x', 'y', 'z'] as axis, i (axis)}
        <label>{axis}</label>
        <input type="range" min="-1" max="1" step="0.005" bind:value={target[i]} />
        <span class="val">{target[i].toFixed(3)}</span>
      {/each}
      <label>grasp</label>
      <div class="graspmode">
        <button class:active={graspMode === 'top'} onclick={() => (graspMode = 'top')}>Top</button>
        <button class:active={graspMode === '45'} onclick={() => (graspMode = '45')}>45°</button>
        <button class:active={graspMode === 'front'} onclick={() => (graspMode = 'front')}>Front</button>
        <button class:active={graspMode === 'none'} onclick={() => (graspMode = 'none')}>None</button>
      </div>
      <span class="val"></span>
      <label title="rotate the gripper about its approach axis (or fix wrist roll in None)">roll</label>
      <input type="range" min="-180" max="180" step="5" bind:value={graspRollDeg} />
      <span class="val">{graspRollDeg}°</span>
      <label title="prefer an IK branch — wrist_flex bends up (top) or down (bottom)">direction</label>
      <div class="graspmode">
        <button class:active={armBranch === 'auto'} onclick={() => (armBranch = 'auto')}>Auto</button>
        <button class:active={armBranch === 'top'} onclick={() => (armBranch = 'top')}>Top</button>
        <button class:active={armBranch === 'bottom'} onclick={() => (armBranch = 'bottom')}>Bottom</button>
      </div>
      <span class="val"></span>
      <label>grip</label>
      <input
        type="range"
        min={gripperRange[0]}
        max={gripperRange[1]}
        step="0.01"
        bind:value={gripperCmd}
      />
      <span class="val">
        {Math.round(((gripperCmd - gripperRange[0]) / (gripperRange[1] - gripperRange[0])) * 100)}%
      </span>
    </div>

    <div class="ikstatus {ikOk ? 'ok' : 'bad'}">
      {ikOk ? 'reached' : 'unreached'} · error {ikErrMm.toFixed(1)} mm
    </div>

    <table class="angles">
      <thead><tr><th>joint</th><th>rad</th><th>deg</th></tr></thead>
      <tbody>
        {#each ['pan', 'lift', 'elbow', 'wrist_flex', 'wrist_roll'] as name, i (name)}
          {@const a = jointAngles[i]}
          <tr>
            <td>{name}</td>
            <td>{a != null ? a.toFixed(3) : '—'}</td>
            <td>{a != null ? ((a * 180) / Math.PI).toFixed(1) : '—'}</td>
          </tr>
        {/each}
      </tbody>
    </table>

    {#if mode === 'sim'}
      <div class="realbox">
        <h2>Pick the sample block</h2>
        <p class="hint">
          The block is a real physics object — it falls, rests on the floor, and is grasped for real:
          set its position, then step through approach → descend → grasp → lift. On grasp it welds to
          the gripper and lifts with it; set the stage back to off to drop it.
        </p>
        <div class="pickrow">
          <label for="sbx">Block X (m)</label>
          <input id="sbx" type="number" step="0.01" bind:value={simBlockX} />
          <label for="sby">Block Y (m)</label>
          <input id="sby" type="number" step="0.01" bind:value={simBlockY} />
        </div>
        <div class="pickrow">
          <label for="sbyaw">Block angle (°)</label>
          <input id="sbyaw" type="number" step="15" bind:value={simBlockYawDeg} />
        </div>
        <div class="pickrow">
          <label for="simstage">Stage</label>
          <select id="simstage" value={stage} onchange={(e) => onStageChange(e.currentTarget.value as Stage)}>
            <option value="">— off (manual) —</option>
            <option value="approach">1 · Approach (hover)</option>
            <option value="descend">2 · Descend</option>
            <option value="grasp">3 · Grasp (close)</option>
            <option value="lift">4 · Lift</option>
          </select>
        </div>
        <div class="pickrow">
          <label for="sdepth">Grasp depth (m)</label>
          <input id="sdepth" type="number" step="0.005" bind:value={graspDepth} />
        </div>
        <div class="pickrow">
          <label for="sgx">Grasp X offset (m)</label>
          <input id="sgx" type="number" step="0.005" bind:value={graspX} />
        </div>
        <div class="pickrow">
          <label for="sgy">Grasp Y offset (m)</label>
          <input id="sgy" type="number" step="0.005" bind:value={graspY} />
        </div>
        <div class="pickrow">
          <label for="gyaw">Grip rotation (°)</label>
          <input id="gyaw" type="number" step="15" bind:value={gripYawDeg} />
        </div>
        <div class="status {stage !== '' ? 'ok' : 'warn'}">
          {stage !== '' ? `holding at ${stage}` : 'pick off — choose a stage to drive the arm'}
        </div>
      </div>
    {/if}

    {#if mode === 'real'}
      <div class="realbox">
        <h2>Camera feed</h2>
        <p class="hint">Live onboard camera — what the tag detection & shelf navigation see.</p>
        <!-- svelte-ignore a11y_media_has_caption -->
        <video bind:this={previewVideo} class="camfeed" playsinline muted></video>
      </div>

      <div class="realbox">
        <h2>Real arm</h2>
        <p class="hint">
          Solved angles stream to the Feetech servos at ~15 Hz. Needs a joint calibration
          (from the Joint calibration tab) to map sim radians → servo ticks.
        </p>
        <div class="controls">
          {#if !realConnected}
            <button class="primary" onclick={connectRobot}>Connect servos…</button>
          {:else}
            <button onclick={disconnectRobot}>Disconnect</button>
          {/if}
          <label class="file-btn">
            {jointCalName ?? 'Load joint_calibration.json…'}
            <input type="file" accept="application/json,.json" onchange={loadJointCal} />
          </label>
        </div>
        {#if realConnected && !jointCal}
          <div class="status warn">Connected — load a joint calibration to start driving.</div>
        {:else if realConnected && jointCal}
          <div class="status ok">Driving arm live from the simulator.</div>
        {/if}
        {#if realError}<div class="status bad">Error: {realError}</div>{/if}
      </div>

      {#if hasBase}
        <div class="realbox">
          <h2>Base wheels</h2>
          <p class="hint">
            The 3 omniwheel motors share the arm's serial bus (IDs {baseConfig.wheelIds.join(', ')}).
            Connect the arm first, then <em>Connect wheels</em> switches them to wheel mode — the
            joystick, rotation panel and the shelf <em>Drive</em> stage then move the real base.
          </p>
          <div class="controls">
            {#if !baseConnected}
              <button class="primary" onclick={connectBase} disabled={!realConnected}>Connect wheels…</button>
            {:else}
              <button onclick={disconnectBase}>Disconnect wheels</button>
              <button onclick={stopWheels}>Stop</button>
            {/if}
          </div>
          <div class="pickrow">
            <label for="wids">Wheel servo IDs</label>
            {#each baseConfig.wheelIds as _id, i (i)}
              <input
                type="number"
                style="width:3.2rem"
                value={baseConfig.wheelIds[i]}
                onchange={(e) => { baseConfig.wheelIds[i] = +e.currentTarget.value; persistBaseConfig(); }}
              />
            {/each}
          </div>
          <div class="pickrow">
            <label for="bspeed">Speed</label>
            <input id="bspeed" type="number" step="25" bind:value={baseConfig.speed} onchange={persistBaseConfig} />
          </div>
          <div class="status {baseConnected ? 'ok' : 'warn'}">
            {baseConnected ? 'wheels live — drive with W/A/D + Q/E' : 'wheels not connected'}
          </div>
          {#if baseError}<div class="status bad">Base error: {baseError}</div>{/if}
        </div>

        <div class="realbox">
          <h2>Direction calibration</h2>
          <p class="hint">
            The base has 3 drive directions 120° apart: <strong>forward</strong>,
            <strong>back-left</strong> (+120°) and <strong>back-right</strong> (−120°). Each is one
            pair of wheels. Hit <em>Test</em> on a pair, watch which of those the base slid toward, and
            click it (the greyed row on the right is the opposite direction, if it went the other way).
            Then the rotation test. Give it clear floor space.
          </p>
          {#each [0, 1, 2] as i (i)}
            <div class="pickrow">
              <button style="min-width:4rem" disabled={!baseConnected || baseTesting} onclick={() => testPair(i)}>
                {baseTesting ? '…' : `▶ Pair ${i + 1}`}
              </button>
              <div class="graspmode wrap">
                <button disabled={baseTesting} onclick={() => labelPair(i, 'forward')}>Forward</button>
                <button disabled={baseTesting} onclick={() => labelPair(i, 'backLeft')}>Back-left</button>
                <button disabled={baseTesting} onclick={() => labelPair(i, 'backRight')}>Back-right</button>
                <button class="ghost" disabled={baseTesting} onclick={() => labelPair(i, 'backward')}>Backward</button>
                <button class="ghost" disabled={baseTesting} onclick={() => labelPair(i, 'frontRight')}>Front-right</button>
                <button class="ghost" disabled={baseTesting} onclick={() => labelPair(i, 'frontLeft')}>Front-left</button>
              </div>
            </div>
          {/each}
          <div class="pickrow">
            <button style="min-width:4rem" disabled={!baseConnected || baseTesting} onclick={testRotate}>
              {baseTesting ? '…' : '▶ Rotate'}
            </button>
            <div class="graspmode wrap">
              <button disabled={baseTesting} onclick={() => labelRotate(true)}>Turned left (CCW)</button>
              <button disabled={baseTesting} onclick={() => labelRotate(false)}>Turned right (CW)</button>
            </div>
          </div>
          <div class="controls">
            <button onclick={resetBaseCalibration} title="restore default drive patterns">Reset</button>
          </div>
          <div class="status {baseConnected ? 'ok' : 'warn'}">
            {baseConnected
              ? `fwd [${baseConfig.forward.join(',')}] · bk-L [${baseConfig.backLeft.join(',')}] · bk-R [${baseConfig.backRight.join(',')}]`
              : 'connect the wheels to test'}
          </div>
          {#if baseError}<div class="status bad">{baseError}</div>{/if}
        </div>

        <div class="realbox">
          <h2>Navigate to shelf (tag {SHELF_TAG_ID})</h2>
          <p class="hint">
            Point the onboard camera at the shelf's ArUco tag (ID {SHELF_TAG_ID} — print it with
            <em>make_shelf_tag.py</em>). <em>Drive to tag</em> approaches to the standoff and centres
            it; once parked, <em>Reset MuJoCo position</em> snaps the sim robot to the matching
            standoff in front of drawer {shelfSel + 1}, correcting sim/reality drift.
          </p>
          <div class="pickrow">
            <label for="navsd">Standoff (m)</label>
            <input id="navsd" type="number" step="0.02" bind:value={navStandoff} />
          </div>
          <div class="controls">
            <button
              class="primary"
              disabled={!baseConnected || (!shelfTagCam && !navigating)}
              onclick={toggleNavigate}
            >
              {navigating ? 'Stop' : `Drive to tag ${SHELF_TAG_ID}`}
            </button>
            <button disabled={!hasBase} onclick={snapSimToStandoff}>Reset MuJoCo position</button>
            <button onclick={() => (navSquareSign = -navSquareSign)} title="if it squares up the wrong way">
              Flip square-up {navSquareSign === 1 ? '+' : '−'}
            </button>
          </div>
          <div class="status {shelfTagCam ? 'ok' : 'warn'}">
            {#if shelfTagCam}
              tag seen · {shelfTagCam[2].toFixed(2)} m ahead ·
              off-centre {Math.round((Math.atan2(shelfTagCam[0], shelfTagCam[2]) * 180) / Math.PI)}° ·
              off-square {Math.round((shelfTagCam[3] * 180) / Math.PI)}°{navigating ? ' · navigating…' : ''}
            {:else}
              tag {SHELF_TAG_ID} not seen — aim the onboard camera at the shelf
            {/if}
          </div>
        </div>
      {/if}

      <div class="realbox">
        <h2>Pick a block</h2>
        <p class="hint">
          The camera detects the block's ArUco tag and solvePnP gives its board position (using the
          ChArUco camera intrinsics); it's copied into the sim, then the arm approaches, descends,
          grasps and lifts.
        </p>
        {#if !intrinsics}
          <div class="status bad">
            No camera intrinsics — run the ChArUco camera calibration first (Connect and calibrate!).
          </div>
        {/if}
        <div class="pickrow">
          <label for="blocktag">Block tag ID</label>
          <input id="blocktag" type="number" bind:value={blockTag} />
          <label for="stage">Stage</label>
          <select
            id="stage"
            value={stage}
            disabled={!realConnected || !jointCal}
            onchange={(e) => onStageChange(e.currentTarget.value as Stage)}
          >
            <option value="">— off (manual) —</option>
            <option value="approach">1 · Approach (hover)</option>
            <option value="descend">2 · Descend</option>
            <option value="grasp">3 · Grasp (close)</option>
            <option value="lift">4 · Lift</option>
          </select>
        </div>
        <div class="pickrow">
          <label for="depth">Grasp depth (m)</label>
          <input id="depth" type="number" step="0.005" bind:value={graspDepth} />
        </div>
        <div class="pickrow">
          <label for="gx">Grasp X offset (m)</label>
          <input id="gx" type="number" step="0.005" bind:value={graspX} />
        </div>
        <div class="pickrow">
          <label for="gy">Grasp Y offset (m)</label>
          <input id="gy" type="number" step="0.005" bind:value={graspY} />
        </div>
        <div class="pickrow">
          <label for="tox">Tag→block X (mm)</label>
          <input id="tox" type="number" step="1" bind:value={tagOffX} />
        </div>
        <div class="pickrow">
          <label for="toy">Tag→block Y (mm)</label>
          <input id="toy" type="number" step="1" bind:value={tagOffY} />
        </div>
        <div class="pickrow">
          <label for="yaw">Block yaw offset (°)</label>
          <input id="yaw" type="number" step="90" bind:value={blockYawOffsetDeg} />
        </div>
        <div class="status {detected.has(blockTag) ? 'ok' : 'warn'}">
          {#if stage !== ''}
            holding at <strong>{stage}</strong>
          {:else if detected.has(blockTag)}
            block {blockTag} detected (solvePnP)
          {:else}
            block {blockTag} not detected — hold it on the board in view of the camera
          {/if}
        </div>
        {#if detectMsg}<div class="status bad">{detectMsg}</div>{/if}
      </div>
    {/if}

    {#if hasBase}
      <div class="realbox">
        <h2>Shelves</h2>
        <p class="hint">
          Step through the stages: <em>Drive</em> in front of the drawer, <em>Approach</em> the rod,
          <em>Enter</em> onto it, <em>Grip</em>, then <em>Open</em> (pull) or <em>Close</em> (push).
          {#if mode === 'real'}The same targets drive the real arm (and the Drive stage drives the wheels).{:else}Physics-only — the arm actually pushes/pulls the rod.{/if}
        </p>
        <div class="pickrow">
          <label for="shx">Shelf X / Y (m)</label>
          <input id="shx" type="number" step="0.02" bind:value={shelfX} />
          <input type="number" step="0.02" bind:value={shelfY} />
        </div>
        <div class="pickrow">
          <label for="shyaw">Shelf yaw / elev (°, m)</label>
          <input id="shyaw" type="number" step="15" bind:value={shelfYawDeg} />
          <input type="number" step="0.02" bind:value={shelfElevation} />
        </div>
        <div class="pickrow">
          <span>Drawer</span>
          <div class="graspmode">
            {#each shelfOpens as open, i (i)}
              <button class:active={shelfSel === i} onclick={() => (shelfSel = i)}>
                {i + 1} · {Math.round(open * 100)}cm
              </button>
            {/each}
          </div>
        </div>
        <div class="pickrow">
          <span>Stage</span>
          <div class="graspmode wrap">
            <button class:active={shelfStage === ''} onclick={() => (shelfStage = '')}>Off</button>
            <button class:active={shelfStage === 'drive'} onclick={() => (shelfStage = 'drive')}>Drive</button>
            <button class:active={shelfStage === 'approach'} onclick={() => (shelfStage = 'approach')}>Approach</button>
            <button class:active={shelfStage === 'enter'} onclick={() => (shelfStage = 'enter')}>Enter</button>
            <button class:active={shelfStage === 'grip'} onclick={() => (shelfStage = 'grip')}>Grip</button>
            <button class:active={shelfStage === 'open'} onclick={() => (shelfStage = 'open')}>Open</button>
            <button class:active={shelfStage === 'close'} onclick={() => (shelfStage = 'close')}>Close</button>
          </div>
        </div>
        <div class="status {shelfStage !== '' ? 'ok' : 'warn'}">
          drawer {shelfSel + 1} · {shelfStage || 'off'} · {Math.round((shelfOpens[shelfSel] ?? 0) * 100)} cm open
        </div>
      </div>
    {/if}
  </div>
</div>

<!-- Hidden camera source for block detection in real mode. -->
<video bind:this={video} playsinline muted style="display:none"></video>

<style>
  .camfeed {
    width: 100%;
    border-radius: 6px;
    background: #000;
    display: block;
    aspect-ratio: 4 / 3;
    object-fit: contain;
  }
  .graspmode .ghost {
    opacity: 0.5;
    font-size: 0.85em;
  }
  .sim {
    display: grid;
    grid-template-columns: 1fr 300px;
    gap: 1.25rem;
    align-items: start;
  }
  @media (max-width: 980px) {
    .sim {
      grid-template-columns: 1fr;
    }
  }
  .viewer {
    position: relative;
    width: 100%;
  }
  .joyoverlay {
    position: absolute;
    left: 0.6rem;
    bottom: 0.6rem;
    z-index: 5;
    opacity: 0.92;
    display: flex;
    gap: 0.5rem;
    align-items: flex-start;
  }
  .robotbar {
    display: flex;
    align-items: center;
    flex-wrap: wrap;
    gap: 0.5rem;
    margin-bottom: 0.5rem;
    font-size: 0.85rem;
  }
  .robotbar select {
    padding: 0.2rem 0.4rem;
  }
  .robotbar .baseoff {
    display: inline-flex;
    align-items: center;
    gap: 0.3rem;
    color: var(--text-soft, #999);
  }
  .robotbar .baseoff input {
    width: 4.5rem;
    padding: 0.15rem 0.3rem;
  }
  .viewer canvas {
    width: 100%;
    display: block;
    border: 1px solid var(--line-soft);
    border-radius: 8px;
  }
  .overlay {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    color: var(--muted);
    font-size: 0.95rem;
  }
  .overlay.err {
    color: var(--bad);
    padding: 1rem;
    text-align: center;
  }
  h2 {
    font-size: 0.95rem;
    margin: 1rem 0 0.5rem;
  }
  .modeswitch {
    display: flex;
    gap: 0.25rem;
  }
  .modeswitch button {
    flex: 1;
  }
  .graspmode { display: flex; gap: 0.25rem; }
  .graspmode.wrap { flex-wrap: wrap; }
  .graspmode button {
    flex: 1;
    padding: 0.15rem 0.3rem;
    font-size: 0.78rem;
    cursor: pointer;
  }
  .graspmode button.active {
    background: var(--accent, #3b82f6);
    color: #fff;
  }
  .modeswitch button.active {
    background: var(--accent);
    border-color: var(--accent);
    color: #fff;
  }
  .sliders {
    display: grid;
    grid-template-columns: auto 1fr auto;
    gap: 0.4rem 0.6rem;
    align-items: center;
  }
  .sliders label {
    font-family: ui-monospace, monospace;
    color: var(--muted);
  }
  .sliders .val {
    font-family: ui-monospace, monospace;
    font-size: 0.85rem;
    width: 3.5rem;
    text-align: right;
  }
  .ikstatus {
    margin-top: 0.6rem;
    font-family: ui-monospace, monospace;
    font-size: 0.85rem;
  }
  .ikstatus.ok {
    color: var(--ok);
  }
  .ikstatus.bad {
    color: var(--warn);
  }
  table.angles {
    width: 100%;
    margin-top: 0.75rem;
    border-collapse: collapse;
    font-family: ui-monospace, monospace;
    font-size: 0.82rem;
  }
  table.angles th,
  table.angles td {
    text-align: right;
    padding: 0.25rem 0.5rem;
    border-bottom: 1px solid var(--line-soft);
  }
  table.angles th:first-child,
  table.angles td:first-child {
    text-align: left;
    color: var(--muted);
  }
  .file-btn {
    display: inline-flex;
    align-items: center;
    background: var(--surface);
    border: 1px solid var(--line-soft);
    border-radius: 6px;
    padding: 0.45rem 0.9rem;
    font-size: 0.85rem;
    cursor: pointer;
  }
  .file-btn input {
    display: none;
  }
  .pickrow {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    margin: 0.6rem 0;
  }
  .pickrow label {
    font-size: 0.85rem;
    color: var(--muted);
  }
  .pickrow input[type='number'] {
    width: 4.5rem;
    background: var(--surface);
    color: var(--ink);
    border: 1px solid var(--line-soft);
    border-radius: 4px;
    padding: 0.25rem 0.4rem;
  }
  .pickrow select {
    flex: 1;
    background: var(--surface);
    color: var(--ink);
    border: 1px solid var(--line-soft);
    border-radius: 4px;
    padding: 0.28rem 0.4rem;
  }
</style>
