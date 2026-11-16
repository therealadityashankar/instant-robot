<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import {
    loadMujocoModule,
    mountModel,
    type ModelFiles,
    type Session,
  } from './lib/mujocoSession';
  import { IKSolver } from './lib/ik';
  import { MujocoRenderer, PovView } from './lib/mujocoRender';
  import Map2DView from './Map2DView.svelte';
  import AiChat from './AiChat.svelte';
  import type { RobotStateContext, RobotActionCallbacks } from './lib/aiAssistant';
  import { CALIBRATION_PLAN, simRadToServo, servoToSimRad, type JointCalibration } from './lib/joints';
  import {
    saveJointCalibration,
    loadJointCalibration,
    loadIntrinsics,
    saveRobotId,
    loadRobotId,
    saveAutoConnect,
    loadAutoConnect,
    loadRemoteConfig,
    loadPickCalibration,
  } from './lib/storage';
  import type { PickCalibration, GoalQuadConfig } from './lib/pickCalibration';
  import { computeQuadCenter, computeQuadAvgSize, computeQuadOrientationDeg, loadGoalQuad } from './lib/pickCalibration';
  import { DEFAULT_PICK_CALIBRATION } from './lib/pickCalibration';
  import { pickCalState, type AutoNavMode } from './lib/pickCalState.svelte';
  import { wheelSpeeds, bodyToPrimitives, PRIMITIVE_DIRS } from './lib/lekiwiBase';
  import {
    buildBoardSceneXml,
    withWristRollRef,
    STATION_TAG_ID,
    BOUNDARY_TAG_ID,
    STATION_OBJECT_TAG,
    STATION_X,
    STATION_Y,
    STATION_H,
    DEFAULT_STATIONS,
    STATIONS,
    setCustomStations,
    type StationDef,
    type StationType,
    type PropKind,
    DEFAULT_ACT_MODELS,
    type ActModelDef,
    DEFAULT_MISSIONS,
    type PathPlan,
    type PathPlanStep,
    stationTagWorld,
    interiorToSim,
    SQUARE_MM,
    INSET_MM,
    BLOCK_HALF_Z,
    BLOCK_SCATTER_R,
    ROBOT_BASE_TIP_X,
    LINE_X,
  } from './lib/boardSim';
  import { zipSync, unzipSync, strToU8, strFromU8 } from 'fflate';
  import { ROBOTS, robotById, injectBase, type RobotId, type BaseManifest } from './lib/robots';
  import { injectShelves, shelfJointName, rodLocal, SHELF } from './lib/shelves';
  import { phaseTarget, DEFAULT_PICK, type PickPhase } from './lib/pick';
  import LoadingScreen from './LoadingScreen.svelte';
  import DriveTriad from './DriveTriad.svelte';
  import RotationControl from './RotationControl.svelte';
  import { loadCv, type Cv } from './lib/cv';
  import { boardTagCentres, BORDERED_IDS, type TagCentres } from './lib/board';
  import { createDetector, detectMarkers, detectAllMarkers, type Detector, type MarkerDetection } from './lib/homography';
  import { boardPoseFromTags, blockBoardXY } from './lib/detect3d';
  import { solvePnpMarkerPose, matMul3, apply3 } from './lib/pose';
  import {
    LEFT_NAV_ID, RIGHT_NAV_ID,
    findLabeledTagPairs,
    hasLabeledTagPair, extractLabeledTag, initTesseract, terminateTesseract,
    isTagPairReadyForOcr,
    type LabeledTagInfo,
  } from './lib/labelTag';
  import {
    matMul3x3,
    rotAngleBetween,
    rotateAboutApproach,
    slewRotation,
    nearestHalfTurn,
  } from './lib/rot';
  import type { Intrinsics } from './lib/charuco';
  import { armLink } from './lib/armLink.svelte';
  import { RemoteCameraSink } from './lib/remoteVideo';
  import { baseLink, robot } from './lib/baseLink.svelte';
  import { settings } from './lib/settings.svelte';
  import { markerCanvas } from './lib/arucoImage';

  const isDev = import.meta.env.DEV;

  // ── Camera mounting ────────────────────────────────────────────────────────
  // Measured by eye against the rendered scene. Base offsets are relative to the
  // robot base; wrist offsets and angles are relative to the wrist-cam bracket.
  const CAM_BASE_FWD = 0.11;
  const CAM_BASE_LAT = 0.0;
  const CAM_BASE_UP = -0.09;
  // Offsets and angles of the wrist camera within the bracket's frame, tuned by
  // eye against the rendered scene.
  const CAM_WRIST_FWD = 0.002;
  const CAM_WRIST_LAT = 0.0765;
  const CAM_WRIST_UP = -0.011;
  const CAM_WRIST_ROLL_DEG = -19;
  const CAM_WRIST_PITCH_DEG = -81.25;
  const CAM_WRIST_YAW_DEG = -70.75;
  // Trim, in the camera's own frame after the angles above: standoff along the
  // view axis, then small rotations about the camera's own faces.
  const CAM_AHEAD = -0.004;
  const CAM_TRIM_ROLL_DEG = 173.25;
  const CAM_TRIM_PITCH_DEG = 17.5;
  const CAM_TRIM_YAW_DEG = -3;
  // Where the printed bracket hangs off the gripper frame.
  const MOUNT_OFF = [-0.03, 0, 0];
  const MOUNT_RPY_DEG = [0, 90, 0];

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
    'wrist_cam_mount.stl', // SO-ARM100 optional wrist-cam bracket
    'boundary_base_plate.stl',
    'boundary_stand_upright.stl',
    'boundary_stop_stand.stl',
  ];
  const ARM_DOFS = [0, 1, 2, 3, 4];
  const GRASP_SITE = 'graspframe';

  let { onOpenCalibrate }: { onOpenCalibrate?: () => void } = $props();

  let canvas: HTMLCanvasElement;
  let stepsOpen = $state(false); // the 1-8 pick steps, collapsed by default
  let manualArm = $state(false); // low-level arm controls, hidden until asked for
  let shelfOpen = $state(false); // the shelves section, collapsed by default

  // ── Remembered places ───────────────────────────────────────────────────────
  // Nav tags (200+) the robot has seen, with where they were in the world at the
  // time. This is what turns a station from something you have to know
  // into a button that appears once the robot has looked around.
  interface KnownTag {
    id: number;
    x: number;
    y: number;
    z: number;
    /** Direction the tag's printed face points, world radians. Needed to park
     *  square-on rather than merely nearby. */
    faceYaw: number;
    /** OCR'd label from the labeled tag (e.g. "APPLE"). */
    label?: string;
    /** OCR'd description from the labeled tag (e.g. "Red fruit, pick it up"). */
    description?: string;
  }
  let knownTags = $state<Map<number, KnownTag>>(new Map());
  // Labeled tag OCR state: tracks the last-seen pair to avoid re-OCR'ing every frame.
  let lastLabeledOcr = $state<{ label: string; ts: number } | null>(null);
  let labeledOcrBusy = $state(false);
  let lastOcrText = $state('');
  let lastOcrDesc = $state('');
  let lastOcrLabelImgUrl = $state('');
  let lastOcrDescImgUrl = $state('');
  let lastOcrFullImgUrl = $state('');
  let atTag = $state<number | null>(null); // the place we've most recently arrived at
  let exploring = $state(false);
  let exploringType = $state<'tags' | 'names' | null>(null);
  let exploreCancel = false;
  let exploreProgress = $state(0); // 0..1 of progress
  let hasExplored = $state(false);
  let calibratedRevTimeMs: number | null = null;
  let lastTagTimings = new Map<number, number>();
  let lastRefTime: number | null = null;
  // Detected tags in the CAMERA frame (R marker→camera, t mm) — transformed to world
  // for the tag view via the onboard-camera mount.
  type CamTag = { id: number; R: number[]; t: number[]; sizeMm: number };
  let tagPoses = $state<CamTag[]>([]);
  let robotGeomIds: number[] = []; // arm + base geoms (excludes board/block/shelves)
  // Onboard camera mounted on the base: forward offset + height above the arm base.
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
  // wrist_flex, wrist_roll]. "Set rest to current" overwrites it.
  let restTargetPose = $state([1.3, 0.0, 0.9, 0.0, 0.0]);
  let armMode = $state<'active' | 'hold' | 'rest'>('active');
  let lockedArmPose = $state<number[]>([1.3, 0.0, 0.9, 0.0, 0.0]);

  function setArmMode(mode: 'active' | 'hold' | 'rest') {
    armMode = mode;
    if (mode === 'hold') {
      lockedArmPose = jointAngles.slice(0, 5);
    } else if (mode === 'active') {
      targetToCurrentEE();
    }
  }

  // A pose the arm is asked to hold outright, bypassing IK (visual pick steps and
  // "reload from last position" use it). Null = follow the EE target.
  let jointHoldPose = $state<number[] | null>(null);
  // True while a hold/rest/view frame ran, so the next manual frame re-anchors the
  // EE target on where the arm actually ended up instead of yanking to a stale one.
  let wasHolding = false;
  // The base/arm meshes are authored 45° off about vertical, so the whole body is
  // drawn yawed by this much; the driving/logic yaw is unaffected.
  const BODY_YAW_OFFSET = (45 * Math.PI) / 180;

  // The last blind drop onto the object (m). Tuned by hand, and it tracks the
  // hover height: the approach parks that far off along the tag's normal, so the
  // descent has roughly that far to travel to seat the jaws around the block.
  // Hover height plus a little more, because the tag is on the object's *top*
  // face: descending exactly the hover distance puts the jaws level with the top
  // surface, where they close on the upper edge and slide off. The extra takes
  // them down the side of a 15 mm block to grip it around the middle.
  // Measured once the descent could actually complete. 62 mm lands the grasp site
  // at ~180 mm, where the finger pads straddle a block occupying 160-175 mm and it
  // registers contact. 75 mm is worse, not better: the fingers extend below the
  // site, so it puts them under the block entirely — and with the pedestal no
  // longer stopping the arm, there is nothing to catch that mistake.
  let finalDropM = $state(0.062);
  let liftM = $state(0.1); // step 8: how far to lift once gripped (m)

  // ── Global arm speed limit ───────────────────────────────────────────────────
  // Every commanded arm pose — manual IK, pick stages, shelf stages, rest, board
  // view — is routed through limitPose(), which slews the actual command toward
  // the desired one at no more than `settings.maxArmSpeedDeg` per second per joint. So
  // nothing ever snaps, and there's one knob for it instead of a per-mode rate.
  let cmdPose: number[] | null = null; // the rate-limited pose actually commanded
  let lastPoseMs = 0;

  /**
   * Slew the commanded pose toward `desired` (5 arm joints, rad) and return it.
   * On the first call after a reset the command starts at `seed` (the arm's live
   * pose), so motion always begins from where the arm actually is.
   */
  function limitPose(desired: number[], seed?: number[]): number[] {
    const now = performance.now();
    if (!cmdPose) {
      cmdPose = (seed ?? desired).slice(0, 5);
      lastPoseMs = now;
      return cmdPose.slice();
    }
    const dt = Math.min(0.1, (now - lastPoseMs) / 1000); // cap so a stalled tab doesn't jump
    lastPoseMs = now;
    const step = ((settings.maxArmSpeedDeg * Math.PI) / 180) * dt;
    for (let i = 0; i < 5; i++) {
      const diff = desired[i] - cmdPose[i];
      cmdPose[i] += Math.max(-step, Math.min(step, diff));
    }
    return cmdPose.slice();
  }

  /** Forget the slew state so the next command re-seeds from the live pose. */
  function resetPoseLimiter() {
    cmdPose = null;
  }
  // Gripper orientation held fixed while a visual step runs (null = free).
  let holdRd: number[] | null = null;
  /**
   * How hard to insist on that orientation.
   *
   * The arm has five joints and a pose has six numbers, so position and
   * orientation are in direct competition — at full weight the solver holds the
   * wrist angle and simply declines to move. That is not hypothetical: the final
   * descent commanded 83 mm and achieved 25 mm, leaving the jaws closing 50 mm
   * above the block, every single time. Steps that must *arrive* turn this down so
   * position wins; steps that are only holding an attitude leave it high.
   */
  let holdOriWeight = 1;
  let ikOk = $state(true);
  let ikErrMm = $state(0);
  let jointAngles = $state<number[]>([0, 0, 0, 0, 0]);
  let gripperRange = $state<[number, number]>([-0.1745, 1.7453]); // [open, closed]

  // ── sim / real ─────────────────────────────────────────────────────────────
  let realConnected = $state(false);
  // No sim/real switch: if the servos are connected we're driving the real robot,
  // otherwise it's sim-only. Every `mode === 'real'` check downstream still works.
  const mode = $derived<'sim' | 'real'>(realConnected ? 'real' : 'sim');
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
  type Stage = '' | 'open' | 'approach' | 'descend' | 'grasp' | 'lift';
  // Stepped-descent height (m above the board) while the auto-pick lowers toward
  // the tag; null = use the configured grasp depth in one go.
  let descendZ = $state<number | null>(null);
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
  let simBlockX = $state(STATION_X);
  let simBlockY = $state(STATION_Y);
  // Finger-opening direction (deg) for the top-down grasp. Adjust to line the
  // gripper up with the block (0 and 180 are equivalent; 90 rotates it a quarter).
  let gripYawDeg = $state(0);
  let simBlockYawDeg = $state(0); // resting orientation of the sample block

  // Board perspective correction (from the Test-calibration tab's JSON).
  // Camera intrinsics (mandatory — from the ChArUco step). Reloaded when the
  // real-arm camera starts, so a fresh calibration is picked up without reload.
  let intrinsics = $state<Intrinsics | null>(loadIntrinsics());
  let pickCal = $state<PickCalibration>(loadPickCalibration());
  const TAG_MM = 16; // board border-tag size
  const BLOCK_MARKER_MM = 15; // block ArUco marker size (the black square, not the white border)
  // Onboard-camera nav fiducial: labeled tags use a pair of markers (200 left,
  // 201 right) with text between them. The station identity comes from the OCR'd
  // label, not from the tag number. The nav system drives toward the LEFT marker
  // (200) by default — its pose is what solvePnP uses for the approach.
  let navTagId = $state(201); // the tag the base is currently driving toward
  let navTagMm = $state(40); // printed black marker side (mm) — must match the printout
  const NAV_TAG_LO = 200, NAV_TAG_HI = 499; // nav-fiducial id range to auto-offer (200 anchor + 201..499 stations)
  // Nav-fiducial ids currently in view (from the onboard camera) — each gets its
  // own "Drive to tag N" button, so no manual id selector is needed.
  let visibleNavTags = $state<number[]>([]);
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
  let camStream = $state<MediaStream | null>(null);

  // Cameras on a remotely-driven robot. Frames arrive as JPEGs over the WebRTC
  // datachannel and are decoded onto canvases, which the detector and the preview
  // panels consume exactly like a local webcam. Camera 0 is the base, 1 the arm —
  // the order the Pi enumerates them by USB port.
  const remoteCams = new RemoteCameraSink();
  const REMOTE_BASE_CAM = 0;
  const REMOTE_ARM_CAM = 1;
  // Reactive mirror of the sink (a plain class can't be read in markup); flipped
  // only when a camera's presence actually changes.
  let remoteBaseLive = $state(false);
  let remoteArmLive = $state(false);
  // Visible panel canvases; the sink's own canvases are offscreen and sized by
  // the incoming frames, so each is blitted here to fit the panel.
  let remoteBaseCanvas = $state<HTMLCanvasElement>();
  let remoteArmCanvas = $state<HTMLCanvasElement>();
  remoteCams.onFrame = (cam) => {
    if (cam === REMOTE_BASE_CAM && !remoteBaseLive) remoteBaseLive = true;
    if (cam === REMOTE_ARM_CAM && !remoteArmLive) remoteArmLive = true;
    paintRemote(cam);
  };

  let activeRemoteCam = $state<number>(REMOTE_BASE_CAM);

  async function selectRemoteCamera(cam: number) {
    activeRemoteCam = cam;
    if (cam === REMOTE_ARM_CAM) remoteArmLive = true;
    if (cam === REMOTE_BASE_CAM) remoteBaseLive = true;
    if (robot.connected) {
      try {
        appendLog(`[Camera] Switching to camera ${cam === REMOTE_ARM_CAM ? 'Arm (1)' : 'Base (0)'}...`);
        const res = await robot.setActiveCamera(cam) as any;
        if (res && typeof res.activeCamera === 'number') {
          activeRemoteCam = res.activeCamera;
          if (robot.info && typeof res.hardwareCameras === 'number') {
            robot.info.cameras = res.hardwareCameras;
          }
          appendLog(`[Camera] Active camera is now ${activeRemoteCam === REMOTE_ARM_CAM ? 'Arm (1)' : 'Base (0)'}`);
        }
      } catch (e) {
        console.warn('setActiveCamera failed', e);
        appendLog(`[Camera] Camera switch error: ${e}`);
      }
    }
    paintRemote(REMOTE_BASE_CAM);
    paintRemote(REMOTE_ARM_CAM);
  }

  let latestBaseDetections: MarkerDetection[] = [];
  let latestArmDetections: MarkerDetection[] = [];
  let armObsCorners = new Map<number, Corners>();

  /** Blit the newest decoded frame into whichever panel canvas owns that camera. */
  function paintRemote(cam: number) {
    const src = remoteCams.canvas(cam);
    const dst = cam === REMOTE_BASE_CAM ? remoteBaseCanvas : remoteArmCanvas;
    if (!dst) return;
    if (src && (dst.width !== src.width || dst.height !== src.height)) {
      dst.width = src.width;
      dst.height = src.height;
    } else if (!src && (dst.width !== 640 || dst.height !== 480)) {
      dst.width = 640;
      dst.height = 480;
    }
    const ctx = dst.getContext('2d');
    if (!ctx) return;

    if (src) {
      ctx.drawImage(src, 0, 0);
    } else {
      ctx.fillStyle = '#111318';
      ctx.fillRect(0, 0, dst.width, dst.height);
    }

    const detections = cam === REMOTE_BASE_CAM ? latestBaseDetections : latestArmDetections;
    if (detections && detections.length > 0) {
      ctx.save();
      for (const det of detections) {
        const c = det.corners;
        ctx.strokeStyle = '#00ff66';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(c[0][0], c[0][1]);
        ctx.lineTo(c[1][0], c[1][1]);
        ctx.lineTo(c[2][0], c[2][1]);
        ctx.lineTo(c[3][0], c[3][1]);
        ctx.closePath();
        ctx.stroke();

        const cx = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4;
        const cy = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4;
        ctx.fillStyle = 'rgba(0, 0, 0, 0.75)';
        ctx.fillRect(cx - 24, cy - 14, 48, 20);
        ctx.fillStyle = '#00ff66';
        ctx.font = 'bold 12px monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(`ID ${det.id}`, cx, cy - 4);
      }
      ctx.restore();
    }
  }
  let camDeviceId = $state<string>(''); // selected onboard/base camera (blank → default)
  // `baseCamOff` is the user having deliberately turned the base camera off — it
  // stops real mode from starting it back up. The bandwidth levers themselves
  // (one-at-a-time, capture size) live in Settings.
  let baseCamOff = $state(false);
  let grabCtx: CanvasRenderingContext2D | null = null;
  let srcMat: any = null;
  let grayMat: any = null;
  let tagCentresMm: TagCentres = new Map();
  let lastDetect = 0;
  // What the last base-camera frame yielded, shown beside the cell label so a
  // silent detection failure is visible rather than inferred.
  let seenIds = $state<number[]>([]);

  // Arm-mounted camera (separate from the base camera): a raise-and-photograph
  // pipeline that finds a tag on the ArUco board and picks it up.
  let armVideo: HTMLVideoElement;
  let armPreview = $state<HTMLVideoElement>();
  let armCamStream = $state<MediaStream | null>(null);
  // Vision steps work off a real arm camera or the rendered one — detection takes
  // whichever exists, so gating those buttons on a USB stream locked them out of
  // the simulation for no reason.
  // A remote robot's arm camera counts too — the pick steps only need *a* stream
  // of arm-eye frames, and gating on the local USB one locked remote mode out of
  // the whole pick pipeline.
  const armCamReady = $derived(!!armCamStream || !!armPovView || remoteArmLive);
  let armGrabCtx: CanvasRenderingContext2D | null = null;
  let armSrcMat: any = null;
  let armGrayMat: any = null;
  let armCamW = $state(640);
  let armCamH = $state(480);
  let videoDevices = $state<MediaDeviceInfo[]>([]);
  let armCamDeviceId = $state<string>('');
  let armView = $state(false); // arm raised to a top-down "look at the board" pose
  let armPickMsg = $state<string | null>(null);
  // Running log of the pick. Every step already narrates itself into armPickMsg,
  // but each line overwrites the last, so a failure four steps in leaves no trace
  // of how it got there. Mirroring the messages keeps that history.
  let pickLog = $state<string[]>([]);
  const PICK_LOG_MAX = 300;
  let pickLogT0 = 0;
  // The buffer is a plain array on purpose. Appending by reading `pickLog` inside
  // the effect that writes it makes the effect depend on its own output, which
  // re-runs it forever and takes the whole component down with it.
  let pickLogBuf: string[] = [];

  function appendLog(msg: string) {
    if (!pickLogT0) pickLogT0 = performance.now();
    const t = ((performance.now() - pickLogT0) / 1000).toFixed(1).padStart(5);
    pickLogBuf = [...pickLogBuf, `${t}s  ${msg}`].slice(-PICK_LOG_MAX);
    pickLog = pickLogBuf;
  }

  $effect(() => {
    const msg = armPickMsg;
    if (!msg) return;
    appendLog(msg);
  });
  let armPickBusy = $state(false);
  let logOpen = $state(false);
  /**
   * The pose the arm eases to in order to look at whatever the robot is parked at,
   * as joint angles (rad): [pan, lift, elbow, wrist_flex, wrist_roll].
   */
  const VIEW_POSE = [0.0, -1.5203, 0.4602, 1.553, 0.0];
  /**
   * Where to look if that pose shows nothing: [pan°, lift°, wrist_flex°] offsets,
   * tried in order.
   *
   * Panning dominates because from this pose the surface is already square below
   * the camera — the only question left is left-or-right. One fixed pose would
   * assume the base parked exactly where it meant to, and a couple of degrees of
   * heading error is enough to slide the board out of frame.
   */
  const VIEW_SWEEP: [number, number, number][] = [
    [0, 0, 0],
    [-12, 0, 0], [12, 0, 0],
    [-24, 0, 0], [24, 0, 0],
    [-36, 0, 0], [36, 0, 0],
    [0, 0, 10], [0, 0, -10],
  ];
  // Object/board tags the arm camera currently sees (for one-click pick + tag view).
  let armDetectedTags = $state<number[]>([]);
  let tagsFromArm = false; // whether tagPoses came from the arm camera vs the base camera
  let lastArmDetect = 0;

  let session: Session | null = null;
  let solver: IKSolver | null = null;
  let renderer: MujocoRenderer | null = null;
  // Stands in for the onboard camera when no real one is attached: the same
  // scene, rendered from where the robot's camera would be. Lets the whole
  // explore-and-discover flow be exercised with nothing plugged in.
  // The robot's-eye views. They render the main renderer's scene from the camera
  // poses, so the geometry is built and animated once however many views exist.
  let povCanvas = $state<HTMLCanvasElement>();
  let armPovCanvas = $state<HTMLCanvasElement>();
  let baseCamOverlayCanvas = $state<HTMLCanvasElement>();
  let armCamOverlayCanvas = $state<HTMLCanvasElement>();
  let povView: PovView | null = null;
  let armPovView = $state<PovView | null>(null);
  // Bumped whenever a scene finishes loading. `session`/`mj` are plain lets, so an
  // effect can't react to them being assigned — without this the POV renderer is
  // only ever built if its canvas happened to mount after MuJoCo had loaded,
  // which it usually hasn't (hence a permanently black camera view).
  let sceneEpoch = $state(0);
  let mj: Awaited<ReturnType<typeof loadMujocoModule>> | null = null;

  // ── robot selection + mobile-base placement ─────────────────────────────────
  // LeKiwi by default: the stations are spread around the room, so exploring and
  // driving to them is the whole job — the fixed arm can only ever reach whatever
  // happens to be within arm's length of it. A stored choice still wins.
  let selectedRobot = $state<RobotId>(loadRobotId() ?? 'lekiwi');
  // Base position relative to the arm mount (m). Tuned live via the base mocap
  // body, so no remount is needed while dialing it in.
  // Fixed by the robot's definition, not adjustable and not stored. It was a live
  // control saved to localStorage, which is a poor fit for a measurement of how
  // the hardware is bolted together: the value was right in one browser and
  // absent everywhere else, and nothing said so.
  const initOffset = robotById(selectedRobot).base?.defaultOffset ?? [0, 0, 0];
  // Whole-robot world pose (LeKiwi drives/turns around the sim).
  let robotX = $state(0.412);
  let robotY = $state(0.046);
  let robotYawDeg = $state(-19.7);

  // Custom user-uploaded or ZIP-bundled 3D STL meshes mapped by name (e.g. "gear.stl")
  let customStlAssets = $state<Record<string, Uint8Array>>({});

  // Latest drive intent as amounts of the three primitive directions (forward,
  // back-left, back-right) plus rotation, each −1..1. applyBaseMotion turns it into
  // the sim pose; wheelSpeeds turns it into the real wheel command.
  let baseVel = $state({ fwd: 0, bl: 0, br: 0, turn: 0 });
  const zeroBaseVel = () => { baseVel.fwd = 0; baseVel.bl = 0; baseVel.br = 0; baseVel.turn = 0; };

  // The drive triad sets which primitive directions are held.
  function setDrive(fwd: number, bl: number, br: number) {
    // The joystick emits (0,0,0) every frame while idle, so whenever something
    // else owns the base its input has to be ignored or it wipes the command.
    if (!hasBase || navigating || exploring || nudging || baseLink.testing) return;
    baseVel.fwd = fwd;
    baseVel.bl = bl;
    baseVel.br = br;
  }
  function turnRobot(direction: number) {
    if (!hasBase || navigating || exploring || nudging || baseLink.testing) return;
    baseVel.turn = direction;
  }

  // Integrate the drive intent into the sim robot pose (the sim is the ground truth
  // for what a command means; calibration makes the real base match it).
  //
  // While navigating for real this is skipped on purpose: the physical robot is
  // the one moving, and the sim pose is resynced afterwards with "Reset MuJoCo
  // position" rather than dead-reckoned. In sim there is no real robot, so
  // integrating here is the only thing that makes the base actually go.
  function applyBaseMotion() {
    if (!hasBase) return;
    if (navigating && mode === 'real') return;
    const D = PRIMITIVE_DIRS;
    // Sum the held primitives into a body velocity (vx forward, vy left).
    let vx = baseVel.fwd * D.forward[0] + baseVel.bl * D.backLeft[0] + baseVel.br * D.backRight[0];
    let vy = baseVel.fwd * D.forward[1] + baseVel.bl * D.backLeft[1] + baseVel.br * D.backRight[1];
    let wz = baseVel.turn;
    if (Math.abs(vx) < 1e-4 && Math.abs(vy) < 1e-4 && Math.abs(wz) < 1e-4) return;

    // Simulation-only wheel jitter & slip (realistic physical imperfections):
    // In real life, 3-omni wheels experience slight friction variances, micro-slips,
    // and ground unevenness. In sim mode, we add subtle perturbations so the control loop
    // actively corrects motion rather than relying on perfect algebraic dead-reckoning.
    if (mode === 'sim') {
      const speed = Math.hypot(vx, vy);
      if (speed > 0.01) {
        // Micro variations in forward/lateral travel (±2.5% random variation)
        const slipX = (Math.random() - 0.5) * 0.035 * speed;
        const slipY = (Math.random() - 0.5) * 0.035 * speed;
        // Subtle cross-coupling and heading drift from omni roller rotation (±0.03°/frame)
        const slipTurn = (Math.random() - 0.5) * 0.045 * speed;
        vx += slipX;
        vy += slipY;
        wz += slipTurn;
      }
    }

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
  let lastWheelSent = 0;
  let armSending = false; // an arm position write is in flight
  let wheelSending = false; // a wheel speed write is in flight
  // Onboard-camera navigation to the shelf tag.
  // Shelf tag in the camera frame: [x, y, z] metres + `sq` = how far off square-on
  // the tag face is (rad; 0 = viewing it head-on, sign = which way it's rotated).
  let shelfTagCam = $state<[number, number, number, number] | null>(null);
  let navigating = $state(false); // auto-driving toward the shelf tag
  const SCAN_STANDOFF = 0.34; // standoff for wide-angle camera OCR scan
  const WORK_STANDOFF = 0.18; // close standoff for arm picking reach
  let navStandoff = $state(WORK_STANDOFF);
  /**
   * Where to park to do everything from.
   *
   * Seeing and reaching looked like they wanted different distances — far enough
   * to get the board in frame, near enough for the arm to cross it — and there was
   * a two-stage drive to satisfy both. That was a workaround for looking from the
   * wrong pose: from VIEW_POSE the board is plainly visible from here, so
   * one stop does both jobs.
   */
  const NAV_PICK_M = 0.12;
  /**
   * The same standoff as the camera measures it.
   *
   * navStandoff is the distance from the *robot* to the tag — the number shown
   * against each place, and the one worth thinking in, because it is the arm's
   * reach that matters. The onboard camera sits CAM_BASE_FWD ahead of the robot,
   * so it reads that much less. Comparing its depth against navStandoff directly
   * parked the robot a whole camera-offset too far back — asking for 0.21 m and
   * getting 0.32 — which is where the arm's missing reach was going.
   */
  const navCamDepth = $derived(Math.max(0.05, navStandoff - CAM_BASE_FWD));
  let navSquareSign = $state(1); // flip if the square-up turns the wrong way
  // Simple test-and-flip direction calibration: pulse one body motion, watch the
  // real base vs the sim, flip that axis if they disagree.
  // Shelves (LeKiwi only): configurable count + independent per-drawer open amount.
  let shelfCount = $state(3);
  let shelfOpens = $state<number[]>([0, 0, 0]); // open amount (m) per drawer
  // Movable/adjustable shelf placement (world). Placed to the left, farther out.
  // Out with the bottle and plant, well clear of the robot's working area — it
  // used to sit at (-0.36, 0.6), close enough to block the onboard camera.
  let shelfX = $state(-0.80);
  let shelfY = $state(-2.35);
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
  // Fixed STL display correction — the arm mesh is authored off-angle on the base.
  // Not a user setting; it never varies for a given robot.
  const armYawDeg = -45;
  let hasBase = $state(false); // true once a mobile base is mounted in the scene
  let armLift = 0; // how far the arm is raised on the base (m) so wheels touch the floor
  // Mocap body indices, resolved by name each load (order-independent). -1 = absent.
  let blockMocapId = 0;
  let baseMocapId = -1;
  let armMocapId = -1;
  let raf = 0;

  // ── Physics pacing ──────────────────────────────────────────────────────────
  // MuJoCo's timestep here is 2 ms, and one mj_step per animation frame advances
  // the simulation 2 ms per ~16 ms of wall clock — the arm runs at an eighth of
  // real speed no matter what the speed limit says, because the limiter is in
  // wall-clock deg/s while the actuators track in sim time. Stepping enough times
  // per frame to keep up fixes that; the cap stops a stalled tab from trying to
  // catch up on seconds of missed time all at once.
  const SIM_MAX_STEPS = 24;
  let simClockMs = 0;

  /** How many physics steps to run this frame to keep sim time with real time. */
  function physicsStepsDue(): number {
    const now = performance.now();
    const ts = ((session?.model as any)?.opt?.timestep as number) || 0.002;
    if (!simClockMs) {
      simClockMs = now;
      return 1;
    }
    const dt = Math.min(0.25, (now - simClockMs) / 1000);
    simClockMs = now;
    return Math.max(1, Math.min(SIM_MAX_STEPS, Math.round(dt / ts)));
  }

  // ── sim-mode physics (block is a real free body, arm driven by actuators) ────
  // Whether the current scene is the physics (sim-only) variant. Reactive because
  // the markup gates on it: it is set while the scene loads, long after the first
  // render, and a plain `let` would leave anything guarded by it stuck at the
  // value it had at mount — which is exactly how the block-randomiser button came
  // to never appear.
  let physics = $state(false);
  let loadedMode: 'sim' | 'real' | null = null; // which mode the scene was built for
  let graspSiteId = -1;
  let fingerSiteIds: [number, number] = [-1, -1]; // static, moving fingertips
  /**
   * Every movable prop, keyed by the tag the pick aims at → its free-joint
   * addresses. The block used to be the only one, and every routine that knew
   * about "the block" silently did nothing for anything else — an apple could be
   * approached, gripped and lifted while staying exactly where it was.
   */
  let propAdr = new Map<number, { q: number; d: number }>();
  let blockQadr = -1; // free-block qpos address (pos[3] + quat[4])
  let blockDadr = -1; // free-block dof address (lin[3] + ang[3])
  // Relative transform of the block in the grasp-site frame, captured on grab.

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
    // The base/arm meshes are authored ~45° off (STL mismatch), so the whole body
    // is rendered yawed by BODY_YAW_OFFSET about the base — the driving/logic yaw
    // (robotYawDeg) is unchanged; only the placement of the geometry is corrected.
    const psi = (robotYawDeg * Math.PI) / 180 + BODY_YAW_OFFSET;
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

  /**
   * Drop the block somewhere arbitrary on the station's board, at an arbitrary
   * angle — so a pick can be tried against a spread of approach geometries rather
   * than the one pose that happens to be easy.
   *
   * BLOCK_SCATTER_R keeps the whole block on the board whatever the angle.
   */
  function randomiseBlock() {
    const a = Math.random() * 2 * Math.PI;
    const r = BLOCK_SCATTER_R * Math.sqrt(Math.random()); // sqrt → uniform over the area
    simBlockX = STATION_X + r * Math.cos(a);
    simBlockY = STATION_Y + r * Math.sin(a);
    simBlockYawDeg = Math.round(Math.random() * 360);
    // Reported through copyMsg, not armPickMsg: that one renders in the arm-camera
    // panel, so clicking a button up here appeared to do nothing at all.
    copyMsg = `Block moved ${(r * 1000).toFixed(0)} mm off centre, turned ${simBlockYawDeg}°.`;
  }

  // ── Holding the block ───────────────────────────────────────────────────────
  //
  // Once both fingers are genuinely around the object, it rides with the gripper.
  //
  // This is not a shortcut past the perception — the fingers have to actually be
  // on opposite sides of the block and close to it before anything locks, so a
  // mis-aimed grasp still misses. It is a shortcut past MuJoCo's contact solver,
  // where a friction grip on a small box is delicate in a way a real rubber-padded
  // gripper is not: the sim would have the jaws visibly around the block and still
  // let it sit there while the arm lifted away.
  let grabbedRel: { p: number[]; R: number[] } | null = null;
  /**
   * Whether anything is currently in the jaws, for the markup to read.
   *
   * Separate from grabbedRel because that is rewritten on every physics step and
   * is deliberately not reactive — assigning $state at frame rate would re-run
   * every effect that touches it. This flips only when the hold starts or ends.
   */
  let holdingItem = $state(false);

  /** The gripper's frame right now: rotation (row-major) and origin. */
  function graspFrame(): { R: number[]; t: number[] } | null {
    if (!session || graspSiteId < 0) return null;
    try {
      const xm = session.data.site_xmat as Float64Array;
      const xp = session.data.site_xpos as Float64Array;
      const o = graspSiteId * 9, p = graspSiteId * 3;
      return {
        R: [xm[o], xm[o + 1], xm[o + 2], xm[o + 3], xm[o + 4], xm[o + 5], xm[o + 6], xm[o + 7], xm[o + 8]],
        t: [xp[p], xp[p + 1], xp[p + 2]],
      };
    } catch {
      return null;
    }
  }

  /** Both fingertips against the block, with the block between them. */
  function fingersOnBlock(tag: number = blockTag): boolean {
    const adr = propAdr.get(tag);
    if (!session || !adr || fingerSiteIds[0] < 0 || fingerSiteIds[1] < 0) return false;
    try {
      const sx = session.data.site_xpos as Float64Array;
      const q = session.data.qpos as Float64Array;
      const blockQadr = adr.q;
      const a = [sx[fingerSiteIds[0] * 3], sx[fingerSiteIds[0] * 3 + 1], sx[fingerSiteIds[0] * 3 + 2]];
      const b = [sx[fingerSiteIds[1] * 3], sx[fingerSiteIds[1] * 3 + 1], sx[fingerSiteIds[1] * 3 + 2]];
      const c = [q[blockQadr], q[blockQadr + 1], q[blockQadr + 2]];
      const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
      const gap = Math.hypot(d[0], d[1], d[2]);
      if (gap < 1e-6 || gap > 0.06) return false; // jaws wide open: nothing is held
      const u = [d[0] / gap, d[1] / gap, d[2] / gap];
      const ca = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
      const sa = ca[0] * u[0] + ca[1] * u[1] + ca[2] * u[2];
      if (sa <= 0 || sa >= gap) return false; // block centre is not between the jaws
      // …and the jaw line passes close to it, rather than above or beside it.
      const perp = Math.hypot(ca[0] - sa * u[0], ca[1] - sa * u[1], ca[2] - sa * u[2]);
      return perp < 0.025;
    } catch {
      return false;
    }
  }

  /** Latch the block to the gripper while it's held; let go when the jaws open. */
  function updateGrasp() {
    const adr = propAdr.get(blockTag);
    if (!session || !adr) return;
    const blockQadr = adr.q, blockDadr = adr.d;
    const closing = gripperCmd > (gripperRange[0] + gripperRange[1]) / 2;
    if (!closing) {
      if (grabbedRel) holdingItem = false;
      grabbedRel = null;
      return;
    }
    const g = graspFrame();
    if (!g) return;
    const q = session.data.qpos as Float64Array;
    if (!grabbedRel) {
      if (!fingersOnBlock(blockTag)) return;
      holdingItem = true;
      // Remember where the block sits in the gripper's frame, and hold that.
      const d = [q[blockQadr] - g.t[0], q[blockQadr + 1] - g.t[1], q[blockQadr + 2] - g.t[2]];
      const Rt = [g.R[0], g.R[3], g.R[6], g.R[1], g.R[4], g.R[7], g.R[2], g.R[5], g.R[8]];
      const bq = [q[blockQadr + 3], q[blockQadr + 4], q[blockQadr + 5], q[blockQadr + 6]];
      grabbedRel = {
        p: [
          Rt[0] * d[0] + Rt[1] * d[1] + Rt[2] * d[2],
          Rt[3] * d[0] + Rt[4] * d[1] + Rt[5] * d[2],
          Rt[6] * d[0] + Rt[7] * d[1] + Rt[8] * d[2],
        ],
        R: matMul3x3(Rt, quatToMat(bq)),
      };
    }
    const rel = grabbedRel;
    q[blockQadr] = g.t[0] + g.R[0] * rel.p[0] + g.R[1] * rel.p[1] + g.R[2] * rel.p[2];
    q[blockQadr + 1] = g.t[1] + g.R[3] * rel.p[0] + g.R[4] * rel.p[1] + g.R[5] * rel.p[2];
    q[blockQadr + 2] = g.t[2] + g.R[6] * rel.p[0] + g.R[7] * rel.p[1] + g.R[8] * rel.p[2];
    const bq = matToQuat(matMul3x3(g.R, rel.R));
    q[blockQadr + 3] = bq[0];
    q[blockQadr + 4] = bq[1];
    q[blockQadr + 5] = bq[2];
    q[blockQadr + 6] = bq[3];
    const v = session.data.qvel as Float64Array;
    for (let k = 0; k < 6; k++) v[blockDadr + k] = 0;
  }

  /** Quaternion (w,x,y,z) → row-major 3×3. */
  function quatToMat(q: number[]): number[] {
    const [w, x, y, z] = q;
    return [
      1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
    ];
  }

  /**
   * The physics block's height (m), or null when there isn't one to read.
   *
   * Used to tell a grasp from a miss. "The arm lifted 86 mm" says nothing about
   * whether anything came with it — the gripper closing on air reports exactly the
   * same number, which made a failed pick indistinguishable from a good one.
   */
  function blockHeightM(tag: number = blockTag): number | null {
    const adr = propAdr.get(tag);
    if (!session || !adr) return null;
    return (session.data.qpos as Float64Array)[adr.q + 2];
  }

  /** Reset the physics block to a resting pose at [x, y], rotated `yawDeg`. */
  function resetPhysicsBlock(x: number, y: number, yawDeg: number) {
    if (!session || blockQadr < 0) return;
    const h = (yawDeg * Math.PI) / 180 / 2;
    const q = session.data.qpos as Float64Array;
    q[blockQadr] = x;
    q[blockQadr + 1] = y;
    // Drop it onto the station's top face rather than the floor.
    q[blockQadr + 2] = STATION_H + BLOCK_HALF_Z;
    q[blockQadr + 3] = Math.cos(h);
    q[blockQadr + 4] = 0;
    q[blockQadr + 5] = 0;
    q[blockQadr + 6] = Math.sin(h);
    const v = session.data.qvel as Float64Array;
    for (let k = 0; k < 6; k++) v[blockDadr + k] = 0;
  }

  // Advance the physics sim one frame: drive the arm actuators toward the IK
  // solution (computed on a snapshot so it doesn't disturb the live state),
  // step, then hold the block rigidly if it's grasped.
  function stepPhysics(
    solveTarget: [number, number, number],
    Rd: number[],
    grip: number,
    opts: {
      free?: boolean;
      rollRad?: number;
      dofLimits?: Record<number, [number, number]>;
      oriWeight?: number;
    } = {},
  ) {
    if (!session || !solver || !mj) return;
    const d = session.data;
    const nq = session.model.nq as number;
    const nv = session.model.nv as number;
    const qsave = Float64Array.from((d.qpos as Float64Array).subarray(0, nq));
    const vsave = Float64Array.from((d.qvel as Float64Array).subarray(0, nv));
    const ctrl = d.ctrl as Float64Array;
    const before = ARM_DOFS.map((i) => qsave[i]); // where the arm is right now
    let desired: number[];
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
      desired = [res.qpos[0], res.qpos[1], res.qpos[2], res.qpos[3], roll];
    } else {
      // Low orientation weight → best-effort: reach the position, approximate the
      // approach angle (a slight tilt is fine if it reaches better).
      const res = solver.solvePose(solveTarget, Rd, {
        dofIndices: ARM_DOFS,
        maxIters: 10,
        oriWeight: opts.oriWeight ?? 0.25,
        dofLimits: opts.dofLimits,
      });
      ikOk = res.ok;
      ikErrMm = res.error * 1000;
      (d.qpos as Float64Array).set(qsave);
      (d.qvel as Float64Array).set(vsave);
      // Wrist roll is commanded, not inferred. With five DOFs against a
      // position-plus-weak-orientation target, roll sits in the solver's null
      // space: it can drift or oscillate frame to frame while the end-effector
      // stays exactly where it should. Everything bolted to the wrist — the
      // bracket and the camera — then appear to rotate on their own.
      //
      // Unless a visual step is holding an orientation, in which case the roll it
      // asked for is *in* that orientation and pinning the joint here silently
      // cancelled it — the wrist simply never turned.
      desired = [res.qpos[0], res.qpos[1], res.qpos[2], res.qpos[3], opts.rollRad ?? res.qpos[4]];
    }
    // Rate-limit the IK solution to the global max arm speed before commanding it.
    const pose = limitPose(desired, before);
    for (let i = 0; i < 5; i++) ctrl[i] = pose[i];
    // Gripper: same open/close flip the renderer uses, so a "close" command
    // physically closes the model's fingers.
    ctrl[5] = gripperRange[0] + gripperRange[1] - grip;

    const steps = physicsStepsDue();
    for (let k = 0; k < steps; k++) {
      mj.mj_step(session.model, d);
    }
    jointAngles = ARM_DOFS.map((i) => (d.qpos as Float64Array)[i]);

    updateGrasp(); // carry the block with the jaws once they're around it
    readShelfOpens();
    mj.mj_forward(session.model, d);
  }

  // The block is gripped for real: it shares a collision channel with the arm and
  // is held by friction between the fingers. It used to be pinned to the grasp
  // site by a transform captured when the gripper closed, which meant it never
  // behaved like an object — it could not be nudged, slip, or be dropped.

  const staticAssetCache = new Map<string, Uint8Array | string>();
  let loadSceneBusy = false;
  let queuedReloadRobotId: RobotId | null = null;

  /**
   * Fetch + mount the selected robot's scene, replacing any current one. On
   * mobile robots the arm root is a mocap body, so the x/y/z placement offset
   * moves the arm LIVE (via setArmMocap) with no remount; the base stays planted.
   */
  async function loadScene(robotId: RobotId) {
    if (!mj) return;
    if (loadSceneBusy) {
      queuedReloadRobotId = robotId;
      return;
    }
    loadSceneBusy = true;
    const t0 = performance.now();
    ready = false;
    errorMsg = null;
    status = 'Loading model…';
    renderer?.dispose();
    povView?.dispose();
    armPovView?.dispose();
    renderer = null;
    session?.dispose();
    session = null;

    try {
      const robot = robotById(robotId);
      const xmlUrl = `/models/so101/${XML}`;
      let armXml: string;
      if (staticAssetCache.has(xmlUrl)) {
        armXml = staticAssetCache.get(xmlUrl) as string;
      } else {
        armXml = await (await fetch(xmlUrl)).text();
        staticAssetCache.set(xmlUrl, armXml);
      }

      const files: ModelFiles = {};
      for (const name of ASSETS) {
        const url = `/models/so101/assets/${name}`;
        let buf: Uint8Array;
        if (staticAssetCache.has(url)) {
          buf = staticAssetCache.get(url) as Uint8Array;
        } else {
          const res = await fetch(url);
          buf = new Uint8Array(await res.arrayBuffer());
          staticAssetCache.set(url, buf);
        }
        files[`assets/${name}`] = buf;
        files[name] = buf;
      }

      hasBase = false;
      armLift = 0;
      let manifest: BaseManifest | null = null;
      if (robot.base) {
        const manifestUrl = `/models/${robot.base.dir}/${robot.base.manifest}`;
        let manifestText: string;
        if (staticAssetCache.has(manifestUrl)) {
          manifestText = staticAssetCache.get(manifestUrl) as string;
        } else {
          manifestText = await (await fetch(manifestUrl)).text();
          staticAssetCache.set(manifestUrl, manifestText);
        }
        manifest = JSON.parse(manifestText) as BaseManifest;
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
      let sceneXml = buildBoardSceneXml(withWristRollRef(armXml), {
        physics,
        availableMeshes: Object.keys(customStlAssets),
      });
      if (robot.base && manifest) {
        sceneXml = injectBase(sceneXml, manifest);
        for (const m of manifest.meshes) {
          const mUrl = `/models/${robot.base.dir}/assets/${m.file}`;
          let mBuf: Uint8Array;
          if (staticAssetCache.has(mUrl)) {
            mBuf = staticAssetCache.get(mUrl) as Uint8Array;
          } else {
            const res = await fetch(mUrl);
            mBuf = new Uint8Array(await res.arrayBuffer());
            staticAssetCache.set(mUrl, mBuf);
          }
          files[`assets/${m.file}`] = mBuf;
          files[m.file] = mBuf;
        }
        hasBase = true;
        // Shelves are part of the mobile-robot scene (LeKiwi only).
        sceneXml = injectShelves(sceneXml, shelfCount);
        loadedShelfCount = shelfCount;
      }

      // Mount any user-uploaded or ZIP-bundled custom STL meshes into all MuJoCo virtual search paths
      for (const [name, buf] of Object.entries(customStlAssets)) {
        const cleanName = name.replace(/^assets\//, '').replace(/^models\//, '');
        files[name] = buf;
        files[`assets/${cleanName}`] = buf;
        files[`models/${cleanName}`] = buf;
        files[`assets/models/${cleanName}`] = buf;
        files[cleanName] = buf;
      }

      files['scene.xml'] = new TextEncoder().encode(sceneXml);

      session = mountModel(mj, files, 'scene.xml');
      solver = new IKSolver(mj, session.model, session.data, GRASP_SITE);
      blockMocapId = physics ? -1 : mocapIdOf('block');
      mountMocapId = mocapIdOf('wrist_cam_mount');
      mountBodyId = mj.mj_name2id(session.model, 1 /* mjOBJ_BODY */, 'wrist_cam_mount');
      baseMocapId = hasBase ? mocapIdOf('lekiwi_base') : -1;
      armMocapId = hasBase ? mocapIdOf('base') : -1;
      graspSiteId = mj.mj_name2id(session.model, 6 /* mjOBJ_SITE */, GRASP_SITE);
      fingerSiteIds = [
        mj.mj_name2id(session.model, 6 /* mjOBJ_SITE */, 'static_fingertip'),
        mj.mj_name2id(session.model, 6 /* mjOBJ_SITE */, 'moving_fingertip'),
      ];
      if (physics) {
        const jid = mj.mj_name2id(session.model, 3 /* mjOBJ_JOINT */, 'block_free');
        blockQadr = jid >= 0 ? (session.model.jnt_qposadr as Int32Array)[jid] : -1;
        blockDadr = jid >= 0 ? (session.model.jnt_dofadr as Int32Array)[jid] : -1;
        propAdr = new Map();
        if (blockQadr >= 0) propAdr.set(STATION_OBJECT_TAG, { q: blockQadr, d: blockDadr });
        for (const st of STATIONS) {
          if (!st.propTag) continue;
          const pj = mj.mj_name2id(session.model, 3, `prop_${st.navTag}_free`);
          if (pj < 0) continue;
          propAdr.set(st.propTag, {
            q: (session.model.jnt_qposadr as Int32Array)[pj],
            d: (session.model.jnt_dofadr as Int32Array)[pj],
          });
        }
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
      robotGeomIds = computeRobotGeoms();
      session.forward();
      target = solver.sitePosition();

      // Gripper ctrlrange (joint index 5) for the gripper slider.
      const cr = Array.from(session.model.actuator_ctrlrange as ArrayLike<number>);
      if (cr[10] < cr[11]) gripperRange = [cr[10], cr[11]];

      renderer = new MujocoRenderer(canvas, mj, session.model, session.data);
      renderer.setTarget(target);
      povView?.dispose();
      povView = null;
      armPovView?.dispose();
      armPovView = null;
      sceneEpoch += 1;
      buildTagTextures();
      fitCanvas();
      ready = true;
      status = 'Ready';
      console.log(`[Simulator] Scene compiled & ready in ${(performance.now() - t0).toFixed(1)}ms (${Object.keys(customStlAssets).length} custom STLs mounted)`);
    } finally {
      loadSceneBusy = false;
      if (queuedReloadRobotId) {
        const nextId = queuedReloadRobotId;
        queuedReloadRobotId = null;
        loadScene(nextId);
      }
    }
  }

  onMount(async () => {
    try {
      // Needed even with no hardware: the sim paints real markers and runs the
      // real detector over its rendered frames.
      loadCv().then((c) => {
        cv = c;
        detector = createDetector(c);
        tagCentresMm = boardTagCentres(SQUARE_MM, 16, 2, 10, 8);
        buildTagTextures();
      }).catch((e) => {
        detectMsg = 'OpenCV failed to load: ' + (e instanceof Error ? e.message : String(e));
      });
      initTesseract().catch(() => {});
      mj = await loadMujocoModule();
      await loadScene(selectedRobot);
      loop();

      if (loadAutoConnect()) {
        const savedRemote = loadRemoteConfig();
        if (savedRemote) {
          const signalOrigin =
            (typeof location !== 'undefined' && (location.hostname === 'localhost' || location.hostname === '127.0.0.1'))
              ? 'wss://instant.river.berlin'
              : (typeof location !== 'undefined' ? `${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}` : 'wss://instant.river.berlin');
          armLink.run(async () => {
            await connectRobotRemote({
              signalOrigin,
              room: savedRemote.room,
              token: savedRemote.token,
            });
          });
        }
      }
    } catch (e) {
      errorMsg = e instanceof Error ? e.message : String(e);
      status = 'Failed to load';
    }
  });

  async function onRobotChange(id: RobotId) {
    if (id === selectedRobot && session) return;
    selectedRobot = id;
    saveRobotId(id);
    const off = robotById(id).base?.defaultOffset ?? [0, 0, 0];
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
    const rx = robotX, ry = robotY, rψ = robotYawDeg, ayaw = armYawDeg;
    void rx; void ry; void rψ; void ayaw; // tracked so the robot re-places on change
    if (hasBase && session) placeRobot();
  });

  // Auto-navigate runner for Pick Setup & Goal Square Visual Servoing
  $effect(() => {
    pickCalState.registerScanPosRunner(async () => {
      setArmMode('active');
      armPickMsg = 'Moving arm to scan position (pan 0, roll 0)...';
      await raiseToView();
      targetToCurrentEE();
      return {
        success: true,
        msg: 'Arm is in scan position pointing straight down. Ready to auto-navigate!',
      };
    });

    pickCalState.registerAutoNavRunner(async (
      goal: GoalQuadConfig,
      tagId: number,
      mode: AutoNavMode = 'all',
      stepDelayMs = 400,
      stepGain = 0.5,
      downStepMm = 6,
      tolerancePx = 14
    ) => {
      setArmMode('active');
      const modeLabel = mode === 'xy' ? 'X-Y Lateral' : mode === 'z' ? 'Z Height' : 'Stepwise (X-Y → Descend)';
      armPickMsg = `Auto-navigating ${modeLabel} to Tag #${tagId}...`;
      targetToCurrentEE();

      const [gx, gy] = computeQuadCenter(goal.corners);
      const goalSize = computeQuadAvgSize(goal.corners);
      const goalYaw = computeQuadOrientationDeg(goal.corners);
      const tol = Math.max(3, tolerancePx);
      const gain = Math.max(0.1, Math.min(1.0, stepGain));

      if (mode === 'xy') {
        // Pure X-Y lateral alignment until within tolerance region
        const maxSteps = 20;
        let step = 0;
        let matched = false;
        while (step < maxSteps) {
          if (pickCalState.abortRequested) break;
          step++;
          const c = armObsCorners.get(tagId);
          if (!c || c.length < 4) {
            if (step === 1) {
              await raiseToView();
              await new Promise((r) => setTimeout(r, Math.max(stepDelayMs, 500)));
              continue;
            }
            await new Promise((r) => setTimeout(r, stepDelayMs));
            continue;
          }
          const cx = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4;
          const cy = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4;
          const tagRotDeg = (Math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0]) * 180) / Math.PI;
          const errX = cx - gx;
          const errY = cy - gy;
          let errYaw = tagRotDeg - goalYaw;
          while (errYaw > 180) errYaw -= 360;
          while (errYaw < -180) errYaw += 360;

          const distPx = Math.hypot(errX, errY);
          if (distPx <= tol && Math.abs(errYaw) <= 5) {
            matched = true;
            break;
          }

          const a = robotYawRad();
          const curr = target ? [...target] : ([...solver!.sitePosition()] as [number, number, number]);
          const dFwd = (-errY / 480) * 0.045 * gain;
          const dLat = (-errX / 640) * 0.045 * gain;
          curr[0] += dFwd * Math.cos(a) - dLat * Math.sin(a);
          curr[1] += dFwd * Math.sin(a) + dLat * Math.cos(a);
          graspRollDeg = +(graspRollDeg + (errYaw * 0.35 * gain)).toFixed(1);
          target = [curr[0], curr[1], curr[2]];
          renderer?.setTarget(target);

          await new Promise((r) => setTimeout(r, stepDelayMs));
        }
        targetToCurrentEE();
        if (pickCalState.abortRequested) {
          return { success: false, msg: '⏹️ X-Y auto-navigation stopped by user.' };
        }
        return {
          success: matched,
          msg: matched ? `✓ Tag #${tagId} aligned in X-Y tolerance region!` : `X-Y alignment reached closest point.`,
        };
      }

      if (mode === 'z') {
        // Pure Z elevation until side length matches
        const maxSteps = 16;
        let step = 0;
        let matched = false;
        while (step < maxSteps) {
          if (pickCalState.abortRequested) break;
          step++;
          const c = armObsCorners.get(tagId);
          if (!c || c.length < 4) {
            await new Promise((r) => setTimeout(r, stepDelayMs));
            continue;
          }
          const s0 = Math.hypot(c[1][0] - c[0][0], c[1][1] - c[0][1]);
          const s1 = Math.hypot(c[2][0] - c[1][0], c[2][1] - c[1][1]);
          const s2 = Math.hypot(c[3][0] - c[2][0], c[3][1] - c[2][1]);
          const s3 = Math.hypot(c[0][0] - c[3][0], c[0][1] - c[3][1]);
          const tagSize = (s0 + s1 + s2 + s3) / 4;
          const errSize = tagSize - goalSize;

          if (Math.abs(errSize) <= tol) {
            matched = true;
            break;
          }

          const curr = target ? [...target] : ([...solver!.sitePosition()] as [number, number, number]);
          const dZ = (errSize / goalSize) * 0.025 * gain;
          curr[2] = Math.max(0.02, Math.min(0.65, curr[2] + dZ));
          target = [curr[0], curr[1], curr[2]];
          renderer?.setTarget(target);

          await new Promise((r) => setTimeout(r, stepDelayMs));
        }
        targetToCurrentEE();
        if (pickCalState.abortRequested) {
          return { success: false, msg: '⏹️ Z auto-navigation stopped by user.' };
        }
        return {
          success: matched,
          msg: matched ? `✓ Tag #${tagId} height matched goal size!` : `Z navigation reached closest height.`,
        };
      }

      // Stepwise Approach (mode === 'all'):
      // Loop: 1) Align X-Y & yaw -> 2) Check if at goal quad error region -> 3) Descend downStepMm -> Repeat!
      const maxStages = 16;
      let stage = 0;
      let targetReached = false;

      while (stage < maxStages) {
        if (pickCalState.abortRequested) break;
        stage++;

        // 1. Align X-Y lateral position & orientation
        let xySteps = 0;
        while (xySteps < 12) {
          if (pickCalState.abortRequested) break;
          xySteps++;
          const c = armObsCorners.get(tagId);
          if (!c || c.length < 4) {
            if (stage === 1 && xySteps === 1) {
              await raiseToView();
              await new Promise((r) => setTimeout(r, Math.max(stepDelayMs, 500)));
              continue;
            }
            await new Promise((r) => setTimeout(r, stepDelayMs));
            continue;
          }

          const cx = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4;
          const cy = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4;
          const tagRotDeg = (Math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0]) * 180) / Math.PI;
          const errX = cx - gx;
          const errY = cy - gy;
          let errYaw = tagRotDeg - goalYaw;
          while (errYaw > 180) errYaw -= 360;
          while (errYaw < -180) errYaw += 360;

          const distPx = Math.hypot(errX, errY);
          if (distPx <= tol && Math.abs(errYaw) <= 5) {
            break; // X-Y alignment achieved for this stage
          }

          const a = robotYawRad();
          const curr = target ? [...target] : ([...solver!.sitePosition()] as [number, number, number]);
          const dFwd = (-errY / 480) * 0.045 * gain;
          const dLat = (-errX / 640) * 0.045 * gain;
          curr[0] += dFwd * Math.cos(a) - dLat * Math.sin(a);
          curr[1] += dFwd * Math.sin(a) + dLat * Math.cos(a);
          graspRollDeg = +(graspRollDeg + (errYaw * 0.35 * gain)).toFixed(1);
          target = [curr[0], curr[1], curr[2]];
          renderer?.setTarget(target);

          await new Promise((r) => setTimeout(r, stepDelayMs));
        }

        if (pickCalState.abortRequested) break;

        // 2. Check if all 4 corners of tag fall inside the goal quad tolerance region
        const cNow = armObsCorners.get(tagId);
        if (cNow && cNow.length >= 4) {
          let maxCornerDist = 0;
          for (let i = 0; i < 4; i++) {
            const d = Math.hypot(cNow[i][0] - goal.corners[i][0], cNow[i][1] - goal.corners[i][1]);
            if (d > maxCornerDist) maxCornerDist = d;
          }
          const s0 = Math.hypot(cNow[1][0] - cNow[0][0], cNow[1][1] - cNow[0][1]);
          const s1 = Math.hypot(cNow[2][0] - cNow[1][0], cNow[2][1] - cNow[1][1]);
          const s2 = Math.hypot(cNow[3][0] - cNow[2][0], cNow[3][1] - cNow[2][1]);
          const s3 = Math.hypot(cNow[0][0] - cNow[3][0], cNow[0][1] - cNow[3][1]);
          const tagSize = (s0 + s1 + s2 + s3) / 4;

          // If within tolerance border or reached/exceeded goal size
          if (maxCornerDist <= tol || tagSize >= goalSize - (tol * 0.5)) {
            targetReached = true;
            break;
          }
        }

        if (pickCalState.abortRequested) break;

        // 3. Move downward by downStepMm (e.g. 6mm)
        armPickMsg = `Stage ${stage}: Aligned X-Y, descending ${downStepMm}mm...`;
        const curr = target ? [...target] : ([...solver!.sitePosition()] as [number, number, number]);
        const deltaM = -(downStepMm / 1000);
        curr[2] = Math.max(0.02, Math.min(0.65, curr[2] + deltaM));
        target = [curr[0], curr[1], curr[2]];
        renderer?.setTarget(target);

        await new Promise((r) => setTimeout(r, Math.max(stepDelayMs, 300)));
      }

      targetToCurrentEE();
      if (pickCalState.abortRequested) {
        return {
          success: false,
          msg: '⏹️ Stepwise approach stopped by user.',
        };
      }
      if (targetReached) {
        return {
          success: true,
          msg: `✓ Tag #${tagId} reached target pick position inside the error region!`,
        };
      } else {
        return {
          success: true,
          msg: `Stepwise approach completed closest approach to Tag #${tagId}.`,
        };
      }
    });

    pickCalState.registerNudgeHeightRunner(async (deltaMm: number) => {
      setArmMode('active');
      targetToCurrentEE();
      const curr = target ? [...target] : ([...solver!.sitePosition()] as [number, number, number]);
      const deltaM = deltaMm / 1000;
      curr[2] = Math.max(0.02, Math.min(0.65, curr[2] + deltaM));
      target = [curr[0], curr[1], curr[2]];
      renderer?.setTarget(target);

      const dir = deltaMm < 0 ? 'down' : 'up';
      armPickMsg = `Moved arm ${Math.abs(deltaMm)} mm ${dir} to Z = ${(curr[2] * 1000).toFixed(1)} mm.`;
      return {
        success: true,
        msg: `Arm moved ${Math.abs(deltaMm)} mm ${dir} (Z = ${(curr[2] * 1000).toFixed(1)} mm).`,
      };
    });

    pickCalState.registerPickRunner(async (downMm: number = 20) => {
      setArmMode('active');
      targetToCurrentEE();

      // 1. Ensure gripper is open before descent
      if (gripperCmd > gripperRange[0] + 1e-3) {
        armPickMsg = 'Opening gripper before descending…';
        gripperCmd = Math.max(DEFAULT_PICK.gripperOpen, gripperRange[0]);
        await new Promise((r) => setTimeout(r, 600));
      }

      // 2. Move arm down to grasp object
      armPickMsg = `Moving ${downMm} mm down to grasp object…`;
      const curr = target ? [...target] : ([...solver!.sitePosition()] as [number, number, number]);
      const deltaM = -(downMm / 1000);
      curr[2] = Math.max(0.012, curr[2] + deltaM);
      target = [curr[0], curr[1], curr[2]];
      renderer?.setTarget(target);
      await new Promise((r) => setTimeout(r, 700));

      // 3. Close gripper and bind physics object
      armPickMsg = 'Closing gripper…';
      gripperCmd = Math.min(DEFAULT_PICK.gripperClose, gripperRange[1]);
      await new Promise((r) => setTimeout(r, 1000));

      const adr = propAdr.get(blockTag);
      const g = graspFrame();
      if (adr && g && session) {
        const q = session.data.qpos as Float64Array;
        const bQ = adr.q;
        const d = [q[bQ] - g.t[0], q[bQ + 1] - g.t[1], q[bQ + 2] - g.t[2]];
        const Rt = [g.R[0], g.R[3], g.R[6], g.R[1], g.R[4], g.R[7], g.R[2], g.R[5], g.R[8]];
        const bq = [q[bQ + 3], q[bQ + 4], q[bQ + 5], q[bQ + 6]];
        grabbedRel = {
          p: [
            Rt[0] * d[0] + Rt[1] * d[1] + Rt[2] * d[2],
            Rt[3] * d[0] + Rt[4] * d[1] + Rt[5] * d[2],
            Rt[6] * d[0] + Rt[7] * d[1] + Rt[8] * d[2],
          ],
          R: matMul3x3(Rt, quatToMat(bq)),
        };
      }
      holdingItem = true;

      // 4. Lift arm
      armPickMsg = 'Lifting arm with object…';
      curr[2] = Math.min(0.65, curr[2] + 0.10);
      target = [curr[0], curr[1], curr[2]];
      renderer?.setTarget(target);
      await new Promise((r) => setTimeout(r, 800));

      targetToCurrentEE();
      armPickMsg = '✓ Pick completed and arm lifted!';
      return {
        success: true,
        msg: `✓ Descended ${downMm}mm, gripped object, and lifted!`,
      };
    });

    return () => {
      pickCalState.unregisterAutoNavRunner();
      pickCalState.unregisterScanPosRunner();
      pickCalState.unregisterNudgeHeightRunner();
      pickCalState.unregisterPickRunner();
    };
  });

  // Each canvas lives in its own grid cell now, so measure the cell containers
  // rather than the raw canvas element so grid expansion is correctly tracked.
  function fitCanvas() {
    if (canvas && renderer) {
      const parent = canvas.parentElement;
      const r = parent ? parent.getBoundingClientRect() : canvas.getBoundingClientRect();
      if (r.width > 0 && r.height > 0) {
        renderer.resize(Math.max(1, Math.round(r.width)), Math.max(1, Math.round(r.height)));
      }
    }
  }

  // Robot geoms = everything not on the worldbody (board/floor/tags) and not rooted
  // at the block or shelf unit — i.e. just the arm + base.
  function computeRobotGeoms(): number[] {
    if (!session) return [];
    const model = session.model;
    const geomBody = model.geom_bodyid as Int32Array;
    const parent = model.body_parentid as Int32Array;
    const rgba = model.geom_rgba as Float32Array;
    const group = model.geom_group as Int32Array;
    // Anything that is scenery rather than robot. The tag view shows only the
    // robot and the fiducials it has detected — the station belongs to the room,
    // and showing the sim's copy of it would make a modelled object look like a
    // measurement.
    // Scenery includes the free-body props. They are their own world bodies, not
    // part of any station, so without naming them here the tag view drew the
    // apple and banana alongside the robot — modelled objects presented as if
    // they were measurements, which is exactly what that view must not do.
    const scenery = [
      'block',
      'shelf_unit',
      'wrist_cam_mount',
      ...STATIONS.map((st) => `station_${st.navTag}`),
      ...STATIONS.map((st) => `prop_${st.navTag}`),
    ]
      .map((n) => mj.mj_name2id(model, 1, n))
      .filter((id) => id >= 0);
    const ids: number[] = [];
    for (let g = 0; g < model.ngeom; g++) {
      if (rgba[g * 4 + 3] <= 0 || group[g] >= 3) continue;
      let b = geomBody[g];
      if (b === 0) continue; // worldbody: board surface, floor, printed tags
      let root = b;
      while (parent[root] !== 0) root = parent[root];
      if (scenery.includes(root)) continue;
      ids.push(g);
    }
    return ids;
  }

  // Camera pose in world (R columns = camera x/y/z axes; OpenCV: x right, y down,
  // z forward) from the robot's yaw + the onboard-camera mount.
  function cameraWorld(): { R: number[]; t: [number, number, number] } {
    const psi = (robotYawDeg * Math.PI) / 180;
    const c = Math.cos(psi), s = Math.sin(psi);
    // rows of the row-major camera→world rotation.
    const R = [s, 0, c, -c, 0, s, 0, -1, 0];
    const bx = hasBase ? robotX : 0, by = hasBase ? robotY : 0;
    // Forward is the heading; left is 90° off it. Offsets come from the rig so
    // the mount can be measured by eye rather than guessed.
    const fwd = CAM_BASE_FWD, lat = CAM_BASE_LAT;
    return {
      R,
      t: [
        bx + fwd * c - lat * s,
        by + fwd * s + lat * c,
        armLift + CAM_BASE_UP,
      ],
    };
  }

  /** Camera-frame rotation: yaw (about down) ∘ pitch (about right) ∘ roll (about the view axis). */
  function camRotation(rollDeg: number, pitchDeg: number, yawDeg: number): number[] {
    const rad = (d: number) => (d * Math.PI) / 180;
    const cr = Math.cos(rad(rollDeg)), sr = Math.sin(rad(rollDeg));
    const cp = Math.cos(rad(pitchDeg)), sp = Math.sin(rad(pitchDeg));
    const cy = Math.cos(rad(yawDeg)), sy = Math.sin(rad(yawDeg));
    const Rroll = [cr, -sr, 0, sr, cr, 0, 0, 0, 1]; // about camera z (view axis)
    const Rpitch = [1, 0, 0, 0, cp, -sp, 0, sp, cp]; // about camera x (right)
    const Ryaw = [cy, 0, sy, 0, 1, 0, -sy, 0, cy]; // about camera y (down)
    return matMul3x3(Ryaw, matMul3x3(Rpitch, Rroll));
  }

  /**
   * The wrist-cam bracket's actual pose in the world, read back from the model
   * after the forward pass — `xpos`/`xmat` of the bracket body itself.
   *
   * This is deliberately a *read*, not a recomputation. Deriving the camera from
   * the gripper site in parallel with the bracket meant two expressions of the
   * same thing that could disagree — and they did, every time one was evaluated
   * at a different point in the frame than the other. Taking the bracket's own
   * transform makes "the camera is bolted to the bracket" true by construction:
   * whatever pose the bracket is drawn at is the pose the camera sees from.
   */
  function mountBodyFrame(): { p: [number, number, number]; R: number[] } | null {
    if (!session || mountBodyId < 0) return null;
    const xp = session.data.xpos as Float64Array;
    const xm = session.data.xmat as Float64Array;
    const i = mountBodyId * 3;
    const j = mountBodyId * 9;
    return {
      p: [xp[i], xp[i + 1], xp[i + 2]],
      R: Array.from(xm.subarray(j, j + 9)), // row-major; columns are the body's axes
    };
  }

  /**
   * Arm-camera pose in world: the bracket's frame, offset and aimed by the rig
   * values. Camera axes are OpenCV's (x right, y down, z forward) and map to the
   * bracket's as z←x, x←y, y←z; the three rig angles then rotate the camera
   * within the bracket, as one fixed composition so each is independent of the
   * others.
   */
  function armCameraWorld(): { R: number[]; t: [number, number, number] } {
    const frame = mountBodyFrame();
    // Before the bracket body exists (first frames after a load) fall back to the
    // gripper site, so the view is merely approximate rather than absent.
    const p = frame ? frame.p : solver!.sitePosition();
    const M = frame ? frame.R : (solver!.siteRotation() as number[]);
    const ax = [M[0], M[3], M[6]]; // bracket x → camera forward
    const ay = [M[1], M[4], M[7]]; // bracket y → camera right
    const az = [M[2], M[5], M[8]]; // bracket z → camera down

    const t: [number, number, number] = [
      p[0] + ax[0] * CAM_WRIST_FWD + ay[0] * CAM_WRIST_LAT + az[0] * CAM_WRIST_UP,
      p[1] + ax[1] * CAM_WRIST_FWD + ay[1] * CAM_WRIST_LAT + az[1] * CAM_WRIST_UP,
      p[2] + ax[2] * CAM_WRIST_FWD + ay[2] * CAM_WRIST_LAT + az[2] * CAM_WRIST_UP,
    ];

    // Camera basis in world before aiming: columns are right, down, forward.
    const base = [
      ay[0], az[0], ax[0],
      ay[1], az[1], ax[1],
      ay[2], az[2], ax[2],
    ];
    // Aim, then trim. Both are yaw ∘ pitch ∘ roll about the camera's own axes,
    // composed on the right so each angle is independent of the others.
    const R = matMul3x3(
      base,
      matMul3x3(
        camRotation(CAM_WRIST_ROLL_DEG, CAM_WRIST_PITCH_DEG, CAM_WRIST_YAW_DEG),
        camRotation(CAM_TRIM_ROLL_DEG, CAM_TRIM_PITCH_DEG, CAM_TRIM_YAW_DEG),
      ),
    );
    // Stand the lens ahead of the mount along the view axis it ended up with —
    // after the rotation, so it always moves the way the camera is looking.
    const view = [R[2], R[5], R[8]];
    t[0] += view[0] * CAM_AHEAD;
    t[1] += view[1] * CAM_AHEAD;
    t[2] += view[2] * CAM_AHEAD;
    return { R, t };
  }

  // Build the simulated onboard camera once its canvas mounts (it only exists
  // while there's no real base camera).
  // The robot's-eye views share the main renderer's scene, so nothing is built
  // twice. Recreated when a scene loads or a camera is unplugged.
  $effect(() => {
    sceneEpoch;
    if (povCanvas && !povView && renderer) povView = new PovView(povCanvas, renderer.sceneRef);
    if (armPovCanvas && !armPovView && renderer) armPovView = new PovView(armPovCanvas, renderer.sceneRef);
  });

  /** Copy the sim view's current camera placement, to paste back as a default. */
  async function copyCameraOrientation() {
    const c = renderer?.cameraState();
    if (!c) return;
    const f = (v: number[]) => v.map((n) => n.toFixed(3)).join(', ');
    const text = `camera.position.set(${f(c.pos)}); controls.target.set(${f(c.target)});`;
    try {
      await navigator.clipboard.writeText(text);
      copyMsg = `Copied — ${text}`;
      appendLog(`[Clipboard] Copied camera orientation: ${text}`);
    } catch {
      copyMsg = text;
    }
  }

  /** Copy the robot base's live position and rotation to the clipboard. */
  async function copyRobotPose() {
    const rx = hasBase ? robotX : 0;
    const ry = hasBase ? robotY : 0;
    const rYaw = hasBase ? robotYawDeg : 0;
    const rYawRad = (rYaw * Math.PI) / 180;
    const text = `robot.position: [${rx.toFixed(3)}, ${ry.toFixed(3)}, 0]; robot.rotation: ${rYaw.toFixed(1)}° (${rYawRad.toFixed(3)} rad); (robotX = ${rx.toFixed(3)}; robotY = ${ry.toFixed(3)}; robotYawDeg = ${rYaw.toFixed(1)};)`;
    try {
      await navigator.clipboard.writeText(text);
      copyMsg = `Copied robot pose — x: ${rx.toFixed(3)}m, y: ${ry.toFixed(3)}m, yaw: ${rYaw.toFixed(1)}°`;
      appendLog(`[Clipboard] Copied robot pose: x=${rx.toFixed(3)}m, y=${ry.toFixed(3)}m, yaw=${rYaw.toFixed(1)}°`);
    } catch {
      copyMsg = text;
    }
  }

  // Right-click context menu on the MuJoCo simulation viewport
  let simContextMenu = $state<{ x: number; y: number; visible: boolean }>({
    x: 0,
    y: 0,
    visible: false,
  });

  function handleSimContextMenu(e: MouseEvent) {
    e.preventDefault();
    const menuW = 270;
    const menuH = 180;
    const x = Math.min(e.clientX, Math.max(10, window.innerWidth - menuW - 10));
    const y = Math.min(e.clientY, Math.max(10, window.innerHeight - menuH - 10));
    simContextMenu = { x, y, visible: true };
  }

  function closeSimContextMenu() {
    if (simContextMenu.visible) {
      simContextMenu = { x: 0, y: 0, visible: false };
    }
  }

  // ── Real ArUco markers in the sim ──────────────────────────────────────────
  // Paint the actual fiducials onto the tag geoms, so a rendered frame contains
  // something the real detector can find. This is what lets sim and hardware
  // share one detection path instead of two implementations that can disagree.
  let tagTextures = new Map<number, HTMLCanvasElement>();

  function wrapCanvasText(ctx: CanvasRenderingContext2D, text: string, maxWidth: number): string[] {
    const words = text.split(/\s+/).filter(Boolean);
    if (!words.length) return [''];
    const lines: string[] = [];
    let curr = '';
    for (const w of words) {
      const testLine = curr ? `${curr} ${w}` : w;
      if (ctx.measureText(testLine).width <= maxWidth || !curr) {
        curr = testLine;
      } else {
        lines.push(curr);
        curr = w;
      }
    }
    if (curr) lines.push(curr);
    return lines;
  }

  /** A labeled card canvas on a white plate, with multi-line wrapping. */
  /**
   * Render an ArUco marker oriented for a vertical +X box face so Corner 0
   * is upright and at top-left.
   */
  function verticalMarkerCanvas(cv: Cv, dictId: number, id: number, sidePx = 192): HTMLCanvasElement {
    const raw = markerCanvas(cv, dictId, id, sidePx);
    const c = document.createElement('canvas');
    c.width = sidePx;
    c.height = sidePx;
    const ctx = c.getContext('2d')!;
    ctx.translate(sidePx / 2, sidePx / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.drawImage(raw, -sidePx / 2, -sidePx / 2, sidePx, sidePx);
    return c;
  }

  /** A labeled card canvas on a white plate, with multi-line wrapping. */
  function stationLabelCanvas(label: string, desc: string): HTMLCanvasElement {
    const w = 240, h = 480; // w corresponds to height in Z (20mm), h corresponds to width in Y (30mm)
    const c = document.createElement('canvas');
    c.width = w;
    c.height = h;
    const ctx = c.getContext('2d')!;
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, w, h);

    // Map 2D coordinates to 3D +X face: U is along -Z (vertical), V is along +Y (horizontal)
    ctx.save();
    ctx.translate(w / 2, h / 2);
    ctx.rotate(-Math.PI / 2);

    const W_eff = h; // 480 (horizontal width)
    const H_eff = w; // 240 (vertical height)
    const pad = 18;
    const maxTextW = W_eff - pad * 2;

    // Top half: LABEL
    const labelH = H_eff / 2;
    let labelFont = 44;
    ctx.font = `bold ${labelFont}px ui-sans-serif, Arial, sans-serif`;
    let labelLines = wrapCanvasText(ctx, label.toUpperCase(), maxTextW);
    while (labelFont > 20 && (labelLines.length > 2 || labelLines.length * labelFont * 1.2 > labelH - 20)) {
      labelFont -= 2;
      ctx.font = `bold ${labelFont}px ui-sans-serif, Arial, sans-serif`;
      labelLines = wrapCanvasText(ctx, label.toUpperCase(), maxTextW);
    }

    ctx.fillStyle = '#000000';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const labelLineH = labelFont * 1.2;
    const totalLabelH = labelLines.length * labelLineH;
    let yL = -H_eff / 2 + (labelH - totalLabelH) / 2 + labelLineH / 2;
    for (const line of labelLines) {
      ctx.fillText(line, 0, yL);
      yL += labelLineH;
    }

    // Separator line
    ctx.strokeStyle = '#bbbbbb';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(-W_eff * 0.44, 0);
    ctx.lineTo(W_eff * 0.44, 0);
    ctx.stroke();

    // Bottom half: DESCRIPTION
    if (desc) {
      const descH = H_eff / 2;
      let descFont = 24;
      ctx.font = `${descFont}px ui-sans-serif, Arial, sans-serif`;
      let descLines = wrapCanvasText(ctx, desc, maxTextW);
      while (descFont > 14 && (descLines.length > 3 || descLines.length * descFont * 1.3 > descH - 20)) {
        descFont -= 2;
        ctx.font = `${descFont}px ui-sans-serif, Arial, sans-serif`;
        descLines = wrapCanvasText(ctx, desc, maxTextW);
      }
      ctx.fillStyle = '#333333';
      const descLineH = descFont * 1.3;
      const totalDescH = descLines.length * descLineH;
      let yD = (descH - totalDescH) / 2 + descLineH / 2;
      for (const line of descLines) {
        ctx.fillText(line, 0, yD);
        yD += descLineH;
      }
    }

    ctx.restore();
    return c;
  }

  /**
   * Render the standardized boundary stop tag.
   * White card with red rounded border, red ArUco tag border, centered Tag 240, and red text:
   * "Okay, gotta stop here, explore the other side!"
   * UV-mapped onto MuJoCo box geom +X face (46mm wide x 56mm high).
   */
  function boundaryTagCanvas(cv: Cv, id: number): HTMLCanvasElement {
    const w = 336, h = 276; // w corresponds to height in Z (56mm), h corresponds to width in Y (46mm)
    const c = document.createElement('canvas');
    c.width = w;
    c.height = h;
    const ctx = c.getContext('2d')!;
    ctx.clearRect(0, 0, w, h);

    // Map 2D coordinates to 3D +X face: U is along -Z (vertical), V is along +Y (horizontal)
    ctx.save();
    ctx.translate(w / 2, h / 2);
    ctx.rotate(-Math.PI / 2);

    const W_eff = h; // 276 (horizontal width)
    const H_eff = w; // 336 (vertical height)

    // White cutout paper card with rounded corners and bold red border
    const cardR = 24;
    ctx.beginPath();
    ctx.roundRect(-W_eff / 2 + 3, -H_eff / 2 + 3, W_eff - 6, H_eff - 6, cardR);
    ctx.fillStyle = '#ffffff';
    ctx.fill();
    ctx.strokeStyle = '#dc2626';
    ctx.lineWidth = 6;
    ctx.stroke();

    // ArUco marker (Tag 240) centered horizontally near the top with clean quiet zone
    const markerPx = 144;
    const mkCanvas = markerCanvas(cv, cv.DICT_6X6_250, id, markerPx);
    const markerTop = -H_eff / 2 + 20;
    ctx.drawImage(mkCanvas, -markerPx / 2, markerTop, markerPx, markerPx);

    // Bold red border around the ArUco marker
    ctx.strokeStyle = '#dc2626';
    ctx.lineWidth = 3.5;
    ctx.strokeRect(-markerPx / 2 - 2, markerTop - 2, markerPx + 4, markerPx + 4);

    // Crisp black text directly underneath the marker
    ctx.fillStyle = '#0f172a';
    ctx.font = 'bold 16px ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('Okay, gotta stop here,', 0, markerTop + markerPx + 32);
    ctx.fillText('explore the other side!', 0, markerTop + markerPx + 54);

    ctx.restore();
    return c;
  }

  function buildTagTextures() {
    if (!cv || !session || !mj) return;
    const m = new Map<number, HTMLCanvasElement>();
    const add = (geomName: string, id: number) => {
      const gid = mj!.mj_name2id(session!.model, 5 /* mjOBJ_GEOM */, geomName);
      if (gid < 0) return;
      m.set(gid, markerCanvas(cv!, cv!.DICT_6X6_250, id, 192));
    };
    const addVertical = (geomName: string, id: number) => {
      const gid = mj!.mj_name2id(session!.model, 5 /* mjOBJ_GEOM */, geomName);
      if (gid < 0) return;
      m.set(gid, verticalMarkerCanvas(cv!, cv!.DICT_6X6_250, id, 192));
    };

    // Boundary posts (Start & End rack limit markers with "STOP" badge matching tag generator)
    const setBoundary = (geomName: string, id: number) => {
      const gid = mj!.mj_name2id(session!.model, 5 /* mjOBJ_GEOM */, geomName);
      if (gid < 0) return;
      m.set(gid, boundaryTagCanvas(cv!, id));
    };
    setBoundary('boundary_navtag_start', BOUNDARY_TAG_ID);
    setBoundary('boundary_navtag_end', BOUNDARY_TAG_ID);

    // Every station renders a dual-tag (unique st.navTag left, 200 right anchor) navigation card
    // with its label (top) and description (bottom) in between.
    for (const st of STATIONS) {
      if (st.isBoundary) continue;
      addVertical(`station_navtag_${st.navTag}`, st.navTag);
      addVertical(`station_navtag_right_${st.navTag}`, 200);
      const lid = mj!.mj_name2id(session!.model, 5, `station_label_${st.navTag}`);
      if (lid >= 0) m.set(lid, stationLabelCanvas(st.label || st.prop, st.description || ''));
      if (st.propTag) add(`ptag_${st.propTag}`, st.propTag);
    }
    add('block_tag', STATION_OBJECT_TAG);
    tagTextures = m;
    // One scene, one set of textures — the POV views render from it too.
    renderer?.setGeomTextures(m);
  }

  /**
   * Intrinsics for a rendered camera. A perspective render is a perfect pinhole,
   * so these are exact rather than calibrated: focal length falls straight out of
   * the vertical FOV, the principal point is the image centre, no distortion.
   */
  function povIntrinsics(fovDeg: number, w: number, h: number): Intrinsics {
    const f = h / 2 / Math.tan(((fovDeg * Math.PI) / 180) / 2);
    return {
      cameraMatrix: [f, 0, w / 2, 0, f, h / 2, 0, 0, 1],
      distCoeffs: [0, 0, 0, 0, 0],
      imgW: w,
      imgH: h,
      rms: 0,
    };
  }

  // The wrist-cam bracket is a mocap body so it can ride the gripper frame
  // without being welded into the arm model.
  let mountMocapId = -1;
  let mountBodyId = -1; // the bracket body itself, for reading its world pose back

  /** Rotation matrix (row-major, columns = axes) → quaternion (w,x,y,z). */
  function matToQuat(m: number[]): [number, number, number, number] {
    const tr = m[0] + m[4] + m[8];
    if (tr > 0) {
      const sq = Math.sqrt(tr + 1) * 2;
      return [sq / 4, (m[7] - m[5]) / sq, (m[2] - m[6]) / sq, (m[3] - m[1]) / sq];
    }
    if (m[0] > m[4] && m[0] > m[8]) {
      const sq = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2;
      return [(m[7] - m[5]) / sq, sq / 4, (m[1] + m[3]) / sq, (m[2] + m[6]) / sq];
    }
    if (m[4] > m[8]) {
      const sq = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2;
      return [(m[2] - m[6]) / sq, (m[1] + m[3]) / sq, sq / 4, (m[5] + m[7]) / sq];
    }
    const sq = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2;
    return [(m[3] - m[1]) / sq, (m[2] + m[6]) / sq, (m[5] + m[7]) / sq, sq / 4];
  }

  /** Ride the wrist-cam bracket along with the gripper. */
  /**
   * The wrist-cam bracket's frame in world: position and a row-major rotation
   * whose columns are the bracket's own x/y/z axes.
   *
   * Both the drawn bracket and the camera hang off this. The camera used to be
   * offset from the gripper site directly, which meant its offsets were measured
   * in a frame the physical camera isn't attached to — change the bracket's
   * rotation and the camera stayed put, when in reality it rides along.
   */
  function wristMountFrame(): { t: [number, number, number]; R: number[] } {
    const p = solver!.sitePosition();
    const st = solver!.siteRotation(); // columns = site x/y/z in world
    const cx = [st[0], st[3], st[6]], cy = [st[1], st[4], st[7]], cz = [st[2], st[5], st[8]];
    const t: [number, number, number] = [
      p[0] + cx[0] * MOUNT_OFF[0] + cy[0] * MOUNT_OFF[1] + cz[0] * MOUNT_OFF[2],
      p[1] + cx[1] * MOUNT_OFF[0] + cy[1] * MOUNT_OFF[1] + cz[1] * MOUNT_OFF[2],
      p[2] + cx[2] * MOUNT_OFF[0] + cy[2] * MOUNT_OFF[1] + cz[2] * MOUNT_OFF[2],
    ];
    // Same roll-pitch-yaw the bracket body is given, as a matrix: R = site · Rx · Ry · Rz.
    const rad = (d: number) => (d * Math.PI) / 180;
    const [rr, pp, yy] = MOUNT_RPY_DEG.map(rad);
    const cr = Math.cos(rr), sr = Math.sin(rr);
    const cp = Math.cos(pp), sp = Math.sin(pp);
    const cyw = Math.cos(yy), syw = Math.sin(yy);
    const Rx = [1, 0, 0, 0, cr, -sr, 0, sr, cr];
    const Ry = [cp, 0, sp, 0, 1, 0, -sp, 0, cp];
    const Rz = [cyw, -syw, 0, syw, cyw, 0, 0, 0, 1];
    const R = matMul3x3(matMul3x3(matMul3x3(st as unknown as number[], Rx), Ry), Rz);
    return { t, R };
  }

  function updateWristMount() {
    if (!session || !solver || mountMocapId < 0) return;
    const { t, R } = wristMountFrame();
    const q = matToQuat(R);
    const mp = session.data.mocap_pos as Float64Array;
    const mq = session.data.mocap_quat as Float64Array;
    mp[mountMocapId * 3] = t[0];
    mp[mountMocapId * 3 + 1] = t[1];
    mp[mountMocapId * 3 + 2] = t[2];
    mq[mountMocapId * 4] = q[0];
    mq[mountMocapId * 4 + 1] = q[1];
    mq[mountMocapId * 4 + 2] = q[2];
    mq[mountMocapId * 4 + 3] = q[3];
  }

  let lastPov = 0;
  function renderPov() {
    const now = performance.now();
    if (now - lastPov < 70) return; // ~14 Hz — enough to feed a 10 Hz detector
    lastPov = now;
    // Camera resolution, not the size of the grid cell they're shown in: these
    // frames are the detector's input, and at cell size an 80 mm tag lands on
    // ~30 px — under 4 px per marker cell, which never decodes.
    const w = settings.camResW;
    const h = Math.round((w * 3) / 4);
    if (povView) {
      const cam = cameraWorld();
      povView.setPose(cam.R, cam.t);
      povView.render(w, h);
    }
    if (armPovView && solver) {
      const cam = armCameraWorld();
      armPovView.setPose(cam.R, cam.t);
      armPovView.render(w, h);
    }
  }

  let lastRenderedTags: CamTag[] | null = null;
  // Drive the arm toward a fixed joint pose (no IK), rate-limited by the global
  // speed cap. Used by Rest (park at home) and the board View pose.
  /**
   * The forward pass every render goes through. The only correction left is the
   * gripper, which the model draws inverted against the real arm in the kinematic
   * path; the joint value is restored afterwards so IK and the streamed command
   * never see it.
   *
   * There used to be a wrist-roll correction here too, applied by writing into
   * qpos before forwarding. It is gone: a render-time edit of the model's state
   * is invisible to every other reader of those frames, so any path that skipped
   * it — or ran at a different point in the frame — saw the wrist rotated 90°,
   * which is what made the camera appear to flip at random. The camera's mounting
   * angle now lives in CAM_WRIST_ROLL_DEG, where it only describes the camera.
   */
  function forwardForDisplay(flipGripper: boolean) {
    if (!session) return;
    const q = session.data.qpos as Float64Array;
    const grip = q[5];
    if (flipGripper) q[5] = gripperRange[0] + gripperRange[1] - gripperCmd;
    readShelfOpens();
    session.forward(); // wrist site now current
    // The bracket is placed from the wrist frame this pass just computed, then
    // forwarded again so its own body pose is current — the camera reads that
    // pose back. Placing it after the render, or reading it before this second
    // forward, leaves it a frame stale, which looks like the mount rotating on
    // its own as the arm moves.
    updateWristMount();
    session.forward(); // bracket body now current
    q[5] = grip;
  }

  function holdPose(desired: number[]) {
    if (!session || !mj || !renderer || !solver) return;
    const d = session.data;
    const q = d.qpos as Float64Array;
    const pose = limitPose(desired, ARM_DOFS.map((i) => q[i])); // seed = live pose
    jointAngles = pose.slice();
    if (physics) {
      const ctrl = d.ctrl as Float64Array;
      for (let i = 0; i < 5; i++) ctrl[i] = pose[i];
      ctrl[5] = gripperRange[0] + gripperRange[1] - gripperCmd;
      const steps = physicsStepsDue();
      for (let k = 0; k < steps; k++) mj.mj_step(session.model, d);
      // Keep hold of whatever is in the jaws. This path steps physics itself
      // rather than going through stepPhysics, so without this the grasp stopped
      // being maintained the moment the arm parked — and the item fell out on the
      // way to rest, every time.
      updateGrasp();
      forwardForDisplay(false); // physics gripper closes for real — nothing to correct
    } else {
      for (let i = 0; i < 5; i++) q[i] = pose[i];
      forwardForDisplay(true);
    }
    renderer.setTarget(solver.sitePosition(), undefined); // no active EE target
    renderer.update();
  }

  const restArm = () => holdPose(restTargetPose);
  /**
   * Pan/tilt offset (rad) that the search last found something at, applied to
   * VIEW_POSE by every "raise to view".
   *
   * Without it the search is wasted: it would pan 10° left, find the tag, and then
   * the pick's own raise-to-view would snap back to the unshifted pose and report
   * the tag as not visible. "Raise to view" has to mean "look where we last saw
   * something", not "look where we guessed".
   */
  let viewOffset = $state<[number, number, number]>([0, 0, 0]);
  function viewPoseNow(): number[] {
    const p = VIEW_POSE.slice();
    p[0] += viewOffset[0]; // shoulder pan
    p[1] += viewOffset[1]; // shoulder lift
    p[3] += viewOffset[2]; // wrist flex — where the camera actually points
    return p;
  }
  const viewArm = () => holdPose(viewPoseNow());

  /**
   * Hand control back to the manual sliders *without moving the arm*: adopt the
   * current end-effector position as the target and pin the wrist roll where it
   * is. Position-only IK ("None") is the one grasp mode that holds the pose
   * exactly — the top/45/front modes each force an approach direction and would
   * rotate the wrist on handover. Switch the mode back when you want to move on.
   */
  function handBackToManual() {
    if (!solver) return;
    // Explore parks the arm and leaves it parked; without clearing that here, the
    // arm would snap back home the moment a routine handed control back.
    armMode = 'active';
    target = [...solver.sitePosition()] as [number, number, number];
    graspRollDeg = Math.round((jointAngles[4] * 180) / Math.PI);
    graspMode = 'none';
    renderer?.setTarget(target);
  }

  // Snap the EE target to where the end-effector currently is (forward kinematics):
  // the arm then holds its present pose instead of chasing a stale target.
  function targetToCurrentEE() {
    if (!solver || !renderer) return;
    target = [...solver.sitePosition()] as [number, number, number];
    renderer.setTarget(target);
  }

  // Copy the current arm joint angles (rad) to the clipboard as a JS array literal,
  // handy for pasting into a custom pose (e.g. a bespoke rest position).
  let copyMsg = $state<string | null>(null);
  async function copyArmPose() {
    const text = '[' + jointAngles.map((a) => (a ?? 0).toFixed(4)).join(', ') + ']';
    try {
      await navigator.clipboard.writeText(text);
      copyMsg = `Copied ${text}`;
    } catch {
      copyMsg = text; // clipboard blocked — show it so it can be copied manually
    }
  }

  // Make the current arm pose the rest/home pose, so "Rest" parks here from now on.
  function setRestToCurrent() {
    restTargetPose = jointAngles.slice(0, 5);
    copyMsg = 'Rest pose set to the current arm position.';
  }

  function loop() {
    if (!solver || !renderer || !session) return;
    const q = session.data.qpos as Float64Array;

    // Rest / view / hold mode: park at HOME, raise to look at the board, or lock
    // joint angles while the base moves, skipping the manual IK path.
    const isHolding = armMode === 'hold';
    const isResting = armMode === 'rest' || (navigating && armMode === 'active');
    if (isResting || isHolding || armView || jointHoldPose) {
      // Entering a hold: re-seed the speed limiter from where the arm actually is.
      // Only the kinematic path feeds limitPose every frame, so in sim the slew
      // state is left wherever the *last* hold routine finished. Slewing from that
      // sends the arm back to the old pose first and then crawls to the new one —
      // which is why reloading a saved position appeared to go home instead.
      if (!wasHolding) resetPoseLimiter();
      if (jointHoldPose) holdPose(jointHoldPose);
      else if (armView) viewArm();
      else if (isHolding) holdPose(lockedArmPose);
      else restArm();

      maybeDetect();
      maybeDetectArm();
      if (remoteBaseLive) paintRemote(REMOTE_BASE_CAM);
      if (remoteArmLive) paintRemote(REMOTE_ARM_CAM);
      renderPov();
      applyBaseMotion();
      maybeStreamToRobot();
      maybeStreamBase();
      wasHolding = true;
      raf = requestAnimationFrame(loop);
      return;
    }
    // Coming back from rest / view / navigation: adopt the pose we're actually in
    // as the manual target, so handing back to manual doesn't yank the arm to a
    // stale one.
    if (wasHolding) {
      wasHolding = false;
      targetToCurrentEE();
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
    } else if (stage === 'open') {
      // Hold the current point and open the fingers wide, squaring the wrist to
      // straight top-down at zero rotation — a known, repeatable starting pose
      // before the arm travels to the tag (grip rotation is applied later).
      Rd = [0, 1, 0, 0, 0, -1, -1, 0, 0]; // top-down, 0° roll
      grip = gripperRange[0]; // widest opening (0% on the grip slider)
      gripperCmd = grip;
      pickPhase = 'open';
      manual = false;
    } else if (stage !== '' && pickBlock) {
      // A selected stage overrides the manual target with that phase's target,
      // top-down (fingers at gripYaw).
      const s = phaseTarget(
        pickBlock,
        stage,
        { ...DEFAULT_PICK, graspZ: graspDepth, graspXOffset: graspX, graspYOffset: graspY },
        descendZ ?? undefined, // stepped descent overrides the final grasp height
      );
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

    // While the approach is running, the gripper's orientation is pinned to
    // whatever it had when the approach started. Position-only IK leaves the wrist
    // free, so reaching sideways also tilts the camera — and a tilt moves the tag
    // in the image the opposite way to the translation that caused it. Past a
    // certain reach the tilt wins, the correction inverts, and the loop walks the
    // tag out of frame. Holding the orientation removes that coupling: the camera
    // translates without turning, so image error responds to the target the way
    // the geometry says it should.
    if (holdRd) Rd = holdRd;
    // A held orientation is an instruction, not a preference. The default weight
    // is deliberately low so the arm favours reaching the point over matching the
    // approach angle — but that also meant a commanded wrist roll barely moved the
    // wrist, so the squaring computed during the approach never actually happened.
    const oriWeight = holdRd ? holdOriWeight : 0.25;
    // Position-only IK (approach free) for the shelf stages, and for the manual
    // "None" grasp mode. Wrist roll is fixed (0 for the shelf, the roll slider else).
    const free = !holdRd && (shelfActive || (manual && graspMode === 'none'));
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
      stepPhysics(solveTarget, Rd, grip, {
        free,
        rollRad: holdRd ? undefined : rollRad,
        dofLimits,
        oriWeight,
      });
      forwardForDisplay(false);
      renderer.setTarget(solveTarget, free ? undefined : Rd);
      renderer.update();
    } else {
      // Kinematic path (real mode): solve IK, write the solution into qpos for
      // display, and stream the solved joints to the servos. Honors the same
      // free / orientation / branch choices as the physics path, so the grasp
      // modes and shelf stages behave identically to sim-only mode.
      const before = ARM_DOFS.map((i) => q[i]); // pose before solving = where the arm is
      let res;
      if (free) {
        q[4] = rollRad; // fix wrist roll; reach position with the other four joints
        res = solver.solve(solveTarget, { dofIndices: [0, 1, 2, 3], maxIters: 12, dofLimits });
      } else {
        res = solver.solvePose(solveTarget, Rd, { dofIndices: ARM_DOFS, maxIters: 10, oriWeight, dofLimits });
        if (!holdRd) q[4] = rollRad; // roll is commanded, unless a visual step owns it
      }
      ikOk = res.ok;
      ikErrMm = res.error * 1000;
      // Rate-limit the IK solution to the global max arm speed, then write the
      // limited pose back into qpos so display and streaming both use it.
      const pose = limitPose(ARM_DOFS.map((i) => q[i]), before);
      for (let i = 0; i < 5; i++) q[i] = pose[i];
      jointAngles = pose.slice(); // the commanded joints (streamed as-is)

      forwardForDisplay(true);
      renderer.setTarget(solveTarget, free ? undefined : Rd);
      renderer.update();
    }

    maybeDetect();
    maybeDetectArm();
    if (remoteBaseLive) paintRemote(REMOTE_BASE_CAM);
    if (remoteArmLive) paintRemote(REMOTE_ARM_CAM);
    renderPov();
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
    if (camStream || robot.mode === 'remote') return; // already acquired or remote WebRTC video
    intrinsics = loadIntrinsics(); // pick up the latest calibration
    try {
      if (!cv) {
        cv = await loadCv();
        detector = createDetector(cv);
        tagCentresMm = boardTagCentres(SQUARE_MM, 16, 2, 10, 8);
      }
      camStream = await navigator.mediaDevices.getUserMedia({
        video: {
          ...(camDeviceId ? { deviceId: { exact: camDeviceId } } : { facingMode: 'environment' }),
          width: { ideal: settings.camResW },
          height: { ideal: Math.round((settings.camResW * 3) / 4) },
        },
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
      await listCameras(); // labels are available now that permission is granted
    } catch (e) {
      detectMsg = 'Camera error: ' + (e instanceof Error ? e.message : String(e));
    }
  }

  // Restart the onboard (base) camera — e.g. after picking a different device.
  async function reconnectBaseCamera() {
    detectMsg = null;
    if (settings.exclusiveCam) disconnectArmCamera(); // free the bus for this one
    baseCamOff = false;
    stopCamera();
    await startCamera();
  }

  /** Release the base camera (and keep it released) — frees USB bandwidth. */
  function disconnectBaseCamera() {
    baseCamOff = true;
    stopCamera();
    shelfTagCam = null; // its detections are stale now
    detectMsg = null;
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

  // ── Arm camera + board-pick routine ──────────────────────────────────────────
  const delay = (ms: number) => new Promise<void>((r) => window.setTimeout(r, ms));

  async function listCameras() {
    try {
      const devs = await navigator.mediaDevices.enumerateDevices();
      videoDevices = devs.filter((d) => d.kind === 'videoinput');
      if (!armCamDeviceId && videoDevices.length) {
        armCamDeviceId = videoDevices[videoDevices.length - 1].deviceId; // guess: last = arm cam
      }
    } catch (e) {
      armPickMsg = 'Could not list cameras: ' + (e instanceof Error ? e.message : String(e));
    }
  }

  async function connectArmCamera() {
    if (armCamStream || robot.mode === 'remote') return;
    armPickMsg = null;
    try {
      if (!cv) {
        cv = await loadCv();
        detector = createDetector(cv);
        tagCentresMm = boardTagCentres(SQUARE_MM, 16, 2, 10, 8);
      }
      if (settings.exclusiveCam) {
        baseCamOff = true; // free the bus before opening the second camera
        stopCamera();
      }
      armCamStream = await navigator.mediaDevices.getUserMedia({
        video: {
          ...(armCamDeviceId ? { deviceId: { exact: armCamDeviceId } } : {}),
          width: { ideal: settings.camResW },
          height: { ideal: Math.round((settings.camResW * 3) / 4) },
        },
      });
      armVideo.srcObject = armCamStream;
      await armVideo.play();
      if (armPreview) {
        armPreview.srcObject = armCamStream;
        armPreview.play().catch(() => {});
      }
      const w = armVideo.videoWidth || 640, h = armVideo.videoHeight || 480;
      armCamW = w;
      armCamH = h;
      const grab = document.createElement('canvas');
      grab.width = w;
      grab.height = h;
      armGrabCtx = grab.getContext('2d', { willReadFrequently: true });
      armSrcMat = new cv.Mat(h, w, cv.CV_8UC4);
      armGrayMat = new cv.Mat();
      await listCameras(); // labels are available now that permission is granted
    } catch (e) {
      armPickMsg = 'Arm camera error: ' + (e instanceof Error ? e.message : String(e));
    }
  }

  function disconnectArmCamera() {
    armCamStream?.getTracks().forEach((t) => t.stop());
    armCamStream = null;
    try {
      armSrcMat?.delete();
      armGrayMat?.delete();
    } catch {
      /* ignore */
    }
    armSrcMat = null;
    armGrayMat = null;
    armGrabCtx = null;
    armDetectedTags = [];
  }

  // Restart the arm/wrist camera with the currently-selected device (switches
  // feeds; re-triggers the permission prompt when a new device needs it).
  async function reconnectArmCamera() {
    disconnectArmCamera();
    await connectArmCamera();
  }

  /**
   * Draw real-time OpenCV-style marker detection outlines and ID badges
   * onto the camera overlay canvas.
   */
  /**
   * Draw real-time OpenCV-style marker detection outlines, ID badges,
   * grouped card enclosures, connecting baselines, and optical center indicators
   * onto the camera overlay canvas.
   */
  function drawMarkerOverlays(
    ctx: CanvasRenderingContext2D,
    corners: Map<number, Corners>,
    width: number,
    height: number,
    allDetections?: MarkerDetection[],
  ) {
    if (ctx.canvas.width !== width || ctx.canvas.height !== height) {
      ctx.canvas.width = width;
      ctx.canvas.height = height;
    }
    ctx.clearRect(0, 0, width, height);

    const detections: { id: number; corners: Corners }[] =
      allDetections && allDetections.length > 0
        ? allDetections
        : Array.from(corners.entries()).map(([id, c]) => ({ id, corners: c }));

    const camCenterX = width / 2;
    const camCenterY = height / 2;

    // Find all valid station pairs: [Station 20x (Left)] + [Anchor 200 (Right)]
    const labeledPairs = findLabeledTagPairs(detections);
    const isAnyCentered = labeledPairs.some(
      (p) => Math.abs((p.centerPx ? p.centerPx[0] : 0) - camCenterX) <= 6,
    );

    // ── 1. Camera Optical Center Boresight & Alignment Crosshair ───────
    ctx.save();
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = isAnyCentered ? 'rgba(34, 197, 94, 0.75)' : 'rgba(255, 255, 255, 0.25)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(camCenterX, 0);
    ctx.lineTo(camCenterX, height);
    ctx.stroke();

    // Central crosshair ticks
    ctx.setLineDash([]);
    ctx.strokeStyle = isAnyCentered ? '#22c55e' : 'rgba(255, 255, 255, 0.4)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(camCenterX - 12, camCenterY);
    ctx.lineTo(camCenterX + 12, camCenterY);
    ctx.moveTo(camCenterX, camCenterY - 12);
    ctx.lineTo(camCenterX, camCenterY + 12);
    ctx.stroke();
    ctx.restore();

    // ── 2. Draw Grouping Bracket & Center Indicator for Paired Stations ───
    for (const pair of labeledPairs) {
      const cLeft = pair.left;
      const cRight = pair.right;
      const leftCx = (cLeft[0][0] + cLeft[1][0] + cLeft[2][0] + cLeft[3][0]) / 4;
      const leftCy = (cLeft[0][1] + cLeft[1][1] + cLeft[2][1] + cLeft[3][1]) / 4;
      const rightCx = (cRight[0][0] + cRight[1][0] + cRight[2][0] + cRight[3][0]) / 4;
      const rightCy = (cRight[0][1] + cRight[1][1] + cRight[2][1] + cRight[3][1]) / 4;

      const midX = (leftCx + rightCx) / 2;
      const midY = (leftCy + rightCy) / 2;
      const isCentered = Math.abs(midX - camCenterX) <= 6;

      // Group bounding box
      const allPts = [...cLeft, ...cRight];
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      for (const p of allPts) {
        if (p[0] < minX) minX = p[0];
        if (p[0] > maxX) maxX = p[0];
        if (p[1] < minY) minY = p[1];
        if (p[1] > maxY) maxY = p[1];
      }

      const padX = 8;
      const padY = 6;
      const boxX = minX - padX;
      const boxY = minY - padY;
      const boxW = (maxX - minX) + padX * 2;
      const boxH = (maxY - minY) + padY * 2;

      // Outer Group Framing Box
      ctx.save();
      ctx.fillStyle = isCentered ? 'rgba(34, 197, 94, 0.12)' : 'rgba(56, 189, 248, 0.08)';
      ctx.fillRect(boxX, boxY, boxW, boxH);

      ctx.strokeStyle = isCentered ? '#22c55e' : '#38bdf8';
      ctx.lineWidth = isCentered ? 2.5 : 1.8;
      ctx.setLineDash(isCentered ? [] : [6, 4]);
      ctx.strokeRect(boxX, boxY, boxW, boxH);
      ctx.restore();

      // Connecting Baseline between Left Tag and Right Anchor
      ctx.save();
      ctx.strokeStyle = isCentered ? '#22c55e' : 'rgba(56, 189, 248, 0.85)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(leftCx, leftCy);
      ctx.lineTo(rightCx, rightCy);
      ctx.stroke();

      // Draw Center Indicator & Reticle at midX, midY
      const lineH = Math.max(24, boxH * 0.8);
      ctx.strokeStyle = isCentered ? '#22c55e' : '#fbbf24';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(midX, midY - lineH / 2);
      ctx.lineTo(midX, midY + lineH / 2);
      ctx.stroke();

      // Outer reticle circle
      ctx.fillStyle = isCentered ? 'rgba(34, 197, 94, 0.3)' : 'rgba(251, 191, 36, 0.25)';
      ctx.beginPath();
      ctx.arc(midX, midY, 8, 0, 2 * Math.PI);
      ctx.fill();
      ctx.strokeStyle = isCentered ? '#22c55e' : '#fbbf24';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Inner center dot
      ctx.fillStyle = isCentered ? '#22c55e' : '#fbbf24';
      ctx.beginPath();
      ctx.arc(midX, midY, 3, 0, 2 * Math.PI);
      ctx.fill();

      // Station Group Header Badge
      const stationInfo = knownTags.get(pair.stationId) || STATIONS.find((s) => s.navTag === pair.stationId);
      const labelName = stationInfo?.label ? `${stationInfo.label} ` : '';
      const groupText = isCentered
        ? `[CENTERED] ${labelName}#${pair.stationId}`
        : `${labelName}#${pair.stationId} [${pair.stationId} + 200]`;

      ctx.font = 'bold 11px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
      const tw = ctx.measureText(groupText).width;
      const gbw = tw + 10;
      const gbh = 18;
      const gbx = Math.max(2, Math.min(width - gbw - 2, midX - gbw / 2));
      const gby = Math.max(gbh + 2, boxY - 4);

      ctx.fillStyle = isCentered ? 'rgba(22, 101, 52, 0.95)' : 'rgba(15, 23, 42, 0.9)';
      ctx.beginPath();
      ctx.roundRect(gbx, gby - gbh, gbw, gbh, 4);
      ctx.fill();
      ctx.strokeStyle = isCentered ? '#22c55e' : '#38bdf8';
      ctx.lineWidth = 1.2;
      ctx.stroke();

      ctx.fillStyle = '#ffffff';
      ctx.fillText(groupText, gbx + 5, gby - 4);
      ctx.restore();
    }

    // ── 3. Draw Individual Tag Outlines & Corner Badges ─────────────────
    for (const d of detections) {
      const id = d.id;
      const c = d.corners;
      const isBoundary = id === BOUNDARY_TAG_ID;
      const isAnchor = id === RIGHT_NAV_ID;
      const strokeCol = isBoundary ? '#ef4444' : isAnchor ? '#38bdf8' : '#22c55e';
      const fillCol = isBoundary
        ? 'rgba(239, 68, 68, 0.15)'
        : isAnchor
          ? 'rgba(56, 189, 248, 0.15)'
          : 'rgba(34, 197, 94, 0.15)';

      // 1. Draw 4-corner polygon outline
      ctx.strokeStyle = strokeCol;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(c[0][0], c[0][1]);
      ctx.lineTo(c[1][0], c[1][1]);
      ctx.lineTo(c[2][0], c[2][1]);
      ctx.lineTo(c[3][0], c[3][1]);
      ctx.closePath();
      ctx.stroke();

      ctx.fillStyle = fillCol;
      ctx.fill();

      // 2. Orientation vertex indicator (Corner 0 in red, others in stroke color)
      ctx.fillStyle = '#ef4444';
      ctx.beginPath();
      ctx.arc(c[0][0], c[0][1], 3.5, 0, 2 * Math.PI);
      ctx.fill();

      for (let i = 1; i < 4; i++) {
        ctx.fillStyle = strokeCol;
        ctx.beginPath();
        ctx.arc(c[i][0], c[i][1], 2.5, 0, 2 * Math.PI);
        ctx.fill();
      }

      // 3. ID Label Badge Pill above corner 0
      const tagText = isBoundary ? `STOP #${id}` : isAnchor ? `ANCHOR #${id} (RIGHT)` : `STATION #${id} (LEFT)`;
      ctx.font = 'bold 10px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
      const tw = ctx.measureText(tagText).width;
      const pad = 4;
      const bw = tw + pad * 2;
      const bh = 15;
      const bx = Math.max(2, Math.min(width - bw - 2, c[0][0] - 2));
      const by = Math.max(bh + 2, c[0][1] - 4);

      ctx.fillStyle = 'rgba(15, 23, 42, 0.88)';
      ctx.beginPath();
      ctx.roundRect(bx, by - bh, bw, bh, 3);
      ctx.fill();
      ctx.strokeStyle = strokeCol;
      ctx.lineWidth = 1;
      ctx.stroke();

      ctx.fillStyle = '#ffffff';
      ctx.fillText(tagText, bx + pad, by - 3);
    }
  }

  // Continuous arm-camera detection (runs while the arm is raised to view the
  // board): tracks which object tags are in view — offered as one-click pick
  // targets — and, in tag-view mode, feeds the 3-D layout the board + object tags
  // the arm camera sees (placed via the gripper-site camera pose).
  // Runs every frame the arm camera is connected (throttled to ~10 Hz), whatever
  // the arm is doing — so the list of tags it can see is always live.
  function maybeDetectArm() {
    if (!cv || !detector) return;
    const remote = remoteCams.canvas(REMOTE_ARM_CAM);
    const live = !!armCamStream || !!remote;
    const source: CanvasImageSource | null =
      remote ?? (armCamStream ? armVideo : (armPovCanvas ?? null));
    if (!source) return;
    if (!ensureArmGrab(live)) return;
    const intr = intrinsics ?? povIntrinsics(armPovView?.fovDeg ?? 60, armCamW, armCamH);
    if (!armGrabCtx || !armSrcMat || !armGrayMat) return;
    const now = performance.now();
    if (now - lastArmDetect < 100) return; // ~10 Hz
    lastArmDetect = now;

    const { width, height } = armGrabCtx.canvas;
    armGrabCtx.drawImage(source, 0, 0, width, height);
    armSrcMat.data.set(armGrabCtx.getImageData(0, 0, width, height).data);
    cv.cvtColor(armSrcMat, armGrayMat, cv.COLOR_RGBA2GRAY);
    const allDetections = detectAllMarkers(cv, detector, armGrayMat);
    latestArmDetections = allDetections;
    const corners = new Map<number, Corners>();
    for (const d of allDetections) corners.set(d.id, d.corners);
    armObsCorners = corners;
    pickCalState.updateLiveCamera(armGrabCtx.canvas, corners);

    // Draw real-time detection overlay on arm camera
    if (armCamOverlayCanvas) {
      const oCtx = armCamOverlayCanvas.getContext('2d');
      if (oCtx) drawMarkerOverlays(oCtx, corners, width, height, allDetections);
    }

    // Object tags (bordered ids, not the board border tags) → drive/pick buttons.
    const objs = [...corners.keys()]
      .filter((id) => (BORDERED_IDS.has(id) || (id >= 1 && id < NAV_TAG_LO)) && !tagCentresMm.has(id))
      .sort((a, b) => a - b);
    if (objs.join(',') !== armDetectedTags.join(',')) armDetectedTags = objs;

    // Tag view: full pose of every detected marker (board border tags included, so
    // the board itself appears), in the arm-camera frame. The arm camera owns the
    // tag view while it's raised, or whenever the base camera is off — otherwise
    // the two would overwrite each other's poses every frame.
    if (armView || !camStream) {
      const poses: TagPose[] = [];
      for (const [id, c] of corners) {
        const sizeMm = markerSizeFor(id);
        const mp = solvePnpMarkerPose(cv, c, sizeMm, armCamW, armCamH, intr);
        if (mp) poses.push({ id, R: mp.R, t: mp.t, sizeMm });
      }
      tagPoses = poses;
      tagsFromArm = true;
    }

    // Keep pickCalState.estimatedTag updated live for Pick Calibration visualizer
    const calTargetId = pickCalState.estimatedTag?.id ?? pickCal.targetTagId ?? 201;
    const targetCorner = corners.get(calTargetId) ?? (corners.size > 0 ? corners.values().next().value : null);
    const targetFoundId = corners.has(calTargetId) ? calTargetId : (corners.size > 0 ? corners.keys().next().value : null);
    if (targetCorner && targetFoundId != null) {
      const sizeMm = markerSizeFor(targetFoundId);
      const mp = solvePnpMarkerPose(cv, targetCorner, sizeMm, armCamW, armCamH, intr);
      if (mp) {
        const pc = [mp.t[0] / 1000, mp.t[1] / 1000, mp.t[2] / 1000];
        const cam = armCameraWorld();
        const R = cam.R;
        const worldTag: [number, number, number] = [
          cam.t[0] + R[0] * pc[0] + R[1] * pc[1] + R[2] * pc[2],
          cam.t[1] + R[3] * pc[0] + R[4] * pc[1] + R[5] * pc[2],
          cam.t[2] + R[6] * pc[0] + R[7] * pc[1] + R[8] * pc[2],
        ];
        const robTag = toRobot(worldTag);
        const robGrip = solver ? toRobot(solver.sitePosition() as [number, number, number]) : [0, 0, 0];
        const rollRad = Math.atan2(targetCorner[1][1] - targetCorner[0][1], targetCorner[1][0] - targetCorner[0][1]);

        pickCalState.estimatedTag = {
          id: targetFoundId,
          x: robTag[0],
          y: robTag[1],
          z: robTag[2],
          rollDeg: (rollRad * 180) / Math.PI,
          visible: true,
          timestamp: Date.now(),
          camX: mp.t[0],
          camY: mp.t[1],
          camZ: mp.t[2],
          relX: robTag[0] - robGrip[0],
          relY: robTag[1] - robGrip[1],
          relZ: robTag[2] - robGrip[2],
          Rcam: mp.R,
        };
      }
    }
  }

  // Arm joint indices: 0 shoulder_pan, 1 shoulder_lift, 2 elbow_flex,
  // 3 wrist_flex, 4 wrist_roll.

  // Whether the jaws line up with the tag's y axis rather than its x axis. The
  // right answer depends on which way the object under the tag is long, which a
  // square tag cannot tell us.
  let gripAcrossTag = $state(true);

  /**
   * Come down on the object vertically, rather than along the tag's measured
   * normal.
   *
   * For anything lying on a flat surface the normal *is* world-up, so measuring it
   * adds noise and nothing else — and the noise is not small: solvePnP on a planar
   * marker has two nearly-equal solutions whose normals differ, and at an oblique
   * angle it flips between them, which showed up as the aim demanding 17°, then
   * 76°, then 52° on three consecutive sightings of a stationary block. Fixing the
   * approach axis to straight down removes that entirely. The tag is still what
   * says *where* to descend and which way to turn the jaws; it just no longer gets
   * a vote on which way is down.
   */
  let topDownGrasp = $state(true);

  // How high above the tag the gripper is parked, along the tag's normal (m).
  //
  // Closer is better where the camera still sees the tag: the final sighting is
  // taken from here, and every error that scales with distance — depth from
  // apparent size, the lever arm on a joint-angle error — shrinks with it. The
  // limit is the wrist camera sitting ~76 mm off to the side, so past a point the
  // tag slides out of frame and the last stretch has to be flown blind.
  let hoverM = $state(0.05);

  // Sideways shift onto the grasp line, before the final descent (m).
  //
  // Centring the tag in the image centres it under the *camera*, and the camera
  // is bolted to one side of the wrist — so the jaws straddle a line a few mm off
  // the tag. That offset is fixed by the mount, which makes it a constant to
  // apply rather than something to servo on. Positive shifts toward the static
  // ("nose") jaw; negative goes the other way.
  // 8 mm, kept because it measurably beat 0: with the shift the block moved 3-6 mm
  // on contact, without it 0. Not yet a grip, but the jaws are reaching it.
  let graspLeftM = $state(0.008);

  // ── Tag observation ─────────────────────────────────────────────────────────
  //
  // The remembered fix is kept in the ROBOT's frame — origin at the base, +x
  // ahead, +y left — because that is the frame the tag stays still in while the
  // arm moves. World coordinates would go stale the moment the base was nudged;
  // camera coordinates would go stale every iteration.

  function robotYawRad(): number {
    return hasBase ? (robotYawDeg * Math.PI) / 180 : 0;
  }

  /** World point → robot frame. */
  function toRobot(p: number[]): [number, number, number] {
    const a = -robotYawRad();
    const dx = p[0] - (hasBase ? robotX : 0);
    const dy = p[1] - (hasBase ? robotY : 0);
    return [dx * Math.cos(a) - dy * Math.sin(a), dx * Math.sin(a) + dy * Math.cos(a), p[2]];
  }

  /** Robot frame → world point. */
  function fromRobot(p: number[]): [number, number, number] {
    const a = robotYawRad();
    return [
      (hasBase ? robotX : 0) + p[0] * Math.cos(a) - p[1] * Math.sin(a),
      (hasBase ? robotY : 0) + p[0] * Math.sin(a) + p[1] * Math.cos(a),
      p[2],
    ];
  }

  /** Apply calibrated physical pick offset (in the robot's local body frame) to a world point. */
  function applyPickOffset(worldPoint: [number, number, number]): [number, number, number] {
    const pRob = toRobot(worldPoint);
    pRob[0] += (pickCal.xOffsetMm || 0) / 1000;
    pRob[1] += (pickCal.yOffsetMm || 0) / 1000;
    pRob[2] += (pickCal.zOffsetMm || 0) / 1000;
    return fromRobot(pRob);
  }

  /**
   * Intrinsics for whichever arm camera is feeding the detector — the calibrated
   * ones for a real camera, exact pinhole ones for the rendered view. Reaching for
   * `intrinsics` directly is wrong: it is null until a real camera is calibrated,
   * so in simulation every visual step would bail out.
   */
  function armIntrinsics(): Intrinsics {
    if (intrinsics) return intrinsics;
    return povIntrinsics(armPovView?.fovDeg ?? 60, armCamW, armCamH);
  }

  interface TagObs {
    /** false when this is dead reckoning rather than a live detection. */
    live: boolean;
    cx: number; // tag centre in the image (px)
    cy: number;
    left: number; // leftmost corner (px)
    edgePx: number;
    depthM: number;
    rollRad: number;
    /** Tag→camera rotation from solvePnP (row-major). Only on a live sighting. */
    Rcam?: number[];
    /** Tag centre in the camera frame (m). Only on a live sighting. */
    pc?: [number, number, number];
  }

  // Last place the tag was seen, in the robot frame, plus its measured size. The
  // tag doesn't move, so this stays valid while the arm does — which is what lets
  // the approach carry on through the moments the gripper's own jaw hides it.
  let tagFix: { id: number; p: [number, number, number]; sizeM: number; rollRad: number } | null =
    null;

  /**
   * Where the tag is in the image: measured if it can be seen, otherwise predicted
   * by projecting the last known position through the camera's current pose.
   *
   * The prediction is what makes an occluded tag survivable. Every quantity the
   * approach servos on — centre, left edge, apparent size — falls out of the
   * projection, so the loop doesn't care which kind of observation it got.
   */
  function observeTag(id: number): TagObs | null {
    const cam = armCameraWorld();
    const intr = armIntrinsics();
    const fx = intr?.cameraMatrix[0] || armCamW;
    const fy = intr?.cameraMatrix[4] || fx;

    if (cv && detector && armGrabCtx && armSrcMat && armGrayMat && intr) {
      const { width, height } = armGrabCtx.canvas;
      const armSrc =
        remoteCams.canvas(REMOTE_ARM_CAM) ??
        (armCamStream ? armVideo : (armPovCanvas as CanvasImageSource));
      armGrabCtx.drawImage(armSrc, 0, 0, width, height);
      armSrcMat.data.set(armGrabCtx.getImageData(0, 0, width, height).data);
      cv.cvtColor(armSrcMat, armGrayMat, cv.COLOR_RGBA2GRAY);
      const c = detectMarkers(cv, detector, armGrayMat).get(id);
      if (c) {
        const cx = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4;
        const cy = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4;
        let edgePx = 0;
        for (let k = 0; k < 4; k++) {
          const a = c[k], b = c[(k + 1) % 4];
          edgePx = Math.max(edgePx, Math.hypot(b[0] - a[0], b[1] - a[1]));
        }
        const left = Math.min(c[0][0], c[1][0], c[2][0], c[3][0]);
        const rollRad = Math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0]);
        const mp = solvePnpMarkerPose(cv, c, markerSizeFor(id), armCamW, armCamH, intr);
        const depthM = mp ? mp.t[2] / 1000 : 0.2;
        if (mp) {
          // Camera frame (mm) → world → robot frame, and remember it.
          const pc = [mp.t[0] / 1000, mp.t[1] / 1000, mp.t[2] / 1000];
          const R = cam.R;
          const world = [
            cam.t[0] + R[0] * pc[0] + R[1] * pc[1] + R[2] * pc[2],
            cam.t[1] + R[3] * pc[0] + R[4] * pc[1] + R[5] * pc[2],
            cam.t[2] + R[6] * pc[0] + R[7] * pc[1] + R[8] * pc[2],
          ];
          tagFix = { id, p: toRobot(world), sizeM: markerSizeFor(id) / 1000, rollRad };
        }
        const pcam: [number, number, number] | undefined = mp
          ? [mp.t[0] / 1000, mp.t[1] / 1000, mp.t[2] / 1000]
          : undefined;
        return { live: true, cx, cy, left, edgePx, depthM, rollRad, Rcam: mp?.R, pc: pcam };
      }
    }

    // Nothing seen — fall back to the remembered position.
    if (!tagFix || tagFix.id !== id) return null;
    const w = fromRobot(tagFix.p);
    const d = [w[0] - cam.t[0], w[1] - cam.t[1], w[2] - cam.t[2]];
    const R = cam.R; // world→camera is its transpose: dot with each column
    const pc = [
      R[0] * d[0] + R[3] * d[1] + R[6] * d[2],
      R[1] * d[0] + R[4] * d[1] + R[7] * d[2],
      R[2] * d[0] + R[5] * d[1] + R[8] * d[2],
    ];
    if (pc[2] < 0.02) return null; // behind the camera: nothing sensible to predict
    const edgePx = (fx * tagFix.sizeM) / pc[2];
    const cx = (fx * pc[0]) / pc[2] + armCamW / 2;
    const cy = (fy * pc[1]) / pc[2] + armCamH / 2;
    // A prediction that lands well outside the frame is a stale fix, not a
    // sighting: servoing on it produced errors in the thousands of pixels.
    if (Math.abs(cx - armCamW / 2) > armCamW || Math.abs(cy - armCamH / 2) > armCamH) return null;
    return { live: false, cx, cy, left: cx - edgePx / 2, edgePx, depthM: pc[2], rollRad: tagFix.rollRad };
  }

  // Fraction of the observed error corrected per iteration. Halving each time
  // converges in a handful of steps while staying stable against overshoot.
  // Fraction of the observed error corrected per iteration. Halving each time
  // converges in a handful of steps while staying stable against overshoot.
  const SERVO_GAIN = 0.5;

  // ── Approach: iterated look-then-move ───────────────────────────────────────
  //
  // Each pass takes ONE measurement, works out where the gripper should be, and
  // moves most of the way there. No per-pixel feedback loop: solvePnP already
  // gives the tag's full pose, so the goal is computed rather than hunted for.
  //
  // Why this shape. Image-space servoing reduces a 6-DOF measurement to two
  // scalars and then tries to recover the geometry by trial and error, which is
  // where the sign confusion, the axis coupling and the frame-lag sensitivity all
  // came from. Here every iteration is an independent measurement and an absolute
  // goal, so a bad step is corrected by the next one rather than compounding.
  //
  // And it still tolerates the calibration error that made absolute positions
  // untrustworthy in the first place: a systematic error (hand-eye, arm zero,
  // depth scale) is a roughly constant *fraction* of the distance remaining, so
  // moving a fixed fraction of the way each time shrinks it geometrically. Three
  // or four passes converge even with a 20% error in the transform.
  //
  // Orientation goes FIRST, then travels with the position. The gripper turns to
  // face the tag before it moves, and every pass re-aims from the freshest
  // sighting — so this is a full 6-DOF pose servo, not a translation followed by a
  // turn. The cost is that the opening turn swings the camera and can take the tag
  // out of frame; the measurement that produced the turn is kept as a fallback
  // goal so the approach can still fly to it if that happens.
  const PBVS_ITERS = 8;
  const PBVS_ALPHA = 0.6; // fraction of the computed move to take each pass
  const PBVS_MAX_STEP = 0.06; // m — cap on a single pass, in case a reading is wild
  const PBVS_DONE_M = 0.006; // m — close enough to hand over to the blind drop
  // ~5°. Not tighter: the jaws span the object, so a few degrees costs nothing at
  // the grasp, while demanding 2° from an estimate that jitters by more than that
  // just spends passes chasing noise.
  const PBVS_AIM_DONE_RAD = 0.087;
  const PBVS_MAX_TURN = 0.44; // rad (~25°) — cap on a single pass's turn
  // Two thresholds, because "can finish blind" and "should finish blind" are not
  // the same question. Under PREFER, the gap is short enough that a blind run is
  // as good as another look. Between PREFER and MAX, it is worth closing some
  // distance and looking again — losing the tag at 185 mm and flying the rest open
  // loop threw away every measurement the approach exists to make. MAX is the
  // hard cap: past it, a blind run is a guess with the table in the way.
  const OPEN_LOOP_PREFER_M = 0.06;
  const OPEN_LOOP_MAX_M = 0.2;
  const STALL_M = 0.01; // a pass that closes less than this counts as no progress
  const STALL_PASSES = 2; // …this many in a row means the arm can't get there
  /**
   * A stall this close to the goal is arrival, not failure.
   *
   * The arm creeps in to 26 mm, the last two passes close a millimetre each
   * because it is at the edge of its envelope, and the stall test — which exists
   * to catch a hopeless 150 mm gap — fires and throws away a pick that was
   * essentially finished. Whatever is left gets added to the descent instead.
   */
  const GOOD_ENOUGH_M = 0.06;
  /** Set when the approach gave up because the goal is outside the arm's envelope. */
  let approachOutOfReach = $state(false);
  /** How far short it was (m) — how much the base needs to make up. */
  let approachShortfallM = $state(0);
  /** Rough base speed under a full-forward nudge (m/s), measured from the sim. */
  const NUDGE_SPEED = 0.09;

  /**
   * Finish the move without the camera, from the last computed goal, holding the
   * aim that sighting produced — gripper pointing down the tag's normal, jaws
   * across it.
   *
   * The goal is held in the robot's frame, so it stays valid even if the base is
   * nudged, and by the time this runs the remaining distance is small.
   */
  async function finishOpenLoop(
    id: number,
    goalRobot: [number, number, number],
    aim: number[] | null,
  ): Promise<boolean> {
    if (!solver) return false;
    const goal = fromRobot(goalRobot);
    const at = solver.sitePosition();
    const delta = [goal[0] - at[0], goal[1] - at[1], goal[2] - at[2]];
    const dist = Math.hypot(delta[0], delta[1], delta[2]);
    if (dist > OPEN_LOOP_MAX_M) {
      armPickMsg = `Lost tag ${id} with ${(dist * 1000).toFixed(0)} mm still to go — too far to finish blind.`;
      return false;
    }
    armPickMsg = `Tag ${id} out of view — finishing the last ${(dist * 1000).toFixed(0)} mm from the last fix…`;
    if (aim) holdRd = aim;
    target = [target[0] + delta[0], target[1] + delta[1], target[2] + delta[2]];
    await settleAndMeasure();
    // The arm has moved, so the tag is often back in view here — worth one more
    // look before settling for the blind result.
    await recentreOverTag(id);
    armPickMsg = `Hovering over tag ${id} (last ${(dist * 1000).toFixed(0)} mm flown blind).`;
    return true;
  }

  /**
   * The gripper orientation that points at the tag: approach straight down the
   * tag's normal, with the jaws lying along the chosen tag edge.
   *
   * Built rather than searched. The jaws' direction within the wrist is a fixed
   * property of the gripper, so the roll needed to line them up with an edge is
   * one subtraction of angles, not a servo loop.
   */
  function aimAtTag(normal: number[], edge: number[]): number[] | null {
    if (!solver) return null;
    const jawW = jawAxisWorld();
    if (!jawW) return null;
    const norm = (v: number[]) => {
      const n = Math.hypot(v[0], v[1], v[2]) || 1;
      return [v[0] / n, v[1] / n, v[2] / n];
    };
    const dot = (a: number[], b: number[]) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    const cross = (a: number[], b: number[]) => [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
    ];
    // Approach: straight down the tag's normal.
    const x = norm([-normal[0], -normal[1], -normal[2]]);
    // A reference pair perpendicular to it, to measure roll angles against.
    const seed = Math.abs(x[2]) > 0.9 ? [1, 0, 0] : [0, 0, 1];
    const y0 = norm(cross(seed, x));
    const z0 = cross(x, y0);
    // Where the jaws sit in the wrist, as an angle in that pair — constant for the
    // gripper, so it can be measured once from the current pose.
    const st = solver.siteRotation();
    const sx = [st[0], st[3], st[6]], sy = [st[1], st[4], st[7]], sz = [st[2], st[5], st[8]];
    const jl = [dot(jawW, sx), dot(jawW, sy), dot(jawW, sz)]; // jaw in the wrist's frame
    const jaw = Math.atan2(jl[2], jl[1]);
    // Where the tag's edge sits in the same pair.
    const ep = norm([
      edge[0] - dot(edge, x) * x[0],
      edge[1] - dot(edge, x) * x[1],
      edge[2] - dot(edge, x) * x[2],
    ]);
    const want = Math.atan2(dot(ep, z0), dot(ep, y0));
    // Roll the reference pair so the jaws land on the edge. Half-turn wrap: a
    // grasp with the jaws swapped is the same grasp.
    const th = wrapHalfTurn(want - jaw);
    const c = Math.cos(th), sn = Math.sin(th);
    const y = [0, 1, 2].map((k) => y0[k] * c + z0[k] * sn);
    const z = [0, 1, 2].map((k) => -y0[k] * sn + z0[k] * c);
    // Keep the choice of half-turn continuous with where the wrist already is;
    // otherwise noise near the ±90° fold flips the aim 180° between sightings.
    const aim = [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
    return nearestHalfTurn(aim, holdRd ?? solver.siteRotation());
  }

  /** The tag's pose in the world from one sighting: centre, normal, chosen edge. */
  function tagPoseNow(id: number): { tag: [number, number, number]; normal: number[]; edge: number[] } | null {
    const obs = observeTag(id);
    if (!obs || !obs.live || !obs.pc || !obs.Rcam) return null;
    const cam = armCameraWorld();
    const R = cam.R;
    const pc = obs.pc;
    const rawTag: [number, number, number] = [
      cam.t[0] + R[0] * pc[0] + R[1] * pc[1] + R[2] * pc[2],
      cam.t[1] + R[3] * pc[0] + R[4] * pc[1] + R[5] * pc[2],
      cam.t[2] + R[6] * pc[0] + R[7] * pc[1] + R[8] * pc[2],
    ];
    const tag = applyPickOffset(rawTag);
    const tagR = matMul3x3(R, obs.Rcam); // columns: the tag's own axes in world
    let normal = [tagR[2], tagR[5], tagR[8]];
    // Point it back at the camera: solvePnP's z can come out either way.
    const toCam = [cam.t[0] - tag[0], cam.t[1] - tag[1], cam.t[2] - tag[2]];
    if (normal[0] * toCam[0] + normal[1] * toCam[1] + normal[2] * toCam[2] < 0) {
      normal = normal.map((v) => -v);
    }
    // Which tag edge the jaws should lie along. A square tag can't say which way
    // the object under it is long, so this is a choice, not a measurement.
    const edge = gripAcrossTag ? [tagR[1], tagR[4], tagR[7]] : [tagR[0], tagR[3], tagR[6]];
    // One override does the whole job: the hover point is placed along this vector
    // and the approach axis is its opposite, so pinning it to vertical makes both
    // the parking spot and the descent straight-down.
    return { tag, normal: topDownGrasp ? [0, 0, 1] : normal, edge };
  }

  /**
   * Settle over the tag: the last few passes, correcting both where the gripper is
   * and where it points.
   *
   * This is the most valuable measurement of the run — taken from the closest,
   * most head-on vantage point, which is where the arm's own calibration error
   * matters least. Every earlier reading was made from further away, so anything
   * still wrong here is what the grasp inherits.
   */
  async function recentreOverTag(id: number, passes = 3): Promise<void> {
    if (!solver) return;
    for (let i = 0; i < passes; i++) {
      const m = tagPoseNow(id);
      if (!m) {
        armPickMsg = `Tag ${id} out of view while settling — leaving the gripper here.`;
        return;
      }
      const goal = [0, 1, 2].map((k) => m.tag[k] + m.normal[k] * hoverM);
      const at = solver.sitePosition();
      const delta = [goal[0] - at[0], goal[1] - at[1], goal[2] - at[2]];
      const dist = Math.hypot(delta[0], delta[1], delta[2]);
      const aim = aimAtTag(m.normal, m.edge);
      const turn = aim ? rotAngleBetween(holdRd ?? solver.siteRotation(), aim) : 0;
      const off = `${(dist * 1000).toFixed(0)} mm, ${((turn * 180) / Math.PI).toFixed(1)}°`;
      if (dist < PBVS_DONE_M && turn < PBVS_AIM_DONE_RAD) {
        armPickMsg = `Settled over tag ${id} — ${off} off.`;
        return;
      }
      armPickMsg = `Settling over tag ${id} — ${off} off…`;
      if (aim) holdRd = slewRotation(holdRd ?? solver.siteRotation(), aim, PBVS_MAX_TURN);
      const scale = Math.min(0.8, PBVS_MAX_STEP / dist);
      target = [
        target[0] + delta[0] * scale,
        target[1] + delta[1] * scale,
        target[2] + delta[2] * scale,
      ];
      await settleAndMeasure();
      await delay(APPROACH_SETTLE_MS);
    }
  }

  /**
   * Turn the gripper to point at the tag, before any move toward it.
   *
   * One capped turn, then straight into the approach — deliberately not a loop
   * that turns until square. Turning until square starved the translation
   * entirely: it burned every pass on the spot without closing a millimetre, and
   * because swinging the wrist swings the camera, each new sighting produced a
   * *different* aim (71° to go, then 53°, then 61°), so it never converged and
   * lost the tag still 267 mm out. Orientation still leads, but the main loop is
   * where it finishes, alongside the translation that keeps the tag in frame.
   */
  async function faceTag(
    id: number,
  ): Promise<{ goal: [number, number, number]; aim: number[] | null } | null> {
    if (!solver) return null;
    for (let i = 0; i < 4; i++) {
      const m = tagPoseNow(id);
      if (!m) {
        armPickMsg = `Tag ${id} out of view — waiting for a sighting to aim at (${i + 1}/4)…`;
        await delay(300);
        continue;
      }
      const goal = toRobot([0, 1, 2].map((k) => m.tag[k] + m.normal[k] * hoverM));
      const aim = aimAtTag(m.normal, m.edge);
      if (!aim) return { goal, aim: null };
      const ref = holdRd ?? solver.siteRotation();
      const turn = rotAngleBetween(ref, aim);
      if (turn > PBVS_AIM_DONE_RAD) {
        armPickMsg = `Turning to face tag ${id} — ${((turn * 180) / Math.PI).toFixed(0)}° to go…`;
        holdRd = slewRotation(ref, aim, PBVS_MAX_TURN);
        await settleAndMeasure();
        await delay(APPROACH_SETTLE_MS);
      }
      return { goal, aim };
    }
    return null;
  }

  /**
   * Hover the gripper over tag `id`, pointing at it.
   *
   * The gripper turns to face the tag first, then flies to it, re-aiming from each
   * new sighting as it goes. The goal is the *gripper's* pose, not the camera's:
   * hoverM above the tag along its normal, approach axis pointing down that
   * normal, jaws lying along the chosen tag edge.
   */
  async function approachTag(id: number): Promise<boolean> {
    const intr = armIntrinsics();
    if (!solver || !intr) {
      armPickMsg = 'No arm camera intrinsics — calibrate the camera, or use the simulated view.';
      return false;
    }

    handBackToManual(); // anchor: the target is where the gripper is right now
    // Hold the wrist's attitude rather than letting IK pick it. With position-only
    // IK the arm satisfies a sideways target partly by tilting the wrist, and a
    // camera that turns while it translates makes each measurement disagree with
    // the last.
    holdRd = solver.siteRotation();
    let missed = 0;
    // Goal and aim from the last good measurement, kept in the ROBOT's frame — the
    // frame the tag is stationary in while the arm moves. The tag leaves the view
    // near the end of every approach, so this is what lets the move finish rather
    // than fail.
    let goalFix: [number, number, number] | null = null;
    let aimFix: number[] | null = null;
    let stalled = 0;
    let lastDist = Infinity;
    approachOutOfReach = false;

    // Aim before moving. If the turn takes the tag out of frame, the sighting it
    // was computed from is still a usable goal for the rest of the move.
    const first = await faceTag(id);
    if (!first) {
      armPickMsg = `Tag ${id} not visible — cannot approach.`;
      return false;
    }
    goalFix = first.goal;
    aimFix = first.aim;

    for (let i = 0; i < PBVS_ITERS; i++) {
      const m = tagPoseNow(id);
      if (!m) {
        missed++;
        if (goalFix) {
          // Out of view, but we know where we were going. Finishing blind is only
          // sane over a short gap; over a long one, close some of it and look
          // again — the tag usually comes back, because getting nearer widens what
          // the camera covers and undoes whatever turn pushed it out of frame.
          const goal = fromRobot(goalFix);
          const at = solver.sitePosition();
          const d = [goal[0] - at[0], goal[1] - at[1], goal[2] - at[2]];
          const dist = Math.hypot(d[0], d[1], d[2]);
          if (dist <= OPEN_LOOP_PREFER_M || missed > 4) {
            return await finishOpenLoop(id, goalFix, aimFix);
          }
          // Same stall test as the sighted branch, and it matters more here: a
          // blind arm grinding against its own reach limit looks identical to one
          // patiently closing the gap, and it was flying the last 150 mm open loop
          // and closing on nothing, every time.
          stalled = lastDist - dist < STALL_M ? stalled + 1 : 0;
          lastDist = dist;
          if (stalled >= STALL_PASSES) {
            approachShortfallM = dist;
            if (dist <= GOOD_ENOUGH_M) return await finishOpenLoop(id, goalFix, aimFix);
            approachOutOfReach = true;
            armPickMsg =
              `Tag ${id} is out of the arm's reach — ${(dist * 1000).toFixed(0)} mm away ` +
              `and no longer closing. The base needs to get nearer.`;
            return false;
          }
          const scale = Math.min(PBVS_ALPHA, PBVS_MAX_STEP / dist);
          target = [target[0] + d[0] * scale, target[1] + d[1] * scale, target[2] + d[2] * scale];
          armPickMsg =
            `Tag ${id} out of view — closing ${(dist * scale * 1000).toFixed(0)} mm ` +
            `of the remaining ${(dist * 1000).toFixed(0)} mm and looking again (${missed}/4)…`;
          await settleAndMeasure();
          await delay(APPROACH_SETTLE_MS);
          continue;
        }
        if (missed > 3) {
          armPickMsg = `Tag ${id} not visible — cannot approach.`;
          return false;
        }
        armPickMsg = `Tag ${id} out of view — waiting for a sighting (${missed}/3)…`;
        await delay(300);
        continue;
      }
      missed = 0;

      // Where the gripper should end up, and how it should be turned.
      const goal = [0, 1, 2].map((k) => m.tag[k] + m.normal[k] * hoverM);
      goalFix = toRobot(goal);
      aimFix = aimAtTag(m.normal, m.edge) ?? aimFix;

      const at = solver.sitePosition();
      const delta = [goal[0] - at[0], goal[1] - at[1], goal[2] - at[2]];
      const dist = Math.hypot(delta[0], delta[1], delta[2]);
      const turn = aimFix ? rotAngleBetween(holdRd ?? solver.siteRotation(), aimFix) : 0;

      if (dist < PBVS_DONE_M && turn < PBVS_AIM_DONE_RAD) {
        approachShortfallM = 0; // arrived properly: nothing for the descent to make up
        await recentreOverTag(id);
        armPickMsg = `Hovering over tag ${id}, ${(hoverM * 1000).toFixed(0)} mm above it.`;
        return true;
      }

      // Re-aim from this sighting, then take most of the way there. IK tracks the
      // target closely, so moving the target by a fraction of the gap moves the
      // gripper by the same fraction.
      if (aimFix) holdRd = slewRotation(holdRd ?? solver.siteRotation(), aimFix, PBVS_MAX_TURN);
      const scale = Math.min(PBVS_ALPHA, PBVS_MAX_STEP / dist);
      target = [
        target[0] + delta[0] * scale,
        target[1] + delta[1] * scale,
        target[2] + delta[2] * scale,
      ];
      armPickMsg =
        `Approaching tag ${id} — ${(dist * 1000).toFixed(0)} mm to go, ` +
        `${((turn * 180) / Math.PI).toFixed(0)}° to turn · ` +
        `moving ${(dist * scale * 1000).toFixed(0)} mm…`;
      await settleAndMeasure();
      await delay(APPROACH_SETTLE_MS); // the detector runs behind the motion

      // Commanding a move the arm cannot make looks exactly like a move it simply
      // hasn't finished, and silently burns every remaining pass. If the gap stops
      // closing, the target is outside the arm's envelope — which is a job for the
      // base, so say so rather than grinding to the iteration limit.
      const after = solver.sitePosition();
      const left = Math.hypot(goal[0] - after[0], goal[1] - after[1], goal[2] - after[2]);
      lastDist = left;
      stalled = dist - left < STALL_M ? stalled + 1 : 0;
      if (stalled >= STALL_PASSES) {
        approachShortfallM = left;
        if (left <= GOOD_ENOUGH_M) {
          armPickMsg =
            `Close enough to tag ${id} — ${(left * 1000).toFixed(0)} mm off and no longer ` +
            `closing; grasping from here.`;
          await recentreOverTag(id);
          return true;
        }
        approachOutOfReach = true;
        armPickMsg =
          `Tag ${id} is out of the arm's reach — ${(left * 1000).toFixed(0)} mm away and ` +
          `the last ${STALL_PASSES} passes closed almost nothing.`;
        return false;
      }
    }
    if (goalFix) return await finishOpenLoop(id, goalFix, aimFix);
    armPickMsg = `Gave up approaching tag ${id} after ${PBVS_ITERS} passes.`;
    return false;
  }

  /** Step 2 of the pick: the whole approach loop, as a one-shot. */
  async function runApproachStep(): Promise<boolean> {
    if (armPickBusy) {
      armPickMsg = 'Approach skipped — another arm routine is still running.';
      return false;
    }
    armPickBusy = true;
    try {
      return await approachTag(blockTag);
    } catch (err) {
      armPickMsg = `Approach failed: ${err instanceof Error ? err.message : String(err)}`;
      console.error('approachTag', err);
      return false;
    } finally {
      holdRd = null; // release the orientation lock however the step ended
      jointHoldPose = null;
      handBackToManual();
      armPickBusy = false;
    }
  }

  const ROLL_TOL_RAD = 0.087; // ~5° — square enough for a jaw that spans the object
  const ROLL_STEP_MAX = 0.3; // rad — biggest wrist_roll nudge per iteration
  const ROLL_ITERS = 30;
  const APPROACH_SETTLE_MS = 140; // let the camera catch up before measuring again
  const ROLL_SETTLE_MS = 300; // let the camera catch up before the next measurement
  // wrist_roll travel, from the model's joint range in so101_new_calib.xml.
  const ROLL_LO = -2.7438, ROLL_HI = 2.8412;

  /**
   * Fold an angle into ±90°: the jaws are symmetric, so a half turn is the same
   * grasp — but a quarter turn is NOT. The object under a square tag is usually
   * oblong (the sim's block is 75 × 25 mm), and the jaws have to close across its
   * short side. Treating quarter turns as equivalent let the arm settle happily on
   * the long axis, where the fingers can't reach around the block at all — and
   * because it was already "aligned", it never rotated.
   */
  function wrapHalfTurn(a: number): number {
    return a - Math.PI * Math.round(a / Math.PI);
  }

  /**
   * Step 3 — rotate wrist_roll until the tag's edges line up with the jaws.
   *
   * Which way up the tag ends is irrelevant: the jaws are symmetric, so parallel
   * and anti-parallel are the same grasp. The error is wrapped into ±90° and the
   * arm takes whichever of the two is the shorter rotation — insisting on upright
   * used to send it on a near-half-turn, sometimes flipping the tag over on the
   * way. If that rotation would hit a joint limit, the identical alignment a
   * half-turn away is used instead.
   *
   * This one needs a real sighting: a predicted observation carries the roll from
   * wherever the tag was last actually seen, which is exactly what we're measuring.
   */
  /** World direction the jaws open along: fingertip to fingertip. */
  function jawAxisWorld(): number[] | null {
    if (!session || fingerSiteIds[0] < 0 || fingerSiteIds[1] < 0) return null;
    try {
      const sx = session.data.site_xpos as Float64Array;
      const a = fingerSiteIds[0] * 3;
      const b = fingerSiteIds[1] * 3;
      const v = [sx[b] - sx[a], sx[b + 1] - sx[a + 1], sx[b + 2] - sx[a + 2]];
      const n = Math.hypot(v[0], v[1], v[2]);
      return n > 1e-6 ? [v[0] / n, v[1] / n, v[2] / n] : null;
    } catch {
      return null;
    }
  }

  /**
   * How far the wrist must roll to square the tag up, in radians, computed rather
   * than searched.
   *
   * Servoing on the tag's apparent tilt only works if rolling the wrist spins the
   * image, and with the camera mounted at a steep angle to the wrist axis it
   * mostly swings the view instead — the measured tilt barely responds, then
   * responds the wrong way, which is why the iterative version wandered.
   *
   * So the angle comes from geometry. solvePnP gives the tag's full orientation,
   * the camera's pose gives its right axis, and both are projected onto the plane
   * the wrist actually rotates in (perpendicular to the approach axis, which the
   * grasp site's own x axis is). The answer is the angle between them — the same
   * criterion the image version aimed at ("the tag's top edge lies along the image
   * x axis"), solved in one step instead of hunted for.
   */
  function rollErrorRad(id: number): number | null {
    if (!solver) return null;
    const obs = observeTag(id);
    if (!obs || !obs.live || !obs.Rcam) return null;
    const cam = armCameraWorld();
    const tagWorld = matMul3x3(cam.R, obs.Rcam); // columns: tag axes in world
    // Which edge of the tag the jaws should lie along. A square tag can't say
    // which way the object under it is long, so this is a choice, not a
    // measurement: "across" lines the jaws up with the tag's y axis instead of x.
    const edge = gripAcrossTag
      ? [tagWorld[1], tagWorld[4], tagWorld[7]]
      : [tagWorld[0], tagWorld[3], tagWorld[6]];
    const st = solver.siteRotation();
    const axis = [st[0], st[3], st[6]]; // approach = the wrist's roll axis
    // Reference direction: the line between the fingertips — the way the jaws
    // actually open. Using the camera's right axis instead only matched while the
    // camera happened to be mounted square to the gripper; once it wasn't, the
    // measured error settled 90° away from square, which is exactly the tag's
    // other edge.
    const jaw = jawAxisWorld();
    if (!jaw) return null;

    const proj = (v: number[]) => {
      const d = v[0] * axis[0] + v[1] * axis[1] + v[2] * axis[2];
      const p = [v[0] - d * axis[0], v[1] - d * axis[1], v[2] - d * axis[2]];
      const n = Math.hypot(p[0], p[1], p[2]);
      return n > 1e-6 ? [p[0] / n, p[1] / n, p[2] / n] : null;
    };
    const a = proj(jaw);
    const b = proj(edge);
    if (!a || !b) return null; // edge-on to the roll axis: no angle to speak of
    const cross = [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
    ];
    const sin = cross[0] * axis[0] + cross[1] * axis[1] + cross[2] * axis[2];
    const cos = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    // Half-turn wrap: the same grasp, jaws swapped.
    return wrapHalfTurn(Math.atan2(sin, cos));
  }

  /**
   * Step 3 — roll the wrist so the tag sits square in the gripper.
   *
   * One computed move gets almost all of it; the loop only refines what the step
   * clamp and the arm's own tracking error leave behind. The joint's sign against
   * the geometric angle is confirmed on the first move and then fixed, so a
   * re-mounted camera can't send it the wrong way indefinitely.
   */
  async function alignByRoll(id: number): Promise<boolean> {
    // Measure the response instead of assuming it.
    //
    // Every fixed-sign version of this has failed, and the logs say why: the error
    // does not move one-for-one against the joint. With the camera mounted at a
    // steep angle to the wrist axis, 17° of roll was shifting the measured error
    // by 3-5°, so a step sized for a 1:1 response undershot badly and a step whose
    // direction was guessed could not recover.
    //
    // So each iteration estimates the local slope d(error)/d(joint) from the last
    // move — a secant method — and steps by −error/slope. The sign falls out of
    // the measurement rather than being hard-coded, and the size adapts to however
    // weakly the wrist happens to move the tag from this pose. Slopes too small to
    // trust are ignored, keeping the last good estimate.
    const J_MIN = 0.08; // below this the move told us nothing useful
    const J_MOVED = 0.03; // rad — a joint change big enough to measure a slope from
    let slope = -1; // starting guess: rolling one way squares it up, one-for-one
    let prev: { joint: number; err: number } | null = null;

    for (let i = 0; i < ROLL_ITERS; i++) {
      const err = rollErrorRad(id);
      if (err === null) {
        armPickMsg = `Tag ${id} not visible — cannot square it up.`;
        return false;
      }
      const deg = (err * 180) / Math.PI;
      if (Math.abs(err) < ROLL_TOL_RAD) {
        armPickMsg = `Tag ${id} square — ${deg.toFixed(1)}° off.`;
        return true;
      }

      const joint = jointAngles[4];
      if (prev && Math.abs(joint - prev.joint) > J_MOVED) {
        const measured = wrapHalfTurn(err - prev.err) / (joint - prev.joint);
        if (Math.abs(measured) > J_MIN) slope = measured;
      }
      prev = { joint, err };

      armPickMsg =
        `Squaring tag ${id} — ${deg.toFixed(1)}° off · ` +
        `wrist_roll ${((joint * 180) / Math.PI).toFixed(1)}° · ` +
        `response ${slope.toFixed(2)}…`;

      const raw = -err / slope;
      const size = Math.max(-ROLL_STEP_MAX, Math.min(ROLL_STEP_MAX, raw));
      // Equivalent alignments, in order of preference: the direct one, then the
      // same grasp a quarter or half turn away, for when travel runs out.
      const q = Math.PI / 2;
      const wants = [
        joint + size,
        joint + size + q, joint + size - q,
        joint + size + 2 * q, joint + size - 2 * q,
      ];
      const want = wants.find((w) => w >= ROLL_LO && w <= ROLL_HI);
      if (want === undefined) {
        armPickMsg = `Cannot square tag ${id} — wrist_roll is at its limit, ${deg.toFixed(1)}° off.`;
        return false;
      }
      const pose = jointAngles.slice(0, 5);
      pose[4] = want;
      jointHoldPose = pose;
      await settleAndMeasure();
      await delay(ROLL_SETTLE_MS); // the image lags the motion; measuring early reads a stale angle
    }
    const last = rollErrorRad(id);
    armPickMsg =
      `Gave up squaring tag ${id} after ${ROLL_ITERS} tries` +
      `${last === null ? '' : ` — ${((last * 180) / Math.PI).toFixed(1)}° off`}.`;
    return false;
  }

  /** Step 3 of the pick: rotate the wrist until the tag sits square. */
  async function runRollStep(): Promise<boolean> {
    if (armPickBusy) return false;
    armPickBusy = true;
    try {
      return await alignByRoll(blockTag);
    } finally {
      jointHoldPose = null;
      handBackToManual();
      armPickBusy = false;
    }
  }

  /**
   * Step 4 — the final blind drop. By this point the tag is aimed, at a known
   * apparent size and squared up, so the rest of the distance is a fixed offset
   * rather than something to measure: the camera can no longer see the tag
   * usefully once the gripper is around it. x-y is held.
   */
  async function runFinalDrop() {
    if (armPickBusy || !solver) return;
    armPickBusy = true;
    try {
      handBackToManual(); // anchor on the current pose
      const [x, y, z] = target;
      armPickMsg = `Dropping the last ${(finalDropM * 1000).toFixed(0)} mm…`;
      target = [x, y, z - finalDropM];
      await settleAndMeasure();
      const now = solver.sitePosition();
      const drift = Math.hypot(now[0] - x, now[1] - y) * 1000;
      armPickMsg =
        `Down ${((z - now[2]) * 1000).toFixed(0)} mm · x-y drift ${drift.toFixed(0)} mm — ready to grip.`;
    } finally {
      handBackToManual();
      armPickBusy = false;
    }
  }

  /** Let go: open the jaws and leave the arm where it is. */
  async function openGripper() {
    if (armPickBusy) return;
    armPickBusy = true;
    try {
      armPickMsg = 'Opening the gripper…';
      // Eased open rather than snapped. Jumping straight to the open command lets
      // the solver resolve a large contact violation in one step, which flings
      // whatever was held across the room instead of setting it down.
      const from = gripperCmd;
      const to = Math.max(DEFAULT_PICK.gripperOpen, gripperRange[0]);
      for (let i = 1; i <= OPEN_STEPS; i++) {
        gripperCmd = from + ((to - from) * i) / OPEN_STEPS;
        await delay(OPEN_STEP_MS);
      }
      await delay(300); // let it settle where it was put down
      holdingItem = false;
      armPickMsg = 'Gripper open.';
    } finally {
      armPickBusy = false;
    }
  }
  const OPEN_STEPS = 14;
  const OPEN_STEP_MS = 55;

  /** Step 5 — close the gripper on the object. Nothing else moves. */
  async function runCloseGripper() {
    if (armPickBusy) return;
    armPickBusy = true;
    try {
      armPickMsg = 'Closing the gripper…';
      gripperCmd = Math.min(DEFAULT_PICK.gripperClose, gripperRange[1]);
      await delay(1500); // closes in place — no travel to wait on
      armPickMsg = 'Gripper closed.';
    } finally {
      armPickBusy = false;
    }
  }

  /**
   * The last step, from the hover pose: shift sideways onto the grasp line,
   * descend along the approach axis, close the jaws, lift.
   *
   * Down here means *along the gripper's approach axis*, not world −Z. The two
   * agree for a tag lying flat, and only the approach axis is right for one on a
   * slope or a vertical face — and by this point the gripper is pointing straight
   * at the tag, so it is the direction that actually closes the gap.
   *
   * The wrist's orientation is pinned for the whole step. Handing back to
   * position-only IK would let the wrist tilt to satisfy the descent, which is
   * precisely the alignment the approach spent its passes establishing.
   */
  async function runGraspStep(id: number = blockTag): Promise<boolean> {
    if (armPickBusy || !solver) return false;
    armPickBusy = true;
    try {
      handBackToManual(); // anchor: the target is where the gripper is right now
      holdRd = solver.siteRotation();

      // Open before descending. Nothing else in the sequence opens the jaws, so
      // after one pick they stay shut and every later descent drives closed jaws
      // into the block — which closes on nothing and reports a clean miss.
      if (gripperCmd > gripperRange[0] + 1e-3) {
        armPickMsg = 'Opening the jaws before descending…';
        gripperCmd = Math.max(DEFAULT_PICK.gripperOpen, gripperRange[0]);
        await delay(900);
      }

      const jaw = jawAxisWorld();
      if (jaw && graspLeftM !== 0) {
        armPickMsg = `Shifting ${(graspLeftM * 1000).toFixed(0)} mm onto the grasp line…`;
        target = [
          target[0] - jaw[0] * graspLeftM,
          target[1] - jaw[1] * graspLeftM,
          target[2] - jaw[2] * graspLeftM,
        ];
        await settleAndMeasure();
      }

      const st = solver.siteRotation();
      const ap = [st[0], st[3], st[6]]; // approach axis: the way the gripper points
      // Whatever the approach couldn't close, the descent makes up. Stopping short
      // of the hover point and then descending a fixed distance leaves the jaws
      // short by exactly that much — which is a miss, from a pick that had arrived.
      const drop = finalDropM + Math.min(approachShortfallM, GOOD_ENOUGH_M);
      armPickMsg =
        `Descending ${(drop * 1000).toFixed(0)} mm onto the block` +
        `${approachShortfallM > 0.001 ? ` (${(approachShortfallM * 1000).toFixed(0)} mm of it making up the approach)` : ''}…`;
      const beforeDrop = solver.sitePosition().slice();
      // Getting down onto the block matters more than holding the wrist perfectly
      // square; a few degrees of tilt still grips, stopping 50 mm short never does.
      holdOriWeight = 0.15;
      target = [target[0] + ap[0] * drop, target[1] + ap[1] * drop, target[2] + ap[2] * drop];
      await settleAndMeasure();
      // What was commanded and what happened are different questions at the edge
      // of the arm's envelope: a descent it cannot make looks identical in the log
      // to one it made, and leaves the jaws closing on air above the block.
      const afterDrop = solver.sitePosition();
      const moved = Math.hypot(
        afterDrop[0] - beforeDrop[0],
        afterDrop[1] - beforeDrop[1],
        afterDrop[2] - beforeDrop[2],
      );
      armPickMsg =
        `Descended ${(moved * 1000).toFixed(0)} mm of ${(drop * 1000).toFixed(0)} mm commanded ` +
        `(${(beforeDrop[2] * 1000).toFixed(0)} → ${(afterDrop[2] * 1000).toFixed(0)} mm high).`;
      await delay(250);

      armPickMsg = 'Closing the gripper…';
      gripperCmd = Math.min(DEFAULT_PICK.gripperClose, gripperRange[1]);
      await delay(1500); // closes in place — no travel to wait on

      const [x, y, z] = target;
      const blockBefore = blockHeightM();
      armPickMsg = `Lifting ${(liftM * 1000).toFixed(0)} mm…`;
      target = [x, y, z + liftM];
      await settleAndMeasure();
      const now = solver.sitePosition();
      const armRose = (now[2] - z) * 1000;
      const blockAfter = blockHeightM();
      if (blockBefore != null && blockAfter != null) {
        const blockRose = (blockAfter - blockBefore) * 1000;
        // Half the arm's travel is a generous bar — a block that is really held
        // rises with the gripper, while one that was nudged or missed does not.
        const held = blockRose > Math.max(10, armRose * 0.5);
        armPickMsg = held
          ? `Picked up tag ${id} — block up ${blockRose.toFixed(0)} mm (arm ${armRose.toFixed(0)} mm).`
          : `Missed tag ${id} — arm rose ${armRose.toFixed(0)} mm but the block only ${blockRose.toFixed(0)} mm.`;
        return held;
      }
      armPickMsg = `Lifted ${armRose.toFixed(0)} mm (no block to check).`;
      return true;
    } catch (err) {
      armPickMsg = `Grasp failed: ${err instanceof Error ? err.message : String(err)}`;
      console.error('runGraspStep', err);
      return false;
    } finally {
      holdOriWeight = 1;
      holdRd = null;
      handBackToManual();
      armPickBusy = false;
    }
  }

  /** Step 6 — lift straight up, holding x-y and the grip. */
  async function runLift() {
    if (armPickBusy || !solver) return;
    armPickBusy = true;
    try {
      handBackToManual();
      const [x, y, z] = target;
      armPickMsg = `Lifting ${(liftM * 1000).toFixed(0)} mm…`;
      target = [x, y, z + liftM];
      await settleAndMeasure();
      const now = solver.sitePosition();
      armPickMsg = `Lifted ${((now[2] - z) * 1000).toFixed(0)} mm.`;
    } finally {
      handBackToManual();
      armPickBusy = false;
    }
  }

  /**
   * Move to the board-view pose as a one-shot: ease up there, then hand control
   * straight back to the manual target. It's an action, not a mode — there's
   * nothing to un-toggle afterwards, and the arm just stays where it arrived.
   */
  async function raiseToView() {
    if (armPickBusy) return;
    armPickBusy = true;
    armPickMsg = 'Raising to the board-view pose…';
    try {
      armView = true;
      await settleAndMeasure();
    } finally {
      armView = false;
      handBackToManual(); // hold this pose; sliders take over from here
      armPickBusy = false;
      armPickMsg = 'At the board-view pose — the arm is yours.';
    }
  }

  // Wait until the arm stops moving (the speed limiter has caught up with its
  // target) rather than guessing a fixed delay — so the sequence stays correct at
  // any max speed. Returns how far the end-effector ended up from the commanded
  // target (m), so a caller can tell "arrived" from "gave up short".
  async function settleAndMeasure(timeoutMs = 20000): Promise<number> {
    const t0 = performance.now();
    let prev = jointAngles.slice();
    let still = 0;
    while (performance.now() - t0 < timeoutMs) {
      await delay(100);
      const now = jointAngles;
      const moved = now.some((a, i) => Math.abs(a - (prev[i] ?? 0)) > 1e-3);
      still = moved ? 0 : still + 1;
      prev = now.slice();
      if (still >= 2) break; // ~200 ms of no motion → settled
    }
    if (!solver) return Infinity;
    const ee = solver.sitePosition();
    return Math.hypot(ee[0] - target[0], ee[1] - target[1], ee[2] - target[2]);
  }

  // Printed side length (mm) of a marker by id, so its solvePnP depth is metric.
  function markerSizeFor(id: number): number {
    if (tagCentresMm.has(id)) return TAG_MM; // board border tag
    if (id >= NAV_TAG_LO && id <= NAV_TAG_HI) return navTagMm; // nav / shelf fiducial (200..250)
    if (id === navTagId) return navTagMm;
    if (BORDERED_IDS.has(id)) return BLOCK_MARKER_MM; // object/block tag
    return 30; // unknown tag — assume a middling size
  }

  /**
   * Make sure the grab canvas and OpenCV mats match the current frame source.
   * The rendered view and a real camera are different sizes, and switching
   * between them mid-session has to re-fit the buffers or solvePnP silently
   * works from the wrong image dimensions.
   */
  function ensureGrab(live: boolean): boolean {
    if (!cv) return false;
    const remote = remoteCams.canvas(REMOTE_BASE_CAM);
    const w = remote ? remote.width : live ? video.videoWidth || 640 : settings.camResW;
    const h = remote
      ? remote.height
      : live
        ? video.videoHeight || 480
        : Math.round((settings.camResW * 3) / 4);
    if (w < 8 || h < 8) return false;
    if (grabCtx && camW === w && camH === h) return true;
    camW = w;
    camH = h;
    const grab = grabCtx?.canvas ?? document.createElement('canvas');
    grab.width = w;
    grab.height = h;
    grabCtx = grab.getContext('2d', { willReadFrequently: true });
    try {
      srcMat?.delete();
      grayMat?.delete();
    } catch {
      /* ignore */
    }
    srcMat = new cv.Mat(h, w, cv.CV_8UC4);
    grayMat = new cv.Mat();
    return !!grabCtx;
  }

  function ensureArmGrab(live: boolean): boolean {
    if (!cv) return false;
    const remote = remoteCams.canvas(REMOTE_ARM_CAM);
    const w = remote ? remote.width : live ? armVideo.videoWidth || 640 : settings.camResW;
    const h = remote
      ? remote.height
      : live
        ? armVideo.videoHeight || 480
        : Math.round((settings.camResW * 3) / 4);
    if (w < 8 || h < 8) return false;
    if (armGrabCtx && armCamW === w && armCamH === h) return true;
    armCamW = w;
    armCamH = h;
    const grab = armGrabCtx?.canvas ?? document.createElement('canvas');
    grab.width = w;
    grab.height = h;
    armGrabCtx = grab.getContext('2d', { willReadFrequently: true });
    try {
      armSrcMat?.delete();
      armGrayMat?.delete();
    } catch {
      /* ignore */
    }
    armSrcMat = new cv.Mat(h, w, cv.CV_8UC4);
    armGrayMat = new cv.Mat();
    return !!armGrabCtx;
  }

  function maybeDetect() {
    if (!cv || !detector || !session) return;
    const remote = remoteCams.canvas(REMOTE_BASE_CAM);
    const live = !!camStream || !!remote;
    const source: CanvasImageSource | null = remote ?? (camStream ? video : (povCanvas ?? null));
    if (!source) return;
    if (!ensureGrab(live)) return;
    const intr = intrinsics ?? povIntrinsics(povView?.fovDeg ?? 60, camW, camH);
    if (!grabCtx || !srcMat || !grayMat) return;
    const now = performance.now();
    if (now - lastDetect < 100) return; // ~10 Hz
    lastDetect = now;

    const { width, height } = grabCtx.canvas;
    grabCtx.drawImage(source, 0, 0, width, height);
    const frame = grabCtx.getImageData(0, 0, width, height);
    srcMat.data.set(frame.data);
    cv.cvtColor(srcMat, grayMat, cv.COLOR_RGBA2GRAY);
    const allDetections = detectAllMarkers(cv, detector, grayMat);
    latestBaseDetections = allDetections;
    const corners = new Map<number, Corners>();
    for (const d of allDetections) corners.set(d.id, d.corners);
    const ids = allDetections.map((d) => d.id).sort((a, b) => a - b);
    if (ids.join(',') !== seenIds.join(',')) seenIds = ids;

    // Draw real-time OpenCV-style detection overlay on base camera
    if (baseCamOverlayCanvas) {
      const oCtx = baseCamOverlayCanvas.getContext('2d');
      if (oCtx) drawMarkerOverlays(oCtx, corners, width, height, allDetections);
    }
    // 3-D tag view: full camera-frame pose of every detected marker. While the arm
    // is raised to view the board, the arm camera owns the tag view instead.
    if (!armView) {
      const poses: TagPose[] = [];
      for (const [id, c] of corners) {
        const sizeMm = markerSizeFor(id);
        const mp = solvePnpMarkerPose(cv, c, sizeMm, camW, camH, intr);
        if (mp) {
          poses.push({ id, R: mp.R, t: mp.t, sizeMm });
          if (id < NAV_TAG_LO) {
            const cam = cameraWorld();
            const pc = [mp.t[0] / 1000, mp.t[1] / 1000, mp.t[2] / 1000];
            const R = cam.R;
            const world = [
              cam.t[0] + R[0] * pc[0] + R[1] * pc[1] + R[2] * pc[2],
              cam.t[1] + R[3] * pc[0] + R[4] * pc[1] + R[5] * pc[2],
              cam.t[2] + R[6] * pc[0] + R[7] * pc[1] + R[8] * pc[2],
            ];
            const rollRad = Math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0]);
            tagFix = { id, p: toRobot(world), sizeM: sizeMm / 1000, rollRad };
          }
        }
      }
      tagPoses = poses;
      tagsFromArm = false;
    }

    // Which nav fiducials (ids 200–250) are currently visible — offered as
    // one-click drive targets. Only reassign when the set changes (avoid churn).
    const seen = [...corners.keys()].filter((id) => id >= NAV_TAG_LO && id <= NAV_TAG_HI).sort((a, b) => a - b);
    if (seen.join(',') !== visibleNavTags.join(',')) visibleNavTags = seen;

    // Remember where each one is — accumulated during exploration (strictly IDs > 200)
    // Remember where each one is — accumulated live or during exploration (strictly IDs > 200)
    // Always pair unique station tag with companion anchor 200 placed to its RIGHT
    const pairs = findLabeledTagPairs(allDetections);
    for (const pair of pairs) {
      const id = pair.stationId;
      const cLeft = pair.left;
      const cRight = pair.right;
      const mpL = solvePnpMarkerPose(cv, cLeft, navTagMm, camW, camH, intr);
      const mpR = solvePnpMarkerPose(cv, cRight, navTagMm, camW, camH, intr);
      if (!mpL) continue;
      const cam = cameraWorld();
      let pw: [number, number, number];
      let nw: [number, number, number];
      if (mpR) {
        // Precise geometric midpoint between unique station tag and companion anchor 200
        const midCam: [number, number, number] = [
          (mpL.t[0] + mpR.t[0]) / 2000,
          (mpL.t[1] + mpR.t[1]) / 2000,
          (mpL.t[2] + mpR.t[2]) / 2000,
        ];
        pw = apply3(cam.R, midCam);
        nw = apply3(cam.R, [(mpL.R[2] + mpR.R[2]) / 2, (mpL.R[5] + mpR.R[5]) / 2, (mpL.R[8] + mpR.R[8]) / 2]);
      } else {
        pw = apply3(cam.R, [mpL.t[0] / 1000 + 0.055, mpL.t[1] / 1000, mpL.t[2] / 1000]);
        nw = apply3(cam.R, [mpL.R[2], mpL.R[5], mpL.R[8]]);
      }
      rememberTag(id, [pw[0] + cam.t[0], pw[1] + cam.t[1], pw[2] + cam.t[2]], nw);
    }

    // Handle standalone boundary stop tag (240)
    if (corners.has(BOUNDARY_TAG_ID)) {
      const cBound = corners.get(BOUNDARY_TAG_ID)!;
      const mpB = solvePnpMarkerPose(cv, cBound, navTagMm, camW, camH, intr);
      if (mpB) {
        const cam = cameraWorld();
        const pw = apply3(cam.R, [mpB.t[0] / 1000, mpB.t[1] / 1000, mpB.t[2] / 1000]);
        const nw = apply3(cam.R, [mpB.R[2], mpB.R[5], mpB.R[8]]);
        rememberTag(BOUNDARY_TAG_ID, [pw[0] + cam.t[0], pw[1] + cam.t[1], pw[2] + cam.t[2]], nw);
      }
    }

    latestCorners = corners;

    // ── Labeled station card OCR & tracking: [Unique Left Tag ID] + [Anchor Right Tag 200] ──
    const labeledPairs = findLabeledTagPairs(allDetections);
    let bestPair: LabeledTagPair | null = null;
    let minCenterDist = Infinity;
    for (const p of labeledPairs) {
      const midX = p.centerPx ? p.centerPx[0] : (p.leftCenter[0] + p.rightCenter[0]) / 2;
      const hasLabel = !!knownTags.get(p.stationId)?.label;
      const effectiveDist = Math.abs(midX - camW / 2) + (hasLabel ? 120 : 0);
      if (effectiveDist < minCenterDist) {
        minCenterDist = effectiveDist;
        bestPair = p;
      }
    }
    const activePair = (navTagId ? labeledPairs.find((p) => p.stationId === navTagId) : null) || bestPair || labeledPairs[0];
    const leftTag = activePair?.left;
    const rightTag = activePair?.right;
    const activeStationId = activePair?.stationId;
    const baseMovingFast = navigating && (Math.abs(baseVel.fwd) > 0.1 || Math.abs(baseVel.turn) > 0.1);
    if (
      leftTag &&
      rightTag &&
      isTagPairReadyForOcr(leftTag, rightTag, camW, camH) &&
      !labeledOcrBusy &&
      !baseMovingFast
    ) {
      const now2 = performance.now();
      if (!lastLabeledOcr || now2 - lastLabeledOcr.ts > 1200) {
        labeledOcrBusy = true;
        appendLog(`[Scan] Station card #${activeStationId ?? 201} detected in frame. Extracting homography warp...`);
        extractLabeledTag(cv!, corners, grayMat!, activeStationId).then((info) => {
          labeledOcrBusy = false;
          if (!info) {
            appendLog('[Scan] Homography extraction returned null.');
            return;
          }
          lastOcrFullImgUrl = info.fullDataUrl || '';
          lastOcrLabelImgUrl = info.labelDataUrl || '';
          lastOcrDescImgUrl = info.descDataUrl || '';
          lastLabeledOcr = { label: info.label, ts: performance.now() };
          lastOcrText = info.label;
          lastOcrDesc = info.description;

          if (info.label) {
            appendLog(`[OCR Success] Station #${info.stationId} Label: "${info.label}" (${info.confidence}%), Desc: "${info.description}"`);
          } else {
            appendLog(`[OCR Raw] Station #${info.stationId}: "${info.rawLabelText || '(empty)'}", Desc: "${info.rawDescText || '(empty)'}"`);
          }

          console.log(`[Tesseract OCR] Decoded Station #${info.stationId} label: "${info.label}" (${info.description})`);

          // Match station in 3D space and update knownTags directly by unique station ID
          const targetStationId = info.stationId ?? activeStationId;
          if (targetStationId !== undefined && targetStationId !== null) {
            const mp = solvePnpMarkerPose(cv, leftTag, navTagMm, camW, camH, intr);
            const cam = cameraWorld();
            let pw = [0, 0, 0];
            let nw = [0, 0, 1];
            if (mp) {
              const pLocal = apply3(cam.R, [mp.t[0] / 1000, mp.t[1] / 1000, mp.t[2] / 1000]);
              pw = [pLocal[0] + cam.t[0], pLocal[1] + cam.t[1], pLocal[2] + cam.t[2]];
              nw = apply3(cam.R, [mp.R[2], mp.R[5], mp.R[8]]);
            }

            const existing = knownTags.get(targetStationId);
            if (existing) {
              const next = new Map(knownTags);
              next.set(targetStationId, { ...existing, label: info.label, description: info.description });
              knownTags = new Map([...next].sort((a, b) => a[1].y - b[1].y));
              appendLog(`[Places Updated] Station #${targetStationId} assigned label "${info.label}"`);
            } else if (mp) {
              rememberTag(targetStationId, pw, nw, info.label, info.description);
              appendLog(`[Places Discovered] Station #${targetStationId} registered with label "${info.label}"`);
            }
          }
        }).catch((err) => {
          console.warn('[Tesseract OCR] Error:', err);
          appendLog(`[OCR Error] ${err}`);
          labeledOcrBusy = false;
        });
      }
    }

    // ── Onboard-camera dual nav tag card tracking ──
    let cardCamPose: [number, number, number, number] | null = null;
    if (leftTag && rightTag) {
      const mpL = solvePnpMarkerPose(cv, leftTag, navTagMm, camW, camH, intr);
      const mpR = solvePnpMarkerPose(cv, rightTag, navTagMm, camW, camH, intr);
      if (mpL && mpR) {
        const sqL = Math.atan2(mpL.R[2], -mpL.R[8]);
        const sqR = Math.atan2(mpR.R[2], -mpR.R[8]);
        cardCamPose = [
          (mpL.t[0] + mpR.t[0]) / 2000,
          (mpL.t[1] + mpR.t[1]) / 2000,
          (mpL.t[2] + mpR.t[2]) / 2000,
          (sqL + sqR) / 2,
        ];
      }
    }
    shelfTagCam = cardCamPose;

    // Board pose (camera→board) from border tags, then each block via solvePnP.
    const pose = boardPoseFromTags(cv, corners, tagCentresMm, TAG_MM, intr, camW, camH);
    const m = new Map<number, [number, number, number]>();
    if (pose) {
      for (const [id, c] of corners) {
        if (tagCentresMm.has(id) || !BORDERED_IDS.has(id)) continue;
        const b = blockBoardXY(cv, c, BLOCK_MARKER_MM, intr, camW, camH, pose, [tagOffX, tagOffY]);
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

  let latestCorners = new Map<number, Corners>();

  async function forceScanOcr(targetStationId?: number): Promise<LabeledTagInfo | null> {
    return new Promise((resolve) => {
      if (!cv) {
        appendLog('[Scan] OpenCV not loaded yet.');
        resolve(null);
        return;
      }
      const remote = remoteCams.canvas(REMOTE_BASE_CAM);
      const live = !!camStream || !!remote;
      const source: CanvasImageSource | null = remote ?? (camStream ? video : (povCanvas ?? null));
      if (!source) {
        appendLog('[Scan] No active camera image source.');
        resolve(null);
        return;
      }
      if (!ensureGrab(live) || !grabCtx || !srcMat || !grayMat) {
        appendLog('[Scan] Frame grab buffer not ready.');
        resolve(null);
        return;
      }
      const { width, height } = grabCtx.canvas;
      grabCtx.drawImage(source, 0, 0, width, height);
      const frame = grabCtx.getImageData(0, 0, width, height);
      srcMat.data.set(frame.data);
      cv.cvtColor(srcMat, grayMat, cv.COLOR_RGBA2GRAY);
      const allDetections = detectAllMarkers(cv, detector, grayMat);
      const corners = new Map<number, Corners>();
      for (const d of allDetections) corners.set(d.id, d.corners);
      latestCorners = corners;
      const ids = allDetections.map((d) => d.id).sort((a, b) => a - b);
      const labeledPairs = findLabeledTagPairs(corners);
      const activePair = (targetStationId ? labeledPairs.find((p) => p.stationId === targetStationId) : null) || labeledPairs[0];
      const leftTag = activePair?.left || (targetStationId ? corners.get(targetStationId) : null);
      const rightTag = activePair?.right || corners.get(RIGHT_NAV_ID);
      const stationId = activePair?.stationId ?? targetStationId;
      appendLog(`[Scan] Photo (${width}x${height}) captured. Detected markers: [${ids.join(', ')}]`);
      if (!leftTag || !rightTag) {
        appendLog(`[Scan] Cannot extract: Missing ${!leftTag ? `Station Tag #${stationId ?? 'Left'}` : ''} ${!rightTag ? 'Anchor Tag 200' : ''}.`);
        resolve(null);
        return;
      }
      labeledOcrBusy = true;
      extractLabeledTag(cv, corners, grayMat, stationId).then((info) => {
        labeledOcrBusy = false;
        if (!info) {
          appendLog('[Scan] Homography extraction returned null.');
          resolve(null);
          return;
        }
        lastOcrFullImgUrl = info.fullDataUrl || '';
        lastOcrLabelImgUrl = info.labelDataUrl || '';
        lastOcrDescImgUrl = info.descDataUrl || '';
        lastLabeledOcr = { label: info.label, ts: performance.now() };
        lastOcrText = info.label;
        lastOcrDesc = info.description;
        if (info.label) {
          appendLog(`[OCR Success] Station #${info.stationId} Label: "${info.label}" (${info.confidence}%), Desc: "${info.description}"`);
        } else {
          appendLog(`[OCR Raw] Station #${info.stationId}: "${info.rawLabelText || '(empty)'}", Desc: "${info.rawDescText || '(empty)'}"`);
        }
        const targetStationId = info.stationId ?? stationId;
        if (targetStationId !== undefined && targetStationId !== null) {
          const curIntr = live ? intrinsics : (povView ? povIntrinsics(povView.fovDeg, camW, camH) : null);
          let pw = [0, 0, 0];
          let nw = [0, 0, 1];
          let mp: MarkerPose | null = null;
          if (curIntr && leftTag) {
            mp = solvePnpMarkerPose(cv, leftTag, navTagMm, camW, camH, curIntr);
            if (mp) {
              const cam = cameraWorld();
              const pLocal = apply3(cam.R, [mp.t[0] / 1000, mp.t[1] / 1000, mp.t[2] / 1000]);
              pw = [pLocal[0] + cam.t[0], pLocal[1] + cam.t[1], pLocal[2] + cam.t[2]];
              nw = apply3(cam.R, [mp.R[2], mp.R[5], mp.R[8]]);
            }
          }
          const existing = knownTags.get(targetStationId);
          if (existing) {
            const next = new Map(knownTags);
            next.set(targetStationId, { ...existing, label: info.label, description: info.description });
            knownTags = new Map([...next].sort((a, b) => a[1].y - b[1].y));
            appendLog(`[Places Updated] Station #${targetStationId} assigned label "${info.label}"`);
          } else if (mp) {
            rememberTag(targetStationId, pw, nw, info.label, info.description);
            appendLog(`[Places Discovered] Station #${targetStationId} registered with label "${info.label}"`);
          }
        }
        resolve(info);
      }).catch((err) => {
        console.warn('[Tesseract OCR] Manual error:', err);
        appendLog(`[OCR Error] ${err}`);
        labeledOcrBusy = false;
        resolve(null);
      });
    });
  }

  function onStageChange(next: Stage, useScanned = false) {
    detectMsg = null;
    if (next === '') {
      handBackToManual();
      stage = '';
      pickPhase = 'idle';
      return;
    }
    // Arm-camera routine already set pickBlock from its scan — keep it.
    if (useScanned) {
      if (!pickBlock) {
        detectMsg = 'No scanned tag position.';
        return;
      }
      stage = next;
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

  async function afterConnect() {
    try {
      await seedSimFromRobot(); // match the sim & IK target to the arm's live physical pose FIRST
    } catch (e) {
      console.warn('[afterConnect] seedSimFromRobot warning:', e);
    }
    realConnected = true; // only start streaming servo commands after poses are synced
    try {
      await robot.setTorque(
        CALIBRATION_PLAN.map((j) => j.servoId),
        true,
      );
      await autoDetectWheels(); // a >6-servo chain means a mobile base is attached
    } catch (e) {
      console.warn('[afterConnect] servo torque/wheel detection warning:', e);
    }
    saveAutoConnect(true);
  }

  async function connectRobotLocal() {
    realError = null;
    try {
      await robot.connectLocal();
      await afterConnect();
    } catch (e) {
      realError = e instanceof Error ? e.message : String(e);
    }
  }

  let closeVideoWs: (() => void) | null = null;

  async function connectRobotRemote(opts: import('./lib/remoteRobot').RemoteRobotOptions) {
    realError = null;
    try {
      console.log('[Remote] Calling robot.connectRemote...');
      await robot.connectRemote(opts);
      console.log('[Remote] robot.connectRemote finished!');
      remoteCams.clear();
      activeRemoteCam = REMOTE_BASE_CAM;
      remoteBaseLive = true;
      remoteArmLive = false;
      robot.onVideoFrame((cam, jpeg) => void remoteCams.push(cam, jpeg));

      // Connect to Cloudflare Edge Video Relay for zero-freeze streaming
      closeVideoWs?.();
      const wsUrl = `wss://lekiwi-video-relay.theadityashankar.workers.dev/?room=${encodeURIComponent(opts.room)}&role=browser`;
      closeVideoWs = remoteCams.connectWebSocket(wsUrl);

      console.log('[Remote] Calling afterConnect...');
      await afterConnect();
      console.log('[Remote] afterConnect finished! realConnected =', realConnected);
    } catch (e) {
      realError = e instanceof Error ? e.message : String(e);
      console.error('[Remote] connectRobotRemote failed:', realError);
    }
  }

  // Let the header's Connect button drive the same logic, and mirror the state
  // back so it can show connected / error without owning any of it.
  armLink.connectLocal = connectRobotLocal;
  armLink.connectRemote = connectRobotRemote;
  armLink.disconnect = disconnectRobot;
  $effect(() => {
    armLink.connected = realConnected;
    armLink.error = realError;
  });

  // Read the arm's live servo positions and set the simulator (and EE target) to
  // match, so connecting doesn't yank the real arm toward a stale target. Needs a
  // joint calibration to map servo ticks → sim radians; skipped otherwise.
  async function seedSimFromRobot() {
    if (!session || !solver || !renderer || !jointCal) return;
    try {
      const ids = CALIBRATION_PLAN.map((j) => j.servoId);
      const pos = await robot.syncReadPositions(ids);
      const q = session.data.qpos as Float64Array;
      CALIBRATION_PLAN.forEach((def, i) => {
        const raw = pos.get(def.servoId);
        const cal = jointCal![def.joint];
        if (raw == null || !cal) return;
        const sim = servoToSimRad(raw, cal);
        if (sim == null) return;
        if (i < 5) q[i] = sim;
        else gripperCmd = sim;
      });
      resetPoseLimiter(); // slew restarts from this freshly-read pose
      session.forward();
      jointAngles = ARM_DOFS.map((idx) => q[idx]);
      targetToCurrentEE(); // EE target = where the arm actually is now
    } catch (e) {
      realError = e instanceof Error ? e.message : String(e);
    }
  }

  /**
   * Closed-loop alignment to a visual station group (Station Tag #stationId + Anchor Tag #200).
   * 1. Optical Rotation Alignment: Compares the outer-left edge height of the left tag against
   *    the outer-right edge height of the right tag. If one is larger, it rotates until heights match
   *    (guaranteeing the camera is perpendicular to the card).
   * 2. Lateral Centering: Drives laterally until the group midpoint is centered in the camera (320px).
   * 3. Standoff Distance: Adjusts forward/backward to settle at the exact scan standoff distance.
   */
  async function alignToGroup(
    stationId: number,
    targetRatio = settings.tagCameraRatio,
    timeoutMs = 7000,
  ): Promise<boolean> {
    const t0 = performance.now();
    const camCenter = 320;
    const clamp1 = (v: number) => Math.max(-1, Math.min(1, v));
    let stableCount = 0;
    while (performance.now() - t0 < timeoutMs) {
      if (exploreCancel || navCancel) return false;
      const rawDetections = latestBaseDetections.length > 0 ? latestBaseDetections : Array.from(latestCorners.entries()).map(([id, corners]) => ({ id, corners }));
      const pairs = findLabeledTagPairs(rawDetections);
      const targetPair = pairs.find((p) => p.stationId === stationId) || (pairs.length === 1 ? pairs[0] : null);
      const leftC = targetPair?.left || latestCorners.get(stationId);
      const rightC = targetPair?.right || latestCorners.get(RIGHT_NAV_ID);
      let latErrPx = 0;
      let yawErr = 0;
      let curRatio = computeMaxTagRatio(latestCorners);
      let pairVisible = false;
      if (targetPair && targetPair.left && targetPair.right) {
        pairVisible = true;
        const groupMidX = targetPair.centerPx ? targetPair.centerPx[0] : (targetPair.leftCenter[0] + targetPair.rightCenter[0]) / 2;
        latErrPx = groupMidX - camCenter;
        const hLeftOuter = Math.hypot(targetPair.left[3][0] - targetPair.left[0][0], targetPair.left[3][1] - targetPair.left[0][1]);
        const hRightOuter = Math.hypot(targetPair.right[2][0] - targetPair.right[1][0], targetPair.right[2][1] - targetPair.right[1][1]);
        const avgHeight = (hLeftOuter + hRightOuter) / 2;
        if (avgHeight > 0) yawErr = (hLeftOuter - hRightOuter) / avgHeight;
      } else if (leftC) {
        const leftMidX = (leftC[0][0] + leftC[1][0] + leftC[2][0] + leftC[3][0]) / 4;
        const markerW = Math.hypot(leftC[1][0] - leftC[0][0], leftC[1][1] - leftC[0][1]);
        latErrPx = leftMidX + markerW * 0.75 - camCenter;
      } else if (rightC) {
        const rightMidX = (rightC[0][0] + rightC[1][0] + rightC[2][0] + rightC[3][0]) / 4;
        const markerW = Math.hypot(rightC[1][0] - rightC[0][0], rightC[1][1] - rightC[0][1]);
        latErrPx = rightMidX - markerW * 0.75 - camCenter;
      }
      const ratioErr = targetRatio > 0 && curRatio > 0 ? (targetRatio - curRatio) / targetRatio : 0;
      const isLatGood = Math.abs(latErrPx) <= 4.0;
      const isYawGood = Math.abs(yawErr) <= 0.035;
      const isDistGood = Math.abs(ratioErr) <= 0.12;
      if (pairVisible && isLatGood && isYawGood && isDistGood) {
        stableCount += 1;
        if (stableCount >= 3) {
          zeroBaseVel();
          return true;
        }
      } else {
        stableCount = 0;
      }

      // Proportional control outputs:
      // Lateral correction (vy)
      const vyNorm = latErrPx / camCenter;
      const vy = latErrPx !== 0 ? (latErrPx > 0 ? -Math.max(0.06, Math.min(0.35, Math.abs(vyNorm) * 0.8)) : Math.max(0.06, Math.min(0.35, Math.abs(vyNorm) * 0.8))) : 0;

      // Standoff distance correction (vx)
      let vx = 0;
      if (curRatio > 0) {
        if (curRatio < targetRatio * 0.88) vx = Math.max(0.08, Math.min(0.20, ratioErr * 0.35));
        else if (curRatio > targetRatio * 1.12) vx = -Math.max(0.08, Math.min(0.20, Math.abs(ratioErr) * 0.35));
      }

      // Turn correction
      const turn = clamp1(1.6 * yawErr);

      const p = bodyToPrimitives(vx, vy);
      baseVel.fwd = p.fwd;
      baseVel.bl = p.bl;
      baseVel.br = p.br;
      baseVel.turn = turn;

      await delay(40);
    }

    zeroBaseVel();
    return false;
  }

  // Probe the bus for the wheel servos; if the chain has them (>6 servos), switch
  // them to wheel mode automatically — no separate "connect wheels" step.
  async function autoDetectWheels() {
    baseLink.error = null;
    try {
      const ids = [...baseLink.config.wheelIds];
      const pos = await robot.syncReadPositions(ids);
      if (ids.every((id) => pos.has(id))) {
        await robot.setWheelMode(ids);
        baseLink.connected = true;
      }
    } catch {
      /* no wheels on this chain — arm-only robot */
    }
  }

  async function disconnectRobot() {
    saveAutoConnect(false);
    closeVideoWs?.();
    closeVideoWs = null;
    if (baseLink.connected) await disconnectBase().catch(() => {});
    try {
      await robot.setTorque(CALIBRATION_PLAN.map((j) => j.servoId), false);
    } catch {
      /* ignore */
    }
    await robot.disconnect().catch(() => {});
    realConnected = false;
  }

  // ── Base motors ──────────────────────────────────────────────────────────────
  async function disconnectBase() {
    try {
      await baseLink.stopWheels();
    } catch {
      /* ignore */
    }
    baseLink.connected = false;
  }

  // Drive toward the shelf tag using the onboard camera. First rotate to FACE the
  // tag (turn until it's centred ahead), then drive forward to the standoff while
  // keeping it centred — so the robot orients itself before/while approaching.
  // Sets the base velocity (streamed by maybeStreamBase); the sim pose is left
  // untouched — use "Reset MuJoCo position" to snap it once parked.
  let navCancel = false;

  function toggleNavigate() {
    baseLink.error = null;
    navigating = !navigating;
    if (!navigating) {
      navCancel = true;
      zeroBaseVel();
      baseLink.stopWheels().catch(() => {});
    }
  }

  /**
   * 3-Step Station Navigation & Docking:
   * 1. Reverses straight backward into the transit corridor if already parked/docked at a station (never turns away).
   * 2. Strafes laterally and performs closed-loop group alignment (centering and squaring yaw).
   * 3. Once rotation and position are verified correct, advances forward to the pickup docking distance.
   */
  async function driveToTag(targetId: number) {
    if (!hasBase) return;
    if (navigating) {
      navCancel = true;
      navigating = false;
      await delay(120);
    }
    navigating = true;
    navCancel = false;
    navTagId = targetId;
    baseLink.error = null;

    const targetStation = knownTags.get(targetId) || STATIONS.find((s) => s.navTag === targetId);
    appendLog(`[Navigation] Initiating lateral docking sequence to Station #${targetId}...`);

    try {
      const clamp1 = (v: number) => Math.max(-1, Math.min(1, v));
      const norm = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
      let alignedYawRad = (robotYawDeg * Math.PI) / 180; // Dynamically tracking true perpendicular heading

      // ── Step 1: Establish Standardized Scan Standoff Baseline ────────────
      // To ensure that the 8.5s forward docking distance is 100% identical on the
      // 1st, 2nd, and all subsequent clicks, the robot regulates its standoff to
      // the exact targetScanRatio baseline before any lateral strafe or docking:
      // - If docked / too close (or tags out of view): reverse straight back.
      // - If too far: advance forward to the standard scan line.
      appendLog(`[Navigation] Step 1: Establishing standardized scan standoff baseline...`);
      const backupT0 = performance.now();
      let standoffReached = false;
      const targetScanRatio = settings.tagCameraRatio; // configured tag side-size ratio (e.g. 0.05 / 32px)
      const ratioMin = targetScanRatio * 0.92;
      const ratioMax = targetScanRatio * 1.08;

      while (navigating && !navCancel && !standoffReached && performance.now() - backupT0 < 15000) {
        const rawDetections =
          latestBaseDetections.length > 0
            ? latestBaseDetections
            : Array.from(latestCorners.entries()).map(([id, corners]) => ({ id, corners }));
        const visibleCount = rawDetections.length;
        const ratio = computeMaxTagRatio(latestCorners);
        const pairs = findLabeledTagPairs(rawDetections);
        const targetPair = pairs.find((p) => p.stationId === targetId) || pairs[0];

        let yawErr = 0;
        if (targetPair && targetPair.left && targetPair.right) {
          const hLeftOuter = Math.hypot(targetPair.left[3][0] - targetPair.left[0][0], targetPair.left[3][1] - targetPair.left[0][1]);
          const hRightOuter = Math.hypot(targetPair.right[2][0] - targetPair.right[1][0], targetPair.right[2][1] - targetPair.right[1][1]);
          const avgHeight = (hLeftOuter + hRightOuter) / 2;
          if (avgHeight > 0) yawErr = (hLeftOuter - hRightOuter) / avgHeight;
          alignedYawRad = (robotYawDeg * Math.PI) / 180;
        } else {
          const yaw = (robotYawDeg * Math.PI) / 180;
          yawErr = norm(alignedYawRad - yaw);
        }

        if (visibleCount === 0) {
          // No tag visible (too close or out of FOV): reverse straight back to expand camera FOV
          const p = bodyToPrimitives(-0.22, 0);
          baseVel.fwd = p.fwd;
          baseVel.bl = p.bl;
          baseVel.br = p.br;
          baseVel.turn = clamp1(1.5 * yawErr);
        } else if (ratio > ratioMax) {
          // Closer than Scan Standoff: reverse back to baseline
          const normErr = (ratio - targetScanRatio) / targetScanRatio;
          const speed = Math.max(0.12, Math.min(0.25, normErr * 0.3));
          const p = bodyToPrimitives(-speed, 0);
          baseVel.fwd = p.fwd;
          baseVel.bl = p.bl;
          baseVel.br = p.br;
          baseVel.turn = clamp1(1.5 * yawErr);
        } else if (ratio < ratioMin) {
          // Farther than Scan Standoff: advance forward to baseline
          const normErr = (targetScanRatio - ratio) / targetScanRatio;
          const speed = Math.max(0.12, Math.min(0.22, normErr * 0.3));
          const p = bodyToPrimitives(speed, 0);
          baseVel.fwd = p.fwd;
          baseVel.bl = p.bl;
          baseVel.br = p.br;
          baseVel.turn = clamp1(1.5 * yawErr);
        } else {
          // Exact Scan Standoff baseline reached!
          standoffReached = true;
          zeroBaseVel();
          appendLog(`[Navigation] Standardized scan baseline reached (${(ratio * 100).toFixed(1)}% tag ratio).`);
          break;
        }

        await delay(40);
      }

      zeroBaseVel();
      await delay(150);
      atTag = null;
      if (!navigating || navCancel) return;

      // ── Step 2: Lateral Strafe & Group Position + Rotation Alignment ─────
      appendLog(`[Navigation] Step 2: Strafing & aligning position and rotation to Station #${targetId} group...`);
      const strafeT0 = performance.now();

      while (navigating && !navCancel && performance.now() - strafeT0 < 25000) {
        const rawDetections =
          latestBaseDetections.length > 0
            ? latestBaseDetections
            : Array.from(latestCorners.entries()).map(([id, corners]) => ({ id, corners }));
        const pairs = findLabeledTagPairs(rawDetections);
        const targetPair = pairs.find((p) => p.stationId === targetId);

        if (targetPair) {
          // When group is visible, execute precision closed-loop position & rotation alignment
          const aligned = await alignToGroup(targetId, targetScanRatio, 8000);
          if (aligned) {
            alignedYawRad = (robotYawDeg * Math.PI) / 180;
            appendLog(`[Navigation] Station #${targetId} position, rotation (${robotYawDeg.toFixed(1)}°), and distance fully aligned.`);
            break;
          }
        } else {
          // Strafe laterally towards station's estimated Y coordinate
          const targetY = targetStation?.y ?? 0;
          const dy = targetY - robotY;
          const vy = dy > 0 ? 0.35 : -0.35;
          const yaw = (robotYawDeg * Math.PI) / 180;
          const yawErr = norm(alignedYawRad - yaw);
          const p = bodyToPrimitives(0, vy);
          baseVel.fwd = p.fwd;
          baseVel.bl = p.bl;
          baseVel.br = p.br;
          baseVel.turn = clamp1(1.5 * yawErr);
        }

        await delay(40);
      }

      zeroBaseVel();
      await delay(200);
      if (!navigating || navCancel) return;

      // ── Step 3: Forward Docking Advance for User-Configured Seconds ───────
      const targetDurationSec = settings.navWorkForwardSec ?? 8.5;
      const targetDurationMs = targetDurationSec * 1000;

      if (targetDurationMs > 0) {
        appendLog(`[Navigation] Step 3: Advancing forward for ${targetDurationSec.toFixed(2)}s to dock at Station #${targetId}...`);
        const dockT0 = performance.now();
        const camCenter = 320;

        while (navigating && !navCancel && performance.now() - dockT0 < targetDurationMs) {
          const rawDetections =
            latestBaseDetections.length > 0
              ? latestBaseDetections
              : Array.from(latestCorners.entries()).map(([id, corners]) => ({ id, corners }));
          const pairs = findLabeledTagPairs(rawDetections);
          const targetPair = pairs.find((p) => p.stationId === targetId);

          let latCorrection = 0;
          let yawErr = 0;

          if (targetPair && targetPair.left && targetPair.right) {
            const groupMidX = targetPair.centerPx ? targetPair.centerPx[0] : (targetPair.leftCenter[0] + targetPair.rightCenter[0]) / 2;
            const diffPx = groupMidX - camCenter;
            latCorrection = diffPx > 0 ? -Math.min(0.2, (diffPx / camCenter) * 0.5) : Math.min(0.2, Math.abs(diffPx / camCenter) * 0.5);

            // Maintain continuous optical perpendicular rotation throughout forward advance
            const hLeftOuter = Math.hypot(targetPair.left[3][0] - targetPair.left[0][0], targetPair.left[3][1] - targetPair.left[0][1]);
            const hRightOuter = Math.hypot(targetPair.right[2][0] - targetPair.right[1][0], targetPair.right[2][1] - targetPair.right[1][1]);
            const avgHeight = (hLeftOuter + hRightOuter) / 2;
            if (avgHeight > 0) yawErr = (hLeftOuter - hRightOuter) / avgHeight;
            alignedYawRad = (robotYawDeg * Math.PI) / 180;

            // Optional emergency tag size cutoff
            const hLeft = Math.hypot(targetPair.left[2][1] - targetPair.left[1][1], targetPair.left[2][0] - targetPair.left[1][0]);
            const hRight = Math.hypot(targetPair.right[2][1] - targetPair.right[1][1], targetPair.right[2][0] - targetPair.right[1][0]);
            const avgH = (hLeft + hRight) / 2;
            if (avgH >= settings.maxTagSizePx) {
              appendLog(`[Navigation] Max tag size cutoff reached (${avgH.toFixed(0)}px).`);
              break;
            }
          } else {
            const yaw = (robotYawDeg * Math.PI) / 180;
            yawErr = norm(alignedYawRad - yaw);
          }

          // Drive forward at docking speed with continuous optical centering & rotation lock
          const p = bodyToPrimitives(0.20, latCorrection);
          baseVel.fwd = p.fwd;
          baseVel.bl = p.bl;
          baseVel.br = p.br;
          baseVel.turn = clamp1(1.5 * yawErr);

          await delay(40);
        }
      } else {
        appendLog(`[Navigation] Docking forward time is 0.0s — parked at visual centering lock.`);
      }

      zeroBaseVel();
      atTag = targetId;
      appendLog(`[Navigation] Arrived and docked at Station #${targetId}. Ready for arm pick operations.`);
      // Automatically look for items once navigated to the location
      if (!navCancel && !armPickBusy) {
        appendLog(`[Navigation] Automatically looking for items at Station #${targetId}...`);
        lookForItems().catch((err) => {
          console.error('Auto lookForItems error:', err);
        });
      }
    } finally {
      zeroBaseVel();
      baseLink.stopWheels().catch(() => {});
      navigating = false;
      navTagId = null;
    }
  }

  /**
   * Remember where a nav tag is, in world coordinates. Called whenever one is seen
   * — during Explore, or just because the robot happens to be facing it.
   */
  function rememberTag(id: number, pWorld: number[], nWorld: number[], label?: string, description?: string) {
    if (id <= 200 || id > NAV_TAG_HI || id === BOUNDARY_TAG_ID) return; // only station nav fiducials > 200 (exclude boundary tag 250)
    const next = new Map(knownTags);
    const existing = next.get(id);

    // Resolved label: ONLY set from real OCR text (no hardcoded sim values)
    const resolvedLabel = label || existing?.label || '';
    const resolvedDesc = description || existing?.description || '';

    next.set(id, {
      id,
      x: pWorld[0],
      y: pWorld[1],
      z: pWorld[2],
      faceYaw: Math.atan2(nWorld[1], nWorld[0]),
      label: resolvedLabel,
      description: resolvedDesc,
    });
    // Order places along the straight-line rack by Y coordinate (left-to-right)
    knownTags = new Map([...next].sort((a, b) => a[1].y - b[1].y));
  }

  function forgetTags() {
    knownTags = new Map();
    atTag = null;
    hasExplored = false;
  }

  /** Human-readable "where is this place" for a remembered tag. */
  function placeBearing(t: KnownTag): string {
    const bx = hasBase ? robotX : 0, by = hasBase ? robotY : 0;
    const dx = t.x - bx, dy = t.y - by;
    void t.faceYaw;
    const dist = Math.hypot(dx, dy);
    const rel = ((Math.atan2(dy, dx) * 180) / Math.PI - robotYawDeg + 540) % 360 - 180;
    const dir =
      Math.abs(rel) < 25 ? 'ahead'
      : Math.abs(rel) > 155 ? 'behind'
      : rel > 0 ? `${Math.round(rel)}° left`
      : `${Math.round(-rel)}° right`;
    return `${dist.toFixed(2)} m · ${dir}`;
  }

  /** Summarize markers seen by the base camera along with their live distance. */
  function baseCamMarkerSummary(): string {
    if (!seenIds.length) return 'no markers';
    const parts: string[] = [];
    for (const id of seenIds) {
      if (!tagsFromArm && tagPoses.length) {
        const p = tagPoses.find((tp) => tp.id === id);
        if (p) {
          const d = Math.hypot(p.t[0], p.t[1], p.t[2]) / 1000;
          parts.push(`Tag ${id} (${d.toFixed(2)}m)`);
          continue;
        }
      }
      if (shelfTagCam && id === navTagId) {
        const d = Math.hypot(shelfTagCam[0], shelfTagCam[1], shelfTagCam[2]);
        parts.push(`Tag ${id} (${d.toFixed(2)}m)`);
        continue;
      }
      parts.push(`Tag ${id}`);
    }
    return parts.join(', ');
  }

  async function returnToPose(gx: number, gy: number, wantYawDeg: number): Promise<void> {
    const clamp1 = (v: number) => Math.max(-1, Math.min(1, v));
    const norm = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
    const t0 = performance.now();

    while (!exploreCancel && performance.now() - t0 < 18000) {
      const dx = gx - robotX;
      const dy = gy - robotY;
      const dist = Math.hypot(dx, dy);
      const yaw = (robotYawDeg * Math.PI) / 180;
      const wantYaw = (wantYawDeg * Math.PI) / 180;

      if (dist > 0.03) {
        const headingToGoal = Math.atan2(dy, dx);
        const rel = norm(headingToGoal - yaw);
        baseVel.turn = clamp1(1.5 * rel);
        const p = bodyToPrimitives(Math.abs(rel) < 0.4 ? 0.6 : 0, 0);
        baseVel.fwd = p.fwd;
        baseVel.bl = p.bl;
        baseVel.br = p.br;
      } else {
        const relYaw = norm(wantYaw - yaw);
        if (Math.abs(relYaw) <= 0.05) {
          break; // Arrived and oriented
        }
        baseVel.fwd = 0;
        baseVel.bl = 0;
        baseVel.br = 0;
        baseVel.turn = clamp1(1.5 * relYaw);
      }
      await delay(50);
    }
    zeroBaseVel();
  }

  /**
   * Measure the largest ArUco marker side-width to camera frame width ratio.
   * Used for visual standoff distance regulation.
   */
  function computeMaxTagRatio(corners: Map<number, Corners>, frameW = 640): number {
    let maxR = 0;
    for (const [id, c] of corners) {
      if (id < NAV_TAG_LO || id > NAV_TAG_HI) continue;
      // Average width from corner 0 to 1 and corner 3 to 2
      const w01 = Math.hypot(c[1][0] - c[0][0], c[1][1] - c[0][1]);
      const w32 = Math.hypot(c[2][0] - c[3][0], c[2][1] - c[3][1]);
      const markerW = (w01 + w32) / 2;
      const r = markerW / frameW;
      if (r > maxR) maxR = r;
    }
    return maxR;
  }

  /**
   * Unified Station Exploration:
   * 1. Regulates standoff distance based on user-configured `settings.tagCameraRatio`.
   * 2. Reverses backward if no tag is visible initially until tags enter FOV.
   * 3. Executes a bidirectional 3-wheel blended lateral strafe sweep between boundary tags (#240).
   * 4. As each new station card is discovered along the rack, pauses the robot momentarily,
   *    captures a frame, runs homography OCR to extract the station label & description,
   *    and continues scanning without requiring a separate exploration pass.
   */
  async function exploreStations() {
    if (exploring || !hasBase) return;
    exploring = true;
    exploringType = 'unified';
    exploreCancel = false;
    exploreProgress = 0;
    atTag = null;
    armMode = 'rest';
    forgetTags(); // Clear remembered places so fresh discovery populates them live
    appendLog('[Station Exploration] Initializing unified station exploration sequence...');
    await settleAndMeasure();

    let alignedYawRad = (robotYawDeg * Math.PI) / 180;
    const discoveredStationIds = new Set<number>();
    const ocrScannedStationIds = new Set<number>();

    try {
      // ── Phase 1: Tag Visibility & Standoff Ratio Alignment ───────────────
      appendLog(`[Station Exploration] Phase 1: Regulating standoff ratio to target ${(settings.tagCameraRatio * 100).toFixed(1)}%...`);
      const TARGET_RATIO = settings.tagCameraRatio;
      const TARGET_RATIO_LO = TARGET_RATIO * 0.85;
      const TARGET_RATIO_HI = TARGET_RATIO * 1.15;

      let standoffAligned = false;
      const standoffStart = performance.now();

      while (!exploreCancel && !standoffAligned) {
        const elapsedStandoff = performance.now() - standoffStart;
        const ratio = computeMaxTagRatio(latestCorners);
        const visibleCount = visibleNavTags.length;

        // Heading error stabilization with dynamic optical alignment
        const norm = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
        const clamp1 = (v: number) => Math.max(-1, Math.min(1, v));
        const yaw = (robotYawDeg * Math.PI) / 180;

        const rawDetections =
          latestBaseDetections.length > 0
            ? latestBaseDetections
            : Array.from(latestCorners.entries()).map(([id, corners]) => ({ id, corners }));
        const pairs = findLabeledTagPairs(rawDetections);
        let yawErr = 0;

        if (pairs.length > 0 && pairs[0].left && pairs[0].right) {
          const p = pairs[0];
          const hLeftOuter = Math.hypot(p.left[3][0] - p.left[0][0], p.left[3][1] - p.left[0][1]);
          const hRightOuter = Math.hypot(p.right[2][0] - p.right[1][0], p.right[2][1] - p.right[1][1]);
          const avgHeight = (hLeftOuter + hRightOuter) / 2;
          if (avgHeight > 0) yawErr = (hLeftOuter - hRightOuter) / avgHeight;
          alignedYawRad = (robotYawDeg * Math.PI) / 180;
        } else {
          yawErr = norm(alignedYawRad - yaw);
        }

        if (visibleCount === 0) {
          // No tag is visible: reverse backward to expand camera FOV until tags appear
          baseVel.fwd = -0.22;
          baseVel.bl = 0.11;
          baseVel.br = 0.11;
          baseVel.turn = clamp1(1.2 * yawErr);
        } else {
          // Tag is visible: check side-width to camera-width ratio
          if (ratio < TARGET_RATIO_LO) {
            // Tag too small / too far: advance forward
            baseVel.fwd = 0.18;
            baseVel.bl = -0.09;
            baseVel.br = -0.09;
            baseVel.turn = clamp1(1.2 * yawErr);
          } else if (ratio > TARGET_RATIO_HI) {
            // Tag too large / too close: back up
            baseVel.fwd = -0.18;
            baseVel.bl = 0.09;
            baseVel.br = 0.09;
            baseVel.turn = clamp1(1.2 * yawErr);
          } else {
            // Ratio is within optimal band!
            standoffAligned = true;
            zeroBaseVel();
            appendLog(`[Station Exploration] Optimal standoff ratio (${(ratio * 100).toFixed(1)}% of frame) established!`);
            break;
          }
        }

        await delay(60);
      }

      zeroBaseVel();
      await delay(300);
      if (exploreCancel) return;

      // ── Phase 2: Bidirectional Linear Strafe Sweep with Simultaneous OCR ─
      appendLog('[Station Exploration] Phase 2: Starting 3-wheel lateral sweep with simultaneous OCR...');
      const sweepStart = performance.now();
      let direction = 1; // 1 = strafe towards +Y, -1 = strafe towards -Y
      let boundaryHits = 0;
      let lastBoundaryHitTime = 0;
      const strafeSpeed = 0.35;

      while (!exploreCancel) {
        const now = performance.now();
        const elapsedSweep = now - sweepStart;

        // Check if Stop/End boundary tag #240 is seen
        if (visibleNavTags.includes(BOUNDARY_TAG_ID) && now - lastBoundaryHitTime > 2500) {
          boundaryHits += 1;
          lastBoundaryHitTime = now;

          if (boundaryHits === 1) {
            appendLog(`[Station Exploration] Boundary tag #${BOUNDARY_TAG_ID} reached! Reversing direction to scan full rack...`);
            direction = -direction;
            zeroBaseVel();
            await delay(400);
          } else {
            appendLog(`[Station Exploration] Opposite boundary tag #${BOUNDARY_TAG_ID} reached! Full bidirectional sweep complete.`);
            break;
          }
        }

        // Check visible unique station tags (exclude anchor 200 and boundary 240)
        const currentVisibleNavTags = visibleNavTags.filter((id) => id > 200 && id !== BOUNDARY_TAG_ID);
        for (const tid of currentVisibleNavTags) {
          if (!discoveredStationIds.has(tid)) {
            discoveredStationIds.add(tid);
            appendLog(`[Station Exploration] Discovered Station Tag #${tid} at Y=${robotY.toFixed(2)}m.`);
          }

          // Simultaneous OCR check: If this station has not yet been OCR-scanned and both pair tags are visible
          const hasAnchor = visibleNavTags.includes(RIGHT_NAV_ID);
          if (!ocrScannedStationIds.has(tid) && hasAnchor) {
            ocrScannedStationIds.add(tid);
            appendLog(`[Station Exploration] Station #${tid} group in view. Aligning position & rotation before photo...`);
            // Actively center and square up with the station card before capturing
            await alignToGroup(tid, TARGET_RATIO, 3500);
            alignedYawRad = (robotYawDeg * Math.PI) / 180; // Lock newly verified perpendicular angle
            zeroBaseVel();
            await delay(200); // Settle camera frame cleanly
            const scanRes = await forceScanOcr(tid);
            if (scanRes && scanRes.label) {
              appendLog(`[Station Exploration] Station #${tid} named: "${scanRes.label}" (Desc: "${scanRes.description || ''}")`);
            }
            await delay(150);
          }
        }

        // Maintain standoff ratio during lateral sweep with forward/backward trim
        const ratio = computeMaxTagRatio(latestCorners);
        let fwdTrim = 0;
        if (ratio > 0 && ratio < TARGET_RATIO_LO) fwdTrim = +0.10;
        else if (ratio > TARGET_RATIO_HI) fwdTrim = -0.10;

        // Heading correction: use live optical height ratio when available, or hold alignedYawRad
        const clamp1 = (v: number) => Math.max(-1, Math.min(1, v));
        const norm = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
        const yaw = (robotYawDeg * Math.PI) / 180;
        const rawSweepDetections =
          latestBaseDetections.length > 0
            ? latestBaseDetections
            : Array.from(latestCorners.entries()).map(([id, corners]) => ({ id, corners }));
        const sweepPairs = findLabeledTagPairs(rawSweepDetections);
        let yawErr = 0;

        if (sweepPairs.length > 0 && sweepPairs[0].left && sweepPairs[0].right) {
          const p = sweepPairs[0];
          const hLeftOuter = Math.hypot(p.left[3][0] - p.left[0][0], p.left[3][1] - p.left[0][1]);
          const hRightOuter = Math.hypot(p.right[2][0] - p.right[1][0], p.right[2][1] - p.right[1][1]);
          const avgHeight = (hLeftOuter + hRightOuter) / 2;
          if (avgHeight > 0) yawErr = (hLeftOuter - hRightOuter) / avgHeight;
          alignedYawRad = (robotYawDeg * Math.PI) / 180;
        } else {
          yawErr = norm(alignedYawRad - yaw);
        }

        // Compute holonomic 3-wheel primitives with forward drift compensation
        const p = bodyToPrimitives(fwdTrim, direction * strafeSpeed);
        baseVel.fwd = p.fwd;
        baseVel.bl = p.bl;
        baseVel.br = p.br;
        baseVel.turn = clamp1(1.2 * yawErr);

        exploreProgress = boundaryHits === 0 ? 0.35 : 0.75;

        await delay(60);
      }
      zeroBaseVel();
      await delay(400);

      if (exploreCancel) return;

      const stationCount = knownTags.size;
      const namedCount = [...knownTags.values()].filter((t) => t.label && !t.label.startsWith('Station')).length;
      appendLog(
        `[Station Exploration] Sweep complete! Found ${stationCount} station(s) (${namedCount} named): [${[...knownTags.values()].map((t) => `#${t.id} ${t.label || ''}`).join(', ')}].`
      );
      exploreProgress = 1.0;
      hasExplored = true;
    } finally {
      zeroBaseVel();
      baseLink.stopWheels().catch(() => {});
      exploring = false;
      exploringType = null;
      exploreProgress = 0;
      navigating = false;
    }
  }

  const exploreTags = exploreStations;
  const exploreNames = exploreStations;
  const explore = exploreStations;

  /**
   * Raise the arm over whatever we've parked at and see what's on it, searching
   * rather than trusting one pose.
   *
   * The board-view pose is a good guess, not a guarantee: where it points depends
   * on the base having parked exactly square at exactly the standoff distance, and
   * a couple of degrees out is enough to leave the surface outside a narrow frame.
   * So it looks, and if it sees nothing it pans and tilts through VIEW_SWEEP until
   * something shows up — the same nudging that was being done by hand, done in
   * order and without moving the base.
   */
  async function lookForItems(): Promise<boolean> {
    if (armPickBusy) return false;
    lookCancel = false;
    lookingForItems = true;
    viewOffset = [0, 0, 0]; // start from the nominal pose, not last run's find
    await raiseToView();
    // The roll is not forced to zero here any more: VIEW_POSE carries a deliberate
    // 30° of wrist roll, and overriding it turned the camera off the board the
    // moment manual control resumed.
    await delay(400); // let the detector catch up with the new view
    if (armDetectedTags.length) {
      armPickMsg = `Found ${armDetectedTags.length} item${armDetectedTags.length === 1 ? '' : 's'}.`;
      lookingForItems = false;
      return true;
    }

    // Check if currently at a 3D printer station (item does not carry an ArUco fiducial marker)
    const curStation = atTag ? STATIONS.find((s) => s.navTag === atTag) : null;
    if (curStation?.stationType === 'printer') {
      armPickMsg = `Docked at 3D Printer #${curStation.navTag} in look position. Item ready on print bed.`;
      lookingForItems = false;
      return true;
    }

    armPickBusy = true;
    try {
      for (let i = 1; i < VIEW_SWEEP.length; i++) {
        if (lookCancel) {
          armPickMsg = 'Stopped looking.';
          return false;
        }
        const [panDeg, liftDeg, flexDeg] = VIEW_SWEEP[i];
        const rad = (d: number) => (d * Math.PI) / 180;
        viewOffset = [rad(panDeg), rad(liftDeg), rad(flexDeg)];
        jointHoldPose = viewPoseNow();
        const sign = (d: number) => `${d > 0 ? '+' : ''}${d}°`;
        const where =
          `pan ${sign(panDeg)}` +
          `${liftDeg ? `, lift ${sign(liftDeg)}` : ''}` +
          `${flexDeg ? `, down ${sign(flexDeg)}` : ''}`;
        armPickMsg = `Looking for items — ${where} (${i}/${VIEW_SWEEP.length - 1})…`;
        await settleAndMeasure();
        await delay(220); // the detector runs behind the motion
        if (armDetectedTags.length) {
          armPickMsg =
            `Found ${armDetectedTags.length} item${armDetectedTags.length === 1 ? '' : 's'} ` +
            `at ${where}.`;
          return true;
        }
      }
      viewOffset = [0, 0, 0]; // found nothing: don't leave a bogus offset behind
      armPickMsg = 'Nothing found anywhere in the sweep — try moving closer, or check the arm camera.';
      return false;
    } finally {
      jointHoldPose = null;
      handBackToManual(); // leave the arm looking wherever it stopped
      armPickBusy = false;
      lookingForItems = false;
    }
  }

  // ── The whole job, end to end ───────────────────────────────────────────────
  //
  // Find the station, drive to it, look at what's on it, pick one up. Each stage
  // is the routine its own button runs, so this adds no second implementation to
  // keep in step — what it adds is the hand-offs, which is where the run actually
  // used to break: arriving at a standoff too far to reach from, and a fixed
  // looking pose that missed the board.
  let runningAll = $state(false);
  let runStep = $state<string | null>(null);

  /** Wait for auto-navigation to finish, or give up. */
  async function waitForArrival(timeoutMs = 60000): Promise<boolean> {
    const t0 = performance.now();
    while (navigating && performance.now() - t0 < timeoutMs) await delay(200);
    if (navigating) {
      // Never leave it driving. A navigation that cannot converge otherwise runs
      // until the tab closes, and everything gated on `navigating` stays stuck.
      toggleNavigate();
      return false;
    }
    return true;
  }

  async function runEverything() {
    if (runningAll || !hasBase) return;
    runningAll = true;
    runStep = null;
    // Take the base and the arm back from anything already running, so a stuck
    // navigation or a half-finished routine can't block a fresh start.
    if (navigating) toggleNavigate();
    exploreCancel = true;
    armPickBusy = false;
    jointHoldPose = null;
    await delay(200);
    exploreCancel = false;
    try {
      // 1 · Find the station, if we don't already know where it is.
      if (!knownTags.has(STATION_TAG_ID)) {
        runStep = 'Exploring for the station…';
        await explore();
      }
      const navId = knownTags.has(STATION_TAG_ID)
        ? STATION_TAG_ID
        : [...knownTags.keys()][0];
      if (navId == null) {
        runStep = 'No nav tag found anywhere — nothing to drive to.';
        return;
      }

      // 2 · Drive straight to picking distance. There used to be a stop further
      //     back to look from first, on the assumption that close range couldn't
      //     see — but the close-up looking pose sees the board perfectly well, so
      //     the extra park, sweep and second drive bought nothing but a minute.
      navStandoff = NAV_PICK_M;
      runStep = `Driving to tag ${navId} — parking at ${(NAV_PICK_M * 100).toFixed(0)} cm…`;
      driveToTag(navId);
      if (!(await waitForArrival())) {
        runStep = `Gave up driving to tag ${navId} — still navigating.`;
        return;
      }

      // 3 · Look at what's on the surface, from right up close.
      runStep = 'Looking for items…';
      if (!(await lookForItems())) {
        runStep = 'Parked, but nothing visible on the surface.';
        return;
      }

      // 5 · Pick one. Prefer the object tag; fall back to whatever is in frame.
      let id = armDetectedTags.includes(STATION_OBJECT_TAG)
        ? STATION_OBJECT_TAG
        : armDetectedTags[0];

      // The nav tag is on the pedestal's side; the object sits on top, further
      // back again. Parking square to the tag can therefore leave the object just
      // outside the arm's envelope, and no amount of arm motion fixes that — so
      // when the approach reports it can't reach, close the gap with the base and
      // look again rather than declaring the pick impossible.
      for (let attempt = 0; attempt < 4; attempt++) {
        runStep = `Picking up tag ${id}…`;
        if (await pickUpTag(id)) {
          runStep = `Done — ${armPickMsg}`;
          return;
        }
        if (!approachOutOfReach) break;
        // Drive the shortfall, not a fixed nudge. A flat 600 ms creep closed about
        // 50 mm a go, so being 160 mm short burned every retry getting there.
        const ms = Math.max(300, Math.min(2500, (approachShortfallM / NUDGE_SPEED) * 1000));
        runStep =
          `Out of reach by ${(approachShortfallM * 1000).toFixed(0)} mm — ` +
          `creeping closer (${attempt + 1}/4)…`;
        await nudgeForward(ms);
        await delay(500);
        // A failed look isn't a reason to stop: we know what tag is there and
        // roughly where, and the approach does its own searching. Bailing out here
        // wasted a creep that had already cut the shortfall from 117 mm to 42 mm.
        if (await lookForItems()) {
          id = armDetectedTags.includes(STATION_OBJECT_TAG) ? STATION_OBJECT_TAG : armDetectedTags[0];
        }
      }
      runStep = `Done — ${armPickMsg}`;
    } catch (err) {
      runStep = `Run failed: ${err instanceof Error ? err.message : String(err)}`;
      console.error('runEverything', err);
    } finally {
      runningAll = false;
    }
  }

  // Looking can be called off part-way: the sweep moves the arm a step at a time,
  // so there is always a sensible place to stop, and waiting out a search you can
  // already see is going nowhere is just waiting.
  let lookingForItems = $state(false);
  let lookCancel = false;

  /**
   * Carry whatever is in the jaws to the basket and set it down.
   *
   * Same shape as the pick: drive to the station, put the arm over it, let go.
   * The item stays gripped the whole way because the grasp is maintained on every
   * physics step, parked or not.
   */
  async function putInBasket(): Promise<boolean> {
    if (armPickBusy || runningAll || !hasBase) return false;
    const basket = STATIONS.find((st) => st.prop === 'basket');
    if (!basket) return false;
    if (!knownTags.has(basket.navTag)) {
      armPickMsg = `Basket (tag ${basket.navTag}) hasn't been found yet — explore first.`;
      return false;
    }
    deliverBusy = true;
    try {
      armPickMsg = 'Carrying it to the basket…';
      // Park the arm on the way. Driving with it stuck out is the one genuinely
      // risky thing here, and the item comes along regardless.
      armMode = 'rest';
      await delay(600);
      navStandoff = NAV_PICK_M;
      driveToTag(basket.navTag);
      if (!(await waitForArrival())) {
        armPickMsg = 'Could not get to the basket.';
        return false;
      }
      armMode = 'active';
      armPickMsg = 'Reaching over the basket…';
      viewOffset = [0, 0, 0];
      jointHoldPose = viewPoseNow(); // leans out over what is directly in front
      await settleAndMeasure();
      await delay(400);
      jointHoldPose = null;
      handBackToManual();
      await openGripper();
      armPickMsg = 'Dropped it in the basket.';
      return true;
    } catch (err) {
      armPickMsg = `Delivery failed: ${err instanceof Error ? err.message : String(err)}`;
      console.error('putInBasket', err);
      return false;
    } finally {
      deliverBusy = false;
      handBackToManual();
    }
  }
  let deliverBusy = $state(false);

  let nudgeMs = $state(600); // how long the forward nudge drives for
  let nudging = $state(false);

  /**
   * Creep forward for a moment. Navigation parks at the standoff distance, which
   * is deliberately conservative; this closes the last bit once you can see it's
   * lined up, without having to re-run the whole approach.
   */
  async function nudgeForward(ms: number = nudgeMs) {
    if (nudging || !hasBase) return;
    nudging = true;
    if (navigating) toggleNavigate(); // hand the base over from auto-nav
    try {
      // Set the intent directly: setDrive ignores callers while the nudge owns
      // the base, which is what keeps the idle joystick from wiping it.
      baseVel.fwd = 1;
      baseVel.bl = 0;
      baseVel.br = 0;
      await delay(ms);
    } finally {
      zeroBaseVel();
      baseLink.stopWheels().catch(() => {});
      nudging = false;
    }
  }

  /**
   * Stepwise auto-approach to the tag goal quadrilateral, descend, grasp, and lift.
   */
  async function pickUpTag(id: number): Promise<boolean> {
    if (armPickBusy) return false;
    armPickBusy = true;
    blockTag = id;
    try {
      setArmMode('active');
      armPickMsg = `Stepwise auto-approaching Tag #${id}…`;
      const currentGoal = loadGoalQuad(mode);
      const res = await pickCalState.autoApproachAndPick(
        currentGoal,
        id,
        400,
        0.5,
        currentGoal.downStepMm ?? 6,
        currentGoal.tolerancePx ?? 14,
        20
      );
      if (!res.success) {
        armPickMsg = `Pick failed: ${res.msg}`;
        return false;
      }
      holdingItem = true;
      armPickMsg = `Picked up Tag #${id}! Ready to put in basket.`;
      return true;
    } catch (err) {
      armPickMsg = `Pick failed: ${err instanceof Error ? `${err.message}` : String(err)}`;
      console.error('pickUpTag', err);
      return false;
    } finally {
      armPickBusy = false;
      handBackToManual();
    }
  }

  /**
   * Pick up the 3D printed item directly from the printer bed via IK and MuJoCo position reading.
   */
  async function pickUpPrintedItem(st?: StationDef): Promise<boolean> {
    if (armPickBusy || !solver || !session || !mj) return false;
    const printer = st || (atTag ? STATIONS.find((s) => s.navTag === atTag) : null) || STATIONS.find((s) => s.stationType === 'printer');
    if (!printer) {
      armPickMsg = 'No 3D printer station found to pick from.';
      return false;
    }

    armPickBusy = true;
    try {
      setArmMode('active');
      graspMode = 'top';

      // Read exact item position in the MuJoCo world
      let itemPos: [number, number, number] = [printer.x, printer.y, STATION_H + 0.016];
      const meshGid = mj.mj_name2id(session.model, 5, `station_custom_stl_${printer.navTag}`);
      const boxGid = mj.mj_name2id(session.model, 5, `printer_${printer.navTag}_printed_item`);
      const gid = meshGid >= 0 ? meshGid : boxGid;
      if (gid >= 0 && session.data.geom_xpos) {
        itemPos = [
          session.data.geom_xpos[gid * 3],
          session.data.geom_xpos[gid * 3 + 1],
          session.data.geom_xpos[gid * 3 + 2],
        ];
      }

      armPickMsg = 'Opening gripper jaws…';
      gripperCmd = Math.max(DEFAULT_PICK.gripperOpen, gripperRange[0]);
      await delay(400);

      // 1. Move to pre-grasp hover position directly above the printed item
      armPickMsg = 'Positioning arm above 3D printed item…';
      target = [itemPos[0], itemPos[1], itemPos[2] + 0.07];
      await settleAndMeasure();

      // 2. Descend onto the printed item
      armPickMsg = 'Descending to grasp printed item…';
      target = [itemPos[0], itemPos[1], itemPos[2] + 0.005];
      await settleAndMeasure();

      // 3. Close gripper securely around the item and latch binding
      armPickMsg = 'Closing gripper on 3D printed item…';
      gripperCmd = Math.min(DEFAULT_PICK.gripperClose, gripperRange[1]);
      await delay(1000);

      blockTag = printer.propTag || printer.navTag;
      const adr = propAdr.get(blockTag);
      const g = graspFrame();
      if (adr && g && session) {
        const q = session.data.qpos as Float64Array;
        const bQ = adr.q;
        const d = [q[bQ] - g.t[0], q[bQ + 1] - g.t[1], q[bQ + 2] - g.t[2]];
        const Rt = [g.R[0], g.R[3], g.R[6], g.R[1], g.R[4], g.R[7], g.R[2], g.R[5], g.R[8]];
        const bq = [q[bQ + 3], q[bQ + 4], q[bQ + 5], q[bQ + 6]];
        grabbedRel = {
          p: [
            Rt[0] * d[0] + Rt[1] * d[1] + Rt[2] * d[2],
            Rt[3] * d[0] + Rt[4] * d[1] + Rt[5] * d[2],
            Rt[6] * d[0] + Rt[7] * d[1] + Rt[8] * d[2],
          ],
          R: matMul3x3(Rt, quatToMat(bq)),
        };
      }
      holdingItem = true;

      // 4. Lift straight up
      armPickMsg = 'Lifting 3D printed item…';
      target = [itemPos[0], itemPos[1], itemPos[2] + 0.12];
      await settleAndMeasure();

      armPickMsg = 'Holding 3D printed item! Click "Put in basket" to deliver.';
      appendLog(`[Arm] Successfully picked up 3D printed item from Station #${printer.navTag} print bed! Ready to deliver to basket.`);
      return true;
    } catch (err) {
      armPickMsg = `Pick failed: ${err instanceof Error ? err.message : String(err)}`;
      console.error('pickUpPrintedItem error:', err);
      return false;
    } finally {
      armPickBusy = false;
      handBackToManual();
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

  /**
   * Teleport the sim robot back to the origin without disturbing the arm: the
   * base pose goes to 0,0,0° while every joint angle is left exactly as it is.
   *
   * The end-effector target then has to follow, since the same joint angles now
   * put the gripper somewhere else in the world. No hand-rolled forward
   * kinematics needed — `session.forward()` is MuJoCo's FK, and reading
   * `solver.sitePosition()` afterwards gives the new grasp-site position
   * directly (that's what handBackToManual does, and it pins the wrist roll too
   * so the arm genuinely holds its pose).
   */
  function resetRobotToCentre() {
    if (!session) return;
    if (navigating) toggleNavigate(); // auto-nav owns the base otherwise
    zeroBaseVel();
    robotX = 0;
    robotY = 0;
    robotYawDeg = 0;
    placeRobot();
    session.forward(); // FK with the unchanged joint angles at the new base pose
    handBackToManual(); // EE target := where the gripper actually is now
  }

  /**
   * Complete Simulation Reset:
   * - Restores all physical items (apple, banana, orange, bottle, plant, blocks) to starting locations via qpos0.
   * - Releases any gripped items.
   * - Resets the robot arm joint angles, target, and kinematics to initial rest pose.
   * - Resets robot base position and orientation to (0, 0, 0°).
   * - Clears remembered places / station discoveries.
   * - Cancels any active navigation, exploration, or pick sequences.
   */
  function resetAll() {
    if (navigating) { navCancel = true; navigating = false; }
    if (exploring) { exploreCancel = true; exploring = false; }
    runningAll = false;
    runStep = null;
    armPickBusy = false;
    jointHoldPose = null;

    zeroBaseVel();
    robotX = 0.412;
    robotY = 0.046;
    robotYawDeg = -19.7;

    grabbedRel = null;
    holdingItem = false;
    gripperCmd = gripperRange[0]; // open gripper
    manualArm = false;

    forgetTags();

    if (session) {
      // Preserve live arm joint angles so the arm slews home smoothly at maxArmSpeedDeg
      // rather than slamming to qpos0 at hardware servo limits.
      const currentArmQ = ARM_DOFS.map((i) => (session.data.qpos as Float64Array)[i]);
      const currentGripQ = (session.data.qpos as Float64Array)[5];

      // Restore all environment scene items / props / drawers from model.qpos0
      (session.data.qpos as Float64Array).set(session.model.qpos0 as Float64Array);
      (session.data.qvel as Float64Array).fill(0);
      if (session.data.ctrl) (session.data.ctrl as Float64Array).fill(0);

      // Restore the arm joints to current pose so the motion controller slews it safely home
      for (let i = 0; i < 5; i++) {
        (session.data.qpos as Float64Array)[i] = currentArmQ[i];
      }
      (session.data.qpos as Float64Array)[5] = currentGripQ;

      placeRobot();
      session.forward();
    }

    // Command arm to park at restTargetPose (Home) smoothly rate-limited by settings.maxArmSpeedDeg
    armMode = 'rest';

    appendLog('[System] Reset complete: base, places, and environment restored; arm returning home at max motor speed.');
  }

  async function maybeStreamBase() {
    if (mode !== 'real' || !baseLink.connected || wheelSending) return;
    const now = performance.now();
    if (now - lastWheelSent < 66) return; // ~15 Hz
    lastWheelSent = now;
    const speeds = wheelSpeeds(baseVel.fwd, baseVel.bl, baseVel.br, baseVel.turn, baseLink.config);
    // Skip overlapping frames so the shared bus queue never backs up.
    wheelSending = true;
    try {
      await robot.syncWriteWheelSpeed(speeds);
    } catch (e) {
      baseLink.error = e instanceof Error ? e.message : String(e);
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

  // ── Station Layout Management ─────────────────────────────────────────────
  let layouts = $state<Record<string, StationDef[]>>({
    'Default Layout': [...DEFAULT_STATIONS],
  });
  let currentLayoutName = $state('Default Layout');

  // ── ACT Models & Mission Path Planner (Browser-based) ──────────────────────
  let actModels = $state<ActModelDef[]>([...DEFAULT_ACT_MODELS]);
  let selectedActModelId = $state<string>(DEFAULT_ACT_MODELS[0]?.id || '');
  let actRunning = $state(false);

  function cloneMissions(plans: PathPlan[]): PathPlan[] {
    return plans.map((p) => ({
      ...p,
      steps: p.steps.map((s) => ({ ...s })),
    }));
  }

  let layoutMissions = $state<Record<string, PathPlan[]>>({
    'Default Layout': cloneMissions(DEFAULT_MISSIONS),
  });
  let selectedMissionIdx = $state<number>(0);
  let missions = $derived<PathPlan[]>(layoutMissions[currentLayoutName] || []);
  let activePlan = $derived<PathPlan | null>(missions[selectedMissionIdx] || null);

  let layoutActModels = $state<Record<string, ActModelDef[]>>({
    'Default Layout': [...DEFAULT_ACT_MODELS],
  });

  let planRunning = $state(false);
  let planAbortRequested = false;
  let planPauseRequested = false;

  async function runActModel(modelId: string): Promise<string> {
    const model = actModels.find((m) => m.id === modelId) || {
      id: modelId,
      name: modelId.split('/').pop() || modelId,
      datasetId: 'lerobot/dataset_default',
    };
    if (actRunning) return `ACT policy is already executing.`;
    actRunning = true;
    selectedActModelId = model.id;
    appendLog(`[ACT] Initializing policy "${model.name}" (${model.id}) [Dataset: ${model.datasetId}]`);

    try {
      // Minimalist UI simulation of ACT inference & trajectory execution
      runStep = `ACT: Inferring chunk via ${model.name}…`;
      await delay(650);
      runStep = `ACT: Executing action chunk [50 steps]…`;
      await delay(900);
      appendLog(`[ACT] Completed action chunk for "${model.name}".`);
      runStep = `Done — ACT ${model.name}`;
      return `ACT policy "${model.name}" executed successfully.`;
    } finally {
      actRunning = false;
    }
  }

  function togglePlanExecution() {
    if (planRunning) {
      pausePathPlan();
    } else {
      executePathPlan();
    }
  }

  function pausePathPlan() {
    planPauseRequested = true;
    if (navigating) toggleNavigate();
    if (activePlan) activePlan.status = 'paused';
    planRunning = false;
    runStep = `Mission paused`;
    appendLog(`[PathPlanner] Mission "${activePlan?.name || ''}" paused.`);
  }

  function abortPathPlan() {
    planAbortRequested = true;
    if (navigating) toggleNavigate();
    if (activePlan) activePlan.status = 'aborted';
    planRunning = false;
    runStep = `Mission stopped`;
    appendLog('[PathPlanner] Mission stopped.');
  }

  function resetCurrentMission() {
    if (planRunning) pausePathPlan();
    if (activePlan) {
      activePlan.status = 'idle';
      activePlan.currentStepIndex = 0;
      for (const s of activePlan.steps) {
        s.status = 'pending';
        s.error = undefined;
      }
    }
    runStep = null;
  }

  async function executePathPlan(): Promise<string> {
    if (!activePlan || activePlan.steps.length === 0) return 'No active path plan.';
    if (planRunning) return 'Path plan is already running.';

    if (activePlan.status === 'completed' || activePlan.status === 'aborted') {
      activePlan.currentStepIndex = 0;
      for (const s of activePlan.steps) {
        s.status = 'pending';
        s.error = undefined;
      }
    }

    planRunning = true;
    planAbortRequested = false;
    planPauseRequested = false;
    activePlan.status = 'running';
    appendLog(`[PathPlanner] ${activePlan.currentStepIndex > 0 ? 'Resuming' : 'Starting'} mission: "${activePlan.name}" (${activePlan.steps.length} steps)`);

    try {
      for (let i = activePlan.currentStepIndex; i < activePlan.steps.length; i++) {
        if (planAbortRequested) {
          activePlan.status = 'aborted';
          appendLog(`[PathPlanner] Mission aborted.`);
          return 'Mission plan aborted.';
        }
        if (planPauseRequested) {
          activePlan.status = 'paused';
          runStep = `Mission paused: ${activePlan.name}`;
          appendLog(`[PathPlanner] Mission paused at step ${i + 1}.`);
          return 'Mission plan paused.';
        }

        activePlan.currentStepIndex = i;
        const step = activePlan.steps[i];
        step.status = 'active';
        runStep = `Mission Step ${i + 1}/${activePlan.steps.length}: ${step.description}`;
        appendLog(`[PathPlanner] Step ${i + 1}: ${step.description}`);

        if (step.action === 'navigate') {
          let targetTagId: number | null = null;
          if (typeof step.target === 'number') {
            targetTagId = step.target;
          } else if (typeof step.target === 'string') {
            const num = parseInt(step.target, 10);
            if (!isNaN(num)) {
              targetTagId = num;
            } else {
              const matched = STATIONS.find(
                (s) => s.label.toLowerCase() === (step.target as string).toLowerCase().trim(),
              );
              if (matched) targetTagId = matched.navTag;
            }
          }

          if (targetTagId == null) {
            step.status = 'failed';
            step.error = `Station target "${step.target}" not found`;
            throw new Error(step.error);
          }

          navStandoff = NAV_PICK_M;
          driveToTag(targetTagId);
          const arrived = await waitForArrival(45000);
          if (planPauseRequested) {
            step.status = 'pending';
            activePlan.status = 'paused';
            runStep = `Mission paused`;
            return 'Mission plan paused.';
          }
          if (planAbortRequested) {
            step.status = 'pending';
            activePlan.status = 'aborted';
            return 'Mission plan aborted.';
          }
          if (!arrived) {
            step.status = 'failed';
            step.error = `Timed out arriving at station ${targetTagId}`;
            throw new Error(step.error);
          }
        } else if (step.action === 'run_act') {
          const mId = step.modelId || selectedActModelId;
          await runActModel(mId);
        } else if (step.action === 'pick_tag') {
          await lookForItems();
          const targetTag = step.tagId ?? armDetectedTags[0] ?? STATION_OBJECT_TAG;
          await pickUpTag(targetTag);
        } else if (step.action === 'put_basket') {
          await putInBasket();
        } else if (step.action === 'open_gripper') {
          await openGripper();
        } else if (step.action === 'close_gripper') {
          await runCloseGripper();
        } else if (step.action === 'wait') {
          const waitMs = (step.duration || 1) * 1000;
          await delay(waitMs);
        }

        if (planPauseRequested) {
          step.status = 'pending';
          activePlan.status = 'paused';
          runStep = `Mission paused`;
          return 'Mission plan paused.';
        }
        if (planAbortRequested) {
          step.status = 'pending';
          activePlan.status = 'aborted';
          return 'Mission plan aborted.';
        }

        step.status = 'completed';
        activePlan.currentStepIndex = i + 1;
        await delay(300);
      }

      activePlan.status = 'completed';
      runStep = `Done — Mission "${activePlan.name}" completed`;
      appendLog(`[PathPlanner] Mission "${activePlan.name}" completed successfully.`);
      return `Mission "${activePlan.name}" completed successfully.`;
    } catch (err: any) {
      if (activePlan) {
        activePlan.status = 'failed';
        if (activePlan.steps[activePlan.currentStepIndex]) {
          activePlan.steps[activePlan.currentStepIndex].status = 'failed';
        }
      }
      const msg = err instanceof Error ? err.message : String(err);
      runStep = `Mission failed: ${msg}`;
      appendLog(`[PathPlanner] Mission error: ${msg}`);
      return `Mission plan failed: ${msg}`;
    } finally {
      planRunning = false;
    }
  }

  async function applyStationLayout(name: string, stations: StationDef[]) {
    setCustomStations(stations);
    currentLayoutName = name;

    // Synchronize knownTags with current station layout
    for (const s of STATIONS) {
      if (!s.isBoundary && s.navTag !== BOUNDARY_TAG_ID) {
        knownTags.set(s.navTag, {
          id: s.navTag,
          x: s.x,
          y: s.y,
          z: STATION_H,
          faceYaw: 0,
          label: s.label || `Station ${s.navTag}`,
          description: s.description,
        });
      }
    }
    if (ready && mj && mode === 'sim') {
      cancelAnimationFrame(raf);
      try {
        await loadScene(selectedRobot);
        loop();
      } catch (err) {
        errorMsg = err instanceof Error ? err.message : String(err);
        status = 'Failed to load';
      }
    }
  }

  async function addStationFromAi({
    name,
    side = 'right',
    prop,
    station_type,
    stl_name,
    item_count,
  }: {
    name: string;
    side?: 'left' | 'right';
    prop?: string;
    station_type?: 'simple' | 'printer' | 'deposit' | 'modifier' | 'table';
    stl_name?: string;
    item_count?: number;
  }): Promise<string> {
    // If currently on "Default Layout", auto-clone to a new custom layout
    if (currentLayoutName === 'Default Layout') {
      let idx = 1;
      while (layouts[`Custom Layout ${idx}`]) idx++;
      const newLayoutName = `Custom Layout ${idx}`;
      layouts[newLayoutName] = STATIONS.map((s) => ({ ...s }));
      currentLayoutName = newLayoutName;
    }

    const existingNavTags = new Set(STATIONS.map((s) => s.navTag));
    let nextId = 210;
    while (existingNavTags.has(nextId) || nextId === BOUNDARY_TAG_ID) {
      nextId++;
    }

    const existingPropTags = new Set(STATIONS.map((s) => s.propTag));
    let nextPropTag = 110;
    while (existingPropTags.has(nextPropTag)) {
      nextPropTag++;
    }

    const nonBoundary = STATIONS.filter((s) => !s.isBoundary).sort((a, b) => a.y - b.y);
    let newY = 0;

    if (side === 'left') {
      const minY = nonBoundary[0]?.y ?? -0.75;
      newY = Number((minY - 0.35).toFixed(2));
      const startPost = STATIONS.find((s) => s.isBoundary && s.y < 0);
      if (startPost) {
        startPost.y = Number((newY - 0.45).toFixed(2));
      }
    } else {
      const maxY = nonBoundary[nonBoundary.length - 1]?.y ?? 0.65;
      newY = Number((maxY + 0.35).toFixed(2));
      const endPost = STATIONS.find((s) => s.isBoundary && s.y > 0);
      if (endPost) {
        endPost.y = Number((newY + 0.45).toFixed(2));
      }
    }

    // Determine station archetype
    let resolvedType: StationType = station_type || 'simple';
    const lowerName = name.toLowerCase();
    if (!station_type) {
      if (lowerName.includes('printer') || lowerName.includes('3d')) resolvedType = 'printer';
      else if (lowerName.includes('deposit') || lowerName.includes('dispenser') || lowerName.includes('storage')) resolvedType = 'deposit';
      else if (lowerName.includes('modifier') || lowerName.includes('magic') || lowerName.includes('transmute')) resolvedType = 'modifier';
      else if (lowerName.includes('table') || lowerName.includes('bench') || lowerName.includes('desk')) resolvedType = 'table';
    }

    const propKind = (resolvedType === 'table' || resolvedType === 'printer' || resolvedType === 'modifier')
      ? ((prop as PropKind) || 'none')
      : ((prop as PropKind) || 'block');
    const newStation: StationDef = {
      navTag: nextId,
      label: name.trim().toUpperCase(),
      description: `${name} station`,
      x: LINE_X,
      y: newY,
      prop: propKind,
      propTag: propKind === 'basket' || resolvedType === 'table' ? 0 : nextPropTag,
      stationType: resolvedType,
      stlName: stl_name,
      itemCount: resolvedType === 'deposit' ? (item_count ?? 3) : undefined,
      maxItemCount: resolvedType === 'deposit' ? (item_count ?? 3) : undefined,
      state: resolvedType === 'printer' ? 'ready' : undefined,
    };

    const updatedStations = [...STATIONS, newStation].sort((a, b) => a.y - b.y);
    layouts[currentLayoutName] = updatedStations;

    await applyStationLayout(currentLayoutName, updatedStations);

    const confirmation = `Created ${resolvedType.toUpperCase()} station "${name.toUpperCase()}" with Tag #${nextId} on the ${side} at Y=${newY.toFixed(2)}m (Layout: ${currentLayoutName}).`;
    appendLog(confirmation);
    return confirmation;
  }

  function exportCurrentLayout() {
    const data = {
      layoutName: currentLayoutName,
      version: '2.0',
      exportedAt: new Date().toISOString(),
      actModels: actModels.map((m) => ({ ...m })),
      pathPlans: missions.map((m) => ({
        id: m.id,
        name: m.name,
        description: m.description,
        status: 'idle',
        currentStepIndex: 0,
        steps: m.steps.map((s) => ({
          id: s.id,
          action: s.action,
          target: s.target,
          modelId: s.modelId,
          tagId: s.tagId,
          duration: s.duration,
          description: s.description,
          status: 'pending',
        })),
      })),
      pathPlan: activePlan
        ? {
            id: activePlan.id,
            name: activePlan.name,
            description: activePlan.description,
            status: 'idle',
            currentStepIndex: 0,
            steps: activePlan.steps.map((s) => ({
              id: s.id,
              action: s.action,
              target: s.target,
              modelId: s.modelId,
              tagId: s.tagId,
              duration: s.duration,
              description: s.description,
              status: 'pending',
            })),
          }
        : null,
      stations: STATIONS.map((s) => ({
        navTag: s.navTag,
        label: s.label,
        description: s.description,
        stationType: s.stationType || 'simple',
        stlName: s.stlName,
        itemCount: s.itemCount,
        maxItemCount: s.maxItemCount,
        state: s.state,
        outputStlName: s.outputStlName,
        actModelId: s.actModelId,
        actDatasetId: s.actDatasetId,
        x: s.x,
        y: s.y,
        prop: s.prop,
        propTag: s.propTag,
        isBoundary: s.isBoundary,
      })),
      models: Object.keys(customStlAssets),
    };

    const hasCustomStls = Object.keys(customStlAssets).length > 0;
    const cleanName = currentLayoutName.toLowerCase().replace(/[^a-z0-9]/g, '_');

    if (hasCustomStls) {
      // ZIP bundle containing layout.json + all custom STL meshes
      const zipEntries: Record<string, Uint8Array> = {
        'layout.json': strToU8(JSON.stringify(data, null, 2)),
      };

      for (const [name, buf] of Object.entries(customStlAssets)) {
        const fileName = name.startsWith('models/') ? name : `models/${name}`;
        zipEntries[fileName] = buf;
      }

      const zipped = zipSync(zipEntries);
      const blob = new Blob([zipped], { type: 'application/zip' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `station_layout_${cleanName}.zip`;
      a.click();
      URL.revokeObjectURL(url);
      appendLog(`Exported station layout ZIP archive "${currentLayoutName}.zip" (${Object.keys(zipEntries).length} files)`);
    } else {
      // Standalone layout.json download
      const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `station_layout_${cleanName}.json`;
      a.click();
      URL.revokeObjectURL(url);
      appendLog(`Exported station layout JSON "${currentLayoutName}.json" with ACT models & mission path plan`);
    }
  }

  let showUploadLayoutModal = $state(false);
  let uploadLayoutName = $state('');
  let uploadLayoutError = $state<string | null>(null);
  let stagedStations = $state<StationDef[] | null>(null);
  let stagedStlAssets = $state<Record<string, Uint8Array>>({});
  let stagedPathPlans = $state<PathPlan[]>([]);
  let stagedActModels = $state<ActModelDef[] | null>(null);

  function parseLayoutJson(text: string, fallbackName?: string) {
    uploadLayoutError = null;
    stagedStations = null;
    stagedPathPlans = [];
    stagedActModels = null;
    try {
      const parsed = JSON.parse(text);
      let rawList: any[] = [];
      let name = fallbackName || 'Uploaded Layout';

      if (Array.isArray(parsed)) {
        rawList = parsed;
      } else if (parsed && typeof parsed === 'object') {
        if (typeof parsed.layoutName === 'string' && parsed.layoutName.trim()) {
          name = parsed.layoutName.trim();
        }
        if (Array.isArray(parsed.stations)) {
          rawList = parsed.stations;
        } else {
          throw new Error('JSON missing "stations" array.');
        }

        // Parse optional ACT models registered in the layout file
        const rawActs = parsed.actModels || parsed.act_models;
        if (Array.isArray(rawActs)) {
          const parsedActs: ActModelDef[] = [];
          for (const item of rawActs) {
            if (item && typeof item.id === 'string') {
              parsedActs.push({
                id: item.id,
                name: item.name || item.id.split('/').pop() || item.id,
                datasetId: item.datasetId || item.dataset_id || 'lerobot/dataset',
                stationTag: Number(item.stationTag || item.station_tag) || undefined,
                description: item.description,
              });
            }
          }
          if (parsedActs.length > 0) {
            stagedActModels = parsedActs;
            actModels = parsedActs;
            if (!actModels.some((m) => m.id === selectedActModelId)) {
              selectedActModelId = actModels[0].id;
            }
          }
        }

        // Parse optional Path Plans / Missions registered in the layout file
        const rawPlansList: any[] = [];
        if (Array.isArray(parsed.pathPlans)) {
          rawPlansList.push(...parsed.pathPlans);
        } else if (Array.isArray(parsed.missions)) {
          rawPlansList.push(...parsed.missions);
        } else if (parsed.pathPlan || parsed.path_plan || parsed.plan) {
          rawPlansList.push(parsed.pathPlan || parsed.path_plan || parsed.plan);
        }

        const parsedPlans: PathPlan[] = [];
        for (const rawPlan of rawPlansList) {
          if (rawPlan && (Array.isArray(rawPlan.steps) || Array.isArray(rawPlan))) {
            const stepsArray = Array.isArray(rawPlan.steps) ? rawPlan.steps : rawPlan;
            const parsedSteps: PathPlanStep[] = stepsArray.map((s: any, idx: number) => ({
              id: s.id || `step-${Date.now()}-${idx}`,
              action: s.action || 'navigate',
              target: s.target,
              modelId: s.modelId || s.model_id,
              tagId: s.tagId != null ? Number(s.tagId) : (s.tag_id != null ? Number(s.tag_id) : undefined),
              duration: s.duration != null ? Number(s.duration) : undefined,
              description: s.description || `${s.action} ${s.target || s.modelId || ''}`,
              status: 'pending',
            }));
            parsedPlans.push({
              id: rawPlan.id || `mission-${Date.now()}-${parsedPlans.length}`,
              name: typeof rawPlan.name === 'string' ? rawPlan.name : `Mission ${parsedPlans.length + 1}`,
              description: typeof rawPlan.description === 'string' ? rawPlan.description : undefined,
              steps: parsedSteps,
              status: 'idle',
              currentStepIndex: 0,
            });
          }
        }
        stagedPathPlans = parsedPlans;
      } else {
        throw new Error('Invalid JSON format for station layout.');
      }

      if (rawList.length === 0) {
        throw new Error('Station layout contains no stations.');
      }

      const validStations: StationDef[] = [];
      for (const item of rawList) {
        if (typeof item !== 'object' || item === null) continue;
        const navTag = Number(item.navTag);
        if (!Number.isFinite(navTag)) continue;
        const isBoundary = Boolean(item.isBoundary || navTag === BOUNDARY_TAG_ID);
        const y = Number(item.y);
        if (!Number.isFinite(y)) continue;
        const x = Number.isFinite(Number(item.x)) ? Number(item.x) : LINE_X;
        const propTag = Number.isFinite(Number(item.propTag)) ? Number(item.propTag) : 0;
        const prop = (item.prop as PropKind) || 'block';
        const label = typeof item.label === 'string' ? item.label : (isBoundary ? 'STOP' : `Station ${navTag}`);
        const description = typeof item.description === 'string' ? item.description : `${label} station`;
        const stationType = (item.stationType as StationType) || 'simple';
        const stlName = typeof item.stlName === 'string' ? item.stlName : undefined;
        const itemCount = Number.isFinite(Number(item.itemCount)) ? Number(item.itemCount) : undefined;
        const maxItemCount = Number.isFinite(Number(item.maxItemCount)) ? Number(item.maxItemCount) : undefined;
        const state = item.state;
        const outputStlName = typeof item.outputStlName === 'string' ? item.outputStlName : undefined;
        const actModelId = typeof item.actModelId === 'string' ? item.actModelId : (typeof item.act_model_id === 'string' ? item.act_model_id : undefined);
        const actDatasetId = typeof item.actDatasetId === 'string' ? item.actDatasetId : (typeof item.act_dataset_id === 'string' ? item.act_dataset_id : undefined);

        validStations.push({
          navTag,
          label,
          description,
          x,
          y,
          prop,
          propTag,
          isBoundary,
          stationType,
          stlName,
          itemCount,
          maxItemCount,
          state,
          outputStlName,
          actModelId,
          actDatasetId,
        });
      }

      if (validStations.length === 0) {
        throw new Error('No valid stations could be parsed from the file.');
      }

      // Ensure boundary markers exist at both ends if missing
      const nonBoundary = validStations.filter((s) => !s.isBoundary).sort((a, b) => a.y - b.y);
      const hasStartBoundary = validStations.some((s) => s.isBoundary && s.y < (nonBoundary[0]?.y ?? 0));
      const hasEndBoundary = validStations.some((s) => s.isBoundary && s.y > (nonBoundary[nonBoundary.length - 1]?.y ?? 0));

      if (!hasStartBoundary && nonBoundary.length > 0) {
        validStations.push({
          navTag: BOUNDARY_TAG_ID,
          label: 'START',
          description: 'Boundary stop stand (start of line)',
          x: LINE_X,
          y: Number((nonBoundary[0].y - 0.25).toFixed(2)),
          prop: 'none',
          propTag: 0,
          isBoundary: true,
          stationType: 'simple',
        });
      }
      if (!hasEndBoundary && nonBoundary.length > 0) {
        validStations.push({
          navTag: BOUNDARY_TAG_ID,
          label: 'STOP',
          description: 'Boundary stop stand (end of line)',
          x: LINE_X,
          y: Number((nonBoundary[nonBoundary.length - 1].y + 0.25).toFixed(2)),
          prop: 'none',
          propTag: 0,
          isBoundary: true,
          stationType: 'simple',
        });
      }

      validStations.sort((a, b) => a.y - b.y);
      uploadLayoutName = name;
      stagedStations = validStations;
    } catch (err) {
      uploadLayoutError = err instanceof Error ? err.message : String(err);
      stagedStations = null;
    }
  }

  async function handleLayoutFileInput(e: Event) {
    const input = e.currentTarget as HTMLInputElement;
    const file = input.files?.[0];
    if (!file) return;

    uploadLayoutError = null;
    stagedStlAssets = {};

    try {
      const buffer = await file.arrayBuffer();
      const bytes = new Uint8Array(buffer);
      const isZip = bytes.length >= 4 && bytes[0] === 0x50 && bytes[1] === 0x4b && bytes[2] === 0x03 && bytes[3] === 0x04;

      if (isZip || file.name.endsWith('.zip')) {
        const unzipped = unzipSync(bytes);
        const extractedStls: Record<string, Uint8Array> = {};
        let layoutJsonText: string | null = null;

        for (const [path, data] of Object.entries(unzipped)) {
          if (path.endsWith('.stl')) {
            const cleanName = path.split('/').pop() || path;
            extractedStls[cleanName] = data;
            extractedStls[path] = data;
          } else if (path.endsWith('.json') || path === 'layout.json') {
            layoutJsonText = strFromU8(data);
          }
        }

        if (!layoutJsonText) {
          throw new Error('ZIP archive is missing a "layout.json" file.');
        }

        stagedStlAssets = extractedStls;
        const fallbackName = file.name.replace(/\.zip$/i, '');
        parseLayoutJson(layoutJsonText, fallbackName);
      } else {
        const text = new TextDecoder().decode(bytes);
        const fallbackName = file.name.replace(/\.json$/i, '');
        parseLayoutJson(text, fallbackName);
      }
    } catch (err) {
      uploadLayoutError = err instanceof Error ? err.message : String(err);
      stagedStations = null;
    } finally {
      input.value = '';
    }
  }

  async function confirmUploadLayout() {
    if (!stagedStations || stagedStations.length === 0) return;
    let finalName = uploadLayoutName.trim() || 'Uploaded Layout';
    if (layouts[finalName]) {
      let idx = 2;
      while (layouts[`${finalName} (${idx})`]) idx++;
      finalName = `${finalName} (${idx})`;
    }

    // Merge any extracted or bundled custom STL models into active asset registry
    Object.assign(customStlAssets, stagedStlAssets);

    layouts[finalName] = stagedStations;
    if (stagedActModels && stagedActModels.length > 0) {
      layoutActModels[finalName] = stagedActModels;
      actModels = [...stagedActModels];
      selectedActModelId = actModels[0]?.id || '';
    }
    layoutMissions[finalName] = stagedPathPlans.length > 0 ? cloneMissions(stagedPathPlans) : cloneMissions(DEFAULT_MISSIONS);
    selectedMissionIdx = 0;

    currentLayoutName = finalName;
    await applyStationLayout(finalName, stagedStations);
    appendLog(
      `Uploaded & applied station layout "${finalName}" (${stagedStations.filter((s) => !s.isBoundary).length} stations, ${layoutMissions[finalName].length} mission(s))`,
    );
    showUploadLayoutModal = false;
    stagedStations = null;
    stagedStlAssets = {};
    stagedPathPlans = [];
    stagedActModels = null;
    uploadLayoutError = null;
  }

  async function handleCustomStlUpload(name: string, buf: Uint8Array) {
    customStlAssets[name] = buf;
    const cleanName = name.replace(/^assets\//, '').replace(/^models\//, '');
    customStlAssets[cleanName] = buf;
    appendLog(`Added custom 3D model: ${name} (${Math.round(buf.byteLength / 1024)} KB)`);

    // If there is an existing 3D printer, modifier, or simple station, assign the uploaded STL
    const targetStation = STATIONS.find((s) => s.stationType === 'printer' || s.stationType === 'modifier' || s.stationType === 'simple');
    if (targetStation && (!targetStation.stlName || targetStation.stlName === 'gear.stl' || targetStation.stlName === 'custom_gear.stl')) {
      targetStation.stlName = name;
    }
    await applyStationLayout(currentLayoutName, STATIONS);
  }

  function handleLayoutDropdownChange(e: Event) {
    const select = e.currentTarget as HTMLSelectElement;
    const val = select.value;
    if (val === '__new__') {
      let idx = 1;
      while (layouts[`Custom Layout ${idx}`]) idx++;
      const newName = `Custom Layout ${idx}`;
      layoutActModels[currentLayoutName] = actModels;
      layouts[newName] = STATIONS.map((s) => ({ ...s }));
      layoutMissions[newName] = cloneMissions(DEFAULT_MISSIONS);
      layoutActModels[newName] = [...actModels];
      currentLayoutName = newName;
      selectedMissionIdx = 0;
      applyStationLayout(currentLayoutName, layouts[currentLayoutName]);
    } else if (val === '__upload__') {
      showUploadLayoutModal = true;
      uploadLayoutError = null;
      stagedStations = null;
      stagedPathPlans = [];
      stagedActModels = null;
      uploadLayoutName = '';
      select.value = currentLayoutName;
    } else if (layouts[val]) {
      layoutActModels[currentLayoutName] = actModels;
      currentLayoutName = val;
      if (!layoutMissions[val]) {
        layoutMissions[val] = cloneMissions(DEFAULT_MISSIONS);
      }
      selectedMissionIdx = 0;
      if (layoutActModels[val]) {
        actModels = [...layoutActModels[val]];
        if (!actModels.some((m) => m.id === selectedActModelId)) {
          selectedActModelId = actModels[0]?.id || '';
        }
      }
      applyStationLayout(currentLayoutName, layouts[currentLayoutName]);
    }
  }

  const onResize = () => fitCanvas();
  $effect(() => {
    window.addEventListener('resize', onResize);
    return () => window.removeEventListener('resize', onResize);
  });

  // Observe viewer container dimensions to immediately resize 2D map and 3D canvas
  $effect(() => {
    if (!wrap) return;
    const ro = new ResizeObserver(() => {
      fitCanvas();
    });
    ro.observe(wrap);
    return () => ro.disconnect();
  });

  // Attach the live stream to the visible preview whenever the element mounts
  // (covers entering real mode after the camera already started).
  $effect(() => {
    if (previewVideo && camStream) {
      previewVideo.srcObject = camStream;
      previewVideo.play().catch(() => {});
    }
  });
  $effect(() => {
    if (armPreview && armCamStream) {
      armPreview.srcObject = armCamStream;
      armPreview.play().catch(() => {});
    }
  });

  // Start/stop the cameras as real mode is entered/left, honouring a deliberate
  // base-camera disconnect (so turning it off doesn't immediately re-open it).
  $effect(() => {
    if (mode !== 'real') {
      stopCamera();
      disconnectArmCamera();
      armView = false;
      stage = '';
      pickPhase = 'idle';
      return;
    }
    // Remote robot frames arrive over WebRTC (remoteCams), no local getUserMedia needed.
    if (robot.mode === 'remote') {
      stopCamera();
      disconnectArmCamera();
      return;
    }
    if (baseCamOff) stopCamera();
    else startCamera();
  });

  // Re-fit 2D map and canvases when switching between sim and real mode
  $effect(() => {
    const _m = mode;
    fitCanvas();
    requestAnimationFrame(() => {
      fitCanvas();
    });
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

  const aiContext = $derived<RobotStateContext>({
    robotX: hasBase ? robotX : 0,
    robotY: hasBase ? robotY : 0,
    robotYawDeg: hasBase ? robotYawDeg : 0,
    hasBase,
    exploring,
    navigating,
    activeNavTag: navigating ? navTagId : null,
    atTag,
    holdingItem,
    knownStations: [...knownTags.values()].map((t) => {
      const matchingDef = STATIONS.find((s) => s.navTag === t.id);
      return {
        id: t.id,
        label: t.label || `Tag ${t.id}`,
        description: t.description,
        stationType: matchingDef?.stationType || 'simple',
        x: Number.isFinite(t.x) ? t.x : (Array.isArray((t as any).p) ? (t as any).p[0] : (matchingDef?.x ?? 0)),
        y: Number.isFinite(t.y) ? t.y : (Array.isArray((t as any).p) ? (t as any).p[1] : (matchingDef?.y ?? 0)),
      };
    }),
    armDetectedTags: armDetectedTags || [],
    baseDetectedTags: seenIds || [],
    activeCamera: activeRemoteCam === REMOTE_ARM_CAM ? 'arm' : 'base',
    currentLayoutName,
    availableStls: Object.keys(customStlAssets),
    availableActModels: actModels,
    activePlanSummary: activePlan
      ? {
          name: activePlan.name,
          status: activePlan.status,
          currentStepIndex: activePlan.currentStepIndex,
          totalSteps: activePlan.steps.length,
          steps: activePlan.steps.map((s) => ({
            action: s.action,
            description: s.description,
            status: s.status,
          })),
        }
      : null,
  });

  const aiCallbacks: RobotActionCallbacks = {
    onNavigate: async (target: number | string) => {
      if (!hasBase) throw new Error('Robot base is not supported in current mode');
      let targetTagId: number | null = null;
      let targetLabel: string = '';

      if (typeof target === 'number') {
        if (knownTags.has(target)) {
          targetTagId = target;
          targetLabel = knownTags.get(target)?.label || `Tag ${target}`;
        }
      } else if (typeof target === 'string') {
        const query = target.trim().toLowerCase();
        const num = parseInt(query, 10);
        if (!isNaN(num) && knownTags.has(num)) {
          targetTagId = num;
          targetLabel = knownTags.get(num)?.label || `Tag ${num}`;
        } else {
          for (const [id, t] of knownTags.entries()) {
            if (
              t.label?.toLowerCase().includes(query) ||
              t.description?.toLowerCase().includes(query) ||
              query.includes(t.label?.toLowerCase() || '___')
            ) {
              targetTagId = id;
              targetLabel = t.label || `Tag ${id}`;
              break;
            }
          }
          if (targetTagId === null) {
            for (const [id, t] of knownTags) {
              const def = STATIONS.find((s) => s.navTag === id);
              if (def && (def.prop.toLowerCase().includes(query) || (def.label && def.label.toLowerCase().includes(query)))) {
                targetTagId = id;
                targetLabel = t.label || def.label || `Tag ${id}`;
                break;
              }
            }
          }
        }
      }

      if (targetTagId === null) {
        throw new Error(`Station "${target}" has not been discovered yet. Please run Explore first.`);
      }
      driveToTag(targetTagId);
      return `Navigating to station ${targetLabel}`;
    },
    onPickTag: async (tagId?: number) => {
      const printer = STATIONS.find((s) => s.stationType === 'printer');
      if ((atTag && atTag === printer?.navTag) || (printer && tagId === printer.navTag)) {
        const ok = await pickUpPrintedItem(printer);
        return ok ? 'Picked up 3D printed item' : 'Failed to pick up printed item';
      }
      const targetTag = tagId || (armDetectedTags.length ? armDetectedTags[0] : null);
      if (!targetTag) throw new Error('No target object in view to pick.');
      pickUpTag(targetTag);
      return `Initiating pick for tag ${targetTag}`;
    },
    onExplore: async () => {
      if (!hasBase) throw new Error('Base is required for exploration');
      explore();
      return 'Exploring arena';
    },
    onLookForItems: async () => {
      lookForItems();
      return 'Scanning for items at current station';
    },
    onPutInBasket: async () => {
      putInBasket();
      return 'Delivering item to basket';
    },
    onOpenGripper: async () => {
      openGripper();
      return 'Gripper opened';
    },
    onResetToCenter: async () => {
      resetRobotToCentre();
      return 'Reset base to center';
    },
    onSwitchCamera: async (cam: 'base' | 'arm') => {
      selectRemoteCamera(cam === 'arm' ? REMOTE_ARM_CAM : REMOTE_BASE_CAM);
      return `Switched camera to ${cam}`;
    },
    onStop: async () => {
      if (exploring) exploreCancel = true;
      if (navigating) toggleNavigate();
      if (lookingForItems) lookCancel = true;
      return 'Stopped robot actions';
    },
    onAddStation: async ({ name, side, prop, station_type, stl_name, item_count }) => {
      return await addStationFromAi({ name, side, prop, station_type, stl_name, item_count });
    },
    onPrint3dObject: async ({ station_id, stl_name }) => {
      let printer = station_id
        ? STATIONS.find((s) => s.navTag === station_id)
        : STATIONS.find((s) => s.stationType === 'printer');

      if (!printer) {
        // If no printer exists, auto-create one
        const createdMsg = await addStationFromAi({
          name: '3D PRINTER',
          station_type: 'printer',
          stl_name: stl_name || 'gear.stl',
        });
        printer = STATIONS.find((s) => s.stationType === 'printer')!;
        appendLog(createdMsg);
      }

      printer.stationType = 'printer';
      printer.state = 'printing';
      if (stl_name) printer.stlName = stl_name;
      await applyStationLayout(currentLayoutName, STATIONS);
      appendLog(`Station #${printer.navTag} (${printer.label}) started 3D printing ${printer.stlName || 'model'}…`);

      setTimeout(async () => {
        if (printer) {
          printer.state = 'ready';
          await applyStationLayout(currentLayoutName, STATIONS);
          appendLog(`3D print complete at Station #${printer.navTag}! Item ready on print bed.`);
        }
      }, 1200);

      return `3D printer at Station #${printer.navTag} (${printer.label}) is printing ${printer.stlName || 'item'}. Ready shortly!`;
    },
    onRenewDeposit: async ({ station_id }) => {
      const deposit = STATIONS.find((s) => s.navTag === station_id || s.stationType === 'deposit');
      if (!deposit) {
        throw new Error(`No deposit station found with Tag #${station_id}.`);
      }
      deposit.itemCount = deposit.maxItemCount ?? 5;
      await applyStationLayout(currentLayoutName, STATIONS);
      const conf = `Deposit station #${deposit.navTag} (${deposit.label}) replenished to ${deposit.itemCount} items.`;
      appendLog(conf);
      return conf;
    },
    onRunMagicModification: async ({ station_id, output_stl }) => {
      const mod = STATIONS.find((s) => s.navTag === station_id || s.stationType === 'modifier');
      if (!mod) {
        throw new Error(`No magic modification station found with Tag #${station_id}.`);
      }
      mod.state = 'processing';
      if (output_stl) mod.outputStlName = output_stl;
      await applyStationLayout(currentLayoutName, STATIONS);
      appendLog(`Transmutation started at Station #${mod.navTag}…`);

      setTimeout(async () => {
        if (mod) {
          mod.state = 'ready';
          await applyStationLayout(currentLayoutName, STATIONS);
          appendLog(`Transmutation finished at Station #${mod.navTag}! Produced ${mod.outputStlName || 'new item'}.`);
        }
      }, 1200);

      return `Magic modification station #${mod.navTag} (${mod.label}) activated. Producing ${mod.outputStlName || output_stl || 'new object'}.`;
    },
    onGetStationLayout: () => {
      const layoutData = {
        layoutName: currentLayoutName,
        version: '2.0',
        stations: STATIONS.map((s) => ({
          navTag: s.navTag,
          label: s.label,
          description: s.description,
          stationType: s.stationType || 'simple',
          stlName: s.stlName,
          itemCount: s.itemCount,
          maxItemCount: s.maxItemCount,
          state: s.state,
          outputStlName: s.outputStlName,
          actModelId: s.actModelId,
          actDatasetId: s.actDatasetId,
          x: s.x,
          y: s.y,
          prop: s.prop,
          propTag: s.propTag,
          isBoundary: s.isBoundary,
        })),
        actModels: actModels.map((m) => ({ ...m })),
        pathPlan: activePlan
          ? {
              name: activePlan.name,
              status: activePlan.status,
              currentStepIndex: activePlan.currentStepIndex,
              steps: activePlan.steps.map((s) => ({
                id: s.id,
                action: s.action,
                target: s.target,
                modelId: s.modelId,
                tagId: s.tagId,
                duration: s.duration,
                description: s.description,
                status: s.status,
              })),
            }
          : null,
        models: Object.keys(customStlAssets),
      };
      appendLog(`AI inspected current station layout ("${currentLayoutName}")`);
      return layoutData;
    },
    onSetStationLayout: async ({ layout_name, layout_json }) => {
      parseLayoutJson(layout_json, layout_name || 'AI Layout');
      if (uploadLayoutError || !stagedStations) {
        throw new Error(uploadLayoutError || 'Invalid station layout JSON.');
      }
      let finalName = uploadLayoutName.trim() || layout_name?.trim() || 'AI Layout';
      if (layouts[finalName]) {
        let idx = 2;
        while (layouts[`${finalName} (${idx})`]) idx++;
        finalName = `${finalName} (${idx})`;
      }
      layouts[finalName] = stagedStations;
      if (stagedActModels && stagedActModels.length > 0) {
        layoutActModels[finalName] = stagedActModels;
        actModels = [...stagedActModels];
        selectedActModelId = actModels[0]?.id || '';
      }
      layoutMissions[finalName] = stagedPathPlans.length > 0 ? cloneMissions(stagedPathPlans) : cloneMissions(DEFAULT_MISSIONS);
      selectedMissionIdx = 0;
      currentLayoutName = finalName;
      await applyStationLayout(finalName, stagedStations);
      const conf = `Loaded and applied station layout "${finalName}" with ${stagedStations.filter((s) => !s.isBoundary).length} stations and ${layoutMissions[finalName].length} mission(s).`;
      appendLog(conf);
      return conf;
    },
    onRunActModel: async (modelId: string) => {
      return await runActModel(modelId);
    },
    onPlanMissionPath: async ({ plan_name, description, steps, auto_start = true }) => {
      const formattedSteps: PathPlanStep[] = (steps || []).map((s, idx) => ({
        id: `step-${Date.now()}-${idx}`,
        action: s.action,
        target: s.target,
        modelId: s.model_id,
        tagId: s.tag_id,
        duration: s.duration,
        description: s.description || `${s.action} ${s.target || s.model_id || ''}`,
        status: 'pending',
      }));

      const newPlan: PathPlan = {
        id: `mission-ai-${Date.now()}`,
        name: plan_name || 'Gemini Mission Plan',
        description: description || `AI generated mission plan with ${formattedSteps.length} steps`,
        steps: formattedSteps,
        status: 'idle',
        currentStepIndex: 0,
      };

      if (!layoutMissions[currentLayoutName]) {
        layoutMissions[currentLayoutName] = [];
      }
      layoutMissions[currentLayoutName].push(newPlan);
      selectedMissionIdx = layoutMissions[currentLayoutName].length - 1;

      appendLog(`[PathPlanner] AI created mission plan: "${newPlan.name}" with ${formattedSteps.length} steps.`);
      if (auto_start && formattedSteps.length > 0) {
        setTimeout(() => executePathPlan(), 100);
        return `Created mission plan "${newPlan.name}" with ${formattedSteps.length} steps and started execution.`;
      }
      return `Created mission plan "${newPlan.name}" with ${formattedSteps.length} steps.`;
    },
    onGetMissionPlan: () => {
      if (!activePlan) return { message: 'No active mission plan' };
      return {
        name: activePlan.name,
        description: activePlan.description,
        status: activePlan.status,
        currentStepIndex: activePlan.currentStepIndex,
        totalSteps: activePlan.steps.length,
        availableMissions: missions.map((m) => ({ name: m.name, description: m.description, stepsCount: m.steps.length })),
        steps: activePlan.steps.map((s) => ({
          action: s.action,
          target: s.target,
          modelId: s.modelId,
          tagId: s.tagId,
          description: s.description,
          status: s.status,
        })),
      };
    },
    onAbortMissionPlan: async () => {
      abortPathPlan();
      return 'Mission plan aborted.';
    },
  };

  onDestroy(() => {
    cancelAnimationFrame(raf);
    stopCamera();
    disconnectArmCamera();
    if (realConnected) disconnectRobot();
    terminateTesseract();
    try {
      srcMat?.delete();
      grayMat?.delete();
      armSrcMat?.delete();
      armGrayMat?.delete();
    } catch {
      /* ignore */
    }
    srcMat = null;
    grayMat = null;
    armSrcMat = null;
    armGrayMat = null;
    solver?.dispose();
    renderer?.dispose();
    povView?.dispose();
    armPovView?.dispose();
    session?.dispose();
  });
</script>

<svelte:window
  onclick={closeSimContextMenu}
  onkeydown={(e) => {
    if (e.key === 'Escape') closeSimContextMenu();
  }}
/>

{#if simContextMenu.visible}
  <!-- svelte-ignore a11y_click_events_have_key_events -->
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  <div
    class="sim-context-menu"
    style="left: {simContextMenu.x}px; top: {simContextMenu.y}px;"
    onclick={(e) => e.stopPropagation()}
  >
    <div class="sim-context-header">Simulation Actions</div>
    <button
      class="sim-context-item"
      onclick={() => {
        resetRobotToCentre();
        closeSimContextMenu();
      }}
      disabled={!hasBase}
      title="Reset robot base to (0, 0, 0°)"
    >
      <div class="sim-context-text">
        <span class="sim-context-label">Centre Robot</span>
        <span class="sim-context-desc">Reset base to (0, 0, 0°)</span>
      </div>
    </button>
    <button
      class="sim-context-item"
      onclick={() => {
        copyCameraOrientation();
        closeSimContextMenu();
      }}
      title="Copy camera position and target orientation"
    >
      <div class="sim-context-text">
        <span class="sim-context-label">Copy View</span>
        <span class="sim-context-desc">camera.position & target</span>
      </div>
    </button>
    <button
      class="sim-context-item"
      onclick={() => {
        copyRobotPose();
        closeSimContextMenu();
      }}
      disabled={!hasBase}
      title="Copy robot base position and rotation"
    >
      <div class="sim-context-text">
        <span class="sim-context-label">Copy Robot Position and Rotation</span>
        <span class="sim-context-desc">x: {(hasBase ? robotX : 0).toFixed(2)}m, y: {(hasBase ? robotY : 0).toFixed(2)}m, {(hasBase ? robotYawDeg : 0).toFixed(1)}°</span>
      </div>
    </button>
  </div>
{/if}

<div class="sim">
  <div class="viewer" bind:this={wrap}>
    <div class="gridmain" class:real-mode={mode === 'real'}>
      <div class="cell main" class:hidden={mode === 'real'} oncontextmenu={handleSimContextMenu}>
        <select
          class="celllabel cellsel"
          value={selectedRobot}
          aria-label="robot"
          onchange={(e) => onRobotChange((e.currentTarget as HTMLSelectElement).value as RobotId)}
        >
          {#each ROBOTS as r}
            <option value={r.id}>{r.label}</option>
          {/each}
        </select>
        <canvas bind:this={canvas}></canvas>
        {#if hasBase && mode === 'sim'}
          <!-- Inside the world view in sim mode -->
          <div class="joyoverlay">
            <DriveTriad onmove={setDrive} baseForces={baseVel} />
            <RotationControl onrotate={turnRobot} turnForce={baseVel.turn} />
          </div>
        {/if}
      </div>
      <div
        class="cell cell-base-cam"
      >
        <span class="celllabel">
          Base cam{remoteBaseLive || (mode === 'real' && robot.mode === 'remote') ? ' (live)' : camStream ? '' : ' (sim)'} · {baseCamMarkerSummary()}
        </span>
        <!-- svelte-ignore a11y_media_has_caption -->
        <video
          bind:this={previewVideo}
          playsinline
          muted
          class:hidden={!camStream || remoteBaseLive || (mode === 'real' && robot.mode === 'remote')}
        ></video>
        <canvas bind:this={remoteBaseCanvas} class:hidden={!remoteBaseLive && !(mode === 'real' && robot.mode === 'remote')}></canvas>
        {#if !camStream && !remoteBaseLive && !(mode === 'real' && robot.mode === 'remote')}<canvas bind:this={povCanvas}></canvas>{/if}
        <canvas class="cam-overlay-canvas" bind:this={baseCamOverlayCanvas}></canvas>
        {#if hasBase && mode === 'real'}
          <!-- Inside the base cam in real mode -->
          <div class="joyoverlay">
            <DriveTriad onmove={setDrive} baseForces={baseVel} />
            <RotationControl onrotate={turnRobot} turnForce={baseVel.turn} />
          </div>
        {/if}
        {#if !(remoteBaseLive || remoteArmLive || (mode === 'real' && robot.mode === 'remote')) && !remoteBaseLive}
          <div class="camctl">
            <select bind:value={camDeviceId} aria-label="base camera device">
              <option value="">default</option>
              {#each videoDevices as d, i (d.deviceId)}
                <option value={d.deviceId}>{d.label || `camera ${i + 1}`}</option>
              {/each}
            </select>
            {#if camStream}
              <button onclick={reconnectBaseCamera}>Refresh</button>
              <button onclick={disconnectBaseCamera}>Off</button>
            {:else}
              <button class="primary" onclick={reconnectBaseCamera}>Connect</button>
            {/if}
          </div>
        {/if}
      </div>
      <div
        class="cell cell-arm-cam"
      >
        <span class="celllabel">
          Arm cam{remoteArmLive || (mode === 'real' && robot.mode === 'remote') ? ' (live)' : armCamStream ? '' : ' (sim)'}
        </span>
        <!-- svelte-ignore a11y_media_has_caption -->
        <video
          bind:this={armPreview}
          playsinline
          muted
          class:hidden={!armCamStream || remoteArmLive || (mode === 'real' && robot.mode === 'remote')}
        ></video>
        <canvas bind:this={remoteArmCanvas} class:hidden={!remoteArmLive && !(mode === 'real' && robot.mode === 'remote')}></canvas>
        {#if !armCamStream && !remoteArmLive && !(mode === 'real' && robot.mode === 'remote')}<canvas bind:this={armPovCanvas}></canvas>{/if}
        <canvas class="cam-overlay-canvas" bind:this={armCamOverlayCanvas}></canvas>
        {#if !(remoteBaseLive || remoteArmLive || (mode === 'real' && robot.mode === 'remote')) && !remoteArmLive}
          <div class="camctl">
            <select bind:value={armCamDeviceId} aria-label="arm camera device">
              {#each videoDevices as d, i (d.deviceId)}
                <option value={d.deviceId}>{d.label || `camera ${i + 1}`}</option>
              {/each}
            </select>
            {#if armCamStream}
              <button onclick={reconnectArmCamera}>Refresh</button>
              <button onclick={disconnectArmCamera}>Off</button>
            {:else}
              <button class="primary" onclick={connectArmCamera}>Connect</button>
            {/if}
          </div>
        {/if}
      </div>
    </div>
    {#if !ready && !errorMsg}<LoadingScreen message={status} />{/if}
    {#if errorMsg}<div class="overlay err">{errorMsg}</div>{/if}
  </div>

  <div class="panel">
    <div class="sidebrand">
      <div class="brand-titles">
        <h1>Instant Robot</h1>
        <p class="subtitle">SO-101 simulator &amp; control</p>
      </div>
      <select
        class="layout-dropdown"
        value={currentLayoutName}
        onchange={handleLayoutDropdownChange}
        title="Select station layout"
      >
        {#each Object.keys(layouts) as name}
          <option value={name}>{name}</option>
        {/each}
        <option value="__new__">+ New layout…</option>
        <option value="__upload__">Upload layout…</option>
      </select>
    </div>

    <div class="topactions">
      <div class="btngroup">
        <button
          class="btngroup-btn"
          class:active={armLink.connected}
          class:connecting={armLink.busy}
          disabled={!armLink.busy && !armLink.connectLocal && !armLink.connectRemote}
          onclick={() => armLink.toggle()}
          title={armLink.busy ? 'Click to cancel connecting' : (armLink.error ?? 'Connect to the servo bus — local (WebSerial) or remote (WebRTC)')}
        >
          <svg
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            stroke-width="2"
            stroke-linecap="round"
            stroke-linejoin="round"
          >
            {#if armLink.busy}
              <circle cx="12" cy="12" r="10" />
              <line x1="15" y1="9" x2="9" y2="15" />
              <line x1="9" y1="9" x2="15" y2="15" />
            {:else}
              <path d="M12 22v-5" />
              <path d="M9 8V2" />
              <path d="M15 8V2" />
              <path d="M18 8v5a6 6 0 0 1-12 0V8z" />
            {/if}
          </svg>
          {#if armLink.busy}
            <span class="btn-label-connecting">
              <span class="text-default">Connecting…</span>
              <span class="text-hover">Cancel…</span>
            </span>
          {:else}
            <span>{armLink.connected ? 'Disconnect robot' : 'Connect robot'}</span>
          {/if}
        </button>
        <button
          class="btngroup-btn"
          onclick={() => onOpenCalibrate?.()}
          title="Calibrate camera and joints"
        >
          <svg
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            stroke-width="2"
            stroke-linecap="round"
            stroke-linejoin="round"
          >
            <line x1="21" x2="14" y1="4" y2="4" />
            <line x1="10" x2="3" y1="4" y2="4" />
            <line x1="21" x2="12" y1="12" y2="12" />
            <line x1="8" x2="3" y1="12" y2="12" />
            <line x1="21" x2="16" y1="20" y2="20" />
            <line x1="12" x2="3" y1="20" y2="20" />
            <line x1="14" x2="14" y1="2" y2="6" />
            <line x1="8" x2="8" y1="10" y2="14" />
            <line x1="16" x2="16" y1="18" y2="22" />
          </svg>
          <span>Calibrate</span>
        </button>
        <button
          class="btngroup-btn icon-only"
          onclick={() => (settings.open = true)}
          title="Settings"
          aria-label="Settings"
        >
          <svg
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            stroke-width="2"
            stroke-linecap="round"
            stroke-linejoin="round"
          >
            <circle cx="12" cy="12" r="3" />
            <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 1 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 1 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 1 1-2.83-2.83l.06-.06A1.65 1.65 0 0 0 4.6 15a1.65 1.65 0 0 0-1.51-1H3a2 2 0 1 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 1 1 2.83-2.83l.06.06A1.65 1.65 0 0 0 9 4.6a1.65 1.65 0 0 0 1-1.51V3a2 2 0 1 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 1 1 2.83 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 1 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z" />
          </svg>
        </button>
      </div>
    </div>

    {#if copyMsg}
      <!-- svelte-ignore a11y_click_events_have_key_events, a11y_no_static_element_interactions -->
      <div class="status ok" onclick={() => (copyMsg = null)}>{copyMsg}</div>
    {/if}
    <div class="toplinks">
      {#if !manualArm}
        <button class="subtle adv-btn" onclick={() => (manualArm = true)}>Advanced controls</button>
      {:else}
        <button class="subtle adv-btn" onclick={() => (manualArm = false)}>Hide advanced controls</button>
      {/if}
      <span class="sep" aria-hidden="true">·</span>
      <button
        class="subtle log-btn"
        onclick={() => (logOpen = !logOpen)}
        title="View robot log"
      >
        Log {pickLog.length ? `(${pickLog.length})` : ''}
      </button>
      <span class="sep" aria-hidden="true">·</span>
      <button
        class="subtle reset-btn"
        onclick={resetAll}
        title="Reset simulation: restore robot base, arm, places, and all items to initial state"
      >
        Reset
      </button>
      <span class="sep" aria-hidden="true">·</span>
      <button
        class="subtle export-btn"
        onclick={exportCurrentLayout}
        title="Export current station layout to ZIP package"
      >
        Export Layout
      </button>
      <span class="sep" aria-hidden="true">·</span>
      <a
        href="/docs/"
        target="_blank"
        rel="noopener noreferrer"
        class="subtle"
        title="Open documentation"
      >
        Docs
      </a>
    </div>

    {#if logOpen}
      <div class="logbox">
        <div class="loghead">
          <h2>Robot Log</h2>
          <div class="logctrls">
            <button class="linkish" onclick={() => (pickLog = [])}>clear</button>
            <button class="linkish" onclick={() => (logOpen = false)}>hide</button>
          </div>
        </div>

        {#if lastOcrText || lastOcrDesc}
          <div class="ocr-preview-card">
            <div class="ocr-preview-head">
              <span class="ocr-preview-title">Last Station Recognition (Live OCR)</span>
            </div>
            <div class="ocr-preview-item">
              <span class="preview-label">Label (Top):</span>
              {#if lastOcrLabelImgUrl}<img src={lastOcrLabelImgUrl} alt="Warped Label" class="ocr-thumb" />{/if}
              <strong>"{lastOcrText || '(none)'}"</strong>
            </div>
            <div class="ocr-preview-item">
              <span class="preview-label">Desc (Bottom):</span>
              {#if lastOcrDescImgUrl}<img src={lastOcrDescImgUrl} alt="Warped Desc" class="ocr-thumb" />{/if}
              <span>"{lastOcrDesc || '(none)'}"</span>
            </div>
          </div>
        {/if}

        <pre class="logtext">{pickLog.length ? pickLog.join('\n') : 'nothing yet'}</pre>
      </div>
    {/if}

    <!-- Explore: Unified Station Exploration (Simultaneous discovery & OCR) -->
    {#if !hasExplored || exploring}
      <div class="explore-section">
        <div class="explore-row">
          {#if !exploring}
            <button
              class="primary explore-btn"
              disabled={!hasBase || runningAll || navigating}
              onclick={exploreStations}
              title="Full bidirectional sweep to discover stations and read their labels via OCR"
            >
              Explore Stations
            </button>
          {:else}
            <button class="primary explore-btn" disabled>
              Exploring Stations… {Math.round(exploreProgress * 100)}%
            </button>
            <button class="warn" onclick={() => (exploreCancel = true)}>Stop</button>
          {/if}
        </div>
        {#if hasBase && !exploring}
          <p class="explore-hint">
            Run <strong>Explore Stations</strong> to sweep the rack, discover stations, and read names via OCR.
          </p>
        {/if}
      </div>
    {/if}

    <!-- Gemini Voice & Text AI Assistant (placed above map and places) -->
    {#if !exploring}
      <AiChat
        context={aiContext}
        callbacks={aiCallbacks}
        onUploadStl={handleCustomStlUpload}
      />
    {/if}

    <div class="places-section">
      <!-- 2D Map View (pure HTML/SVG interactive station corridor) -->
      <Map2DView
        stations={STATIONS}
        knownTags={knownTags}
        robotX={robotX}
        robotY={robotY}
        robotYawDeg={robotYawDeg}
        hasBase={hasBase}
        navigating={navigating}
        navTagId={navTagId}
        atTag={atTag}
        onSelectStation={(id) => driveToTag(id)}
      />
      <div class="map-footer">
        <div class="map-footer-left">
          <span class="map-tag-pill">MAP</span>
          <span class="map-hint-text">scroll left or right to see more stations</span>
        </div>
      </div>

      <!-- ACT Model Selector below the map and below the label -->
      <div class="act-exec-section">
        <div class="act-section-label">select model to execute</div>
        <div class="act-exec-row">
          <select
            class="act-model-select"
            bind:value={selectedActModelId}
            title={`Selected Model: ${selectedActModelId}\nDataset ID: ${actModels.find((m) => m.id === selectedActModelId)?.datasetId || ''}`}
          >
            {#each actModels as m}
              <option value={m.id}>{m.name}</option>
            {/each}
          </select>
          <button
            class="icon-circle-btn"
            class:running={actRunning}
            disabled={actRunning || armPickBusy || planRunning}
            onclick={() => runActModel(selectedActModelId)}
            title={actRunning ? `Executing ${selectedActModelId}…` : `Run ACT policy`}
            aria-label="Run ACT policy"
          >
            {#if actRunning}
              <span class="btn-spinner"></span>
            {:else}
              <svg class="play-triangle-icon" viewBox="0 0 24 24" fill="currentColor">
                <polygon points="8,5 19,12 8,19" />
              </svg>
            {/if}
          </button>
        </div>
      </div>

      <!-- Missions Section (Transparent background, selectable missions, description, start/pause button) -->
      {#if missions.length > 0}
        <div class="missions-block">
          <div class="missions-section-label">select mission to execute</div>
          <div class="missions-section">
            <div class="missions-header">
              <select
                class="mission-select"
                bind:value={selectedMissionIdx}
                disabled={planRunning}
                title="Select mission"
              >
                {#each missions as m, idx}
                  <option value={idx}>{m.name}</option>
                {/each}
              </select>
              <button
                class="icon-circle-btn"
                class:running={planRunning}
                onclick={togglePlanExecution}
                title={planRunning ? 'Pause mission execution' : (activePlan?.status === 'paused' ? 'Resume mission execution' : 'Start mission execution')}
                aria-label={planRunning ? 'Pause mission' : 'Start mission'}
              >
                {#if planRunning}
                  <svg class="pause-icon" viewBox="0 0 24 24" fill="currentColor">
                    <rect x="6" y="5" width="3.5" height="14" rx="1" />
                    <rect x="14.5" y="5" width="3.5" height="14" rx="1" />
                  </svg>
                {:else}
                  <svg class="play-triangle-icon" viewBox="0 0 24 24" fill="currentColor">
                    <polygon points="8,5 19,12 8,19" />
                  </svg>
                {/if}
              </button>
            </div>

          {#if activePlan?.description}
            <div class="mission-desc">{activePlan.description}</div>
          {/if}

          {#if activePlan && activePlan.steps.length > 0}
            <div class="plan-steps-track">
              {#each activePlan.steps as step, idx}
                <div
                  class="plan-step-node"
                  class:step-done={step.status === 'completed'}
                  class:step-active={step.status === 'active'}
                  class:step-pending={step.status === 'pending'}
                  class:step-failed={step.status === 'failed'}
                  title={step.description}
                >
                  <span class="node-idx">{idx + 1}</span>
                  <span class="node-desc">{step.description}</span>
                  {#if step.status === 'completed'}
                    <span class="node-icon-done">✓</span>
                  {:else if step.status === 'active'}
                    <span class="node-pulse"></span>
                  {/if}
                </div>
              {/each}
            </div>
          {/if}
        </div>
      </div>
    {/if}

      {#if runStep}
        <div class="status {runningAll ? 'warn' : 'ok'}" data-run-step>{runStep}</div>
      {/if}

      {#if navigating}
        <div class="controls"><button class="primary" onclick={toggleNavigate}>Stop</button></div>
      {/if}
      {#if baseLink.error}<div class="status bad">Base: {baseLink.error}</div>{/if}
    </div>

    <!-- Items Section underneath Places (minimalist chip buttons, visible only when items are detected, looking, holding, or printer present) -->
    {#if armDetectedTags.length > 0 || lookingForItems || holdingItem || STATIONS.some(s => s.stationType === 'printer')}
      <div class="items-section">
        <div class="items-chips">
          {#each armDetectedTags as id (id)}
            <button
              class="primary item-chip-btn"
              disabled={armPickBusy}
              onclick={() => pickUpTag(id)}
              title="Pick up item #{id}"
            >
              <span>{armPickBusy && blockTag === id ? 'Picking…' : `Pick up #${id}`}</span>
            </button>
          {/each}
          {#if STATIONS.some(s => s.stationType === 'printer')}
            {@const printer = STATIONS.find(s => s.stationType === 'printer')!}
            <button
              class="item-chip-btn"
              disabled={printer.state === 'printing'}
              onclick={() => aiCallbacks.onPrint3dObject({ station_id: printer.navTag, stl_name: printer.stlName })}
              title="Trigger 3D print cycle at station #{printer.navTag}"
            >
              <span>{printer.state === 'printing' ? 'Printing…' : 'Print Item'}</span>
            </button>
            <button
              class="primary item-chip-btn"
              disabled={armPickBusy || holdingItem || printer.state === 'printing'}
              onclick={() => pickUpPrintedItem(printer)}
              title="Pick up 3D printed item from station #{printer.navTag} bed via IK"
            >
              <span>{armPickBusy && blockTag === (printer.propTag || printer.navTag) ? 'Picking…' : 'Pick up Printed Item'}</span>
            </button>
          {/if}
          {#if lookingForItems}
            <button class="warn item-chip-btn" onclick={() => (lookCancel = true)}>Stop looking</button>
          {/if}
          {#if holdingItem}
            <button class="primary item-chip-btn deliver-btn" disabled={armPickBusy || deliverBusy} onclick={putInBasket}>
              {deliverBusy ? 'Delivering…' : 'Put in basket'}
            </button>
            <button class="item-chip-btn" disabled={armPickBusy} onclick={() => openGripper()}>Open gripper</button>
          {/if}
        </div>
      </div>
    {/if}

    {#if manualArm}
      <details open>
        <summary>
          <span class="sumhead">Arm</span>
          <button class="linkish" onclick={(e) => { e.preventDefault(); manualArm = false; }}>hide</button>
        </summary>
        <div class="graspmode">
          <button class:active={armMode === 'active'} onclick={() => setArmMode('active')}>Active (follow target)</button>
          <button class:active={armMode === 'hold'} onclick={() => setArmMode('hold')}>Hold pose (consistent)</button>
          <button class:active={armMode === 'rest'} onclick={() => setArmMode('rest')}>Rest (home / parked)</button>
        </div>
        <p class="hint">
          {#if armMode === 'active'}
            Active pursues the 3D target in world space. It auto-parks while navigating so the arm doesn't flail.
          {:else if armMode === 'hold'}
            Hold pose locks the arm's current joint angles in place relative to the robot as the base moves.
          {:else}
            Rest parks the arm tucked upright and out of the way.
          {/if}
          {navigating && armMode === 'active' ? ' (Auto-parked during navigation)' : ''}
        </p>
        <h2>End-effector target</h2>
    <div class="sliders">
      {#each ['x', 'y', 'z'] as axis, i (axis)}
        <label>{axis}</label>
        <div class="slider-with-steps">
          <button
            class="step-btn"
            onclick={() => (target[i] = Math.max(-1, Math.min(1, Math.round((target[i] - 0.05) * 1000) / 1000)))}
            title={`Step ${axis.toUpperCase()} -0.05m`}
          >−</button>
          <input type="range" min="-1" max="1" step="0.005" bind:value={target[i]} />
          <button
            class="step-btn"
            onclick={() => (target[i] = Math.max(-1, Math.min(1, Math.round((target[i] + 0.05) * 1000) / 1000)))}
            title={`Step ${axis.toUpperCase()} +0.05m`}
          >+</button>
        </div>
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

    <div class="controls">
      <button class="primary" onclick={raiseToView} disabled={armPickBusy}>Scan for objects position</button>
      <button onclick={targetToCurrentEE}>Move target to current position</button>
      <button onclick={resetRobotToCentre} disabled={!hasBase}>Reset robot to center positioning</button>
      <button onclick={copyArmPose}>Copy arm position</button>
      <button onclick={setRestToCurrent}>Set rest to current</button>
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
      </details>
    {/if}

    {#if mode === 'sim' && manualArm}
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
            <option value="open">1 · Open gripper (hold)</option>
            <option value="approach">2 · Approach (hover above)</option>
            <option value="descend">3 · Descend</option>
            <option value="grasp">4 · Grasp (close)</option>
            <option value="lift">5 · Lift</option>
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

    {#if manualArm}
      <details bind:open={stepsOpen}>
        <summary>Step by step &amp; tuning</summary>
        <div class="pickrow">
          <label for="apicktag">Tag ID</label>
          <input id="apicktag" type="number" bind:value={blockTag} />
          {#each armDetectedTags as id (id)}
            <button class:primary={blockTag === id} onclick={() => (blockTag = id)}>{id}</button>
          {/each}
        </div>
        <div class="controls">
          <button disabled={armPickBusy} onclick={raiseToView} title="move to the board-view pose, then release the arm">
            1 · Raise to view
          </button>
          <button disabled={!armCamReady || armPickBusy} onclick={runApproachStep}>
            2 · Approach
          </button>
          <button disabled={armPickBusy} onclick={() => runGraspStep()} title="shift onto the grasp line, descend, close, lift">
            3 · Grasp & lift
          </button>
          <button disabled={!armCamReady || armPickBusy} onclick={runRollStep}>Square ⟳</button>
          <button disabled={armPickBusy} onclick={runFinalDrop}>Drop</button>
          <button disabled={armPickBusy} onclick={runCloseGripper}>Grip</button>
          <button disabled={armPickBusy} onclick={runLift}>Lift</button>
          <button disabled={armPickBusy} onclick={() => openGripper()}>Open gripper</button>
        </div>
        <div class="pickrow">
          <label for="gripacross" title="line the jaws up with the tag's other axis — pick whichever closes across the object's short side">
            Grip across the tag
          </label>
          <input id="gripacross" type="checkbox" bind:checked={gripAcrossTag} />
          <span class="val"></span>
        </div>
        <div class="pickrow">
          <label for="hoverm" title="how far above the tag the gripper parks, along the tag's normal">
            Hover above tag (m)
          </label>
          <input id="hoverm" type="number" step="0.005" bind:value={hoverM} />
          <span class="val">m</span>
        </div>
        <div class="pickrow">
          <label for="graspleft" title="sideways shift onto the grasp line before descending; positive moves toward the static jaw">
            Grasp shift (m)
          </label>
          <input id="graspleft" type="number" step="0.002" bind:value={graspLeftM} />
          <span class="val">m</span>
        </div>
        <div class="pickrow">
          <label for="finaldrop" title="how far the grasp step descends along the approach axis">Drop (m)</label>
          <input id="finaldrop" type="number" step="0.002" bind:value={finalDropM} />
          <label for="liftby" title="how far to lift once gripped">Lift (m)</label>
          <input id="liftby" type="number" step="0.01" bind:value={liftM} />
        </div>
      </details>
      {#if armPickMsg}<div class="status {armPickBusy ? 'warn' : 'ok'}">{armPickMsg}</div>{/if}
    {/if}

      {#if manualArm && mode === 'real'}
      <div class="realbox">
        <h2>Real arm</h2>
        <p class="hint">
          Solved angles stream to the Feetech servos at ~15 Hz. Needs a joint calibration
          (from the Joint calibration tab) to map sim radians → servo ticks.
        </p>
        <div class="controls">
          <label class="file-btn">
            {jointCalName ?? 'Load joint_calibration.json…'}
            <input type="file" accept="application/json,.json" onchange={loadJointCal} />
          </label>
        </div>
        {#if !realConnected}
          <div class="status warn">Not connected — use <strong>Connect motors</strong> at the top.</div>
        {:else if !jointCal}
          <div class="status warn">Connected — load a joint calibration to start driving.</div>
        {:else}
          <div class="status ok">Driving arm live from the simulator.</div>
        {/if}
        {#if realError}<div class="status bad">Error: {realError}</div>{/if}
      </div>
      {/if}

      {#if hasBase && manualArm}
        <details class="realbox">
          <summary><span class="sumhead">Navigation tuning</span></summary>
          <p class="hint">
            Point the onboard camera at a labeled nav tag (a card with markers 200 &amp; 201
            flanking a label). The app reads the label via OCR and offers it as a drive target.
            Once parked, <em>Reset MuJoCo position</em> snaps the sim robot to the
            matching standoff in front of drawer {shelfSel + 1}.
          </p>
          <div class="pickrow">
            <label for="navmm">Tag size (mm)</label>
            <input id="navmm" type="number" step="5" bind:value={navTagMm} />
            <label for="navsd">Standoff (m)</label>
            <input id="navsd" type="number" step="0.02" bind:value={navStandoff} />
          </div>
          <div class="controls">
            {#if visibleNavTags.length}
              {#each visibleNavTags as id (id)}
                <button
                  class:primary={navigating && navTagId === id}
                  disabled={!hasBase}
                  onclick={() => driveToTag(id)}
                >
                  Drive to tag {id}
                </button>
              {/each}
            {:else}
              <span class="hint">No nav tag in view — aim the onboard camera at one.</span>
            {/if}
          </div>
          <div class="controls">
            {#if navigating}
              <button class="primary" onclick={toggleNavigate}>Stop</button>
            {/if}
            <button
              disabled={!hasBase || nudging}
              onclick={() => nudgeForward()}
              title="creep forward — navigation parks at the standoff, this closes the last bit"
            >
              {nudging ? 'Moving…' : 'Forward a bit'}
            </button>
            <input
              type="number"
              step="100"
              min="100"
              style="width:5rem"
              bind:value={nudgeMs}
              aria-label="nudge duration (ms)"
            />
            <button disabled={!hasBase} onclick={snapSimToStandoff}>Reset MuJoCo position</button>
            <button onclick={() => (navSquareSign = -navSquareSign)} title="if it squares up the wrong way">
              Flip square-up {navSquareSign === 1 ? '+' : '−'}
            </button>
          </div>
          {#if baseLink.error}<div class="status bad">Base: {baseLink.error}</div>{/if}
        </details>
      {/if}

      {#if manualArm}
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
            <option value="open">1 · Open gripper (hold)</option>
            <option value="approach">2 · Approach (hover above)</option>
            <option value="descend">3 · Descend</option>
            <option value="grasp">4 · Grasp (close)</option>
            <option value="lift">5 · Lift</option>
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

      {#if hasBase && knownTags.size}
        <div class="shelves-section">
          {#if !shelfOpen}
            <button class="subtle shelves-btn" onclick={() => (shelfOpen = true)}>
              Shelves…
            </button>
          {:else}
            <div class="realbox shelves-box">
              <div class="shelves-head">
                <h2>Shelves</h2>
                <button class="linkish" onclick={() => (shelfOpen = false)}>hide</button>
              </div>
              <div class="pickrow">
                <label for="shcount">Number of shelves</label>
                <input id="shcount" type="number" min="0" max="8" step="1" bind:value={shelfCount} />
              </div>
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
      {/if}
    {/if}
  </div>
</div>

<!-- Hidden camera source for block detection in real mode. -->
<video bind:this={video} playsinline muted style="display:none"></video>
<video bind:this={armVideo} playsinline muted style="display:none"></video>

{#if showUploadLayoutModal}
  <!-- svelte-ignore a11y_click_events_have_key_events, a11y_no_static_element_interactions -->
  <div class="layout-modal-backdrop" onclick={() => (showUploadLayoutModal = false)}>
    <div class="layout-modal" role="dialog" tabindex="-1" onclick={(e) => e.stopPropagation()}>
      <div class="layout-modal-head">
        <h3>Upload Station Layout</h3>
        <button class="modal-close-btn" onclick={() => (showUploadLayoutModal = false)} title="Close">✕</button>
      </div>

      <div class="layout-modal-body">
        <label class="layout-dropzone" for="layout-file-input">
          <span class="drop-text">Click to choose or drag a <strong>.json</strong> or <strong>.zip</strong> layout file</span>
          <input
            id="layout-file-input"
            type="file"
            accept=".json,.zip,application/json,application/zip"
            onchange={handleLayoutFileInput}
          />
        </label>

        {#if uploadLayoutError}
          <div class="upload-err-msg">{uploadLayoutError}</div>
        {/if}

        {#if stagedStations}
          <div class="staged-box">
            <div class="staged-name-row">
              <label for="staged-name-input">Layout Name</label>
              <input
                id="staged-name-input"
                type="text"
                bind:value={uploadLayoutName}
                placeholder="e.g. Supermarket Layout"
              />
            </div>

            <div class="staged-meta">
              <span class="meta-tag">{stagedStations.filter((s) => !s.isBoundary).length} Stations</span>
              <span class="meta-dist">Rack Span: {Math.abs(stagedStations[stagedStations.length - 1].y - stagedStations[0].y).toFixed(2)}m</span>
            </div>

            <div class="staged-items-list">
              {#each stagedStations.filter((s) => !s.isBoundary) as st}
                <div class="staged-item-pill">
                  <span class="pill-id">#{st.navTag}</span>
                  <span class="pill-name">{st.label || `Station ${st.navTag}`}</span>
                  <span class="pill-prop">({st.prop}{st.propTag ? ` #${st.propTag}` : ''})</span>
                  <span class="pill-y">Y={st.y.toFixed(2)}m</span>
                </div>
              {/each}
            </div>
          </div>
        {/if}
      </div>

      <div class="layout-modal-foot">
        <button class="modal-btn secondary" onclick={() => (showUploadLayoutModal = false)}>Cancel</button>
        <button
          class="modal-btn primary"
          disabled={!stagedStations || stagedStations.length === 0}
          onclick={confirmUploadLayout}
        >
          Import &amp; Apply Layout
        </button>
      </div>
    </div>
  </div>
{/if}

<style>
  .graspmode .ghost {
    opacity: 0.5;
    font-size: 0.85em;
  }
  .sim {
    display: grid;
    grid-template-columns: minmax(0, 3fr) minmax(320px, 1fr);
    gap: 0;
    width: 100vw;
    height: 100vh;
    overflow: hidden;
    align-items: stretch;
  }
  @media (max-width: 1080px) {
    .sim {
      grid-template-columns: 1fr;
      height: auto;
      overflow-y: auto;
    }
  }
  .sidebrand {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.6rem;
    margin-bottom: 0.4rem;
  }
  .brand-titles {
    display: flex;
    flex-direction: column;
    gap: 0.05rem;
  }
  .sidebrand h1 {
    font-size: 1.45rem;
    font-weight: 400;
    margin: 0;
    line-height: 1.15;
    letter-spacing: -0.01em;
  }
  .sidebrand .subtitle {
    margin: 0;
    font-size: 0.78rem;
    color: var(--muted, #6b7280);
    line-height: 1.15;
  }
  .viewer {
    position: relative;
    width: 100%;
    height: 100vh;
    min-height: 100vh;
    border: none;
    border-right: 1px solid var(--line-soft);
    border-radius: 0;
    overflow: hidden;
  }
  @media (max-width: 1080px) {
    .viewer {
      height: 60vh;
      min-height: 24rem;
      border-right: none;
      border-bottom: 1px solid var(--line-soft);
    }
  }
  .panel {
    height: 100vh;
    overflow-y: auto;
    padding: 1rem 1.25rem 3rem;
    box-sizing: border-box;
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
  }
  .panel > * {
    flex-shrink: 0;
  }
  @media (max-width: 1080px) {
    .panel {
      height: auto;
      overflow-y: visible;
    }
  }
  .joyoverlay {
    position: absolute;
    left: 0.6rem;
    bottom: 0.6rem;
    z-index: 5;
    opacity: 0.92;
    display: flex;
    gap: 0.5rem;
    align-items: center;
  }
  /* Layout: Large main 3D world view spanning top row, 2 camera subviews (base cam, arm cam) on bottom row */
  .gridmain {
    position: absolute;
    inset: 0;
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    grid-template-rows: 2.2fr 1fr;
    gap: 2px;
    background: var(--line, #333);
  }
  .cell {
    position: relative;
    overflow: hidden;
    background: #0e1013;
    display: block;
  }
  .cell.main {
    grid-column: 1 / -1;
    grid-row: 1;
  }
  .cell.main.hidden {
    display: none !important;
  }
  .cell-base-cam {
    grid-column: 1;
    grid-row: 2;
  }
  .cell-arm-cam {
    grid-column: 2;
    grid-row: 2;
  }

  /* Connected / Real Robot Mode:
     - Base Cam (left) and Arm Cam (right) side by side */
  .gridmain.real-mode {
    grid-template-columns: 1fr 1fr;
    grid-template-rows: 1fr;
  }
  .gridmain.real-mode .cell-base-cam {
    grid-column: 1;
    grid-row: 1;
  }
  .gridmain.real-mode .cell-arm-cam {
    grid-column: 2;
    grid-row: 1;
  }
  .cell canvas.hidden,
  .cell video.hidden {
    display: none !important;
  }
  .cell canvas,
  .cell video {
    position: absolute;
    inset: 0;
    width: 100%;
    height: 100%;
    display: block;
    object-fit: contain;
    background: #000;
  }
  .cam-overlay-canvas {
    position: absolute;
    inset: 0;
    width: 100% !important;
    height: 100% !important;
    display: block;
    pointer-events: none;
    background: transparent !important;
    z-index: 5;
  }
  .celllabel {
    position: absolute;
    top: 4px;
    left: 6px;
    font-size: 0.68rem;
    letter-spacing: 0.03em;
    color: var(--text-soft, #aaa);
    background: rgba(0, 0, 0, 0.45);
    padding: 1px 6px;
    border-radius: 5px;
    z-index: 2;
    pointer-events: none;
  }
  /* A select dressed as the corner label — it reads as the view's caption until
     you go looking for it, which is where a rarely-changed setting belongs. */
  .cellsel {
    border: 1px solid transparent;
    appearance: none;
    cursor: pointer;
    pointer-events: auto;
    padding-right: 14px;
  }
  .cellsel:hover {
    border-color: var(--line, #666);
    color: var(--text, #eee);
  }
  .cellnote {
    position: absolute;
    bottom: 4px;
    left: 6px;
    font-size: 0.66rem;
    color: var(--text-soft, #999);
    background: rgba(0, 0, 0, 0.45);
    padding: 1px 6px;
    border-radius: 5px;
    z-index: 2;
    pointer-events: none;
  }
  .cellnote.off {
    top: 50%;
    bottom: auto;
    left: 50%;
    transform: translate(-50%, -50%);
  }
  .camctl {
    position: absolute;
    bottom: 4px;
    right: 4px;
    display: flex;
    gap: 3px;
    z-index: 2;
  }
  .camctl select,
  .camctl button {
    font-size: 0.66rem;
    padding: 1px 5px;
    max-width: 7rem;
    /* Overlay controls: compact, not the standard 2.25rem button height. */
    min-height: 0;
  }
  .toplinks {
    display: flex;
    align-items: center;
    justify-content: center;
    flex-wrap: wrap;
    gap: 0.4rem;
    width: 100%;
    margin-top: 0.25rem;
    margin-bottom: 0.65rem;
  }
  .toplinks button.subtle {
    font-size: 0.75rem;
    color: var(--muted);
    padding: 0.15rem 0.35rem;
    min-height: 0;
    line-height: 1.2;
    border-radius: 4px;
    background: transparent;
    border: none;
    cursor: pointer;
  }
  .toplinks button.subtle:hover {
    background: var(--surface-2);
    color: var(--ink);
  }
  .toplinks .sep {
    color: var(--muted);
    font-size: 1rem;
    line-height: 1;
    opacity: 0.65;
    user-select: none;
    display: inline-flex;
    align-items: center;
    justify-content: center;
  }
  .logbox {
    margin-bottom: 0.75rem;
    background: var(--surface-2, #f9fafb);
    border: 1px solid var(--line-soft, #d1d5db);
    border-radius: 6px;
    padding: 0.5rem;
  }
  .logtext {
    max-height: 12rem;
    overflow: auto;
    margin: 0;
    padding: 0.4rem;
    background: var(--surface, #ffffff);
    border: 1px solid var(--line-soft, #d1d5db);
    border-radius: 4px;
    font-size: 0.7rem;
    line-height: 1.4;
    white-space: pre;
  }
  .ocr-preview-item {
    display: flex;
    flex-direction: column;
    gap: 0.2rem;
    font-size: 0.7rem;
  }
  .preview-label {
    font-size: 0.65rem;
    font-weight: 600;
    opacity: 0.75;
  }
  .ocr-thumb {
    max-height: 44px;
    border: 1px solid #ccc;
    border-radius: 3px;
    background: #fff;
    object-fit: contain;
  }
  .explore-section {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    margin-top: 1.85rem;
    margin-bottom: 1.25rem;
    width: 100%;
    text-align: center;
  }
  .explore-row {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 0.5rem;
    width: 100%;
  }
  .explore-btn {
    position: relative;
    z-index: 1;
    min-height: 2.35rem;
    padding: 0.5rem 1.6rem;
    font-size: 0.92rem;
    font-weight: 600;
    border-radius: 6px;
    transition: transform 0.18s ease, box-shadow 0.18s ease;
  }
  .explore-btn::before {
    content: '';
    position: absolute;
    inset: -2px;
    border-radius: 8px;
    background: linear-gradient(
      90deg,
      #3b82f6,
      #8b5cf6,
      #ec4899,
      #f59e0b,
      #10b981,
      #06b6d4,
      #3b82f6
    );
    background-size: 300% 100%;
    z-index: -1;
    opacity: 0;
    transition: opacity 0.25s ease;
  }
  .explore-btn:hover:not(:disabled) {
    transform: translateY(-1px);
    box-shadow: 0 4px 14px rgba(59, 130, 246, 0.35);
  }
  .explore-btn:hover:not(:disabled)::before {
    opacity: 1;
    animation: explore-border-flow 2s linear infinite;
  }
  @keyframes explore-border-flow {
    0% {
      background-position: 0% 50%;
    }
    100% {
      background-position: 300% 50%;
    }
  }
  .explore-hint {
    margin: 0.65rem 0 0 0;
    font-size: 0.78rem;
    color: var(--muted);
    line-height: 1.4;
    text-align: center;
    max-width: 18rem;
  }
  .layout-dropdown {
    align-self: center;
    background: var(--surface, #ffffff);
    color: var(--text, #1e293b);
    border: 1px solid var(--line-soft, #d1d5db);
    border-radius: 5px;
    font-size: 0.74rem;
    font-weight: 600;
    padding: 0.22rem 0.45rem;
    cursor: pointer;
    max-width: 140px;
    text-overflow: ellipsis;
    outline: none;
    transition: border-color 0.15s ease;
  }
  .layout-dropdown:hover {
    border-color: #94a3b8;
  }
  .places-section {
    margin-top: 0.45rem;
    display: flex;
    flex-direction: column;
    gap: 0.65rem;
  }
  .map-footer {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 0.4rem;
    margin: 0.25rem 0 0.1rem 0;
    width: 100%;
  }
  .map-footer-left {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 0.4rem;
  }
  .map-tag-pill {
    font-size: 0.62rem;
    font-weight: 800;
    letter-spacing: 0.05em;
    color: #475569;
    background: rgba(148, 163, 184, 0.22);
    padding: 1.5px 5px;
    border-radius: 3px;
    line-height: 1.2;
  }
  .map-hint-text {
    margin: 0;
    font-size: 0.72rem;
    font-weight: 500;
    color: var(--muted, #64748b);
    text-align: center;
    line-height: 1.2;
  }

  /* ACT Execution Section (Transparent background, circular button with black border and grey triangle, below map label) */
  .act-exec-section {
    display: flex;
    flex-direction: column;
    gap: 0.35rem;
    margin: 0.15rem 0 0.25rem 0;
    background: transparent;
  }
  .act-section-label {
    font-size: 0.68rem;
    font-weight: 600;
    color: #4b5563;
    letter-spacing: 0.02em;
    line-height: 1.2;
    text-align: left;
  }
  .act-exec-row {
    display: flex;
    align-items: center;
    gap: 0.45rem;
    background: transparent;
  }
  .act-model-select {
    flex: 1;
    min-width: 0;
    background: transparent;
    border: 1px solid #9ca3af;
    border-radius: 5px;
    color: #111827;
    font-size: 0.72rem;
    font-weight: 600;
    padding: 4px 8px;
    outline: none;
    cursor: pointer;
    text-overflow: ellipsis;
    white-space: nowrap;
    transition: border-color 0.15s ease;
  }
  .act-model-select:focus {
    border-color: #111111;
  }
  .act-model-select option {
    background: #ffffff;
    color: #111827;
  }

  /* Circular Icon Buttons (Black border, transparent background, grey triangle) */
  .icon-circle-btn {
    width: 24px;
    height: 24px;
    min-width: 24px;
    min-height: 24px;
    border-radius: 50%;
    border: 1.5px solid #111111;
    background: transparent;
    color: #4b5563;
    display: inline-flex;
    align-items: center;
    justify-content: center;
    cursor: pointer;
    flex-shrink: 0;
    padding: 0;
    transition: transform 0.12s ease, background 0.15s ease, color 0.15s ease, border-color 0.15s ease;
  }
  .icon-circle-btn:hover:not(:disabled) {
    background: rgba(0, 0, 0, 0.06);
    color: #111111;
    border-color: #000000;
    transform: scale(1.08);
  }
  .icon-circle-btn:active:not(:disabled) {
    transform: scale(0.95);
    background: rgba(0, 0, 0, 0.12);
  }
  .icon-circle-btn:disabled {
    opacity: 0.35;
    cursor: not-allowed;
  }
  .play-triangle-icon {
    width: 11px;
    height: 11px;
    margin-left: 1.5px; /* optical center for play triangle */
    fill: currentColor;
  }
  .pause-icon {
    width: 11px;
    height: 11px;
    fill: currentColor;
  }
  .btn-spinner {
    width: 11px;
    height: 11px;
    border: 2px solid rgba(0, 0, 0, 0.2);
    border-top-color: #111111;
    border-radius: 50%;
    animation: act-spin 0.6s linear infinite;
  }
  @keyframes act-spin {
    to { transform: rotate(360deg); }
  }

  /* Missions Section (Transparent background, selectable missions, description, circular black-bordered button) */
  .missions-block {
    display: flex;
    flex-direction: column;
    gap: 0.35rem;
    background: transparent;
  }
  .missions-section-label {
    font-size: 0.68rem;
    font-weight: 600;
    color: #4b5563;
    letter-spacing: 0.02em;
    line-height: 1.2;
    text-align: left;
  }
  .missions-section {
    background: transparent;
    border: 1px solid #d1d5db;
    border-radius: 6px;
    padding: 0.5rem 0.6rem;
    display: flex;
    flex-direction: column;
    gap: 0.45rem;
  }
  .missions-header {
    display: flex;
    align-items: center;
    gap: 0.45rem;
  }
  .mission-select {
    flex: 1;
    min-width: 0;
    background: transparent;
    border: 1px solid #9ca3af;
    border-radius: 4px;
    color: #111827;
    font-size: 0.7rem;
    font-weight: 600;
    padding: 4px 8px;
    outline: none;
    cursor: pointer;
    text-overflow: ellipsis;
    white-space: nowrap;
    overflow: hidden;
    transition: border-color 0.15s ease;
  }
  .mission-select:focus {
    border-color: #111111;
  }
  .mission-select option {
    background: #ffffff;
    color: #111827;
  }
  .mission-desc {
    margin: 0;
    font-size: 0.67rem;
    color: #4b5563;
    line-height: 1.35;
    font-style: italic;
    text-align: left;
  }
  .plan-steps-track {
    display: flex;
    flex-direction: column;
    gap: 0.16rem;
    max-height: 100px;
    overflow-y: auto;
  }
  .plan-step-node {
    display: flex;
    align-items: center;
    gap: 0.3rem;
    font-size: 0.64rem;
    color: #4b5563;
    background: rgba(0, 0, 0, 0.03);
    border: 1px solid rgba(0, 0, 0, 0.06);
    border-radius: 4px;
    padding: 2px 5px;
    transition: all 0.15s ease;
  }
  .plan-step-node.step-active {
    background: rgba(139, 92, 246, 0.12);
    border-color: rgba(139, 92, 246, 0.35);
    color: #111827;
    font-weight: 600;
  }
  .plan-step-node.step-done {
    color: #9ca3af;
    text-decoration: line-through;
    opacity: 0.75;
  }
  .node-idx {
    font-size: 0.56rem;
    font-weight: 700;
    color: #a855f7;
    width: 12px;
    text-align: center;
  }
  .node-desc {
    flex: 1;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .node-icon-done {
    color: #22c55e;
    font-size: 0.68rem;
    font-weight: 800;
  }
  .node-pulse {
    width: 6px;
    height: 6px;
    border-radius: 50%;
    background: #a855f7;
    box-shadow: 0 0 6px #a855f7;
    animation: node-blink 1s infinite alternate;
  }
  @keyframes node-blink {
    from { opacity: 0.3; }
    to { opacity: 1; }
  }
  .items-section {
    margin-top: 0.1rem;
    display: flex;
    flex-direction: column;
    gap: 0.1rem;
  }
  .items-chips {
    display: flex;
    flex-wrap: wrap;
    gap: 0.18rem;
    align-items: center;
  }
  .item-chip-btn {
    display: inline-flex;
    align-items: center;
    gap: 0.12rem;
    padding: 0.06rem 0.22rem;
    font-size: 0.48rem;
    font-weight: 600;
    border-radius: 2px;
    line-height: 1.1;
    white-space: nowrap;
  }
  .chip-icon {
    font-size: 0.68em;
  }
  .deliver-btn {
    font-size: 0.48rem;
  }
  .tag-id-pill {
    display: inline-flex;
    align-items: center;
    padding: 0.12rem 0.38rem;
    border-radius: 4px;
    background: rgba(59, 130, 246, 0.12);
    color: #2563eb;
    font-size: 0.72rem;
    font-weight: 700;
    font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
    border: 1px solid rgba(59, 130, 246, 0.3);
    white-space: nowrap;
  }
  .shelves-section {
    margin-top: 0.35rem;
  }
  .shelves-btn {
    font-size: 0.72rem;
    color: var(--muted, #6b7280);
    opacity: 0.75;
    padding: 0.15rem 0.25rem;
    cursor: pointer;
    background: transparent;
    border: none;
    text-decoration: none;
  }
  .shelves-btn:hover {
    opacity: 1;
    color: var(--ink);
    text-decoration: underline;
  }
  .shelves-head {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 0.25rem;
  }
  .shelves-head h2 {
    margin: 0;
  }
  /* Present but easy to skip past — most sessions never need the arm controls. */
  .subtle {
    border: none;
    background: none;
    color: var(--muted, #6b7280);
    opacity: 0.85;
    font-size: 0.8rem;
    padding: 0.15rem 0;
    cursor: pointer;
    text-decoration: none;
    text-align: left;
    display: inline-block;
    transition: opacity 0.15s ease, color 0.15s ease;
  }
  .subtle:hover {
    opacity: 1;
    color: var(--ink);
    text-decoration: none;
  }
  .sumhead {
    font-weight: 600;
    font-size: 0.95rem;
  }
  .linkish {
    border: none;
    background: none;
    color: inherit;
    opacity: 0.6;
    font-size: 0.75rem;
    cursor: pointer;
    text-decoration: underline;
  }
  button.big { padding: 0.45rem 1rem; font-size: 0.9rem; }
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
  .graspmode { display: flex; gap: 0.25rem; }
  .graspmode.wrap { flex-wrap: wrap; }
  .graspmode button {
    flex: 1;
    min-height: 1.9rem; /* segmented row: tighter than a standalone button */
    padding: 0.15rem 0.3rem;
    font-size: 0.78rem;
    cursor: pointer;
  }
  .graspmode button.active {
    background: var(--accent, #3b82f6);
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
  .slider-with-steps {
    display: flex;
    align-items: center;
    gap: 0.35rem;
    width: 100%;
  }
  .slider-with-steps input[type='range'] {
    flex: 1;
    min-width: 0;
  }
  .step-btn {
    padding: 0.1rem 0.45rem;
    font-size: 0.85rem;
    line-height: 1.2;
    font-family: ui-monospace, monospace;
    font-weight: 600;
    border-radius: 4px;
    background: rgba(255, 255, 255, 0.08);
    border: 1px solid rgba(255, 255, 255, 0.15);
    color: var(--text, #eee);
    cursor: pointer;
    user-select: none;
    transition: background 0.15s, border-color 0.15s;
  }
  .step-btn:hover {
    background: rgba(255, 255, 255, 0.2);
    border-color: rgba(255, 255, 255, 0.4);
  }
  .step-btn:active {
    background: rgba(56, 189, 248, 0.35);
    border-color: rgba(56, 189, 248, 0.7);
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

  /* Right-click custom context menu on MuJoCo simulation viewport */
  .sim-context-menu {
    position: fixed;
    z-index: 99999;
    background: rgba(255, 255, 255, 0.95);
    backdrop-filter: blur(14px);
    -webkit-backdrop-filter: blur(14px);
    border: 1px solid var(--line, #111111);
    border-radius: 8px;
    box-shadow: 0 12px 28px -4px rgba(0, 0, 0, 0.22), 0 6px 12px -4px rgba(0, 0, 0, 0.12);
    min-width: 255px;
    padding: 6px;
    display: flex;
    flex-direction: column;
    gap: 3px;
    user-select: none;
    animation: simMenuFadeIn 0.12s cubic-bezier(0.16, 1, 0.3, 1);
  }
  @keyframes simMenuFadeIn {
    from {
      opacity: 0;
      transform: scale(0.95) translateY(-4px);
    }
    to {
      opacity: 1;
      transform: scale(1) translateY(0);
    }
  }
  .sim-context-header {
    font-size: 0.68rem;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: var(--muted, #6b7280);
    padding: 4px 8px 3px;
    border-bottom: 1px solid var(--line-soft, #e5e7eb);
    margin-bottom: 2px;
  }
  .sim-context-item {
    display: flex;
    align-items: center;
    gap: 9px;
    width: 100%;
    padding: 6px 9px;
    background: transparent;
    border: none;
    border-radius: 6px;
    cursor: pointer;
    text-align: left;
    color: var(--ink, #111827);
    transition: background 0.12s ease, color 0.12s ease, transform 0.08s ease;
    font-family: inherit;
    min-height: 0;
  }
  .sim-context-item:hover:not(:disabled) {
    background: rgba(34, 197, 94, 0.14);
    color: #15803d;
  }
  .sim-context-item:active:not(:disabled) {
    transform: scale(0.985);
  }
  .sim-context-item:disabled {
    opacity: 0.45;
    cursor: not-allowed;
  }
  .sim-context-text {
    display: flex;
    flex-direction: column;
    gap: 1px;
    flex-grow: 1;
    overflow: hidden;
  }
  .sim-context-label {
    font-size: 0.82rem;
    font-weight: 500;
    line-height: 1.2;
    white-space: nowrap;
  }
  .sim-context-desc {
    font-size: 0.68rem;
    color: var(--muted, #6b7280);
    line-height: 1.15;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .layout-modal-backdrop {
    position: fixed;
    inset: 0;
    background: rgba(15, 23, 42, 0.6);
    backdrop-filter: blur(3px);
    display: flex;
    align-items: center;
    justify-content: center;
    z-index: 9999;
    padding: 1rem;
  }
  .layout-modal {
    background: var(--surface, #ffffff);
    color: var(--text, #1e293b);
    border: 1px solid var(--line-soft, #cbd5e1);
    border-radius: 12px;
    width: min(100%, 460px);
    box-shadow: 0 20px 40px rgba(0, 0, 0, 0.28);
    display: flex;
    flex-direction: column;
    overflow: hidden;
    animation: modalPop 0.15s ease-out;
  }
  @keyframes modalPop {
    from { opacity: 0; transform: scale(0.96); }
    to { opacity: 1; transform: scale(1); }
  }
  .layout-modal-head {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 0.75rem 1rem;
    border-bottom: 1px solid var(--line-soft, #e2e8f0);
  }
  .layout-modal-head h3 {
    margin: 0;
    font-size: 0.95rem;
    font-weight: 600;
  }
  .modal-close-btn {
    background: none;
    border: none;
    font-size: 1rem;
    cursor: pointer;
    color: var(--muted, #64748b);
    padding: 0.2rem 0.4rem;
    border-radius: 4px;
    line-height: 1;
  }
  .modal-close-btn:hover {
    color: var(--text, #0f172a);
    background: rgba(148, 163, 184, 0.15);
  }
  .layout-modal-body {
    padding: 1rem;
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
  }
  .layout-dropzone {
    border: 2px dashed var(--line-soft, #cbd5e1);
    border-radius: 8px;
    padding: 1.25rem 1rem;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 0.35rem;
    cursor: pointer;
    background: rgba(241, 245, 249, 0.5);
    transition: border-color 0.15s ease, background 0.15s ease;
  }
  .layout-dropzone:hover {
    border-color: #3b82f6;
    background: rgba(59, 130, 246, 0.05);
  }
  .layout-dropzone input {
    display: none;
  }
  .drop-icon {
    font-size: 1.6rem;
  }
  .drop-text {
    font-size: 0.78rem;
    color: var(--muted, #64748b);
    text-align: center;
  }
  .upload-err-msg {
    background: rgba(239, 68, 68, 0.1);
    color: #dc2626;
    border: 1px solid rgba(239, 68, 68, 0.25);
    padding: 0.4rem 0.6rem;
    border-radius: 6px;
    font-size: 0.74rem;
  }
  .staged-box {
    display: flex;
    flex-direction: column;
    gap: 0.5rem;
    background: rgba(248, 250, 252, 0.8);
    border: 1px solid var(--line-soft, #e2e8f0);
    border-radius: 8px;
    padding: 0.75rem;
  }
  .staged-name-row {
    display: flex;
    flex-direction: column;
    gap: 0.25rem;
  }
  .staged-name-row label {
    font-size: 0.72rem;
    font-weight: 600;
    color: var(--muted, #64748b);
  }
  .staged-name-row input {
    padding: 0.3rem 0.5rem;
    font-size: 0.8rem;
    border: 1px solid var(--line-soft, #cbd5e1);
    border-radius: 5px;
    outline: none;
    background: var(--surface, #ffffff);
    color: var(--text, #0f172a);
  }
  .staged-meta {
    display: flex;
    align-items: center;
    justify-content: space-between;
    font-size: 0.72rem;
    color: var(--muted, #64748b);
  }
  .meta-tag {
    font-weight: 700;
    color: #2563eb;
    background: rgba(37, 99, 235, 0.1);
    padding: 1px 6px;
    border-radius: 4px;
  }
  .staged-items-list {
    display: flex;
    flex-wrap: wrap;
    gap: 0.3rem;
    max-height: 120px;
    overflow-y: auto;
  }
  .staged-item-pill {
    display: inline-flex;
    align-items: center;
    gap: 0.25rem;
    font-size: 0.68rem;
    background: #ffffff;
    border: 1px solid var(--line-soft, #cbd5e1);
    border-radius: 4px;
    padding: 0.15rem 0.4rem;
  }
  .pill-id {
    font-weight: 700;
    color: #2563eb;
  }
  .pill-type-badge {
    font-size: 0.62rem;
    font-weight: 600;
    color: #475569;
    background: #f1f5f9;
    padding: 0.5px 4px;
    border-radius: 3px;
  }
  .pill-name {
    font-weight: 600;
  }
  .pill-prop {
    color: #64748b;
  }
  .pill-stl {
    color: #0284c7;
    font-weight: 600;
    font-size: 0.62rem;
  }
  .pill-y {
    color: #94a3b8;
    font-size: 0.62rem;
  }
  .layout-modal-foot {
    display: flex;
    align-items: center;
    justify-content: flex-end;
    gap: 0.5rem;
    padding: 0.75rem 1rem;
    border-top: 1px solid var(--line-soft, #e2e8f0);
    background: rgba(248, 250, 252, 0.5);
  }
  .modal-btn {
    padding: 0.35rem 0.75rem;
    font-size: 0.78rem;
    font-weight: 600;
    border-radius: 6px;
    cursor: pointer;
    transition: all 0.15s ease;
  }
  .modal-btn.secondary {
    background: transparent;
    border: 1px solid var(--line-soft, #cbd5e1);
    color: var(--text, #334155);
  }
  .modal-btn.secondary:hover {
    background: rgba(148, 163, 184, 0.12);
  }
  .modal-btn.primary {
    background: #2563eb;
    border: 1px solid #2563eb;
    color: #ffffff;
  }
  .modal-btn.primary:hover:not(:disabled) {
    background: #1d4ed8;
  }
  .modal-btn.primary:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }
</style>
