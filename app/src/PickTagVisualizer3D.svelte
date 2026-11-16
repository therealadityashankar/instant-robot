<script lang="ts">
  import { onMount } from 'svelte';
  import * as THREE from 'three';
  import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
  import { pickCalState } from './lib/pickCalState.svelte';

  interface Props {
    deltaX: number; // in mm
    deltaY: number; // in mm
    deltaZ: number; // in mm
    deltaRoll: number;
    deltaPitch?: number;
    deltaYaw: number;
    dotSpacingMm: number;
    targetTagId: number;
  }

  let {
    deltaX = 0,
    deltaY = 0,
    deltaZ = 0,
    deltaRoll = 0,
    deltaPitch = 0,
    deltaYaw = 0,
    dotSpacingMm = 25,
    targetTagId = 201,
  }: Props = $props();

  let containerEl: HTMLDivElement;
  let canvasEl: HTMLCanvasElement;

  let renderer: THREE.WebGLRenderer | null = null;
  let scene: THREE.Scene;
  let camera: THREE.PerspectiveCamera;
  let animId: number = 0;

  // Scene Groups & Objects
  let gripperGroup: THREE.Group;
  let movingJawPivot: THREE.Group;
  let fixedTagGroup: THREE.Group;
  let dotMesh: THREE.Group;
  let alignLeaderLine: THREE.Line;
  let camLensMesh: THREE.Mesh;
  let leftTipMesh: THREE.Mesh;

  // Live telemetry state for UI overlay
  let gripperWorldX = $state<number>(-65.0);
  let gripperWorldY = $state<number>(8.0);
  let gripperWorldZ = $state<number>(120.0);
  let isLiveVision = $state<boolean>(false);
  let camDepthMm = $state<number>(101.0);
  let alignDistMm = $state<number>(0.0);

  // Orbit state
  let isDragging = false;
  let prevMouseX = 0;
  let prevMouseY = 0;
  let spherical = { radius: 0.32, theta: 0.85, phi: 1.05 };
  const targetLookAt = new THREE.Vector3(0, 0.015, 0.035);

  // Geometry Constants:
  // Camera mount and lens optical center on the outer bracket side (+Y)
  const CAM_LENS_LOCAL = new THREE.Vector3(0.008, 0.065, -0.019);
  // Static left fingertip site in gripper local frame
  const LEFT_TIP_LOCAL = new THREE.Vector3(-0.012, 0.0, -0.104);

  function updateCameraPosition() {
    if (!camera) return;
    camera.position.x =
      targetLookAt.x + spherical.radius * Math.sin(spherical.phi) * Math.cos(spherical.theta);
    camera.position.y =
      targetLookAt.y + spherical.radius * Math.sin(spherical.phi) * Math.sin(spherical.theta);
    camera.position.z = targetLookAt.z + spherical.radius * Math.cos(spherical.phi);
    camera.lookAt(targetLookAt);
  }

  function setViewPreset(preset: 'perspective' | 'top' | 'camera' | 'side') {
    if (preset === 'perspective') {
      spherical = { radius: 0.32, theta: 0.85, phi: 1.05 };
    } else if (preset === 'top') {
      spherical = { radius: 0.26, theta: 0.0, phi: 0.08 };
    } else if (preset === 'camera') {
      // Look from above the camera angle looking down toward the tag
      spherical = { radius: 0.20, theta: 1.57, phi: 0.35 };
    } else if (preset === 'side') {
      spherical = { radius: 0.28, theta: 0.0, phi: Math.PI / 2 - 0.05 };
    }
    updateCameraPosition();
  }

  function onMouseDown(e: MouseEvent) {
    isDragging = true;
    prevMouseX = e.clientX;
    prevMouseY = e.clientY;
  }

  function onMouseMove(e: MouseEvent) {
    if (!isDragging) return;
    const dx = e.clientX - prevMouseX;
    const dy = e.clientY - prevMouseY;
    prevMouseX = e.clientX;
    prevMouseY = e.clientY;

    spherical.theta -= dx * 0.008;
    spherical.phi = Math.max(0.05, Math.min(Math.PI / 2 - 0.02, spherical.phi - dy * 0.008));
    updateCameraPosition();
  }

  function onMouseUp() {
    isDragging = false;
  }

  function onWheel(e: WheelEvent) {
    e.preventDefault();
    spherical.radius = Math.max(0.08, Math.min(0.8, spherical.radius + e.deltaY * 0.0006));
    updateCameraPosition();
  }

  onMount(() => {
    if (!canvasEl || !containerEl) return;

    const width = containerEl.clientWidth || 480;
    const height = 360;

    // 1. Scene & Renderer
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x090d16);

    camera = new THREE.PerspectiveCamera(40, width / height, 0.005, 5);
    camera.up.set(0, 0, 1); // Z is up
    updateCameraPosition();

    renderer = new THREE.WebGLRenderer({
      canvas: canvasEl,
      antialias: true,
      powerPreference: 'high-performance',
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

    // 2. Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.95);
    scene.add(ambientLight);

    const dirLight1 = new THREE.DirectionalLight(0x38bdf8, 1.4);
    dirLight1.position.set(0.3, 0.4, 0.5);
    scene.add(dirLight1);

    const dirLight2 = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight2.position.set(-0.3, -0.3, 0.4);
    scene.add(dirLight2);

    const redLight = new THREE.PointLight(0xef4444, 1.5, 0.4);
    redLight.position.set(-0.02, 0.02, 0.06);
    scene.add(redLight);

    // 3. Table Floor Plane & Grid (at Z=0)
    const grid = new THREE.GridHelper(0.35, 35, 0x0284c7, 0x1e293b);
    grid.rotation.x = Math.PI / 2; // XY plane
    grid.position.set(0, 0, 0);
    scene.add(grid);

    // 4. Fixed Printable Calibration Strip & solvePnP Tag (Anchored at 0, 0, 0)
    fixedTagGroup = new THREE.Group();
    fixedTagGroup.position.set(0, 0, 0);
    scene.add(fixedTagGroup);

    // 4a. Strip Base Plate Graphic (White/dark outline strip)
    const stripPlateGeo = new THREE.PlaneGeometry(0.045, 0.09);
    const stripPlateMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      side: THREE.DoubleSide,
      roughness: 0.6,
    });
    const stripPlate = new THREE.Mesh(stripPlateGeo, stripPlateMat);
    stripPlate.position.set(0, 0.015, 0.0002);
    fixedTagGroup.add(stripPlate);

    // 4b. ArUco Marker Base (30mm x 30mm at 0, 0, 0)
    const tagPlaneGeo = new THREE.PlaneGeometry(0.03, 0.03);
    const tagPlaneMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      side: THREE.DoubleSide,
      roughness: 0.5,
    });
    const tagPlane = new THREE.Mesh(tagPlaneGeo, tagPlaneMat);
    tagPlane.position.set(0, 0, 0.0005);
    fixedTagGroup.add(tagPlane);

    // Inner white square of ArUco marker
    const tagInnerGeo = new THREE.PlaneGeometry(0.02, 0.02);
    const tagInnerMat = new THREE.MeshBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide });
    const tagInner = new THREE.Mesh(tagInnerGeo, tagInnerMat);
    tagInner.position.set(0, 0, 0.0007);
    fixedTagGroup.add(tagInner);

    // Dark ArUco binary matrix pattern simulation
    const tagCodeGeo = new THREE.PlaneGeometry(0.012, 0.012);
    const tagCodeMat = new THREE.MeshBasicMaterial({ color: 0x090d16, side: THREE.DoubleSide });
    const tagCode = new THREE.Mesh(tagCodeGeo, tagCodeMat);
    tagCode.position.set(0, 0, 0.0009);
    fixedTagGroup.add(tagCode);

    // solvePnP coordinate axes on tag (RGB: X=Red/Fwd, Y=Green/Left, Z=Blue/Up)
    const tagAxes = new THREE.AxesHelper(0.025);
    tagAxes.position.set(0, 0, 0.001);
    fixedTagGroup.add(tagAxes);

    // 4c. Target Red Dot (Located on calibration strip at +dotSpacingMm to the left / +Y)
    dotMesh = new THREE.Group();
    const dotRingGeo = new THREE.RingGeometry(0.002, 0.005, 16);
    const dotRingMat = new THREE.MeshBasicMaterial({ color: 0xef4444, side: THREE.DoubleSide });
    const dotRing = new THREE.Mesh(dotRingGeo, dotRingMat);
    dotMesh.add(dotRing);

    const dotCoreGeo = new THREE.CircleGeometry(0.0018, 16);
    const dotCore = new THREE.Mesh(dotCoreGeo, dotRingMat);
    dotMesh.add(dotCore);
    fixedTagGroup.add(dotMesh);

    // 5. Gripper Assembly (Physical Reference Frame that MOVES relative to fixed tag)
    gripperGroup = new THREE.Group();
    scene.add(gripperGroup);

    // Load Real STLs for Gripper & Camera Mount
    const loader = new STLLoader();

    // Material definitions
    const primaryStlMat = new THREE.MeshStandardMaterial({
      color: 0x0284c7,
      roughness: 0.35,
      metalness: 0.4,
    });
    const jawStlMat = new THREE.MeshStandardMaterial({
      color: 0x0ea5e9,
      roughness: 0.3,
      metalness: 0.5,
    });
    const motorStlMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.5,
      metalness: 0.7,
    });
    const camMountStlMat = new THREE.MeshStandardMaterial({
      color: 0x334155,
      roughness: 0.3,
      metalness: 0.65,
    });

    // 5a. Fixed Jaw & Gripper Body STL (wrist_roll_follower)
    loader.load(
      '/models/so101/assets/wrist_roll_follower_so101_v1_simplified.stl',
      (geom) => {
        geom.computeVertexNormals();
        const mesh = new THREE.Mesh(geom, primaryStlMat);
        mesh.rotation.x = Math.PI; // MuJoCo quat="0 1 0 0" (180 deg about X)
        gripperGroup.add(mesh);
      },
      undefined,
      (err) => console.warn('Could not load wrist_roll_follower STL', err)
    );

    // 5b. Moving Jaw STL (attached to STS3215 servo horn pivot)
    movingJawPivot = new THREE.Group();
    movingJawPivot.position.set(0.0202, 0.0188, -0.0234);
    movingJawPivot.rotation.x = Math.PI / 2; // MuJoCo quat="0.707107 0.707107 0 0" (+90 deg about X)
    movingJawPivot.rotation.z = 0.35; // Hinge open angle (~20 deg)
    gripperGroup.add(movingJawPivot);

    loader.load(
      '/models/so101/assets/moving_jaw_so101_v1_simplified.stl',
      (geom) => {
        geom.computeVertexNormals();
        const mesh = new THREE.Mesh(geom, jawStlMat);
        // MuJoCo geom pos="-5.55e-17 -5.55e-17 0.0189"
        mesh.position.set(0, 0, 0.0189);
        movingJawPivot.add(mesh);
      },
      undefined,
      (err) => console.warn('Could not load moving_jaw STL', err)
    );

    // 5c. Gripper Motor STL (sts3215)
    const motorGroup = new THREE.Group();
    motorGroup.position.set(0.0077, 0.0001, -0.0234);
    motorGroup.rotation.x = Math.PI / 2;
    gripperGroup.add(motorGroup);

    loader.load(
      '/models/so101/assets/sts3215_03a_v1_simplified.stl',
      (geom) => {
        geom.computeVertexNormals();
        const mesh = new THREE.Mesh(geom, motorStlMat);
        motorGroup.add(mesh);
      },
      undefined,
      (err) => console.warn('Could not load sts3215 STL', err)
    );

    // 5d. Camera Mount STL (wrist_cam_mount.stl)
    // Scaled from mm to meters, positioned on the outer (+Y) side
    loader.load(
      '/models/so101/assets/wrist_cam_mount.stl',
      (geom) => {
        geom.computeVertexNormals();
        const mesh = new THREE.Mesh(geom, camMountStlMat);
        mesh.scale.set(0.001, 0.001, 0.001);
        mesh.rotation.y = Math.PI; // Aligns bracket on +Y side matching MuJoCo
        gripperGroup.add(mesh);
      },
      undefined,
      (err) => console.warn('Could not load wrist_cam_mount STL', err)
    );

    // 5e. Left Finger Reference Indicator (Red Sphere on the static/left gripper fingertip)
    const leftTipGeo = new THREE.SphereGeometry(0.0035, 16, 16);
    const leftTipMat = new THREE.MeshStandardMaterial({
      color: 0xdc2626,
      emissive: 0xef4444,
      emissiveIntensity: 0.7,
      roughness: 0.2,
      metalness: 0.8,
    });
    leftTipMesh = new THREE.Mesh(leftTipGeo, leftTipMat);
    leftTipMesh.position.copy(LEFT_TIP_LOCAL);
    gripperGroup.add(leftTipMesh);

    // 5f. Physical Camera Lens on the Camera Mount
    const camLensGeo = new THREE.CylinderGeometry(0.0045, 0.0045, 0.006, 16);
    const camLensMat = new THREE.MeshStandardMaterial({
      color: 0x0ea5e9,
      emissive: 0x38bdf8,
      emissiveIntensity: 0.6,
      metalness: 0.85,
      roughness: 0.15,
    });
    camLensMesh = new THREE.Mesh(camLensGeo, camLensMat);
    camLensMesh.rotation.x = Math.PI / 2;
    camLensMesh.position.copy(CAM_LENS_LOCAL);
    gripperGroup.add(camLensMesh);

    // 6. Alignment Leader Line (Moving Left Finger Tip -> Fixed Target Red Dot)
    const alignGeo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0, 0),
    ]);
    const alignMat = new THREE.LineDashedMaterial({
      color: 0xef4444,
      dashSize: 0.004,
      gapSize: 0.002,
    });
    alignLeaderLine = new THREE.Line(alignGeo, alignMat);
    scene.add(alignLeaderLine);

    // 7. Resize Observer
    const ro = new ResizeObserver(() => {
      if (!containerEl || !renderer) return;
      const w = containerEl.clientWidth;
      const h = 360;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    });
    ro.observe(containerEl);

    // 8. Animation Loop
    function animate() {
      animId = requestAnimationFrame(animate);

      // Target Dot is fixed on the strip at (+dotSpacingMm along +Y / left of tag center)
      dotMesh.position.set(0, dotSpacingMm / 1000, 0.0008);

      // ── Gripper Base Rotation with -90 deg rendering offset ───────────────────
      // Base orientation has a -90 deg yaw offset while rendering
      const baseYawRad = -Math.PI / 2;
      const curYawRad = baseYawRad + (deltaYaw * Math.PI) / 180;
      const curPitchRad = (deltaPitch * Math.PI) / 180;
      const curRollRad = (deltaRoll * Math.PI) / 180;

      gripperGroup.rotation.z = curYawRad;
      gripperGroup.rotation.y = curPitchRad;
      gripperGroup.rotation.x = curRollRad;

      // ── Position Gripper in 3D Space ─────────────────────────────────────────
      // Tag is anchored at (0, 0, 0).
      // Camera lens local position is (0.008, 0.065, -0.019).
      // Under -90 deg base yaw rotation, local (x, y) rotates to (y, -x) = (0.065, -0.008).
      // To place camera lens directly above tag (0, 0, 0) at hover height (lens Z ~ 0.101m, gripper Z ~ 0.120m):
      let baseGripX = -0.065;
      let baseGripY = 0.008;
      let baseGripZ = 0.120;

      const liveTag = pickCalState.estimatedTag;
      const isLive = !!(liveTag && liveTag.visible && Date.now() - liveTag.timestamp < 3500);
      isLiveVision = isLive;

      if (isLive && liveTag) {
        if (typeof liveTag.relX === 'number' && typeof liveTag.relY === 'number') {
          // Vision shift from nominal centered aim
          const vShiftX = liveTag.relX - (-0.012);
          const vShiftY = liveTag.relY - 0.024;
          baseGripX -= vShiftX;
          baseGripY -= vShiftY;
          if (typeof liveTag.relZ === 'number') {
            baseGripZ -= liveTag.relZ;
          }
        }
      }

      // Add user calibration offsets (deltaX, deltaY, deltaZ) to gripper position:
      const dx = deltaX / 1000;
      const dy = deltaY / 1000;
      const dz = deltaZ / 1000;

      const curGripX = baseGripX + dx;
      const curGripY = baseGripY + dy;
      const curGripZ = Math.max(0.040, baseGripZ + dz);

      gripperGroup.position.set(curGripX, curGripY, curGripZ);

      // Update reactive UI telemetry coordinates
      gripperWorldX = curGripX * 1000;
      gripperWorldY = curGripY * 1000;
      gripperWorldZ = curGripZ * 1000;

      // World positions of camera lens and left fingertip
      const camLensWorld = new THREE.Vector3();
      camLensMesh.getWorldPosition(camLensWorld);

      const leftTipWorld = new THREE.Vector3();
      leftTipMesh.getWorldPosition(leftTipWorld);

      // Target Dot world position (fixed at 0, dotSpacing, 0)
      const dotWorld = new THREE.Vector3();
      dotMesh.getWorldPosition(dotWorld);

      // Optical depth distance from lens to tag origin (0, 0, 0)
      const tagCenterWorld = new THREE.Vector3(0, 0, 0.001);
      camDepthMm = camLensWorld.distanceTo(tagCenterWorld) * 1000;

      // Update Alignment Leader Line (Moving Left Fingertip -> Fixed Target Dot)
      alignLeaderLine.geometry.setFromPoints([leftTipWorld, dotWorld]);
      alignLeaderLine.computeLineDistances();

      // Real-time Euclidean alignment error distance (in mm)
      alignDistMm = leftTipWorld.distanceTo(dotWorld) * 1000;

      renderer?.render(scene, camera);
    }

    animate();

    return () => {
      cancelAnimationFrame(animId);
      ro.disconnect();
      renderer?.dispose();
    };
  });
</script>

<div class="visualizer-wrapper" bind:this={containerEl}>
  <div class="viewport-header">
    <div class="title-group">
      <span class="viewport-title">STL Gripper & Tag Offset Display</span>
      <span class="subtext">Fixed Tag (0,0,0) · Moving STL Gripper & Camera Holder</span>
    </div>

    <!-- Quick View Presets -->
    <div class="view-presets">
      <button
        class="preset-btn"
        onclick={() => setViewPreset('perspective')}
        title="3D Perspective View"
      >
        3D View
      </button>
      <button
        class="preset-btn"
        onclick={() => setViewPreset('top')}
        title="Top-Down Strip Alignment View"
      >
        Top-Down
      </button>
      <button
        class="preset-btn"
        onclick={() => setViewPreset('camera')}
        title="Camera Sightline View"
      >
        Cam POV
      </button>
      <button
        class="preset-btn"
        onclick={() => setViewPreset('side')}
        title="Side Profile Hover View"
      >
        Side
      </button>
    </div>

    <div class="legend">
      <span class="legend-item tag-raw"><span class="dot raw"></span> Tag at (0,0,0)</span>
      <span class="legend-item cam-holder"><span class="dot holder"></span> Camera Holder</span>
      <span class="legend-item align-dot"><span class="dot red"></span> Target Dot</span>
      <span class="legend-item tip-dot"><span class="dot tip"></span> Left Fingertip</span>
    </div>
  </div>

  <canvas
    bind:this={canvasEl}
    onmousedown={onMouseDown}
    onmousemove={onMouseMove}
    onmouseup={onMouseUp}
    onmouseleave={onMouseUp}
    onwheel={onWheel}
  ></canvas>

  <!-- Live Telemetry HUD Overlay -->
  <div class="hud-overlay">
    <div class="hud-col">
      <div class="hud-header-row">
        <span class="hud-label">Tag Anchor & Vision</span>
        {#if isLiveVision}
          <span class="status-pill live">● LIVE TRACKING</span>
        {:else}
          <span class="status-pill nominal">○ SOLVEPNP POSE</span>
        {/if}
      </div>
      <span class="hud-val">
        Tag #{targetTagId} · [0.0, 0.0, 0.0] mm
      </span>
      <span class="hud-sub">
        Optical Depth: {camDepthMm.toFixed(1)}mm · Dot: +{dotSpacingMm}mm (Y)
      </span>
    </div>

    <div class="hud-col">
      <span class="hud-label">Moving Gripper Pose</span>
      <span class="hud-val accent">
        X: {gripperWorldX >= 0 ? '+' : ''}{gripperWorldX.toFixed(1)}mm · Y: {gripperWorldY >= 0 ? '+' : ''}{gripperWorldY.toFixed(1)}mm · Z: {gripperWorldZ.toFixed(1)}mm
      </span>
      <span class="hud-sub">
        ΔOffset: [{deltaX >= 0 ? '+' : ''}{deltaX.toFixed(1)}, {deltaY >= 0 ? '+' : ''}{deltaY.toFixed(1)}, {deltaZ >= 0 ? '+' : ''}{deltaZ.toFixed(1)}] mm · R:{deltaRoll >= 0 ? '+' : ''}{deltaRoll.toFixed(0)}° P:{(deltaPitch ?? 0) >= 0 ? '+' : ''}{(deltaPitch ?? 0).toFixed(0)}° Y:{deltaYaw >= 0 ? '+' : ''}{deltaYaw.toFixed(0)}°
      </span>
    </div>

    <div class="hud-col">
      <span class="hud-label">Fingertip Alignment Error</span>
      <div class="align-status-wrap">
        <span class="hud-val {alignDistMm < 1.5 ? 'success' : alignDistMm < 5.0 ? 'warning' : 'danger'}">
          {alignDistMm.toFixed(1)} mm
        </span>
        {#if alignDistMm < 1.5}
          <span class="status-pill success">ALIGNED</span>
        {:else if alignDistMm < 5.0}
          <span class="status-pill warning">NEAR</span>
        {:else}
          <span class="status-pill neutral">NUDGE TO ALIGN</span>
        {/if}
      </div>
      <span class="hud-sub">
        Left Finger → Target Dot
      </span>
    </div>
  </div>

  <div class="hint-drag">Drag to rotate 3D view · Scroll to zoom</div>
</div>

<style>
  .visualizer-wrapper {
    position: relative;
    width: 100%;
    background: #090d16;
    border: 1px solid rgba(56, 189, 248, 0.25);
    border-radius: 8px;
    overflow: hidden;
    margin-bottom: 1rem;
    box-shadow: inset 0 2px 10px rgba(0, 0, 0, 0.5), 0 4px 14px rgba(0, 0, 0, 0.25);
  }
  .viewport-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 0.6rem 0.9rem;
    background: rgba(15, 23, 42, 0.95);
    border-bottom: 1px solid rgba(148, 163, 184, 0.15);
    flex-wrap: wrap;
    gap: 0.6rem;
  }
  .title-group {
    display: flex;
    flex-direction: column;
    gap: 0.1rem;
  }
  .viewport-title {
    font-size: 0.88rem;
    font-weight: 600;
    color: #f1f5f9;
  }
  .subtext {
    font-size: 0.72rem;
    color: var(--muted, #94a3b8);
  }
  .view-presets {
    display: flex;
    align-items: center;
    gap: 0.35rem;
  }
  .preset-btn {
    background: rgba(30, 41, 59, 0.8);
    border: 1px solid rgba(148, 163, 184, 0.25);
    color: #e2e8f0;
    border-radius: 4px;
    padding: 0.25rem 0.5rem;
    font-size: 0.72rem;
    font-weight: 500;
    cursor: pointer;
    transition: all 0.15s ease;
  }
  .preset-btn:hover {
    background: rgba(56, 189, 248, 0.2);
    border-color: #38bdf8;
    color: #38bdf8;
  }
  .legend {
    display: flex;
    gap: 0.75rem;
    font-size: 0.72rem;
    color: #94a3b8;
    flex-wrap: wrap;
  }
  .legend-item {
    display: flex;
    align-items: center;
    gap: 0.3rem;
  }
  .dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    display: inline-block;
  }
  .dot.raw { background: #0f172a; border: 1px solid #ffffff; }
  .dot.holder { background: #334155; border: 1px solid #94a3b8; }
  .dot.red { background: #ef4444; }
  .dot.tip { background: #dc2626; border: 1px solid #ffffff; }

  canvas {
    display: block;
    width: 100%;
    height: 360px;
    cursor: grab;
  }
  canvas:active {
    cursor: grabbing;
  }

  .hud-overlay {
    position: absolute;
    bottom: 24px;
    left: 8px;
    right: 8px;
    display: flex;
    justify-content: space-between;
    background: rgba(15, 23, 42, 0.88);
    backdrop-filter: blur(6px);
    border: 1px solid rgba(148, 163, 184, 0.18);
    border-radius: 6px;
    padding: 0.5rem 0.85rem;
    font-family: monospace;
    font-size: 0.75rem;
    pointer-events: none;
    flex-wrap: wrap;
    gap: 0.75rem;
  }
  .hud-col {
    display: flex;
    flex-direction: column;
    gap: 0.12rem;
    min-width: 170px;
  }
  .hud-header-row {
    display: flex;
    align-items: center;
    gap: 0.4rem;
  }
  .hud-label {
    font-size: 0.68rem;
    color: #94a3b8;
    text-transform: uppercase;
    font-weight: 600;
  }
  .hud-val {
    color: #f1f5f9;
    font-weight: 700;
    font-size: 0.82rem;
  }
  .hud-val.accent {
    color: #38bdf8;
  }
  .hud-val.success {
    color: #4ade80;
  }
  .hud-val.warning {
    color: #facc15;
  }
  .hud-val.danger {
    color: #f87171;
  }
  .hud-sub {
    font-size: 0.68rem;
    color: #64748b;
  }
  .status-pill {
    font-size: 0.62rem;
    font-weight: 700;
    padding: 0.1rem 0.35rem;
    border-radius: 4px;
    text-transform: uppercase;
  }
  .status-pill.live {
    background: rgba(34, 197, 94, 0.18);
    color: #4ade80;
    border: 1px solid rgba(34, 197, 94, 0.4);
  }
  .status-pill.nominal {
    background: rgba(56, 189, 248, 0.12);
    color: #38bdf8;
    border: 1px solid rgba(56, 189, 248, 0.3);
  }
  .status-pill.success {
    background: rgba(34, 197, 94, 0.2);
    color: #4ade80;
  }
  .status-pill.warning {
    background: rgba(234, 179, 8, 0.2);
    color: #facc15;
  }
  .status-pill.neutral {
    background: rgba(148, 163, 184, 0.15);
    color: #94a3b8;
  }
  .align-status-wrap {
    display: flex;
    align-items: center;
    gap: 0.4rem;
  }
  .hint-drag {
    position: absolute;
    bottom: 4px;
    right: 12px;
    font-size: 0.65rem;
    color: #64748b;
    pointer-events: none;
  }
</style>
