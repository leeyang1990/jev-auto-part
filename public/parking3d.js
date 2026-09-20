import { PARKING_BAY } from "./parking-goal.js";
import * as THREE from "/vendor/three/three.module.js";

export function createParkingScene(canvas, { accent }) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: "high-performance" });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.08;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const scene = new THREE.Scene();
  scene.fog = new THREE.FogExp2(0x0b1112, 0.045);

  const camera = new THREE.PerspectiveCamera(34, 1, 0.1, 80);
  const cameraState = { azimuth: Math.PI / 4, elevation: 0.62, distance: 9.2 };
  const cameraTarget = new THREE.Vector3(0, 0, 0);
  const defaultCameraTarget = new THREE.Vector3(0, 0, 0);
  let followCandidates = false;

  scene.add(new THREE.HemisphereLight(0xd8f1ee, 0x182020, 1.45));
  const sun = new THREE.DirectionalLight(0xf3fff9, 3.1);
  sun.position.set(-5, 10, 7);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left = -8;
  sun.shadow.camera.right = 8;
  sun.shadow.camera.top = 8;
  sun.shadow.camera.bottom = -8;
  sun.shadow.bias = -0.0006;
  scene.add(sun);

  const lot = new THREE.Group();
  scene.add(lot);
  buildLot(lot, accent);

  const activeCar = createCar(accent, true);
  activeCar.position.y = 0.18;
  scene.add(activeCar);

  const currentPose = { x: 0, y: 0, heading: 0 };
  let disposed = false;
  let dragging = false;
  let editEnabled = false;
  let editMode = null;
  let poseChangeHandler = null;
  let previousX = 0;
  let previousY = 0;
  let driveGeneration = 0;

  // ---------------------------------------------------------------------------
  // Perception overlay: the occupancy grid the car has actually mapped, the
  // objects it has detected, and where it predicts they will be next.
  // ---------------------------------------------------------------------------
  const MAX_CELLS = 4000;
  const cellMesh = new THREE.InstancedMesh(
    new THREE.PlaneGeometry(0.1, 0.1),
    new THREE.MeshBasicMaterial({ color: 0x6fe3b0, transparent: true, opacity: 0.42, depthWrite: false }),
    MAX_CELLS,
  );
  cellMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  cellMesh.frustumCulled = false;
  cellMesh.count = 0;
  scene.add(cellMesh);

  const predictedMarkers = [];
  for (let index = 0; index < 16; index += 1) {
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.08, 0.12, 16),
      new THREE.MeshBasicMaterial({ color: 0xffc46b, transparent: true, opacity: 0.8, side: THREE.DoubleSide }),
    );
    ring.rotation.x = -Math.PI / 2;
    ring.visible = false;
    scene.add(ring);
    predictedMarkers.push(ring);
  }

  const scratch = new THREE.Object3D();

  // Perception geometry is available for diagnostics, but remains hidden in
  // the normal comparison so no boxes or markers are drawn over a vehicle.
  cellMesh.visible = false;
  for (const marker of predictedMarkers) marker.visible = false;

  // Candidate menu: one polyline per legal and rejected option, drawn the way
  // JevPilot shows its sampled paths, with the chosen one lit up.
  // Candidates are flat ground ribbons anchored to the car and morphed
  // smoothly from one decision's shape into the next.
  const MAX_CANDIDATES = 12;
  const RIBBON_POINTS = 61;
  const NORMAL_ALPHA = 0.65;
  const SELECTED_ALPHA = 0.94;
  // JevPilot's values are sized for a roughly 1.9 m wide vehicle. This demo's
  // compact car is 0.82 m wide, so scale the ribbons with the vehicle instead
  // of copying the world-space widths verbatim.
  const VEHICLE_SCALE = 0.82 / 1.9;
  const NORMAL_HALF_WIDTH = 0.055 * VEHICLE_SCALE;
  // Keep the selected path crisp in this compact scene. The previous 0.2 m
  // ribbon plus 0.5 m glow scaled down poorly and obscured nearby candidates.
  const SELECTED_HALF_WIDTH = 0.078 * VEHICLE_SCALE;
  const SELECTED_COLOUR = 0x168cff;

  const BLEND_TAU = 0.22;

  // Ported from JevPilot's ribbon(): a shader-driven band that fades in just
  // ahead of the bumper and fades out at the
  // far end, so the paths never draw over the vehicle.
  function createRibbon(color, opacity, halfWidth) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(RIBBON_POINTS * 2 * 3), 3).setUsage(THREE.DynamicDrawUsage));
    const progress = [];
    for (let at = 0; at < RIBBON_POINTS; at += 1) progress.push(at / (RIBBON_POINTS - 1), at / (RIBBON_POINTS - 1));
    geometry.setAttribute("progress", new THREE.Float32BufferAttribute(progress, 1));
    const indices = [];
    for (let at = 0; at < RIBBON_POINTS - 1; at += 1) { const n = at * 2; indices.push(n, n + 1, n + 2, n + 1, n + 3, n + 2); }
    geometry.setIndex(indices);
    const material = new THREE.ShaderMaterial({
      uniforms: {
        tint: { value: new THREE.Color(color) },
        alpha: { value: opacity },
        time: { value: 0 },
        pulse: { value: 0 },
        ego: { value: new THREE.Vector3() },
        body: { value: new THREE.Vector2(0.53, 1.02) },
      },
      vertexShader: "attribute float progress; varying float vProgress; varying vec2 vWorld; void main() { vProgress = progress; vWorld = (modelMatrix * vec4(position, 1.0)).xz; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }",
      fragmentShader: "uniform vec3 tint; uniform float alpha; uniform float pulse; uniform vec3 ego; uniform vec2 body; varying float vProgress; varying vec2 vWorld; void main() { vec2 delta = vWorld - ego.xy; float right = dot(delta, vec2(-sin(ego.z), cos(ego.z))); float ahead = dot(delta, vec2(cos(ego.z), sin(ego.z))); if (abs(right) < body.x && abs(ahead) < body.y) discard; float fade = (1.0 - smoothstep(0.72, 1.0, vProgress)) * smoothstep(0.015, 0.08, vProgress); gl_FragColor = vec4(tint, alpha * fade * (1.0 - pulse * 0.4)); }",
      transparent: true,
      depthWrite: false,
      depthTest: true,
      side: THREE.DoubleSide,
      toneMapped: false,
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.frustumCulled = false;
    mesh.userData.halfWidth = halfWidth;
    return mesh;
  }
  const candidateGroup = new THREE.Group();
  scene.add(candidateGroup);
  const candidatePool = [];
  for (let index = 0; index < MAX_CANDIDATES; index += 1) {
    const core = createRibbon(0x4fc3f7, 0.4, NORMAL_HALF_WIDTH);
    core.visible = false;
    core.renderOrder = 3;
    candidateGroup.add(core);
    candidatePool.push({ core, points: null, target: null, width: NORMAL_HALF_WIDTH, base: NORMAL_ALPHA, age: 0 });
  }

  function resample(points, count) {
    if (!points.length) return [];
    if (points.length === 1) return new Array(count).fill(points[0]);
    const cumulative = [0];
    for (let at = 1; at < points.length; at += 1) {
      cumulative.push(cumulative.at(-1) + Math.hypot(points[at].x - points[at - 1].x, points[at].y - points[at - 1].y));
    }
    const total = cumulative.at(-1);
    if (total <= 0.0001) return new Array(count).fill(points[0]);
    const out = [];
    let segment = 1;
    for (let at = 0; at < count; at += 1) {
      const wanted = (at / (count - 1)) * total;
      while (segment < cumulative.length - 1 && cumulative[segment] < wanted) segment += 1;
      const start = cumulative[segment - 1], span = Math.max(0.0001, cumulative[segment] - start);
      const mix = (wanted - start) / span, a = points[segment - 1], b = points[segment];
      out.push({ x: THREE.MathUtils.lerp(a.x, b.x, mix), y: THREE.MathUtils.lerp(a.y, b.y, mix) });
    }
    return out;
  }

  function writeRibbon(mesh, points, alpha) {
    const attribute = mesh.geometry.getAttribute("position");
    const half = mesh.userData.halfWidth;
    for (let at = 0; at < points.length; at += 1) {
      const point = points[at];
      const ahead = points[Math.min(points.length - 1, at + 1)];
      const behind = points[Math.max(0, at - 1)];
      const dx = ahead.x - behind.x, dy = ahead.y - behind.y;
      const length = Math.hypot(dx, dy) || 1;
      const nx = -dy / length, ny = dx / length;
      attribute.setXYZ(at * 2, point.x + nx * half, 0.07, point.y + ny * half);
      attribute.setXYZ(at * 2 + 1, point.x - nx * half, 0.07, point.y - ny * half);
    }
    attribute.needsUpdate = true;
    mesh.geometry.setDrawRange(0, Math.max(0, (points.length - 1) * 6));
    mesh.material.uniforms.alpha.value = alpha;
  }

  // Same bicycle model as the server, integrated from the car's live pose so
  // every option starts at the car and fans out ahead of and behind it.
  const WHEELBASE = 1.05;
  function integrateCandidate(origin, segments) {
    const points = [{ x: origin.x, y: origin.y, heading: origin.heading }];
    let pose = { x: origin.x, y: origin.y, heading: origin.heading };
    for (const [direction, steerDeg, travelDistance] of segments) {
      const travel = Number(travelDistance) || 0;
      const steer = Number(steerDeg) || 0;
      const speedMagnitude = Math.min(0.48, Math.max(0.16, 0.16 + travel * 0.65));
      const duration = travel / speedMagnitude;
      const steps = Math.max(3, Math.ceil(duration / 0.035));
      const dt = duration / steps;
      const speed = (direction < 0 ? -1 : 1) * speedMagnitude;
      const steerRad = steer * Math.PI / 180;
      for (let step = 0; step < steps; step += 1) {
        const theta = pose.heading * Math.PI / 180;
        pose = {
          x: pose.x + speed * Math.cos(theta) * dt,
          y: pose.y + speed * Math.sin(theta) * dt,
          heading: pose.heading + (speed / WHEELBASE) * Math.tan(steerRad) * dt * 180 / Math.PI,
        };
        points.push({ ...pose });
      }
    }
    return points;
  }

  function updateCandidateOverlay(deltaSeconds) {
    canvas.dataset.candidateWidth = String(Math.round(NORMAL_HALF_WIDTH * 1000) / 1000);
    const blend = 1 - Math.exp(-Math.min(deltaSeconds, 0.1) / BLEND_TAU);
    candidatePool.forEach((slot, index) => {
      const item = candidateItems[index];
      if (!item || !(item.segments || []).length) {
        slot.core.visible = false;
        slot.points = null;
        slot.target = null;
        return;
      }
      slot.age += deltaSeconds;
      // Candidate geometry is stored relative to the pose that generated it.
      // Every animation frame reprojects the same shape from the live car pose,
      // exactly like JevPilot's animatePath().
      const target = item.relativePath || [];
      const sameDirection = slot.direction === item.gear;
      if (!sameDirection || !slot.points || slot.points.length !== RIBBON_POINTS) {
        slot.points = target.map((point) => ({ ...point }));
      } else {
        slot.points = slot.points.map((point, at) => ({
          right: point.right + (target[at].right - point.right) * (at === 0 ? 1 : blend),
          ahead: point.ahead + (target[at].ahead - point.ahead) * (at === 0 ? 1 : blend),
        }));
      }
      slot.direction = item.gear;
      const heading = THREE.MathUtils.degToRad(currentPose.heading);
      const cos = Math.cos(heading), sin = Math.sin(heading);
      const worldPoints = slot.points.map((point) => ({
        x: currentPose.x + point.ahead * cos - point.right * sin,
        y: currentPose.y + point.ahead * sin + point.right * cos,
      }));
      // JevPilot leaves the candidate fan visible until the next batch. Keeping
      // the old batch at full strength also avoids the lines disappearing while
      // the slower LLM is deciding.
      const held = 1;
      const alpha = slot.base * held;
      slot.width = item.chosen ? SELECTED_HALF_WIDTH : NORMAL_HALF_WIDTH;
      slot.core.userData.halfWidth = slot.width;
      writeRibbon(slot.core, worldPoints, Math.min(1, (item.chosen ? SELECTED_ALPHA : NORMAL_ALPHA) * held));
      slot.core.renderOrder = item.chosen ? 5 : 3;
      for (const mesh of [slot.core]) {
        mesh.material.uniforms.ego.value.set(currentPose.x, currentPose.y, THREE.MathUtils.degToRad(currentPose.heading));
        mesh.material.uniforms.body.value.set(0.41 + 0.12, 0.875 + 0.15);
        mesh.material.uniforms.time.value = performance.now() / 1000;
      }
      slot.core.visible = true;
    });
  }

  let candidateItems = [];

  function candidateColour(item) {
    if (item.chosen) return SELECTED_COLOUR;
    if (item.blocked) return 0xff6f3c;
    if (!item.legal || item.horizonDanger) return 0xffb300;
    if (item.conflict) return 0xff6f3c;
    return item.gear === "reverse" ? 0xb388ff : 0x4fc3f7;
  }

  function updateCamera() {
    const horizontal = Math.cos(cameraState.elevation) * cameraState.distance;
    camera.position.set(
      cameraTarget.x + Math.sin(cameraState.azimuth) * horizontal,
      Math.sin(cameraState.elevation) * cameraState.distance,
      cameraTarget.z + Math.cos(cameraState.azimuth) * horizontal,
    );
    camera.lookAt(cameraTarget);
  }

  function updateCandidateCamera() {
    if (!followCandidates || !candidateItems.length || dragging) return;
    const chosen = candidateItems.find((item) => item.chosen) || candidateItems[0];
    const projected = integrateCandidate(currentPose, chosen?.displaySegments || chosen?.segments || []);
    const end = projected.at(-1) || currentPose;
    const forward = THREE.MathUtils.degToRad(currentPose.heading);
    const goalX = currentPose.x + (end.x - currentPose.x) * 0.32 + Math.cos(forward) * 0.38;
    const goalZ = currentPose.y + (end.y - currentPose.y) * 0.32 + Math.sin(forward) * 0.38;
    cameraTarget.x += (goalX - cameraTarget.x) * 0.12;
    cameraTarget.z += (goalZ - cameraTarget.z) * 0.12;
    cameraState.distance += (6.4 - cameraState.distance) * 0.1;
    updateCamera();
  }

  function resize() {
    const width = Math.max(1, canvas.clientWidth);
    const height = Math.max(1, canvas.clientHeight);
    const pixelWidth = Math.round(width * renderer.getPixelRatio());
    const pixelHeight = Math.round(height * renderer.getPixelRatio());
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    }
  }

  let lastFrameTime = performance.now();
  function animate() {
    if (disposed) return;
    requestAnimationFrame(animate);
    const now = performance.now();
    const deltaSeconds = Math.min(0.1, (now - lastFrameTime) / 1000);
    lastFrameTime = now;
    updateCandidateOverlay(deltaSeconds);
    updateCandidateCamera();
    resize();
    renderer.render(scene, camera);
  }

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const groundPoint = new THREE.Vector3();
  const headingHandle = new THREE.Group();
  const handleStem = new THREE.Mesh(
    new THREE.CylinderGeometry(0.025, 0.025, 1.15, 10),
    new THREE.MeshBasicMaterial({ color: 0x31b7ff, transparent: true, opacity: 0.9 }),
  );
  handleStem.rotation.z = Math.PI / 2;
  handleStem.position.x = 0.58;
  const handleKnob = new THREE.Mesh(
    new THREE.SphereGeometry(0.14, 18, 12),
    new THREE.MeshBasicMaterial({ color: 0x8bdcff, depthTest: false }),
  );
  handleKnob.position.x = 1.15;
  handleKnob.renderOrder = 9;
  headingHandle.add(handleStem, handleKnob);
  headingHandle.position.y = 0.09;
  headingHandle.visible = false;
  scene.add(headingHandle);

  function updateHeadingHandle() {
    headingHandle.position.x = currentPose.x;
    headingHandle.position.z = currentPose.y;
    headingHandle.rotation.y = -THREE.MathUtils.degToRad(currentPose.heading);
  }

  function pointerOnGround(event) {
    const rect = canvas.getBoundingClientRect();
    pointer.set(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.setFromCamera(pointer, camera);
    return raycaster.ray.intersectPlane(ground, groundPoint) ? groundPoint : null;
  }

  function notifyPoseChange() {
    candidateItems = [];
    for (const slot of candidatePool) slot.core.visible = false;
    followCandidates = false;
    poseChangeHandler?.({ ...currentPose });
  }

  canvas.addEventListener("pointerdown", (event) => {
    if (editEnabled) {
      const point = pointerOnGround(event);
      if (!point) return;
      const frontX = currentPose.x + Math.cos(THREE.MathUtils.degToRad(currentPose.heading)) * 1.15;
      const frontY = currentPose.y + Math.sin(THREE.MathUtils.degToRad(currentPose.heading)) * 1.15;
      const onHandle = Math.hypot(point.x - frontX, point.z - frontY) < 0.38;
      const onCar = Math.hypot(point.x - currentPose.x, point.z - currentPose.y) < 1.05;
      if (!onHandle && !onCar) return;
      editMode = onHandle ? "rotate" : "move";
      dragging = true;
      canvas.setPointerCapture(event.pointerId);
      event.preventDefault();
      return;
    }
    dragging = true;
    previousX = event.clientX;
    previousY = event.clientY;
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!dragging) return;
    if (editEnabled && editMode) {
      const point = pointerOnGround(event);
      if (!point) return;
      if (editMode === "move") {
        currentPose.x = THREE.MathUtils.clamp(point.x, -4.25, 4.25);
        currentPose.y = THREE.MathUtils.clamp(point.z, -3.25, 3.25);
      } else {
        currentPose.heading = Math.atan2(point.z - currentPose.y, point.x - currentPose.x) * 180 / Math.PI;
      }
      applyPose(activeCar, currentPose, { ...currentPose });
      updateHeadingHandle();
      notifyPoseChange();
      return;
    }
    cameraState.azimuth -= (event.clientX - previousX) * 0.008;
    cameraState.elevation = THREE.MathUtils.clamp(cameraState.elevation + (event.clientY - previousY) * 0.006, 0.38, 1.17);
    previousX = event.clientX;
    previousY = event.clientY;
    updateCamera();
  });
  canvas.addEventListener("pointerup", () => { dragging = false; editMode = null; });
  canvas.addEventListener("pointercancel", () => { dragging = false; editMode = null; });
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    cameraState.distance = THREE.MathUtils.clamp(cameraState.distance + event.deltaY * 0.012, 9, 20);
    updateCamera();
  }, { passive: false });

  updateCamera();
  animate();

  return {
    setPose(pose) {
      driveGeneration += 1;
      applyPose(activeCar, currentPose, pose);
      setSteering(activeCar, 0);
      updateHeadingHandle();
      if (!candidateItems.length) {
        followCandidates = false;
        cameraTarget.copy(defaultCameraTarget);
        cameraState.distance = 9.2;
        updateCamera();
      }
    },
    getPose() { return { ...currentPose }; },
    setEditor(enabled, onPoseChange = null) {
      editEnabled = Boolean(enabled);
      poseChangeHandler = editEnabled ? onPoseChange : null;
      headingHandle.visible = editEnabled;
      canvas.dataset.editing = String(editEnabled);
      updateHeadingHandle();
    },
    // Perception data remains available to the decision loop. The normal 3D
    // view deliberately omits diagnostic boxes drawn over parked vehicles.
    setScene(snapshot, truth) {
      const debugPerception = canvas.dataset.debugPerception === "true";
      cellMesh.visible = debugPerception;
      const cells = debugPerception ? (snapshot?.cells || []) : [];
      const count = Math.min(cells.length, MAX_CELLS);
      for (let index = 0; index < count; index += 1) {
        const [x, y] = cells[index];
        scratch.position.set(x, 0.03, y);
        scratch.rotation.set(-Math.PI / 2, 0, 0);
        scratch.scale.set(1, 1, 1);
        scratch.updateMatrix();
        cellMesh.setMatrixAt(index, scratch.matrix);
      }
      cellMesh.count = count;
      cellMesh.instanceMatrix.needsUpdate = true;

      const tracks = snapshot?.tracks || [];
      let markerIndex = 0;
      for (const track of debugPerception ? tracks : []) {
        for (const [x, y] of track.predicted || []) {
          if (markerIndex >= predictedMarkers.length) break;
          const marker = predictedMarkers[markerIndex];
          markerIndex += 1;
          marker.visible = true;
          marker.position.set(x, 0.05, y);
        }
      }
      for (let index = markerIndex; index < predictedMarkers.length; index += 1) predictedMarkers[index].visible = false;

    },

    setCandidates(list, originPose = currentPose) {
      const origin = { ...originPose };
      const originHeading = THREE.MathUtils.degToRad(origin.heading);
      const cos = Math.cos(originHeading), sin = Math.sin(originHeading);
      // Stable spatial ordering keeps nearby paths in the same interpolation
      // slot when a new batch arrives, matching JevPilot's render ordering.
      candidateItems = (list || []).slice(0, MAX_CANDIDATES).sort((a, b) => {
        const directionA = a.gear === "reverse" ? -1 : 1;
        const directionB = b.gear === "reverse" ? -1 : 1;
        const steerA = Number(a.segments?.[0]?.[1]) || 0;
        const steerB = Number(b.segments?.[0]?.[1]) || 0;
        return directionB - directionA || steerA * directionA - steerB * directionB;
      }).map((item) => {
        const direction = item.gear === "reverse" ? -1 : 1;
        let source;
        if (Array.isArray(item.path) && item.path.length >= 2) {
          source = item.path.map(([x, y, heading = origin.heading]) => {
            const angle = THREE.MathUtils.degToRad(heading);
            const bumper = 0.875 * direction;
            return { x: x + Math.cos(angle) * bumper, y: y + Math.sin(angle) * bumper };
          });
        } else {
          source = integrateCandidate(origin, item.displaySegments || item.segments || []);
        }
        const sampled = resample(source, RIBBON_POINTS);
        return {
          ...item,
          relativePath: sampled.map((point) => {
            const dx = point.x - origin.x, dy = point.y - origin.y;
            return { right: -dx * sin + dy * cos, ahead: dx * cos + dy * sin };
          }),
        };
      });
      followCandidates = candidateItems.length > 0;
      if (!followCandidates) {
        cameraTarget.copy(defaultCameraTarget);
        cameraState.distance = 9.2;
        updateCamera();
      }
      candidateItems.forEach((item, index) => {
        const slot = candidatePool[index];
        const colour = candidateColour(item);
        slot.core.material.uniforms.tint.value.setHex(colour);
        slot.base = item.chosen ? SELECTED_ALPHA : (item.legal ? NORMAL_ALPHA : 0.3);
        slot.age = 0;
      });
      canvas.dataset.candidateLines = String(candidateItems.filter((item) => (item.segments || []).length).length);
      const chosen = candidateItems.find((item) => item.chosen);
      canvas.dataset.chosenCandidate = chosen ? chosen.id : "";
      return candidateItems.length;
    },
    async drivePath(path, control, shouldCancel = () => false) {
      if (!Array.isArray(path) || path.length < 2) return true;
      const generation = ++driveGeneration;
      const durationMs = Math.max(180, Number(control?.duration || .8) * 1000);
      const profile = Array.isArray(control?.steeringProfile) ? control.steeringProfile : [];
      const profileDurations = profile.map((item) => Number(item.duration) || 0);
      const startTime = performance.now();
      let last = { ...currentPose };
      setSteering(activeCar, Number(control?.steerDeg) || 0);

      return new Promise((resolve) => {
        function frame(now) {
          if (disposed || generation !== driveGeneration || shouldCancel()) {
            setSteering(activeCar, 0);
            resolve(false);
            return;
          }
          // Some embedded browsers can deliver the first rAF timestamp a few
          // milliseconds before a performance.now() value captured just before
          // scheduling it. Clamp both ends so that the path index never becomes
          // negative and the first frame always starts at path[0].
          const progress = THREE.MathUtils.clamp((now - startTime) / durationMs, 0, 1);
          if (profile.length) {
            const elapsedS = progress * (Number(control?.duration) || .8);
            let accumulated = 0, segment = profile.at(-1);
            for (let at = 0; at < profile.length; at += 1) {
              accumulated += profileDurations[at];
              if (elapsedS <= accumulated) { segment = profile[at]; break; }
            }
            setSteering(activeCar, Number(segment?.steerDeg) || 0);
          }
          const scaled = progress * (path.length - 1);
          const index = Math.min(path.length - 2, Math.floor(scaled));
          const mix = scaled - index;
          const a = path[index], b = path[index + 1];
          const pose = {
            x: THREE.MathUtils.lerp(a.x, b.x, mix),
            y: THREE.MathUtils.lerp(a.y, b.y, mix),
            heading: a.heading + shortestAngle(b.heading - a.heading) * mix,
          };
          const travelled = Math.hypot(pose.x - last.x, pose.y - last.y);
          const direction = Math.sign(Number(control?.speed) || 0);
          rollWheels(activeCar, travelled * direction / .16);
          applyPose(activeCar, currentPose, pose);
          last = pose;
          if (progress < 1) requestAnimationFrame(frame);
          else {
            setSteering(activeCar, 0);
            resolve(true);
          }
        }
        requestAnimationFrame(frame);
      });
    },
    stopDrive() { driveGeneration += 1; setSteering(activeCar, 0); },
    dispose() { disposed = true; renderer.dispose(); },
  };
}

function applyPose(car, currentPose, pose) {
  currentPose.x = Number(pose.x) || 0;
  currentPose.y = Number(pose.y) || 0;
  currentPose.heading = Number(pose.heading) || 0;
  car.position.x = currentPose.x;
  car.position.z = currentPose.y;
  car.rotation.y = -THREE.MathUtils.degToRad(currentPose.heading);
}

function setSteering(car, steerDeg) {
  for (const wheel of car.userData.frontWheels || []) wheel.rotation.y = -THREE.MathUtils.degToRad(steerDeg);
}

function rollWheels(car, radians) {
  for (const wheel of car.userData.allWheels || []) wheel.userData.spin.rotation.z -= radians;
}

function buildLot(group, accent) {
  const asphalt = new THREE.Mesh(
    new THREE.BoxGeometry(10.6, 0.24, 8.2),
    new THREE.MeshStandardMaterial({ color: 0x465254, roughness: 0.91, metalness: 0.02 }),
  );
  asphalt.position.y = -0.13;
  asphalt.receiveShadow = true;
  group.add(asphalt);

  const subBase = new THREE.Mesh(
    new THREE.BoxGeometry(11.15, 0.32, 8.75),
    new THREE.MeshStandardMaterial({ color: 0x202a2b, roughness: 1 }),
  );
  subBase.position.y = -0.36;
  subBase.receiveShadow = true;
  group.add(subBase);

  const lineMaterial = new THREE.MeshStandardMaterial({ color: 0xdce3df, roughness: 0.72, emissive: 0x151918 });
  addParkingLines(group, lineMaterial);
  addCurbs(group);

  const targetMaterial = new THREE.MeshBasicMaterial({ color: accent, transparent: true, opacity: 0.78 });
  const targetFill = new THREE.MeshBasicMaterial({ color: accent, transparent: true, opacity: 0.055, depthWrite: false });
  const { width, length, lineWidth } = PARKING_BAY;
  addBox(group, [0, 0.016, -2.45], [width - lineWidth, 0.012, length - lineWidth], targetFill);
  for (const sign of [-1, 1]) {
    addBox(group, [sign * width / 2, 0.026, -2.45], [lineWidth, 0.018, length], targetMaterial);
    addBox(group, [0, 0.026, -2.45 + sign * length / 2], [width + lineWidth, 0.018, lineWidth], targetMaterial);
  }

  const tanCar = createCar(0xd0b793, false);
  tanCar.position.set(-2.05, 0.18, -2.45);
  tanCar.rotation.y = -Math.PI / 2;
  group.add(tanCar);
  const blueCar = createCar(0x718b9e, false);
  blueCar.position.set(2.1, 0.18, -2.45);
  blueCar.rotation.y = -Math.PI / 2;
  group.add(blueCar);

  addLamp(group, -4.6, -3.45);
  addLamp(group, 4.55, -3.45);
  addLamp(group, -4.6, 3.45);
}

function addParkingLines(group, material) {
  for (const x of [-3.1, -1.05, 1.05, 3.1]) {
    addBox(group, [x, 0.018, -2.45], [0.055, 0.022, 2.2], material);
    addBox(group, [x, 0.018, 2.45], [0.055, 0.022, 2.2], material);
  }
  addBox(group, [0, 0.018, -1.35], [8.25, 0.022, 0.055], material);
  addBox(group, [0, 0.018, 1.35], [8.25, 0.022, 0.055], material);
}

function addCurbs(group) {
  const concrete = new THREE.MeshStandardMaterial({ color: 0xaab5b2, roughness: 0.82 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x20292a, roughness: 0.9 });
  const specs = [
    [[0, .16, -4.18], [11.1, .32, .34]],
    [[0, .16, 4.18], [11.1, .32, .34]],
    [[-5.48, .16, 0], [.34, .32, 8.05]],
    [[5.48, .16, 0], [.34, .32, 8.05]],
  ];
  specs.forEach(([position, scale]) => addBox(group, position, scale, concrete));
  for (let i = -5; i <= 5; i += 1) {
    if (i % 2 === 0) addBox(group, [i, .33, -4.18], [.72, .08, .35], dark);
  }
}

function createCar(color, active) {
  const car = new THREE.Group();
  car.userData.frontWheels = [];
  car.userData.allWheels = [];
  const paint = new THREE.MeshPhysicalMaterial({ color, roughness: 0.3, metalness: 0.18, clearcoat: 0.9, clearcoatRoughness: 0.2 });
  const glass = new THREE.MeshPhysicalMaterial({ color: 0x102126, roughness: 0.1, metalness: 0.32, transmission: 0.12 });
  const tire = new THREE.MeshStandardMaterial({ color: 0x101314, roughness: 0.94 });
  const rim = new THREE.MeshStandardMaterial({ color: 0x9ca8a6, roughness: 0.3, metalness: 0.8 });
  const trim = new THREE.MeshStandardMaterial({ color: 0x172022, roughness: 0.5, metalness: 0.48 });
  const chrome = new THREE.MeshStandardMaterial({ color: 0xc5cfcd, roughness: 0.24, metalness: 0.78 });
  const whiteLight = new THREE.MeshBasicMaterial({ color: 0xf4fbff, toneMapped: false });
  const directionMarker = new THREE.MeshBasicMaterial({ color: 0xf4fbff, toneMapped: false, side: THREE.DoubleSide });
  const redLight = new THREE.MeshBasicMaterial({ color: 0xff1838, toneMapped: false });
  const darkGlass = new THREE.MeshPhysicalMaterial({ color: 0x091317, roughness: 0.08, metalness: 0.24, clearcoat: 1 });

  // Positive X is the nose of the car. The stepped bonnet and rearward cabin
  // make that orientation readable even when the lamps are viewed from above.
  const lower = roundedBox(1.75, 0.32, 0.82, 0.11, paint);
  lower.position.y = 0.31;
  lower.castShadow = true; lower.receiveShadow = true;
  car.add(lower);

  const bonnet = roundedBox(0.72, 0.13, 0.72, 0.08, paint);
  bonnet.position.set(0.5, 0.5, 0);
  bonnet.castShadow = true;
  car.add(bonnet);

  const cabin = roundedBox(0.84, 0.4, 0.7, 0.13, paint);
  cabin.position.set(-0.24, 0.65, 0);
  cabin.castShadow = true;
  car.add(cabin);

  const windshield = new THREE.Mesh(new THREE.BoxGeometry(0.29, 0.25, 0.65), glass);
  windshield.position.set(0.29, 0.67, 0);
  windshield.rotation.z = -0.42;
  car.add(windshield);
  const rearGlass = windshield.clone();
  rearGlass.scale.set(0.72, 0.9, 1);
  rearGlass.position.x = -0.63;
  rearGlass.rotation.z = 0.5;
  car.add(rearGlass);

  // The panoramic roof sits aft of the windshield. Together with the longer
  // painted bonnet it makes the front readable from the bird's-eye camera.
  const roof = roundedBox(0.55, 0.025, 0.58, 0.08, darkGlass);
  roof.position.set(-0.24, 0.855, 0);
  car.add(roof);

  // Dark side glazing forms one clear passenger compartment instead of a
  // second body-coloured box that can be mistaken for another bonnet.
  for (const z of [-0.361, 0.361]) {
    const sideWindow = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.22, 0.018), glass);
    sideWindow.position.set(-0.2, 0.67, z);
    car.add(sideWindow);
    addBox(car, [-0.2, 0.67, z * 1.01], [0.025, 0.24, 0.026], trim);
  }

  for (const x of [-0.56, 0.56]) for (const z of [-0.45, 0.45]) {
    const wheel = new THREE.Group();
    const spin = new THREE.Group();
    const rubber = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.16, 0.13, 18), tire);
    rubber.rotation.x = Math.PI / 2;
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.135, 12), rim);
    hub.rotation.x = Math.PI / 2;
    spin.add(rubber, hub);
    wheel.add(spin);
    wheel.userData.spin = spin;
    wheel.position.set(x, 0.24, z);
    car.add(wheel);
    car.userData.allWheels.push(wheel);
    if (x > 0) car.userData.frontWheels.push(wheel);
  }

  const headlight = active ? whiteLight : new THREE.MeshBasicMaterial({ color: 0xfff1c7, toneMapped: false });
  const tailLight = redLight;
  for (const z of [-0.27, 0.27]) {
    addBox(car, [0.901, 0.43, z], [0.045, 0.12, 0.2], headlight);
    addBox(car, [-0.901, 0.43, z], [0.045, 0.12, 0.2], tailLight);
  }
  // The rear light bar stays visible from high parking-camera angles.
  addBox(car, [-0.906, 0.49, 0], [0.045, 0.045, 0.64], tailLight);
  addBox(car, [0.897, 0.32, 0], [0.035, 0.12, 0.34], trim);
  addBox(car, [0.91, 0.24, 0], [0.035, 0.045, 0.58], chrome);
  addBox(car, [-0.91, 0.25, 0], [0.035, 0.06, 0.56], trim);
  if (active) {
    // A small luminous chevron points toward positive X: the vehicle's nose.
    const marker = new THREE.Shape();
    marker.moveTo(0.2, 0);
    marker.lineTo(-0.12, 0.18);
    marker.lineTo(-0.04, 0);
    marker.lineTo(-0.12, -0.18);
    marker.closePath();
    const markerMesh = new THREE.Mesh(new THREE.ShapeGeometry(marker), directionMarker);
    markerMesh.rotation.x = -Math.PI / 2;
    markerMesh.position.set(0.56, 0.585, 0);
    markerMesh.renderOrder = 4;
    car.add(markerMesh);
  }
  return car;
}

function roundedBox(width, height, depth, radius, material) {
  const shape = new THREE.Shape();
  const x = -width / 2, y = -height / 2;
  shape.moveTo(x + radius, y);
  shape.lineTo(x + width - radius, y);
  shape.quadraticCurveTo(x + width, y, x + width, y + radius);
  shape.lineTo(x + width, y + height - radius);
  shape.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
  shape.lineTo(x + radius, y + height);
  shape.quadraticCurveTo(x, y + height, x, y + height - radius);
  shape.lineTo(x, y + radius);
  shape.quadraticCurveTo(x, y, x + radius, y);
  const geometry = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: true, bevelSize: 0.035, bevelThickness: 0.035, bevelSegments: 2 });
  geometry.center();
  return new THREE.Mesh(geometry, material);
}

function addLamp(group, x, z) {
  const metal = new THREE.MeshStandardMaterial({ color: 0x768382, roughness: 0.42, metalness: 0.72 });
  const glow = new THREE.MeshBasicMaterial({ color: 0xdfffee });
  const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.065, 2.5, 10), metal);
  pole.position.set(x, 1.25, z); pole.castShadow = true; group.add(pole);
  addBox(group, [x + .2, 2.48, z], [.45, .09, .18], metal);
  addBox(group, [x + .25, 2.42, z], [.32, .025, .12], glow);
}

function addBollard(group, x, z) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(.08, .1, .72, 10), new THREE.MeshStandardMaterial({ color: 0xc2cfcc, roughness: .65 }));
  mesh.position.set(x, .36, z); mesh.castShadow = true; group.add(mesh);
}

function addBox(group, position, scale, material) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...scale), material);
  mesh.position.set(...position);
  mesh.castShadow = scale[1] > 0.05;
  mesh.receiveShadow = true;
  group.add(mesh);
  return mesh;
}

function shortestAngle(value) {
  let angle = value % 360;
  if (angle > 180) angle -= 360;
  if (angle < -180) angle += 360;
  return angle;
}
