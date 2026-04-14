(function () {
  const config = window.ROBOT_UI_CONFIG || {};
  const defaults = config.defaults || {};
  const initialStatus = config.initialStatus || null;
  const STATUS_TIMEOUT_MS = 3500;
  const ACTION_TIMEOUT_MS = 15000;
  const LONG_ACTION_TIMEOUT_MS = 20000;
  const ACTION_LOCK_KEY = "robot-ui-action-lock";
  const STATUS_LEADER_KEY = "robot-ui-status-leader";
  const ACTION_LOCK_BUFFER_MS = 2000;
  const STATUS_LEASE_MS = 4000;
  const tabId = "tab-" + Math.random().toString(36).slice(2, 10);
  const modeLabels = {
    N: "Stand",
    F: "Forward",
    B: "Backward",
    L: "Left",
    R: "Right",
    LL: "Turn Left",
    RR: "Turn Right",
  };
  const keyMap = {
    KeyQ: "LL",
    KeyW: "F",
    KeyE: "RR",
    KeyA: "L",
    KeyS: "B",
    KeyD: "R",
    Space: "N",
  };
  const calibrationLegLabels = ["L1", "L2", "L3", "R1", "R2", "R3"];
  const calibrationJointLabels = {
    coxa_deg: "Coxa",
    femur_deg: "Femur",
    tibia_deg: "Tibia",
  };
  const calibrationJointKeys = ["coxa_deg", "femur_deg", "tibia_deg"];
  const xboxCalibrationButtonMap = {
    y: "coxa_deg",
    x: "femur_deg",
    a: "tibia_deg",
  };
  const XBOX_CALIBRATION_POLL_MS = 120;
  const XBOX_CALIBRATION_ARM_MS = 6000;
  const XBOX_CALIBRATION_MAX_STEP_DEG = 1.2;

  let formsBootstrapped = false;
  let lastStatus = null;
  let overrideTimer = null;
  let activeKeyCode = null;
  let cameraFallbackTimer = null;
  let calibrationAutoTimer = null;
  let calibrationRequestInFlight = false;
  let actionInFlight = false;
  let refreshInFlight = false;
  let pageActive = true;
  let statusPollTimer = null;
  let xboxCalibrationPollTimer = null;
  let calibrationSelectedLeg = 0;
  let calibrationSelectedJointKey = "coxa_deg";
  let lastXboxButtonSeq = null;
  let calibrationXboxArmedUntil = 0;

  function el(id) {
    return document.getElementById(id);
  }

  function setText(id, value) {
    const node = el(id);
    if (node) {
      node.textContent = value;
    }
  }

  function setHTML(id, value) {
    const node = el(id);
    if (node) {
      node.innerHTML = value;
    }
  }

  function setHref(id, href, label) {
    const node = el(id);
    if (!node) {
      return;
    }
    node.href = href;
    if (label) {
      node.textContent = label;
    }
  }

  function setValue(id, value) {
    const node = el(id);
    if (node) {
      node.value = value;
    }
  }

  function setChecked(id, value) {
    const node = el(id);
    if (node) {
      node.checked = Boolean(value);
    }
  }

  function setPre(id, lines, fallback) {
    const node = el(id);
    if (node) {
      node.textContent = lines && lines.length ? lines.join("\n") : fallback;
    }
  }

  function toFixedAngle(value) {
    const numeric = Number(value);
    return Number.isFinite(numeric) ? numeric.toFixed(1) : "0.0";
  }

  function calibrationDefaults() {
    return {
      leg_index: 0,
      coxa_deg: Number(defaults.calibration && defaults.calibration.coxa_deg != null ? defaults.calibration.coxa_deg : 0),
      femur_deg: Number(defaults.calibration && defaults.calibration.femur_deg != null ? defaults.calibration.femur_deg : 0),
      tibia_deg: Number(defaults.calibration && defaults.calibration.tibia_deg != null ? defaults.calibration.tibia_deg : 0),
    };
  }

  function calibrationLegs(calibrationState) {
    return calibrationState && Array.isArray(calibrationState.legs) ? calibrationState.legs : [];
  }

  function calibrationLegEntry(calibrationState, legIndex) {
    const legs = calibrationLegs(calibrationState);
    if (legIndex < 0 || legIndex >= legs.length) {
      return null;
    }
    return legs[legIndex];
  }

  function calibrationAnglesFromEntry(entry) {
    const fallback = { coxa_deg: 90, femur_deg: 64.2, tibia_deg: 116.3 };
    const angles = entry && entry.angles_deg ? entry.angles_deg : {};
    return {
      coxa_deg: Number(angles.coxa_deg != null ? angles.coxa_deg : fallback.coxa_deg),
      femur_deg: Number(angles.femur_deg != null ? angles.femur_deg : fallback.femur_deg),
      tibia_deg: Number(angles.tibia_deg != null ? angles.tibia_deg : fallback.tibia_deg),
    };
  }

  function calibrationDraftFromConfig(config) {
    const fallback = calibrationDefaults();
    return {
      coxa_deg: Number(config && config.coxa_deg != null ? config.coxa_deg : fallback.coxa_deg),
      femur_deg: Number(config && config.femur_deg != null ? config.femur_deg : fallback.femur_deg),
      tibia_deg: Number(config && config.tibia_deg != null ? config.tibia_deg : fallback.tibia_deg),
    };
  }

  function calibrationDraftForLeg(calibrationState, legIndex) {
    const config = calibrationState && calibrationState.config;
    if (config && Number(config.leg_index || 0) === legIndex) {
      return calibrationDraftFromConfig(config);
    }
    return calibrationDefaults();
  }

  function currentCalibrationState() {
    return lastStatus && lastStatus.calibration ? lastStatus.calibration : null;
  }

  function currentCalibrationLabel() {
    return calibrationLegLabels[calibrationSelectedLeg] || "L1";
  }

  function currentCalibrationJointLabel() {
    return calibrationJointLabels[calibrationSelectedJointKey] || "Coxa";
  }

  function normalizeXboxButtonSeq(buttonSeq) {
    const source = buttonSeq || {};
    return {
      a: Number(source.a || 0),
      b: Number(source.b || 0),
      x: Number(source.x || 0),
      y: Number(source.y || 0),
      lb: Number(source.lb || 0),
      rb: Number(source.rb || 0),
    };
  }

  function clampNumber(value, lo, hi) {
    return Math.min(Math.max(value, lo), hi);
  }

  function renderCalibrationJointSelection() {
    setText("calibration-selected-joint", currentCalibrationJointLabel());
    document.querySelectorAll("[data-calibration-joint]").forEach(function (node) {
      node.classList.toggle("active", node.dataset.calibrationJoint === calibrationSelectedJointKey);
    });
  }

  function setCalibrationSelectedJoint(jointKey) {
    if (!calibrationJointKeys.includes(jointKey)) {
      return;
    }
    calibrationSelectedJointKey = jointKey;
    renderCalibrationJointSelection();
  }

  function armXboxCalibration(durationMs) {
    calibrationXboxArmedUntil = Date.now() + Math.max(durationMs || 0, XBOX_CALIBRATION_ARM_MS);
  }

  function selectedCalibrationSlider() {
    return el("calibration-" + calibrationSelectedJointKey.replace("_deg", ""));
  }

  function setCalibrationSliderValues(angles) {
    setValue("calibration-coxa", toFixedAngle(angles.coxa_deg));
    setValue("calibration-femur", toFixedAngle(angles.femur_deg));
    setValue("calibration-tibia", toFixedAngle(angles.tibia_deg));
  }

  function selectedCalibrationAngles() {
    const fallback = calibrationDefaults();
    return {
      coxa_deg: el("calibration-coxa") ? Number(el("calibration-coxa").value) : fallback.coxa_deg,
      femur_deg: el("calibration-femur") ? Number(el("calibration-femur").value) : fallback.femur_deg,
      tibia_deg: el("calibration-tibia") ? Number(el("calibration-tibia").value) : fallback.tibia_deg,
    };
  }

  function loadCalibrationLegIntoEditor(legIndex) {
    const state = currentCalibrationState();
    setCalibrationSliderValues(calibrationDraftForLeg(state, legIndex));
  }

  function setCalibrationSelectedLeg(legIndex, loadSliders) {
    if (!Number.isInteger(legIndex) || legIndex < 0 || legIndex >= calibrationLegLabels.length) {
      return;
    }
    calibrationSelectedLeg = legIndex;
    if (loadSliders !== false) {
      loadCalibrationLegIntoEditor(legIndex);
    }
    renderCalibrationLegSelection(currentCalibrationState());
    updateCommandPreviews();
  }

  function buildCalibrationFrame(payload, calibrationState) {
    const frame = [];
    for (let legIndex = 0; legIndex < calibrationLegLabels.length; legIndex += 1) {
      const entry = calibrationLegEntry(calibrationState, legIndex);
      const angles = calibrationAnglesFromEntry(entry);
      frame.push(
        toFixedAngle(angles.coxa_deg),
        toFixedAngle(angles.femur_deg),
        toFixedAngle(angles.tibia_deg)
      );
    }
    const start = payload.leg_index * 3;
    const savedEntry = calibrationLegEntry(calibrationState, payload.leg_index);
    const savedAngles = calibrationAnglesFromEntry(savedEntry);
    frame[start] = toFixedAngle(clampAngle(savedAngles.coxa_deg + Number(payload.coxa_deg || 0)));
    frame[start + 1] = toFixedAngle(clampAngle(savedAngles.femur_deg + Number(payload.femur_deg || 0)));
    frame[start + 2] = toFixedAngle(clampAngle(savedAngles.tibia_deg + Number(payload.tibia_deg || 0)));
    return frame;
  }

  function selectedCalibrationAppliedPose(payload, calibrationState) {
    const frame = buildCalibrationFrame(payload, calibrationState);
    const start = payload.leg_index * 3;
    return frame.slice(start, start + 3);
  }

  function calibrationSafeRange(calibrationState) {
    const source = calibrationState && calibrationState.servo_safe_range_deg ? calibrationState.servo_safe_range_deg : {};
    return {
      min_deg: Number(source.min_deg != null ? source.min_deg : 30),
      max_deg: Number(source.max_deg != null ? source.max_deg : 150),
      neutral_deg: Number(source.neutral_deg != null ? source.neutral_deg : 90),
      pwm_min_us: Number(source.pwm_min_us != null ? source.pwm_min_us : 1000),
      pwm_neutral_us: Number(source.pwm_neutral_us != null ? source.pwm_neutral_us : 1520),
      pwm_max_us: Number(source.pwm_max_us != null ? source.pwm_max_us : 2000),
    };
  }

  function calibrationSafetySnapshot(payload, calibrationState) {
    const applied = selectedCalibrationAppliedPose(payload, calibrationState).map(function (value) {
      return Number(value);
    });
    const safeRange = calibrationSafeRange(calibrationState);
    const violations = applied
      .map(function (value, jointIndex) {
        if (value < safeRange.min_deg) {
          return calibrationJointLabels[calibrationJointKeys[jointIndex]] + " " +
            toFixedAngle(value) + "\u00b0 < " + toFixedAngle(safeRange.min_deg) + "\u00b0";
        }
        if (value > safeRange.max_deg) {
          return calibrationJointLabels[calibrationJointKeys[jointIndex]] + " " +
            toFixedAngle(value) + "\u00b0 > " + toFixedAngle(safeRange.max_deg) + "\u00b0";
        }
        return "";
      })
      .filter(Boolean);
    return {
      applied,
      safeRange,
      violations,
      safe: violations.length === 0,
    };
  }

  function updateCalibrationSafetyReadout(payload, calibrationState) {
    const safety = calibrationSafetySnapshot(payload, calibrationState);
    if (el("calibration-safe-window")) {
      setText(
        "calibration-safe-window",
        toFixedAngle(safety.safeRange.min_deg) + " .. " + toFixedAngle(safety.safeRange.max_deg) +
          " deg (" + String(Math.round(safety.safeRange.pwm_min_us)) + " / " +
          String(Math.round(safety.safeRange.pwm_neutral_us)) + " / " +
          String(Math.round(safety.safeRange.pwm_max_us)) + " us)"
      );
    }
    if (el("calibration-safe-state")) {
      setText(
        "calibration-safe-state",
        safety.safe
          ? "Within current S6510 runtime window."
          : "Outside runtime window: " + safety.violations.join(" · ")
      );
    }
  }

  function escapeHTML(value) {
    return String(value || "")
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;")
      .replaceAll('"', "&quot;")
      .replaceAll("'", "&#39;");
  }

  function formatDuration(seconds) {
    const amount = Number(seconds || 0);
    if (!Number.isFinite(amount) || amount <= 0) {
      return "0.0s";
    }
    if (amount < 60) {
      return amount.toFixed(1) + "s";
    }
    const minutes = Math.floor(amount / 60);
    return minutes + "m " + (amount - minutes * 60).toFixed(1) + "s";
  }

  function formatTimestamp(seconds) {
    if (!seconds) {
      return "Not saved yet.";
    }
    return new Date(Number(seconds) * 1000).toLocaleString();
  }

  function calibrationProfileSummary(calibration) {
    const savedCount = Number(calibration && calibration.saved_count ? calibration.saved_count : 0);
    const totalCount = calibrationLegLabels.length;
    const prefix = (calibration && calibration.complete ? "Complete" : "In progress") +
      " · " + String(savedCount) + " / " + String(totalCount) + " legs saved";
    if (calibration && calibration.profile_updated_at) {
      return prefix + " · updated " + formatTimestamp(calibration.profile_updated_at);
    }
    return prefix + (savedCount > 0 ? "" : " · waiting for first save");
  }

  function syncCalibrationEditorFromStatus(data) {
    if (!data || !data.calibration || !data.calibration.config) {
      return;
    }
    calibrationSelectedLeg = Number(data.calibration.config.leg_index || 0);
    loadCalibrationLegIntoEditor(calibrationSelectedLeg);
    updateCommandPreviews();
  }

  function setStatusChip(id, running, runningText, stoppedText, idleText) {
    const node = el(id);
    if (!node) {
      return;
    }
    node.classList.remove("running", "stopped", "idle");
    if (running === true) {
      node.classList.add("running");
      node.textContent = runningText;
      return;
    }
    if (running === false) {
      node.classList.add("stopped");
      node.textContent = stoppedText;
      return;
    }
    node.classList.add("idle");
    node.textContent = idleText;
  }

  function showToast(message, type) {
    const stack = el("toast-stack");
    if (!stack) {
      return;
    }
    const toast = document.createElement("div");
    toast.className = "toast " + (type || "info");
    toast.textContent = message;
    stack.appendChild(toast);
    window.setTimeout(() => {
      toast.remove();
    }, 2800);
  }

  function safeStorageGet(key) {
    if (!window.localStorage) {
      return null;
    }
    try {
      const raw = window.localStorage.getItem(key);
      return raw ? JSON.parse(raw) : null;
    } catch (error) {
      return null;
    }
  }

  function safeStorageSet(key, value) {
    if (!window.localStorage) {
      return;
    }
    try {
      window.localStorage.setItem(key, JSON.stringify(value));
    } catch (error) {
      // Ignore storage failures in restrictive browser modes.
    }
  }

  function safeStorageRemove(key) {
    if (!window.localStorage) {
      return;
    }
    try {
      window.localStorage.removeItem(key);
    } catch (error) {
      // Ignore storage failures in restrictive browser modes.
    }
  }

  function currentActionLock() {
    const lock = safeStorageGet(ACTION_LOCK_KEY);
    if (!lock || !lock.owner || !lock.expires_at) {
      return null;
    }
    if (Number(lock.expires_at) <= Date.now()) {
      safeStorageRemove(ACTION_LOCK_KEY);
      return null;
    }
    return lock;
  }

  function currentActionLockOwner() {
    const lock = currentActionLock();
    if (!lock || lock.owner === tabId) {
      return null;
    }
    return lock.owner;
  }

  function syncActionButtons() {
    setActionButtonsDisabled(Boolean(actionInFlight || currentActionLockOwner()));
  }

  function acquireActionLock(timeoutMs) {
    const leaseMs = Math.max(timeoutMs || ACTION_TIMEOUT_MS, ACTION_TIMEOUT_MS) + ACTION_LOCK_BUFFER_MS;
    const existing = currentActionLock();
    if (existing && existing.owner !== tabId) {
      return false;
    }
    safeStorageSet(ACTION_LOCK_KEY, {
      owner: tabId,
      expires_at: Date.now() + leaseMs,
    });
    const confirmed = currentActionLock();
    return Boolean(confirmed && confirmed.owner === tabId);
  }

  function releaseActionLock() {
    const existing = currentActionLock();
    if (existing && existing.owner === tabId) {
      safeStorageRemove(ACTION_LOCK_KEY);
    }
  }

  function currentStatusLeader() {
    const leader = safeStorageGet(STATUS_LEADER_KEY);
    if (!leader || !leader.owner || !leader.expires_at) {
      return null;
    }
    if (Number(leader.expires_at) <= Date.now()) {
      safeStorageRemove(STATUS_LEADER_KEY);
      return null;
    }
    return leader;
  }

  function claimStatusLeader() {
    if (!pageActive || (typeof document !== "undefined" && document.visibilityState === "hidden")) {
      return false;
    }
    const leader = currentStatusLeader();
    if (leader && leader.owner !== tabId) {
      return false;
    }
    safeStorageSet(STATUS_LEADER_KEY, {
      owner: tabId,
      expires_at: Date.now() + STATUS_LEASE_MS,
    });
    const confirmed = currentStatusLeader();
    return Boolean(confirmed && confirmed.owner === tabId);
  }

  function releaseStatusLeader() {
    const leader = currentStatusLeader();
    if (leader && leader.owner === tabId) {
      safeStorageRemove(STATUS_LEADER_KEY);
    }
  }

  function setActionButtonsDisabled(disabled) {
    [
      "deploy-manual",
      "deploy-automated",
      "stop-all",
      "runtime-stop",
      "camera-start-detect",
      "camera-start-raw",
      "camera-stop",
      "recording-start-manual",
      "recording-stop",
      "calibration-start",
      "calibration-stop",
      "calibration-send",
      "calibration-save",
      "calibration-neutral",
    ].forEach(function (id) {
      const node = el(id);
      if (node) {
        node.disabled = disabled;
      }
    });
    document.querySelectorAll("[data-dance-preset]").forEach(function (node) {
      node.disabled = disabled;
    });
  }

  async function runAction(handler, timeoutMs) {
    if (actionInFlight) {
      showToast("Another action is still running", "warning");
      return;
    }
    if (!acquireActionLock(timeoutMs)) {
      syncActionButtons();
      showToast("Another page is controlling the robot", "warning");
      return;
    }
    actionInFlight = true;
    syncActionButtons();
    try {
      await handler();
    } finally {
      actionInFlight = false;
      releaseActionLock();
      syncActionButtons();
    }
  }

  async function requestJSON(url, options, allowError, timeoutMs) {
    const requestOptions = Object.assign({}, options || {});
    const controller = typeof AbortController === "function" ? new AbortController() : null;
    let timeoutId = null;
    if (controller) {
      requestOptions.signal = controller.signal;
      timeoutId = window.setTimeout(function () {
        controller.abort();
      }, timeoutMs || STATUS_TIMEOUT_MS);
    }
    try {
      const response = await fetch(url, requestOptions);
      let data = {};
      try {
        data = await response.json();
      } catch (error) {
        data = {};
      }
      if (!response.ok && !allowError) {
        throw new Error(data.error || data.message || ("Request failed: " + response.status));
      }
      return { ok: response.ok, status: response.status, data };
    } catch (error) {
      if (error && error.name === "AbortError") {
        throw new Error("Request timed out");
      }
      throw error;
    } finally {
      if (timeoutId !== null) {
        window.clearTimeout(timeoutId);
      }
    }
  }

  async function postJSON(url, payload, allowError, timeoutMs) {
    return requestJSON(
      url,
      {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload || {}),
      },
      allowError,
      timeoutMs
    );
  }

  function runtimePayload() {
    return {
      port: el("runtime-port") ? el("runtime-port").value : defaults.runtime.port,
      baudrate: el("runtime-baudrate") ? Number(el("runtime-baudrate").value) : defaults.runtime.baudrate,
      verbose: el("runtime-verbose") ? el("runtime-verbose").checked : defaults.runtime.verbose,
      leg_count: el("runtime-leg-count") ? Number(el("runtime-leg-count").value) : defaults.runtime.leg_count,
      deadzone: el("runtime-deadzone") ? Number(el("runtime-deadzone").value) : defaults.runtime.deadzone,
      stride: el("runtime-stride") ? Number(el("runtime-stride").value) : defaults.runtime.stride,
      lift_scale: el("runtime-lift-scale") ? Number(el("runtime-lift-scale").value) : defaults.runtime.lift_scale,
      frequency: el("runtime-frequency") ? Number(el("runtime-frequency").value) : defaults.runtime.frequency,
      yaw_stride_deg: el("runtime-yaw-stride-deg") ? Number(el("runtime-yaw-stride-deg").value) : defaults.runtime.yaw_stride_deg,
      stance_z: el("runtime-stance-z") ? Number(el("runtime-stance-z").value) : defaults.runtime.stance_z,
      duration: defaults.runtime.duration,
      cmd_scale: defaults.runtime.cmd_scale,
      timeout: defaults.runtime.timeout,
      override_file: defaults.runtime.override_file,
      calibration_file: defaults.runtime.calibration_file,
    };
  }

  function calibrationPayload() {
    const runtimeDefaults = defaults.runtime || {};
    const angles = selectedCalibrationAngles();
    return {
      port: el("runtime-port") ? el("runtime-port").value : (runtimeDefaults.port || "/dev/serial0"),
      baudrate: el("runtime-baudrate") ? Number(el("runtime-baudrate").value) : (runtimeDefaults.baudrate || 115200),
      leg_index: calibrationSelectedLeg,
      coxa_deg: angles.coxa_deg,
      femur_deg: angles.femur_deg,
      tibia_deg: angles.tibia_deg,
    };
  }

  function cameraPayload(modeOverride) {
    return {
      mode: modeOverride || defaults.camera.mode,
      host: el("camera-host") ? el("camera-host").value : defaults.camera.host,
      port: el("camera-port") ? Number(el("camera-port").value) : defaults.camera.port,
      detector: el("camera-detector") ? el("camera-detector").value : defaults.camera.detector,
    };
  }

  function buildRuntimePreview(data) {
    const parts = [
      "python3",
      "-m",
      "runtime.main",
      "--port",
      data.port,
      "--baudrate",
      String(data.baudrate),
    ];
    if (data.verbose) {
      parts.push("--verbose");
    }
    parts.push(
      "gait-leg-bridge",
      "--leg-count",
      String(data.leg_count),
      "--duration",
      String(data.duration),
      "--cmd-scale",
      String(data.cmd_scale),
      "--timeout",
      String(data.timeout),
      "--deadzone",
      String(data.deadzone),
      "--stride",
      String(data.stride),
      "--lift-scale",
      String(data.lift_scale),
      "--frequency",
      String(data.frequency),
      "--yaw-stride-deg",
      String(data.yaw_stride_deg),
      "--stance-z",
      String(data.stance_z),
      "--override-file",
      data.override_file,
      "--calibration-file",
      data.calibration_file
    );
    return parts.join(" ");
  }

  function buildCameraPreview(data) {
    const parts = ["python3", "live.py", data.mode, "--host", data.host, "--port", String(data.port)];
    if (data.mode === "detect") {
      parts.push("--detector", data.detector);
    }
    return parts.join(" ");
  }

  function buildCalibrationPreview(data, calibrationState) {
    return "N:" + buildCalibrationFrame(data, calibrationState).join(",");
  }

  function bootstrapForms(data) {
    if (formsBootstrapped) {
      return;
    }
    if (data.runtime && data.runtime.config) {
      setValue("runtime-port", data.runtime.config.port);
      setValue("runtime-baudrate", data.runtime.config.baudrate);
      setChecked("runtime-verbose", data.runtime.config.verbose);
      setValue("runtime-leg-count", data.runtime.config.leg_count);
      setValue("runtime-deadzone", data.runtime.config.deadzone);
      setValue("runtime-stride", data.runtime.config.stride);
      setValue("runtime-lift-scale", data.runtime.config.lift_scale);
      setValue("runtime-frequency", data.runtime.config.frequency);
      setValue("runtime-yaw-stride-deg", data.runtime.config.yaw_stride_deg);
      setValue("runtime-stance-z", data.runtime.config.stance_z);
      setText("override-file-readout", data.runtime.config.override_file);
    }
    if (data.calibration && data.calibration.config) {
      calibrationSelectedLeg = Number(data.calibration.config.leg_index || 0);
      loadCalibrationLegIntoEditor(calibrationSelectedLeg);
    }
    if (data.xbox && lastXboxButtonSeq === null) {
      lastXboxButtonSeq = normalizeXboxButtonSeq(data.xbox.button_seq);
    }
    if (data.camera && data.camera.config) {
      setValue("camera-host", data.camera.config.host);
      setValue("camera-port", data.camera.config.port);
      setValue("camera-detector", data.camera.config.detector);
    }
    formsBootstrapped = true;
    updateCommandPreviews();
  }

  function updateCommandPreviews() {
    if (defaults.runtime) {
      setText("runtime-command-preview", buildRuntimePreview(runtimePayload()));
    }
    if (defaults.camera) {
      setText("camera-command-preview", buildCameraPreview(cameraPayload("detect")));
    }
    if (el("calibration-coxa") && el("calibration-coxa-value")) {
      setText("calibration-coxa-value", toFixedAngle(el("calibration-coxa").value));
    }
    if (el("calibration-femur") && el("calibration-femur-value")) {
      setText("calibration-femur-value", toFixedAngle(el("calibration-femur").value));
    }
    if (el("calibration-tibia") && el("calibration-tibia-value")) {
      setText("calibration-tibia-value", toFixedAngle(el("calibration-tibia").value));
    }
    const calibrationState = currentCalibrationState();
    const calibrationData = calibrationPayload();
    if (el("calibration-command-preview")) {
      setText("calibration-command-preview", buildCalibrationPreview(calibrationData, calibrationState));
    }
    if (el("calibration-last-pose")) {
      setText("calibration-last-pose", selectedCalibrationAppliedPose(calibrationData, calibrationState).join(" / "));
    }
    updateCalibrationSafetyReadout(calibrationData, calibrationState);
    if (el("override-magnitude") && el("override-magnitude-value")) {
      setText("override-magnitude-value", el("override-magnitude").value);
    }
    renderCalibrationJointSelection();
  }

  function scheduleCalibrationPreview(delayMs) {
    if (calibrationAutoTimer) {
      window.clearTimeout(calibrationAutoTimer);
    }
    calibrationAutoTimer = window.setTimeout(function () {
      calibrationAutoTimer = null;
      if (calibrationRequestInFlight) {
        scheduleCalibrationPreview(60);
        return;
      }
      sendCalibrationPose(false);
    }, Math.max(0, Number(delayMs) || 0));
  }

  function adjustSelectedCalibrationJoint(deltaDeg) {
    const slider = selectedCalibrationSlider();
    if (!slider) {
      return;
    }
    const current = Number(slider.value);
    const min = Number(slider.min || 0);
    const max = Number(slider.max || 180);
    const next = clampNumber(current + deltaDeg, min, max);
    if (Math.abs(next - current) < 0.05) {
      return;
    }
    slider.value = toFixedAngle(next);
    updateCommandPreviews();
    scheduleCalibrationPreview(40);
  }

  function clearCameraFallbackTimer() {
    if (cameraFallbackTimer) {
      window.clearTimeout(cameraFallbackTimer);
      cameraFallbackTimer = null;
    }
  }

  function disconnectCameraEmbed() {
    const stream = el("camera-stream");
    const frame = el("camera-frame");
    const empty = el("camera-empty");

    clearCameraFallbackTimer();

    if (stream) {
      stream.removeAttribute("src");
      stream.dataset.src = "";
      stream.dataset.loaded = "";
      stream.classList.add("hidden");
    }
    if (frame) {
      frame.src = "about:blank";
      frame.dataset.src = "";
      frame.dataset.loaded = "";
      frame.classList.add("hidden");
    }
    if (empty) {
      empty.classList.remove("hidden");
    }
  }

  function showCameraFrameFallback() {
    const frame = el("camera-frame");
    const stream = el("camera-stream");
    const empty = el("camera-empty");
    if (!frame || !frame.dataset.src) {
      return;
    }
    frame.dataset.loaded = "true";
    frame.classList.remove("hidden");
    if (stream) {
      stream.classList.add("hidden");
    }
    if (empty) {
      empty.classList.add("hidden");
    }
    clearCameraFallbackTimer();
  }

  function refreshCameraEmbed(camera) {
    const stream = el("camera-stream");
    const frame = el("camera-frame");
    const empty = el("camera-empty");
    const streamUrl = getActiveCameraStreamUrl(camera);
    const pageUrl = camera && camera.page_url ? camera.page_url : "";
    const streamVisible = Boolean(stream && !stream.classList.contains("hidden"));
    const frameVisible = Boolean(frame && !frame.classList.contains("hidden"));
    const streamLoaded = Boolean(
      stream &&
      stream.dataset.loaded === "true" &&
      stream.dataset.src === streamUrl
    );
    const frameLoaded = Boolean(
      frame &&
      frame.dataset.loaded === "true" &&
      frame.dataset.src === pageUrl
    );
    const embedLoaded = streamLoaded || frameLoaded || streamVisible || frameVisible;

    if (!stream && !frame && !empty) {
      return;
    }

    clearCameraFallbackTimer();

    if (!streamUrl && !pageUrl) {
      if (stream) {
        stream.removeAttribute("src");
        stream.dataset.src = "";
        stream.dataset.loaded = "";
        stream.classList.add("hidden");
      }
      if (frame) {
        frame.src = "about:blank";
        frame.dataset.src = "";
        frame.classList.add("hidden");
      }
      if (empty) {
        empty.classList.remove("hidden");
      }
      return;
    }

    if (!camera.running && embedLoaded) {
      if (empty) {
        empty.classList.add("hidden");
      }
      return;
    }

    if (stream) {
      if (stream.dataset.src !== streamUrl) {
        stream.dataset.src = streamUrl;
        stream.dataset.loaded = "";
        stream.classList.add("hidden");
        stream.src = streamUrl;
      }
    }
    if (frame) {
      if (frame.dataset.src !== pageUrl) {
        frame.dataset.src = pageUrl;
        frame.dataset.loaded = "";
        frame.classList.add("hidden");
        frame.src = pageUrl || "about:blank";
      }
    }
    if (empty && !embedLoaded) {
      empty.classList.remove("hidden");
    }
    if (pageUrl && !embedLoaded) {
      cameraFallbackTimer = window.setTimeout(function () {
        const currentStream = el("camera-stream");
        if (currentStream && currentStream.dataset.loaded === "true") {
          return;
        }
        showCameraFrameFallback();
      }, 2500);
    }
  }

  function toggleButtonVisibility(id, visible) {
    var node = el(id);
    if (node) {
      node.style.display = visible ? "" : "none";
    }
  }

  function syncStartStopButtons(data) {
    var rtRunning = data.runtime.running;
    var camRunning = data.camera.running;
    var recActive = data.dance && data.dance.recording && data.dance.recording.active;
    var anyRunning = rtRunning || camRunning;

    // Dashboard: deploy vs stop-all
    toggleButtonVisibility("deploy-manual", !rtRunning);
    toggleButtonVisibility("deploy-automated", !anyRunning);
    toggleButtonVisibility("stop-all", anyRunning);

    // Manual page: deploy vs stop
    toggleButtonVisibility("runtime-stop", rtRunning);

    // Camera page: start vs stop
    toggleButtonVisibility("camera-start-detect", !camRunning);
    toggleButtonVisibility("camera-start-raw", !camRunning);
    toggleButtonVisibility("camera-stop", camRunning);

    // Dance page: start vs stop recording
    toggleButtonVisibility("recording-start-manual", !recActive);
    toggleButtonVisibility("recording-stop", Boolean(recActive));
  }

  function applyStatus(data) {
    lastStatus = data;
    if (lastXboxButtonSeq === null && data.xbox) {
      lastXboxButtonSeq = normalizeXboxButtonSeq(data.xbox.button_seq);
    }
    bootstrapForms(data);
    updateCommandPreviews();
    updateSharedStatus(data);
    updateOverview(data);
    updateManual(data);
    updateAutomated(data);
    updateCreateDance(data);
    updateCameraPage(data);
    syncStartStopButtons(data);
  }

  function showStatusError(message) {
    const text = message || "Unable to reach robot dashboard.";
    setStatusChip("sidebar-runtime-chip", null, "Runtime running", "Runtime stopped", "Runtime unavailable");
    setStatusChip("sidebar-camera-chip", null, "Camera running", "Camera stopped", "Camera unavailable");
    setText("overview-robot-state", "Unavailable");
    setText("overview-robot-detail", text);
    setText("overview-runtime-state", "Unavailable");
    setText("overview-runtime-detail", text);
    setText("overview-camera-state", "Unavailable");
    setText("overview-camera-detail", text);
    setText("manual-runtime-detail", text);
    setText("override-state", text);
    setText("controller-state", "Unavailable");
    setStatusChip("manual-runtime-chip", null, "Runtime running", "Runtime stopped", "Runtime unavailable");
    setStatusChip("calibration-chip", null, "Calibration active", "Calibration stopped", "Calibration unavailable");
    setText("calibration-status", text);
    setText("calibration-progress", "-- / 6 saved");
    setText("calibration-selected-leg", "--");
    setText("calibration-selected-joint", "--");
    setText("calibration-last-command", text);
    setText("calibration-safe-window", text);
    setText("calibration-safe-state", text);
    setText("calibration-file-path", text);
    setText("calibration-profile-status", text);
    setStatusChip("automated-camera-chip", null, "Camera live", "Camera stopped", "Camera unavailable");
    setText("dance-detail", text);
    setText("dance-last-request", "Unavailable");
    setStatusChip("dance-recording-chip", null, "Recording live", "Recording stopped", "Recording unavailable");
    setText("recording-status", "Unavailable");
    setText("recording-detail", text);
    setText("recording-save-path", text);
    setText("recording-save-detail", text);
    setText("recording-current-name", "Unavailable");
    setText("recording-duration", "Duration: --");
    setText("recording-event-count", "--");
    setText("recording-last-name", "Unavailable");
    setText("recording-last-path", text);
    setText("recording-auto-root", text);
    setHTML("recording-recent-list", '<div class="empty-copy">' + escapeHTML(text) + "</div>");
    setStatusChip("camera-page-chip", null, "Camera live", "Camera stopped", "Camera unavailable");
    setText("camera-summary", "Unavailable");
    setText("camera-summary-copy", text);
    setText("camera-mode-chip", "Camera mode: --");
    setText("calibration-state", "Unavailable");
    setText("calibration-detail", text);
    setText("tag-count-chip", "Tags: --");
    setText("tag-ids-chip", "IDs: --");
    [
      "runtime-log",
      "camera-log",
    ].forEach(function (id) {
      setPre(id, ["Status refresh failed: " + text], "Status refresh failed.");
    });
  }

  function updateOverview(data) {
    if (!el("overview-robot-state")) {
      return;
    }
    setText("overview-robot-state", data.runtime.running ? "Ready for control" : "Idle / safe");
    setText("overview-robot-detail", data.override.active ? "Web override active" : "No active override");
    setText("overview-runtime-state", data.runtime.running ? "Running" : "Stopped");
    setText(
      "overview-runtime-detail",
      data.runtime.running
        ? "Serial " + data.runtime.config.port + " · pid " + data.runtime.pid
        : "Last exit " + (data.runtime.last_exit_code ?? "none")
    );
    setText("overview-camera-state", data.camera.running ? "Running" : "Stopped");
    setText(
      "overview-camera-detail",
      data.camera.running ? data.camera.config.mode + " mode on :" + data.camera.config.port : "Start from Camera or Automated page"
    );
  }

  function renderCalibrationLegSelection(calibration) {
    document.querySelectorAll("[data-calibration-leg]").forEach(function (node) {
      const legIndex = Number(node.dataset.calibrationLeg);
      const entry = calibrationLegEntry(calibration, legIndex);
      const saved = Boolean(entry && entry.saved);
      node.classList.toggle("active", legIndex === calibrationSelectedLeg);
      node.classList.toggle("saved", saved);
      node.classList.toggle("pending", !saved);
      const stateNode = node.querySelector(".calibration-leg-state");
      if (stateNode) {
        stateNode.textContent = saved ? "Saved" : "Pending";
      }
    });

    const selectedEntry = calibrationLegEntry(calibration, calibrationSelectedLeg);
    const selectedSaved = Boolean(selectedEntry && selectedEntry.saved);
    setText("calibration-selected-leg", currentCalibrationLabel());
    setText("calibration-editor-title", currentCalibrationLabel());
    setText(
      "calibration-leg-detail",
      selectedSaved
        ? "Saved neutral pose loaded. Draft sliders are relative offsets around that pose."
        : "Pending save. Preview draft offsets, then save this leg to rebase its neutral pose."
    );
  }

  function updateManual(data) {
    if (!el("manual-runtime-chip")) {
      return;
    }
    setStatusChip("manual-runtime-chip", data.runtime.running, "Runtime running", "Runtime stopped", "Runtime idle");
    setText(
      "manual-runtime-detail",
      data.runtime.running
        ? "pid " + data.runtime.pid + " on " + data.runtime.config.port
        : "Deploy manual mode to start runtime."
    );
    setText(
      "override-state",
      data.override.active
        ? "Web override " + (modeLabels[data.override.mode] || data.override.mode) + " at magnitude " + data.override.magnitude
        : "No active override."
    );
    setText("controller-state", data.override.active ? (modeLabels[data.override.mode] || data.override.mode) : "Idle");
    setPre("runtime-log", data.runtime.logs, "No runtime logs yet.");

    var calibration = data.calibration || {};
    if (el("calibration-chip")) {
      setStatusChip(
        "calibration-chip",
        calibration.active ? true : null,
        "Calibration active",
        "Calibration stopped",
        "Calibration idle"
      );
    }
    if (el("calibration-status")) {
      var calibrationSafety = calibrationSafetySnapshot(calibrationPayload(), calibration);
      setText(
        "calibration-status",
        calibration.active
          ? "Draft offset preview sent over the manual UART channel for " + currentCalibrationLabel() + "." +
              (calibration.runtime_stopped ? " Runtime stopped." : "") +
              (data.xbox && data.xbox.process_running ? " Xbox remains available for trimming." : "") +
              (calibrationSafety.safe ? "" : " Current preview is outside the runtime safe window.")
          : "Select a leg, trim draft offsets around its saved neutral pose, then save to rebase that pose back to zero."
      );
    }
    if (el("calibration-progress")) {
      setText(
        "calibration-progress",
        String(calibration.saved_count || 0) + " / " + calibrationLegLabels.length + " saved"
      );
    }
    if (el("calibration-file-path")) {
      setText(
        "calibration-file-path",
        calibration.profile_path || defaults.runtime.calibration_file || "Unavailable"
      );
    }
    if (el("calibration-profile-status")) {
      setText("calibration-profile-status", calibrationProfileSummary(calibration));
    }
    var selectedEntry = calibrationLegEntry(calibration, calibrationSelectedLeg);
    if (el("calibration-saved-neutral")) {
      var neutral = calibrationAnglesFromEntry(selectedEntry);
      setText(
        "calibration-saved-neutral",
        [
          toFixedAngle(neutral.coxa_deg),
          toFixedAngle(neutral.femur_deg),
          toFixedAngle(neutral.tibia_deg),
        ].join(" / ")
      );
    }
    if (el("calibration-saved-offset")) {
      var savedOffsets = selectedEntry && selectedEntry.offsets_deg ? selectedEntry.offsets_deg : {};
      setText(
        "calibration-saved-offset",
        [
          toFixedAngle(savedOffsets.coxa_deg),
          toFixedAngle(savedOffsets.femur_deg),
          toFixedAngle(savedOffsets.tibia_deg),
        ].join(" / ")
      );
    }
    if (el("calibration-last-pose")) {
      var applied = selectedCalibrationAppliedPose(calibrationPayload(), calibration);
      setText("calibration-last-pose", applied.join(" / "));
    }
    updateCalibrationSafetyReadout(calibrationPayload(), calibration);
    if (el("calibration-last-command")) {
      setText("calibration-last-command", calibration.command_text || "Not sent yet.");
    }
    renderCalibrationLegSelection(calibration);
    renderCalibrationJointSelection();

    // Xbox controller status
    var xbox = data.xbox;
    if (xbox && el("xbox-chip")) {
      var xboxConnected = xbox.connected && xbox.process_running;
      setStatusChip("xbox-chip", xbox.process_running ? (xbox.connected ? true : null) : false,
        "Connected", xbox.process_running ? "Searching..." : "Stopped", "Disconnected");
      setText("xbox-device-name", xbox.device_name || "--");
      setText("xbox-connection-type", xbox.connection_type === "usb" ? "USB" : xbox.connection_type === "bluetooth" ? "Bluetooth" : "--");
      setText("xbox-battery", xbox.battery != null ? xbox.battery + "%" : (xbox.connection_type === "usb" ? "USB Powered" : "--"));
      if (xboxConnected) {
        setText("xbox-axes", "fwd: " + (xbox.forward_cmd || 0) + "  str: " + (xbox.strafe_cmd || 0) + "  turn: " + (xbox.turn_cmd || 0));
      } else {
        setText("xbox-axes", "fwd: 0  str: 0  turn: 0");
      }
    }
  }

  function updateAutomated(data) {
    if (!el("automated-camera-chip")) {
      return;
    }
    setStatusChip("automated-camera-chip", data.camera.running, "Camera live", "Camera stopped", "Camera idle");
    setText("dance-state", data.dance.implemented ? "Live" : "Placeholder");
    setText(
      "dance-detail",
      data.dance.last_request
        ? "Last preset " + data.dance.last_request.preset + " requested at " + new Date(data.dance.last_request.requested_at * 1000).toLocaleTimeString()
        : "Preset buttons are ready; choreography runtime is not attached yet."
    );
    setText(
      "dance-last-request",
      data.dance.last_request ? data.dance.last_request.preset : "No preset requested yet."
    );
    setPre("camera-log", data.camera.logs, "No camera logs yet.");
  }

  function updateCameraPage(data) {
    if (!el("camera-page-chip")) {
      return;
    }
    setStatusChip("camera-page-chip", data.camera.running, "Camera live", "Camera stopped", "Camera idle");
    setText(
      "camera-summary",
      data.camera.running
        ? data.camera.config.mode + " mode on :" + data.camera.config.port
        : "Camera service stopped"
    );
    setPre("camera-log", data.camera.logs, "No camera logs yet.");
  }

  function renderRecordingList(recording) {
    const node = el("recording-recent-list");
    if (!node) {
      return;
    }
    const recent = (recording && Array.isArray(recording.recent) ? recording.recent : []).filter(function (item) {
      return item && item.mode === "manual";
    });
    if (!recent.length) {
      node.innerHTML = '<div class="empty-copy">No manual recordings saved yet.</div>';
      return;
    }
    node.innerHTML = recent
      .map(function (item) {
        return (
          '<article class="recording-item">' +
          "<strong>" + escapeHTML(item.name || item.directory_name || "untitled") + "</strong>" +
          "<span>" + escapeHTML(item.directory_name || "") + "</span>" +
          "<span>Saved " + escapeHTML(formatTimestamp(item.saved_at)) + "</span>" +
          "<span>Duration " + escapeHTML(formatDuration(item.duration_sec)) + " · " + escapeHTML(String(item.event_count || 0)) + " events</span>" +
          "<code>" + escapeHTML(item.path || "") + "</code>" +
          "</article>"
        );
      })
      .join("");
  }

  function updateCreateDance(data) {
    if (!el("dance-recording-chip")) {
      return;
    }
    const recording = (data.dance && data.dance.recording) || {};
    setStatusChip(
      "dance-recording-chip",
      recording.active ? true : null,
      "Recording live",
      "Recording stopped",
      "Recording idle"
    );
    setText("recording-status", recording.active ? "Recording live" : "Idle");
    setText(
      "recording-detail",
      recording.active
        ? "Manual overrides are being captured in real time. You can keep driving from the Manual page while this stays active."
        : "Start a manual recording, then steer the robot and stop when the routine is finished."
    );
    setText(
      "recording-save-path",
      recording.active
        ? recording.path
        : (recording.storage_roots && recording.storage_roots.manual) || "Unavailable"
    );
    setText(
      "recording-save-detail",
      recording.active
        ? "This active session will save into the folder above when you stop."
        : "New recordings save as timestamped folders inside the manual dance storage root."
    );
    setText(
      "recording-current-name",
      recording.active
        ? (recording.name || recording.directory_name || "untitled")
        : "No active recording."
    );
    setText("recording-duration", "Duration: " + formatDuration(recording.duration_sec));
    setText("recording-event-count", String(recording.event_count || 0));
    setText(
      "recording-last-name",
      recording.last_saved ? (recording.last_saved.name || recording.last_saved.directory_name) : "Nothing saved yet."
    );
    setText(
      "recording-last-path",
      recording.last_saved
        ? recording.last_saved.path + " · " + formatDuration(recording.last_saved.duration_sec)
        : "The newest recording path will appear here."
    );
    setText("recording-auto-root", (recording.storage_roots && recording.storage_roots.auto) || "Unavailable");
    renderRecordingList(recording);
  }

  function getActiveCameraStreamUrl(camera) {
    if (!camera) {
      return "";
    }
    if (camera.active_stream_url) {
      return camera.active_stream_url;
    }
    if (camera.config && camera.config.mode === "camera" && camera.raw_stream_url) {
      return camera.raw_stream_url;
    }
    return camera.stream_url || "";
  }

  function updateSharedStatus(data) {
    setStatusChip("sidebar-runtime-chip", data.runtime.running, "Runtime running", "Runtime stopped", "Runtime idle");
    setStatusChip("sidebar-camera-chip", data.camera.running, "Camera running", "Camera stopped", "Camera idle");
    setText("camera-mode-chip", "Camera mode: " + data.camera.config.mode);
    setText("camera-summary-copy", data.camera.running ? "Live " + data.camera.config.mode + " view is active." : "Camera is currently stopped.");
    setText("camera-summary", data.camera.running ? "Running" : "Stopped");
    setHref("camera-page-link", data.camera.page_url, data.camera.page_url);
    const streamUrl = getActiveCameraStreamUrl(data.camera);
    setHref("camera-stream-link", streamUrl || "#", streamUrl || "Stream unavailable");

    refreshCameraEmbed(data.camera);

    setText("runtime-command-preview", buildRuntimePreview(runtimePayload()));
    setText("camera-command-preview", buildCameraPreview(cameraPayload("detect")));
  }

  async function refreshDetections() {
    if (!el("calibration-state")) {
      return;
    }
    if (!lastStatus || !lastStatus.camera.running) {
      setText("calibration-state", "Waiting for camera");
      setText("calibration-detail", "Start detect mode to read AprilTags.");
      setText("tag-count-chip", "Tags: --");
      setText("tag-ids-chip", "IDs: --");
      return;
    }

    const result = await requestJSON("/api/camera/detections", undefined, true);
    const payload = result.data || {};
    if (!result.ok) {
      setText("calibration-state", "Detection unavailable");
      setText("calibration-detail", payload.error || "Unable to fetch detections.");
      setText("tag-count-chip", "Tags: --");
      setText("tag-ids-chip", "IDs: --");
      return;
    }

    const detections = Array.isArray(payload.detections) ? payload.detections : [];
    const tagIds = detections
      .map((item) => item.tag_id)
      .filter((item) => item !== null && item !== undefined);
    setText("calibration-state", payload.count > 0 ? "Tag lock visible" : "No tag in frame");
    setText(
      "calibration-detail",
      payload.error
        ? payload.error
        : payload.count > 0
          ? "Detected " + payload.count + " tag(s). Use this for alignment and calibration."
          : "Camera is live. Place an AprilTag in frame to verify calibration."
    );
    setText("tag-count-chip", "Tags: " + payload.count);
    setText("tag-ids-chip", "IDs: " + (tagIds.length ? tagIds.join(", ") : "--"));
  }

  async function refreshStatus() {
    if (refreshInFlight || actionInFlight) {
      return;
    }
    if (!pageActive) {
      return;
    }
    if (typeof document !== "undefined" && document.visibilityState === "hidden") {
      return;
    }
    if (!claimStatusLeader()) {
      return;
    }
    refreshInFlight = true;
    try {
      const result = await requestJSON("/api/status", undefined, false, STATUS_TIMEOUT_MS);
      applyStatus(result.data);
      await refreshDetections();
    } catch (error) {
      showStatusError(error.message);
    } finally {
      refreshInFlight = false;
    }
  }

  function processXboxCalibrationStatus(status) {
    if (!status) {
      return;
    }

    const nextSeq = normalizeXboxButtonSeq(status.button_seq);
    if (lastXboxButtonSeq === null) {
      lastXboxButtonSeq = nextSeq;
      return;
    }
    if (Object.keys(nextSeq).some(function (key) { return nextSeq[key] < lastXboxButtonSeq[key]; })) {
      lastXboxButtonSeq = nextSeq;
      return;
    }

    if (nextSeq.lb > lastXboxButtonSeq.lb) {
      setCalibrationSelectedLeg((calibrationSelectedLeg + calibrationLegLabels.length - 1) % calibrationLegLabels.length, true);
      armXboxCalibration(XBOX_CALIBRATION_ARM_MS);
    }
    if (nextSeq.rb > lastXboxButtonSeq.rb) {
      setCalibrationSelectedLeg((calibrationSelectedLeg + 1) % calibrationLegLabels.length, true);
      armXboxCalibration(XBOX_CALIBRATION_ARM_MS);
    }
    Object.keys(xboxCalibrationButtonMap).forEach(function (buttonName) {
      if (nextSeq[buttonName] > lastXboxButtonSeq[buttonName]) {
        setCalibrationSelectedJoint(xboxCalibrationButtonMap[buttonName]);
        armXboxCalibration(XBOX_CALIBRATION_ARM_MS);
      }
    });
    if (nextSeq.b > lastXboxButtonSeq.b) {
      armXboxCalibration(XBOX_CALIBRATION_ARM_MS);
      if (calibrationRequestInFlight) {
        window.setTimeout(function () {
          saveCalibrationPose();
        }, 120);
      } else {
        saveCalibrationPose();
      }
    }
    lastXboxButtonSeq = nextSeq;

    const calibrationAllowed = Boolean(
      status.process_running &&
      status.connected &&
      (status.calibration_active || !status.runtime_running || Date.now() < calibrationXboxArmedUntil)
    );
    if (!calibrationAllowed) {
      return;
    }

    const adjust = Number(status.calibration_adjust || 0);
    if (!Number.isFinite(adjust) || Math.abs(adjust) < 60) {
      return;
    }

    const delta = Math.round((adjust / 1000) * XBOX_CALIBRATION_MAX_STEP_DEG * 10) / 10;
    if (!delta) {
      return;
    }
    armXboxCalibration(1500);
    adjustSelectedCalibrationJoint(delta);
  }

  async function refreshXboxCalibrationStatus() {
    if (!pageActive || actionInFlight || !el("calibration-selected-joint")) {
      return;
    }
    if (typeof document !== "undefined" && document.visibilityState === "hidden") {
      return;
    }
    if (!claimStatusLeader()) {
      return;
    }
    try {
      const result = await requestJSON("/api/xbox/status", undefined, true, 1800);
      if (!result.ok) {
        return;
      }
      processXboxCalibrationStatus(result.data || {});
    } catch (_error) {
      // Xbox polling is best-effort; the main dashboard poll still updates status.
    }
  }

  function setActiveDriveButton(mode) {
    document.querySelectorAll("[data-hold-mode]").forEach((node) => {
      node.classList.toggle("active", node.dataset.holdMode === mode);
    });
  }

  function overridePayload(mode) {
    return {
      mode: mode,
      magnitude: el("override-magnitude") ? Number(el("override-magnitude").value) : 700,
      expires_after: 0.7,
    };
  }

  function beginOverride(mode) {
    if (overrideTimer) {
      window.clearInterval(overrideTimer);
    }
    setActiveDriveButton(mode);
    setText("controller-state", modeLabels[mode] || mode);
    const push = async function () {
      try {
        await postJSON("/api/override/set", overridePayload(mode));
      } catch (error) {
        console.error(error);
      }
    };
    push();
    overrideTimer = window.setInterval(push, 250);
  }

  async function clearOverride() {
    const shouldClearRemote = Boolean(
      overrideTimer ||
      activeKeyCode ||
      (lastStatus && lastStatus.override && lastStatus.override.active)
    );
    if (overrideTimer) {
      window.clearInterval(overrideTimer);
      overrideTimer = null;
    }
    setActiveDriveButton(null);
    setText("controller-state", "Idle");
    if (!shouldClearRemote) {
      return;
    }
    try {
      await postJSON("/api/override/clear", {});
    } catch (error) {
      console.error(error);
    }
    await refreshStatus();
  }

  async function submitCalibration(url, successMessage) {
    if (calibrationRequestInFlight) {
      return;
    }
    calibrationRequestInFlight = true;
    try {
      const result = await postJSON(url, calibrationPayload(), false, ACTION_TIMEOUT_MS);
      applyStatus(result.data);
      syncCalibrationEditorFromStatus(result.data);
      armXboxCalibration(XBOX_CALIBRATION_ARM_MS);
      if (successMessage) {
        showToast(successMessage, "success");
      }
    } catch (error) {
      showToast(error.message, "error");
    } finally {
      calibrationRequestInFlight = false;
    }
  }

  async function sendCalibrationPose(notify) {
    await submitCalibration(
      "/api/calibration/send",
      notify ? ("Preview sent for " + currentCalibrationLabel()) : ""
    );
  }

  async function saveCalibrationPose() {
    await submitCalibration(
      "/api/calibration/save",
      "Saved calibration for " + currentCalibrationLabel()
    );
  }

  async function startCalibration() {
    await runAction(async function () {
      const result = await postJSON("/api/calibration/start", calibrationPayload(), false, ACTION_TIMEOUT_MS);
      applyStatus(result.data);
      syncCalibrationEditorFromStatus(result.data);
      armXboxCalibration(XBOX_CALIBRATION_ARM_MS);
      showToast("Calibration started for " + currentCalibrationLabel(), "success");
    }, ACTION_TIMEOUT_MS);
  }

  async function stopCalibration() {
    await runAction(async function () {
      const result = await postJSON("/api/calibration/stop", {}, false, ACTION_TIMEOUT_MS);
      applyStatus(result.data);
      syncCalibrationEditorFromStatus(result.data);
      showToast("Calibration stopped", "info");
    }, ACTION_TIMEOUT_MS);
  }

  function scheduleCalibrationAutoApply() {
    if (!el("calibration-auto-apply") || !el("calibration-auto-apply").checked) {
      if (calibrationAutoTimer) {
        window.clearTimeout(calibrationAutoTimer);
        calibrationAutoTimer = null;
      }
      return;
    }
    scheduleCalibrationPreview(140);
  }

  function bindDriveButtons() {
    const buttons = document.querySelectorAll("[data-hold-mode]");
    if (!buttons.length) {
      return;
    }
    buttons.forEach((node) => {
      const mode = node.dataset.holdMode;
      node.addEventListener("pointerdown", function (event) {
        event.preventDefault();
        beginOverride(mode);
      });
      ["pointerup", "pointerleave", "pointercancel"].forEach((name) => {
        node.addEventListener(name, function () {
          clearOverride();
        });
      });
    });
    const release = el("override-release");
    if (release) {
      release.addEventListener("click", function () {
        clearOverride();
      });
    }
    window.addEventListener("pointerup", function () {
      clearOverride();
    });
  }

  function bindKeyboardControls() {
    if (!document.querySelector("[data-hold-mode]")) {
      return;
    }
    window.addEventListener("keydown", function (event) {
      const tagName = event.target && event.target.tagName;
      if (tagName === "INPUT" || tagName === "SELECT" || tagName === "TEXTAREA" || event.target.isContentEditable) {
        return;
      }
      const mode = keyMap[event.code];
      if (!mode) {
        return;
      }
      event.preventDefault();
      if (activeKeyCode === event.code && event.repeat) {
        return;
      }
      activeKeyCode = event.code;
      setText("keyboard-readout", event.code + " → " + (modeLabels[mode] || mode));
      beginOverride(mode);
    });

    window.addEventListener("keyup", function (event) {
      if (event.code !== activeKeyCode) {
        return;
      }
      activeKeyCode = null;
      setText("keyboard-readout", "No key pressed.");
      clearOverride();
    });

    window.addEventListener("blur", function () {
      activeKeyCode = null;
      setText("keyboard-readout", "No key pressed.");
      clearOverride();
    });
  }

  function bindInputs() {
    [
      "runtime-port",
      "runtime-baudrate",
      "runtime-verbose",
      "runtime-leg-count",
      "runtime-deadzone",
      "runtime-stride",
      "runtime-lift-scale",
      "runtime-frequency",
      "runtime-yaw-stride-deg",
      "runtime-stance-z",
      "calibration-coxa",
      "calibration-femur",
      "calibration-tibia",
      "calibration-auto-apply",
      "camera-host",
      "camera-port",
      "camera-detector",
      "override-magnitude",
    ].forEach((id) => {
      const node = el(id);
      if (!node) {
        return;
      }
      node.addEventListener("input", updateCommandPreviews);
      node.addEventListener("change", updateCommandPreviews);
      if (
        id === "calibration-coxa" ||
        id === "calibration-femur" ||
        id === "calibration-tibia" ||
        id === "calibration-auto-apply"
      ) {
        node.addEventListener("input", scheduleCalibrationAutoApply);
        node.addEventListener("change", scheduleCalibrationAutoApply);
      }
    });
  }

  function bindCalibrationControls() {
    const startButton = el("calibration-start");
    const stopButton = el("calibration-stop");
    const sendButton = el("calibration-send");
    const saveButton = el("calibration-save");
    const neutralButton = el("calibration-neutral");
    document.querySelectorAll("[data-calibration-leg]").forEach(function (node) {
      node.addEventListener("click", function () {
        setCalibrationSelectedLeg(Number(node.dataset.calibrationLeg), true);
        renderCalibrationLegSelection(currentCalibrationState());
      });
    });
    document.querySelectorAll("[data-calibration-joint]").forEach(function (node) {
      node.addEventListener("click", function () {
        setCalibrationSelectedJoint(node.dataset.calibrationJoint);
        armXboxCalibration(XBOX_CALIBRATION_ARM_MS);
      });
    });
    if (sendButton) {
      sendButton.addEventListener("click", function () {
        sendCalibrationPose(true);
      });
    }
    if (startButton) {
      startButton.addEventListener("click", function () {
        startCalibration();
      });
    }
    if (stopButton) {
      stopButton.addEventListener("click", function () {
        stopCalibration();
      });
    }
    if (saveButton) {
      saveButton.addEventListener("click", function () {
        saveCalibrationPose();
      });
    }
    if (neutralButton) {
      neutralButton.addEventListener("click", function () {
        setCalibrationSliderValues(calibrationDefaults());
        updateCommandPreviews();
        sendCalibrationPose(true);
      });
    }
  }

  function bindCameraEmbed() {
    const stream = el("camera-stream");
    const frame = el("camera-frame");
    const empty = el("camera-empty");
    if (stream) {
      stream.addEventListener("load", function () {
        stream.dataset.loaded = "true";
        stream.classList.remove("hidden");
        if (frame) {
          frame.classList.add("hidden");
          frame.dataset.loaded = "";
        }
        if (empty) {
          empty.classList.add("hidden");
        }
        clearCameraFallbackTimer();
      });
      stream.addEventListener("error", function () {
        stream.dataset.loaded = "";
        showCameraFrameFallback();
      });
    }
    if (frame) {
      frame.addEventListener("load", function () {
        frame.dataset.loaded = "true";
        const currentStream = el("camera-stream");
        if (currentStream && currentStream.dataset.loaded === "true") {
          return;
        }
        showCameraFrameFallback();
      });
    }
  }

  function bindActions() {
    const actionMap = [
      {
        id: "deploy-manual",
        handler: async function () {
          await postJSON("/api/deploy/manual", { runtime: runtimePayload() }, false, ACTION_TIMEOUT_MS);
          showToast("Manual mode deployed", "success");
          await refreshStatus();
        },
      },
      {
        id: "deploy-automated",
        handler: async function () {
          await postJSON("/api/deploy/automated", {
            runtime: runtimePayload(),
            camera: cameraPayload("detect"),
          }, false, ACTION_TIMEOUT_MS);
          showToast("Automated mode deployed", "success");
          await refreshStatus();
        },
      },
      {
        id: "recording-start-manual",
        handler: async function () {
          const result = await postJSON(
            "/api/dance/recording/start",
            {
              mode: "manual",
              name: el("recording-name") ? el("recording-name").value : "",
            },
            true
          );
          if (!result.ok) {
            throw new Error(result.data.error || "Unable to start manual recording");
          }
          showToast("Manual dance recording started", "success");
          await refreshStatus();
        },
      },
      {
        id: "recording-stop",
        handler: async function () {
          const result = await postJSON("/api/dance/recording/stop", {}, true);
          if (!result.ok) {
            throw new Error(result.data.error || "Unable to stop dance recording");
          }
          showToast("Dance recording saved", "success");
          await refreshStatus();
        },
      },
      {
        id: "stop-all",
        handler: async function () {
          disconnectCameraEmbed();
          await new Promise(function (resolve) {
            window.setTimeout(resolve, 180);
          });
          try {
            await postJSON("/api/deploy/stop-all", {}, false, ACTION_TIMEOUT_MS);
          } catch (_ignored) {
            // Stop is fire-and-forget on the backend; a timeout is fine.
          }
          showToast("Stopping all services\u2026", "info");
          await refreshStatus();
        },
      },
      {
        id: "runtime-stop",
        handler: async function () {
          await postJSON("/api/runtime/stop", {}, false, ACTION_TIMEOUT_MS);
          showToast("Runtime stopped", "info");
          await refreshStatus();
        },
      },
      {
        id: "camera-start-detect",
        handler: async function () {
          await postJSON("/api/camera/start", cameraPayload("detect"), false, ACTION_TIMEOUT_MS);
          showToast("Detect camera started", "success");
          await refreshStatus();
        },
      },
      {
        id: "camera-start-raw",
        handler: async function () {
          await postJSON("/api/camera/start", cameraPayload("camera"), false, ACTION_TIMEOUT_MS);
          showToast("Raw camera started", "success");
          await refreshStatus();
        },
      },
      {
        id: "camera-stop",
        handler: async function () {
          disconnectCameraEmbed();
          await new Promise(function (resolve) {
            window.setTimeout(resolve, 180);
          });
          await postJSON("/api/camera/stop", {}, false, LONG_ACTION_TIMEOUT_MS);
          showToast("Camera stopped", "info");
          await refreshStatus();
        },
      },
    ];

    actionMap.forEach((item) => {
      const node = el(item.id);
      if (!node) {
        return;
      }
      node.addEventListener("click", function () {
        runAction(async function () {
          try {
            await item.handler();
          } catch (error) {
            showToast(error.message, "error");
          }
        }, item.id === "stop-all" || item.id === "camera-stop" ? LONG_ACTION_TIMEOUT_MS : ACTION_TIMEOUT_MS);
      });
    });

    document.querySelectorAll("[data-dance-preset]").forEach((node) => {
      node.addEventListener("click", function () {
        runAction(async function () {
          const preset = node.dataset.dancePreset;
          const result = await postJSON("/api/dance/start", { preset: preset }, true, ACTION_TIMEOUT_MS);
          if (result.ok) {
            showToast("Dance preset queued", "success");
          } else {
            showToast(result.data.message || "Dance engine not implemented yet", "warning");
          }
          await refreshStatus();
        }, ACTION_TIMEOUT_MS);
      });
    });
  }

  function shutdownPage() {
    pageActive = false;
    releaseStatusLeader();
    if (!actionInFlight) {
      releaseActionLock();
    }
    disconnectCameraEmbed();
    clearCameraFallbackTimer();
    if (statusPollTimer) {
      window.clearInterval(statusPollTimer);
      statusPollTimer = null;
    }
    if (xboxCalibrationPollTimer) {
      window.clearInterval(xboxCalibrationPollTimer);
      xboxCalibrationPollTimer = null;
    }
    if (overrideTimer) {
      window.clearInterval(overrideTimer);
      overrideTimer = null;
    }
    if (calibrationAutoTimer) {
      window.clearTimeout(calibrationAutoTimer);
      calibrationAutoTimer = null;
    }
  }

  bindActions();
  bindInputs();
  bindCalibrationControls();
  bindCameraEmbed();
  bindDriveButtons();
  bindKeyboardControls();
  syncActionButtons();
  if (typeof document !== "undefined") {
    window.addEventListener("storage", function (event) {
      if (event.key === ACTION_LOCK_KEY) {
        syncActionButtons();
      }
    });
    document.addEventListener("visibilitychange", function () {
      if (document.visibilityState === "visible") {
        pageActive = true;
        refreshStatus();
        refreshXboxCalibrationStatus();
        return;
      }
      disconnectCameraEmbed();
      releaseStatusLeader();
    });
  }
  window.addEventListener("pagehide", shutdownPage);
  window.addEventListener("beforeunload", shutdownPage);
  updateCommandPreviews();
  if (initialStatus) {
    applyStatus(initialStatus);
    refreshDetections();
  } else {
    showStatusError("Waiting for dashboard bootstrap data.");
  }
  refreshStatus();
  statusPollTimer = window.setInterval(function () {
    refreshStatus();
  }, 500);
  refreshXboxCalibrationStatus();
  xboxCalibrationPollTimer = window.setInterval(function () {
    refreshXboxCalibrationStatus();
  }, XBOX_CALIBRATION_POLL_MS);
})();
