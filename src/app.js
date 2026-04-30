import { InertialNavigator, normalizeMotionEvent } from "./ins.js";

const nav = new InertialNavigator();
const state = {
  running: false,
  path: [],
  calibration: [],
  calibratingUntil: 0,
};

const $ = (id) => document.getElementById(id);
const ui = {
  start: $("startBtn"),
  calibrate: $("calibrateBtn"),
  reset: $("resetBtn"),
  status: $("status"),
  posX: $("posX"),
  posY: $("posY"),
  posZ: $("posZ"),
  speed: $("speed"),
  attitude: $("attitude"),
  sampleRate: $("sampleRate"),
  stationary: $("stationary"),
  quality: $("quality"),
  canvas: $("trajectory"),
};

const ctx = ui.canvas.getContext("2d");

function setStatus(text, kind = "") {
  ui.status.textContent = text;
  ui.status.className = `status ${kind}`.trim();
}

function fmt(value, unit = "") {
  return `${value.toFixed(3)}${unit}`;
}

function drawTrajectory() {
  const { width, height } = ui.canvas;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0b1117";
  ctx.fillRect(0, 0, width, height);

  ctx.strokeStyle = "#20303a";
  ctx.lineWidth = 1;
  for (let x = 0; x <= width; x += 45) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y <= height; y += 45) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  ctx.strokeStyle = "#f0b84f";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(width / 2 - 10, height / 2);
  ctx.lineTo(width / 2 + 10, height / 2);
  ctx.moveTo(width / 2, height / 2 - 10);
  ctx.lineTo(width / 2, height / 2 + 10);
  ctx.stroke();

  if (state.path.length < 2) return;

  const maxAbs = Math.max(
    0.7,
    ...state.path.map((p) => Math.max(Math.abs(p.x), Math.abs(p.y))),
  );
  const scale = Math.min(width, height) * 0.42 / maxAbs;

  ctx.strokeStyle = "#22c1a5";
  ctx.lineWidth = 4;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.beginPath();

  state.path.forEach((p, i) => {
    const x = width / 2 + p.x * scale;
    const y = height / 2 - p.y * scale;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  const last = state.path[state.path.length - 1];
  ctx.fillStyle = "#eef4f2";
  ctx.beginPath();
  ctx.arc(width / 2 + last.x * scale, height / 2 - last.y * scale, 6, 0, Math.PI * 2);
  ctx.fill();
}

function render(snapshot) {
  ui.posX.textContent = fmt(snapshot.position.x, " m");
  ui.posY.textContent = fmt(snapshot.position.y, " m");
  ui.posZ.textContent = fmt(snapshot.position.z, " m");
  ui.speed.textContent = fmt(snapshot.speed, " m/s");
  ui.attitude.textContent =
    `roll ${snapshot.attitude.roll.toFixed(1)}°, ` +
    `pitch ${snapshot.attitude.pitch.toFixed(1)}°, ` +
    `yaw ${snapshot.attitude.yaw.toFixed(1)}°`;
  ui.sampleRate.textContent = `${snapshot.sampleHz.toFixed(0)} Hz`;
  ui.stationary.textContent = String(snapshot.stationary);
  ui.quality.textContent = snapshot.quality;
  drawTrajectory();
}

function onMotion(event) {
  const sample = normalizeMotionEvent(event);

  if (state.calibratingUntil > performance.now()) {
    state.calibration.push(sample);
    setStatus("校准中", "warn");
    return;
  }

  if (state.calibration.length) {
    nav.calibrateStill(state.calibration);
    state.calibration = [];
    setStatus("运行中", "good");
  }

  const snapshot = nav.update(sample);
  state.path.push(snapshot.position);
  if (state.path.length > 1400) state.path.shift();
  render(snapshot);
}

function onOrientation(event) {
  nav.setAbsoluteOrientation({
    alpha: event.alpha,
    beta: event.beta,
    gamma: event.gamma,
  });
}

async function requestSensorPermission() {
  if (typeof DeviceMotionEvent !== "undefined" && DeviceMotionEvent.requestPermission) {
    const motion = await DeviceMotionEvent.requestPermission();
    if (motion !== "granted") throw new Error("DeviceMotion 权限被拒绝");
  }

  if (typeof DeviceOrientationEvent !== "undefined" && DeviceOrientationEvent.requestPermission) {
    const orientation = await DeviceOrientationEvent.requestPermission();
    if (orientation !== "granted") throw new Error("DeviceOrientation 权限被拒绝");
  }
}

async function start() {
  if (state.running) return;

  try {
    await requestSensorPermission();
    window.addEventListener("devicemotion", onMotion, { passive: true });
    window.addEventListener("deviceorientation", onOrientation, { passive: true });
    state.running = true;
    ui.start.textContent = "已启动";
    setStatus("运行中", "good");
  } catch (error) {
    setStatus("权限失败", "bad");
    ui.quality.textContent = error.message;
  }
}

function calibrate() {
  if (!state.running) {
    setStatus("先启动", "warn");
    return;
  }
  state.calibration = [];
  state.calibratingUntil = performance.now() + 2500;
  nav.velocity = { x: 0, y: 0, z: 0 };
  setStatus("静置手机", "warn");
}

function reset() {
  nav.reset();
  state.path = [];
  state.calibration = [];
  state.calibratingUntil = 0;
  render(nav.snapshot());
  setStatus(state.running ? "运行中" : "未启动", state.running ? "good" : "");
}

ui.start.addEventListener("click", start);
ui.calibrate.addEventListener("click", calibrate);
ui.reset.addEventListener("click", reset);

render(nav.snapshot());
drawTrajectory();
