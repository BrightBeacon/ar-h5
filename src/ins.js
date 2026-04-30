const G = 9.80665;
const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function norm3(v) {
  return Math.hypot(v.x, v.y, v.z);
}

function lowPass(prev, next, alpha) {
  return {
    x: prev.x + (next.x - prev.x) * alpha,
    y: prev.y + (next.y - prev.y) * alpha,
    z: prev.z + (next.z - prev.z) * alpha,
  };
}

function quatNormalize(q) {
  const n = Math.hypot(q.w, q.x, q.y, q.z) || 1;
  return { w: q.w / n, x: q.x / n, y: q.y / n, z: q.z / n };
}

function quatMultiply(a, b) {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

function quatConjugate(q) {
  return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
}

function quatRotate(q, v) {
  const p = { w: 0, x: v.x, y: v.y, z: v.z };
  const r = quatMultiply(quatMultiply(q, p), quatConjugate(q));
  return { x: r.x, y: r.y, z: r.z };
}

function quatSlerp(a, b, t) {
  let cosHalfTheta = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  let target = b;

  if (cosHalfTheta < 0) {
    target = { w: -b.w, x: -b.x, y: -b.y, z: -b.z };
    cosHalfTheta = -cosHalfTheta;
  }

  if (cosHalfTheta > 0.9995) {
    return quatNormalize({
      w: a.w + t * (target.w - a.w),
      x: a.x + t * (target.x - a.x),
      y: a.y + t * (target.y - a.y),
      z: a.z + t * (target.z - a.z),
    });
  }

  const halfTheta = Math.acos(clamp(cosHalfTheta, -1, 1));
  const sinHalfTheta = Math.sqrt(1 - cosHalfTheta * cosHalfTheta);
  const ratioA = Math.sin((1 - t) * halfTheta) / sinHalfTheta;
  const ratioB = Math.sin(t * halfTheta) / sinHalfTheta;

  return {
    w: a.w * ratioA + target.w * ratioB,
    x: a.x * ratioA + target.x * ratioB,
    y: a.y * ratioA + target.y * ratioB,
    z: a.z * ratioA + target.z * ratioB,
  };
}

function quatFromEuler(alpha, beta, gamma) {
  const z = alpha * DEG2RAD;
  const x = beta * DEG2RAD;
  const y = gamma * DEG2RAD;
  const cz = Math.cos(z / 2);
  const sz = Math.sin(z / 2);
  const cx = Math.cos(x / 2);
  const sx = Math.sin(x / 2);
  const cy = Math.cos(y / 2);
  const sy = Math.sin(y / 2);

  return quatNormalize({
    w: cz * cx * cy - sz * sx * sy,
    x: cz * sx * cy - sz * cx * sy,
    y: cz * cx * sy + sz * sx * cy,
    z: sz * cx * cy + cz * sx * sy,
  });
}

function eulerFromQuat(q) {
  const sinr = 2 * (q.w * q.x + q.y * q.z);
  const cosr = 1 - 2 * (q.x * q.x + q.y * q.y);
  const roll = Math.atan2(sinr, cosr);

  const sinp = 2 * (q.w * q.y - q.z * q.x);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);

  const siny = 2 * (q.w * q.z + q.x * q.y);
  const cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
  const yaw = Math.atan2(siny, cosy);

  return { roll: roll * RAD2DEG, pitch: pitch * RAD2DEG, yaw: yaw * RAD2DEG };
}

function integrateGyro(q, gyro, dt) {
  const omega = {
    w: 0,
    x: gyro.x * DEG2RAD,
    y: gyro.y * DEG2RAD,
    z: gyro.z * DEG2RAD,
  };
  const dq = quatMultiply(q, omega);
  return quatNormalize({
    w: q.w + 0.5 * dq.w * dt,
    x: q.x + 0.5 * dq.x * dt,
    y: q.y + 0.5 * dq.y * dt,
    z: q.z + 0.5 * dq.z * dt,
  });
}

export class InertialNavigator {
  constructor(options = {}) {
    this.options = {
      orientationGain: options.orientationGain ?? 0.035,
      accelNoiseFloor: options.accelNoiseFloor ?? 0.035,
      stationaryAccelThreshold: options.stationaryAccelThreshold ?? 0.16,
      stationaryGyroThreshold: options.stationaryGyroThreshold ?? 1.4,
      zuptGain: options.zuptGain ?? 0.72,
      velocityDamping: options.velocityDamping ?? 0.018,
      positionDampingWhenStationary: options.positionDampingWhenStationary ?? 0.08,
    };

    this.reset();
  }

  reset() {
    this.q = { w: 1, x: 0, y: 0, z: 0 };
    this.position = { x: 0, y: 0, z: 0 };
    this.velocity = { x: 0, y: 0, z: 0 };
    this.accelBias = { x: 0, y: 0, z: 0 };
    this.gyroBias = { x: 0, y: 0, z: 0 };
    this.filteredLinearAccel = { x: 0, y: 0, z: 0 };
    this.lastTimestamp = 0;
    this.stationaryScore = 0;
    this.samples = 0;
    this.sampleHz = 0;
    this.quality = "等待数据";
  }

  setAbsoluteOrientation({ alpha, beta, gamma }) {
    if (![alpha, beta, gamma].every(Number.isFinite)) return;
    const measured = quatFromEuler(alpha, beta, gamma);
    this.q = quatSlerp(this.q, measured, this.options.orientationGain);
  }

  calibrateStill(samples) {
    if (!samples.length) return;
    const gyro = { x: 0, y: 0, z: 0 };
    const accel = { x: 0, y: 0, z: 0 };

    for (const s of samples) {
      gyro.x += s.gyro.x;
      gyro.y += s.gyro.y;
      gyro.z += s.gyro.z;
      accel.x += s.linearAccel.x;
      accel.y += s.linearAccel.y;
      accel.z += s.linearAccel.z;
    }

    const inv = 1 / samples.length;
    this.gyroBias = { x: gyro.x * inv, y: gyro.y * inv, z: gyro.z * inv };
    this.accelBias = { x: accel.x * inv, y: accel.y * inv, z: accel.z * inv };
    this.velocity = { x: 0, y: 0, z: 0 };
    this.quality = "已静止校准";
  }

  fusePositionObservation(observation, options = {}) {
    const gain = clamp(options.gain ?? 0.18, 0, 1);
    const velocityGain = clamp(options.velocityGain ?? 0.08, 0, 1);
    const observed = {
      x: Number(observation.x) || 0,
      y: Number(observation.y) || 0,
      z: Number(observation.z) || 0,
    };
    const residual = {
      x: observed.x - this.position.x,
      y: observed.y - this.position.y,
      z: observed.z - this.position.z,
    };

    this.position.x += residual.x * gain;
    this.position.y += residual.y * gain;
    this.position.z += residual.z * gain;

    if (Number.isFinite(options.dt) && options.dt > 0) {
      this.velocity.x += residual.x / options.dt * velocityGain;
      this.velocity.y += residual.y / options.dt * velocityGain;
      this.velocity.z += residual.z / options.dt * velocityGain;
    }
  }

  update(sample) {
    const timestamp = sample.timestamp || performance.now();
    const rawDt = this.lastTimestamp ? (timestamp - this.lastTimestamp) / 1000 : 0;
    this.lastTimestamp = timestamp;

    if (!rawDt) return this.snapshot();

    const dt = clamp(rawDt, 0.001, 0.05);
    this.samples += 1;
    this.sampleHz = 0.9 * this.sampleHz + 0.1 * (1 / dt);

    const gyro = {
      x: sample.gyro.x - this.gyroBias.x,
      y: sample.gyro.y - this.gyroBias.y,
      z: sample.gyro.z - this.gyroBias.z,
    };
    const linearDevice = {
      x: sample.linearAccel.x - this.accelBias.x,
      y: sample.linearAccel.y - this.accelBias.y,
      z: sample.linearAccel.z - this.accelBias.z,
    };

    this.q = integrateGyro(this.q, gyro, dt);
    const worldAccel = quatRotate(this.q, linearDevice);
    const filtered = {
      x: Math.abs(worldAccel.x) < this.options.accelNoiseFloor ? 0 : worldAccel.x,
      y: Math.abs(worldAccel.y) < this.options.accelNoiseFloor ? 0 : worldAccel.y,
      z: Math.abs(worldAccel.z) < this.options.accelNoiseFloor ? 0 : worldAccel.z,
    };
    this.filteredLinearAccel = lowPass(this.filteredLinearAccel, filtered, 0.42);

    const accelMag = norm3(this.filteredLinearAccel);
    const gyroMag = norm3(gyro);
    const stationary =
      accelMag < this.options.stationaryAccelThreshold &&
      gyroMag < this.options.stationaryGyroThreshold;
    this.stationaryScore = clamp(this.stationaryScore + (stationary ? dt * 3.5 : -dt * 5), 0, 1);

    this.velocity.x += this.filteredLinearAccel.x * dt;
    this.velocity.y += this.filteredLinearAccel.y * dt;
    this.velocity.z += this.filteredLinearAccel.z * dt;

    const damping = Math.exp(-this.options.velocityDamping * dt);
    this.velocity.x *= damping;
    this.velocity.y *= damping;
    this.velocity.z *= damping;

    if (this.stationaryScore > 0.75) {
      const gain = this.options.zuptGain;
      this.velocity.x *= 1 - gain;
      this.velocity.y *= 1 - gain;
      this.velocity.z *= 1 - gain;
      this.position.x *= 1 - this.options.positionDampingWhenStationary * dt;
      this.position.y *= 1 - this.options.positionDampingWhenStationary * dt;
      this.position.z *= 1 - this.options.positionDampingWhenStationary * dt;
    }

    this.position.x += this.velocity.x * dt;
    this.position.y += this.velocity.y * dt;
    this.position.z += this.velocity.z * dt;

    this.quality = this.sampleHz > 45 ? "高" : this.sampleHz > 25 ? "中" : "低";
    return this.snapshot();
  }

  snapshot() {
    return {
      attitude: eulerFromQuat(this.q),
      position: { ...this.position },
      velocity: { ...this.velocity },
      speed: norm3(this.velocity),
      stationary: this.stationaryScore > 0.75,
      sampleHz: this.sampleHz,
      quality: this.quality,
    };
  }
}

export function normalizeMotionEvent(event) {
  const accel = event.acceleration || {};
  const rotation = event.rotationRate || {};

  return {
    timestamp: event.timeStamp || performance.now(),
    linearAccel: {
      x: Number(accel.x) || 0,
      y: Number(accel.y) || 0,
      z: Number(accel.z) || 0,
    },
    gyro: {
      alpha: Number(rotation.alpha) || 0,
      beta: Number(rotation.beta) || 0,
      gamma: Number(rotation.gamma) || 0,
      x: Number(rotation.beta) || 0,
      y: Number(rotation.gamma) || 0,
      z: Number(rotation.alpha) || 0,
    },
  };
}

export { G };
