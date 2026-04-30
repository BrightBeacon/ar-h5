# H5 惯性导航实验台

这是一个无依赖的 iOS / Android H5 惯性导航实现，包含：

- `src/ins.js`：惯导核心，四元数姿态积分、浏览器姿态融合、重力剔除、零速更新、漂移抑制。
- `src/app.js`：H5 传感器权限、`devicemotion` / `deviceorientation` 采集和页面渲染。
- `index.html`：可直接部署到 HTTPS 或通过 localhost 测试的调试界面。

## 运行

```bash
python3 -m http.server 8080
```

然后用手机打开同一网络下的地址，或在本机打开：

```text
http://localhost:8080
```

iOS Safari 需要用户点击按钮后才允许请求运动传感器权限。生产环境必须使用 HTTPS；localhost 通常可用于本地调试。

## 精度说明

手机 IMU 单独做位置积分会快速漂移，H5 环境还会受到采样率、时间戳、系统滤波、浏览器权限和设备差异影响。这个实现已经使用了适合 H5 的高精度工程策略：

- 四元数姿态表示，避免欧拉角万向节锁。
- 陀螺积分 + `deviceorientation` 姿态慢融合。
- 使用 `event.acceleration` 线加速度，避免重复估算重力。
- 静止检测 + ZUPT 零速更新，降低短距离漂移。
- 静止校准陀螺和加速度偏置。
- `fusePositionObservation()` 外部观测融合接口，可接 GPS、UWB、BLE、视觉 SLAM 或地图约束来压住长期漂移。

如果要达到厘米级或长时间稳定导航，需要融合外部观测，例如 GPS、UWB、BLE AoA、视觉 SLAM、地图约束或服务端滤波。
