# PICO + SONIC Whole-Body + MuJoCo 精度与延迟实验设计

本文用于量化当前双机链路：PICO/XRoboToolkit 与 MuJoCo 在本机运行，SONIC v1.1
Whole-Body/TensorRT 在配置文件指定的 GPU 主机运行。机器人模型为 G1 29 个身体关节加左右
Dex3 手各 7 个执行器，共 43 个执行器。

实验工具是**被动订阅器**，不会打开第二个 XRoboToolkit SDK 客户端，也不会向机器人
发送控制。运行时不要另起任何直接调用 `xrobotoolkit_sdk` 的诊断进程。

## 1. 测量目标与验收分层

链路精度分三层，三者不能混为一个数字：

1. **内部控制跟踪精度**：SONIC/WBC 的目标关节、VR 三点任务与 MuJoCo 实测状态之差。
2. **PICO 跟踪精度**：PICO 对真实头部、手柄和 tracker 运动的重复性、漂移和绝对误差。
3. **端到端体验**：人体开始运动到 MuJoCo 机器人产生可见运动的总延迟。

本仓库可自动完成第 1 层、PICO 采样质量、软件往返延迟和安全降级时间。第 2 层的
绝对精度需要外部相机/光学动捕真值；第 3 层的 motion-to-photon 需要高速相机。

建议的通过条件如下。它们是当前仿真链路的工程门槛，不是 PICO 或 SONIC 官方规格。

| 项目 | 通过门槛 |
|---|---:|
| PICO pose 有效频率 | 中位持续频率 ≥ 45 Hz |
| PICO 帧间隔 P99 | < 50 ms |
| 活跃段 pose/source 丢帧估计 | < 1% |
| 本地 manager 样本接收年龄 P95 | < 20 ms |
| pose → 远端 WBC target → 本地反馈往返 P95 | < 150 ms |
| 活跃身体关节目标/实测 RMSE | < 10° |
| 活跃身体关节滞后中位数 | < 160 ms |
| 活跃单关节滞后 | < 250 ms，相关系数 ≥ 0.5 |
| VR 三点到实测 FK 位置 P95 | < 0.15 m |
| VR 三点到实测 FK 姿态 P95 | < 20° |
| 双手 7-DoF 目标/实测 RMSE | < 15° |
| 独立站立 30 s | `z<0.5 m` 为 0，roll P95 < 5°，pitch P95 < 10° |
| PICO 失联到 STOP | 4.5–6.5 s（当前 stale timeout 为 5 s） |
| STOP 后 WBC 不再产生反馈 | ≤ 1.5 s；重新启动不复用旧目标 |

## 2. 时间轴与采集点

```text
PICO device timestamp
  → XRoboToolkit PC Service
  → manager sample timestamp (本机 monotonic)
  → pose ZMQ :5556                  ┐
  → 4090 SONIC/WBC                  │ 软件往返：全部用本机接收时钟
  → g1_debug ZMQ :5557              ┘
  → body command DDS/TCP bridge
  → MuJoCo measured q / odostate (本机 DDS)
  → viewer draw/scan-out（仅高速相机可测）
```

`record_pico_sonic_trace.py` 将以下流按到达时间追加到 SQLite，不在内存中积累长实验：

| 流 | 来源 | 主要字段 |
|---|---|---|
| `pose` | 本机 manager :5556 | frame index、PICO device ns、manager monotonic、SMPL、VR 三点、手目标 |
| `planner` | 本机 manager :5556 | locomotion、速度、朝向、VR 三点、双手目标 |
| `manager_state` | 本机 manager :5556 | OFF/POSE/PLANNER/VR_3PT 模式 |
| `command` | 本机 manager :5556 | start、stop、planner；用于失联 STOP 计时 |
| `g1_debug` | 4090 :5557 | 29-DoF 目标/实测、dq、双手、基座、WBC 实际收到的 VR 三点 |
| `odostate` | 本机 `rt/odostate` | MuJoCo 基座位置、四元数、线/角速度 |
| `marker` | 本机 UDP :5572 | 实验动作分段标签 |

### 两台主机的时钟

两台主机的 wall clock 不能直接相减。2026-08-28 的 200 次实际探针结果为：

- `offset(remote-local) = -84.925 ms`；
- min RTT `1.674 ms`，该次 offset 不确定度上界 `0.837 ms`；
- 原始结果：`logs/teleop_experiments/clock-probe-20260828-185400/clock_probe.json`。

负 offset 表示 4090 的墙钟比本机慢。校正关系为：

```text
t_remote_on_local_clock = t_remote - offset_remote_minus_local
```

探针不修改 NTP 或系统时钟。由于 Wi-Fi 往返路径可能不对称，一次 NTP 四时间戳估计的
单向误差至少为 min-RTT/2；正式实验在开始、中间、结束各测一次并报告漂移。自动分析的
核心延迟使用同一台本机的 monotonic 接收时钟做 round-trip，不依赖上述 offset。

远端 `Streaming data mean delay` 是远端收到最新 streaming 数据后的年龄，不是
PICO→MuJoCo viewer 的端到端延迟，也不应与本机 wall clock 数字混算。

## 3. 自动指标定义

设目标关节为 `q_t[i,k]`，MuJoCo 实测为 `q_m[i,k]`：

```text
error[i,k] = wrap/直接差(q_t[i,k] - q_m[i,k])
RMSE[k] = sqrt(mean_i(error[i,k]^2))
P95[k] = percentile_i(abs(error[i,k]), 95)
```

本模型关节工作范围不跨越 ±π 跳变，当前实现使用直接差。目标活动范围小于 3° 的关节
不进入“活跃关节滞后”汇总。

关节滞后用 0–500 ms 的非负互相关搜索：

```text
lag[k] = argmax_tau corr(q_t(t), q_m(t + tau))
```

只有信号长度足够且具有运动幅值的关节计算 lag。`pose→g1_debug` 使用同一本机接收的
两个不规则时间序列，重采样到 50 Hz 后搜索 0–400 ms，因此包含 local→GPU 与
GPU→local 两个 Wi-Fi 方向以及远端处理，是稳健的软件往返指标。

任务空间误差用正确的 `scene_43dof.xml` 对实测 29-DoF 做 MuJoCo FK，并使用官方
控制点偏置：

```text
left wrist  [0.18, -0.025, 0.0] m
right wrist [0.18,  0.025, 0.0] m
torso/head  [0.0,   0.0,   0.35] m
```

基座使用 `base_quat_measured`，比较点为左右 wrist yaw link 和 torso link。该误差是
“WBC 任务目标与机器人响应”的误差，不是 PICO 对真实世界的绝对误差。

稳定性由 odometry 计算基座高度、水平漂移、速度、roll/pitch 以及 `z < 0.5 m` 样本数。
丢帧由 source index 正向跳变估算；接收频率、P50/P95/P99 间隔和间隔标准差同时报告。

## 4. 实验前准备

1. PICO 的 XRoboToolkit-PICO 处于 `WORKING / Full Body`；两个手柄和两个脚踝 tracker
   已校准。
2. 操作者周围无障碍物；全身、浅蹲和踏步动作均缓慢、小幅。
3. 启动完整链路：

```bash
cd "$HOME/Work/GR00T-WholeBodyControl"
./scripts/start_pico_sonic_mujoco.sh
```

4. 等远端 `Init Done` 后按 A+B+X+Y 进入 PLANNER。点击 MuJoCo viewer 按一次 `9`
   解除 band，独立站稳至少 30 s 后才进入 POSE。
5. 另开一个**本机**终端执行实验脚本。不要在 recorder 之外打开 XRT SDK。

实验脚本开始前会被动采样 `manager_state` 和 `rt/odostate`：baseline 必须稳定处于
PLANNER，其他协议必须稳定处于 POSE，且基座高度不能低于跌倒阈值。采集器写出
`recorder.ready.json` 后 marker 调度才开始，因此动作分段以真实采集起点为准。

### 时钟探针

在 4090 终端启动只读 server：

```bash
"$HOME/miniforge3/envs/groot-wbc-sonic-trt1013/bin/python" \
  "$HOME/Work/GR00T-WholeBodyControl/gear_sonic/scripts/clock_probe.py" \
  --server --bind 0.0.0.0 --port 5570 --duration 30
```

本机同时执行：

```bash
RUN=logs/teleop_experiments/clock-$(date +%Y%m%d-%H%M%S)
"$HOME/miniforge3/envs/groot-wbc-pico/bin/python" \
  gear_sonic/scripts/clock_probe.py --client --host "$WBC_HOST" \
  --port 5570 --count 200 --interval 0.02 --output "$RUN/clock_probe.json"
```

## 5. 实验矩阵与实际动作

每个正常实验至少重复 3 次；左右不对称动作交换左右再做一组。实验脚本会在固定时刻
自动发 marker，蜂鸣后按终端提示动作。动作切换尽量在 marker 时刻完成。

### E0：独立站立基线

```bash
./scripts/run_pico_sonic_experiment.sh --protocol baseline
```

保持 PLANNER/IDLE、band 已解除、操作者不动 30 s。检查跌倒、高度、roll/pitch、速度和
网络接收稳定性。若此项不通过，不进行 PICO 动作实验。

### E1：静态姿态与漂移

```bash
./scripts/run_pico_sonic_experiment.sh --protocol static
```

进入 POSE，依次保持：中立 15 s、双臂前伸 15 s、双臂侧平举 15 s、躯干缓慢左右转
20 s、回中立。每个保持段最后 10 s 用于静态抖动/漂移统计。

### E2：右臂频率响应

```bash
./scripts/run_pico_sonic_experiment.sh --protocol right_arm
```

中立 10 s；右臂侧抬，以节拍器做 0.5 Hz 20 s；休息 10 s；做 1.0 Hz 20 s；回中立。
报告右肩/肘/腕幅值比、RMSE、滞后与相关系数。幅度以舒适、安全为限，不能快速甩臂。

### E3：全身覆盖

```bash
./scripts/run_pico_sonic_experiment.sh --protocol whole_body
```

依次执行左臂、右臂、躯干 yaw、浅蹲、原地小幅踏步，各约 20 s，最后回中立。验证
29 个身体关节的有限值、动作范围、双臂/躯干/下肢协同与站立安全。

### E4：双手

```bash
./scripts/run_pico_sonic_experiment.sh --protocol hands
```

身体保持静止：左 trigger 5 次、右 trigger 5 次、左右交替 5 次、同时 5 次。当前官方
helper 是 trigger→预设手指 IK，不是独立手指动捕；只评价 7-DoF 目标响应和延迟。

### E5：PICO 失联降级

```bash
./scripts/run_pico_sonic_experiment.sh --protocol safety
```

先在 POSE 保持小幅静态目标。看到/听到 `safety:stop_pico_now` 时，将 XRoboToolkit
退出前台或停止 Full Body 数据，但不杀 recorder。预期最后设备时间戳后约 5 s，manager
发出官方 STOP 并退出；远端 planner 在约 1 s 内回 IDLE/退出。重新启动时必须重新校准，
不能自动恢复旧目标。

### 手工 marker

需要补充动作点时执行：

```bash
./scripts/send_trace_marker.sh "whole_body:custom_event"
```

## 6. 输出与复算

每次实验输出到：

```text
logs/teleop_experiments/<protocol>-<timestamp>/
├── trace.sqlite3
└── analysis/
    ├── summary.md
    ├── summary.json
    ├── joint_error.csv
    ├── markers.csv
    └── segments.csv
```

离线复算不会连接 PICO，也不会运行策略：

```bash
"$HOME/miniforge3/bin/conda" run --no-capture-output -n groot-wbc-pico \
  python gear_sonic/scripts/analyze_pico_sonic_trace.py \
  logs/teleop_experiments/<run>/trace.sqlite3 \
  --output-dir logs/teleop_experiments/<run>/analysis-rerun \
  --model-xml gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml
```

分析器数值单元测试：

```bash
"$HOME/miniforge3/bin/conda" run --no-capture-output -n groot-wbc-pico \
  python gear_sonic/scripts/analyze_pico_sonic_trace.py --self-test
```

预期：注入 80 ms，回算 `80.0 ms`，correlation 接近 1。

## 7. 外部绝对精度实验

没有外部真值时，不能声称测得 PICO 的“绝对精度”。推荐方案优先级如下：

1. Vicon/OptiTrack：头显、两个手柄、两个 tracker 各固定一组反光 marker；100–200 Hz
   同步记录刚体位姿。
2. 低成本方案：每个设备固定 AprilTag 刚体板，用已标定的双目/多目相机 60–120 Hz
   记录，避免单目深度误差被误认为 PICO 误差。

步骤：

1. 采集不少于 20 个覆盖工作空间的静态姿态，用刚体变换/Umeyama 求
   `T_truth_to_pico`，校准样本与测试样本分开。
2. 静态点每点保持 10 s，报告位置 mean/RMSE/P95/max、姿态 geodesic angle
   RMSE/P95、10 s 漂移和帧间抖动。
3. 动态做 0.25/0.5/1.0 Hz、10–40 cm 幅值的单轴正弦，互相关求滞后，拟合幅值比。
4. 全程保留原始真值/PICO timestamp；用 TTL LED、屏幕 flash 或共同可见冲击事件对时。
5. 至少 3 轮重启/重新校准，分别报告“单次校准重复性”和“跨校准重复性”。

## 8. 高速相机端到端延迟

以 240 fps 或更高帧率让同一画面同时看到操作者手柄和 MuJoCo viewer。操作者做明显但
安全的单次方向反转；逐帧标注真实手柄首次运动帧 `F_h` 与机器人首次可见运动帧 `F_r`：

```text
motion_to_photon_ms = (F_r - F_h) / camera_fps * 1000
quantization_uncertainty = ±1 frame
```

至少 30 次，报告 P50/P95/max。相机方案包括 PICO 跟踪、Wi-Fi、manager、GPU、WBC、
DDS bridge、MuJoCo 和显示扫描，是最终体验指标；它应大于软件 round-trip 的相应部分，
两者不可互相替代。

## 9. 网络鲁棒性扩展

只有在独占测试网络且确认不会影响其他用户时，才用 Linux `tc netem` 注入 10/30/50 ms
延迟、1/3/5% 丢包和 5 ms jitter。公用 4090 与当前 Wi-Fi 不执行此破坏性实验。每档
重复 E0/E2，并确认超过安全边界时 fail closed，而不是维持陈旧动作。

## 10. 已完成的工具验收（2026-08-28）

| 验收 | 实际结果 |
|---|---|
| Python/Bash 静态检查 | recorder、analyzer、clock probe、runner 全部通过 |
| 空链路 2 s | SQLite 和 `N/A` 报告正常生成，无连接时不崩溃 |
| 合成完整流 500 帧 | pose/g1_debug/odom/29-DoF/双手/FK/markers 全部解析 |
| 合成关节滞后 | 注入 80 ms，回算 80.00 ms |
| 合成安全时序 | marker→STOP 5000 ms，last pose→STOP 3 ms，STOP→last feedback 1000 ms |
| 本机 clock probe 50 次 | min RTT 0.204 ms，offset 0.036 ms，不确定度 ≤ 0.102 ms |
| 本机↔4090 clock probe 200 次 | min RTT 1.674 ms，offset -84.925 ms，不确定度 ≤ 0.837 ms |
| 真实 keyboard SONIC 启动 | `Init Done`、CONTROL、Planner/IDLE 均成功；最后用官方 `O` 正常退出 |
| E0 现场尝试 | 未完成：180 s 内 viewer 未收到物理键 9，基座仍被 band 悬挂，不能算站立通过 |
| E0 PICO 首次采集 | 作废：已落地但误按 A+X 进入 POSE；preflight/mode gate 因此加入正式 runner |

E0 尝试保留了三组只读 trace：

- `keyboard-pre-band-20260828-1903`：5 s，`g1_debug 50.91 Hz`、odometry
  `152.69 Hz`、基座 z 中位数 `0.9645 m`；
- `keyboard-post-band-20260828-1904`：12 s，辅助总线字符键未被 GLFW 接收，z 中位数
  仍为 `0.9645 m`；
- `keyboard-keycode9-check-20260828-1905`：6 s，后台物理 keycode 同样被桌面焦点策略隔离，
  z 中位数 `0.9641 m`。

这些数据只验证采集器在真实 SONIC/MuJoCo 流上的工作频率，不用于站立、关节误差或
遥操精度结论。下次现场续跑必须点击 MuJoCo viewer 手动按一次 `9`，确认 z 降到约
`0.79 m` 后，重新执行正式 30 s baseline。

随后 `pico-e0-baseline-20260828-1920` 虽已成功落地并采到 pose/g1_debug/odometry，
但 manager 日志确认基线期间发生 `PLANNER -> POSE`；关节 RMSE `18.79°`、基座最小
z `0.417 m` 和约 `6.25 m` 水平位移都证明它不是静止基线。该 trace 明确标记为
**invalid**，不能用于性能结论。由此加入两项自动保护：baseline 开始前必须连续验证
PLANNER 与站立高度；recorder ready 后才开始 marker 时间表。

此前真实硬件链已完成约 80 s POSE 全身动作、48–50 Hz PICO、右臂覆盖、独立站立和
PICO stale STOP；详见 `docs/VALIDATION_20260828_PICO_SONIC_MUJOCO.md`。这些旧数据没有
使用本次新增 SQLite recorder，因此不能反推新增指标。正式的 E0–E5 要在链路重新启动
且操作者按动作协议配合时逐项生成 trace。
