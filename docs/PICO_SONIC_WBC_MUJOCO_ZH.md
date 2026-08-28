# PICO + SONIC v1.1 Whole-Body + MuJoCo 操作手册

本文只描述 NVIDIA 官方 `GR00T-WholeBodyControl` 的 **SONIC Whole-Body** 链路；不使用
`decoupled_wbc`。当前基线为官方提交 `a0732b6`（tag/message `v1.1`）。

## 1. 系统边界

机器人不是“29 DoF 或 43 DoF”二选一：

- SONIC 策略控制 29 个 G1 身体关节；decoder 输出为 `[1, 29]`。
  - 左右 Dex3 手各 7 个执行器，由 PICO controller 数据经手部 IK 单独控制。
- MuJoCo 场景因此共有 `29 + 7 + 7 = 43` 个执行器。
- 正确场景是
  `gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml`。
- 正确 WBC 配置是
  `gear_sonic/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml`。

本项目采用两机模式：

```text
PICO 4 / 手柄 / 2 个脚踝追踪器
  → XRoboToolkit-PICO APK
  → 本机工作站: XRoboToolkit PC Service + Python manager
  → ZMQ :5556（pose / planner / command）
  → GPU 主机: 官方 g1_deploy_onnx_ref
       ├─ SONIC v1.1 encoder (1751 → token 64)
       ├─ SONIC v1.1 decoder (observation 994 → action 29)
       └─ planner_sonic (qpos [4,36] → [64,36])
       ↕ 本机 lo DDS
    远端 CDR/ZMQ bridge
       ↕ TCP :5560（state）/:5561（command）
    本机 CDR/ZMQ bridge
       ↕ 本机 lo DDS
  → 本机 MuJoCo 43-actuator viewer
  → ZMQ :5557（g1_debug）返回 manager
```

4090 主机不启动 MuJoCo、PICO 或桌面窗口。官方 C++ 程序把 TensorRT 推理和 WBC
控制循环编译在同一进程中，因此整个 `g1_deploy_onnx_ref` 进程位于 GPU 主机；其他
组件均留在本机。两端 Unitree DDS 都使用官方同机仿真的 `lo`；桥只转发 CycloneDDS
原始 CDR 字节，不改字段、关节顺序或数值。这样避免 DDS multicast/高频双向 topic
直接跨双 Wi-Fi 主机时的断流。

## 2. 固定环境

### 本机：PICO + MuJoCo

```bash
cd "$HOME/Work/GR00T-WholeBodyControl"
./scripts/setup_local_pico_env.sh
"$HOME/miniforge3/bin/conda" run -n groot-wbc-pico python - <<'PY'
import mujoco, torch, pinocchio, xrobotoolkit_sdk
print("mujoco", mujoco.__version__)
print("torch", torch.__version__, "CUDA", torch.cuda.is_available())
print("pinocchio", pinocchio.__version__)
print("XRoboToolkit binding OK")
PY
```

本机没有 NVIDIA GPU 也可以运行；这里的 PyTorch 是 CPU 版本。

### 4090 主机：TensorRT 10.13

官方仓库明确要求 x86 部署使用 TensorRT 10.13。环境名固定为
`groot-wbc-sonic-trt1013`，不得安装到系统 Python：

```bash
cd "$HOME/Work/GR00T-WholeBodyControl"
set -a; source config/pico_wbc_split.env; set +a
ssh "${REMOTE_USER}@${WBC_HOST}"
cd "$HOME/Work/GR00T-WholeBodyControl"
./scripts/setup_remote_sonic_env.sh
```

该脚本使用 `environment.sonic-trt1013.yml` 创建独立 Conda 环境，并安装：

- Python 3.10、CUDA Toolkit 12.9、TensorRT 10.13.3.9；
- NVIDIA 官方同版本 C++ headers/ONNX parser development package；
- ONNX Runtime GPU 1.19.2；
- CMake、Ninja、Eigen、ZeroMQ/cppzmq、msgpack、Boost、GTest 等构建依赖；
- 仅供跨机桥使用的 CycloneDDS 0.10.2、pyzmq 26.4.0 和 Unitree SDK2 Python 1.0.1；
- 独立构建目录 `gear_sonic_deploy/build-trt1013`。

验证：

```bash
./scripts/run_remote_sonic_wbc.sh --check
```

预期包含 `TensorRT 10.13.3.9`、`CUDA 12.9`，且 `ldd` 中
`libnvinfer.so.10`、`libnvonnxparser.so.10` 和 `libcudart.so.12` 都来自
`$HOME/miniforge3/envs/groot-wbc-sonic-trt1013`。旧 TensorRT 版本生成的 `.trt`
缓存不能复用。

## 3. 模型

```bash
cd "$HOME/Work/GR00T-WholeBodyControl"
"$HOME/miniforge3/bin/conda" run -n groot-wbc-pico \
  python gear_sonic/scripts/download_pinned_sonic_v1_1.py
```

必须同时存在且配套：

```text
gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx
gear_sonic_deploy/policy/sonic_v1_1/model_decoder.onnx
gear_sonic_deploy/policy/sonic_v1_1/observation_config.yaml
gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx
```

不要把 default、low-latency、v1.1 的 encoder/decoder/config 混用。固定模型来自
`nvidia/GEAR-SONIC@6733128a3d8a523b1418b06bca3cdf61c8b0987f`；
`config/sonic_v1_1_models.sha256` 记录四个文件的校验值。

## 4. PICO 端安装

PICO 上只需安装 **XRoboToolkit-PICO APK**；不安装 Python、Conda、MuJoCo 或
GR00T。全身追踪还需两个 PICO 手柄及两个绑在脚踝的 PICO Motion Tracker。

1. 在 PICO 设置中开启 Developer Mode 和允许未知来源安装。
2. 用头显浏览器下载并安装
   [XRoboToolkit-PICO 1.1.1](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/tag/v1.1.1)。
3. 或用 USB/ADB 安装：

```bash
adb devices
adb install -r -g XRoboToolkit-PICO-1.1.1.apk
```

4. 在 PICO 系统中配对两个 Motion Tracker，并按系统指引完成全身校准。
5. 启动 XRoboToolkit，在 `PC Service` 中填写 `SIM_HOST_IP` 对应的本机 Wi-Fi IP。
6. 状态必须为 `WORKING`；勾选 `Head`、`Controller`，选择 Data/Control `Send`，
   Motion Tracker 选择 `Full body`。

APK 已安装后，ADB 不是运行链路的一部分；实际数据走同一局域网。

## 5. 本机 XRoboToolkit PC Service

官方系统级安装：

```bash
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
bash /opt/apps/roboticsservice/runService.sh
```

本机当前采用不需要 root 的用户级安装：

```bash
./scripts/start_pico_pc_service.sh
ss -lntp | grep 60061
```

PICO 路径只保留两个必要差异：允许通过 `XROBO_SERVICE_SCRIPT` 指向用户级启动脚本；
复用仓库已有的 PICO 时间戳陈旧检测，在活动遥操时连续 5 秒无新数据即发送官方
STOP 并退出。未修改跟踪坐标、姿态映射或模式切换规则。

> **单客户端约束：** 运行 manager 时不要再启动第二个
> `xrobotoolkit_sdk` Python 进程做按键诊断。PC Service 的数据流是共享的，
> 第二个客户端调用 `xrt.close()` 可使主 manager 的设备时间戳停止。需诊断时
> 只读 manager 日志、`g1_debug` 或 DDS 状态；如必须直读 XRT，先停掉整条控制链。

## 6. 数据规范

XRoboToolkit body frame 是 `(24, 7)`：`[x,y,z,qx,qy,qz,qw]`。位置为米，设备
时间戳为纳秒；manager 丢弃重复时间戳，并以设备时间差计算 `dt`/FPS。默认发送频率
50 Hz，planner 20 Hz；MuJoCo 步长为 0.005 s（200 Hz）。

官方转换为：

- Unity：X-right、Y-up、Z-forward、左手系；
- Robot：X-forward、Y-left、Z-up、右手系；
- 变换 `Unity [x,y,z] → Robot [-x,z,y]`；
- 三点目标使用 root/pelvis 0、left wrist 22、right wrist 23、neck 12；
- 三点位置和姿态均转为 pelvis 局部坐标，四元数送入 WBC 时为 `wxyz`；
- 全身 POSE 路径将 24 个全局旋转转为 SMPL 局部轴角，再由 SONIC v1.1 SMPL
  encoder 消费；
- 手部路径输出左右各 7 个关节目标。当前官方 PICO helper 用左/右
  trigger 分别生成预设手指 IK；grip 数值可读，但不参与该预设的 7-DoF 闭合。

## 7. 网络检查

先创建本机私有配置（此文件不会入 Git）：

```bash
cp config/pico_wbc_split.env.example config/pico_wbc_split.env
# 编辑 WBC_HOST、SIM_HOST_IP、REMOTE_USER、REMOTE_REPO
set -a; source config/pico_wbc_split.env; set +a
```

```bash
# 本机
ip -br -4 addr                    # 确认 SIM_HOST_IP
ping -c 3 "$WBC_HOST"

# GPU 主机
ip -br -4 addr                    # 确认 WBC_HOST
ping -c 3 "$SIM_HOST_IP"
```

跨机只使用 TCP，不让 DDS multicast 直接穿过 Wi-Fi：

- 本机监听 `5556`（PICO command/pose）、`5560`（sim state）、`5561`（WBC command）；
- 远端监听 `5557`（`g1_debug` feedback）；
- MuJoCo 与本机桥使用本机 `lo` DDS；WBC 与远端桥也使用远端 `lo` DDS。

启动后可检查：

```bash
# 本机应看到 GPU 主机与 5560/5561 均为 ESTAB
ss -ntp | grep -E ':5560|:5561'
```

## 8. 启动与操作

先确保 PICO XRoboToolkit 显示 `WORKING` 且 Full body 已开启。

### 推荐：一条命令启动两机

```bash
cd "$HOME/Work/GR00T-WholeBodyControl"
./scripts/start_pico_sonic_mujoco.sh
```

脚本会：

1. 读取 `config/pico_wbc_split.env`；
2. 在本机启动 MuJoCo viewer、PICO manager 和 sim bridge；
3. 通过同一次 SSH 先执行远端 `--check`，验证 TensorRT/CUDA/模型/二进制，再启动
   SONIC/TensorRT WBC 和 WBC bridge；
4. 在终端显示完整远端输出，并把两端日志汇总到
   `logs/pico_wbc_split/orchestrator-<timestamp>/`；
5. Ctrl+C、远端退出或 PICO 失联 STOP 后统一清理本地子进程。

SSH 会在终端询问一次密码；脚本不保存密码。在共用 4090 主机上只使用
`$HOME/miniforge3/envs/groot-wbc-sonic-trt1013`，不写系统 Python、不启动远端 viewer。

看到 `Init Done` 后的操作顺序只有：

```text
A+B+X+Y 短按并松开  -> 校准并启动 PLANNER
点击 MuJoCo 窗口，按 9 -> 解除 elastic band，确认独立站稳
A+X 短按并松开      -> 进入 POSE 全身遥操
A+X 再按一次           -> 返回 PLANNER
A+B+X+Y 再按一次       -> STOP/OFF 并退出
```

不要长按或连按 A+X；模式切换是上升沿触发。

### 备用：两个终端分别启动

### 终端 A：本机

```bash
cd "$HOME/Work/GR00T-WholeBodyControl"
set -a; source config/pico_wbc_split.env; set +a
./scripts/run_local_pico_mujoco.sh
```

这会启动本机 DDS/ZMQ bridge、显示 MuJoCo viewer，同时启动 PICO manager。仅验证
模型/viewer 时：

```bash
./scripts/run_local_pico_mujoco.sh --sim-only
```

### 终端 B：4090 主机

```bash
cd "$HOME/Work/GR00T-WholeBodyControl"
set -a; source config/pico_wbc_split.env; set +a
ssh "${REMOTE_USER}@${WBC_HOST}"
cd "$HOME/Work/GR00T-WholeBodyControl"
set -a; source config/pico_wbc_split.env; set +a
./scripts/run_remote_sonic_wbc.sh
```

远端脚本先启动 WBC 侧 bridge，再启动 GPU WBC。首次运行会为 RTX 4090 生成三个
TensorRT engine。看到以下日志才表示模型与状态链均初始化完成：

```text
Action dimension: 29
Token dimension: 64
Dimension match: Configuration is valid
TensorRT planner model loaded successfully
Init Done
```

### 官方启动顺序

1. 操作者站直、双脚并拢，上臂贴身，前臂向前弯 90°，手掌相对。
2. 同时按 PICO 的 **A+B+X+Y**：完成首次 CALIB_FULL，并从 OFF 进入 PLANNER；
   远端应出现 `transitioning to CONTROL`、`Planner initialized successfully`。
3. 点击 MuJoCo 窗口并按一次 **9**，关闭官方 elastic band，让机器人落地独立平衡。
4. 先保持 PLANNER/IDLE，确认独立站稳，再让身体与机器人姿态对齐。
5. 按 **A+X** 进入 POSE 全身跟随；再按一次返回 PLANNER。
6. 左摇杆控制移动方向，右摇杆水平控制朝向；左右 trigger 控制对应手。

常用模式：

| 操作 | 功能 |
|---|---|
| A+B+X+Y | 启动；运行中再次按为 STOP/OFF |
| A+X | PLANNER ↔ POSE |
| B+Y | POSE ↔ PLANNER_FROZEN_UPPER |
| 左摇杆按下 | Planner 模式 ↔ VR_3PT，并重新校准手腕 |
| A+B / X+Y | 下一个 / 上一个 locomotion mode |
| C++ 终端 O | 立即停止 WBC |
| MuJoCo 窗口 9 | 开/关 elastic band |

进入 POSE 或 VR_3PT 前必须先对齐人和机器人姿态；否则目标会发生阶跃。

## 9. 验收

依次执行，不能跳过：

1. `run_local_pico_mujoco.sh --sim-only`：场景加载、43 个 actuator、viewer 正常。
2. `run_remote_sonic_wbc.sh --check`：TensorRT/CUDA/链接路径均为隔离环境。
3. 启动两端，远端到 `Init Done`，再启动策略；保持 band 时只用于防坠落。
4. 按 9 后至少观察 30 秒：机器人不倒地、不持续振荡，roll/pitch 与关节速度收敛。
5. PLANNER/IDLE、缓慢行走和回到 IDLE 均稳定。
6. PICO 数据日志持续接近 50 Hz；POSE 中头、双腕、身体、双手方向正确且无突跳。
7. PICO STOP 和 C++ `O` 都能终止控制；重新启动不得自动恢复旧目标。
8. 中断远端 WBC/bridge：本机 bridge 在 0.25 秒后发布 `kp=0, kd=8` 阻尼命令；
   中断 PICO body timestamp：manager 在 5 秒后发 STOP 并退出。

只看到 viewer、只到 `Init Done`、或机器人仍靠 band 吊着，都不算跑通。

### 2026-08-28 本机实测结论

- 官方 SONIC v1.1 encoder/decoder/planner 在远端 TensorRT 10.13.3.9 加载成功；维度分别
  为 `1751→64`、`994→29`、`[1,4,36]→[1,64,36]`。
- PICO 头显的 24 关节 Full Body 数据已实际连通；设备地址与序列号不入库；
  manager 进入 POSE 后长段稳定在 `48–50 Hz`。
- 远端实际解码 protocol v3：5 帧 chunk、24 SMPL joints、21 SMPL poses、VR 3-point 以及
  29 身体 + 7 左手 + 7 右手状态；SMPL encoder 模式为 ID 2。
- 第二轮 POSE 连续运行 3,948 个目标帧（约 80 秒），操作者完成全身动作测试，
  然后正常 `POSE -> PLANNER`；远端记录的 streaming delay 均值 `11.313 ms`、标准差
  `8.866 ms`。
- PLANNER 松绑落地后 18 秒实测：基座高度 `0.7849–0.7871 m`，水平位移范围
  `0.0133/0.0159 m`，线速度 p95 `0.0337 m/s`，roll p95 `0.822°`，pitch p95 `8.019°`，
  `z < 0.5 m` 样本数为 0。机器人未靠 elastic band 悬挂。
- 受控右臂测试采到右腕 roll `84.8°`、右肘 `59.2°`、右肩 `16–21°`；29 身体、
  左手 7 与右手 7 反馈数组维度和有限值检查通过。
- 停止 PICO 数据后，manager 在 5 秒后检出 stale timestamp 并发 STOP；远端 planner
  1 秒超时回 IDLE，WBC 正常打印 `Program exiting normally`，本地运行脚本清理 viewer/bridge。
- 完整时间线、实测命令、原始日志位置与已知限制见
  `docs/VALIDATION_20260828_PICO_SONIC_MUJOCO.md`。
- 精度、延迟、动作分段、外部真值与安全时序的可复现实验见
  `docs/PICO_SONIC_MUJOCO_ACCURACY_LATENCY_EXPERIMENT_ZH.md`。完整链运行后，在第二个
  本机终端执行 `./scripts/run_pico_sonic_experiment.sh --protocol baseline` 开始采集。
- 完整代码边界、分批提交命令、固定模型 revision 和第三方从零复现步骤见
  `docs/PICO_SONIC_CHANGESET_AND_REPRODUCIBILITY_ZH.md`。

## 10. 故障定位

| 现象 | 检查 |
|---|---|
| `waiting for body data` | PICO 应用状态 WORKING、PC IP、Full body、PC Service :60061 |
| 正在运行时突然 `Timestamps stale` | 确认没有第二个 `xrobotoolkit_sdk` 客户端调用 `xrt.close()`；必要时重启两端 |
| 远端一直等 LowState | 本机 bridge、TCP 5560/5561、两端均使用 lo DDS，确认没有重复 simulator |
| 缺 `NvOnnxParser.h` | 运行 `setup_remote_sonic_env.sh`，不要只装 Python wheel |
| 找不到 `nvparsers` | TensorRT 10 已移除旧库；使用仓库保留的 TRT10 CMake 兼容改动 |
| planner 动作错误 | 必须用 TensorRT 10.13 重新生成 cache，不复用 10.15 cache |
| robot 摔倒/振荡 | 不继续 PICO；先确认只有一个 `run_sim_loop.py`，再检查 TRT 版本、启动顺序和 bridge 频率 |
| 手臂突跳 | 返回 PLANNER，重新对齐，再进入 POSE/VR_3PT |

官方原始说明见：

- `docs/source/getting_started/vr_teleop_setup.md`
- `docs/source/tutorials/vr_wholebody_teleop.md`
- `docs/source/tutorials/keyboard.md`
- `docs/source/references/deployment_code.md`

本文件只增加当前双主机、独立 Conda、用户级 PC Service、TensorRT 10.13 和原始 CDR
跨机桥的落地细节。
