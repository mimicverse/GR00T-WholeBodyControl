# PICO + SONIC + MuJoCo 变更清单与入库复现指南

本文说明本地实现相对 NVIDIA 官方 `GR00T-WholeBodyControl` 的代码边界、推荐提交方式，
以及第三方如何从空环境恢复同一条双机遥操链。官方基线固定为：

```text
repository: https://github.com/NVlabs/GR00T-WholeBodyControl.git
commit:     a0732b642c0333077e127a2f56ab0014c196bca4
branch:     main
```

## 1. 什么应当入库

### A. 环境和 TensorRT 10.13 兼容

- `environment.pico-wbc.yml`：本机 CPU PyTorch、MuJoCo、DDS、XRoboToolkit binding；
- `environment.sonic-trt1013.yml`：远端独立 CUDA/TensorRT 构建环境；
- `scripts/setup_local_pico_env.sh`、`scripts/setup_remote_sonic_env.sh`；
- `gear_sonic_deploy/cmake/FindTensorRT.cmake`：TensorRT 10 已移除 `nvparsers`；
- `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/CMakeLists.txt`：非 ROS 构建显式包含 msgpack；
- 固定模型 downloader 与 `config/sonic_v1_1_models.sha256`。

### B. 双机实时链路与安全

- `gear_sonic/scripts/dds_zmq_bridge.py`：本机/远端原始 DDS CDR TCP 桥；
- `gear_sonic/scripts/pico_manager_thread_server.py` 的必要差异：用户级 PC Service 路径、
  复用现有带 stale 检测的 reader、PICO 失联 fail-closed STOP、导出设备时间戳；
- `scripts/run_local_pico_mujoco.sh`、`run_remote_sonic_wbc.sh`、
  `start_pico_sonic_mujoco.sh`、`start_pico_pc_service.sh` 和键盘诊断入口；
- `config/pico_wbc_split.env.example`，不包含密码或本机私有配置。

### C. 实验与验收

- 被动 SQLite recorder、离线 analyzer、clock probe、preflight gate；
- `run_pico_sonic_experiment.sh` 和 marker 工具；
- 操作手册、实验方案与实测记录。

上述改动不改变官方 29 个身体关节顺序、SONIC observation/action 定义、43 actuator
MuJoCo XML、PICO 坐标变换或 WBC 控制增益。

## 2. 什么绝不能入库

- `logs/`：运行日志、SQLite trace、分析 CSV/JSON；
- `config/pico_wbc_split.env`：机器 IP、用户名和路径的本机副本；
- `*.onnx`、`*.engine`、`*.trt`：模型和 GPU 相关缓存；
- Conda 环境、`~/.cache`、用户级 XRoboToolkit PC Service；
- SSH 密码、PICO 序列号和其他凭据。

`.gitignore` 已覆盖以上运行产物。提交前必须使用 `git status --short` 再确认。

## 3. 推荐入库方式

不要执行 `git add .`。从官方基线创建功能分支，并按审查边界拆成四个提交：

```bash
git switch -c feature/pico-sonic-mujoco-split

# 1/4：隔离环境、固定模型和 TRT 10.13 构建兼容
git add .gitignore \
  environment.pico-wbc.yml environment.sonic-trt1013.yml \
  config/pico_wbc_split.env.example config/sonic_v1_1_models.sha256 \
  gear_sonic/scripts/download_pinned_sonic_v1_1.py \
  scripts/setup_local_pico_env.sh scripts/setup_remote_sonic_env.sh \
  gear_sonic_deploy/cmake/FindTensorRT.cmake \
  gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/CMakeLists.txt
git commit -m "build: isolate PICO and TensorRT 10.13 environments"

# 2/4：运行链路和失联安全
git add gear_sonic/scripts/pico_manager_thread_server.py \
  gear_sonic/scripts/dds_zmq_bridge.py \
  scripts/start_pico_pc_service.sh scripts/run_local_pico_mujoco.sh \
  scripts/run_remote_sonic_wbc.sh scripts/start_pico_sonic_mujoco.sh \
  scripts/connect_remote_sonic_keyboard.sh
git commit -m "feat: run PICO SONIC MuJoCo across isolated hosts"

# 3/4：精度、延迟和安全实验工具
git add gear_sonic/scripts/record_pico_sonic_trace.py \
  gear_sonic/scripts/analyze_pico_sonic_trace.py \
  gear_sonic/scripts/check_pico_sonic_preflight.py \
  gear_sonic/scripts/clock_probe.py \
  scripts/run_pico_sonic_experiment.sh scripts/send_trace_marker.sh
git commit -m "test: add passive whole-body latency and accuracy experiments"

# 4/4：文档与已完成验证
git add README.md docs/PICO_SONIC_WBC_MUJOCO_ZH.md \
  docs/PICO_SONIC_MUJOCO_ACCURACY_LATENCY_EXPERIMENT_ZH.md \
  docs/PICO_SONIC_CHANGESET_AND_REPRODUCIBILITY_ZH.md \
  docs/VALIDATION_20260828_PICO_SONIC_MUJOCO.md
git commit -m "docs: document PICO SONIC MuJoCo reproduction and validation"

git status --short
git log --oneline -4
```

若提交到公开上游，先从实测记录中删除本地 IP、用户名和设备序列号；内部仓库可保留
脱敏后的验证数值。只有仓库维护者确认远端地址和权限后才执行 `git push`。

当前公用 4090 的既存 checkout 含大量 Git/LFS 网格文件脏状态，不能在该机执行
`git add .`、reset 或清理命令。正式提交只在本机干净功能分支完成；远端部署应在新的
独立 clone/worktree 中 checkout 最终提交，或仅同步上面明确列出的文件。

## 4. 第三方从零复现

### 4.1 两台机器取得同一个提交

本机和 GPU 主机都必须 checkout 上述四个提交后的同一 Git SHA。不能只复制启动脚本，
因为桥、manager 安全补丁和 C++ 构建补丁同样是链路的一部分。

```bash
git clone https://github.com/<owner>/GR00T-WholeBodyControl.git
cd GR00T-WholeBodyControl
git checkout <pico-sonic-commit-sha>
cp config/pico_wbc_split.env.example config/pico_wbc_split.env
```

在两台机器的私有配置中填写实际 `WBC_HOST`、`SIM_HOST_IP`、`REMOTE_USER` 和
`REMOTE_REPO`。端口默认使用 5556、5557、5560、5561。

本机可在人工网络/SSH 检查前加载这些值：

```bash
set -a; source config/pico_wbc_split.env; set +a
```

### 4.2 本机安装

要求 Ubuntu 22.04 x86_64、已有 Miniforge/Conda、PICO 和本机处于同一局域网：

```bash
./scripts/setup_local_pico_env.sh
```

该脚本只写入 `~/miniforge3/envs/groot-wbc-pico`、`~/.cache/gr00t-wbc-deps` 和
`~/.local/share/xrobotoolkit-pc-service/1.0.0`。不会修改系统 Python 或 `/opt`。

PICO 安装 `XRoboToolkit-PICO-1.1.1.apk`：

```text
URL:    https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/download/v1.1.1/XRoboToolkit-PICO-1.1.1.apk
SHA256: 6b2bb282405673d24abcb1980e3478b8f1052e90f7207b1f24cc56a59f8d8261
```

### 4.3 GPU 主机安装

要求 NVIDIA 驱动可用。仓库放在 `REMOTE_REPO` 后执行：

```bash
./scripts/setup_remote_sonic_env.sh
```

该脚本只使用 `~/miniforge3/envs/groot-wbc-sonic-trt1013` 与用户缓存，下载固定 revision
模型并构建 `g1_deploy_onnx_ref`。模型来源固定为：

```text
nvidia/GEAR-SONIC@6733128a3d8a523b1418b06bca3cdf61c8b0987f
```

验证模型而不重新下载：

```bash
~/miniforge3/envs/groot-wbc-sonic-trt1013/bin/python \
  gear_sonic/scripts/download_pinned_sonic_v1_1.py --verify-only
sha256sum -c config/sonic_v1_1_models.sha256
./scripts/run_remote_sonic_wbc.sh --check
```

### 4.4 启动与验收

PICO 应用设为 `WORKING`、Head/Controller Send、Motion Tracker Full body，然后在本机：

```bash
./scripts/start_pico_sonic_mujoco.sh
```

一键脚本会先检查本机 Conda 环境和 43-DoF XML，并在同一次 SSH 内执行远端
`run_remote_sonic_wbc.sh --check`；任一门禁失败都会停止并清理本地子进程。

看到远端 `Init Done` 后依次：A+B+X+Y、MuJoCo 窗口按 `9`、PLANNER 独立站稳、
最后 A+X 进入 POSE。正式实验入口：

```bash
./scripts/run_pico_sonic_experiment.sh --protocol baseline
./scripts/run_pico_sonic_experiment.sh --protocol static
```

实验脚本会在开始前验证模式和基座高度；条件不满足时拒绝采集。详细安全步骤与验收阈值
见 `PICO_SONIC_WBC_MUJOCO_ZH.md` 和
`PICO_SONIC_MUJOCO_ACCURACY_LATENCY_EXPERIMENT_ZH.md`。

## 5. 入库前最小 CI/冒烟命令

```bash
git diff --check
for f in scripts/*.sh; do bash -n "$f"; done
~/miniforge3/bin/conda run -n groot-wbc-pico python -m py_compile \
  gear_sonic/scripts/{pico_manager_thread_server,dds_zmq_bridge,record_pico_sonic_trace,analyze_pico_sonic_trace,check_pico_sonic_preflight,clock_probe,download_pinned_sonic_v1_1}.py
~/miniforge3/bin/conda run -n groot-wbc-pico \
  python gear_sonic/scripts/analyze_pico_sonic_trace.py --self-test
~/miniforge3/bin/conda run -n groot-wbc-pico \
  python gear_sonic/scripts/download_pinned_sonic_v1_1.py --verify-only
```

硬件验收结果不能用单元测试代替；最终仍要保存 E0–E5 的 `summary.md` 和安全 STOP 时序。
