# PICO + SONIC v1.1 + MuJoCo 验收记录（2026-08-28）

## 入库复现检查

- `scripts/setup_local_pico_env.sh` 已在现有机器完整重跑成功：Conda 环境更新、
  XRoboToolkit SDK 1.0.2 native binding 重编译、PC Service 1.0.0 下载与 SHA-256
  校验、固定 SONIC v1.1 模型校验均通过；
- 本机实际版本：Python 3.10.21、MuJoCo 3.2.7、Pinocchio 2.6.21、
  PyTorch 2.4.1+cpu（CUDA False）；
- 4090 只读复核：Python 3.10.21、CUDA nvcc 12.9.86、TensorRT 10.13.3.9，
  encoder/decoder/config/planner 的 SHA-256 与本机完全一致；检查时没有残留
  `g1_deploy_onnx_ref` 或 bridge 进程；
- analyzer 的 80 ms 合成时延自测返回 80.00 ms；recorder ready-file、SQLite
  integrity 和无链路 preflight fail-closed 均通过；
- 代码与入库边界详见 `docs/PICO_SONIC_CHANGESET_AND_REPRODUCIBILITY_ZH.md`。

## 验收对象

- 仓库：本机功能分支 checkout
- 官方基线：`a0732b642c0333077e127a2f56ab0014c196bca4` (SONIC v1.1)
- 本机：无 NVIDIA GPU，运行 PICO PC Service、manager、MuJoCo viewer 和 sim bridge
- GPU 机：RTX 4090，只运行 SONIC/TensorRT WBC 与 WBC bridge
- PICO：设备地址和序列号已从公开记录删除
- 模型：G1 29 身体 DoF + Dex3 7×2 = 43 actuators
- 场景：`gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml`
- 本地 Conda：`groot-wbc-pico`
- 远端 Conda：`groot-wbc-sonic-trt1013`（TensorRT 10.13.3.9 / CUDA 12.9）

## 可复现启动

推荐在本地只执行一条命令：

```bash
cd "$HOME/Work/GR00T-WholeBodyControl"
./scripts/start_pico_sonic_mujoco.sh
```

该脚本内部等价于以下两半（只用于分开排障）：

```bash
# 本地
set -a; source config/pico_wbc_split.env; set +a
./scripts/run_local_pico_mujoco.sh

# GPU 机
ssh "${REMOTE_USER}@${WBC_HOST}"
cd "$HOME/Work/GR00T-WholeBodyControl"
set -a; source config/pico_wbc_split.env; set +a
./scripts/run_remote_sonic_wbc.sh
```

实际运行日志：

- 首轮：`logs/pico_wbc_split/20260828-181124/`
- 重启验收轮：`logs/pico_wbc_split/20260828-182209/`

启动后必须依次看到：

```text
device found
[Manager] ZMQ socket bound to port 5556
Init Done
[Manager] StreamMode switch: OFF -> PLANNER
Planner initialized successfully!
```

然后才在 MuJoCo viewer 按 `9` 解除 elastic band，站稳后用 PICO `A+X` 进入 POSE。

## 时间线与结果

1. 官方 43-actuator scene 在本地 MuJoCo 3.2.7 加载，viewer 可见。
2. 远端加载 SONIC v1.1 decoder `994 -> 29`、encoder `1751 -> 64` 和 planner
   `[1,4,36] -> [1,64,36]`；三个 TensorRT engine 在 RTX 4090 上创建/加载成功。
3. 键盘基线测试先验证官方 SONIC -> MuJoCo：进入 `SLOW_WALK`、执行前进、
   回 IDLE；里程计位移 `0.0657 m`，速度峰值 `0.1935 m/s`，最小高度 `0.7777 m`，
   未跌倒。
4. PICO Full Body 连通后，实际发送 24 body joints、21 SMPL poses、5-frame protocol-v3
   chunks 与 VR 3-point；远端选择 SMPL encoder mode 2。
5. 首轮受控动作采样：`g1_debug` 399 帧 / 8 s，`49.88 Hz`，索引连续无跳帧；
   29 + 7 + 7 反馈数组尺寸正确且全部有限。
6. 右臂动作窗口：右腕 roll `84.8 deg`、右肘 `59.2 deg`、右肩各轴约
   `16–21 deg`；躯干和下肢同时做 whole-body 平衡协同。
7. PLANNER 独立落地 18 秒：基座高度 `0.7849–0.7871 m`，z 低于 `0.5 m` 样本数 0；
   roll p95 `0.822 deg`，pitch p95 `8.019 deg`，速度 p95 `0.0337 m/s`，水平位移范围
   `0.0133/0.0159 m`。
8. 重启验收轮 POSE 连续输出 3,948 个目标帧，约 80 s，长段频率 `48–50 Hz`；
   操作者完成一轮全身动作后正常回 PLANNER。远端 streaming delay 均值
   `11.313 ms`，标准差 `8.866 ms`。
9. 安全降级：PICO 数据停止后 5 s，manager 记录 `Timestamps stale`、发送 STOP 并退出；
   远端 planner 1 s 超时回 IDLE，WBC 正常退出，本地启动器清理所有子进程。

## 故障复盘

诊断 PICO 按键时曾并发启动第二个 `xrobotoolkit_sdk` 客户端。该进程调用
`xrt.close()` 后，PC Service 的共享 body stream 被取消，主 manager 5 s 后失联停机。
恢复步骤为：

1. 确认旧的 local launcher 和远端 WBC 均已退出。
2. 重启 `run_local_pico_mujoco.sh`。
3. 将 PICO XRoboToolkit 切回前台并确认 `WORKING`/Full Body。
4. 重启 `run_remote_sonic_wbc.sh`，等 `Init Done`。
5. 重做 A+B+X+Y 校准/启动，再落地、进 POSE。

结论：控制链必须保持单一 XRT SDK 客户端。运行中的诊断只读 `g1_debug`、DDS 或
manager 日志，不再另起 XRT 客户端。

## 当前边界

- G1 29-DoF 模型没有独立头部执行关节；PICO 头显数据作为 SMPL root/head 与
  VR 3-point 观测进入 whole-body policy，不应期待 MuJoCo 里出现额外的“脖子 DoF”。
- 当前官方 PICO helper 用 trigger 生成预设手指 IK，不是独立手指动捕。左右各 7 执行器
  在 MuJoCo/DDS/feedback 中存在，但手势表达能力受该 helper 限制。
- 所有控制进程已在最后的 PICO 失联 STOP 测试后退出；下次运行需从两条启动命令
  重新开始，不会自动沿用旧目标。
