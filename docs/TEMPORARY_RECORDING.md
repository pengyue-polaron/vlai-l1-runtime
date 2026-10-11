# 临时双摄像头＋电机角度录制

入口：`python -m vlai_l1_runtime.temporary_recording`。
这是独立的 CSV＋MJPEG AVI 实验录制工具，不是正式 LeRobot 数据集。
不需要 AgentView，也不会修改正式采集配置或裁掉静止片段。

## 使用

在仓库目录中，先确保正常的左右遥操均已启动并运行。
`just sdk-start left` / `just sdk-start right` 会使能和对齐机械臂，仍需遵守
原有现场安全流程。录制器不启动、停止、复位或使能任何机械臂。
不要同时运行 `just collect`、`just sdk-observe` 或其他状态接收器。
摄像头服务也不能同时占用这两台摄像头；如已开启，先 `just cameras stop`。

为普通用户准备状态接收目录（与原有 `sdk-observe` 的目录准备一致）：

```bash
cd /home/nvidia/vlai-l1-runtime
sudo install -d -m 0755 -o "$(id -un)" -g "$(id -gn)" /run/vlai-l1

.venv/bin/python -m vlai_l1_runtime.temporary_recording \
  --output data/temporary/experiment_001 \
  --duration 30
```

两台摄像头和左右电机状态均就绪后才开始计时。录满 30 秒后关闭文件、释放
摄像头和状态 socket。也可以 Ctrl+C 提前结束并保留录制数据。
**录制结束（包括出错）不会停止正在运行的遥操，机械臂可能仍处于使能状态。**
必须换一个不存在的实验目录才能再次录制，不覆盖或删除任何旧实验。
电机状态超过系统规定的新鲜度、摄像头中断、保存后视频无法完整解码或帧率
低于临时配置阈值时，返回失败，并保留文件及 `metadata.json` 中的错误。
突然断电或强制杀进程可能留下 `incomplete` 记录，不能当作完整实验使用。

只验证两台摄像头录制、无需启动遥操：

```bash
.venv/bin/python -m vlai_l1_runtime.temporary_recording \
  --check-cameras --duration 10 --output data/temporary/camera_check_001
```

该模式明确标为 `cameras_only_check`，不生成或冒充电机数据。

## 文件及时间戳

```text
experiment_001/
  left.avi               左腕 RGB 视频（MJPEG，640×480，配置 30 FPS）
  right.avi              右腕 RGB 视频
  left_frames.csv        左视频帧号、主机取帧完成 monotonic_ns
  right_frames.csv       右视频帧号、主机取帧完成 monotonic_ns
  left_motors.csv        左 leader 8 个＋左 follower 8 个电机位置
  right_motors.csv       右 leader 8 个＋右 follower 8 个电机位置
  metadata.json          模式、配置快照、帧数、实测帧率、完成状态或错误
```

每个电机 CSV 含源序号、源 monotonic_ns、接收 monotonic_ns，以及
`leader_joint_1_deg` 到 `leader_joint_7_deg`、`leader_gripper_deg`，
`follower_joint_1_deg` 到 `follower_joint_7_deg`、`follower_gripper_deg`。
两份 CSV 覆盖四条臂的 32 个电机位置，全部为角度（度）。不包含电流、力矩、
速度、深度图。按 sidecar 实际发布频率记录（当前配置为每侧 100 Hz），
源序号来自 SDK 回调，正常情况下相邻发布记录的源序号也可能不连续。

视频和电机独立采样，共用本机单调时钟；**不能按 CSV 行号直接对应**。
离线应按 `read_monotonic_ns` 和 `source_monotonic_ns` 查找最近样本或插值，
并自行检验所需的时间误差。摄像头时间为软件读取完成时间，不是硬件曝光时间；
本工具不声称硬件同步，不保证正式采集流程的相机／电机配对误差。
AVI 回放按配置帧率播放；实验真实时序应以 CSV 时间戳为准。

## 临时身份配置

`configs/recording/two_wrist.json` 单独绑定已通过拔插识别的左右摄像头：
左 RealSense 序列号 `260322278153`、右 `260422274280`。
工具在打开视频前，用 RealSense 枚举核对序列号对应的 USB 物理路径，
并检查设备占用。USB 口变更导致配置不匹配时会拒绝录制，不猜测左右关系。
图像尺寸和帧率、状态 socket 路径、新鲜度限值来自引用的 System 配置。
本工具只接收已有 SDK 状态，不打开 CAN，因此保留原有全部关节保护。
