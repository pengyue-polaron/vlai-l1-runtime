# 临时双摄像头与电机角度录制

独立 CSV＋MJPEG AVI 工具，不写正式 LeRobot 数据集，不需要 AgentView。
它读取已运行的左右遥操状态，不启动、停止、复位或使能机械臂。
状态 socket 和两台摄像头需空闲：不要同时运行其他状态接收器或占用它们的相机服务。

## 使用

```bash
cd /home/nvidia/vlai-l1-runtime
sudo install -d -m 0755 -o "$(id -un)" -g "$(id -gn)" /run/vlai-l1
.venv/bin/python -m vlai_l1_runtime.temporary_recording \
  --output data/temporary/experiment_001 --duration 30
```

摄像头和电机状态就绪后计时。Ctrl+C 可提前结束；录制结束不会停止遥操。
输出目录须不存在。失败会记录错误；未完成录制不能当作完整样本。

只检查摄像头：

```bash
.venv/bin/python -m vlai_l1_runtime.temporary_recording \
  --check-cameras --duration 10 --output data/temporary/camera_check_001
```

该模式不生成电机数据。

## 输出与配置

- `left.avi/right.avi`：左右腕 RGB 视频。
- `left_frames.csv/right_frames.csv`：帧号及主机采集时间。
- `left_motors.csv/right_motors.csv`：各侧 Leader/Follower 七关节及夹爪角度。
- `metadata.json`：模式、配置、帧数、帧率、完成状态或错误。

视频与电机独立采样，共用主机单调时钟；按时间戳对齐，不能按行号直接配对。
视频时间是软件读取完成时间，工具不声称硬件同步。真实时序以 CSV 为准。
`configs/recording/two_wrist.json` 绑定摄像头身份并引用 System 配置中的
尺寸、帧率、socket 和新鲜度设置；录制器不打开 CAN。
