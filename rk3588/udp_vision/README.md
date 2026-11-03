# UDP 摄像头实时识别与黄色检测框

发布目录、依赖和当前验证状态见[仓库说明](../../README.md)。下方原部署路径为开发记录。


配套 FPGA 位流：`camera_udp_sensor_20261019_091257.fs`。
输入为 FUDP v1、1280×720、RGB565 高字节在前；FPGA 图像发送协议无需修改。
程序仅接收图像、执行检测和显示，不打开控制串口、不发送电机命令。

## 画面效果

每个通过置信度和类别筛选的目标均使用黄色矩形框，标注英文类别与置信度。
默认显示现有 COCO 模型支持的所有类别，可筛选 person、bottle、cup 等。
这属于物体检测，不提供人脸身份识别、颜色识别、目标测距或车辆跟随。
未检出目标时显示当前原始画面；输入或推理结果过期时显示黑屏。

UDP 接收线程复用原接收器的 Assembler，只接受完整且已收到确认包的图像。
待推理队列仅容纳一帧，新帧替换尚未处理的旧帧。框只画在实际参与本次推理的图像上。
结果超过 `--max-frame-age` 就丢弃，显示线程也独立检查到期时间，因此 NPU 卡住时
仍能清除之前显示的结果。默认 1 秒从本机收齐图像起计算，不是摄像头曝光时间；
协议没有采集时间戳，网络缓存和拼帧时间不包含在该年龄中。
显示期限检查周期约 20 ms；向管道发送一帧需要时间，GStreamer/Weston 也可能缓存，
不能据此承诺严格的一秒物理显示时限。显示管道阻塞一秒时报错并终止显示子进程。

## 依赖与目录

RK3588 上需要与现有模型匹配的 RKNNLite/Runtime 环境，以及 Python 3.10+、NumPy、OpenCV。
现有模型代码针对 RKNNLite 1.5.2。请先核对板端实际版本，不覆盖系统运行库。
显示需要当前可工作的 Wayland 登录环境和 GStreamer 的 fdsrc、rawvideoparse、
videoconvert、queue、waylandsink。程序不调用 OpenCV GUI，不需要 cv2.imshow。
Windows 上的 ONNX 检测结果不能代替板端 RKNN NPU 验收。

部署时保留下列相对结构（此任务没有自动部署）：

```text
<部署根目录>/
  camera_udp_sensor_uart/linux/receiver.py
  rk3588/
    detector_yolov5_rknn.py
    yolov5_postprocess.py
    models/yolov5s_relu_rk3588.rknn
    udp_vision/
      main.py
      capture.py
      overlay.py
      display.py
      run.sh
      README.md
```

复用文件只读引用，未复制、修改旧工程。若接收器或模型放在其他目录，用
`--receiver /实际路径/receiver.py --model /实际路径/模型.rknn` 指定。
当前目录中的 `test_pipeline.py` 为电脑端验证入口，不是板端启动依赖。

## 启动

先确认已烧录配套 UDP 位流、专用千兆网络已配置且原接收器能收到完整图像。
启动识别前退出原 `preview.sh` / `receiver.py`，两者不能同时占用 UDP 6102。
本程序不自动停止其他进程、不添加 IP、不修改默认路由、不安装软件。

在原先能显示 UDP 画面的同一个板端终端环境，进入部署后的 `rk3588/udp_vision`：

```sh
# 检查依赖并加载 NPU 模型，不打开网络接收。
sh run.sh --check

# 默认识别所有支持的类别并用黄框显示。
sh run.sh

# 只识别人和水瓶；提高阈值可减少低置信度框。
sh run.sh --label person --label bottle --min-score 0.5

# 无窗口检查 30 秒接收和实际 NPU 推理。
sh run.sh --no-display --seconds 30
```

如果之前通过 PYTHONPATH 指定了 RKNNLite 的安装目录，请沿用已验证的环境。
`--source` 默认 192.168.50.2，源 UDP 端口 5000，接收端口 6102，与当前位流一致。
Ctrl+C 退出并释放接收端口、NPU 和显示子进程。日志输出到终端，默认不保存视频或图像。
脚本请求全屏，不更改 Weston 的镜像、屏幕旋转或布局设置。

## 判断是否工作

- `listening`：网络接收已启动，不代表已有图像。
- `stats.frames_ok`：完整图像数；`incomplete_frames`、`missing_packets` 可观察丢包。
- `detections`：本帧已经实际推理；`boxes=[]` 表示该帧没有通过筛选的物体。
- `processing_ms`：检测函数耗时，包含预处理、推理、后处理；不包括网口和显示。
- `local_frame_age_ms`：从完整帧交付到检测结果输出的本机耗时。
- `stale_results`：过期图像或结果的数量；持续增长时检查推理性能，不能当作有效实时识别。
- `pending_replaced`：等待推理的帧被新帧替换数量，是主动限制延迟，不等同于 UDP 丢包。

有时限运行结束时，至少处理过一帧返回 0；没有有效识别处理帧返回 2；错误返回 1。
这里“处理过一帧”包括正常推理但没检出物体。`--check` 返回 0 仅表示依赖和 NPU
初始化通过，并没有验证实物图像、检测准确率、持续帧率或显示效果。

## 本地验证

```text
python -B test_pipeline.py
python -B test_pipeline.py --report
```

`--report` 只新增 `output/tests.log` 与 `output/verification.json`，已有报告时拒绝覆盖。
测试覆盖 RGB565 字节序、黄框像素、标签参数、720p 坐标还原、类别与置信度筛选、
完整帧重组、乱序、丢包、冲突重复包、帧号回绕、UDP 源端过滤、队列替换、
推理过期结果拒绝及显示独立清屏，并运行原有 YOLO 后处理测试。
如当前电脑解释器已有 OpenCV 和 ONNX Runtime，还会用现有 bus.jpg 做完整
720p RGB565/FUDP 往返、真实 CPU 检测和黄框渲染；否则明确跳过该项。
报告同时记录复用文件哈希和是否变化。板端 NPU、实际摄像头和 Wayland 显示仍需实测。

## 2026-10-19 本地验证记录

在已有 WSL Ubuntu 的 `rknn-1.5.2` Python 环境运行，21 项测试通过且无跳过。
真实 CPU 测试使用已有 ONNX 模型：bus.jpg 等比例放入 1280×720 画布，经过
RGB565/FUDP 编码与重组后，检测到 1 辆公交车、3 个人，并实际渲染黄色框和文字。
Linux 管道测试验证了初始黑帧、完整彩色帧、到期黑帧的写入顺序；管道接收程序
代替 GStreamer，因此该测试不代表实际 Wayland 全屏显示通过。

初次 Windows 受限环境测试中，回环 UDP 未收到完整帧，原因未定位；同项 Linux
测试通过。Windows 解释器也缺少 OpenCV/ONNX Runtime。正式报告使用上述已有
Linux 视觉环境生成；没有新增依赖或改动现有系统。

本次未连接 RK3588、未上传文件、未烧录 FPGA。实物验收仍需在板端确认模型加载、
网口持续完整帧、实际黄色检测框与类别、断流清屏以及推理和显示的长期稳定性。
