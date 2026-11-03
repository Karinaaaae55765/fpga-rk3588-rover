# ROVER 统一感知与任务六控制（2026-11-03）

发布目录、依赖和当前验证状态见[仓库说明](../../README.md)。下方原部署路径为开发记录。


本次接入类别/颜色识别、里程计估计位置和任务六控制接口。代码接入不等于完成实车标定或导航验收。
只有浏览器主动启动任务且所有检查通过，才发送 ARM/速度指令；启动服务、配对、识别目标不会自动驱动车轮。

## 数据与资源共用

- `pipeline.py` 独占 UDP 6102，只创建一个 RKNN 检测器。
- `task5_bridge.py` 复用现有任务五的颜色分类、观察结果与视觉锁定，同一帧供板载屏幕、网页和任务六使用。
- `motion_bridge.py` 是唯一串口所有者，115200、8N1；同一字节流解析 V3 传感器帧和任务六 0x36 遥测。
- `telemetry_v3.py` 的 `accept` 只解析数据，不再另开串口线程。语音观察器仍使用这份传感器数据。
- 独立任务五/六进程不要与此服务同时启动；否则串口或图像端口独占检查会失败。
- `pose_estimate.py` 复用任务六里程计，人工设置初始 X/Y/朝向后才更新位置。比例参数未经实测时不得当成测量值。

默认依赖路径：

- `/userdata/task6_bundle_20261028_235520/task5_perception`（可用 `--task5-root` 指定）
- `/userdata/task6_bundle_20261028_235520/task6_navigation`（可用 `--task6-root` 指定）
- `/userdata/udp_vision_20261019/rk3588/udp_vision` 及其父目录 RKNN 检测器
- `/root/receiver.py`、`/userdata/rk3588-app/models/yolov5s_relu_rk3588.rknn`
- `/userdata/rknn-python` 和 `/root/gst-rawparse-1.22.2`（原 run.sh 保留）
- `/dev/ttyUSB0`；UI 和任务六配置中的设备名必须一致。

没有复制、更改现有任务五/六算法、地图、标定文件或 FPGA 位流。

## 操作

1. 打开板子热点地址的 8080 端口，用原有配对码连接。
2. 视频同时显示类别与颜色；人物用上身/下身区域，物品用主体内部区域。mixed 为混色、unknown 为无法判断；像素占比不是置信度。
3. 位置初始化：车辆停稳，人工输入实体地图上的 X、Y（m）与朝向（°，+X 为 0，逆时针为正），点击初始化。
   不默认车辆位于 S；运动时不能重设原点。数据中断、计数/航向跳变或 FPGA 重启使位置失效，需要重新初始化。
4. 选择接近可见杯子/水瓶或跟随可见人物，选择颜色及距离，查看页面列出的未就绪原因。
5. 已完成真实标定且传感器、图像、FPGA 状态就绪后，才能主动启动任务六。
   可见目标模式不需要地图起点；全场搜索必须使用已核实的实体地图及起始姿态。
6. 停止按钮只停止本服务任务，随后检查新鲜遥测是否显示未 ARM、左右轮停稳。未收到确认会明确标为未确认。

目前任务六 `map.json` 是 6×4 m 仿真地图，不能与本页面实体打印地图混用，搜索会被拒绝。
本次没有填写或伪造任何标定数据；原 calibration.json 未完成时启动仍会被拒绝，并显示具体缺项。

## 控制与通信语义

接口均要求原配对认证、同源 Origin、固定字段的 JSON；不接受 shell 命令或任意文件路径。

- `POST /api/pose`：x、y、heading_deg；仅停稳时设定内存原点，不修改标定文件。
- `POST /api/motion/start`：mode、label、color、distance、client_id；检查通过后排队，HTTP 成功不代表电机已启动。
- `POST /api/motion/heartbeat`：mission_id、client_id；仅启动该任务的页面可续约。
- `POST /api/motion/stop`：空对象；已配对页面可停止本服务的活动任务。

控制线程按配置（默认 20 Hz）运行，与 NPU 线程分离。ARM 必须取得对应 session/序号应答后才发送速度；
未应答会提示检查 TX 接线及固件状态。租期 1.5 s，关闭/隐藏页面或心跳停止后终止任务，过期心跳不能恢复任务。
控制循环超过 200 ms、遥测过期、视觉结果过期/重启、人工接管或原执行器终止均停止，不自动重启运动。
FPGA 仍保留原 ±30 RPM、300 ms 超时、手动接管等限制。实际无线延迟、停车距离与电机动作仍需实测。
STOP 下发不等于停稳；页面只有在更新的 0x36 遥测显示 state=0、双轮绝对转速≤3 RPM 时报告停稳确认。
程序退出时先关闭运动桥并发送必要的 STOP，再关闭视觉；串口异常仍依赖 FPGA 原超时机制。

位置使用轮速几何与 IMU 增量估计，有累计漂移；遥测超过 200 ms 失效时不显示当前车辆箭头，历史轨迹保留为灰色。
无 SLAM、无自动绝对定位、无自动确认物理接线或烧录版本。目标登记不等于人物身份重识别。

## 固件预设路线（2026-11-02）

路线来源为 `camera_udp_task6/build/camera_udp_task6_candidate_20261102_120015/source_snapshot/`
中的 `route_6x4.json`、`route_report.json`，目标点与 `route_config.vh` 核对。
历史文件名含 6x4，但此版本使用实体打印地图 1900 × 1266.67 mm，固件高度取整为 1267 mm。
它与 6 × 4 m 仿真图不同，不能混用。

`app.js` 嵌入快照全部 214 个路径采样点，世界坐标为局部坐标加起点 (130, 1140) mm，
此快照初始航向为 0°；原点在左下角，X 向右、Y 向上。
路线长度由采样点累计计算约 4.21 m，是配置值而非行驶距离实测。
路径点由原图读取并取整到 10 mm，未经现场测绘；采样点与各到点门限对应位置有毫米级差异。
P1–P6 配置停留 4 s、G 停车。网页显示方向箭头和点位坐标，不模拟车辆运动。
当前烧录版本和固件路线到点状态尚未核验，路线显示不能证明实车任务完成。橙色轨迹由显式初始化后的编码器/IMU估计生成，不是绝对定位。

本次仅更新 `app.js`、`index.html`、`README.md`，分别备份为
`app.js.before-route-20261102`、`index.html.before-route-20261102`、`README.md.before-route-20261102`。
该历史路线变更仅修改静态文件。2026-11-03 的控制整合涉及 Python 后台，部署后必须重启服务。

## 启动、部署和恢复

现有 `/etc/init.d/S99udp-vision` 和 `/userdata/rover_ui_live/autostart.py` 保留，启动统一 UI：

```sh
python3 -B /userdata/rover_ui_live/autostart.py status
python3 -B /userdata/rover_ui_live/autostart.py start
# 修改 Python 代码部署成功后：
python3 -B /userdata/rover_ui_live/autostart.py restart
# 停止统一服务（活动 UI 任务会停止）：
python3 -B /userdata/rover_ui_live/autostart.py stop
```

开机服务等候热点地址并在后台运行。地址变化需使用新地址；摄像头专用 eth1 保持 192.168.50.1/24。
`run.sh --check` 只检查依赖、导入和配置，不打开摄像头/NPU/串口，不证明运动已就绪。

本次更新七个已有文件：pipeline.py、telemetry_v3.py、server.py、app.js、index.html、test_live.py、README.md。
每个原文件先分别备份为 `<文件名>.before-integration-20261103`，新增 task5_bridge.py、motion_bridge.py、pose_estimate.py。
电脑与板端的备份分别就地保存；原有备份不覆盖。部署只覆盖确认过的原文件并核验 SHA256，不重跑旧版 autostart 安装器。
服务停稳、全部文件核验通过后才切换；启动失败恢复七个原文件并恢复旧服务，新模块保留但旧代码不会调用它们。
断电时无法保证多文件覆盖原子性，必须保留备份。未连接板子时，电脑修改不代表板端已部署。

配对凭据 `/userdata/rover_ui_live/pairing.key` 权限 0600，不修改、不写入源码、不通过静态 HTTP 提供。
浏览器保存原配对，刷新会自动连接；刷新不继承正在运行任务的控制租期。清除浏览器数据需要重新配对。
日志为现有 service.log（轮转上限 2 MiB）及 .1，进程记录为 /var/run/rover-ui.pid 和 .lock。
目标登记继续保存板端 targets 下的裁剪图和元数据；这不是运动启动操作。

`server.py --serve-package` 的 FILES 白名单包含三个新增模块，不包含任何配对码、日志或备份。
后台服务仅用于已信任的私人热点，保留认证、同源请求检查，不做公网端口映射。

## 验证

```powershell
python -B D:\FPGA\rk3588\ui_live\test_live.py
node --check D:\FPGA\rk3588\ui_live\app.js
```

测试构造串口/视觉记录，检查资源共用、标定拒绝、ARM 应答、超时停止、位置有效期、鉴权及网页配对回归。
构造的标定仅存在于测试对象内，不写入 calibration.json；不能作为实车标定证据。
OpenCV 颜色对应测试与 Linux 专属检查应在 RK3588 上补跑；测试通过不等于完成实车闭环验收。

## 登记后的绿色框（2026-11-03）

登记成功后，使用已保存记录的 frame_id、epoch、类别和框位置，在短期帧历史中找到对应目标，
沿连续帧的重叠框关联，并在当前画面画绿色框和 REGISTERED 标记。网页 JPEG 和板载屏幕共用这份叠加结果。
网页确认和板载按键确认读取同一个登记记录，因此都适用。

这只是当前连续画面中的关联，不是跨遮挡、跨场景的身份重识别。目标消失、同类重叠框存在歧义、
可靠颜色改变、图像流重启或超时后取消登记绿色框，不自动重认同一个保存记录；需重新登记。
远处无重叠的同类物品不会被染绿。任务六原有视觉锁定仍有独立的绿色框及 LOCK ID 标记。
登记绿色框不更改任务六 tracking.confirmed、目标选择或电机状态，不会启动小车。

升级重启后不根据磁盘上的旧照片恢复身份；请在当前画面重新登记。目标照片和原登记记录不删除。
本次修改 pipeline.py、task5_bridge.py、test_live.py、README.md，原文件分别备份为
<文件名>.before-registration-green-20261103，部署需重启 UI。
