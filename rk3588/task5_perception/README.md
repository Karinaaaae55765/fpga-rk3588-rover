# 任务五：类别、颜色和视觉跟踪（U 盘部署版）

发布目录、依赖和当前验证状态见[仓库说明](../../README.md)。下方原部署路径为开发记录。


这是新增的独立程序，不覆盖原 UI、不发送电机命令、不连接云端、不保存摄像头画面。
提供 COCO 目标检测、物体主体/人物上衣/下装颜色分析，以及按类别和颜色筛选后多帧锁定。
没有接入真实大模型、人物身份识别、目标距离、实车跟随或导航。

## 从 U 盘使用

在 RK3588 的串口终端登录，运行 `lsblk -f` 或 `mount` 查找 U 盘的真实挂载点。
**不要猜测或格式化设备**。若没有自动挂载，需按实际设备名和 exFAT 支持情况处理。
电脑上的 E: 不是 Linux 路径；下面 `/实际U盘挂载点` 必须替换。

可先直接从 U 盘检查，不用安装：

```sh
cd /实际U盘挂载点/task5_perception
sh run.sh --self-test
sh run.sh --check --no-display
```

`--self-test` 只检查合成数据；`--check` 只检查依赖和 NPU 初始化，不代表实际摄像头或颜色准确率已通过。

需要安装到板端时，仅在新目录不存在时复制这 10 个文件：

```sh
python3 -B - <<'PY'
from pathlib import Path
import hashlib
src = Path.cwd()
dst = Path('/userdata/task5_perception')
names = ('main.py','config.json','observations.py','color_classifier.py',
         'target_selector.py','target_tracker.py','overlay.py',
         'test_perception.py','run.sh','README.md')
if dst.exists():
    raise SystemExit('STOP: target exists; nothing overwritten')
data = {name: (src/name).read_bytes() for name in names}
dst.mkdir()
for name, content in data.items():
    with (dst/name).open('xb') as f:
        f.write(content)
    if hashlib.sha256((dst/name).read_bytes()).digest() != hashlib.sha256(content).digest():
        raise SystemExit('ERROR: copy verification failed: ' + name)
print('COPY OK: 10 files; services unchanged')
PY
```

## 依赖

本包故意复用此前板端部署，不携带模型或新运行库：

- Python 3.10+、NumPy、OpenCV、RKNNLite 1.5.2 与匹配的板端 Runtime。
- `/userdata/udp_vision_20261019/rk3588/detector_yolov5_rknn.py`、`yolov5_postprocess.py`。
- 上述目录下 `udp_vision/capture.py`、`udp_vision/display.py`。
- `/root/receiver.py`：现有 FUDP 1280×720 RGB565 接收器。
- `/userdata/rk3588-app/models/yolov5s_relu_rk3588.rknn`。
- 显示复用 GStreamer、Wayland 和 `/root/gst-rawparse-1.22.2`。

缺少依赖时程序报出具体路径并退出，不自动安装。路径不同时使用
`--vision-root /实际rk3588目录 --receiver /实际receiver.py --model /实际模型.rknn`。
本轮未能连接板子，因此上述现存路径需要用 `--check` 在板端核实。

## 切换画面并测试

新程序需要独占 UDP 6102。旧 UI/识别服务正在运行时会报端口占用，不自动杀进程。
若板端目前确实使用 `/userdata/rover_ui_live/autostart.py` 管理已部署服务，先查看状态：

```sh
python3 -B /userdata/rover_ui_live/autostart.py status
```

记下原状态。只有该服务原来运行、且准备切换时才执行 `stop`。
暂停会使原画面/UI暂时不可用；若服务由别的脚本管理，不要照抄本命令。

```sh
python3 -B /userdata/rover_ui_live/autostart.py stop
cd /userdata/task5_perception
# 未安装则改为 U 盘上的目录。
sh run.sh --seconds 60
# 仅锁定连续确认的红色瓶子。
sh run.sh --label bottle --color red --seconds 60
# 蓝色上衣的人；人物颜色默认分析上衣。
sh run.sh --label person --color blue --region upper --seconds 60
```

Ctrl+C 可退出。测试结束后，**只恢复原来处于运行状态的旧服务**：

```sh
python3 -B /userdata/rover_ui_live/autostart.py start
```

不设置开机启动，不修改旧服务配置。只将文件复制到 U 盘不会自动升级板子。
若无显示环境，可先用 `sh run.sh --no-display --seconds 30` 检查真实收帧和推理。
FPGA 到 RK3588 的摄像头专用网线仍需连接，U 盘只代替电脑到板子的代码传输。

## 画面与数据

- 黄框：检测对象；绿框：满足指定类别、颜色并连续确认的目标。
- `body`：物体主体区域；`upper/lower`：人物上衣/下装近似区域。
- red/orange/yellow/green/cyan/blue/purple/black/gray/white 对应红/橙/黄/绿/青/蓝/紫/黑/灰/白。
- `mixed`：颜色不集中；`unknown`：有效区域太小。均不满足颜色筛选条件。
- `observe_only`：未指定目标，仅显示；`confirming`：正在多帧确认。
- `ambiguous`：多个符合目标，取消锁定；需缩小条件或调整场景，当前版没有点击选择 UI。
- `locked`：视觉关联成立，不代表人物身份验证或允许电机动作。
- `not_found/lost_timeout/stale`：没有匹配目标、断流或过期。恢复后重新确认，使用新编号。

颜色百分比是近似区域内的像素占比，不是校准概率。
区域编号 `index` 仅在单帧内有效；`track_id` 才是连续关联编号，重启程序后重新计数。
输出 JSON 含帧号、流代次、目标框、颜色、横向偏差和跟踪状态，`target_distance_m=null`。
不输出可直接驱动车轮的动作。

## 必须了解的边界与实物验收

HSV 阈值是待标定初值。没有实例分割或人体姿态估计；检测框偏大、弯腰、遮挡、反光、
透明瓶、彩色灯光、肤色和背景均可能污染结果。当前没有棕色、粉色细分；深橙/粉色可能
落入现有色域，不能宣称覆盖全部自然颜色。人物不是按整个框平均颜色判断。

跟踪采用位置重叠、类别与指定颜色条件，不是 ReID；即使同位置出现替换目标也可能误关联。
显式丢失/跳变后重新确认；多个匹配对象不会任意选中。人物跟随和物品导航尚未实现。

断流后现有显示线程会到期清屏；NPU 阻塞时主线程 JSON 不能持续更新，未来下游必须自行
检查时间戳和通信超时，不能持有最后一次 locked。时间从本地完整帧重组起算，不是曝光时间。

板端验收至少检查：红/蓝/绿实体物体、白/黑/灰、同类不同色、两个同色目标、人物上下装、
遮挡重现、图像断流，以及正常室内光与阴影变化。记录识别错误、实际处理速度和过期帧。
电脑合成测试不能替代上述实物验证。本版尚未在 RK3588 上验收。

## 2026-10-28 电脑端验证记录

已有 WSL 环境（OpenCV 4.5.5、NumPy 1.23.4）20 项测试通过，无跳过。
包含红色色相回绕、多色拒绝、人物上下分区、候选歧义、丢失重编号、过期/重复帧、
图像叠加、Python 3.10 语法以及注入 RGB565 帧和假 NPU 的主入口集成检查。
另用已有 ONNX 模型对 bus.jpg 做真实 CPU 推理，检测到 3 人和 1 辆公交车，
成功进入新颜色分析和画框流程；此项不代表样例颜色判断准确或板端 NPU 已通过。
单独拷贝后的板端 `--self-test` 会跳过依赖电脑相邻源码树的主入口注入测试，其余测试照常执行。
