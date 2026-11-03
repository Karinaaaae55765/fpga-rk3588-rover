# RK3588 离线状态语音播报

发布目录、依赖和当前验证状态见[仓库说明](../../README.md)。下方原部署路径为开发记录。


任务四路径点、终点，任务六搜索点、目标到达，以及程序明确检测到的故障均可播报。
播放预生成的中文 PCM WAV，不需要网络、大模型或板端语音合成库。板端使用已有 `aplay`。
语音不会发送运动指令，不代替停车逻辑。没有声卡、播放器失败等情况只报告文字错误。

## 包含什么

`player.py`：独立播放线程、8条有界队列、10秒过期、8秒播放超时；故障取消队列并中断
正在播放的普通提示。相同故障连续采样不重播，新故障事件同文案至少间隔5秒。
`events.py`：将已确认的任务状态转成播报，任务失败不会播报成功。
`telemetry.py`：解析现有0x28遥测帧，不自行打开串口。
`config.json`：21条中文文案、设备及限流参数。文案修改后需重新生成对应WAV。
`demo.py`：无串口、无电机的独立试听。`test_status_voice.py`：模拟与虚拟串口测试。

## 路径点语义

匹配现有任务四/任务六候选固件的0x28帧：状态8为NAV，9为REJOIN，6为COMPLETE；
waypoint高四位是当前目标索引0～6。索引0～5对应P1～P6，6是最后终点。
由0推进至1表示FPGA认可P1已到达，依此类推；不是“正在前往P1”时播报。
原任务六候选基线的中间点允许通过，较新任务四有停留逻辑；播报遵循各固件认可的
路径点推进事件，不额外宣称中间点一定停稳。终点播报还检查成功状态和轮速。
这是相对里程计/现有控制器的到达确认，不是独立绝对定位证明。

开始监听时已完成、半路接入、编号回退、跨度大于一个点、反馈间隔超过0.5秒、
复位或故障均不会补造到达事件。故障可取消尚未播放的到达提示，可能少播一次，不能误报。
同一次通过只尝试播报一次，重复串口反馈不刷播；停止后重新开始路线可再次播报。

任务六只有SEARCH→SCAN（已到搜索点并停稳）播报“已到达搜索点，开始观察”；
APPROACH→COMPLETE且原因是target_reached_and_stopped才播报到达和完成。
结束人物跟随是“任务已停止”，不会误报成“任务已完成”。

## 部署布局

保持 `status_voice`、`task6_navigation`、`task5_perception`、`ui_live` 四个目录并列。
离线包使用新的 `/userdata/rover_voice_status_20261030_225017` 目录，勿覆盖以前部署。
旧任务六校准数据不会被自动迁移或变更；使用已实测校准时通过原有--config/--calibration参数指定。
本包不烧录FPGA，不安装开机服务，不复制配对凭据，也不修改音量或系统声卡路由。
原 `ui_live/server.py --serve-package` 仍是旧界面文件传输协议，**不携带新的兄弟语音目录**；
安装本功能需使用本次完整离线包，不能只更新server.py。

## 先试听（不会让车运动）

```sh
cd /userdata/rover_voice_status_20261030_225017
python3 -B status_voice/test_status_voice.py
python3 -B status_voice/demo.py --play --prompt test
python3 -B status_voice/demo.py --play --prompt waypoint_p1
```

如果默认声卡无声，先执行 `aplay -l` / `aplay -L` 查看实际设备；然后用
`--device '实际设备名'` 单独试听。不要猜声卡编号或自动把音量调到最大。
`--all`逐段试听全部21条；不加--play只模拟事件，不调用声卡。
aplay返回成功仅证明播放进程完成，必须由人耳确认RK3588实际外放。

## 任务四及原界面播报

先按原流程停止占用UDP6102和串口的旧UI服务（通常为下列已有脚本），再前台启动新副本：

```sh
python3 /userdata/rover_ui_live/autostart.py stop
sh /userdata/rover_voice_status_20261030_225017/ui_live/run.sh --voice play
```

界面程序复用唯一串口读取器，额外解析0x28；摄像头数据异常由界面服务监测。
原传感器显示仍使用0x10，原HTTP配对和访问控制保持。新副本仍需输入其输出的配对码。
原服务路径不存在时先核对部署，不能再启动第二个串口读取进程。
不新增自动运动功能：任务四仍由手柄X启动，语音只监听反馈。
这次没有改开机启动项，重启会按原部署启动；不要把一次前台运行视为永久安装。

## 任务六播报

任务六和原UI仍不能同时占用串口/视频。先退出上面的UI，再运行任务六。
先检查并试听标定提示（不打开串口/相机/电机，--voice play会输出音频）：

```sh
cd /userdata/rover_voice_status_20261030_225017/task6_navigation
sh run.sh --check --voice play
```

新固件加载、实际标定和运动检查全部完成后，才使用：

```sh
sh run.sh --execute --mode search --label bottle --color red --voice play
sh run.sh --execute --mode follow --label person --color blue --voice play
```

`--audio-device`可指定实际声卡；`--voice dry`只模拟播报；默认off兼容原行为。
任务停止先发送STOP，再排队播报。异常路径也先执行车辆资源清理，再报原因。
已知串口异常只说“控制板通信异常”，不猜测电机损坏；无法定位的异常播报“任务运行异常”。
断电、系统完全死机等使RK3588自身无法运行的故障，不能靠该程序继续播报。

## 本地验证与限制

测试覆盖路径点编号/重复/回绕/复位/断线、错误完成、故障优先与取消、慢播放器不阻塞、
播放超时、音频缺失、WAV完整性、虚拟串口混合0x10/0x28，以及任务六真实执行器的模拟到达。
Windows首轮界面HTTP测试有一项连接中断；Linux中对应HTTP测试及界面回归通过。
音频在电脑本地合成，尚未通过本次板端扬声器试听或实车播报验收。

生成音频（拒绝覆盖已有WAV）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File D:\FPGA\rk3588\status_voice\generate_prompts.ps1
```
