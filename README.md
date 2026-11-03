# FPGA + RK3588 智能小车

本仓库提供任务一至六当前集成版本的展开源码和同版本 ZIP。点击 [下载 ZIP](fpga-rk3588-rover-20261103.zip) 获取完整源码。根目录的 SHA256SUMS 可校验 ZIP 及各源码文件。

- FPGA 使用 GW5AT-LV60PG484AC1/I0，设备版本 B。最终集成快照来自 camera_udp_task6_candidate_20261102_120015，包含图像、底盘、路径和任务六 UART 接口。约束见 fpga/source/board.cst，协议见 fpga/PROTOCOL.md。
- 在 Gowin Tcl Shell 中执行 source fpga/build.tcl 的实际路径即可生成新构建。请使用支持目标器件的工具版本，脚本会在 fpga/build 下创建构建目录。发布副本调整了路径，板端烧录文件需另行构建和验证。
- FPGA 仿真使用 Icarus Verilog，执行 PowerShell 的 ./fpga/verify.ps1。可用 -Only tb_task6_control 运行单项。
- RK3588 使用 Python 3.10、NumPy、OpenCV、与板端匹配的 RKNNLite 运行时和兼容的 YOLOv5 RKNN 模型。板载显示依赖 Wayland 和 GStreamer rawvideoparse，语音播放使用 ALSA aplay。模型和工具链需要另行准备。
- UI 和任务五/六、语音目录应保持并列。各子目录的启动脚本也支持单独测试。旧启动服务配置保留作参考，使用前按实际安装路径配置。
- Task 5 有实车物体及颜色识别观察记录。Task 6 搜索、接近和跟随仍须按实车标定流程验证，不能把仿真通过当成实车验收。未经验证的标定保留原状态。
- FPGA 路线采用 1900 × 1266.67 mm 实体图，任务六的 6 × 4 m 地图用于仿真。前向 TFmini-S 提供单点测距。
- ZIP 不含模型权重、工具链、烧录位流、私有运行数据和凭据。

## 启动网页与语音

从仓库根目录执行，先把 MODEL 设置成兼容的模型绝对路径。使用终端输出的配对信息访问页面。串口参数需与实际接线匹配。

```sh
sh rk3588/ui_live/run.sh --bind 0.0.0.0 \
  --vision-root rk3588/udp_vision --task5-root rk3588/task5_perception \
  --task6-root rk3588/task6_navigation \
  --receiver camera_udp_sensor_uart/linux/receiver.py --model "$MODEL" --voice play
```

目标登记状态和绿框是感知结果。UI 的实车控制需新鲜遥测、明确启动及已验证的标定。位置估计依赖轮速里程计和人工设置初始位姿。单独检查导航配置可使用：

```sh
sh rk3588/task6_navigation/run.sh --check --mode search --label cup --color orange
```

## 第三方来源

时钟 IP 的 Gowin Semiconductor 版权声明保留在 fpga/source/clock_reset.sv。工程未添加统一的开源许可证，第三方文件适用各自的许可条件。

## STMicroelectronics BSD-3-Clause 条款

Copyright 2019(-2020) STMicroelectronics.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
