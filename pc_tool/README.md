# Solar IMU 路牌监测工具 V1.2.0

这是 STM32 路牌监测终端的 Windows 上位机。界面面向安装、运维和售后人员，使用 USART2（115200-8-N-1）读取设备状态、调整唤醒参数并检查 4G 上报。

## V1.2.0 功能更新（2026-08-15）

- 实时姿态改为 `0.1 / 0.2 / 0.5 / 1 / 2 秒`固定档位。新固件使用 `GET_IMU_LIVE (0x13)` 单次返回六轴与姿态，去掉旧 `GET_ANGLE` 约 150 ms 的八次平均等待。
- “ML307C”统一改为用户可理解的“4G模块”，设备总览采用电压、网络、传感器和待发送事件卡片。
- “读取全部状态”同时读取 IMEI、MCU 唯一编号和实际 MQTT 上报地址；仅在存在待发送事件时读取事件队列，因此空队列不会显示为错误。
- 参数使用下拉选项：加速度以 mg、倾角以度、低压统一以 V、RTC 统一以分钟显示；默认 RTC 周期为 60 分钟，最低 10 分钟。
- 修复工作参数写入链路。写入后自动执行 `GET_STATUS + GET_IMU_DIAG`，逐项回读一致后才报告成功。
- 事件队列并入设备总览，不再单独占用页面。
- 六轴页面可分别勾选加速度、角速度和姿态角。
- 恢复并提交完整 Python 源码，后续版本不再依赖安装包逆向恢复。
- 实时姿态改为带 XYZ 参考坐标的 3D 实体预览，直观显示设备当前 Pitch 和 Roll。
- 增加 4G 网络状态检测、可选 GPS 的立即采集上报、低功耗待机按钮及统一操作反馈；设备身份和上报地址支持复制。
- 界面与安装包预留并使用杭州电子科技大学 Logo 资源。

## 推荐参数

路牌监测建议从 `750 mg + 30°` 开始。750 mg 用于降低车辆经过、风振和轻微触碰造成的误唤醒；30°在 LSM6DS3TR-C 的四档 6D 阈值中兼顾防误报和明显倾倒检测。实际部署后应结合一周事件数据再微调。

## 开发运行

```powershell
cd pc_tool
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
$env:PYTHONPATH='.'
.\.venv\Scripts\python.exe -m app.main
```

## MQTT远程消息页面

上位机新增“MQTT远程消息”页面，串口连接和 Broker 连接互相独立。Broker 详细设置默认收起，需要时点击“显示 Broker 设置”展开。发布消息和订阅消息区域也可以分别收起。发布消息支持两种模式：`手动文本`保留原来的多行 JSON 编辑器和默认格式；`自动填充`只需要填写加速度阈值 `wu`、倾角阈值 `tilt` 和唤醒/休眠时间 `sleep`，页面会实时生成配置 JSON，并在发布时使用当前真实 UTC Unix 时间戳（秒）作为 `cmd_id`。同一秒内重复发布会被阻止，不会人为加一生成未来时间戳；两种模式的内容分别保存，切换模式不会覆盖手动文本。设备只接受当前时间前后7200秒（含边界）内的 `cmd_id`，因此自动发布前应确保电脑时间和设备RTC已完成校时。页面提供三个独立订阅窗口，默认分别为 `device/+/data`、`device/+/ack` 和空白自定义 Topic。每个窗口都可以单独选择 QoS、订阅或退订、清空历史，并以 MQTTX 风格的连续历史文本展示接收时间、实际 Topic、QoS、Retain 和 Payload。页面可自定义 Broker 地址、端口、用户名、密码和 Client ID，并支持发布/订阅、QoS 0/1/2 和 Retain；这些输入配置会保存到当前用户的 Qt 配置存储中。

固件远程配置使用按设备 Topic：

```text
device/<IMEI>/settings
```

配置 Payload 不再包含 `imei`，例如：

```json
{"cmd_id":1760000000,"ver":1,"wu":750,"tilt":30,"sleep":1800}
```

观察所有设备上报时可订阅：

```text
device/+/data
```

`/device/+/data` 与 `device/+/data` 是两个不同的 MQTT Topic，页面会按用户输入原样订阅，不自动增加或删除前导斜杠。当前开发 Broker 使用明文 1883，量产环境应改用 TLS 和独立设备凭据。

## Conda solarimu环境

```powershell
conda activate solarimu
cd D:\.code\solar_imu\pc_tool
python -m pip install -r requirements.txt
$env:PYTHONPATH = (Get-Location).Path
$env:QT_QPA_PLATFORM = 'offscreen'
python -m unittest discover -s tests -v
python -m app.main --smoke-test
```



```powershell
$env:PYTHONPATH='.'
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\pyinstaller.exe --noconfirm SolarIMU_Tool.spec
```

正式发布使用与 V1.0.1 相同的 Inno Setup 安装包模式：

```powershell
.\build_installer.ps1
```

V1.2.0 继续保留 V1.1.1 的发布环境隔离修复，避免开发机 Anaconda ICU DLL 混入发布包、导致目标电脑无法加载 QtWidgets。

生成文件为 `pc_tool/release_dist/SolarIMU_Tool_Setup_v1.2.0_Win64.exe`。安装包会把主程序和完整 Qt 运行库安装到当前用户目录，并默认创建开始菜单与桌面快捷方式；安装目录和开始菜单中也会生成清晰的“卸载 Solar IMU 路牌监测工具”入口。目标电脑不需要安装 Python、PySide6 或串口库。

构建脚本会临时隔离系统 `PATH`，防止开发机上的 Anaconda、Matlab 或其他 Qt 软件把不兼容的 ICU/Qt DLL 混入发布包；构建后还会检查发布目录中是否出现外部 ICU DLL。正式发布前应在没有 Python 环境的 Windows 电脑上验证 CH340 连接、参数写入回读和卸载流程。

安装后可用 `SolarIMU_Tool.exe --smoke-test` 执行发布包自检：程序成功加载 Qt、主题、全部页面和串口模块后会自动以退出码0结束。
