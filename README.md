# Bus Servo Driver

ESP32-C3 + DRV8876 电机驱动板固件，用于把自研直流电机驱动板做成飞特 HLS/SCS 风格的总线舵机设备。当前代码对应新版 V1.4 硬件，可直接配合 `FT SCServo Debug V1.9.8.6` 搜索、配置、读状态和做位置/速度/电流控制测试。

当前实测链路：

```text
PC / FT SCServo Debug
        |
      COM48, 115200, NONE
        |
桥接板，本机虚拟 ID2
        |
  单线 BUS, 1 Mbps
        |
+-------+----------------+
|                        |
官方飞特舵机 ID1          从机板 ID3
```

## 功能

- 兼容飞特 HLS/SCS 风格数据包：`PING`、`READ`、`WRITE`、`REG_WRITE`、`ACTION`、`RESET`、`SYNC_WRITE`
- 桥接固件支持 PC 串口到 1 Mbps 单线 BUS 的透明转发，同时本板响应虚拟舵机 ID2
- 从机固件只挂在 BUS 上，默认响应 ID3
- 支持位置、速度、电流、PWM 直驱模式
- 支持多圈绝对位置，命令范围 `-30000..30000`
- 支持 MT6701 / AS5600 编码器自动识别和运行中重探测
- 支持输入电压、电机电流、故障脚、编码器状态等反馈
- 支持 DRV8876 IPROPI 电流采样和软限流
- 提供串口调试命令和 `tools/auto_tune.py` 自动测试脚本

## 当前硬件

硬件说明见 [flyservo_esp32c3_hardware_map.md](flyservo_esp32c3_hardware_map.md)。

| ESP32-C3 GPIO | 用途 |
| ---: | --- |
| GPIO0 | BUS UART TX |
| GPIO1 | BUS UART RX |
| GPIO3 | 输入电压 ADC，VCC 分压 |
| GPIO4 | DRV8876 IPROPI 电流 ADC |
| GPIO5 | DRV8876 EN/IN1，PWM |
| GPIO6 | DRV8876 PH/IN2，方向 |
| GPIO7 | DRV8876 nFAULT，低有效 |
| GPIO9 | BOOT 按键 |
| GPIO10 | 状态 LED |
| GPIO18 | I2C SCL |
| GPIO19 | I2C SDA |
| GPIO20 | UART0 RX，下载/调试口 |
| GPIO21 | UART0 TX，下载/调试口 |

固件不启用 Wi-Fi/BLE。GPIO18/GPIO19 固定用作 I2C，不再作为 USB Serial/JTAG 使用。

## 编码器自动识别

固件启动后会扫描 I2C，并按下面顺序识别编码器：

1. 优先尝试 MT6701，7-bit 地址 `0x06`，读取 `0x03/0x04` 的 14 bit 角度
2. 如果 MT6701 不存在或读数失败，再尝试 AS5600，7-bit 地址 `0x36`，读取 `0x0C/0x0D` 的 12 bit 角度
3. AS5600 读数会左移 2 bit，统一换算成内部 14 bit 原始角度，再转换为 4096 count/圈
4. 如果运行中编码器掉线，固件会节流重探测；重新识别后会尽量对齐原来的多圈位置，避免位置突然大跳

如果两种编码器同时挂在 I2C 上，固件优先使用 MT6701。正常板子只接其中一种即可。

调试寄存器：

| 地址 | 含义 |
| ---: | --- |
| `0x57..0x58` | 编码器原始 14 bit 角度 |
| `0x59..0x5C` | 多圈位置，int32，单位 4096 count/圈 |
| `0x5D` | I2C 扫描到的第一个地址 |
| `0x5E` | 当前使用的编码器地址，MT6701=`0x06`，AS5600=`0x36` |
| `0x5F` | 最近一次编码器读取状态，`0/1` 为 MT6701，`2` 为 AS5600，`0xF0` 为未识别 |
| `0x60..0x61` | IPROPI 原始电压，mV |
| `0x62..0x63` | 电流换算值，mA |
| `0x64..0x65` | 最近电机输出力度，-1000..1000 |
| `0x66` | 编码器类型，`0` none，`1` MT6701，`2` AS5600 |

## 编译和烧录

安装依赖：

```powershell
pip install platformio pyserial
```

编译默认环境：

```powershell
python -m platformio run
```

当前 V1.4 实测端口：

```powershell
# 桥接板，PC/上位机接 COM48
python -m platformio run -e bridge_com48_v14 -t upload

# 从机板，BUS 节点接 COM41
python -m platformio run -e node_com41_v14 -t upload
```

如果换了串口号，可以改 [platformio.ini](platformio.ini)，或者临时加 `--upload-port COMx`。V1.4 板没有自动下载电路时，需要手动进入下载模式：按住 `BOOT`，短按 `EN`，开始上传后再松开 `BOOT`。

## 上位机测试

打开 `FT SCServo Debug V1.9.8.6`：

```text
Port: COM48
Baud: 115200
Parity: NONE
```

这里的 `115200` 是 PC 到桥接板的串口波特率；桥接板到 BUS 舵机侧固定按飞特总线习惯跑 `1 Mbps`。搜索时应能看到：

```text
ID1  官方飞特舵机
ID2  自研桥接板虚拟舵机
ID3  自研从机板
```

基础位置测试建议先用小幅动作，确认电机方向和电流反馈正常后再扩大范围。

## 串口调试

从机固件的 USB 串口支持：

```text
status
diag
phase 0|1
torque 0|1
read <addr> <len>
```

`diag` 会直接显示当前编码器类型，例如：

```text
encoder=AS5600 raw14=28 first_i2c=0x36 active_i2c=0x36 enc_status=0x02
```

自动测试脚本直接走飞特串口协议：

```powershell
python tools\auto_tune.py --port COM48 --ids 2 3 --apply-best --output tuning_results.json
```

测试结果 JSON、日志、CSV 等本地调试产物已经在 `.gitignore` 中忽略。

## 当前验证记录

2026-06-29 使用 COM48 桥接测试：

- ID1 / ID2 / ID3 均可搜索和读取状态
- ID1 官方舵机可做小幅位置动作
- ID2 / ID3 自研板可做小幅位置往返动作
- ID2 识别 AS5600，空闲 IPROPI 约 3-4 mV
- ID3 识别 AS5600，空闲 IPROPI 约 10-12 mV
- 7 V 供电下电压反馈约 7.0-7.1 V，错误码为 0

## 仓库结构

```text
.
|-- platformio.ini
|-- flyservo_esp32c3_hardware_map.md
|-- include/
|   |-- BoardServoHardware.h
|   `-- HLS3606Emu.h
|-- src/
|   |-- bridge/main.cpp
|   `-- node/main.cpp
`-- tools/
    `-- auto_tune.py
```

发布前检查：

```powershell
python -m platformio run
git diff --check
git status --short
```
