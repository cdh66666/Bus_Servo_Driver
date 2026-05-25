# Bus Servo Driver

基于 ESP32-C3 SuperMini 的飞特 BUS 总线舵机兼容驱动板固件。

这个仓库包含两套 PlatformIO Arduino 固件：

- `bridge_com57`：主机/桥接板固件。电脑通过 USB CDC 打开 COM57，固件把飞特官方上位机协议转发到单线 BUS，同时本机也模拟一个 HLS3606 兼容舵机。
- `node_com58`：从机/总线节点固件。作为 BUS 上的第二个虚拟 HLS3606 兼容舵机。

当前代码已经按 `FT SCServo Debug V1.9.8.6` 官方上位机测试过。

## 当前测试拓扑

```text
电脑 / 飞特 FD 官方上位机
        |
      COM57
        |
ESP32-C3 主机桥接板，虚拟舵机 ID 2
        |
单线 BUS DATA
        |
+-------------------+-------------------+
|                   |                   |
真实 HLS3915M ID 1  ESP32-C3 从机 ID 3  其他 BUS 舵机
1 Mbps BUS          1 Mbps BUS
```

电脑端仍然选择 `COM57`、`115200` 波特率。桥接固件会在板间 BUS 上使用 `1 Mbps` 与真实 HLS3915M 和 ID3 从机通信，再把回包转回电脑。

## 硬件引脚

ESP32-C3 SuperMini 引脚分配：

| GPIO | 用途 |
| --- | --- |
| GPIO21 | BUS UART TX |
| GPIO20 | BUS UART RX |
| GPIO1 | DRV8876 IN1 |
| GPIO2 | DRV8876 IN2 |
| GPIO3 | DRV8876 nFAULT |
| GPIO4 | 电机电流 ADC |
| GPIO8 | MT6701 I2C SDA |
| GPIO9 | MT6701 I2C SCL |

当前固件重点用于飞特 BUS 协议兼容和上位机调试。电机驱动、磁编码器和闭环控制相关头文件已经保留在 `include/` 中，但 ID2/ID3 的位置、电流、温度等反馈目前是模拟值，还没有接入真实电机和 MT6701 闭环。

## BUS 电气连接

当前板子使用单线 BUS 自动收发电路，DATA 线上需要上拉电阻。本项目已经用外接 `10k` 上拉测试通过。

接线检查：

- 所有 ESP32 板、真实舵机电源必须共地。
- 真实飞特舵机需要单独合适的供电。
- BUS DATA 需要串接到所有设备。
- 同一条 BUS 上不能有重复 ID。
- 如果真实舵机没有供电，上位机只能搜到虚拟 ID2/ID3。

## 默认 ID 和波特率

当前默认配置：

| 设备 | ID | BUS 波特率 |
| --- | ---: | ---: |
| 真实飞特 HLS3915M | 1 | 1,000,000 |
| COM57 主机板本地虚拟舵机 | 2 | 本地处理，不走物理 BUS |
| COM58 从机板虚拟舵机 | 3 | 1,000,000 |

两个虚拟舵机会返回 HLS3606 兼容型号：

```text
固件版本：3.40
型号版本：10.10
官方 FD 型号名：HLS3606
```

如果想让官方 FD 显示自定义名称 `cdh`，需要修改 FD 软件目录中的型号表：

```text
ft_setup_bat/setup.log
ft_setup_bat/setup_en.log
```

把：

```text
10,10,10,0,HLS3606
```

改成：

```text
10,10,10,0,cdh
```

修改后重启 FD 软件才会生效。这个改动只影响本机这份 FD 软件，不影响固件协议。

## 仓库结构

```text
.
├── include/
│   ├── HLS3606Emu.h          # 飞特/HLS 协议模拟器，主机和从机共用
│   ├── DRV8876_PH_EN.h       # DRV8876 电机驱动辅助代码，预留
│   ├── MT6701MultiTurn.h     # MT6701 磁编码器辅助代码，预留
│   └── ClosedLoopServo.h     # 闭环控制辅助代码，预留
├── src/
│   ├── bridge/main.cpp       # COM57 主机桥接固件，本地虚拟 ID2
│   └── node/main.cpp         # COM58 从机节点固件，虚拟 ID3
├── platformio.ini
└── README.md
```

## 编译

在仓库根目录运行：

```powershell
python -m platformio run
```

只编译主机桥接板：

```powershell
python -m platformio run -e bridge_com57
```

只编译从机节点：

```powershell
python -m platformio run -e node_com58
```

## 烧录

烧录前请关闭飞特 FD 软件、串口助手、PlatformIO Monitor 等会占用串口的软件。

烧录 COM57 主机桥接板：

```powershell
python -m platformio run -e bridge_com57 -t upload
```

烧录 COM58 从机节点：

```powershell
python -m platformio run -e node_com58 -t upload
```

当前 `platformio.ini` 默认：

- 主机桥接板：`COM57`
- 从机节点板：`COM58`

如果 Windows 重新分配了端口，请修改 `platformio.ini` 里的 `upload_port` 和 `monitor_port`。

查看端口：

```powershell
python -m platformio device list
```

## 使用飞特官方上位机测试

1. 烧录两块 ESP32-C3 板。
2. 给真实 HLS3915M 舵机供电。
3. 打开 `FT SCServo Debug V1.9.8.6`。
4. 端口选择 `COM57`。
5. 波特率选择 `115200`。
6. 点击打开。
7. 点击搜索。

预期搜索结果：

```text
ID 1  HLS3915
ID 2  HLS3606 或 cdh
ID 3  HLS3606 或 cdh
```

如果右侧“舵机反馈”一直显示通信超时，请先点击左侧列表中的某个 ID 行，让它变成选中状态。未选中舵机时，上位机可能不会持续读取对应设备。

## 通信测试参考结果

当前实测，COM57 对电脑端为 `115200`，BUS 端 ID1/ID3 为 `1 Mbps`：

```text
ID1 平均响应：约 1.2 ms
ID2 平均响应：约 0.3 ms
ID3 平均响应：约 1.3 ms
```

ID2 最快，因为它在 COM57 主机板本地直接处理，不经过物理 BUS。ID3 需要 COM57 转发到 BUS，再由 COM58 回包，所以天然比 ID2 多一跳。

## 协议支持

固件支持常见飞特总线包格式：

```text
FF FF ID LENGTH INSTRUCTION PARAMS... CHECKSUM
```

已实现指令：

| 指令 | 代码 | 状态 |
| --- | ---: | --- |
| PING | `0x01` | 支持 |
| READ | `0x02` | 支持 |
| WRITE | `0x03` | 支持 |
| REG_WRITE | `0x04` | 支持 |
| ACTION | `0x05` | 支持 |
| RESET | `0x06` | 支持 |
| SYNC_WRITE | `0x83` | 支持 |

HLS 风格控制表当前包含：

- 固件版本
- 型号版本
- ID
- 波特率
- 应答状态级别
- 角度限制
- PID 和电流环相关寄存器
- 扭矩开关
- 目标位置
- 目标速度
- 当前位置
- 当前速度
- 当前负载
- 当前电压
- 当前温度
- 移动标志
- 当前电流

## 已知限制

- ID2/ID3 的位置、速度、电流、电压、温度为模拟反馈。
- 当前还没有把 DRV8876 和 MT6701 接入闭环控制。
- 桥接固件针对当前测试环境做了加速假设：真实 HLS3915M 为 ID1、1 Mbps；从机虚拟舵机为 ID3、1 Mbps。
- 如果在 FD 软件中修改虚拟舵机的 ID 或波特率，固件能保存部分配置，但为了保持最快搜索和通信速度，建议同步更新桥接固件里的默认假设。

## 常见问题

### 烧录失败

请检查：

- FD 软件是否关闭。
- 串口助手是否关闭。
- COM57/COM58 是否仍然存在。
- ESP32-C3 是否需要重新插拔。

### FD 搜不到设备

请检查：

- COM57 是否选择正确。
- PC 侧波特率是否为 `115200`。
- 真实舵机是否供电。
- 所有设备是否共地。
- BUS DATA 是否接好。
- DATA 上拉电阻是否存在。
- 是否存在重复 ID。

### ID3 响应慢或超时

请检查：

- COM58 是否烧录了本仓库最新 `node_com58` 固件。
- ID3 的波特率寄存器 `0x06` 是否为 `0`，即 1 Mbps。
- COM57 和 COM58 之间 DATA/GND 是否可靠。
- BUS 线上是否有过长飞线或接触不良。

### 型号显示 Unknown

虚拟舵机必须返回型号字节：

```text
0A 0A
```

官方 FD 原始配置会显示为 `HLS3606`。如果你希望显示 `cdh`，按上面的“默认 ID 和波特率”章节修改 FD 配置表，然后重启 FD。
