# ESP32-C3 电机驱动板 V1.4 硬件映射

参考原理图：`C:\Users\admin\Downloads\SCH_Schematic1_4_2026-06-24.pdf`

本文件记录当前固件使用的 V1.4 硬件连接，便于后续调试、SMT 小批量和固件移植。

## 总体结构

```text
6-9V 输入
   |
   +-- 5V LDO -- 3.3V LDO -- ESP32-C3 / 编码器 / BUS 电平
   |
   +-- DRV8876 VM -- 直流电机

ESP32-C3
   |-- UART0: 下载/调试
   |-- UART1: 单线半双工 BUS
   |-- I2C: MT6701 或 AS5600 编码器
   |-- PWM/PH: DRV8876 电机驱动
   |-- ADC: 输入电压、电机电流
```

固件不启用 Wi-Fi/BLE，板上也不需要接 RF 天线。

## GPIO 分配

| ESP32-C3 GPIO | 网络/功能 | 方向 | 说明 |
| ---: | --- | --- | --- |
| GPIO0 | `BUS_TX` | 输出 | BUS UART TX，经三态门驱动 DATA |
| GPIO1 | `BUS_RX` | 输入 | BUS UART RX，从 DATA 接收 |
| GPIO2 | 启动脚上拉 | 输入 | 10k 上拉到 3.3V，不做外设 |
| GPIO3 | `IO3` / VIN ADC | 输入 | VCC 分压采样 |
| GPIO4 | `IO4` / IPROPI ADC | 输入 | DRV8876 电流反馈 |
| GPIO5 | `IO5` / Motor EN | 输出 | PWM |
| GPIO6 | `IO6` / Motor PH | 输出 | 方向 |
| GPIO7 | `IO7` / nFAULT | 输入 | DRV8876 故障，低有效 |
| GPIO8 | 启动脚上拉 | 输入 | 10k 上拉到 3.3V，不做外设 |
| GPIO9 | BOOT | 输入 | 下载模式按键，10k 上拉，按下到 GND |
| GPIO10 | LED | 输出 | 高电平点亮 |
| GPIO18 | I2C SCL | 输出 | 5.1k 上拉到 3.3V |
| GPIO19 | I2C SDA | 双向 | 5.1k 上拉到 3.3V |
| GPIO20 | `U0RXD` | 输入 | USB-TTL TXD 接入 |
| GPIO21 | `U0TXD` | 输出 | USB-TTL RXD 接入 |

## 下载接口

H1：

| H1 pin | 网络 | 外部 USB-TTL |
| ---: | --- | --- |
| 1 | GND | GND |
| 2 | `U0RXD` | TXD |
| 3 | `U0TXD` | RXD |

手动下载流程：

1. USB-TTL 使用 3.3V 电平，并与板子共地
2. 按住 `BOOT`
3. 短按 `EN`
4. 开始烧录后松开 `BOOT`

当前 H1 只引出 RX/TX/GND，不能自动控制 EN/BOOT；如果后续要自动烧录，需要额外引出 EN 和 BOOT，或做 ESP-Prog 兼容下载口。

## 电源

```text
VCC / 6-9V -> 5V LDO -> +5V
+5V       -> 3.3V LDO -> +3V3
```

调试注意：

- ESP32-C3、编码器和 BUS 逻辑使用 3.3V
- DRV8876 电机电源使用 VCC
- 电机启动和刹车会冲击电源，电机回流路径要和 ESP32 模拟/数字地尽量分清
- 已验证 7V 输入下固件电压反馈约 7.0-7.1V

## 输入电压检测

```text
VCC -> 20k -> GPIO3 -> 5.1k -> GND
GPIO3 -> 1uF -> GND
```

换算：

```text
VCC = Vadc * (20k + 5.1k) / 5.1k
VCC = Vadc * 4.9216
```

固件将结果写入飞特 `Present Voltage`，单位为 0.1V。

## 编码器接口

I2C：

```text
SCL = GPIO18
SDA = GPIO19
I2C clock = 400 kHz
```

支持两种磁编码器：

| 编码器 | 地址 | 角度寄存器 | 固件处理 |
| --- | ---: | --- | --- |
| MT6701 | `0x06` | `0x03/0x04` | 14 bit 原始角度 |
| AS5600 | `0x36` | `0x0C/0x0D` | 12 bit 角度左移到 14 bit |

固件启动时自动识别；运行中掉线会重探测。诊断寄存器 `0x66` 给出类型：`1=MT6701`，`2=AS5600`。

## BUS 单线半双工

BUS 接口：

| pin | 网络 |
| ---: | --- |
| 1 | GND |
| 2 | DATA |
| 3 | VCC |

固件映射：

```text
BUS_TX = GPIO0
BUS_RX = GPIO1
默认 BUS 波特率 = 1,000,000 baud
```

硬件逻辑：

- U8 `SN74LVC1G126` 将 `BUS_TX` 驱动到 `DATA`
- Q1 根据 `BUS_TX` 自动生成 TXEN
- U6 `SN74LVC1G126` 将 `DATA` 接收到 `BUS_RX`
- DATA 需要上拉；多板并联后等效上拉会降低

1 Mbps 下如果总线只挂 1-2 块板且线较长，建议实测 DATA 上升沿；必要时将某一块板的上拉调整到 2.2k-4.7k，或在主控端加外部上拉。

## DRV8876 电机驱动

| DRV8876 信号 | ESP32-C3 | 说明 |
| --- | ---: | --- |
| EN/IN1 | GPIO5 | PWM 输出 |
| PH/IN2 | GPIO6 | 方向 |
| nFAULT | GPIO7 | 故障输入，低有效 |
| IPROPI | GPIO4 | 电流 ADC |
| VM | VCC | 电机电源 |

电流检测：

```text
IPROPI -> GPIO4
GPIO4 -> R14 2.5k -> GND
GPIO4 -> C18 100nF -> GND
```

固件当前按 `I_IPROPI ~= I_MOTOR / 1000` 粗略换算：

```text
I_MOTOR ~= V_IO4 / 2.5
约等于 0.4 mA / mV
```

这个值适合调试和限流起步，最终量产前建议用电流表做标定。

## V1.4 调试重点

- 无源晶振负载电容要确认是电容，不要误焊成电阻
- VDDA / VDD3P3 相关去耦电容必须接到 3.3V，不能悬空
- `EN` 按下应从 3.3V 拉到 0V，松开回到 3.3V
- `BOOT/GPIO9` 松开应为 3.3V，按下应低于 0.3V
- UART0 RX/TX 交叉接 USB-TTL，烧录时 RX 线上应能看到下载波形
- 编码器上电后 `diag` 应显示 `encoder=AS5600` 或 `encoder=MT6701`
- 空载无动作时 IPROPI 通常应接近 0mV，实测几 mV 属正常偏差

## 固件初始化顺序

1. 配置 LED、DRV8876、nFAULT，确保电机默认关闭
2. 初始化 I2C，自动识别 MT6701/AS5600
3. 初始化 ADC，采样输入电压和 IPROPI
4. 初始化 BUS UART，默认 1 Mbps
5. 根据编码器当前位置恢复多圈位置
6. 进入协议响应和闭环控制循环
