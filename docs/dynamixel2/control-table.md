# TK Servo Control Table

所有多字节值均为小端序。`RW/NVM` 表示写入运行参数后，需向地址 `152` 写 `1` 才保存；运行目标、使能状态和多圈计数不保存。

| 地址 | 长度 | 类型 | 权限 | 名称 | 说明 |
| ---: | ---: | --- | --- | --- | --- |
| 0 | 2 | U16 | RO | Model Number | 型号，当前 `0x0001` |
| 2 | 1 | U8 | RO | Firmware Version | 固件版本 |
| 3 | 1 | U8 | RO | Protocol Version | 固定为 `2` |
| 4 | 1 | U8 | RW/NVM | Node ID | `1..252` |
| 5 | 1 | U8 | RW/NVM | Baud Code | `2/3/4` 对应 115200/1M/2M |
| 6 | 2 | U16 | RW/NVM | Serial Watchdog | 毫秒；`0` 禁用 |
| 8 | 1 | U8 | RW/NVM | Node Position | 广播 Ping 回复顺序，从 `1` 开始 |
| 9 | 2 | U16 | RW/NVM | Reply Slot | 微秒，最小 `50` |
| 16 | 1 | U8 | RW | Control Source | `0=关闭`，`1=DYNAMIXEL`，`2=PWM输入` |
| 17 | 1 | U8 | RW | Servo Mode | `0=电流`，`1=速度`，`2=位置` |
| 18 | 2 | U16 | RW | Control Word | bit0 使能，bit2 清故障 |
| 20 | 2 | I16 | RW | Target Current | mA |
| 22 | 4 | I32 | RW | Target Velocity | 内部速度单位 |
| 26 | 4 | I32 | RW | Target Position | 原点偏移后的内部位置 |
| 30 | 4 | U32 | RW | Execute Tick | `0` 立即应用；非零为 1 ms 时基 |
| 40 | 2 | U16 | RO | Status Word | 运行状态位 |
| 42 | 2 | U16 | RO | Fault Code | 故障码 |
| 44 | 2 | I16 | RO | Actual Current | 带方向的逻辑电流，mA |
| 46 | 4 | I32 | RO | Actual Velocity | 内部速度单位 |
| 50 | 4 | I32 | RO | Actual Position | 原点偏移后的内部单圈位置 |
| 54 | 4 | I32 | RO | Multi-turn Position | 运行期多圈累计，不写 NVM |
| 58 | 2 | I16 | RO | Drive Output | 有符号驱动输出 |
| 60 | 2 | U16 | RO | Supply Voltage | mV |
| 62 | 1 | I8 | RO | Temperature | 摄氏度 |
| 64 | 15 | 混合 | RW/NVM | Current PID | 见 PID 子结构 |
| 80 | 15 | 混合 | RW/NVM | Velocity PID | 见 PID 子结构 |
| 96 | 15 | 混合 | RW/NVM | Position PID | 见 PID 子结构 |
| 112 | 1 | I8 | RW/NVM | Temperature Limit | `20..85 C` |
| 114 | 2 | U16 | RW/NVM | Speed Limit | 最小 `1000` |
| 116 | 1 | U8 | RW/NVM | PWM Mode | `2/3/4` |
| 117 | 1 | Bool | RW/NVM | Encoder Direction | 停机时写入 |
| 118 | 2 | U16 | RW/NVM | Encoder Offset | `0..16383` |
| 120 | 1 | U8 | RW/NVM | Fail-safe Policy | `0=关闭输出`，`1=制动`，`2=回退PWM` |
| 128 | 2 | U16 | RO | Last Diagnostic | 最近通信诊断码 |
| 130 | 4 | U32 | RO | Diagnostic Count | 通信诊断总数 |
| 134 | 4 | U32 | RO | UART Error Count | UART 错误数 |
| 138 | 4 | U32 | RO | CRC Error Count | CRC 错误数 |
| 142 | 4 | U32 | RO | Bad Packet Count | 坏包数 |
| 146 | 4 | U32 | RO | RX Packet Count | 有效接收包数 |
| 150 | 1 | U8 | WO | Clear Diagnostics | 写 `1` 清零诊断 |
| 152 | 1 | U8 | WO | Save NVM | 写 `1` 请求保存配置 |

## PID 子结构

电流环、速度环和位置环的布局相同，以下偏移相对于各自基地址：

| 偏移 | 长度 | 类型 | 名称 |
| ---: | ---: | --- | --- |
| +0 | 2 | U16 | Kp |
| +2 | 2 | U16 | Ki |
| +4 | 2 | U16 | Kd |
| +6 | 4 | I32 | Integral Max |
| +10 | 2 | U16 | Output Max |
| +12 | 2 | U16 | Output Min |
| +14 | 1 | U8/WO | 写 `1` 复位该环运行状态 |

PID 写入立即作用于运行参数，但不会切换控制源、不会清故障、不会使能输出，也不会自动保存到 NVM。
