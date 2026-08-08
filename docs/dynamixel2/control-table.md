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
| 9 | 2 | U16 | RW/NVM | Reply Slot Width | 响应时隙宽度下限，us；范围 `50..8000`，实际值会按响应包最坏填充后的线速时长自动增大 |
| 16 | 1 | U8 | RW | Control Source | `0=关闭`，`1=DYNAMIXEL`，`2=PWM输入` |
| 17 | 1 | U8 | RW | Servo Mode | `0=电流`，`1=速度`，`2=位置` |
| 18 | 2 | U16 | RW | Control Word | bit0 使能，bit1 使用 Execute Tick 调度，bit2 清故障 |
| 20 | 2 | I16 | RW | Target Current | mA |
| 22 | 4 | I32 | RW | Target Velocity | 编码器计数/秒，counts/s |
| 26 | 4 | I32 | RW | Target Position | 原点偏移后的有符号多圈位置，count |
| 30 | 4 | U32 | RW | Execute Tick | 绝对 1 ms 时基；`0` 是回绕后的合法时刻，不代表立即执行 |
| 34 | 2 | U16 | RW | Command Sequence | 主站周期命令序号，按模 `65536` 递增 |
| 36 | 2 | U16 | RO | Applied Sequence | 最近已生效命令的 Command Sequence |
| 38 | 1 | U8 | RO | Last Command Result | 最近命令映像的标准 Status Error；`0` 表示成功 |
| 39 | 1 | U8 | RO | Reserved | 固定读取为 `0`，用于保持推荐反馈映像连续 |
| 40 | 2 | U16 | RO | Status Word | 位定义见下文 |
| 42 | 2 | U16 | RO | Fault Code | 枚举见下文 |
| 44 | 2 | I16 | RO | Actual Current | 带方向的逻辑电流，mA |
| 46 | 4 | I32 | RO | Actual Velocity | 编码器计数/秒，counts/s |
| 50 | 4 | I32 | RO | Actual Position | 原点偏移后的单圈位置，`0..16383` count |
| 54 | 4 | I32 | RO | Multi-turn Position | 有符号多圈位置，count；运行期累计，不写 NVM |
| 58 | 2 | I16 | RO | Drive Output | PWM 满量程千分比，`-1000..1000` |
| 60 | 2 | U16 | RO | Supply Voltage | mV |
| 62 | 1 | I8 | RO | Temperature | 摄氏度 |
| 64 | 15 | 混合 | RW/NVM | Current PID | 见 PID 子结构 |
| 80 | 15 | 混合 | RW/NVM | Velocity PID | 见 PID 子结构 |
| 96 | 15 | 混合 | RW/NVM | Position PID | 见 PID 子结构 |
| 112 | 1 | I8 | RW/NVM | Temperature Limit | `20..85 C` |
| 114 | 2 | U16 | RW/NVM | Speed Limit | `1000..65535 counts/s`；当前为兼容保留的 U16 字段 |
| 116 | 1 | U8 | RW/NVM | PWM Mode | `2/3/4` |
| 117 | 1 | Bool | RW/NVM | Encoder Direction | 停机时写入 |
| 118 | 2 | U16 | RW/NVM | Encoder Offset | `0..16383` |
| 120 | 1 | U8 | RW/NVM | Fail-safe Policy | `0=关闭输出`，`1=制动`，`2=回退PWM` |
| 122 | 4 | U32 | RO | Current Tick | 从协议初始化开始累计的 1 ms 时基 |
| 126 | 2 | U16 | RO | PWM Input Low Width | 最近捕获的 PWM 低电平宽度，µs；仅观测，不保存 |
| 128 | 2 | U16 | RO | Last Diagnostic | 枚举见下文 |
| 130 | 4 | U32 | RO | Diagnostic Count | 通信诊断总数 |
| 134 | 4 | U32 | RO | UART Error Count | UART 错误数 |
| 138 | 4 | U32 | RO | CRC Error Count | CRC 错误数 |
| 142 | 4 | U32 | RO | Bad Packet Count | 坏包数 |
| 146 | 4 | U32 | RO | RX Packet Count | 有效接收包数 |
| 150 | 1 | U8 | WO | Clear Diagnostics | 写 `1` 清零诊断 |
| 152 | 1 | U8 | WO | Save NVM | 写 `1` 保存配置；单播 Status 在 Flash 完成后返回 |

## Response / ACK Policy

本设备不开放可写的 Status Return Level，固定采用等级 `2` 的单播回复策略。所有回复均为标准 DYNAMIXEL Protocol 2.0 Status Packet：

```text
FF FF FD 00 | Device ID | Length | 55 | Error | Parameters... | CRC16
```

不引入 `AA 55`、自定义 `RESULT` 字段或包头级 `SEQ`。周期序号通过控制表地址 `34/36` 传递，因此仍可使用标准 DYNAMIXEL SDK。

| 操作 | 单播行为 | 广播 `0xFE` 行为 |
| --- | --- | --- |
| Ping | 必须返回型号、固件版本和 `Error=0` | 所有节点按 Node Position/Reply Slot 排队回复 |
| Read | 必须返回 `Error + Data`；失败时无 Data | 不执行，不回复 |
| Write | 必须返回 `Error`，该 Status 即 ACK | 执行匹配写入，不回复 |
| Reg Write | 必须确认“已登记”；数据尚未生效 | 可登记，不回复 |
| Action | 必须确认已执行；Save NVM 除外，见下文 | 执行已登记写入，不回复 |
| Sync Write | 不适用 | 匹配节点执行，不回复 |
| Sync Read | 不适用 | ID 列表中的节点按列表顺序返回 `Error + Data` |
| 未支持指令 | 返回 Instruction Error | 不回复 |

广播包即使长度、地址或数据错误也不返回 Status Packet，避免多节点同时发送造成总线冲突。CRC 错误包同样不回复，因为完整 ID 和包体尚未通过校验；错误只记录到 Last Diagnostic 和 CRC Error Count。

### Status Packet Error

`Error` 的 bit7 是硬件告警位：`Fault Code != 0` 时置 `1`，主站随后应读取 Fault Code；bit0..6 仍表示当前事务的标准错误编号：

| 值 | 名称 | 本固件使用场景 |
| ---: | --- | --- |
| `0x00` | None | 指令成功，或 Read 成功并附带数据 |
| `0x01` | Result Fail | NVM 擦写失败、重复的未完成 Save NVM 等无法归类的执行失败 |
| `0x02` | Instruction Error | 单播收到未实现指令 |
| `0x03` | CRC Error | 标准保留语义；接收 CRC 错误时本设备不回包 |
| `0x04` | Data Range Error | 值超出允许范围 |
| `0x05` | Data Length Error | 参数数量或对象宽度错误 |
| `0x06` | Data Limit Error | 标准保留语义；当前未使用 |
| `0x07` | Access Error | 地址不存在、写只读对象或状态不允许写入 |

### Save NVM 完成确认

地址 `152` 是特殊的延迟事务：Flash 擦写不在 UART 回调中执行。单播 Write 写 `1` 后，从站先把请求交给主循环，只有 `NvmParam_Save()` 真正完成后才发送 Status Packet：

- `Error=0x00`：参数已经持久化，或内容与最新记录相同，无需重复擦写。
- `Error=0x01`：Flash 擦除、编程或参数保存失败。
- 当前输出、活动命令或待生效命令处于使能状态时拒绝保存，并返回 `Access Error=0x07`。主站必须先显式停机并确认 `Status Word.Output Enabled=0`。
- 前一笔单播保存尚未完成时再次请求，返回 `0x01`。
- 广播或 Sync Write 保存仍不回复；需要可靠确认时必须对每个节点执行单播 Save NVM。

Reg Write 地址 `152` 的 Status 只表示保存命令已登记；后续单播 Action 的 Status 才表示 Flash 保存的最终结果。

### 周期广播的序号确认

推荐主站用 Sync Write 从地址 `16` 写入完整 `20` 字节命令映像：

| 偏移 | 地址 | 长度 | 字段 |
| ---: | ---: | ---: | --- |
| `+0` | 16 | 1 | Control Source |
| `+1` | 17 | 1 | Servo Mode |
| `+2` | 18 | 2 | Control Word |
| `+4` | 20 | 2 | Target Current |
| `+6` | 22 | 4 | Target Velocity |
| `+10` | 26 | 4 | Target Position |
| `+14` | 30 | 4 | Execute Tick |
| `+18` | 34 | 2 | Command Sequence |

旧的地址 `16`、长度 `14` 命令映像继续兼容，但不携带 Execute Tick 和序号。主站应令 Command Sequence 按模 `65536` 递增；回绕后从 `65535` 变为 `0`。

完整 `20` 字节命令映像按一次事务校验和提交，不会逐字段产生中间状态。固件仅保留一个待生效命令槽：

- 与待生效序号或最近已应用序号相同的完整命令视为重传，幂等返回成功，不重复应用，也不刷新 Serial Watchdog。
- 待生效槽已占用时，收到不同序号的完整命令返回 `Result Fail=0x01`，不会覆盖原命令。
- Control Word bit1=`0` 时完整映像在下一个 1 ms 协议周期应用，Execute Tick 字段被忽略。
- Control Word bit1=`1` 时在 Execute Tick 到期后应用；地址 `0` 是合法绝对时刻。
- 长度 `14` 的兼容映像始终立即提交，不参与序号去重或绝对时刻调度。

周期控制后的确认使用 Sync Read：从地址 `36` 读取 `27` 字节。返回数据依次包含：

```text
Applied Sequence U16
Last Command Result U8
Reserved U8
Status Word ... Temperature（地址 40..62）
```

`Applied Sequence == 本周期 Command Sequence` 且 `Last Command Result == 0` 表示该命令映像已经在控制周期中生效。序号不相等表示命令尚未到 Execute Tick、未收到或校验失败；具体驱动故障仍读取 Fault Code，不能只看 Last Command Result。

## TK Sync Control（私有指令 `0xA0`）

`0xA0` 保留 DYNAMIXEL Protocol 2.0 包头、Length、Byte Stuffing 和 CRC，仅定义私有 Instruction 与 Parameters。请求必须使用广播 ID `0xFE`：

使用单播 ID 发送 `0xA0` 时，从站返回 `Instruction Error`，不会提交控制命令。

```text
Sequence       U16
Execute Mode   U8
Execute Value  U32
ACK Mask       U16
Node Count     U8

repeat Node Count:
    Node ID          U8
    Control Source   U8
    Servo Mode       U8
    Control Word     U16
    Target Current   I16
    Target Velocity  I32
    Target Position  I32
```

参数总长度为 `10 + 15 × Node Count`，V1 的 Node Count 范围为 `1..8`。节点记录顺序同时定义回复顺序，首条记录的回复索引为 `0`。

### 全局校验

以下情况使整包无效，所有节点均不执行、不回复：

- CRC、包长或 Byte Stuffing 无效；
- Node Count 超出 `1..8`，或参数长度不等于 `10 + 15 × Node Count`；
- Node ID 不在 `1..252`，或同一包存在重复 Node ID；
- ACK Mask 的 bit10..bit15 非零。

所有节点必须完成全局校验后才能提交自己的记录。没有出现在记录中的节点保持静默。

### Execute Mode V1

V1 只支持 `Execute Mode=0 (NEXT_UPDATE)`：通过校验的命令进入本节点唯一 pending 槽，并在下一次本地 1 ms 协议更新点生效。Execute Value 被保留但忽略；节点记录中的 Control Word bit1 必须为 `0`。该模式不表示多节点控制周期已经同步。

其他 Execute Mode 对包含本节点的结构有效请求返回 `Data Range Error`，不更新 Accepted Sequence，也不刷新 Serial Watchdog。

### 序号与 pending 槽

- 新序号成功进入 pending 槽后更新 Accepted Sequence，并刷新 Serial Watchdog。
- 与最近 Accepted Sequence 相同的请求是幂等重传：返回 ACK，但不重复执行、不覆盖 pending、不刷新 Watchdog，并在 ACK State 中置 Duplicate。
- pending 已被不同序号占用时返回 `Result Fail`，保留旧 Accepted/Applied/Result。
- Applied Sequence 仅在控制周期真正接管命令时更新。
- Last Command Result 只描述 Applied Sequence；被拒绝的新命令不能覆盖它。

### A0 ACK

A0 ACK 仍是标准 Status Packet。Parameters 固定从以下 6 字节开始：

```text
Accepted Sequence   U16
Applied Sequence    U16
Last Command Result U8
ACK State           U8
```

ACK State：bit0=`AcceptedValid`，bit1=`AppliedValid`，bit2=`Pending`，bit3=`Duplicate`，bit4..bit7 保留为零。启动后的 Sequence 数值初始化为零，但对应 Valid 位为零，因此不会与合法序号 0 混淆。

ACK Mask 决定随后附加的字段；无论事务成功或失败，同一 A0 请求中的所有节点都按同一个 Mask 返回相同字段结构：

| bit | 字段 | 长度 |
| ---: | --- | ---: |
| 0 | Status Word | 2 |
| 1 | Fault Code | 2 |
| 2 | Actual Current | 2 |
| 3 | Actual Velocity | 4 |
| 4 | Actual Position | 4 |
| 5 | Multi-turn Position | 4 |
| 6 | Drive Output | 2 |
| 7 | Supply Voltage | 2 |
| 8 | Temperature | 1 |
| 9 | Current Tick | 4 |

可选字段严格按 bit0 到 bit9 顺序编码。状态来自控制循环发布的双缓冲快照；协议收到 A0 后立即复制当前已发布快照，后续编码和 DMA 不再读取变化中的控制变量。

### 非阻塞回复时隙

完整帧接收事件记录 TIM1 的 1 MHz 时间戳。节点计算：

```text
wire_bytes = 11 + parameter_bytes + floor((parameter_bytes + 2) / 3)
packet_time = ceil(wire_bytes × 10 × 1000000 / baud)
slot_width = max(ReplySlotUs, packet_time + 50 us)
reply_deadline = packet_end + 50 us + reply_index × slot_width
```

`wire_bytes` 按最坏 Byte Stuffing 计算，因此错误 ACK 与正常 ACK 即使实际填充字节不同也不会侵入下一时隙。等待由 TIM1 Compare 中断完成，到期后启动 UART TX DMA；UART RX 回调不执行微秒忙等。当前 16 位回复计时器要求单节点 deadline 不超过 60000 us，超出时记录 TX Drop 并保持静默。

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

## Status Word

未列出的位均为保留位，读取为 `0`。

| bit | 掩码 | 名称 | 当前语义 |
| ---: | ---: | --- | --- |
| 0 | `0x0001` | Ready | 协议实例已初始化；当前实现固定为 `1` |
| 1 | `0x0002` | PWM Input Valid | 收到范围有效且未超过 100 ms 的外部 PWM 输入 |
| 2 | `0x0004` | Output Enabled | 当前选定控制源通过保护检查并处于使能状态 |
| 3 | `0x0008` | Fault Present | `Fault Code != 0` |
| 4 | `0x0010` | Protection Inhibit | 欠压或超温保护正在禁止输出 |
| 5 | `0x0020` | Undervoltage | 电源低于 Power Save Voltage，或欠压锁存尚未越过恢复回差 |
| 6 | `0x0040` | Overtemperature | 当前温度高于 Temperature Limit |
| 8 | `0x0100` | PWM Source | 控制源为外部 PWM 输入 |
| 9 | `0x0200` | Serial Source | 控制源为 DYNAMIXEL 串口 |
| 11 | `0x0800` | Fault Free | `Fault Code == 0` |
| 12 | `0x1000` | Protocol Active | 协议任务已运行；当前实现固定为 `1` |

bit2 表示经过控制源选择和保护检查后输出已使能，不表示 H 桥当前一定存在非零占空比；零目标和输出死区仍可能使实际输出为零。

## Fault Code

Fault Code 是需要主站明确清除的锁存驱动故障。当前已实现枚举如下；未列出的数值保留。

| 值 | 名称 | 说明 |
| ---: | --- | --- |
| `0x0000` | None | 无锁存故障 |
| `0x000A` | Serial Watchdog | 已使能串口控制超过 Serial Watchdog 时间未刷新 |

低电压和超温不是锁存 Fault Code，而是实时保护原因。它们置 Status Word bit4，并分别置 bit5/bit6，且无条件撤销输出。欠压使用 `500 mV` 恢复回差；超温在温度不再高于限制值时解除。上位机不得把这些状态伪报为 Serial Watchdog。

## Last Diagnostic

Last Diagnostic 记录最近一次通信层诊断；Diagnostic Count 是饱和累计次数。向地址 `150` 写 `1` 会清除所有通信诊断计数。

| 值 | 名称 | 说明 |
| ---: | --- | --- |
| `0` | None | 尚无通信诊断 |
| `1` | UART Error | HAL UART 或 DMA 错误 |
| `2` | RX CRC | 数据包 CRC 不匹配 |
| `3` | RX Bad Packet | 包头、ID、长度或包体无效 |
| `4` | TX Drop | 当前发送和一级待发槽均占用，响应被丢弃 |
| `5` | Watchdog | 已使能串口控制停止刷新 |

## 速度和位置单位

MT6701 每圈为 `16384 count`。目标速度和实际速度均为有符号 `counts/s`：

```text
rpm = velocity_counts_per_second * 60 / 16384
velocity_counts_per_second = rpm * 16384 / 60
deg/s = velocity_counts_per_second * 360 / 16384
```

Speed Limit 地址 `114` 沿用 U16，因此范围为 `1000..65535 counts/s`，在 `16384 count/rev` 下约为 `3.66..240.00 rpm`。若后续需要超过该上限，必须通过版本化控制表新增 U32 字段，不能在现有地址上静默改变宽度。

位置分辨率为 `360 / 16384 = 0.02197265625 degree/count`。Actual Position 是偏移和方向修正后的单圈位置；Target Position 与 Multi-turn Position 使用同一个多圈 count 坐标。当前多圈计数器为 `int16_t`，因此可反馈范围约为 `-536870912..536870911 count`。

## Drive Output 和 PID 限幅单位

Drive Output 是最终物理执行器指令，`1000` 表示正方向 100% PWM，`-1000` 表示反方向 100% PWM。驱动层将绝对值小于 `10` 的指令归零。

PID 的 Output Max 表示正负对称限幅，实际范围为 `[-Output Max, +Output Max]`。Output Min 表示非零输出的最小绝对值，不是负方向下限。

| 环路 | Output Max/Min 单位 | 备注 |
| --- | --- | --- |
| 位置环 | counts/s | 输出为速度目标；Max 和 Min 均生效 |
| 速度环 | mA | 输出为电流目标；Max 和 Min 均生效 |
| 电流环 | PWM 千分比 | `1000=100%`；Max 和 Min 均生效 |

## PID 增益缩放

位置环和电流环使用位置式离散 PID：

```text
Kp_actual = Kp_register / 1000
Ki_actual_per_update = Ki_register / 1000
Kd_actual_per_update = Kd_register / 1000
```

速度环使用增量式 PID：

```text
Kp_actual = Kp_register / 1000
Ki_actual_per_update = Ki_register / 10000
Kd_actual = Kd_register / 1000
```

当前电流环、速度环和位置环更新周期分别为 `1 ms`、`5 ms` 和 `10 ms`。Ki/Kd 是离散周期系数，不是与采样周期无关的连续域参数。

## Execute Tick 和回绕

Current Tick 与 Execute Tick 都是无符号 32 位毫秒计数。主站应先读取 Current Tick，再按模 `2^32` 运算生成 Execute Tick：

```text
execute_tick = (current_tick + delay_ms) mod 2^32
```

是否使用绝对时刻由 Control Word bit1 决定，不再把数值 `0` 当作特殊标记。bit1=`0` 表示立即提交；bit1=`1` 时从站使用 `(int32_t)(current_tick - execute_tick) >= 0` 判断到期。因此最大未来延迟为 `0x7FFFFFFF ms`，约 24.85 天；超过半个 32 位周期的目标会被解释为已经到期。命令应用后调度标志被清除，Execute Tick 寄存器保留最近写入值用于诊断。

## Fail-safe Policy

| 值 | 行为 |
| ---: | --- |
| `0` | 关闭 H 桥输出并清零驱动输出 |
| `1` | 切换到制动模式并清零驱动输出 |
| `2` | 将控制源切换到外部 PWM 输入，同时撤销串口命令使能 |

Policy=2 回退的是**控制权**，不是一个保存的 PWM 占空比。Fail-safe Policy 本身保存于 NVM；串口目标、使能状态、最近 PWM 输入和多圈计数均不保存。地址 `126` 只能观察 RAM 中最近捕获的 PWM 低电平宽度，没有独立的“回退 PWM 值”存储位置。

外部 PWM 当前按低电平有效的单圈位置命令解释：`1000 µs -> 0 count`，`2000 µs -> 16383 count`，中间线性插值。`900..2100 µs` 视为可接受输入，命令值在映射前钳位到 `1000..2000 µs`；超过 `100 ms` 没有完整有效脉冲时立即撤销 PWM 命令使能。有效外部 PWM 是独立的物理使能来源，因此 Policy=2 超时切换后，只有存在有效 PWM 输入才会重新产生输出。

Policy=2 下 `Fault Code=0x000A` 仍保持锁存，Status Packet 的 Alert 位和 Status Word 的 Fault Present 位仍为 `1`，但 Output Enabled 可以同时为 `1`。这表示主站失联后由明确配置的 PWM 后备源接管，不是故障被自动清除。欠压或超温保护仍具有更高优先级，并禁止 PWM 后备输出。

## Serial Watchdog 刷新规则

Serial Watchdog 只由“成功接收、通过校验并被控制层接受的新串口控制命令”刷新：

- 地址 `16/17/18/20/22/26` 的有效控制写入；
- 地址 `16` 的有效长度 `14` 或 `20` 命令映像；
- 匹配本节点且通过全部校验的 Sync Write 控制命令。

Ping、Read、Sync Read、参数读取、配置/PID 写入、Execute Tick 或 Sequence 单字段写入、被拒绝的命令以及重复 Sequence 重传都不刷新 Watchdog。只有控制源为 Serial 且活动或待生效命令已经请求使能时才累计超时。

## Node ID 与 Baud Code 生效时序

- 单播写 Node ID 时，从站先使用旧 ID 编码并通过 UART DMA 发送 Status ACK；只有 UART Transmission Complete 回调到达后才提交新 ID。
- Reg Write Node ID 只登记数据；单播 Action 仍先用旧 ID ACK，发送完成后再提交新 ID。
- 广播 Write/Sync Write 没有 ACK，匹配节点会立即更新 Node ID，因此主站必须避免多个节点同时写成相同 ID。
- 单播写 Baud Code 时，Status ACK 完整使用旧波特率发送；UART TC 到达后，主循环暂停响应发送、重初始化 UART 并重启 Receive-to-Idle DMA，然后才提交新波特率。写入只改变运行配置，仍需 Save NVM 才能跨复位保留。
