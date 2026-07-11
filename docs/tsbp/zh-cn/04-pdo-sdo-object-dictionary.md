# 04 PDO、SDO 与对象字典

## 4.1 PDO 与 SDO 分工

PDO 是周期实时数据，只传控制目标和反馈状态。SDO 是非周期参数访问，只用于配置、诊断、保存、恢复和 PDO 映射。

规则：

- PDO 不带 index/subindex，按初始化时锁定的映射表打包。
- SDO 每次访问一个对象字典项。
- PDO 映射只能在 `PREOP` 修改。
- 进入 `SAFEOP` 后 PDO 映射锁定。
- 进入 `OP` 后 PDO 周期开始驱动或反馈过程数据。

## 4.2 数据类型

| Type | 大小 | 编码 |
| --- | --- | --- |
| `u8` | 1 | 无符号 |
| `i8` | 1 | 有符号补码 |
| `u16` | 2 | 小端 |
| `i16` | 2 | 小端 |
| `u32` | 4 | 小端 |
| `i32` | 4 | 小端 |
| `bool` | 1 | `0 = false`，非零为 true |

第一版 PDO/SDO 对象不使用浮点数。

## 4.3 PDO 映射项

每个 PDO 映射项固定 4 字节：

```text
u16 index
u8  subindex
u8  type
```

类型值：

```text
0x01 = u8
0x02 = i8
0x03 = u16
0x04 = i16
0x05 = u32
0x06 = i32
0x07 = bool
```

限制：

```text
max_rx_pdo_items = 8
max_tx_pdo_items = 8
recommended_node_slot_bytes <= 24
recommended_total_pdo_frame_bytes <= 192
```

## 4.4 PDO 配置与编译逻辑

PDO 映射配置分为“写映射表”和“编译映射表”两步。写映射表仍然使用对象字典格式，便于主站工具理解；编译后从站只保留运行期需要的偏移、长度和直接访问方式。

```text
Master in PREOP
  |
  v
Write 0x1600 RX PDO mapping items
Write 0x1A00 TX PDO mapping items
  |
  v
Write PDO_COMPILE command
  |
  v
Slave validates every mapping item
  |
  +-- invalid type/access/range --> reject, stay PREOP
  |
  v
Slave builds compact runtime layout
  |
  v
Master reads RxPdoByteSize / TxPdoByteSize
  |
  v
Master calculates bus load
  |
  v
Enter SAFEOP and lock mapping
```

从站编译后的运行期结构建议：

```text
PdoRuntime
  u8  rx_len
  u8  tx_len
  u8  node_stride
  u8  position
  u16 rx_offset
  u16 tx_offset
  PdoCopyItem rx_items[MAX_RX_PDO_ITEMS]
  PdoCopyItem tx_items[MAX_TX_PDO_ITEMS]
```

`PdoCopyItem` 不保存字符串，也不保存 XML 信息：

```text
PdoCopyItem
  u8  pdo_offset
  u8  size
  u16 local_offset_or_id
```

第一版从站可以把 `local_offset_or_id` 实现为本地结构体偏移或少量枚举 ID。不要在 OP 周期做对象字典搜索。

映射编译结果示意：

```text
Object dictionary mapping:
  0x2002:00 ControlWord       -> RX offset 0, size 2
  0x2001:00 ServoMode         -> RX offset 2, size 1
  0x2003:00 TargetCurrent_mA  -> RX offset 3, size 2

Compiled RX copy table:
  {pdo_offset=0, size=2, local=CONTROL_WORD}
  {pdo_offset=2, size=1, local=SERVO_MODE}
  {pdo_offset=3, size=2, local=TARGET_CURRENT}
```

## 4.5 默认 RX PDO

RX PDO 由主站写入、从站消费。

```text
u16 control_word
u8  mode
i16 target_current_mA
i32 target_speed
i32 target_position
```

| Offset | Object | Type | 说明 |
| --- | --- | --- | --- |
| 0 | `0x2002:00 ControlWord` | `u16` | 控制字 |
| 2 | `0x2001:00 ServoMode` | `u8` | 电流/速度/位置模式 |
| 3 | `0x2003:00 TargetCurrent_mA` | `i16` | 电流目标 |
| 5 | `0x2004:00 TargetSpeed` | `i32` | 速度目标 |
| 9 | `0x2005:00 TargetPosition` | `i32` | 位置目标 |

`control_word` 位定义：

```text
bit0 enable
bit1 quick_stop
bit2 clear_fault
bit3 hold_position
bit4 use_target_current
bit5 use_target_speed
bit6 use_target_position
bit7 reserved
bit8..15 user/reserved
```

规则：

- `enable = 0` 时从站不得输出驱动。
- `quick_stop = 1` 时从站应按故障策略快速停止。
- `clear_fault = 1` 只请求清故障，不应自动使能。
- `use_target_*` 用于明确本周期哪些目标字段有效。

## 4.6 默认 TX PDO

TX PDO 由从站写入、主站消费。

```text
u16 status_word
u16 fault_code
i16 current_mA
i32 speed
i32 position
u16 vcc_mV
i8  temp_c
```

| Offset | Object | Type | 说明 |
| --- | --- | --- | --- |
| 0 | `0x2100:00 StatusWord` | `u16` | 状态字 |
| 2 | `0x2104:00 FaultCode` | `u16` | 故障码 |
| 4 | `0x2101:00 ActualCurrent_mA` | `i16` | 实测电流 |
| 6 | `0x2102:00 ActualSpeed` | `i32` | 实测速度 |
| 10 | `0x2103:00 ActualPosition` | `i32` | 实测位置 |
| 14 | `0x2601:00 Vcc_mV` | `u16` | 电源电压 |
| 16 | `0x2602:00 Temperature_C` | `i8` | MCU 温度 |

`status_word` 位定义：

```text
bit0 ready_to_switch_on
bit1 switched_on
bit2 operation_enabled
bit3 fault
bit4 voltage_enabled
bit5 quick_stop_active
bit6 switch_on_disabled
bit7 warning
bit8 control_source_pwm
bit9 control_source_serial
bit10 target_reached
bit11 pdo_watchdog_ok
bit12 encoder_ok
bit13 current_limit_active
bit14 voltage_limit_active
bit15 reserved
```

## 4.7 高密度 PDO_FAST 过程映像

实时控制默认使用 `PDO_FAST`，不是 `PDO_CYCLE`。`PDO_FAST` 的核心原则是：所有描述信息在 `PREOP` 阶段通过 SDO 配置好，OP 周期内只传过程数据。

`PDO_FAST` 不携带：

- index/subindex；
- node_id；
- position；
- rx_slot_size；
- tx_slot_size；
- node_count；
- 每节点时间戳。

这些信息全部来自主站和从站在 `SAFEOP` 前锁定的 PDO 配置。

### 链式模式滚动过程映像

链式模式推荐使用滚动过程映像，而不是固定完整过程映像。主站发送给 Slave1 的命令数据最多；每个从站从剩余命令区最前面消费自己的 RX PDO，继续转发后级节点命令，然后把自己的 TX PDO 追加到反馈区。

```text
Master -> Slave1:
  RX1 RX2 RX3 ... RXN

Slave1 -> Slave2:
  RX2 RX3 ... RXN TX1

Slave2 -> Slave3:
  RX3 ... RXN TX1 TX2

SlaveN -> Master:
  TX1 TX2 ... TXN
```

每经过一个从站，命令区递减，反馈区递增。总帧长取决于 `rx_slot_size` 和 `tx_slot_size`；只有两者相等时，总帧长才保持不变。

每个从站根据锁定的拓扑和 slot 长度计算期望输入/输出过程数据长度：

```text
remaining_rx_count = node_count - position + 1
collected_tx_count = position - 1

in_process_len  = remaining_rx_count * rx_slot_size
                + collected_tx_count * tx_slot_size

out_process_len = (remaining_rx_count - 1) * rx_slot_size
                + (collected_tx_count + 1) * tx_slot_size

rx_offset = 0
forward_remaining_rx_offset = rx_slot_size
forward_collected_tx_offset = remaining_rx_count * rx_slot_size
append_tx_offset            = out_process_len - tx_slot_size
```

从站处理流程：

```text
read RX PDO at process offset 0
forward remaining RX commands after the consumed RX slot
forward previously collected TX feedback unchanged
append local TX PDO feedback
recompute outgoing CRC and update outgoing length expectation
```

这样从站不需要在周期帧里搜索 ID，也不需要解析对象字典，只使用固定计数和固定 slot 长度。第一版建议先使用 store-and-forward，因为从站可以先验证输入 CRC 再转发；后续可启用 cut-through 提升性能，但不能把上游 CRC 错误重新包装成合法帧。

链式 OP 快路径建议：

```text
UART/DMA receives expected inbound PDO_FAST length
  |
  v
check SOF, SEQ, length, and CRC
  |
  v
copy RX PDO at process offset 0 into command shadow
  |
  v
build outbound process image:
  remaining RX commands + collected TX feedback + local TX feedback
  |
  v
recompute outgoing CRC
  |
  v
start UART DMA forwarding
```

OP 快路径禁止事项：

- 禁止按 index/subindex 查对象字典。
- 禁止解析 XML 或字符串。
- 禁止动态内存。
- 禁止 printf。
- 禁止在周期内重新计算 PDO 映射。
- 禁止在中断里运行 PID 或复杂控制逻辑。

从站收到 `PDO_FAST` 后只更新运行期 shadow 命令，实际 PID 仍由 1 ms 控制调度统一执行。这样 UART 解析和运动控制不会互相拉长执行时间。

### 并联模式过程映像

并联模式默认使用“广播 RX 过程映像 + 分时 TX 回复”：

```text
Master PDO_FAST broadcast:
  SOF SEQ CYCLE_ID Slave1_RX_PDO Slave2_RX_PDO ... SlaveN_RX_PDO CRC16

Slave replies in configured time slot:
  SOF SEQ CYCLE_ID TX_PDO CRC16
```

每个从站根据锁定的并联过程映像 slot 计算自己的 RX offset。TX 回复由主站分配 reply slot，避免二极管汇总 TX 线上多个从站同时发送。

```text
parallel_rx_offset = parallel_slot_index * rx_slot_size
reply_delay_us     = reply_slot_index * reply_slot_us
```

`PDO_FAST_POLL` 只保留为诊断或兼容用途。如果实现该帧，必须在 `RX_PDO` 前增加被寻址的 `node_id` 字段；否则并联总线上所有从站都会看到同一个 poll，并可能同时回复。

### 时间同步字段

`PDO_FAST` 只携带 `CYCLE_ID`，不在每帧携带完整 `master_time_us`。完整时间通过低频 `SYNC` 管理帧校准。

推荐：

```text
SYNC period      = 10 ms 或 100 ms
PDO_FAST period  = 1 ms / 2 ms / 5 ms
```

如果需要更高同步精度，可以把 `apply_cycle_offset` 固化在对象字典中，而不是每个 PDO 周期重复传输。

### PDO_CYCLE 的用途

`PDO_CYCLE` 保留为调试、兼容和拓扑诊断帧。它可以携带 node_id、slot_size、last_seen_position 和时间戳，但不作为高性能 OP 周期的默认选择。

## 4.8 SDO 帧 Payload

`SDO_READ`：

```text
u16 index
u8  subindex
u8  reserved
```

`SDO_WRITE`：

```text
u16 index
u8  subindex
u8  type
u8  size
u8  data[8]
```

`SDO_REPLY`：

```text
u16 index
u8  subindex
u8  type
u8  size
u8  status
u8  data[8]
```

`status`：

```text
0x00 = OK
0x01 = object not found
0x02 = access denied
0x03 = type mismatch
0x04 = range error
0x05 = state error
0x06 = busy
```

第一版仅支持 expedited SDO，单次数据最大 8 字节。分段传输只预留，不在 STM32G030 第一版固件中实现。

## 4.9 对象字典分区

| 范围 | 名称 | 用途 |
| --- | --- | --- |
| `0x1000` | Identity | 设备信息、协议版本、UID |
| `0x1100` | Communication | 节点 ID、拓扑、波特率、看门狗 |
| `0x1200` | Network State | 状态切换、故障状态 |
| `0x1300` | Distributed Clock | 软件分布式时钟 |
| `0x1600` | RX PDO Mapping | RX PDO 映射表 |
| `0x1A00` | TX PDO Mapping | TX PDO 映射表 |
| `0x2000` | Servo Command | 控制源、模式、目标值 |
| `0x2100` | Servo Status | 状态、电流、速度、位置 |
| `0x2200` | Current Loop PID | 电流环 PID |
| `0x2300` | Speed Loop PID | 速度环 PID |
| `0x2400` | Position Loop PID | 位置环 PID |
| `0x2500` | PWM Input Control | PWM 输入控制 |
| `0x2600` | Limits and Protection | 限幅、温度、电压、FailSafe |
| `0x2700` | Encoder | 编码器参数与反馈 |
| `0x2800` | NVM Save/Restore | 参数保存、加载、擦除 |

## 4.10 关键对象

### 通信周期与同步

| Index | Name | Type | Access | 说明 |
| --- | --- | --- | --- | --- |
| `0x1103:00` | SerialWatchdogMs | `u16` | rw | 串口控制源通信看门狗 |
| `0x1106:00` | PdoCycleTimeUs | `u32` | rw-preop | `PDO_FAST` 固定周期 |
| `0x1107:00` | PdoJitterLimitUs | `u16` | rw-preop | 主站允许的周期抖动 |
| `0x1108:00` | PdoApplyCycleOffset | `u8` | rw-preop | PDO 目标延后几个周期生效 |
| `0x1300:00` | DcEnable | `u8` | rw | 软件分布式时钟使能 |
| `0x1302:00` | MasterOffsetUs | `i32` | ro | 估算出的软件时间偏移 |
| `0x1304:00` | LastAppliedCycle | `u16` | ro | 控制调度器最后应用的总线周期 |
| `0x1305:00` | PathDelayUs | `u16` | rw-preop | 主站到本节点的估计链路延迟 |

这些对象在 `PREOP` 配置并在 `SAFEOP` 锁定。进入 `OP` 后，主站必须按 `PdoCycleTimeUs` 固定周期发送 `PDO_FAST`。如果要改变周期或 apply offset，必须退出 OP。

### 伺服命令

| Index | Name | Type | Access | 当前固件关系 |
| --- | --- | --- | --- | --- |
| `0x2000:00` | ControlSource | `u8` | rw | PWM/SERIAL_PDO/SERIAL_SDO 选择 |
| `0x2001:00` | ServoMode | `u8` | rw/pdo | `ServoCommand.mode` |
| `0x2002:00` | ControlWord | `u16` | rw/pdo | enable、quick stop |
| `0x2003:00` | TargetCurrent_mA | `i16` | rw/pdo | `ServoCommand.target_current_mA` |
| `0x2004:00` | TargetSpeed | `i32` | rw/pdo | `ServoCommand.target_speed` |
| `0x2005:00` | TargetPosition | `i32` | rw/pdo | `ServoCommand.target_position` |

`ControlSource`：

```text
0 = disabled
1 = serial_pdo
2 = pwm_input
3 = serial_sdo
```

`ControlSource` 不得保存到 NVM。上电和 `BOOT` 必须强制为 `pwm_input`，除非主站在当前会话中显式切换到 `serial_pdo` 或 `serial_sdo`。

`serial_sdo` 控制源用于低频调试、简单主站和参数整定。主站通过 expedited `SDO_WRITE` 写 `0x2001` 到 `0x2005` 更新 `ServoCommand` shadow；从站在 1 ms 控制调度边界应用最新 SDO 命令，不提供 PDO 固定周期同步。

`serial_sdo` 规则：

- 只支持 expedited SDO，不支持分段 SDO 控制命令。
- 只有 `OP` 状态允许 SDO 命令驱动输出；`PREOP/SAFEOP` 可写目标 shadow，但 `Enable` 不得生效。
- `ControlWord.enable = 0` 必须关闭输出。
- 每次有效 SDO 写命令对象后刷新 `last_serial_command_time`。
- 超过 `SerialWatchdogMs` 未收到新的有效 SDO 控制写入时，执行 `FailSafePolicy`。
- `serial_sdo` 不参与 `PdoCycleTimeUs`、`PdoApplyCycleOffset` 和分布式时钟同步。

### 伺服状态

| Index | Name | Type | Access | 当前固件关系 |
| --- | --- | --- | --- | --- |
| `0x2100:00` | StatusWord | `u16` | ro/pdo | 由状态生成 |
| `0x2101:00` | ActualCurrent_mA | `i16` | ro/pdo | `Param.INA181_mA` |
| `0x2102:00` | ActualSpeed | `i32` | ro/pdo | `Param.EncoderSpeed` |
| `0x2103:00` | ActualPosition | `i32` | ro/pdo | `Param.EncoderMultiTurnValue` |
| `0x2104:00` | FaultCode | `u16` | ro/pdo | 保护/协议故障 |
| `0x2105:00` | DrivePower | `i16` | ro/pdo | `Param.DrivePower` |

### PID 对象

`0x2200`、`0x2300`、`0x2400` 分别表示电流环、速度环、位置环。每组对象：

```text
sub 0x01 Kp:u16
sub 0x02 Ki:u16
sub 0x03 Kd:u16
sub 0x04 IntegralMax:i32
sub 0x05 OutMax:u16
sub 0x06 OutMin:u16
sub 0x07 Reset:wo u8
```

### PWM 输入控制

| Index | Name | Type | Access | Default |
| --- | --- | --- | --- | --- |
| `0x2500:00` | PwmMode | `u8` | rw | speed |
| `0x2501:00` | PwmMinUs | `u16` | rw | `1000` |
| `0x2502:00` | PwmMidUs | `u16` | rw | `1500` |
| `0x2503:00` | PwmMaxUs | `u16` | rw | `2000` |
| `0x2504:00` | PwmDeadbandUs | `u16` | rw | `20` |
| `0x2505:00` | PwmTimeoutMs | `u16` | rw | `50` |
| `0x2506:00` | PwmRawCapture | `u16` | ro | `Param.DutyRatio` |

### NVM 保存

| Index | Name | Type | Access | 说明 |
| --- | --- | --- | --- | --- |
| `0x2800:00` | NvmCommand | `u8` | wo | save/load/erase |
| `0x2801:00` | NvmStatus | `u8` | ro | idle/busy/ok/error |
| `0x2802:00` | NvmSequence | `u32` | ro | 最新保存序号 |

运行目标值、当前模式目标和 `Enable` 不应掉电保存，避免上电自动动作。PID、限幅、方向、保护阈值、PDO 默认映射等配置项可以保存。
