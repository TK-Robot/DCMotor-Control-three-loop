# 03 状态机与拓扑发现

## 3.1 从站网络状态

```text
BOOT -> INIT -> PREOP -> SAFEOP -> OP
  |       |       |        |      |
  +-------+-------+--------+-------> FAULT
```

| 状态 | 说明 |
| --- | --- |
| `BOOT` | 上电初始状态，默认 PWM 控制，串口不驱动电机 |
| `INIT` | 串口已准备，等待发现、枚举或基础 SDO |
| `PREOP` | 允许 SDO 配置参数、PDO 映射和控制源 |
| `SAFEOP` | PDO 映射锁定，允许同步和反馈，但不允许自动输出 |
| `OP` | 实时 PDO 生效，串口控制可驱动电机 |
| `FAULT` | 保护或协议故障，输出按 FailSafe 策略处理 |

## 3.2 状态规则

- `BOOT` 默认 `ControlSource = PWM`。
- `INIT` 只允许发现、枚举、读取身份信息和进入 `PREOP`。
- `PREOP` 允许写对象字典和 PDO 映射，但不允许 PDO 驱动输出。
- `SAFEOP` 表示配置已锁定，主站可以开始同步和读取反馈。
- `OP` 只有在 `ControlSource = SERIAL_PDO` 且 PDO watchdog 正常，或 `ControlSource = SERIAL_SDO` 且串口命令 watchdog 正常时，才能用串口驱动电机。
- 任意保护故障进入 `FAULT`，必须清故障后重新进入 `INIT` 或 `PREOP`。
- 从 `PWM` 切到 `SERIAL_PDO` 或 `SERIAL_SDO` 时，主站必须先写 `Enable = 0`，确认状态后再使能。

## 3.3 状态切换命令

状态切换通过写 SDO 对象 `0x1200 NetworkControl` 完成。

| 值 | 命令 | 说明 |
| --- | --- | --- |
| `0x01` | `CMD_ENTER_INIT` | 进入 INIT |
| `0x02` | `CMD_ENTER_PREOP` | 进入 PREOP |
| `0x03` | `CMD_ENTER_SAFEOP` | 进入 SAFEOP |
| `0x04` | `CMD_ENTER_OP` | 进入 OP |
| `0x05` | `CMD_STOP` | 停止输出并回 SAFEOP |
| `0x06` | `CMD_CLEAR_FAULT` | 清故障 |
| `0x07` | `CMD_REBOOT` | 软件复位，预留 |

## 3.4 并联总线发现

流程：

```text
Master -> Broadcast DISCOVERY
Unassigned slaves wait randomized backoff based on UID
One slave -> DISCOVERY_REPLY
Master -> ASSIGN_ID for that UID
Repeat until no more replies
```

规则：

- 未分配节点使用 MCU 唯一 ID 参与发现。
- 退避时间由 UID、轮次和随机扰动计算，降低多个节点同时回复概率。
- 发现响应必须短，包含 UID、协议版本、产品码和基础能力。
- 主站分配 ID 后，节点进入 `INIT` 或 `PREOP`。
- 如果发生回复冲突，主站缩小发现窗口或重新开始该轮。

`DISCOVERY` payload:

```text
u8  discovery_round
u8  min_backoff_slot
u8  max_backoff_slot
u8  requested_product_class
u32 master_time_us
```

`DISCOVERY_REPLY` payload:

```text
u8  protocol_version
u8  capability_flags
u16 product_code
u32 revision
u32 uid_low
u32 uid_mid
u32 uid_high
```

## 3.5 链式枚举

接线：

```text
Master TX -> Slave1 RX
Slave1 TX -> Slave2 RX
Slave2 TX -> Slave3 RX
...
SlaveN TX -> Master RX
```

枚举流程：

```text
Master sends ENUM_START with node_count = 0
Slave1 receives frame, position = node_count + 1, node_count++
Slave1 appends or updates its node descriptor
Slave1 forwards frame to Slave2
...
SlaveN forwards the frame back to Master
Master receives ENUM_REPLY and knows chain order
Master sends ASSIGN_ID by position
```

链式模式不需要从站抢答。帧只有一个方向，节点按经过顺序写入自己的 position。这个顺序是链路顺序，不承诺机械安装顺序。

`ENUM_START` payload:

```text
u8  topology = 1
u8  node_count
u8  max_nodes
u8  descriptor_size
u16 last_seen_position
u16 reserved
descriptor[max_nodes]
```

每个 descriptor：

```text
u8  position
u8  protocol_version
u16 product_code
u32 revision
u32 uid_low
u32 uid_mid
u32 uid_high
```

枚举规则：

- 如果 `node_count >= max_nodes`，从站设置 overflow 错误并继续转发。
- 每个从站转发前更新 `last_seen_position`。
- 未分配 ID 的从站保持 `node_id = 0xFF`，直到收到 `ASSIGN_ID`。
- 主站如果发现返回的 `node_count` 超过配置上限，应拒绝枚举结果。
- 枚举期间主站必须停止 PDO。

## 3.6 节点 ID 分配

并联 payload：

```text
u32 uid_low
u32 uid_mid
u32 uid_high
u8  assigned_node_id
u8  reserved[3]
```

链式 payload：

```text
u8  count
struct {
    u8 position
    u8 assigned_node_id
} item[count]
```

规则：

- 节点 ID 范围为 `0x01` 到 `0x7E`。
- 主站必须避免重复 ID。
- 重新枚举后，主站可以保留旧 ID，也可以按 position 重新分配。
- 如果节点数量变化，主站必须重新配置 PDO 映射和通信周期。

## 3.7 拓扑变化

- 并联模式新增节点：主站在重连周期内执行 DISCOVERY，不影响已在线节点 PDO。
- 并联模式移除节点：该节点 offline，PDO 移除，其它节点继续运行。
- 链式模式新增或移除节点：回环长度变化，整链必须退出 OP，重新 ENUM 和配置。

## 3.8 初始化配置总流程

TSBP 的初始化原则是：主站承担复杂决策，从站只保存编译后的结果。拓扑发现、对象读取、PDO 映射、总线负载计算都在进入 OP 前完成，OP 周期不再做这些工作。

```text
Power on
  |
  v
Slave BOOT: default PWM control, serial output disabled
  |
  v
Master selects topology
  |
  +-- Parallel bus --------------------------+
  |                                          |
  |  DISCOVERY -> ASSIGN_ID -> node table    |
  |                                          |
  +-- Ring-chain ----------------------------+
                                             |
     ENUM_START -> returned ENUM -> ASSIGN_ID
                                             |
                                             v
Master reads identity/capability objects by SDO
  |
  v
Master enters PREOP
  |
  v
Master writes communication, FailSafe, control-source, and PDO mapping objects
  |
  v
Slave compiles PDO mapping into fixed offsets and fixed lengths
  |
  v
Master calculates bus load and selects PDO_FAST period
  |
  v
Master enters SAFEOP
  |
  v
SYNC calibration, Enable remains 0
  |
  v
Master enters OP
  |
  v
PDO_FAST cyclic control
```

从站状态侧逻辑：

```text
BOOT
  - output disabled for serial control
  - PWM remains the default control source
  - accept DISCOVERY / ENUM / basic SDO only

INIT
  - node_id may be assigned
  - identity objects readable
  - no PDO output

PREOP
  - accept SDO configuration
  - accept PDO mapping writes
  - compile mapping when requested

SAFEOP
  - PDO mapping is locked
  - PDO_FAST length and offsets are fixed
  - sync is allowed
  - output still not enabled by default

OP
  - only fixed-offset PDO_FAST process data is handled in the fast path
  - SDO is allowed only if it does not change locked PDO timing
```

## 3.9 主站初始化伪代码

```text
load XML-like slave profile
open UART at default baud

if topology == parallel:
    repeat DISCOVERY until no unassigned node replies
    assign node IDs by UID
else if topology == ring-chain:
    send ENUM_START
    receive returned ENUM
    assign node IDs by position

for each node:
    read identity objects
    read capability objects
    command ENTER_PREOP
    write communication objects
    write FailSafe policy
    write ControlSource = PWM first
    write RX PDO mapping
    write TX PDO mapping
    write PDO compile/lock request
    read compiled RX/TX byte sizes

calculate PDO_FAST frame size and bus usage
if bus usage >= 80%:
    reject OP and reduce PDO mapping or cycle rate

for each node:
    command ENTER_SAFEOP

send several SYNC frames

for each node:
    write ControlSource = SERIAL_PDO or SERIAL_SDO if serial control is required
    write Enable = 0
    command ENTER_OP

start PDO_FAST cycle
```
