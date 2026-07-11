# 02 数据链路层与帧格式

## 2.1 二进制帧

所有多字节字段均使用小端格式。

```text
Offset  Size  Field    Description
0       1     SOF0     0xA5
1       1     SOF1     0x5A
2       1     VER      协议版本，当前草案为 0x01
3       1     TYPE     帧类型
4       1     FLAGS    拓扑、ACK、分段和错误策略
5       1     SEQ      序号
6       1     DST      目标节点 ID
7       1     SRC      源节点 ID
8       1     LEN_L    Payload 长度低字节
9       1     LEN_H    Payload 长度高字节
10      N     PAYLOAD  帧负载
10+N    2     CRC16    从 VER 到 PAYLOAD 最后字节的 CRC16-CCITT
```

最小帧长为 12 字节。第一代固件的最大 payload 应受 RX 缓冲区限制。协议预留最大 256 字节 payload，但 STM32G030 固件应使用更小的限制。

## 2.2 双帧体系

TSBP 使用两类帧，而不是所有数据都使用同一种完整帧：

```text
管理帧：DISCOVERY / ENUM / ASSIGN_ID / SDO / ERROR
快帧：OP 状态下的 PDO_FAST 实时过程数据
```

管理帧优先清晰和可诊断，保留 `VER/TYPE/FLAGS/DST/SRC/LEN` 等字段。快帧只在 `SAFEOP` 后 PDO 映射、节点顺序、slot 长度都已锁定时使用，优先信息密度和解析速度。

`PDO_FAST` 帧格式：

```text
Offset  Size  Field
0       1     SOF      0xA6
1       1     SEQ
2       2     CYCLE_ID
4       N     PROCESS_IMAGE
4+N     2     CRC16
```

快帧规则：

- `PDO_FAST` 没有 `TYPE`、`DST`、`SRC`、`LEN`，长度由 `PREOP` 阶段的 PDO 配置决定。
- 并联模式下，广播帧和回复帧长度由锁定的并联 PDO slot 决定。
- 链式滚动过程映像下，每个从站根据 `node_count`、`position`、`rx_slot_size`、`tx_slot_size` 计算自己的期望输入和输出长度。
- `PROCESS_IMAGE` 是纯过程数据，不携带 index/subindex、node_id、slot_size、position。
- CRC16 覆盖 `SEQ`、`CYCLE_ID` 和 `PROCESS_IMAGE`。
- 快帧只允许在 `OP` 状态使用；非 OP 状态收到快帧必须丢弃。
- 如果主站需要改变 PDO 映射或节点数，必须退出 OP，回到 `PREOP` 重新配置。

例如链式 4 节点、每节点 RX 13 字节、TX 17 字节时，第一段携带 `4 * 13 = 52` 字节过程数据，最后回主站的一段携带 `4 * 17 = 68` 字节过程数据。中间链路逐步减少 RX 命令、增加 TX 反馈。

## 2.3 地址

| 值 | 含义 |
| --- | --- |
| `0x00` | 主站 |
| `0x01` - `0x7E` | 已分配从站 |
| `0x7F` | 广播 |
| `0x80` - `0xFE` | 保留 |
| `0xFF` | 未分配节点 |

规则：

- `DST = 0x7F` 表示广播。
- `SRC = 0x00` 表示主站。
- `SRC = 0xFF` 只允许用于发现或枚举阶段。
- 从站必须忽略不支持的 `VER`。

## 2.4 FLAGS

```text
bit0..1 topology
  00 = parallel bus
  01 = ring-chain
  10..11 reserved

bit2 ack request
  0 = no explicit ACK
  1 = reply required when addressed

bit3 error reply enable
  0 = silent drop on error
  1 = send ERROR when allowed

bit4 segmented frame
  0 = single frame
  1 = segmented transfer, reserved for future versions

bit5 pdo locked
  仅管理/诊断帧使用，表示本帧引用的 PDO 映射已锁定

bit6..7 reserved
```

`FLAGS` 不应承载运动控制状态，只用于链路层控制。运动控制状态必须放入 PDO 或对象字典。

## 2.5 帧类型

| TYPE | 名称 | 方向 | 用途 |
| --- | --- | --- | --- |
| `0x01` | `SYNC` | 主站到从站 | 无过程数据同步 |
| `0x02` | `PDO_CYCLE` | 主站到从站再回主站 | 周期过程数据 |
| `0x10` | `SDO_READ` | 主站到从站 | 读对象字典 |
| `0x11` | `SDO_WRITE` | 主站到从站 | 写对象字典 |
| `0x12` | `SDO_REPLY` | 从站到主站 | SDO 响应 |
| `0x20` | `DISCOVERY` | 主站到广播 | 并联发现 |
| `0x21` | `DISCOVERY_REPLY` | 从站到主站 | 发现响应 |
| `0x22` | `ENUM_START` | 主站到链路 | 链式枚举 |
| `0x23` | `ENUM_REPLY` | 链路到主站 | 枚举回环 |
| `0x24` | `ASSIGN_ID` | 主站到从站 | 分配节点 ID |
| `0x30` | `PING` | 主站到从站 | 低频重连探测 |
| `0x31` | `HEARTBEAT` | 从站到主站 | 可选心跳 |
| `0x7F` | `ERROR` | 从站到主站 | 协议或状态错误 |

`PDO_FAST` 不是 `TYPE = 0x03` 的管理帧，而是由独立 `SOF = 0xA6` 选择的快帧。实现时应先按 SOF 分流：

```text
SOF A5 5A -> management frame, parse VER/TYPE/DST/SRC/LEN
SOF A6    -> PDO_FAST frame, length from locked PDO runtime config
```

## 2.6 CRC

CRC 类型：CRC16-CCITT-FALSE。

```text
poly   = 0x1021
init   = 0xFFFF
xorout = 0x0000
refin  = false
refout = false
range  = VER through last payload byte
```

规则：

- 管理帧 CRC 不覆盖 `SOF0` 和 `SOF1`，范围为 `VER` 到 payload 末尾。
- `PDO_FAST` CRC 不覆盖 `SOF`，范围为 `SEQ`、`CYCLE_ID` 和 `PROCESS_IMAGE`。
- 接收端 CRC 错误时不得更新控制目标。
- PDO CRC 错误时，主站和从站都应计入 miss/error 计数。
- 链式 cut-through 模式下，从站修改 PDO_TX slot 后必须重新计算发出帧 CRC，并覆盖帧尾 CRC 字节。
- 链式 store-and-forward 兼容模式也必须在修改 PDO_TX slot 后重新计算整帧 CRC。
- 第一版链式实现建议使用 store-and-forward，以便先检查输入 CRC 再转发。cut-through 必须先定义如何避免上游 CRC 错误被重新计算的输出 CRC 掩盖。

## 2.7 序号

`SEQ` 用于识别重复帧、丢帧和链式回环匹配：

- 主站每发一帧递增 `SEQ`。
- 从站回复时复制请求帧的 `SEQ`。
- 链式 `PDO_FAST` 或诊断 `PDO_CYCLE` 回到主站时必须保持同一个 `SEQ`。
- 从站收到重复 `PDO_FAST` / `PDO_CYCLE` 不得重复执行控制目标，只允许重复转发或重复反馈。

## 2.8 通用错误 Payload

`ERROR` payload:

```text
u8  error_code
u8  detail
u16 object_index
u8  object_subindex
u8  state
u16 reserved
```

| Code | 名称 | 含义 |
| --- | --- | --- |
| `0x01` | `ERR_CRC` | CRC 错误 |
| `0x02` | `ERR_UNSUPPORTED_VERSION` | 不支持的版本 |
| `0x03` | `ERR_UNSUPPORTED_TYPE` | 不支持的帧类型 |
| `0x04` | `ERR_BAD_LENGTH` | 长度错误 |
| `0x05` | `ERR_BAD_STATE` | 当前状态不允许 |
| `0x06` | `ERR_ACCESS` | 对象访问权限错误 |
| `0x07` | `ERR_RANGE` | 参数超范围 |
| `0x08` | `ERR_BUSY` | 从站忙 |
| `0x09` | `ERR_PDO_MAP` | PDO 映射错误 |
| `0x0A` | `ERR_WATCHDOG` | 通信超时 |

## 2.9 时序说明

UART 8N1 下每字节约等于 10 bit 传输时间：

```text
byte_time_us = 10 * 1000000 / baud
frame_time_us = frame_bytes * byte_time_us
```

主站必须把帧传输时间、从站转发延迟和保护间隔计入周期预算。
