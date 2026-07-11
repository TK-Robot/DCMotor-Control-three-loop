# 03 State Machine and Topology Discovery

## 3.1 Slave Network States

```text
BOOT -> INIT -> PREOP -> SAFEOP -> OP
  |       |       |        |      |
  +-------+-------+--------+-------> FAULT
```

| State | Description |
| --- | --- |
| `BOOT` | Power-on state, PWM default, serial output disabled |
| `INIT` | UART ready, waiting for discovery, enumeration, or basic SDO |
| `PREOP` | SDO configuration for parameters, PDO mapping, and control source |
| `SAFEOP` | PDO locked, sync and feedback allowed, output not automatically enabled |
| `OP` | Real-time PDO active, serial control may drive the motor |
| `FAULT` | Protection or protocol fault, output follows FailSafe policy |

## 3.2 State Rules

- `BOOT` defaults to `ControlSource = PWM`.
- `INIT` accepts only discovery, enumeration, identity read, and transition to `PREOP`.
- `PREOP` allows object dictionary writes and PDO mapping, but PDO shall not drive output.
- `SAFEOP` means configuration is locked. The master may start synchronization and feedback.
- `OP` may drive the motor by serial bus only when `ControlSource = SERIAL_PDO` and the PDO watchdog is healthy, or when `ControlSource = SERIAL_SDO` and the serial-command watchdog is healthy.
- Any protection fault enters `FAULT`; clearing the fault shall restart from `INIT` or `PREOP`.
- When switching from `PWM` to `SERIAL_PDO` or `SERIAL_SDO`, the master shall write `Enable = 0` first and enable only after state confirmation.

## 3.3 State Transition Commands

State transitions are performed through SDO writes to object `0x1200 NetworkControl`.

| Value | Command | Description |
| --- | --- | --- |
| `0x01` | `CMD_ENTER_INIT` | Enter INIT |
| `0x02` | `CMD_ENTER_PREOP` | Enter PREOP |
| `0x03` | `CMD_ENTER_SAFEOP` | Enter SAFEOP |
| `0x04` | `CMD_ENTER_OP` | Enter OP |
| `0x05` | `CMD_STOP` | Stop output and return to SAFEOP |
| `0x06` | `CMD_CLEAR_FAULT` | Clear fault |
| `0x07` | `CMD_REBOOT` | Software reset, reserved |

## 3.4 Parallel Bus Discovery

Flow:

```text
Master -> Broadcast DISCOVERY
Unassigned slaves wait randomized backoff based on UID
One slave -> DISCOVERY_REPLY
Master -> ASSIGN_ID for that UID
Repeat until no more replies
```

Rules:

- Unassigned nodes use the MCU unique ID during discovery.
- Backoff time is derived from UID, discovery round, and random jitter to reduce reply collisions.
- Discovery reply shall be short and contain UID, protocol version, product code, and basic capability flags.
- After ID assignment, the node enters `INIT` or `PREOP`.
- If replies collide, the master narrows the discovery window or restarts the round.

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

## 3.5 Ring-chain Enumeration

Wiring:

```text
Master TX -> Slave1 RX
Slave1 TX -> Slave2 RX
Slave2 TX -> Slave3 RX
...
SlaveN TX -> Master RX
```

Enumeration flow:

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

Ring-chain mode does not require slaves to compete for reply slots. The frame has one direction only, and each node writes its position by pass-through order. This order is link order, not guaranteed mechanical order.

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

Each descriptor:

```text
u8  position
u8  protocol_version
u16 product_code
u32 revision
u32 uid_low
u32 uid_mid
u32 uid_high
```

Enumeration rules:

- If `node_count >= max_nodes`, the slave sets an overflow error and forwards the frame.
- Each slave updates `last_seen_position` before forwarding.
- A slave without assigned ID keeps `node_id = 0xFF` until `ASSIGN_ID`.
- Master shall reject enumeration if returned `node_count` exceeds its configured maximum.
- Master shall stop PDO during enumeration.

## 3.6 ASSIGN_ID

Parallel payload:

```text
u32 uid_low
u32 uid_mid
u32 uid_high
u8  assigned_node_id
u8  reserved[3]
```

Ring-chain payload:

```text
u8  count
struct {
    u8 position
    u8 assigned_node_id
} item[count]
```

Rules:

- Node IDs range from `0x01` to `0x7E`.
- The master shall avoid duplicate IDs.
- After re-enumeration, the master may preserve old IDs or reassign by position.
- If node count changes, the master shall reconfigure PDO mapping and cycle time.

## 3.7 Topology Change

- Parallel new node: the master performs DISCOVERY during reconnect periods without stopping existing PDO nodes.
- Parallel removed node: the node becomes offline and is removed from PDO; other nodes keep running.
- Ring-chain new or removed node: loop length changes, so the whole chain shall exit OP, re-enumerate, and reconfigure.

## 3.8 Full Initialization Flow

TSBP initialization follows one rule: the master does the complex decisions, while the slave stores compiled results. Topology discovery, object reads, PDO mapping, and bus-load calculation are completed before OP. OP cycles do not repeat this work.

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

Slave-side state logic:

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

## 3.9 Master Initialization Pseudocode

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
