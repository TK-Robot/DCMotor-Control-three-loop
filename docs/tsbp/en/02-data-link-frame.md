# 02 Data Link and Frame Format

## 2.1 Binary Frame

All multi-byte fields are little-endian.

```text
Offset  Size  Field    Description
0       1     SOF0     0xA5
1       1     SOF1     0x5A
2       1     VER      Protocol version, 0x01 for this draft
3       1     TYPE     Frame type
4       1     FLAGS    Topology, ACK policy, segmented flag, error policy
5       1     SEQ      Sequence number
6       1     DST      Destination node ID
7       1     SRC      Source node ID
8       1     LEN_L    Payload length low byte
9       1     LEN_H    Payload length high byte
10      N     PAYLOAD  Frame payload
10+N    2     CRC16    CRC16-CCITT over VER..PAYLOAD
```

Minimum frame length is 12 bytes. Maximum payload length for the first firmware generation should be limited by configured RX buffer size. The protocol reserves up to 256 bytes payload, but STM32G030 firmware should use smaller limits.

## 2.2 Dual-frame Model

TSBP uses two frame classes instead of forcing all traffic through one full header:

```text
Management frames: DISCOVERY / ENUM / ASSIGN_ID / SDO / ERROR
Fast frames: PDO_FAST real-time process data in OP
```

Management frames prioritize clarity and diagnostics, so they keep `VER/TYPE/FLAGS/DST/SRC/LEN`. Fast frames are used only after PDO mapping, node order, and slot sizes are locked after `SAFEOP`; they prioritize density and parsing speed.

`PDO_FAST` frame format:

```text
Offset  Size  Field
0       1     SOF      0xA6
1       1     SEQ
2       2     CYCLE_ID
4       N     PROCESS_IMAGE
4+N     2     CRC16
```

Fast-frame rules:

- `PDO_FAST` has no `TYPE`, `DST`, `SRC`, or `LEN`; length is determined by the PDO configuration made in `PREOP`.
- In parallel mode, the broadcast/reply lengths are fixed by the locked parallel PDO slots.
- In ring-chain rolling-image mode, each slave computes its expected inbound and outbound lengths from `node_count`, `position`, `rx_slot_size`, and `tx_slot_size`.
- `PROCESS_IMAGE` is pure process data. It does not carry index/subindex, node_id, slot_size, or position.
- CRC16 covers `SEQ`, `CYCLE_ID`, and `PROCESS_IMAGE`.
- Fast frames are allowed only in `OP`; non-OP slaves shall drop them.
- If the master needs to change PDO mapping or node count, it shall leave OP and reconfigure in `PREOP`.

For example, with 4 ring-chain nodes, 13-byte RX PDO and 17-byte TX PDO per node, the first segment carries `4 * 13 = 52` process bytes and the final return segment carries `4 * 17 = 68` process bytes. Intermediate segments carry fewer RX commands and more TX feedback.

## 2.3 Addressing

| Value | Meaning |
| --- | --- |
| `0x00` | Master |
| `0x01` - `0x7E` | Assigned slave nodes |
| `0x7F` | Broadcast |
| `0x80` - `0xFE` | Reserved |
| `0xFF` | Unassigned slave |

Rules:

- `DST = 0x7F` is broadcast.
- `SRC = 0x00` is the master.
- `SRC = 0xFF` is allowed only during discovery or enumeration.
- Slaves shall ignore frames with unsupported `VER`.

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
  Used only by management/diagnostic frames to indicate that the referenced PDO mapping is locked.

bit6..7 reserved
```

`FLAGS` shall not carry motion-control state. Motion-control state belongs in PDO data or the object dictionary.

## 2.5 Frame Types

| TYPE | Name | Direction | Purpose |
| --- | --- | --- | --- |
| `0x01` | `SYNC` | Master to slave(s) | Clock synchronization without process data |
| `0x02` | `PDO_CYCLE` | Master to slave(s) to master | Cyclic process data |
| `0x10` | `SDO_READ` | Master to slave | Read object dictionary |
| `0x11` | `SDO_WRITE` | Master to slave | Write object dictionary |
| `0x12` | `SDO_REPLY` | Slave to master | SDO response |
| `0x20` | `DISCOVERY` | Master to broadcast | Parallel-bus discovery |
| `0x21` | `DISCOVERY_REPLY` | Slave to master | Discovery response |
| `0x22` | `ENUM_START` | Master to chain | Ring-chain enumeration |
| `0x23` | `ENUM_REPLY` | Chain to master | Returned enumeration frame |
| `0x24` | `ASSIGN_ID` | Master to slave(s) | Assign node IDs |
| `0x30` | `PING` | Master to slave | Low-rate reconnect check |
| `0x31` | `HEARTBEAT` | Slave to master | Optional non-PDO alive state |
| `0x7F` | `ERROR` | Slave to master | Protocol or state error |

`PDO_FAST` is not a `TYPE = 0x03` management frame. It is selected by the separate `SOF = 0xA6` fast-frame marker. Implementations should dispatch by SOF first:

```text
SOF A5 5A -> management frame, parse VER/TYPE/DST/SRC/LEN
SOF A6    -> PDO_FAST frame, length from locked PDO runtime config
```

## 2.6 CRC

CRC type: CRC16-CCITT-FALSE.

```text
poly   = 0x1021
init   = 0xFFFF
xorout = 0x0000
refin  = false
refout = false
range  = VER through last payload byte
```

Rules:

- Management-frame CRC excludes `SOF0` and `SOF1`; its range is `VER` through the last payload byte.
- `PDO_FAST` CRC excludes `SOF`; its range is `SEQ`, `CYCLE_ID`, and `PROCESS_IMAGE`.
- A receiver shall not update control targets when CRC is invalid.
- PDO CRC errors shall increment miss/error counters on both master and slave.
- In ring-chain cut-through mode, a slave shall recompute outgoing-frame CRC after modifying its PDO_TX slot and overwrite the frame-tail CRC bytes.
- In ring-chain store-and-forward compatibility mode, a slave shall also recompute the full-frame CRC after modifying its PDO_TX slot.
- The first ring-chain implementation should use store-and-forward so incoming CRC can be checked before forwarding. Cut-through shall define how it prevents upstream CRC errors from being hidden by a recomputed outgoing CRC.

## 2.7 Sequence Number

`SEQ` identifies duplicate frames, dropped frames, and ring-chain loopback:

- The master increments `SEQ` for each transmitted frame.
- A slave reply copies the request `SEQ`.
- A ring-chain `PDO_FAST` or diagnostic `PDO_CYCLE` shall return to the master with the same `SEQ`.
- A slave receiving a duplicate `PDO_FAST` / `PDO_CYCLE` shall not apply the target twice. It may forward or report feedback again.

## 2.8 Generic Error Payload

`ERROR` payload:

```text
u8  error_code
u8  detail
u16 object_index
u8  object_subindex
u8  state
u16 reserved
```

| Code | Name | Meaning |
| --- | --- | --- |
| `0x01` | `ERR_CRC` | CRC error |
| `0x02` | `ERR_UNSUPPORTED_VERSION` | Unsupported version |
| `0x03` | `ERR_UNSUPPORTED_TYPE` | Unsupported frame type |
| `0x04` | `ERR_BAD_LENGTH` | Bad length |
| `0x05` | `ERR_BAD_STATE` | State does not allow the request |
| `0x06` | `ERR_ACCESS` | Object access error |
| `0x07` | `ERR_RANGE` | Parameter out of range |
| `0x08` | `ERR_BUSY` | Slave busy |
| `0x09` | `ERR_PDO_MAP` | PDO mapping error |
| `0x0A` | `ERR_WATCHDOG` | Communication timeout |

## 2.9 Timing Notes

For UART 8N1, one byte takes about 10 bit times:

```text
byte_time_us = 10 * 1000000 / baud
frame_time_us = frame_bytes * byte_time_us
```

The master shall include frame transmission time, slave forwarding delay, and guard time in the cycle budget.
