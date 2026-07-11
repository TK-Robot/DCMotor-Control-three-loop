# 04 PDO, SDO, and Object Dictionary

## 4.1 PDO and SDO Separation

PDO is cyclic real-time data for control targets and feedback status. SDO is acyclic parameter access for configuration, diagnostics, save/restore, and PDO mapping.

Rules:

- PDO does not carry index/subindex fields. It is packed according to mappings locked during initialization.
- SDO accesses one object dictionary entry per request.
- PDO mapping may be modified only in `PREOP`.
- PDO mapping is locked after entering `SAFEOP`.
- PDO cycles drive or report process data after entering `OP`.

## 4.2 Data Types

| Type | Size | Encoding |
| --- | --- | --- |
| `u8` | 1 | Unsigned |
| `i8` | 1 | Signed two's complement |
| `u16` | 2 | Little-endian |
| `i16` | 2 | Little-endian |
| `u32` | 4 | Little-endian |
| `i32` | 4 | Little-endian |
| `bool` | 1 | `0 = false`, non-zero = true |

Floating-point values are not used in v1 PDO/SDO objects.

## 4.3 PDO Mapping Item

Each PDO mapping item is 4 bytes:

```text
u16 index
u8  subindex
u8  type
```

Type values:

```text
0x01 = u8
0x02 = i8
0x03 = u16
0x04 = i16
0x05 = u32
0x06 = i32
0x07 = bool
```

Limits:

```text
max_rx_pdo_items = 8
max_tx_pdo_items = 8
recommended_node_slot_bytes <= 24
recommended_total_pdo_frame_bytes <= 192
```

## 4.4 PDO Configuration and Compilation Logic

PDO mapping uses two steps: write the mapping table, then compile the mapping table. The written mapping stays object-dictionary based so master tools can understand it. After compilation, the slave stores only offsets, lengths, and direct access descriptors needed at runtime.

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

Recommended compiled runtime structure:

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

`PdoCopyItem` stores no strings and no XML data:

```text
PdoCopyItem
  u8  pdo_offset
  u8  size
  u16 local_offset_or_id
```

The first slave implementation may represent `local_offset_or_id` as a local structure offset or a small enum ID. Object-dictionary searches shall not run in OP cycles.

Compiled mapping example:

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

## 4.5 Default RX PDO

RX PDO is written by the master and consumed by the slave.

```text
u16 control_word
u8  mode
i16 target_current_mA
i32 target_speed
i32 target_position
```

| Offset | Object | Type | Description |
| --- | --- | --- | --- |
| 0 | `0x2002:00 ControlWord` | `u16` | Control word |
| 2 | `0x2001:00 ServoMode` | `u8` | Current/speed/position mode |
| 3 | `0x2003:00 TargetCurrent_mA` | `i16` | Current target |
| 5 | `0x2004:00 TargetSpeed` | `i32` | Speed target |
| 9 | `0x2005:00 TargetPosition` | `i32` | Position target |

`control_word` bits:

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

Rules:

- When `enable = 0`, the slave shall not drive output.
- `quick_stop = 1` requests a fast stop according to the configured policy.
- `clear_fault = 1` requests fault clearing only and shall not automatically enable output.
- `use_target_*` bits mark which target fields are valid in this cycle.

## 4.6 Default TX PDO

TX PDO is written by the slave and consumed by the master.

```text
u16 status_word
u16 fault_code
i16 current_mA
i32 speed
i32 position
u16 vcc_mV
i8  temp_c
```

| Offset | Object | Type | Description |
| --- | --- | --- | --- |
| 0 | `0x2100:00 StatusWord` | `u16` | Status word |
| 2 | `0x2104:00 FaultCode` | `u16` | Fault code |
| 4 | `0x2101:00 ActualCurrent_mA` | `i16` | Measured current |
| 6 | `0x2102:00 ActualSpeed` | `i32` | Measured speed |
| 10 | `0x2103:00 ActualPosition` | `i32` | Measured position |
| 14 | `0x2601:00 Vcc_mV` | `u16` | Supply voltage |
| 16 | `0x2602:00 Temperature_C` | `i8` | MCU temperature |

`status_word` bits:

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

## 4.7 High-density PDO_FAST Process Image

Real-time control uses `PDO_FAST` by default, not `PDO_CYCLE`. The principle is: all description data is configured through SDO in `PREOP`; OP cycles carry only process data.

`PDO_FAST` does not carry:

- index/subindex;
- node_id;
- position;
- rx_slot_size;
- tx_slot_size;
- node_count;
- per-node timestamps.

These values come from PDO configuration locked before `SAFEOP`.

### Ring-chain Rolling Process Image

Ring-chain mode should use a rolling process image, not a fixed full image. The master sends the largest command payload to Slave1. Each slave consumes its own RX PDO from the front of the remaining command area, forwards the remaining downstream commands, then appends its TX PDO feedback to the collected feedback area.

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

The command part decreases at each slave and the feedback part increases at each slave. Total frame length depends on `rx_slot_size` and `tx_slot_size`; it is constant only when both lengths are equal.

Each slave computes expected inbound and outbound process lengths from locked topology and slot sizes:

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

Slave processing:

```text
read RX PDO at process offset 0
forward remaining RX commands after the consumed RX slot
forward previously collected TX feedback unchanged
append local TX PDO feedback
recompute outgoing CRC and update outgoing length expectation
```

This avoids ID search and object-dictionary parsing during the real-time cycle. The slave only uses fixed counts and fixed slot sizes. Store-and-forward is the safer first implementation because the slave can verify incoming CRC before forwarding. Cut-through may be enabled later for performance, but it must not mask upstream CRC errors.

Recommended ring-chain OP fast path:

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

Forbidden in the OP fast path:

- No index/subindex object-dictionary lookup.
- No XML or string parsing.
- No dynamic memory.
- No printf.
- No PDO remapping.
- No PID or complex control logic in the UART interrupt path.

After receiving `PDO_FAST`, the slave updates only the runtime shadow command. The actual PID still runs from the 1 ms control scheduler. This keeps UART parsing and motion control from extending each other's execution time.

### Parallel-bus Process Image

Parallel mode uses broadcast RX process image plus slotted TX replies as the default OP design:

```text
Master PDO_FAST broadcast:
  SOF SEQ CYCLE_ID Slave1_RX_PDO Slave2_RX_PDO ... SlaveN_RX_PDO CRC16

Slave replies in configured time slot:
  SOF SEQ CYCLE_ID TX_PDO CRC16
```

Each slave derives its RX offset from its assigned slot in the locked parallel process image. TX replies are protected by master-assigned reply slots, so diode-combined TX lines do not contend.

```text
parallel_rx_offset = parallel_slot_index * rx_slot_size
reply_delay_us     = reply_slot_index * reply_slot_us
```

`PDO_FAST_POLL` is reserved only for diagnostic or compatibility use. If implemented, it shall include an addressed `node_id` field before `RX_PDO`; otherwise every slave on the parallel bus would see the same poll and may respond at the same time.

### Time Synchronization Fields

`PDO_FAST` carries only `CYCLE_ID`; it does not carry full `master_time_us` in every frame. Full time is calibrated with a lower-rate `SYNC` management frame.

Recommended:

```text
SYNC period      = 10 ms or 100 ms
PDO_FAST period  = 1 ms / 2 ms / 5 ms
```

If higher sync precision is required, `apply_cycle_offset` should be configured in the object dictionary instead of repeated in every PDO cycle.

### PDO_CYCLE Use

`PDO_CYCLE` remains available for debug, compatibility, and topology diagnostics. It may carry node_id, slot_size, last_seen_position, and timestamps, but it is not the default high-performance OP cycle.

## 4.8 SDO Frame Payload

`SDO_READ`:

```text
u16 index
u8  subindex
u8  reserved
```

`SDO_WRITE`:

```text
u16 index
u8  subindex
u8  type
u8  size
u8  data[8]
```

`SDO_REPLY`:

```text
u16 index
u8  subindex
u8  type
u8  size
u8  status
u8  data[8]
```

`status`:

```text
0x00 = OK
0x01 = object not found
0x02 = access denied
0x03 = type mismatch
0x04 = range error
0x05 = state error
0x06 = busy
```

The first version supports expedited SDO only, with up to 8 bytes of data. Segmented transfer is reserved and not implemented in the first STM32G030 firmware.

## 4.9 Object Dictionary Ranges

| Range | Name | Purpose |
| --- | --- | --- |
| `0x1000` | Identity | Device information, protocol version, UID |
| `0x1100` | Communication | Node ID, topology, baud rate, watchdog |
| `0x1200` | Network State | State transition and fault state |
| `0x1300` | Distributed Clock | Software distributed clock |
| `0x1600` | RX PDO Mapping | RX PDO mapping table |
| `0x1A00` | TX PDO Mapping | TX PDO mapping table |
| `0x2000` | Servo Command | Control source, mode, targets |
| `0x2100` | Servo Status | Status, current, speed, position |
| `0x2200` | Current Loop PID | Current-loop PID |
| `0x2300` | Speed Loop PID | Speed-loop PID |
| `0x2400` | Position Loop PID | Position-loop PID |
| `0x2500` | PWM Input Control | PWM input control |
| `0x2600` | Limits and Protection | Limits, temperature, voltage, FailSafe |
| `0x2700` | Encoder | Encoder parameters and feedback |
| `0x2800` | NVM Save/Restore | Save, load, erase configuration |

## 4.10 Key Objects

### Communication Cycle and Synchronization

| Index | Name | Type | Access | Notes |
| --- | --- | --- | --- | --- |
| `0x1103:00` | SerialWatchdogMs | `u16` | rw | Watchdog for serial control source |
| `0x1106:00` | PdoCycleTimeUs | `u32` | rw-preop | Fixed `PDO_FAST` cycle |
| `0x1107:00` | PdoJitterLimitUs | `u16` | rw-preop | Allowed master-cycle jitter |
| `0x1108:00` | PdoApplyCycleOffset | `u8` | rw-preop | Future cycle offset for applying PDO targets |
| `0x1300:00` | DcEnable | `u8` | rw | Software distributed-clock enable |
| `0x1302:00` | MasterOffsetUs | `i32` | ro | Estimated software time offset |
| `0x1304:00` | LastAppliedCycle | `u16` | ro | Last bus cycle applied by the control scheduler |
| `0x1305:00` | PathDelayUs | `u16` | rw-preop | Estimated path delay from master to this node |

These objects are configured in `PREOP` and locked in `SAFEOP`. After entering `OP`, the master shall send `PDO_FAST` at the fixed `PdoCycleTimeUs` period. To change cycle or apply offset, the master shall leave OP.

### Servo Command

| Index | Name | Type | Access | Current firmware relation |
| --- | --- | --- | --- | --- |
| `0x2000:00` | ControlSource | `u8` | rw | PWM/SERIAL_PDO/SERIAL_SDO selection |
| `0x2001:00` | ServoMode | `u8` | rw/pdo | `ServoCommand.mode` |
| `0x2002:00` | ControlWord | `u16` | rw/pdo | enable, quick stop |
| `0x2003:00` | TargetCurrent_mA | `i16` | rw/pdo | `ServoCommand.target_current_mA` |
| `0x2004:00` | TargetSpeed | `i32` | rw/pdo | `ServoCommand.target_speed` |
| `0x2005:00` | TargetPosition | `i32` | rw/pdo | `ServoCommand.target_position` |

`ControlSource`:

```text
0 = disabled
1 = serial_pdo
2 = pwm_input
3 = serial_sdo
```

`ControlSource` shall not be saved to NVM. Power-on and `BOOT` always force `pwm_input` unless the master explicitly switches to `serial_pdo` or `serial_sdo` during the current session.

`serial_sdo` is intended for low-rate debugging, simple masters, and tuning. The master updates the `ServoCommand` shadow by expedited `SDO_WRITE` access to `0x2001` through `0x2005`; the slave applies the latest SDO command at the 1 ms control scheduler boundary. It does not provide fixed-period PDO synchronization.

`serial_sdo` rules:

- Only expedited SDO is supported; segmented SDO control commands are not supported.
- Only `OP` may let SDO commands drive output. `PREOP/SAFEOP` may accept target shadow writes, but `Enable` shall not take effect.
- `ControlWord.enable = 0` shall disable output.
- Each valid SDO write to a command object refreshes `last_serial_command_time`.
- If no valid SDO control write arrives within `SerialWatchdogMs`, the slave executes `FailSafePolicy`.
- `serial_sdo` does not participate in `PdoCycleTimeUs`, `PdoApplyCycleOffset`, or distributed-clock synchronization.

### Servo Status

| Index | Name | Type | Access | Current firmware relation |
| --- | --- | --- | --- | --- |
| `0x2100:00` | StatusWord | `u16` | ro/pdo | generated from state |
| `0x2101:00` | ActualCurrent_mA | `i16` | ro/pdo | `Param.INA181_mA` |
| `0x2102:00` | ActualSpeed | `i32` | ro/pdo | `Param.EncoderSpeed` |
| `0x2103:00` | ActualPosition | `i32` | ro/pdo | `Param.EncoderMultiTurnValue` |
| `0x2104:00` | FaultCode | `u16` | ro/pdo | protection/protocol fault |
| `0x2105:00` | DrivePower | `i16` | ro/pdo | `Param.DrivePower` |

### PID Objects

`0x2200`, `0x2300`, and `0x2400` represent the current loop, speed loop, and position loop. Each range uses:

```text
sub 0x01 Kp:u16
sub 0x02 Ki:u16
sub 0x03 Kd:u16
sub 0x04 IntegralMax:i32
sub 0x05 OutMax:u16
sub 0x06 OutMin:u16
sub 0x07 Reset:wo u8
```

### PWM Input Control

| Index | Name | Type | Access | Default |
| --- | --- | --- | --- | --- |
| `0x2500:00` | PwmMode | `u8` | rw | speed |
| `0x2501:00` | PwmMinUs | `u16` | rw | `1000` |
| `0x2502:00` | PwmMidUs | `u16` | rw | `1500` |
| `0x2503:00` | PwmMaxUs | `u16` | rw | `2000` |
| `0x2504:00` | PwmDeadbandUs | `u16` | rw | `20` |
| `0x2505:00` | PwmTimeoutMs | `u16` | rw | `50` |
| `0x2506:00` | PwmRawCapture | `u16` | ro | `Param.DutyRatio` |

### NVM Save

| Index | Name | Type | Access | Notes |
| --- | --- | --- | --- | --- |
| `0x2800:00` | NvmCommand | `u8` | wo | save/load/erase |
| `0x2801:00` | NvmStatus | `u8` | ro | idle/busy/ok/error |
| `0x2802:00` | NvmSequence | `u32` | ro | latest saved sequence |

Runtime targets, current command mode target values, and `Enable` shall not be saved to NVM to avoid automatic motion after power-on. PID gains, limits, direction, protection thresholds, and default PDO mappings may be saved.
