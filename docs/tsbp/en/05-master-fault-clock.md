# 05 Master Rules, Fault Handling, and Clock Sync

## 5.1 Master Responsibilities

The master is not a simple serial transmitter; it is the real-time bus scheduler. It shall maintain:

- Node table: node ID, UID, topology position, online state, and fault state.
- State machine: `INIT/PREOP/SAFEOP/OP/FAULT` for each node.
- PDO mapping: RX/TX PDO items, slot size, and cycle period for each node.
- Timeout counters: PDO miss, SDO timeout, and ring-chain loopback timeout.
- Reconnect timers: low-rate PING or DISCOVERY for offline nodes.
- Synchronization state: cycle ID, master time, apply cycle, and chain-delay estimates.
- Safety policy: disable, brake, or fallback PWM after communication loss.

## 5.2 OP Entry Checklist

Before entering OP, the master shall complete:

1. Discover or enumerate all required nodes.
2. Assign unique node IDs.
3. Read identity and capability objects.
4. Set topology, baud, watchdog, and reconnect parameters.
5. Configure control source, FailSafe policy, limits, and servo mode.
6. Configure RX/TX PDO mappings in `PREOP`.
7. Check PDO byte size and bus-load budget.
8. Enter `SAFEOP`.
9. Run at least one synchronization cycle.
10. Enter `OP` with `Enable = 0`.
11. Enable outputs only when commanded by the application.

## 5.3 Bus-load Calculation

UART 8N1:

```text
byte_time_us = 10 * 1000000 / baud
frame_time_us = frame_bytes * byte_time_us
```

Recommended full budget for one UART link:

```text
wire_time_us = frame_bytes * 10 * 1000000 / baud
guard_time_us = max(50, node_count * 10)
```

The final OP budget is topology-specific. Parallel mode is dominated by the master broadcast plus reply slots. Ring-chain store-and-forward mode shall sum every chain segment because each node verifies and rebuilds the frame before forwarding:

```text
ring_required_cycle_us =
    sum(segment_frame_bytes[i] * 10 * 1000000 / baud)
  + node_count * node_process_delay_us
  + guard_time_us

bus_usage = ring_required_cycle_us / configured_cycle_us
```

| Bus usage | Master behavior |
| --- | --- |
| `< 60%` | Normal, OP allowed |
| `60% - 80%` | Warning, OP allowed only if configured |
| `> 80%` | OP forbidden |

If bus usage is too high, the master shall reduce PDO item count, reduce node count per cycle, lower update rate, or increase baud rate after validation.

### PDO_FAST Load Calculation

Real-time OP cycles should be calculated from `PDO_FAST`, not the full management frame:

```text
fast_overhead_bytes = 6

// Ring-chain rolling image, segment i:
// i = 0 is Master -> Slave1
// i = node_count is SlaveN -> Master
segment_process_bytes[i] =
    (node_count - i) * rx_slot_size
  + i * tx_slot_size

segment_frame_bytes[i] = fast_overhead_bytes + segment_process_bytes[i]

store_forward_required_us =
    sum(segment_frame_bytes[0..node_count] * 10 * 1000000 / baud)
  + node_count * node_process_delay_us
  + guard_time_us
```

Example with 4 nodes, 13-byte RX PDO and 17-byte TX PDO per node:

```text
segment process bytes:
  Master -> Slave1: 4*13 + 0*17 = 52
  Slave1 -> Slave2: 3*13 + 1*17 = 56
  Slave2 -> Slave3: 2*13 + 2*17 = 60
  Slave3 -> Slave4: 1*13 + 3*17 = 64
  Slave4 -> Master: 0*13 + 4*17 = 68

segment frame bytes including 6-byte fast overhead:
  58 + 62 + 66 + 70 + 74 = 330 bytes

store_forward_wire_time_at_1Mbps = 330 * 10 us = 3300 us
```

This cannot fit a 1 ms cycle in the safe store-and-forward first implementation. The master should select a multi-millisecond cycle or reduce PDO content. If speed mode uses only required fields, for example 8-byte RX and 8-byte TX:

```text
each segment process bytes = 4 * 8 = 32
each segment frame bytes = 38
5 segments * 38 bytes = 190 bytes
store_forward_wire_time_at_1Mbps = 1900 us
```

This still does not fit a 1 ms store-and-forward cycle at 1 Mbps. For 1 ms chain operation, the implementation needs fewer nodes per cycle, a higher baud rate after validation, or a future cut-through mode with explicitly defined CRC/error handling.

## 5.4 Communication Period Selection

The master should select cycle time from PDO size and control mode:

```text
current loop internal period  = 1 ms
speed loop internal period    = 5 ms
position loop internal period = 10 ms
```

Recommended rules:

- Current mode PDO: target 1 ms to 2 ms.
- Speed mode PDO: target 2 ms to 5 ms.
- Position mode PDO: target 5 ms to 10 ms.
- If bus load is too high, keep safety status and feedback first, then reduce target update rate.

### Fixed-cycle Requirement

After entering `OP`, the master shall send `PDO_FAST` at a fixed period. The period is configured in `PREOP` and locked in `SAFEOP`; it shall not change dynamically in `OP`.

If the period must change, the master shall run:

```text
OP -> SAFEOP -> PREOP -> rewrite cycle objects -> SAFEOP -> OP
```

Recommended jitter limits:

```text
PdoCycleTimeUs      = configured fixed cycle
PdoJitterLimitUs    = min(PdoCycleTimeUs / 10, 100 us)
PdoApplyCycleOffset = 1 or 2 cycles
```

Rules:

- The master shall send at the fixed period and shall not send only when new data exists.
- The slave uses `CYCLE_ID` to detect missing, duplicated, or jumped cycles.
- If no valid PDO is received for more than `PDO_MISS_LIMIT` consecutive cycles, the PDO watchdog triggers.
- Cycle change is a communication configuration change, not a normal runtime command.

## 5.5 Parallel-bus Offline and Reconnect

Default parameters:

```text
PDO_MISS_LIMIT = 3
RECONNECT_PERIOD_MS = 1000
SERIAL_WATCHDOG_MS = 100
```

Master behavior:

```text
if node is online:
    send/expect PDO according to schedule
    if reply missing or invalid:
        miss_count++
    if miss_count >= PDO_MISS_LIMIT:
        online = false
        pdo_enabled = false
        stop sending PDO to this node

if node is offline:
    every RECONNECT_PERIOD_MS:
        send PING or DISCOVERY
        if reply valid:
            enter PREOP
            reconfigure SDO/PDO
            enter SAFEOP then OP
```

In parallel mode, one offline node shall not slow down all nodes. The master removes it from PDO cycles and keeps only low-rate reconnect traffic. After recovery, the node shall be reconfigured before returning to OP.

## 5.6 Ring-chain Fault and Reconnect

Ring-chain failure conditions:

- `PDO_FAST` loopback does not return before timeout in OP.
- Diagnostic `PDO_CYCLE` does not return before timeout during diagnostics.
- Returned frame has invalid CRC.
- Returned `SEQ` does not match.
- Returned `node_count` does not match expected topology.
- `last_seen_position` is lower than expected.

Master behavior:

```text
if ring PDO loopback fails PDO_MISS_LIMIT times:
    chain_state = CHAIN_FAULT
    stop PDO_FAST
    mark fault_after_position if possible
    every RECONNECT_PERIOD_MS:
        send ENUM_START
        if ENUM returns:
            rebuild node table
            assign IDs
            reconfigure PDO
            enter SAFEOP
            sync clocks
            enter OP
```

In ring-chain mode, any broken point may affect the whole loop. The master shall not simply remove one node and keep PDO running. It shall stop chain PDO and rebuild topology through ENUM.

## 5.7 Fault Point Marking

Ring-chain fault location is either confirmed or inferred:

- Confirmed fault: a loopback frame returns and `last_seen_position` is valid. The master marks `fault_after_position = last_seen_position`.
- Inferred fault: no loopback returns. The master marks `CHAIN_RETURN_LOST` and infers from the last valid topology.
- If `last_seen_position = N` and expected node count is also `N`, the fault may be on `SlaveN TX -> Master RX`.

## 5.8 Slave Watchdog and FailSafe

Objects:

```text
0x1103 SerialWatchdogMs
0x2608 FailSafePolicy
```

`FailSafePolicy`:

```text
0 = disable output
1 = brake
2 = fallback to PWM
```

When the slave uses a serial control source, timeout shall execute FailSafe:

- `SERIAL_PDO`: no valid PDO for more than `PDO_MISS_LIMIT` cycles, or no valid PDO within `SerialWatchdogMs`.
- `SERIAL_SDO`: no valid SDO control write within `SerialWatchdogMs`.

- `disable output`: disable output, `DriveRunMode = 0`.
- `brake`: enter brake mode.
- `fallback to PWM`: return to PWM input control.

Recommended default: use `disable output` for industrial safety. Use `fallback to PWM` for RC/servo-like applications.

Watchdog priority:

- In `OP`, PDO cycle supervision is primary: `miss_count >= PDO_MISS_LIMIT` triggers PDO watchdog and FailSafe.
- In `SERIAL_PDO`, `SerialWatchdogMs` is a backup limit for PDO supervision and shall be greater than or equal to `PdoCycleTimeUs * PDO_MISS_LIMIT / 1000`.
- In `SERIAL_SDO`, `SerialWatchdogMs` directly limits the maximum interval between SDO control writes.
- Outside `OP`, serial control writes update only the shadow command and shall not drive output.
- PWM default control is not disabled merely because serial traffic is absent.

## 5.9 Lightweight Distributed Clock

TSBP does not synchronize MCU hardware clocks like full EtherCAT hardware. It synchronizes process timing:

- when targets are applied,
- when feedback is sampled,
- which cycle a PDO belongs to.

`SYNC` payload:

```text
u16 cycle_id
u32 master_time_us
u16 apply_cycle
u16 flags
```

Diagnostic `PDO_CYCLE` may include the same fields:

```text
u16 cycle_id
u32 master_time_us
u16 apply_cycle
```

High-rate `PDO_FAST` does not carry full `master_time_us`; it carries only `CYCLE_ID`. The slave uses the latest `SYNC` to calculate software time offset and applies targets at the scheduler boundary using the fixed `apply_cycle_offset`.

Slave behavior:

```text
on valid SYNC or PDO_CYCLE:
    local_rx_time = local_time_us()
    error = master_time_us - local_rx_time
    filtered_offset += error / filter_div
    if cycle_id == apply_cycle:
        apply pending PDO target at scheduler boundary
```

Rules:

- PDO targets shall not apply immediately in the middle of parsing.
- Targets apply at a 1 ms scheduler boundary or configured apply cycle.
- Ring-chain slaves should record `rx_latch_us` and `tx_latch_us`.
- Master may estimate per-node forwarding delay from latch timestamps.

### Slave Clock-offset Implementation

The slave does not need to tune the MCU hardware clock. It maintains a software offset:

```text
bus_time_us = local_time_us + bus_time_offset_us
```

Where:

- `local_time_us`: slave-local monotonic microsecond counter, from TIM or extended SysTick.
- `bus_time_offset_us`: software offset between master time and local time.
- `path_delay_us`: estimated communication delay from master to this slave. The first version may use 0 or a master-estimated value from position.

When receiving SYNC, the slave should capture the local timestamp as early as possible:

```text
on SYNC frame received:
    rx_local_us = capture_local_time_as_early_as_possible()
    expected_master_rx_us = sync.master_time_us + path_delay_us
    sample_offset = expected_master_rx_us - rx_local_us
    bus_time_offset_us += (sample_offset - bus_time_offset_us) / filter_div
    last_sync_cycle_id = sync.cycle_id
```

Recommended `filter_div`:

```text
first sync: filter_div = 1, lock immediately
stable run: filter_div = 8 or 16, reduce jitter
```

The slave uses `bus_time_us` to derive the current bus cycle:

```text
bus_time_us = local_time_us + bus_time_offset_us
current_cycle = bus_time_us / PdoCycleTimeUs
cycle_phase_us = bus_time_us % PdoCycleTimeUs
```

When `PDO_FAST` arrives, the slave does not feed the target to PID immediately. It records a future apply cycle:

```text
on PDO_FAST:
    if crc_ok and seq_ok:
        pending_command = rx_pdo
        pending_apply_cycle = pdo_fast.cycle_id + PdoApplyCycleOffset
```

At the 1 ms control scheduler boundary:

```text
on control_1ms_tick:
    bus_time_us = local_time_us + bus_time_offset_us
    current_cycle = bus_time_us / PdoCycleTimeUs

    if pending_valid and current_cycle >= pending_apply_cycle:
        active_command = pending_command
        pending_valid = false
```

This decouples UART receive timing from motor-control apply timing and reduces node-to-node action skew caused by ring-chain forwarding delay.

### Ring-chain path_delay_us Estimation

The first version may use a simple estimate:

```text
path_delay_us = (position - 1) * average_node_forward_delay_us
```

For higher precision, the master may use diagnostic `PDO_CYCLE` timestamps `rx_latch_us` / `tx_latch_us` to estimate each node's delay, then write:

```text
0x1305 PathDelayUs
```

Note: at tens-of-centimeters wiring, physical propagation delay is small. The dominant delay comes from UART byte time, DMA receive completion, software forwarding, and scheduler boundaries. The first version should prioritize fixed cycles and future-cycle application over nanosecond-level synchronization.

## 5.10 Safety Rules for Motion Control

- `Enable = 0` shall disable output.
- `clear_fault` shall not implicitly enable output.
- When switching `ServoMode`, the slave shall clear unused PID history and integrators.
- When switching from `PWM` to `SERIAL_PDO` or `SERIAL_SDO`, the target should be near current feedback or `Enable = 0`.
- Undervoltage, overtemperature, encoder error, and current fault have higher priority than PDO commands.

## 5.11 Validation Plan

- Document review: the master can initialize a node using only the XML-like slave file.
- PC protocol simulation: parallel 1, 4, 8, and 16 nodes; ring-chain 1, 4, and 8 nodes; node loss and reconnect.
- Error tests: CRC error, duplicate SEQ, missing frame, and wrong node count.
- Firmware validation: UART DMA can handle maximum selected frame size; PDO parsing does not block the 1 ms control loop; serial timeout applies FailSafe; PWM remains the default after power-on.
- Bus-load validation: calculate minimum cycle time for different node counts and PDO lengths at 1 Mbps.

## 5.12 Slave Performance Cost Reduction Rules

The STM32G030 slave has limited resources, so the implementation shall follow the rule: do more work during configuration, and less work during runtime.

### Work Allowed During Configuration

The following work is allowed in `PREOP/SAFEOP` because it does not run in the real-time path:

- Validate PDO mapping items.
- Generate RX/TX copy tables from the object dictionary.
- Calculate `rx_len`, `tx_len`, `node_stride`, `rx_offset`, and `tx_offset`.
- Calculate the fixed PDO_FAST frame length.
- Save non-volatile configuration parameters.
- Configure FailSafe policy, PWM mode, PID, and limits.

### Forbidden Work in OP Cycles

The following work shall not run in the PDO_FAST cycle:

- Object-dictionary lookup.
- XML or string parsing.
- Dynamic memory allocation.
- printf, sprintf, or floating-point formatting.
- Slot search by node ID.
- PDO remapping.
- PID execution or blocking I2C inside UART callbacks.
- Flash save in the high-rate path.

### OP Fast-path Target

The slave path for one `PDO_FAST` frame should be short:

```text
DMA/idle frame ready
  -> CRC check
  -> fixed offset RX copy
  -> update command shadow
  -> fixed offset TX copy
  -> CRC update
  -> DMA forward/reply
```

Keep the UART layer and control layer separated:

```text
UART layer:
  receives PDO_FAST
  updates ServoCommand shadow
  fills feedback shadow
  forwards/replies

Control layer:
  runs at 1 ms scheduler
  consumes latest ServoCommand shadow
  runs current/speed/position loops
  updates feedback variables
```

### Recommended Runtime Layout

```text
typedef struct {
    uint8_t rx_len;
    uint8_t tx_len;
    uint8_t node_stride;
    uint8_t position;
    uint16_t rx_offset;
    uint16_t tx_offset;
    PdoCopyItem rx_items[8];
    PdoCopyItem tx_items[8];
} PdoRuntime;
```

This structure is locked in `SAFEOP` and shall not be modified in OP cycles. If the master needs to change PDO, it shall move the node out of OP and back to PREOP.

### CRC Cost Strategy

The first implementation may use table-driven CRC16 for speed. If Flash becomes tighter, it may use a smaller table or bitwise CRC. Selection rule:

- Speed priority: 256-entry CRC table.
- Flash priority: 16-entry nibble table or bitwise CRC.
- OP-cycle CRC shall not be removed just to save cost.

### Documentation Acceptance Criteria

When implementing the protocol code later, code review should be able to answer:

- Is there any object-dictionary lookup in OP cycles?
- Is there any dynamic memory or string handling in OP cycles?
- Is PDO mapping compiled only in PREOP?
- Do UART callbacks avoid PID, Flash, and blocking I2C operations?
- Is the 1 ms control cycle still scheduled by `ServoControl` instead of driven directly by UART interrupts?
