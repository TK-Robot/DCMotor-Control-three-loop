# 01 Overview and Physical Layer

## 1.1 Protocol Role

TSBP targets compact DC servo slaves. Its primary purpose is to let a master configure and control multiple motor slaves over UART while keeping PWM input as the default control source.

Design priorities:

- Small code size: the slave MCU does not parse XML, does not allocate dynamic memory, and does not use printf for control frames.
- High information density: binary little-endian frames and mapped PDO slots.
- Asynchronous receive: UART DMA / idle reception on the slave, parsed by the main loop or a lightweight state machine.
- Master scheduling: all real-time traffic is initiated by the master. Slaves reply or forward only inside defined windows.
- Hot connection: a newly connected node is not driven by serial control until it is discovered, configured, and placed in OP.

## 1.2 Roles

| Role | Description |
| --- | --- |
| Master | Bus controller for discovery, configuration, synchronization, PDO cycles, and recovery |
| Slave | Node with an assigned node ID |
| Unassigned Slave | Powered node without an assigned ID, defaulting to PWM control |
| Offline Node | Node considered lost by the master and removed from PDO cycles |
| Fault Node | Node reporting protection or protocol fault |

## 1.3 Supported Topologies

### TOPO_PARALLEL_BUS

```text
Master TX -> Slave1 RX
Master TX -> Slave2 RX
Master TX -> SlaveN RX

Slave1 TX --diode--+
Slave2 TX --diode--+-> Master RX
SlaveN TX --diode--+
```

Rules:

- All slaves receive the master TX signal in parallel.
- Slave TX lines are diode-combined to the master RX.
- Slaves must reply only in their assigned time slots.
- Broadcast writes do not reply by default to avoid bus contention.

### TOPO_RING_CHAIN

```text
Master TX -> Slave1 RX
Slave1 TX -> Slave2 RX
Slave2 TX -> Slave3 RX
...
SlaveN TX -> Master RX
```

Rules:

- Single-UART unidirectional ring chain.
- A frame sent by the master travels through Slave1 to SlaveN and finally returns to the master.
- OP `PDO_FAST` uses a rolling process image: each slave consumes its own RX command from the front of the command area and appends its TX feedback to the feedback area.
- The first implementation should use store-and-forward so the slave can verify CRC before forwarding. Cut-through may be added later as an optimized mode, but its CRC/error behavior shall be specified before use.
- Ring-chain mode avoids multi-slave TX contention, but any broken point may break the loopback.

## 1.4 UART Requirements

| Item | Default | Notes |
| --- | --- | --- |
| Format | 8N1 | 8 data bits, no parity, 1 stop bit |
| Default baud | 1 Mbps | First default |
| Reserved baud | 2 Mbps | Enable after validation |
| Endian | Little-endian | Multi-byte fields |
| Encoding | Binary only | ASCII is forbidden for control frames |
| Recommended length | Tens of centimeters | Not a long-distance industrial bus |

## 1.5 Power-on and Hot-connect Behavior

- After power-on, a slave enters `BOOT` and defaults to `ControlSource = PWM`.
- Serial PDO shall not drive motor output before the node enters `OP`.
- SDO may configure parameters. Under the `SERIAL_SDO` control source it may write low-rate targets, but target writes shall still not automatically enable output.
- A hot-connected node may only respond to discovery/enumeration before ID assignment.
- During power loss, undervoltage, overtemperature, or communication loss, the slave shall apply its configured FailSafe policy: disable, brake, or fallback to PWM.
