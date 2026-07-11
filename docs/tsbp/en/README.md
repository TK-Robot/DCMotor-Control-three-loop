# TK Servo Bus Protocol Documentation

TK Servo Bus Protocol, abbreviated as TSBP, is a lightweight UART servo-bus protocol for the STM32G030 DC motor cascaded-control slave used by this project. It is designed for ordinary UART wiring, small Flash size, DMA-based communication, and a 1 ms control loop while still supporting master scheduling, multiple slaves, PDO/SDO separation, automatic node assignment, reconnect handling, default PWM control, and lightweight distributed-clock synchronization.

TSBP is not wire-compatible with EtherCAT, CANopen, or DYNAMIXEL. It only adopts design ideas that fit this project:

- CANopen: object dictionary, real-time PDO data, SDO parameter access, and network states.
- EtherCAT: process data image, chain topology, distributed clocks, and slave description files.
- DYNAMIXEL Protocol 2.0: UART multi-drop devices, broadcast ID, sync write/read, and compact binary packets.

## Document Modules

1. [Overview and Physical Layer](01-overview-physical.md)
2. [Data Link and Frame Format](02-data-link-frame.md)
3. [State Machine and Topology Discovery](03-state-topology.md)
4. [PDO, SDO, and Object Dictionary](04-pdo-sdo-object-dictionary.md)
5. [Master Rules, Fault Handling, and Clock Sync](05-master-fault-clock.md)
6. [English XML-like Slave Configuration Example](TKServoSlave.en.tks.xml)

## Master Design Principles

The master shall follow these rules:

- The master is the only bus scheduler. A slave shall not transmit without a master-triggered window.
- Broadcast frames do not generate replies unless the frame explicitly defines discovery, enumeration, or slotted response behavior.
- Before entering OP, the master shall finish node discovery, SDO configuration, PDO mapping, bus-load calculation, and synchronization setup.
- PDO carries real-time process data only. It shall not carry debug strings or large configuration data.
- SDO is primarily used for parameters, configuration, diagnostics, save commands, and PDO mapping. Under the `SERIAL_SDO` control source, it may also write low-rate servo command objects.
- The default power-on control source is PWM. The serial bus may take over only after SDO switches the node to `SERIAL_PDO` or `SERIAL_SDO`.
- The master shall stop cyclic PDO traffic to offline nodes and use only low-rate PING, DISCOVERY, or ENUM reconnect traffic.
- The master shall calculate the communication cycle from baud rate, node count, and PDO size. OP is forbidden when bus usage exceeds 80%.
- In ring-chain mode, any loopback failure shall stop the whole chain PDO cycle until topology is recovered.

## Versioning

This document defines TSBP version `0.1-draft`.

- `VER = 0x01` in binary frames means this draft frame format.
- XML-like slave files use `schemaVersion="1.0"` for the first compatible schema.
- Breaking changes must increment the frame `VER` or XML `schemaVersion`.
- Backward-compatible object additions may keep the same protocol version.

## References

- DYNAMIXEL Protocol 2.0: https://emanual.robotis.com/docs/en/dxl/protocol2/
- CANopen overview: https://en.wikipedia.org/wiki/CANopen
- EtherCAT overview: https://en.wikipedia.org/wiki/EtherCAT
- EtherCAT Technology Group: https://www.ethercat.org/
