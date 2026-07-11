# TK Servo Bus Protocol 中文文档

TK Servo Bus Protocol，简称 TSBP，是面向本项目 STM32G030 直流电机三环伺服从站的轻量串口控制协议。协议目标是在普通 UART、较小 Flash、DMA 接收/发送和 1 ms 控制周期约束下，实现主站调度、多从站控制、PDO/SDO 分层、自动节点分配、故障重连、PWM 默认控制和轻量分布式时钟同步。

TSBP 不是 EtherCAT、CANopen 或 DYNAMIXEL 的兼容实现。它只借鉴这些协议中适合本项目的设计思想：

- CANopen：对象字典、PDO 实时数据、SDO 参数访问、网络状态机。
- EtherCAT：过程数据映像、链式拓扑、分布式时钟和主站配置文件思想。
- DYNAMIXEL Protocol 2.0：串口多设备、广播 ID、同步写/同步读、紧凑二进制帧。

## 文档模块

1. [总览与物理层](01-overview-physical.md)
2. [数据链路层与帧格式](02-data-link-frame.md)
3. [状态机与拓扑发现](03-state-topology.md)
4. [PDO、SDO 与对象字典](04-pdo-sdo-object-dictionary.md)
5. [主站规则、故障处理与时钟同步](05-master-fault-clock.md)
6. [中文 XML 类从站配置示例](TKServoSlave.zh-cn.tks.xml)

## 主站设计原则

主站必须遵守以下原则：

- 主站是总线唯一调度者，从站不得无请求主动发送。
- 广播帧默认不回复，除非该帧明确定义了发现、枚举或分时回复窗口。
- 进入 OP 状态前，主站必须完成节点发现、SDO 参数配置、PDO 映射配置、周期负载计算和同步准备。
- PDO 只传实时过程数据，不用于调试字符串或大块配置。
- SDO 主要用于参数、配置、诊断、保存命令和 PDO 映射；在 `SERIAL_SDO` 控制源下，也允许低频写入伺服命令对象。
- 默认上电由 PWM 控制；串口必须通过 SDO 显式切换到 `SERIAL_PDO` 或 `SERIAL_SDO` 才能接管电机。
- 主站不得对 offline 节点继续发送 PDO，只允许低频 PING、DISCOVERY 或 ENUM 重连。
- 主站必须根据波特率、节点数和 PDO 长度自动计算通信周期；总线占用超过 80% 不允许进入 OP。
- 链式模式下，任意回环失败都必须停止整链 PDO，先恢复拓扑再恢复实时控制。

## 版本规则

本文档定义 TSBP 协议版本 `0.1-draft`。

- 二进制帧中 `VER = 0x01` 表示当前草案帧格式。
- XML 类从站文件第一版使用 `schemaVersion="1.0"`。
- 不兼容修改必须增加帧版本或 XML schema 版本。
- 只新增对象字典项且兼容旧主站时，可以保持协议版本不变。

## 参考资料

- DYNAMIXEL Protocol 2.0: https://emanual.robotis.com/docs/en/dxl/protocol2/
- CANopen overview: https://en.wikipedia.org/wiki/CANopen
- EtherCAT overview: https://en.wikipedia.org/wiki/EtherCAT
- EtherCAT Technology Group: https://www.ethercat.org/
