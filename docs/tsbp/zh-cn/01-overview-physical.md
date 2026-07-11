# 01 总览与物理层

## 1.1 协议定位

TSBP 面向小体积直流伺服从站。它的核心任务是让主站通过串口配置和控制多个电机从站，同时保留 PWM 输入作为默认主控方式。

协议设计重点：

- 低代码体积：从站 MCU 不解析 XML，不使用动态内存，不使用 printf 格式化控制帧。
- 高信息密度：二进制小端帧，PDO 固定映射后按 slot 传输。
- 异步接收：从站使用 UART DMA / idle 接收，主循环或轻量状态机解析。
- 主站调度：所有实时通信由主站发起，从站只在规定窗口回复或转发。
- 热连接：新节点插入后默认不被串口驱动，必须经过发现、配置和 OP 使能。

## 1.2 角色

| 角色 | 说明 |
| --- | --- |
| Master | 总线控制器，负责发现、配置、同步、PDO 周期和故障恢复 |
| Slave | 已分配节点 ID 的从站 |
| Unassigned Slave | 已上电但未分配 ID 的从站，默认 PWM 控制 |
| Offline Node | 主站认为失联的节点，不再参与 PDO 周期 |
| Fault Node | 上报保护或协议错误的节点 |

## 1.3 支持拓扑

### TOPO_PARALLEL_BUS

```text
Master TX -> Slave1 RX
Master TX -> Slave2 RX
Master TX -> SlaveN RX

Slave1 TX --diode--+
Slave2 TX --diode--+-> Master RX
SlaveN TX --diode--+
```

规则：

- 所有从站同时收到主站下发数据。
- 从站 TX 通过二极管汇总到主站 RX。
- 从站必须按主站分配的 reply slot 分时回复。
- 广播写入默认无回复，避免多个 TX 同时驱动总线。

### TOPO_RING_CHAIN

```text
Master TX -> Slave1 RX
Slave1 TX -> Slave2 RX
Slave2 TX -> Slave3 RX
...
SlaveN TX -> Master RX
```

规则：

- 单 UART 单向环形链。
- 主站发出的帧依次经过 Slave1 到 SlaveN，最后回到主站。
- OP `PDO_FAST` 使用滚动过程映像：每个从站从命令区最前面消费自己的 RX 命令，并把自己的 TX 反馈追加到反馈区。
- 第一版建议使用 store-and-forward，让从站先验证 CRC 再转发。后续可以增加 cut-through 优化模式，但启用前必须明确 CRC 和错误传播规则。
- 链式模式天然避免多从站同时发送冲突，但任意断点会影响断点后的节点和回环。

## 1.4 UART 要求

| 项目 | 默认值 | 说明 |
| --- | --- | --- |
| 格式 | 8N1 | 8 数据位、无校验、1 停止位 |
| 默认波特率 | 1 Mbps | 第一版默认值 |
| 预留波特率 | 2 Mbps | 实测稳定后启用 |
| 字节序 | 小端 | 多字节字段 |
| 编码 | 纯二进制 | 控制帧禁止 ASCII |
| 推荐线长 | 几十厘米 | 不承诺长距离工业总线 |

## 1.5 上电与热连接行为

- 上电后从站进入 `BOOT`，默认 `ControlSource = PWM`。
- 未进入 `OP` 前，串口 PDO 不允许驱动电机输出。
- SDO 可配置参数；在 `SERIAL_SDO` 控制源下可写入低频目标值，但仍不得自动使能输出。
- 热插入节点在未分配 ID 前只允许响应发现/枚举帧。
- 掉电、低压、过温或通信失联时，从站必须按 FailSafe 策略关闭、刹车或回退 PWM。
