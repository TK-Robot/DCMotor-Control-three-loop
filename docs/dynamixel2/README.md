# TK Servo DYNAMIXEL Protocol 2.0

本固件使用标准 DYNAMIXEL Protocol 2.0 数据包和指令，不再编译原有 TSBP 实现。

## 支持范围

- 数据包：`FF FF FD 00 | ID | Length | Instruction | Parameters | CRC16`。
- CRC：DYNAMIXEL CRC-16，多项式 `0x8005`，初值 `0`。
- 支持 Byte Stuffing。
- 指令：`Ping (0x01)`、`Read (0x02)`、`Write (0x03)`、`Reg Write (0x04)`、`Action (0x05)`、`Sync Read (0x82)`、`Sync Write (0x83)`。
- 广播 ID：`0xFE`；节点 ID：`1..252`。
- 波特率代码：`2=115200`、`3=1 Mbps`、`4=2 Mbps`；修改后重启生效。
- 当前不支持 Factory Reset、Reboot、Bulk Read/Write 和动态 PDO 映射。

## 总线约束

主站 TX 可广播到所有从站 RX；多个从站 TX 经硬件二极管汇入主站 RX。主站必须是唯一调度者，从站不得主动上报。

- 单节点 `Read/Write`：仅被寻址节点回复。
- `Sync Write`：广播写入，各节点不回复。
- `Sync Read`：请求参数末尾 ID 列表的顺序就是回复顺序。
- 广播 `Ping`：按 NVM 中的 `NodePosition` 排队回复。
- `ReplySlotUs` 是最小回复槽宽；固件会按波特率和响应长度自动增大，避免相邻响应重叠。

## 安全语义

- 上电控制源固定回到 PWM 输入，串口输出保持未使能。
- 写控制源不会自动使能，控制源切换会撤销当前使能。
- 清故障写入不会使能；必须随后单独显式写使能位。
- 串口看门狗仅在 DYNAMIXEL 串口控制且输出已使能时计时。
- 超时执行 NVM 配置的失联策略，默认关闭输出。
- 多圈运行计数和运动目标不写入 NVM。

## 主站最小流程

1. 单节点 `Ping` 确认 ID。
2. `Read` 地址 `0..3` 确认型号、固件和协议版本。
3. `Write` 地址 `16` 为 `1`，切换到 DYNAMIXEL 串口控制；此时仍未使能。
4. 写地址 `17/20/22/26` 设置模式和目标。
5. 最后单独写地址 `18` 为 `0x0001` 显式使能。
6. 运行期间周期性 `Read` 或刷新命令，间隔必须小于地址 `6` 的看门狗时间。
7. 停止时先写安全目标，再写地址 `18` 为 `0`。

控制表详见 [control-table.md](control-table.md)。
