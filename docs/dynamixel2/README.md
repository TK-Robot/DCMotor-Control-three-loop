# TK Servo DYNAMIXEL Protocol 2.0

本固件使用标准 DYNAMIXEL Protocol 2.0 数据包和指令，不再编译原有 TSBP 实现。

## 支持范围

- 数据包：`FF FF FD 00 | ID | Length | Instruction | Parameters | CRC16`。
- CRC：DYNAMIXEL CRC-16，多项式 `0x8005`，初值 `0`。
- 支持 Byte Stuffing。
- 指令：`Ping (0x01)`、`Read (0x02)`、`Write (0x03)`、`Reg Write (0x04)`、`Action (0x05)`、`Sync Read (0x82)`、`Sync Write (0x83)`。
- 广播 ID：`0xFE`；节点 ID：`1..252`。
- 波特率代码：`2=115200`、`3=1 Mbps`、`4=2 Mbps`、`5=230400`、`6=420000 (ELRS/CRSF)`；单播 ACK 发送完成后运行期切换，写 NVM 后才跨复位保留。
- 当前不支持 Factory Reset、Reboot、Bulk Read/Write 和动态 PDO 映射。
- USART2 同时识别 CRSF `RC Channels Packed (0x16)`；DYNAMIXEL 用于配置/诊断，CRSF 可作为实时位置控制源。两种完整帧可交替发送，但不能字节交织。

## 总线约束

主站 TX 可广播到所有从站 RX；多个从站 TX 经硬件二极管汇入主站 RX。主站必须是唯一调度者，从站不得主动上报。

- 单节点 `Read/Write`：仅被寻址节点回复。
- `Sync Write`：广播写入，各节点不回复。
- `Sync Read`：请求参数末尾 ID 列表的顺序就是回复顺序。
- 广播 `Ping`：按 NVM 中的 `NodePosition` 排队回复。
- `ReplySlotUs` 是响应时隙宽度的可配置下限，不是固定保护延迟。实际时隙为 `max(ReplySlotUs, ceil((11 + DataLength) * 10 * 1000000 / Baud) + 50 us)`；例如 115200 baud、27 字节反馈时约为 `3349 us`。

## 回复和确认

本设备固定使用等价于 `Status Return Level=2` 的策略，不提供可配置寄存器：

- 单播 Ping、Read、Write、Reg Write 和 Action 必须返回标准 `0x55` Status Packet。
- Read 成功时 `Error=0` 且 Parameters 是读取数据，该包同时就是读取 ACK。
- 普通 Write 成功时 `Error=0` 且无 Parameters；失败时返回标准 Error 编号。
- 广播 Write、Reg Write、Action 和 Sync Write 不回复，防止多节点碰撞。
- 广播 Ping 和 Sync Read 是当前仅有的广播回复操作，按配置时隙排序。
- 地址 `152` 的单播 Save NVM 仅在 Flash 实际保存完成后回复成功或 Result Fail。
- 输出已使能或存在使能中的活动/待执行命令时，Save NVM 返回 Access Error，必须先停机。
- 周期 Sync Write 不逐节点 ACK；主站写入 Command Sequence 后，通过 Sync Read 读取 Applied Sequence、Last Command Result 和实时状态完成确认。
- 私有指令 `0xA0 TK Sync Control` 支持最多 8 节点的原子广播控制，并由节点按记录顺序分时返回等结构 ACK。V1 只支持“下一本地控制更新点生效”，不承诺多节点控制周期相位同步。
- 广播 Ping、Sync Read 和 `0xA0` 的回复等待使用 TIM1 比较中断与 UART TX DMA，不在 UART RX 回调中忙等。

完整事务矩阵、错误码和周期命令格式见 [control-table.md](control-table.md#response--ack-policy)。

## 安全语义

- 上电可从 NVM 恢复控制源选择，但不会恢复任何活动命令、手动使能或软使能状态。
- Ping、Read 和参数轮询不刷新 Serial Watchdog；只有成功接受的新控制命令刷新。
- Fail-safe Policy=2 会锁存 Watchdog Fault，同时允许有效外部 PWM 接管；欠压/超温保护仍禁止所有输出。
- 写控制源不会自动使能，控制源切换会撤销当前使能。
- 清故障写入不会使能；必须随后单独显式写使能位。
- 串口看门狗仅在 DYNAMIXEL 串口控制且输出已使能时计时。
- 超时执行 NVM 配置的失联策略，默认关闭输出。
- 多圈运行计数和运动目标不写入 NVM。
- 力矩模式使用地址 `210` 的轴端目标；`250..344` 的电机/机械/电流保护参数可保存，地址 `248` 的运行期绕组温度不保存。
- `Kt/Ke/端电阻/编码器分辨率/力矩电流上限` 任一无效时，力矩模式禁止输出并上报模型配置故障。
- 模型有效时，速度/位置模式采用“串级反馈 + 惯量/摩擦前馈”；未配置有效 `Kt` 时仅这两个模式兼容回退到原速度 PI 直接输出电流路径，力矩模式不会回退。
- CRSF 使用独立帧看门狗；DYNAMIXEL 事务不会刷新它。
- 电流保护分三级：地址 `338` 为 INA181 测量边界的单 PWM 周期滑行；地址 `336/344` 为周期平均模型电流的持续过载门槛/确认时间，PWM 冷却后自动重试，串口/CRSF 需重新使能；地址 `340/342/344` 另用于速度/位置模式的延时堵转保护。当前固件不生成 `0x000B` 锁存故障。
- 诊断模式 `Servo Mode=4` 将地址 `20` 解释为直通占空比并在驱动层钳位到 `-1000..1000 permille`，只用于空载机械标定；它不经过电流/力矩模型和 `SpeedMax`，但仍受欠压、过温、编码器有效性、过流保护和串口看门狗约束，主站必须另设独立速度停止阈值。

## 主站最小流程

1. 单节点 `Ping` 确认 ID。
2. `Read` 地址 `0..3` 确认型号、固件和协议版本。
3. `Write` 地址 `16` 为 `1`，切换到 DYNAMIXEL 串口控制；此时仍未使能。
4. 写地址 `17/20/22/26` 设置模式和目标。
5. 最后单独写地址 `18` 为 `0x0001` 显式使能。
6. 运行期间周期性发送新的有效控制命令，间隔必须小于地址 `6` 的看门狗时间；`Read/Ping` 不算刷新。
7. 停止时先写安全目标，再写地址 `18` 为 `0`。

控制表详见 [control-table.md](control-table.md)。
