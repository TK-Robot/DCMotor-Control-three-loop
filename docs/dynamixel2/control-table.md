# TK Servo Control Table

所有多字节值均为小端序。`RW/NVM` 表示写入运行参数后，需向地址 `152` 写 `1` 才保存；运行目标、使能状态和多圈计数不保存。

| 地址 | 长度 | 类型 | 权限 | 名称 | 说明 |
| ---: | ---: | --- | --- | --- | --- |
| 0 | 2 | U16 | RO | Model Number | 型号，当前 `0x0001` |
| 2 | 1 | U8 | RO | Firmware Version | 固件版本 |
| 3 | 1 | U8 | RO | Protocol Version | 固定为 `2` |
| 4 | 1 | U8 | RW/NVM | Node ID | `1..252` |
| 5 | 1 | U8 | RW/NVM | Baud Code | `2/3/4` 对应 115200/1M/2M |
| 6 | 2 | U16 | RW/NVM | Serial Watchdog | 毫秒；`0` 禁用 |
| 8 | 1 | U8 | RW/NVM | Node Position | 广播 Ping 回复顺序，从 `1` 开始 |
| 9 | 2 | U16 | RW/NVM | Reply Slot Width | 响应时隙宽度下限，us；范围 `50..8000`，实际值会按响应包最坏填充后的线速时长自动增大 |
| 10 | 2 | U16 | RW/NVM | Acceleration Limit | 速度规划加速度；单位 `1000 count/s²`，范围 `1..65535`，默认 `60` 表示 `60000 count/s²`；`65535` 在当前速度范围内等效为不限制 |
| 12 | 1 | U8 | RW/NVM | Position Deadband | 位置误差绝对值不大于该值时停止位置环并清除位置积分；单位 `count`，范围 `1..255`，默认 `16` |
| 16 | 1 | U8 | RW/NVM | Control Source | `0=关闭`，`1=DYNAMIXEL`，`2=PWM输入`，`3=CRSF`；切换始终撤销当前使能 |
| 17 | 1 | U8 | RW | Servo Mode | `0=混合电流`，`1=速度`，`2=位置`，`3=估算力矩`；受单向采样限制，模式0/3不是四象限实测电流闭环 |
| 18 | 2 | U16 | RW | Control Word | bit0 使能，bit1 使用 Execute Tick 调度，bit2 清故障，bit3 位置多圈模式；读回值仅反映 bit0 |
| 20 | 2 | I16 | RW | Target Current | mA |
| 22 | 4 | I32 | RW | Target Velocity | 编码器计数/秒，counts/s |
| 26 | 4 | I32 | RW | Target Position | bit3=0 时为单圈位置（低14位，默认、最短路径）；bit3=1 时为有符号多圈累计位置，count |
| 30 | 4 | U32 | RW | Execute Tick | 绝对 1 ms 时基；`0` 是回绕后的合法时刻，不代表立即执行 |
| 34 | 2 | U16 | RW | Command Sequence | 主站周期命令序号，按模 `65536` 递增 |
| 36 | 2 | U16 | RO | Applied Sequence | 最近已生效命令的 Command Sequence |
| 38 | 1 | U8 | RO | Last Command Result | 最近命令映像的标准 Status Error；`0` 表示成功 |
| 39 | 1 | U8 | RO | Reserved | 固定读取为 `0`，用于保持推荐反馈映像连续 |
| 40 | 2 | U16 | RO | Status Word | 位定义见下文 |
| 42 | 2 | U16 | RO | Fault Code | 枚举见下文 |
| 44 | 2 | I16 | RO | Logical Current | PA0 约 26 µs 硬件过采样窗口的桥侧平均电流，经低通后由目标方向补充符号；样本过期返回 `0`。公共低侧分流无法测到制动/续流回路，因此不宣称为四象限绕组电流 |
| 46 | 4 | I32 | RO | Actual Velocity | 编码器计数/秒，counts/s |
| 50 | 4 | I32 | RO | Actual Position | 原点偏移后的单圈位置，`0..16383` count |
| 54 | 4 | I32 | RO | Multi-turn Position | 有符号多圈位置，count；运行期累计，不写 NVM |
| 58 | 2 | I16 | RO | Drive Output | PWM 满量程千分比，`-1000..1000` |
| 60 | 2 | U16 | RO | Supply Voltage | mV |
| 62 | 1 | I8 | RO | Temperature | 摄氏度 |
| 64 | 15 | 混合 | RW/NVM | Current PI | 可观测电动区的 1 kHz 电压修正；反馈为地址 `346` 的硬件时间平均桥电流，缺少新样本、换向和再生区冻结 |
| 80 | 15 | 混合 | RW/NVM | Velocity PID | 见 PID 子结构 |
| 96 | 15 | 混合 | RW/NVM | Position PID | 见 PID 子结构 |
| 112 | 1 | I8 | RW/NVM | Temperature Limit | `20..85 C` |
| 114 | 2 | U16 | RW/NVM | Speed Limit | `1000..65535 counts/s`；当前为兼容保留的 U16 字段 |
| 116 | 1 | U8 | RW/NVM | PWM Mode | `2=慢衰减`，`3=快衰减`，`4=受限自动混合衰减` |
| 117 | 1 | Bool | RW/NVM | Encoder Direction | 停机时写入 |
| 118 | 2 | U16 | RW/NVM | Encoder Offset | `0..16383` |
| 120 | 1 | U8 | RW/NVM | Fail-safe Policy | `0=关闭输出`，`1=制动`，`2=回退PWM` |
| 122 | 4 | U32 | RO | Current Tick | 从协议初始化开始累计的 1 ms 时基 |
| 126 | 2 | U16 | RO | PWM Input Low Width | 最近捕获的 PWM 低电平宽度，µs；仅观测，不保存 |
| 128 | 2 | U16 | RO | Last Diagnostic | 枚举见下文 |
| 130 | 4 | U32 | RO | Diagnostic Count | 通信诊断总数 |
| 134 | 4 | U32 | RO | UART Error Count | UART 错误数 |
| 138 | 4 | U32 | RO | CRC Error Count | CRC 错误数 |
| 142 | 4 | U32 | RO | Bad Packet Count | 坏包数 |
| 146 | 4 | U32 | RO | RX Packet Count | 有效接收包数 |
| 150 | 1 | U8 | WO | Clear Diagnostics | 写 `1` 清零诊断 |
| 152 | 1 | U8 | WO | Save NVM | 写 `1` 保存配置；单播 Status 在 Flash 完成后返回 |
| 154 | 2 | U16 | RW/NVM | Deceleration Limit | 速度规划减速度；单位 `1000 count/s²`，范围 `1..65535`，默认 `60` 表示 `60000 count/s²`；`65535` 在当前速度范围内等效为不限制 |
| 160 | 1 | U8 | RW/NVM | CRSF Position Channel | `0=禁用`，`1..16`；位置目标通道 |
| 161 | 1 | U8 | RW/NVM | CRSF Center Channel | `0=禁用`，`1..16`；低于阈值后再次达到阈值时记录当前位置为中心参考 |
| 162 | 1 | U8 | RW/NVM | CRSF Enable Channel | `0=无独立使能通道`，`1..16` |
| 163 | 1 | Bool | RW/NVM | CRSF Auto Enable | 是否允许 CRSF 有效时按当前使能通道状态直接进入 Arm Tracking |
| 164 | 2 | U16 | RO | Reserved | 保留，固定读取 `0` |
| 166 | 2 | U16 | RW/NVM | CRSF Channel Min | 通道最小值，默认 `172` |
| 168 | 2 | U16 | RW/NVM | CRSF Channel Center | 通道中值，默认 `992` |
| 170 | 2 | U16 | RW/NVM | CRSF Channel Max | 通道最大值，默认 `1811`；要求 Min < Center < Max <= 2047 |
| 172 | 2 | U16 | RW/NVM | CRSF Center Trigger | 中心通道触发阈值，默认 `1200` |
| 174 | 2 | U16 | RW/NVM | CRSF Enable Threshold | `>=` 阈值请求使能，`<` 阈值立即撤销使能；默认 `1200` |
| 176 | 2 | U16 | RO | Reserved | 保留，固定读取 `0` |
| 178 | 2 | U16 | RW/NVM | CRSF Arm Current Limit | 软使能跟随阶段电流限制，mA，`1..30000` |
| 180 | 4 | U32 | RW/NVM | CRSF Arm Speed | 软使能跟随阶段速度限制，counts/s，`1..1000000` |
| 184 | 2 | U16 | RW/NVM | CRSF Arm Follow Error | 进入 Active 的位置误差阈值，count |
| 186 | 2 | U16 | RW/NVM | CRSF Arm Timeout | 软使能跟随超时，ms，`100..10000` |
| 188 | 2 | U16 | RW/NVM | CRSF Watchdog | CRSF 有效帧超时，ms，`20..2000` |
| 190 | 4 | I32 | RW/NVM | CRSF Negative Position Limit | 位置通道负端对应的有符号多圈位置，count |
| 194 | 4 | I32 | RW/NVM | CRSF Positive Position Limit | 位置通道正端对应的有符号多圈位置，count；必须大于负端 |
| 198 | 2 | U16 | RO | CRSF Status | 位定义见“CRSF 输入”章节 |
| 200 | 2 | U16 | RO | CRSF Raw Position | 当前位置通道原始 11-bit 值 |
| 202 | 2 | U16 | RO | CRSF Raw Enable | 当前使能通道原始值；未配置时为 `0` |
| 204 | 2 | U16 | RO | CRSF Raw Center | 当前中心通道原始值；未配置时为 `0` |
| 206 | 4 | I32 | RO | CRSF Center Reference | 当前中心参考的有符号多圈位置，count；不写 NVM |
| 210 | 4 | I32 | RW | Target Shaft Torque | 轴端负载力矩目标，uN·m；不保存 |
| 214 | 4 | I32 | RO | Target Electromagnetic Torque | 力矩或模型辅助速度/位置模式中，机械补偿后的电磁力矩目标，uN·m |
| 218 | 4 | I32 | RO | Actual Electromagnetic Torque | 根据逻辑电流和 Kt 估算，uN·m |
| 222 | 4 | I32 | RO | Estimated Shaft Load Torque | 扣除惯性和内部损耗后的轴端负载力矩估算，uN·m |
| 226 | 4 | I32 | RO | Inertia Torque | 总惯量与实测加速度对应的力矩，uN·m |
| 230 | 4 | I32 | RO | Internal Loss Torque | 库仑摩擦与粘性阻尼合计，uN·m |
| 234 | 4 | I32 | RO | Required Motor Voltage | 当前电流、转速下模型所需端电压，mV |
| 238 | 4 | I32 | RO | Back EMF | 当前转速对应的反电动势，mV |
| 242 | 4 | I32 | RO | Available Current | 当前母线电压和转速下同方向可实现电流，mA |
| 246 | 2 | U16 | RO | Torque Model Status | 位定义见“力矩模型状态” |
| 248 | 2 | I16 | RW | Motor Winding Temperature | 绕组温度或热模型估计值，`-40..200 °C`；运行期可更新，不保存 |
| 250 | 2 | U16 | RW/NVM | Torque Encoder Counts/Rev | 机械模型使用的每圈编码器计数，`1..65535` |
| 252 | 2 | U16 | RW/NVM | Torque Current Limit | 模型力矩通路电流上限，`1..30000 mA`；力矩模式下 `0` 表示配置无效，速度/位置回退路径不使用该限制 |
| 254 | 2 | I16 | RW/NVM | Motor Reference Temperature | 电机参数参考温度，`-40..200 °C` |
| 256 | 4 | U32 | RW/NVM | Torque Constant Kt | `uN·m/A`，范围 `1..1000000` |
| 260 | 4 | I32 | RW/NVM | Kt Temperature Coefficient | `ppm/°C`，范围 `-10000..10000`；`0` 禁用修正 |
| 264 | 4 | U32 | RW/NVM | Back-EMF Constant Ke | `uV/rpm`，范围 `1..10000000` |
| 268 | 4 | U32 | RW/NVM | Terminal Resistance | 参考温度下端电阻，`mΩ`，范围 `1..10000000` |
| 272 | 2 | U16 | RW/NVM | Resistance Temperature Coefficient | `ppm/°C`，范围 `0..10000`；铜绕组可从约 `4000` 开始标定 |
| 274 | 2 | U16 | RW/NVM | Brush Drop | 正负电刷总压降，`0..5000 mV` |
| 276 | 4 | U32 | RW/NVM | Total Inertia | 转子加折算负载总惯量，`ug·cm²`，范围 `0..100000000` |
| 280 | 4 | U32 | RW/NVM | Coulomb Friction | 库仑摩擦幅值，`uN·m`，范围 `0..10000000` |
| 284 | 4 | U32 | RW/NVM | Viscous Friction | 粘性阻尼系数，`nN·m/rpm`，范围 `0..10000000` |
| 288 | 2 | U16 | RW/NVM | Friction Deadband | 摩擦方向判定速度死区，counts/s |
| 334 | 2 | U16 | RW/NVM | Motor Inductance | 绕组等效电感标定记录，`1..10000 uH`；当前没有可信在线标定状态，因此不参与 PWM 限幅 |
| 336 | 2 | U16 | RW/NVM | Current Peak Limit | 硬件时间平均桥电流低通后的非锁存削波阈值，`100..1829 mA` 且必须小于地址 `338`；默认 `1500 mA` |
| 338 | 2 | U16 | RW/NVM | Current Absolute Limit | 硬件时间平均桥电流低通后的持续绝对过流锁存阈值，必须大于地址 `336` 且不超过 `1830 mA`；默认 `1750 mA` |
| 340 | 2 | U16 | RW/NVM | Stall Current Threshold | 堵转判定电流门槛，`50..1500 mA`；默认 `300 mA` |
| 342 | 2 | U16 | RW/NVM | Stall Speed Threshold | 堵转判定速度门槛，`10..65535 counts/s`；默认 `300 counts/s` |
| 344 | 2 | U16 | RW/NVM | Stall Confirm Time | 堵转条件连续确认时间，`500..10000 ms`；默认 `3000 ms` |
| 346 | 2 | I16 | RO | Current Cycle Average | PA0 约 26 µs 硬件过采样窗口再经低通的桥侧电流，电流 PI 主反馈，mA；不是四象限绕组电流 |
| 348 | 2 | U16 | RO | Peak Chop Events | 非锁存峰值削波累计次数，饱和计数 |
| 350 | 2 | U16 | RO | Stall Elapsed | 当前连续堵转条件累计时间，条件解除即清零，ms |
| 352 | 2 | U16 | RW/NVM | Low-speed Compensation Cutoff | `0=关闭`；`500..5000 counts/s`，低于一半全量补偿，之后线性渐退至零 |
| 354..369 | 16×1 | I8 | RW/NVM | Forward Phase Current Trim | 正转 16 区电流幅值偏差，`-100..100 mA`；每区 `1024 count` |
| 370..385 | 16×1 | I8 | RW/NVM | Reverse Phase Current Trim | 反转 16 区电流幅值偏差，`-100..100 mA`；每区 `1024 count` |

地址 `164..165` 和 `176..177` 为保留字节，读取为 `0`，写入返回 Access Error。CRSF 配置只能在输出关闭且不处于 Arm Tracking/Active 时写入。除地址 `248` 的运行期绕组温度外，力矩模型和低速补偿配置也只能停机写入。参数写入不会自动使能，保存仍需向地址 `152` 写 `1`。

低速补偿表表示相对于全局库仑/粘性摩擦模型的周期电流偏差，不是绝对电流命令。固件按修正后的单圈位置在相邻区间间线性插值；只在速度/位置模式、速度环电流与规划运动同向时叠加。超速或反向制动、零速保持、电流模式和力矩模式均不使用该表。补偿后的电流仍受命令电流限制、Torque Current Limit 和全部硬件保护约束。标定时应先把地址 `352` 写 `0`，再写两张表，最后写截止速度；这样不会在半张表状态下启用。

## 力矩模型状态

| 位 | 掩码 | 含义 |
| ---: | ---: | --- |
| 0 | `0x0001` | Kt 有效，可进行电流/电磁力矩换算 |
| 1 | `0x0002` | Ke、端电阻和编码器分辨率有效，电压能力模型有效 |
| 2 | `0x0004` | 机械模型有效 |
| 3 | `0x0008` | 当前力矩命令因供电电压能力不足被限幅 |
| 4 | `0x0010` | 当前模型工作点所需电压超过母线电压 |
| 5 | `0x0020` | 已选择力矩模式，但必要参数或电流上限无效，输出被禁止 |
| 6 | `0x0040` | INA181 在当前PWM有效导通区获得了电流幅值样本 |
| 7 | `0x0080` | 地址44仅有电流幅值实测，方向由目标推断，不是完整双向实测 |
| 8 | `0x0100` | 当前没有足够新的合格PWM导通窗口幅值样本 |

## 电流环状态（地址 306）

| 位 | 掩码 | 含义 |
| ---: | ---: | --- |
| 0 | `0x0001` | 电流 PI 正在可观测区运行 |
| 1 | `0x0002` | 当前电流样本有效 |
| 2 | `0x0004` | 地址44的符号由目标方向推断 |
| 3 | `0x0008` | 目标非零但电流 PI 冻结 |
| 4 | `0x0010` | 目标方向变化，电流 PI 已复位 |
| 5 | `0x0020` | 最终模型电压达到 PWM 限幅 |
| 6 | `0x0040` | 同步峰值削波正在生效 |
| 7 | `0x0080` | 不可观测再生象限使用 Ke/R 限幅的刹车/滑行 PWM，电流 PI 冻结 |

## 混合串级控制层级

位置和速度模式采用串级反馈加模型电压执行：

```text
位置 P/PD -> 规划速度 -> 速度 PI -> 轴端反馈力矩
                                      + 规划惯性力矩
                                      + 库仑/粘性摩擦前馈
                                      -> 电磁力矩目标
                                      -> Kt 与绕组温度换算目标电流
                                      + 正/反向位置分区低速电流偏差
                                      -> 电压能力和电流上限
                                       -> R·I + Ke·ω 电压模型 -> PWM
```

为了兼容已经保存的 PID 参数和现有控制表，速度 PI 的寄存器输出仍按 `mA`
等效量缩放；固件在速度环边界通过温度修正后的 `Kt` 将它转换为轴端反馈
力矩。模型有效时，地址 `214` 会反映叠加机械前馈后的电磁力矩目标。
速度/位置命令存在但轴仍近似静止时，速度 PI 的积分项根据速度误差逐步增加
电流，越过不同齿轮位置实际所需的启转阈值后再随速度误差连续回退。该路径
始终受命令限流和 `Torque Current Limit` 约束，不使用固定启转电流常数。
在规划速度不低于三倍堵转速度门槛、实测速度仍处于堵转速度区时，固件仅临时
提高积分增益；积分状态不跳变，恢复转动后自动回到配置的正常 `Ki`。

若速度/位置模式尚未配置有效 `Kt`，固件保留旧版“速度 PI 直接输出目标电流”
路径，地址 `214` 返回 `0`。该兼容回退不会放宽力矩模式：力矩模式仍要求
`Kt/Ke/端电阻/编码器分辨率/Torque Current Limit` 均有效。

力矩模式按以下关系计算：

```text
轴端目标力矩 + 惯性力矩 + 内部损耗力矩
    -> 电磁力矩目标
    -> Kt 与绕组温度换算目标电流
    -> Ke、转速、端电阻、电刷压降和母线电压限制
    -> Torque Current Limit 限制
    -> 电压/PWM执行与导通区峰值保护
```

这些力矩均为模型估算量，不等同于力矩传感器测量值。Kt、摩擦和折算负载惯量的误差会直接进入轴端估算结果。

电流底层先采用 `V = R(T)·I + Ke·ω + Vbrush` 计算 PWM 前馈。DRV8837 公共低侧
分流电阻只能观测桥侧单向电流，无法直接看到制动续流回路。TIM3 CH4 固定在 PWM
周期中点触发，PA0 使用 16 次硬件过采样；一次结果跨越约 26 µs，把窄 PWM 脉冲、
DRV8837 开关延迟和 INA181 的 350 kHz 响应做时间平均。该路径与最初稳定的 40/110 mA
实测版本一致，避免把某个亚微秒开关时刻误报为 1.8 A。

任何非零 PWM 请求都可产生时间平均样本。电流 PI 使用地址 `346`，在样本年龄不超过 3 ms、
目标方向与模型驱动电压同向时运行；首次获得反馈先经过两个 1 ms 稳定更新。
换向先强制滑行约 `1.25 ms`，期间旧方向残余样本仍参与绝对过流保护，但不累计
软峰值削波。换向或样本过期时冻结积分；不可观测的再生象限依据 Ke/R 将目标制动电流换算为
同相刹车/滑行 PWM，在电机绕组内耗散能量，避免仅滑行造成速度极限环；回到可观测电动象限后自动重新闭合电流 PI。PI 修正也不会把
驱动电压反向。负电流方向仍由目标推断，所以系统不是硬件四象限电流传感器。

滤波后的时间平均桥电流达到地址 `336` 时仅跳过一个 PWM 周期并累计地址 `348`，不锁存故障；
达到更高的地址 `338` 才立即触发 `0x000B` 锁存停机。堵转则是独立的慢保护：
仅速度/位置模式在运动指令存在、实际速度低且周期平均电流高时累计，连续达到
地址 `344` 才触发 `0x000C`；电流/力矩保持模式不会因零速本身被判为堵转。

为防止旧NVM中曾作为兼容保留值的参数在升级后意外接管输出，电流反馈只接受
`Kp<=2000`、`Ki<=500`、`Kd=0`、`0<=Integral Max<=30000`、
`Output Max<=2000 mV` 且 `Output Min<=Output Max` 的配置；通信写入超出范围时
返回 Data Range Error，加载到越界的旧NVM配置时恢复安全默认值。默认值为
`Kp=250`、`Ki=30`、`Kd=0`、`Integral Max=16000`、`Output Max=500 mV`。

### PWM 自动混合衰减

地址 `116=4` 不再等同于固定慢衰减。稳定或尚无合格样本时从慢衰减开始；目标电流方向改变或幅值明显下降时，进入约 `2.5 ms` 的快衰减保持段。时间平均桥电流超过目标 `50 mA` 会触发快衰减，回落到目标以上 `20 mA` 内才返回慢衰减。

地址 `334` 保留电感标定值，但当前没有“已标定”状态，固件不会再把默认 `10 uH` 代入 `di/dt≈V/L` 截断 PWM。这样避免未实测参数静默限制力矩；完成在线电感辨识并增加可信状态前，不恢复该模型限幅。

该逻辑只选择 DRV8837 的续流方式，不替代电流 PI。自动衰减和电流 PI 都使用同一条时间平均、低通后的桥电流，模式 `2/3` 始终保持固定慢/快衰减，便于诊断和标定。

电流采样触发点不再随慢/快衰减和占空比移动。模式或方向切换时仍会重新初始化滤波上下文，避免把不同桥状态的时间平均值混合；地址 `346` 不再二次乘占空比或使用未标定的 R/L 重构。

新的桥模式或方向下，第一个合格硬件平均样本只用于初始化对应低通滤波器，不参与 PI、削波或绝对过流判断。地址 `294` 和 20 ms 统计窗口保留每次硬件平均结果；控制与保护使用地址 `296` 的稳定滤波路径。

速度反馈始终由 MT6701 位置和实际采样周期经过同一个测速器得到，不随衰减模式改变。衰减模式改变的是执行器力矩响应和电流纹波，而不是编码器测速比例。实机带时间戳的位置差分表明原 alpha-beta 观察器会把约 `3～4 kcps` 的实际窗口速度放大为 `6～7 kcps` 尖峰，进而引发驱动/制动切换。当前先按每次位置增量除以该次实际采样周期得到瞬时速度，再使用约 `16 ms` 等效固定点 IIR 平滑；采样周期突变不会被旧周期平均值放大，也没有非重叠窗口的阶梯保持。

## Response / ACK Policy

本设备不开放可写的 Status Return Level，固定采用等级 `2` 的单播回复策略。所有回复均为标准 DYNAMIXEL Protocol 2.0 Status Packet：

```text
FF FF FD 00 | Device ID | Length | 55 | Error | Parameters... | CRC16
```

不引入 `AA 55`、自定义 `RESULT` 字段或包头级 `SEQ`。周期序号通过控制表地址 `34/36` 传递，因此仍可使用标准 DYNAMIXEL SDK。

| 操作 | 单播行为 | 广播 `0xFE` 行为 |
| --- | --- | --- |
| Ping | 必须返回型号、固件版本和 `Error=0` | 所有节点按 Node Position/Reply Slot 排队回复 |
| Read | 必须返回 `Error + Data`；失败时无 Data | 不执行，不回复 |
| Write | 必须返回 `Error`，该 Status 即 ACK | 执行匹配写入，不回复 |
| Reg Write | 必须确认“已登记”；数据尚未生效 | 可登记，不回复 |
| Action | 必须确认已执行；Save NVM 除外，见下文 | 执行已登记写入，不回复 |
| Sync Write | 不适用 | 匹配节点执行，不回复 |
| Sync Read | 不适用 | ID 列表中的节点按列表顺序返回 `Error + Data` |
| 未支持指令 | 返回 Instruction Error | 不回复 |

广播包即使长度、地址或数据错误也不返回 Status Packet，避免多节点同时发送造成总线冲突。CRC 错误包同样不回复，因为完整 ID 和包体尚未通过校验；错误只记录到 Last Diagnostic 和 CRC Error Count。

### Status Packet Error

`Error` 的 bit7 是硬件告警位：`Fault Code != 0` 时置 `1`，主站随后应读取 Fault Code；bit0..6 仍表示当前事务的标准错误编号：

| 值 | 名称 | 本固件使用场景 |
| ---: | --- | --- |
| `0x00` | None | 指令成功，或 Read 成功并附带数据 |
| `0x01` | Result Fail | NVM 擦写失败、重复的未完成 Save NVM 等无法归类的执行失败 |
| `0x02` | Instruction Error | 单播收到未实现指令 |
| `0x03` | CRC Error | 标准保留语义；接收 CRC 错误时本设备不回包 |
| `0x04` | Data Range Error | 值超出允许范围 |
| `0x05` | Data Length Error | 参数数量或对象宽度错误 |
| `0x06` | Data Limit Error | 标准保留语义；当前未使用 |
| `0x07` | Access Error | 地址不存在、写只读对象或状态不允许写入 |

### Save NVM 完成确认

地址 `152` 是特殊的延迟事务：Flash 擦写不在 UART 回调中执行。单播 Write 写 `1` 后，从站先把请求交给主循环，只有 `NvmParam_Save()` 真正完成后才发送 Status Packet：

- `Error=0x00`：参数已经持久化，或内容与最新记录相同，无需重复擦写。
- `Error=0x01`：Flash 擦除、编程或参数保存失败。
- 当前输出、活动命令或待生效命令处于使能状态时拒绝保存，并返回 `Access Error=0x07`。主站必须先显式停机并确认 `Status Word.Output Enabled=0`。
- 前一笔单播保存尚未完成时再次请求，返回 `0x01`。
- 广播或 Sync Write 保存仍不回复；需要可靠确认时必须对每个节点执行单播 Save NVM。

Reg Write 地址 `152` 的 Status 只表示保存命令已登记；后续单播 Action 的 Status 才表示 Flash 保存的最终结果。

### 周期广播的序号确认

推荐主站用 Sync Write 从地址 `16` 写入完整 `20` 字节命令映像：

| 偏移 | 地址 | 长度 | 字段 |
| ---: | ---: | ---: | --- |
| `+0` | 16 | 1 | Control Source |
| `+1` | 17 | 1 | Servo Mode |
| `+2` | 18 | 2 | Control Word |
| `+4` | 20 | 2 | Target Current |
| `+6` | 22 | 4 | Target Velocity |
| `+10` | 26 | 4 | Target Position |
| `+14` | 30 | 4 | Execute Tick |
| `+18` | 34 | 2 | Command Sequence |

旧的地址 `16`、长度 `14` 命令映像继续兼容，但不携带 Execute Tick 和序号。主站应令 Command Sequence 按模 `65536` 递增；回绕后从 `65535` 变为 `0`。

完整 `20` 字节命令映像按一次事务校验和提交，不会逐字段产生中间状态。固件仅保留一个待生效命令槽：

- 与待生效序号或最近已应用序号相同的完整命令视为重传，幂等返回成功，不重复应用，也不刷新 Serial Watchdog。
- 待生效槽已占用时，收到不同序号的完整命令返回 `Result Fail=0x01`，不会覆盖原命令。

目标轴端力矩位于地址 `210`，为保持旧命令映像兼容，不属于地址 `16` 开始的 `20` 字节映像。进入力矩模式时应先保持输出关闭，写入并等待地址 `210` 的目标生效，再发送 `Servo Mode=3` 的命令映像使能输出。已经处于力矩模式时，可直接用单播 Write 或 Sync Write 更新地址 `210`；每次写入作为普通待执行命令在下一个控制节拍生效。
- Control Word bit1=`0` 时完整映像在下一个 1 ms 协议周期应用，Execute Tick 字段被忽略。
- Control Word bit1=`1` 时在 Execute Tick 到期后应用；地址 `0` 是合法绝对时刻。
- Control Word bit3=`0`（默认）时，位置环将 Target Position 解释为 `0..16383` 的单圈绝对位置，并持续按环形误差选择最短路径跨越零点；bit3=`1` 时直接跟踪 I32 多圈累计目标。PWM 输入固定使用单圈，CRSF 固定使用多圈。
- 长度 `14` 的兼容映像始终立即提交，不参与序号去重或绝对时刻调度。

周期控制后的确认使用 Sync Read：从地址 `36` 读取 `27` 字节。返回数据依次包含：

```text
Applied Sequence U16
Last Command Result U8
Reserved U8
Status Word ... Temperature（地址 40..62）
```

`Applied Sequence == 本周期 Command Sequence` 且 `Last Command Result == 0` 表示该命令映像已经在控制周期中生效。序号不相等表示命令尚未到 Execute Tick、未收到或校验失败；具体驱动故障仍读取 Fault Code，不能只看 Last Command Result。

## TK Sync Control（私有指令 `0xA0`）

`0xA0` 保留 DYNAMIXEL Protocol 2.0 包头、Length、Byte Stuffing 和 CRC，仅定义私有 Instruction 与 Parameters。请求必须使用广播 ID `0xFE`：

使用单播 ID 发送 `0xA0` 时，从站返回 `Instruction Error`，不会提交控制命令。

```text
Sequence       U16
Execute Mode   U8
Execute Value  U32
ACK Mask       U16
Node Count     U8

repeat Node Count:
    Node ID          U8
    Control Source   U8
    Servo Mode       U8
    Control Word     U16
    Target Current   I16
    Target Velocity  I32
    Target Position  I32
```

参数总长度为 `10 + 15 × Node Count`，V1 的 Node Count 范围为 `1..8`。节点记录顺序同时定义回复顺序，首条记录的回复索引为 `0`。

### 全局校验

以下情况使整包无效，所有节点均不执行、不回复：

- CRC、包长或 Byte Stuffing 无效；
- Node Count 超出 `1..8`，或参数长度不等于 `10 + 15 × Node Count`；
- Node ID 不在 `1..252`，或同一包存在重复 Node ID；
- ACK Mask 的 bit10..bit15 非零。

所有节点必须完成全局校验后才能提交自己的记录。没有出现在记录中的节点保持静默。

### Execute Mode V1

V1 只支持 `Execute Mode=0 (NEXT_UPDATE)`：通过校验的命令进入本节点唯一 pending 槽，并在下一次本地 1 ms 协议更新点生效。Execute Value 被保留但忽略；节点记录中的 Control Word bit1 必须为 `0`。该模式不表示多节点控制周期已经同步。

其他 Execute Mode 对包含本节点的结构有效请求返回 `Data Range Error`，不更新 Accepted Sequence，也不刷新 Serial Watchdog。

### 序号与 pending 槽

- 新序号成功进入 pending 槽后更新 Accepted Sequence，并刷新 Serial Watchdog。
- 与最近 Accepted Sequence 相同的请求是幂等重传：返回 ACK，但不重复执行、不覆盖 pending、不刷新 Watchdog，并在 ACK State 中置 Duplicate。
- pending 已被不同序号占用时返回 `Result Fail`，保留旧 Accepted/Applied/Result。
- Applied Sequence 仅在控制周期真正接管命令时更新。
- Last Command Result 只描述 Applied Sequence；被拒绝的新命令不能覆盖它。

### A0 ACK

A0 ACK 仍是标准 Status Packet。Parameters 固定从以下 6 字节开始：

```text
Accepted Sequence   U16
Applied Sequence    U16
Last Command Result U8
ACK State           U8
```

ACK State：bit0=`AcceptedValid`，bit1=`AppliedValid`，bit2=`Pending`，bit3=`Duplicate`，bit4..bit7 保留为零。启动后的 Sequence 数值初始化为零，但对应 Valid 位为零，因此不会与合法序号 0 混淆。

ACK Mask 决定随后附加的字段；无论事务成功或失败，同一 A0 请求中的所有节点都按同一个 Mask 返回相同字段结构：

| bit | 字段 | 长度 |
| ---: | --- | ---: |
| 0 | Status Word | 2 |
| 1 | Fault Code | 2 |
| 2 | Actual Current | 2 |
| 3 | Actual Velocity | 4 |
| 4 | Actual Position | 4 |
| 5 | Multi-turn Position | 4 |
| 6 | Drive Output | 2 |
| 7 | Supply Voltage | 2 |
| 8 | Temperature | 1 |
| 9 | Current Tick | 4 |

可选字段严格按 bit0 到 bit9 顺序编码。状态来自控制循环发布的双缓冲快照；协议收到 A0 后立即复制当前已发布快照，后续编码和 DMA 不再读取变化中的控制变量。

### 非阻塞回复时隙

完整帧接收事件记录 TIM1 的 1 MHz 时间戳。节点计算：

```text
wire_bytes = 11 + parameter_bytes + floor((parameter_bytes + 2) / 3)
packet_time = ceil(wire_bytes × 10 × 1000000 / baud)
slot_width = max(ReplySlotUs, packet_time + 50 us)
reply_deadline = packet_end + 50 us + reply_index × slot_width
```

`wire_bytes` 按最坏 Byte Stuffing 计算，因此错误 ACK 与正常 ACK 即使实际填充字节不同也不会侵入下一时隙。等待由 TIM1 Compare 中断完成，到期后启动 UART TX DMA；UART RX 回调不执行微秒忙等。当前 16 位回复计时器要求单节点 deadline 不超过 60000 us，超出时记录 TX Drop 并保持静默。

## PID 子结构

三个历史PID参数块继续保持相同布局，以兼容既有控制表和NVM。当前实际使用位置P、速度PI和受限电流PI；电流PI只在硬件可观测区修正模型前馈。

| 偏移 | 长度 | 类型 | 名称 |
| ---: | ---: | --- | --- |
| +0 | 2 | U16 | Kp |
| +2 | 2 | U16 | Ki |
| +4 | 2 | U16 | Kd |
| +6 | 4 | I32 | Integral Max |
| +10 | 2 | U16 | Output Max |
| +12 | 2 | U16 | Output Min |
| +14 | 1 | U8/WO | 写 `1` 复位该环运行状态 |

PID 写入立即作用于运行参数，但不会切换控制源、不会清故障、不会使能输出，也不会自动保存到 NVM。

## Status Word

未列出的位均为保留位，读取为 `0`。

| bit | 掩码 | 名称 | 当前语义 |
| ---: | ---: | --- | --- |
| 0 | `0x0001` | Ready | 协议实例已初始化；当前实现固定为 `1` |
| 1 | `0x0002` | PWM Input Valid | 收到范围有效且未超过 100 ms 的外部 PWM 输入 |
| 2 | `0x0004` | Output Enabled | 当前选定控制源通过保护检查并处于使能状态 |
| 3 | `0x0008` | Fault Present | `Fault Code != 0` |
| 4 | `0x0010` | Protection Inhibit | 欠压或超温保护正在禁止输出 |
| 5 | `0x0020` | Undervoltage | 电源低于 Power Save Voltage，或欠压锁存尚未越过恢复回差 |
| 6 | `0x0040` | Overtemperature | 当前温度高于 Temperature Limit |
| 8 | `0x0100` | PWM Source | 控制源为外部 PWM 输入 |
| 9 | `0x0200` | Serial Source | 控制源为 DYNAMIXEL 串口 |
| 10 | `0x0400` | CRSF Source | 控制源为 CRSF 遥控通道 |
| 11 | `0x0800` | Fault Free | `Fault Code == 0` |
| 12 | `0x1000` | Protocol Active | 协议任务已运行；当前实现固定为 `1` |

bit2 表示经过控制源选择和保护检查后输出已使能，不表示 H 桥当前一定存在非零占空比；零目标和输出死区仍可能使实际输出为零。

## CRSF 输入

DYNAMIXEL 与 CRSF 共用 USART2 和相同的用户配置波特率。调试主站可以交替发送两种协议，但每个帧必须完整发送后才能开始另一帧，禁止字节级交织。接收端按帧头和各自 CRC 分流：

- DYNAMIXEL：`FF FF FD 00`，标准 CRC16；继续用于读取、配置、保存和完整控制。
- CRSF：地址 `0xC8`、Length=`24`、Type=`0x16 RC Channels Packed`、22 字节 16×11-bit 通道、CRC8-D5。

当前只实现 CRSF `RC Channels Packed (0x16)`，不实现链路统计、设备参数、遥测回传或自动波特率。CRSF 从不主动占用 TX；DYNAMIXEL Status Packet 仍由主站事务触发。

选择 `Control Source=3` 后，CRSF 位置通道映射到原点偏移后的有符号多圈位置。标准 `0x16` 通道默认标定为 `172 / 992 / 1811`，最终目标始终限制在地址 `190/194` 的有符号多圈位置范围内。Position Channel 为 `0` 时 CRSF 控制保持关闭。使能语义如下：

1. Enable Channel 为零且 Auto Enable=1：有效 CRSF 帧直接请求 Arm Tracking。
2. Enable Channel 为零且 Auto Enable=0：DYNAMIXEL 写 Control Word bit0 请求 CRSF 手动使能。
3. Enable Channel 非零且 Auto Enable=1：有效 CRSF 帧中该通道 `>= Enable Threshold` 即可请求 Arm Tracking；`<` 阈值立即撤销使能。
4. Enable Channel 非零且 Auto Enable=0：进入 CRSF 控制或 CRSF 恢复时，即使通道已经为高也不自动使能；必须先观察到 `<` 阈值，再观察到新的 `>=` 阈值边沿。DYNAMIXEL 手动使能在此模式下不替代独立使能通道。

Center Channel 只有在先观察到 `< Center Trigger`、随后观察到 `>= Center Trigger` 时触发一次，触发值是当前原点偏移后的内部多圈位置。首次 Arm Tracking 尚无中心参考时也以当前实际位置建立参考，但后续 Arm 不会自动覆盖已建立的参考。

从未使能切入时进入 Arm Tracking：在切入瞬间锁定一次 Arm Target，并用地址 `178/180` 的临时电流和速度限制跟随该目标；Arm 期间实时位置通道变化不会移动这个验证目标。误差进入地址 `184` 后转为 Active，之后才恢复实时位置通道跟随。超过地址 `186` 仍未到位则进入 Arm Failed，必须撤销使能请求后重试。CRSF 帧超过地址 `188` 未更新会立即撤销输出并清除使能边沿资格；恢复后必须重新经过 Arm Tracking。任何 DYNAMIXEL Ping、Read、Write 或 Status 都不会刷新 CRSF Watchdog。

CRSF Status：

| bit | 掩码 | 名称 | 语义 |
| ---: | ---: | --- | --- |
| 0 | `0x0001` | Frame Valid | 最近 RC 帧未超过 CRSF Watchdog |
| 1 | `0x0002` | Position Channel Valid | Position Channel 在 `1..16` |
| 2 | `0x0004` | Enable Channel Valid | 已配置 Enable Channel |
| 3 | `0x0008` | Enable Request | 当前使能条件成立 |
| 4 | `0x0010` | Arm Tracking | 正在限流/限速软跟随 |
| 5 | `0x0020` | Active | 已进入正常位置控制 |
| 6 | `0x0040` | Arm Failed | 软使能跟随超时 |
| 7 | `0x0080` | Timeout | 曾收到 CRSF，但最近帧已超时 |
| 8 | `0x0100` | Center Reference Valid | 已建立中心参考 |

## Fault Code

Fault Code 是需要主站明确清除的锁存驱动故障。当前已实现枚举如下；未列出的数值保留。

| 值 | 名称 | 说明 |
| ---: | --- | --- |
| `0x0000` | None | 无锁存故障 |
| `0x000A` | Serial Watchdog | 已使能串口控制超过 Serial Watchdog 时间未刷新 |
| `0x000B` | Absolute Overcurrent | PWM 同步导通窗口电流达到地址 `338` 的绝对阈值；立即滑行关断并锁存故障 |
| `0x000C` | Stall | 速度/位置模式中，大电流、低实际速度且存在运动指令的条件连续达到地址 `344`；滑行关断并锁存故障 |
| `0x000D` | Encoder Timeout | 速度/位置模式已使能，但 MT6701 反馈无效或超过 `5 ms` 未更新；滑行关断并锁存故障 |

低电压和超温不是锁存 Fault Code，而是实时保护原因。它们置 Status Word bit4，并分别置 bit5/bit6，且无条件撤销输出。欠压使用 `500 mV` 恢复回差；超温在温度不再高于限制值时解除。上位机不得把这些状态伪报为 Serial Watchdog。

## Last Diagnostic

Last Diagnostic 记录最近一次通信层诊断；Diagnostic Count 是饱和累计次数。向地址 `150` 写 `1` 会清除所有通信诊断计数。

| 值 | 名称 | 说明 |
| ---: | --- | --- |
| `0` | None | 尚无通信诊断 |
| `1` | UART Error | HAL UART 或 DMA 错误 |
| `2` | RX CRC | 数据包 CRC 不匹配 |
| `3` | RX Bad Packet | 包头、ID、长度或包体无效 |
| `4` | TX Drop | 当前发送和一级待发槽均占用，响应被丢弃 |
| `5` | Watchdog | 已使能串口控制停止刷新 |

## 速度和位置单位

MT6701 每圈为 `16384 count`。目标速度和实际速度均为有符号 `counts/s`：

```text
rpm = velocity_counts_per_second * 60 / 16384
velocity_counts_per_second = rpm * 16384 / 60
deg/s = velocity_counts_per_second * 360 / 16384
```

Speed Limit 地址 `114` 沿用 U16，因此范围为 `1000..65535 counts/s`，在 `16384 count/rev` 下约为 `3.66..240.00 rpm`。若后续需要超过该上限，必须通过版本化控制表新增 U32 字段，不能在现有地址上静默改变宽度。

位置分辨率为 `360 / 16384 = 0.02197265625 degree/count`。Actual Position 是偏移和方向修正后的单圈位置；Target Position 与 Multi-turn Position 使用同一个多圈 count 坐标。当前多圈计数器为 `int16_t`，因此可反馈范围约为 `-536870912..536870911 count`。

## Drive Output 和 PID 限幅单位

Drive Output 是最终物理执行器指令，`1000` 表示正方向 100% PWM，`-1000` 表示反方向 100% PWM。当前驱动层只按定时器分辨率处理零占空比，不再人为设置10‰或200‰的最小脉冲。

PID 的 Output Max 表示正负对称限幅，实际范围为 `[-Output Max, +Output Max]`。Output Min 表示非零输出的最小绝对值，不是负方向下限。

| 环路 | Output Max/Min 单位 | 备注 |
| --- | --- | --- |
| 位置环 | counts/s | 输出为速度目标；Max 和 Min 均生效 |
| 速度环 | mA 等效量 | 经 Kt 转为反馈力矩；保留该单位用于兼容既有参数；Max 和 Min 均生效 |
| 电流PID | mV | 增益按1000缩放；输出为叠加到R/Ke模型上的电压修正；同步样本含PWM纹波，建议Kd保持0 |

## PID 增益缩放

位置环使用位置式控制器；D 对反馈位置微分并按 `5 ms` 周期换算为实际速度，目标阶跃不会产生微分冲击：

```text
Kp_actual = Kp_register / 1000
D = -Kd_register * actual_speed_counts_per_second / 1000
Kd_register范围为0..1000
```

速度环使用带抗积分饱和的位置式PI，D项不参与计算：

```text
P = Kp_register * speed_error / 1000
I_accumulator += Ki_register * speed_error * dt_ms
I = I_accumulator / 50000
```

当前同步电流幅值采样/峰值保护随约 25.6 kHz PWM 运行，速度观测与速度 PI 周期为 `1 ms`，位置 PID 周期为 `5 ms`。速度积分和位置微分均显式包含时间尺度，切换调度频率时不会再隐式改变增益强度。

位置、速度与电流执行按层级串联；惯量和摩擦项是并联前馈，不是第四个力矩 PID。电流层在可观测电动区使用同步实测校正的周期平均电流闭环；续流盲区由模型补齐，但仍不是硬件四象限反馈环。

## Execute Tick 和回绕

Current Tick 与 Execute Tick 都是无符号 32 位毫秒计数。主站应先读取 Current Tick，再按模 `2^32` 运算生成 Execute Tick：

```text
execute_tick = (current_tick + delay_ms) mod 2^32
```

是否使用绝对时刻由 Control Word bit1 决定，不再把数值 `0` 当作特殊标记。bit1=`0` 表示立即提交；bit1=`1` 时从站使用 `(int32_t)(current_tick - execute_tick) >= 0` 判断到期。因此最大未来延迟为 `0x7FFFFFFF ms`，约 24.85 天；超过半个 32 位周期的目标会被解释为已经到期。命令应用后调度标志被清除，Execute Tick 寄存器保留最近写入值用于诊断。

## Fail-safe Policy

| 值 | 行为 |
| ---: | --- |
| `0` | 关闭 H 桥输出并清零驱动输出 |
| `1` | 切换到制动模式并清零驱动输出 |
| `2` | 将控制源切换到外部 PWM 输入，同时撤销串口命令使能 |

Policy=2 回退的是**控制权**，不是一个保存的 PWM 占空比。Fail-safe Policy 本身保存于 NVM；串口目标、使能状态、最近 PWM 输入和多圈计数均不保存。地址 `126` 只能观察 RAM 中最近捕获的 PWM 低电平宽度，没有独立的“回退 PWM 值”存储位置。

外部 PWM 当前按低电平有效的单圈位置命令解释：`1000 µs -> 0 count`，`2000 µs -> 16383 count`，中间线性插值。`900..2100 µs` 视为可接受输入，命令值在映射前钳位到 `1000..2000 µs`；超过 `100 ms` 没有完整有效脉冲时立即撤销 PWM 命令使能。有效外部 PWM 是独立的物理使能来源，因此 Policy=2 超时切换后，只有存在有效 PWM 输入才会重新产生输出。

Policy=2 下 `Fault Code=0x000A` 仍保持锁存，Status Packet 的 Alert 位和 Status Word 的 Fault Present 位仍为 `1`，但 Output Enabled 可以同时为 `1`。这表示主站失联后由明确配置的 PWM 后备源接管，不是故障被自动清除。欠压或超温保护仍具有更高优先级，并禁止 PWM 后备输出。

## Serial Watchdog 刷新规则

Serial Watchdog 只由“成功接收、通过校验并被控制层接受的新串口控制命令”刷新：

- 地址 `16/17/18/20/22/26` 的有效控制写入；
- 地址 `16` 的有效长度 `14` 或 `20` 命令映像；
- 匹配本节点且通过全部校验的 Sync Write 控制命令。

Ping、Read、Sync Read、参数读取、配置/PID 写入、Execute Tick 或 Sequence 单字段写入、被拒绝的命令以及重复 Sequence 重传都不刷新 Watchdog。只有控制源为 Serial 且活动或待生效命令已经请求使能时才累计超时。

## Node ID 与 Baud Code 生效时序

- 单播写 Node ID 时，从站先使用旧 ID 编码并通过 UART DMA 发送 Status ACK；只有 UART Transmission Complete 回调到达后才提交新 ID。
- Reg Write Node ID 只登记数据；单播 Action 仍先用旧 ID ACK，发送完成后再提交新 ID。
- 广播 Write/Sync Write 没有 ACK，匹配节点会立即更新 Node ID，因此主站必须避免多个节点同时写成相同 ID。
- 单播写 Baud Code 时，Status ACK 完整使用旧波特率发送；UART TC 到达后，主循环暂停响应发送、重初始化 UART 并重启 Receive-to-Idle DMA，然后才提交新波特率。写入只改变运行配置，仍需 Save NVM 才能跨复位保留。
