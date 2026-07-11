# 05 主站规则、故障处理与时钟同步

## 5.1 主站职责

主站不是简单串口发送器，而是整个总线的实时调度器。主站必须维护：

- 节点表：node ID、UID、拓扑位置、在线状态、故障状态。
- 状态机：每个节点的 `INIT/PREOP/SAFEOP/OP/FAULT`。
- PDO 映射：每个节点 RX/TX PDO 项、slot 大小、周期。
- 超时计数：PDO miss、SDO timeout、链式回环 timeout。
- 重连计时器：offline 节点低频 PING 或 DISCOVERY。
- 同步状态：cycle ID、master time、apply cycle、链式延迟估算。
- 安全策略：失联后 disable、brake 或 fallback PWM。

## 5.2 进入 OP 前检查

进入 OP 前，主站必须完成：

1. 发现或枚举所有需要的节点。
2. 分配唯一节点 ID。
3. 读取身份信息和能力对象。
4. 设置拓扑、波特率、看门狗和重连参数。
5. 配置控制源、FailSafe 策略、限幅和伺服模式。
6. 在 `PREOP` 配置 RX/TX PDO 映射。
7. 检查 PDO 字节数和总线负载。
8. 进入 `SAFEOP`。
9. 至少运行一次同步周期。
10. 以 `Enable = 0` 进入 `OP`。
11. 只有应用明确请求时才使能输出。

## 5.3 总线负载计算

UART 8N1：

```text
byte_time_us = 10 * 1000000 / baud
frame_time_us = frame_bytes * byte_time_us
```

单条 UART 链路推荐预算：

```text
wire_time_us = frame_bytes * 10 * 1000000 / baud
guard_time_us = max(50, node_count * 10)
```

最终 OP 预算与拓扑有关。并联模式主要由主站广播和各节点 reply slot 决定。链式 store-and-forward 模式必须累加每一段链路，因为每个从站都要先验证并重建帧再转发：

```text
ring_required_cycle_us =
    sum(segment_frame_bytes[i] * 10 * 1000000 / baud)
  + node_count * node_process_delay_us
  + guard_time_us

bus_usage = ring_required_cycle_us / configured_cycle_us
```

| 总线占用 | 主站行为 |
| --- | --- |
| `< 60%` | 正常，允许 OP |
| `60% - 80%` | 警告，配置允许时可 OP |
| `> 80%` | 禁止 OP |

如果总线占用过高，主站应减少 PDO 项、减少单周期节点数、降低更新频率，或在验证后提高波特率。

### PDO_FAST 负载计算

OP 状态的实时周期应按 `PDO_FAST` 计算，而不是按完整管理帧计算：

```text
fast_overhead_bytes = 6

// 链式滚动过程映像，第 i 段：
// i = 0 表示 Master -> Slave1
// i = node_count 表示 SlaveN -> Master
segment_process_bytes[i] =
    (node_count - i) * rx_slot_size
  + i * tx_slot_size

segment_frame_bytes[i] = fast_overhead_bytes + segment_process_bytes[i]

store_forward_required_us =
    sum(segment_frame_bytes[0..node_count] * 10 * 1000000 / baud)
  + node_count * node_process_delay_us
  + guard_time_us
```

例子：4 个节点，每个节点 RX PDO 13 字节、TX PDO 17 字节：

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

第一版安全 store-and-forward 实现无法放入 1 ms 周期。主站应选择多毫秒周期或减少 PDO 内容。若只传速度模式必要字段，例如 RX 8 字节、TX 8 字节：

```text
each segment process bytes = 4 * 8 = 32
each segment frame bytes = 38
5 segments * 38 bytes = 190 bytes
store_forward_wire_time_at_1Mbps = 1900 us
```

这个配置在 1 Mbps、store-and-forward 下仍无法进入 1 ms 周期。若链式必须做到 1 ms，需要减少单周期节点数、实测后提高波特率，或在后续版本实现 CRC/错误传播规则明确的 cut-through 模式。

## 5.4 通信周期选择

主站应根据 PDO 自动选择周期：

```text
current loop internal period  = 1 ms
speed loop internal period    = 5 ms
position loop internal period = 10 ms
```

推荐规则：

- 电流模式 PDO：目标周期 1 ms 到 2 ms。
- 速度模式 PDO：目标周期 2 ms 到 5 ms。
- 位置模式 PDO：目标周期 5 ms 到 10 ms。
- 如果总线负载超限，优先保留安全状态和反馈，降低目标更新率。

### 固定周期要求

进入 `OP` 后，主站必须以固定周期发送 `PDO_FAST`。周期由 `PREOP` 阶段配置并在 `SAFEOP` 锁定，进入 `OP` 后不得动态改变。

如果需要改变周期，必须执行：

```text
OP -> SAFEOP -> PREOP -> rewrite cycle objects -> SAFEOP -> OP
```

推荐抖动限制：

```text
PdoCycleTimeUs      = configured fixed cycle
PdoJitterLimitUs    = min(PdoCycleTimeUs / 10, 100 us)
PdoApplyCycleOffset = 1 or 2 cycles
```

规则：

- 主站应按固定周期发送，不能“有数据才发”。
- 从站用 `CYCLE_ID` 检查丢帧、重复帧和周期跳变。
- 连续超过 `PDO_MISS_LIMIT` 个周期未收到有效 PDO，触发 PDO watchdog。
- 周期改变属于通信配置改变，不属于普通运行命令。

## 5.5 并联失联与重连

默认参数：

```text
PDO_MISS_LIMIT = 3
RECONNECT_PERIOD_MS = 1000
SERIAL_WATCHDOG_MS = 100
```

主站行为：

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

并联模式下，一个节点失联不应拖慢全部节点。主站应把该节点移出 PDO 周期，只保留低频重连。恢复后必须重新配置，不能直接回 OP。

## 5.6 链式故障与重连

链式故障条件：

- OP 状态下 `PDO_FAST` 回环超时未返回。
- 诊断阶段 `PDO_CYCLE` 超时未返回。
- 返回帧 CRC 错误。
- 返回 `SEQ` 不匹配。
- 返回 `node_count` 不符合预期拓扑。
- `last_seen_position` 小于预期。

主站行为：

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

链式模式下任意断点会影响整条链，因此不能只移除单节点继续 PDO。必须先停止整链 PDO，再通过 ENUM 重建拓扑。

## 5.7 故障点标记

链式故障点分为确定和推测：

- 确定故障：回环帧能返回，并且 `last_seen_position` 有效。主站标记 `fault_after_position = last_seen_position`。
- 推测故障：完全没有回环。主站只能标记 `CHAIN_RETURN_LOST`，结合上一次成功拓扑推测。
- 如果 `last_seen_position = N` 且期望节点数也是 `N`，则故障可能在 `SlaveN TX -> Master RX`。

## 5.8 从站看门狗与失效保护

对象：

```text
0x1103 SerialWatchdogMs
0x2608 FailSafePolicy
```

`FailSafePolicy`：

```text
0 = disable output
1 = brake
2 = fallback to PWM
```

从站在串口控制源下超时应执行 FailSafe：

- `SERIAL_PDO`：连续超过 `PDO_MISS_LIMIT` 个周期未收到有效 PDO，或超过 `SerialWatchdogMs` 未收到有效 PDO。
- `SERIAL_SDO`：超过 `SerialWatchdogMs` 未收到有效 SDO 控制写入。

- `disable output`：关闭输出，`DriveRunMode = 0`。
- `brake`：进入刹车模式。
- `fallback to PWM`：回到 PWM 输入控制。

默认建议：工业安全优先使用 `disable output`；遥控/舵机场景可配置为 `fallback to PWM`。

看门狗优先级：

- `OP` 状态下，PDO 周期监督优先：`miss_count >= PDO_MISS_LIMIT` 触发 PDO watchdog 和 FailSafe。
- `SERIAL_PDO` 下，`SerialWatchdogMs` 是 PDO 周期监督的兜底时间限制，应大于或等于 `PdoCycleTimeUs * PDO_MISS_LIMIT / 1000`。
- `SERIAL_SDO` 下，`SerialWatchdogMs` 直接限制两次 SDO 控制写入的最大间隔。
- 非 `OP` 状态下，串口控制写入只更新 shadow，不允许驱动输出。
- 默认 PWM 控制不会仅因为没有串口通信而被关闭。

## 5.9 轻量分布式时钟

TSBP 不像完整 EtherCAT 硬件那样同步 MCU 硬件时钟，而是同步过程时序：

- 目标何时生效；
- 反馈何时采样；
- PDO 属于哪个周期。

`SYNC` payload：

```text
u16 cycle_id
u32 master_time_us
u16 apply_cycle
u16 flags
```

`PDO_CYCLE` 诊断帧可以包含相同同步字段：

```text
u16 cycle_id
u32 master_time_us
u16 apply_cycle
```

高频 `PDO_FAST` 不携带完整 `master_time_us`，只携带 `CYCLE_ID`。从站根据最近一次 `SYNC` 计算软件时间偏移，并按固定 `apply_cycle_offset` 在调度边界应用目标。

从站行为：

```text
on valid SYNC or PDO_CYCLE:
    local_rx_time = local_time_us()
    error = master_time_us - local_rx_time
    filtered_offset += error / filter_div
    if cycle_id == apply_cycle:
        apply pending PDO target at scheduler boundary
```

规则：

- PDO 目标不得在解析中途立即生效。
- 目标应在 1 ms 调度边界或配置的 apply cycle 生效。
- 链式从站应记录 `rx_latch_us` 和 `tx_latch_us`。
- 主站可以根据 latch 时间戳估算每个节点的转发延迟。

### 从站时钟偏移实现

从站不需要真正调节 MCU 硬件时钟，只维护一个软件偏移：

```text
bus_time_us = local_time_us + bus_time_offset_us
```

其中：

- `local_time_us`：从站本地单调递增微秒计数，可由 TIM 或 SysTick 扩展得到。
- `bus_time_offset_us`：主站时间和本地时间之间的软件偏移。
- `path_delay_us`：主站到该从站的估计通信延迟，第一版可配置为 0 或由主站按 position 估算。

SYNC 接收时，从站应尽早捕获本地时间戳：

```text
on SYNC frame received:
    rx_local_us = capture_local_time_as_early_as_possible()
    expected_master_rx_us = sync.master_time_us + path_delay_us
    sample_offset = expected_master_rx_us - rx_local_us
    bus_time_offset_us += (sample_offset - bus_time_offset_us) / filter_div
    last_sync_cycle_id = sync.cycle_id
```

推荐 `filter_div`：

```text
首次同步：filter_div = 1，直接锁定
稳定运行：filter_div = 8 或 16，降低抖动
```

从站使用 `bus_time_us` 判断当前总线周期：

```text
bus_time_us = local_time_us + bus_time_offset_us
current_cycle = bus_time_us / PdoCycleTimeUs
cycle_phase_us = bus_time_us % PdoCycleTimeUs
```

`PDO_FAST` 到达时，从站不立即把目标送入 PID，而是记录待生效周期：

```text
on PDO_FAST:
    if crc_ok and seq_ok:
        pending_command = rx_pdo
        pending_apply_cycle = pdo_fast.cycle_id + PdoApplyCycleOffset
```

1 ms 控制调度边界执行：

```text
on control_1ms_tick:
    bus_time_us = local_time_us + bus_time_offset_us
    current_cycle = bus_time_us / PdoCycleTimeUs

    if pending_valid and current_cycle >= pending_apply_cycle:
        active_command = pending_command
        pending_valid = false
```

这样可以把 UART 接收时刻和电机控制生效时刻解耦，减少链式转发延迟造成的节点间动作差异。

### 链式 path_delay_us 估算

第一版可以用简单估算：

```text
path_delay_us = (position - 1) * average_node_forward_delay_us
```

更高精度时，主站可通过诊断 `PDO_CYCLE` 中的 `rx_latch_us` / `tx_latch_us` 估计每个节点延迟，再通过 SDO 写入：

```text
0x1305 PathDelayUs
```

注意：几十厘米线长下，物理传播延迟很小，主要延迟来自 UART 字节传输、DMA 接收完成、软件转发和调度边界。因此第一版优先保证固定周期和 future-cycle 生效，比追求纳秒级对时更实际。

## 5.10 运动控制安全规则

- `Enable = 0` 必须关闭输出。
- `clear_fault` 不得同时隐式使能输出。
- 切换 `ServoMode` 时，从站必须清空未使用 PID 历史和积分。
- 从 `PWM` 切换到 `SERIAL_PDO` 或 `SERIAL_SDO` 时，应先保持目标为当前反馈附近或 `Enable = 0`。
- 低压、过温、编码器异常、电流异常优先级高于 PDO 命令。

## 5.11 验证计划

- 文档评审：主站只根据 XML 类配置文件能完成初始化。
- PC 协议仿真：并联 1、4、8、16 节点；链式 1、4、8 节点；节点掉线和重连。
- 错误测试：CRC 错误、重复 SEQ、缺帧、错误 node count。
- 固件验证：UART DMA 能处理最大帧；PDO 解析不阻塞 1 ms 控制；串口超时执行 FailSafe；上电默认 PWM。
- 总线负载验证：按 1 Mbps 计算不同节点数和 PDO 长度下的最小周期。

## 5.12 从站性能开销节约规则

STM32G030 从站资源有限，协议实现必须遵守“配置期多做，运行期少做”的原则。

### 配置期允许做的工作

这些工作只在 `PREOP/SAFEOP` 执行，可以相对复杂：

- 校验 PDO 映射项是否合法。
- 根据对象字典生成 RX/TX copy table。
- 计算 `rx_len`、`tx_len`、`node_stride`、`rx_offset`、`tx_offset`。
- 计算 PDO_FAST 固定帧长。
- 保存可掉电保存的配置参数。
- 根据主站配置切换 FailSafe、PWM 模式、PID、限幅。

### OP 周期禁止做的工作

这些工作不能放进 PDO_FAST 周期：

- 查找对象字典。
- 解析 XML 或字符串。
- 动态分配内存。
- printf、sprintf、浮点格式化。
- 按节点 ID 搜索 slot。
- 重新计算 PDO 映射。
- 在 UART 回调里执行 PID 或阻塞 I2C。
- 在高频周期保存 Flash。

### OP 快路径目标

从站处理一帧 `PDO_FAST` 的目标路径应足够短：

```text
DMA/idle frame ready
  -> CRC check
  -> fixed offset RX copy
  -> update command shadow
  -> fixed offset TX copy
  -> CRC update
  -> DMA forward/reply
```

推荐把串口层和控制层解耦：

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

### 从站运行期数据布局建议

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

这类结构在 `SAFEOP` 锁定后不得在 OP 周期修改。主站如果需要改 PDO，必须让节点退出 OP，回到 PREOP。

### CRC 开销策略

第一版可以使用表驱动 CRC16 提高速度；如果 Flash 更紧张，可使用小表或位运算 CRC。选择规则：

- 追求速度：256 项 CRC 表。
- 追求 Flash：16 项 nibble 表或 bitwise CRC。
- 不允许为了省 CRC 而取消 OP 周期校验。

### 文档验收标准

后续实现协议代码时，应能在代码评审中回答：

- OP 周期是否存在对象字典查找？
- OP 周期是否存在动态内存或字符串处理？
- PDO 映射是否只在 PREOP 编译？
- UART 回调是否避免执行 PID、Flash、I2C 阻塞操作？
- 1 ms 控制周期是否仍由 `ServoControl` 调度，而不是串口中断直接驱动？
