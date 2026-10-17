# pico-usb-io 改造设计：固件只提供通用 PIO 原语，用途全部交给上位机

> 状态：**定稿并已实现**（一次性替换，不保留旧的 I2S / pulse / bitstream 功能）
> 结论先行：精确计时必须留在 RP2040 的 PIO/DMA 硬件上；上位机只负责**装载程序 + 写配置 + 上下传数据**。上位机不逐位翻转引脚。

---

## 1. 背景与目标

### 改造前的问题

固件内建了三类"有明确用途"的功能，主机只能按固定参数调用，不能自行组合：

| 功能 | 协议 op | 固件内的实现 |
|---|---|---|
| I2S | 18 / 19 | 写死的 I2S PIO 程序 + DMA 环形缓冲 + 左右声道打包 + 事件帧推流；强制 WS=SCK+1、采样率 8k–48k |
| 脉冲收发 | 14 / 15 / 16 | 固定脉冲时序发送/捕获程序 + 边沿事件 |
| 位流 | 17 | WS2812 位时序编解码 |

这违反"固件只做基础通讯、业务逻辑在上位机"的原则。

### 目标

1. 固件只提供**通用可编程原语**：装载 PIO 程序、配置状态机、TX 推送、RX 读取、DMA 流式回传。
2. I2S 录音、WS2812 等用途全部由上位机用这些原语表达，效果与改造前等价。
3. 旧的专用 op 14–19、`EVT_PULSE` 事件、`timing.rs` 全部删除，不做兼容。

### 范围

- 保留基础通讯 op 1–13（VERSION / PIN / GPIO / PWM / ADC / I2C / SPI / UART），行为不变。
- 保留 USB 通路（CDC 串口）和帧格式。
- `PROTO_VERSION` 升为 2，固件版本号为 2；旧上位机与新固件不兼容，需一起更新。

---

## 2. 关键设计原则

1. **精度来源**：所有 ns/µs 级边沿由 PIO + DMA 在片上生成/采样。上位机何时下发可抖动几毫秒，一旦程序跑起来，时序完全由硬件生成。
2. **上位机是"程序员"，不是"实时执行者"**：上位机上传 PIO 指令字、分频值、引脚映射、数据，并读回结果。
3. **数据整形放在 PIO 程序里**：如"每声道只取高 16 位并左右拼成一个字"由上位机写的 PIO 程序完成，固件 DMA 只原样搬运，不认识数据含义。
4. **Linux/Windows 无差异**：实时任务不在主机侧，主机调度抖动不影响精度。
5. **帧格式不变**：`SOF + HEADER(8) + BODY(≤1024) + CRC16`（见 `protocol/src/lib.rs`）。

---

## 3. 资源划分

| 资源 | 用途 |
|---|---|
| PIO0、PIO1 | 全部交给通用引擎，`pio` 字段取 0 或 1，每块 4 个 SM、32 条指令 |
| DMA 通道 0 / 1 + DMA_IRQ_0 | 连续流的 ping-pong（同一时间只有一个流） |
| RAM | DMA 双缓冲 2×1024 字 + 环形 16384 字 ≈ 72 KB；固件 `.bss` 合计约 85 KB / 264 KB |
| 引脚 | 统一走 `Role` 仲裁，PIO 占用的引脚为 `Role::Pio` |

固件直接按 PAC 运行时编号访问两个 PIO 块（`embassy_rp::init` 已解除全部外设复位），不使用 embassy 的 `Pio` 驱动。

---

## 4. 协议

### 4.1 操作码

| op | 名称 | 说明 |
|---|---|---|
| 1–13 | 基础通讯 | 不变 |
| 20 | `OP_PIO_LOAD` | 把 PIO 指令字装载到指令内存 |
| 21 | `OP_PIO_UNLOAD` | 释放一段指令内存 |
| 22 | `OP_PIO_SM_CONFIG` | 配置一个状态机（程序、分频、引脚、移位、FIFO） |
| 23 | `OP_PIO_SM_ENABLE` | 使能/停用状态机 |
| 24 | `OP_PIO_SM_RESTART` | 复位状态机（移位计数、分频相位，可选清 FIFO） |
| 25 | `OP_PIO_TX_PUSH` | 批量写入 SM 的 TX FIFO |
| 26 | `OP_PIO_RX_POP` | 批量读取 SM 的 RX FIFO |
| 27 | `OP_PIO_STREAM_START` | 启动流：SM RX FIFO → 环形缓冲 → 事件帧回传 |
| 28 | `OP_PIO_STREAM_STOP` | 停止流 |
| 29 | `OP_PIO_STREAM_STATUS` | 查询流状态与丢弃计数 |
| 30 | `OP_PIO_STREAM_EVENT` | **仅作为事件帧（`KIND_EVENT`）的 op**，body 为流数据 |

op 14–19 已删除，发送会返回 `ST_BAD_ARG`。各标志位常量统一定义在 `protocol/src/lib.rs`（`PIO_*`），固件和上位机共用。

### 4.2 body 布局

所有多字节字段**小端**。`pio`：0 或 1。`sm`：0..3。

#### `OP_PIO_LOAD`（20）

```
[0] pio
[1] flags        bit0 PIO_LOAD_AUTO: 1=自动分配连续指令内存；0=装到 origin 指定位置
[2] origin       flags.bit0=0 时有效（0..31）
[3] wrap_target  相对程序起点
[4] wrap         相对程序起点
[5] count        指令条数 1..32
[6..6+2*count]   指令字 u16 序列
```

响应 body：`[origin:u8, count:u8]`（实际装载位置）。

程序按**相对地址**汇编。固件装载时把 JMP 指令的目标地址、`wrap_target`、`wrap` 加上实际 origin；自动分配从低地址起找第一段连续空闲。空间不足返回 `ST_NO_SPACE`。

#### `OP_PIO_UNLOAD`（21）

```
[0] pio  [1] origin  [2] count
```

origin/count 与装载记录不符返回 `ST_BAD_ARG`；有已使能的 SM 正在使用该段时返回 `ST_BUSY`。

#### `OP_PIO_SM_CONFIG`（22）

固定 24 字节（`PIO_SM_CONFIG_LEN`）：

```
偏移  字段             说明
0     pio
1     sm
2     origin           已装载程序的 origin（wrap 取自装载记录）
3-4   clkdiv_int       u16，必须 ≥1
5     clkdiv_frac      0..255
6     out_base
7     out_count        0..32
8     set_base
9     set_count        0..5
10    in_base
11    in_count         仅用于引脚仲裁与 funcsel；RP2040 的 IN 本身无计数
12    sideset_base
13    sideset_bits     含可选位在内的总位数 0..5
14    flags            bit0 sideset_opt, bit1 sideset_pindirs,
                       bit2 out_shift_right, bit3 in_shift_right,
                       bit4 autopull, bit5 autopush
15    pull_thresh      1..32
16    push_thresh      1..32
17    fifo_join        0 none, 1 tx, 2 rx
18    jmp_pin          0xFF 表示不用
19    保留，填 0
20-23 out_pindir_mask  u32，位 n=1 表示 GPIO n 设为输出；其余被引用的引脚设为输入
```

固件处理顺序：

1. 完整校验 body，不改任何状态：程序是否已装载、分频、side-set 位数、阈值、fifo_join 等不合法返回 `ST_BAD_ARG`；out/set/in/sideset 区间与 jmp_pin 中任一引脚 ≥30 返回 `ST_BAD_PIN`；`out_pindir_mask` 含集合外引脚返回 `ST_BAD_ARG`。校验失败时引脚原来的功能不受影响。
2. 对用到的引脚做 `Role` 仲裁（释放旧角色）。
3. 若该 SM 正在流，先停流；停用该 SM；把引脚 funcsel 设为对应 PIO。
4. 写 CLKDIV / EXECCTRL / SHIFTCTRL，逐个引脚执行 `SET PINDIRS` 设方向，再写 PINCTRL。
5. 清 FIFO，执行 `JMP origin`。

#### `OP_PIO_SM_ENABLE`（23）

```
[0] pio  [1] sm  [2] enable(0/1)
```

未配置过程序的 SM 使能返回 `ST_BAD_ARG`。

#### `OP_PIO_SM_RESTART`（24）

```
[0] pio  [1] sm  [2] flags  bit0 PIO_RESTART_CLKDIV, bit1 PIO_RESTART_FIFOS, bit2 PIO_RESTART_SM
```

#### `OP_PIO_TX_PUSH`（25）

```
[0] pio  [1] sm  [2] flags(bit0 PIO_TX_WAIT: 等待 FIFO 空间)  [3] word_count(1..255)
[4..4+4*word_count]  u32 字序列
```

响应 body：`[pushed:u8]`。等待为**有界等待**（总计不超过 50 ms，期间让出执行器），超时返回已写入的个数。

#### `OP_PIO_RX_POP`（26）

```
[0] pio  [1] sm  [2] max_words(1..255)
```

响应 body：`u32` 字序列（可能为空）。

#### `OP_PIO_STREAM_START`（27）

```
[0] pio  [1] sm
[2] flags        bit0 PIO_STREAM_SPARSE: 稀疏模式
[3] chunk_words  每个事件帧最多携带的字数 1..255，0 表示 255
```

同一时间只允许一个流，已有流时返回 `ST_BUSY`。

- **连续模式**：DMA 通道 0/1 ping-pong，TREQ = 对应 PIO 的 `RX{sm}`，每半区 1024 字；DMA_IRQ_0 中把完成的半区**原样**拷入 16384 字环形，再重新装填该通道。环形不足时只丢弃本半区放不下的部分，并累加丢弃计数。
- **稀疏模式**：不用 DMA，主循环在无命令时轮询该 SM 的 RX FIFO，有数据就发。DMA 半区要攒满才完成，不适合稀疏数据。

#### `OP_PIO_STREAM_STOP`（28）

```
[0] pio  [1] sm
```

不是当前流的 SM 时直接返回成功。固件停止顺序（来自调试经验，不可调换）：

1. 先停用该 SM（否则 RX DREQ 持续存在，DMA abort 后 busy 位可能清不掉）。
2. 关 DMA_IRQ_0 与 INTE 对应位，abort 通道 0/1。
3. 有界等待 busy 清零（不能死等，否则执行器冻结、USB 一起失联）。
4. 清 INTS、清空环形。
5. **引脚保持不动**：仍归 PIO 驱动、保持最后电平，不设为 HiZ。需要的确定电平由上位机随后用 `OP_PIN_CONFIG` 设置。

#### `OP_PIO_STREAM_STATUS`（29）

响应 body：`[active:u8][ring_used:u32][dropped_words:u32]`。

#### 流事件帧（`KIND_EVENT`, op=`OP_PIO_STREAM_EVENT`=30）

```
seq = 0, status = 0, body = u32 字序列（小端）
```

---

## 5. 固件结构

- `firmware/src/pio.rs`：通用 PIO 引擎 `Engine`。
  - 指令内存：每块一个 32 位占用位图 + 按 origin 记录的 count/wrap。
  - 状态机：记录每个 SM 绑定的 origin 和占用引脚。
  - 流：静态 DMA 双缓冲 + 环形 + DMA_IRQ_0 处理函数 `StreamDmaHandler`。
- `firmware/src/main.rs`：基础通讯处理不变；分发表新增 op 20–29；`Role::Pio` 释放时停用占用该引脚的 SM。
- 主循环：命令优先；无命令时调用 `stream_read` 把流数据作为 op 30 事件帧发出；空闲时让出 1 ms。
- 会话结束：DTR 掉线时调用 `Engine::reset()`，停流、停用全部 SM、清空指令内存记录，引脚保持不动。上位机异常退出后重新连接，指令内存不会被残留程序占满。
- 已删除：`timing.rs`、op 14–19、`EVT_PULSE`，以及固件对 `pio`、`fixed` crate 的依赖。

---

## 6. 上位机结构

### 6.1 依赖

host 引入 `pio = "0.3"`，用 `pio::pio_asm!` 汇编（不写 `.origin`），取 `program.code`、`program.wrap`、`program.side_set` 填协议字段。

### 6.2 API（`host/src/lib.rs`）

```rust
pub type Program = pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }>;
pub fn clkdiv(freq_hz: u32) -> (u16, u8);

impl Board {
    pub fn pio_load(&mut self, pio: u8, program: &Program) -> Result<PioProgram, Error>;
    pub fn pio_unload(&mut self, prog: PioProgram) -> Result<(), Error>;
    pub fn pio_sm_config(&mut self, sm: u8, prog: &PioProgram, cfg: &SmConfig) -> Result<(), Error>;
    pub fn pio_sm_enable(&mut self, pio: u8, sm: u8, on: bool) -> Result<(), Error>;
    pub fn pio_sm_restart(&mut self, pio: u8, sm: u8, flags: u8) -> Result<(), Error>;
    pub fn pio_tx_push(&mut self, pio: u8, sm: u8, words: &[u32]) -> Result<(), Error>;
    pub fn pio_rx_pop(&mut self, pio: u8, sm: u8, max: u8) -> Result<Vec<u32>, Error>;
    pub fn pio_stream_start(&mut self, pio: u8, sm: u8, sparse: bool, chunk_words: u8) -> Result<(), Error>;
    pub fn pio_stream_stop(&mut self, pio: u8, sm: u8) -> Result<(), Error>;
    pub fn pio_stream_status(&mut self) -> Result<StreamStatus, Error>;
    pub fn read_stream(&mut self, wait: Duration) -> Result<Vec<u32>, Error>;
}
```

- `PioProgram` 记录装载位置和 side-set 信息，`pio_sm_config` 据此填 side-set 字段。
- `SmConfig` 与 `OP_PIO_SM_CONFIG` 一一对应，实现了 `Default`。
- `pio_tx_push` 自动按 255 字分帧并带 `PIO_TX_WAIT`，一次也推不进去时返回 `ST_BUSY`。
- 串口读写都在调用者线程里按顺序进行（先写完请求再读应答），不另开读线程：Windows 同步串口句柄上的读和写会被系统排队，后台挂着的读会让每条命令多等一个读超时。`call()` 期间收到的流帧放进队列，不丢。

### 6.3 分频计算

与 embassy `calculate_pio_clock_divider` 相同（sysclk 固定 125 MHz）：

```
clkdiv_int  = sysclk / freq
clkdiv_frac = ((sysclk % freq) << 8) / freq
```

48 kHz I2S 的 PIO 时钟 `48000 * 128 = 6.144 MHz`，得 `(20, 88)`；WS2812 的 8 MHz 得 `(15, 160)`。

---

## 7. 用例

### 7.1 I2S 录音（`host/src/record.rs`）

PIO 程序只采每声道前 16 个时钟的数据，后 16 个时钟只出时钟不采样；autopush 阈值 32，每帧正好 1 个 u32：高 16 位为左声道，低 16 位为右声道。带宽 192 KB/s。

side-set 2 位：bit0 = SCK，bit1 = WS。

```
.side_set 2
.wrap_target
    set x, 15          side 0b00
left_in:
    in pins, 1         side 0b01
    jmp x-- left_in    side 0b00
    set x, 13          side 0b01
left_skip:
    nop                side 0b00
    jmp x-- left_skip  side 0b01
    nop                side 0b00
    nop                side 0b11
    set x, 15          side 0b10
right_in:
    in pins, 1         side 0b11
    jmp x-- right_in   side 0b10
    set x, 13          side 0b11
right_skip:
    nop                side 0b10
    jmp x-- right_skip side 0b11
    nop                side 0b10
    nop                side 0b01
.wrap
```

共 16 条指令，每声道 64 个 PIO 周期（`set` 1 + 采样 16×2 + `set` 1 + 跳过 14×2 + `nop` 2）。已用逐周期模拟与改造前的 32 位程序比对：4 帧内 SCK/WS 波形逐周期一致，采样时刻正好是旧程序每个 32 位字的前 16 位，所以结果与旧固件的打包逐位相同。

流程：

1. `pio_load(0, ...)`。
2. `pio_sm_config`：`sideset_base=SCK`（`WS=SCK+1` 由上位机断言）；`in_pins=(SD,1)`；`out_pindirs` 含 SCK、WS；`autopush`、`push_thresh=32`；`fifo_join=rx`；`clkdiv(RATE*128)`。
3. `pio_sm_restart(CLKDIV | FIFOS | SM)`。
4. `pio_stream_start(连续, 255)`。
5. `pio_sm_enable(on)`。先开流再使能，避免 FIFO 满导致开头卡时钟。
6. 循环 `read_stream()`，每字 `(word >> 16) as i16` 为左声道 → 高通 + 增益 → WAV。
7. 结束：`pio_stream_stop` → `pio_sm_enable(off)` → `pin_output(SCK/WS, 低)`、`pin_input(SD)`（INMP441 在时钟线悬空时会进入异常，必须断电才恢复）→ `pio_unload`。

### 7.2 WS2812（`host/src/ws2812.rs`）

使用 pico-examples 的 4 条指令程序（`.side_set 1`，每位 10 个周期），PIO 时钟 8 MHz 即 800 kHz 位率；`autopull`、`pull_thresh=24`、`fifo_join=tx`。每个 LED 一个字：三个颜色字节放在 bit31..8，按板子实测的 RGB 顺序。FIFO 空时停在 `out`，引脚保持低电平，即复位码。结束时停用 SM、引脚拉低、卸载程序。

### 7.3 舵机（`host/src/servo.rs`）

PIO 时钟 1 MHz，用 `set pins` 输出 50 Hz 脉冲。每个字低 16 位是高电平计数、高 16 位是低电平计数（`out_shift_right`）；程序用 `pull noblock`，FIFO 空时重复上一个字，所以上位机只在角度变化时推一次，电脑定时抖动不会让脉冲中断。程序每周期有 8 条额外指令，`pulse_word()` 扣掉这部分开销。使能前必须先推第一个字，否则 X=0 会输出极窄的脉冲。

### 7.4 红外接收（`host/src/ir.rs`）

PIO 跑在 125 MHz，用 `wait pin` / `jmp pin` 交替测量高、低电平段的长度（每轮 2 个周期），每段推一个计数值；用稀疏流回传，上位机换算成微秒，按 NEC 协议解码按键。这替代了改造前固件内建的脉冲捕获功能。

### 7.5 与改造前的差异

- `WS=SCK+1`、采样率范围、颜色字节顺序等检查与约定全部在上位机。
- 停止后的引脚电平由上位机负责设置。
- 旧的脉冲收发与边沿事件不再由固件提供，改为上位机的 PIO 程序（7.3 舵机输出、7.4 红外边沿测量）。

---

## 8. 接收流的两种模式

二者共用同一套原语，区别只在 PIO 程序与 `PIO_STREAM_SPARSE`：

- **连续模式**：DMA 把 RX FIFO 连续搬入环形，主循环按 `chunk_words` 回传。用于 I2S 等高吞吐连续采样。
- **稀疏模式**：主循环轮询 RX FIFO，有数据才回传。用于红外接收、边沿时间戳等稀疏场景。

---

## 9. 风险与对策

| 风险 | 对策 |
|---|---|
| 跳转地址未重定位导致程序跑飞 | 程序按相对地址上传，固件装载时统一加 origin |
| sideset 引脚必须相邻 | 上位机断言并给出明确错误 |
| 通用流带宽翻倍 | 数据整形放在 PIO 程序内，I2S 每帧 1 个 u32，保持 192 KB/s |
| 分频取整不一致 | 公式与 `calculate_pio_clock_divider` 相同（6.3） |
| `MAX_BODY=1024` 限制单帧数据量 | 单帧 ≤255 个 u32；大流量走流原语 |
| 停止时 DMA 卡死、执行器冻结 | 先停 SM、再 abort、有界等待（4.2 STREAM_STOP） |
| 停止后引脚悬空导致麦克风异常 | 固件停止时不改引脚；上位机用 `OP_PIN_CONFIG` 设确定电平 |
| 主机断开后流仍在推、端口卡死 | DTR 掉线时固件 `reset()` 引擎 |
| 上位机异常退出导致指令内存泄漏 | 同上，`reset()` 清空指令内存记录 |
| TX 推送阻塞执行器 | 有界等待 50 ms，期间让出执行器 |

---

## 10. 验证计划

1. **I2S 录音**：48 kHz 录 15 s，对比 `record` 打印的峰值、削顶数、采样数/用时、板上丢弃字数（应为 0），并人工试听，应与改造前无可感知差异。
2. **稳定性**：连续录制 3 次以上、每次 15 s，不拔插 USB，不出现卡顿、保存延迟、第二次无法启动或静音。
3. **异常退出**：录音中途关掉程序，再次运行应能正常启动（验证 DTR 掉线 reset）。
4. **WS2812**：`ws2812.bat` 依次显示红、绿、蓝、灭。
5. **舵机 / 红外**：`servo.bat` 能转到指定角度并保持不抖；`ir.bat` 按遥控器能打印对应按键。
6. **基础功能**：`blink.bat` 行为不变。
7. **边界**：指令内存装满返回 `ST_NO_SPACE`、卸载后可再装；非法引脚返回 `ST_BAD_PIN`；重复启动流返回 `ST_BUSY`；发送旧 op 14–19 返回 `ST_BAD_ARG`。
