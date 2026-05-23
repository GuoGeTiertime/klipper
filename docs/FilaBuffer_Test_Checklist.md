# FilaBuffer 测试清单

本文档为 [filabuffer](../klippy/extras/filabuffer.py) 模块的实机/台架逐项测试清单。

- 示例配置：[config/sample-filabuffer.cfg](../config/sample-filabuffer.cfg)
- 实现源码：`klippy/extras/filabuffer.py`

**建议环境**：可手动拉高/拉低各 IN 的台架；示波器可选测 `step`/`enable` 与 `pinout_delay`。

**记录方式**：□ 通过 / □ 失败 / 备注 / `FILA_BUFFER_STATUS` 或日志

---

## 0. 测试准备

| # | 项 | 操作 | 期望 |
|---|-----|------|------|
| 0.1 | Klipper 启动 | `include sample-filabuffer.cfg` 或自有配置 | 无 `config_error`，`klippy:ready` 成功 |
| 0.2 | 上电忽略窗 | Ready 后 **2s 内** 抖动各 IN | 不触发送进/报错（`STARTUP_IGNORE_TIME`） |
| 0.3 | 状态基线 | `FILA_BUFFER_STATUS BUFFER=buffer0` | `mode=disabled`，无丝 feeder `state=empty`，`active=None` |
| 0.4 | 引脚极性 | 核对 `^` 与传感器常开/常闭 | `inlet=1`/`buffer=1` 表示**有丝**（与 STATUS 一致） |
| 0.5 | 双缓冲隔离 | 同时 STATUS `buffer0` / `buffer1` | 两套 `jam/low/full`、feeder 互不影响 |

---

## 1. 配置解析与启动校验

逐项改配置后重启 Klipper。

### 1.1 缓冲器段 `[filabuffer <name>]` — 必填

| # | 配置项 | 测试方法 | 期望 |
|---|--------|----------|------|
| 1.1.1 | `jam_pin` / `low_pin` / `full_pin` | 缺一项启动 | `config_error` |
| 1.1.2 | 引脚同 MCU | 故意跨 MCU 组合 | `button pins must be on same mcu` |
| 1.1.3 | 无 feeder | 只有三 IN、无 `step_pin_0` | `no feeders` |
| 1.1.4 | 重复 buffer 名 | 两个 `[filabuffer buffer0]` | `Duplicate filabuffer` |

### 1.2 缓冲器段 — 可选参数

| # | 配置项 | 测试 | 期望 |
|---|--------|------|------|
| 1.2.1 | `pinout_delay` | 0.010 / 0.050 边界；非法 0.005 | 合法生效；非法拒绝 |
| 1.2.2 | `watchdog_time` | 改 0.1，观察补料/超时周期 | 定时器间隔变化 |
| 1.2.3 | `feed_speed` / `feed_speed_init` | 低速 5、高速 60 | 工作/初始化送进速度不同 |
| 1.2.4 | `max_feed_time` / `max_feed_len` | 故意很小（如 2s / 10mm） | `MODE=work` 下触发 `feed_timeout` + `low_timeout_gcode` |
| 1.2.5 | `init_max_feed_time` / `init_max_feed_len` | 同上，feeder `init` 阶段 | 初始化超时用 init 限值 |
| 1.2.6 | `retract_len` / `retract_speed` | 线材初始化 | 缓冲端有丝后回撤约 `retract_len` |
| 1.2.7 | `min_buffer_travel_mm` | 设得过小 + 高速 | Ready 时 `config_error`（需 ≥ speed×delay） |
| 1.2.8 | `button_latency` / `hw_latency` | 仅影响校验计算 | 与 `full_stop_mode` 联动 |
| 1.2.9 | `full_stop_mode: host` | FULL 时仅 Host 停 PWM/enable | 见 §4.3、§7.2 |
| 1.2.10 | `full_stop_mode: hardware_enable` | FULL 时硬件已断使能 | Host 仍 `motor_halt`；电机不继续转 |
| 1.2.11 | `pause_on_error: True` | 任意 ERROR + 打印中 | 队列 G-code 前带 `PAUSE` |
| 1.2.12 | `pause_on_error: False` | 同上 | 无 `PAUSE`，仅模板宏 |
| 1.2.13 | `jam_gcode` / `low_timeout_gcode` / `break_gcode` / `runout_gcode` | 各触发一次 | 对应宏执行且 `M400` 串行 |

### 1.3 送进 feeder `_0` / `_1` / `_2`

| # | 配置项 | 测试 | 期望 |
|---|--------|------|------|
| 1.3.1 | `step_pin_N` 必填 | 无 step | `step_pin_N is required` |
| 1.3.2 | 链接丝检 | `[filament_*_sensor]` 设 `filabuffer_feeder` + `filabuffer_role` | 边沿上报 inlet/buffer |
| 1.3.3 | `feeder_name_N` | 重复名 | `duplicate feeder_name` |
| 1.3.4 | `dir_pin_N` 省略 | 送进/回撤 | 仅 PWM step，方向固定 |
| 1.3.5 | `dir_pin_N` 配置 | 正向送、负距回撤 | 方向脚电平正确切换 |
| 1.3.6 | `enable_pin_N` 省略 | 启动送进 | 仅 PWM；使能靠外部或 TMC |
| 1.3.7 | `enable_pin_N` 配置 | `motor.start` / `motor_halt` | enable 与 PWM 同步 |
| 1.3.8 | `microstep` / `full_steps_per_rotation` / `rotate_distance` | 改 `rotate_distance` | 同速度下 step 频率变化 |
| 1.3.9 | `gear_ratio` / `gear_ratio_1` | feeder1 不设、feeder1 单独设 | feeder1 继承 feeder0 或独立齿比 |
| 1.3.10 | `max_speed` | 请求速度 > max | 钳位到 `max_speed` |
| 1.3.11 | 仅 feeder0 共享机械参数 | feeder1 只配引脚 | feeder1 `gearing` 与 feeder0 一致（STATUS） |

---

## 2. 缓冲器三 IN（`BufferSensors`）

**引脚顺序**：jam → bit0，low → bit1，full → bit2。

| # | 场景 | 操作 | 期望 |
|---|------|------|------|
| 2.1 | JAM 单独 | 仅 jam=1 | STATUS `jam=1 low=0 full=0`；边沿上升 → 停**所有**单元电机、`mode=error`、`jam_gcode` |
| 2.2 | LOW 单独 | 仅 low=1 | `low=1`；可与 jam 并存（非 full 时） |
| 2.3 | FULL 单独 | 仅 full=1 | `full=1`；不与 jam/low 同时为 1 |
| 2.4 | FULL 互斥 | full=1 同时 jam 或 low=1 | `full_exclusive_violation`，ERROR |
| 2.5 | JAM+LOW | jam+low，full=0 | 允许（1/2 可并存） |
| 2.6 | JAM 边沿 | work 中送料，再触发 jam | 立即停送；不依赖 LOW 释放 |

---

## 3. Feeder 丝检（链接 `filabuffer_feeder` / `filabuffer_role`）

| # | 场景 | 操作 | 期望 |
|---|------|------|------|
| 3.1 | inlet 有/无 | 手动触发 inlet | STATUS `inlet=0/1` 实时更新 |
| 3.2 | buffer 有/无 | 手动触发 buffer | STATUS `buf=0/1` |
| 3.3 | 双路 buffer 互斥 | unit0、unit1 同时 buffer=1 | `multi_buffer_filament`，ERROR，停所有电机 |
| 3.4 | 非 active feeder inlet 断 | `SELECT FEEDER=unit0`，拔 unit1 inlet | **不**触发 runout |
| 3.5 | active inlet 断 | work 中拔 active inlet | `runout_gcode`；电机停；非打印 `mode=error` |
| 3.6 | active buffer 断 | work 中 active `buffer` 1→0 | `break_gcode`，ERROR |
| 3.7 | init 非目标 feeder | `INIT_FILAMENT FEEDER=unit0`，只动 unit1 丝检 | unit0 流程不受影响 |

---

## 4. 缓冲器模式（`FilaBuffer.mode`）

| 模式 | 值 | 测试入口 | 核心期望 |
|------|-----|----------|----------|
| M0 | `disabled` | 上电默认 / STOP / SYNC 清错 | 电机停；不响应 LOW 送料 |
| M1 | `work` | `START MODE=work`（默认） | LOW∧¬FULL 时 active feeder 送料；FULL 或 ¬LOW 停 |
| M2 | `error` | 各类故障 | `error_msg` 有值；`SYNC_SENSORS CLEAR_ERROR=1` 回 disabled |

| # | 项 | 操作 | 期望 |
|---|-----|------|------|
| 4.1 | START work | 各 feeder empty/ready/buffered，无 jam | `mode=work`，可 `_sync_work_feed` |
| 4.2 | START 有 feeder 忙 | init/feed 运行中 START | `Feeder X is busy` |
| 4.3 | START 双 buffer 丝 | 两路 buffer=1 | `Over one buffer filament detected` |
| 4.4 | START 非法 MODE | `MODE=foo` | `Invalid MODE (use work or disabled)` |
| 4.5 | STOP | `FILA_BUFFER_STOP` | `disabled`，各 feeder 按传感器 `sync`，清 `init_phase` |
| 4.6 | SYNC 清错 | ERROR 后 `SYNC_SENSORS CLEAR_ERROR=1` | 清 error，`disabled`，可重新 START |

---

## 5. Feeder 状态（`feeder_state` + `init_phase`）

**稳定态**（由 inlet/buffer + 是否 `active_feeder` 推导）：

| inlet | buffer | 选中 | `feeder_state` |
|-------|--------|------|----------------|
| 0 | 0 | * | `empty` |
| 1 | 0 | * | `ready` |
| 1 | 1 | 是 | `active` |
| 1 | 1 | 否 | `buffered` |
| 0 | 1 | * | `error` |

**运行态**（电机动作中，`init_phase` 仅 init 时有效）：

| 状态 | 值 | 如何进入 | 验证 |
|------|-----|----------|------|
| `init` | `forward` / `retract` | empty 且 inlet 0→1（自动或 `INIT_FILAMENT`） | 回撤完成 buffer=0 → `ready` |
| `feed` | — | `SELECT_FEEDER` 或 work 下 LOW 送料 | buffer 0→1 → `active` |

| # | 项 | 期望 |
|---|-----|------|
| 5.1 | SELECT 后 | 仅更新 `active_feeder`；`state` 由传感器决定 |
| 5.2 | init/feed 时 STATUS | `state=init`/`feed`，`feed=1`，`len` 增加 |
| 5.3 | 停送后 | `feed=0`，`len` 反映本次 `get_move_mm()`，`state` 回到稳定态 |

---

## 6. 工作流（端到端）

### 6.1 线材初始化 `init`

**前置**：feeder 为 `empty`（inlet=0 buffer=0）

| 步骤 | G-code / 操作 | 期望 |
|------|----------------|------|
| A1 | inlet 0→1（或 `INIT_FILAMENT` 等待后插入） | `state=init`，`phase=forward`，电机转 |
| A2 | 送进途中 | inlet 保持，buffer 未到 | 持续送进（至 init 限长/时或 buffer 到） |
| A3 | buffer 0→1 | — | 停送 → `phase=retract`，回撤 `retract_len` |
| A3b | 回撤定量 | `motor.start(-retract_len, ...)` | 回撤距离约 `retract_len`（非无限续段） |
| A4 | buffer 1→0 | — | `state=ready`，`phase=None` |
| A4b | 回撤后 buffer 仍为 1 | — | `init_retract_fail`，`state=error` |
| A5 | 送进中拔 inlet | inlet 1→0 | `init_runout`，`state=error` |
| A6 | `INIT_FILAMENT` 且 inlet 已有 | INIT 命令 | 立即 `init`，不等插入边沿 |

### 6.2 选中 feeder `SELECT_FEEDER`

**前置**：A 完成，feeder `ready` 或 `buffered`；`FILA_BUFFER_START MODE=work`

| 步骤 | G-code / 操作 | 期望 |
|------|----------------|------|
| B0 | `FILA_BUFFER_SELECT_FEEDER BUFFER=buffer0 FEEDER=unit0` | `state=feed` → buffer 1 → `active` |
| B1 | 他路非 empty/ready | SELECT_FEEDER 时 unit1 为 `buffered` | `other feeders must be empty or ready` |
| B2 | 超时 | 极小 `init_max_feed_time` / `max_feed_time` | `feed_timeout` + `low_timeout_gcode` |

### 6.3 正常工作 `work`

**前置**：`START MODE=work`，`SELECT` 指定 active feeder

| 步骤 | 操作 | 期望 |
|------|------|------|
| C1 | low=1, full=0 | active feeder 送进 |
| C2 | low **保持**有效 | 释放 full 后再置 low | 仍应再次送料（`_sync_work_feed` 电平逻辑） |
| C3 | full=1 | 停送；`hardware_enable` 时硬件断使能 |
| C4 | low=0 | 停送 |
| C5 | low 短脉冲 | 仅在 LOW∧¬FULL 期间送 |
| C6 | 打印中 runout | 模拟打印 + inlet 断 | `runout` + 可能 `PAUSE` |
| C7 | 空闲 runout | 非打印 inlet 断 | `runout_gcode`，`mode=error` |

### 6.4 多 feeder 切换

| 步骤 | 操作 | 期望 |
|------|------|------|
| D1 | unit0 init 完成，`SELECT FEEDER=unit1` | unit1 ready |
| D2 | unit1 init 完成 | 仅 unit1 buffer 参与互斥 |
| D3 | work 仅 active 送料 | SELECT unit0 + LOW | 仅 unit0 电机转 |

### 6.5 双缓冲器

| 步骤 | 操作 | 期望 |
|------|------|------|
| E1 | buffer0 work，buffer1 disabled | 互不影响 |
| E2 | 两 buffer 分别 jam | 各自 `jam_gcode` 文本不同 |
| E3 | `FILA_BUFFER_ALL_STOP` 宏 | 两套均 stopped |

### 6.6 推荐启动顺序（参考）

```gcode
FILA_BUFFER_START BUFFER=buffer0 MODE=work
# 插入线材：empty + inlet 自动 init，或：
FILA_BUFFER_INIT_FILAMENT BUFFER=buffer0 FEEDER=unit0
# 等待 state=ready
FILA_BUFFER_SELECT BUFFER=buffer0 FEEDER=unit0
FILA_BUFFER_SELECT_FEEDER BUFFER=buffer0 FEEDER=unit0
# 等待 state=active；LOW 时自动补料
```

---

## 7. G-code 命令

| 命令 | 参数 | 测试用例 | 期望 |
|------|------|----------|------|
| `FILA_BUFFER_SYNC_SENSORS` | `BUFFER`, `CLEAR_ERROR` | ready 后 / 清错 | 从链接丝检读状态并 `sync` feeder；默认清 error |
| `FILA_BUFFER_SELECT` | `BUFFER`, `FEEDER` | 合法/非法 FEEDER | 设置 `active_feeder`；`state` 由传感器推导 |
| `FILA_BUFFER_SELECT_FEEDER` | `BUFFER`, `FEEDER` | feeder 须 `ready`/`buffered` | 送进至 `active`（须 work 模式） |
| `FILA_BUFFER_FEEDER_MOVE` | `BUFFER`, `FEEDER`, `SPEED`, `LENGTH` | 手动定量 | 正/负 LENGTH；`motor.is_moving()` 互斥 |
| `FILA_BUFFER_START` | `BUFFER`, `MODE` | `work` / `disabled` | §4、§6 |
| `FILA_BUFFER_STOP` | `BUFFER` | 运行中停止 | §4.5 |
| `FILA_BUFFER_INIT_FILAMENT` | `BUFFER`, `FEEDER` | 指定 feeder | §6.1（可选，与自动 init 等价） |
| `FILA_BUFFER_STATUS` | `BUFFER` | 各阶段 | mode/active/jam/low/full/每 feeder |

---

## 8. 电机与实时性（`FilaMotor`）

| # | 项 | 操作 | 期望 |
|---|-----|------|------|
| 8.1 | `pinout_delay` / `MCU_PIN_EVENT_DELAY` | 改模块常量或板级延迟 | step/enable 相对调度有足够间隔 |
| 8.2 | 急停 | FULL 或 jam | `motor_halt`，PWM 停 |
| 8.3 | 分段 PWM | work 长时 LOW | 每段 ≤ `MOTOR_MAX_CHUNK_TIME`（4s），定时续段 |
| 8.4 | 无 enable | 省略 enable_pin | PWM 仍输出 |
| 8.5 | 手动移动 | `FEEDER_MOVE` | 定量走完 `on_feeder_motor_stop`；忙时拒绝新命令 |

---

## 9. 错误矩阵

| error_msg | 触发方式 | 电机 | G-code 队列 |
|-----------|----------|------|-------------|
| `jam` | jam 上升沿 | 全停 | `jam_gcode` |
| `full_exclusive_violation` | full+jam/low | 全停 | 无专用模板 |
| `multi_buffer_filament` | 两路 buffer=1 | 全停 | 无 |
| `init_runout` | init 送进中 inlet 断 | 停 | 无 |
| `init_retract_fail` | init 回撤后 buffer 仍为 1 | 停 | 无 |
| error→empty | error 时用户拔光丝（inlet/buffer 均 0） | — | 自动 `empty`；否则需 `SYNC_SENSORS` |
| `runout` | active feeder inlet 断 | 停 | `runout_gcode` |
| `break` | work 中 active buffer 断 | 停 | `break_gcode` |
| `feed_timeout` | 超 init/work 时间 | 全停 | `low_timeout_gcode` |

每条：□ 触发 □ STATUS `error=` 正确 □ SYNC 清错可恢复 □ `pause_on_error` 符合配置

---

## 10. 完整回归顺序（单 buffer、双 feeder）

```text
1. §0 准备 + STATUS 基线
2. §1 配置项（可分多次重启）
3. §2 缓冲器 IN（disabled）
4. §3 feeder IN + 互斥
5. §6.1 unit0 线材初始化
6. §6.2 SELECT_FEEDER → active
7. §6.3 work 电平 LOW/FULL（含 C2）
8. §9 故障抽样 + SYNC 清错
9. §6.4 unit1 子集
10. §6.5 buffer1 子集
11. §8 FilaMotor / FEEDER_MOVE（可选）
```

---

## 11. 记录模板

```text
日期：
固件 commit：
配置：buffer0 / buffer1，full_stop_mode=：
feeder：unit0 step=____ enable=有/无

[ ] 1.1.x 配置校验
[ ] 2.x  缓冲器 IN
[ ] 3.x  feeder IN
[ ] 6.1  init
[ ] 6.2  SELECT_FEEDER
[ ] 6.3  work + LOW 保持
[ ] 9.x  错误矩阵
[ ] E.x  双缓冲

失败项编号：____  现象：____  日志：____
```
