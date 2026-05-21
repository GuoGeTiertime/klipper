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
| 0.3 | 状态基线 | `FILA_BUFFER_STATUS BUFFER=buffer0` | `mode=disabled`，无丝 unit `state=empty`，`active=None` |
| 0.4 | 引脚极性 | 核对 `^` 与传感器常开/常闭 | `inlet=1`/`buffer=1` 表示**有丝**（与 STATUS 一致） |
| 0.5 | 双缓冲隔离 | 同时 STATUS `buffer0` / `buffer1` | 两套 `jam/low/full`、unit 互不影响 |

---

## 1. 配置解析与启动校验

逐项改配置后重启 Klipper。

### 1.1 缓冲器段 `[filabuffer <name>]` — 必填

| # | 配置项 | 测试方法 | 期望 |
|---|--------|----------|------|
| 1.1.1 | `jam_pin` / `low_pin` / `full_pin` | 缺一项启动 | `config_error` |
| 1.1.2 | 引脚同 MCU | 故意跨 MCU 组合 | `button pins must be on same mcu` |
| 1.1.3 | 无 unit | 只有三 IN、无 `step_pin_0` | `no feed units` |
| 1.1.4 | 重复 buffer 名 | 两个 `[filabuffer buffer0]` | `Duplicate filabuffer` |

### 1.2 缓冲器段 — 可选参数

| # | 配置项 | 测试 | 期望 |
|---|--------|------|------|
| 1.2.1 | `pinout_delay` | 0.010 / 0.050 边界；非法 0.005 | 合法生效；非法拒绝 |
| 1.2.2 | `watchdog_time` | 改 0.1，观察补料/超时周期 | 定时器间隔变化 |
| 1.2.3 | `feed_speed` / `feed_speed_init` | 低速 5、高速 60 | 工作/初始化送进速度不同 |
| 1.2.4 | `max_feed_time` / `max_feed_len` | 故意很小（如 2s / 10mm） | `MODE=work` 下触发 `feed_timeout` + `low_timeout_gcode` |
| 1.2.5 | `init_max_feed_time` / `init_max_feed_len` | 同上，`MODE=init_work` | 初始化超时用 init 限值 |
| 1.2.6 | `retract_len` / `retract_speed` | 线材初始化 | 缓冲端有丝后回撤约 `retract_len` |
| 1.2.7 | `min_buffer_travel_mm` | 设得过小 + 高速 | Ready 时 `config_error`（需 ≥ speed×delay） |
| 1.2.8 | `button_latency` / `hw_latency` | 仅影响校验计算 | 与 `full_stop_mode` 联动 |
| 1.2.9 | `full_stop_mode: host` | FULL 时仅 Host 停 PWM/enable | 见 §4.3、§7.2 |
| 1.2.10 | `full_stop_mode: hardware_enable` | FULL 时硬件已断使能 | Host 仍 `stop_immediate`；电机不继续转 |
| 1.2.11 | `pause_on_error: True` | 任意 ERROR + 打印中 | 队列 G-code 前带 `PAUSE` |
| 1.2.12 | `pause_on_error: False` | 同上 | 无 `PAUSE`，仅模板宏 |
| 1.2.13 | `jam_gcode` / `low_timeout_gcode` / `break_gcode` / `runout_gcode` | 各触发一次 | 对应宏执行且 `M400` 串行 |

### 1.3 送进单元 `_0` / `_1` / `_2`

| # | 配置项 | 测试 | 期望 |
|---|--------|------|------|
| 1.3.1 | `step_pin_N` 必填 | 有 `inlet_pin_N` 无 step | `step_pin_N required` |
| 1.3.2 | `inlet_pin_N` + `buffer_pin_N` | 缺一对 | `inlet_pin and buffer_pin required` |
| 1.3.3 | `unit_name_N` | 重复名 | `duplicate unit_name` |
| 1.3.4 | `dir_pin_N` 省略 | 送进/回撤 | 仅 PWM step，方向固定 |
| 1.3.5 | `dir_pin_N` 配置 | 正向送、负速回撤 | 方向脚电平正确切换 |
| 1.3.6 | `enable_pin_N` 省略 | 启动送进 | `bfeeder_on` 逻辑仍跑；使能靠外部 |
| 1.3.7 | `enable_pin_N` 配置 | `start_continuous` / `stop_immediate` | enable 与 PWM 同步，停时先断 enable |
| 1.3.8 | `microstep` / `full_steps_per_rotation` / `rotate_distance` | 改 `rotate_distance` | 同速度下 step 频率变化 |
| 1.3.9 | `gear_ratio` / `gear_ratio_1` | unit1 不设、unit1 单独设 | unit1 继承 unit0 或独立齿比 |
| 1.3.10 | `max_speed` | 请求速度 > max | 钳位到 `max_speed` |
| 1.3.11 | 仅 unit0 共享机械参数 | unit1 只配引脚 | unit1 `gearing` 与 unit0 一致（STATUS） |

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

## 3. 单元丝检 IN（`inlet_pin` / `buffer_pin`）

| # | 场景 | 操作 | 期望 |
|---|------|------|------|
| 3.1 | inlet 有/无 | 手动触发 inlet | STATUS `inlet=0/1` 实时更新 |
| 3.2 | buffer 有/无 | 手动触发 buffer | STATUS `buf=0/1` |
| 3.3 | 双路 buffer 互斥 | unit0、unit1 同时 buffer=1 | `multi_buffer_filament`，ERROR，停所有电机 |
| 3.4 | 非 active 单元 inlet 断 | `SELECT unit0`，拔 unit1 inlet | **不**触发 runout |
| 3.5 | active inlet 断 | work/init_work 中拔 active inlet | `runout_gcode`；单元 `stop`；非打印 `mode=error` |
| 3.6 | active buffer 断 | work 中 active `buffer` 1→0 | `break_gcode`，ERROR |
| 3.7 | init_fila 非目标单元 | `INIT_FILAMENT UNIT=unit0`，只动 unit1 丝检 | unit0 流程不受影响 |

---

## 4. 缓冲器模式（`FilaBuffer.mode`）

| 模式 | 值 | 测试入口 | 核心期望 |
|------|-----|----------|----------|
| M0 | `disabled` | 上电默认 / STOP / RESET | 电机停；不响应 LOW 送料 |
| M1 | `init` | 插入线材或 `INIT_FILAMENT` | 见 §6.1 |
| M2 | `init_work` | `START MODE=init_work` | 送至 `full_pin` 上升沿 → 自动 `work` |
| M3 | `work` | `START MODE=work` | LOW∧¬FULL 电平送料；FULL 或 ¬LOW 停 |
| M4 | `error` | 各类故障 | `error_msg` 有值；RESET 回 disabled |

| # | 项 | 操作 | 期望 |
|---|-----|------|------|
| 4.1 | START 无 SELECT | 直接 `START MODE=work` | `FILA_BUFFER_SELECT required` |
| 4.2 | START 无 inlet | SELECT 后 inlet=0，`MODE=work` | `inlet has no filament` |
| 4.3 | START 他路有 buffer 丝 | unit1 buffer=1，SELECT unit0 无 buffer | `Buffer filament on another unit` |
| 4.4 | START 非法 MODE | `MODE=foo` | `Invalid MODE` |
| 4.5 | START MODE=init_fila | `MODE=init_fila` | `Invalid MODE`（已废弃） |
| 4.6 | STOP | `FILA_BUFFER_STOP` | `disabled`，各 unit 按传感器 `sync`（通常 `empty`/`ready`），清 `init_phase` |
| 4.7 | RESET | ERROR 后 `RESET` | 清 error，`disabled`，可重新 SELECT |

---

## 5. 送进单元状态（`unit_state` + `init_phase`）

**稳定态**（由 inlet/buffer + 是否 `active_unit` 推导）：

| inlet | buffer | 选中 | `unit_state` |
|-------|--------|------|--------------|
| 0 | 0 | * | `empty` |
| 1 | 0 | * | `ready` |
| 1 | 1 | 是 | `active` |
| 1 | 1 | 否 | `error` |
| 0 | 1 | * | `error` |

**运行态**（电机动作中，`init_phase` 仅 init 时有效）：

| 状态 | 值 | 如何进入 | 验证 |
|------|-----|----------|------|
| `init` | `forward` / `retract` | empty 且 inlet 0→1（自动或 `INIT_FILAMENT`） | buffer 1→0 → `ready` |
| `feed` | — | `SELECT_UNIT` 或 work/init_work 送料 | buffer 0→1 → `active` |

| # | 项 | 期望 |
|---|-----|------|
| 5.1 | SELECT 后 | 仅更新 `active`；`state` 由传感器决定（inlet=1 buffer=0 → `ready`） |
| 5.2 | init/feed 时 STATUS | `state=init`/`feed`，`feed=1`，`len` 增加 |
| 5.3 | 停送后 | `feed=0`，`len` 归零，`state` 回到稳定态 |

---

## 6. 工作流（端到端）

### 6.1 线材初始化 `init`

**前置**：unit 为 `empty`（inlet=0 buffer=0）

| 步骤 | G-code / 操作 | 期望 |
|------|----------------|------|
| A1 | inlet 0→1（或 `INIT_FILAMENT` 等待后插入） | `state=init`，`phase=forward`，电机转 |
| A2 | 送进途中 | inlet 保持，buffer 未到 | 持续送进（至 init 限长/时或 buffer 到） |
| A3 | buffer 0→1 | — | 停送 → `phase=retract`，回撤 `retract_len` |
| A3b | （已知问题，暂未修） | 看门狗续送上限为 `init_max_feed_len` 非 `retract_len` | 实际反转可能明显长于配置 `retract_len`；见 `filabuffer.py` 头 `TODO(init)` |
| A4 | buffer 1→0 | — | `state=ready`，`phase=None` |
| A4b | 回撤后 buffer 仍为 1 | — | `init_retract_fail`，`state=error` |
| A5 | 送进中拔 inlet | inlet 1→0 | `init_runout`，`state=error` |
| A6 | `INIT_FILAMENT` 且 inlet 已有 | INIT 命令 | 立即 `init`，不等插入边沿 |

### 6.2 选中单元 `SELECT_UNIT` + 工作初始化

**前置**：A 完成，unit `ready`

| 步骤 | G-code / 操作 | 期望 |
|------|----------------|------|
| B0 | `FILA_BUFFER_SELECT_UNIT BUFFER=buffer0 UNIT=unit0` | `state=feed` → buffer 1 → `active` |
| B1 | `FILA_BUFFER_START BUFFER=buffer0 MODE=init_work` | 立即送进，`state=feed` |
| B2 | `full_pin` 0→1 | 停送，`mode=work`，unit `active` |
| B3 | 未满超时 | 极小 `init_max_feed_time` | `feed_timeout` + `low_timeout_gcode` |

### 6.3 正常工作 `work`

**前置**：B 完成或 `START MODE=work`

| 步骤 | 操作 | 期望 |
|------|------|------|
| C1 | low=1, full=0 | active unit 送进 |
| C2 | low **保持**有效 | 释放 full 后再置 low | 仍应再次送料（`_sync_work_feed` 电平逻辑） |
| C3 | full=1 | 停送；`hardware_enable` 时硬件断使能 |
| C4 | low=0 | 停送 |
| C5 | low 短脉冲 | 仅在 LOW∧¬FULL 期间送 |
| C6 | 打印中 runout | 模拟打印 + inlet 断 | `runout` + 可能 `PAUSE` |
| C7 | 空闲 runout | 非打印 inlet 断 | `runout_gcode`，`mode=error` |

### 6.4 多单元切换

| 步骤 | 操作 | 期望 |
|------|------|------|
| D1 | unit0 init 完成，`SELECT unit1` | unit1 ready |
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
# 插入线材：empty + inlet 自动 init，或 INIT_FILAMENT
FILA_BUFFER_INIT_FILAMENT BUFFER=buffer0 UNIT=unit0
# 等待 state=ready
FILA_BUFFER_SELECT_UNIT BUFFER=buffer0 UNIT=unit0
# 等待 state=active
FILA_BUFFER_START BUFFER=buffer0 MODE=init_work
# 等待 full，自动进入 work
FILA_BUFFER_START BUFFER=buffer0 MODE=work
```

---

## 7. G-code 命令

| 命令 | 参数 | 测试用例 | 期望 |
|------|------|----------|------|
| `FILA_BUFFER_SYNC_SENSORS` | `BUFFER`, `CLEAR_ERROR` | ready 后 / RESET 后 | 从链接丝检读状态并 `sync` unit；默认清 error |
| `FILA_BUFFER_SELECT` | `BUFFER`, `UNIT` | 合法/非法 UNIT | 设置 `active` 名称；`state` 由传感器推导 |
| `FILA_BUFFER_SELECT_UNIT` | `BUFFER`, `UNIT` | unit 须 `ready` | 送进至 `active` |
| `FILA_BUFFER_START` | `BUFFER`, `MODE` | disabled/init_work/work | §4、§6 |
| `FILA_BUFFER_STOP` | `BUFFER` | 运行中停止 | §4.6 |
| `FILA_BUFFER_INIT_FILAMENT` | `BUFFER`, `UNIT` | 指定 unit | §6.1（可选，与自动 init 等价） |
| `FILA_BUFFER_RESET` | `BUFFER` | error 后 | §4.7 |
| `FILA_BUFFER_STATUS` | `BUFFER` | 各阶段 | mode/active/jam/low/full/每 unit |

---

## 8. 电机与实时性（`FeedMotor`）

| # | 项 | 操作 | 期望 |
|---|-----|------|------|
| 8.1 | `pinout_delay` | 0.010 vs 0.050 | step/enable 相对 IN 边沿延迟可辨 |
| 8.2 | 急停顺序 | FULL 或 jam 边沿 | enable 先断再停 PWM |
| 8.3 | 连续送进 | work 长时 LOW | ~50mm 一块续送，`cur_feed_len` 累加 |
| 8.4 | 无 enable | 省略 enable_pin | PWM 仍输出 |
| 8.5 | 高速过冲 | `feed_speed=60`，短 LOW | 物理过冲 ≤ `min_buffer_travel_mm` 量级 |

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
| `runout` | active unit inlet 断 | 停 | `runout_gcode` |
| `break` | work/init_work active buffer 断 | 停 | `break_gcode` |
| `feed_timeout` | 超 init/work 时间或长度 | 全停 | `low_timeout_gcode` |

每条：□ 触发 □ STATUS `error=` 正确 □ RESET 可恢复 □ `pause_on_error` 符合配置

---

## 10. 完整回归顺序（单 buffer、双 unit）

```text
1. §0 准备 + STATUS 基线
2. §1 配置项（可分多次重启）
3. §2 缓冲器 IN（disabled）
4. §3 单元 IN + 互斥
5. §6.1 unit0 线材初始化
6. §6.2 init_work → 自动 work
7. §6.3 work 电平 LOW/FULL（含 C2）
8. §9 故障抽样 + RESET
9. §6.4 unit1 子集
10. §6.5 buffer1 子集
11. §8 电机时序（可选）
```

---

## 11. 记录模板

```text
日期：
固件 commit：
配置：buffer0 / buffer1，full_stop_mode=：
单元：unit0 step=____ enable=有/无

[ ] 1.1.x 配置校验
[ ] 2.x  缓冲器 IN
[ ] 3.x  单元 IN
[ ] 6.1  init_fila
[ ] 6.2  init_work
[ ] 6.3  work + LOW 保持
[ ] 9.x  错误矩阵
[ ] E.x  双缓冲

失败项编号：____  现象：____  日志：____
```
