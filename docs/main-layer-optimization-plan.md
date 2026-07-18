# main.py 策略层修复规划

基于 `docs/strategy-optimization-notes.md` 第一章 "main.py 策略层"。

## 覆盖情况

| 小节 | 状态 | 描述 |
|------|------|------|
| 1.1-1 球预测 | ✅ P1-9 | `_predict_ball_pos()` + `_select_closest_attacker()` 已使用预测位置 |
| 1.1-2 动态角色交换 | ✅ P2-13 | attacker 停滞计数器，45帧<0.15m 则换人 |
| **1.1-3 盯人防守** | ❌ 待实现 | support 未阻挡对方威胁球员 |
| **1.1-4 攻防权重区分** | ❌ 待实现 | 球位决定进攻/防守阵型 |
| 1.2 定位球 | ✅ P1-7/8 | 6种我方定位球 + 对方定位球防线 |
| 1.3 开球 | ✅ P0-3 | 短传开球 + 支援前插 |
| 1.4 READY | ✅ P2-15 | 按角色分配站位 |
| **1.5-3 异常处理** | ❌ 待实现 | `_act_*` 未保护 |

---

## 1.1-3 support 盯人防守

### 目标
support 不再仅站在球→球门连线上，而是盯防"最危险"的对方球员，阻断对方持球人→球门的传球/射门路线。

### 方案

**`main.py`**:
- 在 `_act_normal()` 中，选出 guard 后，遍历 `context.opponents` 计算每个对手的威胁分
- 威胁分 = `dist(opp→our_goal)`（越小越危险）
- 把最危险对手的 id 传给 `p.support(mark_opponent_id=...)`

**`player.py`**:
- `support()` 新增可选参数 `mark_opponent_id: int | None = None`
- 若指定了盯人目标：
  - 拿到对方球员位置
  - 计算该对手→我方球门的连线中点
  - 站位到该中点附近（距球门更近一侧，形成阻挡）
- 若无指定对手，fallback 当前逻辑（球→门连线动态距离）

### 改动文件
- `src/main.py:_act_normal()` — 计算威胁对手，传入 support
- `src/player.py:support()` — 新增 mark_opponent_id 参数和盯人走位逻辑

---

## 1.1-4 进攻/防守权重区分

### 目标
根据球的位置自动切换阵型：
- 球在我方半场 (x < -1.0)：**防守模式** — guard + 2×support（密集防守）
- 球在对方半场 (x > 3.0)：**进攻模式** — 双前锋前压 + guard
- 球在中场 (-1.0 ≤ x ≤ 3.0)：**均衡模式** — 当前默认策略

### 方案

**`main.py:_act_normal()`**:
```
if ball.x < DEFEND_BALL_X:
    # 防守模式: 三人回收
    attacker = closest  → 追球压迫
    guard + 1×support  → 都做 support（密集，距球1.5-2.5m）
elif ball.x > ATTACK_BALL_X:
    # 进攻模式: 双前锋
    primary = closest  → attack
    secondary = 2nd closest  → also attack
    guard → guard
else:
    # 均衡模式: 当前逻辑不变
```

### 改动文件
- `src/main.py:_act_normal()` — 加球位判断 + 阵型切换
- `src/param.py` — 新增 `DEFEND_BALL_X` / `ATTACK_BALL_X` 阈值

---

## 1.5-3 异常处理

### 目标
任何 `_act_*` 函数抛出异常时不要崩溃整个线程，而是记录错误并安全刹车。

### 方案

**`main.py:play()`**:
```python
try:
    if phase == Phase.NORMAL:
        _act_normal(context, active, store)
    elif ...:
        ...
except Exception:
    _log.exception("Phase %s crashed", phase.value)
    for p in active:
        p.stop()
```

### 改动文件
- `src/main.py:play()` — 包 try/except

---

## 改动汇总

| 文件 | 改动内容 |
|------|----------|
| `src/param.py` | 新增 DEFEND_BALL_X, ATTACK_BALL_X |
| `src/main.py` | _act_normal() 攻防权重 + 盯人计算 + play() try/except |
| `src/player.py` | support() 新增 mark_opponent_id 参数 + 盯人走位 |
