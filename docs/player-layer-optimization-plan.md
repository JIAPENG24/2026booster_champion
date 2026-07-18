# player.py 动作层修复规划

基于 `docs/strategy-optimization-notes.md` 第二章 "player.py 动作层"。

## 覆盖情况

| 小节 | 状态 | 实现 |
|------|------|------|
| 2.1-1 提前切球路线 | ❌ 待实现 | attack() 仍直奔当前球位 |
| 2.1-2 带球变向 | ✅ P2-11 | `_pick_dribble_direction()` |
| 2.1-3 禁区内轻推 | ✅ P1-10 | ball.x>4.0 → power*0.5 |
| 2.1-4 角度小传球 | ✅ P1-6 | `kick_can_score → pass_to` |
| 2.2-1 守门员横向跟随 | ✅ P0-2 | guard() 三模式 |
| 2.2-2 出击 | ✅ P2-12 | RUSH mode |
| 2.2-3 前提站位 | ✅ P2-12 | SWEEP mode |
| 2.3-1 动态距离 | ✅ P0-4 | support() 动态距离 |
| **2.3-2 support 盯人** | ✅ 已完成 | mark_opponent_id 参数 |
| **2.3-3 support 卡住换人** | ❌ 待实现 | support 不动时临时切 attack |
| 2.4 pass_to | ✅ P1-6 | |
| 2.5-1 球位射角落 | ✅ P0-1 | plan_kick() 球 y→角落 |
| **2.5-2 守门员位置感知** | ❌ 待实现 | plan_kick() 无守门员感知 |
| **2.5-3 远距离解围** | ❌ 待实现 | 后场仍射门而非解围 |
| **2.5-4 角度差阈值** | ❌ 待实现 | plan_kick 无角度检查 |
| **2.6 KICK_ENTER_M** | ❌ 待实现 | 2.0→推荐 1.2-1.5 |

---

## 2.1-1 attack() 提前切球路线

### 问题
`attack()` 中追球时调用 `walk_to(_behind_ball(ball.x, ball.y, ...))`，目标点基于**当前**球位。球正在运动时，球员会追球跑而不是拦截球路，效率低。

### 方案

1. **Player 类增加球速追踪字段**（跨帧状态）
   ```python
   self._ball_track_prev: tuple[float, float] | None = None
   self._ball_track_time: float | None = None
   self._ball_vx: float = 0.0
   self._ball_vy: float = 0.0
   ```

2. **在 attack() 走过场前更新球速**
   - 用 `self._ball_track_prev` 和当前 `ball.x/y` 计算速度向量
   - 只有 `last_seen_at` 变化合理时才更新

3. **走位目标改为预测拦截位**
   - 距球远时 (d > KICK_ENTER_M * 2)，走位到预测 0.5s 后球位
   - 距球近时仍用当前球位（确保能碰到球）

### 改动文件
- `src/player.py:Player.__init__` — 新增追球跟踪字段
- `src/player.py:Player.attack()` — 追球时用预测拦截位

---

## 2.3-3 support() 卡住换人

### 问题
support 球员被障碍卡住不动时，始终保持 support 行为，不会切换为进攻。

### 方案

Player 已有字段 `_support_last_pos` / `_support_stationary_since` / `_support_last_update_at`，用帧数计数代替时间戳：

1. 在 `support()` 末尾（走位后）检测：
   ```python
   if self.pose is not None:
       frames = getattr(self, '_support_stuck_frames', 0)
       if self._support_last_pos is not None:
           moved = dist(self.pose.x, self.pose.y, *self._support_last_pos)
           if moved < 0.05:      # 几乎没动
               frames += 1
           else:
               frames = 0
       self._support_last_pos = (self.pose.x, self.pose.y) if self.pose else None
       self._support_stuck_frames = frames
   ```

2. frames > 60 (≈2s@30fps) 时临时切换 attack：
   ```python
   if frames > SUPPORT_STUCK_FRAMES_MAX:
       self.action = "support_stuck_attack"
       self.attack()
       return
   ```

3. 新增 `SUPPORT_STUCK_FRAMES_MAX = 60` 到 param.py

### 改动文件
- `src/param.py` — 新增 SUPPORT_STUCK_FRAMES_MAX
- `src/player.py:Player.support()` — 卡住检测 + 临时 attack

---

## 2.5-2 plan_kick() 守门员位置感知

### 问题
`plan_kick()` 只根据球位置选射门目标，不看对方守门员站位。守门员在左时仍射左柱浪费机会。

### 方案

1. 新增 `_find_goalie() → Pose2D | None`：
   ```python
   def _find_goalie(self) -> Pose2D | None:
       """返回对方守门员位置（距对方球门最近者）。"""
       ctx = self.context
       if ctx is None:
           return None
       gx, gy = opponent_goal(ctx)
       best, best_dist = None, math.inf
       for opp in ctx.opponents.values():
           if opp.pose is None:
               continue
           d = dist(opp.pose.x, opp.pose.y, gx, gy)
           if d < best_dist:
               best_dist, best = d, opp.pose
       return best
   ```

2. 在 `plan_kick()` 射门目标选择中增加守门员感知：
   - 找到对方守门员
   - 守门员在左侧 (y > 0.2) → 射右门柱
   - 守门员在右侧 (y < -0.2) → 射左门柱
   - 守门员居中或找不到 → 沿用当前逻辑

### 改动文件
- `src/player.py:Player.plan_kick()` — 增加守门员感知
- `src/player.py:Player._find_goalie()` — 新增 helper

---

## 2.5-3 plan_kick() 远距离解围

### 问题
球在我方半场 (x < 0) 时，`_in_backfield()` 只加大了踢球力度，但目标仍是射门。远距离射门成功率低，应该解围到安全区域。

### 方案

在 `plan_kick()` 中，球在我方半场时改为大脚解围：
- 目标改为中场边路
- 力度保持 `KICK_POWER_BACKFIELD`
- 方向选己方 id 奇偶决定的边路

```python
if self._in_backfield():
    # 远距离解围: 向中场边路大脚
    side = 1.0 if self.id % 2 == 0 else -1.0
    clear_target = (0.0, side * ctx.field.width * 0.3)
    kick_direction = angle_to(ball.x, ball.y, *clear_target)
    return kick_direction, KICK_POWER_BACKFIELD
```

### 改动文件
- `src/player.py:Player.plan_kick()` — 后场改为解围逻辑

---

## 2.5-4 plan_kick() 角度差阈值

### 问题
从极偏角度射门时成功率低，plan_kick 不做角度判断。虽然 `attack()` 中有 `kick_can_score()` 检查，但 plan_kick 本身也应考虑角度。

### 方案

在 `plan_kick()` 末尾，若球门朝向角 < 30°（即偏离 > 60°），返回 None 表示不应射门，由调用方 `attack()` 的 `kick_can_score` 处理传球。

```python
# 角度检查: 球→球门中心方向与当前射门方向的偏差
goal_dir = angle_to(ball.x, ball.y, goal_x, 0.0)
angle_deviation = abs(normalize_angle(kick_direction - goal_dir))
if angle_deviation > math.radians(60):
    return None  # 角度太偏,attack() 会切传球
```

注意：`plan_kick` 返回 None 时，`attack()` 已有处理（stop + fallback），但当前 `attack()` 的 `kick_can_score` 也会处理。这里额外加角度阈值作为双重安全。

### 改动文件
- `src/player.py:Player.plan_kick()` — 加角度偏离检查

---

## 2.6 KICK_ENTER_M 减小

### 问题
`KICK_ENTER_M = 2.0` 让球员在距球 2m 处就开始踢球动作，过早。球未完全控稳就踢出，容易踢偏或被拦截。

### 方案

```python
KICK_ENTER_M = 1.2  # 原为 2.0
```

同时调整迟滞退出距离保持 0.3m 余量：

```python
KICK_EXIT_M = 1.5  # 原为 2.5
```

### 改动文件
- `src/param.py`

---

## 改动汇总

| 文件 | 改动 |
|------|------|
| `src/param.py` | KICK_ENTER_M 2.0→1.2, KICK_EXIT_M 2.5→1.5, 新增 SUPPORT_STUCK_FRAMES_MAX=60 |
| `src/player.py:__init__` | 新增 `_ball_track_prev/_time/_vx/_vy`, `_support_stuck_frames` |
| `src/player.py:attack()` | 走位目标改为预测拦截位（距球远时） |
| `src/player.py:plan_kick()` | 后场解围 + 守门员感知 + 角度偏离检查 |
| `src/player.py:_find_goalie()` | 新增 helper |
| `src/player.py:support()` | 卡住检测 + 临时 attack |
