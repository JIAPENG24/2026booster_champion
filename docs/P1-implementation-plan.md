# P1 级修复实现方案

基于 `docs/strategy-optimization-notes.md`，共 5 项修复。

**修改范围**：`src/main.py` `src/player.py` `src/param.py`

---

## P1-6: 进攻加传球（player.py + main.py）

### 当前问题

`attack()` 永远追球→射门，即使角度极差或被防守人堵死。全队无传球配合。

### 改动方案

1. **player.py** 新增 `pass_to(teammate_id)` 方法
2. **player.py** 修改 `attack()`：进入踢球范围后，若 `kick_can_score` 返回 False（角度无法射门），找位置最佳的队友传球，不盲目射门

### 具体代码

**player.py — 新增 pass_to()，放在 attack() 之前（约第 540-570 行区域）：**

```python
def pass_to(self, teammate_id: int) -> bool:
    """传球给指定队友。返回是否成功发出。"""
    ctx = self.context
    ball = ctx.ball if ctx is not None else None
    mate = ctx.teammates.get(teammate_id) if ctx is not None else None
    if ball is None or mate is None or mate.pose is None or self.pose is None:
        return False
    kick_dir = angle_to(ball.x, ball.y, mate.pose.x, mate.pose.y)
    dist_to_mate = dist(ball.x, ball.y, mate.pose.x, mate.pose.y)
    power = clamp(dist_to_mate * 1.5, 2.0, 6.0)
    self.kick(kick_dir, power)
    self.action = f"pass_to_{teammate_id}"
    return True
```

**player.py — 修改 attack()（约第 571 行）：**

```python
def attack(self, kick_target: tuple[float, float] | None = None) -> None:
    ball = self.context.ball if self.context is not None else None
    if ball is None or self.pose is None:
        self.stop()
        return
    if kick_target is None:
        kick_target = opponent_goal(self.context)

    d = dist(self.pose.x, self.pose.y, ball.x, ball.y)
    self._kicking = d <= (KICK_EXIT_M if self._kicking else KICK_ENTER_M)
    if self._kicking:
        kick_plan = self.plan_kick()
        if kick_plan is None:
            self.stop()
            return
        kick_direction, kick_power = kick_plan
        # 角度太差且有人可传时传球
        if not self.kick_can_score(kick_direction):
            ctx = self.context
            if ctx is not None:
                # 找离球门最近的我方球员（非自己）传球
                best_mate = None
                best_score = -999
                for pid, mate in ctx.teammates.items():
                    if pid == self.id or mate.pose is None:
                        continue
                    # 评分：越靠近对方球门越好
                    score = mate.pose.x
                    if score > best_score:
                        best_score = score
                        best_mate = pid
                if best_mate is not None and self.pass_to(best_mate):
                    return
        self.kick(kick_direction, kick_power)
    else:
        self.release_kick()
        self.walk_to(
            _behind_ball(ball.x, ball.y, kick_target, CHASE_BEHIND_M)
        )
```

---

## P1-7: 定位球类型区分（main.py: `_act_our_set_play()`）

### 当前问题

无论界外球/角球/球门球/任意球，全部回调 `_act_normal`，无任何定位球战术。

### 改动方案

按 set_play 类型执行不同策略：

| 类型 | 策略 |
|------|------|
| THROW_IN | 距球最近者走到球位，短传给最近的接应队友 |
| CORNER_KICK | 距球最近者走向角球点传中，另外两人禁区抢点 |
| GOAL_KICK | 守门员从禁区发球给后卫，后卫接球推进（回调 normal） |
| DIRECT_FREE_KICK | 选最佳射门角度的人直接射门，其余抢点补射 |
| INDIRECT_FREE_KICK | 短传配合后射门（类似开球） |
| PENALTY_KICK | 选一个人直接射门 |

### 具体代码

**main.py — 替换 `_act_our_set_play()`（约第 324 行）：**

```python
def _act_our_set_play(context: Context, players: list[Player], store) -> None:
    if not players:
        return

    set_play = get_set_play_type(context)

    if set_play == SetPlay.THROW_IN:
        _act_set_play_throw_in(context, players, store)
    elif set_play == SetPlay.CORNER_KICK:
        _act_set_play_corner(context, players, store)
    elif set_play == SetPlay.GOAL_KICK:
        _act_set_play_goal_kick(context, players, store)
    elif set_play in (SetPlay.DIRECT_FREE_KICK, SetPlay.INDIRECT_FREE_KICK):
        _act_set_play_free_kick(context, players, store)
    elif set_play == SetPlay.PENALTY_KICK:
        _act_set_play_penalty(context, players, store)
    else:
        _act_normal(context, players, store)


def _act_set_play_throw_in(context: Context, players: list[Player], store) -> None:
    """界外球：发球人走向球位，短传最近的队友。"""
    attacker = _select_closest_attacker(context, players)
    rest = [p for p in players if p is not attacker]
    ball = context.ball

    if ball is not None:
        # 发球人走到球的位置
        attacker.action = "throw_in"
        # 走到球边，踢向最近的队友
        if rest:
            target = min(rest, key=lambda p: dist(
                p.pose.x, p.pose.y, ball.x, ball.y,
            )) if rest[0].pose is not None else rest[0]
            # 靠近球后踢给队友
            attacker.walk_to((ball.x, ball.y), face=0.0, avoid_ball=False)
        for p in rest:
            p.action = "throw_in_support"
            p.move_to_position((ball.x + 2.0, p.pose.y))
    else:
        _act_normal(context, players, store)


def _act_set_play_corner(context: Context, players: list[Player], store) -> None:
    """角球：发球→禁区，两人门前抢点。"""
    attacker = _select_closest_attacker(context, players)
    rest = [p for p in players if p is not attacker]

    ball = context.ball
    if ball is not None:
        attacker.action = "corner_kicker"
        # 直接踢向球门前
        goal_x = context.field.length / 2.0
        kick_dir = angle_to(ball.x, ball.y, goal_x, 0.0)
        attacker.kick(kick_dir, power=6.0)
    else:
        attacker.stop()

    # 其余人门前抢点，站在距门 2m 处左右拉开
    slot_y = [-0.8, 0.8]
    for p, sy in zip(rest, slot_y):
        p.action = "corner_charge"
        p.walk_to(
            (context.field.length / 2.0 - 1.0, sy),
            face=0.0, avoid_ball=True, avoid_robots=True,
        )


def _act_set_play_goal_kick(context: Context, players: list[Player], store) -> None:
    """球门球：守门员推给后卫，后卫推进 = normal"""
    _act_normal(context, players, store)


def _act_set_play_free_kick(context: Context, players: list[Player], store) -> None:
    """任意球：如果角度能直接射门则直接射，否则短传。"""
    attacker = _select_closest_attacker(context, players)
    rest = [p for p in players if p is not attacker]

    ball = context.ball
    if ball is not None:
        if attacker.kick_can_score(angle_to(ball.x, ball.y, *opponent_goal(context))):
            attacker.action = "free_kick_shoot"
            attacker.kick()
        else:
            attacker.action = "free_kick_pass"
            attacker.kick(0.1, power=3.0)

    for p in rest:
        p.action = "free_kick_support"
        p.move_to_position((ball.x + 3.0, p.pose.y * 0.5 + 0.5)) if ball is not None else p.stop()


def _act_set_play_penalty(context: Context, players: list[Player], store) -> None:
    """点球：选主罚者直接射门，其余人在禁区外。"""
    attacker = _select_closest_attacker(context, players)
    attacker.action = "penalty_shoot"
    attacker.kick()
    for p in players:
        if p is not attacker:
            p.action = "penalty_wait"
            p.stop()
```

---

## P1-8: 对方定位球全员回防（main.py: `_act_opp_set_play()`）

### 当前问题

对方获得定位球时，我方仍按 normal 逻辑（一人冲到对方半场），防守空虚。

### 改动方案

全员退回己方半场防守，守门员守门，另外两人分别站在球→球门连线左右两侧挡射门路线。

### 具体代码

**main.py — 替换 `_act_opp_set_play()`（约第 339 行）：**

```python
def _act_opp_set_play(context: Context, players: list[Player], store) -> None:
    """对方定位球：全员回防，堵射门路线。"""
    if not players:
        return

    # 守门员回门线
    gx, gy = own_goal(context)
    guard = min(players, key=lambda p: dist(p.pose.x, p.pose.y, gx, gy))
    rest = [p for p in players if p is not guard]

    guard.guard()

    ball = context.ball
    if ball is not None:
        # 其余人在球→球门线两侧 1.5m 处封堵
        bx, by = ball.x, ball.y
        dx, dy = gx - bx, gy - by
        d = math.hypot(dx, dy)
        if d > 1e-6:
            ux, uy = dx / d, dy / d
            # 法向量
            nx, ny = -uy, ux
            for i, p in enumerate(rest):
                side = 1.0 if i % 2 == 0 else -1.0
                # 站在球到球门连线中间点，左右分开 1.2m
                mid_x = bx + ux * d * 0.5
                mid_y = by + uy * d * 0.5
                wall_x = mid_x + nx * side * 1.2
                wall_y = mid_y + ny * side * 1.2
                # 不要站到球门后面
                wall_x = max(wall_x, -context.field.length / 2.0 + 0.5)
                p.action = "def_wall"
                p.move_to_position((wall_x, wall_y))
        else:
            for p in rest:
                p.action = "def_fallback"
                p.move_to_position((-4.0, 0.0))
    else:
        for p in rest:
            p.action = "def_fallback"
            p.move_to_position((-4.0, 0.0))
```

---

## P1-9: 球速度方向预测 + 拦截（main.py）

### 当前问题

每帧选距离**球当前位置**最近的人做 attacker。球运动时球员追的不是拦截点而是球屁股后面，永远慢一步。

### 改动方案

在 `store` 中记录上一帧的球位置，计算速度向量，预测球未来的位置，用预测位置选 attacker。

### 具体代码

**main.py — `init_store()` 加字段（约第 115 行），`play()` 中计算（约第 122 行）：**

```python
def init_store(self, store) -> None:
    _log.info("init_store called")
    store.prev_phase = None
    store.cur_phase = None
    store.kickoff_taker = None
    store.normal_attacker = None
    store.prev_ball_x = None   # 新增：球速预测
    store.prev_ball_y = None
    store.prev_ball_time = None
```

**main.py — 新增辅助函数（放在 `_player_dist_to_ball` 上方，约第 195 行）：**

```python
def _predict_ball_pos(
    context: Context, store,
    predict_ahead: float = 0.5,
) -> tuple[float, float] | None:
    """根据历史位置预测球在 ``predict_ahead`` 秒后的坐标。
    
    返回 ``(x, y)`` 或 None（无法预测时回退到球当前位置）。
    """
    ball = context.ball
    if ball is None:
        return None

    prev_x = getattr(store, "prev_ball_x", None)
    prev_y = getattr(store, "prev_ball_y", None)
    prev_t = getattr(store, "prev_ball_time", None)

    if prev_x is not None and prev_y is not None and prev_t is not None:
        dt = ball.last_seen_at - prev_t
        if 0.001 < dt < 1.0:
            vx = (ball.x - prev_x) / dt
            vy = (ball.y - prev_y) / dt
            return (ball.x + vx * predict_ahead, ball.y + vy * predict_ahead)

    return (ball.x, ball.y)
```

**main.py — `play()` 中每帧更新球历史（在 phase 分派之前，约第 160 行）：**

```python
# 更新球历史（用于速度预测）
if context.ball is not None:
    store.prev_ball_x = context.ball.x
    store.prev_ball_y = context.ball.y
    store.prev_ball_time = context.ball.last_seen_at
```

**main.py — 修改 `_select_closest_attacker()`（约第 208 行）使用预测位置：**

```python
def _select_closest_attacker(
    context: Context,
    players: list[Player],
    preferred_id: int | None = None,
    store=None,
) -> Player:
    # 使用预测球位选人
    if store is not None:
        pred = _predict_ball_pos(context, store)
        target_x, target_y = pred if pred is not None else (
            context.ball.x if context.ball is not None else (0.0, 0.0)
        )
    else:
        target_x = context.ball.x if context.ball is not None else 0.0
        target_y = context.ball.y if context.ball is not None else 0.0

    ranked = []
    for p in players:
        d = dist(p.pose.x, p.pose.y, target_x, target_y) + _fallen_time_cost(p)
        ranked.append((p, d))
    best, best_dist = min(ranked, key=lambda item: item[1])
    preferred = next((item for item in ranked if item[0].id == preferred_id), None)
    if (
        preferred is not None
        and preferred[1] <= best_dist + ATTACKER_KEEP_DIST_MARGIN_M
    ):
        return preferred[0]
    return best
```

**main.py — 更新所有 `_select_closest_attacker` 调用处传 `store`（3 处）：**

第 236 行 `_act_normal` 中：
```python
attacker = _select_closest_attacker(context, players, getattr(store, "normal_attacker", None), store)
```

第 265 行 `_act_our_kickoff` 中：
```python
store.kickoff_taker = _select_closest_attacker(context, players, store=store).id
```

---

## P1-10: 禁区内轻推射门（player.py: `attack()`）

### 当前问题

无论距离球门多远都用同一力度（默认 5.0），近距离也大力射门容易打飞或被挡。

### 改动方案

在 `attack()` 中，当球靠近球门（`ball.x > 4.0`）时，降低射门力度到 2.0-3.0，增加精度。

### 具体代码

**player.py — 修改 `attack()`（约第 571 行），在调用 `plan_kick()` 后覆盖力度：**

```python
    if self._kicking:
        kick_plan = self.plan_kick()
        if kick_plan is None:
            self.stop()
            return
        kick_direction, kick_power = kick_plan
        # P1-10: 禁区内轻推
        if ball.x > 4.0:
            kick_power = clamp(kick_power * 0.5, 2.0, 3.5)
        # P1-6: 角度太差时传球（接续已有修改）
        if not self.kick_can_score(kick_direction):
            ...
        self.kick(kick_direction, kick_power)
```

> 注意：这段需要与 P1-6 中对 `attack()` 的修改合并，最终 `attack()` 中的踢球逻辑是：
> 1. 调用 `plan_kick()` 获取方向和力度
> 2. 如果在禁区内（ball.x > 4.0），力度减半（2.0-3.5）
> 3. 如果角度无法射门，尝试传球给最近球门的队友
> 4. 执行 kick()

**param.py — 新增禁区力度参数（可选）：**

```python
KICK_POWER_CLOSE_RANGE = 3.0     # 禁区附近射门力度
```

---

## 修改清单汇总

| # | 文件 | 修改内容 | 行号 |
|---|------|---------|------|
| P1-6a | `src/player.py` | 新增 `pass_to(teammate_id)` | ~540-570 |
| P1-6b | `src/player.py` | 修改 `attack()` — 射门角度差时传球 | ~571-598 |
| P1-7 | `src/main.py` | 替换 `_act_our_set_play()` — 5 种定位球策略 | ~324-336 |
| P1-8 | `src/main.py` | 替换 `_act_opp_set_play()` — 全员回防堵门 | ~339-341 |
| P1-9a | `src/main.py` | `init_store()` 加球历史字段 | ~115-121 |
| P1-9b | `src/main.py` | 新增 `_predict_ball_pos()` | ~195-213 |
| P1-9c | `src/main.py` | `play()` 中更新球历史 | ~160 区域 |
| P1-9d | `src/main.py` | 修改 `_select_closest_attacker()` 用预测位置 | ~208-225 |
| P1-10 | `src/player.py` | `attack()` 禁区内减力度 | ~571-598 合并 |

### 执行顺序

1. **P1-6a**: 先加 `pass_to()`（纯新增，无冲突）
2. **P1-6b + P1-10**: 合并修改 `attack()`（两个改动在同一方法，需一起做）
3. **P1-9a → P1-9b → P1-9c → P1-9d**: 球预测，涉及 4 处修改（顺序依赖）
4. **P1-7 + P1-8**: 定位球策略（修改两个函数，独立）

### 验证方法

- **P1-6**: 观察 attacker 在边线/底线附近时是否传球而非强行射门
- **P1-7**: 模拟不同定位球，观察球员位置和动作是否符合预期
- **P1-8**: 对方发定位球时，三人是否全部退守己方半场
- **P1-9**: 球快速运动时，attacker 是否提前跑向球路前方拦截
- **P1-10**: 禁区内射门时控制台 power 值是否降到 2.0-3.5
