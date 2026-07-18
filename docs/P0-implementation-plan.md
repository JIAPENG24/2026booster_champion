# P0 级修复实现方案

基于 `docs/strategy-optimization-notes.md` 中定义的 P0 优先级，共 5 项修复。

**修改范围**：`src/param.py` `src/player.py` `src/main.py`

---

## P0-1: 射门目标多样化（player.py: `plan_kick()`）

### 当前问题

`plan_kick()` 永远计算射向对方球门正中心 `(7.25, 0)`，守门员和防守人很容易预判封堵。

### 改动方案

将 `plan_kick()` 中的固定目标 `opponent_goal(ctx)` 替换为根据球的位置选择左/右门柱的逻辑。

### 具体代码

**修改 `player.py` `plan_kick()` 方法 (约第 232-252 行)：**

```python
def plan_kick(self) -> tuple[float, float] | None:
    ctx = self.context
    ball = ctx.ball if ctx is not None else None
    if ctx is None or ball is None:
        return None

    half_goal = ctx.field.goal_width / 2.0
    goal_x = ctx.field.length / 2.0

    # 根据球的位置选择射门目标
    if ball.y > 0.5:
        kick_target = (goal_x, -half_goal + 0.2)
    elif ball.y < -0.5:
        kick_target = (goal_x, half_goal - 0.2)
    else:
        side = 1.0 if self.id % 2 == 0 else -1.0
        kick_target = (goal_x, side * half_goal * 0.3)

    kick_direction = angle_to(ball.x, ball.y, *kick_target)
    kick_power = (
        KICK_POWER_BACKFIELD if self._in_backfield()
        else KICK_POWER_DEFAULT
    )

    self._draw_kick_target(kick_target)
    return kick_direction, kick_power
```

---

## P0-2: 守门员横向跟随（player.py: `guard()`）

### 当前问题

`guard()` 始终站小禁区中心 `(-6.5, 0)`，不随球的横向位置移动。对方从边路进攻时球门大半边空门。

### 改动方案

守门员 y 坐标跟随球的 y 坐标按比例移动，限制在球门宽度内。球远侧时回中。

### 具体代码

**修改 `player.py` `guard()` 方法 (约第 595-619 行)：**

```python
def guard(self) -> None:
    ctx = self.context
    if ctx is None or self.pose is None:
        self.action = "guard:stop"
        self.stop()
        return

    ball = ctx.ball
    home_x = own_goal_area_center(ctx)[0]

    if ball is not None:
        max_y = ctx.field.goal_width / 2.0 - 0.3
        target_y = clamp(ball.y * 0.5, -max_y, max_y)
        if ball.x > 0:
            target_y *= 0.6
    else:
        target_y = 0.0

    face = 0.0
    if GUARD_FACE_BALL and ball is not None:
        face = angle_to(self.pose.x, self.pose.y, ball.x, ball.y)

    self.action = "guard:home"
    debugdraw.point(home_x, target_y, rgb=(0.0, 0.6, 1.0), scale=0.2, ns="guard_home")
    self.walk_to((home_x, target_y), face=face, avoid_ball=True, avoid_robots=True)
```

---

## P0-3: 我方开球短传配合（main.py: `_act_our_kickoff()`）

### 当前问题

开球手 `attacker.kick(0.1, KICK_POWER_OUR_KICKOFF)` 直接把球踢向对方，另外两人原地 stop 无人接应。

### 改动方案

开球手短传给最近的队友，接应队员向前跑位接球。

### 具体代码

**修改 `main.py` `_act_our_kickoff()` (约第 257-284 行)：**

```python
def _act_our_kickoff(context: Context, players: list[Player], store) -> None:
    if not players:
        return

    active_ids = {p.id for p in players}
    if store.prev_phase != Phase.OUR_KICKOFF or store.kickoff_taker not in active_ids:
        store.kickoff_taker = _select_closest_attacker(context, players).id

    attacker_id = store.kickoff_taker
    attacker = next((p for p in players if p.id == attacker_id), None)
    if attacker is None:
        return

    rest = [p for p in players if p is not attacker]
    gx, gy = own_goal(context)
    guard = min(rest, key=lambda p: dist(p.pose.x, p.pose.y, gx, gy)) if rest else None
    supporters = [p for p in rest if p is not guard] if guard else rest

    if guard is not None:
        guard.guard()

    ball = context.ball

    if store.prev_phase != Phase.OUR_KICKOFF:
        if supporters and ball is not None:
            target = min(supporters, key=lambda p: dist(
                p.pose.x, p.pose.y, ball.x, ball.y,
            ))
            if target.pose is not None:
                kick_dir = angle_to(ball.x, ball.y, target.pose.x, target.pose.y)
                attacker.action = "kickoff_pass"
                attacker.kick(kick_dir, power=2.5)
            else:
                attacker.action = "kickoff"
                attacker.kick(0.1, KICK_POWER_OUR_KICKOFF)
        else:
            attacker.action = "kickoff"
            attacker.kick(0.1, KICK_POWER_OUR_KICKOFF)

        for p in supporters:
            p.action = "kickoff_support"
            if ball is not None:
                p.walk_to(
                    (ball.x + 2.0, p.pose.y * 0.5),
                    avoid_ball=True, avoid_robots=True,
                )
    else:
        _act_normal(context, players, store)
```

---

## P0-4: support 动态站位（player.py: `support()`）

### 当前问题

`support()` 固定站在球→己方门连线距球 3.0m 处。球靠近底线时站位太靠边无用，球在对方半场时太靠前导致防守空虚。

### 改动方案

根据球的 x 坐标决定支援距离：球在己方半场→收紧防守（1.5-2.5m），球在对方半场→拉开空间（3.5-4.0m）。

### 具体代码

**修改 `player.py` `support()` 方法 (约第 621-649 行)：**

```python
def support(self) -> None:
    ctx = self.context
    if ctx is None:
        self.stop()
        return

    ball = ctx.ball
    if ball is None:
        self.move_to_position(own_goal_area_center(ctx))
        return

    if ball.x < -3.0:
        dist_factor = 1.5
    elif ball.x < 0:
        dist_factor = 2.5
    elif ball.x < 3.0:
        dist_factor = 3.5
    else:
        dist_factor = 4.0

    gx, gy = own_goal(ctx)
    bx, by = ball.x, ball.y
    dx, dy = gx - bx, gy - by
    d = math.hypot(dx, dy)
    if d < 1e-6:
        ux, uy = -1.0, 0.0
    else:
        ux, uy = dx / d, dy / d
    along = min(dist_factor, d)
    tx = bx + ux * along
    ty = by + uy * along

    half_l = ctx.field.length / 2.0
    half_w = ctx.field.width / 2.0 - 0.3
    tx = clamp(tx, -half_l + 0.3, half_l)
    ty = clamp(ty, -half_w, half_w)

    self.move_to_position((tx, ty))
```

---

## P0-5: KICK_POWER_BACKFIELD 加大（param.py）

### 当前问题

`KICK_POWER_BACKFIELD = 5.0` 与 `KICK_POWER_DEFAULT` 相同，后场大脚力度不够。

### 改动方案

5.0 → 8.0

### 具体代码

**修改 `param.py` 第 22 行：**

```python
KICK_POWER_BACKFIELD = 8.0
```

---

## 修改清单汇总

| # | 文件 | 修改内容 | 行号 |
|---|------|---------|------|
| P0-1 | `src/player.py` | `plan_kick()` — 射门目标多样化 | ~232-252 |
| P0-2 | `src/player.py` | `guard()` — 守门员横向跟随 | ~595-619 |
| P0-3 | `src/main.py` | `_act_our_kickoff()` — 短传开球 | ~257-284 |
| P0-4 | `src/player.py` | `support()` — 动态支持距离 | ~621-649 |
| P0-5 | `src/param.py` | `KICK_POWER_BACKFIELD` 5→8 | 第 22 行 |

### 执行顺序

1. **P0-5**: param.py 1 行修改
2. **P0-1 + P0-2 + P0-4**: player.py 改 3 个方法，可并发
3. **P0-3**: main.py 开球逻辑，最复杂放最后
