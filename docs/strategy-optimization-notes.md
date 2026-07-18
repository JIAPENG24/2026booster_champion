# 3v3 SoccerSim 策略优化笔记

**只允许修改 `src/main.py` `src/player.py` `src/param.py` 三个文件。**  
`src/framework/` 和 `src/utils/` 均不可修改。

---

## 场地与数据速查

| 项目 | 值 |
|------|-----|
| 场地 | 14m × 9m，中心(0,0)，+X = 对方球门 |
| 球门宽度 | 2.6m |
| 中圈半径 | 1.5m |
| 小禁区(goal area) | 1m × 4m，从我方门线沿+X伸入 |
| 大禁区(penalty area) | 3m × 6m |
| 球门深度 | 0.6m |
| 对方球门中心 | (7.25, 0) |
| 我方球门中心 | (-7.0, 0) |
| 人数 | 每队 3 人（id 1-3） |

### Context 关键字段

```
context.ball         → BallState {x, y, last_seen_at, confidence}
context.teammates    → dict[int, RobotState]
context.opponents    → dict[int, RobotState]
context.game         → GameControlState {state, set_play, kicking_team, secondary_time, stopped}
context.field        → FieldDimensions {length=14, width=9, goal_width=2.6, circle_radius=1.5, goal_area_length=1, ...}
context.team_id      → 1 或 2
```

### Player 关键方法

| 方法 | 说明 |
|------|------|
| `walk_to(xy, *, avoid_ball, avoid_robots, face)` | 走位核心，含全局A* + 局部VFH |
| `attack(kick_target)` | 追球射门（不避障） |
| `guard()` | 回小禁区防守 |
| `support()` | 球→己方门连线上支援 |
| `kick(direction, power)` | 直接踢球，方向用场地坐标 |
| `plan_kick()` | 默认射向球门中心 |
| `kick_can_score(direction)` | 直线轨迹能否进球 |
| `block_path_projection(oid)` | 自己在对手→球连线上的垂足 |
| `move_to_position(target)` | 走到站位点，面向球，避球避人 |
| `ensure_ready()` → bool | 活性自理，本帧能否行动 |
| `stop()` | 停车 + 释放踢球 |

### store 跨帧状态

```
store.cur_phase / prev_phase  → Phase 边缘检测
store.normal_attacker          → normal 阶段的 attacker id（防震荡）
store.kickoff_taker            → 开球手 id
```

---

## 一、main.py 策略层

### 1.1 NORMAL 角色分配（`_act_normal`）

当前：`最近球者 attack → 剩者离己方门最近者 guard → 其余 support`

**弱点：无敌方意识、无球轨迹预测、角色固定**

优化方向：

1. **加球运动方向预测**：检测 `context.ball` 每帧位移，判断球是向我方还是对方半场运动
2. **动态角色交换**：如果 attacker 倒地/被罚下/很久没接近球，立刻换人
3. **盯人防守**：support 应去挡"对方持球人→我方球门"的连线
4. **进攻/防守权重区分**：球在我方半场→全员回收，球在前场→双人前压

### 1.2 定位球策略

当前 `_act_our_set_play` / `_act_opp_set_play` 全部回调 `_act_normal`。

**必须按类型区分：**

| 定位球类型 | 我方主罚 | 对方主罚 |
|-----------|---------|---------|
| 界外球 | 边路发球，两人拉开接应 | 就近逼抢 |
| 角球 | 一脚踢向门前后卫抢点 | 全员回防堵门 |
| 球门球 | 短传给后卫组织推进 | 前压逼抢 |
| 直接任意球 | 直接射门 | 人墙堵门 |
| 间接任意球 | 短传配合后射门 | 人墙 + 盯人 |

### 1.3 开球（`_act_our_kickoff`）

当前：`attacker.kick(0.1, 5.0)` — 直接踢给对方

应该改为：短传给最近的队友（传入中后场），由队友组织进攻。guard 和 support 不要 stop，要走位接应。

### 1.4 READY 站位（`_act_ready`）

注意：READY 阶段 `g.kicking_team` 指示哪方开球，利用好这个信息。

- 我方开球：1人站球旁，1人站中后场，1人站边路
- 对方开球：1人站门线，1人中圈外堵截，1人中卫位置

### 1.5 全场统一改进

- 加 `store` 字段记录"上次球位置"来计算球的速度向量
- 减少 phase 切换时的震荡（用 `prev_phase` 检测边缘）
- 所有 `_act_*` 函数保证不抛出异常（否则整个线程崩掉）

---

## 二、player.py 动作层

### 2.1 `attack()` 改进

当前：不避障、永远冲球、永远射门中心

- 接近球时不要直奔球位，而是提前切到球路线上
- 进入踢球范围后带球变向绕过对手再射
- 禁区内用轻推（power=2-3）避免打飞
- 角度太小（<30°）时传球给位置更好的队友

### 2.2 `guard()` 改进

当前：站小禁区中心 + 面向球，不做横向移动

- 横向跟随：守门员 y 坐标应随球 y 坐标移动（比例 0.4-0.6 倍）
- 出击逻辑：球在 `x > -5.0`（距门约 2m）时主动前冲出禁区缩小角度
- 球在对方半场时站位略微前提（x = -5.0 附近），便于接球组织

### 2.3 `support()` 改进

当前：固定站在球→己方门连线距球 3.0m 处

- 改为动态距离：球在己方半场时距球 2m（密集防守），球在对方半场时距球 4m（拉开空间）
- 盯对方最危险球员：算每个对手到球门的威胁，站位堵在传球路线上
- 如果 support 已经很久不动（卡住），临时切换为 attacker 替换

### 2.4 增加 `pass_to()`

```python
def pass_to(self, teammate_id: int) -> bool:
    ctx = self.context
    mate = ctx.teammates.get(teammate_id)
    if mate is None or mate.pose is None:
        return False
    kick_dir = angle_to(ctx.ball.x, ctx.ball.y, mate.pose.x, mate.pose.y)
    dist_to_mate = dist(ctx.ball.x, ctx.ball.y, mate.pose.x, mate.pose.y)
    power = clamp(dist_to_mate * 1.5, 2.0, 6.0)  # 距离越远越大力
    self.kick(kick_dir, power)
    return True
```

### 2.5 `plan_kick()` 改进

当前：永远射向球门正中心 `(7.25, 0)`

- 根据球的位置选择射左/右角：球在左半场→射右门柱，球在右半场→射左门柱
- 守门员在中间时射角落，守门员偏一侧时射另一侧
- 远距离（球在我方半场）用大脚解围而不是射门
- 角度差 > 60° 时放弃射门改为传球

### 2.6 踢球迟滞参数

`KICK_ENTER_M = 2.0` 有点早，可以减小到 1.2-1.5，让球员更靠近球再踢。

---

## 三、param.py 参数调优

### 运动控制

```python
MAX_LINEAR = 2.0           # 够用；如追不上对方可加
MAX_ANGULAR = 2.0          # 转向速度；太慢可加到 2.5-3.0
LINEAR_GAIN = 1.5          # 平移P增益；走得抖就降
ANGULAR_GAIN = 2.0         # 转向P增益；转向慢就加
TURN_THRESHOLD = 0.5       # 转身阈值(rad), 0.5≈28°；可降到0.3
OMNI_DIST = 1.0            # 全向控制距离
ARRIVE_DIST = 0.15         # 到达判定；可降到0.1
```

### 踢球

```python
KICK_POWER_DEFAULT = 5.0       # 中远射力度
KICK_POWER_BACKFIELD = 5.0     # 后场出球 → 加大到7-8
KICK_POWER_OUR_KICKOFF = 5.0   # 开球 → 减小到2-3（短传）
KICK_ENTER_M = 2.0             # → 减小到1.2-1.5
KICK_EXIT_M = 2.5              # 迟滞0.5m，合理
CHASE_BEHIND_M = 0.35          # 追球站位后退量；太小碰不到球，可0.4-0.5
```

### 战术

```python
SUPPORT_DIST_M = 3.0           # → 动态 2.0-4.0
ATTACKER_KEEP_DIST_MARGIN_M = 0.3  # 合理
FALLEN_COST = 10.0             # 摔倒惩罚 → 减小到5.0
GUARD_FACE_BALL = True         # 合理
GOAL_TARGET_DEPTH_M = 0.25     # 打门柱就加大到0.35
```

### 避障

```python
BALL_OBSTACLE_RADIUS = 0.5    # 进攻时ball不应作为障碍（attack()不传avoid_ball）
OPPONENT_RADIUS = 0.55        # 常碰撞就加
TEAMMATE_RADIUS = 0.48        #
SAFETY_MARGIN = 0.22          # → 可0.25-0.30
```

### 路径规划

```python
USE_GLOBAL_PATH_PLANNER = True      # 计算量大；如果卡顿设为False
GLOBAL_GRID_RESOLUTION_M = 0.35     # 够用
GLOBAL_PATH_LOOKAHEAD_M = 0.9       # 合理
PLAN_LOOKAHEAD = 1.2                # 探测距离
PLAN_STEP = 15°                      # 绕不过障碍就减到10°
PLAN_CLEARANCE = 0.35               # 太保守→0.2-0.25
PLAN_MAX_OFFSET = 100°              # 够用
```

---

## 四、优化优先级

### P0 — 立刻改（直接影响胜负）

1. 射门目标多样化：不永远射中心 (`plan_kick` → player.py)
2. 守门员横向跟随 (`guard` → player.py)
3. 我方开球不要直接踢给对方 (`_act_our_kickoff` → main.py)
4. support 动态站位 / 盯人 (`support` → player.py)
5. KICK_POWER_BACKFIELD 加大到 7-8 (param.py)

### P1 — 明显提升

6. 进攻加传球 (`pass_to` → player.py, main.py)
7. 定位球类型区分 (`_act_our_set_play` → main.py)
8. 对方定位球全员回防 (`_act_opp_set_play` → main.py)
9. 球速度方向预测 + 拦截 (`main.py`)
10. 禁区内轻推射门 (`attack` → player.py)

### P2 — 进阶打磨

11. 带球变向过人 (`attack` → player.py)
12. 守门员出击 (`guard` → player.py)
13. 动态角色交换（attacker 卡死时换人）
14. keep-away 控球拖延时间
15. READY 站位精细化

---

## 五、调参方法论

1. **单变量原则**：每次只改一个参数，在模拟器中观察 2-3 分钟
2. **先参数后代码**：能在 param.py 解决的不要改 player.py，能在 player.py 解决的不要改 main.py 结构
3. **可视化验证**：`debugdraw.*` 已覆盖：目标点绿点、规划路径青线、heading 黄箭头、角色标签。善用这些
4. **看控制台**：每帧打印 `phase= state= set= secondary_time=` 以及心跳日志（每 ~2s）

---

## 六、常见踩坑

- `attack()` 内部 `walk_to` 不传 `avoid_ball/robots`，所以会撞人和撞球——这是设计的（直取球）。但 support/guard 传了 `avoid_ball=True, avoid_robots=True`
- `_act_normal` 中的 `players` 已经是过滤后的"可行动球员"（pose 已知、已就绪、未被罚下）
- `kick()` 参数方向是**场地坐标系**，底层自动转体坐标系
- `context.ball` 可能为 None（球不可见），所有用到的地方都要判空
- `g.secondary_time` 单位是**秒**（int），不是 tick
- 球员摔倒后 `ensure_ready()` 返回 False，不会进入 active 列表，不会参与角色分配
- `plan_kick()` 返回 `(方向, 力度)`，但 `_goal_target_for_direction` 只用于可视化，不影响实际踢球
- `walk_to` 中的 `arrive_dist` 默认 0.15m，如果目标点频繁变化可能导致球员永远到不了
- 全局 A* (`USE_GLOBAL_PATH_PLANNER`) 如果找不到路径就 fallback 到局部 VFH。表演赛环境可能卡顿，关闭全局规划可能更稳
