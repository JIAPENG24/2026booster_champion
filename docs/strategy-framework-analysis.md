# 3v3 机器人足球仿真 — 策略框架现状分析

> 本文档基于 `src/` 全部核心文件通读整理，描述**当前已实现**的策略体系。
> 适用对象：策略重写、调参、新增角色时的现状参考。
> 控制频率：30Hz（`control_hz=30`）。依赖：`py_trees==2.4.0`。
> 运行环境：Booster Studio 虚拟机器人（基于 booster_agent_framework）。

---

## 目录

1. [整体策略框架](#1-整体策略框架)
2. [独立区策略内容](#2-独立区策略内容)
3. [运动控制参数（详细分类）](#3-运动控制参数详细分类)
4. [角色分配形式](#4-角色分配形式)

---

## 1. 整体策略框架

### 1.1 分层架构（单向依赖，下层不依赖上层）

```
┌─────────────────────────────────────────────────────┐
│  main.py            Booster Agent 入口               │
├─────────────────────────────────────────────────────┤
│  runtime.py         SoccerKit 装配 + 控制循环(30Hz)   │  ← 装配层
├─────────────────────────────────────────────────────┤
│  play/              PLAY 阶段动态角色策略核心          │  ← 策略重写主入口
│   ├─ playbook.py    角色分配 + chaser 选举             │
│   ├─ default_roles.py  4 个默认角色实现                │
│   ├─ play_subtree.py   PLAY 子树 + 开球阶段机          │
│   ├─ nodes.py       PLAY 叶节点                       │
│   └─ role.py        RoleStrategy 基类 + Registry       │
├─────────────────────────────────────────────────────┤
│  behavior_tree/     BT 运行时框架层                    │
├─────────────────────────────────────────────────────┤
│  tactics/           纯数学/物理模型层（无 BT、无 ROS） │
│   ├─ motion.py      反应式运动控制器                   │
│   ├─ targeting/     战术目标层（纯函数）               │
│   ├─ navigation.py  障碍收集                          │
│   ├─ ready_stance.py READY 定位                       │
│   └─ kick_hysteresis.py 踢球滞回                      │
├─────────────────────────────────────────────────────┤
│  soccer_framework/  硬件适配层，公共 API               │  ← 最底层
│   ├─ config.py      SoccerStrategyTuning 全调参        │
│   ├─ types.py       PlayContext/RobotCommand/数据类型  │
│   └─ ...            ROS2 真值/GameController 适配      │
└─────────────────────────────────────────────────────┘
```

**关键设计原则**：`tactics/` 是纯函数层，不依赖 BT 和 ROS，可独立单元测试；`play/` 是策略大脑，通过 `tactics/` 的函数计算目标，通过 `behavior_tree/` 的节点执行。

### 1.2 坐标系约定（至关重要）

统一使用 **team-view 场地坐标**（仿真真值已是相对当前队伍的坐标）：

| 方向 | 含义 |
|------|------|
| `+x` | 进攻方向（对方球门方向） |
| `-x` | 防守方向（我方球门方向） |
| 我方球门 | `x = -field_length/2 = -7.0` |
| 对方球门 | `x = +field_length/2 = +7.0` |

> ⚠️ **禁止在 `play/tactics` 内再次按 `team_id` 镜像坐标**。若未来输入为绝对坐标，应在 `PlayContextProvider` 适配层归一化为 team-view。

### 1.3 场地尺寸（ADULT_FIELD_DIMENSIONS，`types.py`）

| 参数 | 值 | 说明 |
|------|-----|------|
| `length` | 14.0 m | 场地长度 |
| `width` | 9.0 m | 场地宽度 |
| `penalty_dist` | 2.1 m | 罚球区距离 |
| `goal_width` | 2.6 m | 球门宽度 |
| `circle_radius` | 1.5 m | 中圈半径 |
| `penalty_area_length` | 3.0 m | 罚球区长度 |
| `penalty_area_width` | 6.0 m | 罚球区宽度 |
| `goal_area_length` | 1.0 m | 球门区长度 |
| `goal_area_width` | 4.0 m | 球门区宽度 |
| `goal_depth` | 0.6 m | 球门深度 |

### 1.4 顶层行为树结构（`docs/bt_structure.md`）

顶层 `Sequence(TeamRoot, memory=False)`，每 tick 顺序执行 4 大块：

```
TeamRoot (Sequence, memory=False)
│
├─ 1. DataLayer（每帧更新黑板）
│   ├─ UpdateClock          → /clock/now
│   ├─ UpdatePlayContext    → /play_context (PlayContext)
│   ├─ UpdateGameState      → stale 则置 None
│   ├─ UpdateRecentBall     → stale 则置 None
│   ├─ UpdateRobotPoses     → stale 则置 None
│   └─ UpdateRobotStatus(N) → /robot_status/{id}
│
├─ 2. MatchControl (Selector)  ← 比赛阶段控制
│   │
│   ├─ SafetyGuards (Selector, 任一命中即 StopAll)
│   │   ├─ 无 GameController 状态        → StopAll
│   │   ├─ 全员 inactive                  → StopAll
│   │   ├─ stopped=true                   → StopAll
│   │   ├─ 非 PLAYING 状态                → StopAll
│   │   └─ PLAYING 但无球                  → StopAll
│   │
│   ├─ ReadyPhase
│   │   └─ IsGameInState(READY) + Parallel(ReadySlots)
│   │       └─ 各玩家 GoReadyTarget(N)
│   │
│   ├─ PlayingPhase   ← 策略核心
│   │   └─ IsGameInState(PLAYING) + Sequence(PlaybookCore)
│   │       ├─ AssignRoles（每帧调 Playbook.assign_roles）
│   │       └─ Parallel(Roles)（各玩家按角色执行子树）
│   │
│   └─ 兜底 StopAll("unsupported state")
│
├─ 3. SafetyOverrides (Parallel, SuccessOnAll)  ← 硬件兜底
│   └─ 每玩家 PlayerSafety(N)
│       ├─ AllowedGuard:   penalty → StopPlayer
│       ├─ FallDownGuard:  倒地 → TriggerGetUp + StopPlayer
│       └─ WalkModeGuard:  非行走模式但需移动 → TriggerEnterWalkMode
│
└─ 4. CommitTeamCommands  ← 收集最终命令并派发
```

**黑板键（BlackboardKeys）**：`/clock/now`, `/play_context`, `/team/roles`, `/safety/active`, `/robot_status/{id}`, `/cmd/{id}`。

### 1.5 控制循环（`runtime.py` SoccerTeamRuntime）

- 周期 `period = 1/control_hz`（≈33.3ms）
- 每 tick：`tree.tick(now, executor=self)`
- 异常兜底：`robot_manager.stop_all`
- 2s 周期状态日志

### 1.6 PLAY 子树与开球阶段机（`play/play_subtree.py`）

`PlayingPhase` 内嵌一个 **KICKOFF_PHASE 黑板状态机**，4 个阶段按优先级驱动：

```
PlayingPhase = Sequence(IsGameInState(PLAYING), PlayKickoffController, Selector(KickoffPhase))
```

| Phase | 名称 | 进入条件 | 行为 |
|-------|------|----------|------|
| 0 | **NormalPlay** | 默认 / 其他 Phase 退出后回落 | DefaultPlaybook.assign_roles 动态分配 |
| 1 | **ActiveKickoff** | 我方 kickoff：`state=PLAYING, set_play=NONE, kicking_team=us, ball 距中心<1.0m` | CENTER→Kicker（`approach_target(known_ball, _KICKOFF_ANGLE=0.785rad, offset=0.4)` → `KickAtAngle(power=kickoff_kick_power=1.0, landing=2.5m)`）；SIDE→`MoveToLandingPoint`（冲落球点，speed 1.3x）；KEEPER→守门 |
| 2 | **RoleLockPlay** | Phase 1 后球移动>0.15m，记录 kick_at 再延迟 0.5s | `AssignFixedRoles`：center→supporter, side→chaser, keeper→goalkeeper |
| 3 | **OppKickoffDefense** | 对方 kickoff 结束：`secondary_time` 从 >0 降到 0 的下降沿（优先级最高，可抢占 0/1/2） | 各槽位 `approach_target(known_ball, attack_theta, offset=0.4)` 不同速度：CENTER 0.5x, SIDE 1.3x, KEEPER 1.0x |

**优先级**：Phase3 > Phase1 > Phase2 > Phase0。转换由 `PlayKickoffController` 每帧驱动。

**阶段转换条件详解**（5 条转换规则）：
- **Phase 0→1**：我方 kickoff（`state=PLAYING, set_play=NONE, kicking_team=us, ball 距场地中心<1.0m`）
- **Phase 1→2**：球移动>0.15m 后记录 `kick_at`，再等 0.5s 才转 Phase 2
- **Phase 2→0**：SIDE 球员距球<0.3m 持续 1.0s（exit delay），或 2.0s 超时
- **Phase 0/1/2→3**：对方 kickoff 结束（`secondary_time` 从 >0 降到 0 的下降沿），优先级最高可抢占任何阶段
- **Phase 3→0**：我方球员触球（dist<0.3m），或 5.0s 超时

**每玩家子树分支顺序**：
1. `KickoffHold`（IsOpponentKickoffActive → GoReadyTarget）
2. `PenaltyAvoid`（IsOpponentRestartActive → AvoidOpponentRestart）
3. 各 role 注册顺序的 `AsRole` 分支
4. `WaitForBall` 兜底

---

## 2. 独立区策略内容

> "独立区"指场地按球位置划分的功能区域，不同区域触发不同的战术行为。
> 核心判定逻辑在 `tactics/targeting/predicates.py`。

### 2.1 区域划分（按 ball.x）

```
   我方球门            防守区           中场/我方半场        进攻区           对方球门
   x=-7.0  ────────── x=-2.8 ────────── x=+2.8 ────────────── x=+7.0
            │<- gk挑战区->│<-- SIDE 总挑战 -->│<-- SIDE 条件挑战 -->│
                        area_x=-2.8        midfield_or_own_half
```

| 区域 | x 范围 | 判定函数 | 触发行为 |
|------|--------|----------|----------|
| **守门员防守区** | `ball.x < -2.8` 且 `|ball.y| ≤ 2.2` | `ball_in_own_defensive_area` | KEEPER 可参与争球（RUSH_OUT/扑救）；chaser 锁定 3.0s |
| **己方半场/中场** | `ball.x < +2.8` | `ball_is_in_midfield_or_own_half` | SIDE 总是参与挑战；chaser 锁定 2.0s（若 `ball.x < area_x`） |
| **进攻区** | `ball.x ≥ +2.8` | （上述取反） | SIDE 仅当比 CENTER 近 0.20m 才挑战；chaser 锁定 0.5s |
| **危险区** | `ball.x < own_goal_x + 1.5 = -5.5` | desperation 判定 | 守门员 desperation 模式（直接扑球） |

**区域边界计算**（`predicates.py`，函数签名统一为 `(config, ...)`，第一个参数恒为 `SoccerStrategyTuning`）：
- `goalkeeper_defensive_area(config)` → `(area_x, area_y)`：`area_x = -field_length * goalkeeper_challenge_area_x_ratio = -14*0.20 = -2.8m`；`area_y = min(field_width/2-0.35, goalkeeper_challenge_area_y) = min(4.15, 2.2) = 2.2m`
- `ball_in_own_defensive_area(config, ball)`：`ball.x < area_x(-2.8)` 且 `abs(ball.y) ≤ area_y(2.2)`
- `ball_is_in_midfield_or_own_half(config, ball)`：`ball.x < field_length * 0.20 = +2.8m`
- `ball_claim_score(config, slot, pose, ball)`、`side_should_challenge(config, slot, context)`、`pose_for_slot(config, context, slot)` 同样以 config 为首参

### 2.2 边线/球门线恢复区

| 区域 | 判定 | 恢复行为 |
|------|------|----------|
| **近边线** | `|ball.y| ≥ field_width/2 - sideline_recovery_margin_m = 4.5-0.90 = 3.6` | `sideline_recovery_target`：拉回场内，`target_x=ball.x+0.75`, `target_y=ball.y-sign*1.60`，clamp margin=0.35 |
| **球门线外** | `|ball.x| > half_length + goal_line_recovery_margin_m = 7.0+0.15 = 7.15` | 触发恢复逻辑 |
| **我方球门线外** | `ball.x < -7.15` | 守门员/防守优先 |

### 2.3 各区域触发的策略差异（汇总）

#### 防守区（`ball.x < -2.8`）
- **chaser 锁定延长**：`ball.x < own_goal_x+1.5(-5.5)` → 锁 3.0s；`ball.x < area_x(-2.8)` → 锁 2.0s（防乒乓切换）
- **KEEPER 参与争球**：`_slot_can_challenge(KEEPER)` 返回 True（用 `goalkeeper_clear_hold_sec=1.5s` 时间保持 + `goalkeeper_challenge_hysteresis_m=0.30m` 滞回）
- **守门员 desperation**：`ball.x < -5.5` → 直接扑球，`wants_to_kick` 总 True
- **守门员 RUSH_OUT**：球预测停止点在防守区内 → 冲出拦截解围
- **守门员 LATERAL**：球预测将穿过球门线门柱内 → 横向封堵

#### 中场/己方半场（`ball.x < +2.8`）
- **SIDE 总是参与挑战**（`side_should_challenge` 返回 True）
- **chaser 锁定 0.5s**（默认）

#### 进攻区（`ball.x ≥ +2.8`）
- **SIDE 条件挑战**：仅当 `side_dist + 0.20 < center_dist`（SIDE 明显比 CENTER 近）才参与
- **ball.x ≥ 0**：守门员**强制 GUARD** 态（不冲出）
- **chaser 锁定 0.5s**

#### 对方 restart 避让区
- 距球 `< opponent_restart_avoid_distance_m + 0.25 = 1.85m` → escape 模式（推到 `min_distance+0.35` 缓冲）
- 否则 → `base_ready_target`
- 经 `_apply_safety_chain`：`_radius_safe`（确保≥min_distance）+ `_goal_kick_safe`（GOAL_KICK 时推出对方禁区）

### 2.4 开球阶段独立策略（Phase 1/2/3，见 1.6 表）

开球阶段是独立于正常 PLAY 的特殊区域策略，按 KICKOFF_PHASE 状态机驱动，每个槽位（CENTER/SIDE/KEEPER）有独立的速度倍率和目标点。

---

## 3. 运动控制参数（详细分类）

> 全部参数定义在 `soccer_framework/config.py` 的 `SoccerStrategyTuning` dataclass。
> 运动控制器实现在 `tactics/motion.py`（反应式，无全局路径规划）。

### 3.1 速度限制（全局上限）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_linear_speed` | 0.8 m/s | 最大线速度（前进/后退） |
| `max_lateral_speed` | 0.6 m/s | 最大侧向速度（strafe 模式） |
| `max_angular_speed` | 1.0 rad/s | 最大角速度（转弯） |

### 3.2 行走控制常量（`motion.py` 模块级，PLAY/READY 共享）

| 常量 | 值 | 说明 |
|------|-----|------|
| `_ARRIVE_DISTANCE` | 0.15 m | 到达距离阈值 |
| `_ARRIVE_ANGLE` | 0.20 rad (~11.5°) | 到达角度阈值（放松避免微调） |
| `_TURN_THRESHOLD` | 0.5 rad | 大角度误差纯转弯阈值 |
| `_ANGULAR_SPEED_FLOOR` | 0.25 rad/s | 角速度下限（降低小角度过冲） |
| `_ANGULAR_DEAD_ZONE` | 0.15 rad | 死区，低于此用纯比例控制不加 floor |
| `_LINEAR_SPEED_FLOOR` | 0.3 m/s | 线速度下限 |
| `_LINEAR_GAIN` | 2.0 | 线速度增益 |

**速度计算公式**：
- `_linear_speed`：`vx = gain * distance * cos(err) * mult`，floor + clamp 到 `max_linear_speed * mult`
- `_angular_velocity`：`omega = clamp(2*err, ±max)`，死区内纯比例不加 floor，否则 `floor = min(0.25, max_angular_speed)`

### 3.3 踢球滞回（KickHysteresis）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `soccer_kick_enter_distance` | 2.0 m | 进入踢球范围阈值（非 active 时） |
| `soccer_kick_exit_distance` | 2.5 m | 退出踢球范围阈值（active 时保持，>exit 延迟后退出） |
| `soccer_kick_power` | 1.5 | 默认踢球力度 |
| `kickoff_kick_power` | 1.0 | 开球踢球力度 |
| `soccer_kick_min_active_sec` | 0.7 s | 最小 active 持续时间 |
| `soccer_kick_exit_delay_sec` | 0.2 s | 退出延迟 |

**模型**（`kick_hysteresis.py`）：每 `player_id` 一个 `PlayerKickState(active, far_since)`。active 时 `distance ≤ exit` 保持，`> exit` 延迟 `exit_delay` 后退出；非 active 时 `distance ≤ enter` 进入。

### 3.4 路径绕行（第一避障层，改 target）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `opponent_obstacle_radius` | 0.55 m | 对手障碍半径 |
| `teammate_obstacle_radius` | 0.48 m | 队友障碍半径 |
| `obstacle_safety_margin` | 0.22 m | 障碍安全余量 |
| `obstacle_start_ignore_distance` | 0.35 m | 起点附近忽略障碍距离 |
| `obstacle_target_ignore_distance` | 0.35 m | 目标附近忽略障碍距离 |

**机制**（`_avoidance_target`）：找第一个阻挡障碍，生成侧向 via 点为新 target，侧向选择跨帧记忆。

### 3.5 偏航避让（第二避障层，加 vyaw bias）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `yaw_avoid_horizon_sec` | 1.0 s | 预测时域 |
| `yaw_avoid_min_distance_m` | 0.78 m | 触发偏航避让的最小距离 |
| `yaw_avoid_bias_max` | 0.6 rad/s | 最大偏航 bias |

**机制**（`_apply_yaw_avoidance`）：仅作用于 MoveIntent 且有移动；计算每邻居 `yaw_contribution`（scale∈[0,1] 按 `min_distance/horizon`），`side_sign` 按相对 lateral 或 `player_id` 奇偶；累加 bias 到 vyaw，clamp 到 `max_angular_speed`。

### 3.6 定位球 / Restart

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `restart_touch_distance` | 0.45 m | restart 触球距离 |
| `opponent_restart_avoid_distance_m` | 1.6 m | 对方 restart 避让距离（规则 1.45+0.15 缓冲） |

### 3.7 球权仲裁

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `teammate_challenge_tie_margin_m` | 0.15 m | 平局带宽，最小 player_id 胜出 |

### 3.8 传球决策

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pass_enabled` | True | 是否启用传球 |
| `pass_min_score` | 0.60 | 最小传球得分（低于不传） |
| `pass_min_forward_m` | 0.35 m | 最小前向增益（过滤向后传球） |
| `pass_lane_clearance` | 0.75 m | 传球通道净空 |

**得分公式**（`best_pass_target`）：`score = lane_clear*0.55 + forward_gain*0.30 + center_pull*0.15 - distance_penalty`。

### 3.9 盘带

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `dribble_advance_m` | 1.5 m | 盘带前进距离 |
| `dribble_center_pull` | 0.65 | 向中线拉扯系数 |

**目标**（`dribble_target`）：`(ball.x + 1.5, ball.y * 0.65)`。

### 3.10 接应站位

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `support_depth_m` | 1.05 m | 接应纵深 |
| `support_lateral_m` | 1.25 m | 接应横向 |
| `support_min_spacing_m` | 0.9 m | 队友最小间距 |

**动态距离规则**（`support_target`）：当前<1m→目标 1m（后撤）；>2.8m→目标 2.8m（贴近）；否则保持距离仅侧向 strafe 调三角角。

### 3.11 守门员 / 挑战

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `goalkeeper_challenge_area_x_ratio` | 0.20 | 挑战区 x 比例（×field_length=-2.8m） |
| `goalkeeper_challenge_area_y` | 2.2 m | 挑战区 y 半宽 |
| `goalkeeper_challenge_hysteresis_m` | 0.30 m | 挑战滞回（防乒乓） |
| `goalkeeper_clear_hold_sec` | 1.5 s | 解围保持时间 |
| `goalkeeper_rush_speed_multiplier` | 2.2 | 冲出速度倍率 |
| `goalkeeper_kick_power` | 3.5 | 守门员踢球力度（最强） |
| `goalkeeper_lateral_speed` | 1.0 m/s | 横向封堵速度 |
| `goalkeeper_guard_arc_radius` | 1.4 m | 弧形站位半径 |
| `goalkeeper_guard_depth_m` | 1.3 m | 守门深度（goal_line_x = own_goal_x + 1.3） |
| `goalkeeper_rush_speed_ratio` | 0.8 | 冲出速度比例 |

### 3.12 球轨迹预测（BallPredictor，守门员专用）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ball_prediction_history_size` | 10 | 历史帧数 |
| `ball_prediction_kp` | 0.6 | PID 比例 |
| `ball_prediction_ki` | 0.05 | PID 积分 |
| `ball_prediction_kd` | 0.1 | PID 微分 |
| `ball_prediction_friction` | 0.3 | 初始摩擦系数 |
| `ball_prediction_max_horizon_sec` | 2.0 s | 最大预测时域 |

**模型**：指数摩擦 `v(t)=v0*exp(-mu*t)`，`x(t)=x0+(v0/mu)*(1-exp(-mu*t))`。`mu` 在线更新：`0.9*old+0.1*decel`，clamp[0.01,5.0]。提供 `predict_rest_point`（停止点）和 `predict_goal_crossing`（过门线解算）。`predict_goal_crossing` 的过门时间上限为 `max_horizon*3 = 2.0*3 = 6.0s`，超过则返回 None（防止极慢球误判）。

### 3.13 守门员状态机

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gk_state_confirm_frames` | 2 | 进入新状态确认帧数（去抖） |
| `gk_state_release_frames` | 4 | 退出 RUSH_OUT 确认帧数 |
| `gk_target_smooth_speed` | 2.0 m/s | 目标轨迹平滑限速 |
| `gk_rush_out_margin_m` | 0.8 m | RUSH_OUT 触发余量 |
| `gk_rush_out_exit_margin_m` | 0.3 m | RUSH_OUT 退出余量 |
| `gk_lateral_hold_min_sec` | 0.8 s | LATERAL 最小保持时间 |
| `gk_desperation_clear_margin_m` | 1.5 m | desperation 触发余量（ball.x < own_goal_x+1.5） |

### 3.14 边线 / 球门线恢复

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sideline_recovery_margin_m` | 0.90 m | 边线恢复触发余量 |
| `sideline_recovery_infield_m` | 1.60 m | 拉回场内距离 |
| `sideline_recovery_advance_m` | 0.75 m | 边线恢复前进距离 |
| `goal_line_recovery_margin_m` | 0.15 m | 球门线恢复余量 |

### 3.15 导航障碍（`navigation.py` ObstacleCollector，无状态）

| 障碍类型 | 半径 | 说明 |
|----------|------|------|
| 对手 | 0.55 m | `opponent_obstacle_radius` |
| 队友（排除自己） | 0.48 m | `teammate_obstacle_radius` |
| 球门结构（U 形） | 门柱 0.18 / 网边 0.20 | 4 门柱 + 3 网边按 `net_step=0.35` 采样 |
| 守门员专属球门区 | 0.30 / 0.36 | 3+2 区域（仅守门员感知） |

### 3.16 运动控制三层流程（`move_to_target`）

```
每帧输入：当前 pose, target, obstacles, mult(speed_multiplier), strafe, hold_vyaw, lateral_speed
    │
    ▼
1. 路径绕行 _avoidance_target
   └─ 找第一个阻挡障碍 → 生成侧向 via 点为新 target（跨帧记忆侧向选择）
    │
    ▼
2. 行走控制 _compute_velocity
   ├─ 到达检查（distance<0.15 且 angle<0.20）：
   │   ├─ hold_vyaw≠0 → active hold（角跟踪 target.theta）
   │   └─ 否则 → stop
   ├─ 近但未对齐 → 原地转弯
   ├─ strafe 模式：
   │   └─ raw=gain*dist*mult → vx=raw*cos(err), vy=raw*sin(err)
   │       各轴限速，magnitude floor，heading 校正
   └─ 非 strafe：
       ├─ angle_error>0.5 → 纯转弯
       └─ 否则 → vx=linear_speed, vy=0, vyaw=angular_velocity
    │
    ▼
3. 偏航避让 _apply_yaw_avoidance
   └─ 每邻居 yaw_contribution（scale 按 min_distance/horizon）
       累加 bias 到 vyaw，clamp 到 max_angular_speed
```

---

## 4. 角色分配形式

### 4.1 ReadySlot 三槽位（`types.py`）

READY 阶段的静态站位槽位，也是 PLAY 阶段角色分配的输入：

| 槽位 | 枚举 | 默认 player_id | 含义 |
|------|------|----------------|------|
| `CENTER` | ReadySlot.CENTER | 1 | 中路主攻手（默认→Chaser） |
| `SIDE` | ReadySlot.SIDE | 2 | 侧翼（默认→Supporter） |
| `KEEPER` | ReadySlot.KEEPER | 3 | 守门员（默认→Goalkeeper） |

- `DEFAULT_READY_SLOT_SEQUENCE = (CENTER, SIDE, KEEPER)`
- `ready_slots = {1: CENTER, 2: SIDE, 3: KEEPER}`

### 4.2 RoleStrategy 基类契约（`play/role.py`）

```python
class RoleStrategy:
    name: ClassVar[str]  # 动态角色标签，用于 RoleAssignment 映射

    def build_subtree(self, kit: SoccerKit, player_id: int) -> Behaviour:
        """唯一契约：构建该角色的行为子树"""
        ...

    # 以下为辅助方法（非契约，子类可覆盖）：
    def target(self, kit, player_id, context) -> Pose2D: ...
    def wants_to_kick(self, kit, player_id, context) -> bool: ...
    def kick_target(self, kit, player_id, context) -> KickTarget: ...
```

**RoleRegistry**：`register` / `replace` / `unregister` / `get`。**注册顺序 = PLAY Selector 分支优先级**（先注册的角色先匹配）。

### 4.3 RoleAssignment（`playbook.py`，frozen dataclass）

```python
@dataclass(frozen=True)
class RoleAssignment:
    by_player: Mapping[int, str]  # player_id → role_name，MappingProxyType 冻结只读

    def role_of(self, pid: int) -> str: ...      # 缺省返回 ROLE_NONE
    def players_of(self, name: str) -> list[int]: ...  # 反查
```

### 4.4 DefaultPlaybook 角色分配算法（`playbook.py`）

**默认注册顺序**：`ChaserRole` → `SupporterRole` → `GoalkeeperRole`（`DefenderRole` 留给自定义注册，不在默认 playbook）。

#### assign_roles(context) 每帧流程

```
assign_roles(context) → RoleAssignment
│
├─ 1. chaser_id = select_chaser(context)        ← 动态选举主追球手
├─ 2. goalkeeper_id = _configured_goalkeeper()   ← 固定（ReadySlot.KEEPER 的 player_id）
├─ 3. 其余 player_id → ROLE_SUPPORTER
│
└─ 每 2s 周期调用 _log_performance：
    zone = danger/defensive/midfield/attack（按 ball.x 四段分区，**日志专用**）：
      danger(<-field_length*0.25=-3.5) / defensive(<0) / midfield(<+3.5) / attack(≥+3.5)
    ⚠️ 此分区与 predicates 的区域判定（area_x=-2.8, midfield<+2.8）不同，仅用于日志聚合
    记录每玩家 role/dist/pose/in_attack、最近球员、team_centroid_x、比分
```

#### select_chaser(context) 决策优先级（核心算法）

```
select_chaser(context) → player_id
│
├─ Step 1: ReadySlot 资格过滤（_slot_can_challenge）
│   ├─ KEEPER：仅危险球参与（ball_in_own_defensive_area）
│   │   ├─ 时间保持：goalkeeper_clear_hold_sec=1.5s
│   │   └─ 滞回：goalkeeper_challenge_hysteresis_m=0.30m
│   └─ SIDE：仅合适时参与（side_should_challenge）
│       ├─ 中场/我方半场（ball.x<+2.8）：总是 True
│       └─ 进攻区（ball.x≥+2.8）：仅当 side_dist+0.20 < center_dist
│
├─ Step 2: ball_claim_score 成本计算（低分优）
│   ├─ KEEPER：危险区 distance-0.75；否则 distance+field_length（推到最后）
│   ├─ CENTER：distance-0.20（略优先）
│   └─ SIDE：distance-0.10
│
├─ Step 3: 平局仲裁
│   ├─ teammate_challenge_tie_margin_m=0.15m 带宽内 → 最小 player_id 胜出
│   └─ 切换时若旧 chaser 分数 ≤ best+0.5 → 保留旧 chaser（防乒乓）
│
└─ Step 4: 动态 chaser 锁定（防乒乓）
    ├─ ball.x < own_goal_x+1.5(-5.5) → 锁 3.0s
    ├─ ball.x < area_x(-2.8)         → 锁 2.0s
    └─ 否则                           → 锁 0.5s
    （防守区每帧刷新锁）
```

#### waiting_command（无角色兜底）
无角色分配的玩家 → ReadySlot tagged stop。

### 4.5 四个默认角色详解（`play/default_roles.py`）

#### ChaserRole（name="chaser"）— 主追球手
| 属性 | 值/行为 |
|------|---------|
| target | 球后方 `_CHASER_APPROACH_OFFSET=0.15m`，对齐踢向 |
| kick_target | `select_kick_target`（CENTER 视角：shoot/pass/dribble/sideline/restart） |
| build_subtree | `build_attack_subtree(speed_multiplier=2.0, kick_power=2.5)` |

#### SupporterRole（name="supporter"）— 接应手
| 属性 | 值/行为 |
|------|---------|
| target | `support_target`（追踪 chaser 而非球，动态距离 1-2.8m，10°-45° 侧向成三角，始终 face_ball） |
| wants_to_kick | 非对称滞回：`min_gap = 队友球距 - 自身球距`；进入需 `min_gap > 1.5`（队友比自身距球远 1.5m+），退出当 `min_gap < 0.5`（队友不再明显更远） |
| kick_target | `select_kick_target` |
| build_subtree | `build_attack_subtree(hold_vyaw=0.25, strafe=True, speed=2.0, kick_power=2.5)` |
| 状态日志 | backup(<1m)/closein(>2.8m)/hold/fallback，0.5Hz |

#### DefenderRole（name="defender"）— 扩展防守（默认未注册）
| 属性 | 值/行为 |
|------|---------|
| target | 复用 `SupporterRole.target` |
| build_subtree | `MoveToTarget(hold_vyaw=0.12)` |

#### GoalkeeperRole（name="goalkeeper"）— 守门员
三态状态机 + desperation：

```
        ┌─────────────────────────────────────────┐
        │  ball.x ≥ 0 → 强制 GUARD                │
        └─────────────────────────────────────────┘
                           │
                           ▼
   ┌─────────────────────────────────────┐
   │          GUARD（默认弧形站位）        │
   │   goalkeeper_guard_target            │
   │   goal_line_x = own_goal_x + 1.3     │
   │   R = 1.4m 弧形                      │
   └────┬──────────────────┬──────────────┘
        │ 球预测停止点       │ 球预测穿过门柱内
        │ 在防守区内         │
        ▼                  ▼
   ┌──────────┐       ┌──────────────┐
   │ RUSH_OUT │       │   LATERAL    │
   │ 冲出拦截  │       │ 横向封堵      │
   │ 解围      │       │ 沿守门深度线  │
   └──────────┘       └──────────────┘

   desperation: ball.x < own_goal_x+1.5(-5.5) → 直接扑球
     target = approach_target(ball, kick_theta=0.0(向+x进攻方向), approach_offset=0.2)
```

| 属性 | 值/行为 |
|------|---------|
| approach offset | `_APPROACH_OFFSET=0.18m` |
| wants_to_kick | desperation 总 True；RUSH_OUT 时若外场队友更近（<my_dist+1.0m）则让球 |
| kick_target | desperation→对方球门；**静态球门区**（`ball.x < own_goal_x+0.8(-6.2)` 且球速<0.3）→强制中路；否则 5 候选（center/top门柱/bottom门柱/top边线/bottom边线）按 `lane_clear_score - 0.3*turn` 选最优并锁定整个解围周期 |
| build_subtree | `build_attack_subtree(hold_vyaw=0.12, strafe=True, speed=2.2, kick_power=3.5, lateral_speed=1.0)` |
| 状态去抖 | 进入 confirm=2 帧；退出 RUSH_OUT release=4 帧；LATERAL 保持 min=0.8s |
| target 平滑 | `gk_target_smooth_speed=2.0m/s` 限速 |

### 4.6 PLAY 叶节点（`play/nodes.py`）

| 节点 | 作用 |
|------|------|
| `AssignRoles` | 每帧调 `Playbook.assign_roles` 写 `/team/roles` |
| `IsRole` | 读角色判断（Selector 分支条件） |
| `WaitForBall` | 兜底命令 |
| `MoveToTarget` | 每帧 `target_fn` 计算 + `move_to_target`，ValueError→FAILURE |
| `KickAction` | `kick_target_fn` + `kick_command` |
| `IsKickWanted` | `wants_kick_fn` 判断 |
| `build_attack_subtree` | `Selector(Attack)` → `Sequence(KickBranch)[IsKickWanted, IsInKickRange, KickAction]` \| `MoveToTarget` |

**AttackSubtreeConfig 字段**：`target_fn`, `kick_target_fn`, `wants_kick_fn`, `reason_fn`, `kick_reason_fn`, `hold_vyaw`, `strafe`, `speed_multiplier`, `kick_power`, `lateral_speed`。

**开球专用节点**：`SetPhase`, `InitiateKickoff`, `AssignFixedRoles`, `KickAtAngle`, `MoveToLandingPoint`。

### 4.7 角色分配扩展入口（README 总结）

| 想改什么 | 改哪里 |
|----------|--------|
| 角色分配逻辑 | `Playbook.assign_roles` |
| 踢/传目标 | `ChaserRole.kick_target` |
| 接应站位 | `SupporterRole.target` |
| 守门行为 | `GoalkeeperRole.target` |
| 新增角色 | 继承 `RoleStrategy` + `register_role` |
| 注册新 Playbook | `PLAYBOOKS.register` |
| 追球评分 | `select_chaser` |
| PLAY 子树形状 | `play_subtree.py` |

> `DefaultPlaybook` 注册在 `play/__init__.py` 末尾。

---

## 附录：未深入读取的文件（对策略框架分析非必需）

- `behavior_tree/{blackboard,tree,ready_subtree,safety_subtree,nodes/*}.py`：BT 运行时框架实现细节（通过 `docs/bt_structure.md` 和调用点了解行为）
- `soccer_framework/{game_state,ros_truth,robot,game_controller,ros_adapter,telemetry}.py`：ROS2/GameController 硬件适配
- `main.py`、`agent.toml`、`build.toml`、`docs/developer_protocol.md`：部署/配置/开发协议

如需深入这些文件的实现细节，可单独提出。
