# 3v3 机器人足球仿真 — 策略体系逐环节评价与优化建议

> 本文档基于 `src/` 全部核心文件通读后的专业评价，配套阅读：
>
> - 现状参考：`.omo/docs/strategy-framework-analysis.md`
>
> 优先级标注：🔴 高（直接影响胜负/稳定性） / 🟡 中（提升上限） / 🟢 低（锦上添花）
> 每条建议格式：**问题** → **建议** → **预期收益**，参数建议附具体数值与理由。

---

## 目录

1. [架构与控制循环评价](#1-架构与控制循环评价)
2. [行为树与阶段机评价](#2-行为树与阶段机评价)
3. [独立区策略评价](#3-独立区策略评价)
4. [运动控制评价](#4-运动控制评价)
5. [踢球系统评价](#5-踢球系统评价)
6. [角色分配与 chaser 选举评价](#6-角色分配与-chaser-选举评价)
7. [传球与盘带评价](#7-传球与盘带评价)
8. [接应站位评价](#8-接应站位评价)
9. [守门员系统评价](#9-守门员系统评价)
10. [球轨迹预测评价](#10-球轨迹预测评价)
11. [导航与避障评价](#11-导航与避障评价)
12. [优化优先级总表](#12-优化优先级总表)

---

## 1. 架构与控制循环评价

### ✅ 优点

- **分层单向依赖**设计优秀：`tactics/` 纯函数无 BT/ROS 依赖，可独立单测，是整个项目最稳健的设计决策。
- **team-view 坐标归一化**在适配层完成，下游策略无需关心 team_id，降低了认知负担。
- **SoccerKit 装配层**把 playbook 无关工具（config/motion/obstacles/targeting）集中持有，角色子树通过 kit 访问，解耦清晰。

### ⚠️ 问题与建议

#### 1.1 ✅ DataLayer stale→None 导致策略突变（已实施）

**问题**：`UpdateGameState`/`UpdateRecentBall`/`UpdateRobotPoses` 在数据 stale 时直接置 None。下游 `SafetyGuards` 的"PLAYING 无球→StopAll"会导致**全员骤停**——即使球仅被短暂遮挡（传感器抖动、GC 延迟），整队也会冻结，对手可趁机抢断。

**建议**：

- **参数/策略**：引入"最后已知良好值（LKG, Last Known Good）"保持窗口。球丢失后在 `ball_stale_grace_sec`（建议 0.3s，约 9 帧）内沿用上一帧球位置 + 简单惯性外推，而非立即 None。
- `SafetyGuards` 的"无球→StopAll"应改为"无球且超过 grace 窗口→StopAll"，窗口内降级为"原地面向上次球方向 + 保持阵型"。
- **预期收益**：消除传感器抖动导致的整队冻结，显著减少非受迫性丢球。

**实施状态（2026-07）**：

- ✅ **球 LKG**：`src/soccer_framework/ball_lkg.py:BallLkgBuffer`，匀速外推，`ball_fresh_sec=0.3` + `ball_stale_grace_sec=0.3`（合计 0.6s = 18 帧容忍）。`UpdateRecentBall` 经 `kit.ball_lkg.update()` 接入；grace 内返回 confidence=0.5 的外推球，过期才置 None 触发 `NoPlayingBallStop`。SafetyGuards 无需改造即天然兼容。
- ✅ **位姿 LKG**：`src/soccer_framework/pose_lkg.py:PoseLkgBuffer`，对 teammates/opponents 各一实例（`kit.pose_lkg_teammates`/`kit.pose_lkg_opponents`），匀速外推含 theta（unwrap 回归），`robot_pose_fresh_sec=0.3` + `robot_pose_stale_grace_sec=0.2`（合计 0.5s），位移 clamp 0.3m。`UpdateRobotPoses(kit)` 经两个 buffer 接入。
- ⚠️ **game_state 不做 LKG（有意保留）**：`_GAME_STATE_STALE_SEC=2.0` 仍直接置 None 触发 `NoGameStop`。理由：GC 状态离散，沿用旧 PLAYING 而裁判实际已判 STOPPED 会导致技术犯规。保守停车是安全选择。
- **架构合规**：AR-02（黑板唯一数据源）保留——LKG 仍经 `context.ball`/`context.teammates`/`context.opponents` 流转；AR-08（SafetyGuards 不绕过）保留——位姿 LKG 不触及 SafetyGuards。

#### 1.2 🟡 控制循环异常兜底过于粗暴

**问题**：`SoccerTeamRuntime` 异常→`robot_manager.stop_all`。单帧某个 target 计算抛异常会导致全员停车。

**建议**：

- **策略**：区分异常级别——`ValueError`（target 计算失败，如球在场地外）→ 该玩家降级为 `waiting_command`（ReadySlot tagged stop），其他玩家继续；只有 `RobotCommunicationError` 等硬件级异常才 stop_all。
- `MoveToTarget` 已有 `ValueError→FAILURE` 的单玩家降级，但控制循环层面的 stop_all 会覆盖它，需要确认异常传播路径。
- **预期收益**：单点异常不再波及全队。

#### 1.3 🟢 缺乏性能指标闭环

**问题**：`_log_performance` 每 2s 记录 zone/role/dist 等状态，但只是日志，没有反馈到策略调整。

**建议**：

- **策略**：将性能日志接入运行时统计（如 chaser 切换频率、传球成功率、被抢断次数），用于赛后分析和自适应调参。当前是开环系统，无法在线学习。
- **预期收益**：为后续策略迭代提供数据基础。

---

## 2. 行为树与阶段机评价

### ✅ 优点

- `MatchControl` 的 Selector + 兜底 StopAll 结构清晰，安全兜底完备。
- `SafetyOverrides` 硬件层（penalty/倒地/行走模式）与策略层解耦，独立 `Parallel(SuccessOnAll)` 确保每帧都检查。

### ⚠️ 问题与建议

#### 2.1 ✅ SafetyGuards "PLAYING 无球→StopAll" 过于保守（已实施）

 （与 1.1 同一问题的行为树层面表现，由 `BallLkgBuffer` 解决——grace 内 `context.ball` 非 None，`IsBallKnown` 通过，StopAll 不触发）

**问题**：5 个全局守卫中"PLAYING 无球→StopAll"是最容易误触发的。仿真环境下球被机器人遮挡是常态。

**建议**：

- **策略**：拆分为"球完全丢失（超 grace）→StopAll" + "球暂时遮挡（grace 内）→维持当前角色 + 继续移动到上次 target"。后者由 Playbook 在球丢失时复用上一帧 `RoleAssignment`。
- **预期收益**：避免整队在正常对抗中频繁冻结。

#### 2.2 🟡 开球阶段机参数全硬编码

**问题**：`_KICKOFF_ANGLE=0.785rad(45°)`、`_KICKOFF_LANDING_DIST=2.5m`、Phase 2 触发条件"球移动>0.15m + 0.5s 延迟"全部硬编码在 `play_subtree.py`。

**建议**：

- **参数**：将开球参数移入 `SoccerStrategyTuning`（如 `kickoff_angle_rad`, `kickoff_landing_dist_m`, `kickoff_ball_move_threshold_m`, `kickoff_role_lock_delay_sec`），便于不重新编译调参。
- **策略**：开球踢向应根据对手站位动态选择——若对手防线偏向一侧，开球踢向另一侧空当。当前固定 45° 可能被对手预判。
- Phase 2 的 `AssignFixedRoles`（center→supporter, side→chaser）是**开球结果无关**的固定映射，若开球被对手截走，side→chaser 可能不是最优。建议 Phase 2 仍用动态 `assign_roles`，仅锁定开球者短暂优势期。
- **预期收益**：开球策略更灵活，不易被对手针对。

#### 2.3 🟢 ReadyPhase 与 PlayingPhase 之间缺乏过渡

**问题**：READY→PLAYING 切换瞬间，机器人从 ready_target 直接切换到动态角色 target，可能产生急转弯。

**建议**：

- **策略**：PLAYING 前几帧（如 0.3s）对 target 做插值平滑（从 ready_pose 到 role_target 线性过渡），避免突变导致失稳。
- **预期收益**：减少阶段切换时的姿态抖动。

---

## 3. 独立区策略评价

### ✅ 优点

- `predicates.py` 纯函数风格，区域判定逻辑集中，易于测试。
- 区域边界由 `SoccerStrategyTuning` 参数驱动，可配置。

### ⚠️ 问题与建议

#### 3.1 ✅ area_x 一参多用，耦合严重（已实施）

**问题**：`goalkeeper_challenge_area_x_ratio=0.20` 推出的 `area_x=-2.8m` 同时用于：

1. KEEPER 是否参与争球（`ball_in_own_defensive_area`）
2. chaser 锁定时间阶梯（`ball.x < area_x` → 锁 2.0s）
3. SIDE 挑战判定（`ball_is_in_midfield_or_own_half` 用 `field_length*0.20=+2.8`，数值巧合相同但语义不同）

这三个决策的关注点完全不同（守门员出击范围 vs chaser 稳定性 vs SIDE 参与度），共用一个比例参数会导致调一个动三个。
（注：第 3 点实际情况比描述更严重——`predicates.ball_is_in_midfield_or_own_half` 原本用的是**硬编码字面量 `0.20`**，根本不读任何 config 字段；且方向相反（+2.8 vs -2.8），数值相同纯属巧合。）

**建议**：

- **参数**：拆分为独立参数：
  - `goalkeeper_challenge_area_x_ratio`（KEEPER 出击范围，保持 0.20）
  - `chaser_lock_defensive_x_threshold_m`（chaser 锁定阶梯边界，可独立调）
  - `midfield_boundary_x_ratio`（中场/进攻分界，当前隐式 0.20，建议显式化）
- **预期收益**：各区域策略可独立调优，不互相干扰。

**实施状态（2026-07）**：

- ✅ **参数拆分**：`src/soccer_framework/config.py:SoccerStrategyTuning` 新增两个独立 ratio 字段（均默认 0.20，行为零变化）：
  - `chaser_lock_defensive_x_ratio=0.20` — chaser 锁定阶梯防守档边界（own-side，-2.8m）。
  - `midfield_boundary_x_ratio=0.20` — SIDE 中场/进攻挑战分界（attack-side，+2.8m）。

  > 单位选择：经评估采用 `_ratio`（比例）而非 issue 建议的 `_m`（米），理由：与既有 `goalkeeper_challenge_area_x_ratio` 风格一致，且随场地尺寸自动缩放（场地尺寸可变）。默认 0.20 → -2.8m / +2.8m 完全保留原行为。
  >
- ✅ **SIDE 挑战解耦**：`src/tactics/targeting/predicates.py:ball_is_in_midfield_or_own_half` 由硬编码 `field_length*0.20` 改读 `config.strategy.midfield_boundary_x_ratio`，消除魔法数字。
- ✅ **chaser 锁定解耦 + 消除重复**：`src/play/playbook.py:DefaultPlaybook` 新增 `_chaser_lock_defensive_x()` / `_chaser_lock_duration(ball)` 两个 helper，集中三档阶梯逻辑（3.0s/2.0s/0.5s），读取新参数（不再借用 GK 参数）。`select_chaser` 中原本**重复两遍**的阶梯计算（旧 line 242-249 / 274-281）+ 防守区锁定刷新判断全部替换为调用 helper，DRY 违规一并消除。
- ✅ **文档同步**：`AGENTS.md` 谓词表与 Key Parameters 段已标注三个 ratio 的独立语义。
- **架构合规**：AR-04（`play/` 不导入 py_trees）保留——仅通过 `self.kit.config` 读字段；改动是"加字段 + 改读源"，无新依赖；AR-02/08 无影响。
- **后续**：issue 3.2（中场/进攻边界改非对称、调小到 0.10）现可仅改 `midfield_boundary_x_ratio` 默认值一行落地，不再牵动 KEEPER/chaser lock。`ready_stance.py:59` 的 `field_length*0.20`（对手开球 READY 站位）属独立语义，不在本次范围。

#### 3.2 🟡 中场/进攻边界对称划分不合理

**问题**：`ball_is_in_midfield_or_own_half` 用 `ball.x < field_length*0.20=+2.8`，意味着球在 `ball.x < +2.8`（从己方球门 -7 到对方半场 2.8m 处，共 9.8m，占全场 70%）时 SIDE **总是**挑战。即使在对方半场前段（`ball.x=+2.5`，深入对方半场 36%）SIDE 仍然无条件前压，可能导致 SIDE 过度前压、后方空虚。注：`+2.8` 边界相对进攻半场（0~+7）深度占 40%，相对全场（14m）占 20%。

**建议**：

- **参数**：中场/进攻分界应改为非对称——我方半场总是挑战（`ball.x < 0`），对方半场前段条件挑战（`0 ≤ ball.x < +3.0`），深入进攻区（`ball.x ≥ +3.0`）严格条件挑战。建议 `midfield_boundary` 改为 `field_length*0.10=+1.4` 或直接 `0`（中线）。
- **策略**：SIDE 是否挑战还应考虑双方人数对比——若我方在进攻区人数劣势（如有人倒地），SIDE 应回防而非前压。
- **预期收益**：SIDE 站位更合理，减少身后空当。

#### 3.3 🟡 边线恢复 advance 方向固定，不考虑球速

**问题**：`sideline_recovery_target` 的 `target_x=ball.x+0.75`（向对方半场推进），但如果球在我方半场边线且正在向己方滚动，向 +x 推进可能让追球者跑过头。

**建议**：

- **策略**：advance 方向应根据球速方向调整——球向 +x 滚则 `+0.75`，球向 -x 滚则 `+0.3`（小幅向前即可，优先控制球）。或直接 advance 到球的预测停止点。
- **预期收益**：边线恢复更精准，减少控球失误。

---

## 4. 运动控制评价

### ✅ 优点

- 反应式控制器设计简洁，每帧计算量小，适合 30Hz 实时控制。
- 到达检查 + hold_vyaw 机制（到达后保持朝向）对踢球前的对齐很重要。
- 三层流程（绕行→行走→偏航避让）职责分明。

### ⚠️ 问题与建议

#### 4.1 🔴 反应式避障无全局规划，密集场景易卡死

**问题**：`MotionController` 完全反应式——只看当前障碍算侧向 via 点，没有局部窗口规划。在禁区混战、多人围抢（3v3 经常出现）时，机器人可能在两个绕行方向间振荡，或被多个障碍"包围"而无法找到出路。

**建议**：

- **策略**：在路径绕行层（`_avoidance_target`）之前加入简单的局部规划：
  - **方案 A（推荐，低成本）**：对目标方向做扇形采样（如每 15° 一个候选，±90° 范围共 13 条射线），选障碍代价最低的方向作为临时 target。比当前的"第一个阻挡障碍"策略更鲁棒。
  - **方案 B（中成本）**：引入动态窗口法（DWA），在 `[vx, vyaw]` 空间采样，选最优轨迹。
- 检测"卡死"状态（连续 N 帧位移<阈值）→ 触发后退/侧移脱困行为。
- **预期收益**：解决密集场景下的振荡和卡死，这是 3v3 比赛中最常见的失分场景。

#### 4.2 🟡 队友层面双层避障冗余

> ⚠️ **勘误（v2）**：原版标注为 🔴"两层避障方向矛盾"，经核查 `motion.py` 代码后**降级为 🟡**。PLAY 阶段 `avoid_opponents=False`（line 101-102），偏航避让（`_apply_yaw_avoidance`）**只作用于队友、不含对手**（line 460）。因此原论述"路径绕行处理对手 vs 偏航避让也处理对手产生矛盾方向"**不成立**——对手仅由路径绕行层处理，两层不会对同一对手产生反向决策。

**问题（实际存在的，降级为 🟡）**：在**队友**层面，路径绕行（`_avoidance_target`）已为阻挡队友生成侧向 via 点（改 target），偏航避让又对同一队友叠加偏航 bias。两者方向通常一致（都远离该队友），不会产生"之字形"或原地打转，但属于**冗余计算**——同一队友障碍被避让了两遍，bias 叠加可能导致略微过冲（转弯过度）。

**建议**：

- **策略**：偏航避让的邻居列表中，可排除"已被路径绕行处理过的队友"（即已在 via 点生成中选定的那个障碍），避免对同一队友重复避让。
- 或保持现状（冗余但方向一致、危害有限），仅在实测观察到队友间运动过冲时再优化。
- **预期收益**：减少队友避让冗余计算，运动更平稳（收益有限，故降级为 🟡）。

#### 4.3 🟡 近距离减速不够平滑（_LINEAR_GAIN 与 max 配合）

**问题**：`_LINEAR_GAIN=2.0` + `max_linear_speed=0.8`，意味着 `distance > 0.4m` 时就达到满速（`2.0*0.4=0.8`）。`distance` 在 `0.15~0.4m` 区间减速很快，加上 `cos(err)` 衰减，可能导致接近目标时频繁"冲刺-急停-冲刺"的启停现象。

**建议**：

- **参数**：考虑引入速度曲线平滑——到达阈值（0.15m）外用一个更平滑的减速段。例如：
  - `0.15~0.5m`：`vx = max_linear_speed * (distance-0.15)/(0.5-0.15)`（线性减速到 0）
  - `>0.5m`：满速
  - 这比当前 `gain*distance` 的硬比例更平滑。
- 或降低 `_LINEAR_GAIN` 到 1.5（满速距离推迟到 ~0.53m），同时把 `_LINEAR_SPEED_FLOOR` 降到 0.2，减少低速地板抖动。
- **预期收益**：接近目标时更平稳，减少姿态失稳。

#### 4.4 🟡 角速度控制死区与 floor 配合可能抖动

**问题**：`_ANGULAR_DEAD_ZONE=0.15`（死区）和 `_ANGULAR_SPEED_FLOOR=0.25`（floor）配合：误差在 `0.15~0.5`（TURN_THRESHOLD）区间时，角速度 floor=0.25 但误差可能只有 0.16，导致 `omega=0.25` 过冲到另一侧，下一帧又回来，形成小幅度振荡。

**建议**：

- **参数**：死区内纯比例（当前已实现），死区外到 TURN_THRESHOLD 之间应线性过渡到 floor 而非直接跳到 floor。例如：
  - `err ∈ [0.15, 0.3]`：`omega = 2*err`（纯比例，0.3~0.6）
  - `err ∈ [0.3, 0.5]`：`omega = max(2*err, floor)`
  - `err > 0.5`：纯转弯
- 或将 `_ANGULAR_SPEED_FLOOR` 降到 0.15（与死区一致），避免死区边界突变。
- **预期收益**：消除接近目标朝向时的小幅振荡。

#### 4.5 🟢 strafe 模式 lateral 限速偏低

**问题**：`max_lateral_speed=0.6`，而 SupporterRole 的 strafe 模式 lateral_speed 未显式传入（用 max_lateral_speed），接应横移时速度上限 0.6 可能偏慢。

**建议**：

- **参数**：若硬件允许，`max_lateral_speed` 可提到 0.7~0.8。但需确认仿真机器人侧移能力。
- **预期收益**：接应横移更快，跟防更紧密。

---

## 5. 踢球系统评价

### ✅ 优点

- `KickHysteresis` 滞回模型（enter=2.0/exit=2.5）设计合理，避免在边界附近反复触发/退出踢球。
- 不同角色使用不同 kick_power（Chaser=2.5, Supporter=2.5, Goalkeeper=3.5），层次分明。
- `soccer_kick_min_active_sec=0.7s` 防止踢球判定过短。

### ⚠️ 问题与建议

#### 5.1 🟡 kick_power 配置不一致需确认

**问题**：`SoccerStrategyTuning.soccer_kick_power=1.5`（全局默认），但 `ChaserRole.build_subtree` 硬编码 `kick_power=2.5`，`GoalkeeperRole` 硬编码 `kick_power=3.5`。全局默认值 1.5 实际未被任何默认角色使用，可能造成混淆——调参者改了 `soccer_kick_power` 却发现没有效果。

**建议**：

- **参数/策略**：角色 kick_power 应从 config 读取（如 `chaser_kick_power=2.5`, `supporter_kick_power=2.5`），而非硬编码在 `default_roles.py`。确保所有踢球力度集中可调。
- 确认 `soccer_kick_power=1.5` 是否为遗留值或被其他路径使用（如 `waiting_command`），若无用则移除。
- ⚠️ 注意 `kickoff_kick_power=1.0` **并非无用值**——它被 `play_subtree.py` Phase 1 的 `KickAtAngle` 节点使用（开球专用）。本条仅指 `soccer_kick_power=1.5`（PLAY 常规默认）未被任何默认角色使用。
- **预期收益**：调参入口统一，避免"改了没效果"的陷阱。

#### 5.2 🟡 踢球力度不考虑距离与障碍

**问题**：ChaserRole 的 `kick_power=2.5` 是固定值，无论射门还是传球都用同样力度。射门到对方球门（距离可能 5-10m）和短传（2-3m）用相同力度显然不合理——大力短传会让队友接不住。

**建议**：

- **策略**：kick_power 应根据 kick_target 动态计算：
  - 射门：`power = base + distance * factor`（远射加力）
  - 短传（`decision == "pass"`）：`power = min(base, pass_power)`（轻传，便于接球）
  - 解围：`power = max_power`（全力清场）
- `select_kick_target` 返回的 decision 可传入 kick_power 计算函数。
- **预期收益**：传球更精准，射门更有力，解围更彻底。

#### 5.3 🟢 踢球方向不考虑守门员站位（射门时）

**问题**：`select_kick_target` 的射门决策用 `shot_lane_is_clear` 判断通道是否畅通，但不考虑对方守门员的具体站位——即使通道畅通，守门员可能已封堵该角度。

**建议**：

- **策略**：射门目标选择应加入对方守门员位置作为惩罚项——选择守门员距离最远的门柱方向射门。需要从 `PlayContext` 获取对手守门员位置（若 team-view 提供对手个体位置）。
- **预期收益**：提高射门得分率。

---

## 6. 角色分配与 chaser 选举评价

### ✅ 优点

- `select_chaser` 的 4 步算法（资格过滤→成本评分→平局仲裁→动态锁定）逻辑完备。
- chaser 锁定机制（防乒乓）是关键设计，避免两个队友在球旁反复切换谁去追。
- `ball_claim_score` 的槽位偏好（CENTER 略优先于 SIDE）合理，避免侧翼无谓争抢中路球。

### ⚠️ 问题与建议

#### 6.1 🔴 chaser 选举完全不考虑对手位置

**问题**：`ball_claim_score` 只算"队友到球的距离 + 槽位偏好"，完全不考虑对手。在对抗中，离球最近的队友可能正被对手卡位，实际触球时间反而更晚。这会导致选出的 chaser 被"截胡"。

**建议**：

- **策略**：在 `ball_claim_score` 中加入对手干扰项：
  ```
  score = distance - slot_bonus + opponent_interference
  opponent_interference = max(0, (clearance - dist_to_nearest_opponent) * weight)
  ```

  其中 `clearance` 是对手到"队友-球"连线的距离阈值，对手越近则干扰越大。- 若对手已在球-队友之间（截球路径上），应大幅加分（降低该队友被选概率）。
- **预期收益**：选出的 chacer 更可能实际抢到球，减少被截。

#### 6.2 🟡 chaser 锁定时间阶梯硬编码且不随球速调整

**问题**：锁定时间（3.0s/2.0s/0.5s）按 `ball.x` 三段硬编码。球速很快时（如对手大力解围），0.5s 锁定期内球可能已飞越半个场地，旧 chaser 锁定毫无意义；球速很慢时，频繁切换的 0.5s 又太短。

**建议**：

- **参数**：锁定时间应结合球到 chaser 的预期触球时间：
  ```
  lock_time = base_lock + estimated_touch_time
  estimated_touch_time = distance / (max_linear_speed * speed_multiplier)
  ```

  快球→锁定长（等 chaser 追到），慢球→锁定短（快速重评估）。
- 或将三段阶梯改为连续函数：`lock_time = 0.5 + 2.5 * sigmoid(-(ball.x - own_goal_x) / 3.0)`。
- **预期收益**：锁定时间更贴合实际，减少无效锁定和延迟切换。

#### 6.3 🟡 ball_claim_score 的槽位偏好是硬编码偏移

**问题**：`CENTER: distance-0.20`, `SIDE: distance-0.10`, `KEEPER: distance-0.75`（危险区）/`distance+field_length`（非危险区）。这些偏移是经验值，不可调且不随场景变化。

**建议**：

- **参数**：将槽位偏好移入 `SoccerStrategyTuning`：
  - `center_claim_bonus_m=0.20`
  - `side_claim_bonus_m=0.10`
  - `keeper_danger_bonus_m=0.75`
- **策略**：偏好应随区域动态调整——进攻区 CENTER 偏好应更大（主力射门），防守区 KEEPER 偏好可降低（避免守门员贸然出击）。
- **预期收益**：槽位偏好可调且场景适应。

#### 6.4 🟢 缺乏角色疲劳/体能概念

**问题**：3v3 比赛中 chaser 持续高强度追球，但没有"体能"概念——某个玩家可能一直是 chaser 而过度消耗（仿真中体现为持续高速导致姿态失稳概率增加）。

**建议**：

- **策略**：引入简单的"角色轮换"——chaser 持续时间超过阈值（如 15s）后，在平局时优先切换到另一玩家。当前 `_last_chaser_id` 已有记录，可扩展。
- **预期收益**：分散负荷，降低单机失稳风险。

---

## 7. 传球与盘带评价

### ✅ 优点

- `best_pass_target` 的多因子评分（lane_clear 0.55 + forward 0.30 + center 0.15）结构合理。
- `pass_min_forward_m=0.35` 过滤向后传球，避免倒脚。
- `shot_lane_is_clear` 的滞回（未射门 0.45/已射门 0.25）防止射门决策抖动。

### ⚠️ 问题与建议

#### 7.1 🟡 pass_min_score=0.60 可能过于保守

**问题**：传球得分需 ≥0.60 才执行，否则 fallback 到盘带。但 `lane_clear_score` 在有对手围抢时很容易低于 0.55（权重 0.55*0.55=0.30），加上 forward/center 后总分可能卡在 0.5~0.6 之间，导致**很少传球、过度盘带**。3v3 场地小，盘带容易被围抢。

**建议**：

- **参数**：`pass_min_score` 降到 0.50~0.55，或改为动态阈值：
  - 周围对手多（<2m 内有 ≥2 对手）→ 阈值降到 0.45（鼓励快速出球）
  - 空旷 → 阈值保持 0.60（允许盘带推进）
- **预期收益**：减少被围抢丢球，传球更积极。

#### 7.2 🟡 传球评分不考虑接球者后续射门机会

**问题**：`best_pass_target` 只评估传球通道和前向增益，不评估**接球者拿球后能否射门**。传给一个被对手贴防的队友，即使传球成功也会立即丢球。

**建议**：

- **策略**：在评分中加入接球者的"射门潜力"：
  ```
  score += shoot_potential_weight * teammate_shoot_lane_clear
  ```

  其中 `teammate_shoot_lane_clear` 是队友位置到对方球门的通道净空。
- 排除被对手贴防（<0.5m）的接球候选。
- **预期收益**：传球选择更有进攻价值，减少"传球即丢球"。

#### 7.3 🟡 dribble_target 向中线拉扯可能撞向对手

**问题**：`dribble_target = (ball.x+1.5, ball.y*0.65)`，向中线拉扯。但中线区域可能正是对手密集区，盘带过去等于自投罗网。

**建议**：

- **策略**：盘带方向应避开最近对手——若最近对手在盘带方向上（夹角<45° 且距离<2m），改为向边线一侧盘带或直接传球。
- `dribble_center_pull` 可动态：对手在中路密集时降到 0.3（少向中线），边路空旷时保持 0.65。
- **预期收益**：盘带更安全，减少撞人丢球。

#### 7.4 🟢 SIDE 视角不射门（select_clear_or_pass_target）

**问题**：`select_clear_or_pass_target`（SIDE 专用）只做 sideline/pass/clear，不射门。即使 SIDE 在对方禁区拿球也不射门，可能浪费得分机会。

**建议**：

- **策略**：SIDE 在对方禁区附近（`ball.x > half_length - 3.0`）且射门通道畅通时，应允许射门。可复用 `shot_lane_is_clear`。
- **预期收益**：增加得分点。

---

## 8. 接应站位评价

### ✅ 优点

- `support_target` 追踪 chaser（通过 `RoleAssignment.chaser_id`，下传真实已分配 chaser），而非最近非 GK 队友扫描（旧方案在 keeper 当选 chaser 时两 supporter 互为参照）。
- 动态距离（1-2.2m）+ 三角角（10-45°）+ side 按 player_id 奇偶交替，保证两个接应手不重叠。
- `_spaced_support_target` 两趟推开（队友 + 对手）机制避免接应手挤在一起或站在对手身旁。
- **稳定性锚定**：带内 + 角度容差内返回当前位姿（hold），消除切向轨道抖动；叠加速率限制平滑吸收帧间跳变。
- **危险区 fallback**：守门员为真实 chaser 时（球在本方危险区），外场分 cover（球门线前护门）和 outlet（前场接解围），互不参照。
- **远距重接应**：sc_dist > 4.0m 时弃三角侧偏为直线逼近，消除横向绕行。
- **strafe 朝向优先**：heading 误差 > 1.2rad 时缩平移至纯旋转，消除 strafe 背对球。

### ⚠️ 问题与建议

#### 8.0 ✅ 接应轨道抖动（稳定性根因，已实施 2026-07）

**问题（仿真观察）**：`_chaser_relative_target` 每帧用"当前 sc_dist + 公式角度"重算目标。带内时 `desired_dist = sc_dist`，目标落在"同半径、公式角度"上；球/chaser 移动 + 球位置噪声使公式角度持续漂移 → supporter 永远沿圆弧切向追一个滑移目标，触发不了 `arrive`（0.15m）→ 持续绕 chaser 侧滑（strafe 模式下面向球横挪），表现为不合理的来回移动。这与原 docstring"带内应基本站定"完全不符。

**实施**：

- **死区锚定**（`tactics/targeting/support.py:_chaser_relative_target`）：算完理想目标后，若 `sc_dist ∈ [support_min_distance_m, support_max_distance_m]` 且 `|normalize_angle(sc_angle − ideal_angle)| ≤ support_angle_hold_deadzone(0.35rad≈20°)` → 直接返回 `own_pose`（hold，仅由 motion 的 active-hold 面向球）。跨带/跨角才全量重定位。
- **速率限制平滑**（`play/default_roles.py:SupporterRole._smooth_target`）：仿 GK `gk_target_smooth_speed`，smoothed 目标以 `support_target_smooth_speed(2.5m/s)` 速率追 raw；`|Δ|>1.5m`（chaser/角色切换）snap。
- 新增配置：`support_angle_hold_deadzone`、`support_target_smooth_speed`。

#### 8.1 ✅ 接应位置不规避对手（已实施 2026-07）

**原问题**：`_spaced_support_target` 只推开队友（`support_min_spacing_m=0.9`），不推离对手。接应手可能站在对手身旁，接到球立即被抢。

**实施**（`tactics/targeting/support.py`）：

- `_spaced_support_target` 改为两趟顺序推开：① 最近队友→`support_min_spacing_m`；② **最近对手→`support_opponent_avoid_radius_m(0.6m)`**。
- 抽出 `_push_out_from(field, ball, target, obstacle, radius, player_id)` 复用，末尾统一 `clamp_inside_field` + `face_ball_theta`。
- 新增配置：`support_opponent_avoid_radius_m`。读 `context.opponents`（与 motion.py 一致）。
- **预期收益**：接应手接到球后有处理空间，减少接球即丢。

#### 8.2 🟡 三角角不随对手压迫调整

**问题**：`base_deg = 10 + t*35`（t 按球在场内位置），三角角只随球位置变化，不考虑对手压迫。对手高位逼抢时，接应手应拉开更远（更大角度），拉开传球空间。

**建议**：

- **策略**：三角角应随"chaser 周围对手数量"调整：
  - chaser 被 ≥2 对手围抢 → 角度增大到 35-45°（拉边接应）
  - chaser 空旷 → 角度减小到 10-20°（靠近支援）
- **预期收益**：接应站位更贴合比赛节奏。

#### 8.3 ✅ 接应距离上限 2.8m 对 3v3 偏大（已实施 2026-07）

**原问题**：3v3 场地 14×9m，每队仅 3 人。接应手距 chaser 2.8m 意味着两人间距占场地宽度的 31%，传球距离长→传球精度下降→易被拦截。

**实施**（`tactics/targeting/support.py` + `soccer_framework/config.py`）：

- 距离界 `1.0 / 2.8` 硬编码改为配置 `support_min_distance_m(1.0)` / `support_max_distance_m(2.2)`。
- 上限 2.8 → **2.2**（约占场地宽 24%，兼顾拉开与传球精度）。
- 原死字段 `support_depth_m` / `support_lateral_m` 标记 `[deprecated, unused]`（保留以免破坏外部 playbook）。
- **预期收益**：传球更精准，接应更紧密。

#### 8.4 🟢 chaser 相对方向 blend 过渡区间窄

**问题**：`_chaser_relative_target` 的 `blend=(bc_len-0.3)/0.5`，在 chaser 球距 `0.3~0.8m` 之间过渡。这个 0.5m 的过渡带较窄，可能不够平滑。

**建议**：

- **参数**：过渡区间可扩到 `0.3~1.0m`（blend 分母 0.7），让方向混合更平滑。
- **预期收益**：接应方向切换更连续。

---

## 9. 守门员系统评价

### ✅ 优点

- 三态机（GUARD/RUSH_OUT/LATERAL）+ desperation 设计完整，覆盖了守门员的主要场景。
- `BallPredictor` 驱动 RUSH_OUT/LATERAL，前瞻性好——不是等球到了才动，而是预测球会进才提前封堵。
- 状态去抖（confirm=2/release=4）+ LATERAL 保持（0.8s）避免状态抖动。
- 解围目标 5 候选 + 锁定整个周期，避免解围途中反复改向。

### ⚠️ 问题与建议

#### 9.1 ✅ desperation 移除——改为 max-dist 过滤 + 队友拦截检查（已实施 2026-07）

**原问题**：`gk_desperation_clear_margin_m=1.5` → `ball.x < own_goal_x+1.5 = -5.5` 触发 desperation（直接扑球）。守门员冲出球门区扑球，若扑空则**完全空门**。

**实施**（`default_roles.py` + `config.py`）：

- **移除 desperation**：删掉 `gk_desperation_clear_margin_m` 配置和 `target()`/`wants_to_kick()`/`kick_target()` 中所有 desperation 逻辑。取代方案：
  - **`gk_rush_out_max_dist_m=1.5`**：仅在预测停止点距球门线 ≤ 1.5m 时允许 RUSH_OUT（`rest_x < own_goal_x + 1.5`）。防止守门员为远处球贸然出击。
  - **`_gk_teammate_is_closer(target_x, target_y, kit, context, extra_margin=0.2)`**：进入 RUSH_OUT 前检查是否有外场队友更靠近预测停止点（差值 > 0.2m），若有则 defer 到 GUARD。
  - 两项组合效果：只有球**真正危险**（近球门线）**且外场来不及处理**时守门员才出击。
- **预期收益**：消除空门风险，出击更精准。

#### 9.2 🟡 gk_target_smooth_speed 限速可能过慢

**问题**：`gk_target_smooth_speed=2.0m/s` 限制守门员目标轨迹平滑速度。但 `max_linear_speed=0.8`，守门员实际最大速度也就 0.8*2.2(rush_multiplier)=1.76m/s。平滑限速 2.0 > 实际最大速度，这个限速**形同虚设**。

**建议**：

- **参数**：确认 `gk_target_smooth_speed` 是限目标点的变化速率还是限机器人速度。若是限目标点变化，2.0m/s 在球速快时会让目标点跟不上球——LATERAL 横移封堵时球可能已入门。建议提到 3.0~4.0 或改为不限制（直接跟踪预测点）。
- LATERAL 的 `lateral_speed=1.0` 应配合球的横移速度动态调整——球横向速度快时 lateral_speed 应提到 1.2~1.5。
- **预期收益**：守门员反应更快，封堵更及时。

#### 9.3 🟡 RUSH_OUT 让球逻辑（已部分实施 2026-07）

**原问题**：RUSH_OUT 时若"外场队友更近（<my_dist+1.0m）"则让球（`wants_to_kick=False`）。但队友更近不代表队友能解围——队友可能在球的另一侧、被对手卡位、或背对球门。

**部分实施**：

- **分阶段出击**：新增 `_rush_approach_stage` 标志（`"far"`/`"near"`）。Stage 1 (far) 时 `wants_to_kick()` 返回 `False`，守门员直线跑向预测停止点；Stage 2 (near) 才评估踢球。确保守门员到达球附近之前不触发踢球（即使进入 RUSH_OUT 状态）。
- **进入前拦截检查**：新增 `_gk_teammate_is_closer()` 在 RUSH_OUT 入口前检查外场队友距离（见 9.1），避免守门员为队友可处理的球出击。
- **未实施部分**：让球条件仍以距离为主（`my_dist + 1.0`），未考虑"队友是否在解围路径上"或"球距球门线"。队友路径评估复杂度较高，留作后续优化。
- **预期收益**：分阶段机制消除接近过程中的误踢；入口拦截减少不必要的出击。

#### 9.4 🟡 GUARD/RUSH_OUT 条件（已部分实施 2026-07）

**原问题**：`ball.x≥0` 强制 GUARD，意味着球只要在我方半场守门员就可能 RUSH_OUT。但球在 `x=-1` 缓慢滚动时，守门员没必要冲出——外场队友完全能处理。

**部分实施**：

- **max-dist 滤网**（`gk_rush_out_max_dist_m=1.5`）：RUSH_OUT 额外要求预测停止点在球门线附近（`rest_x < own_goal_x + 1.5`），过滤掉远处球的出击（见 9.1）。
- **队友拦截检查**（`_gk_teammate_is_closer`）：RUSH_OUT 入口前检查外场队友距离，有队友更近则留在 GUARD。
- **未改变**：基本触发条件（`rest_x < area_x - rush_margin`）保持不变，防守区 `area_x=-2.8` 仍比球门区（-6.0）大。max-dist 滤网配合队友拦截已大幅减少不必要出击，但理论上的"球门区边界触发"调整留作后续。
- **预期收益**：max-dist+队友检查双重过滤显著减少贸然出击。

#### 9.5 ✅ 弧形站位动态缩放（已实施 2026-07）

**原问题**：`goalkeeper_guard_arc_radius=1.4`、`goalkeeper_guard_depth_m=1.3` 决定弧形站位。smoothstep 在 `(0.7, 1.3)` 之间 lerp，这些经验值固定不变。

**实施**（`ready_stance.py:goalkeeper_guard_target`）：

- **动态缩放**：基于球到球门线的距离对弧形半径和站位深度做动态线性缩放：
  - `scale = 1.0 - 0.4 * norm`，`norm = ball_dist / (field_length * 0.5)`，距离球门 0m → 1.0（全宽），7m → 0.6（收紧）
  - `R = base_R * scale`；`depth = base_depth * (1.0 - 0.3 * norm)`（1.3m → 0.91m）
- **效果**：球近时守门员宽幅封堵角度；球远时回缩节省体力、防止被远射吊门。
- **预期收益**：站位随场景自适应。

---

## 10. 球轨迹预测评价

### ✅ 优点

- 指数摩擦模型 `v(t)=v0*exp(-mu*t)` 物理合理，比纯线性外推准确。
- `predict_goal_crossing` 解对数方程求过门线时间，数学严谨。
- 摩擦系数在线估计（`0.9*old+0.1*decel`），适应不同场地。

### ⚠️ 问题与建议

#### 10.1 ✅ history_size=10→20 + 3 帧差分回归（已实施 2026-07）

**原问题**：`ball_prediction_history_size=10`，@30Hz 只有 0.33s 历史。最小二乘线性回归在 10 个样本上估计速度，信噪比低——球位置测量有噪声时，速度估计可能波动很大，导致 `predict_goal_crossing` 误判。

**实施**（`ball_prediction.py`）：

- **`history_size` 10→20**（0.67s 历史），增加样本量平稳估计。
- **`_regress_velocity` 替换**：旧的最小二乘线性回归（匀速假设，与摩擦模型矛盾）改为**最近 3 帧差分 + 摩擦补偿**：
  ``` python
  v(t1) = v_avg * mu*dt / (exp(mu*dt) - 1)
  ```
  3 帧平均速度推算瞬时速度，与指数摩擦模型 `v(t)=v0*exp(-mu*t)` 一致。
- **`_MAX_REST_DISTANCE` 20→16m**（`field_length+2`），防止异常预测值流入下游。
- **预期收益**：速度估计更稳定且模型一致，预测精度提升。

#### 10.2 ✅ 摩擦在线更新太慢（已实施 2026-07）

**原问题**：`mu = 0.9*old + 0.1*decel`，更新率 0.1 意味着需要约 10 帧才能收敛到新值。如果球场地摩擦和初始值 `friction_init=0.3` 差异大（如实际 0.5），前 0.3s 预测全部偏差。

**实施**（`ball_prediction.py:_update_friction`）：

- **自适应更新率**：`_update_count` 计数器记录摩擦更新次数。
  - 前 30 帧：`rate = 0.3`（`0.7*old + 0.3*decel`），快速收敛到实际摩擦。
  - 30 帧后：`rate = 0.1`（`0.9*old + 0.1*decel`），稳定跟踪。
- `reset()` 重置 `_update_count`，确保每半场重新快速收敛。
- **预期收益**：开局摩擦估计更快收敛，预测更准确。

#### 10.3 ✅ 线性回归假设匀速，与摩擦模型矛盾（已实施 2026-07）

**原问题**：速度估计用最小二乘线性回归（假设位置-时间线性，即匀速），但实际球有摩擦减速（非线性）。用匀速模型估出的速度是"平均速度"，比瞬时速度偏小，代入指数摩擦模型会累积误差。

**实施**（`ball_prediction.py:_regress_velocity`）：

- 旧的最小二乘回归替换为**最近 3 帧差分 + 摩擦补偿**（见 10.1 实施方案）——正是原建议的简化方案 `v0 = v_recent * exp(mu * dt_avg)` 的精确实现。
- 当只有 2 帧时回退到普通 2 点差分。
- **预期收益**：速度估计模型一致，瞬时速度计算准确。

#### 10.4 ✅ predict_rest_distance 上限 20m 过大（已实施 2026-07）

**原问题**：`_MAX_REST_DISTANCE=20m`，场地才 14m。球不可能滚 20m 不停（会撞墙或出界）。

**实施**（`ball_prediction.py`）：

- `_MAX_REST_DISTANCE` 20m → **16m**（`field_length+2`），clamp 到合理范围。
- **预期收益**：避免异常预测值传入下游。

---

## 11. 导航与避障评价

### ✅ 优点

- `ObstacleCollector` 无状态设计，每帧从真值重新收集，不会积累错误。
- 不同类型障碍用不同半径（对手 0.55 / 队友 0.48 / 门柱 0.18），层次合理。
- `keeper_goal_obstacles` 守门员专属障碍，确保守门员不会撞自家球门结构。

### ⚠️ 问题与建议

#### 11.1 🟡 障碍半径不随相对速度调整

**问题**：所有对手障碍半径固定 0.55m。但一个高速冲向我的对手（相对速度 2m/s）和静止对手需要的避让距离完全不同——高速接近的对手应视为更大半径（提前避让）。

**建议**：

- **策略**：动态障碍半径：
  ```
  effective_radius = base_radius + rel_speed * time_horizon * weight
  ```

  其中 `rel_speed` 是对手相对我的速度，`time_horizon` 是预测时域（如 0.5s）。
- **预期收益**：动态避让更智能，减少高速碰撞。

#### 11.2 🟡 球门障碍对所有玩家一视同仁

**问题**：`goal_structure_obstacles` 对所有玩家（包括 Chaser）都生成球门障碍（门柱 0.18 / 网边 0.20）。Chaser 在对方禁区射门时需要贴近球门，固定 0.18 半径可能过度避让，导致射门位置偏外。

**建议**：

- **策略**：对方球门障碍对 Chaser（射门时）可减小半径（如 0.10），或仅在 Chaser 非射门状态时全半径避让。区分"我方球门"（全避让）和"对方球门"（射门时可贴近）。
- **预期收益**：Chaser 射门位置更优。

#### 11.3 🟢 障碍无优先级

**问题**：所有障碍等权处理。但对手（动态、可能主动碰撞）和队友（合作、会主动避让）应有不同优先级。

**建议**：

- **策略**：队友障碍半径可略小（队友会互相避让），对手障碍应略大（对手不会让你）。当前 teammate=0.48 < opponent=0.55 已有差异，但可进一步拉开。
- **预期收益**：避让资源分配更合理。

---

## 12. 优化优先级总表

> 按"投入产出比"排序，🔴 为最值得优先处理的问题。

| 优先级 | 环节 | 问题 | 建议类型 | 预期收益 |
| ------ | ---- | ---- | -------- | -------- |
| ✅ | 1.1/2.1 | stale→None 导致全员骤停（已实施） | 策略+参数 | 消除传感器抖动丢球；球+位姿 LKG 已落地，game_state 有意保留保守 |
| 🔴 | 4.1 | 反应式避障密集场景卡死 | 策略（加局部规划） | 解决混战振荡 |
| 🟡 | 4.2 | 队友层双层避障冗余（已降级，原 🔴"方向矛盾"经核查不成立） | 策略（去重邻居） | 运动更平稳 |
| 🔴 | 6.1 | chaser 选举忽略对手 | 策略（加对手干扰项） | 减少 chaser 被截 |
| 🔴 | 8.1 | 接应位不规避对手 | 策略（加对手推开） | 接球后不丢球 |
| ✅ | 8.0/8.1/8.3 | 接应轨道抖动+对手规避+距离收紧（已实施） | 策略+参数 | 消除侧滑抖动；接球有空间；传球更精准 |
| ✅ | 9.1 | desperation 移除→max-dist+队友拦截（已实施） | 策略+参数 | 消除空门风险 |
| ✅ | 10.1 | 球预测历史短+回归模型矛盾（已实施） | 参数+策略 | 预测更准且模型一致 |
| ✅ | 3.1 | area_x 一参多用耦合（已实施） | 参数（拆分） | 独立调优 |
| 🟡 | 3.2 | 中场/进攻边界过宽 | 参数+策略 | SIDE 站位合理 |
| 🟡 | 4.3 | 近距离减速不平滑 | 参数（速度曲线） | 减少启停 |
| 🟡 | 4.4 | 角速度死区抖动 | 参数（线性过渡） | 消除振荡 |
| 🟡 | 5.1 | kick_power 配置不一致 | 策略（集中可调） | 统一调参入口 |
| 🟡 | 5.2 | 踢球力度不考虑距离 | 策略（动态 power） | 传球准/射门狠 |
| 🟡 | 6.2 | chaser 锁定不随球速 | 策略（动态锁定） | 锁定更贴合 |
| 🟡 | 6.3 | 槽位偏好硬编码 | 参数（移入 config） | 可调 |
| 🟡 | 7.1 | 传球阈值过高 | 参数(0.50~0.55) | 传球更积极 |
| 🟡 | 7.2 | 传球不考虑接球者射门 | 策略（加潜力分） | 传球更有价值 |
| 🟡 | 7.3 | 盘带撞向对手 | 策略（避对手方向） | 盘带更安全 |
| 🟡 | 8.2 | 三角角不随压迫调整 | 策略（动态角度） | 接应更灵活 |
| ✅ | 8.3 | 接应距离偏大（已实施） | 参数(2.8→2.2) | 传球更精准 |
| 🟡 | 9.2 | 守门员限速形同虚设 | 参数（确认/调整） | 反应更及时 |
| 🟡 | 9.3 | RUSH_OUT 让球逻辑（部分已实施） | 策略（严格条件） | 分阶段防误踢 |
| 🟡 | 9.4 | GUARD/RUSH_OUT 条件（部分已实施） | 策略（加球门区判定） | max-dist+队友双重过滤 |
| ✅ | 9.5 | 弧形站位动态缩放（已实施） | 策略（动态半径） | 站位自适应 |
| ✅ | 10.2 | 摩擦在线更新太慢（已实施） | 参数（自适应率） | 开局收敛快 |
| ✅ | 10.3 | 线性回归与摩擦矛盾（已实施） | 策略（3帧差分+补偿） | 速度估计模型一致 |
| ✅ | 10.4 | rest_distance 上限过大（已实施） | 参数(20→16m) | 避免异常值 |
| 🟡 | 11.1 | 障碍半径不随速度 | 策略（动态半径） | 高速不碰撞 |
| 🟡 | 11.2 | 球门障碍对 Chaser 过度 | 策略（区分敌我球门） | 射门位置更优 |
| 🟢 | 1.2 | 异常兜底过粗暴 | 策略（分级降级） | 单点不波及全队 |
| 🟢 | 1.3 | 缺乏性能闭环 | 策略（统计反馈） | 数据驱动迭代 |
| 🟢 | 2.2 | 开球参数硬编码 | 参数（移入 config） | 可调 |
| 🟢 | 2.3 | 阶段切换无过渡 | 策略（插值平滑） | 减少抖动 |
| 🟢 | 3.3 | 边线恢复方向固定 | 策略（随球速） | 恢复更精准 |
| 🟢 | 4.5 | lateral 限速偏低 | 参数（确认硬件） | 横移更快 |
| 🟢 | 5.3 | 射门不考虑守门员 | 策略（加守门员惩罚） | 得分率提升 |
| 🟢 | 6.4 | 缺乏角色轮换 | 策略（体能概念） | 分散负荷 |
| 🟢 | 7.4 | SIDE 不射门 | 策略（禁区允许） | 增加得分点 |
| 🟢 | 8.4 | blend 过渡区间窄 | 参数（扩到0.7） | 切换更平滑 |
| 🟢 | 11.3 | 障碍无优先级 | 策略（差异化） | 避让更合理 |
| ✅ | 新 | 守门员当 chaser 外场互漂+危险区站法（已实施 2026-07） | 策略 | 消除外场漂移出比赛区域 |
| ✅ | 新 | chaser 射门区域门槛+was_dribbling 迟滞（已实施 2026-07） | 策略+参数 | 减少本方半场远射+决策抖动 |
| ✅ | 新 | strafe 朝向优先+远距直线追赶（已实施 2026-07） | 策略+参数 | supporter 面向球更快到位 |
| ✅ | 新 | GK 分阶段出击 far→near+max-dist 滤网（已实施 2026-07） | 策略+参数 | 减少贸然出击+防空门 |
| ✅ | 新 | GK 弧形站位动态缩放（已实施 2026-07） | 策略 | 站位自适应 |
| 🟢     | 4.5         | lateral 限速偏低                                          | 参数（确认硬件）     | 横移更快                                                        |
| 🟢     | 5.3         | 射门不考虑守门员                                          | 策略（加守门员惩罚） | 得分率提升                                                      |
| 🟢     | 6.4         | 缺乏角色轮换                                              | 策略（体能概念）     | 分散负荷                                                        |
| 🟢     | 7.4         | SIDE 不射门                                               | 策略（禁区允许）     | 增加得分点                                                      |
| 🟢     | 8.4         | blend 过渡区间窄                                          | 参数（扩到0.7）      | 切换更平滑                                                      |
| 🟢     | 9.5         | 弧形站位参数固定                                          | 策略（动态半径）     | 站位优化                                                        |
| 🟢     | 10.4        | rest_distance 上限过大                                    | 参数(降到16m)        | 避免异常值                                                      |
| 🟢     | 11.3        | 障碍无优先级                                              | 策略（差异化）       | 避让更合理                                                      |
| ✅     | 新          | 守门员当 chaser 外场互漂+危险区站法（已实施 2026-07）     | 策略                 | 消除外场漂移出比赛区域                                          |
| ✅     | 新          | chaser 射门区域门槛+was_dribbling 迟滞（已实施 2026-07）  | 策略+参数            | 减少本方半场远射+决策抖动                                       |
| ✅     | 新          | strafe 朝向优先+远距直线追赶（已实施 2026-07）            | 策略+参数            | supporter 面向球更快到位                                        |

---

## 13. 已实施优化汇总（Phase 1+2+3，2026-07 日志驱动）

针对仿真日志 `Agent Runtime.log`（16:11:15–16:22:33，score 3:1）中发现的 **7 大问题** 的三阶段修复。

### Phase 1 — 守门员当 chaser 时外场互漂消除（P1/P2/P3）

**架构缺陷**：球在本方危险区时 `select_chaser` 可能返回守门员（KEEPER 在危险区 eligible），但 `assign_roles` 先匹配 GK 再匹配 chaser → 守门员获 GK 角色、无球员获 CHASER、外场全变 SUPPORTER。support_target 的 chaser 扫描排除了 GK → 两 supporter 互为 chaser 参照 → 三角几何+pushout+clamp 漂移出比赛区域。日志 16:16:32–16:17:06（34 秒）球卡在球门时，p1 沿 y=-4.25（边线）从前场角旗漂移向中场。

**实施文件**：`play/playbook.py`(RoleAssignment 增字段/assign_roles 填充)、`runtime.py`(SoccerKit.current_roles 槽)、`play/nodes.py`(AssignRoles 写 kit)、`play/default_roles.py`(SupporterRole.target 分派)、`tactics/targeting/support.py`(+chaser_id/危险区 fallback)、`targeting/__init__.py`(透传)

**关键设计**：

- `RoleAssignment` 增 `chaser_id`/`goalkeeper_id` 字段；`SoccerKit.current_roles(Any)` 避免 runtime→play 反向依赖
- `support_target` 接真实 `chaser_id`；当 `chaser_id == goalkeeper_id`（无外场 chaser）→ 调用 `_danger_zone_support_target`
- 危险区站法：sorted 外场列表按 index 奇偶分 **cover**（球→球门线 × `cover_depth` 封射门）和 **outlet**（前场 `outlet_forward`/`outlet_lateral` 接解围）
- 通用于任意 `keeper_id`（1/2/3）
- 新配置：`support_danger_cover_depth_m=1.2`、`support_danger_outlet_forward_m=4.0`、`support_danger_outlet_lateral_m=2.0`

**验证**：cover(-7.037,1.199) ✓；outlet(-3.030,2.960) ✓；gk=2 时 p1(cover)/p3(outlet) ✓。

### Phase 2 — chaser 决策门槛（P5/P6）

**P6 本方半场远射**：日志多处显示 ball.x=-6.5~-3.5 时 shoot at (7,0)（13m+），低命中率长传。
**P5 shoot/dribble 快速抖动**：16:16:00.093–.362 之间 shoot↔dribble 翻转 2 次。

**实施文件**：`tactics/targeting/attack.py`(select_kick_target 加区域门/shot_lane_is_clear 加 was_dribbling)、`play/default_roles.py`(ChaserRole 透传)、`targeting/__init__.py`(透传)

**关键设计**：

- `_shot_zone_allowed`：`ball.x ≥ shoot_min_ball_x_m AND dist(ball, opp_goal) ≤ shoot_max_distance_m`
- `shot_lane_is_clear` 增加 `was_dribbling` 参数 → 从 dribble 切 shoot 需 `shoot_enter_from_dribble_score(0.65)`（idle→shoot: 0.45、shoot→hold: 0.25）
- 新配置：`shoot_min_ball_x_m=0.0`、`shoot_max_distance_m=7.0`、`shoot_enter_from_dribble_score=0.65`

### Phase 3 — 机动质量（P4/P7）

**P7 strafe 背对球**：日志显示 supporter facing_err 高达 2–3 rad（114°–172°）。strafe 模式下无对齐门 → 严重失配时边平移边慢转。
**P4 supporter 被遗留在后**：16:15:40–46 球在前场 (5.19,-1.8) 时 p1 仍在 (-5.67,-1.40)、sc_dist≈7.5–8.6。远距时三角侧偏使 supporter 横向跑弧形而非直接追赶。

**实施文件**：`tactics/motion.py`(strafe 块加 align_factor)、`tactics/targeting/support.py`(远距重接应对)

**关键设计**：

- strafe `align_factor = max(0, 1 − |final_theta_error| / strafe_align_gate_rad)` 缩放 vx/vy；1.2 rad→0（纯旋转），0 rad→1.0（全速）
- 远距 `sc_dist > support_reengage_distance_m` 时三角侧偏角置 0（直线逼近 chaser 至 max_dist 内再走三角）
- 新配置：`strafe_align_gate_rad=1.2`、`support_reengage_distance_m=4.0`

### Phase 4 — GK 守门员系统改进（P8）

**max-dist RUSH_OUT 过滤**：守门员常在球离球门线还远（>1.5m）时贸然出击，给对手吊射空门机会。日志 19:06:55-19:08:05 显示 GK 反复触发 RUSH_OUT→GUARD 循环但实际不靠近球。
**队友拦截检查**：外场队友已逼近预测停止点时 GK 仍冲出去，两者争球给对手制造空门机会。
**分阶段出击**：RUSH_OUT 中 GK 在到达球前（>0.5m）直接跑向预测停止点（stage far），到近处（≤0.5m）才绕到球后调整踢球方向（stage near），消除接近途中的误踢。
**滚动站位**：GK GUARD 弧形半径和站位深度随球距动态缩放，近球全宽封堵、远球收紧防吊门。

**实施文件**：`default_roles.py`(+_rush_approach_stage/_gk_teammate_is_closer/_gk_log_throttled/max-dist/两阶段_rush_out_target)、`config.py`(+gk_rush_out_max_dist_m/ball_prediction_history_size→20)、`ready_stance.py`(动态缩放)、`ball_prediction.py`(3 帧差分+补偿回归/自适应摩擦率)

### 全部改动文件（14+ 个）

`playbook.py` `runtime.py` `nodes.py` `play_subtree.py` `default_roles.py` `support.py` `targeting/__init__.py` `attack.py` `motion.py` `config.py` `ready_stance.py` `ball_prediction.py` + 文档 N 个。`python -m compileall src` exit 0。

---

## 附：优化实施建议路径

若要系统性优化，建议按以下顺序推进（每步可独立验证）：

### 第 0 波（日志驱动稳定性，✅ 已实施 2026-07）

1. **守门员当 chaser 外场互漂**（Phase 1）— 支持者按真实角色锚定，危险区分 cover/outlet，消除漂移
2. **接应位规避对手**（8.1，Phase 1 含）— 提升接球后控球率
3. **挤位轨道锚定**（8.0/8.3，前序实施）— 死区+速率限制消除 side oscillation
4. **射门区域门槛+盘带迟滞**（Phase 2）— 减少本方半场无意义远射+决策翻转
5. **strafe 朝向优先+远距直线追赶**（Phase 3）— supporter 面向球更快到位
6. **GK 分阶段出击+max-dist 滤网**（Phase 4）— 消除空门风险、防止贸然出击
7. **GK 弧形站位动态缩放**（Phase 4）— 近球宽幅封堵、远球收紧防吊门
8. **球预测全面升级**（Phase 4）— history_size 10→20、3 帧差分+摩擦补偿、自适应摩擦率、MAX_REST_DISTANCE 20→16

### 第一波（稳定性，🔴 优先）

1. **stale grace 窗口**（1.1/2.1）— 消除非受迫丢球，最直接影响比赛稳定性
2. **chaser 加对手干扰**（6.1）— 提升抢球成功率
3. ~~**desperation 阈值收紧**（9.1）— 已实施~~ ✅

### 第二波（运动质量，🔴+🟡）

5. **队友避让去重**（4.2，已降级 🟡）— 减少冗余过冲
6. **密集场景局部规划**（4.1）— 解决混战卡死
7. **近距离速度平滑**（4.3/4.4）— 减少启停振荡
8. ~~**球预测历史扩容**（10.1）— 已实施~~ ✅

### 第三波（战术提升，🟡）

9. **传球系统优化**（7.1/7.2/5.2）— 传球更积极更有价值
10. **参数解耦集中化**（3.1/5.1/6.3）— 为后续调参打基础
11. **区域策略细化**（3.2/9.4）— 站位和出击更精准

### 第四波（锦上添花，🟢）

12. 按需实施绿色项

> 每波结束后建议做 A/B 对比测试（旧策略 vs 新策略各跑 N 场），用得分/控球率/丢球数量化收益。
