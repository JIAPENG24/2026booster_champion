# game3V3 — 3v3 Simulation Soccer Agent

## Project Identity

Python 3v3 simulation soccer Agent built on `booster_agent_framework`. Reads team-view ROS2 ground truth + GameController referee state, runs a `py_trees==2.4.0` behavior-tree strategy, and sends walk/kick commands to 3 robots. Runs inside **Booster Studio** virtual robots.

- Entry: `src/main.py:SoccerSimAgent`
- Config: `agent.toml` (id=`com.example.game3v3`, version=`1.1.0`)
- Build: `build.toml` (depends on `py_trees==2.4.0`, supports `sim_x86_64/sim_aarch64/real_jetson`)
- Control loop: 30 Hz (`SOCCER_CONTROL_HZ` env override)
- API level: `min_api_level=10700`, models = `Booster T1`, `Booster K1`

## Architecture

### Layer Diagram (one-way dependencies)

```
play/              -> behavior_tree + tactics + soccer_framework (+ SoccerKit from runtime)
behavior_tree/     -> runtime (types only) + tactics + soccer_framework
tactics/           -> soccer_framework
runtime.py         -> play + behavior_tree + tactics + soccer_framework
soccer_framework/  -> (no internal dependencies)
```

`soccer_framework/` never imports any upper layer. `SoccerKit` in `runtime` does not depend on `play/` — the two are decoupled via the `Playbook` protocol.

### Directory Structure

```
src/
├── soccer_framework/   # Hardware adapter + public API (PlayContext, RobotCommand, SoccerConfig)
│   ├── config.py       # SoccerConfig (from_env), SoccerStrategyTuning (tuning params), SoccerDebugConfig
│   ├── types.py        # Core data types, PlayContextProvider, ADULT_FIELD_DIMENSIONS
│   ├── game_state.py   # GameController JSON encode/decode
│   ├── ros_truth.py    # RosTruthProvider — ROS ground truth adapter
│   ├── robot.py        # TeamRobotManager + kick/control adapters
│   ├── game_controller.py # GameControllerRosProvider
│   ├── ros_adapter.py  # SoccerRosAdapter (node/subscription/executor owner)
│   └── telemetry.py    # SoccerLogger + JSONL structured logging
├── tactics/            # Pure model layer — NO BT/ROS dependency
│   ├── geometry.py     # TeamFieldFrame + coordinate transforms + field clamp
│   ├── navigation.py   # ObstacleCollector — obstacle definitions
│   ├── targeting/      # Tactical targets by responsibility
│   │   ├── __init__.py       # Targeting facade (stable public API)
│   │   ├── predicates.py     # Field zone / role predicates
│   │   ├── attack.py         # Shoot/pass/dribble scoring
│   │   ├── support.py        # Support positioning
│   │   ├── recovery.py       # Sideline recovery targets
│   │   ├── restart.py        # Opponent restart avoidance
│   │   └── ball_prediction.py # Ball trajectory prediction (for GK)
│   ├── motion.py       # MotionController — obstacle avoidance + walk/kick commands
│   ├── kick_hysteresis.py # Kick enter/exit hysteresis
│   └── ready_stance.py # READY positioning for kickoff/set plays
├── behavior_tree/      # BT framework — blackboard, nodes, subtrees, assembly
│   ├── blackboard.py   # BlackboardKeys + BlackboardClient
│   ├── tree.py         # TeamStrategyTree + create_team_tree
│   ├── ready_subtree.py
│   ├── safety_subtree.py
│   └── nodes/
│       ├── data.py        # Blackboard-writing data leaf nodes
│       ├── conditions.py  # Ball/rule/hardware conditions
│       └── actions.py     # StopAll, GoReadyTarget, CommitTeamCommands, etc.
├── play/               # CORE — PLAY-phase dynamic role strategy
│   ├── playbook.py     # Playbook base, DefaultPlaybook, RoleAssignment, select_chaser
│   ├── default_roles.py # ChaserRole, SupporterRole, GoalkeeperRole, DefenderRole
│   ├── role.py         # RoleStrategy base + RoleRegistry
│   ├── registry.py     # PlaybookRegistry + global PLAYBOOKS
│   ├── play_subtree.py # PlayingPhase subtree + kickoff state machine
│   └── nodes.py        # Shared leaf nodes + build_attack_subtree
├── runtime.py          # SoccerKit + SoccerTeamRuntime — assembly/control loop
└── main.py             # Agent entry + lifecycle
```

## Behavior Tree Overview

```
Sequence(TeamRoot)
├── Sequence(DataLayer)                      # Blackboard refresh (clock, context, game, ball, poses, status x3)
├── Selector(MatchControl)                   # Core match decision
│   ├── Selector(SafetyGuards)               # 5 guards — any hit → StopAll
│   │   ├── NoGameStop                       # No GameController state
│   │   ├── AllInactiveStop                  # All players penalized/offline
│   │   ├── StoppedPlayStop                  # Referee stopped=true
│   │   ├── NonPlayingStop                   # TIMEOUT/INITIAL/SET/FINISHED
│   │   └── NoPlayingBallStop                # PLAYING but ball missing
│   ├── Sequence(ReadyPhase)                 # READY → GoReadyTarget(N)
│   ├── Sequence(PlayingPhase)               # PLAYING → PlaybookCore
│   │   └── Sequence(PlaybookCore)
│   │       ├── AssignRoles(...)             # /team/roles blackboard
│   │       └── Parallel(Roles)
│   │           └── Selector(Player(N))      # x3 players (see below)
│   └── StopAll("unsupported state")         # Fallback
├── Parallel(SafetyOverrides, SuccessOnAll)  # Hardware safety overlay
│   └── Sequence(PlayerSafety(N)) x3
│       ├── Selector(AllowedGuard)           # Penalized → StopPlayer
│       ├── Selector(FallDownGuard)          # Fallen → TriggerGetUp + StopPlayer
│       └── Selector(WalkModeGuard)          # Not walk mode + need move → TriggerEnterWalkMode
└── CommitTeamCommands                       # Final dispatch
```

### Player(N) Template (inside PlayingPhase)

```
Selector(Player(N))
├── Sequence(KickoffHold)                    # Opponent kickoff → hold ready position
│   ├── IsOpponentKickoffActive
│   └── GoReadyTarget(N)
├── Sequence(PenaltyAvoid)                   # Opponent restart → avoid distance
│   ├── IsOpponentRestartActive
│   └── AvoidOpponentRestart(N)
├── Sequence(AsChaser)                       # IsRole("chaser") → kick or approach
├── Sequence(AsSupporter)                    # IsRole("supporter") → support position
├── Sequence(AsGoalkeeper)                   # IsRole("goalkeeper") → guard
├── Sequence(AsDefender)                     # IsRole("defender") — custom extension slot
└── WaitForBall(N)                          # Fallback Stop when no role matches
```

### Kickoff Phase State Machine (PlayKickoffController)

4 phases on `/play/kickoff_phase` blackboard:

- **Phase 0 NormalPlay**: DefaultPlaybook.assign_roles runs normally
- **Phase 1 ActiveKickoff**: Our kickoff before ball touched → CENTER=Kicker(approach offset=0.4, kick angle=45°, power=1.0, landing dist=2.5m), SIDE=Chaser(to landing point, speed 1.3x), KEEPER=goalkeeper guard
- **Phase 2 RoleLockPlay**: Ball moved >0.15m + 0.5s delay → AssignFixedRoles(center=supporter, side=chaser, keeper=goalkeeper)
- **Phase 3 OppKickoffDefense**: Opponent kickoff after secondary_time→0 → approach ball with speed multiplier (CENTER 0.5x, SIDE 1.3x, KEEPER 1.0x, all approach_offset=0.4)

Transitions: Phase0→1 (our kickoff detected), Phase1→2 (ball moved + 0.5s), Phase2→0 (SIDE<0.3m for 1.0s or 2.0s timeout), any→3 (opponent kickoff ended, secondary_time falling edge), Phase3→0 (teammate<0.3m to ball or 5.0s timeout)

## Coordinate System

- **Team-view field coordinates**: own goal at `x=-field_length/2=-7.0`, opponent goal at `x=+field_length/2=+7.0`, +x = attacking direction, -x = defensive
- Simulation ground truth is already team-relative. **Do NOT mirror by team_id** inside `play/` or `tactics/`.
- If future input provides absolute coordinates, normalize in PlayContextProvider adapter.

### Field Dimensions (ADULT_FIELD_DIMENSIONS)

| Parameter | Value |
|-----------|-------|
| length | 14.0 m |
| width | 9.0 m |
| penalty_dist | 2.1 m |
| goal_width | 2.6 m |
| circle_radius | 1.5 m |
| penalty_area_length | 3.0 m |
| penalty_area_width | 6.0 m |
| goal_area_length | 1.0 m |
| goal_area_width | 4.0 m |
| goal_depth | 0.6 m |

## ReadySlot Mapping

| Slot | Default player_id | Role |
|------|------------------|------|
| CENTER | 1 | Center kickoff |
| SIDE | 2 | Side support |
| KEEPER | 3 | Goalkeeper |

`DEFAULT_READY_SLOT_SEQUENCE = (CENTER, SIDE, KEEPER)`. Player IDs from `SOCCER_ROBOT_NAMES` env (first=1, second=2, third=3). **Never hard-code player_id or team_id**.

## Role System

### Default Roles (registration order = Selector branch priority)

| Role | Class | build_subtree params | Priority |
|------|-------|---------------------|----------|
| `chaser` | ChaserRole | speed=2.0, kick_power=2.5, approach_offset=0.15m behind ball | 1 |
| `supporter` | SupporterRole | speed=2.0, kick_power=2.5, hold_vyaw=0.25, strafe=True, dynamic 1-2.8m distance | 2 |
| `goalkeeper` | GoalkeeperRole | speed=2.2, kick_power=3.5, lateral_speed=1.0, hold_vyaw=0.12, strafe=True | 3 |
| `defender` | DefenderRole | hold_vyaw=0.12, reuses SupporterRole.target | 4 (extensioin) |

### Goalkeeper State Machine (3-state + desperation)

1. **GUARD** (default): Arc-based positioning via `goalkeeper_guard_target` formula — intersection of ball→goal-center line with arc(R=1.4) at depth=1.3m. Smoothstep lerps between arc and goal line. Face ball always.
2. **RUSH_OUT**: Ball predicted rest point inside defensive area (rest_x < area_x(own-2.8) - rush_margin(0.8 enter/0.3 exit), abs(y) ≤ 2.2) → rush out to clear. Uses `gk_state_confirm_frames=2` to enter, `gk_state_release_frames=4` to exit.
3. **LATERAL**: Ball predicted to cross goal line inside posts → slide along goal depth line. Minimum hold time `gk_lateral_hold_min_sec=0.8s`.
4. **DESPERATION**: ball.x < own_goal_x(-7.0) + gk_desperation_clear_margin_m(1.5) = -5.5 → direct rush at ball with offset=0.2m, kick_theta=0.0, always wants_to_kick.

Ball friction model: exponential decay v(t)=v0*exp(-mu*t), mu online-updated (0.9*old+0.1*decel, clamped[0.01,5.0]). Max horizon=2.0s, goal crossing max search=6.0s.

### select_chaser Algorithm (4-step, evaluated every frame)

1. **Slot eligibility**: KEEPER only in danger zone. SIDE only when `side_should_challenge() = True` (midfield/own half always; attack zone only if side_dist+0.20 < center_dist).
2. **Cost scoring**: `ball_claim_score(config, slot, pose, ball)` — distance-based with slot bias: KEEPER gets distance-0.75 in danger, distance+field_length otherwise; CENTER distance-0.20; SIDE distance-0.10.
3. **Tie-breaking**: `teammate_challenge_tie_margin_m=0.15m` bandwidth; min player_id wins.
4. **Dynamic chaser lock**: Prevents ping-pong. Lock durations: ball.x < own_x+1.5 → 3.0s; ball.x < `chaser_lock_defensive_x_ratio`(0.20→-2.8m) → 2.0s; else 0.5s. In defensive zone, lock refreshes every frame. If old chaser's score ≤ best+0.5, keep old chaser.

## MotionController — 3-Layer Reactive Controller

No global path planning. Computes velocity every frame from current obstacle layout.

### Layer 1: Path avoidance (`_avoidance_target`)
- Find first blocking obstacle on target direction
- Generate lateral via-point
- Side selection persisted across frames (memory)

### Layer 2: Walk control (`_compute_velocity`)
- Arrival check: distance<0.15m + angle<0.20rad → hold or stop
- Near but misaligned: pure rotation
- Strafe mode: vx=v*cos(err), vy=v*sin(err), per-axis clamped + heading correction; align_factor=max(0,1-|err|/strafe_align_gate_rad) scales vx/vy for heading priority (⪆1.2rad→pure rotation)
- Non-strafe: angle_error>0.5rad → pure rotation; else vx=gain*distance*cos(err)*mult, vy=0, vyaw=angular_velocity(err)

### Layer 3: Yaw avoidance (`_apply_yaw_avoidance`)
- PLAY stage: only teammates, **NOT opponents** (`avoid_opponents=False`)
- Each neighbor contributes yaw bias (scale[0,1] by min_distance/horizon), side_sign by relative lateral or player_id parity
- Accumulated bias applied to vyaw, clamped to max_angular_speed

### Walking Constants

| Constant | Value | Note |
|----------|-------|------|
| ARRIVE_DISTANCE | 0.15 m | Threshold reached target |
| ARRIVE_ANGLE | 0.20 rad (~11.5°) | Arrival heading tolerance |
| TURN_THRESHOLD | 0.50 rad | Large error → pure rotation |
| ANGULAR_SPEED_FLOOR | 0.25 rad/s | Prevents small overshoot |
| ANGULAR_DEAD_ZONE | 0.15 rad | Below this → pure proportional |
| LINEAR_SPEED_FLOOR | 0.30 m/s | Min linear speed |
| LINEAR_GAIN | 2.0 | Linear velocity gain |

## Key SoccerStrategyTuning Parameters (config.py)

### Speed Limits
- max_linear_speed=0.8, max_lateral_speed=0.6, max_angular_speed=1.0

### Kick Hysteresis
- soccer_kick_enter_distance=2.0, exit=2.5, power=1.5, kickoff_kick_power=1.0, min_active_sec=0.7, exit_delay=0.2

### Path Obstacle Avoidance (Layer 1)
- opponent_obstacle_radius=0.55, teammate_obstacle_radius=0.48, safety_margin=0.22, start_ignore=0.35, target_ignore=0.35

### Yaw Avoidance (Layer 2, teammates only in PLAY)
- horizon_sec=1.0, min_distance_m=0.78, bias_max=0.6

### Restart
- opponent_restart_avoid_distance_m=1.6 (rule 1.45 + 0.15 buffer), restart_touch_distance=0.45

### Ball Claim / Passing / Dribble / Support
- teammate_challenge_tie_margin_m=0.15
- chaser_lock_defensive_x_ratio=0.20 (chaser lock ladder defensive tier, decoupled from KEEPER area — issue 3.1)
- midfield_boundary_x_ratio=0.20 (SIDE midfield/attack challenge boundary — issue 3.1)
- pass_enabled=True, pass_min_score=0.60, pass_min_forward_m=0.35, pass_lane_clearance=0.75
- dribble_advance_m=1.5, dribble_center_pull=0.65
- support_min_spacing_m=0.9, support_min_distance_m=1.0, support_max_distance_m=2.2, support_angle_hold_deadzone=0.35, support_opponent_avoid_radius_m=0.6, support_target_smooth_speed=2.5
- support_danger_cover_depth_m=1.2, support_danger_outlet_forward_m=4.0, support_danger_outlet_lateral_m=2.0, support_reengage_distance_m=4.0
- (deprecated, unused) support_depth_m=1.05, support_lateral_m=1.25

### Shoot Decision
- shoot_min_ball_x_m=0.0, shoot_max_distance_m=7.0, shoot_enter_from_dribble_score=0.65

### Motion Strafe (Phase 3)
- strafe_align_gate_rad=1.2

### Goalkeeper
- challenge_area_x_ratio=0.20, challenge_area_y=2.2, challenge_hysteresis_m=0.30
- clear_hold_sec=1.5, rush_speed_multiplier=2.2, kick_power=3.5, lateral_speed=1.0
- guard_arc_radius=1.4, guard_depth_m=1.3, rush_speed_ratio=0.8
- desperation_clear_margin_m=1.5

### GK State Machine
- confirm_frames=2, release_frames=4, target_smooth_speed=2.0
- rush_out_margin_m=0.8, rush_out_exit_margin_m=0.3
- lateral_hold_min_sec=0.8
- gk_state_confirm_frames=2 (enter RUSH_OUT), gk_state_release_frames=4 (exit RUSH_OUT)

### Ball Prediction
- history_size=10, kp=0.6, ki=0.05, kd=0.1, friction_init=0.3, max_horizon_sec=2.0

### Sideline / Goal Line Recovery
- recovery_margin_m=0.90, infield_m=1.60, advance_m=0.75, goal_line_margin_m=0.15

## Red Lines (Architecture Constraints — NEVER BREAK)

### Technical Compliance
- Only use documented public ROS topics / Agent API / SDK interfaces
- No websocket/HTTP/shared-memory access to simulation internals
- No fake control commands (move ball, move/reset robot, pause/close sim)
- No connection to GameController admin interface
- No port scanning / /proc reading / Docker metadata access
- No interference with opponent agent, host services, or referee

### Architecture (framework invariants)
- AR-01: Must NOT bypass SafetyOverrides — it runs BEFORE CommitTeamCommands
- AR-02: Must NOT bypass DataLayer — blackboard is sole data source per frame
- AR-03: Must NOT bypass CommitTeamCommands — only handshake point to executor
- AR-04: `play/` MUST NOT import `py_trees` — strategy layer stays framework-agnostic
- AR-05: Strategy code must NOT call boosteros directly — only output RobotCommand
- AR-06: Player(N) guards (KickoffHold/PenaltyAvoid) must be preserved
- AR-07: WaitForBall fallback must be preserved
- AR-08: SafetyGuards must NOT be bypassed (higher priority than PlayingPhase)
- AR-09: SafetyOverrides 3 sub-guards (Allowed/FallDown/WalkMode) must NOT be bypassed
- AR-10: set_velocity and SoccerKickManager are mutually exclusive per robot per frame

### Coordinate & Config
- CR-01: No hardcoded team_id — read from `SOCCER_TEAM_ID` env
- CR-02: No hardcoded player_id mapping — read from `SOCCER_ROBOT_NAMES` env
- CR-03: Own goal at x=-7.0, opponent goal at x=+7.0, +x = attack direction
- CR-04: No secondary coordinate mirroring by team_id in play/tactics
- CR-05: No hardcoded field dimensions — read from config

## Field Zone Predicates (for strategy conditions)

All from `tactics/targeting/predicates.py`, first argument is always `config`.

| Function | Condition | Threshold |
|----------|-----------|-----------|
| `ball_in_own_defensive_area(config, ball)` | ball.x < area_x AND abs(ball.y) ≤ area_y | area_x = -field_len*`goalkeeper_challenge_area_x_ratio`(0.20) = -2.8m, area_y = min(width/2-0.35, 2.2) = 2.2m |
| `ball_beyond_goal_line(config, ball)` | abs(ball.x) > half_len + 0.15 | 7.15m |
| `ball_beyond_own_goal_line(config, ball)` | ball.x < -7.15 | -7.15m |
| `ball_near_sideline(config, ball)` | abs(ball.y) ≥ width/2 - 0.90 | 3.6m |
| `ball_is_in_midfield_or_own_half(config, ball)` | ball.x < field_len * `midfield_boundary_x_ratio`(0.20) | +2.8m |

> Decoupling note (issue 3.1): the three field-zone ratios are independent params.
> `goalkeeper_challenge_area_x_ratio` (KEEPER challenge area, -2.8m),
> `chaser_lock_defensive_x_ratio` (chaser lock ladder defensive tier, -2.8m, see `play/default_roles.py`/`playbook.py`), and
> `midfield_boundary_x_ratio` (SIDE challenge boundary, +2.8m) all default to 0.20 but can be tuned separately.

### Performance Log Zones (separate from predicates, used in `_log_performance`)
- danger: ball.x < -3.5 (field_len*0.25)
- defensive: ball.x < 0
- midfield: ball.x < +3.5
- attack: ball.x ≥ +3.5

## Attack Scoring (`tactics/targeting/attack.py`)

### select_kick_target decision order (chaser/supporter)
1. Sideline recovery (ball near sideline)
2. Restart touch (our kickoff/set play, distance<0.45)
3. Shot (zone gate: ball.x ≥ shoot_min_ball_x_m AND dist_to_goal ≤ shoot_max_distance_m; lane_clear_score ≥ 0.45 enter / 0.25 hold, with was_shooting hysteresis; from dribble: require ≥ shoot_enter_from_dribble_score(0.65) with was_dribbling)
4. Pass (best_pass_target — weighted: lane_clear*0.55 + forward_gain*0.30 + center_pull*0.15 - distance_penalty, min_score=0.60, min_forward=0.35)
5. Dribble (target_x = ball.x+1.5, target_y = ball.y*0.65)

### GK kick_target decision
1. Desperation → opponent goal center
2. Static ball in goal area (ball.x < own_goal_x+0.8 = -6.2, speed<0.3) → center
3. 5 candidates (center/top-post/bottom-post/top-sideline/bottom-sideline), scored by lane_clear_score - 0.3*turn_difficulty, locked for entire clearance cycle

## Support Positioning (`tactics/targeting/support.py`)

- Targets chaser via `RoleAssignment.chaser_id` (down from `SupporterRole.target`). When chaser is an outfield player, use that specific teammate as reference. When `chaser_id == goalkeeper_id` (keeper elected as chaser in danger zone) → **danger zone fallback**.
- Dynamic distance=`[support_min_distance_m(1.0), support_max_distance_m(2.2)]`, lateral 10°-45° triangle.
- Behind direction: blend of goal direction + chaser direction (blend t = (bc_len-0.3)/0.5, transition 0.3-0.8m)
- When <min from target → push to min; when >max → pull to max; **in band + within `support_angle_hold_deadzone`(0.35rad) of ideal angle → hold current pose** (anti-orbit anchor, stops tangential sliding)
- `SupporterRole._smooth_target` rate-limits the target at `support_target_smooth_speed`(2.5m/s); >1.5m jump snaps (chaser/role switch)
- Spacing repulsion (two passes in `_spaced_support_target`): nearest teammate → `support_min_spacing_m`(0.9); nearest opponent → `support_opponent_avoid_radius_m`(0.6)
- Danger zone fallback (`_danger_zone_support_target`): sorted outfield players split by index parity into **cover** (ball→own goal line at `support_danger_cover_depth_m` from goal, blocks shot channel) and **outlet** (upfield at `support_danger_outlet_forward_m`/±`support_danger_outlet_lateral_m`, receives keeper clear).
- Reengage: when `sc_dist > support_reengage_distance_m(4.0)`, zero lateral offset (beeline approach to chaser at max_dist).
- No chaser (neither assigned nor eligible) → fallback to ball→own goal line 2.5m behind

## Known Design Gaps & Evaluation

See `.omo/docs/strategy-framework-analysis.md` (comprehensive architecture doc) and `.omo/docs/strategy-evaluation.md` (42 issues: 4🔴 19🟡 13🟢 open, ✅6 resolved — 1.1/2.1/3.1/8.0/8.1/8.3 + Phase 1/2/3 log-driven fixes below). Key 🔴 issues to be aware of:

1. ~~**Stale→None causes full-team stop**~~ **(RESOLVED 2026-07)** — ball and robot pose staleness now flow through LKG buffers: `BallLkgBuffer` (`soccer_framework/ball_lkg.py`, `ball_fresh_sec`+`ball_stale_grace_sec`=0.6s total) and `PoseLkgBuffer` (`soccer_framework/pose_lkg.py`, `robot_pose_fresh_sec`+`robot_pose_stale_grace_sec`=0.5s, per-team instances in `kit.pose_lkg_teammates`/`kit.pose_lkg_opponents`). `UpdateRecentBall`/`UpdateRobotPoses` delegate to these buffers; grace window does constant-velocity extrapolation (incl. theta, unwrapped) before clearing to None. Note: `game_state` stale→None is intentionally kept conservative (`_GAME_STATE_STALE_SEC=2.0`) to avoid playing through a referee STOP.
2. **Reactive avoidance deadlocks in dense play** — no path-planning layer; multiple obstacles in a narrow gap cause oscillating via-points.
3. **Chaser ignores opponent goalie** — `ball_claim_score` only considers distance+slot bias, does not account for opponent goalkeeper position when chasing near opponent goal.
4. ~~**Supporter ignores opponent robots**~~ **(RESOLVED 2026-07)** — `_spaced_support_target` now applies a second push-out pass from the nearest opponent at `support_opponent_avoid_radius_m`(0.6m), via the shared `_push_out_from` helper. Also fixed the orbit-instability root cause: in-band + within `support_angle_hold_deadzone` the supporter holds its pose (`_chaser_relative_target`), and `SupporterRole._smooth_target` rate-limits the target (`support_target_smooth_speed`). Distance bounds de-hardcoded to `support_min/max_distance_m` (max 2.8→2.2). See `docs/strategy-evaluation.md` §8.0/8.1/8.3.
5. **GK desperation clears into empty net** — approach_offset=0.2m, kick_theta=0.0 (straight toward opponent), no check if goalie is between ball and our goal.
6. **Ball prediction history too short** — 10 frames at 30Hz = 0.33s, insufficient for goal-crossing prediction.
7. **GK target speed limit defeated** — `gk_target_smooth_speed=2.0` but actual RUSH_OUT commands use `speed_multiplier=2.2`, and the speed limit only applies to the target smoothing, not the movement command.
8. ~~**Keeper-as-chaser leaves no outfield chaser**~~ **(RESOLVED 2026-07)** — when ball is in danger zone and `select_chaser` returns the goalkeeper (KEEPER eligible in danger zone), `assign_roles`'s goalkeeper-first check left no player with ROLE_CHASER → both outfielders became SUPPORTER, mutual-referenced → drifted to sideline. Fix: `RoleAssignment` carries `chaser_id`/`goalkeeper_id`; `SupporterRole.target` uses real chaser_id; when `chaser_id == goalkeeper_id`, outfielders split into **cover** (ball→goal line) and **outlet** (upfield). See `docs/strategy-evaluation.md` §13 Phase 1.
9. ~~**Chaser shoots from own half; shoot/dribble oscillation**~~ **(RESOLVED 2026-07)** — added zone gate (`shoot_min_ball_x_m`/`shoot_max_distance_m`) and `was_dribbling` hysteresis (`shoot_enter_from_dribble_score`). See §13 Phase 2.
10. ~~**Supporter strafes backward; far-distance lateral orbit**~~ **(RESOLVED 2026-07)** — strafe alignment gate (`strafe_align_gate_rad`) scales translation when heading misalignment >1.2rad; far suppression uses beeline (`support_reengage_distance_m`). See §13 Phase 3.

## Where to Start for Common Changes

| Goal | File |
|------|------|
| Role assignment (e.g., all-out attack) | `play/playbook.py:DefaultPlaybook.assign_roles()` |
| Chaser kick/pass target | `play/default_roles.py:ChaserRole.kick_target()` |
| Supporter positioning | `play/default_roles.py:SupporterRole.target()` |
| Goalkeeper guard formula | `tactics/ready_stance.py:goalkeeper_guard_target()` |
| Add new role (interceptor, second striker) | Derive `RoleStrategy`, call `register_role()` in `Playbook.__init__` |
| Register new Playbook | `PLAYBOOKS.register(name, factory)` in `play/registry.py` |
| Ball chasing score / who chases | `play/playbook.py:DefaultPlaybook.select_chaser()` |
| PLAY subtree shape (kickoff phases, etc.) | `play/play_subtree.py` |
| Shooting lane / pass score / dribble | `tactics/targeting/attack.py` |
| Support positioning algorithm | `tactics/targeting/support.py` |
| Sideline recovery / restart avoidance | `tactics/targeting/recovery.py` / `restart.py` |
| Obstacle avoidance / yaw avoidance | `tactics/motion.py` / `navigation.py` |
| Tune velocities, kick distances, margins | `soccer_framework/config.py:SoccerStrategyTuning` |

## Testing & Validation

- Build output: `build/com.example.game3v3-1.1.0.agent`
- Logs: JSONL structured logging via `SoccerLogger`
- Run in Booster Studio with simulator
- Default Playbook registered in `play/__init__.py` (last line)
- To test custom Playbook: `PLAYBOOKS.create("default", kit)` or pass object directly to `TeamStrategyTree`

## Development Convention

- `player_ids = range(1, len(robot_names)+1)` — do not hardcode
- Control loop: `SoccerTeamRuntime` — period = 1/control_hz, tree.tick(now, executor)
- Blackboard keys: `/clock/now`, `/play_context(PlayContext)`, `/team/roles(RoleAssignment)`, `/safety/active(bool)`, `/robot_status/{id}`, `/cmd/{id}`
- Kick hysteresis: enter at distance≤2.0m, exit at distance>2.5m (per-player state)
- All targeting functions return `Pose2D` — immutable (x, y, theta)
- Field line intersections use signed distance from field boundaries
