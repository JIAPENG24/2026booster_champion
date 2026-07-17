# game3V3 — 3v3 Simulation Soccer Agent

A complete Python 3v3 simulation soccer agent for the **Booster Champion** competition, built on `booster_agent_framework`. The agent reads team-view ROS2 ground truth and GameController referee state, runs a `py_trees==2.4.0` behavior-tree strategy at 30 Hz, and sends walk/kick commands to 3 robots inside **Booster Studio** virtual robots.

| Metadata | Value |
|----------|-------|
| Agent ID | `com.example.game3v3` |
| Version | `1.1.0` |
| API Level | `10700` |
| Supported Models | Booster T1, Booster K1 |
| Entry Point | `src/main.py:SoccerSimAgent` |

---

## Table of Contents

- [Architecture](#architecture)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Behavior Tree](#behavior-tree)
- [Coordinate System](#coordinate-system)
- [Role System](#role-system)
- [Kickoff State Machine](#kickoff-state-machine)
- [Goalkeeper System](#goalkeeper-system)
- [Key Tuning Parameters](#key-tuning-parameters)
- [Where to Start for Common Changes](#where-to-start-for-common-changes)
- [Extending the Agent](#extending-the-agent)
- [Development Notes](#development-notes)

---

## Architecture

### Layer Diagram (One-Way Dependencies)

```
play/              -> behavior_tree + tactics + soccer_framework (+ SoccerKit from runtime)
behavior_tree/     -> runtime (types only) + tactics + soccer_framework
tactics/           -> soccer_framework
runtime.py         -> play + behavior_tree + tactics + soccer_framework
soccer_framework/  -> (no internal dependencies)
```

`soccer_framework/` never imports any upper layer — it is the pure data-contract layer. `SoccerKit` in `runtime` does not depend on `play/`; the two are decoupled via the `Playbook` protocol.

### Directory Structure

```
src/
├── soccer_framework/       # Hardware adapter + public API
│   ├── config.py           # SoccerConfig, SoccerStrategyTuning (all tuning knobs)
│   ├── types.py            # Core data types: Pose2D, BallState, RobotState,
│   │                       #   PlayContext, GameControlState, RobotCommand
│   ├── game_state.py       # GameController JSON encode/decode
│   ├── ros_truth.py        # RosTruthProvider — ROS ground truth adapter
│   ├── robot.py            # TeamRobotManager + kick/control adapters
│   ├── game_controller.py  # GameControllerRosProvider
│   ├── ros_adapter.py      # SoccerRosAdapter (node/subscription/executor)
│   ├── ball_lkg.py         # Last-Known-Good ball buffer (stale extrapolation)
│   ├── pose_lkg.py         # Last-Known-Good pose buffer (per-team)
│   └── telemetry.py        # SoccerLogger + JSONL structured logging
├── tactics/                # Pure model layer — NO BT/ROS dependency
│   ├── geometry.py         # TeamFieldFrame + field clamp + coordinate transforms
│   ├── navigation.py       # ObstacleCollector — obstacle definitions
│   ├── motion.py           # MotionController — 3-layer reactive control
│   │                       #   (path detour → walking control → yaw avoidance)
│   ├── kick_hysteresis.py  # Kick enter/exit hysteresis
│   ├── ready_stance.py     # READY positioning for kickoff/set plays
│   └── targeting/          # Tactical targets by responsibility
│       ├── __init__.py     # Targeting facade (stable public API)
│       ├── predicates.py   # Field zone / role predicates
│       ├── attack.py       # Shoot/pass/dribble scoring + GK-aware shooting
│       ├── support.py      # Support positioning + spacing repulsion
│       ├── recovery.py     # Sideline recovery targets
│       ├── restart.py      # Opponent restart avoidance
│       └── ball_prediction.py  # Ball trajectory prediction (for GK)
├── behavior_tree/          # BT framework
│   ├── blackboard.py       # BlackboardKeys + BlackboardClient
│   ├── tree.py             # TeamStrategyTree + create_team_tree
│   ├── ready_subtree.py    # READY subtree factory
│   ├── safety_subtree.py   # SafetyGuards + SafetyOverrides
│   └── nodes/
│       ├── data.py         # Blackboard-writing data nodes
│       ├── conditions.py   # Ball/rule/hardware conditions
│       └── actions.py      # StopAll, GoReadyTarget, CommitTeamCommands, etc.
├── play/                   # PLAY-phase dynamic role strategy
│   ├── playbook.py         # Playbook base, DefaultPlaybook, RoleAssignment, select_chaser
│   ├── default_roles.py    # ChaserRole, SupporterRole, GoalkeeperRole, DefenderRole
│   ├── role.py             # RoleStrategy base + RoleRegistry
│   ├── registry.py         # PlaybookRegistry + global PLAYBOOKS
│   ├── play_subtree.py     # PlayingPhase subtree + kickoff state machine
│   └── nodes.py            # Shared leaf nodes + build_attack_subtree
├── runtime.py              # SoccerKit + SoccerTeamRuntime — assembly/control loop
└── main.py                 # Agent entry + lifecycle
```

---

## Quick Start

### Prerequisites

- [Booster Studio](https://studio.booster.tech/) (simulation environment)
- Python 3.10+ (for local development / unit testing)

### Run a Match in Booster Studio

1. Open Booster Studio and create a simulation session.
2. Set environment variables:
   - `SOCCER_TEAM_ID` — your team number (1 or 2)
   - `SOCCER_ROBOT_NAMES` — comma-separated robot names (e.g. `Nao1,Nao2,Nao3`)
   - `SOCCER_CONTROL_HZ` — control loop frequency (default 30, optional)
3. Build the agent: `build/com.example.game3v3-1.1.0.agent`
4. Load the agent into Booster Studio.

### Run Locally (Python)

```python
from src.behavior_tree import TeamStrategyTree
from src.play import PLAYBOOKS
from src.runtime import SoccerKit
from src.soccer_framework import SoccerConfig

kit = SoccerKit(SoccerConfig())
tree = TeamStrategyTree(kit, PLAYBOOKS.create_default(kit), context_provider)
```

---

## Configuration

### agent.toml

```toml
id = "com.example.game3v3"
version = "1.1.0"
name = { en = "game3V3", zh = "game3V3" }
entry = "src/main.py:SoccerSimAgent"
# ...
```

### build.toml

```toml
[python.dependencies]
common = ["py_trees==2.4.0"]

[platform]
supports = ["sim_x86_64", "sim_aarch64", "real_jetson"]
```

### Environment Variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `SOCCER_TEAM_ID` | Team number (1 or 2) | `1` |
| `SOCCER_ROBOT_NAMES` | Comma-separated 3 robot names | `robot1,robot2,robot3` |
| `SOCCER_CONTROL_HZ` | Control loop frequency (Hz) | `30` |

### SoccerStrategyTuning (`src/soccer_framework/config.py`)

All tactical parameters live in `SoccerStrategyTuning` as dataclass fields. Key groups:

- **Speed limits**: `max_linear_speed=0.8`, `max_lateral_speed=0.8`, `max_angular_speed=1.0`
- **Kick hysteresis**: `soccer_kick_enter_distance=2.0`, `soccer_kick_exit_distance=2.5`
- **Obstacle avoidance**: `opponent_obstacle_radius=0.55`, `teammate_obstacle_radius=0.48`
- **Passing**: `pass_min_score=0.60`, `pass_lane_clearance=0.75`
- **Support**: `support_min_distance_m=1.0`, `support_max_distance_m=2.2`
- **Goalkeeper**: `gk_rush_out_max_dist_m=1.5`, `gk_state_confirm_frames=2`

---

## Behavior Tree

### Tree Overview

```
Sequence(TeamRoot)
├── Sequence(DataLayer)                     # Blackboard refresh
│   ├── UpdateClock
│   ├── UpdatePlayContext
│   ├── UpdateGameState
│   ├── UpdateRecentBall
│   ├── UpdateRobotPoses
│   └── UpdateRobotStatus(N)               # x3 players
├── Selector(MatchControl)                 # Core match decision
│   ├── Selector(SafetyGuards)             # 5 guards — any hit → StopAll
│   │   ├── NoGameStop                     # No GameController state
│   │   ├── AllInactiveStop                # All players penalized/offline
│   │   ├── StoppedPlayStop                # Referee stopped
│   │   ├── NonPlayingStop                 # TIMEOUT/INITIAL/SET/FINISHED
│   │   └── NoPlayingBallStop              # PLAYING but ball missing+stale
│   ├── Sequence(ReadyPhase)              # READY → GoReadyTarget(N)
│   ├── Sequence(PlayingPhase)            # PLAYING → PlaybookCore
│   └── StopAll("unsupported state")
├── Parallel(SafetyOverrides, SuccessOnAll)# Hardware safety (penalty/fallen/walk)
│   └── Sequence(PlayerSafety(N)) x3
└── CommitTeamCommands                     # Final dispatch
```

### Player(N) Template (Inside PlayingPhase)

```
Selector(Player(N))
├── Sequence(KickoffHold)                  # Opponent kickoff → hold
├── Sequence(PenaltyAvoid)                 # Opponent restart → avoid
├── Sequence(AsChaser)                     # IsRole("chaser") → kick/approach
├── Sequence(AsSupporter)                  # IsRole("supporter") → support
├── Sequence(AsGoalkeeper)                 # IsRole("goalkeeper") → guard
├── Sequence(AsDefender)                   # IsRole("defender") — extension slot
└── WaitForBall(N)                         # Fallback Stop
```

### Key Design Invariants (Red Lines)

- **AR-01**: SafetyOverrides runs BEFORE CommitTeamCommands — never bypass.
- **AR-02**: DataLayer runs every frame — blackboard is the sole data source.
- **AR-03**: CommitTeamCommands is the only handshake point to executor.
- **AR-04**: `play/` MUST NOT import `py_trees` — strategy stays framework-agnostic.
- **AR-08/09**: SafetyGuards and SafetyOverrides guards must not be bypassed.
- **AR-10**: `set_velocity` and `SoccerKickManager` are mutually exclusive per robot per frame.

---

## Coordinate System

### Team-View Field

- Own goal at `x = -field_length/2 = -7.0`
- Opponent goal at `x = +field_length/2 = +7.0`
- `+x` = attacking direction, `-x` = defensive direction
- Simulation ground truth is already team-relative. **Do NOT mirror by `team_id`** inside `play/` or `tactics/`.

### Field Dimensions (ADULT_FIELD_DIMENSIONS)

| Parameter | Value |
|-----------|-------|
| Length | 14.0 m |
| Width | 9.0 m |
| Goal width | 2.6 m |
| Goal depth | 0.6 m |
| Penalty distance | 2.1 m |
| Center circle radius | 1.5 m |
| Penalty area (L×W) | 3.0 × 6.0 m |
| Goal area (L×W) | 1.0 × 4.0 m |

### ReadySlot Mapping

| Slot | Default player_id | Role |
|------|------------------|------|
| CENTER | 1 | Kickoff taker |
| SIDE | 2 | Side support |
| KEEPER | 3 | Goalkeeper |

Player IDs from `SOCCER_ROBOT_NAMES` env (first=1, second=2, third=3). Never hardcode player_id or team_id.

---

## Role System

### Default Roles (registered in priority order)

| Role | Class | Key Params | Behavior |
|------|-------|-----------|----------|
| `chaser` | ChaserRole | speed=2.0, kick=2.5, approach_offset=0.15m | Chase ball; shoot/pass/dribble decision |
| `supporter` | SupporterRole | speed=2.0, strafe=True, distance 1.0–2.2m | Dynamic support with spacing repulsion |
| `goalkeeper` | GoalkeeperRole | speed=2.2, lateral=1.0, strafe=True | 3-state guard/rush/lateral |
| `defender` | DefenderRole | hold_vyaw=0.12, reuses support target | Extension slot |

### select_chaser Algorithm (evaluated every frame)

1. **Slot eligibility**: KEEPER only in danger zone; SIDE only when `side_should_challenge()`.
2. **Cost scoring**: `ball_claim_score()` — distance-based with slot bias + opponent interference penalty.
3. **Tie-breaking**: Within 0.15m bandwidth → lower player_id wins.
4. **Dynamic lock**: Prevents ping-pong (3.0s/2.0s/0.5s tiers by zone).

---

## Kickoff State Machine

4-phase controller on `/play/kickoff_phase` blackboard:

| Phase | State | Behavior |
|-------|-------|----------|
| 0 | NormalPlay | `DefaultPlaybook.assign_roles()` runs normally |
| 1 | ActiveKickoff | Our kickoff: CENTER=Kicker, SIDE=Chaser(to landing point) |
| 2 | RoleLockPlay | Ball moved >0.15m + 0.5s → fixed roles (center=supporter, side=chaser) |
| 3 | OppKickoffDefense | Opponent kickoff active → approach ball with reduced speed (CENTER 0.5x) |

Transitions: 0→1 (our kickoff), 1→2 (ball moved), 2→0 (teammate near ball), any→3 (opponent kickoff), 3→0 (teammate touches ball or timeout).

---

## Goalkeeper System

### 3-State Machine

1. **GUARD** (default): Arc-based positioning — intersection of ball→goal-center line with arc at depth=1.3m, radius dynamically scaled by ball distance.
2. **RUSH_OUT**: Ball predicted to rest near goal line (`gk_rush_out_max_dist_m=1.5`) → two-stage approach:
   - **Stage far** (>0.5m): run directly at predicted rest point
   - **Stage near** (≤0.5m): slide behind ball, face clearance
   - Deferred if an outfielder is closer (`_gk_teammate_is_closer`)
3. **LATERAL**: Ball predicted to cross goal line inside posts → slide along goal depth.

### Ball Prediction

- Exponential decay model: `v(t) = v0 × exp(-mu × t)`
- Adaptive friction: fast rate (0.3) first 30 frames, stable (0.1) after
- Max horizon: 2.0s, goal-crossing search: 6.0s

---

## MotionController — 3-Layer Reactive Control

No global path planning. Computes velocity every frame from current obstacle layout.

### Layer 1: Path Avoidance (`_avoidance_target`)
- Find first blocking obstacle on target direction
- Generate lateral via-point with persistent side memory

### Layer 2: Walk Control (`_compute_velocity`)
- Arrival: distance<0.15m + angle<0.20rad → hold or stop
- Strafe mode: `vx=v×cos(err)`, `vy=v×sin(err)`, per-axis clamped, heading-priority scaling via `align_factor`
- Non-strafe: `angle_error>0.5rad → pure rotation`

### Layer 3: Yaw Avoidance (`_apply_yaw_avoidance`)
- PLAY stage: teammates only (NOT opponents)
- Each neighbor contributes vyaw bias, scaled by distance within horizon

### Phase Transition Smoothing (Issue 2.3)
When READY→PLAYING fires, targets are linearly interpolated from the last READY pose to the PLAYING target over 0.3s (9 frames) to prevent sharp direction changes.

---

## Key Tuning Parameters

All in `SoccerStrategyTuning` (`src/soccer_framework/config.py`).

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `max_linear_speed` | 0.8 | Linear speed limit (m/s) |
| `max_lateral_speed` | 0.8 | Strafe lateral limit (m/s) |
| `max_angular_speed` | 1.0 | Angular speed limit (rad/s) |
| `soccer_kick_enter_distance` | 2.0 | Enter kick mode (m) |
| `soccer_kick_exit_distance` | 2.5 | Exit kick mode (m) |
| `opponent_obstacle_radius` | 0.55 | Opponent obstacle radius (m) |
| `teammate_obstacle_radius` | 0.48 | Teammate obstacle radius (m) |
| `pass_min_score` | 0.60 | Minimum pass score to attempt |
| `pass_lane_clearance` | 0.75 | Pass lane clearance (m) |
| `dribble_advance_m` | 1.5 | Dribble forward distance (m) |
| `shoot_min_ball_x_m` | 0.0 | Min ball x for shooting |
| `shoot_max_distance_m` | 7.0 | Max shooting distance (m) |
| `support_min_distance_m` | 1.0 | Min supporter→chaser distance (m) |
| `support_max_distance_m` | 2.2 | Max supporter→chaser distance (m) |
| `gk_rush_out_max_dist_m` | 1.5 | Max GK rush distance from goal (m) |
| `gk_state_confirm_frames` | 2 | Frames to confirm state entry |
| `gk_state_release_frames` | 4 | Frames to confirm state exit |
| `ball_fresh_sec` | 0.3 | Raw ball trust window (s) |
| `ball_stale_grace_sec` | 0.3 | LKG extrapolation grace (s) |

---

## Where to Start for Common Changes

| Goal | File |
|------|------|
| Role assignment (all-out attack, custom logic) | `play/playbook.py:DefaultPlaybook.assign_roles()` |
| Chaser kick/pass/dribble target | `play/default_roles.py:ChaserRole.kick_target()` |
| Shooting lane scoring / GK-aware aim | `tactics/targeting/attack.py` |
| Supporter positioning algorithm | `play/default_roles.py:SupporterRole.target()` |
| Goalkeeper guard formula | `tactics/ready_stance.py:goalkeeper_guard_target()` |
| Add new role (interceptor, second striker) | Derive `RoleStrategy`, call `register_role()` |
| Register new Playbook | `PLAYBOOKS.register(name, factory)` in `play/registry.py` |
| Ball chasing score / who chases | `play/playbook.py:DefaultPlaybook.select_chaser()` |
| Obstacle avoidance / yaw avoidance | `tactics/motion.py` / `navigation.py` |
| Tune velocities, kick distances, margins | `soccer_framework/config.py:SoccerStrategyTuning` |
| Sideline recovery / restart avoidance | `tactics/targeting/recovery.py` / `restart.py` |
| Support positioning algorithm | `tactics/targeting/support.py` |
| READY/Safety subtree shape | `behavior_tree/ready_subtree.py` / `safety_subtree.py` |
| Full-team stop when data missing | `behavior_tree/safety_subtree.py:SafetyGuards` |
| Ball stale → LKG grace window | `soccer_framework/ball_lkg.py` / `pose_lkg.py` |

---

## Extending the Agent

### Custom Playbook

```python
from src.play import DefaultPlaybook, Playbook, RoleAssignment, PlayContext

class AggressivePlaybook(DefaultPlaybook):
    def assign_roles(self, context: PlayContext):
        base = super().assign_roles(context)
        mapping = dict(base.by_player)
        # Pull goalkeeper into attack when losing
        goalkeeper = next((pid for pid, r in mapping.items() if r == "goalkeeper"), None)
        if goalkeeper is not None:
            mapping[goalkeeper] = "supporter"
        return RoleAssignment(mapping)

from src.play import PLAYBOOKS
PLAYBOOKS.register("aggressive", AggressivePlaybook)

# Usage: PLAYBOOKS.create("aggressive", kit)
```

### Custom Role

```python
from src.play import RoleStrategy, MoveToTarget
from src.soccer_framework import Pose2D

class InterceptorRole(RoleStrategy):
    name = "interceptor"

    def target(self, kit, player_id, context):
        ball = context.known_ball
        return Pose2D(ball.x * 0.5, 0.0, 0.0)

    def build_subtree(self, kit, player_id):
        return MoveToTarget(kit, player_id,
            lambda ctx: self.target(kit, player_id, ctx),
            reason_fn=lambda: "interceptor hold")
```

---

## Development Notes

### Testing

- Unit tests cover pure functions in `tactics/` (no BT/ROS dependency).
- Integration tests require Booster Studio.
- Run offline syntax checks: `python -c "import ast; ast.parse(open('src/tactics/motion.py').read())"`

### Logging

- Structured JSONL logging via `SoccerLogger` in `soccer_framework/telemetry.py`.
- Performance zones logged every 2s: danger / defensive / midfield / attack.

### LKG (Last-Known-Good) Buffers

- **Ball**: `BallLkgBuffer` — constant-velocity extrapolation, 0.6s total tolerance (fresh 0.3 + stale 0.3).
- **Pose**: `PoseLkgBuffer` — per-team instances, 0.5s total tolerance, theta-unwrap extrapolation.
- **GameState**: No LKG — intentionally conservative (2.0s stale→None → StopAll).

### Git Hooks

- Pre-commit hooks recommended for syntax checking.
- Do not commit secrets or API keys.
