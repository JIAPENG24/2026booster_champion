# AGENTS.md — 3v3 SoccerSim Agent

## Entrypoint & Packaging

- Entry: `src/main.py:SoccerSimAgent` (declared in `agent.toml`)
- Agent ID: `com.example.lqinternalmatch`, version `1.1.0`
- Scene: `football3v3`, mode: `soccer-match` (`.booster-studio/project.json`)
- Min API level: `10700`; models: `Booster T1 / K1`
- Deploy targets (`build.toml`): `sim_x86_64`, `sim_aarch64`, `real_jetson`

## Where to edit

| Directory / File | Purpose |
|---|---|
| `src/main.py` | **Strategy entry** — Phase state machine, play logic, role assignment |
| `src/player.py` | **Player control** — walk, kick, guard, support, attack |
| `src/param.py` | **Tunable parameters** — always tweak here before hardcoding |
| `src/utils/` | **Geometry / obstacles / path planning** — pure functions |
| `src/framework/` | **DO NOT EDIT** — platform pipeline (runtime, ROS source, types, backend) |

## Architecture rules

- All strategy logic lives in `main.py.play()` — runs 30Hz
- The Phase state machine (`Phase` enum) dispatches to `_act_*` functions
- `AgentBase` must be a **direct** base class for platform validation. Framework uses `SoccerAgentMixin` alongside `AgentBase` (see `src/framework/agent.py` for the pattern)
- Player class must be set as `player_class = Player` in the agent class
- `store` is a `SimpleNamespace` — use it for cross-frame state (phase, kickoff taker, etc.)
- `param.py` is imported via `from .param import *` — add new constants there

## Workflow

1. Open project in Booster Studio
2. Wait for environment preparation
3. Select a virtual robot
4. Click "Activate, build, deploy and run agent"
5. Debug via ROS topics at `/soccer/debug` (MarkerArray), `/soccer/agent_log` (Log)

## Dependency

- `py_trees==2.4.0` (`build.toml`); `booster_agent_framework` and `boosteros` are platform-provided at runtime
- No test, lint, typecheck, or formatter config in this repo

## Field geometry (3v3)

- Field: 14m × 9m, center at (0,0); +X = opponent goal
- Robots: 3 per team (robot1–robot3 for team 1, robot4–robot6 for team 2)

## Key gotchas

- `python-obfuscation = false` in `build.toml` — set to `true` for competition
- Pip repos can be overridden in `build.toml` (currently commented out Tsinghua mirror)
- Keystore signing via `AGENT_SIGN_KEYSTORE` / `AGENT_SIGN_KEYSTORE_PASSWD` env vars (commented out in `build.toml`)
- Debug visualization uses `debugdraw.*` API (no-op on non-ROS machines)
