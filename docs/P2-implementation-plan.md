# P2 级优化 — 实施详细方案

基于 `docs/strategy-optimization-notes.md` P2 优先级（11-15），按 **依赖性由低到高** 排序实施。

---

## P2-13: 动态角色交换（attacker 卡死时换人）

- **文件**: `src/main.py` (+ `src/param.py`)
- **原理**:
  - 在 `store` 中记录当前 attacker 的 id、到球距离、连续停滞帧数
  - 若 attacker 在 `ATTACKER_STUCK_FRAMES_MAX` 帧内到球距离减少 ≤ `ATTACKER_STUCK_DIST_DELTA`，强制清除 `normal_attacker`，触发 `_select_closest_attacker` 换人
- **param.py 新增**:
  - `ATTACKER_STUCK_DIST_DELTA = 0.15` — 单帧最小有效接近距离
  - `ATTACKER_STUCK_FRAMES_MAX = 45` — 连续停滞帧数上限

---

## P2-12: 守门员出击（三模式）

- **文件**: `src/player.py` (+ `src/param.py`)
- **原理**:
  - 三模式切换：`HOME`（门线横向跟球） / `RUSH`（球逼近时前冲封角度） / `SWEEP`（球在对方半场时前提充当清道夫）
  - `guard()` 每次调用根据 `ball.x` 判定当前模式并设定不同目标点
- **param.py 新增**:
  - `GUARD_RUSH_X = -5.0` — 出击触发阈值（球 x > -5.0）
  - `GUARD_RUSH_MAX_RATIO = 0.6` — 出击距离占球→门线比例上限
  - `GUARD_SWEEP_X = -1.0` — 清道夫模式触发阈值（球 x < -1.0）
  - `GUARD_SWEEP_X_POS = -5.0` — 清道夫站位 x 坐标
- **player.py 改动**:
  - `guard()` 加入模式判定 if‑elif‑else 分支
---

## P2-15: READY 站位精细化

- **文件**: `src/main.py`
- **原理**:
  - 不再按 `players` 列表顺序分配，而是先遴选守门员（距己方门最近）
  - **我方开球**: 守门员 → 小禁区线；主罚手 → 中圈旁；支援手 → 中场边路
  - **对方开球**: 守门员 → 门线；拦截手 → 中圈外堵截；后卫 → 大禁区线补防

---

## P2-11: 带球变向过人

- **文件**: `src/player.py` (+ `src/param.py`)
- **原理**:
  - `attack()` 中 `_kicking` 分支：算直接射门方向后，检查附近对手是否挡路
  - 若被阻挡，以 `DRIBBLE_SCAN_STEP` 扫描备选方向，选最朝球门且前方净空 ≥ `DRIBBLE_CLEARANCE` 的方向
  - 用 `DRIBBLE_POWER` 低力度趟球过人
- **param.py 新增**:
  - `DRIBBLE_SCAN_RADIUS = 1.5` — 扫描对手半径
  - `DRIBBLE_SCAN_STEP = math.radians(15)` — 步进角
  - `DRIBBLE_POWER = 2.5` — 趟球力度
  - `DRIBBLE_CLEARANCE = 0.4` — 备选方向最小净空

---

## P2-14: Keep-away 控球拖延

- **文件**: `src/main.py` (+ `src/player.py` + `src/param.py`)
- **原理**:
  - `_should_keep_away()` 判断：领先足够（净胜球 ≥ 2）且比赛进入最后 30 秒
  - 带球队员不进攻，改走向己方半场角落安全区；被逼抢时传给最远的队友
  - 无球队员拉开空间接应
- **param.py 新增**:
  - `KEEP_AWAY_GOAL_DIFF = 2`
  - `KEEP_AWAY_TIME = 30.0`
  - `KEEP_AWAY_PRESSURE_DIST = 1.5`
  - `KEEP_AWAY_CORNER_MARGIN = 1.0`
- **main.py 改动**:
  - `init_store()` 加 `keep_away_active`
  - `play()` 中 NORMAL 阶段先检查 keep-away，跳转到专门处理函数
  - `player.py` 新增 `keep_away()` 方法
