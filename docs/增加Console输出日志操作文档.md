# 增加 Console 输出日志 - 操作文档

## 概述

本项目的日志系统通过 `SoccerLogger`（`src/soccer_framework/telemetry.py:205`）实现，支持 **Console 输出**（标准输出/终端）和 **结构化日志文件**（JSONL）两条路径。本文档说明如何增加 Console 输出，推荐用于本地调试场景。

---

## 一、日志系统架构

```
┌────────────────────────────────────────────────┐
│              调用方 (任意模块)                     │
│  logger.info("msg", console=True, event="xxx") │
└────────────────────┬───────────────────────────┘
                     │
                     ▼
           ┌─────────────────┐
           │  SoccerLogger   │
           │ (telemetry.py)  │
           └────────┬────────┘
                    │
          ┌────────┴────────┐
          ▼                  ▼
   ┌────────────┐   ┌──────────────┐
   │ Console    │   │ Structured   │
   │ (终端输出)   │   │ Plugin       │
   │            │   │ (JSONL 文件)  │
   └────────────┘   └──────────────┘
```

- **Console**：调用 `AgentBase.logger`（底层为 `print`），默认 `console=True`
- **Structured**：写入 JSONL 文件，由环境变量 `SOCCER_LOG` 控制（默认开启）

---

## 二、基础用法（推荐）

### 2.1 对象获取方式

`SoccerLogger` 实例可通过以下方式获得：

| 上下文 | 获取方式 |
|---|---|
| `main.py` (`SoccerSimAgent`) | `self.soccer_logger`（在 `__init__` 中通过 `create_soccer_logger` 创建） |
| `runtime.py` (`SoccerTeamRuntime`) | `self._logger`（由 `main.py` 传入） |
| `SoccerKit` | `kit.logger`（由 `SoccerTeamRuntime` 构造时传入） |
| BT 叶子节点 / Playbook | `self._playbook.kit.logger` 或 `self._kit.logger` |

### 2.2 基本日志调用

```python
logger.info("str", console=True, event="event_name")
logger.warn("str", console=True, event="event_name")
logger.error("str", console=True, event="event_name")
logger.debug("str", console=True, event="event_name")
```

- `message`（第一个参数）：必填，控制台显示的字符串
- `console`（可选，默认 `True`）：是否输出到终端
- `event`（可选）：结构化事件名称，为 `None` 时不写入 JSONL
- `**fields`（可选）：以关键字参数传入结构化字段

### 2.3 关键默认行为

```python
def info(self, message, *, event=None, console=True, **fields):
```

在 `SoccerLogger._log()`（`telemetry.py:298`）中：

```python
def _log(self, level, message, *, event, console, **fields):
    if console:               # console=True → 输出到终端
        self._console(level, message)
    if event is not None:      # event 不为 None → 写入结构化日志
        self._record(event, level, message, **fields)
```

**规律**：`console=False` 且 `event="xxx"` → 只写结构化日志，不输出终端。`console=True` 且 `event=None` → 只输出终端，不写结构化日志。

---

## 三、高频率调试日志

### 3.1 `debug_console` 开关（推荐）

项目预定义了 `SoccerDebugConfig.debug_console: bool = True`（`config.py:54`），用于控制高频调试日志的开关。使用模式如下：

```python
if self._kit.logger is not None and self._kit.config.debug.debug_console:
    self._kit.logger.debug("your debug message")
```

**关闭方式**：构造 `SoccerConfig` 时传入 `SoccerDebugConfig(debug_console=False)`。

**适用场景**：`play_subtree.py`、BT 叶子节点、Playbook 节点中的高频调试信息（如开球阶段转换、条件判断状态等）。

### 3.2 现有示例

`src/play/play_subtree.py:299`：

```python
if self._kit.logger is not None and self._kit.config.debug.debug_console:
    self._kit.logger.debug(
        f"Phase 3: opponent kickoff ended, entering defense",
    )
```

`src/behavior_tree/nodes/conditions.py:228`：

```python
if self._kit.logger is not None and self._kit.config.debug.debug_console:
    self._kit.logger.debug("using custom ball")
```

---

## 四、控制台静音控制

### 4.1 按调用静音

传 `console=False` 使单次调用不输出终端，仅写结构化日志：

```python
# 只写结构化文件，不输出终端
self._logger.info(
    "SoccerTeamRuntime stopped",
    event="runtime_stopped",
    console=False,
)
```

适用于 `runtime.py` 中的控制摘要、配置日志等高频结构化日志。

### 4.2 常见场景对照

| 场景 | `console` | `event` | 效果 |
|---|---|---|---|
| 普通调试 | `True` | `"xxx"` | 终端 + 结构化 |
| 仅终端 | `True` | `None` | 仅终端 |
| 仅结构化 | `False` | `"xxx"` | 仅结构化文件 |
| 静默 | `False` | `None` | 都不输出 |

---

## 五、在 BT 叶子节点中添加 Console 日志

### 5.1 标准模式

```python
class YourNode(py_trees.behaviour.Behaviour):
    def __init__(self, kit, ...):
        self._kit = kit

    def update(self):
        # 关键判断日志
        logger = self._kit.logger
        if logger is not None and self._kit.config.debug.debug_console:
            logger.debug("state transition: A → B")
```

### 5.2 检查 `py_trees.blackboard` 日志

通过 `self._kit.config.debug.bt_trace_ticks` 设置 BT tick 跟踪级别。例如设置 `bt_trace_ticks="on"` 可在结构化日志中输出每次 tick 的状态变更。

---

## 六、`config.debug_console` 关闭方法

```python
from src.soccer_framework import SoccerConfig, SoccerDebugConfig

config = SoccerConfig(
    # ... 其他参数
    debug=SoccerDebugConfig(debug_console=False),
)
runtime = SoccerTeamRuntime(logger=logger, config=config)
```

---

## 七、总结对比

| 特性 | `console=True`（默认） | `console=False` | `debug_console` 保护块 |
|---|---|---|---|
| 输出到终端 | ✅ | ❌ | 受全局开关控制 |
| 写结构化日志 | 取决于 `event` | 取决于 `event` | 通常不传 event |
| 适用场景 | 关键事件、错误 | 高频结构化记录 | 调试阶段的高频诊断 |
| 性能开销 | 低 | 最低 | 可整体关闭 |

---

## 八、常见问题

**Q: 终端没有输出？**  
A: 检查调用时是否传了 `console=False`；检查 `debug_console` 是否为 `True`。

**Q: 结构化日志未写入？**  
A: 检查是否传了 `event` 参数；检查环境变量 `SOCCER_LOG` 是否被设为 `off`。

**Q: `logger` 为 `None`？**  
A: 确保对象通过 `SoccerTeamRuntime` 或 `SoccerKit` 传入了 `logger`；单元测试中需 mock 或传入假 logger。
