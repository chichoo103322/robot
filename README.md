# 人形机器人上层任务规划与通信系统

> Humanoid Robot — Upper Task Planning & Communication System

---

## 项目概述

本项目面向人形机器人自主控制系统开发，实现机器人动作任务的统一规划、调度、通信控制与状态反馈。系统采用三层模块化架构，支持仿真联调与实物验收。

**核心目标：**
- 上层任务规划系统（创建任务、管理队列、动作调度、动作切换）
- 通信控制系统（指令发送、状态接收、心跳检测、实时同步）
- 状态管理与控制界面（状态显示、日志管理、控制面板）
- 支持5+种动作控制：直线行走、原地掉头、转弯行走、后退、侧移、停止
- 视觉避障自主绕行

---

## 系统架构

```
┌─────────────────────────────────────────────────┐
│              控制界面 (成员B)                     │
│         ControlPanel + RobotDashboard           │
│         状态显示 · 日志 · 控制按钮 · 视觉画面      │
└──────────────────┬──────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────┐
│         上层任务规划系统 (成员A)                   │
│    TaskManager · ActionScheduler                │
│    MotionPlanner · ObstacleDetector             │
│    AvoidancePlanner                             │
│    任务创建 · 动作调度 · 路径规划 · 避障检测       │
└──────────────────┬──────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────┐
│           通信系统 (成员C)                        │
│    SocketClient · CommandSender                 │
│    HeartbeatManager · APIService                │
│    TCP指令发送 · 状态接收 · 心跳检测              │
└──────────────────┬──────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────┐
│            机器人底层控制器                        │
└─────────────────────────────────────────────────┘
```

---

## 项目结构

```
project4/
├── README.md                           # 本文件 — 项目总览与协作指南
├── requirements.txt                    # Python 依赖
├── docs/
│   ├── PROTOCOL.md                     # 通信协议文档（成员C必读）
│   ├── API.md                          # 模块接口规范（全员必读）
│   ├── HANDOFF_B.md                    # 成员B交付文档（接口定义+验收标准）
│   └── HANDOFF_C.md                    # 成员C交付文档（接口定义+验收标准）
├── src/
│   ├── main.py                         # 主入口，组装所有模块（headless模式）
│   ├── common/                         # 共享：接口、数据模型、枚举
│   │   ├── interfaces.py               #   抽象接口（各成员的契约）
│   │   ├── models.py                   #   数据模型（Action, Task, Command 等）
│   │   └── enums.py                    #   枚举（ActionType, RobotState 等）
│   ├── task_planner/                   # 成员A：任务规划与避障
│   │   ├── task_manager.py             #   任务生命周期管理
│   │   ├── action_scheduler.py         #   动作调度与优先级抢占
│   │   ├── motion_planner.py           #   动作参数生成（7种动作）
│   │   ├── obstacle_detector.py        #   障碍物检测（深度图+LIDAR融合）
│   │   ├── avoidance_planner.py        #   避障路径规划
│   │   ├── vision_detector.py          #   视觉检测（模拟/深度/YOLO三模式）
│   │   └── reactive_avoidance.py       #   实时避障监控闭环
│   ├── status_ui/                      # 成员B：状态管理与界面
│   │   ├── status_manager.py           #   中心状态存储
│   │   ├── log_system.py               #   结构化日志
│   │   ├── control_panel.py            #   控制面板逻辑
│   │   └── dashboard.py                #   UI仪表盘
│   └── communication/                  # 成员C：通信系统
│       ├── socket_client.py            #   TCP Socket 连接
│       ├── command_sender.py           #   可靠指令发送（队列+重试）
│       ├── heartbeat_manager.py        #   心跳检测与断线重连
│       └── api_service.py              #   统一通信API
├── tests/
│   ├── test_task_planner/              # 成员A测试
│   ├── test_status_ui/                 # 成员B测试
│   └── test_communication/             # 成员C测试
└── scripts/
    └── run_simulation.py               # 仿真运行脚本（Mock Robot Server）
```

---

## 快速开始

### 1. 环境准备

```bash
cd /Users/jzxzhou/code/project4
pip install -r requirements.txt
```

### 2. 运行仿真（无需实物机器人）

```bash
# 启动仿真服务器 + 客户端（headless 模式）
python scripts/run_simulation.py

# 仅启动服务器（供其他成员连接测试）
python scripts/run_simulation.py --server-only
```

### 3. 运行测试

```bash
pytest tests/ -v
```

---

## 人员分工

### 成员A — 任务规划、动作调度与视觉避障

**文件位置：** `src/task_planner/`

| 模块 | 文件 | 职责 |
|------|------|------|
| TaskManager | `task_manager.py` | 任务创建、队列管理、生命周期控制(启/停/暂停/恢复) |
| ActionScheduler | `action_scheduler.py` | 动作调度、优先级抢占、执行状态追踪 |
| MotionPlanner | `motion_planner.py` | 生成6种动作参数、组合动作序列 |
| ObstacleDetector | `obstacle_detector.py` | 深度图+LIDAR融合障碍物检测 |
| AvoidancePlanner | `avoidance_planner.py` | 避障路径规划(侧移绕行策略) |

**对外接口：** 实现 `ITaskManager`, `IActionScheduler`, `IMotionPlanner`（定义在 `src/common/interfaces.py`）

**依赖：**
- 通过 `ICommunication` 接口向成员C发送指令
- 通过成员C接收的 `SensorData` 进行避障检测
- 任务事件通过 `StatusManager.add_log()` 记录到成员B的日志系统

---

### 成员B — 状态管理、控制界面与视觉画面显示

**文件位置：** `src/status_ui/`

| 模块 | 文件 | 职责 |
|------|------|------|
| StatusManager | `status_manager.py` | 中心状态存储，线程安全，支持订阅推送 |
| LogSystem | `log_system.py` | 文件+控制台双通道日志，自动滚动 |
| ControlPanel | `control_panel.py` | 控制按钮逻辑(启动/停止/暂停/手动动作) |
| RobotDashboard | `dashboard.py` | PyQt6主界面：状态栏 + 控制区 + 视觉画面 + 日志 |

**对外接口：** 实现 `IStatusManager`（定义在 `src/common/interfaces.py`）

**依赖：**
- 成员C调用 `update_robot_status()` 写入实时状态
- 成员C调用 `add_log()` 写入通信日志
- UI通过 `get_robot_status()` / `get_logs()` 读取展示

---

### 成员C — 通信系统开发

**文件位置：** `src/communication/`

| 模块 | 文件 | 职责 |
|------|------|------|
| SocketClient | `socket_client.py` | TCP连接管理、自动重连、帧收发 |
| CommandSender | `command_sender.py` | 可靠指令发送（队列+重试+限速） |
| HeartbeatManager | `heartbeat_manager.py` | 心跳检测(1s间隔/3s超时/3次丢失断连) |
| APIService | `api_service.py` | 统一通信API，实现 `ICommunication` |

**对外接口：** 实现 `ICommunication`（定义在 `src/common/interfaces.py`）

**依赖：**
- 接收数据后调用 成员B 的 `StatusManager.update_robot_status()` 写入状态
- 接收数据后调用 成员A 的 `ObstacleDetector.detect()` 进行障碍检测
- 发送成员A生成的 `Command` 到机器人

---

## 协作约定

### 1. 所有模块间通信通过接口

不要直接 import 其他成员的具体实现类，通过 `src/common/interfaces.py` 中的抽象接口交互：

```python
# ✅ 正确：依赖接口
from src.common.interfaces import ICommunication
class ActionScheduler:
    def __init__(self, comm: ICommunication):
        self._comm = comm

# ❌ 错误：直接依赖具体实现
from src.communication.api_service import APIService
```

### 2. 数据传递使用共享模型

所有跨模块数据使用 `src/common/models.py` 中的 dataclass：

```python
# ✅ 正确
from src.common.models import Action, Task, Command, RobotStatus
task = Task(name="demo", actions=[...])

# ❌ 错误：使用自定义 dict
task = {"name": "demo", "actions": [...]}
```

### 3. 新增 ActionType 或 RobotState

在 `src/common/enums.py` 中添加枚举值，所有模块自动支持。

### 4. 日志规范

```python
# 成员B的LogSystem用法
log.info("task_planner", "Task started")
log.warning("communication", "Heartbeat timeout")
log.error("scheduler", "Action failed", exc_info=True)
```

### 5. Git 协作

```
main
├── feat/task-planner    # 成员A 开发分支
├── feat/status-ui       # 成员B 开发分支
└── feat/communication   # 成员C 开发分支
```

- 每个人的改动在自己的分支上进行
- 修改 `src/common/` 下的文件需要先在群内沟通
- 合并前确保 `pytest` 全部通过

---

## 通信协议

详见 [docs/PROTOCOL.md](docs/PROTOCOL.md)

**关键参数：**
- 传输层：TCP，默认端口 9090
- 帧格式：4字节大端长度前缀 + JSON Body
- 心跳：1秒间隔，3秒超时，连续3次丢失判定断连
- 消息类型：`command` / `status` / `sensor` / `heartbeat` / `action_complete` / `error`

---

## API 接口

详见 [docs/API.md](docs/API.md)

每个模块的公开方法、参数、返回值均有完整文档。

---

## 开发计划

| 阶段 | 内容 | 产出 |
|------|------|------|
| 1 | 需求分析与系统设计 | 系统设计文档、通信协议、数据结构 |
| 2 | 通信系统开发 | TCP连接、指令收发、心跳 |
| 3 | 任务规划系统开发 | 任务管理、动作调度、序列执行 |
| 4 | 状态管理与界面开发 | 状态显示、日志、控制面板 |
| 5 | 联调与测试 | 仿真联调、实物测试、5种动作验证 |

---

## 验收标准

- [ ] 5+ 种动作稳定执行（直线行走、原地掉头、转弯行走、后退、停止）
- [ ] 多动作连续执行（Task 内 actions 顺序执行）
- [ ] 视觉避障自主绕行
- [ ] 通信实时同步（延迟 < 100ms）
- [ ] 仿真验收通过
- [ ] 实物验收通过
