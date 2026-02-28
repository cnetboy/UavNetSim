# UavNetSim 项目中文文档

本文件为本仓库的中文导读与模块说明，旨在帮助你快速理解代码组织、关键类/函数的职责，以及如何运行和扩展项目。

> 注：本仓库同时保留自动生成的英文逐函数/类文档：`docs/UavNetSim_documentation.md`（如需逐函数细读可参考该文件）。


## 一、项目概览

UavNetSim-v1 是一个基于 Python 的无人机通信网络仿真平台，采用 SimPy 作为离散事件驱动基础，包含网络层、MAC 层、物理层、无人机移动与能量模型，并提供可插拔的路由、MAC 与拓扑控制算法。

主要用途：验证路由协议、MAC 协议、拓扑/轨迹控制与基于强化学习的策略，并提供可视化支持以便分析仿真行为与指标（PDR、延迟、吞吐率等）。


## 二、运行要求

- Python 3.10（建议）
- 依赖见 `requirements.txt`：matplotlib、numpy、openpyxl、Pillow、scikit_opt、simpy 等

快速运行：

```powershell
cd C:\uavNetSim
# 创建并激活 venv（假设 py -3.10 可用）
py -3.10 -m venv .venv
Set-ExecutionPolicy -Scope Process -ExecutionPolicy RemoteSigned
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
python main.py
```

入口文件：`main.py`（项目根）。


## 三、项目结构（中文说明）

- `main.py`：程序入口，创建 `simpy.Environment`，实例化 `Simulator` 并触发可视化。
- `utils/config.py`：全局仿真参数（地图尺寸、节点数、PHY/MAC 参数、仿真时间等）。
- `simulator/`：仿真控制核心。
  - `simulator/simulator.py`：`Simulator` 类，负责创建无人机、channel 与 metrics，调度全局流程。
  - `simulator/metrics.py`：记录 PDR、平均时延、吞吐等指标并打印。
  - `simulator/log.py`：日志配置（供仿真调试）。
- `entities/`：实体定义。
  - `entities/drone.py`：`Drone` 类——节点行为核心（生成数据包、队列调度、调用路由与 MAC、接收与 SINR 判定）。
  - `entities/packet.py`：定义 `Packet`、`DataPacket`、`AckPacket`、以及 VF / Hello 等控制包类型。
  - `entities/obstacle.py`：障碍物（球形/立方体）定义，用于路径规划/可视化。
- `routing/`：路由协议集合（每个协议目录包含实现与帮助类/表）：
  - `dsdv/`：DSDV 实现（路由表与 hello 消息）
  - `grad/`：GRAd（Gradient Routing）
  - `greedy/`：贪心地理路由
  - `opar/`：OPAR 算法
  - `q_routing/`, `qfanet/`, `qgeo/`, `qmr/`：各种基于 Q-learning 的路由变体
- `mac/`：MAC 层实现（`csma_ca.py`、`pure_aloha.py`、`tdma.py`）。
- `phy/`：物理层与信道模型（`channel.py`, `large_scale_fading.py`, `phy.py`）。
- `mobility/`：无人机移动模型（Gauss-Markov、Random Walk、Random Waypoint、起始位置生成）。
- `path_planning/`：路径规划（A*、路径跟随器）。
- `allocation/`：资源/子信道分配（集中式控制示例）。
- `topology/`：拓扑控制（比如虚拟力实现）。
- `visualization/`：静态与交互式绘图工具（展示初始分布、轨迹、转发表）。
- `tools/`：辅助脚本（例如自动生成文档的 `generate_repo_doc.py`）。


## 四、关键模块与核心类/函数说明（中文摘要）

下面列出项目中最常接触的模块与类，并说明其职责与调用关系（摘要版，逐函数/类细节请参考 `docs/UavNetSim_documentation.md`）。

### 1) 仿真控制 —— `Simulator`
- 文件：`simulator/simulator.py`
- 主要类：`Simulator`
- 作用：创建仿真环境（`simpy.Environment`）、初始化 `Channel`、生成无人机 `Drone` 列表、启动性能显示与绘图进程。
- 交互：向每个 `Drone` 传入 `simulator` 引用；`Simulator` 使用 `self.channel` 提供广播/单播信道接口；`Simulator.metrics` 记录全局性能指标。

### 2) 节点行为 —— `Drone`
- 文件：`entities/drone.py`
- 主要类：`Drone`
- 作用：实现节点生命周期与行为，包括：
  - 数据包生成（`generate_data_packet`，默认 Poisson 流量）
  - 发送队列调度（`feed_packet`），向路由模块请求下一跳并进入 `buffer`（`simpy.Resource`）等待
  - 发送由 MAC 层 (`mac/*.py`) 的 `mac_send` 实现
  - 接收与 SINR 判定（`receive`，调用 `phy.large_scale_fading.sinr_calculator`）
  - 路由相关：当收到控制包（hello/grad 等）时更新路由表或邻居表
- 关键数据结构：`buffer`（simpy Resource, capacity=1）、`transmitting_queue`、`waiting_list`、`inbox`（由 `Channel` 提供用于 SINR 计算）

### 3) 路由协议（统一接口）
- 每个协议（如 `Dsdv`, `Greedy`, `Grad`, `QRouting`, `OPAR` 等）提供主要方法：
  - `next_hop_selection(packet)`：为一个数据包返回是否有可用下一跳、最终发送包（有时是控制包）以及是否需要询问（enquire）
  - `packet_reception(packet, src_id)`：接收包时的处理逻辑
  - 周期性任务（例如广播 hello）
- 设计要点：`Drone` 不直接实现路由逻辑，而是通过协议对象调用统一接口，因此可插拔。

### 4) MAC 层
- 文件：`mac/csma_ca.py`, `mac/pure_aloha.py`, `mac/tdma.py`
- 职责：实现载波侦听、退避、ACK 等细节，控制何时调用 `phy` 的单播/广播函数以把包写入 `Channel` 的 inbox。
- 关键方法：`mac_send(pkd)`（发包主入口），`wait_ack` 等。

### 5) 物理层与信道模型
- 文件：`phy/channel.py`、`phy/large_scale_fading.py`、`phy/phy.py`
- `Channel`：为每个接收者维护 `inbox`（列表），提供 `broadcast_put` / `unicast_put` / `multicast_put` 方法将包写入接收者 inbox（含插入时间与发送方信息）。
- `large_scale_fading.sinr_calculator`：给定候选接收器与所有干扰发射者，计算 SINR 并返回各候选包的 SINR 列表以判定哪个包被成功接收。

### 6) 可视化
- `visualization/visualizer.py`：`SimulationVisualizer` 用于周期性收集场景并调用 `static_drawing` 做绘图。可设置帧间隔与输出目录。


## 五、如何基于本工程改进（可执行建议）

下面列出若干优先级较高的改进点，便于你后续按需求实现：

1. 配置化与可复现实验
   - 将 `utils/config.py` 支持 YAML/JSON 加载，记录每次实验配置为独立文件以便复现。
2. 文档与测试
   - 为每个路由/MAC/PHY 模块补充单元测试与小规模集成测试（pytest）。
   - 扩充 docs，生成每个模块的单独章节与调用时序图（Graphviz）。
3. 性能优化
   - 减少 Python 层循环，使用 numpy 向量化（尤其在 SINR 计算、路径损耗计算处）。
   - 在必要处使用 numba 加速。
4. 可视化改进
   - 提供非阻塞渲染与视频输出（ffmpeg），便于批量实验记录
5. 可插拔接口与类型化
   - 为路由与 MAC 抽象基类并添加类型注释（mypy），让替换实现更安全。


## 六、文档位置与后续操作

- 英文逐函数/类文档（自动生成）：`docs/UavNetSim_documentation.md`
- 我已生成的中文概要文档（当前文件）：`docs/UavNetSim_documentation_zh.md`
- 若你需要，我可以继续：
  - 把英文逐函数文档翻译为中文（完整逐函数翻译，文件较大），
  - 或为指定模块（例如 `entities/drone.py` 或某个路由协议）生成逐函数逐行的中文注释版文件，便于逐步重构与单元测试。


---

如需我把英文的逐函数文档自动翻译为完整中文版本并保存为 `docs/UavNetSim_documentation_zh_full.md`（包含每个函数/类的中文翻译），请回复“完整翻译”，我将开始该操作（会生成较大的文件）。
