# Unity DOTS/ECS 多代理寻路系统

基于 Unity DOTS 的网格寻路与多代理局部避障学习项目，展示从网格数据准备、请求调度、并行 A* 到 KD-Tree 邻居查询、ORCA 速度求解的完整代码流程。

这是用于项目展示和代码阅读的源码版本，包含核心代码、示例场景、预制体、Unity 配置和包版本。演示使用的外部角色模型及动画未包含在仓库中，完整运行前需要补齐资源或使用自有模型替换。

## 环境

| 项目 | 版本 |
| --- | --- |
| Unity Editor | 2023.2.20f1c1 |
| Entities / Unity Physics | 1.3.8 / 1.3.8 |
| Entities Graphics | 1.4.5 |
| Collections / Burst | 2.5.1 / 1.8.18 |
| URP | 16.0.6 |

依赖的完整版本见 [Packages/manifest.json](Packages/manifest.json) 与 [packages-lock.json](Packages/packages-lock.json)。

## 主要实现

- **网格预处理**：Unity Physics OverlapBox 检测障碍物与惩罚区域；横纵可分离箱式滤波结合滑动窗口平滑惩罚值；BFS 洪水填充标记连通区域。
- **请求调度与 A***：请求队列限制每帧提交数量；独立寻路 Entity 保存起终点、结果及路径 Buffer；IJobParallelFor 与 Burst 执行寻路，NativeMinHeap 维护 Open Set，支持替代终点和 GoToClosest 回退。
- **KD-Tree 构建与 KNN 查询**：最长轴与滑动中点划分，双指针重排点索引；最小堆按节点 AABB 距离下界访问，最大堆维护 K 个候选并用于剪枝。
- **ORCA 避障**：根据邻居快照构造 ORCA Line；在最大速度圆内增量求解最接近期望速度的速度；约束冲突时构造 ProjectedLine，降低最大违反量。
- **移动应用**：根据路径目标节点生成期望速度，统一计算所有代理的速度后，再进行前向射线检测并更新位置和朝向。

## 系统流程

```mermaid
flowchart TD
    Grid[网格配置与烘焙] --> Obstacles[障碍检测与惩罚平滑]
    Obstacles --> Islands[BFS 连通区域标记]
    Islands --> Request[PathfindingRequestSystem 请求排队]
    Request --> Search[PathfindingSystem 并行 A*]
    Search --> Path[路径与结果写入请求 Entity]
    Path --> Movement[AgentMovementSystem 消费路径]
    Movement --> Neighbours[KD-Tree 查询邻居]
    Neighbours --> ORCA[ORCA 求解速度]
    ORCA --> Apply[射线检测与统一移动]
    Apply -->|目标变化| Request
```

## 代码阅读入口

| 模块 | 入口 |
| --- | --- |
| 网格障碍与滤波 | [CheckObstaclesSystem.cs](Assets/Athomield/AStar/Systems/CheckObstaclesSystem.cs) |
| 岛屿标记 | [CheckNodeIslandSystem.cs](Assets/Athomield/AStar/Systems/CheckNodeIslandSystem.cs) |
| 请求排队 | [PathfindingRequestSystem.cs](Assets/Athomield/AStar/Systems/PathfindingRequestSystem.cs) |
| 并行寻路 | [PathfindingSystem.cs](Assets/Athomield/AStar/Systems/PathfindingSystem.cs) |
| A* 与路径回溯 | [PathfindingUtility.cs](Assets/Athomield/AStar/Utility/PathfindingUtility.cs) |
| 最小堆 | [NativeMinHeap.cs](Assets/Athomield/AStar/Utility/NativeMinHeap.cs) |
| KD-Tree / KNN | [KnnContainer.cs](Assets/Athomield/AStar/Utility/KNN/KnnContainer.cs) |
| ORCA 与代理移动 | [AgentMovementSystem.cs](Assets/Athomield/AStar/Systems/AgentMovementSystem.cs) |

## 打开项目

1. 克隆仓库，通过 Unity Hub 打开项目根目录，使用上表版本的编辑器。
2. 等待 Package Manager 恢复依赖，并允许 SubScene 完成导入与烘焙。
3. 阅读 [资源说明](DEPENDENCIES.md)，补齐或替换代理模型；缺少模型时相关场景引用可能显示 Missing。
4. 打开 `Assets/Athomield/AStar/Demos/GroupTarget/GroupTarget.unity`。其目标球由方向键控制，代理持续跟踪移动目标。
5. 也可以查看 `Assets/Athomield/AStar/Demos/CircleOfAgents/COA.unity` 的场景配置与对应 SubScene。

邻居数量、最大速度、障碍配置会影响避障效果；约束冲突时得到的是折中速度，不代表所有约束都能同时满足。仓库不提供未经测量的帧率或零分配保证。本次发布核对了文件与依赖清单，未在清洁环境中完成 Unity 导入和运行测试。

## 来源

本地初始工程的 Git 来源为 [suanDumplings/AStar](https://github.com/suanDumplings/AStar)，本仓库整理当前学习与迭代版本，并保留 `Athomield.AStar` 命名空间。原有代码、算法参考与第三方依赖的权利归相应作者；本仓库未额外授予统一开源许可。
