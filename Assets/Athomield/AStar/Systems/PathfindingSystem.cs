using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Athomield.AStar
{
    /// <summary>
    /// A* 寻路系统，负责并行执行所有待处理的寻路操作。
    ///
    /// 工作流程：
    /// 1. 获取当前网格节点数据快照
    /// 2. 遍历所有待处理的 ASPathfindingOperation Entity（由 PathfindingRequestSystem 创建）
    /// 3. 以 IJobParallelFor 并行方式对每个操作执行 A* 算法（ASRunParallelSearch Job）
    /// 4. 将规划出的路径点写入对应 Entity 的 ASPathBufferComponent 缓冲区
    /// 5. 写入 ASPathfindingResult（PathFound + FinishedSearch），通知 AgentMovementSystem
    ///
    /// 并行安全：每个操作有独立的 Entity，通过 NativeDisableContainerSafetyRestriction 标记
    /// 允许在并行 Job 中安全地读写不同 Entity 的组件
    /// </summary>
    partial struct PathfindingSystem : ISystem
    {
        private EntityTypeHandle EntityTypeHandle;

        // 操作数据（只读）
        [NativeDisableContainerSafetyRestriction, ReadOnly]
        public ComponentLookup<ASPathfindingOperation> SystemAllOperations;

        // 操作结果（读写，写入 PathFound/FinishedSearch）
        [NativeDisableContainerSafetyRestriction]
        public ComponentLookup<ASPathfindingResult> SystemAllResults;

        // 路径点缓冲区（读写，写入规划出的路径）
        [NativeDisableContainerSafetyRestriction]
        public BufferLookup<ASPathBufferComponent> SystemAllPathPoints;

        // 网格节点缓冲区（读写，Job 内会临时修改 G/H 代价）
        [NativeDisableContainerSafetyRestriction]
        public BufferLookup<ASGridNodesBufferComponent> SystemAllGridNodes;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            EntityTypeHandle = SystemAPI.GetEntityTypeHandle();
            SystemAllOperations = state.GetComponentLookup<ASPathfindingOperation>(true);
            SystemAllResults = state.GetComponentLookup<ASPathfindingResult>(true);
            SystemAllPathPoints = state.GetBufferLookup<ASPathBufferComponent>(true);
            SystemAllGridNodes = state.GetBufferLookup<ASGridNodesBufferComponent>();

            NativeArray<ComponentType> types = new NativeArray<ComponentType>(2, Allocator.Temp);
            types[0] = ComponentType.ReadOnly<ASPathfindingResult>();
            types[1] = ComponentType.ReadOnly<ASPathfindingOperation>();
            types.Dispose();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            SystemAllPathPoints.Update(ref state);
            SystemAllResults.Update(ref state);
            SystemAllPathPoints.Update(ref state);
            SystemAllGridNodes.Update(ref state);
            EntityTypeHandle.Update(ref state);

            // 查询所有同时拥有 ASPathfindingResult 和 ASPathfindingOperation 的 Entity（寻路操作 Entity）
            NativeArray<ComponentType> types = new NativeArray<ComponentType>(2, Allocator.Temp);
            types[0] = ComponentType.ReadOnly<ASPathfindingResult>();
            types[1] = ComponentType.ReadOnly<ASPathfindingOperation>();

            var query = state.GetEntityQuery(types);
            var chunks = query.ToArchetypeChunkArray(Allocator.Temp);

            // 遍历所有 Grid Entity（通常只有一个）
            foreach (var (grid, gridLocalTransform, gridEntity) in SystemAPI.Query<ASGrid, LocalTransform>().WithEntityAccess())
            {
                // 将网格节点从 DynamicBuffer 复制到 NativeArray（供 Burst Job 使用）
                NativeArray<ASGridNodesBufferComponent> nodesBufferComponents = SystemAllGridNodes[gridEntity].ToNativeArray(Allocator.Temp);
                NativeArray<ASNode> nodes = new NativeArray<ASNode>(nodesBufferComponents.Length, Allocator.TempJob);

                for (int i = 0; i < nodesBufferComponents.Length; i++)
                {
                    nodes[i] = nodesBufferComponents[i].Node;
                }

                // 对每个 Archetype Chunk 并行处理其中的所有寻路操作
                foreach (var archetypeChunk in chunks)
                {
                    // 调度并行 Job：每个操作 Entity 独立执行 A* 算法
                    JobHandle jh = Process(archetypeChunk, grid, gridLocalTransform, nodes, new JobHandle(), SystemAllGridNodes[gridEntity]);
                    jh.Complete(); // 等待完成（同帧完成寻路）
                }

                nodesBufferComponents.Dispose();
                nodes.Dispose();
            }

            types.Dispose();
            chunks.Dispose();
        }

        /// <summary>
        /// 为一个 Chunk 内的所有寻路操作调度并行 A* Job
        /// </summary>
        private JobHandle Process(in ArchetypeChunk _chunk, in ASGrid _grid, in LocalTransform _gridLocalTransform,
            NativeArray<ASNode> _nodes, JobHandle _inputDeps, DynamicBuffer<ASGridNodesBufferComponent> _originalNodes)
        {
            // 每个 Job 需要独立的节点数组副本（并行 Job 间不能共享写入）
            NativeArray<ASNode> newN = new NativeArray<ASNode>(_nodes.Length, Allocator.TempJob);
            NativeArray<ASNode>.Copy(_nodes, newN);

            ASRunParallelSearch searchJob = new ASRunParallelSearch()
            {
                OperationEntities = _chunk.GetNativeArray(EntityTypeHandle), // 本 Chunk 中的操作 Entity 列表
                JobAllOperations = SystemAllOperations,
                JobAllResults = SystemAllResults,
                JobAllPathPoints = SystemAllPathPoints,
                JobNodes = newN,
                JobGrid = _grid,
                JobGridLocalTransform = _gridLocalTransform,
            };

            // 调度并行 Job（每个操作分配到独立线程，批次大小 2）
            return searchJob.Schedule(_chunk.Count, 2, _inputDeps);
        }
    }

    /// <summary>
    /// 并行 A* 搜索 Job。
    /// 为 Chunk 内每个寻路操作 Entity 各启动一个 ASParallelSearch 执行 A* 算法。
    /// BurstCompile 标注使此 Job 被 Burst 编译为原生代码，性能大幅提升
    /// </summary>
    [BurstCompile(CompileSynchronously = true)]
    public struct ASRunParallelSearch : IJobParallelFor
    {
        // 本 Chunk 中的操作 Entity 列表（只读）
        [ReadOnly]
        public NativeArray<Entity> OperationEntities;

        // 各种组件/缓冲区查找（NativeDisableContainerSafetyRestriction 允许并行访问）
        [NativeDisableContainerSafetyRestriction, ReadOnly]
        public ComponentLookup<ASPathfindingOperation> JobAllOperations;

        [NativeDisableContainerSafetyRestriction]
        public ComponentLookup<ASPathfindingResult> JobAllResults;

        [NativeDisableContainerSafetyRestriction]
        public BufferLookup<ASPathBufferComponent> JobAllPathPoints;

        // 每个线程共享同一份节点数组副本（并行写入不同位置，安全）
        [NativeDisableParallelForRestriction, DeallocateOnJobCompletion]
        public NativeArray<ASNode> JobNodes;

        public ASGrid JobGrid;
        public LocalTransform JobGridLocalTransform;

        public void Execute(int index)
        {
            Entity e = OperationEntities[index];
            ASPathfindingResult res = JobAllResults[e];

            // 只处理尚未完成搜索的操作（避免重复计算）
            if (!res.FinishedSearch)
            {
                ASParallelSearch parallelSearch = new ASParallelSearch()
                {
                    OperationEntity = e,
                    AllOperations = JobAllOperations,
                    AllResults = JobAllResults,
                    AllPathPoints = JobAllPathPoints,
                    Nodes = JobNodes,
                    Grid = JobGrid,
                    GridLocalTransform = JobGridLocalTransform,
                };

                parallelSearch.Execute(); // 执行 A* 算法
            }
        }
    }

    /// <summary>
    /// 核心 A* 寻路算法实现。
    ///
    /// 算法流程：
    /// 1. 将起点和终点世界坐标转换为网格节点
    /// 2. 若终点不可通行，使用 BFS 寻找最近的可通行节点作为替代终点
    /// 3. 标准 A* 主循环：
    ///    a. 从 Open Set（最小堆）中取出 F 代价最小的节点
    ///    b. 若到达终点，回溯路径并返回
    ///    c. 遍历 8 方向邻居，计算 G 代价并更新 Open Set
    /// 4. 若未找到路径，返回最接近终点的节点的路径
    /// 5. 路径回溯：通过 PreviousNodeIndex 链从终点追溯到起点，反转后输出
    /// </summary>
    public struct ASParallelSearch
    {
        public Entity OperationEntity;

        public ComponentLookup<ASPathfindingOperation> AllOperations;
        public ComponentLookup<ASPathfindingResult> AllResults;
        public BufferLookup<ASPathBufferComponent> AllPathPoints;

        // A* Closed Set（哈希集合，O(1) 查找）
        [NativeDisableContainerSafetyRestriction, DeallocateOnJobCompletion]
        public NativeHashSet<ASNode> closedNodes;

        // A* Open Set（最小堆，按 F 代价排序）
        [NativeDisableContainerSafetyRestriction, DeallocateOnJobCompletion]
        public NativeMinHeap<ASNode> openNodes;

        public ASGrid Grid;
        public LocalTransform GridLocalTransform;

        // 工作节点数组（从主数组复制，A* 过程中修改 G/H/PreviousNodeIndex 值）
        [NativeDisableContainerSafetyRestriction, DeallocateOnJobCompletion] public NativeArray<ASNode> Nodes;
        [NativeDisableContainerSafetyRestriction, DeallocateOnJobCompletion] public NativeArray<ASNode> NewNodes;

        public void Execute()
        {
            // 复制节点数组（每次寻路使用独立副本，不污染原始数据）
            NewNodes = new NativeArray<ASNode>(Nodes.Length, Allocator.Temp);
            NativeArray<ASNode>.Copy(Nodes, NewNodes);

            ASPathfindingOperation operation = AllOperations[OperationEntity];
            ASPathfindingResult operationResult = AllResults[OperationEntity];
            DynamicBuffer<ASPathBufferComponent> pathBuffer = AllPathPoints[OperationEntity];

            operationResult.PathFound = false;
            operationResult.FinishedSearch = false;

            // 将起点/终点世界坐标转换为网格节点
            ASNode startNode = GridUtility.WorldPosToNode(operation.StartPoint, Grid, GridLocalTransform, NewNodes);
            ASNode targetNode = GridUtility.WorldPosToNode(operation.TargetPoint, Grid, GridLocalTransform, NewNodes);

            // 记录搜索过程中距目标最近的节点（用于无法到达时回退）
            ASNode closestNode = new ASNode();
            int closestNodeDistFromGoal = int.MaxValue;
            bool targetFound = false;

            // ===== 终点不可通行时：BFS 寻找最近可通行节点 =====
            if (!targetNode.IsAccessible)
            {
                ASNode newTargetNode = new ASNode();

                if (ExpandInaccessibleBFS(ref newTargetNode, targetNode, startNode, Grid, NewNodes))
                {
                    targetNode = newTargetNode; // 使用 BFS 找到的替代终点
                }
                else
                {
                    // 完全无法找到可通行的替代终点，终止搜索
                    operationResult.PathFound = false;
                    operationResult.FinishedSearch = true;
                    AllResults[OperationEntity] = operationResult;
                    return;
                }
            }

            // ===== 初始化 A* 数据结构 =====
            openNodes = new NativeMinHeap<ASNode>(Grid.NodeXCount * Grid.NodeYCount, Allocator.Temp) { };
            closedNodes = new NativeHashSet<ASNode>(0, Allocator.Temp);
            openNodes.Add(startNode); // 将起点加入 Open Set

            int d = Grid.NodeXCount > Grid.NodeYCount ? Grid.NodeXCount : Grid.NodeYCount;

            // ===== A* 主循环 =====
            while (openNodes.Length > 0)
            {
                // 取出 F 代价最小的节点（最小堆堆顶）
                ASNode currentNode = openNodes.PopFirstItem();
                closedNodes.Add(currentNode); // 加入 Closed Set

                // 到达终点：回溯路径并返回
                if (currentNode.X == targetNode.X && currentNode.Y == targetNode.Y)
                {
                    operationResult.PathFound = true;
                    operationResult.FinishedSearch = true;

                    RetracePath(startNode, currentNode, Grid, GridLocalTransform, NewNodes, pathBuffer, OperationEntity);
                    targetFound = true;
                    break;
                }

                // 记录距目标最近的节点（用于 GoToClosest 策略）
                int currNodeDistanceFromGoal = GridUtility.NodeDistance(currentNode, targetNode);
                if (currNodeDistanceFromGoal < closestNodeDistFromGoal)
                {
                    closestNodeDistFromGoal = currNodeDistanceFromGoal;
                    closestNode = currentNode;
                }

                // ===== 遍历 8 方向邻居 =====
                NativeArray<int2> neighboursOffsetIndecies = GridUtility.NodePossibleNeighbours(Allocator.Temp);

                foreach (int2 neighbourIndexOffset in neighboursOffsetIndecies)
                {
                    // 边界检查
                    if (neighbourIndexOffset.x + currentNode.X < 0 ||
                        neighbourIndexOffset.x + currentNode.X >= Grid.NodeXCount ||
                        neighbourIndexOffset.y + currentNode.Y < 0 ||
                        neighbourIndexOffset.y + currentNode.Y >= Grid.NodeYCount) continue;

                    // 计算邻居节点的一维索引
                    int neighbourIndex;
                    if (Grid.NodeXCount > Grid.NodeYCount)
                        neighbourIndex = neighbourIndexOffset.x + currentNode.X + ((neighbourIndexOffset.y + currentNode.Y) * d);
                    else neighbourIndex = neighbourIndexOffset.y + currentNode.Y + ((neighbourIndexOffset.x + currentNode.X) * d);

                    ASNode neighbour = NewNodes[neighbourIndex];

                    // 检查邻居是否在 Closed Set 中（已探索过）
                    bool neighbourInClosed = false;
                    foreach (ASNode closedNodeItem in closedNodes)
                    {
                        if (closedNodeItem.X == neighbour.X && closedNodeItem.Y == neighbour.Y)
                        {
                            neighbourInClosed = true;
                        }
                    }

                    // 跳过不可通行节点和已关闭节点
                    if (!neighbour.IsAccessible || neighbourInClosed)
                        continue;

                    // 计算经过当前节点到达邻居的代价（G = 父G + 移动代价 + 地形惩罚）
                    int movementCost = currentNode.GCost + GridUtility.NodeDistance(currentNode, neighbour) + neighbour.Penalty;

                    // 若新代价更低，或邻居尚未在 Open Set 中，更新邻居数据
                    if (movementCost < neighbour.GCost || !openNodes.Contains(neighbour))
                    {
                        neighbour.GCost = movementCost;
                        neighbour.HCost = GridUtility.NodeDistance(neighbour, targetNode);

                        // 记录父节点索引（路径回溯用）
                        int currentNodeIndex;
                        if (Grid.NodeXCount > Grid.NodeYCount)
                            currentNodeIndex = currentNode.X + currentNode.Y * d;
                        else currentNodeIndex = currentNode.Y + currentNode.X * d;

                        neighbour.PreviousNodeIndex = currentNodeIndex;

                        // 更新 Open Set 中已有的邻居节点（代价降低需要重新堆化）
                        for (int i = 0; i < openNodes.Length; i++)
                        {
                            if (openNodes.Items[i].Equals(neighbour))
                            {
                                int nodeHeapIndex = openNodes.Items[i].HeapIndex;
                                neighbour.HeapIndex = nodeHeapIndex;
                                openNodes.SetItemAt(neighbour, i);
                                break;
                            }
                        }

                        if (!openNodes.Contains(neighbour))
                        {
                            openNodes.Add(neighbour); // 新节点加入 Open Set
                        }
                        else
                        {
                            openNodes.UpdateItem(neighbour); // 已有节点更新优先级
                        }

                        NewNodes[neighbourIndex] = neighbour; // 更新工作节点数组
                    }
                }

                neighboursOffsetIndecies.Dispose();
            }

            // ===== 未找到路径：返回距目标最近的节点的路径 =====
            if (!targetFound)
            {
                operationResult.PathFound = true;
                operationResult.FinishedSearch = true;
                RetracePath(startNode, closestNode, Grid, GridLocalTransform, NewNodes, pathBuffer, OperationEntity);
            }

            AllResults[OperationEntity] = operationResult;
        }

        /// <summary>
        /// 路径回溯：从终点通过 PreviousNodeIndex 链追溯到起点，
        /// 然后反转顺序（起点→终点），将路径点世界坐标写入缓冲区
        /// </summary>
        void RetracePath(ASNode _startNode, ASNode _targetNode, ASGrid _grid, LocalTransform _gridLocalTransform,
            NativeArray<ASNode> _nodes, DynamicBuffer<ASPathBufferComponent> _pathPoints, Entity _agentEntity)
        {
            _pathPoints.Clear();

            ASNode pathCursor = _targetNode;
            NativeList<ASNode> pathNodes = new NativeList<ASNode>(Allocator.Temp);

            // 从终点向起点回溯（通过父节点链）
            while (!(pathCursor.X == _startNode.X && pathCursor.Y == _startNode.Y))
            {
                pathNodes.Add(pathCursor);
                pathCursor = _nodes[pathCursor.PreviousNodeIndex]; // 跳到父节点
            }

            // 反转路径（从起点到终点的正向顺序），转换为世界坐标
            for (int i = pathNodes.Length - 1; i >= 0; i--)
            {
                _pathPoints.Add(new ASPathBufferComponent()
                {
                    Point = GridUtility.GridCoordsToWorld(pathNodes[i].X, pathNodes[i].Y, _grid, _gridLocalTransform)
                });
            }

            // 最后追加精确的目标世界坐标（替代最近节点中心，减少最终误差）
            _pathPoints.Add(new ASPathBufferComponent() { Point = AllOperations[_agentEntity].TargetPoint });
        }

        /// <summary>
        /// BFS 扩展搜索：当终点节点不可通行时，从终点向外 BFS 寻找最近的可通行节点。
        /// 搜索范围受 Grid.TargetInaccessibleSearchTolerance 限制，
        /// 在所有找到的可通行节点中选择距起点最近的一个（避免绕远路）
        /// </summary>
        bool ExpandInaccessibleBFS(ref ASNode _closestNode, ASNode _targetNode, ASNode _startingNode,
            ASGrid _grid, NativeArray<ASNode> _nodes)
        {
            int d = Grid.NodeXCount > Grid.NodeYCount ? Grid.NodeXCount : Grid.NodeYCount;
            NativeArray<bool> visited = new NativeArray<bool>(_grid.NodeXCount * _grid.NodeYCount, Allocator.Temp);
            NativeQueue<(int, int)> nodeQueue = new NativeQueue<(int, int)>(Allocator.Temp);
            nodeQueue.Enqueue((_targetNode.X, _targetNode.Y));

            bool foundClosestTarget = false;
            int closestDistance = int.MaxValue;

            while (nodeQueue.Count > 0)
            {
                (int, int) currCorrds = nodeQueue.Dequeue();

                int currIndex;
                if (_grid.NodeXCount > _grid.NodeYCount)
                    currIndex = currCorrds.Item1 + currCorrds.Item2 * d;
                else currIndex = currCorrds.Item2 + currCorrds.Item1 * d;

                // 边界检查 + 搜索范围限制（TargetInaccessibleSearchTolerance * 10 个节点距离）
                if (currCorrds.Item1 < 0 || currCorrds.Item1 >= _grid.NodeXCount ||
                    currCorrds.Item2 < 0 || currCorrds.Item2 >= _grid.NodeYCount ||
                    GridUtility.NodeDistance(_targetNode, _nodes[currIndex]) > _grid.TargetInaccessibleSearchTolerance * 10) continue;

                if (visited[currIndex]) continue;
                
                visited[currIndex] = true;

                // 找到可通行节点：记录距起点最近的一个
                if (_nodes[currIndex].IsAccessible)
                {
                    foundClosestTarget = true;
                    int dist = GridUtility.NodeDistance(_startingNode, _nodes[currIndex]);
                    if (dist < closestDistance)
                    {
                        _closestNode = _nodes[currIndex];
                        closestDistance = dist;
                    }
                    continue; // 不继续扩展（可通行节点是终点）
                }

                

                // 向4方向扩展（只扩展不可通行节点内部，寻找边界上的可通行节点）
                nodeQueue.Enqueue((currCorrds.Item1 + 1, currCorrds.Item2));
                nodeQueue.Enqueue((currCorrds.Item1, currCorrds.Item2 + 1));
                nodeQueue.Enqueue((currCorrds.Item1 - 1, currCorrds.Item2));
                nodeQueue.Enqueue((currCorrds.Item1, currCorrds.Item2 - 1));
            }

            return foundClosestTarget;
        }
    }
}
