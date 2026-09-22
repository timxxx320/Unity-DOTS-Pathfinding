using Unity.Burst;
using Unity.Entities;
using Unity.Transforms;
using Unity.Collections;

namespace Athomield.AStar
{
    /// <summary>
    /// 寻路请求调度系统，负责收集代理的寻路请求并以受控频率提交给 PathfindingSystem。
    ///
    /// 工作流程：
    /// 1. 检查是否有新的寻路请求（ASPathfindingRequester.RequestGiven == false）
    /// 2. 若配置了 DontPath 策略，检查起点和终点是否在同一岛屿，不同则跳过
    /// 3. 将合法请求加入全局操作队列（ASPathfindingOperationsBuffer）
    /// 4. 每帧从队列头部取出最多 MaxPathfindingOpsPerFrame 个操作，
    ///    创建对应的寻路操作 Entity（含 ASPathfindingOperation + ASPathfindingResult + ASPathBufferComponent）
    ///
    /// 系统顺序：在 PathfindingSystem 之前执行（队列生产者）
    /// </summary>
    partial struct PathfindingRequestSystem : ISystem
    {
        // 组件查找缓存
        ComponentLookup<ASPathfindingRequester> AllRequesters;
        ComponentLookup<LocalTransform> AllLocalTransforms;
        ComponentLookup<ASPathfindingResult> AllPathfindingResults;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            AllRequesters = state.GetComponentLookup<ASPathfindingRequester>();
            AllPathfindingResults = state.GetComponentLookup<ASPathfindingResult>(true); // 只读
            AllLocalTransforms = state.GetComponentLookup<LocalTransform>(true);         // 只读
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            AllRequesters.Update(ref state);
            AllLocalTransforms.Update(ref state);
            AllPathfindingResults.Update(ref state);

            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

            // 获取全局单例：寻路操作队列、节点岛屿ID、节点数组
            DynamicBuffer<ASPathfindingOperationsBuffer> operationsQueue = SystemAPI.GetSingletonBuffer<ASPathfindingOperationsBuffer>();
            DynamicBuffer<ASNodeIslandIDBufferComponent> nodeIslandIDs = SystemAPI.GetSingletonBuffer<ASNodeIslandIDBufferComponent>();
            DynamicBuffer<ASGridNodesBufferComponent> nodes = SystemAPI.GetSingletonBuffer<ASGridNodesBufferComponent>();

            ASPathfindingParameters pathfindingParameters = new ASPathfindingParameters();

            foreach (var (grid, gridLocalTransform, pathfindingParams) in SystemAPI
                         .Query<RefRO<ASGrid>, RefRO<LocalTransform>, RefRO<ASPathfindingParameters>>())
            {
                pathfindingParameters = pathfindingParams.ValueRO;

                int d = grid.ValueRO.NodeXCount > grid.ValueRO.NodeYCount ? grid.ValueRO.NodeXCount : grid.ValueRO.NodeYCount;

                // DontPath 策略需要岛屿数据已就绪，否则等待
                if (grid.ValueRO.DifferentIslandsPolicy == DifferentIslandsPolicy.DontPath)
                {
                    if (nodeIslandIDs.Length < nodes.Length) return;
                }

                // 遍历所有有寻路请求的代理
                foreach (var (requester, localTransform, entity) in SystemAPI
                             .Query<RefRW<ASPathfindingRequester>, RefRO<LocalTransform>>().WithEntityAccess())
                {
                    if (!requester.ValueRO.RequestGiven) // RequestGiven=false 表示有新请求
                    {
                        // ===== 岛屿连通性检查（DontPath 策略）=====
                        if (grid.ValueRO.DifferentIslandsPolicy == DifferentIslandsPolicy.DontPath)
                        {
                            // 将代理当前位置和目标位置转换为网格节点
                            ASNode startNode = GridUtility.WorldPosToNode(localTransform.ValueRO.Position, grid.ValueRO,
                                gridLocalTransform.ValueRO, nodes);

                            ASNode targetNode = GridUtility.WorldPosToNode(requester.ValueRO.Destination, grid.ValueRO,
                                gridLocalTransform.ValueRO, nodes);

                            // 计算节点的一维索引
                            int startIndex = 0;
                            if (grid.ValueRO.NodeXCount > grid.ValueRO.NodeYCount)
                                startIndex = startNode.X + startNode.Y * d;
                            else startIndex = startNode.Y + startNode.X * d;

                            int targetIndex = 0;
                            if (grid.ValueRO.NodeXCount > grid.ValueRO.NodeYCount)
                                targetIndex = targetNode.X + targetNode.Y * d;
                            else targetIndex = targetNode.Y + targetNode.X * d;

                            // 如果起点和终点的岛屿 ID 不同，跳过此次寻路请求
                            if (nodeIslandIDs[startIndex].IslandID != nodeIslandIDs[targetIndex].IslandID)
                            {
                                requester.ValueRW.RequestGiven = true; // 标记为已处理（防止反复检测）
                                continue;
                            }
                        }

                        // ===== 添加或更新操作队列 =====
                        bool entityHasRequest = false;

                        // 检查队列中是否已有此代理的请求（更新而非重复添加）
                        for (int i = 0; i < operationsQueue.Length; i++)
                        {
                            if (operationsQueue[i].RequestingAgentEntity == entity)
                            {
                                // 更新已有请求的起点（代理可能已移动）
                                operationsQueue[i] = new ASPathfindingOperationsBuffer()
                                {
                                    RequestingAgentEntity = entity,
                                    Operation = new ASPathfindingOperation()
                                    {
                                        StartPoint = localTransform.ValueRO.Position,
                                        TargetPoint = requester.ValueRO.Destination
                                    }
                                };
                                entityHasRequest = true;
                                break;
                            }
                        }

                        // 队列中没有此代理的请求，新增
                        if (!entityHasRequest)
                        {
                            operationsQueue.Add(new ASPathfindingOperationsBuffer()
                            {
                                RequestingAgentEntity = entity,
                                Operation = new ASPathfindingOperation()
                                {
                                    StartPoint = localTransform.ValueRO.Position,
                                    TargetPoint = requester.ValueRO.Destination
                                }
                            });
                        }

                        requester.ValueRW.RequestGiven = true; // 标记请求已提交到队列
                    }
                }
            }

            // ===== 时间分片：每帧最多取出 MaxPathfindingOpsPerFrame 个操作执行 =====
            int k = 0;

            while (k < pathfindingParameters.MaxPathfindingOpsPerFrame && operationsQueue.Length > 0)
            {
                // 获取队列头部的请求（FIFO 先进先出）
                ASPathfindingRequester requester = AllRequesters[operationsQueue[0].RequestingAgentEntity];

                if (SystemAPI.Exists(requester.RequestEntity))
                {
                    // 操作 Entity 已存在：检查是否还在处理中，若是则更新（代理位置可能已变化）
                    ASPathfindingResult result = AllPathfindingResults[requester.RequestEntity];
                    if (!result.FinishedSearch)
                    {
                        // 尚未完成，更新起点坐标（使用当前实时位置）
                        ecb.SetComponent(requester.RequestEntity, new ASPathfindingResult());
                        ecb.SetComponent(requester.RequestEntity, new ASPathfindingOperation()
                        {
                            StartPoint = AllLocalTransforms[operationsQueue[0].RequestingAgentEntity].Position,
                            TargetPoint = requester.Destination
                        });
                    }
                }
                else
                {
                    // 操作 Entity 不存在：创建新 Entity 并添加寻路所需组件
                    Entity e = state.EntityManager.CreateEntity();
                    requester.RequestEntity = e; // 记录操作 Entity 引用到代理

                    ecb.AddComponent<ASPathfindingResult>(e);  // 结果标志（初始 false/false）
                    ecb.AddComponent<ASPathfindingOperation>(e, new ASPathfindingOperation()
                    {
                        StartPoint = AllLocalTransforms[operationsQueue[0].RequestingAgentEntity].Position,
                        TargetPoint = requester.Destination
                    });
                    ecb.AddBuffer<ASPathBufferComponent>(e);   // 路径点缓冲区
                }

                // 将更新后的 requester（含新的 RequestEntity）写回代理
                AllRequesters[operationsQueue[0].RequestingAgentEntity] = requester;

                operationsQueue.RemoveAt(0); // 从队列移除已处理的请求
                k++;
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }
    }
}
