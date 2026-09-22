using Unity.Burst;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Collections;
using KNN;
using KNN.Jobs;
using Unity.Jobs;
using Unity.Physics;

namespace Athomield.AStar
{
    /// <summary>
    /// 代理移动系统，系统的核心执行层，每帧负责：
    ///
    /// 1. 路径结果同步：
    ///    检查各代理对应的寻路操作 Entity 是否完成，若完成则将路径点复制到代理自身缓冲区
    ///
    /// 2. 目标追踪（ASConstantTargetFollower）：
    ///    检测目标 Entity 位置变化，动态触发重新寻路
    ///
    /// 3. 无避障代理的移动：
    ///    简单沿路径点前进，到达路径点后切换到下一个，并通过物理射线防止穿越障碍物
    ///
    /// 4. RVO 避障代理的移动（核心算法）：
    ///    a. 使用 KD-Tree（KNN）查找每个代理的 K 近邻
    ///    b. 为每个邻居计算 ORCA（最优互惠碰撞避免）速度约束线
    ///    c. 通过线性规划在所有约束下找最优速度
    ///    d. 物理射线检测阻止穿越墙壁
    /// </summary>
    partial struct AgentMovementSystem : ISystem
    {
        // 各组件的查找缓存（在 OnCreate 中初始化，每帧在 OnUpdate 中 Update）
        public ComponentLookup<ASPathfindingResult> AllResults;
        public BufferLookup<ASPathBufferComponent> AllPathPoints;
        public ComponentLookup<ASConstantTargetFollower> AllConstantFollowers;

        public ComponentLookup<ASAgent> AllAgents;
        public ComponentLookup<LocalTransform> AllLocalTransforms;
        public ComponentLookup<ASPathFollower> AllPathFollowers;
        public BufferLookup<ASAgentNeighboursBufferComponent> AllAgentsNeighbours;

        private EntityQuery _avoidanceAgentsQuery;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            AllResults = state.GetComponentLookup<ASPathfindingResult>(true);
            AllPathPoints = state.GetBufferLookup<ASPathBufferComponent>(true);
            AllConstantFollowers = state.GetComponentLookup<ASConstantTargetFollower>();

            AllAgents = state.GetComponentLookup<ASAgent>();
            AllLocalTransforms = state.GetComponentLookup<LocalTransform>();
            AllPathFollowers = state.GetComponentLookup<ASPathFollower>(true);
            AllAgentsNeighbours = state.GetBufferLookup<ASAgentNeighboursBufferComponent>();

            _avoidanceAgentsQuery = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<LocalTransform, ASAvoidanceAgent>()
                .WithAll<ASAgent, ASPathFollower>()
                .WithAll<ASAgentNeighboursBufferComponent>()
                .Build(ref state);

            state.RequireForUpdate<PhysicsWorldSingleton>();
            state.RequireForUpdate<ASGrid>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            // 每帧更新所有 Lookup（确保访问最新数据）
            AllResults.Update(ref state);
            AllPathPoints.Update(ref state);
            AllConstantFollowers.Update(ref state);
            AllAgents.Update(ref state);
            AllLocalTransforms.Update(ref state);
            AllPathFollowers.Update(ref state);
            AllAgentsNeighbours.Update(ref state);

            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

            // 获取 Unity Physics World 单例（用于射线检测）
            var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

            // 获取 Grid 参数（节点大小等）
            var grid = SystemAPI.GetSingleton<ASGrid>();

            // =============================================
            // 阶段 1：路径结果同步 + 目标追踪
            // =============================================
            foreach (var (pathfindingRequester, pathFollower, agentEntity) in SystemAPI
                         .Query<RefRW<ASPathfindingRequester>, RefRW<ASPathFollower>>()
                         .WithAll<ASPathBufferComponent>().WithEntityAccess())
            {
                // ----- 1a. 检查寻路操作是否完成，完成则复制路径 -----
                //消费A*寻路结果(requestEntity)
                if (pathfindingRequester.ValueRO.RequestEntity != Entity.Null &&
                    SystemAPI.Exists(pathfindingRequester.ValueRO.RequestEntity) &&
                    AllResults.HasComponent(pathfindingRequester.ValueRO.RequestEntity))
                {
                    ASPathfindingResult reqResult = AllResults[pathfindingRequester.ValueRO.RequestEntity];

                    if (reqResult.FinishedSearch)
                    {
                        if (reqResult.PathFound &&
                            AllPathPoints.HasBuffer(pathfindingRequester.ValueRO.RequestEntity))
                        {
                            // 从操作 Entity 的路径缓冲区复制路径点到代理的路径缓冲区
                            DynamicBuffer<ASPathBufferComponent> reqPathPoints =
                                AllPathPoints[pathfindingRequester.ValueRO.RequestEntity];
                            DynamicBuffer<ASPathBufferComponent> agentPathPoints =
                                state.EntityManager.GetBuffer<ASPathBufferComponent>(agentEntity);

                            pathFollower.ValueRW.PathAvailable = true;
                            pathFollower.ValueRW.TargetIndex = 0; // 从路径起点开始追踪
                            pathFollower.ValueRW.DestinationReached = false;

                            agentPathPoints.Clear();
                            for (int i = 0; i < reqPathPoints.Length; i++)
                            {
                                agentPathPoints.Add(new ASPathBufferComponent() { Point = reqPathPoints[i].Point });
                            }
                        }
                        else if (reqResult.PathFound)
                        {
                            // 非法/不完整的寻路结果不能继续沿用旧路径。
                            pathFollower.ValueRW.PathAvailable = false;
                        }

                        // 路径已读取，销毁临时操作 Entity（清理内存）
                        ecb.DestroyEntity(pathfindingRequester.ValueRO.RequestEntity);
                    }
                }

                // ----- 1b. 持续目标追踪：检测目标位置变化，触发重新寻路 -----
                if (AllConstantFollowers.HasComponent(agentEntity))
                {
                    ASConstantTargetFollower constantTargetFollower = AllConstantFollowers[agentEntity];

                    if (constantTargetFollower.Target != Entity.Null &&
                        SystemAPI.Exists(constantTargetFollower.Target))
                    {
                        if (AllLocalTransforms.HasComponent(constantTargetFollower.Target))
                        {
                            if (constantTargetFollower.RecalculateOnStationary)
                            {
                                // 无论目标是否移动都重新寻路
                                pathfindingRequester.ValueRW.RequestGiven = false;
                                pathfindingRequester.ValueRW.Destination =
                                    AllLocalTransforms[constantTargetFollower.Target].Position;

                                constantTargetFollower.TargetLastPosition =
                                    AllLocalTransforms[constantTargetFollower.Target].Position;
                                AllConstantFollowers[agentEntity] = constantTargetFollower;
                            }
                            else
                            {
                                // 仅在目标移动超过阈值（0.01 单位距离²）时重新寻路
                                if (math.distancesq(AllLocalTransforms[constantTargetFollower.Target].Position,
                                        constantTargetFollower.TargetLastPosition) > 0.01f)
                                {
                                    pathfindingRequester.ValueRW.RequestGiven = false;
                                    pathfindingRequester.ValueRW.Destination =
                                        AllLocalTransforms[constantTargetFollower.Target].Position;

                                    constantTargetFollower.TargetLastPosition =
                                        AllLocalTransforms[constantTargetFollower.Target].Position;
                                    AllConstantFollowers[agentEntity] = constantTargetFollower;
                                }
                            }
                        }
                    }
                }
            }

            // =============================================
            // 阶段 2：无避障代理的移动（WithAbsent<ASAvoidanceAgent>）
            // =============================================
            foreach (var (agent, pathFollower, agentTransform, pathRequester, agentEntity) in SystemAPI
                         .Query<RefRW<ASAgent>, RefRW<ASPathFollower>, RefRW<LocalTransform>,
                             RefRO<ASPathfindingRequester>>()
                         .WithAll<ASPathBufferComponent>().WithAbsent<ASAvoidanceAgent>().WithEntityAccess())
            {
                if (!pathFollower.ValueRO.PathAvailable || pathFollower.ValueRO.DestinationReached)
                {
                    agent.ValueRW.CurrentVelocity = float3.zero;
                    continue;
                }

                // 强制 Y 轴固定在代理中心高度（防止 Y 轴漂移）
                if (math.abs(agentTransform.ValueRO.Position.y - agent.ValueRO.Height / 2) > 0.001f)
                {
                    agentTransform.ValueRW.Position.y = agent.ValueRO.Height / 2;
                }

                DynamicBuffer<ASPathBufferComponent> agentPath =
                    state.EntityManager.GetBuffer<ASPathBufferComponent>(agentEntity);

                if (agentPath.Length == 0)
                {
                    pathFollower.ValueRW.PathAvailable = false;
                    agent.ValueRW.CurrentVelocity = float3.zero;
                    continue;
                }

                if (pathFollower.ValueRO.TargetIndex < 0)
                {
                    pathFollower.ValueRW.TargetIndex = 0;
                }

                if (pathFollower.ValueRO.TargetIndex >= agentPath.Length)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    agent.ValueRW.CurrentVelocity = float3.zero;
                    continue;
                }

                // 当前追踪路径点（Y 轴对齐代理高度）
                float3 currentPoint = agentPath[pathFollower.ValueRO.TargetIndex].Point;
                currentPoint.y = agent.ValueRO.Height / 2;

                // ----- 路径点切换逻辑 -----
                if (pathFollower.ValueRO.TargetIndex != agentPath.Length - 1)
                {
                    // 非最后一个点：使用 NextTargetTouchDistance 阈值
                    float nextTouchDistance = agent.ValueRO.NextTargetTouchDistance < 0
                        ? grid.NodeSize
                        : agent.ValueRO.NextTargetTouchDistance;

                    if (math.distance(currentPoint, agentTransform.ValueRO.Position) <
                        agent.ValueRO.Radius + nextTouchDistance)
                    {
                        pathFollower.ValueRW.TargetIndex++; // 切换到下一个路径点
                    }
                }
                else
                {
                    // 最后一个点：使用 StoppingDistance 阈值
                    if (math.distance(currentPoint, agentTransform.ValueRO.Position) <
                        agent.ValueRO.Radius + agent.ValueRO.StoppingDistance)
                    {
                        pathFollower.ValueRW.TargetIndex++;
                    }
                }

                // 索引超出路径长度：到达终点
                if (pathFollower.ValueRO.TargetIndex > agentPath.Length - 1)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    agent.ValueRW.CurrentVelocity = float3.zero;
                    continue;
                }

                // 检查是否足够接近最终目的地（精确停止）
                float3 finalDestinationPoint = pathRequester.ValueRO.Destination;
                finalDestinationPoint.y = agent.ValueRO.Height / 2;

                if (math.distance(finalDestinationPoint, agentTransform.ValueRO.Position) <
                    agent.ValueRO.Radius + agent.ValueRO.StoppingDistance)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    agent.ValueRW.CurrentVelocity = float3.zero;
                    continue;
                }

                // 计算朝向当前路径点的速度
                float3 destination = agentPath[pathFollower.ValueRO.TargetIndex].Point;
                destination.y = agent.ValueRO.Height / 2;
                float3 destinationDir = math.normalizesafe(destination - agentTransform.ValueRO.Position);
                agent.ValueRW.CurrentVelocity = destinationDir * agent.ValueRO.MaxSpeed;

                // ----- 障碍物射线检测（防止穿墙）-----
                var collFilter = new CollisionFilter()
                {
                    BelongsTo = ~0u,
                    CollidesWith = (uint)grid.InaccessibleLMIndex,
                    GroupIndex = 0
                };

                bool obsAhead = false;

                if (math.lengthsq(destinationDir) > 0.000001f)
                {
                    float3 rayStart = agentTransform.ValueRO.Position -
                                      agentTransform.ValueRO.Up() * (agent.ValueRO.Height * 0.5f);
                    obsAhead = physicsWorld.CastRay(new RaycastInput()
                    {
                        Start = rayStart,
                        End = rayStart + destinationDir * agent.ValueRO.Radius,
                        Filter = collFilter
                    });
                }

                if (obsAhead)
                {
                    agent.ValueRW.CurrentVelocity = float3.zero;
                }
                else
                {
                    // 无障碍：更新位置
                    agentTransform.ValueRW.Position += agent.ValueRW.CurrentVelocity * SystemAPI.Time.DeltaTime;

                    // 平滑旋转朝向移动方向
                    quaternion destRot = quaternion.LookRotationSafe(destinationDir, agentTransform.ValueRO.Up());
                    float ang = math.angle(destRot, agentTransform.ValueRW.Rotation);

                    if (math.abs(ang) > 0.01f)
                    {
                        float angStep = math.min(ang, math.radians(SystemAPI.Time.DeltaTime * agent.ValueRO.TurnSpeed));
                        agentTransform.ValueRW.Rotation =
                            math.slerp(agentTransform.ValueRW.Rotation, destRot, angStep / ang);
                    }
                }
            }

            // =============================================
            // 阶段 3：构建 KD-Tree，并为每个避障代理写入 K 近邻
            // =============================================
            if (_avoidanceAgentsQuery.IsEmpty)
            {
                ecb.Playback(state.EntityManager);
                ecb.Dispose();
                return;
            }

            NativeArray<Entity> avoidanceEntities =
                _avoidanceAgentsQuery.ToEntityArray(Allocator.TempJob);
            int avoidanceAgentCount = avoidanceEntities.Length;
            int wantedNeighbours = math.clamp(
                grid.AvoidanceNeighbours,
                0,
                math.max(0, avoidanceAgentCount - 1));
            int queryK = wantedNeighbours + 1; // 查询结果包含自身

            NativeArray<float3> avoidancePositions =
                new NativeArray<float3>(avoidanceAgentCount, Allocator.TempJob);
            NativeArray<AvoidanceSnapshot> snapshots =
                new NativeArray<AvoidanceSnapshot>(avoidanceAgentCount, Allocator.TempJob);
            NativeParallelHashMap<Entity, int> snapshotIndices =
                new NativeParallelHashMap<Entity, int>(avoidanceAgentCount, Allocator.TempJob);

            // ORCA 必须让所有代理读取同一时刻的位置和速度，不能边计算边读取本帧已更新的数据。
            for (int i = 0; i < avoidanceAgentCount; i++)
            {
                Entity entity = avoidanceEntities[i];
                ASAgent currentAgent = AllAgents[entity];
                ASPathFollower currentFollower = AllPathFollowers[entity];
                bool stationary = !currentFollower.PathAvailable ||
                                  currentFollower.DestinationReached ||
                                  currentAgent.MaxSpeed <= 0.0001f;

                if (stationary)
                {
                    currentAgent.CurrentVelocity = float3.zero;
                    AllAgents[entity] = currentAgent;
                }

                float3 position = AllLocalTransforms[entity].Position;
                position.y = 0f; // 邻居查询和 ORCA 都只在 XZ 平面进行
                avoidancePositions[i] = position;
                snapshots[i] = new AvoidanceSnapshot
                {
                    Position = PathfindingUtility.V2FromV3(position),
                    Velocity = PathfindingUtility.V2FromV3(currentAgent.CurrentVelocity),
                    Radius = math.max(0f, currentAgent.Radius),
                    IsStationary = stationary,
                    DestinationReached = currentFollower.DestinationReached,
                    IgnoreAvoidanceWhenDestReached = currentAgent.IgnoreAvoidanceWhenDestReached
                };
                snapshotIndices.TryAdd(entity, i);
            }

            NativeArray<int> nearestResults =
                new NativeArray<int>(avoidanceAgentCount * queryK, Allocator.TempJob);
            KnnContainer knnContainer =
                new KnnContainer(avoidancePositions, false, Allocator.TempJob);

            new KnnRebuildJob(knnContainer).Schedule().Complete();  //构建kd-tree

            int batchSize = math.max(1, avoidanceAgentCount / 32);
            QueryKNearestBatchJob queryJob =
                new QueryKNearestBatchJob(knnContainer, avoidancePositions, nearestResults);
            queryJob.ScheduleBatch(avoidanceAgentCount, batchSize).Complete();   //查找最近k个，放进nearestResults
                
            //把查询结果添加进agent的component里
            for (int i = 0; i < avoidanceAgentCount; i++)
            {   
                Entity entity = avoidanceEntities[i];
                DynamicBuffer<ASAgentNeighboursBufferComponent> neighbours = AllAgentsNeighbours[entity];
                neighbours.Clear();

                int added = 0;

                // KnnContainer 的结果顺序是从远到近，因此反向读取，使最近邻优先进入 ORCA。
                for (int j = queryK - 1; j >= 0 && added < wantedNeighbours; j--)
                {
                    int otherIndex = nearestResults[i * queryK + j];
                    if ((uint)otherIndex >= (uint)avoidanceAgentCount || otherIndex == i)
                    {
                        continue;
                    }

                    AvoidanceSnapshot other = snapshots[otherIndex];
                    if (other.DestinationReached && other.IgnoreAvoidanceWhenDestReached)
                    {
                        continue;
                    }

                    neighbours.Add(new ASAgentNeighboursBufferComponent
                    {
                        Agent = avoidanceEntities[otherIndex]
                    });
                    added++;
                }
            }

            // =============================================
            // 阶段 4：计算所有避障代理的新速度（只读快照），然后统一应用
            // =============================================
            NativeArray<float2> newVelocities =
                new NativeArray<float2>(avoidanceAgentCount, Allocator.TempJob,
                    NativeArrayOptions.ClearMemory);

            float deltaTime = SystemAPI.Time.DeltaTime;

            foreach (var (agent, pathFollower, avoidanceAgent, agentTransform, pathRequester, agentEntity) in
                     SystemAPI.Query<RefRO<ASAgent>, RefRW<ASPathFollower>, RefRO<ASAvoidanceAgent>,
                             RefRO<LocalTransform>, RefRO<ASPathfindingRequester>>()
                         .WithAll<ASPathBufferComponent, ASAgentNeighboursBufferComponent>()
                         .WithEntityAccess())
            {
                if (!snapshotIndices.TryGetValue(agentEntity, out int snapshotIndex))
                {
                    continue;
                }

                if (!pathFollower.ValueRO.PathAvailable || pathFollower.ValueRO.DestinationReached)
                {
                    newVelocities[snapshotIndex] = float2.zero;
                    continue;
                }

                DynamicBuffer<ASPathBufferComponent> agentPath =
                    state.EntityManager.GetBuffer<ASPathBufferComponent>(agentEntity);

                if (agentPath.Length == 0)
                {
                    pathFollower.ValueRW.PathAvailable = false;
                    newVelocities[snapshotIndex] = float2.zero;
                    continue;
                }

                if (pathFollower.ValueRO.TargetIndex < 0)
                {
                    pathFollower.ValueRW.TargetIndex = 0;
                }

                if (pathFollower.ValueRO.TargetIndex >= agentPath.Length)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    newVelocities[snapshotIndex] = float2.zero;
                    continue;
                }

                float3 currentPosition = agentTransform.ValueRO.Position;
                currentPosition.y = agent.ValueRO.Height * 0.5f;
                float3 currentPoint = agentPath[pathFollower.ValueRO.TargetIndex].Point;
                currentPoint.y = currentPosition.y;

                if (pathFollower.ValueRO.TargetIndex != agentPath.Length - 1)
                {
                    float nextTouchDistance = agent.ValueRO.NextTargetTouchDistance < 0f
                        ? grid.NodeSize
                        : agent.ValueRO.NextTargetTouchDistance;

                    if (math.distance(currentPoint, currentPosition) <
                        agent.ValueRO.Radius + nextTouchDistance)
                    {
                        pathFollower.ValueRW.TargetIndex++;
                    }
                }
                else if (math.distance(currentPoint, currentPosition) <
                         agent.ValueRO.Radius + agent.ValueRO.StoppingDistance)
                {
                    pathFollower.ValueRW.TargetIndex++;
                }

                if (pathFollower.ValueRO.TargetIndex >= agentPath.Length)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    newVelocities[snapshotIndex] = float2.zero;
                    continue;
                }

                float3 finalDestination = pathRequester.ValueRO.Destination;
                finalDestination.y = currentPosition.y;
                if (math.distance(finalDestination, currentPosition) <
                    agent.ValueRO.Radius + agent.ValueRO.StoppingDistance)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    newVelocities[snapshotIndex] = float2.zero;
                    continue;
                }
                //targetIndex计算完毕

                float3 destination = agentPath[pathFollower.ValueRO.TargetIndex].Point;
                destination.y = currentPosition.y;
                float3 preferredDirection = math.normalizesafe(destination - currentPosition);
                float2 preferredVelocity = PathfindingUtility.V2FromV3(preferredDirection) *
                                           math.max(0f, agent.ValueRO.MaxSpeed);

                DynamicBuffer<ASAgentNeighboursBufferComponent> neighbours =
                    state.EntityManager.GetBuffer<ASAgentNeighboursBufferComponent>(agentEntity);
                NativeList<ASLine> orcaLines =
                    new NativeList<ASLine>(math.max(1, neighbours.Length), Allocator.Temp);

                float2 calculatedVelocity = float2.zero;
                CalculateFinalVelocity(
                    orcaLines,
                    agentEntity,
                    snapshots[snapshotIndex],
                    agent.ValueRO,
                    avoidanceAgent.ValueRO,
                    neighbours,
                    snapshots,
                    snapshotIndices,
                    deltaTime,
                    preferredVelocity,
                    ref calculatedVelocity);

                if (!math.all(math.isfinite(calculatedVelocity)))
                {
                    calculatedVelocity = float2.zero;
                }

                newVelocities[snapshotIndex] = calculatedVelocity;
                orcaLines.Dispose();
            }

            CollisionFilter collisionFilter = new CollisionFilter
            {
                BelongsTo = ~0u,
                CollidesWith = (uint)grid.InaccessibleLMIndex,
                GroupIndex = 0
            };

            // 第二遍统一应用速度，保证 ORCA 计算阶段看到的是严格一致的上一帧快照。
            for (int i = 0; i < avoidanceAgentCount; i++)
            {
                Entity entity = avoidanceEntities[i];
                ASAgent currentAgent = AllAgents[entity];
                LocalTransform currentTransform = AllLocalTransforms[entity];
                currentTransform.Position.y = currentAgent.Height * 0.5f;

                float2 velocity2D = newVelocities[i];
                float3 velocity = new float3(velocity2D.x, 0f, velocity2D.y);
                float speed = math.length(velocity);

                if (speed > 0.0001f)
                {
                    float3 moveDirection = velocity / speed;
                    float3 rayStart = currentTransform.Position -
                                      currentTransform.Up() * (currentAgent.Height * 0.5f);
                    float rayLength = math.max(currentAgent.Radius, speed * math.max(0f, deltaTime));

                    bool obstacleAhead = physicsWorld.CastRay(new RaycastInput
                    {
                        Start = rayStart,
                        End = rayStart + moveDirection * rayLength,
                        Filter = collisionFilter
                    });

                    if (obstacleAhead)
                    {
                        velocity = float3.zero;
                    }
                    else
                    {
                        quaternion targetRotation =
                            quaternion.LookRotationSafe(moveDirection, currentTransform.Up());
                        float angle = math.angle(targetRotation, currentTransform.Rotation);

                        if (math.abs(angle) > 0.01f)
                        {
                            float angleStep = math.min(
                                angle,
                                math.radians(math.max(0f, deltaTime) * currentAgent.TurnSpeed));
                            currentTransform.Rotation = math.slerp(
                                currentTransform.Rotation,
                                targetRotation,
                                angleStep / angle);
                        }
                    }
                }

                currentTransform.Position += velocity * math.max(0f, deltaTime);
                currentAgent.CurrentVelocity = velocity;
                AllLocalTransforms[entity] = currentTransform;
                AllAgents[entity] = currentAgent;
            }

            newVelocities.Dispose();
            snapshotIndices.Dispose();
            snapshots.Dispose();
            nearestResults.Dispose();
            knnContainer.Dispose();
            avoidancePositions.Dispose();
            avoidanceEntities.Dispose();

            ecb.Playback(state.EntityManager);
            ecb.Dispose();

        }

        private struct AvoidanceSnapshot
        {
            public float2 Position;
            public float2 Velocity;
            public float Radius;
            public bool IsStationary;
            public bool DestinationReached;
            public bool IgnoreAvoidanceWhenDestReached;
        }

        /// <summary>
        /// 为一个代理生成 ORCA 半平面，并在最大速度圆内求最接近首选速度的合法速度。
        /// </summary>
        private void CalculateFinalVelocity(
            NativeList<ASLine> orcaLines,
            Entity agentEntity,
            AvoidanceSnapshot agentSnapshot,
            ASAgent agent,
            ASAvoidanceAgent avoidanceAgent,
            DynamicBuffer<ASAgentNeighboursBufferComponent> agentNeighbours,
            NativeArray<AvoidanceSnapshot> snapshots,
            NativeParallelHashMap<Entity, int> snapshotIndices,
            float deltaTime,
            float2 preferredVelocity,
            ref float2 newVelocity)
        {
            float invTimeHorizon = 1f / math.max(avoidanceAgent.TimeHorizon, 0.0001f);
            float invTimeStep = 1f / math.max(deltaTime, 0.0001f);

            for (int i = 0; i < agentNeighbours.Length; i++)
            {
                Entity otherEntity = agentNeighbours[i].Agent;
                if (otherEntity == agentEntity ||
                    !snapshotIndices.TryGetValue(otherEntity, out int otherIndex))
                {
                    continue;
                }

                AvoidanceSnapshot other = snapshots[otherIndex];
                float2 relativePosition = other.Position - agentSnapshot.Position;
                float2 relativeVelocity = agentSnapshot.Velocity - other.Velocity;
                float distanceSq = math.lengthsq(relativePosition);
                float combinedRadius = agentSnapshot.Radius + other.Radius;
                float combinedRadiusSq = combinedRadius * combinedRadius;

                ASLine line;
                float2 correction;

                if (distanceSq > combinedRadiusSq)
                {
                    float2 w = relativeVelocity - invTimeHorizon * relativePosition;
                    float wLengthSq = math.lengthsq(w);
                    float dotProduct = math.dot(w, relativePosition);
                    
                    //圆弧区域
                    if (dotProduct < 0f &&
                        dotProduct * dotProduct > combinedRadiusSq * wLengthSq)
                    {
                        float wLength = math.sqrt(wLengthSq);
                        float2 unitW = math.normalizesafe(
                            w,
                            StableSeparationDirection(agentEntity, otherEntity));

                        line.Direction = new float2(unitW.y, -unitW.x); 
                        correction = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        float leg = math.sqrt(math.max(0f, distanceSq - combinedRadiusSq)); //切线长度

                        if (Det(relativePosition, w) > 0f) //w是在左边,逆时针旋转θ
                        {
                            line.Direction = new float2(
                                relativePosition.x * leg - relativePosition.y * combinedRadius,
                                relativePosition.x * combinedRadius + relativePosition.y * leg) /
                                             math.max(distanceSq, 0.000001f);
                        }
                        else  //在右边就是顺时针旋转θ
                        {
                            line.Direction = -new float2(
                                relativePosition.x * leg + relativePosition.y * combinedRadius,
                                -relativePosition.x * combinedRadius + relativePosition.y * leg) /
                                             math.max(distanceSq, 0.000001f);
                        }
                        //保证line的左侧是安全区域

                        line.Direction = math.normalizesafe(
                            line.Direction,
                            StableSeparationDirection(agentEntity, otherEntity));
                        float projectedSpeed = math.dot(relativeVelocity, line.Direction);
                        correction = projectedSpeed * line.Direction - relativeVelocity;
                    }
                }
                else
                {
                    // 已发生重叠：用当前时间步立即分离。完全重合时使用稳定的相反方向。
                    float2 w = relativeVelocity - invTimeStep * relativePosition;
                    float wLength = math.length(w);
                    float2 unitW = math.normalizesafe(
                        w,
                        StableSeparationDirection(agentEntity, otherEntity));

                    line.Direction = new float2(unitW.y, -unitW.x);
                    correction = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                // 静止代理不会兑现“各承担一半”的移动，因此移动方承担全部修正。
                float responsibility = other.IsStationary ? 1f : 0.5f;
                line.Point = agentSnapshot.Velocity + responsibility * correction;
                orcaLines.Add(line);
            }

            float maxSpeed = math.max(0f, agent.MaxSpeed);
            int failedLine = FindSubOptimalPath(
                orcaLines,
                maxSpeed,
                preferredVelocity,
                false,
                ref newVelocity);

            if (failedLine < orcaLines.Length)
            {
                FindSubOptimalPathWithLines(
                    orcaLines,
                    0,
                    failedLine,
                    maxSpeed,
                    ref newVelocity);
            }
        }

        /// <summary>返回一对完全重合代理各自相反且跨帧稳定的分离方向。</summary>
        private static float2 StableSeparationDirection(Entity first, Entity second)
        {
            int lowIndex = math.min(first.Index, second.Index);
            int highIndex = math.max(first.Index, second.Index);
            uint hash = math.hash(new uint2((uint)lowIndex, (uint)highIndex));
            float angle = hash / 4294967295f * (2f * math.PI);
            float2 direction = new float2(math.cos(angle), math.sin(angle));
            return first.Index <= second.Index ? direction : -direction;
        }

        private static float Det(float2 first, float2 second)
        {
            return first.x * second.y - first.y * second.x;
        }

        /// <summary>在指定 ORCA 线上求同时满足之前约束且最接近目标速度的点。</summary>
        /// 通过算t来求当前最优解result
        private static bool FindOptimalPath(
            NativeList<ASLine> lines,
            int lineNumber,
            float radius,
            float2 optimalVelocity,
            bool directionOptimal,
            ref float2 result)
        {
            ASLine selectedLine = lines[lineNumber];
            float dotProduct = math.dot(selectedLine.Point, selectedLine.Direction);
            float discriminant = dotProduct * dotProduct + radius * radius -
                                 math.lengthsq(selectedLine.Point);  //算半弦长, R * R - d * d
            
            //约束线与速度圆没有交点
            if (discriminant < -0.00001f)
            {
                return false;
            }

            discriminant = math.max(0f, discriminant);
            float sqrtDiscriminant = math.sqrt(discriminant);
            float left = -dotProduct - sqrtDiscriminant;
            float right = -dotProduct + sqrtDiscriminant;
            
            //
            for (int i = 0; i < lineNumber; i++)
            {
                float denominator = Det(selectedLine.Direction, lines[i].Direction);  
                float numerator = Det(
                    lines[i].Direction,
                    selectedLine.Point - lines[i].Point);   

                if (math.abs(denominator) <= 0.00001f)   //线平行
                {
                    if (numerator < 0f) 
                    {
                        return false;
                    }

                    continue;
                }
                
                //合法度量: f(t) = numerator - t * denominator >= 0

                float t = numerator / denominator;   //t为两个line的交点参数
                if (denominator >= 0f)  //斜率为负, t越小越合法，收紧right
                {
                    right = math.min(right, t);
                }
                else //斜率为正，t越大越合法，收紧left
                {
                    left = math.max(left, t);
                }

                if (left > right)
                {
                    return false;
                }
            }

            if (directionOptimal)
            {
                result = math.dot(optimalVelocity, selectedLine.Direction) > 0f
                    ? selectedLine.Point + right * selectedLine.Direction
                    : selectedLine.Point + left * selectedLine.Direction;
            }
            else
            {
                float t = math.clamp(
                    math.dot(selectedLine.Direction, optimalVelocity - selectedLine.Point),
                    left,
                    right);
                result = selectedLine.Point + t * selectedLine.Direction;
            }

            return true;
        }

        /// <summary>ORCA 二维增量线性规划。</summary>
        private static int FindSubOptimalPath(
            NativeList<ASLine> lines,
            float radius,
            float2 optimalVelocity,
            bool directionOptimal,
            ref float2 result)
        {
            if (directionOptimal)
            {
                result = math.normalizesafe(optimalVelocity) * radius;
            }
            else if (math.lengthsq(optimalVelocity) > radius * radius)
            {
                result = math.normalizesafe(optimalVelocity) * radius;
            }
            else
            {
                result = optimalVelocity;
            }

            for (int i = 0; i < lines.Length; i++) //线性增加约束
            {
                // 合法半平面：det(Direction, velocity - Point) >= 0。
                if (Det(lines[i].Direction, lines[i].Point - result) > 0f)
                {
                    float2 previousResult = result;
                    if (!FindOptimalPath(
                            lines,
                            i,
                            radius,
                            optimalVelocity,
                            directionOptimal,
                            ref result))
                    {
                        result = previousResult;
                        return i;
                    }
                }
            }

            return lines.Length;
        }

        /// <summary>处理约束冲突，在投影约束中寻找违反程度最小的速度。</summary>
        private static void FindSubOptimalPathWithLines(
            NativeList<ASLine> lines,
            int obstacleLineCount,
            int beginLine,
            float radius,
            ref float2 result)
        {
            float maxViolation = 0f;

            for (int i = beginLine; i < lines.Length; i++)
            {
                //找到>最大违反量的约束，尽可能压缩maxViolation
                if (Det(lines[i].Direction, lines[i].Point - result) <= maxViolation) 
                {
                    continue;
                }

                NativeList<ASLine> projectedLines =
                    new NativeList<ASLine>(math.max(1, i), Allocator.Temp);

                for (int obstacle = 0; obstacle < obstacleLineCount; obstacle++)
                {
                    projectedLines.Add(lines[obstacle]);
                }

                for (int j = obstacleLineCount; j < i; j++)
                {
                    //对之前的约束线生成projectedLine
                    ASLine projectedLine;
                    float determinant = Det(lines[i].Direction, lines[j].Direction);

                    if (math.abs(determinant) <= 0.00001f) //平行
                    {
                        if (math.dot(lines[i].Direction, lines[j].Direction) > 0f)
                        {
                            //同向,生成不了到两个约束线违反量相等的projectedLine
                            continue;
                        }

                        projectedLine.Point = 0.5f * (lines[i].Point + lines[j].Point);  //平行线的中线
                    }
                    else
                    {
                        //计算交点
                        projectedLine.Point = lines[i].Point +
                            Det(lines[j].Direction, lines[i].Point - lines[j].Point) /
                            determinant * lines[i].Direction;
                    }

                    projectedLine.Direction = math.normalizesafe(
                        lines[j].Direction - lines[i].Direction,
                        new float2(-lines[i].Direction.y, lines[i].Direction.x));
                    projectedLines.Add(projectedLine);
                }

                float2 previousResult = result;
                float2 optimizationDirection =
                    new float2(-lines[i].Direction.y, lines[i].Direction.x);

                if (FindSubOptimalPath(
                        projectedLines,
                        radius,
                        optimizationDirection,
                        true,
                        ref result) < projectedLines.Length)
                {
                    result = previousResult;
                }

                maxViolation = Det(lines[i].Direction, lines[i].Point - result);
                projectedLines.Dispose();
            }
        }
    }
}
