using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace Athomield.AStar
{
    
    public class GridAuthoring : MonoBehaviour
    {
        [Header("Grid")] 
        public Vector2 mGridSize;

        //节点边长
        public float mNodeSize;

        [Min(1)]
        [Tooltip("RVO/ORCA 每个代理最多考虑的邻居数量，推荐 8")]
        public int AvoidanceNeighbours = 8;

        [Tooltip("网格刷新后是否强制所有代理重新规划路径")]
        public bool RecalcPathsAfterGridRefresh;
        
        /// <summary>不可通行节点的物理层（障碍物层），此层碰撞体覆盖的节点会被标记为不可通行</summary>
        public LayerMask InaccessibleNodes;

        /// <summary>惩罚节点的物理层（惩罚区域层），此层碰撞体覆盖的节点会增加寻路代价</summary>
        public LayerMask PenaltyNodes;
        
        [Tooltip("Max number of pathfinding operations to calculate per frame")]
        /// <summary>每帧最多执行的寻路操作数（防止大量请求同帧导致卡顿）</summary>
        public int MaxPathfindingOpsPerFrame;
        
        [Tooltip("What to do when agent and target point are inaccessible to each other")]
        /// <summary>起点与终点位于不同岛屿时的处理策略</summary>
        public DifferentIslandsPolicy DifferentIslandsPolicy;

        [Tooltip("if target is an inaccessible node, how far to to look for an accessible node in a radial direction (best between 10-20 increasing with grid density)")]
        /// <summary>目标节点不可通行时，BFS 搜索可通行节点的最大范围（推荐 10-20）</summary>
        public int TargetInaccessibleSearchTolerance;


        public class GridBaker : Baker<GridAuthoring>
        {
            public override void Bake(GridAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);

                ASGrid grid = new ASGrid
                {
                    NodeXCount = (int)math.round(authoring.mGridSize.x / authoring.mNodeSize),
                    NodeYCount = (int)math.round(authoring.mGridSize.y / authoring.mNodeSize),
                    NodeSize = authoring.mNodeSize,
                    AvoidanceNeighbours = math.max(1, authoring.AvoidanceNeighbours),
                    RecalcPathsAfterGridRefresh = authoring.RecalcPathsAfterGridRefresh,
                    InaccessibleLMIndex = authoring.InaccessibleNodes.value,    // LayerMask 转 int
                    PenaltyLMIndex = authoring.PenaltyNodes.value,
                    DifferentIslandsPolicy = authoring.DifferentIslandsPolicy,
                    TargetInaccessibleSearchTolerance = authoring.TargetInaccessibleSearchTolerance
                };
                
                ASPathfindingParameters pathfindingParameters = new ASPathfindingParameters()
                {
                    MaxPathfindingOpsPerFrame = authoring.MaxPathfindingOpsPerFrame
                };

                
                AddComponent(entity,grid);
                AddComponent(entity, pathfindingParameters);
                
                AddBuffer<ASPathfindingOperationsBuffer>(entity);  // 寻路请求队列
                AddBuffer<ASNodeIslandIDBufferComponent>(entity);  // 岛屿 ID 缓冲区
                
                
                DynamicBuffer<ASGridNodesBufferComponent> nodes = AddBuffer<ASGridNodesBufferComponent>(entity);
                nodes.Length = grid.NodeXCount * grid.NodeYCount;

                
                int d = grid.NodeXCount > grid.NodeYCount ? grid.NodeXCount : grid.NodeYCount;
                
                for (int x = 0; x < grid.NodeXCount; x++)
                {
                    for (int y = 0; y < grid.NodeYCount; y++)
                    {
                        // 创建默认节点（可通行，无惩罚，G/H 代价为0）
                        ASNode newNode = new ASNode()
                        {
                            X = x,
                            Y = y,
                            IsAccessible = true,
                        };

                        // 根据主轴选择索引方式（保证数组紧凑）
                        // 二维转一维
                        int index;
                        if (grid.NodeXCount > grid.NodeYCount)
                            index = x + y * d;
                        else index = y + x * d;

                        nodes[index] = new ASGridNodesBufferComponent() { Node = newNode };
                    }
                }
            }
        }
    }
}
