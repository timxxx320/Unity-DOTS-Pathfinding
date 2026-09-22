using Unity.Mathematics;

namespace Athomield.AStar
{
    /// <summary>
    /// ORCA 速度空间中的一条半平面约束线。
    /// 合法速度满足 det(Direction, velocity - Point) &gt;= 0。
    /// </summary>
    public struct ASLine
    {
        public float2 Direction;
        public float2 Point;
    }
}
