using Unity.Mathematics;

namespace Athomield.AStar
{
    public static class PathfindingUtility
    {
        /// <summary>把世界空间 float3 投影到 ORCA 使用的 XZ 平面。</summary>
        public static float2 V2FromV3(float3 vector)
        {
            return new float2(vector.x, vector.z);
        }
    }
}
