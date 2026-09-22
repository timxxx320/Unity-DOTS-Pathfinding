using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
//using UnsafeUtilityEx = KNN.Internal.UnsafeUtilityEx;

/// <summary>
/// KNN 查询的 Unity Job System 集成。
///
/// 本文件提供多种 Job 类型，将 KnnContainer 的查询操作封装为 Unity 的 IJob / IJobParallelForBatch，
/// 使其可以在 Job 线程中并行执行，并通过 [BurstCompile] 标签编译为高性能的原生代码（LLVM IR）。
///
/// 可用 Job 类型：
/// - QueryKNearestJob：单点 K 近邻查询（单线程 Job）
/// - QueryRangeJob：单点范围查询（单线程 Job）
/// - QueryKNearestBatchJob：批量 K 近邻查询（并行 Job，每线程处理一批点）
/// - QueryRangeBatchJob：批量范围查询（并行 Job）
/// - KnnRebuildJob：KD 树重建（单线程 Job，用于在 Job 线程中异步构建树）
/// </summary>
namespace KNN.Jobs
{
    /// <summary>
    /// KD 树重建 Job。
    ///
    /// 将 KnnContainer.Rebuild() 封装为 IJob，使 KD 树的构建可以在 Job 线程中异步执行，
    /// 避免在主线程阻塞。
    ///
    /// 通常在以下场景使用：
    /// - 点集发生变化（如障碍物移动、动态点云更新）后，需要重建 KD 树索引。
    /// - 初始化时若不想立即同步构建（KnnContainer 构造函数 buildNow=false），
    ///   可通过此 Job 异步构建。
    ///
    /// 使用方式：
    /// <code>
    /// var container = new KnnContainer(points, false, Allocator.Persistent);
    /// var rebuildJob = new KnnRebuildJob(container);
    /// JobHandle handle = rebuildJob.Schedule();
    /// // ... 可在此期间做其他事 ...
    /// handle.Complete(); // 等待重建完成
    /// </code>
    /// </summary>
    [BurstCompile(CompileSynchronously = true)]
    public struct KnnRebuildJob : IJob {
        /// <summary>
        /// 待重建的 KD 树容器（需要写入访问，因为 Rebuild 会修改内部节点数据）。
        /// </summary>
        KnnContainer m_container;

        /// <summary>
        /// 构造函数。
        /// </summary>
        /// <param name="container">需要重建的 KNN 容器</param>
        public KnnRebuildJob(KnnContainer container) {
            m_container = container;
        }

        /// <summary>
        /// Job 执行入口，调用 KnnContainer.Rebuild() 重建 KD 树。
        /// </summary>
        void IJob.Execute() {
            m_container.Rebuild();
        }
    }

    /// <summary>
    /// 为一组查询位置并行执行 K 近邻查询。
    /// 结果采用扁平布局：results[queryIndex * K + neighbourIndex]。
    /// </summary>
    [BurstCompile(CompileSynchronously = true)]
    public struct QueryKNearestBatchJob : IJobParallelForBatch
    {
        [ReadOnly] private KnnContainer _container;
        [ReadOnly] private NativeSlice<float3> _queryPositions;

        // 不同批次只写各自不重叠的切片，但 Unity 无法静态推断这一点。
        [NativeDisableParallelForRestriction]
        private NativeSlice<int> _results;

        private int _neighbourCount;

        public QueryKNearestBatchJob(
            KnnContainer container,
            NativeArray<float3> queryPositions,
            NativeSlice<int> results)
        {
            _container = container;
            _queryPositions = queryPositions;
            _results = results;
            _neighbourCount = queryPositions.Length == 0
                ? 0
                : results.Length / queryPositions.Length;
        }

        public void Execute(int startIndex, int count)
        {
            for (int index = startIndex; index < startIndex + count; index++)
            {
                NativeSlice<int> resultSlice =
                    _results.Slice(index * _neighbourCount, _neighbourCount);
                _container.QueryKNearest(_queryPositions[index], resultSlice);
            }
        }
    }
}
