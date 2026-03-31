using System;
using UnityEngine;

namespace PointCloud
{
    /// <summary>
    /// 存储 MDD 点缓存动画的所有帧数据
    /// positions 是展平的一维数组：positions[frame * pointCount + point] = Vector3
    /// </summary>
    [CreateAssetMenu(fileName = "NewPointCloudAsset", menuName = "PointCloud/Point Cloud Asset")]
    public class PointCloudAsset : ScriptableObject
    {
        public int frameCount;
        public int pointCount;
        public float[] timestamps; // 每帧的时间戳（秒），长度 = frameCount

        [HideInInspector]
        public Vector3[] positions; // 展平存储：长度 = frameCount * pointCount

        // 参考模型映射（可选）：有值时启用三角面渲染
        [HideInInspector] public int[] meshVertexToMddPoint; // 长度 = 参考模型顶点数
        [HideInInspector] public int[] triangles;
        [HideInInspector] public Vector2[] uvs;

        public bool HasMeshMapping =>
            meshVertexToMddPoint != null && meshVertexToMddPoint.Length > 0 &&
            triangles != null && triangles.Length > 0;

        public float Duration => (timestamps != null && timestamps.Length > 0)
            ? timestamps[timestamps.Length - 1]
            : 0f;

        /// <summary>
        /// 将指定帧的顶点数据复制到 output 数组（避免每帧 GC 分配）
        /// </summary>
        public void GetFrame(int frameIndex, Vector3[] output)
        {
            int offset = frameIndex * pointCount;
            Array.Copy(positions, offset, output, 0, pointCount);
        }

        /// <summary>
        /// 根据时间（秒）插值获取帧索引，返回当前帧和下一帧及插值 t
        /// </summary>
        public void GetFrameAtTime(float time, out int frameA, out int frameB, out float t)
        {
            if (timestamps == null || timestamps.Length == 0)
            {
                frameA = frameB = 0;
                t = 0f;
                return;
            }

            // 二分查找当前时间所在帧段
            int lo = 0, hi = timestamps.Length - 1;
            while (lo < hi - 1)
            {
                int mid = (lo + hi) / 2;
                if (timestamps[mid] <= time) lo = mid;
                else hi = mid;
            }

            frameA = lo;
            frameB = Mathf.Min(lo + 1, timestamps.Length - 1);
            float span = timestamps[frameB] - timestamps[frameA];
            t = span > 0.0001f ? (time - timestamps[frameA]) / span : 0f;
        }
    }
}