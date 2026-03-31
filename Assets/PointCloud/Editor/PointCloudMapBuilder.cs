using System.Collections.Generic;
using UnityEngine;

namespace PointCloud.Editor
{
    /// <summary>
    /// 共享工具：为 PointCloudAsset 建立参考模型顶点映射表
    /// MDDImporter 和 PC2Importer 均调用此类
    /// </summary>
    internal static class PointCloudMapBuilder
    {
        /// <summary>
        /// 建立映射：对每个 Mesh 顶点找最近的 MDD/PC2 点（匹配空间中）
        /// 变换链：Unity空间 → (-x,z,-y) → 质心对齐 → 逐轴缩放
        /// 返回最大匹配误差
        /// </summary>
        internal static float Build(PointCloudAsset asset, Mesh refMesh, int bindFrame = 0)
        {
            // 1. 读取绑定帧
            Vector3[] frame = new Vector3[asset.pointCount];
            asset.GetFrame(bindFrame, frame);

            // 2. 轴变换
            Vector3[] mddMatch = new Vector3[asset.pointCount];
            for (int i = 0; i < asset.pointCount; i++)
                mddMatch[i] = new Vector3(-frame[i].x, frame[i].z, -frame[i].y);

            // 3. 质心对齐
            Vector3[] meshVerts = refMesh.vertices;
            Vector3 mddCenter  = Centroid(mddMatch);
            Vector3 meshCenter = Centroid(meshVerts);
            for (int i = 0; i < mddMatch.Length; i++)
                mddMatch[i] += meshCenter - mddCenter;

            // 4. 逐轴缩放（包围盒自动计算）
            Bounds mddBounds  = CalcBounds(mddMatch);
            Bounds meshBounds = CalcBounds(meshVerts);
            float sx = mddBounds.size.x > 0.001f ? meshBounds.size.x / mddBounds.size.x : 1f;
            float sy = mddBounds.size.y > 0.001f ? meshBounds.size.y / mddBounds.size.y : 1f;
            float sz = mddBounds.size.z > 0.001f ? meshBounds.size.z / mddBounds.size.z : 1f;
            for (int i = 0; i < mddMatch.Length; i++)
            {
                Vector3 p = mddMatch[i] - meshCenter;
                mddMatch[i] = meshCenter + new Vector3(p.x * sx, p.y * sy, p.z * sz);
            }
            Debug.Log($"[MapBuilder] 逐轴缩放: sx={sx:F4} sy={sy:F4} sz={sz:F4}");

            // 5. 格子加速最近邻搜索
            float cellSize = EstimateCellSize(mddMatch);
            var grid = BuildGrid(mddMatch, cellSize);

            int meshVertCount = meshVerts.Length;
            int[] mapping = new int[meshVertCount];
            float maxErr = 0f, sumErr = 0f;

            for (int j = 0; j < meshVertCount; j++)
            {
                int best = FindNearest(meshVerts[j], mddMatch, grid, cellSize);
                mapping[j] = best;
                float err = Vector3.Distance(meshVerts[j], mddMatch[best]);
                sumErr += err;
                if (err > maxErr) maxErr = err;
            }

            float avgErr = sumErr / meshVertCount;
            asset.meshVertexToMddPoint = mapping;
            asset.triangles = refMesh.triangles;
            asset.uvs       = refMesh.uv;

            Debug.Log($"[MapBuilder] 绑定帧={bindFrame}  avg={avgErr:F4}  max={maxErr:F4}");
            return maxErr;
        }

        // ── 内部工具 ─────────────────────────────────────────────────────────

        private static Vector3 Centroid(Vector3[] pts)
        {
            Vector3 sum = Vector3.zero;
            foreach (var p in pts) sum += p;
            return sum / pts.Length;
        }

        private static Bounds CalcBounds(Vector3[] pts)
        {
            Vector3 min = pts[0], max = pts[0];
            foreach (var p in pts) { min = Vector3.Min(min, p); max = Vector3.Max(max, p); }
            var b = new Bounds();
            b.SetMinMax(min, max);
            return b;
        }

        private static float EstimateCellSize(Vector3[] pts)
        {
            Vector3 min = pts[0], max = pts[0];
            foreach (var p in pts) { min = Vector3.Min(min, p); max = Vector3.Max(max, p); }
            float vol = (max.x - min.x) * (max.y - min.y) * (max.z - min.z);
            float targetCells = pts.Length / 8f;
            return Mathf.Max(1f, Mathf.Pow(vol / targetCells, 1f / 3f));
        }

        private static Dictionary<Vector3Int, List<int>> BuildGrid(Vector3[] pts, float cellSize)
        {
            var grid = new Dictionary<Vector3Int, List<int>>();
            for (int i = 0; i < pts.Length; i++)
            {
                var key = ToCell(pts[i], cellSize);
                if (!grid.TryGetValue(key, out var list))
                    grid[key] = list = new List<int>();
                list.Add(i);
            }
            return grid;
        }

        private static int FindNearest(Vector3 query, Vector3[] pts,
            Dictionary<Vector3Int, List<int>> grid, float cellSize)
        {
            var center = ToCell(query, cellSize);
            int bestIdx = 0;
            float bestSq = float.MaxValue;

            for (int radius = 0; radius <= 8; radius++)
            {
                if (radius > 1 && bestSq < (radius - 1) * cellSize * ((radius - 1) * cellSize))
                    break;

                for (int dx = -radius; dx <= radius; dx++)
                for (int dy = -radius; dy <= radius; dy++)
                for (int dz = -radius; dz <= radius; dz++)
                {
                    if (Mathf.Abs(dx) < radius && Mathf.Abs(dy) < radius && Mathf.Abs(dz) < radius)
                        continue;
                    var key = new Vector3Int(center.x + dx, center.y + dy, center.z + dz);
                    if (!grid.TryGetValue(key, out var list)) continue;
                    foreach (int i in list)
                    {
                        float sq = (pts[i] - query).sqrMagnitude;
                        if (sq < bestSq) { bestSq = sq; bestIdx = i; }
                    }
                }
            }
            return bestIdx;
        }

        private static Vector3Int ToCell(Vector3 p, float cellSize) =>
            new Vector3Int(
                Mathf.FloorToInt(p.x / cellSize),
                Mathf.FloorToInt(p.y / cellSize),
                Mathf.FloorToInt(p.z / cellSize));
    }
}