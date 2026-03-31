using UnityEngine;

namespace PointCloud
{
    /// <summary>
    /// 从 PointCloudAsset 读取逐帧点云数据，更新 Mesh 并驱动点云渲染
    /// 需要挂载在同一 GameObject 上的 MeshFilter + MeshRenderer
    /// </summary>
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public class PointCloudPlayer : MonoBehaviour
    {
        [Header("Data")]
        public PointCloudAsset asset;

        [Header("Playback")]
        public bool playOnAwake = true;
        public bool loop = true;
        [Range(0.1f, 4f)]
        public float playbackSpeed = 1f;

        [Header("State (Read Only)")]
        [SerializeField] private int currentFrameIndex;
        [SerializeField] private float currentTime;
        [SerializeField] private bool isPlaying;

        [Header("Mesh Mode")]
        [Tooltip("开启后每帧重算法线（画质更好但有性能开销）")]
        public bool recalculateNormals = false;

        private Mesh mesh;
        private MeshFilter meshFilter;
        private Vector3[] frameBuffer;   // 复用缓冲：点云模式=pointCount，网格模式=meshVertexCount
        private int[] pointIndices;      // MeshTopology.Points 索引（点云模式）
        private Vector3[] mddScratchA;   // 网格模式帧缓冲 A（MDD 点数大小）
        private Vector3[] mddScratchB;   // 网格模式帧缓冲 B（用于插值）
        private int cachedFrameA = -1;   // 上一帧已读入 mddScratchA 的帧索引
        private int cachedFrameB = -1;   // 上一帧已读入 mddScratchB 的帧索引

        void Awake()
        {
            meshFilter = GetComponent<MeshFilter>();
        }

        void Start()
        {
            if (asset == null)
            {
                Debug.LogWarning("[PointCloudPlayer] No PointCloudAsset assigned.");
                return;
            }

            InitMesh();

            if (playOnAwake) isPlaying = true;
        }

        void Update()
        {
            if (!isPlaying || asset == null) return;

            currentTime += Time.deltaTime * playbackSpeed;

            if (loop)
            {
                float duration = asset.Duration;
                if (duration > 0f)
                    currentTime %= duration;
            }
            else
            {
                currentTime = Mathf.Clamp(currentTime, 0f, asset.Duration);
                if (currentTime >= asset.Duration)
                    isPlaying = false;
            }

            UpdateMesh();
        }

        // --- 公共控制接口 ---

        public void Play() => isPlaying = true;

        public void Pause() => isPlaying = false;

        public void Stop()
        {
            isPlaying = false;
            currentTime = 0f;
            if (asset != null) UpdateMesh();
        }

        public void SeekToTime(float time)
        {
            currentTime = Mathf.Clamp(time, 0f, asset != null ? asset.Duration : 0f);
            if (asset != null) UpdateMesh();
        }

        // --- 内部方法 ---

        private void InitMesh()
        {
            mesh = new Mesh
            {
                name = asset.name,
                indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
            };
            mesh.MarkDynamic(); // 告知 Unity 该 Mesh 频繁更新，优化 GPU 上传

            if (asset.HasMeshMapping)
                InitMeshMode();
            else
                InitPointCloudMode();

            meshFilter.mesh = mesh;
            currentFrameIndex = 0;
        }

        private void InitPointCloudMode()
        {
            int count = asset.pointCount;
            frameBuffer = new Vector3[count];
            pointIndices = new int[count];
            for (int i = 0; i < count; i++) pointIndices[i] = i;

            asset.GetFrame(0, frameBuffer);
            mesh.SetVertices(frameBuffer);
            mesh.SetIndices(pointIndices, MeshTopology.Points, 0);
            mesh.RecalculateBounds();
        }

        private void InitMeshMode()
        {
            int meshVertCount = asset.meshVertexToMddPoint.Length;
            frameBuffer = new Vector3[meshVertCount];
            mddScratchA  = new Vector3[asset.pointCount];
            mddScratchB  = new Vector3[asset.pointCount];
            cachedFrameA = -1;
            cachedFrameB = -1;

            // 初始化第 0 帧
            asset.GetFrame(0, mddScratchA);
            cachedFrameA = 0;
            RemapToMesh(mddScratchA, frameBuffer);

            mesh.SetVertices(frameBuffer);
            if (asset.uvs != null && asset.uvs.Length > 0)
                mesh.SetUVs(0, asset.uvs);
            mesh.SetTriangles(asset.triangles, 0);
            mesh.RecalculateNormals(); // 仅初始化时调用一次
            mesh.RecalculateBounds();
        }

        private void UpdateMesh()
        {
            asset.GetFrameAtTime(currentTime, out int frameA, out int frameB, out float t);

            if (asset.HasMeshMapping)
            {
                // 仅在帧索引变化时才重新读取数据，避免每帧重复拷贝
                if (frameA != cachedFrameA)
                {
                    asset.GetFrame(frameA, mddScratchA);
                    cachedFrameA = frameA;
                }
                if (frameB != cachedFrameB)
                {
                    asset.GetFrame(frameB, mddScratchB);
                    cachedFrameB = frameB;
                }

                int meshVertCount = asset.meshVertexToMddPoint.Length;
                for (int i = 0; i < meshVertCount; i++)
                {
                    int mddIdx = asset.meshVertexToMddPoint[i];
                    frameBuffer[i] = Vector3.LerpUnclamped(mddScratchA[mddIdx], mddScratchB[mddIdx], t);
                }

                mesh.SetVertices(frameBuffer);
                if (recalculateNormals) mesh.RecalculateNormals();
                mesh.RecalculateBounds();
                currentFrameIndex = frameA;
            }
            else
            {
                // 点云模式：跳帧（保持原有逻辑）
                int targetFrame = t < 0.5f ? frameA : frameB;
                if (targetFrame == currentFrameIndex) return;

                currentFrameIndex = targetFrame;
                asset.GetFrame(currentFrameIndex, frameBuffer);
                mesh.SetVertices(frameBuffer);
                mesh.RecalculateBounds();
            }
        }

        private void RemapToMesh(Vector3[] mddFrame, Vector3[] output)
        {
            int[] map = asset.meshVertexToMddPoint;
            for (int i = 0; i < output.Length; i++)
                output[i] = mddFrame[map[i]];
        }

#if UNITY_EDITOR
        // 在编辑器中拖动时间轴预览
        private void OnValidate()
        {
            if (!Application.isPlaying || asset == null || mesh == null) return;
            UpdateMesh();
        }
#endif
    }
}