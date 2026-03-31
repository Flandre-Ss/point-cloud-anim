using System;
using System.IO;
using UnityEditor;
using UnityEditor.AssetImporters;
using UnityEngine;

namespace PointCloud.Editor
{
    /// <summary>
    /// PC2 点缓存文件导入器（Point Cache 2，小端字节序）
    /// PC2 格式：
    ///   Header : char[12] "POINTCACHE2\0", int32 version=1, int32 numPoints,
    ///            float32 startFrame, float32 sampleRate, int32 numSamples
    ///   Positions : numSamples × numPoints × float32 × 3（X Y Z，小端序）
    /// </summary>
    [ScriptedImporter(1, "pc2")]
    public class PC2Importer : ScriptedImporter
    {
        public enum CoordConversion
        {
            None,
            BlenderToUnity  // X→-X, Y→Z, Z→Y（Blender Z-up → Unity Y-up）
        }

        [Header("Import Settings")]
        public CoordConversion coordinateConversion = CoordConversion.BlenderToUnity;
        public float scale = 1f;

        [Tooltip("每秒帧数，用于将帧编号转换为时间戳（秒）")]
        public float frameRate = 24f;

        [Tooltip("将第 0 帧作为绑定姿势：用于参考模型映射，不参与播放")]
        public bool useFrame0AsBindPose = true;

        [Header("Reference Mesh Mapping (Optional)")]
        [Tooltip("与 PC2 对应的参考模型 Mesh，指定后启用三角面渲染")]
        public Mesh referenceMesh;

        [Header("File Info (Read Only)")]
        public int   totalFrames;
        public int   totalPoints;
        public float fileStartFrame;
        public float fileSampleRate;
        public float mappingMaxError = -1f;

        public override void OnImportAsset(AssetImportContext ctx)
        {
            PointCloudAsset asset = ScriptableObject.CreateInstance<PointCloudAsset>();
            asset.name = Path.GetFileNameWithoutExtension(ctx.assetPath);

            if (!ParsePC2(ctx.assetPath, asset))
            {
                Debug.LogError($"[PC2Importer] Failed to parse: {ctx.assetPath}");
                ctx.AddObjectToAsset("asset", asset);
                ctx.SetMainObject(asset);
                return;
            }

            totalFrames   = asset.frameCount;
            totalPoints   = asset.pointCount;

            if (referenceMesh != null)
            {
                // 若有绑定帧，用绑定帧配准；否则用第 0 帧
                int bindFrame = (useFrame0AsBindPose && asset.frameCount > 1) ? 0 : 0;
                mappingMaxError = PointCloudMapBuilder.Build(asset, referenceMesh, bindFrame);

                // 绑定帧不参与播放：把 positions 裁掉第 0 帧
                if (useFrame0AsBindPose && asset.frameCount > 1)
                    TrimBindFrame(asset);
            }

            ctx.AddObjectToAsset("asset", asset);
            ctx.SetMainObject(asset);

            Debug.Log($"[PC2Importer] Imported {asset.frameCount} frames × {asset.pointCount} points  " +
                      $"startFrame={fileStartFrame}  sampleRate={fileSampleRate}");
        }

        private bool ParsePC2(string path, PointCloudAsset asset)
        {
            try
            {
                using (var reader = new BinaryReader(File.OpenRead(path)))
                {
                    // ── 文件头 ──────────────────────────────────────────────
                    string magic = new string(reader.ReadChars(12)).TrimEnd('\0');
                    if (magic != "POINTCACHE2")
                    {
                        Debug.LogError($"[PC2Importer] 无效的文件头（期望 POINTCACHE2，得到 {magic}）");
                        return false;
                    }

                    int   version    = ReadLEInt(reader);
                    int   numPoints  = ReadLEInt(reader);
                    float startFrame = ReadLEFloat(reader);
                    float sampleRate = ReadLEFloat(reader);
                    int   numSamples = ReadLEInt(reader);

                    if (numPoints <= 0 || numSamples <= 0)
                    {
                        Debug.LogError($"[PC2Importer] 无效数据：points={numPoints}, samples={numSamples}");
                        return false;
                    }

                    fileStartFrame = startFrame;
                    fileSampleRate = sampleRate;

                    // ── 时间戳：frame_i = startFrame + i / sampleRate ────────
                    float safeSampleRate = sampleRate > 0f ? sampleRate : 1f;
                    float safeFrameRate  = frameRate  > 0f ? frameRate  : 24f;

                    float[] timestamps = new float[numSamples];
                    for (int i = 0; i < numSamples; i++)
                        timestamps[i] = (startFrame + i / safeSampleRate) / safeFrameRate;

                    // 归零（从 0 开始）
                    float t0 = timestamps[0];
                    for (int i = 0; i < numSamples; i++)
                        timestamps[i] -= t0;

                    // ── 位置数据 ─────────────────────────────────────────────
                    Vector3[] positions = new Vector3[numSamples * numPoints];

                    for (int frame = 0; frame < numSamples; frame++)
                    {
                        int destOffset = frame * numPoints;
                        for (int point = 0; point < numPoints; point++)
                        {
                            float x = ReadLEFloat(reader);
                            float y = ReadLEFloat(reader);
                            float z = ReadLEFloat(reader);
                            positions[destOffset + point] = ApplyConversion(x, y, z) * scale;
                        }
                    }

                    asset.frameCount  = numSamples;
                    asset.pointCount  = numPoints;
                    asset.timestamps  = timestamps;
                    asset.positions   = positions;
                }
                return true;
            }
            catch (Exception e)
            {
                Debug.LogError($"[PC2Importer] Exception: {e.Message}\n{e.StackTrace}");
                return false;
            }
        }

        /// <summary>
        /// 将第 0 帧（绑定姿势）从播放数据中移除，使动画从第 1 帧开始
        /// </summary>
        private static void TrimBindFrame(PointCloudAsset asset)
        {
            int n = asset.pointCount;
            int newCount = asset.frameCount - 1;

            float[] newTimestamps = new float[newCount];
            Vector3[] newPositions = new Vector3[newCount * n];

            float offset = asset.timestamps[1];
            for (int i = 0; i < newCount; i++)
            {
                newTimestamps[i] = asset.timestamps[i + 1] - offset;
                Array.Copy(asset.positions, (i + 1) * n, newPositions, i * n, n);
            }

            asset.frameCount = newCount;
            asset.timestamps = newTimestamps;
            asset.positions  = newPositions;
        }

        private Vector3 ApplyConversion(float x, float y, float z) =>
            coordinateConversion switch
            {
                CoordConversion.BlenderToUnity => new Vector3(-x, z, y),
                _ => new Vector3(x, y, z),
            };

        // ── 小端序辅助 ────────────────────────────────────────────────────────

        private static int ReadLEInt(BinaryReader reader)
        {
            byte[] b = reader.ReadBytes(4);
            if (!BitConverter.IsLittleEndian) Array.Reverse(b);
            return BitConverter.ToInt32(b, 0);
        }

        private static float ReadLEFloat(BinaryReader reader)
        {
            byte[] b = reader.ReadBytes(4);
            if (!BitConverter.IsLittleEndian) Array.Reverse(b);
            return BitConverter.ToSingle(b, 0);
        }
    }

    [CustomEditor(typeof(PC2Importer))]
    public class PC2ImporterEditor : ScriptedImporterEditor
    {
        public override void OnInspectorGUI()
        {
            var importer = (PC2Importer)target;

            EditorGUILayout.LabelField("File Info", EditorStyles.boldLabel);
            EditorGUI.BeginDisabledGroup(true);
            EditorGUILayout.IntField("Total Frames",   importer.totalFrames);
            EditorGUILayout.IntField("Total Points",   importer.totalPoints);
            EditorGUILayout.FloatField("Start Frame",  importer.fileStartFrame);
            EditorGUILayout.FloatField("Sample Rate",  importer.fileSampleRate);
            EditorGUI.EndDisabledGroup();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Import Settings", EditorStyles.boldLabel);

            serializedObject.Update();
            EditorGUILayout.PropertyField(serializedObject.FindProperty("coordinateConversion"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("scale"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("frameRate"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("useFrame0AsBindPose"));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Reference Mesh Mapping", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("referenceMesh"));

            if (importer.mappingMaxError >= 0f)
            {
                EditorGUI.BeginDisabledGroup(true);
                EditorGUILayout.FloatField("Mapping Max Error", importer.mappingMaxError);
                EditorGUI.EndDisabledGroup();
                if (importer.mappingMaxError > 5f)
                    EditorGUILayout.HelpBox(
                        "最大误差较大。若已启用 useFrame0AsBindPose，请确认 PC2 第 0 帧是 T-pose / 绑定姿势。",
                        MessageType.Warning);
            }

            serializedObject.ApplyModifiedProperties();

            EditorGUILayout.Space();
            ApplyRevertGUI();
        }
    }
}