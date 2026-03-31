using System;
using System.IO;
using UnityEditor;
using UnityEditor.AssetImporters;
using UnityEngine;

namespace PointCloud.Editor
{
    /// <summary>
    /// MDD 点缓存文件导入器
    /// MDD 格式（大端字节序）：
    ///   Header : int32 frameCount, int32 pointCount
    ///   Timestamps : frameCount × float32（每帧时间，秒）
    ///   Positions  : frameCount × pointCount × float32 × 3（X Y Z）
    /// </summary>
    [ScriptedImporter(1, "mdd")]
    public class MDDImporter : ScriptedImporter
    {
        public enum CoordConversion
        {
            None,
            BlenderToUnity  // X→-X, Y→Z, Z→Y（Blender Z-up → Unity Y-up）
        }

        [Header("Import Settings")]
        public CoordConversion coordinateConversion = CoordConversion.BlenderToUnity;
        public float scale = 1f;

        [Tooltip("跳过第一帧（MDD 导出时第 0 帧有时是 T-pose 基础帧）")]
        public bool skipFrame1 = false;

        [Header("Reference Mesh Mapping (Optional)")]
        [Tooltip("与 MDD 对应的参考模型 Mesh。指定后将建立顶点映射，启用三角面渲染")]
        public Mesh referenceMesh;

        [Tooltip("MDD X 轴缩放补偿（由诊断工具测定，默认 2.238）")]
        public float xScaleCorrection = 2.238f;

        // 只读信息，导入后填充
        [Header("File Info (Read Only)")]
        public int totalFrames;
        public int totalPoints;
        public float mappingMaxError = -1f;

        public override void OnImportAsset(AssetImportContext ctx)
        {
            PointCloudAsset asset = ScriptableObject.CreateInstance<PointCloudAsset>();
            asset.name = Path.GetFileNameWithoutExtension(ctx.assetPath);

            if (!ParseMDD(ctx.assetPath, asset))
            {
                Debug.LogError($"[MDDImporter] Failed to parse: {ctx.assetPath}");
                ctx.AddObjectToAsset("asset", asset);
                ctx.SetMainObject(asset);
                return;
            }

            totalFrames = asset.frameCount;
            totalPoints = asset.pointCount;

            if (referenceMesh != null)
                mappingMaxError = PointCloudMapBuilder.Build(asset, referenceMesh);

            ctx.AddObjectToAsset("asset", asset);
            ctx.SetMainObject(asset);

            Debug.Log($"[MDDImporter] Imported {asset.frameCount} frames × {asset.pointCount} points from {Path.GetFileName(ctx.assetPath)}");
        }

        private bool ParseMDD(string path, PointCloudAsset asset)
        {
            try
            {
                using (BinaryReader reader = new BinaryReader(File.OpenRead(path)))
                {
                    // --- Header ---
                    int frameCount = ReadBEInt(reader);
                    int pointCount = ReadBEInt(reader);

                    if (frameCount <= 0 || pointCount <= 0)
                    {
                        Debug.LogError($"[MDDImporter] Invalid header: frames={frameCount}, points={pointCount}");
                        return false;
                    }

                    // --- Timestamps ---
                    float[] timestamps = new float[frameCount];
                    for (int i = 0; i < frameCount; i++)
                    {
                        timestamps[i] = ReadBEFloat(reader);
                    }

                    // 跳过第一帧时，从 index=1 开始
                    int startFrame = skipFrame1 ? 1 : 0;
                    int usedFrames = frameCount - startFrame;

                    // 调整时间戳，从 0 开始
                    float[] usedTimestamps = new float[usedFrames];
                    float timeOffset = timestamps[startFrame];
                    for (int i = 0; i < usedFrames; i++)
                    {
                        usedTimestamps[i] = timestamps[startFrame + i] - timeOffset;
                    }

                    // --- Positions ---
                    Vector3[] positions = new Vector3[usedFrames * pointCount];

                    for (int frame = 0; frame < frameCount; frame++)
                    {
                        bool useFrame = frame >= startFrame;
                        int destFrame = frame - startFrame;

                        for (int point = 0; point < pointCount; point++)
                        {
                            float x = ReadBEFloat(reader);
                            float y = ReadBEFloat(reader);
                            float z = ReadBEFloat(reader);

                            if (!useFrame) continue;

                            Vector3 pos = ApplyConversion(x, y, z) * scale;
                            positions[destFrame * pointCount + point] = pos;
                        }
                    }

                    asset.frameCount = usedFrames;
                    asset.pointCount = pointCount;
                    asset.timestamps = usedTimestamps;
                    asset.positions = positions;
                }
                return true;
            }
            catch (Exception e)
            {
                Debug.LogError($"[MDDImporter] Exception: {e.Message}\n{e.StackTrace}");
                return false;
            }
        }


        private Vector3 ApplyConversion(float x, float y, float z)
        {
            return coordinateConversion switch
            {
                CoordConversion.BlenderToUnity => new Vector3(-x, z, y),
                _ => new Vector3(x, y, z),
            };
        }

        // --- 大端字节序辅助方法 ---

        private static int ReadBEInt(BinaryReader reader)
        {
            byte[] bytes = reader.ReadBytes(4);
            if (BitConverter.IsLittleEndian) Array.Reverse(bytes);
            return BitConverter.ToInt32(bytes, 0);
        }

        private static float ReadBEFloat(BinaryReader reader)
        {
            byte[] bytes = reader.ReadBytes(4);
            if (BitConverter.IsLittleEndian) Array.Reverse(bytes);
            return BitConverter.ToSingle(bytes, 0);
        }
    }

    [CustomEditor(typeof(MDDImporter))]
    public class MDDImporterEditor : ScriptedImporterEditor
    {
        public override void OnInspectorGUI()
        {
            var importer = (MDDImporter)target;

            EditorGUILayout.LabelField("File Info", EditorStyles.boldLabel);
            EditorGUI.BeginDisabledGroup(true);
            EditorGUILayout.IntField("Total Frames", importer.totalFrames);
            EditorGUILayout.IntField("Total Points", importer.totalPoints);
            EditorGUI.EndDisabledGroup();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Import Settings", EditorStyles.boldLabel);

            serializedObject.Update();
            EditorGUILayout.PropertyField(serializedObject.FindProperty("coordinateConversion"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("scale"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("skipFrame1"));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Reference Mesh Mapping", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("referenceMesh"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("xScaleCorrection"));

            if (importer.mappingMaxError >= 0f)
            {
                EditorGUI.BeginDisabledGroup(true);
                EditorGUILayout.FloatField("Mapping Max Error", importer.mappingMaxError);
                EditorGUI.EndDisabledGroup();
                if (importer.mappingMaxError > 5f)
                    EditorGUILayout.HelpBox("最大误差较大，建议检查 xScaleCorrection 或确认 Blender 对象 Scale 已 Apply。", MessageType.Warning);
            }

            serializedObject.ApplyModifiedProperties();

            EditorGUILayout.Space();
            ApplyRevertGUI();
        }
    }
}