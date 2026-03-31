using System.IO;
using UnityEngine;
using UnityEditor;

namespace PointCloud.Editor
{
    /// <summary>
    /// 辅助工具：导出/导入 Python 配准所需的数据文件
    /// 菜单：Tools > PointCloud > Mapping Tool
    /// </summary>
    public class MappingToolWindow : EditorWindow
    {
        private PointCloudAsset asset;
        private Mesh referenceMesh;
        private string exportDir = "";
        private string mappingFile = "";

        [MenuItem("Tools/PointCloud/Mapping Tool")]
        static void Open() => GetWindow<MappingToolWindow>("Mapping Tool");

        void OnGUI()
        {
            EditorGUILayout.LabelField("Python 配准工具", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            asset = (PointCloudAsset)EditorGUILayout.ObjectField(
                "PointCloudAsset", asset, typeof(PointCloudAsset), false);
            referenceMesh = (Mesh)EditorGUILayout.ObjectField(
                "参考模型 Mesh", referenceMesh, typeof(Mesh), false);

            EditorGUILayout.Space();

            // ── Step 1：导出 ────────────────────────────────────────────
            EditorGUILayout.LabelField("Step 1 — 导出数据给 Python", EditorStyles.boldLabel);

            EditorGUI.BeginDisabledGroup(string.IsNullOrEmpty(exportDir));
            EditorGUILayout.LabelField("导出目录:", string.IsNullOrEmpty(exportDir) ? "（未选择）" : exportDir,
                EditorStyles.miniLabel);
            EditorGUI.EndDisabledGroup();

            if (GUILayout.Button("选择导出目录…"))
                exportDir = EditorUtility.OpenFolderPanel("选择导出目录", exportDir, "");

            using (new EditorGUI.DisabledScope(
                asset == null || referenceMesh == null || string.IsNullOrEmpty(exportDir)))
            {
                if (GUILayout.Button("导出 mesh_verts.bin + mdd_frame0.bin"))
                    ExportData();
            }

            EditorGUILayout.Space();

            // ── Step 2：运行 Python ──────────────────────────────────────
            EditorGUILayout.LabelField("Step 2 — 运行 Python 脚本", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox(
                "cd <project>/tools\n" +
                "pip install numpy scipy open3d\n" +
                "python build_mapping.py --export-dir <导出目录> --mdd <MDD文件路径>",
                MessageType.None);

            EditorGUILayout.Space();

            // ── Step 3：导入映射结果 ────────────────────────────────────
            EditorGUILayout.LabelField("Step 3 — 导入 mapping.bin", EditorStyles.boldLabel);

            EditorGUI.BeginDisabledGroup(string.IsNullOrEmpty(mappingFile));
            EditorGUILayout.LabelField("映射文件:", string.IsNullOrEmpty(mappingFile) ? "（未选择）" : mappingFile,
                EditorStyles.miniLabel);
            EditorGUI.EndDisabledGroup();

            if (GUILayout.Button("选择 mapping.bin…"))
                mappingFile = EditorUtility.OpenFilePanel("选择 mapping.bin", exportDir, "bin");

            using (new EditorGUI.DisabledScope(
                asset == null || referenceMesh == null || string.IsNullOrEmpty(mappingFile)))
            {
                if (GUILayout.Button("导入映射并写入 PointCloudAsset"))
                    ImportMapping();
            }
        }

        // ── 导出：mesh 顶点 + MDD 第 0 帧（本地空间，float32 大端序）────────

        private void ExportData()
        {
            // mesh_verts.bin：int32 count, count × (float32 x, float32 y, float32 z)
            string meshPath = Path.Combine(exportDir, "mesh_verts.bin");
            Vector3[] verts = referenceMesh.vertices;
            using (var bw = new BinaryWriter(File.Open(meshPath, FileMode.Create)))
            {
                WriteInt32BE(bw, verts.Length);
                foreach (var v in verts)
                {
                    WriteFloat32BE(bw, v.x);
                    WriteFloat32BE(bw, v.y);
                    WriteFloat32BE(bw, v.z);
                }
            }

            // mdd_frame0.bin：int32 count, count × (float32 x, float32 y, float32 z)
            string mddPath = Path.Combine(exportDir, "mdd_frame0.bin");
            Vector3[] frame0 = new Vector3[asset.pointCount];
            asset.GetFrame(0, frame0);
            using (var bw = new BinaryWriter(File.Open(mddPath, FileMode.Create)))
            {
                WriteInt32BE(bw, frame0.Length);
                foreach (var p in frame0)
                {
                    WriteFloat32BE(bw, p.x);
                    WriteFloat32BE(bw, p.y);
                    WriteFloat32BE(bw, p.z);
                }
            }

            Debug.Log($"[MappingTool] 已导出:\n  {meshPath}\n  {mddPath}");
            EditorUtility.DisplayDialog("导出完成",
                $"已写入:\n{meshPath}\n{mddPath}\n\n请运行 tools/build_mapping.py", "OK");
        }

        // ── 导入：读取 mapping.bin，写入 PointCloudAsset ─────────────────

        private void ImportMapping()
        {
            using (var br = new BinaryReader(File.OpenRead(mappingFile)))
            {
                int meshVertCount = ReadInt32BE(br);
                if (meshVertCount != referenceMesh.vertexCount)
                {
                    EditorUtility.DisplayDialog("错误",
                        $"mapping.bin 顶点数 ({meshVertCount}) 与参考模型 ({referenceMesh.vertexCount}) 不匹配", "OK");
                    return;
                }

                int[] mapping = new int[meshVertCount];
                for (int i = 0; i < meshVertCount; i++)
                    mapping[i] = ReadInt32BE(br);

                // 写入 asset（需要标记 dirty 并保存）
                asset.meshVertexToMddPoint = mapping;
                asset.triangles = referenceMesh.triangles;
                asset.uvs = referenceMesh.uv;
                EditorUtility.SetDirty(asset);
                AssetDatabase.SaveAssets();

                Debug.Log($"[MappingTool] 映射导入成功，共 {meshVertCount} 个顶点");
                EditorUtility.DisplayDialog("导入完成",
                    $"已将 {meshVertCount} 个顶点映射写入 PointCloudAsset", "OK");
            }
        }

        // ── 大端序辅助 ───────────────────────────────────────────────────

        private static void WriteInt32BE(BinaryWriter bw, int v)
        {
            var b = System.BitConverter.GetBytes(v);
            if (System.BitConverter.IsLittleEndian) System.Array.Reverse(b);
            bw.Write(b);
        }

        private static void WriteFloat32BE(BinaryWriter bw, float v)
        {
            var b = System.BitConverter.GetBytes(v);
            if (System.BitConverter.IsLittleEndian) System.Array.Reverse(b);
            bw.Write(b);
        }

        private static int ReadInt32BE(BinaryReader br)
        {
            var b = br.ReadBytes(4);
            if (System.BitConverter.IsLittleEndian) System.Array.Reverse(b);
            return System.BitConverter.ToInt32(b, 0);
        }
    }
}