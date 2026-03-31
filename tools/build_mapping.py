"""
build_mapping.py — 三阶段点云配准，为 MDD/PC2 点云和参考模型建立顶点对应表
阶段 1：包围盒对齐（处理坐标系 + 缩放差异）
阶段 2：FGR 全局配准 + Point-to-Plane ICP（Open3D，刚性精配准）
阶段 3：CPD 非刚性配准（probreg，处理姿势差异）

依赖：
    C:\\Users\\Tianhao\\pc-tools\\Scripts\\pip install open3d numpy scipy probreg

用法：
    C:\\Users\\Tianhao\\pc-tools\\Scripts\\python build_mapping.py ^
        --export-dir  <Unity 导出目录> ^
        --mdd         <.mdd 或 .pc2 文件路径（可选）> ^
        [--frame      <帧索引，默认 0>] ^
        [--beta       <CPD 平滑度，默认 2.0>] ^
        [--lam        <CPD 正则强度，默认 50.0>] ^
        [--no-cpd     跳过 CPD，只做刚性配准]
"""

import argparse
import struct
import sys
import numpy as np
from pathlib import Path


# ─── 文件读取 ─────────────────────────────────────────────────────────────────

def load_bin(path):
    """读取 Unity MappingTool 导出的 bin（大端序）"""
    with open(path, 'rb') as f:
        count = struct.unpack('>i', f.read(4))[0]
        data  = np.frombuffer(f.read(count * 12), dtype='>f4')
    return data.reshape(-1, 3).astype(np.float64), count


def load_mdd_frame(path, frame_index=0):
    """读取 MDD 文件指定帧（大端序）"""
    with open(path, 'rb') as f:
        frame_count = struct.unpack('>i', f.read(4))[0]
        point_count = struct.unpack('>i', f.read(4))[0]
        print(f"  MDD: {frame_count} 帧 × {point_count} 点")
        f.seek(frame_count * 4, 1)
        f.seek(frame_index * point_count * 12, 1)
        pts = np.frombuffer(f.read(point_count * 12), dtype='>f4')
    return pts.reshape(-1, 3).astype(np.float64)


def load_pc2_frame(path, frame_index=0):
    """读取 PC2 文件指定帧（小端序）"""
    with open(path, 'rb') as f:
        magic = f.read(12).decode('ascii').rstrip('\x00')
        assert magic == 'POINTCACHE2', f"非 PC2 文件: {magic}"
        version    = struct.unpack('<i', f.read(4))[0]
        num_points = struct.unpack('<i', f.read(4))[0]
        start_frame= struct.unpack('<f', f.read(4))[0]
        sample_rate= struct.unpack('<f', f.read(4))[0]
        num_samples= struct.unpack('<i', f.read(4))[0]
        print(f"  PC2: {num_samples} 帧 × {num_points} 点  startFrame={start_frame}")
        f.seek(frame_index * num_points * 12, 1)
        pts = np.frombuffer(f.read(num_points * 12), dtype='<f4')
    return pts.reshape(-1, 3).astype(np.float64)


# ─── 坐标变换 ─────────────────────────────────────────────────────────────────

def blender_to_unity(pts):
    """Blender Z-up → Unity Y-up：(-x, z, y)"""
    return np.stack([-pts[:, 0], pts[:, 2], pts[:, 1]], axis=1)

def unity_to_match(pts):
    """Unity 空间 → 匹配空间：(-x, z, -y)"""
    return np.stack([-pts[:, 0], pts[:, 2], -pts[:, 1]], axis=1)


# ─── 阶段 1：包围盒对齐 ───────────────────────────────────────────────────────

def bbox_align(source, target):
    src_c = source.mean(axis=0)
    tgt_c = target.mean(axis=0)
    src_s = source.max(axis=0) - source.min(axis=0)
    tgt_s = target.max(axis=0) - target.min(axis=0)
    scale = np.where(src_s > 1e-6, tgt_s / src_s, 1.0)
    print(f"  逐轴缩放: sx={scale[0]:.4f} sy={scale[1]:.4f} sz={scale[2]:.4f}")
    aligned = (source - src_c) * scale + tgt_c
    return aligned


# ─── 阶段 2：FGR + ICP（Open3D）─────────────────────────────────────────────

def to_o3d(pts, voxel=None):
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    if voxel:
        pcd = pcd.voxel_down_sample(voxel)
    return pcd

def compute_fpfh(pcd, voxel):
    import open3d as o3d
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100))
    return fpfh

def rigid_align_o3d(source, target):
    """FGR 全局配准 → Point-to-Plane ICP 精配准"""
    import open3d as o3d

    diag = np.linalg.norm(target.max(axis=0) - target.min(axis=0))
    voxel = diag * 0.02   # 下采样体素大小 = 对角线 2%

    # 下采样 + FPFH 特征
    src_ds = to_o3d(source, voxel)
    tgt_ds = to_o3d(target, voxel)
    src_fpfh = compute_fpfh(src_ds, voxel)
    tgt_fpfh = compute_fpfh(tgt_ds, voxel)

    print(f"  FGR 下采样: src={len(src_ds.points)} tgt={len(tgt_ds.points)} 点")

    # FGR 全局配准
    fgr = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        src_ds, tgt_ds, src_fpfh, tgt_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=voxel * 3))
    T = np.array(fgr.transformation)
    source_fgr = (T[:3, :3] @ source.T).T + T[:3, 3]
    d_fgr = avg_dist(source_fgr, target)
    print(f"  FGR 完成，avg_err = {d_fgr:.4f}")

    # Point-to-Plane ICP（全分辨率）
    src_pcd = to_o3d(source_fgr)
    tgt_pcd = to_o3d(target)
    tgt_pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))

    icp = o3d.pipelines.registration.registration_icp(
        src_pcd, tgt_pcd,
        max_correspondence_distance=voxel * 2,
        init=np.eye(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
    T2 = np.array(icp.transformation)
    source_icp = (T2[:3, :3] @ source_fgr.T).T + T2[:3, 3]
    d_icp = avg_dist(source_icp, target)
    print(f"  ICP 完成，avg_err = {d_icp:.4f}")

    return source_icp


# ─── 阶段 3：CPD 非刚性配准（probreg）────────────────────────────────────────

def cpd_nonrigid(source, target, beta=2.0, lam=50.0, max_iter=100):
    """
    CPD 非刚性配准
    - 对大型点云自动下采样后配准，再把变形场插值回完整点云
    - beta：平滑度（越大变形越整体，适合大姿势差异）
    - lam： 正则强度（越大越接近刚性）
    """
    from probreg import cpd
    from scipy.spatial import cKDTree

    MAX_PTS = 3000  # CPD 的计算复杂度是 O(N²)，超过此数量先下采样

    # 下采样
    if len(source) > MAX_PTS:
        idx_s = np.random.choice(len(source), MAX_PTS, replace=False)
        src_small = source[idx_s]
    else:
        idx_s = np.arange(len(source))
        src_small = source

    if len(target) > MAX_PTS:
        idx_t = np.random.choice(len(target), MAX_PTS, replace=False)
        tgt_small = target[idx_t]
    else:
        tgt_small = target

    print(f"  CPD 输入: src={len(src_small)} tgt={len(tgt_small)} 点")
    print(f"  CPD 参数: beta={beta} lam={lam} max_iter={max_iter}")

    # 多尺度 CPD：从大 beta（整体变形）到小 beta（局部细节）
    betas = [beta * 4, beta * 2, beta]
    lams  = [lam  * 4, lam  * 2, lam ]

    transformed_small = src_small.copy()
    for b, l in zip(betas, lams):
        tf_param, _, _ = cpd.registration_cpd(
            transformed_small, tgt_small,
            'nonrigid',
            beta=b, lmd=l,
            maxiter=max_iter,
            tol=1e-4)
        transformed_small = tf_param.transform(transformed_small)
        d = avg_dist(transformed_small, tgt_small)
        print(f"    beta={b:.1f} lam={l:.1f} → avg_err={d:.4f}")

    # 把变形场插值回完整点云（RBF：用下采样点的位移插值）
    displacements = transformed_small - src_small  # 下采样点的位移

    if len(source) > MAX_PTS:
        tree = cKDTree(src_small)
        dists, nn_idx = tree.query(source, k=4)
        # 距离加权插值
        weights = 1.0 / (dists + 1e-8)
        weights /= weights.sum(axis=1, keepdims=True)
        interp_disp = (weights[:, :, np.newaxis] * displacements[nn_idx]).sum(axis=1)
        transformed_full = source + interp_disp
    else:
        transformed_full = transformed_small

    d_full = avg_dist(transformed_full, target)
    print(f"  CPD 完成（完整点云插值后），avg_err = {d_full:.4f}")
    return transformed_full


# ─── 建立映射 & 导出 ──────────────────────────────────────────────────────────

def avg_dist(source, target):
    from scipy.spatial import cKDTree
    tree = cKDTree(target)
    dists, _ = tree.query(source[:min(5000, len(source))], k=1)
    return dists.mean()

def build_mapping(mdd_aligned, mesh_verts):
    from scipy.spatial import cKDTree
    tree = cKDTree(mdd_aligned)
    dists, indices = tree.query(mesh_verts, k=1)
    avg_err = dists.mean()
    max_err = dists.max()
    p95_err = np.percentile(dists, 95)
    print(f"  最终匹配误差: avg={avg_err:.4f}  p95={p95_err:.4f}  max={max_err:.4f}")
    return indices.astype(np.int32)

def write_mapping(path, mapping):
    with open(path, 'wb') as f:
        f.write(struct.pack('>i', len(mapping)))
        for idx in mapping:
            f.write(struct.pack('>i', int(idx)))
    print(f"  已写出 {path}（{len(mapping)} 条映射）")


# ─── 主流程 ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--export-dir', required=True)
    parser.add_argument('--mdd',   help='MDD 或 PC2 文件路径（不填则用 mdd_frame0.bin）')
    parser.add_argument('--frame', type=int, default=0)
    parser.add_argument('--beta',  type=float, default=2.0,  help='CPD 平滑度')
    parser.add_argument('--lam',   type=float, default=50.0, help='CPD 正则强度')
    parser.add_argument('--no-cpd', action='store_true')
    args = parser.parse_args()

    export_dir = Path(args.export_dir)
    out_path   = export_dir / 'mapping.bin'

    # ── 1. 读取数据 ──────────────────────────────────────────────────────
    print("\n[1/4] 读取数据")
    mesh_verts, _ = load_bin(export_dir / 'mesh_verts.bin')
    print(f"  Mesh 顶点数: {len(mesh_verts)}")

    if args.mdd:
        p = Path(args.mdd)
        if p.suffix.lower() == '.pc2':
            raw_pts = load_pc2_frame(p, args.frame)
            raw_pts = blender_to_unity(raw_pts)   # PC2 是 Blender 空间
        else:
            raw_pts = load_mdd_frame(p, args.frame)
            raw_pts = blender_to_unity(raw_pts)   # MDD 也是 Blender 空间
        mdd_pts = unity_to_match(raw_pts)
    else:
        mdd_pts_unity, _ = load_bin(export_dir / 'mdd_frame0.bin')
        mdd_pts = unity_to_match(mdd_pts_unity)   # 已是 Unity 空间，只做轴变换

    print(f"  MDD/PC2 点数: {len(mdd_pts)}")

    # ── 2. 包围盒对齐 ────────────────────────────────────────────────────
    print("\n[2/4] 包围盒对齐（坐标系 + 缩放）")
    mdd_aligned = bbox_align(mdd_pts, mesh_verts)
    print(f"  包围盒对齐后 avg_err = {avg_dist(mdd_aligned, mesh_verts):.4f}")

    # ── 3. FGR + ICP 刚性精配准 ─────────────────────────────────────────
    print("\n[3/4] FGR 全局配准 + Point-to-Plane ICP")
    try:
        candidate = rigid_align_o3d(mdd_aligned, mesh_verts)
        before = avg_dist(mdd_aligned, mesh_verts)
        after  = avg_dist(candidate,   mesh_verts)
        if after < before:
            mdd_aligned = candidate
        else:
            print(f"  刚性配准未改善（{before:.4f} → {after:.4f}），保留包围盒对齐")
    except Exception as e:
        print(f"  刚性配准失败: {e}")

    # ── 4. CPD 非刚性配准 ───────────────────────────────────────────────
    if not args.no_cpd:
        print(f"\n[4/4] CPD 非刚性配准（beta={args.beta} lam={args.lam}）")
        try:
            candidate = cpd_nonrigid(mdd_aligned, mesh_verts,
                                     beta=args.beta, lam=args.lam)
            before = avg_dist(mdd_aligned, mesh_verts)
            after  = avg_dist(candidate,   mesh_verts)
            if after < before:
                mdd_aligned = candidate
                print(f"  CPD 采用: {before:.4f} → {after:.4f}")
            else:
                print(f"  CPD 未改善（{before:.4f} → {after:.4f}），保留刚性结果")
        except Exception as e:
            print(f"  CPD 失败: {e}")
    else:
        print("\n[4/4] 跳过 CPD")

    # ── 5. 建立映射并导出 ────────────────────────────────────────────────
    print("\n建立最近邻映射并导出")
    mapping = build_mapping(mdd_aligned, mesh_verts)
    write_mapping(out_path, mapping)
    print(f"\n完成！请在 Unity Tools > PointCloud > Mapping Tool 中导入:\n  {out_path}")


if __name__ == '__main__':
    main()