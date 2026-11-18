# -*- coding: utf-8 -*-
"""分析 diff_chassis 模型：STL 包围盒、惯量一致性、结构缺口。"""

import os
import struct

MESH_DIR = r"D:\git\机器人\diff_chassis\meshes"


def read_stl_bounds(path):
    """返回 (min_xyz, max_xyz, 三角形数, 格式)。"""
    with open(path, "rb") as f:
        head = f.read(84)
        if head[:5] == b"solid" and len(head) >= 84:
            # 可能是 ASCII；再判断文件长度是否匹配二进制布局
            pass
        n_tri = struct.unpack("<I", head[80:84])[0]
        size = os.path.getsize(path)
        if 84 + n_tri * 50 == size:
            fmt = "binary"
            mins = [1e30] * 3
            maxs = [-1e30] * 3
            for i in range(n_tri):
                rec = f.read(50)
                vals = struct.unpack("<12f", rec[:48])
                # 跳过法向量，取三个顶点
                for v in range(3):
                    x, y, z = vals[3 + v * 3: 6 + v * 3]
                    for k, c in enumerate((x, y, z)):
                        if c < mins[k]:
                            mins[k] = c
                        if c > maxs[k]:
                            maxs[k] = c
            return mins, maxs, n_tri, fmt
        # ASCII
        f.seek(0)
        text = f.read().decode("ascii", "ignore")
        mins = [1e30] * 3
        maxs = [-1e30] * 3
        cnt = 0
        for line in text.splitlines():
            line = line.strip()
            if line.startswith("vertex"):
                cnt += 1
                parts = line.split()
                if len(parts) >= 4:
                    for k in range(3):
                        c = float(parts[1 + k])
                        if c < mins[k]:
                            mins[k] = c
                        if c > maxs[k]:
                            maxs[k] = c
        return mins, maxs, cnt // 3, "ascii"


def main():
    print(f"{'mesh':22s} {'size (mm) X x Y x Z':34s} {'min z':>8s} {'tris':>7s}")
    print("-" * 78)
    for name in sorted(os.listdir(MESH_DIR)):
        if not name.lower().endswith(".stl"):
            continue
        p = os.path.join(MESH_DIR, name)
        mins, maxs, n, fmt = read_stl_bounds(p)
        size = [maxs[i] - mins[i] for i in range(3)]
        print(f"{name:22s} "
              f"{size[0]*1000:8.1f} x {size[1]*1000:8.1f} x {size[2]*1000:8.1f}     "
              f"{mins[2]*1000:8.2f} {n:7d}  {fmt}")

    # 车轮半径由包围盒推断
    for name in ("left_wheel_link.STL", "right_wheel_link.STL"):
        p = os.path.join(MESH_DIR, name)
        if os.path.exists(p):
            mins, maxs, _, _ = read_stl_bounds(p)
            dy = (maxs[1] - mins[1]) / 2
            dz = (maxs[2] - mins[2]) / 2
            print(f"\n{name}: 几何半径 y={dy*1000:.2f} mm, z={dz*1000:.2f} mm")


if __name__ == "__main__":
    main()
