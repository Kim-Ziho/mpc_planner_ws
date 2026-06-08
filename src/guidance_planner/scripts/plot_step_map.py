#!/usr/bin/env python3
"""StepMap layer 분포 시각화.

gym_cpp 노드가 run_once 단일 실행 후 덤프한 CSV(step_map_layers.csv)를 읽어
선택된 vis_stages 개 layer 각각을 (x, y, cost) 3축 surface 로 그린다.

배치: vis_stages × 1 칼럼. 초기 layer(가장 이른 시각, stage_order=0)를 맨 아래에 둔다.

사용법:
    python3 plot_step_map.py <csv_path> [--png <out.png>] [--show]

CSV 컬럼: stage_order,gt,layer_time,gx,gy,x,y,cost
"""

import argparse

import numpy as np
import matplotlib


def main():
    parser = argparse.ArgumentParser(description="StepMap layer 분포 시각화")
    parser.add_argument("csv", help="StepMap 덤프 CSV 경로")
    parser.add_argument("--png", default="", help="저장할 PNG 경로 (비우면 저장 안 함)")
    parser.add_argument("--show", action="store_true", help="그래프 창 표시")
    args = parser.parse_args()

    # 창 표시가 없으면 헤드리스 백엔드 사용 (pyplot import 전에 설정)
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (3D projection 등록)

    data = np.genfromtxt(args.csv, delimiter=",", names=True)
    if data.size == 0:
        print("[plot_step_map] empty CSV: %s" % args.csv)
        return

    stage_order = data["stage_order"].astype(int)
    orders = sorted(np.unique(stage_order).tolist())
    n = len(orders)
    zmax = max(float(np.max(data["cost"])), 1e-6)

    fig = plt.figure(figsize=(7.0, 3.6 * n))
    surf = None

    for order in orders:
        mask = stage_order == order
        gx = data["gx"][mask].astype(int)
        gy = data["gy"][mask].astype(int)
        gt = int(data["gt"][mask][0])
        layer_time = float(data["layer_time"][mask][0])

        nx = int(gx.max()) + 1
        ny = int(gy.max()) + 1
        X = np.zeros((nx, ny))
        Y = np.zeros((nx, ny))
        Z = np.zeros((nx, ny))
        X[gx, gy] = data["x"][mask]
        Y[gx, gy] = data["y"][mask]
        Z[gx, gy] = data["cost"][mask]

        # 초기 layer(order=0)를 맨 아래로 → subplot 행 인덱스 = n - order
        ax = fig.add_subplot(n, 1, n - order, projection="3d")
        surf = ax.plot_surface(X, Y, Z, cmap="viridis", vmin=0.0, vmax=zmax,
                               linewidth=0, antialiased=False)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("cost")
        ax.set_zlim(0.0, zmax)
        ax.set_title("stage gt=%d  (t=%.2fs)" % (gt, layer_time))

    if surf is not None:
        fig.colorbar(surf, ax=fig.axes, shrink=0.5, pad=0.1, label="cost")
    fig.suptitle("StepMap layer cost distribution")

    if args.png:
        fig.savefig(args.png, dpi=120, bbox_inches="tight")
        print("[plot_step_map] saved %s" % args.png)
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
