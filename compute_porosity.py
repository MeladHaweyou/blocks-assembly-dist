#!/usr/bin/env python3
import sys, math, pandas as pd

CELL = 0.038
THICK = 0.019
GRID_N = 7

def layer_index(posZ):
    return int(round((posZ / THICK) - 0.5))

def span_from_size(size_x, size_y):
    # map size to cell spans (rounded)
    wx = int(round(size_x / CELL))
    wy = int(round(size_y / CELL))
    return wx, wy

def center_to_anchor(cell_center, span):
    # convert center (float in cell coords) to lower-left anchor (int)
    return int(round(cell_center - (span - 1)/2.0))

def main(csv_path):
    df = pd.read_csv(csv_path)
    L = 43
    grids = [[[False for _ in range(GRID_N)] for _ in range(GRID_N)] for _ in range(L)]
    cx = (GRID_N - 1)/2.0; cy = (GRID_N - 1)/2.0
    for _, r in df.iterrows():
        k = layer_index(r["posZ"])
        wx, wy = span_from_size(r["size_x"], r["size_y"])
        # get center in *cell* coords
        ic = (r["posX"] / CELL) + cx
        jc = (r["posY"] / CELL) + cy
        i0 = center_to_anchor(ic, wx)
        j0 = center_to_anchor(jc, wy)
        for di in range(wx):
            for dj in range(wy):
                i = i0 + di; j = j0 + dj
                if 0 <= i < GRID_N and 0 <= j < GRID_N:
                    grids[k][j][i] = True
    A_env = sum(1 for j in range(GRID_N) for i in range(GRID_N) if any(grids[k][j][i] for k in range(L)))
    N_occ = sum(1 for k in range(L) for j in range(GRID_N) for i in range(GRID_N) if grids[k][j][i])
    P_eff = 1.0 - (N_occ / (A_env * L))
    print(f"A_env_cells={A_env}, N_occ_cells={N_occ}, L={L}, P_eff={P_eff:.6f}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: compute_porosity.py path/to/assembly.csv")
        sys.exit(1)
    main(sys.argv[1])
