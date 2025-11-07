
# Ready-to-run assemblies for your PyBullet app

This folder contains four CSVs you can **load directly** into `app.py` (Streamlit GUI) to simulate.

- `A_xbraced_tube.csv` — Perimeter frame + rungs + alternating brace bands (6-layer repeat).  
- `B_checkerboard.csv` — Checkerboard voxel column with 2×2 bricks wherever possible (2-layer repeat).  
- `C_helical_louver.csv` — Two 1×4 louvers per layer, rotating 45°/layer; single radial tie every 3rd layer.  
- `D_diagrid_core.csv` — Sparse '+' core with diagrid wrap split across layers; radial 1×2 ties (4-layer repeat).  

All designs use a **7×7 grid** with **cell pitch 0.038 m** and **43 layers** at **0.019 m** thickness ⇒ nominal height 0.817 m.

## CSV Schema
Matches your app's schema:
```
name,order,size_x,size_y,size_z,mass,mu_slide,mu_spin,mu_roll,drop_height,posX,posY,posZ,rotX,rotY,rotZ
```

- Units: meters, kilograms. MDF density assumed **750 kg/m³** for mass.  
- Friction: `mu_slide=0.6`, `mu_spin=mu_roll=0.0`.  
- `rotZ` = 0° → long axis along +X; 90° → along +Y.  
- `drop_height` = 0.01 m (spawns block slightly above `posZ`, then drops to settle).

## Coordinate System
- Origin at grid center (0,0). Cell centers are at multiples of **0.038 m**.  
- Each brick's `posX,posY` is the **center** of the spanned cells.  
- The 7×7 footprint corresponds to **26.6 × 26.6 cm**, within the 30 × 30 cm limit.

## Effective Porosity Metric
We compute **envelope porosity**:
- Let `A_env` be the number of unique plan cells (out of 49) occupied on **any** layer.
- Let `N_occ` be the total number of occupied cells **across all layers**.
- With `L=43` layers, effective porosity is:
```
P_eff = 1 - N_occ / (A_env * L)
```
This penalizes skinny towers and rewards internal voids within the structure’s own envelope.

See `porosity_summary.csv` and `manifest.json` for the computed metrics per design.
