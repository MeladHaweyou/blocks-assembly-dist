# Blocks Assembly Sandbox

A lightweight, physics-based sandbox for stacking parametric blocks in PyBullet with a Streamlit UI. Assemblies are described by a single, human-readable CSV file with four sections: **meta**, **materials**, **block_types**, and **assembly**.

## Quick Start (Windows, Anaconda)

### 1. Create and activate the environment
```bash
conda create -n blocksim python=3.12 -y
conda activate blocksim
conda config --add channels conda-forge
conda config --set channel_priority strict
```

### 2. Install the project requirements
```bash
conda install pybullet streamlit numpy pandas -y
```

### 3. Run the Streamlit app
```bash
streamlit run app.py
```

## Unified CSV Overview

Each row belongs to one of four sections, declared in the `section` column:

- **meta** – session-level settings (placement mode, drop/servo distances, camera).
- **materials** – friction & density per material (e.g., `WOOD`, `STEEL`).
- **block_types** – reusable block geometries tied to a material. Supports `shape=box` (default) and `shape=wedge`.
- **assembly** – ordered placements (position, rotation, optional per-row delay).

All units are SI (meters, kilograms, seconds, degrees). Positions reference block centers.

### Minimal Example
```csv
section,key,value,material,density_kgm3,mu_slide,mu_spin,mu_roll,restitution,block_type,size_x_m,size_y_m,size_z_m,material_of_type,shape,wedge_run_x_m,wedge_width_y_m,wedge_rise_z_m,color_rgba,order,label,posX_m,posY_m,posZ_m,rotX_deg,rotY_deg,rotZ_deg,delay_after_s
meta,camera_distance,1.2,,,,,,,,,,,,,,,,,,,,,,,,,
meta,placement_mode,gentle,,,,,,,,,,,,,,,,,,,,,,,,,
meta,servo_clearance_mm,0.3,,,,,,,,,,,,,,,,,,,,,,,,,
materials,,,WOOD,700,0.60,0.02,0.01,0.01,,,,,,,,,,,,,,,,,,
block_types,,,,,,,,,Tile40,0.040,0.040,0.020,WOOD,,,,,,,,,,,,,
assembly,,,,,,,,,Tile40,,,,,,,,,,1,Base,0.0000,0.0000,0.0100,0,0,0,0.0
assembly,,,,,,,,,Tile40,,,,,,,,,,2,Top ,0.0150,0.0000,0.0300,0,0,0,0.2
```

## Sections & Common Fields

### `meta`
- `placement_mode` — `"gentle"` (servo settle) or `"dynamic"` (small drop).
- `servo_clearance_mm` — target approach gap for gentle placement (e.g., `0.3`).
- `drop_distance_mm` — global drop distance for dynamic placement (e.g., `2.0`).
- `camera_distance`, `camera_yaw_deg`, `camera_pitch_deg`, `camera_target_x/y/z` — initial view settings.

### `materials`
- `material` — name used by block types.
- `density_kgm3` — density of the material.
- `mu_slide`, `mu_spin`, `mu_roll` — sliding, torsional, and rolling friction values.
- `restitution` — bounciness (`0–0.05` typical for wood).
- `color_rgba` — optional visualization color in `"r,g,b,a"` format.

### `block_types`
- `block_type` — reference name used by the assembly.
- `size_x_m`, `size_y_m`, `size_z_m` — full block dimensions.
- `material_of_type` — material name defined in the `materials` section.
- `shape` — `"box"` (default) or `"wedge"`.
- For `shape=wedge`, supply `wedge_run_x_m`, `wedge_width_y_m`, `wedge_rise_z_m`.
- `color_rgba` — optional visualization color.

### `assembly`
- `order` — 1-based sequence number.
- `block_type` — which block to place.
- `label` — free-text tag for UI/debug.
- `posX_m`, `posY_m`, `posZ_m` — center position (meters).
- `rotX_deg`, `rotY_deg`, `rotZ_deg` — Euler angles (degrees).
- `delay_after_s` — optional pause after placing this block (useful for demos).

## Simulation Parameters

Mass is computed as material density multiplied by block volume (wedge volume uses the triangular prism formula). Frictional behavior is governed by:

- `mu_slide` resisting sliding.
- `mu_spin` resisting in-place yaw (torsional friction) at contact.
- `mu_roll` resisting gentle rocking/rolling about an edge.

Non-zero `mu_spin`/`mu_roll` adds damping so small perturbations die out.

### Placement Modes
- **Gentle** – the block servo-approaches to `servo_clearance_mm` before release.
- **Dynamic** – the block is placed slightly higher, then dropped by `drop_distance_mm`.

`delay_after_s` causes the runner to advance physics for the specified time before placing the next block, helping stacks settle for clearer demos.

### Recommended Defaults for Small Wooden Blocks
- `WOOD`: `mu_slide ≈ 0.55`, `mu_spin ≈ 0.02`, `mu_roll ≈ 0.01`, `restitution ≈ 0.01`.
- Ground plane friction: `~1.5–2.0` for stable base contact.
- Timestep: `1/240 s`, solver iterations `≥120` (already set in the backend).

## Project Layout (Key Files)
- `app.py` — Streamlit user interface (upload CSV, view definitions, run).
- `io_unified_csv.py` — CSV parser/writer for the unified format.
- `models.py` — data classes and camera/placement config.
- `runner.py` — places blocks using PyBullet.
- `backend_bullet.py` — engine wrapper (contact tuning, wedge support).
- `data/` — example CSVs (you can add your own).

## Troubleshooting

### “Unknown section” or missing columns
Ensure your CSV has a `section` column and follows the four-section layout.

### Wedge collisions
You need a recent PyBullet. The backend falls back to a mesh hull if `GEOM_CONVEX_HULL` is unavailable, but PyBullet ≥ 3.2.6 is recommended.

### Blocks jitter or drift
Increase solver iterations (in `backend_bullet.py`), keep restitution near `0.01`, and use non-zero `mu_spin`/`mu_roll` on wood.
