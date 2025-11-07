"""Streamlit UI for loading unified assembly CSV files."""
from __future__ import annotations

import hashlib
import csv
import io
import tempfile
from pathlib import Path
from typing import Dict, Iterable, List

import pandas as pd
import streamlit as st

from io_unified_csv import load_unified_csv
from models import AssemblyInstance, BlockType, GlobalConfig, Material
from run_controller import RunController
from runner import run_sequence
from validation import apply_meta_to_config, validate_and_resolve


def _write_temp_csv(rows: List[dict]) -> str:
    """Write rows to a temporary CSV file and return its path."""

    fieldnames: List[str] = []
    for row in rows:
        for key in row.keys():
            if key and key not in fieldnames:
                fieldnames.append(key)

    if not fieldnames:
        raise ValueError("No columns detected in uploaded CSV.")

    tmp = tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", newline="", delete=False, suffix=".csv"
    )
    with tmp as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return tmp.name


def _load_unified_from_rows(rows: List[dict]):
    """Persist uploaded rows then load them via :func:`load_unified_csv`."""

    tmp_path: str | None = None
    try:
        tmp_path = _write_temp_csv(rows)
        return load_unified_csv(tmp_path)
    finally:
        if tmp_path:
            Path(tmp_path).unlink(missing_ok=True)


@st.cache_resource
def get_run_controller() -> RunController:
    """Create a cached controller shared across reruns."""

    return RunController()


def _render_definitions(
    materials: Dict[str, Material], block_types: Dict[str, BlockType]
) -> None:
    with st.expander("Definitions", expanded=True):
        st.subheader("Materials")
        material_rows = [
            {
                "material": mat.name,
                "density_kgm3": mat.density_kgm3,
                "mu_slide": mat.mu_slide,
                "mu_spin": mat.mu_spin,
                "mu_roll": mat.mu_roll,
                "restitution": mat.restitution,
                "mu_slide_along": (
                    mat.mu_slide * mat.anisotropic_friction[0]
                    if mat.anisotropic_friction
                    else None
                ),
                "mu_slide_across": (
                    mat.mu_slide * mat.anisotropic_friction[1]
                    if mat.anisotropic_friction
                    else None
                ),
            }
            for mat in sorted(materials.values(), key=lambda m: m.name)
        ]
        st.dataframe(material_rows, width="stretch")

        st.subheader("Block Types")
        block_rows = [
            {
                "block_type": bt.name,
                "size_x_m": bt.size_x_m,
                "size_y_m": bt.size_y_m,
                "size_z_m": bt.size_z_m,
                "material_of_type": bt.material_name,
            }
            for bt in sorted(block_types.values(), key=lambda b: b.name)
        ]
        st.dataframe(block_rows,   width="stretch")


def _render_assembly(assembly: Iterable[AssemblyInstance]) -> None:
    assembly_rows = [
        {
            "order": inst.order,
            "block_type": inst.block_type,
            "posX_m": inst.pos_m[0],
            "posY_m": inst.pos_m[1],
            "posZ_m": inst.pos_m[2],
            "rotX_deg": inst.rot_deg[0],
            "rotY_deg": inst.rot_deg[1],
            "rotZ_deg": inst.rot_deg[2],
            "label": inst.label or "",
        }
        for inst in sorted(assembly, key=lambda a: a.order)
    ]
    st.subheader("Assembly")
    st.dataframe(assembly_rows, width="stretch")


def _configure_global_settings(cfg: GlobalConfig) -> GlobalConfig:
    st.subheader("Global placement & camera")

    placement_options = ["gentle", "dynamic"]
    try:
        placement_index = placement_options.index(cfg.placement_mode)
    except ValueError:
        placement_index = 0
    cfg.placement_mode = st.selectbox(
        "Placement mode",
        placement_options,
        index=placement_index,
        format_func=lambda mode: "Quasi-static" if mode == "gentle" else mode.capitalize(),
    )
    cfg.servo_clearance_m = 0.002

    drop_mm = st.number_input(
        "Drop distance (mm)",
        value=max(0.0, cfg.drop_distance_m * 1000.0),
        min_value=0.0,
        step=0.5,
    )
    cfg.drop_distance_m = drop_mm / 1000.0

    cfg.pre_spawn_delay_s = st.number_input(
        "Delay before first spawn (s)",
        value=cfg.pre_spawn_delay_s,
        min_value=0.0,
        step=0.5,
    )

    st.markdown("**Camera** (applies at start)")
    cfg.camera.distance = st.number_input(
        "Distance",
        value=cfg.camera.distance,
        min_value=0.1,
    )
    cfg.camera.yaw_deg = st.number_input("Yaw (deg)", value=cfg.camera.yaw_deg)
    cfg.camera.pitch_deg = st.number_input("Pitch (deg)", value=cfg.camera.pitch_deg)

    col_x, col_y, col_z = st.columns(3)
    cfg.camera.target_m = (
        col_x.number_input("Target X", value=cfg.camera.target_m[0]),
        col_y.number_input("Target Y", value=cfg.camera.target_m[1]),
        col_z.number_input("Target Z", value=cfg.camera.target_m[2]),
    )

    return cfg


def _drain_controller_queue(controller: RunController) -> None:
    payload = controller.poll()
    while payload is not None:
        kind, token, value = payload
        if kind == "error":
            st.error(f"Run {token} failed: {value}")
        elif kind == "result":
            st.success("Simulation completed.")
        elif kind == "finished" and value:
            st.info("Simulation stopped early.")
        payload = controller.poll()


def main() -> None:
    st.title("Robotic Tectonics III, Assembly Simulator using Pybullet")

    uploaded = st.file_uploader("Upload unified CSV", type=["csv"])
    upload_sha1 = None
    if uploaded:
        try:
            upload_sha1 = hashlib.sha1(uploaded.getvalue()).hexdigest()
        except Exception:
            upload_sha1 = None
    if not uploaded:
        st.info("Upload a unified CSV to inspect and simulate the assembly.")
        _drain_controller_queue(get_run_controller())
        return

    try:
        text = uploaded.getvalue().decode("utf-8")
    except UnicodeDecodeError as exc:
        st.error(f"Unable to decode CSV as UTF-8: {exc}")
        return

    try:
        rows = list(csv.DictReader(io.StringIO(text)))
    except csv.Error as exc:
        st.error(f"Failed to parse CSV: {exc}")
        return

    if "live_edit_enabled" not in st.session_state:
        st.session_state["live_edit_enabled"] = False
    if "raw_rows" not in st.session_state:
        st.session_state["raw_rows"] = rows
    if "last_upload_sha1" not in st.session_state:
        st.session_state["last_upload_sha1"] = upload_sha1
    is_new_upload = (
        upload_sha1 is not None
        and upload_sha1 != st.session_state.get("last_upload_sha1")
    )
    if is_new_upload:
        st.session_state["last_upload_sha1"] = upload_sha1
        st.session_state["pending_seed_rows"] = rows

    st.checkbox(
        "Enable live CSV editor (beta)",
        key="live_edit_enabled",
        help="When enabled, you can edit the uploaded CSV inline. When disabled, the editor is hidden and ignored.",
    )

    if (
        st.session_state["live_edit_enabled"]
        and st.session_state.get("pending_seed_rows") is not None
    ):
        st.session_state["raw_rows"] = st.session_state.pop("pending_seed_rows")

    if st.session_state["live_edit_enabled"]:
        # Show the editor only in this branch
        st.subheader("Edit unified CSV (live)")
        editable_df = st.data_editor(
            pd.DataFrame(st.session_state["raw_rows"]),
            num_rows="dynamic",
            width="stretch",
        )

        col_apply, col_reset, col_download = st.columns(
            [1, 1, 1], vertical_alignment="center"
        )

        with col_apply:
            if st.button("Apply edits & re-parse", type="primary"):
                try:
                    edited_rows = editable_df.to_dict(orient="records")
                    meta, materials, block_types, assembly = _load_unified_from_rows(
                        edited_rows
                    )
                    st.session_state["raw_rows"] = edited_rows
                    st.success("Edits applied.")
                except Exception as exc:
                    st.error(f"Failed to parse edited CSV: {exc}")

        with col_reset:
            if st.button("Revert to uploaded"):
                st.session_state["raw_rows"] = rows
                st.rerun()

        with col_download:
            import csv as _csv
            import io as _io

            buf = _io.StringIO()
            writer = _csv.DictWriter(buf, fieldnames=list(editable_df.columns))
            writer.writeheader()
            writer.writerows(editable_df.to_dict("records"))
            data = buf.getvalue()
            st.download_button(
                "Download CSV",
                data,
                file_name="assembly_edited.csv",
                mime="text/csv",
            )
        rows_for_parse = st.session_state["raw_rows"]
    else:
        rows_for_parse = rows

    if not rows:
        st.warning("Uploaded CSV is empty.")
        return

    # If you've just clicked Apply, the variables are already set.
    # Otherwise parse from current editor contents:
    if "materials" not in locals():
        try:
            meta, materials, block_types, assembly = _load_unified_from_rows(
                rows_for_parse
            )
        except Exception as exc:  # pragma: no cover - surfaced to UI
            st.error(f"Failed to load CSV: {exc}")
            return

    cfg = GlobalConfig(gui=True)
    cfg = apply_meta_to_config(meta, cfg)

    _render_definitions(materials, block_types)
    _render_assembly(assembly)

    cfg = _configure_global_settings(cfg)

    try:
        resolved = validate_and_resolve(materials, block_types, assembly)
        st.success(
            f"Ready: {len(resolved)} instances • {len(materials)} materials • {len(block_types)} types"
        )
    except ValueError as exc:
        st.error(f"Validation failed:\n{exc}")
        resolved = []

    controller = get_run_controller()
    _drain_controller_queue(controller)

    run_disabled = controller.is_running() or not resolved
    stop_disabled = not controller.is_running()

    if st.button("Run full sequence", disabled=run_disabled):
        controller.start(target=run_sequence, args=(cfg, resolved))

    if st.button("Stop simulation", disabled=stop_disabled):
        controller.stop()


if __name__ == "__main__":
    main()
