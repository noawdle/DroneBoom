#!/usr/bin/env python3
"""Offline Streamlit dashboard for UAV GNSS spoofing detection results.

Run from the project root with:
    streamlit run dashboard.py
"""

import json
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st


PROJECT_DIR = Path(__file__).resolve().parent
PROCESSED_DIR = PROJECT_DIR / "data" / "processed"
DEFAULT_NAME = "detected_flight_20261013_193740.csv"
REQUIRED_COLUMNS = {
    "timestamp",
    "gps_east", "gps_north", "gps_up",
    "true_east", "true_north", "true_up",
    "spoofed_gps_east", "spoofed_gps_north", "spoofed_gps_up",
    "horizontal_residual", "detection_state", "detected_spoofing",
}
STATE_COLORS = {
    "NORMAL": "#22c55e",
    "OBSERVE": "#f59e0b",
    "SPOOFING_SUSPECTED": "#ef4444",
}
STATE_CODES = {"NORMAL": 0, "OBSERVE": 1, "SPOOFING_SUSPECTED": 2}


def format_m(value, digits=3):
    return f"{value:.{digits}f} m"


def discovered_csvs():
    return sorted(PROCESSED_DIR.glob("detected_*.csv"), key=lambda path: path.name)


@st.cache_data(show_spinner=False)
def load_csv(path_string):
    dataframe = pd.read_csv(path_string)
    missing = sorted(REQUIRED_COLUMNS - set(dataframe.columns))
    if missing:
        raise ValueError("Required columns are missing: " + ", ".join(missing))

    numeric_columns = [
        "timestamp", "gps_east", "gps_north", "gps_up",
        "true_east", "true_north", "true_up",
        "spoofed_gps_east", "spoofed_gps_north", "spoofed_gps_up",
        "horizontal_residual", "detected_spoofing",
    ]
    for column in numeric_columns:
        dataframe[column] = pd.to_numeric(dataframe[column], errors="coerce")
    if "scenario_spoof_active" in dataframe:
        dataframe["scenario_spoof_active"] = pd.to_numeric(
            dataframe["scenario_spoof_active"], errors="coerce"
        ).fillna(0).astype(int)
    dataframe["detection_state"] = dataframe["detection_state"].fillna("NORMAL")
    return dataframe.dropna(subset=["timestamp", "true_east", "true_north", "horizontal_residual"])


@st.cache_data(show_spinner=False)
def load_scenario_zones(detected_path_string):
    """Load optional injector metadata without using it for detection logic."""

    detected_path = Path(detected_path_string)
    if not detected_path.stem.startswith("detected_"):
        return []
    scenario_path = detected_path.with_name(
        "scenario_" + detected_path.stem[len("detected_"):] + ".json"
    )
    if not scenario_path.is_file():
        return []
    try:
        payload = json.loads(scenario_path.read_text(encoding="utf-8"))
        zones = payload.get("zones", [])
        required = {
            "id", "center_east", "center_north", "radius_east", "radius_north",
        }
        return [zone for zone in zones if required <= set(zone)]
    except (OSError, json.JSONDecodeError, TypeError):
        return []


def thresholds(dataframe):
    """Recalculate the exact baseline thresholds used by detect-gnss-anomaly.py."""

    normal_residual = np.hypot(
        dataframe["gps_east"] - dataframe["true_east"],
        dataframe["gps_north"] - dataframe["true_north"],
    ).dropna()
    baseline_mean = float(normal_residual.mean())
    baseline_std = float(normal_residual.std(ddof=0))
    baseline_max = float(normal_residual.max())
    warning = baseline_mean + 3.0 * baseline_std
    suspect = max(baseline_mean + 5.0 * baseline_std, baseline_max * 1.25)
    return {
        "normal_residual": normal_residual,
        "baseline_mean": baseline_mean,
        "baseline_std": baseline_std,
        "baseline_max": baseline_max,
        "warning": warning,
        "suspect": suspect,
    }


def evaluation_metrics(dataframe):
    """Use scenario_spoof_active only for post-detection evaluation."""

    if "scenario_spoof_active" not in dataframe:
        return None
    actual = dataframe["scenario_spoof_active"].astype(int) == 1
    predicted = dataframe["detected_spoofing"].fillna(0).astype(int) == 1
    tp = int((actual & predicted).sum())
    fp = int((~actual & predicted).sum())
    tn = int((~actual & ~predicted).sum())
    fn = int((actual & ~predicted).sum())
    return {
        "tp": tp,
        "fp": fp,
        "tn": tn,
        "fn": fn,
        "precision": tp / (tp + fp) if tp + fp else 0.0,
        "recall": tp / (tp + fn) if tp + fn else 0.0,
        "accuracy": (tp + tn) / len(dataframe) if len(dataframe) else 0.0,
    }


def base_layout(title, x_title, y_title, equal_axes=False):
    layout = dict(
        title=title,
        xaxis_title=x_title,
        yaxis_title=y_title,
        template="plotly_white",
        height=510,
        margin=dict(l=55, r=30, t=55, b=50),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="left", x=0),
        hovermode="closest",
    )
    if equal_axes:
        layout["yaxis"] = dict(scaleanchor="x", scaleratio=1)
    return layout


def hover_data(dataframe):
    return np.stack(
        [
            dataframe["timestamp"],
            dataframe["horizontal_residual"],
            dataframe["detection_state"],
        ],
        axis=-1,
    )


def add_scenario_overlays(figure, zones):
    """Draw evaluation-only ellipse outlines; never invent spatial samples."""

    for zone in zones:
        angles = np.linspace(0.0, 2.0 * np.pi, 81)
        east = float(zone["center_east"]) + float(zone["radius_east"]) * np.cos(angles)
        north = float(zone["center_north"]) + float(zone["radius_north"]) * np.sin(angles)
        figure.add_trace(go.Scatter(
            x=east, y=north, mode="lines", name=f"Scenario zone {zone['id']}",
            line=dict(color="#7c3aed", width=2, dash="dash"),
            hovertemplate=(
                f"Scenario zone {zone['id']} (evaluation only)<br>"
                f"Centre=({float(zone['center_east']):.2f}, {float(zone['center_north']):.2f}) m"
                "<extra></extra>"
            ),
        ))


def path_figure(dataframe, zones=()):
    figure = go.Figure()
    figure.add_trace(
        go.Scatter(
            x=dataframe["true_east"], y=dataframe["true_north"],
            mode="lines", name="Ground Truth path", line=dict(color="#94a3b8", width=2),
            hoverinfo="skip",
        )
    )
    for state in ("NORMAL", "OBSERVE", "SPOOFING_SUSPECTED"):
        subset = dataframe[dataframe["detection_state"] == state]
        if subset.empty:
            continue
        figure.add_trace(
            go.Scatter(
                x=subset["true_east"], y=subset["true_north"], mode="markers", name=state,
                marker=dict(color=STATE_COLORS[state], size=8, line=dict(width=0.6, color="white")),
                customdata=hover_data(subset),
                hovertemplate=(
                    "t=%{customdata[0]:.2f} s<br>East=%{x:.3f} m<br>North=%{y:.3f} m"
                    "<br>Residual=%{customdata[1]:.3f} m<br>State=%{customdata[2]}<extra></extra>"
                ),
            )
        )
    start, end = dataframe.iloc[0], dataframe.iloc[-1]
    figure.add_trace(go.Scatter(
        x=[start.true_east], y=[start.true_north], mode="markers+text", name="Start",
        text=["Start"], textposition="top center", marker=dict(symbol="circle", size=13, color="#0f172a"),
        hovertemplate="Start<br>East=%{x:.3f} m<br>North=%{y:.3f} m<extra></extra>",
    ))
    add_scenario_overlays(figure, zones)
    figure.add_trace(go.Scatter(
        x=[end.true_east], y=[end.true_north], mode="markers+text", name="End",
        text=["End"], textposition="bottom center", marker=dict(symbol="square", size=12, color="#0f172a"),
        hovertemplate="End<br>East=%{x:.3f} m<br>North=%{y:.3f} m<extra></extra>",
    ))
    figure.update_layout(**base_layout("UAV Path and Detection State", "True East [m]", "True North [m]", True))
    return figure


def comparison_figure(dataframe, show_truth, show_gps, show_spoofed, zones=()):
    figure = go.Figure()
    trace_definitions = [
        (show_truth, "Ground Truth", "true_east", "true_north", "#0f172a", "solid"),
        (show_gps, "Normal GNSS", "gps_east", "gps_north", "#2563eb", "dot"),
        (show_spoofed, "Spoofed GNSS", "spoofed_gps_east", "spoofed_gps_north", "#dc2626", "dash"),
    ]
    for enabled, name, east, north, color, dash in trace_definitions:
        if enabled:
            figure.add_trace(go.Scatter(
                x=dataframe[east], y=dataframe[north], mode="lines+markers", name=name,
                line=dict(color=color, width=2, dash=dash), marker=dict(size=4),
                customdata=hover_data(dataframe),
                hovertemplate=(
                    f"{name}<br>t=%{{customdata[0]:.2f}} s<br>East=%{{x:.3f}} m"
                    "<br>North=%{y:.3f} m<br>Residual=%{customdata[1]:.3f} m<extra></extra>"
                ),
            ))
    add_scenario_overlays(figure, zones)
    figure.update_layout(**base_layout("Ground Truth vs Normal and Spoofed GNSS", "East [m]", "North [m]", True))
    return figure


def residual_figure(dataframe, limits):
    figure = go.Figure()
    figure.add_trace(go.Scatter(
        x=dataframe["timestamp"], y=dataframe["horizontal_residual"], mode="lines",
        name="Horizontal residual", line=dict(color="#2563eb", width=2),
        customdata=hover_data(dataframe),
        hovertemplate=(
            "t=%{x:.2f} s<br>Residual=%{y:.3f} m<br>State=%{customdata[2]}<extra></extra>"
        ),
    ))
    suspected = dataframe[dataframe["detected_spoofing"] == 1]
    figure.add_trace(go.Scatter(
        x=suspected["timestamp"], y=suspected["horizontal_residual"], mode="markers",
        name="SPOOFING_SUSPECTED", marker=dict(color=STATE_COLORS["SPOOFING_SUSPECTED"], size=9),
        hovertemplate="t=%{x:.2f} s<br>Residual=%{y:.3f} m<extra>SPOOFING_SUSPECTED</extra>",
    ))
    figure.add_hline(y=limits["warning"], line_dash="dot", line_color=STATE_COLORS["OBSERVE"], annotation_text="Warning threshold")
    figure.add_hline(y=limits["suspect"], line_dash="dash", line_color=STATE_COLORS["SPOOFING_SUSPECTED"], annotation_text="Suspect threshold")
    figure.update_layout(**base_layout("GNSS–Ground Truth Residual over Time", "Timestamp [s]", "Horizontal residual [m]"))
    return figure


def spatial_residual_figure(dataframe, zones=()):
    """A sampled spatial residual map: no interpolation into unvisited space."""

    figure = go.Figure(go.Scatter(
        x=dataframe["true_east"], y=dataframe["true_north"], mode="markers",
        marker=dict(
            size=10,
            color=dataframe["horizontal_residual"],
            colorscale="Turbo",
            colorbar=dict(title="Residual [m]"),
            showscale=True,
            line=dict(color="white", width=0.35),
        ),
        customdata=hover_data(dataframe),
        hovertemplate=(
            "t=%{customdata[0]:.2f} s<br>True East=%{x:.3f} m<br>True North=%{y:.3f} m"
            "<br>Residual=%{customdata[1]:.3f} m<br>State=%{customdata[2]}<extra></extra>"
        ),
        name="Measured sample",
    ))
    add_scenario_overlays(figure, zones)
    figure.update_layout(**base_layout(
        "Measured Spatial Residual Map (visited samples only)",
        "True East [m]", "True North [m]", True,
    ))
    return figure


def spatial_grid_figure(dataframe, cell_size_m, limits, zones=()):
    """Aggregate only visited true-position cells; unvisited cells stay empty."""

    working = dataframe[[
        "true_east", "true_north", "horizontal_residual", "detected_spoofing",
    ]].copy()
    working["east_cell"] = np.floor(working["true_east"] / cell_size_m) * cell_size_m + cell_size_m / 2.0
    working["north_cell"] = np.floor(working["true_north"] / cell_size_m) * cell_size_m + cell_size_m / 2.0
    grouped = working.groupby(["north_cell", "east_cell"], as_index=False).agg(
        mean_residual=("horizontal_residual", "mean"),
        samples=("horizontal_residual", "size"),
        detection_rate=("detected_spoofing", "mean"),
    )
    east_centres = np.sort(grouped["east_cell"].unique())
    north_centres = np.sort(grouped["north_cell"].unique())
    residual_grid = np.full((len(north_centres), len(east_centres)), np.nan)
    customdata = np.empty((len(north_centres), len(east_centres), 2), dtype=object)
    customdata[:] = np.nan
    east_index = {value: index for index, value in enumerate(east_centres)}
    north_index = {value: index for index, value in enumerate(north_centres)}
    for cell in grouped.itertuples(index=False):
        row, column = north_index[cell.north_cell], east_index[cell.east_cell]
        residual_grid[row, column] = cell.mean_residual
        customdata[row, column] = [cell.samples, cell.detection_rate]

    z_max = max(float(np.nanmax(residual_grid)), limits["suspect"] * 1.05, 0.1)
    warning_fraction = min(limits["warning"] / z_max, 0.45)
    suspect_fraction = min(max(limits["suspect"] / z_max, warning_fraction + 0.08), 0.92)
    figure = go.Figure(go.Heatmap(
        x=east_centres, y=north_centres, z=residual_grid, customdata=customdata,
        zmin=0.0, zmax=z_max, xgap=1, ygap=1, connectgaps=False,
        colorscale=[
            [0.0, "#22c55e"], [warning_fraction, "#22c55e"],
            [suspect_fraction, "#f59e0b"], [1.0, "#ef4444"],
        ],
        colorbar=dict(title="Mean residual [m]"),
        hovertemplate=(
            "Cell centre: East=%{x:.2f} m, North=%{y:.2f} m"
            "<br>Mean residual=%{z:.3f} m"
            "<br>Samples=%{customdata[0]}"
            "<br>Detected rate=%{customdata[1]:.0%}<extra></extra>"
        ),
        name="Visited grid cell",
    ))
    add_scenario_overlays(figure, zones)
    figure.update_layout(**base_layout(
        f"Visited-cell Spatial Grid ({cell_size_m:.1f} m; no interpolation)",
        "True East [m]", "True North [m]", True,
    ))
    # NaN cells are deliberately transparent, exposing the grey no-data background.
    figure.update_layout(plot_bgcolor="#d1d5db")
    return figure


def timeline_figure(dataframe):
    state_code = dataframe["detection_state"].map(STATE_CODES).fillna(0)
    colors = dataframe["detection_state"].map(STATE_COLORS).fillna(STATE_COLORS["NORMAL"])
    figure = go.Figure(go.Scatter(
        x=dataframe["timestamp"], y=state_code, mode="lines+markers",
        line=dict(color="#94a3b8", width=1), marker=dict(color=colors, size=7),
        customdata=hover_data(dataframe),
        hovertemplate="t=%{x:.2f} s<br>State=%{customdata[2]}<br>Residual=%{customdata[1]:.3f} m<extra></extra>",
        name="Detection state",
    ))
    figure.update_layout(**base_layout("Detection State Timeline", "Timestamp [s]", "Detection state"))
    figure.update_yaxes(tickvals=[0, 1, 2], ticktext=["NORMAL", "OBSERVE", "SUSPECTED"], range=[-0.35, 2.35])
    return figure


def confusion_figure(metrics):
    matrix = [[metrics["tn"], metrics["fp"]], [metrics["fn"], metrics["tp"]]]
    return go.Figure(go.Heatmap(
        z=matrix, x=["Predicted Normal", "Predicted Suspected"],
        y=["Actual Normal", "Actual Spoofing"], colorscale="Blues", showscale=False,
        text=matrix, texttemplate="%{text}", hovertemplate="%{y}<br>%{x}: %{z}<extra></extra>",
    )).update_layout(title="Scenario vs Detection Confusion Matrix", height=330, margin=dict(l=30, r=20, t=50, b=30))


def main():
    st.set_page_config(page_title="UAV GNSS Spoofing Dashboard", page_icon="🛰️", layout="wide")
    st.markdown("""
        <style>
        .stApp { background: #f8fafc; }
        [data-testid="stMetricValue"] { font-size: 1.55rem; }
        </style>
    """, unsafe_allow_html=True)
    st.title("UAV GNSS Spoofing Detection & Spatial Visualization")
    st.caption(
        "Offline analysis of PX4/Gazebo flight logs. This shows GNSS anomalies "
        "observed where the UAV actually scanned, not RF coverage of unvisited space."
    )

    candidates = discovered_csvs()
    if not candidates:
        st.error("No detected_*.csv file was found in data/processed/.")
        st.stop()
    names = [path.name for path in candidates]
    default_index = names.index(DEFAULT_NAME) if DEFAULT_NAME in names else 0
    st.sidebar.header("Analysis Controls")
    selected_name = st.sidebar.selectbox("Detected flight CSV", names, index=default_index)
    selected_path = PROCESSED_DIR / selected_name
    st.sidebar.caption(f"Input: data/processed/{selected_name}")
    st.sidebar.markdown("**Visible analyses**")
    show_path = st.sidebar.checkbox("UAV Path", value=True)
    show_comparison = st.sidebar.checkbox("GNSS Comparison", value=True)
    show_residual = st.sidebar.checkbox("Residual Timeline", value=True)
    show_map = st.sidebar.checkbox("Spatial Residual Map", value=True)
    show_grid = st.sidebar.checkbox("Spatial Grid Map", value=True)
    show_truth = st.sidebar.checkbox("Show Ground Truth", value=True)
    show_gps = st.sidebar.checkbox("Show Normal GNSS", value=True)
    show_spoofed = st.sidebar.checkbox("Show Spoofed GNSS", value=True)

    try:
        dataframe = load_csv(str(selected_path))
    except (ValueError, pd.errors.ParserError) as error:
        st.error(f"Could not load {selected_name}: {error}")
        st.stop()
    limits = thresholds(dataframe)
    metrics = evaluation_metrics(dataframe)
    scenario_zones = load_scenario_zones(str(selected_path))
    show_scenario = st.sidebar.checkbox(
        "Show Scenario Ground Truth",
        value=False,
        disabled=not scenario_zones,
        help="Purple outlines are evaluation-only random scenario zones.",
    )
    grid_size = st.sidebar.selectbox("Spatial grid cell size", [0.5, 1.0], index=0)
    if not scenario_zones:
        st.sidebar.caption("No random-zone metadata exists for this selected CSV.")
    overlays = scenario_zones if show_scenario else ()

    spoof_count = int((dataframe["detected_spoofing"] == 1).sum())
    kpi_columns = st.columns(4)
    kpi_columns[0].metric("Total Samples", f"{len(dataframe):,}")
    kpi_columns[1].metric("Max Residual", format_m(float(dataframe["horizontal_residual"].max()), 2))
    kpi_columns[2].metric("Spoof Detected", f"{spoof_count}")
    kpi_columns[3].metric("Accuracy", f"{metrics['accuracy'] * 100:.2f}%" if metrics else "N/A")
    detail_columns = st.columns(3)
    detail_columns[0].metric("Normal Mean Residual", format_m(limits["baseline_mean"], 3))
    detail_columns[1].metric("Precision", f"{metrics['precision']:.4f}" if metrics else "N/A")
    detail_columns[2].metric("Recall", f"{metrics['recall']:.4f}" if metrics else "N/A")
    st.sidebar.info(
        f"Warning: {limits['warning']:.4f} m\n\n"
        f"Suspect: {limits['suspect']:.4f} m\n\n"
        "Thresholds are recalculated from the normal GNSS/Ground Truth baseline."
    )

    overview_tab, spatial_tab, time_tab, data_tab = st.tabs([
        "Overview", "Spatial Analysis", "Time Analysis", "Data"
    ])
    with overview_tab:
        if show_path:
            st.plotly_chart(path_figure(dataframe, overlays), width="stretch")
        if metrics:
            left, right = st.columns([1.1, 1])
            with left:
                st.plotly_chart(confusion_figure(metrics), width="stretch")
            with right:
                st.subheader("Detection Evaluation")
                st.dataframe(
                    pd.DataFrame(
                        {
                            "Metric": ["TP", "FP", "TN", "FN", "Precision", "Recall", "Accuracy"],
                            "Value": [
                                str(metrics["tp"]), str(metrics["fp"]), str(metrics["tn"]), str(metrics["fn"]),
                                f"{metrics['precision']:.4f}", f"{metrics['recall']:.4f}", f"{metrics['accuracy']:.4f}",
                            ],
                        }
                    ),
                    hide_index=True,
                    width="stretch",
                )
                st.caption("Scenario labels are used only here for post-detection evaluation, not for residual thresholds or state decisions.")
    with spatial_tab:
        if show_comparison:
            st.plotly_chart(
                comparison_figure(dataframe, show_truth, show_gps, show_spoofed, overlays),
                width="stretch",
            )
        if show_map:
            st.plotly_chart(spatial_residual_figure(dataframe, overlays), width="stretch")
        if show_grid:
            st.plotly_chart(
                spatial_grid_figure(dataframe, grid_size, limits, overlays), width="stretch"
            )
            st.caption(
                "Each colored cell aggregates only samples taken inside that cell. "
                "Grey cells are No Data: the UAV did not visit them, so no value was interpolated."
            )
    with time_tab:
        if show_residual:
            st.plotly_chart(residual_figure(dataframe, limits), width="stretch")
        st.plotly_chart(timeline_figure(dataframe), width="stretch")
    with data_tab:
        st.subheader("Flight Samples")
        state_filter = st.radio(
            "Detection state filter", ["All", "NORMAL", "OBSERVE", "SPOOFING_SUSPECTED"], horizontal=True
        )
        filtered = dataframe if state_filter == "All" else dataframe[dataframe["detection_state"] == state_filter]
        table_columns = [
            "timestamp", "true_east", "true_north", "gps_east", "gps_north",
            "spoofed_gps_east", "spoofed_gps_north", "horizontal_residual",
            "detection_state", "detected_spoofing",
        ]
        if "scenario_spoof_active" in filtered:
            table_columns.append("scenario_spoof_active")
        if "scenario_zone_id" in filtered:
            table_columns.append("scenario_zone_id")
        st.dataframe(filtered[table_columns], hide_index=True, width="stretch", height=460)
        st.caption(f"Showing {len(filtered):,} of {len(dataframe):,} samples.")


if __name__ == "__main__":
    main()
