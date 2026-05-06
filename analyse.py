#!/usr/bin/env python3
"""Render military-style 2D maps from local shapefiles."""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp/matplotlib-cache")))

import matplotlib.pyplot as plt
import shapefile
from matplotlib.patches import FancyArrowPatch, Polygon, Rectangle


SHAPE_TYPE_POLYGON = shapefile.POLYGON
SHAPE_TYPE_POLYGON_Z = shapefile.POLYGONZ
SHAPE_TYPE_POLYLINE = shapefile.POLYLINE
SHAPE_TYPE_POLYLINE_Z = shapefile.POLYLINEZ
SHAPE_TYPE_POINT = shapefile.POINT
SHAPE_TYPE_POINT_Z = shapefile.POINTZ
DATASET_DIR = Path("dataset")
LAYER_DISPLAY_NAMES = {
    "a_constr": "Built-Up Areas",
    "a_folha": "Map Sheet Boundary",
    "a_hidro": "Water Bodies",
    "a_terreno": "Terrain Area",
    "a_vegetacao": "Vegetation Cover",
    "l_aceiro": "Firebreak Lines",
    "l_curva_nivel": "Contour Lines",
    "l_hidro": "Waterlines",
    "l_lat": "Lateral Boundaries",
    "l_muro_ater_desater": "Walls and Embankments",
    "l_pontes": "Bridges",
    "l_verdes_diversos": "Green Feature Lines",
    "l_vias": "Road Lines",
    "l_workflow": "Workflow Lines",
    "p_geral": "General Points",
    "p_pcota": "Spot Heights",
    "p_pt": "Reference Points",
    "p_tpn": "Topographic Points",
    "p_vg": "Geodetic Vertices",
    "p_vias": "Road Points",
    "p_workflow": "Workflow Points",
}


@dataclass
class ShapeRecord:
    shape_type: int
    points: list[tuple[float, float]]
    parts: list[int]
    attributes: dict[str, str]


@dataclass
class LayerData:
    name: str
    records: list[ShapeRecord]
    shape_type: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a 2D military-style map from shapefiles in the dataset folder."
    )
    parser.add_argument(
        "--input",
        type=Path,
        help="Path to a .shp file. Defaults to the first *terreno*.shp file in dataset/.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("terrain_military_map.png"),
        help="Output PNG path.",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Render one PNG for every .shp file in dataset/.",
    )
    parser.add_argument(
        "--all-aligned",
        action="store_true",
        help="Render one PNG per layer using the same extent as the combined map.",
    )
    parser.add_argument(
        "--combined",
        action="store_true",
        help="Render all .shp layers from dataset/ together in a single aligned map.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("maps"),
        help="Directory used with --all.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="Output DPI for the generated figure.",
    )
    parser.add_argument(
        "--grid",
        type=float,
        default=25.0,
        help="Grid spacing in map units (meters in this dataset).",
    )
    return parser.parse_args()


def find_default_shapefile() -> Path:
    matches = sorted(DATASET_DIR.glob("*terreno*.shp"))
    if not matches:
        raise FileNotFoundError("No *terreno*.shp files were found in dataset/.")
    return matches[0]


def resolve_input_path(shp_path: Path) -> Path:
    if shp_path.exists():
        return shp_path

    candidate = DATASET_DIR / shp_path
    if candidate.exists():
        return candidate

    raise FileNotFoundError(f"Shapefile not found: {shp_path}")


def list_dataset_shapefiles() -> list[Path]:
    return sorted(DATASET_DIR.glob("*.shp"))


def read_shapefile(shp_path: Path) -> tuple[list[ShapeRecord], int]:
    reader = shapefile.Reader(str(shp_path), encoding="latin1")
    if reader.shapeType not in {
        SHAPE_TYPE_POLYGON,
        SHAPE_TYPE_POLYGON_Z,
        SHAPE_TYPE_POLYLINE,
        SHAPE_TYPE_POLYLINE_Z,
        SHAPE_TYPE_POINT,
        SHAPE_TYPE_POINT_Z,
    }:
        raise ValueError(f"Unsupported shapefile type: {reader.shapeType}")

    field_names = [field[0] for field in reader.fields[1:]]
    records: list[ShapeRecord] = []
    for shape_record in reader.iterShapeRecords():
        shape = shape_record.shape
        if shape.shapeType not in {
            SHAPE_TYPE_POLYGON,
            SHAPE_TYPE_POLYGON_Z,
            SHAPE_TYPE_POLYLINE,
            SHAPE_TYPE_POLYLINE_Z,
            SHAPE_TYPE_POINT,
            SHAPE_TYPE_POINT_Z,
        }:
            continue
        attributes = {
            name: str(value).strip()
            for name, value in zip(field_names, shape_record.record)
        }
        raw_points = shape.points if shape.points else []
        records.append(
            ShapeRecord(
                shape_type=shape.shapeType,
                points=[(float(x), float(y)) for x, y in raw_points],
                parts=list(shape.parts) if getattr(shape, "parts", None) else [0],
                attributes=attributes,
            )
        )

    return records, reader.shapeType


def split_parts(
    points: list[tuple[float, float]], parts: list[int], min_points: int = 3
) -> list[list[tuple[float, float]]]:
    part_starts = parts + [len(points)]
    segments = []
    for start, end in zip(part_starts[:-1], part_starts[1:]):
        ring = points[start:end]
        if len(ring) >= min_points:
            segments.append(ring)
    return segments


def shape_type_label(shape_type: int) -> str:
    if shape_type in {SHAPE_TYPE_POLYGON, SHAPE_TYPE_POLYGON_Z}:
        return "Polygon"
    if shape_type in {SHAPE_TYPE_POLYLINE, SHAPE_TYPE_POLYLINE_Z}:
        return "Polyline"
    if shape_type in {SHAPE_TYPE_POINT, SHAPE_TYPE_POINT_Z}:
        return "Point"
    return f"Shape {shape_type}"


def layer_style(stem: str, shape_type: int) -> dict[str, object]:
    prefix = stem.split("_")[-1]
    if shape_type in {SHAPE_TYPE_POLYGON, SHAPE_TYPE_POLYGON_Z}:
        palette = {
            "constr": {"facecolor": "#8f8572", "edgecolor": "#40392d", "hatch": "++"},
            "folha": {"facecolor": "#d9ccb1", "edgecolor": "#3d3a2f", "hatch": None},
            "hidro": {"facecolor": "#8db7c9", "edgecolor": "#2c5f77", "hatch": ".."},
            "terreno": {"facecolor": "#c9b37d", "edgecolor": "#3d3320", "hatch": "///"},
            "vegetacao": {"facecolor": "#93ab77", "edgecolor": "#334b2c", "hatch": "\\\\"},
        }
        return palette.get(prefix, {"facecolor": "#b6a786", "edgecolor": "#43392a", "hatch": "//"})
    if shape_type in {SHAPE_TYPE_POLYLINE, SHAPE_TYPE_POLYLINE_Z}:
        palette = {
            "aceiro": {"color": "#7a4f2a", "linewidth": 1.2, "linestyle": "--"},
            "curva_nivel": {"color": "#8d5c3b", "linewidth": 0.8, "linestyle": "-"},
            "hidro": {"color": "#2b6f8a", "linewidth": 1.2, "linestyle": "-"},
            "lat": {"color": "#6d6d6d", "linewidth": 0.8, "linestyle": ":"},
            "muro_ater_desater": {"color": "#4b4035", "linewidth": 1.0, "linestyle": "-."},
            "pontes": {"color": "#1f1f1f", "linewidth": 1.4, "linestyle": "-"},
            "verdes_diversos": {"color": "#48683e", "linewidth": 1.0, "linestyle": "--"},
            "vias": {"color": "#9a6b3f", "linewidth": 1.0, "linestyle": "-"},
            "workflow": {"color": "#7d3d6b", "linewidth": 1.0, "linestyle": "--"},
        }
        return palette.get(prefix, {"color": "#5a5245", "linewidth": 1.0, "linestyle": "-"})
    palette = {
        "geral": {"color": "#262626", "size": 10, "marker": "o"},
        "pcota": {"color": "#6a3d24", "size": 8, "marker": "^"},
        "pt": {"color": "#1f1f1f", "size": 8, "marker": "s"},
        "tpn": {"color": "#6b1f1f", "size": 16, "marker": "*"},
        "vg": {"color": "#2d5d34", "size": 18, "marker": "D"},
        "vias": {"color": "#8a5a30", "size": 12, "marker": "o"},
        "workflow": {"color": "#7d3d6b", "size": 10, "marker": "x"},
    }
    return palette.get(prefix, {"color": "#262626", "size": 10, "marker": "o"})


def layer_display_name(stem: str) -> str:
    return LAYER_DISPLAY_NAMES.get(stem, stem.replace("_", " ").title())


def nice_length(length: float) -> float:
    if length <= 0:
        return 1.0
    exponent = math.floor(math.log10(length))
    fraction = length / (10 ** exponent)
    if fraction < 1.5:
        nice_fraction = 1
    elif fraction < 3:
        nice_fraction = 2
    elif fraction < 7:
        nice_fraction = 5
    else:
        nice_fraction = 10
    return nice_fraction * (10 ** exponent)


def add_grid(ax, min_x: float, max_x: float, min_y: float, max_y: float, spacing: float) -> None:
    start_x = math.floor(min_x / spacing) * spacing
    end_x = math.ceil(max_x / spacing) * spacing
    start_y = math.floor(min_y / spacing) * spacing
    end_y = math.ceil(max_y / spacing) * spacing

    x = start_x
    while x <= end_x:
        ax.axvline(x, color="#6e7d5a", linewidth=0.35, alpha=0.30, zorder=0)
        x += spacing

    y = start_y
    while y <= end_y:
        ax.axhline(y, color="#6e7d5a", linewidth=0.35, alpha=0.30, zorder=0)
        y += spacing


def add_scale_bar(ax, min_x: float, min_y: float, width: float, height: float) -> None:
    bar_length = nice_length(width * 0.2)
    segment = bar_length / 2
    x0 = min_x + width * 0.05
    y0 = min_y + height * 0.05
    bar_height = height * 0.015

    ax.add_patch(
        Rectangle((x0, y0), segment, bar_height, facecolor="#1f1f1f", edgecolor="#1f1f1f", zorder=5)
    )
    ax.add_patch(
        Rectangle((x0 + segment, y0), segment, bar_height, facecolor="#efe6c9", edgecolor="#1f1f1f", zorder=5)
    )
    ax.text(x0, y0 - bar_height * 1.4, "0", fontsize=8, ha="center", va="top", color="#1f1f1f")
    ax.text(
        x0 + segment,
        y0 - bar_height * 1.4,
        f"{int(segment)} m",
        fontsize=8,
        ha="center",
        va="top",
        color="#1f1f1f",
    )
    ax.text(
        x0 + bar_length,
        y0 - bar_height * 1.4,
        f"{int(bar_length)} m",
        fontsize=8,
        ha="center",
        va="top",
        color="#1f1f1f",
    )


def add_north_arrow(ax, min_x: float, min_y: float, width: float, height: float) -> None:
    x = min_x + width * 0.93
    y = min_y + height * 0.09
    arrow = FancyArrowPatch(
        (x, y),
        (x, y + height * 0.12),
        arrowstyle="simple",
        mutation_scale=18,
        fc="#1f1f1f",
        ec="#1f1f1f",
        zorder=6,
    )
    ax.add_patch(arrow)
    ax.text(x, y + height * 0.135, "N", fontsize=12, fontweight="bold", ha="center", va="bottom")


def add_title_block(
    ax,
    attrs: dict[str, str],
    crs_label: str,
    layer_label: str,
    geometry_type: str,
    feature_count: int,
) -> None:
    lines = [
        "CARTA TATICA 2D",
        f"Camada: {layer_label}",
        f"Geometria: {geometry_type}",
        f"Elementos: {feature_count}",
        f"Folha: {attrs.get('num_folha', 'n/d')}",
        crs_label,
    ]
    ax.text(
        0.015,
        0.985,
        "\n".join(lines),
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=9,
        color="#1f1f1f",
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "#f4ecd2", "edgecolor": "#3d3a2f", "alpha": 0.95},
        zorder=10,
    )


def add_combined_title_block(ax, layers: list[LayerData]) -> None:
    attrs: dict[str, str] = {}
    for layer in layers:
        if layer.records:
            attrs = layer.records[0].attributes
            if attrs:
                break

    summary = ", ".join(layer_display_name(layer.name) for layer in layers[:6])
    if len(layers) > 6:
        summary += ", ..."

    total_features = sum(len(layer.records) for layer in layers)
    lines = [
        "CARTA TATICA 2D",
        "Camadas combinadas",
        f"Layers: {len(layers)}",
        f"Elementos: {total_features}",
        f"Folha: {attrs.get('num_folha', 'n/d')}",
        "CRS: Portuguese National Grid",
        summary,
    ]
    ax.text(
        0.015,
        0.985,
        "\n".join(lines),
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=9,
        color="#1f1f1f",
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "#f4ecd2", "edgecolor": "#3d3a2f", "alpha": 0.95},
        zorder=10,
    )


def collect_bounds(layers: list[LayerData]) -> tuple[float, float, float, float]:
    all_points = [
        point
        for layer in layers
        for record in layer.records
        for point in record.points
    ]
    min_x = min(x for x, _ in all_points)
    max_x = max(x for x, _ in all_points)
    min_y = min(y for _, y in all_points)
    max_y = max(y for _, y in all_points)
    return min_x, max_x, min_y, max_y


def expand_bounds(
    min_x: float, max_x: float, min_y: float, max_y: float, padding_ratio: float = 0.12
) -> tuple[float, float, float, float]:
    width = max_x - min_x
    height = max_y - min_y
    padding = max(width, height) * padding_ratio
    return min_x - padding, max_x + padding, min_y - padding, max_y + padding


def plot_layer(ax, records: list[ShapeRecord], shape_type: int, layer_name: str) -> None:
    if not records:
        return

    all_points = [point for record in records for point in record.points]
    style = layer_style(layer_name, shape_type)
    if shape_type in {SHAPE_TYPE_POLYGON, SHAPE_TYPE_POLYGON_Z}:
        for record in records:
            for ring in split_parts(record.points, record.parts, min_points=3):
                ax.add_patch(
                    Polygon(
                        ring,
                        closed=True,
                        facecolor=style["facecolor"],
                        edgecolor=style["edgecolor"],
                        linewidth=1.2,
                        hatch=style["hatch"],
                        alpha=0.95,
                        zorder=3,
                    )
                )
                xs = [p[0] for p in ring]
                ys = [p[1] for p in ring]
                ax.plot(xs, ys, color=style["edgecolor"], linewidth=0.9, alpha=0.7, zorder=4)
    elif shape_type in {SHAPE_TYPE_POLYLINE, SHAPE_TYPE_POLYLINE_Z}:
        for record in records:
            for segment in split_parts(record.points, record.parts, min_points=2):
                xs = [p[0] for p in segment]
                ys = [p[1] for p in segment]
                ax.plot(
                    xs,
                    ys,
                    color=style["color"],
                    linewidth=style["linewidth"],
                    linestyle=style["linestyle"],
                    alpha=0.92,
                    zorder=3,
                )
    else:
        xs = [x for x, _ in all_points]
        ys = [y for _, y in all_points]
        scatter_kwargs = {
            "s": style["size"],
            "c": style["color"],
            "marker": style["marker"],
            "alpha": 0.9,
            "linewidths": 0.3,
            "zorder": 4,
        }
        if style["marker"] in {"x", "+"}:
            scatter_kwargs["linewidths"] = 0.8
        else:
            scatter_kwargs["edgecolors"] = "#111111"
        ax.scatter(xs, ys, **scatter_kwargs)


def finalize_map(ax, fig, output_path: Path, view_bounds: tuple[float, float, float, float]) -> None:
    view_min_x, view_max_x, view_min_y, view_max_y = view_bounds
    ax.set_xlim(view_min_x, view_max_x)
    ax.set_ylim(view_min_y, view_max_y)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])

    for spine in ax.spines.values():
        spine.set_linewidth(1.5)
        spine.set_color("#3d3a2f")

    fig.subplots_adjust(left=0.03, right=0.97, bottom=0.03, top=0.97)
    fig.savefig(output_path, dpi=fig.dpi)
    plt.close(fig)


def render_map(
    records: list[ShapeRecord],
    shape_type: int,
    output_path: Path,
    dpi: int,
    grid_spacing: float,
    layer_name: str,
    view_bounds: tuple[float, float, float, float] | None = None,
) -> None:
    all_points = [point for record in records for point in record.points]
    min_x = min(x for x, _ in all_points)
    max_x = max(x for x, _ in all_points)
    min_y = min(y for _, y in all_points)
    max_y = max(y for _, y in all_points)

    fig, ax = plt.subplots(figsize=(10, 10))
    fig.set_dpi(dpi)
    fig.patch.set_facecolor("#d9ccb1")
    ax.set_facecolor("#efe6c9")

    if view_bounds is None:
        view_bounds = expand_bounds(min_x, max_x, min_y, max_y)

    view_min_x, view_max_x, view_min_y, view_max_y = view_bounds
    view_width = view_max_x - view_min_x
    view_height = view_max_y - view_min_y

    add_grid(ax, view_min_x, view_max_x, view_min_y, view_max_y, grid_spacing)
    plot_layer(ax, records, shape_type, layer_name)

    add_scale_bar(ax, view_min_x, view_min_y, view_width, view_height)
    add_north_arrow(ax, view_min_x, view_min_y, view_width, view_height)
    add_title_block(
        ax,
        records[0].attributes if records else {},
        "CRS: Portuguese National Grid",
        layer_display_name(layer_name),
        shape_type_label(shape_type),
        len(records),
    )
    finalize_map(ax, fig, output_path, view_bounds)


def render_combined_map(
    layers: list[LayerData],
    output_path: Path,
    dpi: int,
    grid_spacing: float,
) -> None:
    min_x, max_x, min_y, max_y = collect_bounds(layers)
    view_bounds = expand_bounds(min_x, max_x, min_y, max_y)

    fig, ax = plt.subplots(figsize=(10, 10))
    fig.set_dpi(dpi)
    fig.patch.set_facecolor("#d9ccb1")
    ax.set_facecolor("#efe6c9")

    view_min_x, view_max_x, view_min_y, view_max_y = view_bounds
    view_width = view_max_x - view_min_x
    view_height = view_max_y - view_min_y

    add_grid(ax, view_min_x, view_max_x, view_min_y, view_max_y, grid_spacing)

    polygons = [layer for layer in layers if layer.shape_type in {SHAPE_TYPE_POLYGON, SHAPE_TYPE_POLYGON_Z}]
    polylines = [layer for layer in layers if layer.shape_type in {SHAPE_TYPE_POLYLINE, SHAPE_TYPE_POLYLINE_Z}]
    points = [layer for layer in layers if layer.shape_type in {SHAPE_TYPE_POINT, SHAPE_TYPE_POINT_Z}]

    for layer in polygons + polylines + points:
        plot_layer(ax, layer.records, layer.shape_type, layer.name)

    add_scale_bar(ax, view_min_x, view_min_y, view_width, view_height)
    add_north_arrow(ax, view_min_x, view_min_y, view_width, view_height)
    add_combined_title_block(ax, layers)
    finalize_map(ax, fig, output_path, view_bounds)

 
def render_aligned_layer_maps(
    layers: list[LayerData],
    output_dir: Path,
    dpi: int,
    grid_spacing: float,
) -> None:
    min_x, max_x, min_y, max_y = collect_bounds(layers)
    view_bounds = expand_bounds(min_x, max_x, min_y, max_y)
    output_dir.mkdir(parents=True, exist_ok=True)

    for layer in layers:
        output_path = output_dir / f"{layer.name}.png"
        render_map(
            layer.records,
            layer.shape_type,
            output_path,
            dpi,
            grid_spacing,
            layer.name,
            view_bounds=view_bounds,
        )
        print(f"Aligned map written to: {output_path}")


def load_dataset_layers() -> list[LayerData]:
    shp_files = list_dataset_shapefiles()
    if not shp_files:
        raise FileNotFoundError("No .shp files were found in dataset/.")

    layers: list[LayerData] = []
    for shp_path in shp_files:
        records, shape_type = read_shapefile(shp_path)
        if not records:
            print(f"Skipping empty layer: {shp_path.name}")
            continue
        layers.append(LayerData(name=shp_path.stem, records=records, shape_type=shape_type))
    return layers


def main() -> None:
    args = parse_args()
    if args.combined:
        layers = load_dataset_layers()
        if not layers:
            raise RuntimeError("No features were found in the available shapefiles.")

        render_combined_map(layers, args.output, args.dpi, args.grid)
        print(f"Combined map written to: {args.output}")
        return

    if args.all_aligned:
        layers = load_dataset_layers()
        if not layers:
            raise RuntimeError("No features were found in the available shapefiles.")

        render_aligned_layer_maps(layers, args.output_dir, args.dpi, args.grid)
        return

    if args.all:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        shp_files = list_dataset_shapefiles()
        if not shp_files:
            raise FileNotFoundError("No .shp files were found in dataset/.")
        for shp_path in shp_files:
            records, shape_type = read_shapefile(shp_path)
            if not records:
                print(f"Skipping empty layer: {shp_path.name}")
                continue
            output_path = args.output_dir / f"{shp_path.stem}.png"
            render_map(records, shape_type, output_path, args.dpi, args.grid, shp_path.stem)
            print(f"Map written to: {output_path}")
        return

    shp_path = resolve_input_path(args.input) if args.input else find_default_shapefile()
    records, shape_type = read_shapefile(shp_path)
    if not records:
        raise RuntimeError(f"No features found in {shp_path}")

    render_map(records, shape_type, args.output, args.dpi, args.grid, shp_path.stem)
    print(f"Map written to: {args.output}")


if __name__ == "__main__":
    main()
