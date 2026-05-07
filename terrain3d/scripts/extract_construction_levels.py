#!/usr/bin/env python3
"""Extract ground support samples from construction POLYGONZ base elevations."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import shapefile


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract construction footprint base elevations for a terrain blanket."
    )
    parser.add_argument(
        "--sheet",
        type=Path,
        default=Path("dataset/a_folha.shp"),
        help="Map sheet shapefile used to derive local-coordinate origin and target bounds.",
    )
    parser.add_argument(
        "--constructions",
        type=Path,
        default=Path("dataset/a_constr.shp"),
        help="Construction POLYGONZ shapefile.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("terrain3d/data/full_sheet_construction_levels.xyz"),
        help="Output XYZ file with local x/y/z construction-level samples.",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=5.0,
        help="Sampling spacing along construction footprint edges in meters.",
    )
    parser.add_argument(
        "--support-margin",
        type=float,
        default=0.0,
        help="Extra margin around the sheet for retaining support samples.",
    )
    return parser.parse_args()


def sample_segment(
    x0: float,
    y0: float,
    z0: float,
    x1: float,
    y1: float,
    z1: float,
    spacing: float,
) -> list[tuple[float, float, float]]:
    length = math.hypot(x1 - x0, y1 - y0)
    if length <= 1e-9:
        return [(x0, y0, z0)]

    steps = max(1, int(math.ceil(length / spacing)))
    points = []
    for index in range(steps + 1):
        t = index / steps
        points.append(
            (
                x0 + t * (x1 - x0),
                y0 + t * (y1 - y0),
                z0 + t * (z1 - z0),
            )
        )
    return points


def in_bounds(
    x: float,
    y: float,
    bounds: tuple[float, float, float, float],
) -> bool:
    min_x, min_y, max_x, max_y = bounds
    return min_x <= x <= max_x and min_y <= y <= max_y


def main() -> None:
    args = parse_args()
    if args.spacing <= 0:
        raise ValueError("--spacing must be positive")
    if args.support_margin < 0:
        raise ValueError("--support-margin must be non-negative")

    sheet = shapefile.Reader(str(args.sheet), encoding="latin1")
    min_x, min_y, max_x, max_y = sheet.bbox
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    target_bbox = (min_x, min_y, max_x, max_y)
    support_bbox = (
        min_x - args.support_margin,
        min_y - args.support_margin,
        max_x + args.support_margin,
        max_y + args.support_margin,
    )

    constructions = shapefile.Reader(str(args.constructions), encoding="latin1")
    samples: list[tuple[float, float, float, float, float]] = []
    seen: set[tuple[int, int, int]] = set()
    source_counts: dict[str, int] = {}

    for shape_record in constructions.iterShapeRecords():
        shape = shape_record.shape
        z_values = list(getattr(shape, "z", []))
        if len(z_values) != len(shape.points):
            continue

        source = shape_record.record.as_dict().get("source") or "construction"
        parts = list(shape.parts) + [len(shape.points)]
        for start, end in zip(parts[:-1], parts[1:]):
            point_indices = list(range(start, end))
            if len(point_indices) < 2:
                continue

            for index_a, index_b in zip(point_indices[:-1], point_indices[1:]):
                x0, y0 = shape.points[index_a]
                x1, y1 = shape.points[index_b]
                z0 = float(z_values[index_a])
                z1 = float(z_values[index_b])
                for world_x, world_y, level in sample_segment(x0, y0, z0, x1, y1, z1, args.spacing):
                    if not in_bounds(world_x, world_y, support_bbox):
                        continue
                    local_x = world_x - center_x
                    local_y = world_y - center_y
                    key = (round(local_x * 1000), round(local_y * 1000), round(level * 1000))
                    if key in seen:
                        continue
                    seen.add(key)
                    samples.append((local_x, local_y, level, world_x, world_y))
                    source_counts[source] = source_counts.get(source, 0) + 1

    if not samples:
        raise RuntimeError("No construction-level support points were found.")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as output:
        output.write("# local_x local_y z world_x world_y\n")
        output.write("# source construction_POLYGONZ_base_elevations\n")
        output.write(f"# center_x {center_x:.6f}\n")
        output.write(f"# center_y {center_y:.6f}\n")
        output.write(
            f"# target_bbox_local "
            f"{target_bbox[0] - center_x:.3f} {target_bbox[1] - center_y:.3f} "
            f"{target_bbox[2] - center_x:.3f} {target_bbox[3] - center_y:.3f}\n"
        )
        output.write(
            f"# support_bbox_local "
            f"{support_bbox[0] - center_x:.3f} {support_bbox[1] - center_y:.3f} "
            f"{support_bbox[2] - center_x:.3f} {support_bbox[3] - center_y:.3f}\n"
        )
        for local_x, local_y, level, world_x, world_y in samples:
            output.write(
                f"{local_x:.6f} {local_y:.6f} {level:.6f} {world_x:.6f} {world_y:.6f}\n"
            )

    print(
        f"Wrote {len(samples)} construction-level support samples "
        f"from {len(source_counts)} construction classes to {args.output}"
    )


if __name__ == "__main__":
    main()
