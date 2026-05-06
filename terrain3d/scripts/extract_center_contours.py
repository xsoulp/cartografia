#!/usr/bin/env python3
"""Extract contour support samples for either the center patch or the whole sheet."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import shapefile


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract sampled contour points for a terrain blanket."
    )
    parser.add_argument(
        "--sheet",
        type=Path,
        default=Path("dataset/a_folha.shp"),
        help="Map sheet shapefile used to derive the center.",
    )
    parser.add_argument(
        "--contours",
        type=Path,
        default=Path("dataset/l_curva_nivel.shp"),
        help="Contour shapefile with Z values.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("terrain3d/data/center_contours.xyz"),
        help="Output XYZ file with local x/y/z samples.",
    )
    parser.add_argument(
        "--whole-sheet",
        action="store_true",
        help="Use the full map sheet as the target area instead of the 1000 m center patch.",
    )
    parser.add_argument(
        "--target-half",
        type=float,
        default=500.0,
        help="Half-size of the center output patch in meters.",
    )
    parser.add_argument(
        "--support-half",
        type=float,
        default=800.0,
        help="Half-size of the contour support window in meters for center mode.",
    )
    parser.add_argument(
        "--support-margin",
        type=float,
        default=100.0,
        help="Extra margin around the target area used for contour support in whole-sheet mode.",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=2.0,
        help="Sampling spacing along each clipped contour segment in meters.",
    )
    return parser.parse_args()


def build_bounds(
    sheet_bbox: tuple[float, float, float, float],
    whole_sheet: bool,
    target_half: float,
    support_half: float,
    support_margin: float,
) -> tuple[tuple[float, float, float, float], tuple[float, float, float, float], float, float]:
    min_x, min_y, max_x, max_y = sheet_bbox
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0

    if whole_sheet:
        target_bbox = (min_x, min_y, max_x, max_y)
        support_bbox = (
            min_x - support_margin,
            min_y - support_margin,
            max_x + support_margin,
            max_y + support_margin,
        )
    else:
        target_bbox = (
            center_x - target_half,
            center_y - target_half,
            center_x + target_half,
            center_y + target_half,
        )
        support_bbox = (
            center_x - support_half,
            center_y - support_half,
            center_x + support_half,
            center_y + support_half,
        )

    return target_bbox, support_bbox, center_x, center_y


def liang_barsky(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    xmin: float,
    ymin: float,
    xmax: float,
    ymax: float,
) -> tuple[float, float, float, float] | None:
    dx = x1 - x0
    dy = y1 - y0
    p = (-dx, dx, -dy, dy)
    q = (x0 - xmin, xmax - x0, y0 - ymin, ymax - y0)
    u1 = 0.0
    u2 = 1.0

    for pi, qi in zip(p, q):
        if abs(pi) < 1e-12:
            if qi < 0:
                return None
            continue

        t = qi / pi
        if pi < 0:
            if t > u2:
                return None
            if t > u1:
                u1 = t
        else:
            if t < u1:
                return None
            if t < u2:
                u2 = t

    return (
        x0 + u1 * dx,
        y0 + u1 * dy,
        x0 + u2 * dx,
        y0 + u2 * dy,
    )


def sample_segment(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    spacing: float,
) -> list[tuple[float, float]]:
    length = math.hypot(x1 - x0, y1 - y0)
    if length <= 1e-9:
        return [(x0, y0)]

    steps = max(1, int(math.ceil(length / spacing)))
    points = []
    for index in range(steps + 1):
        t = index / steps
        points.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return points


def main() -> None:
    args = parse_args()
    if args.support_half < args.target_half and not args.whole_sheet:
        raise ValueError("--support-half must be greater than or equal to --target-half")
    if args.support_margin < 0:
        raise ValueError("--support-margin must be non-negative")
    if args.spacing <= 0:
        raise ValueError("--spacing must be positive")

    sheet = shapefile.Reader(str(args.sheet), encoding="latin1")
    target_bbox, support_bbox, center_x, center_y = build_bounds(
        sheet.bbox,
        args.whole_sheet,
        args.target_half,
        args.support_half,
        args.support_margin,
    )

    contour_reader = shapefile.Reader(str(args.contours), encoding="latin1")
    samples: list[tuple[float, float, float, float, float]] = []
    seen: set[tuple[int, int, int]] = set()
    levels: set[float] = set()

    for shape in contour_reader.iterShapes():
        z_values = getattr(shape, "z", [])
        if not z_values:
            continue
        level = float(z_values[0])
        parts = list(shape.parts) + [len(shape.points)]
        for start, end in zip(parts[:-1], parts[1:]):
            points = shape.points[start:end]
            for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
                clipped = liang_barsky(x0, y0, x1, y1, *support_bbox)
                if clipped is None:
                    continue
                cx0, cy0, cx1, cy1 = clipped
                for world_x, world_y in sample_segment(cx0, cy0, cx1, cy1, args.spacing):
                    local_x = world_x - center_x
                    local_y = world_y - center_y
                    key = (round(local_x * 1000), round(local_y * 1000), round(level * 1000))
                    if key in seen:
                        continue
                    seen.add(key)
                    levels.add(level)
                    samples.append((local_x, local_y, level, world_x, world_y))

    if not samples:
        raise RuntimeError("No contour support points were found in the requested window.")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as output:
        output.write("# local_x local_y z world_x world_y\n")
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
        f"Wrote {len(samples)} contour support samples across {len(levels)} contour levels "
        f"to {args.output}"
    )


if __name__ == "__main__":
    main()
