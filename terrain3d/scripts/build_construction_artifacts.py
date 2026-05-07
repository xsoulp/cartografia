#!/usr/bin/env python3
"""Build 3D OBJ artifacts for construction polygons."""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import shapefile


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extrude a_constr polygons into OBJ artifacts."
    )
    parser.add_argument(
        "--sheet",
        type=Path,
        default=Path("dataset/a_folha.shp"),
        help="Map sheet shapefile used to derive local-coordinate origin.",
    )
    parser.add_argument(
        "--constructions",
        type=Path,
        default=Path("dataset/a_constr.shp"),
        help="Construction polygon shapefile.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("terrain3d/output/construction_artifacts"),
        help="Directory where OBJ artifacts and manifest files are written.",
    )
    parser.add_argument(
        "--min-height",
        type=float,
        default=-1.0,
        help="Minimum h_campo value to export.",
    )
    parser.add_argument(
        "--assumed-height",
        type=float,
        default=6.0,
        help="Height in meters for construction polygons where h_campo is zero or missing.",
    )
    parser.add_argument(
        "--terrain-support",
        type=Path,
        default=Path("terrain3d/data/full_sheet_contours.xyz"),
        help="Contour support XYZ used to clamp construction bases above the blanket surface.",
    )
    parser.add_argument(
        "--base-clearance",
        type=float,
        default=0.05,
        help="Meters to lift construction bases above the interpolated blanket surface.",
    )
    return parser.parse_args()


def safe_name(value: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9_-]+", "_", value.strip())
    clean = re.sub(r"_+", "_", clean).strip("_")
    return clean or "construction"


def ring_ranges(parts: list[int], point_count: int) -> list[tuple[int, int]]:
    starts = list(parts)
    ends = starts[1:] + [point_count]
    return [(start, end) for start, end in zip(starts, ends) if end - start >= 4]


def without_closing_duplicate(
    points: list[tuple[float, float]],
    z_values: list[float],
    start: int,
    end: int,
) -> list[tuple[float, float, float]]:
    ring = [
        (points[index][0], points[index][1], z_values[index])
        for index in range(start, end)
    ]
    if len(ring) > 1 and ring[0][:2] == ring[-1][:2]:
        ring.pop()
    return ring


def write_material(output_dir: Path) -> None:
    material = output_dir / "construction_artifacts.mtl"
    material.write_text(
        "\n".join(
            [
                "newmtl actual_wall",
                "Ka 0.55 0.10 0.08",
                "Kd 0.90 0.16 0.10",
                "Ks 0.12 0.10 0.08",
                "",
                "newmtl actual_roof",
                "Ka 0.40 0.04 0.03",
                "Kd 0.72 0.08 0.05",
                "Ks 0.12 0.08 0.06",
                "",
                "newmtl actual_base",
                "Ka 0.24 0.04 0.03",
                "Kd 0.42 0.08 0.05",
                "Ks 0.04 0.04 0.04",
                "",
                "newmtl assumed_wall",
                "Ka 0.08 0.20 0.50",
                "Kd 0.10 0.34 0.92",
                "Ks 0.10 0.10 0.12",
                "",
                "newmtl assumed_roof",
                "Ka 0.04 0.12 0.34",
                "Kd 0.06 0.22 0.72",
                "Ks 0.08 0.08 0.12",
                "",
                "newmtl assumed_base",
                "Ka 0.03 0.08 0.22",
                "Kd 0.05 0.14 0.42",
                "Ks 0.04 0.04 0.04",
                "",
            ]
        ),
        encoding="utf-8",
    )


def load_terrain_samples(path: Path) -> list[tuple[float, float, float]]:
    samples: list[tuple[float, float, float]] = []
    with path.open("r", encoding="utf-8") as input_file:
        for line in input_file:
            if not line.strip() or line.startswith("#"):
                continue
            x, y, z, *_rest = (float(value) for value in line.split())
            samples.append((x, y, z))
    if not samples:
        raise RuntimeError(f"No terrain support samples loaded from {path}")
    return samples


class TerrainIndex:
    def __init__(self, samples: list[tuple[float, float, float]], cell_size: float = 100.0):
        self.samples = samples
        self.cell_size = cell_size
        self.cells: dict[tuple[int, int], list[tuple[float, float, float]]] = defaultdict(list)
        for sample in samples:
            cell = self.cell_for(sample[0], sample[1])
            self.cells[cell].append(sample)

    def cell_for(self, x: float, y: float) -> tuple[int, int]:
        return (math.floor(x / self.cell_size), math.floor(y / self.cell_size))

    def interpolate_idw(
        self,
        x: float,
        y: float,
        neighbors: int = 12,
        power: float = 2.0,
    ) -> float:
        cell_x, cell_y = self.cell_for(x, y)
        distances: list[tuple[float, float]] = []
        searched_radius = 0

        for radius in range(0, 12):
            searched_radius = radius
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if radius > 0 and abs(dx) != radius and abs(dy) != radius:
                        continue
                    for sample_x, sample_y, sample_z in self.cells.get((cell_x + dx, cell_y + dy), []):
                        squared_distance = (sample_x - x) ** 2 + (sample_y - y) ** 2
                        if squared_distance <= 1e-6:
                            return sample_z
                        distances.append((squared_distance, sample_z))

            if len(distances) >= neighbors:
                distances.sort(key=lambda item: item[0])
                kth_distance = distances[neighbors - 1][0]
                next_cell_distance = ((radius + 0.5) * self.cell_size) ** 2
                if kth_distance < next_cell_distance:
                    break

        if len(distances) < neighbors:
            for sample_x, sample_y, sample_z in self.samples:
                squared_distance = (sample_x - x) ** 2 + (sample_y - y) ** 2
                if squared_distance <= 1e-6:
                    return sample_z
                distances.append((squared_distance, sample_z))

        distances.sort(key=lambda item: item[0])
        weighted_sum = 0.0
        weight_total = 0.0
        for squared_distance, sample_z in distances[:neighbors]:
            weight = 1.0 / (math.sqrt(squared_distance) ** power)
            weighted_sum += weight * sample_z
            weight_total += weight
        return weighted_sum / weight_total


def clamp_rings_to_blanket(
    rings: list[list[tuple[float, float, float]]],
    terrain_index: TerrainIndex,
    center_x: float,
    center_y: float,
    clearance: float,
) -> list[list[tuple[float, float, float]]]:
    clamped_rings: list[list[tuple[float, float, float]]] = []
    for ring in rings:
        clamped_ring = []
        for world_x, world_y, base_z in ring:
            terrain_z = terrain_index.interpolate_idw(world_x - center_x, world_y - center_y)
            clamped_ring.append((world_x, world_y, max(base_z, terrain_z + clearance)))
        clamped_rings.append(clamped_ring)
    return clamped_rings


def signed_area_xy(ring: list[tuple[float, float, float]]) -> float:
    area = 0.0
    for index, point in enumerate(ring):
        next_point = ring[(index + 1) % len(ring)]
        area += point[0] * next_point[1] - next_point[0] * point[1]
    return area / 2.0


def is_point_in_triangle(
    point: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
) -> bool:
    def cross(u: tuple[float, float], v: tuple[float, float], w: tuple[float, float]) -> float:
        return (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])

    ab = cross(a, b, point)
    bc = cross(b, c, point)
    ca = cross(c, a, point)
    return (ab >= -1e-9 and bc >= -1e-9 and ca >= -1e-9) or (
        ab <= 1e-9 and bc <= 1e-9 and ca <= 1e-9
    )


def triangulate_ring(ring: list[tuple[float, float, float]]) -> list[tuple[int, int, int]]:
    """Return triangle vertex indices for a simple polygon ring using ear clipping."""
    if len(ring) < 3:
        return []
    if len(ring) == 3:
        return [(0, 1, 2)]

    orientation = 1.0 if signed_area_xy(ring) >= 0.0 else -1.0
    remaining = list(range(len(ring)))
    triangles: list[tuple[int, int, int]] = []

    guard = 0
    while len(remaining) > 3 and guard < len(ring) * len(ring):
        guard += 1
        clipped = False
        for cursor, current_index in enumerate(remaining):
            previous_index = remaining[cursor - 1]
            next_index = remaining[(cursor + 1) % len(remaining)]

            previous = ring[previous_index]
            current = ring[current_index]
            next_point = ring[next_index]
            cross = (
                (current[0] - previous[0]) * (next_point[1] - current[1])
                - (current[1] - previous[1]) * (next_point[0] - current[0])
            )
            if cross * orientation <= 1e-9:
                continue

            triangle_xy = (
                (previous[0], previous[1]),
                (current[0], current[1]),
                (next_point[0], next_point[1]),
            )
            has_contained_point = False
            for test_index in remaining:
                if test_index in (previous_index, current_index, next_index):
                    continue
                test_point = ring[test_index]
                if is_point_in_triangle((test_point[0], test_point[1]), *triangle_xy):
                    has_contained_point = True
                    break
            if has_contained_point:
                continue

            triangles.append((previous_index, current_index, next_index))
            del remaining[cursor]
            clipped = True
            break

        if not clipped:
            # Last-resort fan keeps export robust for malformed rings.
            return [(remaining[0], remaining[i], remaining[i + 1]) for i in range(1, len(remaining) - 1)]

    if len(remaining) == 3:
        triangles.append((remaining[0], remaining[1], remaining[2]))
    return triangles


def append_artifact_obj(
    lines: list[str],
    object_name: str,
    rings: list[list[tuple[float, float, float]]],
    height: float,
    center_x: float,
    center_y: float,
    vertex_offset: int,
    material_prefix: str,
) -> int:
    lines.append(f"o {object_name}")

    ring_indices: list[tuple[list[int], list[int]]] = []
    next_index = vertex_offset

    for ring in rings:
        base_indices: list[int] = []
        roof_indices: list[int] = []
        for world_x, world_y, base_z in ring:
            local_x = world_x - center_x
            local_y = world_y - center_y
            lines.append(f"v {local_x:.6f} {local_y:.6f} {base_z:.6f}")
            base_indices.append(next_index)
            next_index += 1
        for world_x, world_y, base_z in ring:
            local_x = world_x - center_x
            local_y = world_y - center_y
            lines.append(f"v {local_x:.6f} {local_y:.6f} {base_z + height:.6f}")
            roof_indices.append(next_index)
            next_index += 1
        ring_indices.append((base_indices, roof_indices))

    lines.append(f"usemtl {material_prefix}_base")
    for ring, (base_indices, _roof_indices) in zip(rings, ring_indices):
        for triangle in triangulate_ring(ring):
            face = [base_indices[index] for index in reversed(triangle)]
            lines.append("f " + " ".join(str(index) for index in face))

    lines.append(f"usemtl {material_prefix}_roof")
    for ring, (_base_indices, roof_indices) in zip(rings, ring_indices):
        for triangle in triangulate_ring(ring):
            face = [roof_indices[index] for index in triangle]
            lines.append("f " + " ".join(str(index) for index in face))

    lines.append(f"usemtl {material_prefix}_wall")
    for base_indices, roof_indices in ring_indices:
        count = len(base_indices)
        for index in range(count):
            next_ring_index = (index + 1) % count
            lines.append(
                "f "
                f"{base_indices[index]} "
                f"{base_indices[next_ring_index]} "
                f"{roof_indices[next_ring_index]} "
                f"{roof_indices[index]}"
            )

    lines.append("")
    return next_index


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_material(args.output_dir)

    sheet = shapefile.Reader(str(args.sheet), encoding="latin1")
    min_x, min_y, max_x, max_y = sheet.bbox
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    terrain_samples = load_terrain_samples(args.terrain_support)
    terrain_index = TerrainIndex(terrain_samples)

    constructions = shapefile.Reader(str(args.constructions), encoding="latin1")
    combined_all_lines = [
        "# Construction artifacts",
        f"# local_origin_world_x {center_x:.6f}",
        f"# local_origin_world_y {center_y:.6f}",
        "mtllib construction_artifacts.mtl",
        "",
    ]
    combined_actual_lines = [
        "# Construction artifacts with actual h_campo",
        f"# local_origin_world_x {center_x:.6f}",
        f"# local_origin_world_y {center_y:.6f}",
        "mtllib construction_artifacts.mtl",
        "",
    ]
    combined_assumed_lines = [
        f"# Construction artifacts with assumed {args.assumed_height:g} m height",
        f"# local_origin_world_x {center_x:.6f}",
        f"# local_origin_world_y {center_y:.6f}",
        "mtllib construction_artifacts.mtl",
        "",
    ]
    manifest = []
    combined_all_vertex_offset = 1
    combined_actual_vertex_offset = 1
    combined_assumed_vertex_offset = 1

    for shape_index, shape_record in enumerate(constructions.iterShapeRecords()):
        record = shape_record.record.as_dict()
        raw_height = float(record.get("h_campo") or 0.0)
        has_actual_height = raw_height > 0.0
        height = raw_height if has_actual_height else args.assumed_height
        if height <= args.min_height:
            continue

        shape = shape_record.shape
        z_values = list(getattr(shape, "z", []))
        if len(z_values) != len(shape.points):
            continue

        rings = [
            without_closing_duplicate(shape.points, z_values, start, end)
            for start, end in ring_ranges(list(shape.parts), len(shape.points))
        ]
        rings = [ring for ring in rings if len(ring) >= 3]
        if not rings:
            continue
        rings = clamp_rings_to_blanket(
            rings,
            terrain_index,
            center_x,
            center_y,
            args.base_clearance,
        )

        source = record.get("source") or "construction"
        height_kind = "actual" if has_actual_height else "assumed"
        material_prefix = "actual" if has_actual_height else "assumed"
        object_name = f"construction_{shape_index:04d}_{height_kind}_{safe_name(source)}"
        file_name = f"{object_name}_h{height:g}m.obj"
        obj_path = args.output_dir / file_name

        lines = [
            f"# {object_name}",
            f"# source {source}",
            f"# h_campo {raw_height:.6f}",
            f"# modeled_height {height:.6f}",
            f"# height_kind {height_kind}",
            f"# local_origin_world_x {center_x:.6f}",
            f"# local_origin_world_y {center_y:.6f}",
            "mtllib construction_artifacts.mtl",
            "",
        ]
        append_artifact_obj(lines, object_name, rings, height, center_x, center_y, 1, material_prefix)
        obj_path.write_text("\n".join(lines), encoding="utf-8")

        combined_all_vertex_offset = append_artifact_obj(
            combined_all_lines,
            object_name,
            rings,
            height,
            center_x,
            center_y,
            combined_all_vertex_offset,
            material_prefix,
        )
        if has_actual_height:
            combined_actual_vertex_offset = append_artifact_obj(
                combined_actual_lines,
                object_name,
                rings,
                height,
                center_x,
                center_y,
                combined_actual_vertex_offset,
                material_prefix,
            )
        else:
            combined_assumed_vertex_offset = append_artifact_obj(
                combined_assumed_lines,
                object_name,
                rings,
                height,
                center_x,
                center_y,
                combined_assumed_vertex_offset,
                material_prefix,
            )

        base_z_values = [point[2] for ring in rings for point in ring]
        bbox = [float(value) for value in shape.bbox]
        manifest.append(
            {
                "shape_index": shape_index,
                "source": source,
                "height_kind": height_kind,
                "h_campo_m": raw_height,
                "height_m": height,
                "base_z_min_m": min(base_z_values),
                "base_z_max_m": max(base_z_values),
                "roof_z_min_m": min(base_z_values) + height,
                "roof_z_max_m": max(base_z_values) + height,
                "shape_area_m2": record.get("Shape_Area"),
                "shape_length_m": record.get("Shape_Leng"),
                "parts": len(rings),
                "vertices_per_part": [len(ring) for ring in rings],
                "world_bbox": bbox,
                "local_bbox": [
                    bbox[0] - center_x,
                    bbox[1] - center_y,
                    bbox[2] - center_x,
                    bbox[3] - center_y,
                ],
                "obj": file_name,
            }
        )

    (args.output_dir / "constructions_all.obj").write_text(
        "\n".join(combined_all_lines),
        encoding="utf-8",
    )
    (args.output_dir / "constructions_actual_height.obj").write_text(
        "\n".join(combined_actual_lines),
        encoding="utf-8",
    )
    (args.output_dir / "constructions_assumed_6m.obj").write_text(
        "\n".join(combined_assumed_lines),
        encoding="utf-8",
    )
    (args.output_dir / "manifest.json").write_text(
        json.dumps(
            {
                "source": str(args.constructions),
                "height_field": "h_campo",
                "assumed_height_m": args.assumed_height,
                "base_elevation_source": "max(POLYGONZ geometry, terrain blanket IDW + base_clearance)",
                "terrain_support": str(args.terrain_support),
                "base_clearance_m": args.base_clearance,
                "local_origin": {"world_x": center_x, "world_y": center_y},
                "artifact_count": len(manifest),
                "actual_height_count": sum(1 for item in manifest if item["height_kind"] == "actual"),
                "assumed_height_count": sum(1 for item in manifest if item["height_kind"] == "assumed"),
                "artifacts": manifest,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"Wrote {len(manifest)} construction OBJ artifacts to {args.output_dir}")
    print(f"Wrote combined OBJ to {args.output_dir / 'constructions_all.obj'}")
    print(f"Wrote actual-height OBJ to {args.output_dir / 'constructions_actual_height.obj'}")
    print(f"Wrote assumed-height OBJ to {args.output_dir / 'constructions_assumed_6m.obj'}")
    print(f"Wrote manifest to {args.output_dir / 'manifest.json'}")


if __name__ == "__main__":
    main()
