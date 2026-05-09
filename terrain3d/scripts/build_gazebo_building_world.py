#!/usr/bin/env python3
"""Generate Gazebo SDF building models and a world from construction OBJs."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import shutil
from pathlib import Path
from xml.sax.saxutils import escape


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build one Gazebo SDF model per construction artifact and a world including all models."
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("terrain3d/output/construction_artifacts/manifest.json"),
        help="Construction artifact manifest produced by build_construction_artifacts.py.",
    )
    parser.add_argument(
        "--construction-dir",
        type=Path,
        default=Path("terrain3d/output/construction_artifacts"),
        help="Directory containing per-building OBJ files and construction_artifacts.mtl.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("terrain3d/gz_assets/gz_worlds/gazebo_buildings"),
        help="Output directory for Gazebo models and world.sdf.",
    )
    parser.add_argument(
        "--models-dir",
        type=Path,
        default=Path("terrain3d/gz_assets/gz_models"),
        help="Directory where generated models are written. World and mesh references use absolute file:// URIs from this folder.",
    )
    parser.add_argument(
        "--sdf-version",
        default="1.7",
        help="SDF version for generated model and world files.",
    )
    parser.add_argument(
        "--world-name",
        default="cartografia_buildings",
        help="Name used for the generated Gazebo world.",
    )
    parser.add_argument(
        "--model-name-prefix",
        default="",
        help="Optional prefix added to every generated model name to avoid collisions in a shared model library.",
    )
    parser.add_argument(
        "--no-collision",
        action="store_true",
        help="Generate visuals only. By default, each building also gets mesh collision geometry.",
    )
    parser.add_argument(
        "--window-size",
        type=float,
        default=None,
        help="Clip output to an axis-aligned square window of this size in local meters.",
    )
    parser.add_argument(
        "--window-min-x",
        type=float,
        default=None,
        help="Local-space minimum X for the clip window. Requires --window-size and --window-min-y.",
    )
    parser.add_argument(
        "--window-min-y",
        type=float,
        default=None,
        help="Local-space minimum Y for the clip window. Requires --window-size and --window-min-x.",
    )
    parser.add_argument(
        "--window-center-x",
        type=float,
        default=None,
        help="Local-space center X for the clip window. Requires --window-size and --window-center-y.",
    )
    parser.add_argument(
        "--window-center-y",
        type=float,
        default=None,
        help="Local-space center Y for the clip window. Requires --window-size and --window-center-x.",
    )
    parser.add_argument(
        "--auto-dense-window",
        action="store_true",
        help="Automatically choose the densest local square window based on building footprint centroids.",
    )
    parser.add_argument(
        "--zero-min-z",
        action="store_true",
        help="Shift world poses so the lowest selected building base is placed at z=0.",
    )
    return parser.parse_args()


def safe_token(value: str) -> str:
    clean = "".join(ch if ch.isalnum() else "_" for ch in value.lower())
    return "_".join(part for part in clean.split("_") if part)


def safe_model_name(shape_index: int, source: str, height_kind: str, prefix: str = "") -> str:
    prefix_clean = safe_token(prefix)
    if prefix_clean:
        prefix_clean = f"{prefix_clean}_"
    clean = safe_token(source)
    return f"{prefix_clean}building_{shape_index:04d}_{height_kind}_{clean or 'construction'}"


def parse_face_vertex(token: str) -> int:
    vertex_index = token.split("/", 1)[0]
    if not vertex_index:
        raise ValueError(f"Unsupported face token {token!r}")
    return int(vertex_index)


def subtract(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
    if length <= 1e-12:
        return (0.0, 0.0, 1.0)
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def triangle_normal(
    vertices: list[tuple[float, float, float]],
    face: tuple[int, int, int],
) -> tuple[float, float, float]:
    a = vertices[face[0] - 1]
    b = vertices[face[1] - 1]
    c = vertices[face[2] - 1]
    return normalize(cross(subtract(b, a), subtract(c, a)))


def orient_triangle(
    vertices: list[tuple[float, float, float]],
    face: tuple[int, int, int],
    center_xy: tuple[float, float],
    base_z: float,
    roof_z: float,
    expected_wall_normal: tuple[float, float] | None = None,
) -> tuple[int, int, int]:
    z_values = [vertices[index - 1][2] for index in face]
    normal = triangle_normal(vertices, face)

    if all(abs(z - roof_z) <= 1e-6 for z in z_values):
        if normal[2] < 0.0:
            return (face[0], face[2], face[1])
        return face

    if all(abs(z - base_z) <= 1e-6 for z in z_values):
        if normal[2] > 0.0:
            return (face[0], face[2], face[1])
        return face

    if expected_wall_normal is not None:
        if normal[0] * expected_wall_normal[0] + normal[1] * expected_wall_normal[1] < 0.0:
            return (face[0], face[2], face[1])
        return face

    centroid_x = sum(vertices[index - 1][0] for index in face) / 3.0
    centroid_y = sum(vertices[index - 1][1] for index in face) / 3.0
    radial = (centroid_x - center_xy[0], centroid_y - center_xy[1])
    if normal[0] * radial[0] + normal[1] * radial[1] < 0.0:
        return (face[0], face[2], face[1])
    return face


def triangulate_face(face: list[int]) -> list[tuple[int, int, int]]:
    if len(face) < 3:
        return []
    if len(face) == 3:
        return [(face[0], face[1], face[2])]
    return [(face[0], face[index], face[index + 1]) for index in range(1, len(face) - 1)]


def signed_area(points: list[tuple[float, float]]) -> float:
    area = 0.0
    for index, point in enumerate(points):
        next_point = points[(index + 1) % len(points)]
        area += point[0] * next_point[1] - next_point[0] * point[1]
    return area / 2.0


def is_point_in_triangle_2d(
    point: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
) -> bool:
    def cross_2d(u: tuple[float, float], v: tuple[float, float], w: tuple[float, float]) -> float:
        return (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])

    ab = cross_2d(a, b, point)
    bc = cross_2d(b, c, point)
    ca = cross_2d(c, a, point)
    return (ab >= -1e-9 and bc >= -1e-9 and ca >= -1e-9) or (
        ab <= 1e-9 and bc <= 1e-9 and ca <= 1e-9
    )


def triangulate_ring_indices(
    vertices: list[tuple[float, float, float]],
    indices: list[int],
) -> list[tuple[int, int, int]]:
    if len(indices) < 3:
        return []
    if len(indices) == 3:
        return [(indices[0], indices[1], indices[2])]

    points = [(vertices[index - 1][0], vertices[index - 1][1]) for index in indices]
    orientation = 1.0 if signed_area(points) >= 0.0 else -1.0
    remaining = list(range(len(indices)))
    triangles: list[tuple[int, int, int]] = []

    guard = 0
    while len(remaining) > 3 and guard < len(indices) * len(indices):
        guard += 1
        clipped = False
        for cursor, current_index in enumerate(remaining):
            previous_index = remaining[cursor - 1]
            next_index = remaining[(cursor + 1) % len(remaining)]
            previous = points[previous_index]
            current = points[current_index]
            next_point = points[next_index]
            cross_value = (
                (current[0] - previous[0]) * (next_point[1] - current[1])
                - (current[1] - previous[1]) * (next_point[0] - current[0])
            )
            if cross_value * orientation <= 1e-9:
                continue

            triangle_points = (previous, current, next_point)
            if any(
                is_point_in_triangle_2d(points[test_index], *triangle_points)
                for test_index in remaining
                if test_index not in (previous_index, current_index, next_index)
            ):
                continue

            triangles.append(
                (indices[previous_index], indices[current_index], indices[next_index])
            )
            del remaining[cursor]
            clipped = True
            break

        if not clipped:
            return [
                (indices[remaining[0]], indices[remaining[index]], indices[remaining[index + 1]])
                for index in range(1, len(remaining) - 1)
            ]

    if len(remaining) == 3:
        triangles.append((indices[remaining[0]], indices[remaining[1]], indices[remaining[2]]))
    return triangles


def flat_prism_face_groups(
    vertices: list[tuple[float, float, float]],
    vertices_per_part: list[int],
    material_prefix: str,
) -> list[tuple[str, list[list[int]]]]:
    base_faces: list[list[int]] = []
    roof_faces: list[list[int]] = []
    wall_faces: list[list[int]] = []
    cursor = 0

    for part_vertex_count in vertices_per_part:
        base_indices = list(range(cursor + 1, cursor + part_vertex_count + 1))
        roof_indices = list(
            range(cursor + part_vertex_count + 1, cursor + part_vertex_count * 2 + 1)
        )
        base_triangles = triangulate_ring_indices(vertices, base_indices)
        for triangle in base_triangles:
            base_faces.append(list(triangle))
            roof_faces.append([roof_indices[base_indices.index(index)] for index in triangle])

        for index, base_index in enumerate(base_indices):
            next_index = (index + 1) % part_vertex_count
            wall_faces.append(
                [
                    base_index,
                    base_indices[next_index],
                    roof_indices[next_index],
                    roof_indices[index],
                ]
            )
        cursor += part_vertex_count * 2

    return [
        (f"{material_prefix}_base", base_faces),
        (f"{material_prefix}_roof", roof_faces),
        (f"{material_prefix}_wall", wall_faces),
    ]


def wall_normal_for_face(
    vertices: list[tuple[float, float, float]],
    face: list[int],
    base_z: float,
    part_area_by_base_vertex: dict[int, float],
) -> tuple[float, float] | None:
    base_indices = [index for index in face if abs(vertices[index - 1][2] - base_z) <= 1e-6]
    if len(base_indices) < 2:
        return None

    first = vertices[base_indices[0] - 1]
    second = vertices[base_indices[1] - 1]
    dx = second[0] - first[0]
    dy = second[1] - first[1]
    length = math.hypot(dx, dy)
    if length <= 1e-12:
        return None

    part_area = part_area_by_base_vertex.get(base_indices[0], 0.0)
    if part_area < 0.0:
        normal = (-dy / length, dx / length)
    else:
        normal = (dy / length, -dx / length)
    return normal


def remove_duplicate_closing_vertices(
    vertices: list[tuple[float, float, float]],
    face_vertex_indices: list[list[int]],
    vertices_per_part: list[int] | None,
) -> tuple[list[tuple[float, float, float]], list[list[int]], list[int] | None]:
    if not vertices_per_part:
        return vertices, face_vertex_indices, vertices_per_part

    cleaned_vertices: list[tuple[float, float, float]] = []
    index_map: dict[int, int] = {}
    cleaned_vertices_per_part: list[int] = []
    cursor = 0

    for part_vertex_count in vertices_per_part:
        base_start = cursor
        roof_start = cursor + part_vertex_count
        next_cursor = cursor + part_vertex_count * 2

        base_vertices = vertices[base_start:roof_start]
        roof_vertices = vertices[roof_start:next_cursor]
        keep_count = part_vertex_count
        if (
            part_vertex_count > 1
            and abs(base_vertices[0][0] - base_vertices[-1][0]) <= 1e-9
            and abs(base_vertices[0][1] - base_vertices[-1][1]) <= 1e-9
        ):
            keep_count -= 1

        part_base_new_indices: list[int] = []
        part_roof_new_indices: list[int] = []
        for index in range(keep_count):
            cleaned_vertices.append(base_vertices[index])
            part_base_new_indices.append(len(cleaned_vertices))
        for index in range(keep_count):
            cleaned_vertices.append(roof_vertices[index])
            part_roof_new_indices.append(len(cleaned_vertices))

        for index in range(part_vertex_count):
            old_base_index = base_start + index + 1
            old_roof_index = roof_start + index + 1
            mapped_index = index if index < keep_count else 0
            index_map[old_base_index] = part_base_new_indices[mapped_index]
            index_map[old_roof_index] = part_roof_new_indices[mapped_index]

        cleaned_vertices_per_part.append(keep_count)
        cursor = next_cursor

    if cursor != len(vertices):
        raise RuntimeError("Part vertex counts do not consume all OBJ vertices.")

    cleaned_faces: list[list[int]] = []
    for face in face_vertex_indices:
        mapped_face: list[int] = []
        for index in face:
            mapped_index = index_map[index]
            if not mapped_face or mapped_face[-1] != mapped_index:
                mapped_face.append(mapped_index)
        if len(mapped_face) > 1 and mapped_face[0] == mapped_face[-1]:
            mapped_face.pop()
        if len(set(mapped_face)) >= 3:
            cleaned_faces.append(mapped_face)

    return cleaned_vertices, cleaned_faces, cleaned_vertices_per_part


def gazebo_material_text() -> str:
    return "\n".join(
        [
            "newmtl actual_wall",
            "Ka 0.28 0.28 0.28",
            "Kd 0.62 0.62 0.62",
            "Ks 0.06 0.06 0.06",
            "",
            "newmtl actual_roof",
            "Ka 0.42 0.18 0.08",
            "Kd 0.72 0.33 0.16",
            "Ks 0.08 0.08 0.08",
            "",
            "newmtl actual_base",
            "Ka 0.18 0.18 0.18",
            "Kd 0.45 0.45 0.45",
            "Ks 0.04 0.04 0.04",
            "",
            "newmtl assumed_wall",
            "Ka 0.28 0.28 0.28",
            "Kd 0.62 0.62 0.62",
            "Ks 0.06 0.06 0.06",
            "",
            "newmtl assumed_roof",
            "Ka 0.42 0.18 0.08",
            "Kd 0.72 0.33 0.16",
            "Ks 0.08 0.08 0.08",
            "",
            "newmtl assumed_base",
            "Ka 0.18 0.18 0.18",
            "Kd 0.45 0.45 0.45",
            "Ks 0.04 0.04 0.04",
            "",
        ]
    )


def rewrite_obj_vertices(
    input_path: Path,
    output_path: Path,
    origin: tuple[float, float, float],
    flatten_height: float | None = None,
    vertices_per_part: list[int] | None = None,
) -> None:
    ox, oy, oz = origin
    lines: list[tuple[str, str]] = []
    vertices: list[tuple[float, float, float]] = []
    face_vertex_indices: list[list[int]] = []
    material_prefix = "assumed"

    with input_path.open("r", encoding="utf-8") as source:
        for line in source:
            if line.startswith("v "):
                parts = line.split()
                if len(parts) < 4:
                    lines.append(("raw", line))
                    continue

                x = float(parts[1]) - ox
                y = float(parts[2]) - oy
                z = float(parts[3]) - oz
                vertices.append((x, y, z))
                lines.append(("vertex", ""))
                continue

            if line.startswith("f "):
                tokens = line.split()[1:]
                if len(tokens) < 3:
                    lines.append(("raw", line))
                    continue
                face_vertex_indices.append([parse_face_vertex(token) for token in tokens])
                lines.append(("face", ""))
                continue

            if line.startswith("usemtl ") and line.split()[1].endswith("_base"):
                material_prefix = line.split()[1].removesuffix("_base")
            lines.append(("raw", line))

    if flatten_height is not None:
        flattened_vertices: list[tuple[float, float, float]] = []

        if vertices_per_part:
            cursor = 0
            for part_vertex_count in vertices_per_part:
                base_end = cursor + part_vertex_count
                roof_end = base_end + part_vertex_count
                if roof_end > len(vertices):
                    raise RuntimeError(
                        f"Part vertex counts in manifest do not match OBJ vertices in {input_path}."
                    )
                flattened_vertices.extend((x, y, 0.0) for x, y, _z in vertices[cursor:base_end])
                flattened_vertices.extend(
                    (x, y, flatten_height) for x, y, _z in vertices[base_end:roof_end]
                )
                cursor = roof_end

            if cursor != len(vertices):
                raise RuntimeError(
                    f"Part vertex counts consumed {cursor} vertices, but {input_path} has {len(vertices)}."
                )
        else:
            if len(vertices) % 2 != 0:
                raise RuntimeError(f"Expected an even vertex count in {input_path} to flatten extrusion.")
            half = len(vertices) // 2
            for index, (x, y, _z) in enumerate(vertices):
                flattened_vertices.append((x, y, 0.0 if index < half else flatten_height))

        vertices = flattened_vertices
        vertices, face_vertex_indices, vertices_per_part = remove_duplicate_closing_vertices(
            vertices,
            face_vertex_indices,
            vertices_per_part,
        )

    base_z = min(vertex[2] for vertex in vertices)
    roof_z = max(vertex[2] for vertex in vertices)
    center_xy = (
        sum(vertex[0] for vertex in vertices) / len(vertices),
        sum(vertex[1] for vertex in vertices) / len(vertices),
    )
    part_area_by_base_vertex: dict[int, float] = {}
    if vertices_per_part:
        cursor = 0
        for part_vertex_count in vertices_per_part:
            base_indices = list(range(cursor + 1, cursor + part_vertex_count + 1))
            ring_points = [(vertices[index - 1][0], vertices[index - 1][1]) for index in base_indices]
            part_area = signed_area(ring_points)
            for index in base_indices:
                part_area_by_base_vertex[index] = part_area
            cursor += part_vertex_count * 2

    face_groups: list[tuple[str | None, list[list[int]]]]
    if flatten_height is not None and vertices_per_part:
        face_groups = flat_prism_face_groups(vertices, vertices_per_part, material_prefix)
    else:
        face_groups = [(None, face_vertex_indices)]

    rendered_groups: list[tuple[str | None, list[str]]] = []
    face_normals: list[tuple[float, float, float]] = []
    for material, faces in face_groups:
        rendered_faces: list[str] = []
        for face in faces:
            wall_normal = wall_normal_for_face(vertices, face, base_z, part_area_by_base_vertex)
            for triangle in triangulate_face(face):
                oriented = orient_triangle(vertices, triangle, center_xy, base_z, roof_z, wall_normal)
                normal = triangle_normal(vertices, oriented)
                face_normals.append(normal)
                normal_index = len(face_normals)
                rendered_faces.append(
                    "f " + " ".join(f"{vertex_index}//{normal_index}" for vertex_index in oriented)
                )
        rendered_groups.append((material, rendered_faces))

    with output_path.open("w", encoding="utf-8") as target:
        for line_kind, raw_line in lines:
            if line_kind == "raw" and not raw_line.startswith("vn "):
                target.write(raw_line)
            if line_kind != "raw":
                break

        for x, y, z in vertices:
            target.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")

        for nx, ny, nz in face_normals:
            target.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")

        for material, rendered_faces in rendered_groups:
            if material is not None:
                target.write(f"usemtl {material}\n")
            for rendered_face in rendered_faces:
                target.write(f"{rendered_face}\n")


def model_sdf(
    sdf_version: str,
    model_name: str,
    mesh_uri: str,
    include_collision: bool,
) -> str:
    collision = ""
    if include_collision:
        collision = f"""
    <collision name="building_collision">
      <geometry>
        <mesh>
          <uri>{escape(mesh_uri)}</uri>
        </mesh>
      </geometry>
    </collision>"""

    return f"""<?xml version="1.0" ?>
<sdf version="{escape(sdf_version)}">
  <model name="{escape(model_name)}">
    <static>true</static>
    <link name="building_link">
      <visual name="building_visual">
        <geometry>
          <mesh>
            <uri>{escape(mesh_uri)}</uri>
          </mesh>
        </geometry>
      </visual>{collision}
    </link>
  </model>
</sdf>
"""


def model_config(model_name: str, source: str) -> str:
    return f"""<?xml version="1.0" ?>
<model>
  <name>{escape(model_name)}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>cartografia terrain pipeline</name>
  </author>
  <description>Generated construction model from {escape(source)}.</description>
</model>
"""


def world_sdf(sdf_version: str, world_name: str, includes: list[tuple[str, str, tuple[float, float, float]]]) -> str:
    include_lines = []
    for model_name, model_uri, (x, y, z) in includes:
        include_lines.append(
            f"""    <include>
      <name>{escape(model_name)}</name>
      <uri>{escape(model_uri)}</uri>
      <pose>{x:.6f} {y:.6f} {z:.6f} 0 0 0</pose>
    </include>"""
        )

    joined_includes = "\n".join(include_lines)
    return f"""<?xml version="1.0" ?>
<sdf version="{escape(sdf_version)}">
  <world name="{escape(world_name)}">
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <ambient>0.65 0.65 0.65 1</ambient>
      <background>0.7 0.82 1 1</background>
      <shadows>false</shadows>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 1000 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -1</direction>
    </light>
{joined_includes}
  </world>
</sdf>
"""


def footprint_centroid(artifact: dict[str, object]) -> tuple[float, float]:
    min_x, min_y, max_x, max_y = artifact["local_bbox"]
    return (
        (float(min_x) + float(max_x)) * 0.5,
        (float(min_y) + float(max_y)) * 0.5,
    )


def resolve_window_bounds(
    args: argparse.Namespace,
    artifacts: list[dict[str, object]],
) -> tuple[float, float, float, float] | None:
    if args.window_size is None:
        if (
            args.window_min_x is not None
            or args.window_min_y is not None
            or args.window_center_x is not None
            or args.window_center_y is not None
            or args.auto_dense_window
        ):
            raise ValueError("Window coordinates require --window-size.")
        return None

    if args.window_size <= 0:
        raise ValueError("--window-size must be positive.")

    using_min = args.window_min_x is not None or args.window_min_y is not None
    using_center = args.window_center_x is not None or args.window_center_y is not None
    using_auto = args.auto_dense_window

    modes_used = sum(bool(mode) for mode in (using_min, using_center, using_auto))
    if modes_used > 1:
        raise ValueError(
            "Choose only one window selection mode: explicit min corner, explicit center, or --auto-dense-window."
        )

    if using_min:
        if args.window_min_x is None or args.window_min_y is None:
            raise ValueError("Both --window-min-x and --window-min-y are required together.")
        return (
            float(args.window_min_x),
            float(args.window_min_y),
            float(args.window_min_x + args.window_size),
            float(args.window_min_y + args.window_size),
        )

    if using_center:
        if args.window_center_x is None or args.window_center_y is None:
            raise ValueError("Both --window-center-x and --window-center-y are required together.")
        half = args.window_size * 0.5
        return (
            float(args.window_center_x - half),
            float(args.window_center_y - half),
            float(args.window_center_x + half),
            float(args.window_center_y + half),
        )

    if using_auto:
        return find_densest_window(artifacts, args.window_size)

    raise ValueError(
        "When --window-size is provided, choose --auto-dense-window or pass explicit window coordinates."
    )


def find_densest_window(
    artifacts: list[dict[str, object]],
    window_size: float,
) -> tuple[float, float, float, float]:
    centers = []
    for index, artifact in enumerate(artifacts):
        cx, cy = footprint_centroid(artifact)
        centers.append((cx, cy, index))

    if not centers:
        raise RuntimeError("Cannot choose a dense window without any artifacts.")

    centers.sort(key=lambda item: item[0])
    active: list[tuple[float, int]] = []
    right = 0
    best_count = -1
    best_area = -1.0
    best_window = None

    for left in range(len(centers)):
        min_x = centers[left][0]
        while right < len(centers) and centers[right][0] <= min_x + window_size:
            _, cy, artifact_index = centers[right]
            bisect.insort(active, (cy, artifact_index))
            right += 1

        if not active:
            continue

        top = 0
        for bottom in range(len(active)):
            min_y = active[bottom][0]
            while top < len(active) and active[top][0] <= min_y + window_size:
                top += 1

            count = top - bottom
            area = 0.0
            for y, artifact_index in active[bottom:top]:
                del y
                area += float(artifacts[artifact_index]["shape_area_m2"])

            if count > best_count or (count == best_count and area > best_area):
                best_count = count
                best_area = area
                best_window = (min_x, min_y, min_x + window_size, min_y + window_size)

        cy = centers[left][1]
        artifact_index = centers[left][2]
        remove_at = bisect.bisect_left(active, (cy, artifact_index))
        if remove_at < len(active) and active[remove_at] == (cy, artifact_index):
            active.pop(remove_at)

    if best_window is None:
        raise RuntimeError("Failed to choose a dense window.")

    return best_window


def filter_artifacts_to_window(
    artifacts: list[dict[str, object]],
    window: tuple[float, float, float, float] | None,
) -> list[dict[str, object]]:
    if window is None:
        return artifacts

    min_x, min_y, max_x, max_y = window
    selected = []
    for artifact in artifacts:
        cx, cy = footprint_centroid(artifact)
        if min_x <= cx <= max_x and min_y <= cy <= max_y:
            selected.append(artifact)
    return selected


def placement_offset(window: tuple[float, float, float, float] | None) -> tuple[float, float]:
    if window is None:
        return (0.0, 0.0)

    min_x, min_y, max_x, max_y = window
    return ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5)


def selection_manifest(
    manifest: dict[str, object],
    window: tuple[float, float, float, float] | None,
    selected_artifacts: list[dict[str, object]],
    z_offset: float,
) -> dict[str, object]:
    offset_x, offset_y = placement_offset(window)
    output = {
        "source_manifest": str(manifest.get("source", "")),
        "artifact_count": len(selected_artifacts),
        "local_origin": manifest.get("local_origin", {}),
        "window_local_bbox": list(window) if window is not None else None,
        "world_pose_offset": [offset_x, offset_y, z_offset],
        "artifacts": selected_artifacts,
    }

    if selected_artifacts:
        min_world_x = min(float(a["world_bbox"][0]) for a in selected_artifacts)
        min_world_y = min(float(a["world_bbox"][1]) for a in selected_artifacts)
        max_world_x = max(float(a["world_bbox"][2]) for a in selected_artifacts)
        max_world_y = max(float(a["world_bbox"][3]) for a in selected_artifacts)
        output["world_bbox"] = [min_world_x, min_world_y, max_world_x, max_world_y]

    return output


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    artifacts = manifest.get("artifacts", [])
    if not artifacts:
        raise RuntimeError(f"No artifacts found in {args.manifest}")

    window = resolve_window_bounds(args, artifacts)
    artifacts = filter_artifacts_to_window(artifacts, window)
    if not artifacts:
        raise RuntimeError("Window selection produced no construction artifacts.")
    offset_x, offset_y = placement_offset(window)
    z_offset = (
        min(float(artifact["base_z_min_m"]) for artifact in artifacts)
        if args.zero_min_z
        else 0.0
    )

    models_dir = args.models_dir if args.models_dir is not None else args.output_dir / "models"
    if args.models_dir is None:
        if models_dir.exists():
            shutil.rmtree(models_dir)
        models_dir.mkdir(parents=True, exist_ok=True)
    else:
        models_dir.mkdir(parents=True, exist_ok=True)

    material_path = args.construction_dir / "construction_artifacts.mtl"
    if not material_path.exists():
        raise FileNotFoundError(f"Missing material file: {material_path}")

    includes: list[tuple[str, str, tuple[float, float, float]]] = []
    for artifact in artifacts:
        obj_name = artifact["obj"]
        source_obj = args.construction_dir / obj_name
        if not source_obj.exists():
            raise FileNotFoundError(f"Missing construction OBJ: {source_obj}")

        min_x, min_y, max_x, max_y = artifact["local_bbox"]
        origin = (
            (float(min_x) + float(max_x)) * 0.5,
            (float(min_y) + float(max_y)) * 0.5,
            float(artifact["base_z_min_m"]),
        )
        model_name = safe_model_name(
            int(artifact["shape_index"]),
            str(artifact["source"]),
            str(artifact["height_kind"]),
            args.model_name_prefix,
        )
        model_dir = models_dir / model_name
        if model_dir.exists():
            shutil.rmtree(model_dir)
        meshes_dir = model_dir / "meshes"
        meshes_dir.mkdir(parents=True, exist_ok=True)

        output_obj = meshes_dir / obj_name
        rewrite_obj_vertices(
            source_obj,
            output_obj,
            origin,
            flatten_height=float(artifact["height_m"]),
            vertices_per_part=[int(value) for value in artifact.get("vertices_per_part", [])],
        )
        (meshes_dir / material_path.name).write_text(gazebo_material_text(), encoding="utf-8")

        (model_dir / "model.sdf").write_text(
            model_sdf(
                args.sdf_version,
                model_name,
                f"model://{model_name}/meshes/{obj_name}",
                not args.no_collision,
            ),
            encoding="utf-8",
        )
        (model_dir / "model.config").write_text(
            model_config(model_name, str(artifact["source"])),
            encoding="utf-8",
        )
        includes.append(
            (
                model_name,
                f"model://{model_name}",
                (origin[0] - offset_x, origin[1] - offset_y, origin[2] - z_offset),
            )
        )

    (args.output_dir / "world.sdf").write_text(
        world_sdf(args.sdf_version, args.world_name, includes),
        encoding="utf-8",
    )
    (args.output_dir / "selection_manifest.json").write_text(
        json.dumps(selection_manifest(manifest, window, artifacts, z_offset), indent=2),
        encoding="utf-8",
    )

    print(f"Wrote {len(includes)} Gazebo building models to {models_dir}")
    print(f"Wrote Gazebo world to {args.output_dir / 'world.sdf'}")
    print(f"Wrote selection manifest to {args.output_dir / 'selection_manifest.json'}")
    if window is not None:
        min_x, min_y, max_x, max_y = window
        print(
            "Selected local window: "
            f"[{min_x:.3f}, {min_y:.3f}] -> [{max_x:.3f}, {max_y:.3f}]"
        )
    if args.zero_min_z:
        print(f"Shifted world Z so minimum base elevation is 0.000 (offset {z_offset:.3f} m)")
    print(f"Use with: gazebo {args.output_dir / 'world.sdf'}")
    print(f"Or with Gazebo Sim: gz sim {args.output_dir / 'world.sdf'}")


if __name__ == "__main__":
    main()
