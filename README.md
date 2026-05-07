# CIGEOE Map Layers and 3D Terrain

This repository contains a clipped CIGEOE vector dataset in `dataset/`, rendered layer previews in `aligned_maps/`, and a small 3D terrain pipeline in `terrain3d/`.

The 3D workflow currently uses:

- `dataset/l_curva_nivel.shp` for ground heights from contour-line Z values.
- `dataset/a_folha.shp` for the map-sheet extent and local coordinate origin.
- `dataset/a_constr.shp` for construction/building footprint polygons.

## Input Dataset

Each shapefile name uses a geometry prefix plus a theme name:

- `a_` = area or polygon layers
- `l_` = line layers
- `p_` = point layers

Examples:

- `a_constr` = construction/building polygons
- `l_vias` = road/path linework
- `p_vg` = geodetic or survey control points

### Polygon Layers

- `a_constr`: constructed areas and building footprints. This is the main built-feature polygon layer. Sample feature types include `A_Casa`, `A_Ruinas`, `A_Grande_construcao`, `A_Campo_de_jogos`, and `A_Estufa`. Important fields: `source`, `Shape_Area`, and `h_campo`. Most `h_campo` values are zero, so this pipeline assumes `6 m` height for those buildings.
- `a_folha`: map sheet extent. Used to derive the full-sheet target area and local coordinate origin.
- `a_hidro`: hydrographic water-area polygons. Sample features include `A_Rio_de_duas_margens`, `A_Limite_de_pantano`, and `A_Lagoa`.
- `a_terreno`: terrain-related polygon area. In this clip it contains `A_Limite_de_areeiro`.
- `a_vegetacao`: vegetation and cultivated-cover polygons. Sample features include `A_Arvoredo_denso`, `A_Vinha`, `A_Pomar_vinha`, `A_Arvoredo_esparso`, and `A_Pomar`.

### Line Layers

- `l_curva_nivel`: contour lines. This is the terrain-height source used by the 3D pipeline. Sample features include regular and master contours such as `L_Curva_de_nivel_Par`, `L_Curva_de_nivel_Impar`, and `L_Curva_de_nivel_Mestra`.
- `l_hidro`: hydrographic linework. This is the main water-line network and includes `L_Linha_de_agua`, `L_Linha_agua_auxiliar`, and `L_Represa`.
- `l_vias`: roads, streets, tracks, and access routes. Sample features include `L_Arruamento`, `L_Caminho_carreteiro`, `L_Acesso_auto`, `L_Estrada_estreita`, and `L_Estrada_larga`.
- `l_aceiro`: firebreak or cleared-strip lines.
- `l_lat`: lattice/grid or auxiliary cartographic linework.
- `l_muro_ater_desater`: walls, embankments, and cuttings. Field `h_campo` may store feature height in some cases.
- `l_pontes`: bridges, overpasses, and tunnels.
- `l_verdes_diversos`: miscellaneous green or boundary features.
- `l_workflow`: internal workflow/editing line layer.

### Point Layers

- `p_geral`: general topographic points such as wells, tanks, ruins, aqueducts, and fountains.
- `p_pcota`: elevation spot points or point-cota markers. Not currently used by the 3D terrain pipeline.
- `p_vg`: geodetic/survey landmark points with elevation-related fields such as `Cota_Verti` and `Cota_Terre`. Not currently used by the 3D terrain pipeline.
- `p_pt`: auxiliary point layer with sparse metadata.
- `p_tpn`: named point features with fields `nome` and `tipo`.
- `p_vias`: road-related point markers.
- `p_workflow`: internal workflow/editing point layer.

Useful field notes:

- `source` is usually the most useful field for identifying feature class.
- `Shape_Area` and `Shape_Leng` are geometry-derived area/length fields.
- `h_campo` is used as explicit building height where populated.
- Workflow layers are production-tracking artifacts rather than map content.

## Produce Terrain Surface and Point Cloud

First extract contour support samples from `l_curva_nivel`.

For the center `1000 m x 1000 m` patch:

```bash
python3 terrain3d/scripts/extract_center_contours.py
```

This writes:

- `terrain3d/data/center_contours.xyz`

For the whole map sheet:

```bash
python3 terrain3d/scripts/extract_center_contours.py \
  --whole-sheet \
  --output terrain3d/data/full_sheet_contours.xyz
```

This writes:

- `terrain3d/data/full_sheet_contours.xyz`

Then generate the terrain blanket mesh and point cloud.

Center patch at `1 m` spacing:

```bash
./terrain3d/build/terrain_blanket
```

Outputs:

- `terrain3d/output/center_blanket_cloud.pcd`
- `terrain3d/output/center_blanket_mesh.ply`

Full sheet preview at `10 m` spacing:

```bash
./terrain3d/build/terrain_blanket \
  --input terrain3d/data/full_sheet_contours.xyz \
  --grid-step 10 \
  --cloud terrain3d/output/full_sheet_blanket_cloud_10m.pcd \
  --mesh terrain3d/output/full_sheet_blanket_mesh_10m.ply
```

Outputs:

- `terrain3d/output/full_sheet_blanket_cloud_10m.pcd`
- `terrain3d/output/full_sheet_blanket_mesh_10m.ply`

Building-level full sheet at `10 m` spacing:

```bash
python3 terrain3d/scripts/extract_construction_levels.py

./terrain3d/build/terrain_blanket \
  --input terrain3d/data/full_sheet_construction_levels.xyz \
  --grid-step 10 \
  --cloud terrain3d/output/full_sheet_building_level_blanket_cloud_10m.pcd \
  --mesh terrain3d/output/full_sheet_building_level_blanket_mesh_10m.ply
```

Outputs:

- `terrain3d/data/full_sheet_construction_levels.xyz`
- `terrain3d/output/full_sheet_building_level_blanket_cloud_10m.pcd`
- `terrain3d/output/full_sheet_building_level_blanket_mesh_10m.ply`

The blanket generator uses inverse-distance weighting over nearest contour samples and exports both:

- PCD point cloud
- PLY triangle mesh

## Produce Construction Objects

Build construction prisms from `dataset/a_constr.shp`:

```bash
python3 terrain3d/scripts/build_construction_artifacts.py \
  --terrain-support terrain3d/data/full_sheet_construction_levels.xyz \
  --terrain-mesh terrain3d/output/full_sheet_building_level_blanket_mesh_10m.ply
```

This script:

- Uses `a_constr` polygon rings as building footprints.
- Uses `h_campo` as real height where populated.
- Assumes `6 m` height where `h_campo` is zero or missing.
- Clamps every building base to sit at least `5 cm` above the displayed building-level blanket.
- Triangulates roof/base faces and keeps side walls as quads, so buildings are simple prismatic polyhedra.

Outputs are written to:

```text
terrain3d/output/construction_artifacts/
```

Important combined outputs:

- `constructions_all.obj`: all construction prisms
- `constructions_actual_height.obj`: buildings with real `h_campo`
- `constructions_assumed_6m.obj`: buildings with assumed `6 m` height
- `manifest.json`: per-building metadata, height source, base/roof elevations, area, bbox, and OBJ filename

Current counts:

- `7,283` total construction objects
- `49` objects with actual `h_campo`
- `7,234` objects using assumed `6 m` height

## Open the Viewer

Open the full-sheet blanket with all construction objects:

```bash
./terrain3d/build/terrain_viewer \
  --mesh terrain3d/output/full_sheet_building_level_blanket_mesh_10m.ply
```

By default the viewer loads:

- The terrain blanket mesh.
- Assumed-height building prisms from `constructions_assumed_6m.obj`.
- Actual-height building prisms from `constructions_actual_height.obj`.
- Stable random colors per individual building.

Viewer controls:

- Drag to orbit.
- Mouse wheel to zoom.
- Right drag to pan.
- Press `s` to hide/show the blanket surface while keeping buildings visible.

Optional: show the point cloud too:

```bash
./terrain3d/build/terrain_viewer \
  --mesh terrain3d/output/full_sheet_building_level_blanket_mesh_10m.ply \
  --cloud terrain3d/output/full_sheet_building_level_blanket_cloud_10m.pcd \
  --show-cloud
```

Optional: open one construction OBJ layer manually:

```bash
./terrain3d/build/terrain_viewer \
  --mesh terrain3d/output/full_sheet_building_level_blanket_mesh_10m.ply \
  --constructions terrain3d/output/construction_artifacts/constructions_all.obj
```

## Compilation

The C++/PCL code that needs compilation is:

- `terrain3d/src/terrain_blanket.cpp`
- `terrain3d/src/terrain_viewer.cpp`

Build configuration:

- `terrain3d/CMakeLists.txt`

Ubuntu dependencies:

```bash
sudo apt-get update
sudo apt-get install -y libpcl-dev python3-pyshp cmake build-essential
```

Configure and compile from the repository root:

```bash
cmake -S terrain3d -B terrain3d/build
cmake --build terrain3d/build -j4
```

Compiled binaries:

- `terrain3d/build/terrain_blanket`
- `terrain3d/build/terrain_viewer`

Python scripts do not need compilation:

- `terrain3d/scripts/extract_center_contours.py`
- `terrain3d/scripts/build_construction_artifacts.py`
