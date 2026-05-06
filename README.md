# CIGEOE Map Layers

This repository contains a small clipped vector dataset in `dataset/` plus rendered previews in `maps/` and `aligned_maps/`.

## Naming Scheme

Each shapefile name uses a geometry prefix plus a theme name:

- `a_` = area or polygon layers
- `l_` = line layers
- `p_` = point layers

Examples:

- `a_constr` = construction/building polygons
- `l_vias` = road/path linework
- `p_vg` = geodetic or survey control points

## Themes

### Polygon Layers

- `a_constr`: constructed areas and building footprints. This is the main built-feature polygon layer. Sample feature types include `A_Casa`, `A_Ruinas`, `A_Grande_construcao`, `A_Campo_de_jogos`, and `A_Estufa`. Important fields: `source`, `Shape_Area`, and `h_campo` (likely height, but mostly zero).
- `a_folha`: map sheet extent. This looks like the polygon for the clipped map tile or sheet boundary. Important fields: `num_folha`, `nom_folha`, `Shape_Area`.
- `a_hidro`: hydrographic water-area polygons. Sample features include `A_Rio_de_duas_margens`, `A_Limite_de_pantano`, and `A_Lagoa`.
- `a_terreno`: terrain-related polygon area. In this clip it contains `A_Limite_de_areeiro`, so it appears to capture a terrain or land-surface class boundary rather than general elevation zones.
- `a_vegetacao`: vegetation and cultivated-cover polygons. Sample features include `A_Arvoredo_denso`, `A_Vinha`, `A_Pomar_vinha`, `A_Arvoredo_esparso`, and `A_Pomar`.

### Line Layers

- `l_aceiro`: firebreak or cleared-strip lines. The name `aceiro` usually refers to a firebreak or vegetation-cleared lane.
- `l_curva_nivel`: contour lines. Sample features include regular and master contours such as `L_Curva_de_nivel_Par`, `L_Curva_de_nivel_Impar`, and `L_Curva_de_nivel_Mestra`.
- `l_hidro`: hydrographic linework. This is the main water-line network and includes `L_Linha_de_agua`, `L_Linha_agua_auxiliar`, and `L_Represa`.
- `l_lat`: lattice or grid linework. The metadata is sparse, so this likely represents auxiliary cartographic linework rather than a physical theme.
- `l_muro_ater_desater`: walls, embankments, and cuttings. Sample features include `L_Aterro`, `L_Desaterro`, `L_Muro_de_alvenaria_em_via`, and retaining-wall variants. Field `h_campo` may store feature height in some cases.
- `l_pontes`: bridges, overpasses, and tunnels. Sample features include concrete bridges, timber bridges, `L_Passagem_superior`, and `L_Tunel_eixo`.
- `l_verdes_diversos`: miscellaneous green or boundary features. In this clip the features are `L_Sebe_ou_valado`, meaning hedge or fenced boundary lines.
- `l_vias`: roads, streets, tracks, and access routes. Sample features include `L_Arruamento`, `L_Caminho_carreteiro`, `L_Acesso_auto`, `L_Estrada_estreita`, and `L_Estrada_larga`. This is the main transport line layer.
- `l_workflow`: internal workflow/editing line layer. This clip has zero records, and its fields look like production status columns rather than map content.

### Point Layers

- `p_geral`: general topographic points. Sample features include `P_Poco`, `P_Tanque`, `P_Ruinas`, `P_Aqueduto_em_via`, and `P_Chafariz_ou_fonte`.
- `p_pcota`: elevation spot points or point-cota markers. The name strongly suggests spot heights, but the clip metadata only exposes a placeholder attribute.
- `p_pt`: auxiliary point layer with sparse metadata. The name is not self-explanatory from the schema alone.
- `p_tpn`: named point features with fields `nome` and `tipo`. This likely stores named toponyms or place-name points.
- `p_vg`: geodetic/survey landmark points. Sample features include `P_VG_deposito_agua` and `P_VG_igreja`. This layer has many coordinate and elevation-related fields such as `Cota_Verti` and `Cota_Terre`.
- `p_vias`: road-related point markers. Sample features are `P_Quilometro_em_caminho_de_ferro` and `P_Quilometro_em_estrada`, so this looks like route or kilometer markers.
- `p_workflow`: internal workflow/editing point layer. The fields describe production steps rather than mapped terrain objects.

## Notes

- Layer names and most attributes are in Portuguese.
- `source` is usually the most useful field for identifying the mapped feature class.
- `Shape_Area` and `Shape_Leng` are geometry-derived area/length fields.
- `h_campo` appears in some layers and likely means feature height, but it is sparsely populated in this clip.
- `workflow` layers appear to be production-tracking artifacts rather than map themes intended for analysis.

## Quick Reference

- Built environment: `a_constr`, `l_muro_ater_desater`, `l_pontes`, `p_geral`
- Hydrography: `a_hidro`, `l_hidro`
- Vegetation and land cover: `a_vegetacao`, `a_terreno`, `l_verdes_diversos`
- Transport: `l_vias`, `p_vias`
- Relief and surveying: `l_curva_nivel`, `p_pcota`, `p_vg`
- Map framing and metadata: `a_folha`, `l_workflow`, `p_workflow`

## 3D Terrain Prototype

There is now a small C++/PCL prototype in `terrain3d/` that builds a simple 3D terrain blanket from the contour layer.

What it does:

- Uses `dataset/a_folha.shp` to find the map-sheet center.
- Extracts contour support samples from `dataset/l_curva_nivel.shp`.
- Keeps the final terrain patch at `1000 m x 1000 m`, centered on the sheet center.
- Uses a larger `1600 m x 1600 m` support window around that center so interpolation remains well supported near the patch edges.
- Interpolates a regular terrain grid with inverse-distance weighting and exports both a point cloud and a mesh.

Files:

- Extractor: `terrain3d/scripts/extract_center_contours.py`
- C++ source: `terrain3d/src/terrain_blanket.cpp`
- Viewer source: `terrain3d/src/terrain_viewer.cpp`
- Build config: `terrain3d/CMakeLists.txt`
- Extracted support points: `terrain3d/data/center_contours.xyz`
- Generated outputs: `terrain3d/output/center_blanket_cloud.pcd`, `terrain3d/output/center_blanket_mesh.ply`
- Viewer binary: `terrain3d/build/terrain_viewer`

Build and run from the repo root:

```bash
./myenv/bin/python terrain3d/scripts/extract_center_contours.py
cmake -S terrain3d -B terrain3d/build
cmake --build terrain3d/build -j4
./terrain3d/build/terrain_blanket
./terrain3d/build/terrain_viewer
```

Current prototype settings:

- Output patch: `1000 m x 1000 m`
- Support window: `1600 m x 1600 m`
- Grid spacing: `1 m`
- Interpolation: inverse-distance weighting over the nearest contour samples
- Current local relief in this test patch: approximately `50 m` to `80 m`

## Web Viewer

There is also a static browser viewer in `webviewer/` that can load the generated terrain mesh and point cloud.

Files:

- Page: `webviewer/index.html`
- App logic: `webviewer/app.js`
- Styles: `webviewer/styles.css`

It can load:

- Center patch terrain
- Full-sheet terrain
- Mesh and point cloud together or separately
- Adjustable vertical exaggeration

Run it from the repo root with a simple local server:

```bash
python3 -m http.server 8000
```

Then open:

```text
http://127.0.0.1:8000/webviewer/
```

Notes:

- The page fetches files from `terrain3d/output/`, so it should be served over HTTP rather than opened directly as a `file://` page.
- The current implementation uses Three.js browser modules.
# cartografia
