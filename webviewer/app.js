const mapLayers = [
  { name: "a_constr", label: "Construction Areas", kind: "area" },
  { name: "a_folha", label: "Sheet Boundary", kind: "area" },
  { name: "a_hidro", label: "Water Areas", kind: "area" },
  { name: "a_terreno", label: "Terrain Areas", kind: "area" },
  { name: "a_vegetacao", label: "Vegetation Areas", kind: "area" },
  { name: "l_aceiro", label: "Firebreak Lines", kind: "line" },
  { name: "l_curva_nivel", label: "Contour Lines", kind: "line" },
  { name: "l_hidro", label: "Hydro Lines", kind: "line" },
  { name: "l_lat", label: "Boundary Lines", kind: "line" },
  { name: "l_muro_ater_desater", label: "Walls and Embankments", kind: "line" },
  { name: "l_pontes", label: "Bridge Lines", kind: "line" },
  { name: "l_verdes_diversos", label: "Green Feature Lines", kind: "line" },
  { name: "l_vias", label: "Road Network", kind: "line" },
  { name: "p_geral", label: "General Points", kind: "point" },
  { name: "p_pcota", label: "Spot Heights", kind: "point" },
  { name: "p_pt", label: "Reference Points", kind: "point" },
  { name: "p_tpn", label: "Topographic Points", kind: "point" },
  { name: "p_vg", label: "Geodetic Vertices", kind: "point" },
  { name: "p_vias", label: "Road Points", kind: "point" },
  { name: "p_workflow", label: "Workflow Points", kind: "point" },
].map((layer) => ({
  ...layer,
  path: `../aligned_maps/${layer.name}.png`,
}));

const coreSelection = [
  "a_folha",
  "a_hidro",
  "a_vegetacao",
  "l_curva_nivel",
  "l_hidro",
  "l_vias",
  "p_pcota",
];

const searchInput = document.querySelector("#layer-search");
const layerList = document.querySelector("#layer-list");
const categoryFilters = document.querySelector("#category-filters");
const showCoreButton = document.querySelector("#show-core-button");
const clearLayersButton = document.querySelector("#clear-layers-button");
const showAllButton = document.querySelector("#show-all-button");
const resetViewButton = document.querySelector("#reset-view-button");
const combinedToggle = document.querySelector("#combined-toggle");
const opacitySlider = document.querySelector("#opacity-slider");
const zoomSlider = document.querySelector("#zoom-slider");
const selectedCount = document.querySelector("#selected-count");
const zoomReadout = document.querySelector("#zoom-readout");
const selectionTitle = document.querySelector("#selection-title");
const mapStatus = document.querySelector("#map-status");
const mapViewport = document.querySelector("#map-viewport");
const mapStage = document.querySelector("#map-stage");

const state = {
  activeCategory: "all",
  searchTerm: "",
  zoom: Number(zoomSlider.value),
  offsetX: 0,
  offsetY: 0,
  dragging: false,
  dragStartX: 0,
  dragStartY: 0,
  originOffsetX: 0,
  originOffsetY: 0,
  selectedLayers: new Set(coreSelection),
  layers: new Map(),
  combined: null,
  referenceBounds: null,
};

function updateStageTransform() {
  mapStage.style.transform = `translate(${state.offsetX}px, ${state.offsetY}px) scale(${state.zoom})`;
  zoomReadout.textContent = `${Math.round(state.zoom * 100)}%`;
}

function ensureLoaded(entry) {
  if (!entry || entry.loaded || entry.loading) {
    return;
  }

  entry.loading = true;
  entry.image.src = entry.path;
}

function recordReferenceBounds(image) {
  const canvas = document.createElement("canvas");
  const context = canvas.getContext("2d", { willReadFrequently: true });
  if (!context) {
    return;
  }

  canvas.width = image.naturalWidth;
  canvas.height = image.naturalHeight;
  context.drawImage(image, 0, 0);

  const { data, width, height } = context.getImageData(0, 0, canvas.width, canvas.height);
  let minX = width;
  let minY = height;
  let maxX = -1;
  let maxY = -1;

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const alpha = data[(y * width + x) * 4 + 3];
      if (!alpha) {
        continue;
      }
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
    }
  }

  if (maxX < minX || maxY < minY) {
    return;
  }

  state.referenceBounds = { minX, minY, maxX, maxY };
}

function fitReferenceBounds() {
  const bounds = state.referenceBounds;
  const viewportWidth = mapViewport.clientWidth;
  const viewportHeight = mapViewport.clientHeight;

  if (!bounds || !viewportWidth || !viewportHeight) {
    state.zoom = 1;
    state.offsetX = 0;
    state.offsetY = 0;
    zoomSlider.value = "1";
    updateStageTransform();
    return;
  }

  const boundsWidth = bounds.maxX - bounds.minX;
  const boundsHeight = bounds.maxY - bounds.minY;
  if (!boundsWidth || !boundsHeight) {
    return;
  }

  const fitZoom = Math.min((viewportWidth * 0.98) / boundsWidth, (viewportHeight * 0.98) / boundsHeight);
  state.zoom = Number(Math.min(3, Math.max(0.35, fitZoom)).toFixed(2));
  zoomSlider.value = String(state.zoom);

  const boundsCenterX = bounds.minX + boundsWidth / 2;
  const boundsCenterY = bounds.minY + boundsHeight / 2;
  state.offsetX = viewportWidth / 2 - boundsCenterX * state.zoom;
  state.offsetY = viewportHeight / 2 - boundsCenterY * state.zoom;
  updateStageTransform();
}

function resetView() {
  fitReferenceBounds();
}

function activeLayers() {
  return mapLayers.filter((layer) => state.selectedLayers.has(layer.name));
}

function updateStatus() {
  const selected = activeLayers();
  const count = selected.length;
  selectedCount.textContent = `${count} layer${count === 1 ? "" : "s"} selected`;

  if (!count) {
    selectionTitle.textContent = "No layers selected";
    mapStatus.textContent = "Select layers from the left to display them on the aligned map sheet.";
    return;
  }

  if (count <= 3) {
    selectionTitle.textContent = selected.map((layer) => layer.label).join(" + ");
  } else {
    selectionTitle.textContent = `${selected[0].label} + ${count - 1} more layers`;
  }

  mapStatus.textContent = combinedToggle.checked
    ? "Selected layers are rendered over the combined reference underlay."
    : "Selected layers are rendered directly from the transparent aligned PNG assets.";
}

function syncLayerVisibility() {
  const opacity = Number(opacitySlider.value);

  if (state.combined) {
    if (combinedToggle.checked) {
      ensureLoaded(state.combined);
    }
    state.combined.image.style.opacity = combinedToggle.checked ? "0.45" : "0";
  }

  state.layers.forEach((entry, name) => {
    const isVisible = state.selectedLayers.has(name);
    if (isVisible) {
      ensureLoaded(entry);
    }
    entry.image.style.opacity = isVisible ? String(opacity) : "0";
    entry.card.classList.toggle("is-selected", isVisible);
    entry.checkbox.checked = isVisible;
  });

  updateStatus();
}

function applyFilter() {
  const term = state.searchTerm.trim().toLowerCase();

  state.layers.forEach((entry) => {
    const matchesCategory = state.activeCategory === "all" || entry.kind === state.activeCategory;
    const matchesSearch =
      !term ||
      entry.label.toLowerCase().includes(term) ||
      entry.name.toLowerCase().includes(term) ||
      entry.kind.toLowerCase().includes(term);

    entry.card.classList.toggle("is-hidden", !(matchesCategory && matchesSearch));
  });
}

function setSelection(layerNames) {
  state.selectedLayers = new Set(layerNames);
  syncLayerVisibility();
}

function buildLayerCard(layer) {
  const card = document.createElement("label");
  card.className = "layer-card";

  const checkbox = document.createElement("input");
  checkbox.type = "checkbox";
  checkbox.checked = state.selectedLayers.has(layer.name);

  const copy = document.createElement("div");
  copy.className = "layer-copy";

  const title = document.createElement("strong");
  title.textContent = layer.label;

  const meta = document.createElement("span");
  meta.textContent = layer.name;

  const swatch = document.createElement("span");
  swatch.className = `swatch ${layer.kind}`;

  const badge = document.createElement("span");
  badge.className = "layer-kind";
  badge.textContent = layer.kind;

  copy.append(title, meta);
  card.append(checkbox, copy, swatch, badge);

  const image = document.createElement("img");
  image.alt = layer.label;
  image.loading = "lazy";
  image.draggable = false;
  mapStage.append(image);

  const entry = {
    ...layer,
    card,
    checkbox,
    image,
    loaded: false,
    loading: false,
  };

  checkbox.addEventListener("change", () => {
    if (checkbox.checked) {
      state.selectedLayers.add(layer.name);
    } else {
      state.selectedLayers.delete(layer.name);
    }
    syncLayerVisibility();
  });

  image.addEventListener("load", () => {
    entry.loaded = true;
    entry.loading = false;
    if (layer.name === "a_folha" && !state.referenceBounds) {
      recordReferenceBounds(image);
      fitReferenceBounds();
    }
  });

  image.addEventListener("error", () => {
    entry.loading = false;
    mapStatus.textContent = `Failed to load ${layer.name}.png from aligned_maps.`;
  });

  layerList.append(card);
  state.layers.set(layer.name, entry);
}

function buildApp() {
  const combinedImage = document.createElement("img");
  combinedImage.alt = "Combined reference map";
  combinedImage.loading = "lazy";
  combinedImage.draggable = false;
  mapStage.append(combinedImage);

  state.combined = {
    image: combinedImage,
    path: "../combined_map.png",
    loaded: false,
    loading: false,
  };

  combinedImage.addEventListener("load", () => {
    state.combined.loaded = true;
    state.combined.loading = false;
  });

  combinedImage.addEventListener("error", () => {
    state.combined.loading = false;
    mapStatus.textContent = "Combined reference image could not be loaded.";
  });

  for (const layer of mapLayers) {
    buildLayerCard(layer);
  }

  applyFilter();
  syncLayerVisibility();
  resetView();
}

searchInput.addEventListener("input", () => {
  state.searchTerm = searchInput.value;
  applyFilter();
});

categoryFilters.addEventListener("click", (event) => {
  const button = event.target.closest("[data-category]");
  if (!button) {
    return;
  }

  state.activeCategory = button.dataset.category;
  for (const chip of categoryFilters.querySelectorAll(".chip")) {
    chip.classList.toggle("is-active", chip === button);
  }
  applyFilter();
});

showCoreButton.addEventListener("click", () => {
  setSelection(coreSelection);
});

clearLayersButton.addEventListener("click", () => {
  setSelection([]);
});

showAllButton.addEventListener("click", () => {
  setSelection(mapLayers.map((layer) => layer.name));
});

combinedToggle.addEventListener("change", syncLayerVisibility);

opacitySlider.addEventListener("input", syncLayerVisibility);

zoomSlider.addEventListener("input", () => {
  state.zoom = Number(zoomSlider.value);
  updateStageTransform();
});

resetViewButton.addEventListener("click", resetView);

mapViewport.addEventListener("pointerdown", (event) => {
  if (!state.selectedLayers.size && !combinedToggle.checked) {
    return;
  }

  state.dragging = true;
  state.dragStartX = event.clientX;
  state.dragStartY = event.clientY;
  state.originOffsetX = state.offsetX;
  state.originOffsetY = state.offsetY;
  mapViewport.classList.add("dragging");
  mapViewport.setPointerCapture(event.pointerId);
});

mapViewport.addEventListener("pointermove", (event) => {
  if (!state.dragging) {
    return;
  }

  state.offsetX = state.originOffsetX + (event.clientX - state.dragStartX);
  state.offsetY = state.originOffsetY + (event.clientY - state.dragStartY);
  updateStageTransform();
});

function stopDragging(event) {
  if (event?.pointerId !== undefined) {
    mapViewport.releasePointerCapture(event.pointerId);
  }
  state.dragging = false;
  mapViewport.classList.remove("dragging");
}

mapViewport.addEventListener("pointerup", stopDragging);
mapViewport.addEventListener("pointercancel", stopDragging);

mapViewport.addEventListener(
  "wheel",
  (event) => {
    event.preventDefault();
    const delta = event.deltaY > 0 ? -0.1 : 0.1;
    state.zoom = Number(Math.min(3, Math.max(0.35, state.zoom + delta)).toFixed(2));
    zoomSlider.value = String(state.zoom);
    updateStageTransform();
  },
  { passive: false }
);

window.addEventListener("resize", () => {
  fitReferenceBounds();
});

buildApp();
