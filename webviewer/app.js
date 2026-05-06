import * as THREE from "https://unpkg.com/three@0.165.0/build/three.module.js";
import { OrbitControls } from "https://unpkg.com/three@0.165.0/examples/jsm/controls/OrbitControls.js";
import { PLYLoader } from "https://unpkg.com/three@0.165.0/examples/jsm/loaders/PLYLoader.js";
import { PCDLoader } from "https://unpkg.com/three@0.165.0/examples/jsm/loaders/PCDLoader.js";

const datasets = {
  center: {
    label: "Center 1 km Patch",
    mesh: "../terrain3d/output/center_blanket_mesh.ply",
    cloud: "../terrain3d/output/center_blanket_cloud.pcd",
  },
  full: {
    label: "Full Sheet",
    mesh: "../terrain3d/output/full_sheet_blanket_mesh.ply",
    cloud: "../terrain3d/output/full_sheet_blanket_cloud.pcd",
  },
};

const canvas = document.querySelector("#scene");
const datasetSelect = document.querySelector("#dataset-select");
const meshToggle = document.querySelector("#mesh-toggle");
const cloudToggle = document.querySelector("#cloud-toggle");
const wireframeToggle = document.querySelector("#wireframe-toggle");
const reloadButton = document.querySelector("#reload-button");
const zScaleInput = document.querySelector("#z-scale");
const zScaleValue = document.querySelector("#z-scale-value");
const statusEl = document.querySelector("#status");

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x091217);
scene.fog = new THREE.Fog(0x091217, 1200, 18000);

const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 100000);
camera.position.set(900, -1200, 850);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.target.set(0, 0, 0);

const hemi = new THREE.HemisphereLight(0xdbe8ef, 0x23353b, 1.2);
scene.add(hemi);

const sun = new THREE.DirectionalLight(0xfff3cf, 1.3);
sun.position.set(1200, -900, 1800);
scene.add(sun);

const grid = new THREE.GridHelper(12000, 48, 0x5d7969, 0x294038);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

const axes = new THREE.AxesHelper(300);
scene.add(axes);

const loaders = {
  mesh: new PLYLoader(),
  cloud: new PCDLoader(),
};

let currentGroup = null;
let currentDatasetKey = datasetSelect.value;

function setStatus(message, tone = "info") {
  statusEl.textContent = message;
  statusEl.style.color = tone === "error" ? "#ef9a8d" : "#9fb2a8";
}

function setRecommendedLayerVisibility(datasetKey) {
  if (datasetKey === "full" && !cloudToggle.dataset.userTouched) {
    cloudToggle.checked = false;
  }
}

function resize() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (!width || !height) {
    return;
  }
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function clearCurrentGroup() {
  if (!currentGroup) {
    return;
  }
  scene.remove(currentGroup);
  currentGroup.traverse((child) => {
    if (child.geometry) {
      child.geometry.dispose();
    }
    if (child.material) {
      if (Array.isArray(child.material)) {
        child.material.forEach((material) => material.dispose());
      } else {
        child.material.dispose();
      }
    }
  });
  currentGroup = null;
}

function frameObject(object) {
  const box = new THREE.Box3().setFromObject(object);
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z);

  controls.target.copy(center);
  camera.near = Math.max(0.1, maxDim / 5000);
  camera.far = Math.max(10000, maxDim * 40);
  camera.position.set(
    center.x + maxDim * 0.9,
    center.y - maxDim * 1.2,
    center.z + maxDim * 0.7
  );
  camera.updateProjectionMatrix();
  controls.update();
}

function applyVerticalScale() {
  const scale = Number(zScaleInput.value);
  zScaleValue.textContent = `${scale.toFixed(2)}x`;
  if (currentGroup) {
    currentGroup.scale.set(1, 1, scale);
    frameObject(currentGroup);
  }
}

async function loadDataset(datasetKey) {
  currentDatasetKey = datasetKey;
  setRecommendedLayerVisibility(datasetKey);
  const dataset = datasets[datasetKey];
  clearCurrentGroup();
  setStatus(`Loading ${dataset.label.toLowerCase()} terrain…`);

  const group = new THREE.Group();
  group.name = `${datasetKey}-terrain`;

  try {
    const jobs = [];
    const loadedKinds = [];
    const failedKinds = [];

    if (meshToggle.checked) {
      jobs.push(
        loaders.mesh
          .loadAsync(dataset.mesh)
          .then((geometry) => {
            geometry.computeVertexNormals();
            const material = new THREE.MeshStandardMaterial({
              color: 0xc6ab6c,
              roughness: 0.95,
              metalness: 0.02,
              wireframe: wireframeToggle.checked,
            });
            const mesh = new THREE.Mesh(geometry, material);
            group.add(mesh);
            loadedKinds.push("mesh");
          })
          .catch((error) => {
            console.error("Mesh load failed:", error);
            failedKinds.push("mesh");
          })
      );
    }

    if (cloudToggle.checked) {
      jobs.push(
        loaders.cloud
          .loadAsync(dataset.cloud)
          .then((points) => {
            points.material = new THREE.PointsMaterial({
              color: 0xf5efcf,
              size: datasetKey === "full" ? 2.0 : 5.0,
              sizeAttenuation: true,
            });
            group.add(points);
            loadedKinds.push("point cloud");
          })
          .catch((error) => {
            console.error("Point cloud load failed:", error);
            failedKinds.push("point cloud");
          })
      );
    }

    if (!jobs.length) {
      setStatus("Enable at least one layer type to load the terrain.", "error");
      return;
    }

    await Promise.all(jobs);

    if (!group.children.length) {
      setStatus(
        "Nothing loaded. If this keeps happening, keep only the mesh enabled or verify that the page has internet access for the Three.js modules.",
        "error"
      );
      return;
    }

    currentGroup = group;
    scene.add(group);
    applyVerticalScale();

    if (failedKinds.length) {
      setStatus(
        `${dataset.label} loaded with ${loadedKinds.join(" + ")}. Failed: ${failedKinds.join(", ")}.`,
        "info"
      );
    } else {
      setStatus(`${dataset.label} loaded with ${loadedKinds.join(" + ")}.`);
    }
  } catch (error) {
    console.error(error);
    setStatus(
      "Failed to load the terrain files. Serve the repo over HTTP and check that the output files exist.",
      "error"
    );
  }
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

datasetSelect.addEventListener("change", () => {
  loadDataset(datasetSelect.value);
});

meshToggle.addEventListener("change", () => {
  loadDataset(currentDatasetKey);
});

cloudToggle.addEventListener("change", () => {
  cloudToggle.dataset.userTouched = "true";
  loadDataset(currentDatasetKey);
});

wireframeToggle.addEventListener("change", () => {
  loadDataset(currentDatasetKey);
});

reloadButton.addEventListener("click", () => {
  loadDataset(currentDatasetKey);
});

zScaleInput.addEventListener("input", () => {
  applyVerticalScale();
});

window.addEventListener("resize", resize);

resize();
applyVerticalScale();
loadDataset(currentDatasetKey);
animate();
