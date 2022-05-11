import * as wasm from "geodesic-path";
import * as THREE from "three";
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader.js';
import { Line2 } from "three/examples/jsm/lines/Line2.js"
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial.js"
import { LineGeometry } from "three/examples/jsm/lines/LineGeometry.js"


await wasm.default();

const dracoLoader = new DRACOLoader();
dracoLoader.setDecoderPath('draco/');
dracoLoader.setDecoderConfig({ type: 'js' });

let bunny: THREE.BufferGeometry;

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.outputEncoding = THREE.sRGBEncoding;
renderer.shadowMap.enabled = true;
document.body.appendChild(renderer.domElement);

window.addEventListener('resize', onWindowResize);

const camera = new THREE.PerspectiveCamera(35, window.innerWidth / window.innerHeight, 0.0001, 1);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enablePan = true;
controls.enableZoom = true;
controls.enableDamping = false;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x443333);

const pathMaterial = new LineMaterial({
  color: 0xff0000,
  linewidth: 2,
  worldUnits: false,
});
pathMaterial.resolution.set(window.innerWidth, window.innerHeight);

const makeModel = (geometry: THREE.BufferGeometry) => {
  const coordinates = geometry.getAttribute("position").array as Float32Array

  // let pathPoints = wasm.get_path(3000, 2000, coordinates,
  //   geometry.index!.array as Uint32Array);
  let pathPoints = wasm.get_path(4000, 25215, coordinates,
    geometry.index!.array as Uint32Array);
  console.log(pathPoints, geometry.index!.array)
  geometry.computeVertexNormals();

  let maxZ = 0;
  let index = 0
  for (let i = 1; i < coordinates.length; i += 3) {
    if (coordinates[i] > maxZ) {
      maxZ = coordinates[i];
      index = (i - 1) / 3
    }
  }

  console.error(index);


  const lineGeometry = new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(pathPoints, 3));
  // const path = new THREE.Line(lineGeometry, new THREE.LineBasicMaterial({ color: "red" }));
  // path.renderOrder = 2
  const pathGeometry = new LineGeometry().fromLine(new THREE.Line(lineGeometry));

  const path = new Line2(pathGeometry, pathMaterial);
  path.renderOrder = 2

  const material = new THREE.MeshStandardMaterial({ color: 0x606060, vertexColors: false, wireframe: false });
  const model = new THREE.Mesh(geometry, material);
  return [path, model];
}

const showBunny = () => {
  const add = () => {
    scene.clear();
    camera.position.set(0.1, 0.2, 0.5);
    camera.lookAt(0, 0.1, 0);
    controls.target.set(0, 0.1, 0);
    scene.add(...makeModel(bunny));
    const spot = new THREE.DirectionalLight();
    spot.position.set(1, 1, 1);

    const spot2 = new THREE.DirectionalLight();
    spot.position.set(-1, 1, -1);

    scene.add(new THREE.AmbientLight(), spot, spot2);
  };

  if (bunny) {
    add()
  } else {
    dracoLoader.load('public/bunny.drc', (geometry) => {
      bunny = geometry;
      add();
      dracoLoader.dispose();
    });
  }
}


showBunny();

animate();

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
  pathMaterial.resolution.set(window.innerWidth, window.innerHeight);

}

function animate() {
  render();
  requestAnimationFrame(animate);
}

function render() {
  renderer.render(scene, camera);
}


