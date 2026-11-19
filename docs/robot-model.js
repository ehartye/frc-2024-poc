import "./vendor/model-viewer-4.1.0.min.js";

const viewer = document.getElementById("robot-model");
const input = document.getElementById("local-model");
const status = document.getElementById("model-status");
const restore = document.getElementById("show-default");
const revision = document.getElementById("model-revision");
const caption = document.getElementById("model-caption");
const description = document.getElementById("replay-description");
const models = {
  swerve: { file: "swerve-r7", title: caption.textContent, label: status.textContent, description: description.textContent },
  "r7-budget": { file: "r7-nominal-d1", poster: "swerve-r7", title: "1780 · R7 nominal mass",
    label: "R7 Fusion assembly · Nominal floor-pose mass budget",
    description: "The middle-wing NOTE seats at 4.363 s and reaches both Speaker sensor pairs at 13.639 s on the straight wall approach. The displayed robot and simulated plant use the same R7 Fusion archive. The view follows the chassis until release, then holds the release frame through the detector event." },
  baseline: { file: "robot-native", title: "1780 · Six-wheel baseline", label: "Historical Fusion assembly · Team-authored geometry",
              description: "First wing NOTE from the six-wheel baseline. The orange NOTE follows its measured path through the Speaker opening." }
};
const homeOrbit = viewer.cameraOrbit;
const wheelsButton = document.getElementById("view-wheels");
function inspectModules() {
  viewer.cameraTarget = "0m .10m 0m";
  viewer.cameraOrbit = "8deg 162deg 1.95m";
  viewer.jumpCameraToGoal();
}
wheelsButton.addEventListener("click", inspectModules);
let localUrl;
let selection = 0;

function resetCamera() {
  viewer.cameraTarget = "auto auto auto";
  viewer.cameraOrbit = homeOrbit;
  viewer.fieldOfView = "auto";
  viewer.jumpCameraToGoal();
}

function restoreModel() {
  selection++;
  viewer.pause();
  replay.hidden = true;
  const model = models[revision.value];
  viewer.src = "media/" + model.file + ".glb";
  viewer.poster = "media/" + (model.poster ?? model.file) + ".png";
  viewer.querySelector('[slot="poster"]').src = viewer.poster;
  caption.textContent = model.title;
  description.textContent = model.description;
  wheelsButton.hidden = revision.value === "baseline";
  if (localUrl) URL.revokeObjectURL(localUrl);
  localUrl = undefined;
  input.value = "";
  restore.hidden = true;
  status.textContent = model.label;
  resetCamera();
}

async function checkEmbeddedGlb(file) {
  const header = new DataView(await file.slice(0, 20).arrayBuffer());
  if (header.byteLength !== 20 || header.getUint32(0, true) !== 0x46546c67 ||
      header.getUint32(4, true) !== 2 || header.getUint32(8, true) !== file.size ||
      header.getUint32(16, true) !== 0x4e4f534a) throw new Error("Invalid GLB");
  const end = 20 + header.getUint32(12, true);
  if (end > file.size) throw new Error("Incomplete GLB");
  const model = JSON.parse(await file.slice(20, end).text());
  const usesUri = object => object && typeof object === "object" &&
    Object.entries(object).some(([key, value]) => key === "uri" || usesUri(value));
  if (!model.meshes?.length || usesUri(model)) throw new Error("Expected embedded meshes");
}

input.addEventListener("change", async () => {
  const file = input.files[0];
  if (!file) return;
  const token = ++selection;
  try {
    await checkEmbeddedGlb(file);
    if (token !== selection) return;
    const previous = localUrl;
    localUrl = URL.createObjectURL(file);
    viewer.pause();
    replay.hidden = true;
    caption.textContent = "Local assembly";
    wheelsButton.hidden = true;
    viewer.src = localUrl;
    if (previous) URL.revokeObjectURL(previous);
    restore.hidden = false;
    status.textContent = `Local assembly · ${file.name}`;
  } catch {
    if (token !== selection) return;
    input.value = "";
    status.textContent = "Choose a self-contained GLB with embedded geometry.";
  }
});
viewer.addEventListener("load", () => {
  if (selection === 0 && new URLSearchParams(location.search).get("view") === "swerve") inspectModules();
  else resetCamera();
});
viewer.addEventListener("error", () => {
  status.textContent = "The interactive model could not be displayed in this browser.";
});
restore.addEventListener("click", restoreModel);
revision.addEventListener("change", restoreModel);
if (new URLSearchParams(location.search).get("view") === "r7-budget") {
  revision.value = "r7-budget";
  // Replay controls are initialized further down this module.
  queueMicrotask(restoreModel);
}

const markers = document.getElementById("r7-events");
const r7Index = fetch("media/r7-first-index.json").then(response => {
  if (!response.ok) throw new Error("R7 markers unavailable");
  return response.json();
}).catch(() => null);

async function showR7Markers(clip) {
  markers.replaceChildren();
  markers.hidden = true;
  if (localUrl || revision.value !== "r7-budget" || !clip) return;
  const token = selection;
  const index = await r7Index;
  const chapter = index?.chapters.find(c => c.animation === clip.name && c.revision === clip.extras.native);
  if (!chapter || token !== selection) return;
  for (const event of chapter.events) {
    const button = document.createElement("button");
    button.textContent = event.target + " · " + event.timeS.toFixed(3) + " s";
    button.addEventListener("click", () => {
      viewer.pause();
      viewer.currentTime = Math.min(viewer.duration - .000001, event.clipS);
      if (event.id === "both-pairs") counterButton.click();
      else if (event.id === "opening") {
        viewer.cameraTarget = clip.extras.openingCenter.map(x => x + "m").join(" ");
        viewer.cameraOrbit = "12deg 77deg 1.6m";
        viewer.jumpCameraToGoal();
      } else resetCamera();
      showPlayhead();
    });
    markers.append(button);
  }
  markers.hidden = false;
}

const film = document.getElementById("first-note-film");
fetch("media/first-film-index.json").then(response => response.json()).then(index => {
  const controls = document.getElementById("film-events");
  for (const event of index.chapters[0].events) {
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = event.target + " · " + event.timeS.toFixed(3) + " s";
    button.addEventListener("click", () => {
      film.pause();
      film.currentTime = Math.ceil(event.clipS * 20) / 20;
    });
    controls.append(button);
  }
}).catch(() => { /* The film's native controls remain available. */ });
document.getElementById("reset-camera").addEventListener("click", resetCamera);

const replay = document.getElementById("replay");
const play = document.getElementById("play-replay");
const slowShot = document.getElementById("slow-shot");
const speed = document.getElementById("replay-speed");
const timeline = document.getElementById("replay-time");
const clock = document.getElementById("replay-clock");
const counterButton = document.getElementById("view-counter");
let events;
let releaseTime = 0;
let frameRequest;

function showPlayhead() {
  timeline.value = viewer.currentTime;
  const matchTime = viewer.currentTime + (events?.sourceStartS ?? 0);
  const atCounter = events?.sensorS != null && matchTime >= events.sensorS - .0002;
  clock.textContent = matchTime.toFixed(3) + " s" + (atCounter ? " · Both sensor pairs blocked" : "");
  play.textContent = viewer.paused ? (events?.pickup ? "Play pickup and shot" : "Play lift and shot") : "Pause replay";
}

function tick() {
  showPlayhead();
  if (!viewer.paused && !replay.hidden) frameRequest = requestAnimationFrame(tick);
}

function playClip() {
  if (viewer.currentTime >= viewer.duration - .01) viewer.currentTime = 0;
  viewer.timeScale = Number(speed.value);
  viewer.play({ repetitions: 1 });
  cancelAnimationFrame(frameRequest);
  tick();
}

viewer.addEventListener("load", () => {
  cancelAnimationFrame(frameRequest);
  const clip = viewer.originalGltfJson?.animations?.find(item => item.name === "Lift and shoot");
  replay.hidden = !clip || localUrl !== undefined;
  events = clip?.extras;
  showR7Markers(clip);
  counterButton.hidden = replay.hidden || !events?.counterCenter;
  if (replay.hidden) return;
  viewer.animationName = clip.name;
  releaseTime = clip.extras.releaseS - clip.extras.sourceStartS;
  timeline.max = viewer.originalGltfJson.accessors[clip.samplers[0].input].max[0];
  viewer.currentTime = 0;
  showPlayhead();
});
viewer.addEventListener("finished", showPlayhead);
play.addEventListener("click", () => {
  if (viewer.paused) playClip();
  else { viewer.pause(); showPlayhead(); }
});
slowShot.addEventListener("click", () => {
  viewer.pause();
  viewer.currentTime = Math.max(0, releaseTime - .6);
  speed.value = ".25";
  viewer.cameraTarget = ".42m .92m 0m";
  viewer.cameraOrbit = "28deg 77deg 5.4m";
  viewer.jumpCameraToGoal();
  playClip();
});
counterButton.addEventListener("click", () => {
  viewer.pause();
  viewer.currentTime = Math.min(events.sensorS - events.sourceStartS, viewer.duration - .00001);
  viewer.cameraTarget = events.counterCenter.map(value => value + "m").join(" ");
  viewer.cameraOrbit = "12deg 77deg 1.4m";
  viewer.jumpCameraToGoal();
  showPlayhead();
});
timeline.addEventListener("input", () => {
  viewer.pause();
  viewer.currentTime = Math.min(Number(timeline.value), viewer.duration - .0001);
  showPlayhead();
});
speed.addEventListener("change", () => { viewer.timeScale = Number(speed.value); });
