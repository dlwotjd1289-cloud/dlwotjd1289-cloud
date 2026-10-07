(() => {
  const viewport = document.getElementById("viewport");
  const metricsEl = document.getElementById("metrics");
  const boxListEl = document.getElementById("boxList");
  const boxCountEl = document.getElementById("boxCount");
  const liveBadge = document.getElementById("liveBadge");
  const forceSummary = document.getElementById("forceSummary");
  const selectedInfo = document.getElementById("selectedInfo");
  const heatCanvas = document.getElementById("heatmap");
  const heatCtx = heatCanvas.getContext("2d");
  const comTraceCanvas = document.getElementById("comTrace");
  const comTraceCtx = comTraceCanvas.getContext("2d");

  const analyticsHistory = {
    lastSampleT: -Infinity,
    maxSamples: 300,
    t: [],
    heightM: [],
    utilPct: [],
    comOffsetMm: [],
    massKg: [],
    comXY: [],
  };

  let quadrantChart = null;
  let boxLoadChart = null;
  let historyChart = null;

  if (!window.THREE) {
    viewport.innerHTML = "<div style='padding:30px'>Three.js failed to load.</div>";
    return;
  }

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1117);
  scene.fog = new THREE.Fog(0x0b1117, 5.0, 11.0);

  const camera = new THREE.PerspectiveCamera(
    42, innerWidth / innerHeight, 0.01, 100
  );
  // Simulation/world convention: Z-up
  camera.up.set(0, 0, 1);
  camera.position.set(2.25, -2.25, 1.75);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(innerWidth, innerHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputEncoding = THREE.sRGBEncoding;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  viewport.appendChild(renderer.domElement);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.28);
  controls.enableDamping = true;
  controls.dampingFactor = 0.07;
  controls.screenSpacePanning = true;
  controls.maxPolarAngle = Math.PI * 0.49;
  controls.minDistance = 0.5;
  controls.maxDistance = 7.0;

  scene.add(new THREE.HemisphereLight(0xd9eeff, 0x26313b, 1.25));

  const key = new THREE.DirectionalLight(0xffffff, 1.55);
  key.position.set(2.3, -1.6, 3.4);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.left = -3;
  key.shadow.camera.right = 3;
  key.shadow.camera.top = 3;
  key.shadow.camera.bottom = -3;
  scene.add(key);

  const fill = new THREE.DirectionalLight(0x9fc8ff, 0.46);
  fill.position.set(-2.0, 1.4, 1.8);
  scene.add(fill);

  const root = new THREE.Group();
  scene.add(root);

  const boxMeshes = new Map();
  const clickable = [];
  let currentPalletKey = "";
  let palletGroup = null;
  let comMesh = null;
  let copMesh = null;
  let heightCage = null;
  let state = null;
  let selectedBoxId = null;

  function makeWoodPallet(p) {
    if (palletGroup) root.remove(palletGroup);
    palletGroup = new THREE.Group();

    const wood = new THREE.MeshStandardMaterial({
      color: 0x906039, roughness: 0.87, metalness: 0.0
    });
    const woodDark = new THREE.MeshStandardMaterial({
      color: 0x6f472a, roughness: 0.92
    });

    const L = p.length_m, W = p.width_m, H = p.deck_height_m;
    const topH = H * 0.22;
    const blockH = H * 0.50;
    const bottomH = H * 0.16;

    [-0.38, -0.19, 0, 0.19, 0.38].forEach(frac => {
      const board = new THREE.Mesh(
        new THREE.BoxGeometry(L, W * 0.13, topH), wood
      );
      board.position.set(0, frac * W, -topH / 2);
      board.castShadow = board.receiveShadow = true;
      palletGroup.add(board);
    });

    [-0.39, 0, 0.39].forEach(xf => {
      [-0.34, 0, 0.34].forEach(yf => {
        const block = new THREE.Mesh(
          new THREE.BoxGeometry(L * 0.14, W * 0.15, blockH), woodDark
        );
        block.position.set(
          xf * L, yf * W, -topH - blockH / 2
        );
        block.castShadow = block.receiveShadow = true;
        palletGroup.add(block);
      });
    });

    [-0.34, 0, 0.34].forEach(yf => {
      const board = new THREE.Mesh(
        new THREE.BoxGeometry(L, W * 0.14, bottomH), wood
      );
      board.position.set(
        0, yf * W, -topH - blockH - bottomH / 2
      );
      board.castShadow = board.receiveShadow = true;
      palletGroup.add(board);
    });

    root.add(palletGroup);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(5.5, 5.5),
      new THREE.MeshStandardMaterial({
        color: 0x202b34, roughness: 0.96
      })
    );
    floor.position.z = -H - 0.02;
    floor.receiveShadow = true;
    palletGroup.add(floor);

    const grid = new THREE.GridHelper(5.0, 25, 0x40515f, 0x283640);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -H - 0.018;
    grid.material.transparent = true;
    grid.material.opacity = 0.34;
    palletGroup.add(grid);

    const cageGeo = new THREE.BoxGeometry(L, W, p.max_height_m);
    const cageEdges = new THREE.EdgesGeometry(cageGeo);
    const cageMat = new THREE.LineBasicMaterial({
      color: 0x56d9ff, transparent: true, opacity: 0.25
    });
    heightCage = new THREE.LineSegments(cageEdges, cageMat);
    heightCage.position.z = p.max_height_m / 2;
    palletGroup.add(heightCage);
  }

  function ensureBoxMesh(b) {
    let mesh = boxMeshes.get(b.id);
    const key = b.size_m.join(",");
    if (mesh && mesh.userData.sizeKey !== key) {
      root.remove(mesh);
      const ix = clickable.indexOf(mesh);
      if (ix >= 0) clickable.splice(ix, 1);
      boxMeshes.delete(b.id);
      mesh = null;
    }

    if (!mesh) {
      const geometry = new THREE.BoxGeometry(
        b.size_m[0], b.size_m[1], b.size_m[2]
      );
      const material = new THREE.MeshStandardMaterial({
        color: new THREE.Color(b.color || "#c98b52"),
        roughness: 0.78,
        metalness: 0.0,
      });
      mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.userData.boxId = b.id;
      mesh.userData.sizeKey = key;

      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(geometry),
        new THREE.LineBasicMaterial({
          color: 0x4d321f, transparent: true, opacity: 0.66
        })
      );
      mesh.add(edges);
      root.add(mesh);
      boxMeshes.set(b.id, mesh);
      clickable.push(mesh);
    }
    return mesh;
  }


  function initCharts() {
    if (!window.Chart) return;

    Chart.defaults.color = "#9fb1c1";
    Chart.defaults.borderColor = "rgba(170,195,215,.10)";
    Chart.defaults.font.size = 9;

    quadrantChart = new Chart(
      document.getElementById("quadrantChart"),
      {
        type: "bar",
        data: {
          labels: ["-X +Y", "+X +Y", "-X -Y", "+X -Y"],
          datasets: [{
            label: "Pallet normal force (N)",
            data: [0, 0, 0, 0],
            borderWidth: 1,
            backgroundColor: [
              "rgba(86,217,255,.55)",
              "rgba(98,223,151,.55)",
              "rgba(255,209,102,.55)",
              "rgba(255,107,107,.55)",
            ],
          }],
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: false,
          plugins: { legend: { display: false } },
          scales: {
            x: { grid: { display: false } },
            y: { beginAtZero: true, title: { display: true, text: "N" } },
          },
        },
      }
    );

    boxLoadChart = new Chart(
      document.getElementById("boxLoadChart"),
      {
        type: "bar",
        data: {
          labels: [],
          datasets: [{
            label: "Force transmitted downward (N)",
            data: [],
            borderWidth: 1,
            backgroundColor: "rgba(255,209,102,.58)",
          }],
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: false,
          indexAxis: "y",
          plugins: { legend: { display: false } },
          scales: {
            x: { beginAtZero: true, title: { display: true, text: "N" } },
            y: { grid: { display: false } },
          },
        },
      }
    );

    historyChart = new Chart(
      document.getElementById("historyChart"),
      {
        type: "line",
        data: {
          labels: [],
          datasets: [
            {
              label: "Height (m)",
              data: [],
              yAxisID: "yHeight",
              borderWidth: 1.6,
              pointRadius: 0,
              borderColor: "#56d9ff",
            },
            {
              label: "Utilization (%)",
              data: [],
              yAxisID: "yPct",
              borderWidth: 1.4,
              pointRadius: 0,
              borderColor: "#62df97",
            },
            {
              label: "CoM offset (mm)",
              data: [],
              yAxisID: "yMm",
              borderWidth: 1.2,
              pointRadius: 0,
              borderColor: "#ffd166",
            },
          ],
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: false,
          interaction: { mode: "index", intersect: false },
          plugins: {
            legend: {
              display: true,
              labels: { boxWidth: 8, boxHeight: 8, font: { size: 8 } },
            },
          },
          scales: {
            x: {
              title: { display: true, text: "simulation time (s)" },
              ticks: { maxTicksLimit: 5 },
            },
            yHeight: {
              type: "linear",
              position: "left",
              beginAtZero: true,
              title: { display: true, text: "m" },
            },
            yPct: {
              type: "linear",
              position: "right",
              beginAtZero: true,
              suggestedMax: 100,
              grid: { drawOnChartArea: false },
              title: { display: true, text: "%" },
            },
            yMm: {
              type: "linear",
              position: "right",
              beginAtZero: true,
              grid: { drawOnChartArea: false },
              display: false,
            },
          },
        },
      }
    );
  }

  function loadQuadrants(loadMap) {
    const grid = loadMap && loadMap.force_n;
    if (!grid || !grid.length) return [0, 0, 0, 0];

    const n = grid.length;
    const half = n / 2;
    // Order: -X +Y, +X +Y, -X -Y, +X -Y
    const q = [0, 0, 0, 0];

    for (let iy = 0; iy < n; iy++) {
      for (let ix = 0; ix < grid[iy].length; ix++) {
        const xPositive = ix >= half;
        const yPositive = iy >= half;
        let qi = 0;
        if (xPositive && yPositive) qi = 1;
        else if (!xPositive && !yPositive) qi = 2;
        else if (xPositive && !yPositive) qi = 3;
        q[qi] += Number(grid[iy][ix] || 0);
      }
    }
    return q;
  }

  function sampleAnalytics(s) {
    const t = Number(s.simulation_time_s || 0);
    if (t - analyticsHistory.lastSampleT < 0.20) return;
    analyticsHistory.lastSampleT = t;

    const m = s.metrics;
    analyticsHistory.t.push(t);
    analyticsHistory.heightM.push(Number(m.current_height_m || 0));
    analyticsHistory.utilPct.push(
      100 * Number(m.allowed_volume_utilization || 0)
    );
    analyticsHistory.comOffsetMm.push(
      1000 * Number(m.com_xy_offset_m || 0)
    );
    analyticsHistory.massKg.push(Number(m.on_pallet_mass_kg || 0));
    analyticsHistory.comXY.push([
      Number(m.on_pallet_com_m?.[0] || 0),
      Number(m.on_pallet_com_m?.[1] || 0),
    ]);

    for (const key of ["t","heightM","utilPct","comOffsetMm","massKg","comXY"]) {
      if (analyticsHistory[key].length > analyticsHistory.maxSamples) {
        analyticsHistory[key].shift();
      }
    }
  }

  function updateAnalytics(s) {
    sampleAnalytics(s);

    if (quadrantChart) {
      quadrantChart.data.datasets[0].data = loadQuadrants(
        s.metrics.load_map
      );
      quadrantChart.update("none");
    }

    if (boxLoadChart) {
      const perBox = s.contact_graph?.per_box || {};
      const entries = s.boxes.map(b => ({
        id: b.id,
        force: Number(perBox[b.id]?.support_force_below_n || 0),
      }))
      .sort((a, b) => b.force - a.force)
      .slice(0, 12);

      boxLoadChart.data.labels = entries.map(x => x.id);
      boxLoadChart.data.datasets[0].data = entries.map(x => x.force);
      boxLoadChart.update("none");
    }

    if (historyChart) {
      historyChart.data.labels = analyticsHistory.t.map(x => x.toFixed(1));
      historyChart.data.datasets[0].data = analyticsHistory.heightM;
      historyChart.data.datasets[1].data = analyticsHistory.utilPct;
      historyChart.data.datasets[2].data = analyticsHistory.comOffsetMm;
      historyChart.update("none");
    }

    drawComTrace(s);
  }

  function drawComTrace(s) {
    const ctx = comTraceCtx;
    const canvas = comTraceCanvas;
    const W = canvas.width;
    const H = canvas.height;
    const pad = 22;
    const p = s.pallet;

    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#0f171e";
    ctx.fillRect(0, 0, W, H);

    const mapX = x =>
      pad + (x / p.length_m + 0.5) * (W - 2 * pad);
    const mapY = y =>
      H - pad - (y / p.width_m + 0.5) * (H - 2 * pad);

    ctx.strokeStyle = "rgba(210,230,245,.42)";
    ctx.lineWidth = 1.2;
    ctx.strokeRect(pad, pad, W - 2 * pad, H - 2 * pad);

    ctx.strokeStyle = "rgba(210,230,245,.12)";
    ctx.beginPath();
    ctx.moveTo(W / 2, pad);
    ctx.lineTo(W / 2, H - pad);
    ctx.moveTo(pad, H / 2);
    ctx.lineTo(W - pad, H / 2);
    ctx.stroke();

    const pts = analyticsHistory.comXY;
    if (pts.length > 1) {
      ctx.strokeStyle = "#56d9ff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      pts.forEach((pt, i) => {
        const x = mapX(pt[0]), y = mapY(pt[1]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    }

    const cur = s.metrics.on_pallet_com_m;
    ctx.fillStyle = "#56d9ff";
    ctx.beginPath();
    ctx.arc(mapX(cur[0]), mapY(cur[1]), 5, 0, Math.PI * 2);
    ctx.fill();

    const cop = s.metrics.load_map?.center_of_pressure_m;
    if (cop) {
      ctx.fillStyle = "#ffd166";
      ctx.beginPath();
      ctx.arc(mapX(cop[0]), mapY(cop[1]), 4, 0, Math.PI * 2);
      ctx.fill();
    }

    ctx.fillStyle = "#93a6b6";
    ctx.font = "10px sans-serif";
    ctx.fillText("+Y", W / 2 + 5, 14);
    ctx.fillText("+X", W - 20, H / 2 - 5);
    ctx.fillStyle = "#56d9ff";
    ctx.fillText("● CoM", 8, H - 8);
    ctx.fillStyle = "#ffd166";
    ctx.fillText("● CoP", 60, H - 8);
  }

  initCharts();

  function updateScene(s) {
    state = s;
    const p = s.pallet;
    const pKey = [
      p.length_m, p.width_m, p.deck_height_m, p.max_height_m
    ].join(",");
    if (pKey !== currentPalletKey) {
      makeWoodPallet(p);
      currentPalletKey = pKey;
    }

    const present = new Set();
    s.boxes.forEach(b => {
      present.add(b.id);
      const mesh = ensureBoxMesh(b);
      mesh.position.fromArray(b.position_m);
      mesh.quaternion.set(
        b.quaternion_xyzw[0],
        b.quaternion_xyzw[1],
        b.quaternion_xyzw[2],
        b.quaternion_xyzw[3]
      );
    });

    for (const [id, mesh] of [...boxMeshes.entries()]) {
      if (!present.has(id)) {
        root.remove(mesh);
        boxMeshes.delete(id);
        const ix = clickable.indexOf(mesh);
        if (ix >= 0) clickable.splice(ix, 1);
      }
    }

    updateMarkers(s);
    updatePanels(s);
    drawHeatmap(s.metrics.load_map);
    updateAnalytics(s);
  }

  function marker(color, radius) {
    return new THREE.Mesh(
      new THREE.SphereGeometry(radius, 24, 18),
      new THREE.MeshStandardMaterial({
        color, emissive: color, emissiveIntensity: .32, roughness: .25
      })
    );
  }

  function updateMarkers(s) {
    const com = s.metrics.on_pallet_com_m;
    if (!comMesh) {
      comMesh = marker(0x56d9ff, 0.025);
      root.add(comMesh);
    }
    comMesh.position.set(com[0], com[1], com[2]);

    const cop = s.metrics.load_map.center_of_pressure_m;
    if (cop) {
      if (!copMesh) {
        copMesh = marker(0xffd166, 0.018);
        root.add(copMesh);
      }
      copMesh.visible = true;
      copMesh.position.set(cop[0], cop[1], 0.018);
    } else if (copMesh) {
      copMesh.visible = false;
    }
  }

  function pct(x) {
    return `${(100 * Number(x || 0)).toFixed(1)} %`;
  }

  function metricRows(rows) {
    return rows.map(([k, v]) =>
      `<div class="metric-k">${k}</div><div class="metric-v">${v}</div>`
    ).join("");
  }

  function updatePanels(s) {
    const m = s.metrics;
    liveBadge.textContent = "LIVE";
    liveBadge.className = "live-badge ok";

    metricsEl.innerHTML = metricRows([
      ["Pallet", `${(s.pallet.length_m*1000).toFixed(0)} × ${(s.pallet.width_m*1000).toFixed(0)} mm`],
      ["Boxes on pallet", `${m.on_pallet_box_count} / ${m.box_count}`],
      ["Mass on pallet", `${m.on_pallet_mass_kg.toFixed(2)} kg`],
      ["Height", `${m.current_height_m.toFixed(3)} / ${m.height_limit_m.toFixed(3)} m`],
      ["Remaining height", `${m.remaining_height_m.toFixed(3)} m`],
      ["Allowed-volume util.", pct(m.allowed_volume_utilization)],
      ["Current-stack util.", pct(m.current_stack_utilization)],
      ["CoM XY offset", `${(m.com_xy_offset_m*1000).toFixed(1)} mm`],
      ["Moving boxes", `${m.moving_box_count}`],
      ["Outside pallet", `${m.outside_pallet_box_count}`],
      ["Max tilt", `${m.max_tilt_deg.toFixed(1)}°`],
    ]);

    const lm = m.load_map;
    forceSummary.textContent =
      `${lm.total_normal_force_n.toFixed(1)} N · ${lm.equivalent_supported_mass_kg.toFixed(1)} kgf`;

    boxCountEl.textContent = String(m.box_count);
    const loadByBox = (s.contact_graph || {}).per_box || {};

    boxListEl.innerHTML = s.boxes.map(b => {
      const d = loadByBox[b.id] || {};
      return `
        <div class="box-row" data-box-id="${b.id}">
          <div class="swatch" style="background:${b.color || "#c98b52"}"></div>
          <div>
            <div class="box-title">${b.id}</div>
            <div class="box-sub">${b.mass_kg.toFixed(1)} kg · z ${b.position_m[2].toFixed(3)} m</div>
          </div>
          <div class="box-force">${Number(d.load_from_above_n || 0).toFixed(0)} N</div>
        </div>
      `;
    }).join("");

    boxListEl.querySelectorAll(".box-row").forEach(el => {
      el.addEventListener("click", () => selectBox(el.dataset.boxId));
    });

    if (selectedBoxId) renderSelected();
  }

  function drawHeatmap(loadMap) {
    const grid = loadMap.force_n;
    if (!grid || !grid.length) return;
    const n = grid.length;
    const maxV = Math.max(1e-9, loadMap.max_cell_force_n || 0);
    const w = heatCanvas.width, h = heatCanvas.height;
    heatCtx.clearRect(0, 0, w, h);

    for (let iy = 0; iy < n; iy++) {
      for (let ix = 0; ix < n; ix++) {
        const t = Math.min(1, grid[iy][ix] / maxV);
        const hue = 220 - 220 * t;
        const light = 18 + 36 * t;
        heatCtx.fillStyle = `hsl(${hue}, 82%, ${light}%)`;
        const x0 = ix * w / n;
        const y0 = (n - 1 - iy) * h / n;
        heatCtx.fillRect(x0, y0, w / n + .5, h / n + .5);
      }
    }

    heatCtx.strokeStyle = "rgba(255,255,255,.08)";
    heatCtx.lineWidth = 1;
    for (let i = 1; i < n; i++) {
      const x = i * w / n;
      heatCtx.beginPath(); heatCtx.moveTo(x,0); heatCtx.lineTo(x,h); heatCtx.stroke();
      const y = i * h / n;
      heatCtx.beginPath(); heatCtx.moveTo(0,y); heatCtx.lineTo(w,y); heatCtx.stroke();
    }
  }

  function selectBox(id) {
    selectedBoxId = id;
    renderSelected();
    const mesh = boxMeshes.get(id);
    if (mesh) {
      controls.target.copy(mesh.position);
    }
  }

  function renderSelected() {
    if (!state || !selectedBoxId) return;
    const b = state.boxes.find(x => x.id === selectedBoxId);
    if (!b) return;
    const d = (state.contact_graph.per_box || {})[b.id] || {};
    const tilt = Math.max(
      Math.abs(b.euler_rad[0]), Math.abs(b.euler_rad[1])
    ) * 180 / Math.PI;
    selectedInfo.textContent =
      `${b.id} · ${b.mass_kg.toFixed(2)} kg · ` +
      `xyz (${b.position_m.map(x=>x.toFixed(3)).join(", ")}) m · ` +
      `tilt ${tilt.toFixed(1)}° · load from above ${Number(d.load_from_above_n || 0).toFixed(1)} N`;
  }

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  renderer.domElement.addEventListener("click", e => {
    const rect = renderer.domElement.getBoundingClientRect();
    pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const hit = raycaster.intersectObjects(clickable, false)[0];
    if (hit) selectBox(hit.object.userData.boxId);
  });

  async function post(url, body) {
    const res = await fetch(url, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: body ? JSON.stringify(body) : "{}"
    });
    return await res.json();
  }

  document.getElementById("nextBtn").onclick = () => post("/api/demo/next");
  document.getElementById("autoBtn").onclick = () => post("/api/demo/auto");
  document.getElementById("resetBtn").onclick = () => {
    selectedBoxId = null;
    selectedInfo.textContent = "Click a box for live details";
    analyticsHistory.lastSampleT = -Infinity;
    for (const key of ["t","heightM","utilPct","comOffsetMm","massKg","comXY"]) {
      analyticsHistory[key].length = 0;
    }
    post("/api/reset");
  };

  function connectWs() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.onopen = () => {
      liveBadge.textContent = "LIVE";
      liveBadge.className = "live-badge ok";
    };
    ws.onmessage = e => {
      const msg = JSON.parse(e.data);
      if (msg.type === "state") updateScene(msg);
    };
    ws.onclose = () => {
      liveBadge.textContent = "RECONNECTING";
      liveBadge.className = "live-badge bad";
      setTimeout(connectWs, 1000);
    };
    ws.onerror = () => ws.close();
  }
  connectWs();

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();

  addEventListener("resize", () => {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  });
})();
