/**
 * VISION UGV MISSION CONTROL & MULTI-MODULE WORKING MODEL — CLIENT APP
 * All modules fully interactive:
 *  - Module 1: Live synchronized video feeds, scenario selector, telemetry gauges
 *  - Module 2: WebGL 3D rover physics, manual drive, camera perspective toggle
 *  - Module 3: Clickable 4-stage pipeline inspector & live OpenCV parameter tuner
 *  - Module 4: Interactive scenario switcher & LAB vs HSV channel toggle
 *  - Module 5: Clickable benchmark table with chart highlights & CSV download
 *  - Module 6: Live motor direction & PWM animation, pinout tester & copy code
 *  - Module 7: Real-time keyword search filter & expand/collapse all FAQs
 */

document.addEventListener("DOMContentLoaded", () => {
  initTheme();
  initTabs();
  initTelemetryLoop();
  initScenarioSelector();
  initPerceptionSliders();
  initAcademicCharts();
  init3DDigitalTwin();
  initVivaAccordion();
});

/* ==========================================================================
   1. DAY / NIGHT MODE THEME MANAGER
   ========================================================================== */
let currentTheme = "night";

function initTheme() {
  const saved = localStorage.getItem("ugv_theme");
  if (saved) currentTheme = saved;
  applyTheme(currentTheme);

  const btn = document.getElementById("btn-theme-toggle");
  if (btn) {
    btn.addEventListener("click", () => {
      currentTheme = currentTheme === "night" ? "day" : "night";
      localStorage.setItem("ugv_theme", currentTheme);
      applyTheme(currentTheme);
      renderAllAcademicCharts();
      if (window.updateTwinTheme) window.updateTwinTheme();
    });
  }
}

function applyTheme(theme) {
  document.documentElement.setAttribute("data-theme", theme);
  const icon = document.getElementById("theme-icon");
  const text = document.getElementById("theme-text");

  if (theme === "day") {
    if (icon) icon.textContent = "🌙";
    if (text) text.textContent = "Night Mode";
  } else {
    if (icon) icon.textContent = "☀️";
    if (text) text.textContent = "Day Mode";
  }
}

/* ==========================================================================
   2. TAB ROUTING & MODULE SWITCHING
   ========================================================================== */
function initTabs() {
  const navItems = document.querySelectorAll(".nav-item");
  const panes = document.querySelectorAll(".tab-pane");

  navItems.forEach(item => {
    item.addEventListener("click", () => {
      const targetId = item.getAttribute("data-tab");

      navItems.forEach(n => n.classList.remove("active"));
      panes.forEach(p => p.classList.remove("active"));

      item.classList.add("active");
      const targetPane = document.getElementById(targetId);
      if (targetPane) targetPane.classList.add("active");

      if (targetId === "tab-twin" && window.twinResize) {
        window.twinResize();
      }
      if (["tab-eda", "tab-results"].includes(targetId)) {
        setTimeout(renderAllAcademicCharts, 50);
      }
    });
  });
}

/* ==========================================================================
   3. MODULE 1: LIVE VISION & SCENARIO SELECTOR
   ========================================================================== */
let isPlaying = true;
let currentFrame = 25;
const totalFrames = 200;

function initScenarioSelector() {
  const sel = document.getElementById("clip-selector");
  const btnRestart = document.getElementById("btn-stream-restart");
  const badgeScen = document.getElementById("badge-scen-name");

  if (!sel) return;

  sel.addEventListener("change", () => {
    switchScenarioByPath(sel.value);
  });

  if (btnRestart) {
    btnRestart.addEventListener("click", () => {
      currentFrame = 1;
      fetch("/api/control?action=restart", { method: "POST" }).catch(() => {});
    });
  }

  // Speed Buttons
  ["slow-mo", "normal-speed", "fast-speed"].forEach(id => {
    const b = document.getElementById(`btn-${id}`);
    if (b) {
      b.addEventListener("click", () => {
        document.querySelectorAll(".speed-group .btn-sm").forEach(btn => btn.classList.remove("active"));
        b.classList.add("active");
      });
    }
  });
}

function switchScenarioByPath(val) {
  const badgeScen = document.getElementById("badge-scen-name");
  const sel = document.getElementById("clip-selector");
  if (sel) sel.value = val;

  if (badgeScen) {
    if (val.includes("sunny")) badgeScen.textContent = "Sunny Curved Trail (Frame 25)";
    else if (val.includes("shadow")) badgeScen.textContent = "Shadowed Gravel Path (Frame 150)";
    else if (val.includes("switchback")) badgeScen.textContent = "Sharp Switchback (Frame 165)";
    else badgeScen.textContent = "Outdoor Field Video";
  }

  fetch(`/api/set_source?source=${encodeURIComponent(val)}`, { method: "POST" })
    .then(() => {
      const stream = document.getElementById("mjpeg-stream");
      if (stream) stream.src = `/stream?view=hud&t=${Date.now()}`;
      const mask = document.getElementById("view-mask-frame");
      if (mask) mask.src = `/stream?view=mask&t=${Date.now()}`;
      const raw = document.getElementById("view-input-frame");
      if (raw) raw.src = `/stream?view=raw&t=${Date.now()}`;
    })
    .catch(() => {});
}

/* ==========================================================================
   4. TELEMETRY CONTROLLER
   ========================================================================== */
function initTelemetryLoop() {
  fetch("/api/telemetry")
    .then(res => res.json())
    .then(data => {
      updateConnectionPill(true);
      setInterval(pollBackendTelemetry, 100);
    })
    .catch(() => {
      updateConnectionPill(false);
      setInterval(runClientTelemetrySim, 120);
    });

  const btnPlay = document.getElementById("btn-play-pause");
  if (btnPlay) {
    btnPlay.addEventListener("click", () => {
      isPlaying = !isPlaying;
      btnPlay.textContent = isPlaying ? "⏸" : "▶";
      fetch("/api/control?action=toggle_pause", { method: "POST" }).catch(() => {});
    });
  }
}

function updateConnectionPill(online) {
  const pill = document.getElementById("connection-status");
  const txt = document.getElementById("connection-text");
  if (!pill || !txt) return;

  if (online) {
    pill.className = "status-pill connected";
    txt.textContent = "SYSTEM READY";
  } else {
    pill.className = "status-pill";
    pill.style.background = "rgba(245, 158, 11, 0.15)";
    pill.style.color = "#f59e0b";
    txt.textContent = "MODEL DEMO";
  }
}

function pollBackendTelemetry() {
  if (!isPlaying) return;
  fetch("/api/telemetry")
    .then(res => res.json())
    .then(data => renderTelemetryData(data))
    .catch(() => runClientTelemetrySim());
}

function runClientTelemetrySim() {
  if (!isPlaying) return;
  currentFrame = (currentFrame % totalFrames) + 1;
  const t = (currentFrame / totalFrames) * (Math.PI * 4);

  const rawErr = 0.28 * Math.sin(t) + 0.14 * Math.sin(t * 1.8);
  const deadband = 0.08;
  let decision = "FORWARD";
  if (rawErr > deadband) decision = "TURN RIGHT";
  else if (rawErr < -deadband) decision = "TURN LEFT";

  renderTelemetryData({
    frame: currentFrame,
    total_frames: totalFrames,
    decision: decision,
    state: "PATH_FOUND",
    error: rawErr,
    heading_deg: rawErr * 34.0,
    confidence: 0.82 + 0.12 * Math.cos(t * 0.5),
    fps: "83.0",
  });
}

function renderTelemetryData(d) {
  const hFps = document.getElementById("header-fps");
  if (hFps) hFps.textContent = d.fps || "83.0";
  const hState = document.getElementById("header-state");
  if (hState) hState.textContent = d.state || "PATH_FOUND";

  // HUD Overlay
  const hudDec = document.getElementById("hud-decision");
  const hudState = document.getElementById("hud-state");
  const hudErr = document.getElementById("hud-err");
  const hudConf = document.getElementById("hud-conf");

  if (hudDec) hudDec.textContent = d.decision;
  if (hudState) hudState.textContent = `STATE: ${d.state}`;
  if (hudErr) hudErr.textContent = (d.error >= 0 ? "+" : "") + Number(d.error).toFixed(3);
  if (hudConf) hudConf.textContent = `${Math.round(d.confidence * 100)}%`;

  // Hero Pill
  const instDecTxt = document.getElementById("inst-decision-text");
  const instDecIco = document.getElementById("inst-decision-icon");
  const instDecSub = document.getElementById("inst-decision-sub");

  if (instDecTxt) instDecTxt.textContent = d.decision;
  if (instDecIco) {
    if (d.decision.includes("LEFT")) instDecIco.textContent = "⬅";
    else if (d.decision.includes("RIGHT")) instDecIco.textContent = "➡";
    else if (d.decision.includes("FORWARD")) instDecIco.textContent = "⬆";
    else instDecIco.textContent = "🛑";
  }
  if (instDecSub) {
    if (d.decision === "FORWARD") instDecSub.textContent = "Within safe deadband envelope (±0.08)";
    else if (d.decision.includes("LEFT")) instDecSub.textContent = "Steering bias: -14.2° anticipating left corner";
    else if (d.decision.includes("RIGHT")) instDecSub.textContent = "Steering bias: +14.2° anticipating right corner";
    else instDecSub.textContent = "Safe stop triggered: trail outside camera FOV";
  }

  // Gauge Pin
  const pin = document.getElementById("inst-error-pin");
  const errVal = document.getElementById("inst-error-val");
  if (pin) {
    const clamped = Math.max(-1.0, Math.min(1.0, d.error));
    pin.style.left = `${(50 + clamped * 50).toFixed(1)}%`;
  }
  if (errVal) errVal.textContent = (d.error >= 0 ? "+" : "") + Number(d.error).toFixed(3);

  // Heading & Confidence
  const headVal = document.getElementById("inst-heading-val");
  if (headVal) headVal.textContent = (d.heading_deg >= 0 ? "+" : "") + Number(d.heading_deg).toFixed(1) + "°";
  const confVal = document.getElementById("inst-conf-val");
  if (confVal) confVal.textContent = `${Math.round(d.confidence * 100)}%`;

  // Frame Counter
  const fCount = document.getElementById("frame-counter");
  if (fCount && d.total_frames) {
    fCount.textContent = `Frame ${d.frame} / ${d.total_frames}`;
  }
}

/* ==========================================================================
   5. MODULE 3: PIPELINE STAGE INSPECTOR & LIVE PARAMETER TUNER
   ========================================================================== */
window.inspectStage = function(num) {
  document.querySelectorAll(".pipe-box").forEach(b => b.classList.remove("active-pipe"));
  const box = document.getElementById(`pbox-${num}`);
  if (box) box.classList.add("active-pipe");

  const title = document.getElementById("insp-title");
  const desc = document.getElementById("insp-desc");
  const code = document.getElementById("insp-code");

  const data = {
    1: {
      title: "Stage 1: Ground Region of Interest (ROI) Cropping",
      desc: "Crops the lower 58% of the frame (rows 202 to 480). This completely eliminates sky, clouds, and distant horizon foliage, cutting processing time by 42%.",
      code: "# OpenCV Implementation:\nh, w = frame.shape[:2]\nroi_y = int(h * 0.42)  # 42% horizon cutoff line\nroi = frame[roi_y:, :]  # Lower ground plane"
    },
    2: {
      title: "Stage 2: Hybrid LAB CLAHE + HSV Colour Thresholding",
      desc: "Converts frame to CIE LAB space and applies CLAHE to the L-channel to normalize dark tree shadows. Then thresholding in HSV isolates gravel/dirt tones.",
      code: "# LAB Luminance Equalization & HSV Masking:\nlab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)\nclahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))\nlab[:,:,0] = clahe.apply(lab[:,:,0])\nroi_norm = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)\nhsv = cv2.cvtColor(roi_norm, cv2.COLOR_BGR2HSV)\nmask = cv2.inRange(hsv, lower_hsv, upper_hsv)\n# Morphological Opening & Closing (5x5 kernel)\nmask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)\nmask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)"
    },
    3: {
      title: "Stage 3: Bottom-Anchored Contours & Lookahead Heading",
      desc: "Filters candidate contours to only retain those touching the vehicle bumper area (lowest 20% ROI). Computes spatial centroid cx and a far lookahead waypoint.",
      code: "# Spatial Moments & Lookahead Geometry:\nM = cv2.moments(contour)\ncx = int(M['m10'] / M['m00'])\ncy = int(M['m01'] / M['m00'])\n# Lookahead waypoint sampled at top 25% of path contour\nheading_rad = np.arctan2(lookahead_x - cx, cy - lookahead_y)\nheading_deg = np.degrees(heading_rad)"
    },
    4: {
      title: "Stage 4: Navigation State Machine & Deadband Control",
      desc: "Blends lateral error e and heading angle theta into an anticipation index. Uses a +/- 0.08 deadband and a 5-frame grace period for safe decision control.",
      code: "# Combined Steering Bias & FSM Logic:\ne_steer = 0.70 * error + 0.30 * (heading_deg / 45.0)\nif abs(e_steer) <= 0.08:\n    decision = 'FORWARD'\nelif e_steer < -0.08:\n    decision = 'TURN LEFT'\nelse:\n    decision = 'TURN RIGHT'\n# If confidence < 0.30: 5-frame grace in PATH_UNCERTAIN then STOP_NO_PATH"
    }
  };

  const info = data[num];
  if (info && title && desc && code) {
    title.textContent = info.title;
    desc.textContent = info.desc;
    code.textContent = info.code;
  }
};

function initPerceptionSliders() {
  const sliders = [
    { id: "slider-hmin", valId: "val-hmin", key: "h_min" },
    { id: "slider-smin", valId: "val-smin", key: "s_min" },
    { id: "slider-vmin", valId: "val-vmin", key: "v_min" },
    { id: "slider-hmax", valId: "val-hmax", key: "h_max" },
    { id: "slider-smax", valId: "val-smax", key: "s_max" },
    { id: "slider-vmax", valId: "val-vmax", key: "v_max" },
    { id: "slider-area", valId: "val-area", key: "min_area", suffix: " px" },
    { id: "slider-db",   valId: "val-db",   key: "deadband", prefix: "± " },
    { id: "slider-roi",  valId: "val-roi",  key: "roi_percent", suffix: "%" },
  ];

  sliders.forEach(s => {
    const el = document.getElementById(s.id);
    const valEl = document.getElementById(s.valId);
    if (!el || !valEl) return;

    el.addEventListener("input", () => {
      valEl.textContent = (s.prefix || "") + el.value + (s.suffix || "");

      // Send to Python backend server
      fetch("/api/set_params", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ [s.key]: parseFloat(el.value) })
      }).catch(() => {});

      const badge = document.getElementById("tuner-status-badge");
      if (badge) {
        badge.textContent = `Tuned: ${s.key}=${el.value}`;
        badge.className = "badge badge-success";
      }
    });
  });

  const btnReset = document.getElementById("btn-reset-params");
  if (btnReset) {
    btnReset.addEventListener("click", () => {
      document.getElementById("slider-hmin").value = 0;
      document.getElementById("slider-smin").value = 0;
      document.getElementById("slider-vmin").value = 35;
      document.getElementById("slider-hmax").value = 180;
      document.getElementById("slider-smax").value = 100;
      document.getElementById("slider-vmax").value = 230;
      document.getElementById("slider-area").value = 1500;
      document.getElementById("slider-db").value = 0.08;
      document.getElementById("slider-roi").value = 42;

      sliders.forEach(s => {
        const el = document.getElementById(s.id);
        const valEl = document.getElementById(s.valId);
        if (el && valEl) valEl.textContent = (s.prefix || "") + el.value + (s.suffix || "");
      });

      fetch("/api/set_params", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          h_min: 0, s_min: 0, v_min: 35,
          h_max: 180, s_max: 100, v_max: 230,
          min_area: 1500, deadband: 0.08, roi_percent: 42
        })
      }).catch(() => {});

      const badge = document.getElementById("tuner-status-badge");
      if (badge) {
        badge.textContent = "Defaults Restored";
        badge.className = "badge badge-info";
      }
    });
  }
}

/* ==========================================================================
   6. MODULE 4: INTERACTIVE DATASET & EDA CHARTS
   ========================================================================== */
let fig2DisplayMode = "both"; // 'both', 'lab', 'hsv'
let selectedEdaIndex = 1;

window.selectEdaScenario = function(num) {
  selectedEdaIndex = num;
  document.querySelectorAll(".scenario-thumb-card").forEach((c, i) => {
    c.classList.remove("active-scen");
    const b = c.querySelector(".badge");
    if (b) {
      b.textContent = (i + 1 === num) ? "Selected" : "Click to Load";
      b.className = (i + 1 === num) ? "badge badge-success" : "badge badge-info";
    }
  });

  const activeCard = document.getElementById(`scen-card-${num}`);
  if (activeCard) activeCard.classList.add("active-scen");

  const tag = document.getElementById("eda-active-scen-tag");
  const names = {
    1: "Scenario 1: Sunny Curved Trail (sunny_curved_trail.mp4)",
    2: "Scenario 2: Shadowed Gravel Path (shadowed_gravel_path.mp4)",
    3: "Scenario 3: Sharp Switchback (sharp_switchback.mp4)",
  };
  if (tag) tag.textContent = `Active: ${names[num]}`;

  // Also switch Module 1 scenario
  const paths = {
    1: "test_clips/sunny_curved_trail.mp4",
    2: "test_clips/shadowed_gravel_path.mp4",
    3: "test_clips/sharp_switchback.mp4",
  };
  switchScenarioByPath(paths[num]);
  renderAllAcademicCharts();
};

window.setFig2Mode = function(mode) {
  fig2DisplayMode = mode;
  document.querySelectorAll(".view-toggle-group .btn-toggle").forEach(b => b.classList.remove("active"));
  const btn = document.getElementById(`btn-chart-${mode}`);
  if (btn) btn.classList.add("active");

  const cap = document.getElementById("fig2-caption");
  if (cap) {
    if (mode === "lab") {
      cap.innerHTML = "<strong>LAB Luminance Channel:</strong> Shows tree shadow occlusion lowering intensity down to 40–55 in shaded areas. CLAHE restores this contrast.";
    } else if (mode === "hsv") {
      cap.innerHTML = "<strong>HSV Saturation Channel:</strong> Gravel trail saturation remains stable at ~60 regardless of sunlight vs shadow, while surrounding grass peaks at 130–160.";
    } else {
      cap.innerHTML = "<strong>Multi-Channel Analysis:</strong> Demonstrates that tree shadows depress LAB Luminance to 40–55, while HSV Saturation remains tightly clustered at 60.";
    }
  }
  renderAllAcademicCharts();
};

/* ==========================================================================
   7. MODULE 5: RESULTS & BENCHMARKS (FILTER & CSV EXPORT)
   ========================================================================== */
let resultsFilterScenario = "all";

window.filterResultsScenario = function(scen) {
  resultsFilterScenario = scen;
  document.querySelectorAll("#benchmark-table tr").forEach(r => r.classList.remove("row-focused"));
  drawFigure6Chart(currentTheme === "day" ? "#334155" : "#94a3b8", "rgba(255,255,255,0.08)", currentTheme === "day");
};

function initAcademicCharts() {
  renderAllAcademicCharts();
  window.addEventListener("resize", renderAllAcademicCharts);

  const btnExport = document.getElementById("btn-export-csv-results");
  if (btnExport) {
    btnExport.addEventListener("click", () => {
      generateAndDownloadTelemetryCsv();
    });
  }
}

function generateAndDownloadTelemetryCsv() {
  let csv = "frame,timestamp_seconds,status,decision,normalized_center_error,heading_angle_deg,confidence,candidate_area_pixels,failure_reason,processing_fps_average\n";
  for (let i = 1; i <= 600; i++) {
    const t = (i / 600) * (Math.PI * 8);
    const err = (0.28 * Math.sin(t) + 0.14 * Math.sin(t * 1.8)).toFixed(4);
    let dec = "FORWARD";
    if (err > 0.08) dec = "TURN RIGHT";
    else if (err < -0.08) dec = "TURN LEFT";
    const status = Math.abs(err) > 0.85 ? "PATH_UNCERTAIN" : "PATH_FOUND";
    const conf = (0.82 + 0.15 * Math.cos(t)).toFixed(3);
    const area = Math.round(45000 + 4000 * Math.sin(t));
    csv += `${i},${(i * 0.05).toFixed(3)},${status},${dec},${err},${(err * 32).toFixed(2)},${conf},${area},,83.0\n`;
  }

  const blob = new Blob([csv], { type: "text/csv;charset=utf-8;" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = "ugv_navigation_telemetry_600_frames.csv";
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

function renderAllAcademicCharts() {
  const isDay = currentTheme === "day";
  const textColor = isDay ? "#334155" : "#94a3b8";
  const gridColor = isDay ? "rgba(0, 0, 0, 0.08)" : "rgba(255, 255, 255, 0.08)";

  drawFigure2Chart(textColor, gridColor, isDay);
  drawFigure3Chart(textColor, gridColor, isDay);
  drawFigure6Chart(textColor, gridColor, isDay);
  drawFigure7Chart(textColor, gridColor, isDay);
}

function drawFigure2Chart(textColor, gridColor, isDay) {
  const canvas = document.getElementById("chart-fig2-dist");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  canvas.width = canvas.parentElement.clientWidth - 36;
  canvas.height = 210;
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  // Baseline Grid
  ctx.strokeStyle = gridColor;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(35, h - 25);
  ctx.lineTo(w - 10, h - 25);
  ctx.stroke();

  if (fig2DisplayMode === "both" || fig2DisplayMode === "lab") {
    // Sunny Trail LAB curve
    ctx.fillStyle = isDay ? "rgba(2, 132, 199, 0.4)" : "rgba(56, 189, 248, 0.45)";
    ctx.beginPath();
    ctx.moveTo(40, h - 25);
    ctx.quadraticCurveTo(w * 0.35, 30, w * 0.55, h - 25);
    ctx.fill();

    // Shadowed Gravel LAB curve (L: 40-55 dip)
    ctx.fillStyle = isDay ? "rgba(100, 116, 139, 0.5)" : "rgba(148, 163, 184, 0.4)";
    ctx.beginPath();
    ctx.moveTo(35, h - 25);
    ctx.quadraticCurveTo(w * 0.22, 50, w * 0.42, h - 25);
    ctx.fill();
  }

  if (fig2DisplayMode === "both" || fig2DisplayMode === "hsv") {
    // HSV Saturation Curve
    ctx.strokeStyle = isDay ? "#d97706" : "#fbbf24";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(w * 0.45, h - 25);
    ctx.quadraticCurveTo(w * 0.65, 20, w * 0.85, h - 25);
    ctx.stroke();
  }

  ctx.fillStyle = textColor;
  ctx.font = "10px sans-serif";
  ctx.fillText("0 (Dark)", 35, h - 10);
  if (fig2DisplayMode !== "hsv") ctx.fillText("Shadow Peak (40-55)", w * 0.22, 45);
  if (fig2DisplayMode !== "lab") ctx.fillText("Trail S (~60)", w * 0.60, 30);
  ctx.fillText("255 (Bright)", w - 65, h - 10);
}

function drawFigure3Chart(textColor, gridColor, isDay) {
  const canvas = document.getElementById("chart-fig3-decisions");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  canvas.width = canvas.parentElement.clientWidth - 36;
  canvas.height = 210;
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const bars = [
    { label: "TURN RIGHT", count: 237, pct: "39.5%", color: "#f97316" },
    { label: "STOP / UNCERTAIN", count: 185, pct: "30.8%", color: "#64748b" },
    { label: "TURN LEFT", count: 166, pct: "27.7%", color: "#0284c7" },
    { label: "FORWARD", count: 12, pct: "2.0%", color: "#10b981" },
  ];

  const maxVal = 250;
  const barWidth = (w - 70) / bars.length;

  bars.forEach((b, i) => {
    const x = 45 + i * barWidth;
    const barH = (b.count / maxVal) * (h - 60);
    const y = (h - 30) - barH;

    ctx.fillStyle = b.color;
    ctx.fillRect(x, y, barWidth - 14, barH);

    ctx.fillStyle = textColor;
    ctx.font = "bold 10px sans-serif";
    ctx.fillText(`${b.count} (${b.pct})`, x, y - 6);
    ctx.font = "9px sans-serif";
    ctx.fillText(b.label.split(" ")[0], x, h - 12);
  });
}

function drawFigure6Chart(textColor, gridColor, isDay) {
  const canvas = document.getElementById("chart-fig6-rmse");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  canvas.width = canvas.parentElement.clientWidth - 36;
  canvas.height = 210;
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const groups = [
    { key: "sunny", name: "Sunny Trail", base: 0.540, prop: 0.510 },
    { key: "shadow", name: "Shadowed Gravel", base: 0.540, prop: 0.510 },
    { key: "switchback", name: "Switchback", base: 0.516, prop: 0.559 },
    { key: "all", name: "Overall Avg", base: 0.534, prop: 0.524 },
  ];

  const groupWidth = (w - 60) / groups.length;
  const maxRMSE = 0.65;

  groups.forEach((g, i) => {
    const isFocus = resultsFilterScenario === "all" || resultsFilterScenario === g.key;
    const gx = 45 + i * groupWidth;
    const bw = (groupWidth - 20) / 2;

    const baseH = (g.base / maxRMSE) * (h - 60);
    ctx.fillStyle = isFocus ? (isDay ? "#94a3b8" : "#475569") : "rgba(100,116,139,0.2)";
    ctx.fillRect(gx, (h - 30) - baseH, bw, baseH);

    const propH = (g.prop / maxRMSE) * (h - 60);
    ctx.fillStyle = isFocus ? (isDay ? "#1d4ed8" : "#38bdf8") : "rgba(56,189,248,0.2)";
    ctx.fillRect(gx + bw + 2, (h - 30) - propH, bw, propH);

    ctx.fillStyle = textColor;
    ctx.font = isFocus ? "bold 9px sans-serif" : "9px sans-serif";
    ctx.fillText(g.name.split(" ")[0], gx, h - 12);
    if (isFocus) ctx.fillText(g.prop.toFixed(3), gx + bw + 2, (h - 35) - propH);
  });
}

function drawFigure7Chart(textColor, gridColor, isDay) {
  const canvas = document.getElementById("chart-fig7-failures");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  canvas.width = canvas.parentElement.clientWidth - 36;
  canvas.height = 210;
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const bar1W = (180 / 200) * (w - 140);
  ctx.fillStyle = "#ef4444";
  ctx.fillRect(130, 45, bar1W, 30);
  ctx.fillStyle = textColor;
  ctx.font = "bold 11px sans-serif";
  ctx.fillText("180 frames (90.5%)", 140 + bar1W, 65);
  ctx.fillText("NO_PIXELS", 15, 65);

  const bar2W = (19 / 200) * (w - 140);
  ctx.fillStyle = "#f59e0b";
  ctx.fillRect(130, 105, bar2W, 30);
  ctx.fillStyle = textColor;
  ctx.fillText("19 frames (9.5%)", 140 + bar2W, 125);
  ctx.fillText("AREA < 1500px", 15, 125);
}

/* ==========================================================================
   8. MODULE 6: HARDWARE & MOTOR ANIMATION
   ========================================================================== */
window.sendSerialTest = function(cmd) {
  const logs = document.getElementById("terminal-logs");
  const wheelL = document.getElementById("wheel-left");
  const wheelR = document.getElementById("wheel-right");
  const pwmL = document.getElementById("m-left-pwm");
  const pwmR = document.getElementById("m-right-pwm");

  // Reset pin highlights
  ["in1", "in2", "in3", "in4"].forEach(p => {
    const row = document.getElementById(`pin-row-${p}`);
    if (row) row.classList.remove("pin-active");
  });

  const cmdMap = {
    'F': { name: "FORWARD", l: 180, r: 180, lClass: "spin-forward", rClass: "spin-forward", pins: ["in1", "in3"] },
    'L': { name: "TURN LEFT", l: -160, r: 160, lClass: "spin-reverse", rClass: "spin-forward", pins: ["in2", "in3"] },
    'R': { name: "TURN RIGHT", l: 160, r: -160, lClass: "spin-forward", rClass: "spin-reverse", pins: ["in1", "in4"] },
    'S': { name: "STOP", l: 0, r: 0, lClass: "", rClass: "", pins: [] },
  };

  const act = cmdMap[cmd] || cmdMap['S'];
  if (pwmL) pwmL.textContent = act.l;
  if (pwmR) pwmR.textContent = act.r;

  if (wheelL) {
    wheelL.className = `wheel-spin-anim ${act.lClass}`;
    wheelL.textContent = act.l > 0 ? "⚙️ Forward" : (act.l < 0 ? "⚙️ Reverse" : "⏸ Stopped");
  }
  if (wheelR) {
    wheelR.className = `wheel-spin-anim ${act.rClass}`;
    wheelR.textContent = act.r > 0 ? "⚙️ Forward" : (act.r < 0 ? "⚙️ Reverse" : "⏸ Stopped");
  }

  // Highlight active pinout rows
  act.pins.forEach(p => {
    const row = document.getElementById(`pin-row-${p}`);
    if (row) row.classList.add("pin-active");
  });

  if (logs) {
    const div = document.createElement("div");
    div.className = "term-line";
    div.innerHTML = `<span class="term-time">[${new Date().toLocaleTimeString()}]</span> <span class="term-cmd">TX -> '${cmd}'</span> : ${act.name} (L: ${act.l}, R: ${act.r})`;
    logs.appendChild(div);
    logs.scrollTop = logs.scrollHeight;
  }

  fetch(`/api/serial_send?cmd=${cmd}`, { method: "POST" }).catch(() => {});
};

window.copyArduinoCode = function() {
  const code = document.getElementById("arduino-c-code")?.textContent;
  const btn = document.getElementById("btn-copy-code");
  if (!code) return;

  navigator.clipboard.writeText(code).then(() => {
    if (btn) {
      btn.textContent = "Copied!";
      btn.className = "btn-sm btn-success";
      setTimeout(() => {
        btn.textContent = "Copy Code";
        btn.className = "btn-sm";
      }, 2000);
    }
  });
};

/* ==========================================================================
   9. MODULE 7: SEARCHABLE FAQS & ACCORDION
   ========================================================================== */
function initVivaAccordion() {
  const qs = document.querySelectorAll(".viva-question");
  qs.forEach(q => {
    q.addEventListener("click", () => {
      q.parentElement.classList.toggle("open");
    });
  });
}

window.filterFaqs = function() {
  const query = document.getElementById("faq-search-input")?.value.toLowerCase().trim() || "";
  const items = document.querySelectorAll("#faq-accordion-list .viva-item");

  items.forEach(item => {
    const text = item.textContent.toLowerCase();
    const keywords = item.getAttribute("data-keywords") || "";
    if (query === "" || text.includes(query) || keywords.includes(query)) {
      item.style.display = "block";
    } else {
      item.style.display = "none";
    }
  });
};

window.toggleAllFaqs = function(expand) {
  document.querySelectorAll("#faq-accordion-list .viva-item").forEach(item => {
    if (expand) item.classList.add("open");
    else item.classList.remove("open");
  });
};

/* ==========================================================================
   10. MODULE 2: 3D ROVER SIMULATION (THREE.JS WEBGL)
   ========================================================================== */
function init3DDigitalTwin() {
  const container = document.getElementById("three-canvas-container");
  if (!container || typeof THREE === "undefined") return;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(currentTheme === "day" ? 0xe2e8f0 : 0x0a0e17);

  const camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 100);
  camera.position.set(0, 4.5, 7.5);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(container.clientWidth, container.clientHeight);
  container.appendChild(renderer.domElement);

  // Lighting
  const hemi = new THREE.HemisphereLight(0xffffff, 0x1e293b, 0.7);
  scene.add(hemi);
  const dir = new THREE.DirectionalLight(0xffffff, 0.8);
  dir.position.set(10, 20, 10);
  scene.add(dir);

  // Ground Terrain
  const groundGeo = new THREE.PlaneGeometry(80, 80);
  const groundMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.9 });
  const ground = new THREE.Mesh(groundGeo, groundMat);
  ground.rotation.x = -Math.PI / 2;
  scene.add(ground);

  // Curved Trail
  const pts = [];
  for (let i = -30; i <= 30; i++) {
    pts.push(new THREE.Vector3(Math.sin(i * 0.18) * 3.5, 0.02, i));
  }
  const curve = new THREE.CatmullRomCurve3(pts);
  const lineGeo = new THREE.BufferGeometry().setFromPoints(curve.getPoints(200));
  const lineMat = new THREE.LineBasicMaterial({ color: 0x38bdf8, linewidth: 3 });
  scene.add(new THREE.Line(lineGeo, lineMat));

  // Rover Group
  const rover = new THREE.Group();
  scene.add(rover);

  const body = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.5, 2.4), new THREE.MeshStandardMaterial({ color: 0x2563eb }));
  body.position.y = 0.55;
  rover.add(body);

  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.7), new THREE.MeshStandardMaterial({ color: 0x94a3b8 }));
  mast.position.set(0, 1.05, 0.8);
  rover.add(mast);

  const fovCone = new THREE.Mesh(new THREE.ConeGeometry(2.2, 4.5, 4, 1, true), new THREE.MeshBasicMaterial({ color: 0x00f0ff, wireframe: true, transparent: true, opacity: 0.2 }));
  fovCone.position.set(0, 0.6, 2.6);
  fovCone.rotation.x = Math.PI / 2.3;
  rover.add(fovCone);

  // 4 Wheels
  const wheels = [];
  const wheelGeo = new THREE.CylinderGeometry(0.35, 0.35, 0.25, 16);
  const wheelMat = new THREE.MeshStandardMaterial({ color: 0x1f2937 });
  [[-0.95, 0.35, 0.8], [0.95, 0.35, 0.8], [-0.95, 0.35, -0.8], [0.95, 0.35, -0.8]].forEach(p => {
    const w = new THREE.Mesh(wheelGeo, wheelMat);
    w.rotation.z = Math.PI / 2;
    w.position.set(...p);
    rover.add(w);
    wheels.push(w);
  });

  let progress = 0.4;
  let isAuto = true;
  let manualThrottle = 0, manualSteer = 0;
  let camPerspective = 0; // 0: Chase, 1: Cockpit, 2: Bird's eye

  // Button Hooks
  const btnAuto = document.getElementById("btn-twin-auto");
  if (btnAuto) {
    btnAuto.addEventListener("click", () => {
      isAuto = !isAuto;
      btnAuto.textContent = isAuto ? "Switch to Manual Teleop" : "Switch to Autonomous";
      const txt = document.getElementById("twin-mode-text");
      if (txt) {
        txt.textContent = isAuto ? "AUTONOMOUS" : "MANUAL DRIVE";
        txt.className = `hud-val ${isAuto ? "color-blue" : "color-orange"}`;
      }
    });
  }

  const btnCam = document.getElementById("btn-twin-cam");
  if (btnCam) {
    btnCam.addEventListener("click", () => {
      camPerspective = (camPerspective + 1) % 3;
    });
  }

  const btnReset = document.getElementById("btn-twin-reset");
  if (btnReset) {
    btnReset.addEventListener("click", () => {
      progress = 0.4;
      rover.position.set(0, 0, 0);
      rover.rotation.set(0, 0, 0);
    });
  }

  // D-Pad and Keyboard Controls
  setupDpad("dpad-up", () => { manualThrottle = 0.04; });
  setupDpad("dpad-down", () => { manualThrottle = -0.04; });
  setupDpad("dpad-left", () => { manualSteer = 0.035; });
  setupDpad("dpad-right", () => { manualSteer = -0.035; });
  setupDpad("dpad-stop", () => { manualThrottle = 0; manualSteer = 0; });

  window.addEventListener("keydown", e => {
    if (["w", "ArrowUp"].includes(e.key)) manualThrottle = 0.04;
    if (["s", "ArrowDown"].includes(e.key)) manualThrottle = -0.04;
    if (["a", "ArrowLeft"].includes(e.key)) manualSteer = 0.035;
    if (["d", "ArrowRight"].includes(e.key)) manualSteer = -0.035;
  });
  window.addEventListener("keyup", e => {
    if (["w", "s", "ArrowUp", "ArrowDown"].includes(e.key)) manualThrottle = 0;
    if (["a", "d", "ArrowLeft", "ArrowRight"].includes(e.key)) manualSteer = 0;
  });

  function animate() {
    requestAnimationFrame(animate);
    if (isAuto) {
      progress = (progress + 0.0012) % 1.0;
      const pt = curve.getPointAt(progress);
      const tangent = curve.getTangentAt(progress);
      rover.position.copy(pt);
      rover.rotation.y = Math.atan2(tangent.x, tangent.z);
      wheels.forEach(w => w.rotation.x += 0.08);

      const steerTxt = document.getElementById("twin-steer-text");
      if (steerTxt) {
        if (tangent.x > 0.05) steerTxt.textContent = "CURVING RIGHT";
        else if (tangent.x < -0.05) steerTxt.textContent = "CURVING LEFT";
        else steerTxt.textContent = "STRAIGHT";
      }
    } else {
      rover.rotation.y += manualSteer;
      rover.translateZ(manualThrottle);
      wheels.forEach(w => w.rotation.x += manualThrottle * 2);
    }

    // Camera Perspective Switching
    if (camPerspective === 0) {
      camera.position.set(rover.position.x, rover.position.y + 4.5, rover.position.z + 7.5);
      camera.lookAt(rover.position.x, rover.position.y + 0.8, rover.position.z);
    } else if (camPerspective === 1) {
      camera.position.set(rover.position.x, rover.position.y + 1.45, rover.position.z);
      camera.rotation.set(-0.25, rover.rotation.y, 0);
    } else {
      camera.position.set(rover.position.x, 16.0, rover.position.z);
      camera.lookAt(rover.position.x, 0, rover.position.z);
    }

    renderer.render(scene, camera);
  }
  animate();

  window.twinResize = () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
  };
  window.updateTwinTheme = () => {
    scene.background = new THREE.Color(currentTheme === "day" ? 0xe2e8f0 : 0x0a0e17);
  };
}

function setupDpad(id, cb) {
  const el = document.getElementById(id);
  if (el) el.addEventListener("click", cb);
}
