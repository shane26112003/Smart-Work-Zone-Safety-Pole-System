/**
 * Smart Work-Zone Safety Pole System - Dashboard Application
 * High-performance HTML5 Canvas BEV Radar, WebSockets, & Web Audio Alerts.
 */

// Configuration & State
let ws = null;
let audioEnabled = true;
let audioCtx = null;
let currentScenario = 'normal';
let lastRiskLevel = 'SAFE';

// Audio Alarm synthesis
function playBeep(freq, type, duration) {
  if (!audioEnabled) return;
  try {
    if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    if (audioCtx.state === 'suspended') audioCtx.resume();

    const osc = audioCtx.createOscillator();
    const gain = audioCtx.createGain();

    osc.type = type;
    osc.frequency.setValueAtTime(freq, audioCtx.currentTime);

    gain.gain.setValueAtTime(0.15, audioCtx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + duration);

    osc.connect(gain);
    gain.connect(audioCtx.destination);

    osc.start();
    osc.stop(audioCtx.currentTime + duration);
  } catch (e) {
    console.error("Audio error", e);
  }
}

// Toggle audio
document.getElementById('audioToggleBtn').addEventListener('click', () => {
  audioEnabled = !audioEnabled;
  const txt = document.getElementById('audioStatusText');
  txt.innerText = audioEnabled ? 'ENABLED' : 'MUTED';
  txt.style.color = audioEnabled ? '#10b981' : '#ef4444';
  if (audioEnabled) playBeep(880, 'sine', 0.15);
});

// Canvas Radar Setup
const canvas = document.getElementById('radarCanvas');
const ctx = canvas.getContext('2d');

function resizeCanvas() {
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = rect.width;
  canvas.height = rect.height;
}
window.addEventListener('resize', resizeCanvas);
setTimeout(resizeCanvas, 100);

// Coordinate Mapper: Metric (X, Y) -> Canvas (px, py)
// Origin (0,0) is Safety Pole, placed laterally at 40% width, vertically at 85% height
function metricToCanvas(xm, ym) {
  const originX = canvas.width * 0.42;
  const originY = canvas.height * 0.88;
  const scale = canvas.height / 42.0; // 42 meters vertical view

  const px = originX + xm * scale;
  const py = originY - ym * scale;
  return { px, py, scale };
}

// Draw Tactical 2D Radar Canvas
function renderRadar(entities, lidar, guidance) {
  if (!canvas.width || !canvas.height) return;

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const originX = canvas.width * 0.42;
  const originY = canvas.height * 0.88;
  const scale = canvas.height / 42.0;

  // 1. Draw Grid Lines (every 5 meters)
  ctx.strokeStyle = '#1a233d';
  ctx.lineWidth = 1;
  ctx.font = '10px monospace';
  ctx.fillStyle = '#4b5563';

  for (let y = 0; y <= 40; y += 5) {
    const pt = metricToCanvas(0, y);
    ctx.beginPath();
    ctx.moveTo(0, pt.py);
    ctx.lineTo(canvas.width, pt.py);
    ctx.stroke();
    ctx.fillText(`${y}m`, 10, pt.py - 3);
  }

  // Range rings centered at Safety Pole
  for (let r = 5; r <= 35; r += 10) {
    ctx.beginPath();
    ctx.arc(originX, originY, r * scale, Math.PI, 2 * Math.PI);
    ctx.stroke();
  }

  // 2. Road Lanes & Work Zone Ground Areas
  // Work Zone (X < 0)
  const wzLeft = metricToCanvas(-12.0, 40.0);
  const wzRight = metricToCanvas(0.0, 0.0);
  ctx.fillStyle = 'rgba(30, 41, 59, 0.4)';
  ctx.fillRect(wzLeft.px, wzLeft.py, wzRight.px - wzLeft.px, wzRight.py - wzLeft.py);

  // Road Lanes (X > 0)
  const roadLeft = metricToCanvas(0.0, 40.0);
  const roadRight = metricToCanvas(8.0, 0.0);
  ctx.fillStyle = 'rgba(15, 23, 42, 0.7)';
  ctx.fillRect(roadLeft.px, roadLeft.py, roadRight.px - roadLeft.px, roadRight.py - roadLeft.py);

  // Safety Cone Divider Line (X = 0)
  ctx.strokeStyle = '#f59e0b';
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 6]);
  ctx.beginPath();
  const cTop = metricToCanvas(0, 40);
  const cBot = metricToCanvas(0, 0);
  ctx.moveTo(cTop.px, cTop.py);
  ctx.lineTo(cBot.px, cBot.py);
  ctx.stroke();
  ctx.setLineDash([]);

  // 3. TSD20 LiDAR Ray
  if (lidar && lidar.valid) {
    const lEnd = metricToCanvas(0.0, lidar.distance_m);
    
    // Gradient beam
    const grad = ctx.createLinearGradient(originX, originY, lEnd.px, lEnd.py);
    grad.addColorStop(0, 'rgba(16, 185, 129, 0.7)');
    grad.addColorStop(1, 'rgba(239, 68, 68, 0.9)');

    ctx.strokeStyle = grad;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(originX, originY);
    ctx.lineTo(lEnd.px, lEnd.py);
    ctx.stroke();

    // Measurement impact bar
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.arc(lEnd.px, lEnd.py, 5, 0, 2 * Math.PI);
    ctx.fill();
  }

  // 4. Safety Pole Base
  ctx.fillStyle = '#3b82f6';
  ctx.beginPath();
  ctx.arc(originX, originY, 9, 0, 2 * Math.PI);
  ctx.fill();
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.fillStyle = '#fff';
  ctx.font = 'bold 10px sans-serif';
  ctx.fillText('SAFETY POLE', originX - 35, originY + 18);

  // 5. Render Fused Entities (Vehicles & Workers)
  if (entities) {
    entities.forEach(ent => {
      const pos = metricToCanvas(ent.x, ent.y);

      // Trajectory ribbon
      if (ent.trajectory && ent.trajectory.length > 0) {
        ctx.strokeStyle = (ent.type === 'vehicle') ? 'rgba(239, 68, 68, 0.4)' : 'rgba(59, 130, 246, 0.4)';
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(pos.px, pos.py);
        ent.trajectory.forEach(pt => {
          const tPt = metricToCanvas(pt[0], pt[1]);
          ctx.lineTo(tPt.px, tPt.py);
        });
        ctx.stroke();
        ctx.setLineDash([]);
      }

      if (ent.type === 'vehicle') {
        // Vehicle Body
        const vWidth = 2.0 * scale;
        const vHeight = 4.2 * scale;

        ctx.save();
        ctx.translate(pos.px, pos.py);
        ctx.rotate((ent.heading_deg || 0) * Math.PI / 180);

        // Body rect
        ctx.fillStyle = ent.lidar_verified ? '#dc2626' : '#ea580c';
        ctx.fillRect(-vWidth / 2, -vHeight / 2, vWidth, vHeight);
        ctx.strokeStyle = '#fef08a';
        ctx.lineWidth = 1.5;
        ctx.strokeRect(-vWidth / 2, -vHeight / 2, vWidth, vHeight);

        // Heading arrow
        ctx.fillStyle = '#fef08a';
        ctx.beginPath();
        ctx.moveTo(0, -vHeight / 2 - 4);
        ctx.lineTo(-4, -vHeight / 2 + 3);
        ctx.lineTo(4, -vHeight / 2 + 3);
        ctx.fill();

        ctx.restore();

        // Label
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 11px sans-serif';
        ctx.fillText(`${ent.id} (${ent.speed_kmh} km/h)`, pos.px + 12, pos.py - 4);
        if (ent.lidar_verified) {
          ctx.fillStyle = '#10b981';
          ctx.font = '9px monospace';
          ctx.fillText(`LiDAR: ${ent.lidar_dist}m`, pos.px + 12, pos.py + 8);
        }

      } else {
        // Worker
        const isFall = ent.worker_state && ent.worker_state.motion_state === 'FALL_DETECTED';

        // Danger clearance radius (1.5m)
        ctx.strokeStyle = isFall ? 'rgba(239, 68, 68, 0.7)' : 'rgba(16, 185, 129, 0.4)';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(pos.px, pos.py, 1.5 * scale, 0, 2 * Math.PI);
        ctx.stroke();

        // Draw Escape Vector Arrow if active guidance
        if (guidance && (guidance.target_worker === ent.id || guidance.target_worker === 'ALL_WORKERS') && 
            (guidance.escape_dx !== 0 || guidance.escape_dy !== 0)) {
          const safePos = metricToCanvas(guidance.safe_x, guidance.safe_y);
          
          ctx.strokeStyle = '#10b981';
          ctx.lineWidth = 3;
          ctx.setLineDash([4, 4]);
          ctx.beginPath();
          ctx.moveTo(pos.px, pos.py);
          ctx.lineTo(safePos.px, safePos.py);
          ctx.stroke();
          ctx.setLineDash([]);

          // Arrowhead at destination
          const angle = Math.atan2(safePos.py - pos.py, safePos.px - pos.px);
          ctx.fillStyle = '#10b981';
          ctx.beginPath();
          ctx.moveTo(safePos.px, safePos.py);
          ctx.lineTo(safePos.px - 10 * Math.cos(angle - Math.PI / 6), safePos.py - 10 * Math.sin(angle - Math.PI / 6));
          ctx.lineTo(safePos.px - 10 * Math.cos(angle + Math.PI / 6), safePos.py - 10 * Math.sin(angle + Math.PI / 6));
          ctx.fill();

          ctx.fillStyle = '#10b981';
          ctx.font = 'bold 10px sans-serif';
          ctx.fillText('ESCAPE ➔', safePos.px - 20, safePos.py - 8);
        }

        // Worker icon
        ctx.fillStyle = isFall ? '#ef4444' : '#f59e0b';
        ctx.beginPath();
        ctx.arc(pos.px, pos.py, 7, 0, 2 * Math.PI);
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Label
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 11px sans-serif';
        const st = ent.worker_state ? ent.worker_state.motion_state : 'OK';
        ctx.fillText(`${ent.id} [${st}]`, pos.px + 10, pos.py - 3);
      }
    });
  }
}

// Process Incoming WebSocket Messages
function handleIncomingTelemetry(data) {
  // 1. Update Video Streams
  if (data.road_image) {
    document.getElementById('roadVideoImg').src = `data:image/jpeg;base64,${data.road_image}`;
  }
  if (data.workzone_image) {
    document.getElementById('workzoneVideoImg').src = `data:image/jpeg;base64,${data.workzone_image}`;
  }

  // 2. Threat Status & Banner
  const riskPill = document.getElementById('mainRiskPill');
  const riskText = document.getElementById('mainRiskText');
  const level = data.overall_level || 'SAFE';

  riskPill.className = `status-pill pill-${level.toLowerCase().replace(' ', '-')}`;
  riskText.innerText = level;

  // Sound Audio Alarm if Risk Escalates
  if (level === 'CRITICAL' && (Date.now() % 800 < 200)) {
    playBeep(920, 'sawtooth', 0.25);
  } else if (level === 'HIGH RISK' && (Date.now() % 1200 < 200)) {
    playBeep(650, 'triangle', 0.15);
  } else if (level === 'CAUTION' && lastRiskLevel === 'SAFE') {
    playBeep(520, 'sine', 0.12);
  }
  lastRiskLevel = level;

  // 3. Risk Metric Values
  const valTtc = document.getElementById('valTtc');
  if (data.min_ttc_sec < 90) {
    valTtc.innerText = `${data.min_ttc_sec.toFixed(1)}s`;
    valTtc.style.color = data.min_ttc_sec < 2.5 ? '#ef4444' : (data.min_ttc_sec < 4.5 ? '#f97316' : '#f59e0b');
  } else {
    valTtc.innerText = '> 10s';
    valTtc.style.color = '#10b981';
  }

  const valDist = document.getElementById('valDist');
  if (data.min_distance_m < 90) {
    valDist.innerText = `${data.min_distance_m.toFixed(1)}m`;
  } else {
    valDist.innerText = '-- m';
  }

  // 4. Hazard Reasons
  const reasonsDiv = document.getElementById('hazardReasonsList');
  if (data.reasons && data.reasons.length > 0) {
    reasonsDiv.innerHTML = data.reasons.map(r => `• ${r}`).join('<br>');
    reasonsDiv.style.color = (level === 'CRITICAL') ? '#ef4444' : '#fb923c';
  } else {
    reasonsDiv.innerText = 'Normal traffic operations. Work zone secure.';
    reasonsDiv.style.color = '#10b981';
  }

  // 5. Radar Overlay Metrics
  document.getElementById('radarLidarDist').innerText = data.lidar && data.lidar.valid ? `${data.lidar.distance_m}m` : '--';
  document.getElementById('radarLidarRate').innerText = data.lidar && data.lidar.valid ? `${data.lidar.range_rate_mps} m/s` : '--';
  document.getElementById('radarCriticalPair').innerText = data.critical_pair ? `${data.critical_pair[0]} ↔ ${data.critical_pair[1]}` : 'None';

  // 5b. Update AI Prescriptive Evasion Directive Card
  if (data.guidance) {
    const g = data.guidance;
    document.getElementById('guidanceWorkerId').innerText = g.target_worker;
    const dirBox = document.getElementById('guidanceDirectiveBox');
    dirBox.innerText = g.directive;
    document.getElementById('guidanceHaptic').innerText = g.haptic;
    document.getElementById('guidanceAction').innerText = g.action;

    const card = document.getElementById('cardGuidance');
    const badge = document.getElementById('guidanceUrgencyBadge');
    badge.innerText = g.urgency;

    if (g.urgency === 'EMERGENCY') {
      card.style.borderLeftColor = '#ef4444';
      badge.style.background = 'rgba(239, 68, 68, 0.25)';
      badge.style.color = '#ef4444';
      dirBox.style.color = '#f87171';
      dirBox.style.borderColor = '#ef4444';
    } else if (g.urgency === 'URGENT') {
      card.style.borderLeftColor = '#f97316';
      badge.style.background = 'rgba(249, 115, 22, 0.25)';
      badge.style.color = '#f97316';
      dirBox.style.color = '#fb923c';
      dirBox.style.borderColor = '#f97316';
    } else if (g.urgency === 'MODERATE') {
      card.style.borderLeftColor = '#f59e0b';
      badge.style.background = 'rgba(245, 158, 11, 0.25)';
      badge.style.color = '#f59e0b';
      dirBox.style.color = '#fbbf24';
      dirBox.style.borderColor = '#f59e0b';
    } else {
      card.style.borderLeftColor = '#10b981';
      badge.style.background = 'rgba(16, 185, 129, 0.2)';
      badge.style.color = '#10b981';
      dirBox.style.color = '#10b981';
      dirBox.style.borderColor = '#1f293d';
    }
  }

  // 6. Actuators Panel
  const acts = data.actuators || {};
  toggleLight('lightCaution', acts.caution_led, 'active-caution');
  toggleLight('lightDanger', acts.danger_led, 'active-danger');
  toggleLight('lightSiren', acts.siren, 'active-siren');
  toggleLight('lightBuzzer', acts.buzzer, 'active-caution');

  // 7. Worker Cards
  if (data.workers) {
    document.getElementById('hdrWorkerCount').innerText = data.workers.length;
    const container = document.getElementById('workerCardsContainer');
    container.innerHTML = data.workers.map(w => {
      let bClass = 'badge-static';
      if (w.motion_state === 'WALKING' || w.motion_state === 'RUNNING') bClass = 'badge-walking';
      if (w.motion_state === 'FALL_DETECTED' || w.motion_state === 'IMPACT') bClass = 'badge-fall';

      return `
        <div class="worker-card">
          <div class="worker-header">
            <span>👷 ${w.worker_id}</span>
            <span class="badge-state ${bClass}">${w.motion_state}</span>
          </div>
          <div class="worker-detail-grid">
            <div>SVM: <b>${w.svm} g</b></div>
            <div>Bat: <b>${w.battery} V</b></div>
            <div>RSSI: <b>${w.rssi} dBm</b></div>
            <div>Dist: <b>${w.est_dist_m} m</b></div>
          </div>
        </div>
      `;
    }).join('');
  }

  // 8. Render Radar Canvas (with Escape Vector Arrow)
  renderRadar(data.entities, data.lidar, data.guidance);
}

function toggleLight(id, active, activeClass) {
  const el = document.getElementById(id);
  if (active) el.classList.add(activeClass);
  else el.classList.remove(activeClass);
}

// WebSocket Connection Management
function connectWebSocket() {
  const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const url = `${proto}//${window.location.host}/ws`;

  ws = new WebSocket(url);
  ws.onopen = () => {
    console.log("[WS] Connected to Safety Pole server.");
  };

  ws.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      handleIncomingTelemetry(data);
    } catch (e) {
      console.error("[WS] Parse error", e);
    }
  };

  ws.onclose = () => {
    console.log("[WS] Closed. Reconnecting in 1.5s...");
    setTimeout(connectWebSocket, 1500);
  };
}

// Switch Scenario
function switchScenario(scenName) {
  currentScenario = scenName;
  document.querySelectorAll('.btn-scenario').forEach(b => b.classList.remove('active'));
  event.target.classList.add('active');

  fetch('/api/scenario', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ scenario: scenName })
  }).then(r => r.json()).then(res => {
    // Log in audit box
    const box = document.getElementById('incidentLogBox');
    const entry = document.createElement('div');
    entry.className = 'log-entry';
    entry.innerText = `[${new Date().toLocaleTimeString()}] Triggered Scenario: ${scenName}`;
    box.prepend(entry);
  });
}

// Initialize on load
window.addEventListener('DOMContentLoaded', () => {
  connectWebSocket();
});
