// ================= SETTINGS =================
// Leaflet uses [y, x] order. Units are metres.
// MUST match RX_X / RX_Y in the Command Hub's Main.cpp.
const ROOM = [[0, 0], [8, 10]];                  // 8 m deep (y), 10 m wide (x)
const RECEIVERS = {                              // name: [y, x]
  Hub: [0, 0],                                   // RX_X[0], RX_Y[0]
  A1:  [0, 10],                                  // RX_X[1], RX_Y[1]
  A2:  [8, 0],                                   // RX_X[2], RX_Y[2]
};
const STALE_MS = 5000;                           // no update for 5 s = signal lost
const LOW_BATT = 20;                             // battery % warning level

// What each Hub mode means, for the side panel
const MODE_TEXT = {
  fix:   'RSSI fix',          // 3+ receivers: trilaterated
  dr:    'Dead reckoning',    // fewer receivers: following steps since last fix
  range: 'Near receiver',     // no position yet: only "near the closest receiver"
};

// ================= MAP =================
const map = L.map('map', { crs: L.CRS.Simple, minZoom: -2, attributionControl: false });
map.fitBounds(ROOM);

L.rectangle(ROOM, { color: '#888', weight: 2, fill: false }).addTo(map);
// L.imageOverlay('floorplan.png', ROOM).addTo(map);   // ← enable when you have a floor plan

for (const [name, pos] of Object.entries(RECEIVERS)) {
  L.circleMarker(pos, { radius: 6, color: '#fff', fillColor: '#444', fillOpacity: 1 })
    .bindTooltip(name, { permanent: true, direction: 'top' })
    .addTo(map);
}

// ================= TRACKERS =================
const trackers = {};   // id → { marker, ring, data, lastSeen }

// Called every time a message arrives from the Hub
function update(msg) {
  const pos = [msg.y, msg.x];
  let t = trackers[msg.id];

  if (!t) {   // first time we've seen this tracker: create its marker + uncertainty ring
    t = trackers[msg.id] = {
      ring: L.circle(pos, { radius: 1, color: '#e53935', weight: 1, fillOpacity: 0.12 })
        .addTo(map),
      marker: L.circleMarker(pos, { radius: 9, color: '#fff', fillColor: '#e53935', fillOpacity: 1 })
        .bindTooltip(msg.id, { permanent: true, direction: 'right' })
        .addTo(map),
    };
  }

  // Ring size = how unsure the position is:
  //   fix   → rms (how much the receivers disagree)
  //   dr / range → distance to the nearest receiver
  const radius = msg.mode === 'fix' ? msg.rms : msg.r;
  t.ring.setLatLng(pos).setRadius(Math.max(radius || 0, 0.5));
  t.ring.setStyle({ dashArray: msg.mode === 'fix' ? null : '4 4' });   // dashed = less certain

  // Dark red for alarms, orange for not-a-real-fix, red for a normal fix
  const alarm = msg.mandown || msg.heat;
  t.marker.setStyle({ fillColor: alarm ? '#b71c1c' : msg.mode === 'fix' ? '#e53935' : '#fb8c00' });

  t.marker.setLatLng(pos);
  t.data = msg;
  t.lastSeen = Date.now();
  renderPanel();
}

// Shows a value with units, or "–" if the Hub didn't send it
function show(value, unit = '', digits = 1) {
  if (value === undefined || value === null || value < 0) return '–';
  return (typeof value === 'number' ? value.toFixed(digits) : value) + unit;
}

// Rebuilds the side panel cards
function renderPanel() {
  const crew = document.getElementById('crew');
  const ids = Object.keys(trackers).sort();

  if (ids.length === 0) {
    crew.innerHTML = '<p class="empty">Waiting for trackers…</p>';
    return;
  }

  crew.innerHTML = ids.map(id => {
    const { data: d, lastSeen } = trackers[id];
    const ageSec = (Date.now() - lastSeen) / 1000;

    const alerts = [];
    if (d.mandown)                                 alerts.push('MAN DOWN');
    if (d.heat)                                    alerts.push('HEAT');
    if (ageSec * 1000 > STALE_MS)                  alerts.push('Signal lost');
    if (d.fault)                                   alerts.push('Sensor fault');
    if (d.bat !== undefined && d.bat < LOW_BATT)   alerts.push('Low battery');
    if (d.moving === 0 && !d.mandown)              alerts.push('Stationary');

    return `
      <div class="card ${alerts.length ? 'alert' : ''}">
        <h3>${id}</h3>
        <div class="row"><span>Position</span><span>${show(d.x)}, ${show(d.y)} m</span></div>
        <div class="row"><span>Accuracy</span><span>±${show(d.mode === 'fix' ? d.rms : d.r, ' m')}</span></div>
        <div class="row"><span>Mode</span><span>${MODE_TEXT[d.mode] || show(d.mode)} (${show(d.rx, '', 0)} rx)</span></div>
        <div class="row"><span>Floor</span><span>${show(d.floor, '', 0)} (${show(d.alt, ' m')})</span></div>
        <div class="row"><span>Temp</span><span>${show(d.temp, ' °C')}</span></div>
        <div class="row"><span>Battery</span><span>${show(d.bat, '%', 0)}</span></div>
        <div class="row"><span>Last seen</span><span>${ageSec.toFixed(0)} s ago</span></div>
        ${alerts.length ? `<div class="alerts">⚠ ${alerts.join(' · ')}</div>` : ''}
      </div>`;
  }).join('');
}

// Refresh every second so "last seen" and "signal lost" stay up to date
setInterval(renderPanel, 1000);

// ================= CONNECTION =================
function setConn(state) {
  const el = document.getElementById('conn');
  el.className = 'conn ' + state;
  el.textContent = { online: 'Live', offline: 'Offline', demo: 'Demo' }[state];
}

function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen    = () => setConn('online');
  ws.onclose   = () => { setConn('offline'); setTimeout(connect, 2000); };  // auto-reconnect
  ws.onmessage = (e) => {
    try { update(JSON.parse(e.data)); }
    catch (err) { console.warn('Bad message from Hub:', e.data); }
  };
}

// ================= DEMO MODE =================
// Fake firefighters in the same format the Hub sends, used when opened from your Mac
function startDemo() {
  setConn('demo');
  let t = 0;
  setInterval(() => {
    t += 0.1;
    update({ id: 'T1', mode: 'fix', x: 5 + 3 * Math.cos(t), y: 4 + 2.5 * Math.sin(t),
             r: 0, rms: 1.2, rx: 3, floor: 0, alt: 0.3, temp: 24.5, bat: 76,
             moving: 1, mandown: 0, heat: 0, fault: 0 });
    update({ id: 'T2', mode: 'dr', x: 2, y: 6, r: 3.5, rms: -1, rx: 2, floor: 1, alt: 2.8,
             temp: 31.0, bat: 15, moving: 0, mandown: 0, heat: 0, fault: 0 });
  }, 500);
}

// ================= START =================
if (location.protocol === 'file:') startDemo();   // opened from Finder
else connect();                                   // served by the Command Hub