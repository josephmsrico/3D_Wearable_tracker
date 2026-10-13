// ================= SETTINGS =================
// Leaflet uses [y, x] order. Units are metres.
const ROOM = [[0, 0], [10, 15]];                 // 10 m deep, 15 m wide
const ANCHORS = { A1: [0, 0], A2: [0, 15], A3: [10, 7.5] };
const STALE_MS = 5000;                           // no update for 5 s = signal lost
const LOW_BATT = 20;                             // battery % warning level

// ================= MAP =================
const map = L.map('map', { crs: L.CRS.Simple, minZoom: -2, attributionControl: false });
map.fitBounds(ROOM);

L.rectangle(ROOM, { color: '#888', weight: 2, fill: false }).addTo(map);
// L.imageOverlay('floorplan.png', ROOM).addTo(map);   // ← enable when you have a floor plan

for (const [name, pos] of Object.entries(ANCHORS)) {
  L.circleMarker(pos, { radius: 6, color: '#fff', fillColor: '#444', fillOpacity: 1 })
    .bindTooltip(name, { permanent: true, direction: 'top' })
    .addTo(map);
}

// ================= TRACKERS =================
const trackers = {};   // id → { marker, data, lastSeen }

// Called every time a message arrives from the Hub
function update(msg) {
  const pos = [msg.y, msg.x];
  let t = trackers[msg.id];

  if (!t) {   // first time we've seen this tracker: create its marker
    t = trackers[msg.id] = {
      marker: L.circleMarker(pos, { radius: 9, color: '#fff', fillColor: '#e53935', fillOpacity: 1 })
        .bindTooltip(msg.id, { permanent: true, direction: 'right' })
        .addTo(map)
    };
  }

  t.marker.setLatLng(pos);
  t.data = msg;
  t.lastSeen = Date.now();
  renderPanel();
}

// Shows a value with units, or "–" if the Hub didn't send it
function show(value, unit = '', digits = 1) {
  if (value === undefined || value === null) return '–';
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
    if (ageSec * 1000 > STALE_MS)                   alerts.push('Signal lost');
    if (d.batt !== undefined && d.batt < LOW_BATT)  alerts.push('Low battery');
    if (d.moving === false)                         alerts.push('Stationary');

    return `
      <div class="card ${alerts.length ? 'alert' : ''}">
        <h3>${id}</h3>
        <div class="row"><span>Position</span><span>${show(d.x)}, ${show(d.y)} m</span></div>
        <div class="row"><span>Altitude</span><span>${show(d.alt, ' m')}</span></div>
        <div class="row"><span>Battery</span><span>${show(d.batt, '%', 0)}</span></div>
        <div class="row"><span>RSSI</span><span>${show(d.rssi, ' dBm', 0)}</span></div>
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
// Fake firefighters, used when the page is opened directly from your Mac
function startDemo() {
  setConn('demo');
  let t = 0;
  setInterval(() => {
    t += 0.1;
    update({ id: 'T1', x: 7.5 + 4 * Math.cos(t), y: 5 + 3 * Math.sin(t),
             alt: 1.2, batt: 76, moving: true, rssi: -65 });
    update({ id: 'T2', x: 3, y: 2, alt: 4.1, batt: 15, moving: false, rssi: -78 });
  }, 500);
}

// ================= START =================
if (location.protocol === 'file:') startDemo();   // opened from Finder
else connect();                                   // served by the Command Hub