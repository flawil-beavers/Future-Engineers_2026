// Node.js regression tests for the parser delivered inside the offline app.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const root = path.resolve(__dirname, '../..');
const html = fs.readFileSync(path.join(__dirname, 'Robot Run Inspector.html'), 'utf8');
const box = {module: {exports: {}}};
vm.runInNewContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], box);
const {analyzeLog, plotSvg, escapeHtml, seatPosition, runGeometry, wholeRunSvg} = box.module.exports;
let checks = 0;
function test(label, fn) {fn(); checks++; console.log('PASS ' + label);}
function fixture(name) {return fs.readFileSync(path.join(root, 'simulation/evidence/parking_exit_diagnostics', name), 'utf8');}
test('507: CW, three laps, gyro failures and exact park abort', () => {
 const r = analyzeLog(fixture('20261007_log_507_cw.txt')).sessions[0];
 assert.equal(r.directions.join(','), 'CW');
 assert.equal(Math.max(...r.laps), 3);
 assert.ok(r.health.gyroTimeout > 0);
 assert.equal(r.parking, 'Parkabbruch protokolliert');
 const f = r.findings.find(f => f.id === 'abort-FINAL PARK ABORT');
 assert.ok(r.rows.find(x => x.line === f.lines[0]).text.includes('dual_marker_scan_incomplete'));
 assert.ok(r.tracks.FINAL_PARK_TRACE.length > 10);
 assert.ok(r.tracks['LATER_TRACK · lap 3'].length > 10);
});
test('506: health failure without invented successful parking', () => {
 const r = analyzeLog(fixture('20261007_log_506_cw.txt')).sessions[0];
 assert.equal(r.health.gyroTimeout, 7);
 assert.equal(r.parking, 'Kein Endergebnis protokolliert');
 assert.ok(r.lastStop.includes('Manual disable'));
});
test('session isolation includes startup only in first run', () => {
 const r = analyzeLog('boot\n[OC] New obstacle run\n[FINAL PARK RESULT] contained=yes stopped=yes\n[OC] New obstacle run\n[FINAL PARK ABORT] second');
 assert.equal(r.sessions.length, 2);
 assert.equal(r.sessions[0].start, 1);
 assert.equal(r.sessions[1].start, 4);
 assert.equal(r.sessions[0].parking, 'Software meldet: enthalten und gestoppt');
 assert.equal(r.sessions[1].parking, 'Parkabbruch protokolliert');
});
test('missing data and excerpt are not presented as success', () => {
 const r = analyzeLog('hello', 'run_excerpt.txt').sessions[0];
 assert.equal(r.builds.length, 0); assert.equal(r.laps.length, 0);
 assert.equal(r.exit, false);assert.ok(r.findings.some(f => f.id === 'excerpt'));
});
test('ToF same markers, invalid records, unknown schemas', () => {
 const r = analyzeLog('[PARK_DIAG] v=2 type=sample s0=1,20,100,100,1,2,1,1\n[PARK_DIAG] v=2 type=sample s0=same\n[PARK_DIAG] v=2 type=sample s0=2,50,-1,9999,1,2,0,0\n[PARK_DIAG] v=3 type=sample s0=3,60,100,100,1,2,1,1').sessions[0];
 assert.equal(r.health.tof.s0.records, 2);assert.equal(r.health.tof.s0.invalid, 1);
 assert.equal(r.health.tof.s0.maxAge, 50);assert.equal(r.health.malformed, 1);
});
test('latest parking outcome and observation holds', () => {
 const r = analyzeLog('[FINAL PARK RESULT] contained=yes stopped=yes\n[FINAL PARK ABORT] later_failure\n[DISCOVERY_TRACE] reason=hold_expired\n[PARK_DIAG] type=correction delta=3,4,-2').sessions[0];
 assert.equal(r.parking, 'Parkabbruch protokolliert');
 assert.ok(r.findings.some(f => f.id === 'discovery-expired'));
 assert.equal(r.health.maxCorrectionMm, 5);assert.equal(r.health.maxCorrectionDeg, 2);
});
test('connector schema, malformed poses and finite SVG', () => {
 const r = analyzeLog('[CONNECTOR_TRACK] t=10 x=2 y=3 h=4 tx=5 ty=6\n[LATER_TRACK] t=20 lap=2 pose=NaN,0,0').sessions[0];
 assert.equal(r.tracks.CONNECTOR_TRACK[0].x, 2);assert.equal(r.health.malformed, 1);
 const svg = plotSvg(r.tracks.CONNECTOR_TRACK);assert.ok(svg.includes('<svg'));assert.ok(!/NaN|Infinity/.test(svg));
});
test('time gaps and resets are not connected', () => {
 const svg = plotSvg([{x:0,y:0,t:10},{x:1,y:1,t:3000},{x:2,y:2,t:0}]);
 assert.equal((svg.match(/<polyline/g)||[]).length, 3);
});
test('untrusted log contents escaped, no remote resources', () => {
 assert.equal(escapeHtml('<img onerror="x">'), '&lt;img onerror=&quot;x&quot;&gt;');
 assert.ok(html.includes("connect-src 'none'"));
 assert.ok(!/<(?:script|link|img)[^>]+(?:src|href)=["']https?:/i.test(html));
});
test('limits and loop pauses remain distinct from sensor timeouts', () => {
 const r = analyzeLog('[GYRO] Main loop did not poll gyro for 1600ms; deferring sensor timeout check.\n[PARK_DIAG] v=2 type=truncated t=5 reason=limit\n*** WARNING: LOG BUFFER OVERFLOW ***').sessions[0];
 assert.equal(r.health.gyroTimeout, 0);assert.equal(r.health.loopPause, 1);assert.equal(r.health.overflow, 2);
});
test('all archived complete logs parse and render', () => {
 const folder = path.join(root, 'simulation/evidence/parking_exit_diagnostics');
 const logs = fs.readdirSync(folder).filter(n => /^\d{8}_log_\d+_(cw|ccw)\.txt$/.test(n));
 assert.ok(logs.length > 100);
 for(const name of logs){const r=analyzeLog(fixture(name),name);for(const s of r.sessions){for(const points of Object.values(s.tracks))assert.ok(!/NaN|Infinity/.test(plotSvg(points)), name);assert.ok(!/NaN|Infinity/.test(wholeRunSvg(s)),name);}}
 console.log('Archived logs verified: '+logs.length);
});
test('combined view matches CW/CCW geometry and accepted maps',()=>{
 assert.equal(seatPosition(5,1).x,500);assert.equal(seatPosition(5,1).y,-900);
 assert.equal(seatPosition(11,-1).x,-1100);assert.equal(seatPosition(11,-1).y,500);
 const a=analyzeLog(fixture('20261006_log_476_ccw.txt')).sessions[0];
 assert.equal(runGeometry(a).pillars.length,6);assert.ok(wholeRunSvg(a).includes('G17'));
 const b=analyzeLog(fixture('20261006_log_481_ccw.txt')).sessions[0];
 assert.equal(runGeometry(b).pillars.length,1);assert.equal(runGeometry(b).pillars[0].seat,5);
});
test('combined view isolates runs, rejects sightings and keeps local rear poses out',()=>{
 const r=analyzeLog('[OC] New obstacle run\n[PARK_DIAG_CONFIG] turn=1\n[PARK_DIAG] type=sample state=rear_drive pose=0,0,0\n[RED SEAT] decision=outside_snap nearest/error=9/151 accepted=-1\n[MAP] Confirmed S0 station=2 side=LEFT color=GREEN\n[OC] New obstacle run\n[PARK_DIAG_CONFIG] turn=-1');
 assert.equal(runGeometry(r.sessions[0]).pillars.length,1);assert.equal(runGeometry(r.sessions[0]).series.length,0);
 assert.equal(runGeometry(r.sessions[1]).pillars.length,0);
 const unknown=analyzeLog('[MAP] Confirmed S0 station=2 side=LEFT color=GREEN').sessions[0];
 assert.equal(runGeometry(unknown).pillars.length,0);
});
test('whole-run corrections and time gaps are not solid travel connections',()=>{
 const r=analyzeLog('[PARK_DIAG_CONFIG] turn=1\n[PARK_DIAG] type=sample state=segment_drive t=10 pose=300,-1300,0\n[PARK_DIAG] type=correction t=15 before=300,-1300,0 after=310,-1300,0\n[PARK_DIAG] type=sample state=segment_drive t=20 pose=310,-1300,0\n[PARK_DIAG] type=sample state=segment_drive t=5000 pose=320,-1300,0').sessions[0];
 const g=runGeometry(r);assert.equal(g.corrections.length,1);
 const svg=wholeRunSvg(r);assert.equal((svg.match(/stroke="#1769aa" stroke-width="2.3"/g)||[]).length,6);
 assert.ok(svg.includes('stroke-dasharray="2 4"'));
});
test('field layer toggles independently and respects logged direction',()=>{
 const r=analyzeLog(fixture('20261006_log_476_ccw.txt')).sessions[0];
 const shown=wholeRunSvg(r),hidden=wholeRunSvg(r,{fieldLayer:false});
 assert.ok(shown.includes('data-layer="field"'));assert.ok(shown.includes('Parkbucht'));
 assert.equal((shown.match(/data-station=/g)||[]).length,24);
 assert.ok(shown.includes('data-direction="CCW"'));assert.ok(!shown.includes('data-direction="CW"'));
 assert.ok(!hidden.includes('data-layer="field"'));assert.ok(hidden.includes('data-seat="17"'));
 const cw=wholeRunSvg(analyzeLog(fixture('20261007_log_507_cw.txt')).sessions[0]);
 assert.ok(cw.includes('data-direction="CW"'));assert.ok(!cw.includes('data-direction="CCW"'));
 const unknown=wholeRunSvg(analyzeLog('[CONNECTOR_TRACK] t=10 x=2 y=3 h=4').sessions[0]);
 assert.ok(!unknown.includes('data-direction='));
});
console.log(`${checks} regression checks passed.`);
