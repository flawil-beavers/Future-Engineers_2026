function hasPlan(svg){return /<polyline\b[^>]*stroke="#303640"[^>]*stroke-dasharray="8 6"/.test(svg);}
// Node.js regression tests for the parser delivered inside the offline app.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const root = path.resolve(__dirname, '../..');
const html = fs.readFileSync(path.join(__dirname, 'Robot Run Inspector.html'), 'utf8');
const box = {module: {exports: {}}};
vm.runInNewContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], box);
const {analyzeLog, plotSvg, escapeHtml, seatPosition, runGeometry, wholeRunSvg, runTimingHtml} = box.module.exports;
let checks = 0;
function test(label, fn) {fn(); checks++; console.log('PASS ' + label);}
function markup(svg){return svg.replace(/data:image\/svg\+xml;base64,[A-Za-z0-9+/=]+/g,'embedded-mat');}
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
 for(const name of logs){const r=analyzeLog(fixture(name),name);for(const s of r.sessions){for(const points of Object.values(s.tracks))assert.ok(!/NaN|Infinity/.test(plotSvg(points)), name);assert.ok(!/NaN|Infinity/.test(markup(wholeRunSvg(s))),name);}}
 console.log('Archived logs verified: '+logs.length);
});
test('combined view matches CW/CCW geometry and accepted maps',()=>{
 assert.equal(seatPosition(5,1).x,500);assert.equal(seatPosition(5,1).y,-900);
 assert.equal(seatPosition(11,-1).x,-1100);assert.equal(seatPosition(11,-1).y,500);
 const a=analyzeLog(fixture('20261006_log_476_ccw.txt')).sessions[0];
 assert.equal(runGeometry(a).pillars.length,6);assert.ok(wholeRunSvg(a).includes('>G17</text>'));
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
 const svg=wholeRunSvg(r);assert.equal((svg.match(/stroke="#1769aa" stroke-width="3.2"/g)||[]).length,6);
 assert.ok(svg.includes('stroke-dasharray="2 4"'));
});
test('official mat is shared by both panels with a fixed coordinate transform',()=>{
 const ccw=wholeRunSvg(analyzeLog(fixture('20261006_log_476_ccw.txt')).sessions[0]);
 const cw=wholeRunSvg(analyzeLog(fixture('20261007_log_507_cw.txt')).sessions[0]);
 for(const svg of [ccw,cw]){
  assert.equal((svg.match(/data-layer="official-mat"/g)||[]).length,2);
  assert.equal((svg.match(/data:image\/svg\+xml;base64,/g)||[]).length,1);
  assert.ok(svg.includes('transform="translate(45 45) scale(0.15625)"'));
  assert.ok(!svg.includes('data-direction='));assert.ok(!svg.includes('data-station='));
  assert.ok(svg.includes('data-layer="nominal-walls"'));
 }
 assert.ok(!html.includes('fieldLayer'));assert.ok(!html.includes('Spielfeld anzeigen'));
});
test('mat remains visible for local-only and missing trajectories without inventing objects',()=>{
 const r=analyzeLog('[PARK_DIAG] type=sample state=rear_drive t=10 pose=0,0,0').sessions[0];
 const svg=wholeRunSvg(r);assert.ok(svg.includes('data-layer="official-mat"'));
 assert.ok(svg.includes('Keine Feldpositionsdaten'));assert.equal(runGeometry(r).series.length,0);
 assert.ok(!svg.includes('data-layer="parking-barriers"'));assert.ok(!svg.includes('data-seat='));
});
test('parking barriers require both placement and gap from this session',()=>{
 const gap='[PARK EXIT] Prototype footprint length/front/rear/width_mm=165/125/40/135 gap_mm=247.5';
 const placement='[PARK FIELD START] turn=CW fixed_line_x=500 rear_axle_x_y_heading=393/-1360/180';
 for(const incomplete of [gap,placement])assert.ok(!wholeRunSvg(analyzeLog(incomplete).sessions[0]).includes('data-layer="parking-barriers"'));
 const r=analyzeLog(gap+'\n'+placement).sessions[0];assert.equal(runGeometry(r).gap,247.5);assert.equal(runGeometry(r).fixedLine,500);
 assert.ok(wholeRunSvg(r).includes('data-fixed-line-mm="500" data-gap-mm="247.5"'));
});
const telemetryLog=`[OC] New obstacle run
[PARK_DIAG_CONFIG] turn=1
[RUN_START] v=1 t=1000 period_ms=250
[RUN_ROUTE] v=1 t=1000 route=1 base=0 kind=connector count=2 closed=0
[RUN_ROUTE_POINT] v=1 route=1 index=0 pose=0,-1200,0
[RUN_ROUTE_POINT] v=1 route=1 index=1 pose=100,-1200,0
[RUN_ROUTE_END] v=1 route=1
[RUN_POSE] v=1 t=1000 elapsed_ms=0 phase=connector lap=1 route=1 frame=0 space=local pose=0,0,0
[RUN_POSE] v=1 t=1250 elapsed_ms=250 phase=connector lap=1 route=1 frame=1 space=field pose=0,-1200,0
[RUN_ROUTE] v=1 t=1300 route=2 base=1 kind=connector count=2 closed=0
[RUN_ROUTE_POINT] v=1 route=2 index=1 pose=200,-1200,0
[RUN_ROUTE_END] v=1 route=2
[RUN_POSE] v=1 t=1500 elapsed_ms=500 phase=connector lap=1 route=2 frame=1 space=field pose=200,-1200,0
[RUN_LAP] v=1 t=7000 elapsed_ms=6000 lap=1 lap_ms=6000
[RUN_END] v=1 t=8000 elapsed_ms=7000 outcome=completed reason=final_parking_stop truncated=0`;
test('new telemetry separates local coordinates and reconstructs changed route points',()=>{
 const r=analyzeLog(telemetryLog).sessions[0],t=r.telemetry;
 assert.equal(t.routes.length,2);assert.equal(t.routes[1].points[0].x,0);assert.equal(t.routes[1].points[1].x,200);
 assert.equal(runGeometry(r).series[0].points.length,2);
 assert.ok(Object.keys(r.tracks).some(k=>k.includes('local')));
 assert.ok(hasPlan(wholeRunSvg(r,{route:'1'})));
 assert.ok(!hasPlan(wholeRunSvg(r,{route:'none'})));
});
test('completion and lap durations include the entire elapsed interval',()=>{
 const r=analyzeLog(telemetryLog).sessions[0];
 assert.ok(runTimingHtml(r).includes('7.000 s'));assert.ok(runTimingHtml(r).includes('6.000 s'));
 assert.ok(runTimingHtml(r).includes('Gesamtzeit'));
 const partial=analyzeLog(telemetryLog.split('[RUN_END]')[0]).sessions[0];
 assert.ok(runTimingHtml(partial).includes('unbekannt'));assert.ok(!runTimingHtml(partial).includes('Gesamtzeit'));
 const stopped=analyzeLog(telemetryLog.replace('outcome=completed','outcome=stopped')).sessions[0];
 assert.ok(runTimingHtml(stopped).includes('unbekannt'));assert.ok(runTimingHtml(stopped).includes('stopped'));
});
test('missing route bases and unfinished route transactions are never drawn',()=>{
 const missing=analyzeLog(telemetryLog.replace('[RUN_ROUTE_END] v=1 route=1','')).sessions[0];
 assert.equal(missing.telemetry.routes.length,0);
 const incomplete=analyzeLog(telemetryLog.replace('[RUN_ROUTE_END] v=1 route=2','')).sessions[0];
 assert.equal(incomplete.telemetry.routes.length,1);
 assert.ok(!hasPlan(wholeRunSvg(incomplete)));
});
test('truncated telemetry, old logs and excerpts preserve unknown completion',()=>{
 const truncated=analyzeLog(telemetryLog.replace('truncated=0','truncated=1')).sessions[0];
 assert.ok(runTimingHtml(truncated).includes('Telemetrie begrenzt'));
 const old=analyzeLog('[CONNECTOR_TRACK] t=1000 x=1 y=2 h=3\n[CONNECTOR_TRACK] t=2500 x=2 y=3 h=4').sessions[0];
 assert.ok(runTimingHtml(old).includes('1.500 s'));assert.ok(runTimingHtml(old).includes('unbekannt'));
 const excerpt=analyzeLog(telemetryLog,'log_excerpt.txt').sessions[0];
 assert.ok(!runTimingHtml(excerpt).includes('Gesamtzeit'));
 const footer=analyzeLog('[RUN_END] v=1 t=8000 elapsed_ms=7000 outcome=completed').sessions[0];
 assert.ok(!runTimingHtml(footer).includes('Gesamtzeit'));
});
test('malformed new pose metadata is safely omitted',()=>{
 const r=analyzeLog('[RUN_POSE] v=1 t=1000 pose=1,2,3\n[RUN_POSE] v=1 t=1250 phase=driving lap=1 route=1 frame=NaN space=field pose=1,2,3').sessions[0];
 assert.equal(r.telemetry.poses.length,0);assert.doesNotThrow(()=>wholeRunSvg(r));
});
test('log 476 correction uses exact recorded endpoints in field coordinates',()=>{
 const r=analyzeLog(fixture('20261006_log_476_ccw.txt')).sessions[0],g=runGeometry(r);
 assert.equal(g.corrections.length,1);
 assert.equal(g.corrections[0].before.join(','),'380.3,-1184.9,358.01');
 assert.equal(g.corrections[0].after.join(','),'357.6,-1199.7,358.01');
 const groups=[...wholeRunSvg(r).matchAll(/<g data-correction-line="\d+">([\s\S]*?)<\/g>/g)];
 assert.equal(groups.length,2);
 assert.ok(groups[0][1].includes('points="354.42,480.14 350.88,482.45"'));
 assert.ok(groups.every(g=>g[1].includes('stroke="#a34ab5" stroke-width="2.3" stroke-dasharray="2 4"')));
});
const pose=(t,frame,space='field')=>`[RUN_POSE] v=1 t=${t} phase=driving lap=1 route=1 frame=${frame} space=${space} pose=300,-1300,0`;
const correction='[RUN_EVENT] v=1 t=15 kind=correction frame=2 before=300,-1300,0 after=310,-1300,0';
const legacyCorrection='[PARK_DIAG] v=2 type=correction t=17 before=300,-1300,0 after=310,-1300,0';
test('local or unknown-frame corrections are excluded; rebases never become travel',()=>{
 const r=analyzeLog('[RUN_START] v=1 t=0\n'+pose(10,0,'local')+'\n'+correction+'\n[RUN_EVENT] v=1 t=18 kind=rebase frame=3 before=300,-1300,0 after=1000,1000,0\n'+pose(20,3)).sessions[0];
 assert.equal(r.telemetry.corrections[0].space,'local');assert.equal(runGeometry(r).corrections.length,0);
 assert.ok(!wholeRunSvg(r).includes('data-correction-line='));
 assert.equal(runGeometry(analyzeLog(correction).sessions[0]).corrections.length,0);
});
test('field correction events render without periodic poses and mixed streams deduplicate',()=>{
 const rebase='[RUN_EVENT] v=1 t=1 kind=rebase frame=1 before=0,0,0 after=300,-1300,0';
 assert.equal(runGeometry(analyzeLog(rebase+'\n'+correction).sessions[0]).corrections.length,1);
 const r=analyzeLog(pose(10,1)+'\n'+correction+'\n'+legacyCorrection+'\n'+pose(20,2)).sessions[0];
 assert.equal(runGeometry(r).corrections.length,1);
 assert.equal((wholeRunSvg(r).match(/data-correction-line=/g)||[]).length,2);
});
test('legacy correction survives missing new events and splits the new solid trajectory',()=>{
 const r=analyzeLog(pose(10,1)+'\n'+legacyCorrection+'\n'+pose(20,1)).sessions[0];
 assert.equal(runGeometry(r).corrections.length,1);
 const svg=wholeRunSvg(r);
 assert.equal((svg.match(/stroke="#087f8c" stroke-width="3.2"/g)||[]).length,4);
 assert.ok(/stroke="#a34ab5" stroke-width="3" stroke-dasharray="2 4"/.test(svg));
});

const roundLog=`[RUN_START] v=1 t=0
[RUN_ROUTE] v=1 t=1 route=1 base=0 kind=lap count=2 closed=0
[RUN_ROUTE_POINT] v=1 route=1 index=0 pose=0,-1200,0
[RUN_ROUTE_POINT] v=1 route=1 index=1 pose=200,-1200,0
[RUN_ROUTE_END] v=1 route=1
[RUN_ROUTE] v=1 t=2 route=2 base=0 kind=lap count=2 closed=0
[RUN_ROUTE_POINT] v=1 route=2 index=0 pose=500,1000,0
[RUN_ROUTE_POINT] v=1 route=2 index=1 pose=600,1000,0
[RUN_ROUTE_END] v=1 route=2
[RUN_POSE] v=1 t=250 phase=driving lap=1 route=1 frame=1 space=field pose=0,-1200,0
[RUN_POSE] v=1 t=500 phase=driving lap=2 route=1 frame=1 space=field pose=10,-1200,0
[RUN_EVENT] v=1 t=510 kind=correction frame=2 before=10,-1200,0 after=20,-1200,0
[RUN_POSE] v=1 t=750 phase=driving lap=2 route=1 frame=2 space=field pose=20,-1200,0
[RUN_POSE] v=1 t=1000 phase=driving lap=3 route=2 frame=2 space=field pose=500,1000,0
[RUN_POSE] v=1 t=1250 phase=final_park lap=3 route=0 frame=2 space=field pose=30,-1200,0
[RUN_EVENT] v=1 t=1260 kind=correction frame=3 before=30,-1200,0 after=40,-1200,0`;
test('round selection scopes trajectories, corrections and automatic route version',()=>{
 const r=analyzeLog(roundLog).sessions[0],svg=markup(wholeRunSvg(r,{round:'2'}));
 assert.ok(svg.includes('data-round="2"'));assert.ok(svg.includes('Plan v1'));
 assert.ok(svg.includes('stroke="#2960b2" stroke-width="3.2"'));
 assert.ok(!svg.includes('stroke="#b35197"'));assert.ok(!svg.includes('stroke="#713da7"'));
 assert.equal((svg.match(/data-correction-line=/g)||[]).length,2);
 assert.ok(wholeRunSvg(r,{round:'3'}).includes('Plan v2'));
 assert.ok(!hasPlan(wholeRunSvg(r,{round:'other'})));
 assert.ok(wholeRunSvg(r,{round:'2',route:'2'}).includes('Plan v2'));
});
test('hiding the FE artwork preserves physical geometry, plans and travel in both views',()=>{
 const r=analyzeLog(fixture('20261006_log_476_ccw.txt')).sessions[0];
 const svg=markup(wholeRunSvg(r,{round:'1',field:'hide'}));
 assert.ok(svg.includes('data-field="hide"'));assert.ok(!svg.includes('data-layer="official-mat"'));
 assert.equal((svg.match(/data-layer="nominal-walls"/g)||[]).length,2);
 assert.equal((svg.match(/data-layer="parking-barriers"/g)||[]).length,2);
 assert.ok(svg.includes('data-seat="17"'));assert.ok(hasPlan(svg));
 assert.ok(svg.includes('stroke="#cc7a12"'));assert.ok(svg.includes('FE-Spielfeld ausgeblendet'));
 assert.ok(wholeRunSvg(r,{field:'show'}).includes('data-layer="official-mat"'));
});
test('filtering away intermediate phases never creates a false connecting line',()=>{
 const p=(t,phase,x)=>`[RUN_POSE] v=1 t=${t} phase=${phase} lap=2 route=1 frame=1 space=field pose=${x},-1200,0`;
 const r=analyzeLog([p(250,'driving',0),p(500,'final_park',10),p(750,'driving',20)].join('\n')).sessions[0];
 const svg=markup(wholeRunSvg(r,{round:'2'}));
 const traces=[...svg.matchAll(/<polyline points="([^"]+)"[^>]*stroke="#2960b2"/g)];
 assert.equal(traces.length,4);assert.ok(traces.every(m=>!m[1].includes(' ')));
 const missing=wholeRunSvg(r,{round:'1'});assert.ok(missing.includes('Keine Feldpositionsdaten in Auswahl'));assert.ok(!hasPlan(missing));
});
test('legacy rounds retain only recorded points and never borrow the connector plan',()=>{
 const r=analyzeLog(fixture('20261006_log_476_ccw.txt')).sessions[0];
 const svg=markup(wholeRunSvg(r,{round:'2'}));
 assert.ok(svg.includes('stroke="#2960b2"'));assert.ok(!svg.includes('stroke="#b35197"'));
 assert.ok(!svg.includes('stroke="#1769aa"'));assert.ok(!hasPlan(svg));
 assert.ok(hasPlan(wholeRunSvg(r,{round:'1'})));
 assert.ok(!hasPlan(wholeRunSvg(r,{round:'1',route:'none'})));
});

console.log(`${checks} regression checks passed.`);
