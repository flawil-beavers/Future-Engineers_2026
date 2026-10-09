function hasPlan(svg){return /<polyline\b[^>]*stroke="#303640"[^>]*stroke-dasharray="8 6"/.test(svg);}
// Behavioral integration checks with a DOM adapter; no browser dependencies.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const {webcrypto} = require('node:crypto');
const html = fs.readFileSync(path.join(__dirname, 'Robot Run Inspector.html'), 'utf8');
const elements = new Map();
class Element {
 constructor() {this.value='';this.innerHTML='';this.textContent='';this.handlers={};this.hidden=false;}
 addEventListener(type, handler) {this.handlers[type]=handler;}
 replaceChildren(...options) {this.value=options[0]?.value||'';}
 scrollIntoView() {}
 async fire(type, event={}) {return this.handlers[type](event);}
}
for(const match of html.matchAll(/id="([^"]+)"/g))elements.set(match[1],new Element());
const downloads = [];
async function savePreview(filename) {
 const directory=process.env.INSPECTOR_EXPORT_DIR;
 if(directory){fs.mkdirSync(directory,{recursive:true});fs.writeFileSync(path.join(directory,filename),await downloads.at(-1).blob.text());}
}
const urls = new Map();
const box = {
 document:{getElementById:id=>elements.get(id),querySelector:()=>({textContent:html.match(/<style>([\s\S]*?)<\/style>/)[1]}),createElement:()=>({click(){downloads.push({name:this.download,blob:urls.get(this.href)});}})},
 Option:class {constructor(text,value){this.text=text;this.value=value;}},
 TextDecoder,Blob,crypto:webcrypto,URL:{createObjectURL(blob){const key='blob:'+urls.size;urls.set(key,blob);return key;},revokeObjectURL(){}},
 setTimeout:fn=>fn(),window:{print(){}},console
};
vm.runInNewContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],box);
async function open(text,name='run.txt'){
 const bytes=new TextEncoder().encode(text);
 await elements.get('file').fire('change',{target:{files:[{name,size:bytes.length,arrayBuffer:async()=>bytes.buffer}]}});
}
(async()=>{
 await open('[OC] New obstacle run\n[GYRO] Sensor report timeout; restarting SPI/SH2...\n[FINAL PARK ABORT] <img onerror="evil">\n[OC] New obstacle run\n[FINAL PARK RESULT] contained=yes stopped=yes');
 assert.equal(elements.get('workspace').hidden,false);
 assert.equal(elements.get('planVersionControl').hidden,true);
 assert.ok(elements.get('cards').innerHTML.includes('Unbekannt'));
 assert.ok(elements.get('summary').innerHTML.includes('Parkabbruch protokolliert'));
 assert.ok(!elements.get('summary').innerHTML.includes('<img'));
 assert.ok(elements.get('summary').innerHTML.includes('&lt;img'));
 elements.get('search').value='GYRO';await elements.get('search').fire('input');
 assert.ok(elements.get('rawInfo').textContent.includes('1 von 1'));
 elements.get('line').value='3';await elements.get('jump').fire('click');
 assert.ok(elements.get('raw').innerHTML.includes('selected'));
 elements.get('notes').value='<script>user observation</script>';await elements.get('notes').fire('input');
 await elements.get('export').fire('click');
 const exported=await downloads.at(-1).blob.text();
 assert.ok(exported.includes('&lt;script&gt;user observation&lt;/script&gt;'));
 assert.ok(!/<script\b/i.test(exported));assert.ok(exported.includes('SHA-256'));
 elements.get('session').value='1';await elements.get('session').fire('change');
 assert.ok(elements.get('summary').innerHTML.includes('enthalten und gestoppt'));
 assert.equal(elements.get('notes').value,'');
 await elements.get('json').fire('click');
 const json=JSON.parse(await downloads.at(-1).blob.text());
 assert.equal(json.sessions.length,2);assert.equal(json.sessions[0].physicalObservation,'<script>user observation</script>');assert.equal(json.sha256.length,64);
 // Restore user observations only against the currently opened original hash.
 elements.get('notes').value='changed';await elements.get('notes').fire('input');
 await elements.get('noteFile').fire('change',{target:{files:[{size:100,text:async()=>JSON.stringify(json)}]}});
 assert.equal(elements.get('notes').value,'');
 elements.get('session').value='0';await elements.get('session').fire('change');
 assert.equal(elements.get('notes').value,'<script>user observation</script>');
 const mismatched={...json,sha256:'0'.repeat(64)};
 await elements.get('noteFile').fire('change',{target:{files:[{size:100,text:async()=>JSON.stringify(mismatched)}]}});
 assert.ok(elements.get('noteStatus').textContent.includes('SHA-256'));
 assert.equal(elements.get('notes').value,'<script>user observation</script>');
 await open('[OC] New obstacle run\n[PARK_DIAG] v=2 type=sample t=10 pose=3,4,1 nominal=0,0,359');
 assert.ok(elements.get('deviations').innerHTML.includes('5.0 mm'));
 assert.ok(elements.get('plot').innerHTML.includes('Violett'));
 await elements.get('csv').fire('click');
 const csv=await downloads.at(-1).blob.text();assert.ok(csv.includes('distance_mm'));assert.ok(csv.includes('"5"'));
 await elements.get('export').fire('click');const comparisonReport=await downloads.at(-1).blob.text();assert.ok(comparisonReport.includes('5.0 mm'));assert.ok(comparisonReport.includes('Violett'));
 await elements.get('workspace').fire('click',{target:{closest:()=>({dataset:{line:'2'}})}});assert.ok(elements.get('raw').innerHTML.includes('selected'));
 await elements.get('workspace').fire('keydown',{key:'Enter',preventDefault(){},target:{closest:()=>({dataset:{line:'2'}})}});
 await open('binary\0data');assert.ok(elements.get('loadStatus').textContent.includes('Binärdaten'));
 await open('');assert.ok(elements.get('summary').innerHTML.includes('Kein Endergebnis'));
 console.log('PASS app import, search, line context, session isolation, HTML/JSON exports, hash, nominal comparison, CSV, point context, hash-bound notes restore and binary/empty input checks.');
 const fixture=fs.readFileSync(path.join(__dirname,'../../simulation/evidence/parking_exit_diagnostics/20261006_log_476_ccw.txt'),'utf8');
 await open(fixture,'476.txt');
 assert.equal(elements.get('planVersionControl').hidden,true);
 assert.ok(elements.get('wholePlot').innerHTML.includes('>G17</text>'));
 assert.ok(elements.get('wholePlot').innerHTML.includes('data-layer="official-mat"'));
 await elements.get('saveWholeSvg').fire('click');
 assert.ok(downloads.at(-1).name.endsWith('-run.svg'));
 assert.ok((await downloads.at(-1).blob.text()).includes('data-seat="17"'));
 await savePreview('run.svg');
 await elements.get('export').fire('click');
 assert.ok((await downloads.at(-1).blob.text()).includes('Gesamtlauf mit Pfosten'));
 await savePreview('report.html');
 assert.ok((await downloads.at(-1).blob.text()).includes('data:image/svg+xml;base64,'));
 assert.ok(!elements.has('fieldLayer'));
 await open('[OC] New obstacle run\n[PARK_DIAG_CONFIG] turn=1\n[MAP] Confirmed S0 station=2 side=LEFT color=GREEN\n[OC] New obstacle run\n[PARK_DIAG_CONFIG] turn=1');
 assert.ok(elements.get('wholePlot').innerHTML.includes('>G5</text>'));
 elements.get('session').value='1';await elements.get('session').fire('change');
 assert.ok(!elements.get('wholePlot').innerHTML.includes('>G5</text>'));assert.ok(!elements.get('saveWholeSvg').disabled);assert.ok(elements.get('wholePlot').innerHTML.includes('Keine Feldpositionsdaten'));
 await open('[RUN_ROUTE] v=1 t=10 route=1 base=0 kind=connector count=2 closed=0\n[RUN_ROUTE_POINT] v=1 route=1 index=0 pose=0,-1200,0\n[RUN_ROUTE_POINT] v=1 route=1 index=1 pose=200,-1200,0\n[RUN_ROUTE_END] v=1 route=1\n[RUN_POSE] v=1 t=250 phase=connector lap=1 route=1 frame=1 space=field pose=0,-1200,0');
 elements.get('planVersion').value='1';await elements.get('planVersion').fire('change');
 await elements.get('export').fire('click');
 assert.ok(hasPlan((await downloads.at(-1).blob.text())));
 assert.ok((await downloads.at(-1).blob.text()).includes('data-layer="official-mat"'));

 elements.get('roundFilter').value='1';await elements.get('roundFilter').fire('change');
 assert.equal(elements.get('planVersion').value,'active');
 elements.get('fieldBackground').value='hide';await elements.get('fieldBackground').fire('change');
 await elements.get('saveWholeSvg').fire('click');
 const filteredSvg=await downloads.at(-1).blob.text();
 assert.ok(filteredSvg.includes('data-round="1" data-layers="both" data-field="hide"'));assert.ok(hasPlan(filteredSvg));assert.ok(!filteredSvg.includes('data-layer="official-mat"'));
 await elements.get('export').fire('click');
 assert.ok((await downloads.at(-1).blob.text()).includes('data-round="1" data-layers="both" data-field="hide"'));
 elements.get('fieldBackground').value='show';await elements.get('fieldBackground').fire('change');
 assert.ok(hasPlan(elements.get('wholePlot').innerHTML));
 elements.get('trajectoryLayers').value='driven';await elements.get('trajectoryLayers').fire('change');
 assert.ok(!hasPlan(elements.get('wholePlot').innerHTML));
 assert.ok(elements.get('wholePlot').innerHTML.includes('data-layer="official-mat"'));
 elements.get('fieldBackground').value='hide';await elements.get('fieldBackground').fire('change');
 elements.get('trajectoryLayers').value='plan';await elements.get('trajectoryLayers').fire('change');
 assert.ok(hasPlan(elements.get('wholePlot').innerHTML));
 assert.ok(!elements.get('wholePlot').innerHTML.includes('data-layer="official-mat"'));
 await elements.get('saveWholeSvg').fire('click');
 assert.ok((await downloads.at(-1).blob.text()).includes('data-layers="plan" data-field="hide"'));
 await elements.get('export').fire('click');
 assert.ok((await downloads.at(-1).blob.text()).includes('data-layers="plan" data-field="hide"'));
 await open('[OC] New obstacle run\n[OC] New obstacle run');
 assert.equal(elements.get('roundFilter').value,'all');assert.equal(elements.get('fieldBackground').value,'show');assert.equal(elements.get('trajectoryLayers').value,'both');assert.equal(elements.get('planVersionControl').hidden,true);
 elements.get('roundFilter').value='2';elements.get('fieldBackground').value='hide';
 elements.get('session').value='1';await elements.get('session').fire('change');
 assert.equal(elements.get('roundFilter').value,'all');assert.equal(elements.get('fieldBackground').value,'show');assert.equal(elements.get('trajectoryLayers').value,'both');assert.equal(elements.get('planVersionControl').hidden,true);
 await open('[RUN_ROUTE] v=1 t=1 route=1 base=0 kind=connector count=0 closed=0\n[RUN_ROUTE_END] v=1 route=1\n[RUN_ROUTE] v=1 t=2 route=2 base=0 kind=learned count=0 closed=1\n[RUN_ROUTE_END] v=1 route=2');
 assert.equal(elements.get('planVersionControl').hidden,false);
 console.log('PASS app import, search, line context, session isolation, HTML/JSON exports, hash and binary/empty input checks.');
})().catch(e=>{console.error(e);process.exitCode=1;});
