// Read-only verification of the authorized S1 browser capture; no browser storage access.
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto'),assert=require('node:assert/strict');
const C=require('../frontend/project-core.js');
const root=path.resolve(__dirname,'..'),dir=path.join(root,'artifacts/s1-browser-20260911');
const read=p=>JSON.parse(fs.readFileSync(p,'utf8'));
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const canonical=o=>o===null||typeof o!=='object'?JSON.stringify(o):Array.isArray(o)?'['+o.map(canonical).join(',')+']':'{'+Object.keys(o).sort().map(k=>JSON.stringify(k)+':'+canonical(o[k])).join(',')+'}';
const baseline=read(path.join(dir,'baseline-edge.json'));
const paths=fs.readdirSync(path.join(root,'data/projects/k2/revisions')).filter(p=>/^\d{8}\.json$/.test(p)).sort();
const checks=paths.map(p=>{const doc=read(path.join(root,'data/projects/k2/revisions',p));
 assert.deepEqual(doc.annotations.boxes,baseline.boxes,'Region content changed in '+p);
 for(const k of ['scene_id','source_hashes','T_lcc_from_las','coordinate_frame'])assert.deepEqual(doc.annotations[k],baseline[k]);
 assert.equal(doc.real_robot_commands,false);for(const o of doc.objects){assert.equal(o.task_qualification,false);assert.equal(o.reviews.length,0);}
 return {revision:doc.revision,regions:doc.annotations.boxes.length,region_sha256:hash(canonical(doc.annotations.boxes)),objects:doc.objects.length,regions_exactly_match_browser_baseline:true};
});
const latest=read(path.join(root,'data/projects/k2/revisions',paths.at(-1)));
const blob=id=>{const bytes=fs.readFileSync(path.join(root,'data/projects/k2/blobs',id));assert.equal(hash(bytes),id);return bytes;};
const evidence=[];
for(const id of new Set(latest.objects.flatMap(o=>o.evidence_ids))){
 const m=JSON.parse(blob(id)),depth=blob(m.depth),valid=blob(m.validity);blob(m.rgb);if(m.result)blob(m.result);
 assert.equal(depth.length,m.width*m.height*4);assert.equal(valid.length,m.width*m.height);
 let n=0;for(let i=0;i<valid.length;i++){const d=depth.readFloatLE(i*4),expected=Number(Number.isFinite(d)&&d>=0&&d<1);assert.equal(valid[i],expected);n+=expected;}
 assert.equal(n,m.valid_pixels);
 const pixels=[[549,192],[549,191]],samples=pixels.map(([x,y])=>{const d=depth.readFloatLE((y*m.width+x)*4),p=C.unproject(m,x,y,d),q=p&&C.projectPoint(m,p);return {pixel:[x,y],depth:d,world:p,roundtrip_pixel_error:q?Math.hypot(q[0]-x-.5,q[1]-y-.5):null};});
 const o=latest.objects.find(o=>o.evidence_ids.includes(id)),regions=latest.annotations.boxes.filter(b=>o.region_ids.includes(b.id));let total=0,inFrame=0,depthMatched=0;
 for(const b of regions){const g=b.geometry;if(g?.type!=='surface-voxels')continue;for(const cell of g.cells){total++;const p=cell.map((v,k)=>g.origin[k]+(v+.5)*g.cell),q=C.projectPoint(m,p);if(!q||q[0]<0||q[1]<0||q[0]>=m.width||q[1]>=m.height)continue;inFrame++;const d=depth.readFloatLE((Math.floor(q[1])*m.width+Math.floor(q[0]))*4);if(Number.isFinite(d)&&d>=0&&d<1&&Math.abs(m.camera.near+d*(m.camera.far-m.camera.near)-q[2])<=Math.max(.06,g.cell*1.7))depthMatched++;}}
 evidence.push({id,kind:m.kind,width:m.width,height:m.height,valid_pixels:n,invalid_pixels:valid.length-n,source:m.depth_source,samples,support:{total,in_frame:inFrame,depth_matched:depthMatched,not_an_accuracy_or_coverage_metric:true}});
}
const report={at:new Date().toISOString(),scope:'one real browser object; stored GPU frame integrity and exact region preservation',checks,evidence,semantic_accuracy_accepted:false,robot_qualification:false};
const out=path.join(dir,'capture-check-'+Date.now()+'.json');fs.writeFileSync(out,JSON.stringify(report,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify({...report,report_path:out},null,2));
