// Read-only analysis of real archived evidence. No model calls or label changes.
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto'),assert=require('node:assert/strict');
const P=require('../frontend/project-core.js');
const root=path.resolve(__dirname,'..'),store=path.join(root,'data/projects/k2');
const read=p=>JSON.parse(fs.readFileSync(p,'utf8'));
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const blob=id=>{const bytes=fs.readFileSync(path.join(store,'blobs',id));assert.equal(hash(bytes),id);return bytes;};
const revisions=fs.readdirSync(path.join(store,'revisions')).filter(n=>/^\d{8}\.json$/.test(n)).sort();
const baseline=read(path.join(store,'revisions','00000006.json')),head=read(path.join(store,'revisions',revisions.at(-1)));
assert.deepEqual(head.annotations,baseline.annotations,'Evidence-only check requires exactly unchanged region export');
assert.equal(head.real_robot_commands,false);
const checks=[];
for(const object of head.objects){
 assert.equal(object.reviews.length,0,'This bounded capture must not invent human review');assert.equal(object.task_qualification,false);
 const regions=head.annotations.boxes.filter(b=>object.region_ids.includes(b.id));
 for(const id of object.evidence_ids){
  const m=JSON.parse(blob(id));if(m.kind!=='recognition')continue;
  assert.deepEqual(m.source_hashes,head.annotations.source_hashes);assert.equal(m.scene_id,head.annotations.scene_id);
  const rgb=blob(m.rgb),depth=blob(m.depth),valid=blob(m.validity),result=JSON.parse(blob(m.result));
  assert.equal(depth.length,m.width*m.height*4);assert.equal(valid.length,m.width*m.height);
  assert.equal(result.width,m.width);assert.equal(result.height,m.height);assert.equal(result.image_sha256,hash(rgb));
  let validCount=0;for(let i=0;i<valid.length;i++){const d=depth.readFloatLE(i*4),ok=Number(Number.isFinite(d)&&d>=0&&d<1);assert.equal(valid[i],ok);validCount+=ok;}assert.equal(validCount,m.valid_pixels);
  const projected=[];let total=0,inFrame=0;
  for(const b of regions){const g=b.geometry;if(g?.type!=='surface-voxels')continue;for(const c of g.cells){total++;const p=c.map((v,k)=>g.origin[k]+(v+.5)*g.cell),q=P.projectPoint(m,p);if(!q||q[0]<0||q[1]<0||q[0]>=m.width||q[1]>=m.height)continue;inFrame++;const x=Math.floor(q[0]),y=Math.floor(q[1]),d=depth.readFloatLE((y*m.width+x)*4);if(Number.isFinite(d)&&d>=0&&d<1&&Math.abs(m.camera.near+d*(m.camera.far-m.camera.near)-q[2])<=Math.max(.06,g.cell*1.7))projected.push({x,y,region_id:b.id});}}
  const masks=result.components.map((c,index)=>{const rows=new Map();let pixels=0;for(const [y,a,b]of c.runs){if(!rows.has(y))rows.set(y,[]);rows.get(y).push([a,b]);pixels+=b-a;}
   return {index,category:c.category,instance_id:c.instance_id,crop_id:c.crop_id,score:c.score,mask_pixels:pixels,support_hits:projected.filter(p=>(rows.get(p.y)||[]).some(([a,b])=>p.x>=a&&p.x<b)).length,bounds:c.runs.length?[Math.min(...c.runs.map(r=>r[1])),c.runs[0][0],Math.max(...c.runs.map(r=>r[2])),c.runs.at(-1)[0]+1]:null};});
  checks.push({object_id:object.object_id,regions:regions.map(b=>({id:b.id,category:b.category,sequence:b.sequence})),evidence_id:id,width:m.width,height:m.height,valid_depth_pixels:validCount,request:m.inference_request,mask_count:masks.length,support:{total,in_frame:inFrame,depth_consistent:projected.length},masks:masks.sort((a,b)=>b.support_hits-a.support_hits)});
 }
}
assert(checks.length,'No archived real recognition evidence yet');
const report={at:new Date().toISOString(),baseline_revision:6,head_revision:head.revision,region_content_exactly_unchanged:true,objects:head.objects.length,checks,interpretation:'Support/mask overlap is a diagnostic, NOT precision, recall, full-object coverage or identity proof. Old masks were not recovered.',human_review_written:false,semantic_accuracy_accepted:false,real_robot_commands:false};
const dir=path.join(root,'artifacts/s1-recognition-20260911');fs.mkdirSync(dir,{recursive:true});const out=path.join(dir,'recognition-check-'+Date.now()+'.json');fs.writeFileSync(out,JSON.stringify(report,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({report:out,...report,checks:checks.map(c=>({...c,masks:c.masks.slice(0,5)}))},null,2));
