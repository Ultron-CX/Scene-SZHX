(function(root){
 'use strict';
 const dimensions=['category','identity','coverage','background','evidence','coordinate'];
 const clone=x=>structuredClone(x);
 function evidenceIds(value){
  const found=new Set();
  function walk(v,depth=0){if(!v||typeof v!=='object'||depth>32)return;
   if(typeof v.evidence_id==='string'&&/^[0-9a-f]{64}$/.test(v.evidence_id))found.add(v.evidence_id);
   if(Array.isArray(v.evidence_ids))for(const id of v.evidence_ids)if(/^[0-9a-f]{64}$/.test(id))found.add(id);
   for(const [k,x]of Object.entries(v))if(!['geometry','cells','runs','positive','negative','flagged','min','max'].includes(k)&&x&&typeof x==='object')walk(x,depth+1);
  }walk(value);return [...found].sort();
 }
 function register(region,objects,id){
  if(!region||objects.some(o=>o.region_ids.includes(region.id)))throw Error('请先选择尚未登记到对象档案的区域');
  return {object_id:'obj-'+id,alias:'',region_ids:[region.id],evidence_ids:evidenceIds(region),reviews:[],
   identity_state:'candidate_not_human_confirmed',original_evidence:evidenceIds(region).length?'archived':'missing',
   created_at:new Date().toISOString(),task_qualification:false};
 }
 // Batch intake creates candidate records, never confirmed physical instances.
 function registrationPlan(regions,objects){
  const ids=new Set(),owners=new Set(objects.flatMap(o=>o.region_ids));
  for(const r of regions){if(!r?.id||ids.has(r.id))throw Error('区域 ID 缺失或重复，不能批量收录');ids.add(r.id);}
  const fresh=regions.filter(r=>!owners.has(r.id));
  return {regions:fresh,existing:regions.length-fresh.length,without_evidence:fresh.filter(r=>!evidenceIds(r).length).length,
   categories:Object.fromEntries([...new Set(fresh.map(r=>r.category))].map(c=>[c,fresh.filter(r=>r.category===c).length]))};
 }
 function registerBatch(regions,objects,newId){
  const plan=registrationPlan(regions,objects),used=new Set(objects.map(o=>o.object_id));
  return plan.regions.map(r=>{const o=register(r,[],newId());if(!/^obj-[0-9a-f-]{36}$/.test(o.object_id)||used.has(o.object_id))throw Error('新对象 ID 无效或重复，未收录');used.add(o.object_id);return o;});
 }
 function link(object,region,objects){
  if(!region||objects.some(o=>o.region_ids.includes(region.id)))throw Error('当前区域已有对象归属或未选中；不猜测合并身份');
  const next=clone(object);next.region_ids.push(region.id);next.evidence_ids=[...new Set([...next.evidence_ids,...evidenceIds(region)])].sort();return next;
 }
 // Exact local comparison, not a certificate. Server stamps a SHA-256 review basis.
 function basis(object,regions){return JSON.stringify({regions:object.region_ids.map(id=>regions.find(r=>r.id===id)||null),evidence_ids:object.evidence_ids,alias:object.alias,...(object.mask_corrections?.length?{mask_corrections:object.mask_corrections}:{})});}
 function reconcile(objects,regions){return objects.map(o=>({...clone(o),evidence_ids:[...new Set([...o.evidence_ids,...o.region_ids.flatMap(id=>evidenceIds(regions.find(r=>r.id===id)))])].sort(),task_qualification:false}));}
 function validate(document,meta,boxCore){
  if(document?.schema!=='scenesem.project.v1'||document.real_robot_commands!==false||!Number.isInteger(document.revision))throw Error('不是 SceneSem 项目版本');
  if(JSON.stringify(document.annotations?.T_lcc_from_las)!==JSON.stringify(meta.T_lcc_from_las))throw Error('项目坐标变换与当前批次不一致');
  const regions=boxCore.validate(document.annotations,meta),ids=new Set(),owners=new Set();
  if(!Array.isArray(document.objects))throw Error('对象档案缺失');
  for(const o of document.objects){if(!/^obj-[0-9a-f-]{36}$/.test(o.object_id)||ids.has(o.object_id)||!Array.isArray(o.region_ids)||!o.region_ids.length||!Array.isArray(o.evidence_ids)||!Array.isArray(o.reviews))throw Error('对象档案格式无效');ids.add(o.object_id);
   for(const r of o.region_ids){if(owners.has(r))throw Error('存在重复区域归属');owners.add(r);}
  }validateSpatial(document.spatial_context??null);validateBusiness(document.business_context??null);return regions;
 }
 function framePayload(frame){
  const depth=new Uint8Array(frame.depth.length*4),data=new DataView(depth.buffer);
  for(let i=0;i<frame.depth.length;i++)data.setFloat32(i*4,frame.depth[i],true);
  let binary='';for(let i=0;i<depth.length;i+=16384)binary+=String.fromCharCode(...depth.subarray(i,i+16384));
  const output={depth_f32le:btoa(binary)};
  for(const key of ['width','height','inverse','projection','camera_position','world_matrix','near','far','fov','horizontal_fov','pose_id'])output[key]=frame[key];
  return output;
 }
 function unproject(frame,x,y,depth){
  if(!Number.isFinite(depth)||depth<0||depth>=1)return null;
  const {near,far,inverse:m}=frame.camera,z=far*depth/(depth*(far-near)+near),v=[(x+.5)/frame.width*2-1,1-(y+.5)/frame.height*2,z*2-1,1];
  const p=[0,1,2,3].map(i=>m[i]*v[0]+m[i+4]*v[1]+m[i+8]*v[2]+m[i+12]);
  return Math.abs(p[3])>1e-8?p.slice(0,3).map(n=>n/p[3]):null;
 }
 function projectPoint(frame,p){const m=frame.camera.projection,v=[...p,1],q=[0,1,2,3].map(i=>m[i]*v[0]+m[i+4]*v[1]+m[i+8]*v[2]+m[i+12]);return q[3]>0?[(q[0]/q[3]+1)*frame.width/2,(1-q[1]/q[3])*frame.height/2,q[3]]:null;}
 function compareEvidence(frame,depth,regions,result){
  const total=regions.reduce((n,r)=>n+(r.geometry?.type==='surface-voxels'?r.geometry.cells.length:0),0),step=Math.max(1,Math.ceil(total/20000));
  const points=[],stats={total,checked:0,offscreen:0,invalid_depth:0,depth_disagreement:0,visible:0,sampled:step>1,unsupported_regions:regions.filter(r=>r.geometry?.type!=='surface-voxels').length};let serial=0;
  for(const r of regions){const g=r.geometry;if(g?.type!=='surface-voxels')continue;for(const cell of g.cells){if(serial++%step)continue;stats.checked++;const p=cell.map((v,k)=>g.origin[k]+(v+.5)*g.cell),q=projectPoint(frame,p);
   if(!q||q[0]<0||q[1]<0||q[0]>=frame.width||q[1]>=frame.height){stats.offscreen++;continue;}
   const x=Math.floor(q[0]),y=Math.floor(q[1]),d=depth.getFloat32((y*frame.width+x)*4,true);
   if(!Number.isFinite(d)||d<0||d>=1){stats.invalid_depth++;continue;}
   if(Math.abs(frame.camera.near+d*(frame.camera.far-frame.camera.near)-q[2])>Math.max(.06,g.cell*1.7)){stats.depth_disagreement++;continue;}
   points.push({x,y,world:p,region_id:r.id,cell:cell.slice()});
  }}stats.visible=points.length;
  const masks=(result?.components||[]).map((c,index)=>{const rows=new Map();let minX=frame.width,minY=frame.height,maxX=0,maxY=0,pixels=0;
   for(const [y,a,b]of c.runs){if(!rows.has(y))rows.set(y,[]);rows.get(y).push([a,b]);minX=Math.min(minX,a);maxX=Math.max(maxX,b);minY=Math.min(minY,y);maxY=Math.max(maxY,y+1);pixels+=b-a;}
   const contains=(x,y)=>(rows.get(y)||[]).some(([a,b])=>x>=a&&x<b),inside=[],outside=[];
   points.forEach((p,i)=>(contains(p.x,p.y)?inside:outside).push(i));return {index,category:c.category,pixels,bounds:pixels?[minX,minY,maxX,maxY]:null,contains,inside,outside};
  });return {stats,points,masks};
 }
 function maskCorrection(object,evidenceId,index){return (object.mask_corrections||[]).filter(c=>c.evidence_id===evidenceId&&c.component_index===index).at(-1)||null;}
 function maskCategory(object,evidenceId,index,raw){return maskCorrection(object,evidenceId,index)?.category||raw;}
 // Workflow hints only: never mutate records, infer quality or grant task eligibility.
 function reviewState(object,regions,savedBasis){
  const missing=object.region_ids.filter(id=>!regions.some(r=>r.id===id)),last=object.reviews.at(-1);
  const stale=!!(object.import_requires_review||(last&&(object.review_stale||savedBasis===undefined||savedBasis!==basis(object,regions))));
  const checks=Object.fromEntries(dimensions.map(d=>{const value=!missing.length&&!stale&&last?.checks?.[d];return [d,['pass','issue','unknown','pending'].includes(value)?value:'pending'];}));
  const values=Object.values(checks),status=missing.length?'missing':stale?'stale':!last?'unreviewed':values.includes('issue')?'issue':values.includes('pending')?'unreviewed':values.includes('unknown')?'unknown':'recorded';
  const evidence=[...new Set([...object.evidence_ids,...object.region_ids.flatMap(id=>evidenceIds(regions.find(r=>r.id===id)))])];
  return {object_id:object.object_id,status,checks,missing,stale,evidence_count:evidence.length,needs_work:status!=='recorded',task_qualification:false};
 }
 function issues(objects,regions,savedBases=new Map()){
  const items=[],owners=new Set(objects.flatMap(o=>o.region_ids));
  const add=(kind,source,o,r,detail)=>items.push({kind,source,object_id:o?.object_id||null,region_id:r||null,detail});
  for(const o of objects){const s=reviewState(o,regions,savedBases.get(o.object_id)),linked=regions.filter(r=>o.region_ids.includes(r.id)),r=linked[0]?.id;
   if(s.missing.length)add('missing','record_check',o,r,'区域关联失效，先修复归属');
   if(s.stale)add('stale','record_check',o,r,'旧审核需重新核查');
   if(!s.evidence_count)add('no_evidence','record_check',o,r,'无归档证据引用，不代表已检查全场证据');
   if(new Set(linked.map(r=>r.category)).size>1)add('category_conflict','record_check',o,r,'关联区域类别不一致，需判断类别或身份');
   for(const d of dimensions)if(s.checks[d]==='issue')add(d,'human_review',o,r,'人工记录问题：'+d);
   if(s.status==='unreviewed'||s.status==='unknown')add(s.status,'review_todo',o,r,s.status==='unknown'?'审核仍含未知':'尚未完成核查，不等于已经发现误标');
  }
  for(const r of regions)if(!owners.has(r.id))add('unregistered','record_check',null,r.id,'尚未收录，可在项目中批量收录');
  const order=['missing','category_conflict','background','coverage','category','identity','evidence','coordinate','stale','no_evidence','unknown','unreviewed','unregistered'];
  return items.sort((a,b)=>order.indexOf(a.kind)-order.indexOf(b.kind));
 }
 function triage(objects,regions,savedBases=new Map(),assessSize=null){
  const items=issues(objects,regions,savedBases),owners=new Map(objects.flatMap(o=>o.region_ids.map(id=>[id,o.object_id])));
  let sizeChecked=0,sizeUnsupported=0;
  for(const r of regions){if(!assessSize||!Array.isArray(r.min)||!Array.isArray(r.max)){sizeUnsupported++;continue;}
   const a=assessSize(r);if(!a.limit){sizeUnsupported++;continue;}sizeChecked++;
   if(a.flagged)items.push({kind:'size_suspect',source:'geometry_heuristic',object_id:owners.get(r.id)||null,region_id:r.id,
    detail:`范围疑似过大：世界轴跨度 ${a.spans.map(n=>n.toFixed(2)).join(' × ')} m；类别上限提示 ${a.limit} m。可能是片段误关联或特殊尺寸，不是确定错误。`,rule:a.rule,measurement:a.spans,threshold:a.limit});
  }
  const anomaly=x=>x.source==='human_review'||['missing','category_conflict','size_suspect'].includes(x.kind);
  const anomalies=items.filter(anomaly),information=items.filter(x=>['no_evidence','stale'].includes(x.kind));
  const states=objects.map(o=>reviewState(o,regions,savedBases.get(o.object_id)));
  const progress={regions:regions.length,object_records:objects.length,unregistered:regions.filter(r=>!owners.has(r.id)).length,
   unreviewed:states.filter(s=>Object.values(s.checks).includes('pending')).length,unknown:states.filter(s=>Object.values(s.checks).includes('unknown')).length};
  return {anomalies,information,all:items,progress,size_checked:sizeChecked,size_unsupported:sizeUnsupported,semantic_accuracy_validated:false};
 }
 // Exact shared support cells on identical lattices. A diagnostic, not identity inference.
 function supportConflicts(regions,{maxCells=500000,maxComparisons=2000000,maxPairs=100000}={}){
  if(!Number.isInteger(maxCells)||maxCells<1||!Number.isInteger(maxComparisons)||maxComparisons<1||!Number.isInteger(maxPairs)||maxPairs<1)throw Error('Invalid overlap budget');
  const stats={regions:regions.length,eligible:0,unsupported:0,budget_skipped_regions:0,indexed_cells:0,pairs_checked:0,incompatible_pairs:0,budget_skipped_pairs:0,cell_comparisons:0},eligible=[],pairs=[];
  for(const r of regions){const g=r.geometry;
   if(g?.type!=='surface-voxels'||!Array.isArray(g.cells)||!g.cells.length||!Number.isFinite(g.cell)||g.cell<=0||!Array.isArray(g.origin)||g.origin.length!==3||!g.origin.every(Number.isFinite)){stats.unsupported++;continue;}
   if(stats.indexed_cells+g.cells.length>maxCells){stats.budget_skipped_regions++;continue;}
   const cells=new Set(),lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];let valid=true;
   for(const c of g.cells){if(!Array.isArray(c)||c.length!==3||!c.every(Number.isSafeInteger)){valid=false;break;}cells.add(c.join(','));for(let k=0;k<3;k++){lo[k]=Math.min(lo[k],c[k]);hi[k]=Math.max(hi[k],c[k]);}}
   if(!valid){stats.unsupported++;continue;}stats.indexed_cells+=g.cells.length;
   eligible.push({r,cells,lo,hi,grid:JSON.stringify([g.cell,g.origin])});
  }
  stats.eligible=eligible.length;
  let considered=0;pairLoop:for(let i=0;i<eligible.length;i++)for(let j=i+1;j<eligible.length;j++){
   if(considered++>=maxPairs){stats.budget_skipped_pairs+=eligible.length*(eligible.length-1)/2-(considered-1);break pairLoop;}
   const a=eligible[i],b=eligible[j];if(a.grid!==b.grid){stats.incompatible_pairs++;continue;}
   if(a.lo.some((v,k)=>v>b.hi[k]||a.hi[k]<b.lo[k])){stats.pairs_checked++;continue;}
   const small=a.cells.size<b.cells.size?a.cells:b.cells,large=small===a.cells?b.cells:a.cells;
   if(stats.cell_comparisons+small.size>maxComparisons){stats.budget_skipped_pairs++;continue;}
   stats.cell_comparisons+=small.size;stats.pairs_checked++;let common=0;for(const key of small)if(large.has(key))common++;
   const ratio=common/small.size;if(common<5||ratio<.3)continue;
   pairs.push({a:a.r.id,b:b.r.id,category_a:a.r.category,category_b:b.r.category,kind:a.r.category===b.r.category?'possible_duplicate':'category_overlap',common_cells:common,cells_a:a.cells.size,cells_b:b.cells.size,fraction_a:common/a.cells.size,fraction_b:common/b.cells.size,smaller_fraction:ratio});
  }
  pairs.sort((a,b)=>b.smaller_fraction-a.smaller_fraction||b.common_cells-a.common_cells||a.a.localeCompare(b.a)||a.b.localeCompare(b.b));
  return {schema:'scenesem.support-conflicts.v1',rule:'exact_same_lattice_intersection_v1',thresholds:{min_common_cells:5,min_smaller_fraction:.3},budget:{maxCells,maxComparisons,maxPairs},stats,pairs,semantic_accuracy_validated:false,real_robot_commands:false};
 }
 // Reference availability is not original-source availability or physical identity.
 function pairEvidenceSources(pair,regions,objects){
  const sides=['a','b'].map(s=>{
   const region=regions.find(r=>r.id===pair[s]);if(!region)throw Error('重叠区域已失效');
   return {region:evidenceIds(region),object:[...new Set(objects.filter(o=>o.region_ids.includes(region.id)).flatMap(o=>o.evidence_ids||[]))]};
  });
  const ids=evidenceIds({regions,objects});
  return ids.map(id=>({id,a:sides[0].region.includes(id)?'region_reference':sides[0].object.includes(id)?'object_reference':'scene_reference',b:sides[1].region.includes(id)?'region_reference':sides[1].object.includes(id)?'object_reference':'scene_reference'}))
   .sort((a,b)=>Number(b.a!=='scene_reference'||b.b!=='scene_reference')-Number(a.a!=='scene_reference'||a.b!=='scene_reference')||a.id.localeCompare(b.id));
 }
 function compareSupportPair(frame,depth,validity,a,b,result,meta){
  const hashes=x=>JSON.stringify(Object.entries(x||{}).sort(([a],[b])=>a.localeCompare(b)));
  if(frame.schema!=='scenesem.evidence.v1'||frame.scene_id!==meta.scene_id||frame.coordinate_frame!==meta.coordinate_frame||hashes(frame.source_hashes)!==hashes(meta.source_hashes))throw Error('对照证据与当前场景批次或坐标不一致');
  const n=frame.width*frame.height,c=frame.camera;
  if(!Number.isInteger(frame.width)||!Number.isInteger(frame.height)||frame.width<1||frame.height<1||n>4194304||depth.byteLength!==n*4||validity.length!==n||!c||!Array.isArray(c.projection)||c.projection.length!==16||!c.projection.every(Number.isFinite)||!Number.isFinite(c.near)||!Number.isFinite(c.far)||c.near<=0||c.far<=c.near)throw Error('对照帧或深度尺寸无效');
  if(frame.depth_encoding!=='float32-le-linear-normalized-top-left')throw Error('不支持的证据深度编码');
  if(result&&(result.width!==frame.width||result.height!==frame.height||result.image_sha256!==frame.rgb||!Array.isArray(result.components)))throw Error('掩膜与归档原图不一致');
  if(a.id===b.id)throw Error('需要两个不同区域');
  // Honor archived validity without changing the caller's depth buffer.
  const raw=new DataView(depth.buffer.slice(depth.byteOffset,depth.byteOffset+depth.byteLength));
  for(let i=0;i<n;i++){if(validity[i]!==0&&validity[i]!==1)throw Error('无效的深度有效性掩膜');if(!validity[i])raw.setFloat32(i*4,NaN,true);}
  const left=compareEvidence(frame,raw,[a],result),right=compareEvidence(frame,raw,[b],result);
  const masks=left.masks.map((m,i)=>({index:m.index,category:m.category,a_inside:m.inside.length,b_inside:right.masks[i].inside.length,a_fraction:left.stats.visible?m.inside.length/left.stats.visible:null,b_fraction:right.stats.visible?right.masks[i].inside.length/right.stats.visible:null}));
  masks.sort((x,y)=>Math.min(y.a_fraction||0,y.b_fraction||0)-Math.min(x.a_fraction||0,x.b_fraction||0)||y.a_inside+y.b_inside-x.a_inside-x.b_inside||x.index-y.index);
  return {schema:'scenesem.support-pair-evidence.v1',a:left,b:right,masks,visibility:left.stats.visible?(right.stats.visible?'both_visible':'a_only'):(right.stats.visible?'b_only':'neither_visible'),identity_conclusion:'unknown',semantic_accuracy_validated:false,real_robot_commands:false};
 }
 // Associate complete object records, not just the two seed fragments. Geometry stays untouched.
 function identityAssociationPlan(regions,objects,targetRegionId,sourceRegionId,newId){
  registrationPlan(regions,objects);
  const byId=new Map(regions.map(r=>[r.id,r])),owners=new Map(),objectIds=new Set();
  for(const o of objects){if(objectIds.has(o.object_id)||!Array.isArray(o.region_ids)||!o.region_ids.length)throw Error('对象档案 ID 或归属无效');objectIds.add(o.object_id);for(const id of o.region_ids){if(owners.has(id))throw Error('区域存在重复归属，先修复记录');owners.set(id,o);}}
  if(targetRegionId===sourceRegionId||!byId.has(targetRegionId)||!byId.has(sourceRegionId))throw Error('请选择两个不同的有效区域');
  const target=owners.get(targetRegionId),source=owners.get(sourceRegionId);
  if(target&&target===source)throw Error('这两个区域已属于同一档案，无需重复关联');
  const targetRefs=target?.region_ids||[targetRegionId],sourceRefs=source?.region_ids||[sourceRegionId],refs=[...targetRefs,...sourceRefs];
  if(refs.some(id=>!byId.has(id)))throw Error('受影响档案包含失效区域，请先修复归属');
  for(const o of [target,source].filter(Boolean))if(o.pending_review||o.pending_mask_correction)throw Error('受影响档案有未提交审核或修正');
  const next=target?clone(target):register(byId.get(targetRegionId),[],newId);
  if(!/^obj-[0-9a-f-]{36}$/.test(next.object_id)||!target&&objectIds.has(next.object_id))throw Error('新对象 ID 无效或重复');
  next.region_ids=refs;
  next.evidence_ids=[...new Set([...(target?.evidence_ids||[]),...(source?.evidence_ids||[]),...refs.flatMap(id=>evidenceIds(byId.get(id)))])].sort();
  if(next.evidence_ids.length>500)throw Error('关联后证据超过 500 条限制');
  next.identity_state='candidate_not_human_confirmed';next.task_qualification=false;next.review_stale=!!next.reviews.length;
  const remaining=objects.filter(o=>o!==target&&o!==source).map(clone);remaining.push(next);
  if(remaining.length>5000)throw Error('关联后对象数超限');
  return {objects:remaining,target_id:next.object_id,source_id:source?.object_id||null,target_before:target?.object_id||null,
   target_regions:targetRefs.slice(),source_regions:sourceRefs.slice(),region_ids:refs.slice(),categories:[...new Set(refs.map(id=>byId.get(id).category))],evidence_count:next.evidence_ids.length,
   target_reviews:target?.reviews.length||0,source_reviews:source?.reviews.length||0,source_mask_corrections:source?.mask_corrections?.length||0,
   geometry_changed:false,identity_conclusion:'candidate_not_human_confirmed'};
 }
 function validateSpatial(value){
  if(value===null)return null;
  const need=ok=>{if(!ok)throw Error('区域层级或归属记录无效');},keys=(v,names)=>v&&typeof v==='object'&&!Array.isArray(v)&&Object.keys(v).sort().join(',')===names.split(',').sort().join(',');
  need(keys(value,'schema,zones,memberships')&&['scenesem.spatial-context.v1','scenesem.spatial-context.v2'].includes(value.schema));need(Array.isArray(value.zones)&&value.zones.length<=1000&&Array.isArray(value.memberships)&&value.memberships.length<=5000);
  const zones=new Map(),seenObjects=new Set(),annotation=r=>need(['manual','automatic_candidate','unknown'].includes(r.source)&&r.verification==='unverified'&&typeof r.notes==='string'&&r.notes.length<=1000);
  for(const z of value.zones){need(keys(z,'zone_id,name,kind,parent_id,source,verification,notes')||(value.schema==='scenesem.spatial-context.v2'&&keys(z,'zone_id,name,kind,parent_id,source,verification,notes,boundary')));if(z.boundary!==undefined)validateZoneBoundary(z.boundary);need(/^zone-[0-9a-f-]{36}$/.test(z.zone_id)&&!zones.has(z.zone_id)&&typeof z.name==='string'&&z.name.trim()&&z.name.length<=100&&['floor','room','functional_area'].includes(z.kind)&&(z.parent_id===null||typeof z.parent_id==='string'));annotation(z);zones.set(z.zone_id,z);}
  for(const z of value.zones){const seen=new Set([z.zone_id]);let parent=z.parent_id;while(parent!==null){need(zones.has(parent)&&!seen.has(parent));seen.add(parent);parent=zones.get(parent).parent_id;}}
  for(const m of value.memberships){need(keys(m,'object_id,zone_id,source,verification,notes'));need(/^obj-[0-9a-f-]{36}$/.test(m.object_id)&&!seenObjects.has(m.object_id)&&(m.zone_id===null||zones.has(m.zone_id)));annotation(m);seenObjects.add(m.object_id);}return value;
 }
 function spatialSummary(value,objects){const v=validateSpatial(value)??{zones:[],memberships:[]},ids=new Set(objects.map(o=>o.object_id)),members=new Map(v.memberships.map(m=>[m.object_id,m]));return {zones:v.zones.length,unassigned:objects.filter(o=>!members.get(o.object_id)?.zone_id).length,missing_objects:v.memberships.filter(m=>!ids.has(m.object_id)).map(m=>m.object_id)};}
 function sameSpatial(a,b){const ordered=x=>Array.isArray(x)?x.map(ordered):x&&typeof x==='object'?Object.fromEntries(Object.entries(x).sort(([a],[b])=>a.localeCompare(b)).map(([k,v])=>[k,ordered(v)])):x;return JSON.stringify(ordered(a))===JSON.stringify(ordered(b));}
 root.ProjectCore={dimensions,evidenceIds,register,registrationPlan,registerBatch,link,basis,reconcile,validate,framePayload,unproject,projectPoint,compareEvidence,maskCorrection,maskCategory,reviewState,issues,triage,supportConflicts,pairEvidenceSources,compareSupportPair,identityAssociationPlan,validateSpatial,spatialSummary};
 root.ProjectCore.sameSpatial=sameSpatial;
 function validateZoneBoundary(b){if(!b||Object.keys(b).sort().join(',')!=='max,min,type'||b.type!=='aabb'||!['min','max'].every(k=>Array.isArray(b[k])&&b[k].length===3&&b[k].every(v=>Number.isFinite(v)&&Math.abs(v)<100000))||b.min.some((v,i)=>b.max[i]-v<.01))throw Error('边界须是有效场景坐标 AABB，各轴跨度至少 0.01');return b;}
 function proposeZoneMemberships(spatial,objects,regions,options={}){
  validateSpatial(spatial);const maxChecks=options.maxChecks??2000000;if(!Number.isSafeInteger(maxChecks)||maxChecks<1)throw Error('检查预算无效');
  const zones=spatial.zones.filter(z=>z.boundary),byZone=new Map(spatial.zones.map(z=>[z.zone_id,z])),byRegion=new Map(regions.map(r=>[r.id,r])),owned=new Set(spatial.memberships.map(m=>m.object_id));
  const stats={objects:objects.length,bounded_zones:zones.length,unbounded_zones:spatial.zones.length-zones.length,checks:0},rows=[];
  const lineage=new Map();for(const z of zones){const parents=new Set();let p=z.parent_id;while(p){parents.add(p);p=byZone.get(p).parent_id;}lineage.set(z.zone_id,parents);}
  const ancestor=(a,b)=>lineage.get(b).has(a);
  const contains=(a,b)=>a.min.every((v,k)=>v<=b.min[k]&&a.max[k]>=b.max[k]);
  for(const o of objects){const base={object_id:o.object_id};if(owned.has(o.object_id)){rows.push({...base,status:'existing_membership'});continue;}if(!zones.length){rows.push({...base,status:'no_boundaries'});continue;}
   const rs=o.region_ids.map(id=>byRegion.get(id));if(!rs.length||rs.some(r=>!r)){rows.push({...base,status:'missing_regions'});continue;}
   if(rs.some(r=>{const g=r.geometry;return g?.type!=='surface-voxels'||!Number.isFinite(g.cell)||g.cell<=0||!Array.isArray(g.origin)||g.origin.length!==3||!g.origin.every(Number.isFinite)||!Array.isArray(g.cells);})){rows.push({...base,status:'unsupported_geometry'});continue;}
   const grid=rs[0].geometry;if(rs.some(r=>JSON.stringify([r.geometry.cell,r.geometry.origin])!==JSON.stringify([grid.cell,grid.origin]))){rows.push({...base,status:'incompatible_grids'});continue;}
   const size=rs.reduce((n,r)=>n+r.geometry.cells.length,0);if(size>50000||size*zones.length+stats.checks>maxChecks){rows.push({...base,status:'budget_skipped'});continue;}
   if(rs.some(r=>r.geometry.cells.some(c=>!Array.isArray(c)||c.length!==3||!c.every(Number.isSafeInteger)))){rows.push({...base,status:'unsupported_geometry'});continue;}
   const cells=[...new Map(rs.flatMap(r=>r.geometry.cells).map(c=>[c.join(','),c])).values()];if(cells.length<10){rows.push({...base,status:'insufficient_support',total:cells.length});continue;}
   const hits=zones.map(z=>{let inside=0;for(const cell of cells){stats.checks++;const lo=cell.map((v,k)=>grid.origin[k]+v*grid.cell),hi=lo.map(v=>v+grid.cell);if(contains(z.boundary,{min:lo,max:hi}))inside++;}return {zone_id:z.zone_id,inside,total:cells.length,fraction:inside/cells.length};});
   const qualified=hits.filter(h=>h.fraction>=.95);
   // The deepest qualifying zone can win only when every competing hit is its containing ancestor.
   const best=qualified.reduce((a,b)=>!a||lineage.get(b.zone_id).size>lineage.get(a.zone_id).size?b:a,null),ambiguous=best&&hits.some(h=>h.zone_id!==best.zone_id&&h.fraction>.05&&!(ancestor(h.zone_id,best.zone_id)&&contains(byZone.get(h.zone_id).boundary,byZone.get(best.zone_id).boundary)));
   rows.push({...base,status:best&&!ambiguous?'candidate':qualified.length?'ambiguous':'no_clear_fit',total:cells.length,hits,...(best&&!ambiguous?{zone_id:best.zone_id,inside:best.inside,fraction:best.fraction}:{})});
  }
  return {schema:'scenesem.zone-membership-proposals.v1',rule:'whole_support_cells_aabb_95pct_v1',stats,rows,semantic_accuracy_validated:false,real_robot_commands:false};
 }
 root.ProjectCore.validateZoneBoundary=validateZoneBoundary;root.ProjectCore.proposeZoneMemberships=proposeZoneMemberships;
 // Recompute only existing automatic, non-null memberships. A failed check is not a deletion proposal.
 function recheckZoneMemberships(spatial,objects,regions,options={}){
  validateSpatial(spatial);
  const byObject=new Map(objects.map(o=>[o.object_id,o])),eligible=spatial.memberships.filter(m=>m.source==='automatic_candidate'&&m.zone_id!==null),ids=new Set(eligible.map(m=>m.object_id));
  const report=proposeZoneMemberships({...spatial,memberships:[]},objects.filter(o=>ids.has(o.object_id)),regions,options),computed=new Map(report.rows.map(r=>[r.object_id,r]));
  const rows=spatial.memberships.map(m=>{
   const base={object_id:m.object_id,old_zone_id:m.zone_id};
   if(m.source!=='automatic_candidate'||m.zone_id===null)return {...base,status:'protected',reason:'manual_unknown_or_explicit_unknown'};
   if(!byObject.has(m.object_id))return {...base,status:'unresolved',reason:'missing_object'};
   const r=computed.get(m.object_id),oldHit=r.hits?.find(h=>h.zone_id===m.zone_id);
   return {...base,status:r.status==='candidate'?(r.zone_id===m.zone_id?'same':'changed'):'unresolved',reason:r.status,new_zone_id:r.status==='candidate'?r.zone_id:null,inside:r.inside??null,total:r.total??null,old_inside:oldHit?.inside??null,hits:r.hits||[]};
  });
  return {schema:'scenesem.zone-membership-recheck.v1',rule:report.rule,stats:{...report.stats,memberships:rows.length},rows,semantic_accuracy_validated:false,real_robot_commands:false};
 }
 function applyZoneMembershipRecheck(spatial,objects,regions,report,selectedIds){
  const current=recheckZoneMemberships(spatial,objects,regions);
  if(!sameSpatial(current,report))throw Error('归属复算已过期，请重新检查');
  if(!Array.isArray(selectedIds)||!selectedIds.length||new Set(selectedIds).size!==selectedIds.length)throw Error('请选择有效且不重复的改区建议');
  const chosen=new Map(current.rows.filter(r=>selectedIds.includes(r.object_id)&&r.status==='changed').map(r=>[r.object_id,r]));
  if(chosen.size!==selectedIds.length)throw Error('只允许采用明确改区建议；人工、未知、未检查和无变化记录不改写');
  const next=clone(spatial);
  for(const m of next.memberships){const r=chosen.get(m.object_id);if(!r)continue;m.zone_id=r.new_zone_id;m.notes=`${current.rule}; recheck ${r.old_zone_id} -> ${r.new_zone_id}; ${r.inside}/${r.total} observed support cells; snapshot only, not revalidated after edits`;}
  validateSpatial(next);return next;
 }
 root.ProjectCore.recheckZoneMemberships=recheckZoneMemberships;root.ProjectCore.applyZoneMembershipRecheck=applyZoneMembershipRecheck;
 function validateBusiness(value){
  if(value===null)return null;
  const need=ok=>{if(!ok)throw Error('业务属性记录无效，请核对来源、记录人和日期');},fields='attribute_id,subject_kind,subject_id,name,value,basis,source,recorder,observed_on,valid_from,valid_until,verification'.split(',').sort().join(',');
  need(value&&Object.keys(value).sort().join(',')==='attributes,schema'&&['scenesem.business-context.v1','scenesem.business-context.v2'].includes(value.schema)&&Array.isArray(value.attributes)&&value.attributes.length<=2000);
  const seen=new Set();for(const a of value.attributes){
   need(a&&(Object.keys(a).sort().join(',')===fields||value.schema==='scenesem.business-context.v2'&&Object.keys(a).sort().join(',')===(fields+',source_snapshot').split(',').sort().join(','))&&typeof a.attribute_id==='string'&&/^attr-[0-9a-f-]{36}$/.test(a.attribute_id)&&!seen.has(a.attribute_id));seen.add(a.attribute_id);
   if(a.source_snapshot!==undefined)validateSourceSnapshot(a.source_snapshot);
   need(['object','zone'].includes(a.subject_kind)&&typeof a.subject_id==='string'&&new RegExp('^'+(a.subject_kind==='object'?'obj-':'zone-')+'[0-9a-f-]{36}$').test(a.subject_id));
   for(const [k,n] of [['name',100],['value',2000],['source',1000],['recorder',100]])need(typeof a[k]==='string'&&a[k].length<=n);
   need(a.name.trim()&&['unknown','simulation','field_report'].includes(a.basis)&&a.verification==='unverified');
   for(const k of ['observed_on','valid_from','valid_until']){const d=a[k];need(d===null||typeof d==='string'&&/^[0-9]{4}-[0-9]{2}-[0-9]{2}$/.test(d)&&d.slice(0,4)!=='0000'&&Number.isFinite(Date.parse(d+'T00:00:00Z'))&&new Date(d+'T00:00:00Z').toISOString().slice(0,10)===d);}
   need(!a.valid_from||!a.valid_until||a.valid_from<=a.valid_until);
   need(a.basis==='unknown'?a.value==='':a.value.trim()&&a.source.trim());
   if(a.basis==='field_report')need(a.recorder.trim()&&a.observed_on);
  }return value;
 }
 root.ProjectCore.validateBusiness=validateBusiness;
 function validateSourceSnapshot(s){if(!s||Object.keys(s).sort().join(',')!=='bytes,name,sha256'||typeof s.sha256!=='string'||!/^[0-9a-f]{64}$/.test(s.sha256)||!Number.isSafeInteger(s.bytes)||s.bytes<1||s.bytes>8*1024*1024||typeof s.name!=='string'||!s.name.trim()||s.name.length>180||/[\x00-\x1f\x7f<>:"/\\|?*]/.test(s.name))throw Error('材料快照无效，单文件限 8 MiB，文件名不能带路径');return s;}
 root.ProjectCore.validateSourceSnapshot=validateSourceSnapshot;
 function checkBusinessAtDate(value,objects,spatial,asOf,maxComparisons=10000){
  validateBusiness(value);validateSpatial(spatial);
  if(typeof asOf!=='string'||!/^\d{4}-\d{2}-\d{2}$/.test(asOf)||asOf.startsWith('0000')||!Number.isFinite(Date.parse(asOf+'T00:00:00Z'))||new Date(asOf+'T00:00:00Z').toISOString().slice(0,10)!==asOf)throw Error('请输入有效检查日期 YYYY-MM-DD');
  if(!Number.isSafeInteger(maxComparisons)||maxComparisons<1||maxComparisons>100000)throw Error('检查预算无效');
  const attrs=value?.attributes||[],subjects={object:new Set(objects.map(o=>o.object_id)),zone:new Set((spatial?.zones||[]).map(z=>z.zone_id))},groups=new Map(),names=new Map();
  const rows=attrs.map(a=>{const temporal=a.valid_from&&asOf<a.valid_from?'not_started':a.valid_until&&asOf>a.valid_until?'expired':a.valid_from&&a.valid_until?'within_declared_period':'incomplete_period';
   const row={attribute_id:a.attribute_id,temporal,missing_subject:!subjects[a.subject_kind].has(a.subject_id),unknown_value:a.basis==='unknown',observed_after_date:!!a.observed_on&&a.observed_on>asOf,mixed_basis:false,pair_notice_count:0};
   const name=JSON.stringify([a.subject_kind,a.subject_id,a.name]);if(!names.has(name))names.set(name,[]);names.get(name).push({a,row});
   if(!['not_started','expired'].includes(temporal)&&!row.observed_after_date&&a.basis!=='unknown'){const key=JSON.stringify([a.subject_kind,a.subject_id,a.name,a.basis]);if(!groups.has(key))groups.set(key,[]);groups.get(key).push({a,row});}return row;});
  for(const group of names.values())if(new Set(group.map(x=>x.a.basis)).size>1)for(const {row} of group)row.mixed_basis=true;
  const totalPairs=[...groups.values()].reduce((n,g)=>n+g.length*(g.length-1)/2,0),pairs=[];let comparisons=0;
  outer:for(const group of groups.values())for(let i=0;i<group.length;i++)for(let j=i+1;j<group.length;j++){
   if(comparisons>=maxComparisons)break outer;comparisons++;const a=group[i],b=group[j],known=a.row.temporal==='within_declared_period'&&b.row.temporal==='within_declared_period';
   const kind=a.a.value===b.a.value?'same_text':known?'different_text':'possible_overlap_unknown_period';pairs.push({a:a.a.attribute_id,b:b.a.attribute_id,kind});a.row.pair_notice_count++;b.row.pair_notice_count++;
  }
  return {schema:'scenesem.business-date-check.v1',as_of:asOf,context_present:value!==null,scope:'declared_dates_and_exact_text_not_business_truth',rows,pairs,comparisons,skipped_comparisons:totalPairs-comparisons,max_comparisons:maxComparisons,source_content_verified:false,task_qualification:false,real_robot_commands:false};
 }
 root.ProjectCore.checkBusinessAtDate=checkBusinessAtDate;
 if(typeof module!=='undefined')module.exports=root.ProjectCore;
})(globalThis);
