(function(root){
 'use strict';
 const C=typeof module!=='undefined'?require('./object-core.js'):root.ObjectCore,B=typeof module!=='undefined'?require('./boxes.js'):root.BoxCore,I=typeof module!=='undefined'?require('./instance-tools.js'):root.InstanceTools;
 const key=p=>p.join(','),copy=x=>structuredClone(x),supported=new Set(['chair','screen','computer','table','sofa','plant','keyboard','mouse','cup','bottle','book','device']);
 function eligibility(b){if(!b||!supported.has(b.category)||b.geometry?.type!=='surface-voxels')return '请先选择已加入的物体可见片段';if(C.geometryWarnings(b).length)return '当前区域跨度异常，请先拆分并选中目标片段';if(b.geometry.origin.some(v=>Math.abs(v)>1e-10))return '此片段已平移，无法直接匹配原场景深度；请重新选择原坐标候选';return '';}
 function windowBounds(b){const pad=b.category==='chair'?[.8,.8,1.15]:['screen','computer'].includes(b.category)?[.65,.65,.65]:[.7,.7,.9];return {min:b.min.map((v,i)=>v-pad[i]),max:b.max.map((v,i)=>v+pad[i])};}
 function project(p,frame){const m=frame.projection,q=[0,1,2,3].map(i=>m[i]*p[0]+m[i+4]*p[1]+m[i+8]*p[2]+m[i+12]);if(q[3]<=0||q.some(v=>!Number.isFinite(v)))return null;return {x:(q[0]/q[3]+1)*frame.width/2,y:(1-q[1]/q[3])*frame.height/2,z:q[2]/q[3]};}
 function unproject(x,y,d,f){const z=f.far*d/(d*(f.far-f.near)+f.near),v=[(x+.5)/f.width*2-1,1-(y+.5)/f.height*2,z*2-1,1],m=f.inverse,p=[0,1,2,3].map(i=>m[i]*v[0]+m[i+4]*v[1]+m[i+8]*v[2]+m[i+12]);return p.slice(0,3).map(v=>v/p[3]);}
 function visibleSeeds(base,frame){
  const pixels=new Map(),g=base.geometry;for(const c of g.cells){const p=c.map((v,i)=>g.origin[i]+(v+.5)*g.cell),q=project(p,frame);if(!q||q.z< -1||q.z>1)continue;const x=Math.floor(q.x),y=Math.floor(q.y);if(x<0||y<0||x>=frame.width||y>=frame.height)continue;const d=frame.depth[y*frame.width+x];if(!Number.isFinite(d)||d<0||d>=1)continue;
   const hit=unproject(x,y,d,frame);if(Math.hypot(...hit.map((v,i)=>v-p[i]))>Math.max(.1,g.cell*2))continue;
   const id=x+','+y;if(!pixels.has(id))pixels.set(id,{x,y,keys:[]});pixels.get(id).keys.push(key(c));
  }return [...pixels.values()];
 }
 function targetROI(base,frame){const b=windowBounds(base),corners=Array.from({length:8},(_,k)=>[0,1,2].map(i=>(k&(1<<i))?b.max[i]:b.min[i])),qs=corners.map(p=>project(p,frame));if(qs.some(q=>!q))return [0,0,frame.width,frame.height];
  const x0=Math.max(0,Math.floor(Math.min(...qs.map(q=>q.x)))),y0=Math.max(0,Math.floor(Math.min(...qs.map(q=>q.y)))),x1=Math.min(frame.width,Math.ceil(Math.max(...qs.map(q=>q.x)))),y1=Math.min(frame.height,Math.ceil(Math.max(...qs.map(q=>q.y))));
  return x1-x0>=64&&y1-y0>=64?[x0,y0,x1,y1]:[0,0,frame.width,frame.height];
 }
 function maskInfo(c,w,h){let lastY=-1,lastEnd=-1,pixels=0;const rows=new Map();for(const r of c.runs){if(!Array.isArray(r)||r.length!==3||!r.every(Number.isInteger))throw Error('掩膜行无效');const [y,a,b]=r;if(y<0||y>=h||a<0||b>w||a>=b||y<lastY||(y===lastY&&a<lastEnd))throw Error('掩膜范围无效');if(!rows.has(y))rows.set(y,[]);rows.get(y).push([a,b]);lastY=y;lastEnd=b;pixels+=b-a;}return {component:c,rows,pixels,contains:p=>(rows.get(p.y)||[]).some(([a,b])=>p.x>=a&&p.x<b)};}
 function overlap(a,b){let intersection=0;for(const [y,ar]of a.rows){const br=b.rows.get(y)||[];let i=0,j=0;while(i<ar.length&&j<br.length){intersection+=Math.max(0,Math.min(ar[i][1],br[j][1])-Math.max(ar[i][0],br[j][0]));if(ar[i][1]<br[j][1])i++;else j++;}}return intersection/Math.max(1,Math.min(a.pixels,b.pixels));}
 function matchMasks(base,frame,result,seeds=visibleSeeds(base,frame)){
  if(seeds.length<3)return {accepted:[],seeds,reason:'有效目标锚点不足：当前视角遮挡、太远或深度不一致'};
  const masks=result.components.filter(c=>c.category===base.category&&!c.crop_truncated).map(c=>maskInfo(c,frame.width,frame.height)).map(m=>({...m,hits:seeds.filter(m.contains).length})).filter(m=>m.hits>=3&&m.hits/seeds.length>=.5).sort((a,b)=>b.hits-a.hits||b.component.score-a.component.score);
  if(!masks.length)return {accepted:[],seeds,reason:'没有同类实例覆盖足够的原目标锚点，不按距离猜测'};
  const best=masks[0],agrees=m=>m===best||(m.component.crop_id!==best.component.crop_id&&overlap(m,best)>=.65);
  if(masks.some(m=>!agrees(m)&&m.hits/seeds.length>=best.hits/seeds.length-.2))return {accepted:[],seeds,reason:'多个实例争抢目标锚点，关联不明确，本视角跳过'};
  return {accepted:masks.filter(agrees),seeds,reason:'锚点匹配与跨裁图一致性检查通过'};
 }
 function analyze(base,frame,result){
  if(result.width!==frame.width||result.height!==frame.height||result.backend!=='instance')throw Error('补识别结果与采集帧不匹配');
  const matched=matchMasks(base,frame,result),limit=windowBounds(base),groups=[],rejected=[];
  for(const m of matched.accepted)for(const part of C.fromMask(m.component,frame,{...result,cell_m:base.geometry.cell,targeted:true})){
   const united={category:base.category,min:part.min.map((v,i)=>Math.min(v,base.min[i])),max:part.max.map((v,i)=>Math.max(v,base.max[i]))};
   if(part.min.some((v,i)=>v<limit.min[i]||part.max[i]>limit.max[i])||C.geometryWarnings(united).length){rejected.push({cells:part.geometry.cells.length,reason:'超出目标搜索范围或类别跨度；不裁剪成合格片段'});continue;}
   groups.push({...part,anchor_hits:m.hits,visible_anchor_pixels:matched.seeds.length,source_instance_id:m.component.instance_id,crop_id:m.component.crop_id});
  }
  const positive=[],negative=[];if(matched.accepted.length)for(const seed of matched.seeds)(matched.accepted.some(m=>m.contains(seed))?positive:negative).push(...seed.keys);
  return {groups,rejected,positive:[...new Set(positive)],negative:[...new Set(negative)],reason:matched.reason,visible_anchor_pixels:matched.seeds.length,matched_masks:matched.accepted.map(m=>({instance_id:m.component.instance_id,crop_id:m.component.crop_id,score:m.component.score,anchor_hits:m.hits,pixels:m.pixels,runs:m.component.runs})),pose_id:frame.pose_id,camera_position:frame.camera_position,image_sha256:result.image_sha256,width:frame.width,height:frame.height,model:result.model,model_sha256:result.model_sha256,threshold:result.threshold};
 }
 function suspectCells(analyses){const yes=new Set(analyses.flatMap(a=>a.positive)),no=new Map();for(const a of analyses)for(const k of a.negative){if(!no.has(k))no.set(k,[]);const positions=no.get(k);if(a.camera_position?.length===3&&!positions.some(p=>Math.hypot(...p.map((v,i)=>v-a.camera_position[i]))<.25))positions.push(a.camera_position);}return new Set([...no].filter(([k,p])=>p.length>=2&&!yes.has(k)).map(([k])=>k));}
 // Geometry intersections are conflicting annotations, never proof of background.
 // Ignore broad boxes: they include unobserved air and may swallow a whole chair.
 function labelConflicts(base,labels){
  const candidates=labels.filter(b=>b.id!==base.id&&b.category!==base.category&&(b.geometry?.type==='surface-voxels'||b.geometry?.type==='surface-mask'||(b.status==='reviewed'&&b.geometry?.type==='prism'))),match=B.compile(candidates),result=new Map(),g=base.geometry;
  for(const c of g.cells){const p=c.map((v,i)=>g.origin[i]+(v+.5)*g.cell),other=match(...p,'all').box;if(other)result.set(key(c),{other_id:other.id,category:other.category,status:other.status,reason:'label_overlap_not_ground_truth'});}return result;
 }
 function auditSupport(base,groups,frames,analyses,labels){
  const all=union(base,groups),old=new Set(base.geometry.cells.map(key)),conflicts=labelConflicts(all,labels),votes=new Map();
  const record=(list,position)=>{if(Array.isArray(position)&&position.length===3&&position.every(Number.isFinite)&&!list.some(p=>Math.hypot(...p.map((v,i)=>v-position[i]))<.25))list.push(position.slice());};
  for(let i=0;i<frames.length;i++){const f=frames[i],a=analyses[i];if(!a?.matched_masks?.length)continue;const masks=a.matched_masks.map(m=>maskInfo(m,f.width,f.height));
   // Reproject ALL proposed cells, not just the original target anchors.
   for(const pixel of visibleSeeds(all,f)){const positive=masks.some(m=>m.contains(pixel));for(const k of pixel.keys){if(!votes.has(k))votes.set(k,{positive:[],negative:[]});record(votes.get(k)[positive?'positive':'negative'],f.camera_position);}}
  }
  const flagged=[],singleView=[];for(const c of all.geometry.cells){const k=key(c),v=votes.get(k)||{positive:[],negative:[]},conflict=conflicts.get(k),disagrees=v.negative.length>=2&&v.negative.length>v.positive.length;
   if(!old.has(k)&&v.positive.length<2)singleView.push(k);
   if(conflict||disagrees)flagged.push({key:k,is_new:!old.has(k),positive_views:v.positive.length,negative_views:v.negative.length,conflict:conflict||null,multiview_disagreement:disagrees});
  }
  return {policy:'all_cells_depth_checked_separated_views_v1',flagged,single_view_new_count:singleView.length,checked_cells:all.geometry.cells.length,frames:frames.length,qualification:false};
 }
 function union(base,groups,remove=new Set()){if(!groups.every(g=>g.category===base.category&&I.sameGrid(base.geometry,g.geometry||{})))throw Error('补识别片段的类别或坐标网格不一致');const cells=new Map(base.geometry.cells.filter(c=>!remove.has(key(c))).map(c=>[key(c),c]));for(const group of groups)for(const c of group.geometry.cells)cells.set(key(c),c);if(!cells.size||cells.size>50000)throw Error('补识别后支持单元数量为空或超过 50000，未修改');const geometry={...copy(base.geometry),cells:[...cells.values()].map(c=>c.slice())};return {...copy(base),geometry,...C.bounds(geometry)};}
 function delta(base,next){const old=new Set(base.geometry.cells.map(key)),now=new Set(next.geometry.cells.map(key));return {added:next.geometry.cells.filter(c=>!old.has(key(c))),removed:base.geometry.cells.filter(c=>!now.has(key(c)))};}
 // Non-security fingerprint, used solely to reject stale edits before restore.
 function signature(g){const s=JSON.stringify(g);let a=2166136261,b=5381;for(let i=0;i<s.length;i++){a=Math.imul(a^s.charCodeAt(i),16777619);b=Math.imul(b,33)^s.charCodeAt(i);}return `${s.length}:${a>>>0}:${b>>>0}`;}
 function build(base,groups,analyses,remove=new Set()){
  const next=union(base,groups,remove),change=delta(base,next);if(!change.added.length&&!change.removed.length)throw Error('没有新增或移除支持点，无需改写标签');
  if(C.geometryWarnings(next).length)throw Error('合并后的目标跨度异常，请取消可疑片段再核查');
  next.status='draft';next.provenance={method:'target_anchored_multiview_refinement_v1',evidence_state:'targeted_instance_refinement',human_modified:true,qualification:false,coverage:'observed_only_completeness_unchecked',refinement_source:copy(base),applied_signature:signature(next.geometry),applied_category:base.category,added_cells:change.added.length,removed_cells:change.removed.length,removal_policy:remove.size?'user_confirmed_multiview_negative_evidence':'no_removal',view_ids:[],observations:[],evidence:analyses.map(({groups,rejected,...a})=>({...copy(a),rejected_parts:copy(rejected)})),accepted_parts:groups.map(g=>({pose_id:g.provenance.observations[0].view_id,instance_id:g.source_instance_id,crop_id:g.crop_id,geometry:copy(g.geometry)}))};
  if(JSON.stringify(next).length>16000000)throw Error('补识别溯源超过 16 MB，请导出后分批整理');if(!B.validBox(next))throw Error('补识别几何无效，未修改');return next;
 }
 function canRestore(b){return b?.geometry?.type==='surface-voxels'&&b.provenance?.evidence_state==='targeted_instance_refinement'&&b.category===b.provenance.applied_category&&signature(b.geometry)===b.provenance.applied_signature&&B.validBox(b.provenance.refinement_source);}
 function restore(b){if(!canRestore(b))throw Error('区域已再次修改，不能直接恢复旧补识别来源');const old=copy(b.provenance.refinement_source);old.id=b.id;return old;}
 function planViews(base,positions,current,max=4){const center=base.min.map((v,i)=>(v+base.max[i])/2),candidates=positions.filter(p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite)).map(p=>({p,d:Math.hypot(...p.map((v,i)=>v-center[i])),a:Math.atan2(p[1]-center[1],p[0]-center[0])})).filter(v=>v.d>=1&&v.d<=6&&Math.hypot(...v.p.map((p,i)=>p-current[i]))>.4).sort((a,b)=>a.d-b.d),chosen=[];for(const v of candidates){if(chosen.some(q=>Math.abs(Math.atan2(Math.sin(q.a-v.a),Math.cos(q.a-v.a)))<.45||Math.hypot(...q.p.map((p,i)=>p-v.p[i]))<.5))continue;chosen.push(v);if(chosen.length>=max)break;}return chosen.map(v=>v.p.slice());}
 root.CompletionCore={eligibility,windowBounds,project,visibleSeeds,targetROI,matchMasks,analyze,suspectCells,labelConflicts,auditSupport,union,delta,signature,build,canRestore,restore,planViews};if(typeof module!=='undefined')module.exports=root.CompletionCore;
})(globalThis);
