(function(root){
 'use strict';
 const key=p=>p.join(','), cell=.08, spatialCache=new WeakMap();
 // Sparse, measured depth support; never fill the bounding box or hidden back side.
 function geometry(points,cellSize=cell){if(!Number.isFinite(cellSize)||cellSize<.02||cellSize>.5)throw Error('Invalid cell size');const map=new Map();for(const p of points){if(!p.every(Number.isFinite))continue;const c=p.map(x=>Math.floor(x/cellSize));map.set(key(c),c);}const cells=[...map.values()];return {type:'surface-voxels',origin:[0,0,0],cell:cellSize,cells};}
 function split(g,minCells=5){const left=new Map(g.cells.map(c=>[key(c),c])),out=[];
  while(left.size){const start=left.values().next().value;left.delete(key(start));const group=[start];for(let i=0;i<group.length;i++){const p=group[i];for(let x=-1;x<=1;x++)for(let y=-1;y<=1;y++)for(let z=-1;z<=1;z++){const k=key([p[0]+x,p[1]+y,p[2]+z]);if(left.has(k)){group.push(left.get(k));left.delete(k);}}}if(group.length>=minCells)out.push({...g,cells:group});}return out;
 }
 function bounds(g){return {min:[0,1,2].map(i=>g.origin[i]+Math.min(...g.cells.map(p=>p[i]))*g.cell),max:[0,1,2].map(i=>g.origin[i]+(Math.max(...g.cells.map(p=>p[i]))+1)*g.cell)};}
 // Connectivity is tested on 12 cm buckets; output retains the original cells.
 // This tolerates sampling gaps without joining supports meters apart.
 function spatialParts(g,minCells=5){
  const cached=spatialCache.get(g.cells);if(cached&&cached.cell===g.cell)return cached.groups.filter(c=>c.length>=minCells).map(cells=>({...g,cells}));
  const size=Math.max(.12,g.cell),buckets=new Map();
  for(const c of g.cells){const q=c.map(v=>Math.floor((v+.5)*g.cell/size)),k=key(q);if(!buckets.has(k))buckets.set(k,{q,cells:[]});buckets.get(k).cells.push(c);}
  const coarse={type:'surface-voxels',origin:[0,0,0],cell:size,cells:[...buckets.values()].map(b=>b.q)};
  const groups=split(coarse,1).map(part=>part.cells.flatMap(q=>buckets.get(key(q)).cells)).sort((a,b)=>b.length-a.length);spatialCache.set(g.cells,{cell:g.cell,groups});return groups.filter(c=>c.length>=minCells).map(cells=>({...g,cells}));
 }
 // Generous class-size sanity limits, not learned object dimensions or truth.
 const maxSpan={chair:2,screen:2.5,computer:1.8,keyboard:1,mouse:.6,cup:.7,bottle:1,book:1.2,plant:4,sofa:5};
 function sizeAssessment(b){const limit=maxSpan[b.category]||null,spans=b.max.map((v,i)=>v-b.min[i]);return {rule:'class_aabb_max_span_v1',limit,spans,flagged:!!limit&&spans.some(v=>v>limit),meaning:'heuristic_not_error_or_complete_object'};}
 function geometryWarnings(b){return sizeAssessment(b).flagged?['尺寸异常']:[];}
 function splitCandidate(b){
  return spatialParts(b.geometry,1).map((g,i)=>({...structuredClone(b),...bounds(g),geometry:{...g,origin:g.origin.slice(),cells:g.cells.map(c=>c.slice())},status:'proposed',provenance:{...structuredClone(b.provenance),parent_region_id:b.id,source_provenance:structuredClone(b.provenance),method:'spatial_support_split_v3',geometry_method:'spatial_components_12cm_v1',component_part:i,view_ids:[],observations:[],evidence_state:'spatial_split_needs_review',human_modified:true,qualification:false}}));
 }
 function supportOverlap(a,b){if(a.geometry?.type!=='surface-voxels'||b.geometry?.type!=='surface-voxels'||a.geometry.cell!==b.geometry.cell||a.geometry.origin.some((v,i)=>v!==b.geometry.origin[i]))return 0;const set=new Set(a.geometry.cells.map(key));let common=0;for(const p of b.geometry.cells)if(set.has(key(p)))common++;return common/Math.min(a.geometry.cells.length,b.geometry.cells.length);}
 function observations(p){return p.provenance.observations||p.provenance.view_ids.map(view_id=>({view_id,instance_id:null}));}
 function viewCount(p){return new Set(observations(p).map(o=>o.view_id)).size;}
 function compatible(a,b){
  if(a.category!==b.category||a.provenance.model!==b.provenance.model)return false;
  if(geometryWarnings(a).length||geometryWarnings(b).length)return false;
  // Distinct instance masks in the same view must never fuse into one chair/table.
  return !observations(a).some(x=>observations(b).some(y=>x.view_id===y.view_id&&x.instance_id!=null&&y.instance_id!=null&&x.instance_id!==y.instance_id));
 }
 function merge(pending,next){for(const candidate of next){
  const matches=pending.filter(p=>compatible(p,candidate)&&!geometryWarnings({category:p.category,min:p.min.map((v,i)=>Math.min(v,candidate.min[i])),max:p.max.map((v,i)=>Math.max(v,candidate.max[i]))}).length).map(p=>({p,overlap:supportOverlap(p,candidate)})).filter(x=>x.overlap>.5&&x.overlap*Math.min(x.p.geometry.cells.length,candidate.geometry.cells.length)/Math.max(x.p.geometry.cells.length,candidate.geometry.cells.length)>=.2).sort((a,b)=>b.overlap-a.overlap);
  // Ambiguous association remains separate, rather than guessing identity.
  const old=matches.length&&(!matches[1]||matches[0].overlap-matches[1].overlap>.15)?matches[0].p:null;
  if(old){const cells=new Map([...old.geometry.cells,...candidate.geometry.cells].map(c=>[key(c),c]));if(cells.size>50000)continue;
   old.geometry={...old.geometry,cells:[...cells.values()]};Object.assign(old,bounds(old.geometry));
   old.provenance.observations=[...new Map([...observations(old),...observations(candidate)].map(o=>[o.view_id+':'+o.instance_id,o])).values()];
   old.provenance.view_ids=[...new Set([...old.provenance.view_ids,...candidate.provenance.view_ids])];
   old.provenance.evidence_ids=[...new Set([...(old.provenance.evidence_ids||[]),...(candidate.provenance.evidence_ids||[])])];
   old.provenance.model_score=Math.min(old.provenance.model_score,candidate.provenance.model_score);
   old.provenance.evidence_state=viewCount(old)>1?'multi_view_supported':'single_view_candidate';
  }else if(pending.length<500)pending.push(candidate);
 }return pending;}
 function fromMask(component,frame,result){const points=[],m=frame.inverse,w=frame.width,h=frame.height;
  for(const [y,a,b] of component.runs){if(![y,a,b].every(Number.isInteger)||y<0||y>=h||a<0||b>w||a>=b)throw Error('Invalid mask run');for(let x=a;x<b;x++){const d=frame.depth[y*w+x];if(!Number.isFinite(d)||d<0||d>=1)continue;const distance=frame.near+d*(frame.far-frame.near);if(distance<.15||distance>12)continue;const z=frame.far*d/(d*(frame.far-frame.near)+frame.near),v=[(x+.5)/w*2-1,1-(y+.5)/h*2,z*2-1,1],p=[0,1,2,3].map(i=>m[i]*v[0]+m[i+4]*v[1]+m[i+8]*v[2]+m[i+12]);if(Math.abs(p[3])<1e-8)continue;points.push(p.slice(0,3).map(a=>a/p[3]));}}
  const cellSize=result.cell_m||cell,base=geometry(points,cellSize),instance=result.backend==='instance';
  // A 2D instance can leak to a background surface. Split in 3D before association.
  const parts=instance?spatialParts(base,result.targeted?3:5):split(base),minimum=result.targeted?3:instance?Math.max(5,(parts[0]?.cells.length||0)*.1):5;
  const surfaces=parts.filter(g=>g.cells.length>=minimum),retained=surfaces.reduce((n,g)=>n+g.cells.length,0);
  return surfaces.filter(g=>g.cells.length<=50000).map((g,index)=>({...bounds(g),geometry:g,category:component.category,status:'proposed',
    provenance:{method:result.method||'segformer_rgb_gs_depth_visible_support_v1',geometry_method:'spatial_components_12cm_v1',source_support_cells:base.cells.length,discarded_support_cells:base.cells.length-retained,source_spatial_parts:parts.length,model:result.model,model_sha256:result.model_sha256,model_label:component.label,model_score:component.score,view_ids:[result.image_sha256+':'+frame.pose_id],observations:[{view_id:frame.pose_id,instance_id:component.instance_id!=null?component.instance_id+':'+index:null,image_sha256:result.image_sha256}],evidence_ids:result.evidence_id?[result.evidence_id]:[],evidence_state:'single_view_candidate',detail:result.detail||'standard',support_crops:component.support_crops,threshold:result.threshold,license_scope:result.license_scope,cell_m:cellSize,depth_source:'playcanvas_gs_alpha_depth',qualification:false,component_part:index,small_part_policy:result.targeted?'retain_at_least_3_cells':'relative_10_percent'}}));
 }
 function decodeDepth(bytes,width,height,flipY){if(bytes.length!==width*height*4)throw Error('深度图尺寸不匹配');const output=new Float32Array(width*height),bits=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength),scratch=new DataView(new ArrayBuffer(4));for(let y=0;y<height;y++)for(let x=0;x<width;x++){const k=((flipY?height-1-y:y)*width+x)*4,b=bits.getUint32(k,false);scratch.setUint32(0,b,false);output[y*width+x]=b===0xffffffff?NaN:scratch.getFloat32(0,false);}return output;}
 root.ObjectCore={geometry,split,spatialParts,splitCandidate,geometryWarnings,sizeAssessment,bounds,supportOverlap,merge,fromMask,decodeDepth,viewCount};if(typeof module!=='undefined')module.exports=root.ObjectCore;
})(globalThis);
