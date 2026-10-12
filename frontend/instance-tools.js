(function(root){
 'use strict';
 const B=typeof module!=='undefined'?require('./boxes.js'):root.BoxCore;
 const C=typeof module!=='undefined'?require('./object-core.js'):root.ObjectCore;
 const cache=new WeakMap(),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
 const identity=[[1,0,0],[0,1,0],[0,0,1]];
 const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
 // Symmetric 3x3 Jacobi eigensolver. Axes are fitted to observed cell centers,
 // never to a category-size template. Include whole cells in the final extents.
 function principalAxes(cov){
  const a=cov.map(r=>r.slice()),v=identity.map(r=>r.slice());
  for(let step=0;step<32;step++){
   let p=0,q=1;for(const [i,j]of [[0,2],[1,2]])if(Math.abs(a[i][j])>Math.abs(a[p][q])){p=i;q=j;}
   if(Math.abs(a[p][q])<1e-12)break;
   const angle=.5*Math.atan2(2*a[p][q],a[q][q]-a[p][p]),c=Math.cos(angle),s=Math.sin(angle),pp=a[p][p],qq=a[q][q],pq=a[p][q];
   for(let k=0;k<3;k++)if(k!==p&&k!==q){const kp=a[k][p],kq=a[k][q];a[k][p]=a[p][k]=c*kp-s*kq;a[k][q]=a[q][k]=s*kp+c*kq;}
   a[p][p]=c*c*pp-2*s*c*pq+s*s*qq;a[q][q]=s*s*pp+2*s*c*pq+c*c*qq;a[p][q]=a[q][p]=0;
   for(let k=0;k<3;k++){const kp=v[k][p],kq=v[k][q];v[k][p]=c*kp-s*kq;v[k][q]=s*kp+c*kq;}
  }
  const axes=[0,1,2].sort((i,j)=>a[j][j]-a[i][i]).map(i=>v.map(r=>r[i]));
  for(const axis of axes){const i=[0,1,2].sort((i,j)=>Math.abs(axis[j])-Math.abs(axis[i]))[0];if(axis[i]<0)for(let j=0;j<3;j++)axis[j]*=-1;}
  axes[2]=cross(axes[0],axes[1]);return axes;
 }
 function fit(cells,cell,axes,method){
  const low=[Infinity,Infinity,Infinity],high=[-Infinity,-Infinity,-Infinity];
  for(const p of cells){const center=p.map(v=>(v+.5)*cell);for(let k=0;k<3;k++){const value=dot(center,axes[k]);low[k]=Math.min(low[k],value);high[k]=Math.max(high[k],value);}}
  for(let k=0;k<3;k++){const padding=cell*.5*axes[k].reduce((s,v)=>s+Math.abs(v),0);low[k]-=padding;high[k]+=padding;}
  const size=high.map((v,i)=>v-low[i]),center=[0,1,2].map(i=>axes.reduce((s,a,k)=>s+a[i]*(low[k]+high[k])/2,0));
  return {method,axes,center,size,volume:size.reduce((a,b)=>a*b,1)};
 }
 function orientedBounds(g){
  if(g?.type!=='surface-voxels'||!g.cells.length)throw Error('定向框需要可见表面单元');
  let local=cache.get(g.cells);
  if(!local||local.cell!==g.cell){
   const mean=[0,0,0];for(const p of g.cells)for(let i=0;i<3;i++)mean[i]+=p[i]/g.cells.length;
   const cov=identity.map(()=>[0,0,0]);for(const p of g.cells){const d=p.map((v,i)=>v-mean[i]);for(let i=0;i<3;i++)for(let j=0;j<3;j++)cov[i][j]+=d[i]*d[j]/g.cells.length;}
   const yaw=.5*Math.atan2(2*cov[0][1],cov[0][0]-cov[1][1]),c=Math.cos(yaw),s=Math.sin(yaw);
   const candidates=[fit(g.cells,g.cell,identity,'world_axes'),fit(g.cells,g.cell,[[c,s,0],[-s,c,0],[0,0,1]],'upright_pca'),fit(g.cells,g.cell,principalAxes(cov),'pca_3d')];
   local=candidates.reduce((best,b)=>b.volume<best.volume*(1-1e-6)?b:best);local={...local,cell:g.cell};cache.set(g.cells,local);
  }
  const center=local.center.map((v,i)=>v+g.origin[i]),axes=local.axes.map(a=>a.slice()),size=local.size.slice();
  const corners=Array.from({length:8},(_,k)=>center.map((v,i)=>v+axes.reduce((s,a,j)=>s+a[i]*size[j]/2*((k&(1<<j))?1:-1),0)));
  return {method:local.method,center,axes,size,corners,coverage:'observed_cells_only',includes_hidden_geometry:false};
 }
 function isAssociated(b){return b?.provenance?.evidence_state==='manual_instance_association';}
 function displayParts(b){return isAssociated(b)||b.provenance?.evidence_state==='targeted_instance_refinement'||b.provenance?.cleanup_grouped?[b.geometry]:C.spatialParts(b.geometry,1);}
 function sameGrid(a,b){return a.type==='surface-voxels'&&b.type==='surface-voxels'&&Math.abs(a.cell-b.cell)<1e-10&&a.origin.every((v,i)=>Math.abs(v-b.origin[i])<1e-10);}
 function mergeRegions(regions){
  if(regions.length<2||regions.length>24)throw Error('一次请选择 2 至 24 个片段');
  const first=regions[0];if(new Set(regions.map(b=>b.id)).size!==regions.length)throw Error('不能重复关联同一区域');
  if(!regions.every(b=>B.validBox(b)&&b.category===first.category&&sameGrid(first.geometry||{},b.geometry||{})))throw Error('只能关联同类别、同一采样网格的可见片段；不同分辨率或已平移片段不能直接合并');
  const cells=new Map();for(const b of regions)for(const p of b.geometry.cells)cells.set(p.join(','),p.slice());
  if(cells.size>50000)throw Error('合并后超过 50000 个支持单元，未修改');
  // Flatten unchanged associations, but retain edited associations as snapshots.
  const sources=regions.flatMap(b=>canRestore(b)?restoreSources(b):[structuredClone(b)]);
  if(sources.length>128||JSON.stringify(sources).length>16000000)throw Error('关联溯源过大，请先导出备份并分组处理');
  const geometry={...structuredClone(first.geometry),cells:[...cells.values()]};
  return {...structuredClone(first),...C.bounds(geometry),geometry,status:'draft',provenance:{method:'manual_same_object_union_v1',evidence_state:'manual_instance_association',human_modified:true,qualification:false,coverage:'observed_only_completeness_unchecked',association_sources:sources,association_category:first.category,association_origin:geometry.origin.slice(),association_cell:geometry.cell,view_ids:[],observations:[]}};
 }
 function canRestore(b){const p=b?.provenance,g=b?.geometry;return isAssociated(b)&&Array.isArray(p.association_sources)&&p.association_sources.length>=2&&p.association_sources.length<=128&&g?.type==='surface-voxels'&&p.association_category===b.category&&g.cell===p.association_cell&&Array.isArray(p.association_origin)&&g.origin.every((v,i)=>v===p.association_origin[i]);}
 function restoreSources(b){
  if(!canRestore(b))throw Error('关联后类别或位置已变化，不能恢复旧片段；可撤销修改或拆分当前离散区域');
  const sources=structuredClone(b.provenance.association_sources);
  if(!sources.every(s=>B.validBox(s)&&s.category===b.category&&sameGrid(s.geometry||{},b.geometry)))throw Error('来源片段校验失败，未修改');
  const cells=new Set(sources.flatMap(s=>s.geometry.cells.map(p=>p.join(','))));
  if(cells.size!==b.geometry.cells.length||!b.geometry.cells.every(p=>cells.has(p.join(','))))throw Error('关联后的支持点已变化，不能恢复旧片段');
  return sources;
 }
 root.InstanceTools={orientedBounds,displayParts,isAssociated,mergeRegions,canRestore,restoreSources,sameGrid};
 if(typeof module!=='undefined')module.exports=root.InstanceTools;
})(globalThis);
