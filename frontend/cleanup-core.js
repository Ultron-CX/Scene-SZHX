(function(root){
 'use strict';
 const B=typeof module!=='undefined'?require('./boxes.js'):root.BoxCore,O=typeof module!=='undefined'?require('./object-core.js'):root.ObjectCore,C=typeof module!=='undefined'?require('./completion-core.js'):root.CompletionCore;
 function brushKeys(points,a,b,radius){if(!Number.isFinite(radius)||radius<2||radius>80)throw Error('画笔半径无效');const dx=b[0]-a[0],dy=b[1]-a[1],len=dx*dx+dy*dy;return points.filter(p=>{const t=len?Math.max(0,Math.min(1,((p.x-a[0])*dx+(p.y-a[1])*dy)/len)):0;return (p.x-a[0]-t*dx)**2+(p.y-a[1]-t*dy)**2<=radius*radius;}).map(p=>p.key);}
 function clean(base,keys,reasons=[]){
  if(!B.validBox(base)||base.geometry?.type!=='surface-voxels')throw Error('只能清理已保存的可见表面单元');
  const original=new Set(base.geometry.cells.map(c=>c.join(',')));if([...keys].some(k=>!original.has(k)))throw Error('包含不属于当前物体的点，未修改');if(!keys.size)throw Error('尚未标记要移除的点');
  const cells=base.geometry.cells.filter(c=>!keys.has(c.join(',')));if(!cells.length)throw Error('不能移除全部支持点；如需删除物体请用区域删除');
  const geometry={...structuredClone(base.geometry),cells:cells.map(c=>c.slice())},next={...structuredClone(base),geometry,...O.bounds(geometry),status:'draft'};
  next.provenance={method:'user_confirmed_point_cleanup_v1',evidence_state:'manual_background_cleanup',human_modified:true,qualification:false,coverage:'observed_only_completeness_unchecked',cleanup_source:structuredClone(base),cleanup_signature:C.signature(geometry),cleanup_category:base.category,cleanup_grouped:['targeted_instance_refinement','manual_instance_association'].includes(base.provenance?.evidence_state)||!!base.provenance?.cleanup_grouped,removed_keys:[...keys],reviewed_reasons:structuredClone(reasons),view_ids:[],observations:[]};
  if(JSON.stringify(next).length>16000000)throw Error('清理溯源超过 16 MB，请先导出备份后分批整理');if(!B.validBox(next))throw Error('清理后几何无效');return next;
 }
 function canRestore(b){return b?.geometry?.type==='surface-voxels'&&b.provenance?.evidence_state==='manual_background_cleanup'&&b.category===b.provenance.cleanup_category&&C.signature(b.geometry)===b.provenance.cleanup_signature&&B.validBox(b.provenance.cleanup_source);}
 function restore(b){if(!canRestore(b))throw Error('清理后区域已修改，不能直接恢复旧位置的点');const old=structuredClone(b.provenance.cleanup_source);old.id=b.id;return old;}
 root.CleanupCore={brushKeys,clean,canRestore,restore};if(typeof module!=='undefined')module.exports=root.CleanupCore;
})(globalThis);
