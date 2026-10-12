(function(root){
 'use strict';
 const C=CompletionCore;let state=null,running=false,stop=false,preview=null,viz={added:[],suspects:[]};
 const chosen=()=>{if(!state)return [];const excluded=new Set((state.audit?.flagged||[]).filter(f=>f.is_new?!$('completionIncludeDisputed').checked:$('completionTrim').checked).map(f=>f.key));return state.groups.filter(g=>g.enabled).flatMap(g=>{const cells=g.geometry.cells.filter(c=>!excluded.has(c.join(',')));if(!cells.length)return [];const geometry={...g.geometry,cells};return [{...g,geometry,...ObjectCore.bounds(geometry)}];});};
 const current=()=>state&&boxes.find(b=>b.id===state.base.id)===state.baseRef&&boxes.length===state.labelsSource.length&&boxes.every((b,i)=>b===state.labelsSource[i]);
 function refresh(){
  const base=boxes.find(b=>b.id===selected),why=C.eligibility(base),ready=!why&&!draftChanged&&!anchors.length&&!association&&!state&&!running&&!globalThis.CleanupUI?.active();
  $('completionTools').hidden=!!why;$('completionCurrent').disabled=!ready;$('completionTour').disabled=!ready;
  $('completionRestore').hidden=base?.provenance?.evidence_state!=='targeted_instance_refinement';$('completionRestore').disabled=running||!!state||draftChanged||!!globalThis.CleanupUI?.active()||!C.canRestore(base);
  $('completionPanel').hidden=!state;if(!state)return;
  $('completionTitle').textContent='补识别：'+boxName(state.base);
  $('completionProgress').textContent=state.progress;$('completionStop').hidden=!running;$('completionStop').disabled=!running;
  $('completionDiscard').disabled=running;$('completionReturn').disabled=running;
  const host=$('completionList');host.replaceChildren();for(const [i,g]of state.groups.entries()){
   const label=document.createElement('label');label.className='completionRow';const check=document.createElement('input');check.type='checkbox';check.checked=g.enabled;check.disabled=running;check.onchange=()=>{g.enabled=check.checked;refresh();};const text=document.createElement('span');text.textContent=`视角 ${g.view_number} · 片段 ${g.part_number} · ${g.new_count} 个新增单元`;label.append(check,text);host.append(label);
  }
  const suspects=new Set((state.audit?.flagged||[]).filter(f=>!f.is_new).map(f=>f.key)),held=(state.audit?.flagged||[]).filter(f=>f.is_new);$('completionTrim').disabled=running||!suspects.size;
  $('completionTrimLabel').textContent=`移除原标签中 ${suspects.size} 个有争议单元（默认保留）`;
  $('completionIncludeDisputed').disabled=running||!held.length;$('completionDisputedLabel').textContent=`仍加入 ${held.length} 个有争议的新增单元（默认隔离）`;
  let change={added:[],removed:[]},error=state.auditError||'';preview=null;
  try{preview=C.union(state.base,chosen(),$('completionTrim').checked?suspects:new Set());change=C.delta(state.base,preview);}catch(e){error=e.message;}
  viz={added:change.added,suspects:(state.audit?.flagged||[]).map(f=>f.key.split(',').map(Number))};
  $('completionStats').textContent=`原有 ${state.base.geometry.cells.length} 单元 · 新增 ${change.added.length} · 将移除 ${change.removed.length}。${error||'青色为新增，橙色为疑似背景；不是完整性百分比。'}`;
  $('completionStats').textContent+=` 隔离 ${$('completionIncludeDisputed').checked?0:held.length} 个新增争议点；单视点支持的新点 ${state.audit?.single_view_new_count||0} 个，不能视为已复核。`;
  if(!current())$('completionStats').textContent+=' 标签已变化，建议不可应用，请丢弃后重试。';
  $('completionApply').disabled=running||!current()||selected!==state.base.id||draftChanged||!!error||(!change.added.length&&!change.removed.length);
  const details=$('completionEvidence');details.replaceChildren();for(const [i,a]of state.analyses.entries()){const p=document.createElement('p');p.className='meta';p.textContent=`视角 ${i+1}：${a.reason}；有效锚点 ${a.visible_anchor_pixels}，匹配 ${a.matched_masks.length} 个掩膜，隔离 ${a.rejected.length} 个越界片段。`;details.append(p);}
  if(state.audit){const p=document.createElement('p');p.className='meta';p.textContent=`逐点复核 ${state.audit.checked_cells} 个单元：${state.audit.flagged.filter(f=>f.conflict).length} 个与其他类别的几何重叠，${state.audit.flagged.filter(f=>f.multiview_disagreement).length} 个多视点证据冲突。接触地面的椅轮也可能被标记，疑点不是删除结论。`;details.append(p);}
 }
 async function run(tour){
  if(globalThis.CleanupUI?.active()){msg('请先应用或取消点级清理');return;}
  if(running||ObjectUI.busy())return;const baseRef=boxes.find(b=>b.id===selected),why=C.eligibility(baseRef);
  if(why||draftChanged||anchors.length||association||state){msg(why||'请先保存修改，或结束已有的关联 / 补识别');return;}
  if(!meta||!gsEntity||loading){msg('请等待 3DGS 加载完成');return;}
  const saved={target:target.clone(),dist,yaw,pitch,view,clip:$('clipZ').value,review:$('reviewMode').checked,filter:$('filterEnabled').checked,color:$('semanticColor').checked};
  const disabled=[...document.querySelectorAll('button,input,select')].map(el=>[el,el.disabled]);
  state={base:clone(baseRef),baseRef,labelsSource:boxes.slice(),groups:[],analyses:[],frames:[],audit:null,progress:'准备选中物体的局部识别…'};running=true;stop=false;pressed.clear();$('completionTrim').checked=false;$('completionIncludeDisputed').checked=false;
  for(const [el]of disabled)el.disabled=true;refresh();
  let done=0,total=1;
  try{
   const status=await ObjectUI.request('/semantic/status');if(!status.instance_ready)throw Error('本地实例模型未就绪');
   const eye=cam.getPosition(),views=[null];
   if(tour){const plan=await ObjectUI.request('/semantic/tour');if(plan.scene_id!==meta.scene_id||JSON.stringify(plan.source_hashes)!==JSON.stringify(meta.source_hashes))throw Error('扫描视点与当前场景批次不匹配');views.push(...C.planViews(state.base,plan.positions,[eye.x,eye.y,eye.z]));}
   total=views.length;$('reviewMode').checked=false;$('filterEnabled').checked=false;$('semanticColor').checked=false;$('clipZ').value=$('clipZ').max;setView('gs');filterScene();
   const center=state.base.min.map((v,i)=>(v+state.base.max[i])/2);
   for(const position of views){if(stop)break;if(!current())throw Error('原标签已变化，本轮停止');
    if(position){cam.setPosition(...position);cam.lookAt(new pc.Vec3(...center),new pc.Vec3(0,0,1));updateCameraClip();}
    state.progress=`补识别 ${done+1}/${total}：先核对目标锚点，再做全图 + 局部识别。可停止；尚未改写标签。`;refresh();
    const frame=await ObjectUI.capture(1024);if(stop)break;
    const seeds=C.visibleSeeds(state.base,frame);let analysis;
    if(seeds.length<3)analysis={groups:[],rejected:[],positive:[],negative:[],matched_masks:[],visible_anchor_pixels:seeds.length,reason:'有效目标锚点不足，跳过本视角',pose_id:frame.pose_id,camera_position:frame.camera_position};
    else{
     const result=await ObjectUI.infer(frame,{confidence:Number($('objectConfidence').value)||.6,backend:'instance',detail:'target',target_roi:C.targetROI(state.base,frame)});
     if(stop)break;if(result.scene_id!==meta.scene_id)throw Error('返回的场景不匹配');analysis=C.analyze(state.base,frame,result);if(result.evidence_id)analysis.evidence_id=result.evidence_id;
    }
    state.analyses.push(analysis);const {image,...auditFrame}=frame;state.frames.push(auditFrame);done++;
    const old=new Set(state.base.geometry.cells.map(c=>c.join(',')));
    analysis.groups.forEach((g,i)=>{const added=g.geometry.cells.filter(c=>!old.has(c.join(','))).length;if(added&&state.groups.length<96)state.groups.push({...g,view_number:done,part_number:i+1,new_count:added,enabled:true});});refresh();
   }
   state.progress=`${stop?'已停止':'本轮结束'}：处理 ${done}/${total} 个视角。请对照新增点核查靠背、椅脚或显示器整体，再应用建议。`;
  }catch(e){state.progress='补识别中止：'+e.message+'；已生成的建议保留，原标签未改变。';}
  finally{
   try{state.audit=C.auditSupport(state.base,state.groups,state.frames,state.analyses,state.labelsSource);}catch(e){state.auditError='逐点复核未完成：'+e.message;}finally{state.frames=[];}
   target=saved.target;dist=saved.dist;yaw=saved.yaw;pitch=saved.pitch;cameraUpdate();setView(saved.view);$('clipZ').value=saved.clip;$('clipValue').textContent=topZ().toFixed(2)+' m';$('reviewMode').checked=saved.review;$('filterEnabled').checked=saved.filter;$('semanticColor').checked=saved.color;
   for(const [el,value]of disabled)el.disabled=value;running=false;refresh();ObjectUI.refresh();filterScene();msg(state.progress);
  }
 }
 function apply(){
  if(!state||running)return;if(!current()||selected!==state.base.id||draftChanged||anchors.length){msg('请回到未修改的原目标，或丢弃本轮建议后重新识别');return;}
  if(state.auditError||!state.audit){msg('逐点复核未完成，不能应用本轮建议');return;}
  try{const remove=$('completionTrim').checked?new Set(state.audit.flagged.filter(f=>!f.is_new).map(f=>f.key)):new Set(),next=C.build(state.base,chosen(),state.analyses,remove),change=C.delta(state.base,next);
   next.provenance.point_audit={...clone(state.audit),user_included_disputed_new:$('completionIncludeDisputed').checked,user_removed_disputed_old:$('completionTrim').checked};if(JSON.stringify(next).length>16000000)throw Error('含逐点证据的标签超过 16 MB，请分批整理');
   if(!confirm(`更新 ${boxName(state.base)}：增加 ${change.added.length} 个支持单元，移除 ${change.removed.length} 个支持单元？\n仍为人工标注 / 未复核，不代表整件物体已找全。保留补识别前的标签，可恢复或撤销。`))return;
   snapshot();boxes=boxes.map(b=>b.id===next.id?next:b);state=null;preview=null;const saved=persist();selectRegion(next,false);filterScene();refresh();if(saved)msg('已应用目标补识别，完整性仍待核查；可恢复补识别前或 Ctrl+Z 撤销。');
  }catch(e){msg('未应用：'+e.message);}
 }
 function discard(){if(running)return;state=null;preview=null;refresh();ObjectUI.refresh();filterScene();msg('已丢弃本轮补识别建议，已保存标签不变');}
 function restore(){const base=boxes.find(b=>b.id===selected);if(running||state||draftChanged||globalThis.CleanupUI?.active())return;try{const old=C.restore(base);if(!confirm('恢复到本次补识别之前的标签？其他物体不变，可撤销。'))return;if(boxes.some(b=>b.id!==base.id&&b.category===old.category&&b.sequence===old.sequence))old.sequence=B.nextSequence(boxes,old.category);snapshot();boxes=boxes.map(b=>b.id===base.id?old:b);const saved=persist();selectRegion(old,false);filterScene();refresh();if(saved)msg('已恢复补识别前的标签，可撤销');}catch(e){msg(e.message);}}
 function draw(){
  if(!state||running||!preview||selected!==state.base.id||!current())return;
  const add=viz.added,removed=viz.suspects,g=state.base.geometry,visible=globalThis.SupportDepth?SupportDepth.checker():()=>true;
  for(const [cells,color]of [[add,'#5fffe3'],[removed,'#ff954e']]){pen.fillStyle=color;const step=Math.max(1,Math.ceil(cells.length/2000));for(let i=0;i<cells.length;i+=step){const world=cells[i].map((v,k)=>g.origin[k]+(v+.5)*g.cell),p=project(world);if(p.z>0&&visible(world,g.cell))pen.fillRect(p.x-2,p.y-2,4,4);}}
  const corners=InstanceTools.orientedBounds(preview.geometry).corners.map(project);pen.strokeStyle='#5fffe3';pen.lineWidth=2;pen.setLineDash([3,5]);for(const [a,b]of [[0,1],[0,2],[0,4],[1,3],[1,5],[2,3],[2,6],[3,7],[4,5],[4,6],[5,7],[6,7]]){if(corners[a].z<=0||corners[b].z<=0)continue;pen.beginPath();pen.moveTo(corners[a].x,corners[a].y);pen.lineTo(corners[b].x,corners[b].y);pen.stroke();}pen.setLineDash([]);
 }
 root.CompletionUI={busy:()=>running,active:()=>!!state,refresh,draw,run,apply,discard,restore};
 $('completionCurrent').onclick=()=>run(false);$('completionTour').onclick=()=>run(true);$('completionStop').onclick=()=>{stop=true;state.progress='正在停止，等待当前本地推理结束；不会应用建议。';refresh();};$('completionApply').onclick=apply;$('completionDiscard').onclick=discard;$('completionTrim').onchange=refresh;$('completionRestore').onclick=restore;
 $('completionIncludeDisputed').onchange=refresh;
 $('completionReturn').onclick=()=>{if(!state||draftChanged||anchors.length)return;const b=boxes.find(b=>b.id===state.base.id);if(b)chooseRegion(b);};
 window.addEventListener('beforeunload',e=>{if(state){e.preventDefault();e.returnValue='';}});refresh();
})(globalThis);
