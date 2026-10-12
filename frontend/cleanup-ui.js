(function(root){
 'use strict';
 let state=null,brush='off',stroke=null,cached=null,cursor=null,cancelRequested=false;
 const current=()=>!!state&&boxes.length===state.labels.length&&boxes.every((b,i)=>b===state.labels[i]);
 const editable=()=>current()&&selected===state.base.id&&!draftChanged&&!anchors.length&&!association&&!globalThis.ObjectUI?.busy();
 const painting=()=>brush!=='off'&&editable()&&!cancelRequested;
 function refresh(){
  const b=boxes.find(b=>b.id===selected),voxel=b?.geometry?.type==='surface-voxels';
  $('cleanupStart').hidden=!voxel;$('cleanupStart').disabled=!!state||draftChanged||!!association||!!anchors.length||!!globalThis.CompletionUI?.active()||!!globalThis.ObjectUI?.busy();
  $('cleanupRestore').hidden=b?.provenance?.evidence_state!=='manual_background_cleanup';$('cleanupRestore').disabled=!!state||draftChanged||!!globalThis.CompletionUI?.active()||!!globalThis.ObjectUI?.busy()||!CleanupCore.canRestore(b);
  $('cleanupPanel').hidden=!state;$('cleanupCancelConfirm').hidden=!state||!cancelRequested;if(!state)return;
  $('cleanupTitle').textContent='清理溢出：'+boxName(state.base);
  $('cleanupStats').textContent=`原有 ${state.base.geometry.cells.length} 单元 · 橙色待删 ${state.remove.size} · 保留 ${state.base.geometry.cells.length-state.remove.size}。${editable()?'预览不会改写标签。':'请返回未修改的目标；标签发生变化时需取消后重开。'}`;
  $('cleanupMode').textContent=brush==='off'?'浏览检查：正常转动视角；选画笔后左键涂点。':`${brush==='erase'?'擦除':'恢复'}画笔：只处理当前 3DGS 视角未被遮挡的目标支持点。右键 / Esc 退出画笔，保留预览。`;
  const host=$('cleanupConflicts');host.replaceChildren();for(const group of state.groups){const label=document.createElement('label');label.className='check';const input=document.createElement('input');input.type='checkbox';input.checked=group.checked;input.disabled=!editable();input.onchange=()=>{group.checked=input.checked;};const span=document.createElement('span');span.textContent=`${group.title} · ${group.keys.length} 单元`;label.append(input,span);host.append(label);}
  if(!state.groups.length)host.textContent='没有找到已有标签冲突；这不代表没有溢出，可用画笔检查。';
  for(const id of ['cleanupErase','cleanupRecover','cleanupMark'])$(id).disabled=!editable()||cancelRequested;
  $('cleanupApply').disabled=cancelRequested||!editable()||!state.remove.size||state.remove.size===state.base.geometry.cells.length;
  $('cleanupUndo').disabled=!state.history.length;$('cleanupClear').disabled=!state.remove.size;$('undo').disabled=!state.history.length;
  $('cleanupErase').classList.toggle('active',brush==='erase');$('cleanupRecover').classList.toggle('active',brush==='restore');
 }
 function checkpoint(){state.history.push([...state.remove]);if(state.history.length>20)state.history.shift();}
 function start(){
  const b=boxes.find(b=>b.id===selected);if(state||!b||b.geometry?.type!=='surface-voxels'||draftChanged||anchors.length||association||globalThis.CompletionUI?.active()||globalThis.ObjectUI?.busy()){msg('请先保存修改，并结束关联或补识别，再清理选中物体');return;}
  const conflicts=CompletionCore.labelConflicts(b,boxes),groups=new Map();
  for(const [key,c]of conflicts){if(!groups.has(c.category))groups.set(c.category,{title:'与'+cat(c.category)[1]+'标签重叠',keys:[],checked:false});groups.get(c.category).keys.push(key);}
  const own=new Set(b.geometry.cells.map(c=>c.join(','))),disputed=(b.provenance?.point_audit?.flagged||[]).filter(f=>f.multiview_disagreement&&own.has(f.key)).map(f=>f.key);
  if(disputed.length)groups.set('views',{title:'补识别记录中的跨视角争议',keys:disputed,checked:false});
  state={base:clone(b),labels:boxes.slice(),remove:new Set(),history:[],groups:[...groups.values()]};selectionSerial++;brush='off';cached=null;refresh();$('cleanupPanel').scrollIntoView?.({block:'nearest'});globalThis.CompletionUI?.refresh();msg('先核查橙色待删点，再应用清理。地面接触点也可能是椅轮，不会自动删除。');
 }
 function setBrush(next){brush=brush===next?'off':next;stroke=null;cursor=null;cached=null;refresh();}
 function stopPainting(){brush='off';stroke=null;cursor=null;refresh();}
 function projected(){
  if(!editable()||view!=='gs'||!globalThis.SupportDepth?.ready())return null;
  const stamp=SupportDepth.stamp();if(cached?.stamp===stamp)return cached.points;
  const g=state.base.geometry,points=[],visible=SupportDepth.checker(true);for(const cell of g.cells){const world=cell.map((v,k)=>g.origin[k]+(v+.5)*g.cell);if(!visible(world,g.cell))continue;const p=project(world);if(p.z>0)points.push({key:cell.join(','),x:p.x,y:p.y});}
  cached={stamp,points};return points;
 }
 function begin(p){if(!painting())return;const points=projected();if(!points){msg('请停稳视角，等待 3DGS 遮挡深度就绪；点云模式或透视后方不能涂删。');return;}stroke={last:p,before:[...state.remove],changed:false};paint(p);}
 function paint(p){
  if(!stroke||!painting())return;const points=projected();if(!points){end();msg('视角深度已变化，已结束这一笔；已有预览保留。');return;}
  const keys=CleanupCore.brushKeys(points,stroke.last,p,Number($('cleanupRadius').value)||24);cursor=p;stroke.last=p;
  for(const key of keys){if(brush==='erase'){if(!state.remove.has(key)){state.remove.add(key);stroke.changed=true;}}else if(state.remove.delete(key))stroke.changed=true;}
  refresh();
 }
 function end(){if(stroke?.changed){state.history.push(stroke.before);if(state.history.length>20)state.history.shift();}stroke=null;refresh();}
 function mark(){if(!editable())return;end();const keys=state.groups.filter(g=>g.checked).flatMap(g=>g.keys);if(!keys.length){msg('先勾选要预览移除的冲突类别；这不是自动确认错误');return;}checkpoint();keys.forEach(k=>state.remove.add(k));refresh();}
 function undoStroke(){if(!state)return;stroke=null;if(state.history.length)state.remove=new Set(state.history.pop());refresh();msg('已撤回点级预览；已保存标签不变');}
 function clear(){if(!state||!state.remove.size)return;end();checkpoint();state.remove.clear();refresh();}
 function close(){state=null;brush='off';stroke=null;cached=null;cursor=null;cancelRequested=false;refresh();refreshList();globalThis.CompletionUI?.refresh();globalThis.ObjectUI?.refresh();}
 function cancel(){if(!state)return true;end();if(state.remove.size){stopPainting();cancelRequested=true;refresh();$('cleanupCancelConfirm').scrollIntoView?.({block:'nearest'});msg('请选择丢弃或保留清理预览；原标签尚未改变');return false;}close();msg('已取消清理，原标签未改变');return true;}
 function discardPreview(){if(!state||!cancelRequested)return;close();msg('已取消清理，原标签未改变');}
 function keepPreview(){cancelRequested=false;refresh();msg('已保留清理预览，可继续检查');}
 function escape(){if(brush!=='off'){stopPainting();msg('已退出画笔，清理预览保留；可转动视角继续检查');return;}cancel();}
 function apply(){
  if(cancelRequested){msg('请先选择丢弃或保留清理预览');return;}if(!editable()){msg('标签已变化或目标已切换，未应用清理');return;}end();
  try{const reasons=state.groups.filter(g=>g.keys.some(k=>state.remove.has(k))).map(g=>({title:g.title,overlap_is_not_ground_truth:true})),next=CleanupCore.clean(state.base,state.remove,reasons);
   if(!confirm(`从 ${boxName(state.base)} 移除 ${state.remove.size} 个橙色支持单元？\n只修改这个区域，保留清理前完整标签，可恢复或撤销。仍为未复核，不能据此宣称整物体准确。`))return;
   snapshot();boxes=boxes.map(b=>b.id===next.id?next:b);close();const saved=persist();selectRegion(next,false);filterScene();if(saved)msg('已清理所选支持点并重新拟合外框；可恢复清理前标签或 Ctrl+Z 撤销。');
  }catch(e){msg('未应用清理：'+e.message);}
 }
 function restore(){if(state||draftChanged||anchors.length||association||globalThis.ObjectUI?.busy()||globalThis.CompletionUI?.active())return;const b=boxes.find(b=>b.id===selected);try{const old=CleanupCore.restore(b);if(!confirm('恢复到本次点级清理之前的标签？其他区域不变，可撤销。'))return;if(boxes.some(x=>x.id!==b.id&&x.category===old.category&&x.sequence===old.sequence))old.sequence=B.nextSequence(boxes,old.category);snapshot();boxes=boxes.map(x=>x.id===b.id?old:x);const saved=persist();selectRegion(old,false);filterScene();if(saved)msg('已恢复清理前标签，可撤销');}catch(e){msg(e.message);}}
 function draw(){if(!state||!current()||selected!==state.base.id)return;const g=state.base.geometry,visible=SupportDepth.checker();pen.fillStyle='#ff954e';for(const key of state.remove){const world=key.split(',').map((v,k)=>g.origin[k]+(+v+.5)*g.cell);if(!visible(world,g.cell))continue;const p=project(world);if(p.z>0)pen.fillRect(p.x-2,p.y-2,4,4);}if(cursor&&painting()){pen.strokeStyle=brush==='erase'?'#ff954e':'#5fffe3';pen.lineWidth=1;pen.beginPath();pen.arc(...cursor,Number($('cleanupRadius').value)||24,0,Math.PI*2);pen.stroke();}}
 root.CleanupUI={active:()=>!!state,painting,refresh,start,setBrush,stopPainting,begin,paint,end,mark,undoStroke,clear,cancel,escape,apply,restore,draw};
 $('cleanupStart').onclick=start;$('cleanupRestore').onclick=restore;$('cleanupErase').onclick=()=>setBrush('erase');$('cleanupRecover').onclick=()=>setBrush('restore');$('cleanupBrowse').onclick=stopPainting;$('cleanupMark').onclick=mark;$('cleanupUndo').onclick=undoStroke;$('cleanupClear').onclick=clear;$('cleanupApply').onclick=apply;$('cleanupCancel').onclick=cancel;$('cleanupReturn').onclick=()=>{if(state&&!draftChanged&&!anchors.length){const b=boxes.find(b=>b.id===state.base.id);if(b)chooseRegion(b);}};
 $('cleanupDiscard').onclick=discardPreview;$('cleanupKeep').onclick=keepPreview;
 window.addEventListener('beforeunload',e=>{if(state?.remove.size){e.preventDefault();e.returnValue='';}});refresh();
})(globalThis);
