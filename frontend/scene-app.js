if(globalThis.location?.pathname.startsWith('/r1-sample/')&&!globalThis.SceneScope)throw Error('隔离模块未加载，停止以保护正式草稿');
'use strict';
const $=id=>document.getElementById(id), B=BoxCore, canvas=$('sceneCanvas'), overlay=$('overlay'), pen=overlay.getContext('2d');
let app,cam,meta,points,vertices,indices,pointEntity,pointMesh,gsEntity,gsAsset,gsOriginal,boxes=[],undoStack=[],anchors=[],draft=null,selected=null,mode='browse',view='gs',target,dist=45,yaw=.5,pitch=.85,drag=null,dirty=false,loading=false;
let positions,rgba,pointVisible,storageKey,filterTimer,proposals=[],draftChanged=false,legacyProposals=[],manualActive=false;
const pressed=new Set();let labelTargets=[];
let surfacePicker=null,pickBusy=false,selectionSerial=0;
let association=null;
const V=(a)=>new pc.Vec3(...a), msg=s=>{$('message').textContent=s;}, clone=x=>structuredClone(x);
const cat=id=>B.categories.find(c=>c[0]===id), boxName=b=>`${cat(b.category)[1]}-${String(b.sequence).padStart(3,'0')}`;
const currentFilter=()=>$('reviewMode').checked&&$('filterEnabled').checked?$('filter').value:'all';
const topZ=()=>+$('clipZ').value;
// The workspace, not the window or the previous canvas size, owns both layers.
function syncViewportSize(){
  if(!app)return;
  const viewport=$('viewport'),w=viewport.clientWidth,h=viewport.clientHeight;
  if(w<1||h<1)return; // Hidden tabs must not collapse the drawing buffer.
  app.setCanvasFillMode(pc.FILLMODE_NONE,w,h);
  if(overlay.width!==w)overlay.width=w;
  if(overlay.height!==h)overlay.height=h;
  if(cam)cam.camera.aspectRatio=w/h;
}
let resizePending=false;
function scheduleViewportResize(){if(resizePending)return;resizePending=true;requestAnimationFrame(()=>{resizePending=false;syncViewportSize();});}
function project(p){const out=new pc.Vec3();cam.camera.worldToScreen(V(p),out);return out;}
function updateCameraClip(){if(!cam?.camera||!meta?.bounds)return;const [lo,hi]=meta.bounds,center=lo.map((x,i)=>(x+hi[i])/2),radius=Math.hypot(...hi.map((x,i)=>(x-lo[i])/2)),p=cam.getPosition();cam.camera.nearClip=.01;cam.camera.farClip=Math.max(500,Math.hypot(p.x-center[0],p.y-center[1],p.z-center[2])+radius+10);}
function cameraUpdate(){if(!cam)return;cam.setPosition(target.x+dist*Math.cos(pitch)*Math.cos(yaw),target.y+dist*Math.cos(pitch)*Math.sin(yaw),target.z+dist*Math.sin(pitch));cam.lookAt(target,new pc.Vec3(0,0,1));updateCameraClip();}
// First-person look: move the virtual target around the eye, never orbit the eye.
function lookAround(dx,dy){
 if(!cam||!target||globalThis.ObjectUI?.busy()||!Number.isFinite(dx)||!Number.isFinite(dy))return;
 const eye=cam.getPosition().clone();
 yaw-=dx*.003;yaw=Math.atan2(Math.sin(yaw),Math.cos(yaw));
 pitch=Math.min(1.569,Math.max(-1.569,pitch+dy*.003));
 target.set(eye.x-dist*Math.cos(pitch)*Math.cos(yaw),eye.y-dist*Math.cos(pitch)*Math.sin(yaw),eye.z-dist*Math.sin(pitch));
 cam.lookAt(target,new pc.Vec3(0,0,1));updateCameraClip();
}
// Screen-plane translation uses stable scene units, not the obsolete orbit radius.
// Bound individual pointer spikes; Alt is precise, Shift faster, Alt takes priority.
function panCamera(dx,dy,e={}){
 if(!cam||!target||globalThis.ObjectUI?.busy()||!Number.isFinite(dx)||!Number.isFinite(dy))return;
 const length=Math.hypot(dx,dy);if(!length)return;
 const speed=e.altKey?.1:e.shiftKey?3:1,scale=.003*speed*Math.min(1,100/length);
 target.add(cam.right.clone().mulScalar(-dx*scale)).add(cam.up.clone().mulScalar(dy*scale));cameraUpdate();
}
function home(){const [lo,hi]=meta.bounds;target=V(lo.map((x,i)=>(x+hi[i])/2));dist=Math.max(...hi.map((x,i)=>x-lo[i]))*1.05;pitch=.9;yaw=.55;cameraUpdate();}
function fitBox(b){if(!b)return;target=V(b.min.map((x,i)=>(x+b.max[i])/2));dist=Math.max(1,...b.max.map((x,i)=>x-b.min[i]))*2.3;cameraUpdate();}
function snapshot(){const state=clone(boxes);globalThis.ProjectUI?.captureUndo?.(state);undoStack.push(state);if(undoStack.length>30)undoStack.shift();}
function undoRegionEdit(){if(!undoStack.length)return;const state=undoStack.at(-1);if(globalThis.ProjectUI?.restoreUndo){globalThis.ProjectUI.restoreUndo(state);return;}boxes=undoStack.pop();persist();resetDraft();filterScene();msg('已撤销');}
function persist(){dirty=true;globalThis.ProjectUI?.regionsChanged();try{(globalThis.SceneScope?.storage||localStorage).setItem(storageKey,JSON.stringify(B.save(boxes,meta)));return true;}catch(e){msg('自动草稿保存失败，请立即导出备份：'+e.message);return false;}}
function instanceUpdate(){const existing=boxes.find(b=>b.id===selected),category=$('category').value;const sequence=existing&&existing.category===category?existing.sequence:B.nextSequence(boxes,category);$('instance').value=boxName({category,sequence});}
function fields(){for(let i=0;i<3;i++){ $('center'+i).value=draft?((draft.min[i]+draft.max[i])/2).toFixed(3):'';$('size'+i).value=draft?(draft.max[i]-draft.min[i]).toFixed(3):'';$('size'+i).disabled=!!draft?.geometry;}$('anchorCount').textContent=anchors.length?`${anchors.length} 个选点 · 点击生成区域`:draft?'已生成空间区域':'未选点';$('editing').textContent=selected?'编辑已有区域':'新建';$('delete').disabled=!selected&&!draft&&!anchors.length;$('currentSelection').textContent=selected?'已选：'+boxName(boxes.find(b=>b.id===selected)):draft?'当前为未保存草稿':'尚未选择区域';$('thickness').disabled=draft?.geometry?.type==='surface-voxels';if(draft?.geometry&&draft.geometry.type!=='surface-voxels')$('thickness').value=(draft.geometry.high-draft.geometry.low).toFixed(3);instanceUpdate();vertexFields();candidateGeometryInfo();}
function resetDraft(){selectionSerial++;association=null;selected=null;draft=null;anchors=[];draftChanged=false;manualActive=false;setMode('browse');fields();refreshList();}
function workflow(){if(!$('editorPanel'))return;$('manualPanel').hidden=!manualActive;$('editorPanel').hidden=!draft&&!manualActive;$('idleHint').hidden=!!draft||manualActive;globalThis.WorkbenchUX?.refresh();}
function selectRegion(b,focus=true){selectionSerial++;selected=b.id;draft=clone(b);anchors=[];draftChanged=false;$('category').value=b.category;$('status').value=b.status;setMode('browse');fields();refreshList();if(focus)fitBox(b);}
function chooseRegion(b,focus=true){if(association){toggleAssociation(b);return;}if(draftChanged||anchors.length){msg('请先保存或取消当前草稿，再选择其他区域');return;}selectRegion(b,focus);}
function vertexFields(){workflow();const host=$('vertexEditor');host.replaceChildren();if(!draft?.geometry)return;if(draft.geometry.type==='surface-voxels'){host.textContent=`自动物体候选：仅标识有语义和深度支持的 ${Math.round(draft.geometry.cell*100)} cm 单元，不填满外框（单元大小不是定位精度）。可修改类别、平移或删除；多余支持点可用“清理溢出点”，缺失部分可补识别。`;return;}if(draft.geometry.type==='surface-mask'){host.textContent='连续结构：边界来自网格覆盖，含空缺。支持平移与厚度调整；需补画时请用“补充标注”。';return;}const heading=document.createElement('p');heading.className='meta';heading.textContent='轮廓顶点（局部 U / V，米）';host.append(heading);draft.geometry.polygon.forEach((p,i)=>{const row=document.createElement('div');row.className='vertexRow';const label=document.createElement('span');label.textContent=i+1;row.append(label);for(let k=0;k<2;k++){const input=document.createElement('input');input.type='number';input.step='.05';input.value=p[k].toFixed(3);input.setAttribute('aria-label',`顶点 ${i+1} ${k?'V':'U'}`);input.onchange=()=>{const g=clone(draft.geometry);g.polygon[i][k]=Number(input.value);if(!B.polygonValid(g.polygon)){msg('此修改会使轮廓自交或退化，未应用');input.value=p[k];return;}draft.geometry=g;Object.assign(draft,B.bounds(g));draftChanged=true;fields();};row.append(input);}const del=document.createElement('button');del.textContent='−';del.title='删除此轮廓顶点';del.disabled=draft.geometry.polygon.length<=3;del.onclick=()=>{const g=clone(draft.geometry);g.polygon.splice(i,1);if(!B.polygonValid(g.polygon)){msg('删除后轮廓无效，未应用');return;}draft.geometry=g;Object.assign(draft,B.bounds(g));draftChanged=true;fields();};row.append(del);host.append(row);});}
function finishShape(){try{draft=$('shape').value==='surface'?B.fromSurfacePoints(anchors,Number($('thickness').value)):B.fromContour(anchors,$('plane').value,Number($('thickness').value),$('shape').value==='rectangle');draftChanged=true;fields();msg($('shape').value==='surface'?'已按表面取点生成自动定向包络；这不是物体分割，未选到的背面需补点':'区域已生成；可调整轮廓、中心和厚度，然后保存');}catch(e){draft=null;fields();msg(e.message);}}
function movement(dt){if(globalThis.ObjectUI?.busy())return;if(!cam||!target||!pressed.size||['INPUT','SELECT','TEXTAREA'].includes(document.activeElement.tagName)||document.activeElement.isContentEditable)return;const direction=new pc.Vec3(),forward=cam.forward.clone().normalize(),right=cam.right.clone().normalize();if(pressed.has('KeyW'))direction.add(forward);if(pressed.has('KeyS'))direction.sub(forward);if(pressed.has('KeyD'))direction.add(right);if(pressed.has('KeyA'))direction.sub(right);if(pressed.has('KeyE'))direction.z+=1;if(pressed.has('KeyQ'))direction.z-=1;if(direction.length()){direction.normalize().mulScalar(Math.min(dt,.05)*(pressed.has('ShiftLeft')||pressed.has('ShiftRight')?8:2.5));target.add(direction);cameraUpdate();}}
function wheelMove(e){e.preventDefault();if(!cam||!target||globalThis.ObjectUI?.busy())return;const units=e.deltaMode===1?16:e.deltaMode===2?Math.max(1,overlay.clientHeight):1,pixels=e.deltaY*units;if(!Number.isFinite(pixels)||!pixels)return;const speed=e.altKey?.1:e.shiftKey?4:1;target.add(cam.forward.clone().normalize().mulScalar(-pixels*.0025*speed));cameraUpdate();}
function deleteCurrent(){if(association){msg('请先完成或取消片段关联');return;}if(selected){const old=boxes.find(b=>b.id===selected);snapshot();boxes=boxes.filter(b=>b.id!==selected);const saved=persist();resetDraft();filterScene();if(saved)msg('已删除 '+boxName(old)+'；可 Ctrl+Z 撤销');}else if(draft||anchors.length){resetDraft();msg('已删除未保存草稿');}else msg('先点击场景标签或列表选择区域');}
function untouchedLegacy(b){const old=legacyProposals.find(p=>p.id===(b.provenance?.proposal_id||b.id));return !!old&&!b.provenance?.human_modified&&b.status==='proposed'&&b.category===old.category&&JSON.stringify(b.geometry)===JSON.stringify(old.geometry)&&JSON.stringify(b.min)===JSON.stringify(old.min)&&JSON.stringify(b.max)===JSON.stringify(old.max);}
function proposalUpdate(){const kept=boxes.filter(b=>!untouchedLegacy(b)),existing=new Set(kept.map(b=>b.provenance?.proposal_id||b.id));return {kept,fresh:proposals.filter(p=>!existing.has(p.id)),remove:boxes.length-kept.length};}
function commitProposals(){const update=proposalUpdate();if(!update.fresh.length&&!update.remove){$('upgradePanel').hidden=true;msg('已是当前结构结果，没有重复加入');return;}snapshot();boxes=update.kept;for(const p of update.fresh){const b=clone(p);b.provenance={...b.provenance,proposal_id:p.id};b.sequence=B.nextSequence(boxes,b.category);if(boxes.some(x=>x.id===b.id))b.id=crypto.randomUUID();boxes.push(b);}persist();resetDraft();$('upgradePanel').hidden=true;filterScene();msg(`已加入 ${update.fresh.length} 个结构，替换 ${update.remove} 个未修改的旧候选；人工标签保留，可撤销`);}
function addProposals(){if(!proposals.length){msg('结构候选尚未就绪');return;}const update=proposalUpdate();if(update.remove){$('upgradeText').textContent=`将替换 ${update.remove} 个未修改的旧候选，保留 ${update.kept.length} 个已有区域，加入 ${update.fresh.length} 个新结构。此操作可撤销。`;$('upgradePanel').hidden=false;}else commitProposals();}
function setMode(m){mode=m;manualActive=m==='pick';$('pick').classList.toggle('active',m==='pick');$('browse').classList.toggle('active',m==='browse');overlay.style.cursor=m==='pick'?'crosshair':'grab';workflow();}
function regionKind(b){return b.geometry?.type==='surface-voxels'?(b.provenance?.evidence_state==='manual_background_cleanup'?'人工点级清理 · 完整性待核查':b.provenance?.evidence_state==='targeted_instance_refinement'?'人工确认补识别 · 完整性待核查':globalThis.InstanceTools?.isAssociated(b)?'人工关联 · 完整性待核查':'可见片段 · 非完整物体'):'空间区域';}
function refreshList(){const f=currentFilter();$('count').textContent=boxes.length;$('boxList').replaceChildren();for(const b of boxes){if(f!=='all'&&f!==b.category)continue;const button=document.createElement('button');button.className='boxItem'+(selected===b.id?' selected':'');button.style.setProperty('--category',cat(b.category)[2]);button.textContent=boxName(b)+' · '+({reviewed:'已复核',proposed:'自动候选',draft:'人工标注'}[b.status])+' · '+regionKind(b);button.onclick=()=>chooseRegion(b);$('boxList').append(button);}if(!$('boxList').children.length)$('boxList').textContent='当前筛选下暂无区域';$('undo').disabled=!undoStack.length;$('categorySummary').replaceChildren();for(const c of B.categories){const count=boxes.filter(b=>b.category===c[0]).length;if(!count)continue;const chip=document.createElement('button');chip.className='categoryChip';chip.style.setProperty('--category',c[2]);chip.textContent=c[1]+' '+count;chip.title='已保存区域数，包含片段；不是完整物体数量';chip.onclick=()=>{$('reviewMode').checked=true;$('filter').value=c[0];$('filterEnabled').checked=true;$('semanticColor').checked=true;$('reviewMode').onchange();};$('categorySummary').append(chip);}}
function saveDraft(){if(association){msg('请先完成或取消片段关联');return;}if(!draft){msg('先完成轮廓并点击“生成区域”');return;}const category=$('category').value,old=boxes.find(b=>b.id===selected);const b={...clone(draft),id:old?.id||crypto.randomUUID(),category,sequence:old?.category===category?old.sequence:B.nextSequence(boxes,category),status:$('status').value};if(old?.provenance)b.provenance={...b.provenance,human_modified:true};if(!B.validBox(b)){msg('请检查区域：轮廓须有效，厚度至少 0.01 米');return;}snapshot();if(old)boxes=boxes.map(x=>x.id===old.id?b:x);else boxes.push(b);const saved=persist();selectRegion(b,false);filterScene();if(saved)msg(`已保存 ${boxName(b)}；可继续点击场景修改其他区域`);}
function editDimensions(){if(!draft)return;const c=[0,1,2].map(i=>Number($('center'+i).value)),s=[0,1,2].map(i=>Number($('size'+i).value));if([...c,...s].some(x=>!Number.isFinite(x))||s.some(x=>x<.01)){msg('请输入有效坐标和至少 0.01 米的尺寸');return;}if(draft.geometry){const old=draft.min.map((x,i)=>(x+draft.max[i])/2);draft.geometry.origin=draft.geometry.origin.map((x,i)=>x+c[i]-old[i]);Object.assign(draft,B.bounds(draft.geometry));}else{draft.min=c.map((x,i)=>x-s[i]/2);draft.max=c.map((x,i)=>x+s[i]/2);}anchors=[];draftChanged=true;fields();msg('已调整位置 / 尺寸，请保存');}
function drawOverlay(){const w=overlay.clientWidth,h=overlay.clientHeight;if(overlay.width!==w||overlay.height!==h){overlay.width=w;overlay.height=h;}pen.clearRect(0,0,w,h);if(!cam)return;
 globalThis.SupportDepth?.update();
 globalThis.ProjectUI?.drawZoneBoundary?.(pen,project);
 const supportVisible=globalThis.SupportDepth?SupportDepth.checker():()=>true;
 const edges=[[0,1],[0,2],[0,4],[1,3],[1,5],[2,3],[2,6],[3,7],[4,5],[4,6],[5,7],[6,7]],f=currentFilter();labelTargets=[];
 function box(b,isDraft,withLabel=true,oriented=null){let corners,lines;if(b.geometry&&b.geometry.type!=='surface-voxels'){const n=b.geometry.polygon.length;corners=B.prismVertices(b.geometry).map(project);lines=[];for(let i=0;i<n;i++)lines.push([i,(i+1)%n],[i+n,(i+1)%n+n],[i,i+n]);}else{corners=(oriented?.corners||Array.from({length:8},(_,k)=>[0,1,2].map(i=>(k&(1<<i))?b.max[i]:b.min[i]))).map(project);lines=edges;}
  const color=cat(isDraft?$('category').value:b.category)[2];pen.strokeStyle=color;pen.lineWidth=isDraft?3:association?.ids.has(b.id)?2.5:1.5;pen.setLineDash((isDraft&&draftChanged)||b.status==='proposed'?[7,4]:[]);
  if(b.geometry?.type!=='surface-voxels'||$('boundsMode')?.value!=='support')for(const [a,z] of lines){const p=corners[a],q=corners[z];if(p.z<=0||q.z<=0)continue;pen.beginPath();pen.moveTo(p.x,p.y);pen.lineTo(q.x,q.y);pen.stroke();}
  pen.setLineDash([]);const p=corners[corners.length-1];if(withLabel&&p.z>0){const type=b.geometry?.type==='surface-voxels'?(b.provenance?.evidence_state==='targeted_instance_refinement'?' · 补识别待核查':globalThis.InstanceTools?.isAssociated(b)?' · 关联物体':' · 可见片段'):'';const text=(isDraft?$('instance').value+(draftChanged?' · 未保存':' · 已选中'):boxName(b))+type;pen.font='14px Microsoft YaHei, sans-serif';const tw=pen.measureText(text).width;pen.fillStyle='#101c2dea';pen.fillRect(p.x-4,p.y-23,tw+12,24);pen.fillStyle=color;pen.fillText(text,p.x+2,p.y-6);if(b.id)labelTargets.push({id:b.id,x:p.x-4,y:p.y-23,w:tw+12,h:24});}
 }
 function region(b,isDraft){if(b.geometry?.type==='surface-voxels'&&globalThis.ObjectCore){
  const parts=globalThis.InstanceTools?InstanceTools.displayParts(b):ObjectCore.spatialParts(b.geometry,1);
  for(let i=0;i<parts.length;i++){const frame=globalThis.InstanceTools&&$('boundsMode')?.value!=='axis'?InstanceTools.orientedBounds(parts[i]):null;box({...b,...ObjectCore.bounds(parts[i]),geometry:parts[i]},isDraft,i===0,frame);}
  if(isDraft||association?.ids.has(b.id)||$('boundsMode')?.value==='support'){const g=b.geometry,step=isDraft&&globalThis.CleanupUI?.active()?1:Math.max(1,Math.ceil(g.cells.length/(isDraft?1500:400)));pen.fillStyle=cat(isDraft?$('category').value:b.category)[2];for(let i=0;i<g.cells.length;i+=step){const world=g.cells[i].map((v,k)=>g.origin[k]+(v+.5)*g.cell),p=project(world);if(p.z>0&&supportVisible(world,g.cell))pen.fillRect(p.x-1.5,p.y-1.5,3,3);}}return;
 }if(b.geometry?.type!=='surface-mask'){box(b,isDraft);return;}const g=b.geometry;pen.strokeStyle=cat(isDraft?$('category').value:b.category)[2];pen.lineWidth=isDraft?2:1;pen.setLineDash([]);for(const edge of g.edges){const [a,c]=edge.map(p=>project(g.origin.map((o,i)=>o+p[0]*g.u[i]+p[1]*g.v[i])));if(a.z<=0||c.z<=0)continue;pen.beginPath();pen.moveTo(a.x,a.y);pen.lineTo(c.x,c.y);pen.stroke();}const p=project(b.min.map((x,i)=>(x+b.max[i])/2));if(p.z>0){const text=isDraft?$('instance').value:boxName(b);pen.font='14px Microsoft YaHei';const width=pen.measureText(text).width+12;pen.fillStyle='#102130ef';pen.fillRect(p.x,p.y-24,width,24);pen.fillStyle=pen.strokeStyle;pen.fillText(text,p.x+5,p.y-6);if(b.id)labelTargets.push({id:b.id,x:p.x,y:p.y-24,w:width,h:24});}}
 for(const b of boxes)if(b.id!==selected&&(association?.ids.has(b.id)||($('reviewMode').checked&&$('showBoundaries').checked&&(f==='all'||f===b.category))))region(b,false);if(draft)region(draft,true);if(globalThis.ObjectUI?.visible())for(const b of ObjectUI.pending())region(b,false);
 globalThis.CompletionUI?.draw();
 globalThis.CleanupUI?.draw();
 let previous=null;for(const a of anchors){const p=project(a);if(p.z<=0){previous=null;continue;}if(previous){pen.strokeStyle='#ffffff';pen.beginPath();pen.moveTo(previous.x,previous.y);pen.lineTo(p.x,p.y);pen.stroke();}pen.fillStyle='#fff';pen.beginPath();pen.arc(p.x,p.y,5,0,Math.PI*2);pen.fill();pen.strokeStyle='#0b1118';pen.stroke();previous=p;}
 $('viewBadge').textContent=`${view==='gs'?'3DGS 实景':'LAS 彩色点云 · 1:100 预览'} / ${mode==='pick'?'点选物体边界':'浏览'}${f==='all'?'':' / '+(f==='unknown'?'未覆盖区域':cat(f)[1])}`;
}
function filterScene(){globalThis.SupportDepth?.invalidate();if(!points)return;const f=currentFilter(),zmax=topZ(),semantic=($('reviewMode').checked&&$('semanticColor').checked)||globalThis.ObjectUI?.visible(),match=B.compile(globalThis.ObjectUI?.visible()?[...boxes,...ObjectUI.pending()]:boxes);
 const colors=new Map(B.categories.map(c=>[c[0],[1,3,5].map(i=>parseInt(c[2].slice(i,i+2),16))]));
 for(let i=0;i<points.length/6;i++){const j=i*6,result=match(points[j],points[j+1],points[j+2],f),show=points[j+2]<=zmax&&result.visible;pointVisible[i]=show?1:0;const col=semantic&&result.box?colors.get(result.box.category):null;for(let k=0;k<3;k++)rgba[i*4+k]=col?col[k]:Math.round(points[j+3+k]*255);rgba[i*4+3]=show?255:0;}
 pointMesh.setColors32(rgba);pointMesh.update(pc.PRIMITIVE_POINTS,false);
 if(gsAsset&&gsOriginal){const data=gsAsset.resource.gsplatData,x=data.getProp('x'),y=data.getProp('y'),z=data.getProp('z'),opacity=data.getProp('opacity'),dc=[0,1,2].map(k=>data.getProp('f_dc_'+k));for(let i=0;i<x.length;i++){const result=match(x[i],y[i],z[i],f);opacity[i]=z[i]<=zmax&&result.visible?gsOriginal.opacity[i]:-100;const col=semantic&&result.box?colors.get(result.box.category):null;for(let k=0;k<3;k++)dc[k][i]=col?(col[k]/255-.5)/.28209479177387814:gsOriginal.dc[k][i];}gsAsset.resource.updateColorData(data);}
 refreshList();
}
function scheduleFilter(){clearTimeout(filterTimer);filterTimer=setTimeout(filterScene,120);}
async function loadGS(quality){if(loading)return;loading=true;$('quality').disabled=true;msg('正在加载全场景 3DGS，可先切换点云标注…');const asset=new pc.Asset('K2-'+quality,'gsplat',{url:'/scene-'+quality+'.ply'},{reorder:false});asset.on('progress',(n,total)=>{if(total)msg('3DGS 加载 '+Math.round(n/total*100)+'%');});app.assets.add(asset);
 try{await new Promise((resolve,reject)=>{asset.once('load',resolve);asset.once('error',reject);app.assets.load(asset);});const entity=new pc.Entity('K2 visual');entity.addComponent('gsplat',{asset,unified:false});app.root.addChild(entity);entity.enabled=view==='gs';const oldEntity=gsEntity,oldAsset=gsAsset;gsEntity=entity;gsAsset=asset;const d=asset.resource.gsplatData;gsOriginal={opacity:d.getProp('opacity').slice(),dc:[0,1,2].map(k=>d.getProp('f_dc_'+k).slice())};if(oldEntity)oldEntity.destroy();if(oldAsset){oldAsset.unload();app.assets.remove(oldAsset);}filterScene();msg('全场景已加载。补充标注默认逐点读取 3DGS 表面深度，可转动视角补点。');}catch(e){app.assets.remove(asset);msg('3DGS 加载失败：'+e+'；点云模式仍可使用');if(!gsEntity)setView('points');}finally{loading=false;$('quality').disabled=false;}}
function setView(v){globalThis.SupportDepth?.invalidate();view=v;if(gsEntity)gsEntity.enabled=v==='gs';if(pointEntity)pointEntity.enabled=v==='points';$('gsMode').classList.toggle('active',v==='gs');$('pointMode').classList.toggle('active',v==='points');}
// Moller-Trumbore, nearest mesh intersection. Mesh is a picking aid, never rendered as GS.
function pickMesh(origin,dir){let nearest=Infinity,answer=null;const f=currentFilter();for(let k=0;k<indices.length;k+=3){const ai=indices[k]*3,bi=indices[k+1]*3,ci=indices[k+2]*3;const ax=vertices[ai],ay=vertices[ai+1],az=vertices[ai+2],ex=vertices[bi]-ax,ey=vertices[bi+1]-ay,ez=vertices[bi+2]-az,fx=vertices[ci]-ax,fy=vertices[ci+1]-ay,fz=vertices[ci+2]-az;const px=dir.y*fz-dir.z*fy,py=dir.z*fx-dir.x*fz,pz=dir.x*fy-dir.y*fx,det=ex*px+ey*py+ez*pz;if(Math.abs(det)<1e-9)continue;const tx=origin.x-ax,ty=origin.y-ay,tz=origin.z-az,u=(tx*px+ty*py+tz*pz)/det;if(u<0||u>1)continue;const qx=ty*ez-tz*ey,qy=tz*ex-tx*ez,qz=tx*ey-ty*ex,v=(dir.x*qx+dir.y*qy+dir.z*qz)/det;if(v<0||u+v>1)continue;const t=(fx*qx+fy*qy+fz*qz)/det;if(t<=.05||t>=nearest)continue;const p=[origin.x+t*dir.x,origin.y+t*dir.y,origin.z+t*dir.z];if(p[2]>topZ()||!B.match(boxes,...p,f).visible)continue;nearest=t;answer=p;}return answer;}
function pickPoint(origin,dir){let best=Infinity,answer=null;const pixelAngle=2*Math.tan(cam.camera.fov*Math.PI/360)*7/overlay.clientHeight;for(let i=0;i<pointVisible.length;i++){if(!pointVisible[i])continue;const j=i*6,dx=points[j]-origin.x,dy=points[j+1]-origin.y,dz=points[j+2]-origin.z,t=dx*dir.x+dy*dir.y+dz*dir.z;if(t<=.05||t>=best)continue;const cross2=dx*dx+dy*dy+dz*dz-t*t;if(cross2>(t*pixelAngle)**2)continue;best=t;answer=[points[j],points[j+1],points[j+2]];}return answer;}
function hit(x,y){const far=cam.camera.screenToWorld(x,y,cam.camera.farClip),origin=cam.getPosition().clone(),dir=far.sub(origin).normalize();return view==='gs'?pickMesh(origin,dir):pickPoint(origin,dir);}
async function renderedSurfaceHit(x,y){
  if(view!=='gs'){const p=hit(x,y);if(!p)throw Error('此处没有点云采样点；请放大后重新点击');return p;}
  if(!gsEntity||loading)throw Error('3DGS 尚未加载完成，请稍后取点');
  const w=canvas.clientWidth,h=canvas.clientHeight;
  if(x<0||y<0||x>=w||y>=h)throw Error('点击位置超出视图区');
  if(!surfacePicker)surfacePicker=new pc.Picker(app,w,h,true);
  surfacePicker.resize(w,h);
  // Only the visible GS entity is enabled in this mode. No proxy mesh fallback.
  surfacePicker.prepare(cam.camera,app.scene);
  const p=await surfacePicker.getWorldPointAsync(Math.floor(x),Math.floor(y));
  if(!p||![p.x,p.y,p.z].every(Number.isFinite))throw Error('此处没有有效 3DGS 深度；请放大、提高画质或换视角，不会改用背景网格');
  if(p.z>topZ()+.02||!B.match(boxes,p.x,p.y,p.z,currentFilter()).visible)throw Error('此处位于当前显示范围之外，请重新选点');
  return [p.x,p.y,p.z];
}
async function pickAnchorAt(x,y){
  if(pickBusy){msg('正在读取上一个表面点，请稍候');return;}
  if($('shape').value!=='surface'){addAnchor(constructionHit(x,y));return;}
  if(selected){msg('请点“补充标注”开始一个新物体');return;}
  pickBusy=true;
  const ref=anchors,count=anchors.length,startDraft=draft,startAsset=gsAsset,startFilter=currentFilter(),startZ=topZ(),startView=view,startShape=$('shape').value,startFrame=cam.getWorldTransform().data.slice(),w=canvas.clientWidth,h=canvas.clientHeight;
  msg('正在读取点击位置的表面深度…');
  try{const p=await renderedSurfaceHit(x,y);if(mode!=='pick'||anchors!==ref||anchors.length!==count||draft!==startDraft||gsAsset!==startAsset||currentFilter()!==startFilter||topZ()!==startZ||view!==startView||$('shape').value!==startShape||canvas.clientWidth!==w||canvas.clientHeight!==h||cam.getWorldTransform().data.some((v,i)=>Math.abs(v-startFrame[i])>1e-6)){msg('取点期间视角或选区已变化，本次取点未加入');return;}addAnchor(p);}catch(e){msg(e.message);}finally{pickBusy=false;}
}
function cancelSelection(){
 if(globalThis.ObjectUI?.busy())return;
 if(globalThis.CleanupUI?.active()){CleanupUI.escape();return;}
 if(association){cancelAssociation();return;}
 if((draftChanged||anchors.length)&&!confirm('当前区域有未保存的修改或取点。放弃这些修改并取消选中？已保存标签不会删除。'))return;
 resetDraft();msg('已取消选中；已保存标签不变。右键拖动仍可平移视角。');
}
async function selectSurface(x,y){
 if(globalThis.ObjectUI?.busy())return;
 if(draftChanged||anchors.length){msg('请先保存或取消当前草稿，再选择其他区域');return;}
 const serial=++selectionSerial,startDraft=draft,startBoxes=boxes,startAsset=gsAsset,startView=view,startFilter=currentFilter(),startZ=topZ(),w=canvas.clientWidth,h=canvas.clientHeight,frame=cam.getWorldTransform().data.slice();
 const stillCurrent=()=>serial===selectionSerial&&mode==='browse'&&draft===startDraft&&boxes===startBoxes&&gsAsset===startAsset&&view===startView&&currentFilter()===startFilter&&topZ()===startZ&&canvas.clientWidth===w&&canvas.clientHeight===h&&!cam.getWorldTransform().data.some((v,i)=>Math.abs(v-frame[i])>1e-6);
 try{
  const p=await renderedSurfaceHit(x,y);if(!stillCurrent())return;
  // Same geometry and precedence as semantic coloring. No coarse mesh fallback.
  const b=B.compile(boxes)(...p,currentFilter()).box;
  if(b){if(association){toggleAssociation(b);return;}selectRegion(b,false);msg('已选中 '+boxName(b)+'；按可见表面匹配，左侧可修改或删除');}
  else if(association){msg('此处没有可关联的已保存片段；先加入识别候选，再进行关联');}
  else{resetDraft();msg('点击的可见表面没有已保存标签；不会穿透到后面的椅子或墙体');}
 }catch(error){if(stillCurrent())msg('未改变选择：'+error.message);}
}
function constructionHit(x,y){if($('shape').value==='surface')throw Error('表面拾取必须使用渲染深度流程');if(!anchors.length)return hit(x,y);const n={xy:[0,0,1],xz:[0,-1,0],yz:[1,0,0]}[$('plane').value],origin=cam.getPosition().clone(),dir=cam.camera.screenToWorld(x,y,500).sub(origin).normalize(),denom=n[0]*dir.x+n[1]*dir.y+n[2]*dir.z;if(Math.abs(denom)<1e-4)return null;const t=(n[0]*(anchors[0][0]-origin.x)+n[1]*(anchors[0][1]-origin.y)+n[2]*(anchors[0][2]-origin.z))/denom;if(t<=.05||t>200)return null;return [origin.x+t*dir.x,origin.y+t*dir.y,origin.z+t*dir.z];}
function addAnchor(p){if(!p){msg('此处无法取点，请换视角重试');return;}if(selected){msg('正在编辑已有区域，请点“补充标注”后选点');return;}if(anchors.length>=64){msg('最多 64 个点，请先生成区域');return;}if($('shape').value==='rectangle'&&anchors.length>=2){msg('矩形已有两个对角点，可保存或撤回一点');return;}anchors.push(p);draft=null;draftChanged=true;fields();if($('shape').value==='surface')msg(`已取第 ${anchors.length} 个表面点；${anchors.length<3?'继续点击物体另一侧':'可继续补点或生成区域'}`);else if($('shape').value==='rectangle'&&anchors.length===2)finishShape();else msg(anchors.length===1?'高级方式：首点固定工作平面':'按轮廓顺序继续取点，完成后生成区域');}
function candidateGeometryInfo(){
 globalThis.ProjectUI?.refresh();
 globalThis.CompletionUI?.refresh();
 globalThis.CleanupUI?.refresh();
 const note=$('geometryNote'),button=$('splitCandidate');if(!note||!button)return;
 associationInfo();
 const g=draft?.geometry;note.hidden=g?.type!=='surface-voxels';button.hidden=true;if(note.hidden||!globalThis.ObjectCore)return;
 const parts=ObjectCore.spatialParts(g,1),warnings=ObjectCore.geometryWarnings(draft);
 note.textContent=`${regionKind(draft)}。可见支持：${g.cells.length} 个单元、${parts.length} 个空间片段。${warnings.length?warnings.join(' / ')+'；':''}${parts.length>1?'片段不一定属于同一物体；可拆分后逐个检查。':'边界仅代表已观测到的表面，不是完整实体。'}`;
 if(globalThis.InstanceTools){const frames=InstanceTools.displayParts(draft).map(g=>InstanceTools.orientedBounds(g));note.textContent+=` 定向框${frames.length>1?'（最大片段）':''}：${frames[0].size.map(v=>v.toFixed(2)).join(' × ')} m；随已观测点拟合，不补背面。`;}
 button.hidden=parts.length<2;button.disabled=!selected||draftChanged||!!association;
}
function associationInfo(){
 const start=$('startAssociation'),panel=$('associationPanel'),restore=$('restoreAssociation');if(!start||!panel||!restore)return;
 const voxel=draft?.geometry?.type==='surface-voxels';start.hidden=!voxel||!!association;start.disabled=!selected||draftChanged;
 restore.hidden=!voxel||!globalThis.InstanceTools?.isAssociated(draft);restore.disabled=!!association||draftChanged||!globalThis.InstanceTools?.canRestore(draft);
 for(const id of ['category','status','commit'])$(id).disabled=!!association;
 $('delete').disabled=!!association||(!selected&&!draft&&!anchors.length);
 for(let i=0;i<3;i++)if($('center'+i))$('center'+i).disabled=!!association;
 panel.hidden=!association;if(!association)return;
 const base=boxes.find(b=>b.id===association.primary);if(!base)return;
 const host=$('associationList');host.replaceChildren();
 const distance=b=>Math.hypot(...b.min.map((v,i)=>(v+b.max[i]-base.min[i]-base.max[i])/2));
 const nearby=boxes.filter(b=>b.id!==base.id&&b.category===base.category&&InstanceTools.sameGrid(base.geometry,b.geometry||{})).sort((a,b)=>distance(a)-distance(b));
 const shown=nearby.filter((b,i)=>i<20||association.ids.has(b.id));
 for(const b of shown){const row=document.createElement('div');row.className='associationRow';const label=document.createElement('label'),check=document.createElement('input');check.type='checkbox';check.checked=association.ids.has(b.id);check.onchange=()=>toggleAssociation(b);const span=document.createElement('span');span.textContent=boxName(b);label.append(check,span);const focus=document.createElement('button');focus.textContent='定位';focus.setAttribute('aria-label','定位 '+boxName(b));focus.onclick=()=>fitBox(b);row.append(label,focus);host.append(row);}
 if(!shown.length)host.textContent='没有同类别、同采样网格的其他片段。请先加入识别候选。';
 $('associationCount').textContent=`基准 ${boxName(base)} · 已选 ${association.ids.size} 个区域。下方列出最近 20 个同类片段；也可点场景添加或移除。`;
 $('confirmAssociation').disabled=association.ids.size<2;
}
function startAssociation(){
 if(globalThis.CleanupUI?.active()){msg('请先应用或取消点级清理');return;}
 if(globalThis.CompletionUI?.active()){msg('请先应用或丢弃补识别建议，再人工关联');return;}
 const base=boxes.find(b=>b.id===selected);if(globalThis.ObjectUI?.busy()||!base||base.geometry?.type!=='surface-voxels')return;
 if(draftChanged||anchors.length){msg('请先保存或取消当前修改');return;}
 association={primary:base.id,ids:new Set([base.id]),source:boxes};selectionSerial++;candidateGeometryInfo();
 msg('关联模式：点击同一物体的其他片段，或勾选左侧列表；基准片段保留。右键 / Esc 取消，不改标签。');
}
function toggleAssociation(b){
 if(!association)return;const base=boxes.find(x=>x.id===association.primary);
 if(b.id===base.id){msg('这是基准片段；取消关联可退出');return;}
 if(b.category!==base.category||!InstanceTools.sameGrid(base.geometry,b.geometry||{})){msg('只能选择同类别、同一采样网格的可见片段；请先核查类别，不能把墙面加入椅子');return;}
 if(association.ids.has(b.id))association.ids.delete(b.id);else if(association.ids.size<24)association.ids.add(b.id);else{msg('一次最多关联 24 个区域');return;}
 selectionSerial++;associationInfo();msg(`已选择 ${association.ids.size} 个区域；转动视角核查是否为同一物体，再确认关联。`);
}
function cancelAssociation(){association=null;selectionSerial++;candidateGeometryInfo();msg('已取消片段关联；原有标签未改变');}
function confirmAssociation(){
 if(!association||globalThis.ObjectUI?.busy())return;
 if(boxes!==association.source||draftChanged){msg('标签已变化，请取消关联后重新选择');return;}
 try{
  const regions=[...association.ids].map(id=>boxes.find(b=>b.id===id));if(regions.some(b=>!b))throw Error('所选片段已变化');
  const merged=InstanceTools.mergeRegions(regions),warning=ObjectCore.geometryWarnings(merged).length?'合并后的跨度超出类别常见范围，请特别核查。':'';
  if(!confirm(`把 ${regions.map(boxName).join('、')} 关联为同一物体？\n保留基准编号 ${boxName(merged)}，仅合并已有支持单元，重置为人工标注 / 未复核，不补齐缺失部位。${warning}\n来源片段会保留，可解除关联或撤销。`))return;
  const ids=association.ids;snapshot();boxes=boxes.flatMap(b=>b.id===merged.id?[merged]:ids.has(b.id)?[]:[b]);association=null;
  const saved=persist();selectRegion(merged,false);filterScene();if(saved)msg(`已关联为 ${boxName(merged)}；保留 ${merged.geometry.cells.length} 个支持单元。完整性待核查，可解除关联或 Ctrl+Z 撤销。`);
 }catch(e){msg('未关联：'+e.message);}
}
function restoreAssociation(){
 const old=boxes.find(b=>b.id===selected);if(!old||association||draftChanged||globalThis.ObjectUI?.busy())return;
 try{
  const sources=InstanceTools.restoreSources(old);if(boxes.length+sources.length-1>5000)throw Error('解除后超过区域数量上限');
  if(!confirm(`解除 ${boxName(old)} 的人工关联，恢复 ${sources.length} 个来源片段及原状态？如原编号已被占用，将分配新编号。可撤销。`))return;
  const allocated=boxes.filter(b=>b.id!==old.id);for(const b of sources){if(allocated.some(x=>x.id===b.id))b.id=crypto.randomUUID();if(allocated.some(x=>x.category===b.category&&x.sequence===b.sequence))b.sequence=B.nextSequence(allocated,b.category);allocated.push(b);}
  snapshot();boxes=boxes.flatMap(b=>b.id===old.id?sources:[b]);const saved=persist();selectRegion(sources[0],false);filterScene();if(saved)msg(`已恢复 ${sources.length} 个来源片段，可 Ctrl+Z 撤销`);
 }catch(e){msg('未解除关联：'+e.message);}
}
function exportAnnotations(){
 const doc=B.save(boxes,meta);
 if(globalThis.InstanceTools)doc.observed_envelopes={kind:'derived_display_bounds_not_instance_ground_truth',coordinate_frame:meta.coordinate_frame,regions:boxes.filter(b=>b.geometry?.type==='surface-voxels').map(b=>({region_id:b.id,coverage:'observed_only_completeness_unchecked',grouping:b.provenance?.evidence_state==='manual_background_cleanup'?'manual_point_cleanup':b.provenance?.evidence_state==='targeted_instance_refinement'?'target_anchored_refinement':InstanceTools.isAssociated(b)?'manual_same_object':'spatial_fragments',frames:InstanceTools.displayParts(b).map(g=>{const {corners,...frame}=InstanceTools.orientedBounds(g);return frame;})}))};
 return doc;
}
function splitCurrentCandidate(){
 if(globalThis.CleanupUI?.active()){msg('请先应用或取消点级清理');return;}
 const old=boxes.find(b=>b.id===selected);
 if(!old||old.geometry?.type!=='surface-voxels'||association||globalThis.ObjectUI?.busy()){msg('先选择可见片段并退出关联模式');return;}
 if(draftChanged||anchors.length){msg('请先保存或取消当前修改');return;}
 const parts=ObjectCore.splitCandidate(old);if(parts.length<2){msg('当前候选只有一个空间片段');return;}
 if(boxes.length+parts.length-1>5000){msg('拆分后超过区域数量上限，未修改');return;}
 if(!confirm(`${boxName(old)} 有 ${parts.length} 个不相连的空间片段。拆成 ${parts.length} 个未复核片段？保留全部支持单元，重置复核状态；细小片段可能是噪声，可撤销。`))return;
 snapshot();let sequence=B.nextSequence(boxes,old.category);
 const next=parts.map((p,i)=>({...p,id:i?crypto.randomUUID():old.id,sequence:i?sequence++:old.sequence}));
 boxes=boxes.flatMap(b=>b.id===old.id?next:[b]);const saved=persist();selectRegion(next[0],false);filterScene();
 if(saved)msg(`已拆成 ${parts.length} 个未复核候选，原编号留给最大的一片；全部支持单元保留，可 Ctrl+Z 撤销。请逐片核查类别，不把分片数当作物体数。`);
}
function bind(){for(const c of B.categories){for(const id of ['category','filter']){const o=document.createElement('option');o.value=c[0];o.textContent=c[1];$(id).append(o);}}$('filter').value='wall';
 $('startAssociation').onclick=startAssociation;$('confirmAssociation').onclick=confirmAssociation;$('cancelAssociation').onclick=cancelAssociation;$('restoreAssociation').onclick=restoreAssociation;$('boundsMode').onchange=candidateGeometryInfo;
 $('confirmUpgrade').onclick=commitProposals;$('cancelUpgrade').onclick=()=>{$('upgradePanel').hidden=true;};$('checkButton').onclick=()=>{$('reviewMode').checked=!$('reviewMode').checked;$('reviewMode').onchange();};$('closeCheck').onclick=()=>{$('reviewMode').checked=false;$('reviewMode').onchange();};$('closeEditor').onclick=resetDraft;
 for(const [prefix,name] of [['center','中心'],['size','尺寸']]){const s=document.createElement('span');s.textContent=name;$('dimensions').append(s);for(let i=0;i<3;i++){const input=document.createElement('input');input.id=prefix+i;input.type='number';input.step='.05';input.setAttribute('aria-label',name+' '+['X','Y','Z'][i]);if(prefix==='size')input.min='.01';input.onchange=editDimensions;$('dimensions').append(input);}}
 const pickHelp=()=>{const surface=$('shape').value==='surface';$('planeOptions').hidden=surface;$('pickHelp').textContent=surface?'每次点击独立读取物体表面位置；可以转动视角补点。至少三个不共线的点，再生成区域。':'高级方式：首点确定固定工作平面，后续按平面轮廓绘制。';};pickHelp();
 $('pick').onclick=()=>{setMode('pick');msg($('pickHelp').textContent);};$('browse').onclick=()=>setMode('browse');$('popAnchor').onclick=()=>{if(!anchors.length)return;anchors.pop();draft=null;draftChanged=true;fields();};$('clearDraft').onclick=resetDraft;$('newBox').onclick=()=>{resetDraft();$('shape').value='surface';$('thickness').value='.03';pickHelp();$('status').value='draft';setMode('pick');msg($('pickHelp').textContent);};$('category').onchange=()=>{instanceUpdate();draftChanged=!!draft;};$('status').onchange=()=>{draftChanged=!!draft;};$('commit').onclick=saveDraft;$('finishShape').onclick=finishShape;
 for(const id of ['shape','plane'])$(id).onchange=()=>{resetDraft();$('thickness').value=$('shape').value==='surface'?'.03':'2.5';pickHelp();setMode('pick');msg($('pickHelp').textContent);};$('thickness').onchange=()=>{const t=Number($('thickness').value);if(!Number.isFinite(t)||t<.01||t>100){msg('厚度须在 0.01 至 100 米之间');return;}if(draft?.geometry&&draft.geometry.type!=='surface-voxels'){const g=draft.geometry,mid=(g.low+g.high)/2;g.low=mid-t/2;g.high=mid+t/2;Object.assign(draft,B.bounds(g));draftChanged=true;fields();msg('已调整区域厚度，请检查是否仍覆盖物体；原始表面取点保持不变');}};
 $('undo').onclick=undoRegionEdit;$('delete').onclick=deleteCurrent;$('splitCandidate').onclick=splitCurrentCandidate;$('applyAuto').onclick=addProposals;$('reviewMode').onchange=()=>{$('reviewPanel').hidden=!$('reviewMode').checked;filterScene();msg($('reviewMode').checked?'语义检查已开启，可按类别或未覆盖区域查看':'已关闭语义检查；标签和复核状态不变');};
 for(const id of ['filterEnabled','filter','semanticColor'])$(id).onchange=()=>{filterScene();msg(currentFilter()==='all'?'显示完整场景':'已按盒内空间筛选；取消开关可对照检查遗漏');};$('clipZ').oninput=()=>{$('clipValue').textContent=topZ().toFixed(2)+' m';scheduleFilter();};$('unclip').onclick=()=>{$('clipZ').value=$('clipZ').max;$('clipZ').oninput();};$('home').onclick=home;$('top').onclick=()=>{pitch=Math.PI/2-.001;cameraUpdate();};$('focus').onclick=()=>fitBox(draft);$('gsMode').onclick=()=>setView('gs');$('pointMode').onclick=()=>setView('points');$('quality').onchange=()=>loadGS($('quality').value);
 $('export').onclick=()=>{const blob=new Blob([JSON.stringify(exportAnnotations(),null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='digit-k2-boxes-'+new Date().toISOString().replace(/[:.]/g,'-')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),10000);dirty=false;msg('已请求下载标签（含派生定向框与关联来源）；请确认浏览器保存完成');};$('importButton').onclick=()=>$('importFile').click();$('importFile').onchange=async e=>{try{const file=e.target.files[0];if(!file)return;if(file.size>64*1024*1024)throw Error('文件超过 64 MB');const next=B.validate(JSON.parse(await file.text()),meta);if(boxes.length&&!confirm('替换当前所有盒标签？可以撤销。'))return;snapshot();boxes=next;persist();resetDraft();filterScene();msg('已导入 '+boxes.length+' 个共享空间标签');}catch(e){msg('导入失败：'+e.message);}finally{$('importFile').value='';}};
const pos=e=>{const r=overlay.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];};overlay.oncontextmenu=e=>e.preventDefault();overlay.onpointerdown=e=>{if(!meta||globalThis.ObjectUI?.busy())return;document.activeElement.blur();overlay.setPointerCapture(e.pointerId);drag={start:pos(e),last:pos(e),button:e.button,moved:0};};overlay.onpointermove=e=>{if(!drag)return;const p=pos(e),dx=p[0]-drag.last[0],dy=p[1]-drag.last[1];drag.moved+=Math.abs(dx)+Math.abs(dy);if(drag.moved<5){drag.last=p;return;}if(drag.button===2){panCamera(dx,dy,e);}else{lookAround(dx,dy);drag.last=p;return;}drag.last=p;cameraUpdate();};overlay.onpointercancel=()=>{drag=null;};overlay.ondblclick=e=>{if(mode==='browse'){const p=hit(...pos(e));if(p){target=V(p);dist=Math.min(dist,6);cameraUpdate();}}};overlay.addEventListener('wheel',wheelMove,{passive:false});document.addEventListener('keydown',e=>{if(['INPUT','SELECT','TEXTAREA'].includes(document.activeElement.tagName)||document.activeElement.isContentEditable)return;if(e.key==='Escape'){e.preventDefault();cancelSelection();return;}if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'){e.preventDefault();$('undo').click();return;}if(e.ctrlKey||e.metaKey||e.altKey)return;if(['KeyW','KeyA','KeyS','KeyD','KeyQ','KeyE','ShiftLeft','ShiftRight'].includes(e.code)){e.preventDefault();pressed.add(e.code);}});document.addEventListener('keyup',e=>pressed.delete(e.code));window.addEventListener('blur',()=>pressed.clear());document.addEventListener('visibilitychange',()=>pressed.clear());window.addEventListener('beforeunload',e=>{if(dirty||draftChanged||anchors.length){e.preventDefault();e.returnValue='';}});
 // Surface-first selection; the label remains a secondary click target.
 overlay.onpointerup=e=>{if(drag&&drag.moved<5&&drag.button===2){cancelSelection();}else if(drag&&drag.moved<5&&drag.button===0){const rect=overlay.getBoundingClientRect(),x=e.clientX-rect.left,y=e.clientY-rect.top;if(mode==='pick')void pickAnchorAt(x,y);else void selectSurface(x,y);}drag=null;};
 // Point cleanup is an explicit left-button mode; normal navigation stays unchanged.
 const navigation={down:overlay.onpointerdown,move:overlay.onpointermove,up:overlay.onpointerup,cancel:overlay.onpointercancel,double:overlay.ondblclick};
 overlay.onpointerdown=e=>{if(e.button===0&&globalThis.CleanupUI?.painting()){document.activeElement.blur();overlay.setPointerCapture(e.pointerId);drag={paint:true};CleanupUI.begin(pos(e));return;}navigation.down(e);};
 overlay.onpointermove=e=>{if(drag?.paint){CleanupUI.paint(pos(e));return;}navigation.move(e);};
 overlay.onpointerup=e=>{if(drag?.paint){CleanupUI.end();drag=null;return;}navigation.up(e);};
 overlay.onpointercancel=e=>{if(drag?.paint)CleanupUI.end();navigation.cancel(e);};
 overlay.ondblclick=e=>{if(!globalThis.CleanupUI?.painting())navigation.double(e);};
 const undoLabels=$('undo').onclick;$('undo').onclick=()=>globalThis.CleanupUI?.active()?CleanupUI.undoStroke():undoLabels();
 // Avoid combining a staged point edit with another modifying workflow.
 for(const id of ['commit','delete','applyAuto','confirmUpgrade','restoreAssociation','importButton','newBox']){const action=$(id).onclick;$(id).onclick=(...args)=>{if(globalThis.CleanupUI?.active()){msg('请先应用或取消点级清理');return;}return action(...args);};}
}
async function init(){try{bind();if(typeof pc==='undefined')throw Error('本地 PlayCanvas 依赖缺失');app=new pc.Application(canvas,{graphicsDeviceOptions:{antialias:false,alpha:false}});app.graphicsDevice.maxPixelRatio=1;app.setCanvasResolution(pc.RESOLUTION_AUTO);app.scene.toneMapping=pc.TONEMAP_NONE;cam=new pc.Entity('Annotation camera');cam.addComponent('camera',{clearColor:new pc.Color(.035,.05,.075),fov:60,nearClip:.05,farClip:500});app.root.addChild(cam);
 const get=async(url,type)=>{const r=await fetch(url);if(!r.ok)throw Error(url+' HTTP '+r.status);return type==='json'?r.json():r.arrayBuffer();};const [m,p,v,i]=await Promise.all([get('/scene.json','json'),get('/points.bin'),get('/mesh-vertices.bin'),get('/mesh-indices.bin')]);meta=m;points=new Float32Array(p);vertices=new Float32Array(v);indices=new Uint32Array(i);const n=points.length/6;positions=new Float32Array(n*3);rgba=new Uint8Array(n*4);pointVisible=new Uint8Array(n);for(let k=0;k<n;k++)positions.set(points.subarray(k*6,k*6+3),k*3);
 pointMesh=new pc.Mesh(app.graphicsDevice);pointMesh.setPositions(positions);pointMesh.setColors32(rgba);pointMesh.update(pc.PRIMITIVE_POINTS);const material=new pc.ShaderMaterial({uniqueName:'DigitLasPoints',attributes:{aPosition:pc.SEMANTIC_POSITION,aColor:pc.SEMANTIC_COLOR},vertexGLSL:'attribute vec3 aPosition; attribute vec4 aColor; uniform mat4 matrix_model; uniform mat4 matrix_viewProjection; varying vec4 color; void main(){color=aColor;gl_Position=matrix_viewProjection*matrix_model*vec4(aPosition,1.0);gl_PointSize=2.0;}',fragmentGLSL:'varying vec4 color;void main(){if(color.a<0.5)discard;gl_FragColor=vec4(color.rgb,1.0);}'});pointEntity=new pc.Entity('LAS preview');pointEntity.addComponent('render',{meshInstances:[new pc.MeshInstance(pointMesh,material)]});app.root.addChild(pointEntity);pointEntity.enabled=false;
 storageKey=globalThis.SceneScope?.key('digit-regions-v6:'+meta.scene_id)||'digit-regions-v6:'+meta.scene_id;let note='',hasSaved=false;try{const saved=(globalThis.SceneScope?.storage||localStorage).getItem(storageKey)||(!globalThis.SceneScope&&(globalThis.SceneScope?.storage||localStorage).getItem('digit-regions-v4:'+meta.scene_id))||(!globalThis.SceneScope&&(globalThis.SceneScope?.storage||localStorage).getItem('digit-regions-v3:'+meta.scene_id))||(!globalThis.SceneScope&&(globalThis.SceneScope?.storage||localStorage).getItem('digit-boxes-v2:'+meta.scene_id));if(saved){hasSaved=true;boxes=B.validate(JSON.parse(saved),meta);note='已恢复标签；点击场景即可修改，或用自动整理更新旧候选';dirty=true;}}catch(e){hasSaved=true;note='旧草稿未恢复：'+e.message;}
 try{const response=await fetch('/auto-labels.json');if(!response.ok)throw Error('HTTP '+response.status);const doc=await response.json();proposals=B.validate(doc,meta);const old=await fetch('/auto-labels-legacy.json');if(old.ok)legacyProposals=B.validate(await old.json(),meta);$('autoSummary').textContent=`已准备 ${proposals.length} 个结构，点击上方按钮应用。人工修改会保留。`;if(!hasSaved)note='先点“自动整理墙体与地板”，再点击场景修改';}catch(e){$('autoSummary').textContent='自动候选读取失败：'+e.message;}
 $('sceneName').textContent=meta.name;const low=Math.min(meta.bounds[0][2],meta.point_bounds[0][2]),high=Math.max(meta.bounds[1][2],meta.point_bounds[1][2]);$('clipZ').min=Math.floor(low*20)/20;$('clipZ').max=Math.ceil(high*20)/20;$('clipZ').value=$('clipZ').max;$('clipValue').textContent=topZ().toFixed(2)+' m';home();fields();filterScene();new ResizeObserver(scheduleViewportResize).observe($('viewport'));window.addEventListener('resize',scheduleViewportResize);document.addEventListener('fullscreenchange',scheduleViewportResize);window.visualViewport?.addEventListener('resize',scheduleViewportResize);syncViewportSize();app.on('postrender',drawOverlay);app.on('update',movement);app.start();await loadGS('medium');if(note)msg(note);
 }catch(e){console.error(e);msg('启动失败：'+e.message+'；请保留页面并反馈');}}
init();
