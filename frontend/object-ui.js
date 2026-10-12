(function(){
 'use strict';
 const C=ObjectCore, pending=[], rejected=new Set();let automatic=false,running=false,stop=false,preview=true,progress='',picker;
 const label=b=>cat(b.category)[1], panel=$('objectPanel');
 const chosen=()=>pending.filter(b=>!C.geometryWarnings(b).length&&!rejected.has(b.id)&&(!$('objectMultiOnly').checked||C.viewCount(b)>=2));
 const api=async(url,options)=>{const r=await fetch(url,options),data=await r.json();if(!r.ok)throw Error(data.error||'本地识别请求失败');return data;};
 const infer=(frame,options)=>api('/semantic/infer',{method:'POST',headers:{'Content-Type':'application/json','X-Digit-Local':'1'},body:JSON.stringify({image:frame.image,...options,scene_id:meta.scene_id,source_hashes:meta.source_hashes,...(globalThis.ProjectCore&&($('archiveEvidence')?.checked||automatic)?{frame:ProjectCore.framePayload(frame)}:{})})});
 function redraw(){
  if(automatic)$('initialProgress').textContent=progress;
  $('objectProgress').textContent=progress||'先识别，再预览；候选不会自动保存为正式标签。';
  const items=chosen(),groups=new Map();for(const b of items)groups.set(label(b),(groups.get(label(b))||0)+1);
  const flagged=pending.filter(b=>C.geometryWarnings(b).length).length;
  $('objectCounts').textContent=(items.length?`${items.length} 个可见表面候选 · `+[...groups].map(([k,n])=>k+' '+n).join(' / '):'暂无可加入候选')+(flagged?`；隔离 ${flagged} 个尺寸异常候选，需人工补画`:'');
  $('objectAccept').disabled=running||!items.length;$('objectDiscard').disabled=running||!pending.length;$('objectStop').hidden=!running;
  $('objectList').replaceChildren();for(const b of pending){if($('objectMultiOnly').checked&&C.viewCount(b)<2)continue;const row=document.createElement('div');row.className='objectRow';const check=document.createElement('input');check.type='checkbox';check.checked=!C.geometryWarnings(b).length&&!rejected.has(b.id);check.disabled=running||!!C.geometryWarnings(b).length;check.setAttribute('aria-label','保留 '+label(b));check.onchange=()=>{if(check.checked)rejected.delete(b.id);else rejected.add(b.id);redraw();filterScene();};
   const focus=document.createElement('button');focus.textContent=`${label(b)}${C.geometryWarnings(b).length?' · 尺寸异常，已隔离':''} · 分数 ${b.provenance.model_score.toFixed(2)} · ${C.viewCount(b)>=2?C.viewCount(b)+' 视角支持':'单视图待核查'}`;focus.title=`模型原类：${b.provenance.model_label}；分数不是准确率。多视角支持只说明同类空间支持重叠，不等于人工确认`;focus.disabled=running;focus.onclick=()=>{fitBox(b);msg('候选：'+label(b)+'；局部边框仅定位，有颜色的深度支持单元才是候选范围；尺寸异常候选不能直接加入');};row.append(check,focus);$('objectList').append(row);
  }
 }
 function accept(){if(running)return;if(association||globalThis.CompletionUI?.active()||globalThis.CleanupUI?.active()){msg('请先完成或取消片段关联 / 补识别 / 点级清理，再加入候选');return;}const items=chosen();if(!items.length)return;snapshot();let count=0,skipped=0;
  for(const item of items){if(boxes.some(b=>b.provenance?.proposal_id===item.id||(b.category===item.category&&C.supportOverlap(b,item)>.6))){skipped++;continue;}const b=clone(item);b.sequence=B.nextSequence(boxes,b.category);b.provenance.proposal_id=item.id;if(!B.validBox(b)){skipped++;continue;}boxes.push(b);count++;}
  // Preserve hidden single-view candidates when the multi-view filter is active.
  const consumed=new Set(items.map(b=>b.id));for(let i=pending.length-1;i>=0;i--)if(consumed.has(pending[i].id)){rejected.delete(pending[i].id);pending.splice(i,1);}const saved=persist();filterScene();redraw();if(saved)msg(`已加入 ${count} 个未复核物体候选，跳过 ${skipped} 个重复或超限区域；未勾选和隐藏候选保留，已有人工标签不变，可撤销`);
 }
 const nextFrame=()=>new Promise((resolve,reject)=>{const timeout=setTimeout(()=>{app.off('postrender',done);reject(Error('等待场景渲染超时，请保持页面可见'));},8000);const done=()=>{clearTimeout(timeout);resolve();};app.once('postrender',done);});
 async function capture(maxSide=1024){
  await nextFrame();await nextFrame();
  return new Promise((resolve,reject)=>{
   const timer=setTimeout(()=>{app.off('postrender',take);reject(Error('场景截图超时'));},8000);
   const take=()=>{clearTimeout(timer);try{
    const scale=Math.min(1,maxSide/Math.max(canvas.width,canvas.height)),w=Math.round(canvas.width*scale),h=Math.round(canvas.height*scale);
    if(Math.min(w,h)<64)throw Error('请展开场景视图区后重试');
    const image=document.createElement('canvas');image.width=w;image.height=h;image.getContext('2d').drawImage(canvas,0,0,w,h);
    const png=image.toDataURL('image/jpeg',.94).split(',')[1],matrix=new pc.Mat4().mul2(cam.camera.camera.projectionMatrix,cam.camera.camera.viewMatrix),projection=Array.from(matrix.data),inverse=matrix.invert().data.slice(),eye=cam.getPosition();
    const near=cam.camera.nearClip,far=cam.camera.farClip,world_matrix=Array.from(cam.getWorldTransform().data),fov=cam.camera.fov,horizontal_fov=cam.camera.horizontalFov,pose_id=world_matrix.map(v=>v.toFixed(4)).join(',');
    if(!picker)picker=new pc.Picker(app,w,h,true);picker.resize(w,h);picker.prepare(cam.camera,app.scene);
    const camera_position=[eye.x,eye.y,eye.z];
    picker.depthBuffer.read(0,0,w,h,{immediate:true,renderTarget:picker.renderTargetDepth}).then(bytes=>resolve({image:png,width:w,height:h,inverse:Array.from(inverse),projection,camera_position,world_matrix,fov,horizontal_fov,near,far,pose_id,depth:C.decodeDepth(bytes,w,h,app.graphicsDevice.isWebGL2)}),reject);
   }catch(error){reject(error);}};app.once('postrender',take);
  });
 }
 async function run(tour,initial=false){
  if(globalThis.CleanupUI?.active()){msg('请先应用或取消点级清理');return;}
  if(globalThis.CompletionUI?.active()){msg('请先应用或丢弃当前物体的补识别建议');return;}
  if(association){msg('请先完成或取消片段关联，再开始识别');return;}
  if(running)return;if(!meta||!gsEntity||loading){msg('请等待 3DGS 场景加载完成');return;}if(draftChanged||anchors.length){msg('请先保存或取消当前手动绘制，再开始自动识别');return;}
  if(initial&&pending.length){msg('高级识别中还有未处理候选，请先加入或丢弃；未清除旧候选。');return;}if(initial&&!proposals.length){msg('结构数据尚未就绪，请稍候');return;}if(initial&&globalThis.ProjectUI?.hasReviewDraft()){msg('高级面板中有未保存填写，请先完成或取消；不要求新审核。');return;}
  automatic=initial;const baseline=JSON.stringify(B.save(boxes,meta));
  const saved={target:target.clone(),dist,yaw,pitch,view,clip:$('clipZ').value,review:$('reviewMode').checked,filter:$('filterEnabled').checked,color:$('semanticColor').checked,preview};
  running=true;stop=false;preview=false;pressed.clear();panel.hidden=initial;$('initialStop').hidden=!initial;progress='准备本地模型与视角…';
  const disabled=[...document.querySelectorAll('button,input,select,textarea')].map(el=>[el,el.disabled]);for(const [el]of disabled)el.disabled=true;$('objectStop').disabled=false;$('initialStop').disabled=false;redraw();
  const confidence=initial?0.6:Number($('objectConfidence').value),backend=initial?'instance':$('objectBackend').value,detail=initial?'standard':backend==='instance'?$('objectDetail').value:'standard';let done=0,total=1,rawCount=0,depthCount=0,failed=false,failureReason='';
  try{
   const status=await api('/semantic/status');if(!status.ready||(backend==='instance'&&!status.instance_ready))throw Error('本地模型或依赖未就绪，请使用语义运行环境启动服务');
   let views=[null];if(tour){const plan=await api('/semantic/tour');if(plan.scene_id!==meta.scene_id||JSON.stringify(plan.source_hashes)!==JSON.stringify(meta.source_hashes))throw Error('扫描位置批次不匹配');views=plan.positions.flatMap(position=>[0,Math.PI/2,Math.PI,Math.PI*1.5].map(angle=>({position,angle})));if(!views.length)throw Error('没有可用扫描位置');}
   total=views.length;$('reviewMode').checked=false;$('filterEnabled').checked=false;$('semanticColor').checked=false;setView('gs');if(tour)$('clipZ').value=$('clipZ').max;filterScene();
   for(const pose of views){if(stop)break;if(pose){cam.setPosition(...pose.position);cam.lookAt(new pc.Vec3(pose.position[0]+Math.cos(pose.angle)*3,pose.position[1]+Math.sin(pose.angle)*3,pose.position[2]-.35),new pc.Vec3(0,0,1));updateCameraClip();await new Promise(resolve=>setTimeout(resolve,250));}
    progress=`识别 ${done+1} / ${total} 个视图 · ${detail==='detail'?'全图 + 4 个局部细节':'全图识别'} · CPU 本地推理中，可停止`;redraw();const frame=await capture(backend==='instance'?1024:512);if(stop)break;
    const result=await infer(frame,{confidence,backend,detail});
    if(stop)break;if(result.width!==frame.width||result.height!==frame.height||result.scene_id!==meta.scene_id)throw Error('模型结果与当前视图不匹配');rawCount+=result.components.length;
    const candidates=result.components.flatMap(c=>C.fromMask(c,frame,result));depthCount+=candidates.length;for(const b of candidates){b.id=crypto.randomUUID();b.sequence=B.nextSequence([...boxes,...pending],b.category);}
    C.merge(pending,candidates);done++;redraw();
   }
   progress=`${stop?'已停止':'本轮完成'}：${done}/${total} 视图，${rawCount} 个二维${backend==='instance'?'实例掩膜':'连通区域'}，${depthCount} 个三维支持候选；当前合并为 ${pending.length} 个候选。不是全场景覆盖率或真实物体数量。`;
  }catch(error){failed=true;failureReason=error.message;progress='识别中止：'+error.message+`；已完成 ${done}/${total} 视图，已有候选保留。`;}
  finally{
   target=saved.target;dist=saved.dist;yaw=saved.yaw;pitch=saved.pitch;cameraUpdate();setView(saved.view);$('clipZ').value=saved.clip;$('clipValue').textContent=topZ().toFixed(2)+' m';$('reviewMode').checked=saved.review;$('filterEnabled').checked=saved.filter;$('semanticColor').checked=saved.color;preview=saved.preview;
   for(const [el,value]of disabled)el.disabled=value;running=false;
   if(initial){try{if(baseline!==JSON.stringify(B.save(boxes,meta)))throw Error('工作台在运行期间变化，未合入新结果；已识别候选留在高级入口');
    const batch={id:crypto.randomUUID(),at:new Date().toISOString(),status:failed?'partial_failure':stop?'stopped':'completed_views',views_done:done,views_total:total};
    const result=InitialCore.plan(boxes,proposals,pending,B,C,()=>crypto.randomUUID(),batch);batch.skipped=result.skipped;
    B.validate(B.save([...boxes,...result.added],meta),meta);if(result.added.length){if(!globalThis.ProjectUI?.applyInitialBatch)throw Error('实例关联模块未就绪，请保存旧工作后刷新；候选已保留');ProjectUI.applyInitialBatch(result.added);resetDraft();}
    pending.length=0;rejected.clear();progress=`${failed?'部分完成（'+failureReason+'）':stop?'已停止，保留已完成结果':'本轮初标完成'}：新增 ${result.added.length} 条标注，跳过 ${Object.values(result.skipped).reduce((a,b)=>a+b,0)} 条异常/重叠/重复结果；物体视图 ${done}/${total}。跳过原因：异常 ${result.skipped.invalid}、重叠 ${result.skipped.overlap}、重复 ${result.skipped.duplicate}、数量上限 ${result.skipped.limit}。${result.added.length?'旧标签未替换；新增批次已关联实例，可撤销，点击保存项目留存版本。':'没有新增标注，旧标签及实例档案未改变，无需保存或撤销本批。'}不是全场覆盖率。`;
   }catch(error){progress='一键初标未完全应用：'+error.message;panel.hidden=false;}}
   redraw();if(initial)$('initialProgress').textContent=progress;$('initialStop').hidden=true;automatic=false;filterScene();msg(progress);
  }
 }
 globalThis.ObjectUI={runInitial:()=>run(true,true),pending:chosen,hasPending:()=>pending.length>0,visible:()=>preview&&!running&&!globalThis.CompletionUI?.active()&&chosen().length>0,busy:()=>running||!!globalThis.CompletionUI?.busy()||!!globalThis.ProjectUI?.busy(),capture,infer,request:api,refresh:redraw};
 $('objectOpen').onclick=()=>{panel.hidden=!panel.hidden;redraw();};$('initialAnnotate').onclick=()=>run(true,true);$('initialStop').onclick=()=>{stop=true;$('initialProgress').textContent='正在停止；当前本地推理结束后保留已完成结果。';};$('objectCurrent').onclick=()=>run(false);$('objectTour').onclick=()=>run(true);$('objectStop').onclick=()=>{stop=true;progress='正在停止，当前本地推理结束后恢复视角…';redraw();};$('objectAccept').onclick=accept;
 $('objectDiscard').onclick=()=>{pending.length=0;rejected.clear();progress='已丢弃未加入的候选，已有标签不变';redraw();filterScene();};$('objectPreview').onchange=()=>{preview=$('objectPreview').checked;filterScene();};
 $('objectMultiOnly').onchange=()=>{redraw();filterScene();};
 $('objectBackend').onchange=()=>{$('objectDetail').disabled=$('objectBackend').value==='legacy';};
 window.addEventListener('beforeunload',e=>{if(pending.length||running){e.preventDefault();e.returnValue='';}});
 redraw();
})();
