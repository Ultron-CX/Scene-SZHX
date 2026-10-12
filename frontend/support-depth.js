(function(root){
 'use strict';
 const C=typeof module!=='undefined'?require('./completion-core.js'):root.CompletionCore;
 function visibleAt(point,cell,frame){
  if(!frame)return false;const q=C.project(point,frame);if(!q||q.z< -1||q.z>1)return false;const x=Math.floor(q.x),y=Math.floor(q.y);if(x<0||y<0||x>=frame.width||y>=frame.height)return false;
  const d=frame.depth[y*frame.width+x];if(!Number.isFinite(d)||d<0||d>=1)return false;
  const normalized=(q.z+1)/2,linear=normalized*frame.near/(frame.far-normalized*(frame.far-frame.near)),distance=frame.near+linear*(frame.far-frame.near),surface=frame.near+d*(frame.far-frame.near);
  return distance<=surface+Math.max(.04,cell*1.5);
 }
 if(typeof module!=='undefined'){module.exports={visibleAt};return;}
 let frame=null,stamp='',revision=0,pending=false,picker=null,lastAttempt=0,lastFailure='',serial=0;
 const note=s=>{const node=$('supportDepthStatus');if(node&&node.textContent!==s)node.textContent=s;};
 const signature=()=>cam&&app?[view,canvas.clientWidth,canvas.clientHeight,revision,...cam.getWorldTransform().data,...cam.camera.camera.projectionMatrix.data].join(','):'';
 function ready(){return view==='gs'&&!loading&&!!gsEntity&&!!frame&&stamp===signature()&&!globalThis.ObjectUI?.busy();}
 function invalidate(){revision++;frame=null;}
 function visible(point,cell,forceDepth=false){if(!forceDepth&&($('supportXray')?.checked||view!=='gs'))return true;return ready()&&visibleAt(point,cell,frame);}
 // One freshness check per draw/brush batch, not a matrix serialization per dot.
 function checker(forceDepth=false){if(!forceDepth&&($('supportXray')?.checked||view!=='gs'))return ()=>true;const f=ready()?frame:null;return (point,cell)=>visibleAt(point,cell,f);}
 function update(){
  if(!app||!cam||!gsEntity||loading||view!=='gs'){frame=null;note(view==='points'?'点云模式：支持点为叠加预览；画笔请切回 3DGS。':'等待 3DGS。');return;}
  const needed=draft?.geometry?.type==='surface-voxels'||globalThis.ObjectUI?.visible()||globalThis.CleanupUI?.active();
  if(!needed||globalThis.ObjectUI?.busy())return;
  const current=signature();if(frame&&stamp===current){note('支持点遮挡判断已就绪；只影响显示，不改变标签。');return;}
  note(lastFailure||'正在读取遮挡深度；暂不显示无法判断的支持点。');
  const now=performance.now();if(pending||now-lastAttempt<(lastFailure?1500:150))return;lastAttempt=now;pending=true;const ticket=++serial,asset=gsAsset;
  try{
   const scale=Math.min(1,640/Math.max(canvas.clientWidth,canvas.clientHeight)),w=Math.max(1,Math.round(canvas.clientWidth*scale)),h=Math.max(1,Math.round(canvas.clientHeight*scale));
   if(!picker)picker=new pc.Picker(app,w,h,true);picker.resize(w,h);picker.prepare(cam.camera,app.scene);
   const snapshot={width:w,height:h,near:cam.camera.nearClip,far:cam.camera.farClip,projection:Array.from(cam.camera.camera.projectionMatrix.data)};
   // Snapshot the full world-to-clip matrix, not just the camera projection.
   snapshot.projection=Array.from(new pc.Mat4().mul2(cam.camera.camera.projectionMatrix,cam.camera.camera.viewMatrix).data);
   Promise.resolve(picker.depthBuffer.read(0,0,w,h,{immediate:true,renderTarget:picker.renderTargetDepth})).then(bytes=>{
    if(ticket!==serial||asset!==gsAsset||signature()!==current||globalThis.ObjectUI?.busy())return;
    frame={...snapshot,depth:ObjectCore.decodeDepth(bytes,w,h,app.graphicsDevice.isWebGL2)};stamp=current;lastFailure='';
   }).catch(()=>{frame=null;lastFailure='遮挡深度读取失败。可开启透视叠加查看；画笔暂不可用。';note(lastFailure);}).finally(()=>{pending=false;});
  }catch(e){pending=false;frame=null;lastFailure='遮挡深度暂不可用。可开启透视叠加查看；画笔暂不可用。';note(lastFailure);}
 }
 root.SupportDepth={update,invalidate,visible,checker,ready,stamp:()=>ready()?stamp:null,visibleAt};
})(globalThis);
