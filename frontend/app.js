'use strict';
const $ = id => document.getElementById(id), core = AnnotationCore;
const canvas = $('canvas'), ctx = canvas.getContext('2d');
let data, labels = new Map(), selection = new Set(), history = [], projected = [], yaw = -.4, pitch = .65, zoom = 1, pan = [0,0], mode = 'orbit', drag = null, dirty = false, storageKey;
let center, extent, framePending = false;
const message = s => { $('message').textContent = s; };
function color(s) {let h = 0; for (const c of s) h = (h * 31 + c.charCodeAt(0)) >>> 0; return `hsl(${h % 360} 72% 65%)`;}
function requestDraw() {if (!framePending) {framePending = true; requestAnimationFrame(() => {framePending = false; draw();});}}
function draw() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w || canvas.height !== h) {canvas.width = w; canvas.height = h;}
  ctx.fillStyle = '#0b1118'; ctx.fillRect(0,0,w,h); if (!data) return;
  const scale = Math.min(w,h) * .8 / extent * zoom, cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
  const low = +$('zmin').value, high = +$('zmax').value;
  projected = data.xyz.map(p => {
    if (p[2] < low || p[2] > high) return null;
    const x = p[0]-center[0], y = p[1]-center[1], z = p[2]-center[2], u = cy*x-sy*y, v = sy*x+cy*y;
    return [w/2+pan[0]+u*scale,h/2+pan[1]-(cp*z-sp*v)*scale,sp*z+cp*v];
  });
  // Draw exactly the same one-pixel frontmost sample used by non-through picking.
  const depth = new Float64Array(w*h).fill(-Infinity), owners = new Int32Array(w*h).fill(-1);
  for (let i=0;i<projected.length;i++) {const p=projected[i];if(!p)continue;const x=Math.round(p[0]),y=Math.round(p[1]);if(x<0||x>=w||y<0||y>=h)continue;const j=y*w+x;if(p[2]>depth[j]){depth[j]=p[2];owners[j]=i;}}
  for (let j=0;j<owners.length;j++) {const i=owners[j];if(i<0)continue;const label=labels.get(data.ids[i]);ctx.fillStyle=selection.has(i)?'#ffffff':label&&$('colored').checked?color(label.category+' / '+label.instance):`rgb(${data.rgb[i].join(',')})`;ctx.fillRect(j%w,Math.floor(j/w),1,1);}
  if(drag && drag.mode==='select'){const r=rectangle(drag.start,drag.last);ctx.fillStyle='#5de0c51a';ctx.fillRect(r[0],r[1],r[2]-r[0],r[3]-r[1]);ctx.strokeStyle='#5de0c5';ctx.strokeRect(r[0],r[1],r[2]-r[0],r[3]-r[1]);}
  $('badge').textContent=`已选 ${selection.size.toLocaleString()} 点 · ${mode==='select'?'框选模式':'旋转模式'}${$('through').checked?' · 穿透开启':''}`;
}
function persist() {dirty=true;try{localStorage.setItem(storageKey,JSON.stringify(core.serialize(labels,data)));}catch(e){message('本地草稿保存失败，请立即导出 JSON：'+e.message);}}
function stats() {
  $('summary').textContent=`${data.ids.length.toLocaleString()} 代表点 / ${labels.size.toLocaleString()} 已标注`;
  const groups=new Map();for(const value of labels.values()){const key=value.category+' / '+value.instance+' / '+(value.status==='reviewed'?'已复核':'待复核');groups.set(key,(groups.get(key)||0)+1);}
  $('instances').replaceChildren();if(!groups.size)$('instances').textContent='暂无标签';
  for(const [name,count] of groups){const item=document.createElement('div');item.textContent=`${name} · ${count}`;item.style.color=color(name.split(' / ').slice(0,2).join(' / '));$('instances').append(item);}
  $('undo').disabled=!history.length;requestDraw();
}
function commit(next) {history.push(labels);if(history.length>30)history.shift();labels=next;persist();stats();}
function apply(erase=false) {
  if(!selection.size){message('请先切换框选模式并拖出一个选区');return;}
  const category=$('category').value.trim(),instance=$('instance').value.trim();if(!erase&&!category){message('请输入类别');return;}
  const next=new Map(labels);for(const i of selection){const id=data.ids[i];if(erase)next.delete(id);else next.set(id,{category,instance,status:$('review').value});}
  commit(next);message(`${erase?'已清除':'已标注'} ${selection.size} 个代表点；可撤销`);
}
function undo(){if(!history.length)return;labels=history.pop();persist();stats();message('已撤销上一步');}
function rectangle(a,b){return [Math.min(a[0],b[0]),Math.min(a[1],b[1]),Math.max(a[0],b[0]),Math.max(a[1],b[1])];}
function position(e){const r=canvas.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];}
function setMode(next){mode=next;$('orbit').classList.toggle('active',mode==='orbit');$('select').classList.toggle('active',mode==='select');canvas.style.cursor=mode==='select'?'crosshair':'grab';requestDraw();}
$('orbit').onclick=()=>setMode('orbit');$('select').onclick=()=>setMode('select');
$('apply').onclick=()=>apply();$('erase').onclick=()=>apply(true);$('undo').onclick=undo;
$('deselect').onclick=()=>{selection.clear();requestDraw();};
$('colored').onchange=requestDraw;$('through').onchange=()=>{selection.clear();message($('through').checked?'穿透开启：选框内前后层都会选中':'仅选择每个像素最前方的点');requestDraw();};
for(const id of ['zmin','zmax'])$(id).oninput=()=>{if(+$('zmin').value>+$('zmax').value)$(id==='zmin'?'zmax':'zmin').value=$(id).value;selection.clear();for(const k of ['zmin','zmax'])$(k+'out').textContent=(+$(k).value).toFixed(2)+' m';requestDraw();};
$('top').onclick=()=>{pitch=Math.PI/2;yaw=0;selection.clear();requestDraw();};$('front').onclick=()=>{pitch=0;yaw=0;selection.clear();requestDraw();};
$('reset').onclick=()=>{yaw=-.4;pitch=.65;zoom=1;pan=[0,0];selection.clear();if(data){$('zmin').value=$('zmin').min;$('zmax').value=$('zmax').max;$('zmin').oninput();}requestDraw();};
canvas.oncontextmenu=e=>e.preventDefault();canvas.onpointerdown=e=>{if(!data)return;canvas.setPointerCapture(e.pointerId);drag={start:position(e),last:position(e),mode:e.button===2?'pan':mode,add:e.shiftKey};};
canvas.onpointermove=e=>{if(!drag)return;const p=position(e),dx=p[0]-drag.last[0],dy=p[1]-drag.last[1];if(drag.mode==='orbit'){yaw+=dx*.008;pitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,pitch+dy*.008));}if(drag.mode==='pan'){pan[0]+=dx;pan[1]+=dy;}drag.last=p;requestDraw();};
canvas.onpointerup=e=>{if(!drag)return;if(drag.mode==='select'){draw();const picked=core.selectRect(projected,rectangle(drag.start,position(e)),canvas.width,canvas.height,$('through').checked);selection=drag.add?new Set([...selection,...picked]):picked;message(`已选 ${selection.size} 点；Shift 框选可追加，白色为选中点`);}drag=null;requestDraw();};
canvas.onpointercancel=()=>{drag=null;requestDraw();};canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(.15,Math.min(25,zoom*Math.exp(-e.deltaY*.001)));requestDraw();};
$('save').onclick=()=>{const blob=new Blob([JSON.stringify(core.serialize(labels,data),null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='digit-k2-labels-'+new Date().toISOString().replace(/[:.]/g,'-')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),10000);dirty=false;message('已请求下载标签文件，请确认浏览器下载完成');};
$('load').onclick=()=>$('import').click();$('import').onchange=async e=>{const file=e.target.files[0];if(!file)return;try{if(file.size>30*1024*1024)throw Error('标签文件超过 30 MB');const next=core.validate(JSON.parse(await file.text()),data);if(labels.size&&!confirm('导入将替换当前标签（可撤销）。继续吗？'))return;commit(next);selection.clear();message(`已导入 ${labels.size} 个标签`);}catch(err){message('导入失败：'+err.message);}finally{e.target.value='';}};
document.addEventListener('keydown',e=>{if(['INPUT','SELECT','TEXTAREA'].includes(document.activeElement.tagName))return;if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='z'){e.preventDefault();undo();}});
window.addEventListener('beforeunload',e=>{if(dirty){e.preventDefault();e.returnValue='';}});new ResizeObserver(requestDraw).observe(canvas.parentElement);
async function init(){try{const response=await fetch('/data.json');if(!response.ok)throw Error('点云服务 '+response.status);data=await response.json();center=[0,0,0];const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];for(const p of data.xyz)for(let i=0;i<3;i++){min[i]=Math.min(min[i],p[i]);max[i]=Math.max(max[i],p[i]);}center=min.map((v,i)=>(v+max[i])/2);extent=Math.max(...max.map((v,i)=>v-min[i]));for(const id of ['zmin','zmax']){$(id).min=min[2];$(id).max=max[2];$(id).value=id==='zmin'?min[2]:max[2];$(id+'out').textContent=(+$(id).value).toFixed(2)+' m';}
storageKey='digit-annotation-v1:'+data.source_sha256+':block01';let restored=false;try{const saved=localStorage.getItem(storageKey);if(saved){labels=core.validate(JSON.parse(saved),data);restored=true;dirty=true;}}catch(e){message('草稿未恢复：'+e.message);}
$('asset').textContent=data.name;for(const id of ['apply','erase','deselect','save','load'])$(id).disabled=false;stats();message(restored?'已恢复本浏览器草稿；导出 JSON 可备份':'点云已就绪；切换框选模式开始标注');
if(document.modelContext?.registerTool){try{await document.modelContext.registerTool({name:'read_annotation_status',description:'读取当前局部点云的选中点和标签数量，不修改数据',inputSchema:{type:'object',properties:{},additionalProperties:false},annotations:{readOnlyHint:true},execute:()=>({selected:selection.size,labelled:labels.size,total:data.ids.length})});}catch(e){console.warn('Optional WebMCP unavailable',e);}}
}catch(e){message('加载失败：'+e.message);$('asset').textContent='数据加载失败';}}
init();
