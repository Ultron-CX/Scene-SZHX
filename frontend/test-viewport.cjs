const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const nodes=new Map(),get=id=>{if(!nodes.has(id))nodes.set(id,{clientWidth:300,clientHeight:150,getContext:()=>({})});return nodes.get(id);};
const callbacks=[],calls=[];
const context={BoxCore:require('./boxes.js'),document:{getElementById:get},pc:{FILLMODE_NONE:'NONE'},requestAnimationFrame:fn=>callbacks.push(fn)};
vm.createContext(context);
vm.runInContext(fs.readFileSync(__dirname+'/scene-app.js','utf8').replace(/init\(\);\s*$/,''),context);
context.testApp={setCanvasFillMode:(...args)=>calls.push(args)};
vm.runInContext('app=testApp;cam={camera:{}};',context);
for(const [w,h] of [[900,600],[1700,980],[900,600]]){
 Object.assign(get('viewport'),{clientWidth:w,clientHeight:h});
 vm.runInContext('scheduleViewportResize();scheduleViewportResize();',context);
 assert.equal(callbacks.length,1);callbacks.pop()();
 assert.deepEqual(calls.at(-1),['NONE',w,h]);
 assert.equal(get('overlay').width,w);assert.equal(get('overlay').height,h);
 assert.equal(vm.runInContext('cam.camera.aspectRatio',context),w/h);
}
get('viewport').clientWidth=0;vm.runInContext('syncViewportSize()',context);assert.equal(calls.length,3);
console.log('PASS: container resize/fullscreen-size/restore, overlay and camera sync, coalescing, hidden container guard');
