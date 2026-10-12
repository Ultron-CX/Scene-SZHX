const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path'),{pathToFileURL}=require('node:url');
(async()=>{
 const {Vec3}=await import(pathToFileURL(path.resolve(__dirname,'../vendor/playcanvas/build/playcanvas/src/core/math/vec3.js')));
 const nodes=new Map(),get=id=>{if(!nodes.has(id))nodes.set(id,{value:'',clientHeight:600,getContext:()=>({})});return nodes.get(id);};
 const doc={getElementById:get,activeElement:{tagName:'BODY',isContentEditable:false}};
 const camera={camera:{},position:new Vec3(),forward:new Vec3(),right:new Vec3(),setPosition(x,y,z){this.position.set(x,y,z);},getPosition(){return this.position;},lookAt(p,up){this.forward.copy(p).sub(this.position).normalize();this.right.cross(this.forward,up).normalize();}};
 camera.up=new Vec3();const originalLook=camera.lookAt;camera.lookAt=function(p,up){originalLook.call(this,p,up);this.up.cross(this.right,this.forward).normalize();};
 const context={BoxCore:require('./boxes.js'),document:doc,pc:{Vec3},camera,console};vm.createContext(context);
 vm.runInContext(fs.readFileSync(__dirname+'/scene-app.js','utf8').replace(/init\(\);\s*$/,''),context);
 const run=s=>vm.runInContext(s,context),reset=(angle=.7,radius=5)=>{run(`cam=camera;meta={bounds:[[-20,-20,-2],[20,20,5]]};target=new pc.Vec3();dist=${radius};yaw=0;pitch=${angle};pressed.clear();cameraUpdate();`);};
 const near=(a,b)=>assert(Math.abs(a-b)<1e-7,`${a} != ${b}`),wheel=(delta,options={})=>{context.event={deltaY:delta,deltaMode:0,preventDefault(){},...options};run('wheelMove(event)');};
 // Looking around must not move the eye, regardless of old orbit radius or pitch.
 for(const radius of [.2,6,60,500])for(const angle of [-1.56,-.7,0,.7,1.56]){
  reset(angle,radius);const eye=camera.position.clone(),direction=camera.forward.clone();
  run('lookAround(40,20)');near(camera.position.clone().sub(eye).length(),0);
  assert(camera.forward.clone().sub(direction).length()>1e-4,'Drag must turn the camera, including near the vertical clamp');
  run('cameraUpdate()');near(camera.position.clone().sub(eye).length(),0);
  for(let i=0;i<50;i++)run('lookAround(3,-2)');near(camera.position.clone().sub(eye).length(),0);
  const forward=camera.forward.clone();run("pressed.add('KeyW');movement(.05)");near(camera.position.clone().sub(eye).dot(forward),.125);
 }
 reset(0,60);run('lookAround(40,0)');near(run('yaw'),-.12);assert(camera.forward.y>0,'Dragging right looks right');
 run('lookAround(0,100000)');near(run('pitch'),1.569);run('lookAround(0,-100000)');near(run('pitch'),-1.569);
 const held=camera.position.clone();context.ObjectUI={busy:()=>true};const heldYaw=run('yaw');run('lookAround(100,100)');near(run('yaw'),heldYaw);near(camera.position.clone().sub(held).length(),0);context.ObjectUI=undefined;
 // W/S use the full camera axis for downward, upward and almost vertical views.
 for(const angle of [.7,-.7,1.569]){reset(angle);const before=camera.position.clone(),forward=camera.forward.clone();run("pressed.add('KeyW');movement(.05)");const change=camera.position.clone().sub(before);near(change.dot(forward),.125);assert(Math.sign(change.z)===-Math.sign(angle));run("pressed.clear();pressed.add('KeyS');movement(.05)");near(camera.position.clone().sub(before).length(),0);}
 reset();const before=camera.position.clone(),right=camera.right.clone();run("pressed.add('KeyD');movement(.05)");near(camera.position.clone().sub(before).dot(right),.125);
 reset();run("pressed.add('KeyE');movement(.05)");near(run('target.z'),.125);
 reset();run("pressed.add('KeyW');pressed.add('KeyD');movement(.05)");near(run('target.length()'),.125);
 reset();doc.activeElement.tagName='INPUT';run("pressed.add('KeyW');movement(.05)");near(run('target.length()'),0);doc.activeElement.tagName='BODY';
 // Scroll goes through the old pivot instead of approaching a 20 cm hard stop.
 reset(0,.2);wheel(-120);assert(camera.position.x<0);near(run('dist'),.2);wheel(-120);assert(camera.position.x<-.3);
 // Retreat beyond 300 m; far clipping follows the scene instead of hiding it at 500 m.
 reset(0,300);wheel(120000);assert(camera.position.x>500);assert(camera.camera.farClip>camera.position.x);near(camera.camera.nearClip,.01);
 reset();wheel(3,{deltaMode:1});const lineDistance=run('target.length()');reset();wheel(48);near(run('target.length()'),lineDistance);
 reset();wheel(-120,{altKey:true});near(run('target.length()'),.03);reset();wheel(-120,{shiftKey:true});near(run('target.length()'),1.2);
 reset();wheel(Infinity);near(run('target.length()'),0);context.ObjectUI={busy:()=>true};wheel(-120);run("pressed.add('KeyW');movement(.05)");near(run('target.length()'),0);
 context.ObjectUI=undefined;
 for(const radius of [.2,6,45,500]){reset(.7,radius);const eye=camera.position.clone(),right=camera.right.clone(),up=camera.up.clone(),forward=camera.forward.clone();run('panCamera(10,20)');const delta=camera.position.clone().sub(eye);near(delta.dot(right),-.03);near(delta.dot(up),.06);near(delta.dot(forward),0);near(camera.forward.clone().sub(forward).length(),0);}
 reset();run('panCamera(10,0,{altKey:true,shiftKey:true})');near(run('target.length()'),.003);
 reset();run('panCamera(10,0,{shiftKey:true})');near(run('target.length()'),.09);
 reset();run('panCamera(10000,10000)');near(run('target.length()'),.3);
 reset();run('panCamera(NaN,10);panCamera(0,0)');near(run('target.length()'),0);context.ObjectUI={busy:()=>true};run('panCamera(10,20)');near(run('target.length()'),0);
 assert.match(fs.readFileSync(__dirname+'/scene-app.js','utf8'),/panCamera\(dx,dy,e\)/);
 assert.match(fs.readFileSync(__dirname+'/scene-app.js','utf8'),/addEventListener\('wheel',wheelMove/);
 console.log('PASS: full view-direction W/S, local strafe/world elevation, diagonal speed, typing guard, wheel through pivot/beyond 300m, adaptive far clip, delta units, fine/fast scroll and busy guard. No browser UI test.');
})().catch(error=>{console.error(error);process.exitCode=1;});
