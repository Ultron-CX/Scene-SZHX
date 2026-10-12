'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const source=fs.readFileSync(__dirname+'/sample-scope.js','utf8');
function context(path){const calls=[],c={sessionStorage:{test:'per-tab'},URL,location:{pathname:path,href:'http://127.0.0.1:18876'+path,origin:'http://127.0.0.1:18876'},fetch:(...args)=>calls.push(args),document:{addEventListener(){}}};vm.runInNewContext(source,c);return {c,calls};}
const {c,calls}=context('/r1-sample/r1-sample-abcdefgh/');
for(const path of ['/project/save','/semantic/infer','/evidence/id/rgb','/project/dataset/1','/scene.json']){
 c.fetch(path,{method:'POST'});assert.equal(calls.at(-1)[0],'http://127.0.0.1:18876/r1-sample/r1-sample-abcdefgh'+path);
}
assert.equal(c.SceneScope.storage,c.sessionStorage);
assert.equal(c.SceneScope.key('draft'),'scenesem-sample:r1-sample-abcdefgh:draft');
assert.throws(()=>c.fetch('/r1-sample/r1-sample-ijklmnop/project/save'));
assert.throws(()=>c.fetch('https://example.com'));
const live=context('/');assert.equal(live.c.SceneScope,undefined);live.c.fetch('/project/save');assert.equal(live.calls[0][0],'/project/save');
assert.throws(()=>context('/r1-sample/invalid/'));
console.log('PASS isolated URL/key routing, production unchanged, cross-sample/external rejection. VM only.');
