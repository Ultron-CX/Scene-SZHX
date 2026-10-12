const fs=require('node:fs'),path=require('node:path'),cp=require('node:child_process');
const root=path.resolve(__dirname,'..'),frontend=path.join(root,'frontend'),results=[];
for(const name of fs.readdirSync(frontend).filter(n=>/^test-.*\.cjs$/.test(n)).sort()){
 const r=cp.spawnSync(process.execPath,[path.join(frontend,name)],{cwd:root,encoding:'utf8',timeout:60000});
 process.stdout.write(r.stdout||'');process.stderr.write(r.stderr||'');
 results.push({name,passed:r.status===0,output:r.stdout?.trim(),error:r.stderr?.trim()});if(r.status!==0)process.exitCode=1;
}
if(process.argv.includes('--record')){const dir=path.join(root,'artifacts/migration-20260911');fs.mkdirSync(dir,{recursive:true});fs.writeFileSync(path.join(dir,'regression.json'),JSON.stringify({project:'SceneSem',at:new Date().toISOString(),node:process.version,tests:results,browser_gpu_tested:false,current_object_precision_validated:false,real_robot_commands:false},null,2)+'\n',{flag:'wx'});}
console.log(`${results.filter(r=>r.passed).length}/${results.length} regression scripts passed. Browser/GPU and real semantic accuracy not tested.`);
