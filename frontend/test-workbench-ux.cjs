const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const html=fs.readFileSync(__dirname+'/workbench.html','utf8'),script=fs.readFileSync(__dirname+'/workbench-ux.js','utf8');
const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);assert.equal(new Set(ids).size,ids.length);
assert(!html.match(/<header>[\s\S]*?id="export"[\s\S]*?<\/header>/),'legacy export is not a primary toolbar action');
assert(html.indexOf('id="category"')>html.indexOf('id="editorPanel"'));assert(html.indexOf('id="category"')<html.indexOf('id="completionTools"'));
assert(html.indexOf('id="geometryNote"')>html.indexOf('id="selectionDiagnostics"'));
assert(html.indexOf('id="categorySummary"')>html.indexOf('id="regionDetails"'));
assert(html.includes('<details id="initialActions">'));assert(html.includes('src="/workbench-ux.js"'));
assert(html.includes('id="exportMain"'));assert(html.includes('id="saveState"'));
const projectMarkup=fs.readFileSync(__dirname+'/project-ui.js','utf8');
assert(projectMarkup.indexOf('id="batchRegister"')>projectMarkup.indexOf('<details id="legacyProjectRegistration">'),'legacy intake stays folded');
const nodes=new Map();function node(){return {hidden:false,open:false,classList:{toggle(){}},onclick:null,click(){return this.onclick?.();}};}const get=id=>{if(!nodes.has(id))nodes.set(id,node());return nodes.get(id);};
let writes=0,resets=0,exportCount=0,fail=false,pending=false,reviewDraft=false,activeTool=false,commitFails=false;
const context={console,$:get,document:{body:{dataset:{},classList:{toggle(){}}}},association:null,draft:{id:'r'},manualActive:false,draftChanged:false,anchors:[],msg:s=>context.message=s,
 CompletionUI:{active:()=>activeTool},CleanupUI:{active:()=>false},ObjectUI:{busy:()=>false,hasPending:()=>pending},
 ProjectUI:{refresh(){},exportSaved(){exportCount++;},saveState:()=>({revision:writes,changed:!writes,busy:false}),hasReviewDraft:()=>reviewDraft,saveWorkbench:async()=>{if(fail)throw Error('conflict');writes++;}},resetDraft:()=>{resets++;context.draft=null;}};
get('commit').onclick=()=>{if(!commitFails)context.draftChanged=false;};vm.createContext(context);vm.runInContext(script,context);
(async()=>{
 assert.equal(context.document.body.dataset.workspaceMode,'annotation');get('projectManageToggle').click();assert.equal(context.document.body.dataset.workspaceMode,'project');assert.equal(get('projectManagement').hidden,false);assert.equal(get('projectPanel').hidden,true);
 get('projectToggle').click();assert.equal(context.document.body.dataset.workspaceMode,'quality');assert.equal(get('projectManagement').hidden,true);assert.equal(get('projectPanel').hidden,false);get('annotationMode').click();
 pending=true;get('projectToggle').click();assert.equal(context.document.body.dataset.workspaceMode,'annotation');await get('saveContinue').click();assert.equal(writes,0);assert.equal(resets,0);pending=false;
 activeTool=true;get('projectManageToggle').click();assert.equal(context.document.body.dataset.workspaceMode,'annotation');await get('saveContinue').click();assert.equal(writes,0);activeTool=false;
 reviewDraft=true;await get('saveContinue').click();assert.equal(writes,0);reviewDraft=false;
 context.anchors=[1];context.draft=null;context.draftChanged=true;await get('saveContinue').click();assert.equal(writes,0);assert.equal(context.anchors.length,1);context.anchors=[];context.draft={id:'r'};
 commitFails=true;await get('saveContinue').click();assert.equal(writes,0);assert.equal(context.draftChanged,true);commitFails=false;
 fail=true;await get('saveContinue').click();assert.equal(writes,0);assert.equal(resets,0);assert(context.draft);assert.match(context.message,/conflict/);fail=false;
 await get('saveMain').click();assert.equal(writes,1);assert.equal(resets,0);await get('saveContinue').click();assert.equal(writes,2);assert.equal(resets,1);assert.match(context.message,/没有自动改变质检结论/);
 get('continueAnnotation').click();assert.equal(get('regionDetails').open,true);assert.equal(writes,2);
 fail=true;await get('exportMain').click();assert.equal(exportCount,0);assert.equal(writes,2);fail=false;
 pending=true;await get('exportMain').click();assert.equal(exportCount,0);pending=false;
 context.draftChanged=true;await get('exportMain').click();assert.equal(exportCount,1);assert.equal(writes,3);assert.equal(context.draftChanged,false);assert.equal(resets,1,'export preserves selection');
 assert.equal(get('saveState').textContent,'已保存 v3');context.draftChanged=true;context.WorkbenchUX.refresh();assert.match(get('saveState').textContent,/未应用/);
 console.log('PASS UX shell: separated modes, progressive disclosure contracts, save-before-deselect, candidate/tool/form/conflict guards. Synthetic DOM/VM only; no browser layout acceptance.');
})().catch(e=>{console.error(e);process.exitCode=1;});
