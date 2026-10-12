(function(root){
 'use strict';
 // Presentation/routing only; existing annotation and project contracts stay in charge.
 let saving=false;
 const staged=()=>association||CompletionUI.active()||CleanupUI.active()||ObjectUI.busy();
 function setSection(mode){
  if(staged()){msg('先完成或取消当前识别 / 修正，内容已保留。');return false;}
  if(mode!=='annotation'&&ObjectUI.hasPending?.()){msg('还有未加入的识别候选，请先处理；候选面板保持可见。');return false;}
  document.body.dataset.workspaceMode=mode;
  $('projectPanel').hidden=mode!=='quality';$('projectManagement').hidden=mode!=='project';
  for(const [id,value]of [['annotationMode','annotation'],['projectToggle','quality'],['projectManageToggle','project']])$(id).classList.toggle('active',mode===value);
  ProjectUI.refresh();return true;
 }
 function refresh(){document.body.classList.toggle('has-selection',!!draft||manualActive);const s=ProjectUI.saveState?.();if(s)$('saveState').textContent=s.busy?'正在保存…':draftChanged||anchors.length?'当前物体有未应用修改':s.changed?'有未保存更改':s.revision?'已保存 v'+s.revision:'尚未保存项目';}
 async function save(continueAfter,exportAfter=false){
  if(saving)return;
  try{
   if(staged())throw Error('先应用或取消当前识别 / 修正，再保存项目。');
   if(ProjectUI.hasReviewDraft())throw Error('质检中有未提交填写，请先提交或取消；内容已保留。');
   if(ObjectUI.hasPending?.())throw Error('还有未加入的识别候选，请先加入或丢弃；保存不会自动接受候选。');
   if(anchors.length&&!draft)throw Error('请先点击“生成区域”，再保存项目；选点已保留。');
   if(draftChanged){$('commit').click();if(draftChanged)throw Error('当前类别 / 形状修改未能应用，请检查输入。');}
   saving=true;await ProjectUI.saveWorkbench();if(exportAfter)ProjectUI.exportSaved();
   if(continueAfter){resetDraft();msg('项目已保存。点击下一件物体继续修正；没有自动改变质检结论。');}
  }catch(e){msg('未完成项目保存：'+e.message+' 当前工作内容保留。');}
  finally{saving=false;refresh();}
 }
 // Keep historical QA/business tools available without making them the primary project flow.
 if(document.createElement){const advanced=document.createElement('details'),title=document.createElement('summary');advanced.id='advancedProjectTools';title.textContent='高级：业务来源与历史诊断';advanced.append(title);for(const id of ['businessManage','projectToggle','spatialManage'])advanced.append($(id));$('projectFileControls').append(advanced);}
 $('functionZones').onclick=()=>{if(setSection('project'))$('spatialManage').click();};
 $('annotationMode').onclick=()=>setSection('annotation');
 $('projectManageToggle').onclick=()=>setSection(document.body.dataset.workspaceMode==='project'?'annotation':'project');
 $('projectToggle').onclick=()=>setSection(document.body.dataset.workspaceMode==='quality'?'annotation':'quality');
 $('projectManageClose').onclick=$('projectClose').onclick=()=>setSection('annotation');
 $('saveMain').onclick=()=>save(false);$('saveContinue').onclick=()=>save(true);$('exportMain').onclick=()=>save(false,true);
 $('continueAnnotation').onclick=()=>{if(!setSection('annotation'))return;$('regionDetails').open=true;msg('点击场景中的物体，或从标签列表选择。修正后点击“保存，继续下一件”。');};
 root.WorkbenchUX={refresh,setSection};document.body.dataset.workspaceMode='annotation';refresh();
})(globalThis);
