(function(root){
 'use strict';
 const match=root.location.pathname.match(/^\/r1-sample\/(r1-sample-[a-z0-9_]{8})\//);
 if(!match){if(root.location.pathname.startsWith('/r1-sample/'))throw Error('无效的隔离样例路径');return;}
 const prefix='/r1-sample/'+match[1];
 const url=value=>{
  const u=new URL(value,root.location.href);
  if(u.origin!==root.location.origin)throw Error('隔离样例禁止跨域请求');
  if(u.pathname.startsWith(prefix+'/'))return u.href;
  if(u.pathname.startsWith('/r1-sample/'))throw Error('禁止跨样例请求');
  u.pathname=prefix+u.pathname;return u.href;
 };
 root.SceneScope={id:match[1],url,storage:root.sessionStorage,key:k=>'scenesem-sample:'+match[1]+':'+k};
 const fetch=root.fetch.bind(root);
 root.fetch=(input,init)=>{if(typeof input!=='string'&&!(input instanceof URL))throw Error('隔离样例仅接受显式 URL');return fetch(url(input),init);};
 document.addEventListener('DOMContentLoaded',()=>{
  const notice=document.createElement('div');notice.textContent='隔离验收样例 · '+match[1]+' · 关闭标签页会丢失未保存临时草稿，请保存项目';notice.setAttribute('role','status');
  notice.style.cssText='background:#775000;color:white;padding:8px;position:fixed;bottom:0;right:0;z-index:10000';document.body.appendChild(notice);
 });
})(globalThis);
