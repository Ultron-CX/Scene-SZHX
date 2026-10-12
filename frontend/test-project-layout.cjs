const assert=require('node:assert/strict');
const fs=require('node:fs');
const css=fs.readFileSync(require('node:path').join(__dirname,'project.css'),'utf8');
// Static guard only. Actual layout and pixel interaction also require browser QA.
const rules=[...css.matchAll(/#evidenceCanvas\s*\{([^}]+)\}/g)];
const style=Object.assign({},...rules.map(m=>Object.fromEntries(m[1].split(';').filter(Boolean).map(d=>d.split(':').map(x=>x.trim())))));
assert.equal(style.position,'static','Replay canvas must not cover the dialog controls');
assert.equal(style.inset,'auto');
assert.equal(style.width,'auto');
assert.equal(style['max-width'],'100%');
assert.equal(style.height,'auto');
console.log('S1 replay canvas layout guard passed (not a browser test).');
