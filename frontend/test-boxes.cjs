const assert=require('node:assert/strict'),B=require('./boxes.js');
const m={scene_id:'test',coordinate_frame:'K2_LCC_SOURCE',source_hashes:{las:'abc'}};
const b={id:'one',category:'wall',sequence:1,status:'draft',...B.fromPoints([[0,0,0],[1,2,3]])};
assert(B.validBox(b));assert.equal(B.nextSequence([b],'wall'),2);assert.equal(B.nextSequence([b],'chair'),1);
assert(B.match([b],.5,1,1,'wall').visible);assert(!B.match([b],.5,1,1,'chair').visible);assert(!B.match([b],.5,1,1,'unknown').visible);assert(B.match([b],20,1,1,'unknown').visible);
const doc=B.save([b],m);assert.deepEqual(B.validate(JSON.parse(JSON.stringify(doc)),m),[b]);assert.throws(()=>B.validate({...doc,source_hashes:{las:'wrong'}},m));assert.throws(()=>B.validate({...doc,boxes:[b,b]},m));assert.throws(()=>B.validate({...doc,boxes:[{...b,min:[NaN,0,0]}]},m));
console.log('PASS: shared 3D boxes, auto numbering, category/unknown filters, JSON roundtrip and invalid data rejection');
