(function (root) {
  'use strict';
  function validate(doc, data) {
    if (!doc || doc.schema !== 'digit.annotations.v1' || doc.source_sha256 !== data.source_sha256 || doc.coordinate_frame !== 'LAS_SOURCE' || doc.coverage !== 'explicit_source_points_only') throw Error('文件版本、源数据或坐标系不匹配');
    if (!Array.isArray(doc.annotations)) throw Error('缺少标签数组');
    const allowed = new Set(data.ids), seen = new Set();
    for (const item of doc.annotations) {
      if (!item || !Number.isSafeInteger(item.point_id) || !allowed.has(item.point_id) || seen.has(item.point_id)) throw Error('存在未知或重复点编号');
      if (typeof item.category !== 'string' || !item.category.trim() || item.category.length > 80 || typeof item.instance !== 'string' || item.instance.length > 80 || !['draft', 'reviewed'].includes(item.status)) throw Error('标签字段不合法');
      seen.add(item.point_id);
    }
    return new Map(doc.annotations.map(x => [x.point_id, { category: x.category, instance: x.instance, status: x.status }]));
  }
  function serialize(labels, data) {
    return {schema:'digit.annotations.v1', source_sha256:data.source_sha256, coordinate_frame:'LAS_SOURCE', coverage:'explicit_source_points_only', unrepresented_points:'UNKNOWN', real_robot_commands:false, saved_at:new Date().toISOString(), annotations:[...labels].map(([point_id, value]) => ({point_id, ...value}))};
  }
  function selectRect(projected, rect, width, height, through) {
    const result = new Set(), depth = new Float64Array(width * height).fill(-Infinity), owner = new Int32Array(width * height).fill(-1);
    for (let i = 0; i < projected.length; i++) {
      const p = projected[i]; if (!p) continue;
      const x = Math.round(p[0]), y = Math.round(p[1]);
      if (x < 0 || x >= width || y < 0 || y >= height) continue;
      if (through) {if (x >= rect[0] && x <= rect[2] && y >= rect[1] && y <= rect[3]) result.add(i);}
      else if (p[2] > depth[y * width + x]) {depth[y * width + x] = p[2]; owner[y * width + x] = i;}
    }
    if (!through) for (let y = Math.max(0, Math.ceil(rect[1])); y <= Math.min(height - 1, rect[3]); y++) for (let x = Math.max(0, Math.ceil(rect[0])); x <= Math.min(width - 1, rect[2]); x++) {const i = owner[y * width + x]; if (i >= 0) result.add(i);}
    return result;
  }
  root.AnnotationCore = {validate, serialize, selectRect};
  if (typeof module !== 'undefined') module.exports = root.AnnotationCore;
})(globalThis);
