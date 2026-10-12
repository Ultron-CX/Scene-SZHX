#!/usr/bin/env python3
"""Convert an authoritative X2 PhysX gait trace into renderer-only link poses.

The converter performs URDF forward kinematics only.  It never starts physics,
publishes commands, or changes the source evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, 0],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, 0],
                     [-sp, cp*sr, cp*cr, 0], [0, 0, 0, 1]], dtype=float)


def quat_matrix_wxyz(q):
    w, x, y, z = q
    n = math.sqrt(w*w+x*x+y*y+z*z)
    w, x, y, z = (v/n for v in (w,x,y,z))
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), 0],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), 0],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), 0], [0,0,0,1]], dtype=float)


def matrix_quat_xyzw(m):
    r = m[:3,:3]; tr = float(np.trace(r))
    if tr > 0:
        s=math.sqrt(tr+1.0)*2; w=.25*s; x=(r[2,1]-r[1,2])/s; y=(r[0,2]-r[2,0])/s; z=(r[1,0]-r[0,1])/s
    elif r[0,0] > r[1,1] and r[0,0] > r[2,2]:
        s=math.sqrt(1+r[0,0]-r[1,1]-r[2,2])*2; w=(r[2,1]-r[1,2])/s; x=.25*s; y=(r[0,1]+r[1,0])/s; z=(r[0,2]+r[2,0])/s
    elif r[1,1] > r[2,2]:
        s=math.sqrt(1+r[1,1]-r[0,0]-r[2,2])*2; w=(r[0,2]-r[2,0])/s; x=(r[0,1]+r[1,0])/s; y=.25*s; z=(r[1,2]+r[2,1])/s
    else:
        s=math.sqrt(1+r[2,2]-r[0,0]-r[1,1])*2; w=(r[1,0]-r[0,1])/s; x=(r[0,2]+r[2,0])/s; y=(r[1,2]+r[2,1])/s; z=.25*s
    q=np.array([x,y,z,w]); q/=np.linalg.norm(q)
    return q.tolist()


def transform(xyz=(0,0,0), rpy=(0,0,0)):
    m=rpy_matrix(rpy); m[:3,3]=xyz; return m


def axis_motion(axis, angle):
    a=np.asarray(axis,dtype=float); a/=np.linalg.norm(a); x,y,z=a; c=math.cos(angle); s=math.sin(angle); C=1-c
    m=np.eye(4); m[:3,:3]=[[c+x*x*C,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,c+y*y*C,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,c+z*z*C]]
    return m


def vec(text, default):
    return tuple(map(float, text.split())) if text else default


def load_urdf(path: Path):
    root=ET.parse(path).getroot(); links=[e.get('name') for e in root.findall('link')]
    children={}; child_links=set(); movable=[]
    for e in root.findall('joint'):
        parent=e.find('parent').get('link'); child=e.find('child').get('link'); child_links.add(child)
        origin=e.find('origin'); axis=e.find('axis')
        j={'name':e.get('name'),'type':e.get('type'),'parent':parent,'child':child,
           'origin':transform(vec(origin.get('xyz') if origin is not None else None,(0,0,0)), vec(origin.get('rpy') if origin is not None else None,(0,0,0))),
           'axis':vec(axis.get('xyz') if axis is not None else None,(1,0,0))}
        children.setdefault(parent,[]).append(j)
        if j['type'] != 'fixed': movable.append(j['name'])
    roots=[x for x in links if x not in child_links]
    if roots != ['base_link']: raise ValueError(f'unexpected URDF roots: {roots}')
    return links, children, movable


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--source-dir',type=Path,required=True); ap.add_argument('--urdf',type=Path,required=True)
    ap.add_argument('--manifest',type=Path,required=True); ap.add_argument('--output',type=Path,required=True); ap.add_argument('--receipt',type=Path,required=True)
    ap.add_argument('--start-s',type=float,default=8.5); ap.add_argument('--end-s',type=float,default=25.5); ap.add_argument('--fps',type=float,default=30.0)
    a=ap.parse_args(); result_path=a.source_dir/'result.json'; trace_path=a.source_dir/'gait1-trace.json'
    result=json.loads(result_path.read_text()); trace=json.loads(trace_path.read_text()); manifest=json.loads(a.manifest.read_text())
    if result.get('status')!='PASSED' or result.get('physics_authority')!='genie_sim_isaac_physx' or result.get('real_robot_commands') is not False: raise SystemExit('unsafe or unsuccessful source')
    if trace.get('schema')!='sceneforge/x2-physx-locomotion-samples/v1': raise SystemExit('wrong trace schema')
    if a.output.exists() or a.receipt.exists(): raise SystemExit('refusing overwrite')
    links,children,movable=load_urdf(a.urdf); source_names=trace['joint_names']; missing=sorted(set(movable)-set(source_names))
    base=trace['base_samples']; physics=trace['physics_samples']; count=min(len(base),len(physics)); times=np.array([float(base[i]['sim_time']) for i in range(count)])
    targets=np.arange(a.start_s,a.end_s+0.25/a.fps,1/a.fps); indices=np.searchsorted(times,targets); indices=np.clip(indices,0,count-1)
    root_pose=manifest['robot_root_pose']; root=transform((root_pose['x'],root_pose['y'],root_pose['z']),(0,0,root_pose['theta'])); inv_root=np.linalg.inv(root)
    path_by_link={'base_link':'/x2/Geometry/base_link'}; ordered=['base_link']
    def walk(parent):
        for j in children.get(parent,[]): path_by_link[j['child']]=path_by_link[parent]+'/'+j['child']; ordered.append(j['child']); walk(j['child'])
    walk('base_link')
    header={'record_type':'schema','schema':'sceneforge/x2-geniesim-render-state/v1','authoritative_physics_source':'geniesim_isaac_physx','base_topic':'offline_authoritative_gait_trace','tf_topic':'offline_urdf_fk','sample_clock':'simulation_time','target_sample_rate_hz':a.fps,'read_only':True,'published_topics':[],'real_robot_commands':False}
    partial=a.output.with_name(a.output.name+'.partial'); a.output.parent.mkdir(parents=True,exist_ok=False) if not a.output.parent.exists() else None
    with partial.open('x',encoding='utf-8') as f:
        f.write(json.dumps(header,sort_keys=True)+'\n')
        for seq,(target,idx) in enumerate(zip(targets,indices)):
            bs=base[int(idx)]; ps=physics[int(idx)]; qmap=dict(zip(source_names,ps['position']))
            world=quat_matrix_wxyz(bs['orientation_wxyz']); world[:3,3]=bs['position_m']; base_local=inv_root@world
            transforms=[]
            def add(link,m): transforms.append({'child_path':path_by_link[link],'parent_path':'map','translation_xyz_m':m[:3,3].tolist(),'rotation_xyzw':matrix_quat_xyzw(m)})
            add('base_link',base_local)
            def emit(parent):
                for j in children.get(parent,[]):
                    m=j['origin']
                    if j['type'] in ('revolute','continuous'): m=m@axis_motion(j['axis'],float(qmap.get(j['name'],0.0)))
                    add(j['child'],m); emit(j['child'])
            emit('base_link')
            rec={'record_type':'sample','schema':'sceneforge/x2-geniesim-render-state/v1','sequence':seq,'sim_time_ns':int(round(float(target)*1e9)),'sim_time_s':float(target),'base_position_xyz_m':list(map(float,bs['position_m'])),'base_orientation_xyzw':[float(bs['orientation_wxyz'][1]),float(bs['orientation_wxyz'][2]),float(bs['orientation_wxyz'][3]),float(bs['orientation_wxyz'][0])],'transforms':transforms}
            f.write(json.dumps(rec,separators=(',',':'))+'\n')
        f.write(json.dumps({'record_type':'summary','schema':'sceneforge/x2-geniesim-render-state/v1','status':'SUCCEEDED','sample_count':len(targets),'first_sim_time_s':float(targets[0]),'last_sim_time_s':float(targets[-1]),'read_only':True,'real_robot_commands':False},sort_keys=True)+'\n'); f.flush(); os.fsync(f.fileno())
    os.replace(partial,a.output)
    receipt={'schema':'sceneforge/x2-geniesim-render-state-receipt/v1','status':'SUCCEEDED','authoritative_physics_source':'geniesim_isaac_physx','source_result_sha256':sha256(result_path),'source_trace_sha256':sha256(trace_path),'urdf_sha256':sha256(a.urdf),'manifest_sha256':sha256(a.manifest),'output_sha256':sha256(a.output),'output_size':a.output.stat().st_size,'sample_count':len(targets),'link_count':len(ordered),'source_start_s':a.start_s,'source_end_s':a.end_s,'target_sample_rate_hz':a.fps,'defaulted_joint_names':missing,'defaulted_joint_value_rad':0.0,'method':'offline_urdf_forward_kinematics_from_authoritative_physx_state','read_only':True,'published_topics':[],'mujoco_state_source_used':False,'real_robot_commands':False}
    tmp=a.receipt.with_name(a.receipt.name+'.tmp'); tmp.write_text(json.dumps(receipt,indent=2,sort_keys=True)+'\n'); os.replace(tmp,a.receipt); print(json.dumps(receipt,indent=2))

if __name__=='__main__': main()
