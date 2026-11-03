"""经/python只读审计导航粗包络与当前精确碰撞几何。"""
def audit(executor):
    import json,numpy as np,math,time
    from pathlib import Path
    from pxr import Usd,UsdPhysics
    station=executor._l3_station
    nav=station._approach
    pose,vertical=nav.pose()
    geo=nav.geometry
    hits=geo.collision(pose,all_hits=True)
    shape_paths=[str(p.GetPath()) for p in Usd.PrimRange(station.runtime.stage.GetPseudoRoot(),Usd.TraverseInstanceProxies())
                 if p.HasAPI(UsdPhysics.CollisionAPI) and p.GetAttribute('physics:collisionEnabled').Get() is not False
                 and str(p.GetPath()).startswith('/humanoid/')]
    safety=executor._arm_transit_safety
    q,poses=safety.refresh();model=safety.model
    exact_shapes={s['path']:s for s in model.shapes}
    def isground(path):return any(x in path.lower() for x in ['ground_plane','groundplane'])
    def transform(shape):
        t=poses[shape['body']] if shape['body'] else np.eye(4)
        return model.coal.Transform3s(t[:3,:3],t[:3,3])
    def distance(path_a,path_b):
        if path_a not in exact_shapes or path_b not in exact_shapes:return None
        a,b=exact_shapes[path_a],exact_shapes[path_b];result=model.coal.DistanceResult()
        d=float(model.coal.distance(a['geometry'],transform(a),b['geometry'],transform(b),model.request,result))
        return {'distance_m':d,'robot_shape':path_a,'obstacle_shape':path_b,'robot_body':a['body'],'obstacle_body':b['body'],
                'robot_type':a['type'],'obstacle_type':b['type']}
    rows=[]
    for key,depth in hits.items():
        if key.startswith('sweep:'):
            _,module,path=key.split(':',2)
            candidates=[p for p in shape_paths if ('caster_'+module+'_link' in p or module+'_wheel_link' in p)]
            exact=[distance(p,path) for p in candidates];exact=[d for d in exact if d]
            rows.append({'kind':'swept_circle','key':key,'module':module,'obstacle':path,'coarse_overlap_depth_m':depth,
                         'actual_nearest':min(exact,key=lambda x:x['distance_m']) if exact else None,'all_current_module_pairs':exact})
        else:
            index,path=key.split(':',1);index=int(index)
            if index>=len(shape_paths):raise RuntimeError('robot box order mismatch')
            rows.append({'kind':'projected_box','key':key,'robot_box_index':index,'robot_shape':shape_paths[index],
                         'robot_box':list(map(lambda v: v.tolist() if isinstance(v,np.ndarray) else float(v),geo.boxes[index])),
                         'obstacle':path,'coarse_overlap_depth_m':depth,'actual_pair':distance(shape_paths[index],path)})
    started=time.perf_counter()
    near=model.query(poses=poses,margin=.08,allowed=lambda a,b: a['robot']==b['robot'] or isground(a['path']) or isground(b['path']))
    query_time=time.perf_counter()-started
    obstacles=[{'path':p,'min':lo.tolist(),'max':hi.tolist()} for lo,hi,p in geo.obstacles if any(p==r['obstacle'] for r in rows)]
    result={'pose':pose.tolist(),'vertical':vertical.tolist(),'coarse_hit_count':len(rows),'coarse_hits':rows,
            'exact_near_pairs':near,'exact_query_time_s':query_time,'obstacles':obstacles,
            'all_robot_shape_count':len(shape_paths),'model_robot_shapes':sum(s['robot'] for s in model.shapes),
            'read_only':True,'current_sim_time':float(executor.world.current_time),
            'nav_start_pose':nav.state.get('start_pose')}
    output=Path('/home/jy/RouteManager/outputs/envelope_recovery');output.mkdir(exist_ok=True)
    (output/'navigation_envelope_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    return result
