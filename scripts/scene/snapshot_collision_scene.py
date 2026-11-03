"""经现有主线程导出场景副本、实测位姿及碰撞几何，不移动机器人。"""
def snapshot(executor, output):
    import json
    from pathlib import Path
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema
    import omni.usd
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    stage=omni.usd.get_context().get_stage()
    stage.GetRootLayer().Export(str(output/'scene_root.usda'))
    stage.Flatten().Export(str(output/'scene_flat.usdc'))
    robot=executor.articulation
    def T(pos,quat):
        m=np.eye(4);m[:3,:3]=Rotation.from_quat(quat).as_matrix();m[:3,3]=pos;return m
    def anchor(pos,quat):
        return T(list(pos),[*quat.GetImaginary(),quat.GetReal()]).tolist()
    pose={}
    for view in [robot]+[v for n in ['_oxford_devices'] if hasattr(executor,n) for v in getattr(executor,n).devices.values()]+list(executor._l3_station.runtime.devices.values()):
        vals=np.asarray(view._physics_view.get_link_transforms())[0]
        paths=view._physics_view.link_paths[0]
        for path,v in zip(paths,vals):pose[path]=T(v[:3],v[3:]).tolist()
    r={'robot_root':executor.robot_prim_path,'names':list(robot.dof_names),
       'q':np.asarray(robot.get_joint_positions()).reshape(-1).tolist(),
       'targets':np.asarray(robot._physics_view.get_dof_position_targets()).reshape(-1).tolist(),
       'gains':[np.asarray(v).tolist() for v in robot.get_gains()],
       'max_forces':np.asarray(robot._physics_view.get_dof_max_forces()).tolist(),
       'limits':np.asarray(robot.get_dof_limits()).tolist(),
       'poses':pose,'joints':[],'bodies':{},'shapes':[],
       'mimic':getattr(executor,'_mimic_map',None),
       'physics_dt':executor.world.get_physics_dt()}
    arrays={};cache=UsdGeom.XformCache()
    for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
        if not p.IsActive():continue
        path=str(p.GetPath())
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            r['bodies'][path]={'name':p.GetName(),'robot':path.startswith(executor.robot_prim_path+'/')}
        if p.IsA(UsdPhysics.Joint) and path.startswith(executor.robot_prim_path+'/'):
            j=UsdPhysics.Joint(p);b0=j.GetBody0Rel().GetTargets();b1=j.GetBody1Rel().GetTargets()
            r['joints'].append({'name':p.GetName(),'path':path,'type':p.GetTypeName(),
                'body0':str(b0[0]) if b0 else None,'body1':str(b1[0]) if b1 else None,
                'T0':anchor(j.GetLocalPos0Attr().Get(),j.GetLocalRot0Attr().Get()),
                'T1':anchor(j.GetLocalPos1Attr().Get(),j.GetLocalRot1Attr().Get()),
                'axis':p.GetAttribute('physics:axis').Get(),
                'collision_enabled':j.GetCollisionEnabledAttr().Get()})
        if not p.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is False:continue
        owner=p
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
        body=str(owner.GetPath()) if owner else None
        matrix=np.array(cache.GetLocalToWorldTransform(p)).T
        if body:
            matrix=np.linalg.inv(np.array(cache.GetLocalToWorldTransform(owner)).T)@matrix
            pose.setdefault(body,np.array(cache.GetLocalToWorldTransform(owner)).T.tolist())
        row={'path':path,'body':body,'type':p.GetTypeName(),'robot':bool(body and body.startswith(executor.robot_prim_path+'/')),
             'approximation':UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() if p.HasAPI(UsdPhysics.MeshCollisionAPI) else None}
        if p.IsA(UsdGeom.Mesh):
            m=UsdGeom.Mesh(p);vertices=np.asarray(m.GetPointsAttr().Get(),dtype=np.float64)
            counts=list(m.GetFaceVertexCountsAttr().Get());idx=list(m.GetFaceVertexIndicesAttr().Get());faces=[];start=0
            for n in counts:
                for k in range(1,n-1):faces.append([idx[start],idx[start+k],idx[start+k+1]])
                start+=n
        elif p.IsA(UsdGeom.Cube):
            import itertools
            vertices=np.array(list(itertools.product([-1,1],repeat=3)))*float(UsdGeom.Cube(p).GetSizeAttr().Get())/2
            from scipy.spatial import ConvexHull
            faces=ConvexHull(vertices).simplices
        else:
            row['matrix']=matrix.tolist()
            row['attributes']={a.GetName():str(a.Get()) for a in p.GetAttributes() if a.GetName() in ['axis','radius','height','extent']}
            r['shapes'].append(row);continue
        key=f'shape_{len(r["shapes"])}'
        vertices=vertices@matrix[:3,:3].T+matrix[:3,3]
        arrays[key+'_v']=vertices;arrays[key+'_f']=np.asarray(faces,dtype=np.int32)
        row['key']=key;r['shapes'].append(row)
    np.savez_compressed(output/'geometry.npz',**arrays)
    (output/'snapshot.json').write_text(json.dumps(r,ensure_ascii=False,indent=2))
    return {'robot_shapes':sum(x['robot'] for x in r['shapes']),'all_shapes':len(r['shapes']),'robot_joints':len(r['joints'])}
