"""在独立 Isaac Sim 6.2 副本验证自碰撞，保留正在使用的GUI场景。"""
import argparse,json
from pathlib import Path


def apply_policy(stage, model):
    from pxr import UsdPhysics, PhysxSchema
    root=stage.GetPrimAtPath(model.root)
    PhysxSchema.PhysxArticulationAPI.Apply(root).CreateEnabledSelfCollisionsAttr(True)
    for pair in model.excluded:
        a,b=sorted(pair)
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a)).CreateFilteredPairsRel().AddTarget(b)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    parser.add_argument('--obstruction',action='store_true');args=parser.parse_args();out=args.directory.resolve()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'width':1280,'height':720,'renderer':'RayTracedLighting'})
    try:
        import numpy as np
        import omni.usd
        from pxr import Usd,Sdf,UsdGeom,UsdPhysics,PhysxSchema
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from robot_collision_guard import CollisionModel,clearance_for
        p=json.loads((out/'policy.json').read_text());model=CollisionModel(out/'backup',p['extra_exclusions'])
        source=Usd.Stage.Open(str(out/'backup/scene_flat.usdc'))
        context=omni.usd.get_context();context.new_stage();stage=context.get_stage()
        # Flatten保留了内部原型引用，必须连原型定义一起复制；父级over不是有效碰撞场景。
        stage.GetRootLayer().TransferContent(source.GetRootLayer())
        for child in list(stage.GetPrimAtPath('/World').GetChildren()):
            if child.GetName() not in ['ground_plane','PhysicsMaterials','Physics_Materials']:
                stage.RemovePrim(child.GetPath())
        collider_count=sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in Usd.PrimRange(stage.GetPrimAtPath('/humanoid'),Usd.TraverseInstanceProxies()))
        assert collider_count==96, f'副本碰撞体缺失：{collider_count}'
        assert stage.GetPrimAtPath('/World/ground_plane/geom').IsDefined()
        apply_policy(stage,model)
        world=World(stage_units_in_meters=1.,physics_dt=1/240,rendering_dt=1/60)
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                px=PhysxSchema.PhysxSceneAPI.Apply(prim);px.CreateSolverTypeAttr('TGS');px.CreateTimeStepsPerSecondAttr(240)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and str(prim.GetPath()).startswith('/humanoid/'):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        import omni.physx
        from pxr import PhysicsSchemaTools
        contacts=[]
        def on_contact(headers,data):
            for h in headers:
                paths=[str(PhysicsSchemaTools.intToSdfPath(v)) for v in [h.actor0,h.actor1]]
                if all('/humanoid/' in v for v in paths) and any(v.endswith('torso_pitch_link') or v.endswith('lift_top_link') for v in paths) and any(v.endswith('right_arm_5_link') for v in paths):
                    contacts.append({'actors':paths,'count':h.num_contact_data})
        subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
        world.reset()
        robot=Articulation(model.root,name='self_collision_copy');robot.initialize()
        assert list(robot.dof_names)==model.names,'副本DOF顺序变动'
        q=np.array(json.loads((out/'safe_pose.json').read_text())['q'],dtype=np.float32)[None,:]
        pose=model.poses0[model.root]
        from scipy.spatial.transform import Rotation
        robot.set_world_poses(positions=pose[:3,3][None,:],orientations=Rotation.from_matrix(pose[:3,:3]).as_quat(scalar_first=True)[None,:])
        robot.set_joint_positions(q);robot.set_joint_velocities(np.zeros_like(q));robot.set_joint_position_targets(q)
        robot.set_gains(kps=np.asarray(model.data['gains'][0],dtype=np.float32),kds=np.asarray(model.data['gains'][1],dtype=np.float32))
        robot._physics_view.set_dof_max_forces(np.asarray(model.data['max_forces'],dtype=np.float32),np.array([0],dtype=np.uint32))
        report={'self_collision':robot.get_enabled_self_collisions().tolist(),'dof_count':len(robot.dof_names),
                'collider_count':collider_count,'filtered_body_pairs':len(model.excluded),'samples':[],'pass':False}
        target=q.copy();start=q.copy();idx=[model.names.index(f'right_arm_{i}_joint') for i in range(1,7)]
        for i in range(2400):
            world.step(render=False)
            if i%120==0:
                real=np.asarray(robot.get_joint_positions()).reshape(-1);links=np.asarray(robot._physics_view.get_link_transforms())[0]
                poses={path:model.poses0[path].copy() for path in model.poses0}
                for path,v in zip(robot._physics_view.link_paths[0],links):
                    t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();t[:3,3]=v[:3];poses[path]=t
                hits=model.query(poses=poses,margin=-.0005,only_self=True)
                row={'time':i/240,'finite':bool(np.isfinite(real).all() and np.isfinite(links).all()),
                    'max_q_drift_rad':float(np.max(abs(real-start.reshape(-1)))),
                    'base_drift_m':float(np.linalg.norm(poses[model.root][:3,3]-pose[:3,3])),
                    'penetrations':hits}
                report['samples'].append(row)
                print('STATIC',row['time'],row['max_q_drift_rad'],row['base_drift_m'],len(hits),flush=True)
                if not row['finite'] or row['max_q_drift_rad']>.1 or row['base_drift_m']>.01 or hits:break
        report['pass']=len(report['samples'])==20 and all(x['finite'] and x['max_q_drift_rad']<.1 and x['base_drift_m']<.01 and not x['penetrations'] for x in report['samples'])
        final=np.asarray(robot.get_joint_positions()).reshape(-1)
        report['joint_drift']={n:float(abs(final[k]-start[0,k])) for k,n in enumerate(model.names)}
        report['max_arm_drift_rad']=max(v for n,v in report['joint_drift'].items() if '_arm_' in n)
        report['pass'] &= report['max_arm_drift_rad']<.01
        if args.obstruction and report['pass']:
            probe=[];bad_target=q.copy();bad_target[0,model.names.index('right_arm_2_joint')]+=.15
            for i in range(960):
                u=min(1.,(i+1)/720);goal=q+(bad_target-q)*(u*u*(3-2*u))
                robot.set_joint_position_targets(goal)
                world.step(render=False)
                if i%24==0:
                    real=np.asarray(robot.get_joint_positions()).reshape(-1)
                    links=np.asarray(robot._physics_view.get_link_transforms())[0];poses=dict(model.poses0)
                    for path,v in zip(robot._physics_view.link_paths[0],links):
                        t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();t[:3,3]=v[:3];poses[path]=t
                    hits=model.query(poses=poses,margin=-.0005,only_self=True)
                    probe.append({'time':i/240,'penetrations':hits,'finite':bool(np.isfinite(real).all()),
                        'joint2_target':float(goal[0,model.names.index('right_arm_2_joint')]),
                        'joint2_actual':float(real[model.names.index('right_arm_2_joint')])})
                    if any(h['distance_m']<-.003 for h in hits) or not probe[-1]['finite']:break
            report['obstruction_probe']=probe
            report['obstruction_contacts']=contacts
            report['obstruction_pass']=len(probe)==40 and bool(contacts) and all(h['distance_m']>-.002 for x in probe for h in x['penetrations']) and abs(probe[-1]['joint2_target']-probe[-1]['joint2_actual'])>.01
            report['pass'] &= report['obstruction_pass']
        stage.GetRootLayer().Export(str(out/'isolated_self_collision.usda'))
        (out/'static_validation.json').write_text(json.dumps(report,indent=2))
        print('STATIC_PASS',report['pass'],flush=True)
    finally:app.close()


if __name__=='__main__':main()
