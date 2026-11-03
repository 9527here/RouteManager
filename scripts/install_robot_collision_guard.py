"""在副本验收通过后，为当前人形安装自碰撞过滤与路径保护。"""
import json
from pathlib import Path


def activate(executor,directory,output):
    import numpy as np
    from pxr import UsdPhysics,PhysxSchema
    import omni.usd
    from robot_collision_guard import MotionGuard,clearance_for
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    acceptance=json.loads((output/'static_validation.json').read_text())
    if not acceptance.get('pass'):raise RuntimeError('副本静止与碰撞阻挡验收未通过，不能安装')
    station=executor._l3_station
    if station.queue or station._robot_busy:raise RuntimeError('请等待当前动作结束')
    stage=omni.usd.get_context().get_stage()
    stage.GetRootLayer().Export(str(output/'before_activation.usda'))
    guard=MotionGuard(executor,directory);guard.enabled=False
    executor._collision_guard=guard
    q=guard.refresh();original=q.copy();hits=guard.violations(q)
    recovery={'initial':hits,'samples':[]}
    if hits:
        baseline={}
        for h in hits:
            pair=frozenset([h['a'],h['b']]);baseline[pair]=min(baseline.get(pair,0),h['distance_m'])
        chosen=None
        # 只允许已经在副本验证的向外松开方向，逐点禁止加深或新增穿插。
        for delta in [-.10,-.15,-.20]:
            target=q.copy();target[guard.model.names.index('right_arm_2_joint')]+=delta
            if guard.violations(target):continue
            good=True
            for u in np.linspace(0,1,151):
                rows=guard.model.query(q+(target-q)*u,margin=-.0002,allowed=guard.allowed)
                if any(h['distance_m']<baseline.get(frozenset([h['a'],h['b']]),0)-.0002 for h in rows):good=False;break
            if good:chosen=target;break
        if chosen is None:raise RuntimeError('当前穿插姿态未找到不加深碰撞的脱离路径，保持原场景')
        station._robot_busy=True;station._robot_cancel=False
        station._status='正在缓慢脱离已有穿插，暂未开启自碰撞';station._refresh_actions()
        try:
            count=max(300,int(1.5*np.max(abs(chosen-q))/.03/executor.world.get_rendering_dt()))
            for i in range(1,count+1):
                if station._robot_cancel:raise InterruptedError('用户停止脱离动作')
                u=i/count;target=q+(chosen-q)*(u*u*(3-2*u))
                executor.send_joint_command(target[guard.arm_indices].tolist(),duration=0,joint_names=[guard.model.names[i] for i in guard.arm_indices])
                executor.step(render=True)
                actual,poses=guard.read();rows=guard.model.query(poses=poses,margin=-.0002,allowed=guard.allowed)
                bad=[h for h in rows if h['distance_m']<baseline.get(frozenset([h['a'],h['b']]),0)-.0005]
                if i%12==0:recovery['samples'].append({'frame':i,'worst_m':min([h['distance_m'] for h in rows]+[0])})
                if bad:raise RuntimeError('脱离过程中出现额外穿插，已停止')
            for _ in range(60):executor.step(render=True)
            if guard.violations(guard.refresh()):raise RuntimeError('脱离后净空不足，暂不启用自碰撞')
        except BaseException:
            station.stop();raise
        finally:
            station._robot_busy=False;station._robot_cancel=False;station._refresh_actions()
            (output/'recovery.json').write_text(json.dumps(recovery,indent=2))
    # 保存所有已注册机构的实测状态和驱动参数，重建同一机器人实例。
    views=[executor.articulation]+list(station.runtime.devices.values())
    if hasattr(executor,'_oxford_devices'):views+=list(executor._oxford_devices.devices.values())
    saved=[]
    for view in views:
        v=view._physics_view
        record={'view':view,'q':np.asarray(view.get_joint_positions()).copy(),
            'targets':np.asarray(v.get_dof_position_targets()).copy(),'gains':[np.asarray(x).copy() for x in view.get_gains()],
            'forces':np.asarray(v.get_dof_max_forces()).copy(),'pose':[np.asarray(x).copy() for x in view.get_world_poses()]}
        saved.append(record)
        view.set_default_state(positions=record['pose'][0],orientations=record['pose'][1])
        view.set_joints_default_state(positions=record['q'],velocities=np.zeros_like(record['q']))
    callbacks=dict(executor.world._physics_functions)
    for name in callbacks:executor.world.remove_physics_callback(name)
    executor.world.stop()
    PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(guard.model.root)).CreateEnabledSelfCollisionsAttr(True)
    for pair in guard.model.excluded:
        a,b=sorted(pair);UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a)).CreateFilteredPairsRel().AddTarget(b)
    executor.world.reset()
    for record in saved:
        view=record['view'];view._on_physics_ready(None)
        view.set_world_poses(positions=record['pose'][0],orientations=record['pose'][1])
        view.set_joint_positions(record['q']);view.set_joint_velocities(np.zeros_like(record['q']))
        view.set_joint_position_targets(record['targets'])
        view.set_gains(kps=record['gains'][0],kds=record['gains'][1])
        view._physics_view.set_dof_max_forces(record['forces'],np.array([0],dtype=np.uint32))
    executor._mimic_targets=None
    for name in station.runtime.devices:
        if hasattr(station.runtime,'_limits_'+name):delattr(station.runtime,'_limits_'+name)
    for name,callback in callbacks.items():executor.world.add_physics_callback(name,callback_fn=callback)
    guard.enabled=True
    executor.world.add_physics_callback('robot_collision_guard',callback_fn=guard.step)
    for _ in range(120):executor.step(render=True)
    real,poses=guard.read();bad=guard.violations(poses=poses,runtime=True)
    result={'dof_count':real.size,'finite':bool(np.isfinite(real).all()),'self_collision':stage.GetPrimAtPath(guard.model.root).GetAttribute('physxArticulation:enabledSelfCollisions').Get(),
        'filtered_body_pairs':len(guard.model.excluded),'current_violations':bad,'recovery_performed':bool(hits),
        'pass':real.size==50 and np.isfinite(real).all() and not bad}
    (output/'activation.json').write_text(json.dumps(result,indent=2))
    if not result['pass']:raise RuntimeError('安装后校验未通过，请检查activation.json')
    station._status='自碰撞与路径保护已启用';station._refresh_actions()
    stage.GetRootLayer().Export(str(output/'scene_protected.usda'))
    return result
