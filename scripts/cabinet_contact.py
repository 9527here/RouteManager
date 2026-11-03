#!/usr/bin/env python3
"""经 RouteManager 主线程验证人形与柜体的真实接触，不下发柜体驱动目标。"""
import argparse
import json
from pathlib import Path
import sys
import urllib.error
import urllib.request


class CabinetContact:
    def __init__(self, executor):
        self.executor = executor
        self.probe = executor._cabinet_probe
        self.output = self.probe.output
        self.side = 'right'
        self.events = []

    def _state(self):
        return self.executor.get_sim_state(ref_link='')

    def _write(self, name, data):
        self.probe._write(name, data)

    def advance(self, distance=0.12):
        import numpy as np
        table = self.executor._link_pose_table()
        start = table['base_link'][0].copy()
        previous = self.output/'base_approach.json'
        if previous.exists():
            saved = json.loads(previous.read_text())
            if np.linalg.norm(start[:2]-np.array(saved['end'])[:2]) < 0.03:
                return
        rows = []
        try:
            for frame in range(600):
                pos = self.executor._link_pose_table()['base_link'][0]
                if pos[0] - start[0] >= distance:
                    break
                if abs(pos[1] - start[1]) > 0.03:
                    raise RuntimeError('底盘横向偏移超过 3 cm，停止接近')
                self.executor.set_base_velocity(vx=0.06, wz=0)
                self.executor.step(render=True)
                if frame % 10 == 0:
                    rows.append(pos.tolist())
            else:
                raise RuntimeError('600 个物理步内未到目标站位')
        finally:
            self.executor.set_base_velocity(vx=0, wz=0)
            for _ in range(120):
                self.executor.step(render=True)
            end = self.executor._link_pose_table()['base_link'][0]
            self._write('base_approach.json', {'start': start.tolist(), 'end': end.tolist(),
                        'delta': (end-start).tolist(), 'samples': rows})
        if not np.isfinite(end).all():
            raise RuntimeError('底盘反馈不正常')

    def lower(self, target=-0.08):
        import numpy as np
        state = self._state()
        index = state['joint_names'].index('lift_joint')
        limits = np.asarray(self.executor.articulation.get_dof_limits()).reshape(-1,2)[index]
        if not limits[0] <= target <= limits[1]:
            raise ValueError('升降目标超出真实关节限位')
        start = state['cmd_positions'][index]
        for frame in range(1,241):
            u=frame/240
            value=start+(target-start)*u*u*(3-2*u)
            self.executor.send_joint_command([value],duration=0,joint_names=['lift_joint'])
            self.executor.step(render=True)
        for _ in range(180):self.executor.step(render=True)
        actual=self._state()['positions'][index]
        self._write('lift_adjustment.json',{'previous_target':start,'target':target,'actual':actual})
        if abs(actual-target)>0.005:raise RuntimeError('机身升降未到位')

    def _kinematics(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        import arm_kin
        state = self._state()
        table = self.executor._link_pose_table()
        urdf = self.executor._config['robot']['presets']['v3_ctag2f120_rm65']['urdf_path']
        chain = arm_kin.load_chain(urdf, 'right_arm_base_link', 'right_gripper_base_link')
        names = arm_kin.joint_names(chain)
        q = np.array([state['positions'][state['joint_names'].index(n)] for n in names])
        base, quat = table['right_arm_base_link']
        A = Rotation.from_quat(quat, scalar_first=True).as_matrix()
        all_names = list(self.executor.articulation.dof_names)
        all_limits = np.asarray(self.executor.articulation.get_dof_limits()).reshape(-1, 2)
        limits = np.array([all_limits[all_names.index(n)] for n in names])
        p, R = arm_kin.ee_world(chain, q, A, base)
        flange = state['sides']['right']['flange']
        error = float(np.linalg.norm(p - flange['position']))
        if error > 0.002:
            raise RuntimeError(f'FK 与真实法兰偏差 {error*1000:.2f} mm，禁止运动')
        return chain, q, A, base, limits, names

    def _tip_point(self):
        import numpy as np
        from pxr import Usd, UsdGeom, UsdPhysics, Gf
        from scipy.spatial.transform import Rotation
        table = self.executor._link_pose_table()
        fp, fq = table['right_gripper_base_link']
        fR = Rotation.from_quat(fq, scalar_first=True).as_matrix()
        cache = UsdGeom.XformCache()
        points = []
        for name in ['right_gripper_right_pad_link', 'right_gripper_left_pad_link']:
            body = next(p for p in self.probe.stage.Traverse() if p.GetName() == name)
            bp, bq = table[name]
            bR = Rotation.from_quat(bq, scalar_first=True).as_matrix()
            inv = cache.GetLocalToWorldTransform(body).GetInverse()
            for mesh in Usd.PrimRange(body, Usd.TraverseInstanceProxies()):
                if not mesh.IsA(UsdGeom.Mesh) or not mesh.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                relative = cache.GetLocalToWorldTransform(mesh) * inv
                local = np.array([relative.Transform(Gf.Vec3d(*v)) for v in UsdGeom.Mesh(mesh).GetPointsAttr().Get()])
                points.extend(((local @ bR.T + bp - fp) @ fR).tolist())
        points = np.array(points)
        tip_z = points[:, 2].max()
        face = points[points[:, 2] > tip_z - 0.0001]
        return np.array([face[:, 0].mean(), face[:, 1].mean(), tip_z])

    def plan_button(self, name='Btn05', approach_gap=.060, approach_yaw_deg=0., candidate_validator=None):
        import numpy as np
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
        from pxr import Usd, UsdGeom, Gf
        import arm_kin
        if name not in ['Btn04', 'Btn05', 'Btn06', 'Btn07']:
            raise ValueError('仅允许四个操作按钮，不能按顶部指示灯')
        chain, q, A, base, limits, names = self._kinematics()
        tip = self._tip_point()
        info = self.probe.controls[name]
        pos, quat = self.probe._pose(info['body1'])
        bR = Rotation.from_quat([quat.GetReal(), *quat.GetImaginary()], scalar_first=True).as_matrix()
        mesh = next(p for p in Usd.PrimRange(self.probe.stage.GetPrimAtPath(info['body1'])) if p.IsA(UsdGeom.Mesh))
        cache = UsdGeom.XformCache()
        rel = cache.GetLocalToWorldTransform(mesh) * cache.GetLocalToWorldTransform(self.probe.stage.GetPrimAtPath(info['body1'])).GetInverse()
        points = np.array([rel.Transform(Gf.Vec3d(*p)) for p in UsdGeom.Mesh(mesh).GetPointsAttr().Get()])
        surface = pos + bR @ np.array([points[:, 0].max(), 0, 0])
        R = np.array([[0,0,1], [1,0,0], [0,1,0]], dtype=float)
        R = Rotation.from_euler('z', approach_yaw_deg, degrees=True).as_matrix() @ R
        rng = np.random.default_rng(42)
        waypoints = []
        gaps = [approach_gap] + [v for v in [0.025, 0.005, 0, -0.002, -0.004, -0.006, -0.008] if v < approach_gap]
        for gap in gaps:
            target = surface - np.array([gap, 0, 0]) - R @ tip
            def error(v):
                p, r = arm_kin.ee_world(chain, v, A, base)
                return np.r_[p - target, 0.25 * Rotation.from_matrix(R @ r.T).as_rotvec()]
            best = None
            for seed_index, seed in enumerate([q] + [rng.uniform(limits[:,0]+0.03, limits[:,1]-0.03) for _ in range(5)]):
                fit = least_squares(error, seed, bounds=(limits[:,0]+0.02, limits[:,1]-0.02), max_nfev=250)
                residual = error(fit.x)
                valid = (np.linalg.norm(residual[:3]) <= .001
                         and np.linalg.norm(residual[3:])/.25 <= np.deg2rad(1.))
                if candidate_validator is not None and not candidate_validator(fit.x):
                    continue
                # 达标邻近解优先，避免为更小的数值残差换到大幅绕行的IK分支。
                score = ((0 if valid else 1), float(np.max(abs(fit.x-q))) if valid else float(np.linalg.norm(residual)))
                if best is None or score < best[0]:
                    best = score, fit.x
                if seed_index == 0 and valid:
                    break
            if best is None:
                raise RuntimeError(f'gap={gap} 未找到无碰撞的IK解')
            q = best[1]
            e = error(q)
            if np.linalg.norm(e[:3]) > 0.001 or np.linalg.norm(e[3:]) / 0.25 > np.deg2rad(1):
                self._write('button_ik_failure.json', {'gap': gap, 'approach_yaw_deg': approach_yaw_deg,
                    'base': base.tolist(), 'target': target.tolist(), 'tip': tip.tolist(),
                    'q': q.tolist(), 'limits': limits.tolist(), 'error': e.tolist()})
                raise RuntimeError(f'gap={gap} IK 误差 {np.linalg.norm(e[:3])*1000:.2f} mm')
            waypoints.append({'gap': gap, 'q': q.tolist(), 'flange': target.tolist(),
                              'position_error_mm': float(np.linalg.norm(e[:3])*1000)})
        plan = {'name': name, 'tip_local': tip.tolist(), 'surface': surface.tolist(),
                'R': R.tolist(), 'waypoints': waypoints, 'joint_names': names,
                'approach_gap_m': approach_gap, 'approach_yaw_deg': approach_yaw_deg}
        self._write('button_plan.json', plan)
        return plan

    def _listen(self, name):
        import omni.physx
        from pxr import PhysxSchema
        self.target_body = self.probe.controls[name]['body1']
        body = self.probe.stage.GetPrimAtPath(self.target_body)
        PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0)
        self.events = []
        self.contact_total = 0
        self.max_contact_impulse = 0.0
        self.subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self._on_contacts)

    def _on_contacts(self, headers, data):
        from pxr import PhysicsSchemaTools
        for header in headers:
            actors = [str(PhysicsSchemaTools.intToSdfPath(header.actor0)),
                      str(PhysicsSchemaTools.intToSdfPath(header.actor1))]
            if self.target_body not in actors:
                continue
            robot = any('/humanoid/' in actor for actor in actors)
            if not robot:
                continue
            self.contact_total += 1
            for index in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                impulse = data[index].impulse
                length = sum(float(v)**2 for v in impulse)**0.5
                self.max_contact_impulse = max(self.max_contact_impulse, length)
            if len(self.events) < 1000:
                event = {'actors': actors, 'contacts': int(header.num_contact_data),
                         'sim_time': float(self.executor.world.current_time)}
                if header.num_contact_data:
                    sample = data[header.contact_data_offset]
                    event['point'] = list(sample.position)
                    event['impulse'] = list(sample.impulse)
                self.events.append(event)

    def _move_arm(self, target, frames=180, settle=120):
        import numpy as np
        state = self._state()
        names = [f'right_arm_{i}_joint' for i in range(1, 7)]
        q0 = np.array([state['positions'][state['joint_names'].index(n)] for n in names])
        target = np.array(target)
        for index in range(1, frames+1):
            u = index/frames
            blend = u*u*(3-2*u)
            q = q0 + blend*(target-q0)
            self.executor.send_joint_command(q.tolist(), duration=0, joint_names=names)
            self.executor.step(render=True)
        for _ in range(settle):
            self.executor.step(render=True)
        state = self._state()
        real = np.array([state['positions'][state['joint_names'].index(n)] for n in names])
        if not np.isfinite(real).all():
            raise RuntimeError('手臂反馈出现 NaN，停止后续动作')
        return float(abs(real-target).max())

    def run_button(self, name='Btn05'):
        import numpy as np
        from scipy.spatial.transform import Rotation
        self.executor.set_gripper_position(side='right', position=7)
        for _ in range(120):
            self.executor.step(render=True)
        plan = self.plan_button(name)
        self._listen(name)
        info = self.probe.controls[name]
        x0, _ = self.probe._relative(info)
        target_attr = self.probe.stage.GetPrimAtPath(info['joint']).GetAttribute('drive:linear:physics:targetPosition')
        if abs(float(target_attr.Get())) > 1e-8:
            raise RuntimeError('按钮驱动目标必须保持在回位值 0，禁止直接驱动按下')
        before = self._state()
        rows = []
        result = {'name': name, 'button_drive_target': 0, 'phases': rows, 'pass': False}
        pre = plan['waypoints'][0]
        try:
            for waypoint in plan['waypoints']:
                error = self._move_arm(waypoint['q'], frames=240 if not rows else 60,
                                       settle=120 if not rows else 30)
                if abs(float(target_attr.Get())) > 1e-8:
                    raise RuntimeError('检测到按钮目标被外部修改')
                table = self.executor._link_pose_table()
                fp, fq = table['right_gripper_base_link']
                tip_world = fp + Rotation.from_quat(fq, scalar_first=True).apply(plan['tip_local'])
                x, _ = self.probe._relative(info)
                row = {'gap': waypoint['gap'], 'arm_error_rad': error,
                       'tip': tip_world.tolist(), 'button_travel_mm': (x0-x)*1000,
                       'contact_events': len(self.events)}
                rows.append(row)
                self._write('button_contact.json', result)
                if waypoint['gap'] == 0.060:
                    self.probe.capture('button_approach.png')
                    expected = np.array(plan['surface']) - [waypoint['gap'],0,0]
                    if np.linalg.norm(tip_world-expected) > 0.005 or error > 0.03:
                        raise RuntimeError('预接近实际末端误差过大，停止按压')
                if row['button_travel_mm'] >= 4.5:
                    break
            self.probe.capture('button_pressed.png')
        finally:
            # 无论触发了哪个验收闸，都先沿预接近位退出，保持按钮目标仍为 0。
            self._move_arm(pre['q'], frames=120, settle=180)
            xf, _ = self.probe._relative(info)
            result['released_travel_mm'] = (x0-xf)*1000
            result['max_travel_mm'] = max([r['button_travel_mm'] for r in rows] or [0])
            result['contact_events'] = self.events
            result['max_contact_impulse_Ns'] = self.max_contact_impulse
            result['pass'] = (result['max_travel_mm'] >= 4.5
                              and abs(result['released_travel_mm']) < 0.5
                              and len(self.events) > 0 and self.max_contact_impulse > 1e-8)
            result['state_before'] = before['sides']['right']
            self._write('button_contact.json', result)
            self.probe.capture('button_released.png')
            self.subscription = None
        return result

    def _solve_pose(self, chain, A, base, limits, target, R, seed):
        import numpy as np
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
        import arm_kin
        def error(q):
            p, r = arm_kin.ee_world(chain, q, A, base)
            return np.r_[p-target, 0.25*Rotation.from_matrix(R@r.T).as_rotvec()]
        fit = least_squares(error, seed, bounds=(limits[:,0]+0.02, limits[:,1]-0.02), max_nfev=250)
        e = error(fit.x)
        if np.linalg.norm(e[:3]) > 0.001 or np.linalg.norm(e[3:]) > 0.25*np.deg2rad(1):
            rng = np.random.default_rng(17)
            for _ in range(6):
                alternative = least_squares(error, rng.uniform(limits[:,0]+0.03,limits[:,1]-0.03),
                    bounds=(limits[:,0]+0.02,limits[:,1]-0.02), max_nfev=250)
                if np.linalg.norm(error(alternative.x)) < np.linalg.norm(e):
                    fit, e = alternative, error(alternative.x)
                if np.linalg.norm(e) < 0.00001: break
        if np.linalg.norm(e[:3]) > 0.001 or np.linalg.norm(e[3:]) > 0.25*np.deg2rad(1):
            self._write('knob_ik_failure.json',{'target':np.asarray(target).tolist(),'R':np.asarray(R).tolist(),
                        'position_error_mm':float(np.linalg.norm(e[:3])*1000),'q':fit.x.tolist(),
                        'A':A.tolist(),'base':base.tolist()})
            raise RuntimeError(f'旋钮 IK 不可达，位置误差 {np.linalg.norm(e[:3])*1000:.2f} mm')
        return fit.x

    def run_knob(self, name='Knob01'):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from pxr import Usd, UsdGeom, Gf
        if name not in ['Knob01', 'Knob02']:
            raise ValueError('仅支持两个旋钮')
        chain, seed, A, base, limits, _ = self._kinematics()
        info = self.probe.controls[name]
        body = self.probe.stage.GetPrimAtPath(info['body1'])
        pos, quat = self.probe._pose(info['body1'])
        body_R = Rotation.from_quat([quat.GetReal(), *quat.GetImaginary()], scalar_first=True)
        mesh = next(p for p in Usd.PrimRange(body) if p.IsA(UsdGeom.Mesh))
        cache = UsdGeom.XformCache()
        rel = cache.GetLocalToWorldTransform(mesh)*cache.GetLocalToWorldTransform(body).GetInverse()
        pts = np.array([rel.Transform(Gf.Vec3d(*v)) for v in UsdGeom.Mesh(mesh).GetPointsAttr().Get()])
        lo, hi = pts.min(0), pts.max(0)
        # 选取突出部前半段，避开旋钮底座；宽度从碰撞 mesh 量出。
        local_center = (lo+hi)/2
        local_center[0] = hi[0] - 0.006
        center = pos + body_R.apply(local_center)
        width_mm = (hi[1]-lo[1])*1000
        stroke = self.executor._config['gripper']['stroke_mm']
        grip_fraction = max(0.0, (width_mm-2.0)/stroke)
        # 在空中先量目标闭合角对应的 TCP，再张开进入，补偿四连杆沿工具轴的漂移。
        self.executor.set_gripper_position(side='right', fraction=grip_fraction)
        for _ in range(120): self.executor.step(render=True)
        state = self._state()['sides']['right']
        fR = Rotation.from_quat(state['flange']['orientation'], scalar_first=True)
        tcp_local = fR.inv().apply(np.array(state['tcp']['position'])-state['flange']['position'])
        self.executor.set_gripper_position(side='right', fraction=min(1.0, (width_mm+20)/stroke))
        for _ in range(120): self.executor.step(render=True)
        R0 = np.array([[0,0,1],[1,0,0],[0,1,0]],float)
        approach = center - [0.060,0,0]
        pre = self._solve_pose(chain,A,base,limits,approach-R0@tcp_local,R0,seed)
        grasp = self._solve_pose(chain,A,base,limits,center-R0@tcp_local,R0,pre)
        plan = []
        q = grasp
        # 仅预解算，全部角度可达后才进入接触阶段。
        for angle in list(range(3,31,3)) + list(range(27,-31,-3)) + list(range(-27,1,3)):
            R = Rotation.from_rotvec([np.deg2rad(angle),0,0]).as_matrix()@R0
            q = self._solve_pose(chain,A,base,limits,center-R@tcp_local,R,q)
            plan.append({'angle':angle,'q':q.tolist()})
        joint = self.probe.stage.GetPrimAtPath(info['joint'])
        attrs = {key:joint.GetAttribute('drive:angular:physics:'+key)
                 for key in ['stiffness','damping','maxForce','targetPosition']}
        original = {key:attr.Get() for key,attr in attrs.items()}
        result = {'name':name,'width_mm':float(width_mm),'grip_fraction':grip_fraction,
                  'mode':'free_joint_contact_test_not_real_switch_torque',
                  'original_drive':original,'center':center.tolist(),'samples':[],'pass':False}
        self._move_arm(pre,frames=180,settle=120)
        self._move_arm(grasp,frames=120,settle=120)
        self._listen(name)
        self.executor.set_gripper_position(side='right',fraction=grip_fraction)
        for _ in range(180):self.executor.step(render=True)
        self.probe.capture('knob_grasp.png')
        _, qinitial = self.probe._pose(info['body1'])
        initial_R = Rotation.from_quat([qinitial.GetReal(),*qinitial.GetImaginary()],scalar_first=True)
        try:
            # 完全停用主动驱动，仅验证几何、夹持和被动关节，不臆造真实档位阻力。
            for key in ['stiffness','damping','maxForce']:attrs[key].Set(0)
            for waypoint in plan:
                error = self._move_arm(waypoint['q'],frames=12,settle=0)
                p, quat = self.probe._pose(info['body1'])
                R = Rotation.from_quat([quat.GetReal(),*quat.GetImaginary()],scalar_first=True)
                actual = float((R*initial_R.inv()).as_rotvec()[0]*180/np.pi)
                result['samples'].append({'target_world_angle':waypoint['angle'],
                       'actual_world_angle':actual,'arm_error_rad':error,'contacts':self.contact_total})
                self._write('knob_contact.json',result)
                if waypoint['angle'] in [30,-30,0]:
                    # 夹持存在滑移，用旋钮实测角补偿工具滚转；不写旋钮驱动目标。
                    tool_angle=float(waypoint['angle'])
                    q_feedback=np.array(waypoint['q'])
                    for correction in range(5):
                        remaining=waypoint['angle']-actual
                        if abs(remaining)<1.5:break
                        tool_angle=float(np.clip(tool_angle+remaining,-50,50))
                        Rgoal=Rotation.from_rotvec([np.deg2rad(tool_angle),0,0]).as_matrix()@R0
                        q_feedback=self._solve_pose(chain,A,base,limits,center-Rgoal@tcp_local,Rgoal,q_feedback)
                        error=self._move_arm(q_feedback,frames=40,settle=30)
                        _, qq=self.probe._pose(info['body1'])
                        rr=Rotation.from_quat([qq.GetReal(),*qq.GetImaginary()],scalar_first=True)
                        actual=float((rr*initial_R.inv()).as_rotvec()[0]*180/np.pi)
                        result['samples'].append({'target_world_angle':waypoint['angle'],
                            'actual_world_angle':actual,'tool_angle':tool_angle,'correction':correction+1,
                            'arm_error_rad':error,'contacts':self.contact_total})
                    result.setdefault('endpoints',[]).append({'target':waypoint['angle'],'actual':actual})
                if waypoint['angle'] in [30,-30]:
                    self.probe.capture('knob_positive.png' if waypoint['angle']>0 else 'knob_negative.png')
            result['max_actual_deg'] = max(s['actual_world_angle'] for s in result['samples'])
            result['min_actual_deg'] = min(s['actual_world_angle'] for s in result['samples'])
            result['final_actual_deg'] = result['samples'][-1]['actual_world_angle']
            result['geometry_pass'] = (all(abs(p['actual']-p['target'])<2 for p in result['endpoints'])
                                       and self.contact_total>0)
            # 位置/接触几何和力学验收分开：零冲量反馈不能当作真实力或扭矩数据。
            result['force_measurement_available'] = self.max_contact_impulse>1e-8
            result['pass'] = result['geometry_pass'] and result['force_measurement_available']
        finally:
            # 旋钮最终回到初始附近后恢复原驱动，再松爪退开。
            for key, value in original.items():attrs[key].Set(value)
            self.executor.set_gripper_position(side='right',fraction=min(1.0,(width_mm+20)/stroke))
            for _ in range(120):self.executor.step(render=True)
            self._move_arm(pre,frames=120,settle=120)
            result['contact_events'] = self.events
            result['max_contact_impulse_Ns'] = self.max_contact_impulse
            result['drive_restored'] = {key:attr.Get() for key,attr in attrs.items()}
            self._write('knob_contact.json',result)
            self.probe.capture('knob_released.png')
            self.subscription = None
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['advance', 'lower', 'plan', 'button', 'knob'])
    parser.add_argument('--url', default='http://127.0.0.1:18765')
    parser.add_argument('--name')
    args = parser.parse_args()
    if args.name is None:
        args.name = 'Knob01' if args.action == 'knob' else 'Btn05'
    path = str(Path(__file__).resolve())
    code = f'''import types
mod = types.ModuleType('cabinet_contact_runtime')
exec(compile(open({path!r}, encoding='utf-8').read(), {path!r}, 'exec'), mod.__dict__)
if not hasattr(executor, '_cabinet_contact'):
    executor._cabinet_contact = mod.CabinetContact(executor)
else:
    executor._cabinet_contact.__class__ = mod.CabinetContact
'''
    if args.action == 'advance':
        code += 'executor._cabinet_contact.advance()'
    elif args.action == 'lower':
        code += 'executor._cabinet_contact.lower()'
    elif args.action == 'plan':
        code += f'executor._cabinet_contact.plan_button({args.name!r})'
    else:
        code += f'executor._cabinet_contact.run_{args.action}({args.name!r})'
    request = urllib.request.Request(args.url+'/python', data=json.dumps({'code':code}).encode(),
                                     headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=600) as response:
            result = json.load(response)
        print(json.dumps(result, ensure_ascii=False))
    except urllib.error.HTTPError as exc:
        print(exc.read().decode(), file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
