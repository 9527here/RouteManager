"""运行期静态几何缓存 + 每物理步新鲜Tensor反馈；USD改动后拒绝继续复用。

不缓存接触事件、q/v或刚体位置；不创建prim、不修改物理状态。
"""
import copy
import math
import time
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
_REPO = str(Path(__file__).resolve().parents[1])
if _REPO not in sys.path:sys.path.insert(0,_REPO)
import gripper_model
from valve_feedback import _pose, _rotation, _joint_quat, _rigid_pose, _contacts, projected_wheel_width_mm


class SceneGuard:
    POSE_FIELDS = {'xformOp:translate', 'xformOp:orient', 'physics:velocity', 'physics:angularVelocity'}

    def __init__(self, stage):
        from pxr import Tf, Usd, UsdGeom, UsdPhysics
        self.stage = stage
        self.reason = None
        self.rigid_paths = set()
        self.camera_paths = set()
        for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):self.rigid_paths.add(str(p.GetPath()))
            if p.IsA(UsdGeom.Camera) and not any(
                    child.HasAPI(UsdPhysics.CollisionAPI) or child.HasAPI(UsdPhysics.RigidBodyAPI)
                    for child in Usd.PrimRange(p)):
                self.camera_paths.add(str(p.GetPath()))
        self.token = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self.changed, stage)

    def changed(self, notice, sender):
        if self.reason is not None:return
        paths = list(notice.GetResyncedPaths())
        if paths:
            self.reason = '场景拓扑改变: ' + str(paths[0]);return
        safe_paths = getattr(self, '_safe_paths', None)
        if safe_paths is None:self._safe_paths = safe_paths = set()
        for path in notice.GetChangedInfoOnlyPaths():
            if path in safe_paths:continue
            prim, field = str(path.GetPrimPath()), path.name
            if (prim in self.camera_paths or (prim in self.rigid_paths and field in self.POSE_FIELDS)
                    or field.startswith(('state:angular:physics:', 'state:linear:physics:'))):
                safe_paths.add(path);continue
            self.reason = '场景几何/约束属性改变: ' + str(path);return

    def check(self):
        if self.reason is not None:raise RuntimeError(self.reason + '；停止并重新采样')

    def close(self):
        if self.token is not None:self.token.Revoke();self.token = None


class FeedbackReader:
    def __init__(self, executor, valve_root, template):
        import omni.usd
        import omni.physx
        from pxr import UsdPhysics
        from isaacsim.core.simulation_manager import SimulationManager
        self.executor, self.valve_root = executor, valve_root
        self.stage = omni.usd.get_context().get_stage()
        self.guard = SceneGuard(self.stage)
        try:
            self.template = copy.deepcopy(template)
            self.interface = omni.physx.get_physx_interface()
            self.paths = [str(p) for p in executor.articulation._physics_view.link_paths[0]]
            self.index = {p.rsplit('/', 1)[-1]:i for i,p in enumerate(self.paths)}
            self.names = list(executor.articulation.dof_names)
            self.leaders = {s:self.names.index(executor._GRIPPER_LEADER[s]) for s in ('left','right')}
            joint = UsdPhysics.Joint(self.stage.GetPrimAtPath(template['wheel_joint']['path']))
            self.local_R0 = _joint_quat(joint.GetLocalRot0Attr().Get())
            self.local_R1 = _joint_quat(joint.GetLocalRot1Attr().Get())
            self.local_p0 = np.asarray(template['wheel_joint']['local_pos0'])
            self.local_p1 = np.asarray(template['wheel_joint']['local_pos1'])
            self.scale0 = np.asarray(template['wheel_joint']['authored_body0_scale'])
            self.scale1 = np.asarray(template['wheel_joint']['authored_body1_scale'])
            self.axis = np.eye(3)['XYZ'.index(template['wheel_joint']['axis_token'])]
            self.pad_points = {s:[np.asarray(x['support_vertices_local_m']) for x in template['state']['sides'][s]['pad_support_sources']] for s in ('left','right')}
            needed = sorted(self.guard.rigid_paths - set(self.paths))
            self.rigid_view = None
            self.rigid_indices = {}
            if needed:
                self.rigid_view = SimulationManager.get_physics_sim_view().create_rigid_body_view(needed)
                self.rigid_indices = {str(p):i for i,p in enumerate(self.rigid_view.prim_paths)}
            self.robot_transforms = None
            self.rigid_poses = {}
            self.frame_time = None
            self.q = None
        except BaseException:
            self.guard.close();raise

    def close(self):self.guard.close()

    def __call__(self):
        import omni.usd
        e = self.executor
        try:
            if omni.usd.get_context().get_stage() != self.stage:
                raise RuntimeError('场景已切换，停止旧反馈缓存')
            self.guard.check()
        except Exception:
            e.world.pause();raise
        e._require_robot()
        art = e.articulation
        self.q = np.asarray(art.get_joint_positions()).reshape(-1).copy()
        velocity = np.asarray(art.get_joint_velocities()).reshape(-1)
        self.robot_transforms = np.asarray(art._physics_view.get_link_transforms())[0].copy()
        if not all(np.isfinite(v).all() for v in (self.q, velocity, self.robot_transforms)):
            raise RuntimeError('缓存反馈的当前Tensor非有限')
        self.rigid_poses = {}
        if self.rigid_view is not None:
            values = np.asarray(self.rigid_view.get_transforms())
            for p, i in self.rigid_indices.items():
                row = values[i];self.rigid_poses[p] = _pose(row[:3],row[3:][[3,0,1,2]])
        def rigid(path):
            if path is None:return _pose([0,0,0],[1,0,0,0])
            if path not in self.rigid_poses:self.rigid_poses[path] = _rigid_pose(self.interface,path)
            return self.rigid_poses[path]
        def link(name):
            row = self.robot_transforms[self.index[name]]
            return _pose(row[:3], row[3:][[3,0,1,2]])
        meta = self.template
        info = dict(meta['wheel_joint'])
        pose0, pose1 = rigid(info['body0']), rigid(info['body1'])
        R0, R1 = _rotation(pose0), _rotation(pose1)
        A0, A1 = R0 @ self.local_R0, R1 @ self.local_R1
        a0 = R0 @ (self.scale0*(self.local_R0 @ self.axis));a0 /= np.linalg.norm(a0)
        a1 = R1 @ (self.scale1*(self.local_R1 @ self.axis));a1 /= np.linalg.norm(a1)
        p0 = np.asarray(pose0['position']) + R0 @ (self.scale0*self.local_p0)
        p1 = np.asarray(pose1['position']) + R1 @ (self.scale1*self.local_p1)
        relative = A0.T @ A1
        quat = Rotation.from_matrix(relative).as_quat()
        angle = (math.degrees(2*math.atan2(float(quat[:3]@self.axis),float(quat[3])))+180)%360-180
        swing = Rotation.from_matrix(relative @ Rotation.from_rotvec(self.axis*math.radians(angle)).as_matrix().T).magnitude()
        info.update(axis_world=a1.tolist(),axis_point_world=p1.tolist(),anchor0_world=p0.tolist(),anchor1_world=p1.tolist(),
                    anchor_gap_mm=float(np.linalg.norm(p1-p0)*1000),axis_alignment_deg=math.degrees(math.acos(float(np.clip(a0@a1,-1,1)))),
                    angle_deg_wrapped=angle,off_axis_swing_deg=math.degrees(swing))
        state = {'sim_time':float(e.world.current_time),'playing':bool(e.world.is_playing()),'ref_link':'',
                 'joint_names':self.names,'positions':self.q.tolist(),'velocities':velocity.tolist(),
                 'cmd_positions':np.asarray(art._physics_view.get_dof_position_targets()).reshape(-1).tolist(),'errors':[],'sides':{}}
        if 'efforts' in meta['state']:
            try:state['efforts'] = np.asarray(art.get_measured_joint_efforts()).reshape(-1).tolist()
            except Exception as error:state['errors'].append('effort: '+str(error))
        for side in ('left','right'):
            flange = link(side+'_gripper_base_link');R = _rotation(flange)
            pads = [link(side+'_gripper_'+s+'_pad_link') for s in ('left','right')]
            points = np.array([v['position'] for v in pads]);tcp = points.mean(axis=0)
            jaw = points[1]-points[0];jaw /= np.linalg.norm(jaw)
            projection = [local@(_rotation(pad).T@jaw)+float(jaw@np.asarray(pad['position'])) for pad,local in zip(pads,self.pad_points[side])]
            faces = [float(projection[0].max()),float(projection[1].min())]
            delta = .5*sum(faces)-float(jaw@tcp)
            theta = float(self.q[self.leaders[side]]);fraction = gripper_model.fraction_for_leader(theta)
            state['sides'][side] = {'flange':flange,'tcp':_pose(tcp,flange['orientation']),
                'tcp_local':(R.T@(tcp-np.asarray(flange['position']))).tolist(),'jaw_axis_world':jaw.tolist(),
                'pad_poses':dict(zip(('left','right'),pads)),
                'inner_face_midpoint_world':(tcp+jaw*delta).tolist(),'inner_face_gap_mm':(faces[1]-faces[0])*1000,
                'inner_face_midpoint_tcp_projection_delta_m':delta,
                'pad_support_sources':meta['state']['sides'][side]['pad_support_sources'],
                'gripper':{'angle_rad':round(theta,5),'opening_mm':round(gripper_model.opening_mm_for_leader(theta),2),
                           'fraction':round(fraction,4),'position':int(round(1+fraction*999))}}
        geometry = dict(meta['geometry'])
        geometry.update(wheel_pose=pose1,link_world=pose1['position'],center=(np.asarray(pose1['position'])+R1@np.asarray(geometry['center_local_m'])).tolist(),
                        yaw_deg=math.degrees(math.atan2(R1[1,0],R1[0,0])),axis_world=a1.tolist(),axis_point_world=p1.tolist(),axis_point=p1.tolist())
        geometry['grasp_support_width_mm'] = {s:projected_wheel_width_mm(geometry,state['sides'][s]['jaw_axis_world']) for s in ('left','right')}
        result = dict(meta)
        for key in ('fixed_anchor','robot_parking_anchor'):
            anchor = dict(meta[key]) if meta[key] else None
            if anchor and anchor['type']=='kinematic_body':anchor['pose'] = rigid(anchor['body'])
            result[key] = anchor
        result.update(capture_wall_time=time.time(),sim_time=state['sim_time'],state=state,arm_base=link('right_arm_base_link'),
                      base_pose=link('base_link'),geometry=geometry,wheel_joint=info,wheel_angle_deg_wrapped=angle,
                      robot_native_fixed_base=bool(art._physics_view.shared_metatype.fixed_base),
                      contacts=_contacts(self.stage,info['body1'],meta['robot_root'],meta['contacts']))
        self.frame_time = state['sim_time']
        return result
