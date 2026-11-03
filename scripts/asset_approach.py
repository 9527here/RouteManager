"""按资产世界位置规划前方操作位；只用舵轮驱动走位，到位停稳后才放行动作。"""
import heapq
import json
import math
import time
from pathlib import Path

import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def rotation(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])


class NavigationBlocked(RuntimeError):
    """仅可预见的转舵/通道受阻能尝试退离；反馈与真实净空异常不能重试。"""


class NavigationGeometry:
    """由实际碰撞几何生成平面包络，保留各形状高度，避免将桌板当作地面墙。"""
    def __init__(self, executor, stage, pose):
        from pxr import Usd, UsdGeom, UsdPhysics, Gf
        from scipy.spatial.transform import Rotation
        table = executor._link_pose_table()
        cache = UsdGeom.XformCache()
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy', 'guide'], False, True)
        robot_boxes, obstacles, module_geometry = [], [], {}
        expected_robot_shapes = 0
        shape_types = {}
        for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or prim.GetAttribute('physics:collisionEnabled').Get() is False:
                continue
            path = str(prim.GetPath())
            if path.startswith('/humanoid/'):
                expected_robot_shapes += 1
                shape_types[prim.GetTypeName()] = shape_types.get(prim.GetTypeName(), 0)+1
                body = prim
                while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                    body = body.GetParent()
                if not body or body.GetName() not in table:
                    raise RuntimeError('碰撞体缺少物理刚体反馈：'+path)
                p, quat = table[body.GetName()]
                local = cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(body).GetInverse()
                if prim.IsA(UsdGeom.Mesh):
                    points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
                    if not points:
                        raise RuntimeError('机器人碰撞网格为空：'+path)
                    points = np.array([local.Transform(Gf.Vec3d(*v)) for v in points])
                elif prim.IsA(UsdGeom.Cube):
                    half = float(UsdGeom.Cube(prim).GetSizeAttr().Get())/2
                    points = np.array([local.Transform(Gf.Vec3d(x,y,z)) for x in (-half,half)
                                       for y in (-half,half) for z in (-half,half)])
                elif prim.IsA(UsdGeom.Cylinder):
                    cylinder=UsdGeom.Cylinder(prim)
                    radius=float(cylinder.GetRadiusAttr().Get());half=float(cylinder.GetHeightAttr().Get())/2
                    extent=np.full(3,radius);extent['XYZ'.index(cylinder.GetAxisAttr().Get())]=half
                    points=np.array([local.Transform(Gf.Vec3d(x,y,z)) for x in (-extent[0],extent[0])
                                     for y in (-extent[1],extent[1]) for z in (-extent[2],extent[2])])
                else:
                    bb = bounds.ComputeUntransformedBound(prim).ComputeAlignedRange()
                    if bb.IsEmpty():
                        raise RuntimeError('碰撞形状包络为空，不能静默略过：'+path)
                    lo, hi = np.array(bb.GetMin()), np.array(bb.GetMax())
                    points = np.array([local.Transform(Gf.Vec3d(x, y, z))
                        for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
                world = points @ Rotation.from_quat(quat, scalar_first=True).as_matrix().T + p
                module_geometry.setdefault(body.GetName(), []).append(world)
                planar = (world[:, :2]-pose[:2]) @ rotation(pose[2])
                robot_boxes.append((planar.min(axis=0), planar.max(axis=0), world[:, 2].min(), world[:, 2].max()))
            else:
                if any(v in path.lower() for v in ('ground_plane', 'groundplane')):
                    continue
                bb = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
                if bb.IsEmpty():
                    continue
                lo, hi = np.array(bb.GetMin()), np.array(bb.GetMax())
                if hi[2] < .025 or lo[2] > 2.3:
                    continue
                obstacles.append((lo, hi, path))
        if not robot_boxes:
            raise RuntimeError('无法读取机器人碰撞几何，禁止走位')
        shell=stage.GetPrimAtPath('/World/Cabinet/CabinetBody/CabinetBody')
        if shell and not shell.HasAPI(UsdPhysics.CollisionAPI):
            bb=bounds.ComputeWorldBound(shell).ComputeAlignedRange()
            if bb.IsEmpty():raise RuntimeError('柜壳可视包络为空，禁止走位')
            obstacles.append((np.array(bb.GetMin()),np.array(bb.GetMax()),'visual_envelope:'+str(shell.GetPath())))
        if len(robot_boxes) != expected_robot_shapes:
            raise RuntimeError('机器人碰撞形状覆盖数量不一致，禁止走位')
        self.coverage = {'expected':expected_robot_shapes,'included':len(robot_boxes),'types':shape_types}
        self.boxes, self.obstacles = robot_boxes, obstacles
        self.circles = []
        model = executor._get_swerve_model()
        if model is not None:
            import xml.etree.ElementTree as ET
            joints = {j.get('name'):j for j in ET.parse(model.source_urdf).getroot().findall('joint')}
            for module in model.modules:
                fork = joints[module.steering_joint].find('child').get('link')
                wheel = joints[module.wheel_joint].find('child').get('link')
                arrays = module_geometry.get(fork, [])+module_geometry.get(wheel, [])
                if not arrays or fork not in table:
                    raise RuntimeError('无法获得完整舵轮扫掠包络：'+module.name)
                vertices = np.concatenate(arrays)
                pivot = np.asarray(table[fork][0])
                radius = float(np.linalg.norm(vertices[:, :2]-pivot[:2], axis=1).max())
                center = (pivot[:2]-pose[:2])@rotation(pose[2])
                self.circles.append((center, radius, vertices[:,2].min(), vertices[:,2].max(), module.name))
        self.margin = .012

    def _coarse_collision(self, pose, all_hits=False):
        # 向量化全部形状对，避免每个物理控制帧做数千次Python/NumPy小调用。
        key = (id(self.boxes), id(self.obstacles), self.margin)
        if getattr(self, '_array_key', None) != key:
            self._array_key = key
            self._low = np.array([x[0] for x in self.boxes])
            self._high = np.array([x[1] for x in self.boxes])
            self._half = (self._high-self._low)/2+self.margin
            self._local_centers = (self._high+self._low)/2
            self._obs_low = np.array([x[0] for x in self.obstacles]).reshape(-1, 3)
            self._obs_high = np.array([x[1] for x in self.obstacles]).reshape(-1, 3)
            self._obs_centers = (self._obs_low[:, :2]+self._obs_high[:, :2])/2
            self._obs_half = (self._obs_high[:, :2]-self._obs_low[:, :2])/2
            zmin = np.array([x[2] for x in self.boxes])[:, None]
            zmax = np.array([x[3] for x in self.boxes])[:, None]
            self._height_overlap = ((zmax >= .025) & (self._obs_high[None, :, 2] > zmin-.004)
                                    & (self._obs_low[None, :, 2] < zmax+.004))
        if not self.obstacles:
            return {} if all_hits else None
        R = rotation(pose[2])
        centers = self._local_centers@R.T+pose[:2]
        broad = self._half@abs(R.T)
        delta = self._obs_centers[None, :, :]-centers[:, None, :]
        global_depth = broad[:, None, :]+self._obs_half[None, :, :]-abs(delta)
        local_depth = self._half[:, None, :]+(self._obs_half@abs(R))[None, :, :]-abs(delta@R)
        mask = self._height_overlap & np.all(global_depth > 0., axis=2) & np.all(local_depth > 0., axis=2)
        pairs = np.argwhere(mask)
        if len(pairs) and not all_hits:
            return self.obstacles[pairs[0, 1]][2]
        depth = np.minimum(np.min(global_depth, axis=2), np.min(local_depth, axis=2))
        hits = {str(i)+':'+self.obstacles[j][2]: float(depth[i, j]) for i, j in pairs}
        for center, radius, zlo, zhi, name in getattr(self, 'circles', []):
            center = pose[:2]+R@center
            closest = np.clip(center, self._obs_low[:,:2], self._obs_high[:,:2])
            penetration = radius+self.margin-np.linalg.norm(closest-center, axis=1)
            intersect = (penetration>0) & (self._obs_high[:,2]>zlo-.004) & (self._obs_low[:,2]<zhi+.004)
            for j in np.flatnonzero(intersect):
                if not all_hits:
                    return self.obstacles[j][2]
                hits['sweep:'+name+':'+self.obstacles[j][2]] = float(penetration[j])
        return hits if all_hits else None

    def collision(self,pose,all_hits=False,steering=None):
        coarse=self._coarse_collision(pose,all_hits)
        if not coarse:return {} if all_hits else None
        exact=getattr(self,'narrow',None)
        if exact is None:return coarse
        result=exact.check(pose,margin=self.margin,steering=steering)
        self.last_refinement={'coarse_candidates':coarse,'exact':result}
        if result['safe']:return {} if all_hits else None
        if all_hits:
            return {r['shape_a']+' ↔ '+r['shape_b']:self.margin-r['distance_m'] for r in result['violations']}
        hit=result['violations'][0]
        return hit['shape_a']+' ↔ '+hit['shape_b']+'，净空'+f"{hit['distance_m']*1000:.1f}毫米"

    def recovery_edge(self, start, goal, baseline, count=81):
        # 已有过近姿态只能沿短退离段减小重叠，不允许新增或加深包络冲突。
        for u in np.linspace(0., 1., count):
            pose = start+(goal-start)*u
            hits = self.collision(pose, all_hits=True)
            if any(key not in baseline or depth > baseline[key]+.002 for key, depth in hits.items()):
                return False
        return True

    def edge(self, start, goal, steering=None):
        length = np.linalg.norm(goal[:2]-start[:2])
        count = max(2, math.ceil(length/.010), math.ceil(abs(wrap(goal[2]-start[2]))/math.radians(2)))
        for u in np.linspace(0., 1., count+1):
            pose = start+(goal-start)*u
            pose[2] = start[2]+u*wrap(goal[2]-start[2])
            if self.collision(pose,steering=steering):
                return False
        return True

    def path(self, start, goal):
        if self.edge(start, goal):
            return [goal]
        # 资产前方工作区的A*；实际包络检查每个节点及相邻边，保留最终连续边核验。
        spacing = .09
        origin = np.minimum(start[:2], goal[:2])-1.2
        upper = np.maximum(start[:2], goal[:2])+1.2
        shape = np.ceil((upper-origin)/spacing).astype(int)+1
        grid = lambda xy: tuple(np.rint((xy-origin)/spacing).astype(int))
        world = lambda key: np.r_[origin+np.array(key)*spacing, goal[2]]
        first, last = grid(start[:2]), grid(goal[:2])
        cost, parent, queue, validity = {first: 0.}, {}, [(0., first)], {}
        while queue and len(cost) < 6000:
            _, key = heapq.heappop(queue)
            if key == last:
                chain = [goal]
                while key != first:
                    chain.append(world(key))
                    key = parent[key]
                chain.reverse()
                chain = [start]+chain
                shortened = [start]
                index = 0
                while index < len(chain)-1:
                    candidate = len(chain)-1
                    while candidate > index+1 and not self.edge(chain[index], chain[candidate]):
                        candidate -= 1
                    if not self.edge(chain[index], chain[candidate]):
                        raise RuntimeError('导航路径连续包络检查未通过')
                    shortened.append(chain[candidate])
                    index = candidate
                return shortened[1:]
            here = world(key)
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = key[0]+dx, key[1]+dy
                if not (0 <= nxt[0] < shape[0] and 0 <= nxt[1] < shape[1]):
                    continue
                if nxt not in validity:
                    validity[nxt] = not self.collision(world(nxt))
                if not validity[nxt] or not self.edge(here, world(nxt)):
                    continue
                value = cost[key]+math.hypot(dx, dy)*spacing
                if value < cost.get(nxt, float('inf')):
                    cost[nxt], parent[nxt] = value, key
                    heapq.heappush(queue, (value+np.linalg.norm(world(nxt)[:2]-goal[:2]), nxt))
        raise RuntimeError('目标前方无可行底盘路径，请先移开障碍物或调整工位')


class AssetApproach:
    def __init__(self, station):
        self.station, self.executor = station, station.executor
        self.profile = json.loads((Path(__file__).resolve().parents[1]/
            'assets/l3_operable/mobile_operation_profile.json').read_text())
        self.state = {'phase': 'idle', 'message': '请先选择资产，操作前将自动走位'}
        self.goal = None
        self.operation_yaw = None
        self.exit_before_alignment = False
        self.waypoints = []
        self.samples = []
        self.settle_window = []
        self.elapsed = self.stable = self.command_time = self.report_time = 0.
        self.output = Path(__file__).resolve().parents[1]/'outputs/mobile_operations'
        self.output.mkdir(parents=True, exist_ok=True)

    @property
    def busy(self):
        return self.state['phase'] in ('preparing', 'moving', 'aligning', 'settling', 'recovering_stop')

    def pose(self):
        robot = self.executor.articulation
        names = list(robot.body_names)
        index = names.index('base_link')
        transforms = np.asarray(robot._physics_view.get_link_transforms()).reshape(-1, 7)
        raw = transforms[index]
        if 'torso_pitch_link' in names:
            cx,cy,cz,cw=transforms[names.index('torso_pitch_link'),3:]
            self.chest_yaw=math.atan2(2*(cw*cz+cx*cy),1-2*(cy*cy+cz*cz))
        p, q = raw[:3], np.asarray(raw[3:], dtype=float)
        q /= np.linalg.norm(q)
        x, y, z, w = q
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(float(np.clip(2*(w*y-z*x), -1., 1.)))
        return np.array([p[0], p[1], yaw]), np.array([p[2], roll, pitch])

    def target(self, asset_id, source='l3'):
        from pxr import UsdGeom
        from scipy.spatial.transform import Rotation
        stage = self.station.runtime.stage
        if source == 'l3':
            path = self.station.runtime.prefix+'/'+asset_id
            matrix = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
            position = np.array(matrix.ExtractTranslation())
            frame = Rotation.from_quat([matrix.ExtractRotationQuat().GetReal(),
                *matrix.ExtractRotationQuat().GetImaginary()], scalar_first=True).as_matrix()
        elif source == 'cabinet':
            probe = self.executor._cabinet_probe
            p, _ = probe._pose(probe.controls[asset_id]['body0'])
            position = np.array(p)
            matrix = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Cabinet'))
            quat = matrix.ExtractRotationQuat()
            frame = Rotation.from_quat([quat.GetReal(), *quat.GetImaginary()], scalar_first=True).as_matrix()
        elif source == 'oxford':
            view = self.executor._oxford_devices.devices[asset_id]
            position = np.asarray(view.get_world_poses()[0]).reshape(-1, 3)[0]
            # 当前迁移面板的可操作面朝-X；只对该面板已确认安装朝向使用。
            frame = np.diag([-1., -1., 1.])
        else:
            raise ValueError('未知资产来源')
        reference = json.loads((Path(__file__).resolve().parents[1]/'assets/l3_operable/humanoid_demo_pose.json').read_text())
        base = np.array(reference['base_link_matrix'])
        ref_asset = np.array([.78, -.4, 1.035])
        # 此偏置来自旧RM65绿色按钮接触验收，属于演示操作位，不是新URDF的通用标定。
        offset_local = np.diag([-1., -1., 1.])@(base[:3,3]-ref_asset)
        offset = frame@offset_local
        # 用户要求胸部正对操作面；旧右臂示教yaw偏置不能当作面板法线。
        forward=-frame[:2,0]
        if np.linalg.norm(forward)<1e-6:raise RuntimeError('操作面朝向不是竖直面，当前走位不支持')
        facing=math.atan2(forward[1],forward[0])
        return np.r_[position[:2]+offset[:2], wrap(facing)], position

    def at_target(self, asset_id, source='l3'):
        if self.state['phase'] != 'arrived' or self.state.get('asset_id') != asset_id or self.state.get('source') != source:
            return False
        pose, _ = self.pose()
        goal, _ = self.target(asset_id, source)
        return bool(np.linalg.norm(pose[:2]-goal[:2]) < .025
                    and abs(wrap(getattr(self,'chest_yaw',pose[2])-goal[2])) < math.radians(self.profile['navigation_yaw_tolerance_deg']))

    def _axis_targets(self,axis):
        model=self.executor._get_swerve_model()
        actual=self.narrow.q[self.narrow.steer_indices]
        return model.solve_cardinal(.05 if axis==0 else 0.,.05 if axis==1 else 0.,actual)['steering']

    def _verify_steering(self,pose,axis,from_steering=None):
        targets=self._axis_targets(axis)
        initial=self.narrow.q[self.narrow.steer_indices] if from_steering is None else np.array(from_steering)
        # 扫掠圆足够远时已覆盖所有舵角；近障碍时必须以真实3D形状扫舵复核。
        if not self.geometry._coarse_collision(pose):
            return {'safe':True,'targets':targets.tolist(),'method':'all_angle_conservative_sweep_clear'}
        report=self.narrow.sweep(pose,initial,targets,self.geometry.margin)
        # 运行指令显式锁定本组舵角，不再让近零噪声选另一组±90后重复扫三次。
        return {**report,'targets':targets.tolist(),'method':'true_3d_steering_sweep'}

    def transit_route(self,start,goal):
        R=rotation(start[2]);delta=R.T@(goal[:2]-start[:2]);options=[]
        initial=self.narrow.q[self.narrow.steer_indices]
        steering_checks={}
        for first_axis in (0,1):
            mid=start.copy();vector=np.zeros(2);vector[first_axis]=delta[first_axis]
            mid[:2]+=R@vector
            chain=[start,mid,goal];axes=[first_axis,1-first_axis]
            filtered=[(point,axis,previous) for point,axis,previous in zip([mid,goal],axes,chain)
                      if np.linalg.norm(point[:2]-previous[:2])>.008]
            previous=start;steering=initial;valid=True;points=[];chosen_axes=[];proofs=[];turn_time=0.
            for point,axis,_ in filtered:
                target=self._axis_targets(axis)
                if not self.geometry.edge(previous,point,steering=target):valid=False;break
                key=(previous.tobytes(),axis,np.asarray(steering).tobytes())
                if key not in steering_checks:
                    steering_checks[key]=self._verify_steering(previous,axis,steering)
                proof=steering_checks[key]
                if not proof['safe']:valid=False;break
                rate=min(self.profile['steering_trajectory_speed_rad_s'],
                         min(m.steering_max_velocity_rad_s for m in self.executor._get_swerve_model().modules))
                turn_time+=1.5*float(np.max(abs(target-steering)))/rate
                points.append(point);chosen_axes.append(axis);proofs.append(proof)
                previous=point;steering=target
            if valid:
                if not points:points,chosen_axes=[goal],[0]
                points[-1]=goal.copy()
                length=sum(np.linalg.norm(b[:2]-a[:2]) for a,b in zip([start]+points,points))
                options.append((length/self.profile['max_linear_speed_m_s']+turn_time,points,chosen_axes,proofs))
        if not options:raise NavigationBlocked('当前轮位不能安全转舵或固定朝向通道受阻')
        _,points,axes,proofs=min(options,key=lambda x:x[0])
        self.state['planned_steering_checks']=proofs
        return points,axes

    def _plan_safe_exit(self,start,goal):
        current=self.narrow.q[self.narrow.steer_indices]
        headings=np.array([m.heading_zero+m.steering_sign*q for m,q in zip(self.executor._get_swerve_model().modules,current)])
        axis=0 if np.max(abs(np.sin(headings)))<math.sin(math.radians(3.)) else 1 if np.max(abs(np.cos(headings)))<math.sin(math.radians(3.)) else None
        if axis is None:raise RuntimeError('轮子当前不在同一前后/左右滚动平面，不能自动退离')
        target=self._axis_targets(axis)
        if not self.narrow.sweep(start,current,target,.012)['safe']:
            raise RuntimeError('当前轮向微调也会接近脚座，保持停止')
        # 先找能解锁后续路线的最短退出，避免不必要的长距离低速爬行积累偏航。
        for distance in (.10,.15,.20,.25):
            for direction in (-1.,1.):
                exit_pose=start.copy();local=np.zeros(2);local[axis]=direction*distance
                exit_pose[:2]+=rotation(start[2])@local
                proof=self.narrow.edge(start,exit_pose,.012,steering=target)
                if not proof['safe']:continue
                # 退出时也保留手/臂5cm规则，不能只用底盘12mm阈值证明整条路安全。
                hand_minimum=float('inf');hands_safe=True
                for u in np.linspace(0.,1.,max(2,math.ceil(distance/.002))+1):
                    hands=self.safety.check(min_clearance_m=self.profile['hand_clearance_m'],refresh=False,
                        poses=self.narrow.predict(start+(exit_pose-start)*u,target))
                    if not hands['safe']:hands_safe=False;break
                    hand_minimum=min(hand_minimum,hands['minimum_distance_m'])
                if not hands_safe:continue
                if goal is None:
                    points,axes=[],[]
                    turn_after_exit=None
                else:
                    turn_after_exit=abs(wrap(goal[2]-exit_pose[2]))>math.radians(self.profile.get('alignment_stop_tolerance_deg',.75))
                    try:
                        if turn_after_exit:
                            saved=self.narrow.q[self.narrow.steer_indices].copy()
                            try:
                                self.narrow.q[self.narrow.steer_indices]=target
                                rotation_proof=self._rotation_plan(exit_pose,float(goal[2]),goal)
                            finally:self.narrow.q[self.narrow.steer_indices]=saved
                            points,axes=[],[]
                        else:points,axes=self.transit_route(exit_pose,goal)
                    except NavigationBlocked:continue
                self.state['safe_exit']={'axis':'x' if axis==0 else 'y','signed_distance_m':direction*distance,
                    'proof':proof,'hand_minimum_distance_m':hand_minimum,
                    'reason':'走位或转舵受阻；先保持现有轮向退出'}
                self.exit_before_alignment=bool(turn_after_exit)
                if turn_after_exit:self.state['safe_exit']['post_exit_rotation_check']=rotation_proof
                self.recovery_indices={0}
                return [exit_pose]+points,[axis]+axes
        raise RuntimeError('已复核真实几何，但没有不碰脚座的保轮向退离路径')

    def _reset_route(self,start):
        self.index=0;self.verified_leg=-1
        self.current_leg_steering=None
        self.position_reached=False;self.heading_reached=True
        self.segment_start=start.copy();self.last_pose=start.copy()
        self.progress_anchor=start.copy();self.progress_time=self.elapsed
        self.last_world_speed=np.zeros(2);self.last_turn_speed=0.;self.command_time=1.
        self.settle_window=[];self.corrections=0
        self.state.update(phase='moving',waypoints=[x.tolist() for x in self.waypoints],
                          waypoint_axes=self.waypoint_axes,no_chassis_rotation=self.state['source']=='manual',
                          translation_no_chassis_rotation=True)

    def _install_safe_exit(self,start,reason):
        if self.recovery_attempts>=self.profile.get('automatic_recovery_max_attempts',1):
            raise RuntimeError('本次自动回退已尝试一次，保持停止，避免反复进退')
        self.recovery_attempts+=1
        self.state.update(automatic_recovery_attempts=self.recovery_attempts,
                          recovery_reason=str(reason),recovery_stage='planning')
        # 手动“左/右移”不加前后绕行补偿：退出后结束，不能误报横移完成。
        self.recovery_manual_only=self.state['source']=='manual'
        self.waypoints,self.waypoint_axes=self._plan_safe_exit(start,None if self.recovery_manual_only else self.goal)
        self._reset_route(start)
        self.state.update(recovery_stage='retreating',message='走位受阻，正在保持轮向自动退离；退离路径与双手净空已通过')

    def _queue_recovery(self,reason):
        if (self.state['phase'] not in ('moving','aligning','settling')
                or self.recovery_attempts>=self.profile.get('automatic_recovery_max_attempts',1)):
            return False
        self.executor.set_base_velocity(0.,0.,vy=0.)
        self.recovery_stop_started=self.elapsed
        self.state['recovery_origin_phase']=self.state['phase']
        self.state.update(phase='recovering_stop',message='走位受阻，先停轮，再检查自动回退路径',
                          recovery_reason=str(reason),recovery_stage='stopping')
        return True

    def _step_recovery_stop(self):
        self.executor.set_base_velocity(0.,0.,vy=0.)
        if self.elapsed-self.recovery_stop_started>8.:
            raise RuntimeError('自动回退前未能确认停稳，保持停止')
        if self.executor._base_cmd is not None:return
        start,_=self.pose()
        # 停稳后重新读取舵角/环境位置，不能沿受阻前冻结的几何回退。
        self.narrow.refresh()
        self.geometry=NavigationGeometry(self.executor,self.station.runtime.stage,start)
        self.geometry.narrow=self.narrow
        body=self.narrow.check(start,.012,nearest=True)
        self.state['whole_body_clearance']=body
        if not body['safe']:raise RuntimeError('停稳后真实净空不足，不能自动回退：'+str(body['nearest']))
        if self.state.get('recovery_origin_phase')=='aligning':
            self.yaw_hold=float(start[2]);self.lock_rotation=rotation(self.yaw_hold)
        self._install_safe_exit(start,self.state['recovery_reason'])

    def _rotation_plan(self,start,desired_yaw,goal=None):
        actual=self.narrow.q[self.narrow.steer_indices].copy()
        error=wrap(desired_yaw-start[2])
        targets=self.executor._get_swerve_model().solve(0.,0.,math.copysign(.1,error),actual)['steering']
        sweep=self.narrow.sweep(start,actual,targets,.012)
        if not sweep['safe']:raise NavigationBlocked('对正前转舵扫掠受阻')
        end=start.copy();end[2]=float(desired_yaw)
        motion=self.narrow.motion_edge(start,end,.012,steering=targets,
                                      hand_clearance=self.profile['hand_clearance_m'])
        if not motion['safe']:raise NavigationBlocked('对正柜门的整段转身净空不足')
        # 先证明对正后能接上平移，不能转完才发现原地已无路可走。
        if goal is not None:
            destination=goal.copy();destination[2]=end[2]
            try:
                self.narrow.q[self.narrow.steer_indices]=targets
                self.transit_route(end,destination)
            finally:self.narrow.q[self.narrow.steer_indices]=actual
        return {'steering_targets':np.asarray(targets).tolist(),'steering_sweep':sweep,
                'body_rotation':motion,'desired_yaw_rad':float(desired_yaw)}

    def _begin_alignment(self,start):
        self.narrow.refresh()
        self.goal[2]=self.operation_yaw
        proof=self._rotation_plan(start,self.operation_yaw,self.goal)
        self.alignment_plan=proof;self.alignment_origin=start.copy();self.alignment_started=self.elapsed
        self.command_time=1.;self.settle_window=[]
        self.state.update(phase='aligning',message='先停轮并切换转向轮姿态，再将胸部正面对准柜门／面板',
                          alignment_check=proof,operation_yaw_rad=self.operation_yaw,
                          facing_error_deg=math.degrees(wrap(start[2]-self.operation_yaw)))

    def _activate_translation(self,start,rebuild=True):
        self.yaw_hold=float(start[2]);self.goal[2]=self.yaw_hold
        self.lock_rotation=rotation(self.yaw_hold)
        if rebuild:
            self.narrow.refresh()
            self.geometry=NavigationGeometry(self.executor,self.station.runtime.stage,start)
            self.geometry.narrow=self.narrow
        self.recovery_indices=set();self.exit_before_alignment=False
        try:self.waypoints,self.waypoint_axes=self.transit_route(start,self.goal)
        except NavigationBlocked as error:self._install_safe_exit(start,error)
        else:
            self._reset_route(start)
            self.state['message']='胸部已对正操作面，保持朝向沿车身轴走位' if self.state['source']!='manual' else '停轮对舵后沿车身左右轴直移'
        self.state['yaw_hold']=self.yaw_hold

    def _step_alignment(self,pose):
        error=wrap(self.operation_yaw-pose[2])
        self.state.update(facing_error_deg=-math.degrees(error),actual_pose=pose.tolist())
        if np.linalg.norm(pose[:2]-self.alignment_origin[:2])>.03:
            raise RuntimeError('原地对正时底盘平移超过3厘米，已停止')
        if abs(error)<=math.radians(self.profile.get('alignment_stop_tolerance_deg',.75)):
            self.executor.set_base_velocity(0.,0.,vy=0.)
            if self.executor._base_cmd is None:
                self.state['alignment_actual_error_deg']=math.degrees(error)
                self._activate_translation(pose)
            return
        if self.elapsed-self.alignment_started>40.:raise RuntimeError('对正操作面超时，保持停止')
        if self.command_time>=.05:
            speed=math.copysign(min(self.profile['max_yaw_speed_rad_s'],max(.04,abs(error)*1.1)),error)
            self.narrow.refresh()
            control=getattr(self.executor,'_swerve_last_control',{}) or {}
            executed=float(control.get('executed_twist',[0.,0.,0.])[2])
            # 减速/反向时已执行角速度可能大于新请求；两侧最远弧段均须覆盖。
            rates=[max(0.,speed,executed),min(0.,speed,executed)]
            for rate in rates:
                if abs(rate)<1e-12:continue
                future=pose.copy();future[2]+=rate*.08
                check=self.narrow.motion_edge(pose,future,.012,steering=self.alignment_plan['steering_targets'],
                                             hand_clearance=self.profile['hand_clearance_m'])
                if not check['safe']:raise NavigationBlocked('对正过程前方净空发生变化')
            self.executor.set_base_velocity(0.,speed,vy=0.);self.command_time=0.
        if self.report_time>=.10:
            control=getattr(self.executor,'_swerve_last_control',{}) or {}
            self.samples.append({'sim_elapsed':self.elapsed,'base_position_xyz':self.state.get('base_position_xyz',
                    [float(pose[0]),float(pose[1]),float(self.vertical0[0])]),
                'base_roll_pitch_yaw_rad':self.state.get('base_roll_pitch_yaw_rad'),
                'axis':'yaw','wz_command':float((self.executor._base_cmd or {}).get('wz',0.)),
                'drive_control':control,'facing_error_deg':-math.degrees(error)})
            self.state['drive_control']=control;self.report_time=0.

    def _safety_module(self):
        import sys,types
        name='arm_transit_safety'
        if name not in sys.modules:
            path=Path(__file__).with_name(name+'.py');module=types.ModuleType(name)
            module.__file__=str(path);sys.modules[name]=module
            exec(compile(path.read_text(),str(path),'exec'),module.__dict__)
        return sys.modules[name]

    def _plan_transport(self):
        start,self.vertical0=self.pose()
        if self.state['source']=='manual':
            direction,distance=self.manual_request
            local=np.array([0.,distance if direction=='left' else -distance])
            self.goal=np.r_[start[:2]+rotation(start[2])@local,start[2]]
            position=np.r_[self.goal[:2],self.vertical0[0]]
        else:
            self.goal,position=self.target(self.state['asset_id'],self.state['source'])
        self.operation_yaw=None if self.state['source']=='manual' else float(self.goal[2])
        self.yaw_hold=float(start[2])
        self.lock_rotation=rotation(self.yaw_hold)
        self.geometry=NavigationGeometry(self.executor,self.station.runtime.stage,start)
        import sys,types
        name='navigation_narrowphase'
        if name not in sys.modules:
            path=Path(__file__).with_name(name+'.py');module=types.ModuleType(name);module.__file__=str(path);sys.modules[name]=module
            exec(compile(path.read_text(),str(path),'exec'),module.__dict__)
        self.narrow=sys.modules[name].NavigationNarrowphase(self.executor,self.safety)
        self.geometry.narrow=self.narrow
        self.recovery_indices=set();self.verified_leg=-1
        current=self.narrow.check(start,.012,nearest=True)
        self.state['whole_body_clearance']=current
        if not current['safe']:raise RuntimeError('真实底盘/障碍净空不足12毫米：'+str(current['nearest']))
        self.state.update(goal_pose=self.goal.tolist(),asset_position=position.tolist(),start_pose=start.tolist(),
                          operation_goal_pose=self.goal.tolist(),operation_yaw_rad=self.operation_yaw,
                          yaw_hold=self.yaw_hold,robot_collision_shapes=len(self.geometry.boxes),
                          collision_coverage=self.geometry.coverage,steering_sweep_modules=len(self.geometry.circles))
        if self.operation_yaw is not None and abs(wrap(start[2]-self.operation_yaw))>math.radians(self.profile['navigation_yaw_tolerance_deg']):
            try:self._begin_alignment(start)
            except NavigationBlocked as error:self._install_safe_exit(start,error)
        else:
            self._activate_translation(start,rebuild=False)
        self.station.overview()

    def request_translation(self,direction,distance=.10):
        if direction not in ('left','right') or not 0<distance<=.30:
            raise ValueError('仅支持机器人坐标左右0～30厘米的直移')
        self.manual_request=(direction,float(distance))
        return self.request('manual_'+direction,source='manual')

    def request(self,asset_id,source='l3'):
        if self.busy:raise RuntimeError('底盘或手臂正在准备，请先等待或停止')
        self.state={'phase':'preparing','asset_id':asset_id,'source':source,
                    'message':'先检查柜体净空并收纳双臂，尚未开始走位'}
        self.samples=[];self.stow_records=[];self.settle_window=[]
        self.elapsed=self.stable=self.command_time=self.report_time=0.
        self.started_wall=time.monotonic();self.monitor_time=1.;self.prep_clock=0.
        self.recovery_attempts=0;self.recovery_manual_only=False
        self.operation_yaw=None;self.exit_before_alignment=False
        self.state['automatic_recovery_enabled']=True
        self.state['automatic_recovery_max_attempts']=self.profile.get('automatic_recovery_max_attempts',1)
        self.prep_sides=['left','right'];self.prep_segment=None;self.prep_side=None
        self.prep_plan_pending=None;self.prep_park_requested=False
        self.prep_base_pose,self.prep_vertical0=self.pose()
        self.state['preparation_start_pose']=self.prep_base_pose.tolist()
        self.state['stow_reference']='当前六轴FK/几何验证的运输姿态；非零位复位'
        try:
            self.executor._base_cfg.update(steering_preposition_rate_rad_s=self.profile['steering_trajectory_speed_rad_s'],
                cardinal_acceleration_m_s2=self.profile['max_linear_acceleration_m_s2'],
                max_yaw_acceleration_rad_s2=self.profile.get('max_yaw_acceleration_rad_s2',.35))
            self.executor.set_render_interval(self.profile.get('gui_render_interval',1))
            drive=self.executor.get_base_drive_info()
            if drive.get('steering_reference') is None:
                drive=self.executor.configure_swerve_base(reference_steering_limit_deg=self.profile['steering_limit_deg'])
            self.state['steering_reference']=drive.get('steering_reference')
            self.executor.set_base_velocity(0.,0.,vy=0.)
            self.safety=getattr(self.executor, '_arm_transit_safety', None)
            if self.safety is None:
                self.safety=self._safety_module().ArmTransitSafety(self.executor)
                self.executor._arm_transit_safety=self.safety
            self.last_safety=self.safety.check(min_clearance_m=self.profile['hand_clearance_m'])
            if not self.last_safety['safe']:
                raise RuntimeError('当前手/臂环境净空不足，保持停止：'+str(self.last_safety['violations'][:1]))
            self.last_report_wall=0.
        except Exception as error:self._finish('failed','运输准备中止：'+str(error))
        return dict(self.state)

    def _finish(self,phase,message):
        try:self.executor.set_base_velocity(0.,0.,vy=0.)
        except Exception:pass
        if getattr(self,'prep_segment',None):
            try:
                names=self.prep_segment['names'];robot=self.executor.articulation
                values=np.asarray(robot.get_joint_positions()).reshape(-1)
                self.executor.send_joint_command([float(values[robot.dof_names.index(n)]) for n in names],duration=0,joint_names=names)
            except Exception:pass
        self.prep_segment=None
        if self.state.get('recovery_stage'):
            if phase=='failed' and self.state.get('recovery_exit_pose'):
                message='已自动退离；后续走位中止：'+message.removeprefix('已停轮取消操作：')
            self.state['recovery_stage']='completed' if phase in ('arrived','rolled_back') else phase
        self.state.update(phase=phase,message=message,elapsed_s=self.elapsed,
                          elapsed_wall_s=time.monotonic()-getattr(self,'started_wall',time.monotonic()))
        filename=self.state.get('source','l3')+'_'+self.state.get('asset_id','unknown')
        (self.output/(filename+'_approach.json')).write_text(json.dumps({'state':self.state,'samples':self.samples,
            'stow_records':getattr(self,'stow_records',[]),
            'drive':'panel_facing_then_fixed_yaw_cardinal_translation' if self.operation_yaw is not None else 'fixed_yaw_cardinal_translation',
            'reference_steering_limit_deg':self.profile['steering_limit_deg'],'calibrated_to_user_hardware':False},ensure_ascii=False,indent=2))

    def cancel(self):
        self._finish('cancelled','已停轮并取消尚未执行的操作')

    def _step_preparing(self,dt):
        pose,vertical=self.pose()
        self.state['preparation_current_pose']=pose.tolist()
        self.state['preparation_roll_pitch_yaw_rad']=[float(vertical[1]),float(vertical[2]),float(pose[2])]
        if abs(wrap(pose[2]-self.prep_base_pose[2]))>math.radians(2.):
            raise RuntimeError('收臂时车身偏航超过参考保护阈值，停止收臂和底盘')
        if max(abs(vertical[1:]))>math.radians(1.):
            raise RuntimeError('收臂时车身倾角超限，停止')
        if self.executor._base_cmd is not None:return
        self.prep_clock+=dt
        if self.prep_segment is None:
            if self.prep_plan_pending is not None:
                side,plan=self.prep_plan_pending
                parking=self.executor.get_base_drive_info().get('parking_verified',False)
                if not parking:raise RuntimeError('动态收臂前未完成前向驻车验证')
                self.prep_side=side;self.prep_plan_pending=None
                self._start_arm_segment(plan)
                return
            if not self.prep_sides:
                self.state['stow_records']=self.stow_records
                self._plan_transport();return
            side=self.prep_sides.pop(0);self.prep_side=side
            cached=getattr(self.executor, '_arm_transit_stow_goals', {}).get(side)
            if cached is not None:
                robot=self.executor.articulation
                actual=np.asarray(robot.get_joint_positions()).reshape(-1)
                indices=[robot.dof_names.index(n) for n in cached['joint_names']]
                if float(np.max(abs(actual[indices]-np.array(cached['goal_q']))))<.0015:
                    self.stow_records.append({'side':side,'reused_verified_transport_pose':True,
                                              'goal_q':cached['goal_q']})
                    return
            plan=self.safety.propose_stow(side,min_clearance_m=self.profile['hand_clearance_m'])
            self.stow_records.append({'side':side,'plan':plan})
            if not plan['safe']:raise RuntimeError(side+'臂未找到完整净空通过的收纳路径：'+plan['reason'])
            if plan.get('already_stowed'):
                if not hasattr(self.executor,'_arm_transit_stow_goals'):
                    self.executor._arm_transit_stow_goals={}
                self.executor._arm_transit_stow_goals[side]={
                    'joint_names':plan['joint_names'],'goal_q':plan['goal_q']}
                self.stow_records[-1]['skipped_motion']=True
                return
            self.prep_plan_pending=(side,plan)
            # 收臂前也不能在脚座旁直接回正舵轮；先按真实几何检查整个回正扫掠。
            import sys,types
            module_name='navigation_narrowphase'
            if module_name not in sys.modules:
                path=Path(__file__).with_name(module_name+'.py');module=types.ModuleType(module_name)
                module.__file__=str(path);sys.modules[module_name]=module
                exec(compile(path.read_text(),str(path),'exec'),module.__dict__)
            parking_check=sys.modules[module_name].NavigationNarrowphase(self.executor,self.safety)
            parked=parking_check.sweep(pose,parking_check.q[parking_check.steer_indices],np.zeros(4),.012)
            self.state['preparation_parking_sweep']=parked
            if not parked['safe']:
                self.prep_plan_pending=None
                raise RuntimeError('收臂前回正车轮将扫到脚座，保持停止；需先沿当前轮向安全退出')
            self.executor.park_base()
            self.state['message']='先将四轮回正驻车，再执行已验证的收臂路径'
            return
        segment=self.prep_segment;segment['t']+=dt
        u=min(segment['t']/segment['duration'],1.)
        command=segment['start']+(segment['goal']-segment['start'])*u*u*(3-2*u)
        self.executor.send_joint_command(command.tolist(),duration=0,joint_names=segment['names'])
        if u>=1.:
            robot=self.executor.articulation;actual=np.asarray(robot.get_joint_positions()).reshape(-1)
            indices=[robot.dof_names.index(n) for n in segment['names']]
            error=float(np.max(abs(actual[indices]-segment['goal'])))
            segment['stable']=segment['stable']+dt if error<.008 else 0.
            if segment['stable']>=.15:
                self.stow_records[-1]['actual_error_rad']=error
                if not hasattr(self.executor, '_arm_transit_stow_goals'):
                    self.executor._arm_transit_stow_goals={}
                self.executor._arm_transit_stow_goals[self.prep_side]={
                    'joint_names':segment['names'],'goal_q':segment['goal'].tolist()}
                self.prep_segment=None
            elif segment['t']>segment['duration']+2.:
                raise RuntimeError('收臂关节未跟踪到位，保持停止')

    def _start_arm_segment(self,plan):
        start=np.array(plan['start_q']);goal=np.array(plan['goal_q'])
        duration=max(.25,1.5*float(np.max(abs(goal-start)))/self.profile['transit_arm_speed_rad_s'])
        self.prep_segment={'names':plan['joint_names'],'start':start,'goal':goal,'duration':duration,'t':0.,'stable':0.}
        self.state['message']=('正在收纳左臂' if self.prep_side=='left' else '正在收纳右臂')+'，四轮驻车保持'

    def step(self,dt):
        if not self.busy:return
        self.elapsed+=dt;self.command_time+=dt;self.report_time+=dt;self.monitor_time+=dt
        try:
            if self.monitor_time>=.08:
                self.last_safety=self.safety.check(min_clearance_m=self.profile['hand_clearance_m'])
                self.monitor_time=0.
                self.state['hand_clearance']=self.last_safety
                if not self.last_safety['safe']:
                    raise RuntimeError('实时手臂净空不足：'+str(self.last_safety['violations'][:1]))
                if self.state['phase']!='preparing' and hasattr(self,'narrow'):
                    self.narrow.refresh(snapshot=(self.safety.q,self.safety.poses))
                    actual=self.pose()[0]
                    body_clear=self.narrow.check(actual,.012,nearest=True)
                    self.state['whole_body_clearance']=body_clear
                    if not body_clear['safe']:raise RuntimeError('真实轮/身体与障碍净空不足：'+str(body_clear['nearest']))
            if self.state['phase']=='preparing':
                self._step_preparing(dt);return
            pose,vertical=self.pose()
            if not np.isfinite(pose).all() or not np.isfinite(vertical).all():raise RuntimeError('底盘反馈非有限')
            if self.operation_yaw is not None:
                self.state['facing_error_deg']=math.degrees(wrap(getattr(self,'chest_yaw',pose[2])-self.operation_yaw))
            yaw_error=wrap(pose[2]-self.yaw_hold)
            rotating=self.state['phase']=='aligning' or (self.state['phase']=='recovering_stop' and self.state.get('recovery_origin_phase')=='aligning')
            if not rotating and abs(yaw_error)>math.radians(self.profile['translation_yaw_drift_limit_deg']):
                raise RuntimeError('车身航向漂移超限，已停轮；不会转车身代替横移')
            if max(abs(vertical[1:]))>math.radians(self.profile['translation_tilt_limit_deg']):
                raise RuntimeError('车身倾角超限，已停轮')
            if abs(vertical[0]-self.vertical0[0])>.025:raise RuntimeError('底盘高度异常，已停止')
            if self.state['phase']=='recovering_stop':
                self._step_recovery_stop();return
            if self.state['phase']=='aligning':
                self.state['base_position_xyz']=[float(pose[0]),float(pose[1]),float(vertical[0])]
                self.state['base_roll_pitch_yaw_rad']=[float(vertical[1]),float(vertical[2]),float(pose[2])]
                self._step_alignment(pose);return
            target=self.waypoints[self.index];axis=self.waypoint_axes[self.index]
            local_error=self.lock_rotation.T@(target[:2]-pose[:2])
            final=self.index==len(self.waypoints)-1
            tolerance=.005 if self.state['source']=='manual' else self.profile['navigation_xy_tolerance_m']
            outer_tolerance=.008 if self.state['source']=='manual' else .025
            axis_tolerance=tolerance/math.sqrt(2)*.9
            if self.index in self.recovery_indices:
                done=abs(local_error[axis])<.004 and abs(local_error[1-axis])<.012
            elif final and self.state['source']=='manual':
                along_limit=.008 if self.state['phase']=='settling' else .005
                cross_limit=.012 if self.state['phase']=='settling' else .010
                done=abs(local_error[axis])<along_limit and abs(local_error[1-axis])<cross_limit
            else:
                done=(np.linalg.norm(target[:2]-pose[:2])<(outer_tolerance if self.state['phase']=='settling' else tolerance)) if final else abs(local_error[axis])<axis_tolerance
            if done:
                self.executor.set_base_velocity(0.,0.,vy=0.)
                if self.executor._base_cmd is not None:return
                if self.index in self.recovery_indices:
                    self.state.update(recovery_exit_pose=pose.tolist(),recovery_stage='retrying',
                                      message='自动回退已完成，正在重新驶向原操作位置')
                    if self.recovery_manual_only:
                        self.state['stationary_verified']=True
                        self._finish('rolled_back','已自动回退至安全位置并停稳；本次横移未完成')
                        return
                    if self.exit_before_alignment:
                        self.exit_before_alignment=False
                        self._begin_alignment(pose)
                        return
                if not final:
                    self.index+=1;self.segment_start=pose.copy();self.command_time=1.;return
                if self.operation_yaw is not None and abs(wrap(pose[2]-self.operation_yaw))>math.radians(self.profile['navigation_yaw_tolerance_deg']):
                    self._begin_alignment(pose);return
                self.state['phase']='settling';self.state['message']='固定朝向已到位，正在确认停稳和双手净空'
                self.settle_window.append((self.elapsed,pose.copy()))
                self.settle_window=[r for r in self.settle_window if self.elapsed-r[0]<=.70]
                values=np.array([r[1] for r in self.settle_window])
                if self.elapsed-self.settle_window[0][0]>=.65 and np.max(np.ptp(values[:,:2],axis=0))<.004:
                    self.state.update(actual_pose=pose.tolist(),along_axis_error_m=float(local_error[axis]),cross_axis_error_m=float(local_error[1-axis]),position_error_m=float(np.linalg.norm(target[:2]-pose[:2])),
                        heading_error_deg=math.degrees(yaw_error),stationary_verified=True)
                    message=('纯左移完成' if self.state['asset_id']=='manual_left' else '纯右移完成') if self.state['source']=='manual' else '已到资产前方'
                    self.state['facing_error_deg']=None if self.operation_yaw is None else math.degrees(wrap(pose[2]-self.operation_yaw))
                    self._finish('arrived',message+('，胸部已正对柜门／面板' if self.operation_yaw is not None else '，车身朝向保持')+'，停稳和双手净空均通过')
                return
            # 单轴到位但横向误差仍超门限时补另一轴，不用混合vx/vy或转车身。
            if final and abs(local_error[axis])<axis_tolerance:
                if self.state['source']=='manual':
                    if abs(local_error[1-axis])>=.010:
                        raise RuntimeError('纯横移前后侧偏超过参考1厘米，停轮，不增加前后运动')
                else:
                    self.corrections+=1
                    if self.corrections>4:raise RuntimeError('固定朝向定位残差未收敛，保持停止')
                    self.waypoints.append(target.copy());self.waypoint_axes.append(1-axis)
                    self.index+=1;self.command_time=1.;return
            self.settle_window=[];self.state['phase']='moving'
            self.state['message']='四轮停稳后对舵，保持车身朝向沿'+('前后轴' if axis==0 else '左右轴')+'直移'
            if self.command_time>=.05:
                limit=.025 if self.index in self.recovery_indices else self.profile['max_linear_speed_m_s']
                value=float(np.clip(.9*local_error[axis],-limit,limit))
                if self.verified_leg!=self.index:
                    self.narrow.refresh()
                    proof=self._verify_steering(pose,axis)
                    self.state['current_steering_check']=proof
                    if not proof['safe']:raise NavigationBlocked('实际转舵路径将碰脚座：'+str(proof.get('nearest')))
                    self.current_leg_steering=np.array(proof['targets'],dtype=float)
                    self.verified_leg=self.index
                if abs(value)<self.profile['min_approach_speed_m_s']:
                    value=math.copysign(self.profile['min_approach_speed_m_s'],value)
                local=np.zeros(2);local[axis]=value
                world=rotation(pose[2])@local
                predicted=pose+np.r_[world*.20,0.]
                if not self.geometry.edge(pose,predicted,steering=self.current_leg_steering):raise NavigationBlocked('完整身体包络前方受阻')
                self.executor.set_base_velocity(vx=float(local[0]),vy=float(local[1]),wz=0.,
                                               steering_targets=self.current_leg_steering.tolist())
                self.command_time=0.
            if self.report_time>=.10:
                row={'sim_elapsed':self.elapsed,'base_position_xyz':[float(pose[0]),float(pose[1]),float(vertical[0])],
                     'base_roll_pitch_yaw_rad':[float(vertical[1]),float(vertical[2]),float(pose[2])],
                     'yaw_drift_deg':math.degrees(yaw_error),'waypoint':self.index,'axis':'x' if axis==0 else 'y',
                     'raw_body_command':dict(self.executor._base_cmd or {}),'wz_command':0.,
                     'drive_control':getattr(self.executor,'_swerve_last_control',None),
                     'hand_clearance':self.last_safety,'whole_body_clearance':self.state.get('whole_body_clearance')}
                self.samples.append(row)
                self.state.update(actual_pose=pose.tolist(),position_error_m=float(np.linalg.norm(self.goal[:2]-pose[:2])),
                    heading_error_deg=math.degrees(yaw_error),elapsed_s=self.elapsed,elapsed_wall_s=time.monotonic()-self.started_wall,
                    drive_control=row['drive_control'],base_roll_pitch_yaw_rad=row['base_roll_pitch_yaw_rad'])
                self.report_time=0.
            if np.linalg.norm(pose[:2]-self.progress_anchor[:2])>.008:
                self.progress_anchor=pose.copy();self.progress_time=self.elapsed
            control=getattr(self.executor,'_swerve_last_control',{}) or {}
            if self.index in self.recovery_indices:
                self.state['message']='走位受阻：保持现有轮向低速自动回退中'
            elif self.state.get('recovery_stage')=='retrying':
                self.state['message']='自动回退已完成，正在重新驶向原操作位置'
            if control.get('phase')=='prealign':
                self.progress_time=self.elapsed
                self.state['message']='滚轮保持停止，四轮正在同步对舵；车身不主动转向'
            if self.elapsed-self.progress_time>15. or self.elapsed>130.:
                raise RuntimeError('直向走位超时，保持停止')
        except NavigationBlocked as error:
            if not self._queue_recovery(error):
                self._finish('failed','自动回退未能排除受阻，已停轮取消操作：'+str(error))
        except Exception as error:self._finish('failed','已停轮取消操作：'+str(error))
