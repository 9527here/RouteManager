"""导航粗包络候选的全身三维复核，转舵必须单独扫掠；不修改物理状态。"""
import math
import numpy as np


class NavigationNarrowphase:
    def __init__(self, executor, safety):
        self.executor, self.safety = executor, safety
        self.model = safety.model
        self.pairs = np.array([(int(i),int(j)) for i,j in safety._original_pairs
            if self.model.shapes[i]['robot'] != self.model.shapes[j]['robot']
            and not any(token in (self.model.shapes[i]['path']+' '+self.model.shapes[j]['path']).lower()
                        for token in ('ground_plane','groundplane'))],dtype=int).reshape(-1,2)
        names=list(executor.articulation.dof_names)
        self.steer_indices=[names.index(m.steering_joint) for m in executor._get_swerve_model().modules]
        self.refresh()

    def refresh(self,snapshot=None):
        # 同一主线程物理帧中，手部监控已读过真值时复用该快照；不跨物理步缓存。
        q,self.poses=self.safety.refresh() if snapshot is None else snapshot
        self.q=np.asarray(q).reshape(-1).copy()
        if self.q.size!=len(self.model.names) or not np.isfinite(self.q).all():
            raise RuntimeError('导航关节快照无效')
        self.root=np.array(self.poses[self.model.root],copy=True)
        if not np.isfinite(self.root).all():raise RuntimeError('导航根位姿非有限')
        self.yaw=math.atan2(self.root[1,0],self.root[0,0])

    def predict(self,pose,steering=None):
        angle=float(pose[2])-self.yaw
        c,s=math.cos(angle),math.sin(angle)
        R=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
        root=self.root.copy();root[:3,:3]=R@root[:3,:3];root[:2,3]=pose[:2]
        q=self.q.copy()
        if steering is not None:q[self.steer_indices]=steering
        poses=self.model.fk(q,root)
        poses.update({p:t for p,t in self.poses.items() if p not in self.model.cluster})
        return poses

    def check(self,pose,margin=.012,steering=None,nearest=False):
        return self._check_poses(self.predict(pose,steering),pose,margin,nearest)

    def _check_poses(self,poses,pose,margin,nearest=False):
        coal=self.model.coal
        frames,low,high=self.safety._shape_bounds(poses,'导航精确复核位姿非有限')
        transforms={}
        def transform(index):
            if index not in transforms:
                frame=frames[index]
                transforms[index]=coal.Transform3s(frame[:3,:3],frame[:3,3])
            return transforms[index]
        i,j=self.pairs.T
        gap=np.maximum(low[i]-high[j],low[j]-high[i])
        lower=np.linalg.norm(np.maximum(gap,0.),axis=1)
        best=None;violations=[]
        order=np.argsort(lower) if nearest else np.flatnonzero(lower<=margin)
        for index in order:
            if nearest and best is not None and best['distance_m']>=0 and lower[index]>=best['distance_m']:
                break
            if nearest and best is not None and best['distance_m']<0 and lower[index]>0:break
            a,b=map(int,self.pairs[index]);sa,sb=self.model.shapes[a],self.model.shapes[b]
            result=coal.DistanceResult()
            distance=float(coal.distance(sa['geometry'],transform(a),sb['geometry'],transform(b),self.model.request,result))
            if not math.isfinite(distance):raise RuntimeError('导航精确距离非有限，保持停止')
            hit={'distance_m':distance,'shape_a':sa['path'],'shape_b':sb['path'],
                 'body_a':sa['body'],'body_b':sb['body']}
            if best is None or distance<best['distance_m']:best=hit
            if distance<margin:violations.append(hit)
        return {'safe':not violations,'minimum_distance_m':None if best is None else best['distance_m'],
                'nearest':best,'violations':sorted(violations,key=lambda x:x['distance_m']),
                'margin_m':margin,'method':'fresh_all_robot_environment_coal_3d','pose':np.asarray(pose).tolist()}

    def _motion_sampling(self,start,goal,steering):
        delta=np.asarray(goal,dtype=float)-np.asarray(start,dtype=float)
        delta[2]=math.atan2(math.sin(delta[2]),math.cos(delta[2]))
        radius=0.
        if abs(delta[2])>1e-12:
            # 原地转向也须按最外侧手/臂/底盘的扫掠弧长采样，不能只检查三个角度。
            poses=self.predict(start,steering)
            for shape in self.model.shapes:
                if not shape['robot']:continue
                frame=poses[shape['body']]
                corners=shape['corners']@frame[:3,:3].T+frame[:3,3]
                radius=max(radius,float(np.linalg.norm(corners[:,:2]-np.asarray(start)[:2],axis=1).max()))
        count=max(2,int(math.ceil((np.linalg.norm(delta[:2])+radius*abs(delta[2]))/.002)))
        return delta,count

    def edge(self,start,goal,margin=.012,steering=None):
        delta,count=self._motion_sampling(start,goal,steering)
        minimum=float('inf');worst=None
        for u in np.linspace(0.,1.,count+1):
            result=self.check(np.asarray(start)+delta*u,margin,steering)
            if result['minimum_distance_m'] is not None and result['minimum_distance_m']<minimum:
                minimum=result['minimum_distance_m'];worst=result['nearest']
            if not result['safe']:return {**result,'fraction':float(u),'safe':False}
        return {'safe':True,'minimum_distance_m':None if not math.isfinite(minimum) else minimum,
                'nearest':worst,'samples':count+1,'margin_m':margin}

    def motion_edge(self,start,goal,margin=.012,steering=None,hand_clearance=.05):
        """同一批连续位姿同时验证全身和双手净空，保留原始距离与采样门槛。"""
        delta,count=self._motion_sampling(start,goal,steering)
        minimum=float('inf');hand_minimum=float('inf');worst=None
        for u in np.linspace(0.,1.,count+1):
            pose=np.asarray(start)+delta*u
            poses=self.predict(pose,steering)
            result=self._check_poses(poses,pose,margin)
            if result['minimum_distance_m'] is not None and result['minimum_distance_m']<minimum:
                minimum=result['minimum_distance_m'];worst=result['nearest']
            if not result['safe']:return {**result,'fraction':float(u),'safe':False}
            hand=self.safety.check(min_clearance_m=hand_clearance,refresh=False,poses=poses)
            if hand['minimum_distance_m'] is not None:
                hand_minimum=min(hand_minimum,hand['minimum_distance_m'])
            if not hand['safe']:
                return {**result,'safe':False,'fraction':float(u),'hand_clearance':hand,
                        'reason':'手臂/夹爪连续净空不足'}
        return {'safe':True,'minimum_distance_m':None if not math.isfinite(minimum) else minimum,
                'hand_minimum_distance_m':None if not math.isfinite(hand_minimum) else hand_minimum,
                'nearest':worst,'samples':count+1,'margin_m':margin,'hand_clearance_m':hand_clearance}

    def sweep(self,pose,from_steering,to_steering,margin=.012):
        a,b=np.asarray(from_steering),np.asarray(to_steering)
        count=max(2,int(math.ceil(np.max(abs(b-a))/math.radians(.25))))
        minimum=float('inf');worst=None
        for u in np.linspace(0.,1.,count+1):
            result=self.check(pose,margin,a+(b-a)*u)
            if result['minimum_distance_m'] is not None and result['minimum_distance_m']<minimum:
                minimum=result['minimum_distance_m'];worst=result['nearest']
            if not result['safe']:return {**result,'fraction':float(u),'safe':False}
        return {'safe':True,'minimum_distance_m':None if not math.isfinite(minimum) else minimum,
                'nearest':worst,'samples':count+1,'margin_m':margin}
