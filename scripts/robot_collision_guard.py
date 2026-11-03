"""从当前USD导出的碰撞几何做离线/运行时检查，规划不改变仿真关节。"""
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


class CollisionModel:
    def __init__(self, directory, exclusions=None):
        import coal
        self.coal=coal
        directory=Path(directory)
        self.data=json.loads((directory/'snapshot.json').read_text())
        self.names=self.data['names'];self.q0=np.array(self.data['q'])
        self.poses0={p:np.array(t) for p,t in self.data['poses'].items()}
        self.joints=[]
        children={j['body1'] for j in self.data['joints']}
        self.root=next(p for p,b in self.data['bodies'].items() if b['robot'] and p not in children)
        pending=list(self.data['joints']);known={self.root}
        while pending:
            found=[j for j in pending if j['body0'] in known]
            if not found:raise ValueError('机器人关节树不连通')
            for j in found:
                self.joints.append({**j,'A':np.array(j['T0']),'B':np.linalg.inv(j['T1']),
                    'index':self.names.index(j['name']) if j['name'] in self.names else None})
                known.add(j['body1']);pending.remove(j)
        # 固定装配视为一个刚性组件；相邻可动组件由原关节的collisionEnabled决定。
        parent={p:p for p in known}
        def find(p):
            while parent[p]!=p:p=parent[p]
            return p
        for j in self.joints:
            if j['type']=='PhysicsFixedJoint':parent[find(j['body1'])]=find(j['body0'])
        self.cluster={p:find(p) for p in known}
        adjacent={frozenset((self.cluster[j['body0']],self.cluster[j['body1']])) for j in self.joints if not j['collision_enabled']}
        self.excluded=set()
        self.exclusion_reasons={}
        for a in known:
            for b in known:
                pair=frozenset((a,b))
                if a==b:continue
                if self.cluster[a]==self.cluster[b] or frozenset((self.cluster[a],self.cluster[b])) in adjacent:
                    self.excluded.add(pair);self.exclusion_reasons[pair]='同一固定组件或相邻关节组件'
        for row in exclusions or []:
            pair=frozenset(row['bodies']);self.excluded.add(pair);self.exclusion_reasons[pair]=row['reason']
        self.shapes=[];arrays=np.load(directory/'geometry.npz')
        for row in self.data['shapes']:
            if row['type']=='Plane':continue
            if 'key' in row:
                vertices=arrays[row['key']+'_v'];faces=arrays[row['key']+'_f']
            elif row['type']=='Cylinder':
                radius=float(row['attributes']['radius']);height=float(row['attributes']['height'])
                angle=np.arange(96)*2*math.pi/96
                vertices=np.array([[radius/math.cos(math.pi/96)*math.cos(a),radius/math.cos(math.pi/96)*math.sin(a),z] for z in [-height/2,height/2] for a in angle])
                axis=row['attributes']['axis']
                if axis=='X':vertices=vertices[:,[2,0,1]]
                elif axis=='Y':vertices=vertices[:,[1,2,0]]
                matrix=np.array(row['matrix']);vertices=vertices@matrix[:3,:3].T+matrix[:3,3]
                faces=None
            else:raise ValueError('未支持碰撞形状：'+row['type'])
            verts=coal.StdVec_Vec3s();verts.extend(vertices)
            if row['approximation']=='none' and faces is not None:
                tri=coal.StdVec_Triangle();tri.extend([coal.Triangle(*map(int,f)) for f in faces])
                geom=coal.BVHModelOBBRSS();geom.beginModel(len(faces),len(vertices));geom.addSubModel(verts,tri);geom.endModel()
            else:
                geom=coal.Convex.convexHull(verts,True,'Qt')
            bounds=np.array([vertices.min(0),vertices.max(0)])
            import itertools
            corners=np.array(list(itertools.product(*zip(bounds[0],bounds[1]))))
            self.shapes.append({**row,'geometry':geom,'corners':corners})
        pairs=[]
        for i,a in enumerate(self.shapes):
            for j in range(i):
                b=self.shapes[j]
                if not (a['robot'] or b['robot']):continue
                if a['body'] and a['body']==b['body']:continue
                if a['robot'] and b['robot'] and frozenset((a['body'],b['body'])) in self.excluded:continue
                pairs.append((i,j))
        self.pairs=np.array(pairs,dtype=int)
        self.request=coal.DistanceRequest();self.request.enable_signed_distance=True

    def fk(self,q,root_pose=None):
        q=np.asarray(q);poses=dict(self.poses0)
        poses[self.root]=np.asarray(root_pose) if root_pose is not None else self.poses0[self.root]
        for j in self.joints:
            delta=np.eye(4)
            if j['index'] is not None:
                value=q[j['index']];axis='XYZ'.index(j['axis'])
                if j['type']=='PhysicsRevoluteJoint':
                    # 轴已由USD约束为XYZ，直接构造等价旋转，避免每个连杆调用通用SO(3)分派。
                    c,s=math.cos(value),math.sin(value)
                    if axis==0:delta[:3,:3]=[[1,0,0],[0,c,-s],[0,s,c]]
                    elif axis==1:delta[:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]]
                    else:delta[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
                else:delta[axis,3]=value
            poses[j['body1']]=poses[j['body0']]@j['A']@delta@j['B']
        return poses

    def query(self,q=None,root_pose=None,poses=None,margin=.005,only_self=False,allowed=None):
        coal=self.coal
        poses=self.fk(self.q0 if q is None else q,root_pose) if poses is None else poses
        transforms=[];lo=[];hi=[]
        for s in self.shapes:
            t=poses[s['body']] if s['body'] else np.eye(4)
            transforms.append(coal.Transform3s(t[:3,:3],t[:3,3]))
            pts=s['corners']@t[:3,:3].T+t[:3,3];lo.append(pts.min(0));hi.append(pts.max(0))
        lo=np.array(lo);hi=np.array(hi);i,j=self.pairs.T
        gap=np.maximum(lo[i]-hi[j],lo[j]-hi[i]);indices=np.flatnonzero(np.max(gap,axis=1)<=margin)
        hits=[]
        for k in indices:
            i,j=self.pairs[k];a,b=self.shapes[i],self.shapes[j]
            if only_self and not (a['robot'] and b['robot']):continue
            if allowed and allowed(a,b):continue
            result=coal.DistanceResult()
            distance=coal.distance(a['geometry'],transforms[i],b['geometry'],transforms[j],self.request,result)
            if distance<margin:
                hits.append({'distance_m':float(distance),'a':a['body'] or a['path'],'b':b['body'] or b['path'],
                    'shape_a':a['path'],'shape_b':b['path'],'self':a['robot'] and b['robot']})
        return sorted(hits,key=lambda h:h['distance_m'])

    def check_fk(self):
        poses=self.fk(self.q0);errors=[]
        for path,t in poses.items():
            if path not in self.cluster:continue
            ref=self.poses0[path]
            errors.append({'body':path,'position_m':float(np.linalg.norm(t[:3,3]-ref[:3,3])),
                'angle_rad':float(Rotation.from_matrix(t[:3,:3]@ref[:3,:3].T).magnitude())})
        return sorted(errors,key=lambda x:x['position_m'],reverse=True)


def clearance_for(hit):
    """装配内部留零间隙接触余量，手臂与身体/外物保留毫米级规划余量。"""
    a,b=hit['a'].split('/')[-1],hit['b'].split('/')[-1]
    if hit['self']:
        if any(a.startswith(side+'_gripper_') and b.startswith(side+'_gripper_') for side in ['left','right']):
            return -.0002
        if not any('_arm_' in n or '_gripper_' in n or 'camera' in n for n in [a,b]):
            return .001
        return .008
    return .005


class MotionGuard:
    def __init__(self,executor,directory):
        self.executor=executor;self.directory=Path(directory)
        self.policy=json.loads((self.directory/'policy.json').read_text())
        self.model=CollisionModel(self.directory/'backup',self.policy['extra_exclusions'])
        self.arm_indices=[self.model.names.index(f'right_arm_{i}_joint') for i in range(1,7)]
        self.support={frozenset(x['bodies']) for x in self.policy['environment_allowed_pairs']}
        self.contact_allowed=False;self.enabled=True;self.events=[];self.last_path=None
        self.target_cap='/World/L3Station/Controls/object_0_variant/Cap'

    def read(self):
        ex=self.executor;poses=dict(self.model.poses0)
        views=[ex.articulation]
        for name in ['_oxford_devices']:
            if hasattr(ex,name):views+=list(getattr(ex,name).devices.values())
        if hasattr(ex,'_l3_station'):views+=list(ex._l3_station.runtime.devices.values())
        for view in views:
            values=np.asarray(view._physics_view.get_link_transforms())[0]
            for path,v in zip(view._physics_view.link_paths[0],values):
                t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();t[:3,3]=v[:3];poses[path]=t
        return np.asarray(ex.articulation.get_joint_positions()).reshape(-1),poses

    def allowed(self,a,b):
        ba,bb=a['body'] or a['path'],b['body'] or b['path']
        if frozenset([ba,bb]) in self.support:return True
        if self.contact_allowed and self.target_cap in [ba,bb]:
            other=bb if ba==self.target_cap else ba
            return other.split('/')[-1] in ['right_gripper_right_pad_link','right_gripper_left_pad_link','right_gripper_right_in_link','right_gripper_left_in_link']
        return False

    def violations(self,q=None,poses=None,runtime=False):
        hits=self.model.query(q=q,poses=poses,margin=.008,allowed=self.allowed)
        def threshold(h):
            value=clearance_for(h)
            if runtime:
                if value<0:return -.0005
                return .002 if value>=.005 else -.0005
            return value
        return [h for h in hits if h['distance_m']<threshold(h)]

    def refresh(self):
        q,poses=self.read();self.model.poses0=poses
        return q

    def arm_clear(self,arm,q=None):
        full=(self.model.q0.copy() if q is None else q.copy());full[self.arm_indices]=arm
        return not self.violations(full)

    def edge(self,a,b,base,step=.02):
        n=max(1,int(np.ceil(np.max(abs(b-a))/step)))
        return all(self.arm_clear(a+(b-a)*u,base) for u in np.linspace(0,1,n+1))

    def _dense_verify(self,path,base):
        # 用所有右臂碰撞体角点的实际位移控制采样间距，避免只检查轨迹端点。
        moving=[s for s in self.model.shapes if s['robot'] and '/right_arm_base_link/' in s['body']]
        def points(arm):
            q=base.copy();q[self.arm_indices]=arm;poses=self.model.fk(q)
            return np.concatenate([s['corners']@poses[s['body']][:3,:3].T+poses[s['body']][:3,3] for s in moving])
        samples=[]
        def visit(a,b,depth=0):
            if not self.arm_clear(a,base) or not self.arm_clear(b,base):raise RuntimeError('轨迹存在碰撞或净空不足')
            mid=(a+b)/2
            if not self.arm_clear(mid,base):raise RuntimeError('轨迹中段存在碰撞或净空不足')
            displacement=float(np.max(np.linalg.norm(points(b)-points(a),axis=1)))
            # 大关节跨度即使端点位置相近也必须拆分，不能漏掉绕回的扫掠。
            if displacement>.002 or np.max(abs(b-a))>.01:
                if depth>=16:raise RuntimeError('轨迹碰撞采样未收敛')
                visit(a,mid,depth+1);visit(mid,b,depth+1)
            else:samples.append({'max_point_step_m':displacement})
        for a,b in zip(path,path[1:]):visit(a,b)
        return {'samples':len(samples),'max_point_step_m':max((x['max_point_step_m'] for x in samples),default=0.)}

    def plan(self,target):
        base=self.refresh();start=base[self.arm_indices].copy();goal=np.asarray(target,dtype=float)
        if not self.arm_clear(start,base):raise RuntimeError('当前位置碰撞或净空不足，已禁止自动回位')
        if not self.arm_clear(goal,base):raise RuntimeError('目标姿态碰撞或净空不足，已禁止自动回位')
        path=[start,goal]
        if not self.edge(start,goal,base):
            limits=np.asarray(self.model.data['limits']).reshape(-1,2)[self.arm_indices]
            rng=np.random.default_rng(729);nodes=[start];parents=[-1]
            solved=False
            for k in range(1200):
                sample=goal if k%5==0 else (np.clip(start+rng.normal(0,.65,6),limits[:,0]+.03,limits[:,1]-.03) if k%3 else rng.uniform(limits[:,0]+.03,limits[:,1]-.03))
                nearest=int(np.argmin(np.linalg.norm(np.asarray(nodes)-sample,axis=1)));a=nodes[nearest]
                delta=sample-a;candidate=a+delta*min(1.,.20/max(np.linalg.norm(delta),1e-8))
                if not self.edge(a,candidate,base):continue
                nodes.append(candidate);parents.append(nearest)
                if self.edge(candidate,goal,base):
                    index=len(nodes)-1;path=[goal]
                    while index>=0:path.append(nodes[index]);index=parents[index]
                    path.reverse();solved=True;break
            if not solved:raise RuntimeError('没有找到可通行的回位路径，手臂保持当前位置')
            # 只合并经过同一碰撞检查的捷径。
            i=0
            while i<len(path)-2:
                for j in range(len(path)-1,i+1,-1):
                    if self.edge(path[i],path[j],base):path=path[:i+1]+path[j:];break
                i+=1
        verification=self._dense_verify(path,base)
        self.last_path={'path':[p.tolist() for p in path],**verification,'contact_allowed':self.contact_allowed}
        return path

    def step(self,dt):
        station=getattr(self.executor,'_l3_station',None)
        if not self.enabled or not station or not getattr(station,'_robot_busy',False) or getattr(station,'_robot_cancel',False):return
        q,poses=self.read()
        if not np.isfinite(q).all():hits=[{'reason':'关节反馈非有限数'}]
        else:hits=self.violations(poses=poses,runtime=True)
        if hits:
            self.events.append({'time':float(self.executor.world.current_time),'hits':hits})
            station.stop();station._status='碰撞保护已停止：手臂接近身体或障碍物'
            station._collision_stop=True
