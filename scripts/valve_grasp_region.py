"""显式局部夹持区域：仅改变抓取宽度/目标，不删减实际轮盘碰撞几何。"""
import math
import numpy as np
from scipy.spatial import ConvexHull


def apply_region(geometry, region):
    point=np.asarray(region['point_local_m'],float)
    axis='XYZ'.index(region['axis'])
    half=float(region['half_span_m'])
    if point.shape!=(3,) or not np.isfinite(point).all() or not math.isfinite(half) or half<=0:
        raise ValueError('局部夹持区域参数无效')
    points=np.asarray(geometry['support_vertices_local_m'],float)
    hull=ConvexHull(points);low,high=point[axis]-half,point[axis]+half
    vertices=[p for p in points if low<=p[axis]<=high]
    edges={tuple(sorted((int(face[i]),int(face[j]))))for face in hull.simplices for i,j in ((0,1),(1,2),(2,0))}
    for i,j in edges:
        a,b=points[i],points[j]
        if abs(b[axis]-a[axis])<1e-15:continue
        for boundary in (low,high):
            t=(boundary-a[axis])/(b[axis]-a[axis])
            if 0<=t<=1:vertices.append(a+(b-a)*t)
    if len(vertices)<4:raise ValueError('局部夹持区域未与实际轮盘凸包相交')
    vertices=np.asarray(vertices);local_hull=ConvexHull(vertices)
    if np.max(local_hull.equations[:,:3]@point+local_hull.equations[:,3])>1e-8:
        raise ValueError('夹持点不在指定局部轮盘区域内')
    result=dict(geometry)
    result['grasp_support_vertices_local_m']=vertices[local_hull.vertices].tolist()
    result['grasp_point_local_m']=point.tolist()
    result['grasp_region']=dict(region)
    return result
