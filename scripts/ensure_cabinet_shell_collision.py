"""为已确认缺失碰撞的柜壳补静态三角网格；仅覆盖会话层，不改源资产。"""
from pathlib import Path


def ensure(executor):
    from pxr import Usd, UsdPhysics, PhysxSchema
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    path = '/World/Cabinet/CabinetBody/CabinetBody'
    prim = stage.GetPrimAtPath(path)
    if not prim:
        return {'installed': False, 'reason': '当前场景没有已知动力柜壳'}
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError('柜壳是动态刚体，不能套用静态三角网格配置')
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(.001)
            api.CreateRestOffsetAttr(0.)
    return {'installed': True, 'path': path, 'layer': 'session', 'source_assets_modified': False,
            'mesh_approximation': UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get(),
            'numerical_contact_offset_m': .001}
