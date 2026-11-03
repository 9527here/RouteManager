"""Tensor拓扑失效时停止控制读取；不能靠重试getDofPositions恢复旧view。"""
_UNSET = object()


def guard(executor, logger, simulation_view=_UNSET):
    fault = getattr(executor, '_physics_view_fault', None)
    if fault is None:
        try:
            if getattr(executor.world, '_scene', None) is None:
                fault = '场景切换已销毁旧World；须重建World后恢复，禁止继续旧主循环'
            elif executor.articulation is None:
                return True
            else:
                if simulation_view is _UNSET:
                    from isaacsim.core.simulation_manager import SimulationManager
                    simulation_view = SimulationManager.get_physics_sim_view()
                ready = (simulation_view is not None and simulation_view.is_valid
                         and executor.articulation._physics_view is not None
                         and executor.articulation._physics_view.check())
                if ready:
                    return True
                fault = 'PhysX Tensor视图已失效；须显式重建后恢复，禁止反复读取旧视图'
        except Exception as error:
            fault = 'PhysX Tensor视图校验失败: ' + str(error)
        executor._physics_view_fault = fault
        # 不再给失效后端下发“停止目标”；先暂停物理并清除待执行任务。
        for name in ('_motion', '_auto_pick', '_base_cmd', '_base_parking'):
            if hasattr(executor, name):
                setattr(executor, name, None)
        logger.error(fault)
    try:
        executor.world.pause()
    except (AttributeError, RuntimeError):
        # open_stage可能已销毁旧World；暂停当前timeline，避免旧World再带崩主循环。
        import omni.timeline
        omni.timeline.get_timeline_interface().pause()
    return False


def acknowledge_rebuilt_view(executor, logger, simulation_view=_UNSET):
    """只供明确重建流程调用；解除失效锁存后仍暂停，不自动恢复旧任务。"""
    old = getattr(executor, '_physics_view_fault', None)
    executor._physics_view_fault = None
    if not guard(executor, logger, simulation_view):
        raise RuntimeError(executor._physics_view_fault)
    executor.world.pause()
    return {'valid': True, 'paused': True, 'previous_fault': old}
