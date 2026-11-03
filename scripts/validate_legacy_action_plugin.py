#!/usr/bin/env python3
"""通过中文插件同一回调验收旧动力柜与迁移机构；不加载或重置机器人。"""
import argparse
import inspect
import json
import urllib.request
from pathlib import Path


def validate(executor):
    import json
    import math
    import time
    import numpy as np
    from pxr import Usd, UsdGeom, UsdShade
    station = executor._l3_station
    station._ensure_actions()
    if station._robot_busy or station.queue or station._legacy_job:
        raise RuntimeError('请先完成当前动作，再验收旧机构')
    probe = getattr(executor, '_cabinet_probe', None)
    oxford = getattr(executor, '_oxford_devices', None)
    robot = executor.articulation
    before = np.asarray(robot.get_joint_positions()).reshape(-1).copy()
    report = {'method': '执行中文插件绑定的同一回调，读取真实PhysX刚体/关节与材质绑定反馈',
              'started_unix': time.time(), 'checks': [], 'cleanup': [], 'pass': False}
    output = station.output / 'legacy_plugin_checks.json'
    def save():
        temporary = output.with_suffix('.tmp')
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        temporary.replace(output)
    def check(title, passed, **values):
        report['checks'].append({'name': title, 'pass': bool(passed), **values})
        save()
    def step(count):
        for frame in range(count):
            executor.step(render=frame % 4 == 0)
    def run(title, sample=None, record=True):
        """单项失败继续验收下一项，所有循环有明确帧数上限。"""
        initial_history = len(station._action_history)
        samples = []
        callback_error = None
        try:
            station.ui_callbacks[title]()
            for frame in range(40000):
                executor.step(render=frame % 4 == 0)
                if sample is not None:
                    samples.append(sample())
                if station._legacy_job is None and not station._approach_busy():
                    break
            timed_out = station._legacy_job is not None or station._approach_busy()
            if timed_out:
                station.stop()
            result = (station._action_history[-1]
                      if len(station._action_history) > initial_history else {})
            good = bool(result.get('pass')) and not timed_out
        except Exception as error:
            callback_error = str(error)
            station.stop()
            result, good = {}, False
        if record:
            check(title, good, result=result, callback_error=callback_error,
                  sample_count=len(samples))
        return good, samples, result
    def lamp_binding(name):
        result = {}
        for prim in Usd.PrimRange(probe.stage.GetPrimAtPath(probe.controls[name]['body1'])):
            if prim.IsA(UsdGeom.Mesh):
                material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
                result[str(prim.GetPath())] = str(material.GetPath()) if material else None
        return result
    try:
        save()
        if probe is None:
            check('动力柜资产已加载', False, reason='未找到动力柜控件探针')
        else:
            for name, title in [('Btn04', '柜按钮4'), ('Btn05', '柜按钮5'), ('Btn06', '柜按钮6'),
                                ('Btn07', '柜按钮7'), ('Lock_Btn', '锁按钮')]:
                if name not in probe.controls:
                    check(title + '控件已加载', False)
                    continue
                try:
                    info = probe.controls[name]
                    baseline, _ = probe._relative(info)
                    good, samples, result = run(title + '按压回位',
                        sample=lambda i=info, x=baseline: float(probe._relative(i)[0] - x), record=False)
                    peak = -min(samples) if samples else 0.
                    residual = abs(samples[-1]) if samples else None
                    check(title + '按下并回位', good and peak >= (.004 if name == 'Lock_Btn' else .0045)
                          and residual is not None and residual < .0005,
                          peak_stroke_m=peak, residual_m=residual, result=result)
                except Exception as error:
                    check(title + '按下并回位', False, reason=str(error))
                    station.stop()
            for name, title in [('Knob01', '柜旋钮1'), ('Knob02', '柜旋钮2')]:
                if name not in probe.controls:
                    check(title + '控件已加载', False)
                    continue
                try:
                    run(title + '转至0°', record=False)
                    _, q0 = probe._relative(probe.controls[name])
                    for target in (45., -45., 0.):
                        good, _, result = run(f'{title}转至{target:g}°', record=False)
                        _, current = probe._relative(probe.controls[name])
                        delta = q0.GetInverse() * current
                        actual = (math.degrees(2. * math.atan2(float(delta.GetImaginary()[0]),
                                                              float(delta.GetReal()))) + 180.) % 360. - 180.
                        check(f'{title}转至{target:g}°', good and abs(actual-target) < 2.,
                              target_deg=target, actual_deg=actual, result=result)
                except Exception as error:
                    check(title + '角度验收', False, reason=str(error))
                    station.stop()
            for name, title in [('Btn01', '柜顶指示灯1'), ('Btn02', '柜顶指示灯2'), ('Btn03', '柜顶指示灯3')]:
                if name not in probe.controls:
                    check(title + '控件已加载', False)
                    continue
                try:
                    station.ui_callbacks[title + '熄灭']()
                    step(8)
                    original = lamp_binding(name)
                    station.ui_callbacks[title + '点亮']()
                    step(8)
                    lit = lamp_binding(name)
                    expected = f'/World/Cabinet/TestMaterials/{name}_On'
                    check(title + '点亮材质绑定', bool(lit) and all(v == expected for v in lit.values()),
                          materials=lit)
                    station.ui_callbacks[title + '熄灭']()
                    step(8)
                    restored = lamp_binding(name)
                    check(title + '熄灭并恢复材质', bool(restored) and restored == original,
                          materials=restored)
                except Exception as error:
                    check(title + '灯态验收', False, reason=str(error))
        if oxford is None:
            check('迁移机构已加载', False, reason='未找到迁移机构物理视图')
        else:
            for name, title, target in [('isolator_switch','隔离开关',90.), ('isolator_switch','隔离开关',0.),
                                        ('safety_switch','安全手柄',-90.), ('safety_switch','安全手柄',0.)]:
                if name not in oxford.devices:
                    check(title + '控件已加载', False)
                    continue
                old_gains = tuple(np.asarray(v).copy() for v in oxford.devices[name].get_gains())
                good, _, result = run(f'{title}转至{target:g}°', record=False)
                q = np.asarray(oxford.devices[name].get_joint_positions()).reshape(-1)
                actual = math.degrees(float(q[oxford.devices[name].dof_names.index('switch_joint')]))
                active = bool(oxford.states[name].active)
                restored = all(np.array_equal(a, b) for a, b in
                               zip(old_gains, oxford.devices[name].get_gains()))
                check(f'{title}转至{target:g}°', good and abs(actual-target) < 2.3
                      and active == bool(target) and restored,
                      actual_deg=actual, activation_state=active, gains_restored=restored, result=result)
            for title, active in [('急停：按下锁定', True), ('急停：旋转解锁并回位', False)]:
                name = 'e_stop_button'
                if name not in oxford.devices:
                    check(title + '控件已加载', False)
                    continue
                old_gains = tuple(np.asarray(v).copy() for v in oxford.devices[name].get_gains())
                good, _, result = run(title, record=False)
                q = dict(zip(oxford.devices[name].dof_names,
                             map(float, np.asarray(oxford.devices[name].get_joint_positions()).reshape(-1))))
                restored = all(np.array_equal(a, b) for a, b in
                               zip(old_gains, oxford.devices[name].get_gains()))
                geometric = (q['button_joint'] < -.0045 if active else abs(q['button_joint']) < .0005)
                geometric = geometric and abs(q['button_twist_joint']) < .04
                check(title, good and geometric and oxford.states[name].active == active and restored,
                      positions=q, activation_state=bool(oxford.states[name].active),
                      gains_restored=restored, result=result)
    except Exception as error:
        check('验收程序完整执行', False, reason=str(error))
    finally:
        station.stop()
        # 使用已有驱动恢复机构，禁止 reset、重载或给机器人写姿态。
        if probe is not None:
            for name in ('Btn04', 'Btn05', 'Btn06', 'Btn07', 'Lock_Btn', 'Knob01', 'Knob02'):
                if name not in probe.controls:
                    continue
                try:
                    info = probe.controls[name]
                    probe.stage.GetPrimAtPath(info['joint']).GetAttribute(
                        f"drive:{info['kind']}:physics:targetPosition").Set(0.)
                except Exception as error:
                    report['cleanup'].append({'device': name, 'pass': False, 'error': str(error)})
            for name, title in [('Btn01','柜顶指示灯1'), ('Btn02','柜顶指示灯2'), ('Btn03','柜顶指示灯3')]:
                if name in probe.controls:
                    try:
                        station.ui_callbacks[title + '熄灭']()
                    except Exception as error:
                        report['cleanup'].append({'device': name, 'pass': False, 'error': str(error)})
        if oxford is not None:
            for title in ('隔离开关转至0°', '安全手柄转至0°', '急停：旋转解锁并回位'):
                if title in station.ui_callbacks:
                    good, _, result = run(title, record=False)
                    report['cleanup'].append({'action': title, 'pass': good, 'result': result})
        try:
            step(120)
            after = np.asarray(robot.get_joint_positions()).reshape(-1)
            arm_ids = [i for i, name in enumerate(robot.dof_names) if '_arm_' in name]
            check('人形物理视图完整', after.size == 50 and np.isfinite(after).all()
                  and np.isfinite(robot._physics_view.get_jacobians()).all(), dof_count=int(after.size),
                  max_arm_drift_rad=float(np.max(np.abs(after[arm_ids]-before[arm_ids]))))
        except Exception as error:
            check('人形物理视图完整', False, reason=str(error))
        for name in ('Btn04', 'Btn05', 'Btn06', 'Btn07', 'Lock_Btn', 'Knob01', 'Knob02'):
            if probe is not None and name in probe.controls:
                info = probe.controls[name]
                try:
                    target = probe.stage.GetPrimAtPath(info['joint']).GetAttribute(
                        f"drive:{info['kind']}:physics:targetPosition").Get()
                    report['cleanup'].append({'device': name, 'pass': abs(float(target)) < 1e-8,
                                              'drive_target': float(target)})
                except Exception as error:
                    report['cleanup'].append({'device': name, 'pass': False, 'error': str(error)})
        station._status = '其它机构验收结束，请查看验收报告'
        station.write()
        report.update(finished_unix=time.time(), pass_count=sum(v['pass'] for v in report['checks']),
                      check_count=len(report['checks']))
        report['pass'] = all(v['pass'] for v in report['checks']) and all(v['pass'] for v in report['cleanup'])
        save()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:18765')
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'outputs/l3_operable/live/legacy_plugin_checks.json')
    args = parser.parse_args()
    code = inspect.getsource(validate) + '\nvalidate(executor)\n'
    request = urllib.request.Request(args.url.rstrip('/') + '/python',
        data=json.dumps({'code': code}).encode(), headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=600) as response:
        print(response.read().decode())
    if args.output.is_file():
        result = json.loads(args.output.read_text())
        print(json.dumps({'pass': result['pass'], 'pass_count': result['pass_count'],
                          'check_count': result['check_count'], 'report': str(args.output)}, ensure_ascii=False))
        if not result['pass']:
            raise SystemExit(1)


if __name__ == '__main__':
    main()
