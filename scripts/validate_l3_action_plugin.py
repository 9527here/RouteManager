#!/usr/bin/env python3
"""在已加载的人形场景验证中文动作回调；使用现有 HTTP 主线程入口。"""
import argparse
import inspect
import json
import urllib.request


def validate(executor):
    import json, math
    import numpy as np
    s = executor._l3_station
    s.stop()
    report = {'method':'调用绑定到中文 UI 按钮的同一回调，检查实际物理反馈', 'checks':[]}
    def save():
        (s.output/'chinese_plugin_checks.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    def check(title, ok, **data):
        report['checks'].append({'name':title,'pass':bool(ok),**data})
        save()
        if not ok: raise RuntimeError('验证失败：'+title)
    def step(n):
        for i in range(n): executor.step(render=(i%4==0))
    def wait_ready(pressed_name=None):
        for i in range(40000):
            executor.step(render=(i%4==0))
            if not s._approach_busy() and (pressed_name is None or s.runtime.states[pressed_name]['pressed']):
                return
        raise RuntimeError('底盘走位或按压到位超时')
    def action(title, name):
        s.select(name)
        old=len(s._action_history)
        s.ui_callbacks[title]()
        for i in range(40000):
            executor.step(render=(i%4==0))
            if not s.queue and not s._approach_busy(): break
        row=s._action_history[-1] if len(s._action_history)>old else {}
        check(name+' '+title, bool(row.get('pass')) and not s.queue, result=row)
    try:
        for name,item in s.runtime.catalog.items():
            if item['kind']!='button':continue
            s.select(name)
            s.ui_callbacks['按下并保持（亮灯）']()
            wait_ready(name)
            state=dict(s.runtime.states[name])
            check(name+' 持续按压',state['pressed'] and state['indicator_on'] and bool(s.queue),feedback=state)
            check(name+' 运行中界面互锁',not any(w.enabled for w in s._asset_widgets) and not s._robot_button_widget.enabled and all(w.enabled == (a=='release') for w,k,a,t in s._control_widgets))
            try:s.command('selector_to','object_3',90.)
            except RuntimeError:rejected=True
            else:rejected=False
            check(name+' 拒绝重叠动作',rejected and s.queue[0][0]==name)
            action('松开复位（灭灯）',name)
            action('按压并复位',name)
        s.select('object_0_variant')
        s.ui_callbacks['按下并保持（亮灯）']();wait_ready('object_0_variant')
        s.ui_callbacks['停止当前动作']();step(100)
        check('停止撤力与弹簧回位',not s.queue and not s.runtime.states['object_0_variant']['pressed'])
        for name,item in s.runtime.catalog.items():
            if item['kind']!='selector':continue
            s.select(name)
            enabled={t:bool(w.enabled) for w,k,a,t in s._control_widgets if a=='selector_to'}
            check(name+' 可用档位',all(enabled[t] == (t in item['detents_deg']) for t in enabled),enabled=enabled)
            for target in item['detents_deg']:
                action(f'切至 {target:g}°',name)
            action('切至 0°',name)
        for name,item in s.runtime.catalog.items():
            if item['kind']!='plate':continue
            action('投出（旋松后拨开）',name)
            action('投入（拨回后旋紧）',name)
        robot=executor.articulation
        q=np.asarray(robot.get_joint_positions())
        check('人形物理视图完整',q.size==50 and np.isfinite(q).all() and np.isfinite(robot._physics_view.get_jacobians()).all(),dof_count=q.size)
        report['pass']=all(row['pass'] for row in report['checks'])
    finally:
        s.stop();s.select('object_0_variant');s.all_view();s.write();save()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:18765')
    args = parser.parse_args()
    code = inspect.getsource(validate) + '\nvalidate(executor)\n'
    request = urllib.request.Request(args.url.rstrip('/') + '/python',
        data=json.dumps({'code': code}).encode(), headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=600) as response:
        print(response.read().decode())


if __name__ == '__main__':
    main()
