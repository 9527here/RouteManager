"""电力柜中文动作面板；机构逐帧执行，人形动作提交到现有主线程任务队列。"""
import json
import math
from pathlib import Path


class ChineseActionsMixin:
    def _ensure_actions(self):
        # 用户明确暂缓旧裸机模型的自碰撞验证；旧软件守卫不能继续锁死人形按钮。
        if not hasattr(self.executor, '_defer_self_collision'):
            self.executor._defer_self_collision = True
        guard = getattr(self.executor, '_collision_guard', None)
        if self.executor._defer_self_collision and guard is not None:
            guard.enabled = False
        for name, value in {'_action_job': None, '_robot_busy': False, '_robot_cancel': False,
                            '_legacy_job': None, '_legacy_widgets': [],
                            '_approach': None, '_pending_action': None,
                            '_http_error': None, '_status': '就绪，请选择资产和动作',
                            '_action_history': [], '_control_widgets': [], '_asset_widgets': []}.items():
            if not hasattr(self, name):
                setattr(self, name, value)
        for name, value in {'_collect_process': None, '_collect_log': None, '_collect_output': None,
                            '_collect_status': '未开始采集', '_collect_poll_at': 0.,
                            '_collect_widgets': []}.items():
            if not hasattr(self, name):
                setattr(self, name, value)

    def _get_approach(self):
        if self._approach is None:
            import sys
            import importlib.util
            name = 'asset_approach'
            if name not in sys.modules or not hasattr(sys.modules[name], 'AssetApproach'):
                path = Path(__file__).with_name(name + '.py')
                spec = importlib.util.spec_from_file_location(name, path)
                module = importlib.util.module_from_spec(spec)
                sys.modules[name] = module
                try:
                    spec.loader.exec_module(module)
                except Exception:
                    sys.modules.pop(name, None)
                    raise
            self._approach = sys.modules[name].AssetApproach(self)
        return self._approach

    def _asset_name(self, source, name):
        if source == 'l3':
            from l3_asset_labels import asset_label
            return asset_label(name, self.runtime.catalog.get(name))
        if source == 'oxford':
            return {'isolator_switch': '隔离旋转开关', 'safety_switch': '安全手柄开关',
                    'e_stop_button': '旋转复位急停按钮'}.get(name, name)
        return {'Lock_Btn': '门锁机构按钮', 'Knob01': '动力柜旋钮开关（Knob01）',
                'Knob02': '动力柜旋钮开关（Knob02）',
                'Btn01': '动力柜绿色指示灯（Btn01）', 'Btn02': '动力柜绿色指示灯（Btn02）',
                'Btn03': '动力柜红色指示灯（Btn03）'}.get(name, '动力柜操作按钮（' + name + '）')

    def _approach_busy(self):
        return self._pending_action is not None or bool(self._approach and self._approach.busy)

    def _navigate_before(self, source, name, method, parameters):
        """操作目标和底盘到位绑定；位移/朝向未验收前不下发任何机构动作。"""
        if getattr(self.executor, '_navigation_edit_lock', False):
            raise RuntimeError('正在修正直向侧移与手部净空，请稍候')
        if self._approach_busy():
            raise RuntimeError('底盘正在驶向操作位置，请等待到位或停止当前动作')
        approach = self._get_approach()
        if approach.at_target(name, source=source):
            return False
        state=approach.request(name, source=source)
        if state.get('phase') in ('failed','cancelled'):
            self._status=state.get('message','走位未开始，操作未执行')
            self._refresh_actions()
            raise RuntimeError(self._status)
        self._pending_action = {'source': source, 'device': name, 'method': method,
                                'parameters': parameters}
        self._status = '底盘先驶向：' + self._asset_name(source, name)
        self._refresh_actions()
        return True

    def _step_approach(self, dt):
        approach = self._approach
        if approach is None:
            return
        try:
            approach.step(dt)
            phase = approach.state.get('phase')
            pending = self._pending_action
            if approach.busy:
                self._approach_terminal_seen=None
                self._status = approach.state.get('message', '底盘正在前往操作位置')
            elif pending and phase == 'arrived':
                self._approach_terminal_seen=(id(approach),phase,approach.state.get('message'))
                # 清待办后才调用原入口；原入口仍会再次校验真实到位状态。
                self._pending_action = None
                getattr(self, pending['method'])(**pending['parameters'])
            elif phase in ('failed', 'cancelled', 'rolled_back'):
                key=(id(approach),phase,approach.state.get('message'))
                self._pending_action = None
                if pending or getattr(self,'_approach_terminal_seen',None)!=key:
                    self._status = approach.state.get('message', '底盘未到位，已取消操作')
                self._approach_terminal_seen=key
                if pending:
                    self._action_history.append({'source': pending['source'], 'device': pending['device'],
                        'pass': False, 'stage': 'approach', 'reason': self._status})
            elif phase=='arrived':
                key=(id(approach),phase,approach.state.get('message'))
                if getattr(self,'_approach_terminal_seen',None)!=key:
                    if not self._robot_busy and self._action_job is None and self._legacy_job is None:
                        self._status=approach.state.get('message','底盘已到位')
                    self._approach_terminal_seen=key
        except Exception as error:
            self._pending_action = None
            approach.cancel()
            self._status = '底盘未到位，操作未执行：' + str(error)

    def approach_selected(self):
        self._ensure_actions()
        if getattr(self.executor, '_navigation_edit_lock', False):
            self._status = '正在修正直向侧移与手部净空，请稍候'
            return
        if self.queue or self._robot_busy or self._legacy_job or self._approach_busy():
            self._status = '请先完成或停止当前动作'
            return
        try:
            approach = self._get_approach()
            state=approach.request(self.selected, source='l3')
            self._status = state.get('message','底盘先驶向：'+self._asset_name('l3',self.selected))
        except Exception as error:
            self._status = '底盘未启动：' + str(error)
        self._refresh_actions()

    def stop_base(self):
        self._ensure_actions()
        self._pending_action = None
        if self._approach is not None:
            self._approach.cancel()
        else:
            self.executor.set_base_velocity(0., 0.)
        self._status = '底盘已停止，尚未执行的操作已取消'
        self._refresh_actions()

    def translate_base(self,direction):
        self._ensure_actions()
        if (getattr(self.executor,'_navigation_edit_lock',False) or self.queue or self._robot_busy
                or self._legacy_job or self._approach_busy()):
            self._status='请先完成或停止当前动作'
            return
        try:
            self._pending_action=None
            state=self._get_approach().request_translation(direction,.10)
            self._status=state.get('message','先收臂并对舵，再保持车身朝向纯横移10厘米')
        except Exception as error:
            self._status='横移未开始：'+str(error)
        self._refresh_actions()

    def command(self, action, device=None, target_deg=None):
        self._ensure_actions()
        name = device or {'press': 'object_0_variant', 'press_down': 'object_0_variant',
            'release': 'object_0_variant', 'selector': 'object_3', 'selector_to': 'object_3',
            'plate_out': 'object_14', 'plate_in': 'object_14'}.get(action)
        release_hold = (action == 'release' and self._action_job and
                        self._action_job['action'] == 'press_down' and self._action_job['device'] == name)
        if self._robot_busy or self._legacy_job or self._approach_busy() or (self.queue and not release_hold):
            raise RuntimeError('正在执行动作，请等待完成或点击“停止当前动作”')
        expected = {'press': 'button', 'press_down': 'button', 'release': 'button',
                    'selector': 'selector', 'selector_to': 'selector', 'plate_out': 'plate', 'plate_in': 'plate'}
        if name not in self.runtime.devices or self.runtime.catalog[name]['kind'] != expected.get(action):
            raise ValueError('请选择与动作类型一致的资产')
        if action in ('selector', 'selector_to'):
            values = self.runtime.catalog[name]['detents_deg']
            if action == 'selector':
                current = self.runtime.states[name]['nearest_detent']
                target_deg = values[(current + 1) % len(values)]
            if target_deg not in values:
                raise ValueError('所选旋钮没有这个档位')
        if self._navigate_before('l3', name, 'command',
                {'action': action, 'device': name, 'target_deg': target_deg}):
            return
        if release_hold:
            self._fixture_stop()
        title = {'press': '按压并复位', 'press_down': '按下并保持', 'release': '松开复位',
                 'selector': '切换下一档', 'selector_to': '切至指定档',
                 'plate_out': '压板投出', 'plate_in': '压板投入'}[action]
        if action in ('press', 'press_down', 'release'):
            self.queue = [(name, 'force', {'button_slide': -8.}, float('inf') if action == 'press_down' else .5)]
            if action == 'press':
                self.queue.append((name, 'force', {}, 1.))
            elif action == 'release':
                self.queue = [(name, 'force', {}, 1.)]
        elif action in ('selector', 'selector_to'):
            values = self.runtime.catalog[name]['detents_deg']
            if action == 'selector':
                current = self.runtime.states[name]['nearest_detent']
                target_deg = values[(current + 1) % len(values)]
            if target_deg not in values:
                raise ValueError('所选旋钮没有这个档位')
            title = f'切至 {target_deg:g}° 档位'
            self.queue = [(name, 'turn', {'selector_turn': math.radians(target_deg)}, 4.),
                          (name, 'force', {}, 1.)]
        else:
            goals = [('upper_turn', 4.4 * math.pi), ('lower_turn', 4.4 * math.pi), ('bridge_swing', math.pi/2)] if action == 'plate_out' else [('bridge_swing', 0.), ('upper_turn', 0.), ('lower_turn', 0.)]
            self.queue = []
            for joint, target in goals:
                self.queue.extend([(name, 'turn', {joint: target}, 8. if joint.endswith('_turn') else 4.),
                                   (name, 'force', {}, .6)])
        self.elapsed = 0.
        self._action_job = {'device': name, 'asset_name': self._asset_name('l3', name), 'action': action, 'title': title,
                            'target_deg': target_deg, 'seen_pressed': False, 'phases': len(self.queue)}
        self._status = f'正在执行：{title}'
        self._refresh_actions()

    def _ui_action(self, action, target_deg=None):
        try:
            self.command(action, self.selected, target_deg)
        except (RuntimeError, ValueError) as error:
            self._status = str(error)
        self._update_label()

    def step(self, dt):
        self._ensure_actions()
        self._poll_collection()
        self._step_approach(dt)
        if self._http_error:
            self._robot_busy = False
            self._status = self._http_error
            self._http_error = None
        job = self._action_job
        if job:
            job['seen_pressed'] |= bool(self.runtime.states.get(job['device'], {}).get('pressed', False))
        self._fixture_step(dt)
        self._legacy_step(dt)
        if job and self._action_job is job:
            state = self.runtime.states[job['device']]
            if self.queue:
                if job['action'] == 'press_down' and state['pressed']:
                    self._status = '已按下，指示灯亮；点击“松开复位”释放'
                else:
                    phase = job['phases'] - len(self.queue) + 1
                    self._status = f"正在执行：{job['title']}（第 {phase}/{job['phases']} 步）"
            else:
                action = job['action']
                if action in ('press', 'release'):
                    ok = not state['pressed'] and state['stroke_m'] < .0002 and (action != 'press' or job['seen_pressed'])
                elif action in ('selector', 'selector_to'):
                    ok = abs(math.degrees(state['angle_rad']) - job['target_deg']) < 3.
                elif action == 'plate_out':
                    ok = state['out'] and abs(math.degrees(state['angle_rad']) - 90.) < 3.
                else:
                    ok = state['connected'] and not state['out']
                self._action_history.append({**job, 'pass': bool(ok), 'feedback': dict(state)})
                self._status = ('完成：' if ok else '未到位，请检查：') + job['title']
                self._action_job = None
        # 控制/反馈保持240Hz，界面无需每个物理子步重排全部按钮。
        self._ui_refresh_elapsed = getattr(self, '_ui_refresh_elapsed', 0.)+dt
        if self._ui_refresh_elapsed >= .10:
            self._refresh_actions()
            self._ui_refresh_elapsed = 0.

    def stop(self):
        self._ensure_actions()
        self._pending_action = None
        if self._approach is not None:
            self._approach.cancel()
        else:
            self.executor.set_base_velocity(0., 0.)
        self._robot_cancel = bool(self._robot_busy)
        if self._robot_busy:
            # 即使正在回撤，也先把手臂目标停在反馈位置；下一步再退出动作循环。
            import numpy as np
            robot = self.executor.articulation
            names = [f'right_arm_{i}_joint' for i in range(1, 7)]
            q = np.asarray(robot.get_joint_positions()).reshape(-1)
            hold = [float(q[robot.dof_names.index(name)]) for name in names]
            self.executor.send_joint_command(hold, duration=0, joint_names=names)
        self._fixture_stop()
        self._finish_legacy('已停止其它机构动作', False)
        self._action_job = None
        self._status = '正在停止人形动作' if self._robot_busy else '已停止，机构外力已撤除'
        self._refresh_actions()

    def _refresh_actions(self):
        self._ensure_actions()
        busy = (bool(self.queue) or self._robot_busy or self._legacy_job is not None or self._approach_busy()
                or getattr(self.executor, '_navigation_edit_lock', False))
        if hasattr(self, '_asset_combo_model') and self.selected in self._asset_order:
            index = self._asset_order.index(self.selected)
            if self._asset_combo_model.get_item_value_model().get_value_as_int() != index:
                self._asset_combo_model.get_item_value_model().set_value(index)
        kind = self.runtime.catalog.get(getattr(self, 'selected', ''), {}).get('kind')
        for widget, expected, action, angle in self._control_widgets:
            compatible = expected == kind
            if action == 'selector_to':
                compatible &= angle in self.runtime.catalog[self.selected].get('detents_deg', [])
            release = action == 'release' and self._action_job and self._action_job['action'] == 'press_down'
            widget.enabled = bool(compatible and (not busy or release) and not self._robot_busy)
        for widget in self._asset_widgets:
            widget.enabled = not busy
        if hasattr(self, '_robot_button_widget'):
            self._robot_button_widget.enabled = not busy and hasattr(self.executor, '_cabinet_probe')
        if hasattr(self, '_approach_button_widget'):
            self._approach_button_widget.enabled = not busy
        if hasattr(self, '_base_stop_widget'):
            self._base_stop_widget.enabled = bool(self._approach and self._approach.busy)
        if getattr(self, '_approach_label', None):
            state = self._approach.state if self._approach is not None else {}
            text = state.get('message', '先选择资产；操作时底盘会先自动驶向对应工作位。')
            if state.get('operation_yaw_rad') is not None and state.get('facing_error_deg') is not None:
                text += f"\n胸部正对面板误差：{abs(state['facing_error_deg']):.1f}°"
            self._approach_label.text = text
        for widget, source, name in self._legacy_widgets:
            if source == 'cabinet':
                ready = name in getattr(getattr(self.executor, '_cabinet_probe', None), 'controls', {})
            else:
                ready = name in getattr(getattr(self.executor, '_oxford_devices', None), 'devices', {})
            widget.enabled = ready and not busy
        collecting = self._collect_process is not None and self._collect_process.poll() is None
        for widget in self._collect_widgets:
            widget.enabled = not collecting
        if hasattr(self, '_collect_stop_widget'):
            self._collect_stop_widget.enabled = collecting
        if getattr(self, '_collect_label', None):
            self._collect_label.text = self._collect_status
        self._update_label()

    def _update_label(self):
        if not getattr(self, 'label', None):
            return
        name = getattr(self, 'selected', 'object_0_variant')
        state = self.runtime.states.get(name)
        if not state:
            return
        kind = {'button': '按钮', 'selector': '旋钮', 'plate': '压板'}[state['kind']]
        if kind == '按钮':
            detail = f"行程 {max(0.,state['stroke_m'])*1000:.2f} 毫米，指示灯{'亮' if state['indicator_on'] else '灭'}"
        elif kind == '旋钮':
            detail = f"当前角度 {math.degrees(state['angle_rad']):.1f}°，{'已到位' if state['settled'] else '切换中'}"
        else:
            detail = f"上下旋松 {state['upper_turns']:.2f}/{state['lower_turns']:.2f} 圈，摆角 {math.degrees(state['angle_rad']):.1f}°\n状态：{'已投入并锁紧' if state['connected'] else ('已投出' if state['out'] else '未锁紧')}"
        self.label.text = f'当前选择：{self._asset_name("l3", name)}\n{detail}\n{getattr(self,"_status","就绪")}\n自碰撞验证暂缓，待带外壳的新机器人模型。'

    def start_collection(self):
        """独立进程拥有自己的 Isaac Lab 场景，避免采集重置桌面上的机器人。"""
        import datetime
        import os
        import subprocess
        self._ensure_actions()
        if self._collect_process is not None and self._collect_process.poll() is None:
            self._collect_status = '已有采集任务正在运行'
            return
        try:
            envs = self._collect_env_model.get_value_as_int()
            episodes = self._collect_episodes_model.get_value_as_int()
            if not 1 <= envs <= 64 or not 1 <= episodes <= 10000:
                raise ValueError('并行环境数需为 1～64，每动作回合数需为 1～10000')
            kind = ('all', 'button', 'selector', 'plate')[
                self._collect_task_model.get_item_value_model().get_value_as_int()]
            project = Path(__file__).resolve().parent.parent
            launcher = project / 'scripts/collect_l3_isaaclab.sh'
            if not launcher.is_file():
                raise RuntimeError('未找到批量采集入口，请检查项目文件')
            base = Path(self._collect_output_model.get_value_as_string()).expanduser()
            if not base.is_absolute():
                base = project / base
            # 每次创建新目录，防止再次点击覆盖已采集的数据。
            output = base / datetime.datetime.now().strftime('采集_%Y%m%d_%H%M%S_%f')
            output.mkdir(parents=True, exist_ok=False)
            command = ['bash', str(launcher), '--num-envs', str(envs), '--episodes', str(episodes),
                       '--tasks', kind, '--output', str(output)]
            if self._collect_rgb_model.get_value_as_bool():
                command.append('--rgb')
            environment = os.environ.copy()
            for key in ('DISPLAY', 'PYTHONPATH', 'LD_LIBRARY_PATH', 'VIRTUAL_ENV'):
                environment.pop(key, None)
            log = (output / 'launcher.log').open('w')
            try:
                process = subprocess.Popen(command, cwd=str(project), env=environment,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            except Exception:
                log.close()
                raise
            self._collect_process, self._collect_log = process, log
            self._collect_output = output
            self._collect_poll_at = 0.
            self._collect_status = f'采集进程启动中，请等待 Isaac Lab 就绪\n输出：{output}'
        except Exception as error:
            self._collect_status = '采集未启动：' + str(error)
        self._refresh_actions()

    def stop_collection(self):
        """由采集循环检查停止文件并关闭数据文件，避免强杀留下半个回合。"""
        self._ensure_actions()
        if self._collect_process is None or self._collect_process.poll() is not None:
            self._collect_status = '当前没有正在运行的采集任务'
        else:
            (self._collect_output / 'STOP').write_text('用户请求停止采集\n')
            self._collect_status = '已请求停止采集，等待保存当前数据并退出'
        self._refresh_actions()

    def _poll_collection(self):
        import time
        self._ensure_actions()
        now = time.monotonic()
        if self._collect_process is None or now < self._collect_poll_at:
            return
        self._collect_poll_at = now + .5
        path = self._collect_output / 'progress.json'
        progress = {}
        if path.is_file():
            try:
                progress = json.loads(path.read_text())
            except (OSError, ValueError):
                # 兼容旧采集程序非原子写入时的短暂不完整内容。
                pass
        status = progress.get('status')
        title = {'running': '采集中', 'completed': '采集完成', 'cancelled': '采集已停止',
                 'failed': '采集失败'}.get(status, '采集进程启动中')
        done = progress.get('completed_episodes', 0)
        total = progress.get('total_episodes', '?')
        if progress:
            self._collect_status = f'{title}：{done}/{total} 回合'
            if progress.get('message'):
                self._collect_status += '\n' + str(progress['message'])
            self._collect_status += '\n输出：' + str(self._collect_output)
        code = self._collect_process.poll()
        if code is not None:
            if code == 2 and status == 'completed':
                self._collect_status = (f"采集完成：{done}/{total} 回合，"
                    f"{progress.get('successful_episodes', 0)} 回合达标；未达标回合已保留并标记\n输出：{self._collect_output}")
            elif code != 0:
                self._collect_status = f'采集进程异常退出（代码 {code}），详情见采集目录中的 launcher.log\n输出：{self._collect_output}'
            elif status not in ('completed', 'cancelled', 'failed'):
                self._collect_status = f'采集进程已退出，请查看采集目录中的结果和 launcher.log\n输出：{self._collect_output}'
            if self._collect_log is not None:
                self._collect_log.close()
                self._collect_log = None
            self._collect_process = None

    def legacy_action(self, source, name, action, target=None):
        """复用已验证机构，逐物理步推进；不在界面回调里阻塞等待运动。"""
        import numpy as np
        self._ensure_actions()
        if self.queue or self._robot_busy or self._legacy_job or self._approach_busy():
            self._status = '请先完成或停止当前动作'
            return
        try:
            if source == 'cabinet':
                probe = getattr(self.executor, '_cabinet_probe', None)
                if probe is None or name not in probe.controls:
                    raise RuntimeError('当前场景未加载这个动力柜控件')
                if action == 'light':
                    if self._navigate_before(source, name, 'legacy_action',
                            {'source': source, 'name': name, 'action': action, 'target': target}):
                        return
                    self._cabinet_light(name, bool(target))
                    self._status = '完成：' + self._asset_name(source, name) + ('点亮' if target else '熄灭')
                    return
                info = probe.controls[name]
                goals = [-.005 if name == 'Lock_Btn' else -.006, 0.] if action == 'press' else [float(target)]
                # USD常以float32保存米制限位；5mm会读为4.999999888mm。
                # 只容忍数值舍入，实际下发目标仍裁到资产的真实限位内。
                if any(value < info['lower'] - 1e-8 or value > info['upper'] + 1e-8 for value in goals):
                    raise ValueError('目标超出资产已有的关节限位')
                goals = [float(np.clip(value, info['lower'], info['upper'])) for value in goals]
                if self._navigate_before(source, name, 'legacy_action',
                        {'source': source, 'name': name, 'action': action, 'target': target}):
                    return
                phases = [({'target': value}, 2.) for value in goals]
                job = {'source': source, 'device': name, 'phases': phases, 'title': self._asset_name(source, name)}
            else:
                runtime = getattr(self.executor, '_oxford_devices', None)
                if runtime is None or name not in runtime.devices:
                    raise RuntimeError('当前场景未加载这个迁移机构')
                demo = getattr(self.executor, '_operation_demo', None)
                view = runtime.devices[name]
                if action == 'press':
                    phases = [({'button_joint': -.005, 'button_twist_joint': 0.}, 2.)]
                elif action == 'unlock':
                    phases = [({'button_joint': -.005, 'button_twist_joint': -1.5}, 4.),
                              ({'button_joint': 0., 'button_twist_joint': -1.5}, 2.),
                              ({'button_joint': 0., 'button_twist_joint': 0.}, 3.)]
                else:
                    phases = [({'switch_joint': math.radians(target)}, 4.)]
                if self._navigate_before(source, name, 'legacy_action',
                        {'source': source, 'name': name, 'action': action, 'target': target}):
                    return
                if demo:
                    demo.stop()
                # 沿用已验收演示的驱动值，动作结束/停止时恢复该机构原有驱动。
                gains = tuple(np.asarray(v).copy() for v in view.get_gains())
                p = [500. if j == 'button_joint' else 5. for j in view.dof_names]
                d = [10. if j == 'button_joint' else .5 for j in view.dof_names]
                view.set_gains(kps=np.array([p]), kds=np.array([d]))
                job = {'source': source, 'device': name, 'phases': phases,
                       'title': self._asset_name(source, name),
                       'saved_gains': gains, 'settle_timeout': 2.}
            self._legacy_job = dict(job, index=0, elapsed=0., initial=None)
            self._status = '正在执行：' + job['title']
        except Exception as error:
            self._status = '动作未启动：' + str(error)
        self._refresh_actions()

    def _cabinet_light(self, name, on):
        from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
        probe = self.executor._cabinet_probe
        saved = getattr(probe, '_lamp_bindings', {})
        material = UsdShade.Material.Define(probe.stage, f'/World/Cabinet/TestMaterials/{name}_On')
        shader = UsdShade.Shader.Define(probe.stage, str(material.GetPath()) + '/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        color = (1., 0., 0.) if name == 'Btn03' else (0., 1., 0.)
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color) * .2)
        shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color) * 3.)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        for prim in Usd.PrimRange(probe.stage.GetPrimAtPath(probe.controls[name]['body1'])):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            binding = UsdShade.MaterialBindingAPI.Apply(prim)
            key = str(prim.GetPath())
            if key not in saved:
                original, _ = binding.ComputeBoundMaterial()
                saved[key] = str(original.GetPath()) if original else None
            if on:
                binding.Bind(material)
            elif saved[key]:
                binding.Bind(UsdShade.Material.Get(probe.stage, saved[key]))
            else:
                binding.UnbindDirectBinding()
        probe._lamp_bindings = saved
        probe._lamp_states = getattr(probe, '_lamp_states', {})
        probe._lamp_states[name] = on

    def _cabinet_feedback(self, info):
        import numpy as np
        from pxr import Gf
        probe = self.executor._cabinet_probe
        p0, q0 = probe._pose(info['body0'])
        p1, q1 = probe._pose(info['body1'])
        joint = probe.stage.GetPrimAtPath(info['joint'])
        local = lambda name: joint.GetAttribute('physics:' + name).Get()
        # 关节锚点包含模型初始偏置；直接减刚体位置会把初始偏置误算成行程。
        a0 = q0 * Gf.Quatd(local('localRot0'))
        a1 = q1 * Gf.Quatd(local('localRot1'))
        if info['kind'] == 'linear':
            x0 = p0 + np.asarray(Gf.Rotation(q0).TransformDir(Gf.Vec3d(local('localPos0'))))
            x1 = p1 + np.asarray(Gf.Rotation(q1).TransformDir(Gf.Vec3d(local('localPos1'))))
            return float(Gf.Rotation(a0).GetInverse().TransformDir(Gf.Vec3d(*(x1-x0)))[0])
        delta = a0.GetInverse() * a1
        return (math.degrees(2 * math.atan2(float(delta.GetImaginary()[0]), float(delta.GetReal()))) + 180.) % 360. - 180.

    def _finish_legacy(self, message, passed):
        job = self._legacy_job
        if not job:
            return
        try:
            if job['source'] == 'oxford':
                view = self.executor._oxford_devices.devices[job['device']]
                p, d = job['saved_gains']
                view.set_gains(kps=p, kds=d)
            else:
                probe = self.executor._cabinet_probe
                info = probe.controls[job['device']]
                value = self._cabinet_feedback(info)
                probe.stage.GetPrimAtPath(info['joint']).GetAttribute(
                    f"drive:{info['kind']}:physics:targetPosition").Set(value)
        except Exception as error:
            message += '；机构恢复反馈异常：' + str(error)
            passed = False
        self._action_history.append({'source': job['source'], 'device': job['device'],
                                     'title': job['title'], 'pass': bool(passed), 'message': message,
                                     'feedback': job.get('feedback')})
        self._legacy_job = None
        self._status = message

    def _legacy_step(self, dt):
        import numpy as np
        job = self._legacy_job
        if not job:
            return
        try:
            target, duration = job['phases'][job['index']]
            job['elapsed'] += dt
            if job['source'] == 'cabinet':
                probe = self.executor._cabinet_probe
                info = probe.controls[job['device']]
                probe.stage.GetPrimAtPath(info['joint']).GetAttribute(
                    f"drive:{info['kind']}:physics:targetPosition").Set(target['target'])
                value = self._cabinet_feedback(info)
                good = abs(value-target['target']) < (.0005 if info['kind'] == 'linear' else 2.)
                job['feedback'] = {'actual': value, 'target': target['target']}
            else:
                runtime = self.executor._oxford_devices
                view = runtime.devices[job['device']]
                q = np.asarray(view.get_joint_positions()).reshape(-1)
                if job['initial'] is None:
                    job['initial'] = dict(zip(view.dof_names, map(float, q)))
                t = min(job['elapsed'] / (duration * .8), 1.)
                smooth = t*t*(3.-2.*t)
                command = [job['initial'][j] + smooth * (target[j] - job['initial'][j]) for j in view.dof_names]
                view.set_joint_position_targets(np.asarray([command], dtype=np.float32))
                good = all(abs(q[i]-target[j]) < (.0005 if j == 'button_joint' else .04)
                           for i, j in enumerate(view.dof_names))
                job['feedback'] = {'positions': dict(zip(view.dof_names, map(float, q))),
                                   'activation_state': runtime.states[job['device']].active}
                if job['index'] + 1 == len(job['phases']):
                    expected = (target['button_joint'] < -.0045 if job['device'] == 'e_stop_button'
                                else abs(target['switch_joint']) > 1.)
                    good = good and runtime.states[job['device']].active == expected
            if job['elapsed'] >= duration:
                if not good:
                    # 原完整演示在急停旋转回零后还保持2秒。拆分成单项操作后，
                    # 仍需等真实反馈收敛；只延长有界等待，不放宽角度或行程判据。
                    if job['elapsed'] >= duration + job.get('settle_timeout', 0.):
                        self._finish_legacy('机构未到位，已停止：' + job['title'], False)
                elif job['index'] + 1 == len(job['phases']):
                    self._finish_legacy('完成：' + job['title'], True)
                else:
                    job.update(index=job['index']+1, elapsed=0., initial=None)
        except Exception as error:
            self._finish_legacy('机构动作异常：' + str(error), False)

    def select(self, name):
        self._ensure_actions()
        if name not in self.runtime.devices:
            raise ValueError('资产不存在：' + name)
        if self.queue or self._robot_busy or self._legacy_job or self._approach_busy():
            return
        self.selected = name
        self._refresh_actions()

    def start_robot(self, cycles=1):
        """UI回调只提交HTTP请求；工作线程不访问Isaac对象。"""
        import threading
        import os
        self._ensure_actions()
        if self.queue or self._robot_busy or self._legacy_job or self._approach_busy():
            self._status = '请先完成或停止当前动作'
            return
        self.selected = 'object_0_variant'
        try:
            if self._navigate_before('l3', self.selected, 'start_robot', {'cycles': cycles}):
                return
        except Exception as error:
            self._status = '人形操作尚未开始：' + str(error)
            self._refresh_actions()
            return
        self._robot_busy, self._robot_cancel = True, False
        self._collision_stop = False
        self._status = '已提交：人形右手按压绿色按钮'
        self.overview()
        self._refresh_actions()
        url = 'http://127.0.0.1:' + os.environ.get('ROUTE_MANAGER_PORT', '18765') + '/python'
        payload = json.dumps({'code': f'executor._l3_station.run_robot(cycles={int(cycles)})'}).encode()
        def submit():
            import urllib.request
            import urllib.error
            import time
            for attempt in range(10):
                try:
                    req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
                    with urllib.request.urlopen(req, timeout=600) as response:
                        response.read()
                    return
                except urllib.error.HTTPError as error:
                    if error.code == 409 and attempt < 9:
                        time.sleep(.15)
                        continue
                    self._http_error = '人形动作提交失败，服务正忙或返回错误：' + str(error.code)
                    return
                except Exception as error:
                    self._http_error = '人形动作连接失败：' + str(error)
                    return
        threading.Thread(target=submit, daemon=True, name='l3-robot-request').start()

    def run_robot(self, cycles=1):
        import sys
        import types
        self._ensure_actions()
        try:
            if self._robot_cancel:
                raise InterruptedError('已取消')
            if not self._get_approach().at_target('object_0_variant', source='l3'):
                raise RuntimeError('底盘没有停稳在目标工作位，人形操作未执行')
            for name in ('cabinet_contact', 'l3_robot_contact'):
                path = Path(__file__).parent / (name + '.py')
                module = sys.modules.get(name) or types.ModuleType(name)
                module.__file__ = str(path)
                sys.modules[name] = module
                exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
            self.executor._l3_robot_contact = sys.modules['l3_robot_contact'].L3RobotContact(self.executor)
            result = self.executor._l3_robot_contact.button(cycles=cycles)
            self._status = '完成：人形按下亮灯、松开灭灯并回撤' if result['pass'] else '人形按压未达标，请检查反馈'
            (self.output / 'plugin_robot_result.json').write_text(json.dumps(result, indent=2))
        except InterruptedError:
            self._status = ('碰撞保护已停止，手臂保持当前位置' if getattr(self, '_collision_stop', False)
                            else '已停止人形动作，手臂保持当前位置')
        except Exception as error:
            self._status = '人形动作未完成：' + str(error)
        finally:
            guard = getattr(self.executor, '_collision_guard', None)
            if guard is not None:
                guard.contact_allowed = False
            self._robot_busy = False
            self._robot_cancel = False
            self._refresh_actions()
            self.write()

    def show_ui(self):
        import omni.ui as ui
        from l3_asset_labels import apply_scene_labels
        apply_scene_labels(self.runtime.stage, self.runtime.prefix, self.runtime.catalog)
        self._ensure_actions()
        if getattr(self, 'window', None):
            self.window.destroy()
        old_demo = getattr(self.executor, '_operation_demo', None)
        if old_demo and getattr(old_demo, 'window', None):
            old_demo.window.visible = False
        self.window = ui.Window('电力柜动作控制', width=660, height=900)
        self.window.frame.style = {'font_size': 18, 'Button': {'font_size': 17}}
        self._control_widgets, self._asset_widgets, self._collect_widgets, self._legacy_widgets = [], [], [], []
        self.ui_callbacks = {}
        with self.window.frame:
            self._actions_scroll = ui.ScrollingFrame()
            with self._actions_scroll:
                with ui.VStack(spacing=5, height=0):
                    ui.Label('电力柜动作控制 · 15 件资产', height=28)
                    ui.Label('参数为参考规格，待实测标定。', height=24)
                    ui.Label('底盘：四轮独立转向/驱动；参考舵角 ±90°（待实测）', height=24)
                    ui.Label('常规行驶上限 0.25 m/s，接近目标自动减速（仿真参考）', height=24)
                    ui.Label('选择操作资产（按实际机构名称）', height=24)
                    self._asset_order = list(self.runtime.catalog)
                    choices = [self._asset_name('l3', name)+'  [原 '+self.runtime.catalog[name]['source_file'].removeprefix('object_').removesuffix('.glb')+']'
                               for name in self._asset_order]
                    selector = ui.ComboBox(self._asset_order.index(self.selected), *choices, height=36)
                    self._asset_combo_model = selector.model
                    def choose_asset(model, item):
                        index = model.get_item_value_model().get_value_as_int()
                        if 0 <= index < len(self._asset_order) and self._asset_order[index] != self.selected:
                            self.select(self._asset_order[index])
                    self._asset_combo_callback = choose_asset
                    selector.model.add_item_changed_fn(choose_asset)
                    self._asset_widgets.append(selector)
                    self.label = ui.Label('', height=110, word_wrap=True)
                    ui.Label('先收纳双臂并正对柜门／面板，再保持朝向走位；停稳后操作', height=25)
                    ui.Label('受阻自动安全回退一次；点击“停止底盘”可取消', height=23)
                    with ui.HStack(height=34):
                        self._approach_button_widget = ui.Button('移动到所选资产前', clicked_fn=self.approach_selected)
                        self._base_stop_widget = ui.Button('停止底盘', clicked_fn=self.stop_base)
                    with ui.HStack(height=32):
                        left=ui.Button('机器人左移约10厘米',clicked_fn=lambda:self.translate_base('left'))
                        right=ui.Button('机器人右移约10厘米',clicked_fn=lambda:self.translate_base('right'))
                        self._asset_widgets.extend([left,right])
                    self.ui_callbacks['机器人左移10厘米']=lambda:self.translate_base('left')
                    self.ui_callbacks['机器人右移10厘米']=lambda:self.translate_base('right')
                    self._approach_label = ui.Label('', height=64, word_wrap=True)
                    self.ui_callbacks['移动到所选资产前'] = self.approach_selected
                    self.ui_callbacks['停止底盘'] = self.stop_base
                    ui.Label('机构直接操作：点击后先走位，再驱动机构', height=23)
                    rows = [[('按下并保持（亮灯）','button','press_down',None),('松开复位（灭灯）','button','release',None),('按压并复位','button','press',None)],
                            [('切至 -45°','selector','selector_to',-45.),('切至 0°','selector','selector_to',0.),('切至 45°','selector','selector_to',45.),('切至 90°','selector','selector_to',90.)],
                            [('投出（旋松后拨开）','plate','plate_out',None),('投入（拨回后旋紧）','plate','plate_in',None)]]
                    for row in rows:
                        with ui.HStack(height=34):
                            for title, kind, action, angle in row:
                                callback = lambda a=action,t=angle:self._ui_action(a,t)
                                self.ui_callbacks[title] = callback
                                button = ui.Button(title, clicked_fn=callback)
                                self._control_widgets.append((button,kind,action,angle))
                    ui.Label('人形机器人操作：' + self._asset_name('l3', 'object_0_variant'), height=24)
                    self._robot_button_widget = ui.Button('人形右手：按压绿色按钮并复位',height=36,clicked_fn=self.start_robot)
                    self.ui_callbacks['人形按压'] = self.start_robot
                    self.ui_callbacks['停止当前动作'] = self.stop
                    with ui.HStack(height=34):
                        ui.Button('停止当前动作',clicked_fn=self.stop)
                        ui.Button('查看全部资产',clicked_fn=self.all_view)
                        ui.Button('查看选中资产',clicked_fn=self.focus)
                        ui.Button('查看人形',clicked_fn=self.overview)
                    with ui.CollapsableFrame('动力柜与迁移机构：已验证的机构操作', collapsed=True):
                        with ui.VStack(spacing=4, height=0):
                            ui.Label('这些按钮直接驱动机构；未载入的控件会禁用。', height=26)
                            for name, title in [('Btn04','柜按钮4'), ('Btn05','柜按钮5'), ('Btn06','柜按钮6'),
                                                ('Btn07','柜按钮7'), ('Lock_Btn','锁按钮')]:
                                callback = lambda n=name:self.legacy_action('cabinet', n, 'press')
                                display = self._asset_name('cabinet', name) + '：按下并回位'
                                button = ui.Button(display, height=28, clicked_fn=callback)
                                self._legacy_widgets.append((button, 'cabinet', name))
                                self.ui_callbacks[title + '按压回位'] = callback
                                self.ui_callbacks[display] = callback
                            for name, title in [('Knob01','柜旋钮1'), ('Knob02','柜旋钮2')]:
                                with ui.HStack(height=28):
                                    ui.Label(self._asset_name('cabinet', name), width=300)
                                    for angle in (-45., 0., 45.):
                                        callback = lambda n=name,a=angle:self.legacy_action('cabinet', n, 'turn', a)
                                        button = ui.Button(f'{angle:g}°', clicked_fn=callback)
                                        self._legacy_widgets.append((button, 'cabinet', name))
                                        self.ui_callbacks[f'{title}转至{angle:g}°'] = callback
                            for name, title in [('Btn01','柜顶指示灯1'), ('Btn02','柜顶指示灯2'), ('Btn03','柜顶指示灯3')]:
                                with ui.HStack(height=28):
                                    ui.Label(self._asset_name('cabinet', name), width=300)
                                    for on, label in [(True,'点亮'), (False,'熄灭')]:
                                        callback = lambda n=name,v=on:self.legacy_action('cabinet', n, 'light', v)
                                        button = ui.Button(label, clicked_fn=callback)
                                        self._legacy_widgets.append((button, 'cabinet', name))
                                        self.ui_callbacks[title + label] = callback
                            for name, title, angles in [('isolator_switch','隔离开关',(0.,90.)),
                                                        ('safety_switch','安全手柄',(0.,-90.))]:
                                with ui.HStack(height=28):
                                    ui.Label(self._asset_name('oxford', name), width=200)
                                    for angle in angles:
                                        callback = lambda n=name,a=angle:self.legacy_action('oxford', n, 'turn', a)
                                        button = ui.Button(f'转至 {angle:g}°', clicked_fn=callback)
                                        self._legacy_widgets.append((button, 'oxford', name))
                                        self.ui_callbacks[f'{title}转至{angle:g}°'] = callback
                            with ui.HStack(height=30):
                                for action, title in [('press','急停：按下锁定'), ('unlock','急停：旋转解锁并回位')]:
                                    callback = lambda a=action:self.legacy_action('oxford', 'e_stop_button', a)
                                    button = ui.Button(title, clicked_fn=callback)
                                    self._legacy_widgets.append((button, 'oxford', 'e_stop_button'))
                                    self.ui_callbacks[title] = callback
                    ui.Separator(height=8)
                    ui.Label('Isaac Lab 批量采集', height=28)
                    ui.Label('独立环境采集机构操作数据；不等同于人形接触示范。', height=26, word_wrap=True)
                    with ui.HStack(height=28):
                        ui.Label('采集任务', width=110)
                        tasks = ui.ComboBox(0, '全部动作', '按钮按压与复位', '旋钮档位切换', '螺丝压板投退')
                        self._collect_task_model = tasks.model
                        self._collect_widgets.append(tasks)
                    with ui.HStack(height=28):
                        ui.Label('每动作回合数', width=110)
                        episodes = ui.IntField()
                        episodes.model.set_value(10)
                        self._collect_episodes_model = episodes.model
                        self._collect_widgets.append(episodes)
                        ui.Label('并行环境数', width=105)
                        envs = ui.IntField()
                        envs.model.set_value(4)
                        self._collect_env_model = envs.model
                        self._collect_widgets.append(envs)
                    with ui.HStack(height=28):
                        ui.Label('输出父目录', width=110)
                        output = ui.StringField()
                        output.model.set_value(str(Path(__file__).resolve().parent.parent / 'outputs/l3_isaaclab'))
                        self._collect_output_model = output.model
                        self._collect_widgets.append(output)
                    with ui.HStack(height=26):
                        rgb = ui.CheckBox(width=24)
                        self._collect_rgb_model = rgb.model
                        ui.Label('保存彩色图像（占用更多显存与磁盘）')
                        self._collect_widgets.append(rgb)
                    with ui.HStack(height=34):
                        start = ui.Button('开始批量采集', clicked_fn=self.start_collection)
                        self._collect_widgets.append(start)
                        self._collect_stop_widget = ui.Button('停止采集并保存', clicked_fn=self.stop_collection)
                    self._collect_label = ui.Label(self._collect_status, height=94, word_wrap=True)
                    self.ui_callbacks['开始批量采集'] = self.start_collection
                    self.ui_callbacks['停止采集并保存'] = self.stop_collection
        self._refresh_actions()
        panel = ui.Workspace.get_window('电力柜动作控制')
        viewport = ui.Workspace.get_window('Viewport')
        if panel and viewport:
            panel.dock_in(viewport, ui.DockPosition.RIGHT, .42)
