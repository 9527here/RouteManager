"""
standalone_server.py — 单文件 Isaac Sim HTTP 任务服务器模板
============================================================

把 simManager 的 HTTP 框架层（task_state + routes + serializer + 主循环）
合并成一个自包含文件，供其他 Isaac Sim 项目直接复用。

使用方法
--------
1. 把本文件拷贝到新项目目录。
2. 修改文件底部的 ``main()``：先启动 SimulationApp（Isaac 硬性约束：
   之后才能 import 任何 isaacsim/omni/pxr 模块），再构造你自己的执行器。
3. ``python standalone_server.py`` 启动。

执行器（executor）约定接口
--------------------------
执行器就是你项目里替代 simManager WorldManager 的对象，必须提供：

    step()            驱动仿真一帧
    simulation_app    SimulationApp 实例（用于 is_running() 判断）
    close()           关闭时释放资源
    <action>(...)     任意与 action 同名的方法，HTTP 层靠反射调用

不需要实现 ``exec_code`` —— 服务器内置了 /python 端点，在主线程
``exec()`` 任意代码（sandbox 里可通过 ``executor`` 访问你的执行器）。

HTTP 协议
---------
    POST /execute  {"action": "...", "parameters": {...}}
                   -> {"status": "success", "message": ..., "data": ...}
    POST /python   {"code": "executor.load_scene('/path/usd')"}
                   -> 同上
    GET  /health   -> {"status": "...", "task": "..."}

默认参数
--------
``CONFIG_DEFAULTS`` 字典按 ``action.replace("load_", "")`` 作 key 提供每个
action 的默认参数（前端只传需覆盖的参数）。也可用 ``load_config_file()``
从外部 config.json 加载（与 simManager 同格式）。

自测
----
不带 Isaac 也能验证 HTTP 层：``python standalone_server.py --demo``
会启动一个假执行器，可用 curl 打 /execute、/python、/health 验证。

顶层不 import 任何 isaacsim/omni/pxr —— 所有 Isaac 相关导入都在
函数内延迟执行，保证初始化顺序正确（SimulationApp 必须最先创建）。
依赖仅：fastapi、uvicorn（Isaac 环境自带，或 pip install fastapi uvicorn）。
"""

import logging
import os
import threading
import time
import traceback
from enum import Enum

import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

logger = logging.getLogger("isaac_server")

# ============================================================
# 一、任务状态机（源自 core/task_state.py）
# ============================================================


class TaskState(Enum):
    IDLE = "IDLE"
    PENDING = "PENDING"
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"


class StateManager:
    """线程安全的任务队列/状态机：HTTP 线程提交，仿真主线程消费。"""

    def __init__(self):
        self.state = TaskState.IDLE
        self.task_name = None
        self.task_kwargs = None
        self.error_msg = ""
        self.result_data = None
        self.lock = threading.Lock()

    def submit_task(self, task_name, **kwargs):
        with self.lock:
            if self.state in (TaskState.PENDING, TaskState.RUNNING):
                logger.warning(f"系统正忙，当前任务: {self.task_name}，拒绝新任务: {task_name}")
                return False, f"系统正忙，正在处理任务: {self.task_name}"
            self.task_name = task_name
            self.task_kwargs = kwargs
            self.error_msg = ""
            self.result_data = None
            self.state = TaskState.PENDING
            logger.info(f"任务已提交: {task_name}，参数: {kwargs}")
            return True, "任务提交成功"

    def get_status(self):
        with self.lock:
            return self.state, self.error_msg

    def set_running(self):
        with self.lock:
            self.state = TaskState.RUNNING

    def set_success(self):
        with self.lock:
            self.state = TaskState.SUCCESS

    def set_failed(self, error: str):
        with self.lock:
            self.state = TaskState.FAILED
            self.error_msg = error


# ============================================================
# 二、结果序列化器（源自 api/serializer.py 的通用骨架）
# ============================================================

_serializer_registry = {}


def register_serializer(task_name):
    """装饰器：为某个 action 注册专用序列化函数。

    用法示例（图像结果转 base64）::

        @register_serializer("capture_image")
        def _serialize_capture_image(rgb):
            import base64, cv2
            _, buffer = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            return {"image_base64": base64.b64encode(buffer).decode('utf-8')}
    """
    def decorator(fn):
        _serializer_registry[task_name] = fn
        return fn
    return decorator


def _to_jsonable(obj):
    """递归把 numpy 标量/数组等转成 JSON 可序列化类型。"""
    try:
        import numpy as np
    except ImportError:
        np = None

    if np is not None:
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, np.bool_):
            return bool(obj)
    if isinstance(obj, dict):
        return {k: _to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_jsonable(v) for v in obj]
    return obj


def serialize(task_name, raw_result):
    """优先使用注册的专用序列化器，否则做通用 JSON 兜底转换。"""
    if task_name in _serializer_registry:
        return _serializer_registry[task_name](raw_result)
    if raw_result is None:
        return None
    return _to_jsonable(raw_result)


# ============================================================
# 三、HTTP 路由（源自 api/routes.py）
# ============================================================

# action -> 默认参数表；按 action.replace("load_", "") 作 key 查找
CONFIG_DEFAULTS = {}


def load_config_file(path):
    """从外部 config.json 加载默认参数表（与 simManager 同格式）。"""
    import json
    with open(path, "r", encoding="utf-8") as f:
        CONFIG_DEFAULTS.update(json.load(f))
    logger.info(f"已加载默认参数配置: {path}，共 {len(CONFIG_DEFAULTS)} 项")


# 运行时由 IsaacTaskServer 注入
_executor = None
_state_mgr = None


class ActionRequest(BaseModel):
    action: str
    parameters: dict = {}


class PythonRequest(BaseModel):
    code: str


def _merge_default_params(action, parameters):
    """合并默认参数（前端传入的具有最高优先级）。"""
    key = action.replace("load_", "")
    default_params = CONFIG_DEFAULTS.get(key, {})
    if isinstance(default_params, dict):
        return {**default_params, **parameters}
    return parameters


def build_app(task_timeout=7200.0):
    """构造 FastAPI 应用并注册路由。"""
    app = FastAPI(title="Isaac Sim Task Server")

    async def wait_for_task_completion():
        """HTTP 线程挂起等待仿真主线程执行任务，并返回结果。"""
        start_time = time.time()
        while time.time() - start_time < task_timeout:
            state, error_msg = _state_mgr.get_status()

            if state == TaskState.SUCCESS:
                with _state_mgr.lock:
                    result_data = _state_mgr.result_data
                    _state_mgr.result_data = None
                    _state_mgr.state = TaskState.IDLE
                resp = {"status": "success", "message": "任务执行成功"}
                if result_data is not None:
                    resp["data"] = result_data
                return resp

            elif state == TaskState.FAILED:
                with _state_mgr.lock:
                    _state_mgr.state = TaskState.IDLE
                logger.error(f"任务执行失败: {error_msg}")
                raise HTTPException(status_code=500, detail=f"任务执行失败: {error_msg}")

            import asyncio
            await asyncio.sleep(0.1)

        with _state_mgr.lock:
            _state_mgr.state = TaskState.IDLE
        logger.warning("任务等待超时")
        raise HTTPException(status_code=408, detail="任务执行超时")

    @app.get("/health")
    async def health():
        state, error_msg = _state_mgr.get_status()
        data = {"status": state.value, "task": _state_mgr.task_name}
        if error_msg:
            data["error"] = error_msg
        return data

    @app.post("/python")
    async def execute_python(req: PythonRequest):
        """在仿真主线程中执行任意 Python 代码（调试/在线修复用）。"""
        logger.info(f"收到 Python 执行请求: {req.code[:80]}...")
        success, msg = _state_mgr.submit_task("exec_code", code=req.code)
        if not success:
            raise HTTPException(status_code=409, detail=msg)
        return await wait_for_task_completion()

    @app.post("/execute")
    async def execute_action(req: ActionRequest):
        """通用执行接口：动态分发到执行器同名方法，自动合并默认参数。"""
        final_params = _merge_default_params(req.action, req.parameters)
        logger.info(f"收到执行请求: action={req.action}, 合并后参数: {final_params}")

        success, msg = _state_mgr.submit_task(req.action, **final_params)
        if not success:
            raise HTTPException(status_code=409, detail=msg)
        return await wait_for_task_completion()

    return app


# ============================================================
# 四、服务器主类（封装 server.py 的双线程主循环）
# ============================================================

# 默认需要等待 stage 资产加载完成的 action（可用构造参数覆盖）
DEFAULT_LOADING_TASKS = {"load_scene", "load_car", "load_camera", "move_camera", "load_grid_map"}


class IsaacTaskServer:
    """双线程 Isaac Sim 任务服务器。

    Parameters
    ----------
    executor : object
        你的执行器对象（替代 WorldManager），必须有 step()、
        simulation_app、close()，其余 action 靠反射调用同名方法。
    host, port : HTTP 监听地址，默认 127.0.0.1:8765。
    task_timeout : 单个任务等待超时秒数。
    loading_tasks : 需要等待 stage 加载完成的 action 名集合。
    run_duration : executor 没有 simulation_app 时的最长运行秒数
        （None = 一直运行，仅用于 --demo 自测）。
    """

    def __init__(self, executor, host="127.0.0.1", port=8765,
                 task_timeout=7200.0, loading_tasks=None, run_duration=None):
        global _executor, _state_mgr
        self.executor = executor
        self.host = host
        self.port = port
        self.run_duration = run_duration
        self.loading_tasks = loading_tasks if loading_tasks is not None else DEFAULT_LOADING_TASKS

        _executor = executor
        _state_mgr = StateManager()
        self.state_mgr = _state_mgr
        self.app = build_app(task_timeout=task_timeout)

    # ---- 对外入口 ----

    def start(self):
        """启动 HTTP 线程并阻塞运行仿真主循环（等价于原 server.py 的 main）。"""
        logger.info(f"启动 HTTP 服务线程 (uvicorn @ {self.host}:{self.port})...")
        server_thread = threading.Thread(target=self._run_uvicorn, daemon=True)
        server_thread.start()

        logger.info("启动仿真主循环...")
        # 执行器可选提供 start() 钩子（WorldManager 有）
        start_hook = getattr(self.executor, "start", None)
        if callable(start_hook):
            start_hook()

        self._run_loop()

    # ---- 内部实现 ----

    def _run_uvicorn(self):
        config = uvicorn.Config(self.app, host=self.host, port=self.port,
                                log_level="info", loop="asyncio")
        uvicorn.Server(config).run()

    def _is_running(self):
        """仿真是否继续运行：有 simulation_app 用 is_running()，demo 模式用时间限制。"""
        sim_app = getattr(self.executor, "simulation_app", None)
        if sim_app is not None:
            return sim_app.is_running()
        if self.run_duration is None:
            return True
        return time.time() - self._start_time < self.run_duration

    def _run_dispatch(self, task_name, task_kwargs):
        """执行单个任务并写入结果（exec_code 内置，其余反射到执行器）。"""
        if task_name == "exec_code":
            # 内置 /python 支持：在主线程 exec 任意代码，sandbox 可访问 executor
            sandbox = {"executor": self.executor}
            exec(task_kwargs["code"], sandbox)
            result = None
        else:
            method = getattr(self.executor, task_name, None)
            if not method or not callable(method):
                raise ValueError(f"动作 '{task_name}' 不存在或不可调用")
            result = method(**task_kwargs)

        with self.state_mgr.lock:
            serialized = serialize(task_name, result)
            if serialized is not None:
                self.state_mgr.result_data = serialized
            elif result is not None:
                self.state_mgr.result_data = result

    def _wait_stage_loading(self):
        """等待 USD 资产加载与渲染收敛（需要 Isaac，延迟导入）。"""
        from isaacsim.core.utils.stage import is_stage_loading
        import omni.usd

        for _ in range(5):
            self.executor.step()
        loading_status = omni.usd.get_context().get_stage_loading_status()
        while is_stage_loading() or (loading_status is not None and loading_status[2] > 0):
            loading_status = omni.usd.get_context().get_stage_loading_status()
            self.executor.step()
            time.sleep(0.01)
        for _ in range(20):
            self.executor.step()
        logger.info("资产加载完成")

    def _run_loop(self):
        """主循环：轮询状态机，PENDING 则在主线程执行任务。"""
        self._start_time = time.time()
        try:
            while self._is_running():
                state, _ = self.state_mgr.get_status()
                if state == TaskState.PENDING:
                    self.state_mgr.set_running()

                    with self.state_mgr.lock:
                        task_name = self.state_mgr.task_name
                        task_kwargs = self.state_mgr.task_kwargs

                    logger.info(f"主线程开始执行任务: {task_name}，参数: {task_kwargs}")
                    try:
                        self._run_dispatch(task_name, task_kwargs)

                        if task_name in self.loading_tasks:
                            logger.info(f"任务 {task_name} 需要等待资产加载完成...")
                            self._wait_stage_loading()

                        self.state_mgr.set_success()
                        logger.info(f"任务 {task_name} 执行成功")
                    except Exception as e:
                        logger.error(f"任务 {task_name} 执行失败: {e}")
                        logger.debug(f"异常堆栈:\n{traceback.format_exc()}")
                        self.state_mgr.set_failed(str(e))

                self.executor.step()

        except KeyboardInterrupt:
            logger.warning("收到键盘中断信号，正在关闭...")
        finally:
            close_hook = getattr(self.executor, "close", None)
            if callable(close_hook):
                close_hook()
            logger.info("仿真已安全关闭")


# ============================================================
# 五、main() —— 模板用户唯一需要修改的部分
# ============================================================

def main():
    # ========================================================
    # ==== 以下为新项目自行修改区 ====
    #
    # Isaac 硬性约束：必须最先启动 SimulationApp，之后才能 import
    # 任何 isaacsim / omni / pxr 模块。
    #
    # 示例（去掉注释按你项目改）::
    #
    #     from isaacsim import SimulationApp
    #     simulation_app = SimulationApp({"headless": os.environ.get("HEADLESS", "0") == "1"})
    #
    #     # —— SimulationApp 启动后才能 import 这些 ——
    #     from isaacsim.core.api.world import World
    #     ... 构造你自己的执行器 ...
    #
    #     server = IsaacTaskServer(executor)
    #     server.start()
    #
    # 最小假执行器示例（自测 HTTP 层用，见下方 --demo）::
    #
    #     class FakeExecutor:
    #         simulation_app = None
    #         def step(self): time.sleep(0.01)
    #         def close(self): pass
    #         def hello(self, name="world"):
    #             return {"msg": f"hello {name}"}
    # ========================================================
    raise NotImplementedError("请按文件顶部 docstring 修改 main() 后运行")


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    if "--demo" in __import__("sys").argv:
        # 自测模式：不起 Isaac，用假执行器验证 HTTP 层
        logging.getLogger("uvicorn.access").setLevel(logging.WARNING)

        class FakeExecutor:
            simulation_app = None

            def step(self):
                time.sleep(0.01)

            def close(self):
                pass

            def hello(self, name="world"):
                return {"msg": f"hello {name}"}

            def slow(self, seconds=2.0):
                time.sleep(seconds)
                return {"slept": seconds}

        CONFIG_DEFAULTS["hello"] = {"name": "demo-default"}

        import sys
        port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 8765
        server = IsaacTaskServer(FakeExecutor(), port=port, run_duration=600)
        server.start()
    else:
        main()
