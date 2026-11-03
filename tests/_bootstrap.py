"""离线回归共享导入路径；不会启动Kit、网络服务或仿真。"""
from pathlib import Path
import sys
import os
import unittest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'scripts'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


def require_model_fixture(path):
    """公开克隆可跳过未分发的模型/现场输入；本机严格验收用环境变量禁止跳过。"""
    path = Path(path)
    if path.is_file():
        return path
    message = '未提供本机模型/现场fixture：' + str(path)
    if os.environ.get('ROUTE_MANAGER_REQUIRE_MODEL_TESTS') == '1':
        raise FileNotFoundError(message)
    raise unittest.SkipTest(message)
