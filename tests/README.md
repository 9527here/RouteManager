# 离线回归

本目录17个unittest模块不启动Kit，也不驱动现场。运行时需要NumPy/SciPy等本机辅助依赖。

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH /path/to/python \
  -m unittest discover -s tests -p 'test_*.py'
```

`test_valve_ik`、`test_valve_plan`的模型/现场输入来自本机URDF与历史snapshot，不随公开源码分发。
缺少这些输入时明确报告skip；核心mock与数学回归继续运行。严格验收应准备对应输入并禁止跳过：

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH ROUTE_MANAGER_REQUIRE_MODEL_TESTS=1 \
  /path/to/python -m unittest discover -s tests -p 'test_*.py'
```

本机完整输入下192项通过。需要真实Kit/HTTP/数据文件的验收入口仍在`scripts/validate_*.py`，不加入离线发现。
离线通过不等于接触、运动、采集数据或真机验收通过。
