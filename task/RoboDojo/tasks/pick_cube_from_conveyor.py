"""Standalone DynamicBench task entry; use scripts/dynamicbench/run_mvp.py.

This task has its own settings/executor and is deliberately not an EvalEnv
benchmark task. The registry can resolve it; the legacy eval CLI is unsupported.
"""
from dynamicbench.envs.conveyor_cube_task import ConveyorCubeTask

class pick_cube_from_conveyor(ConveyorCubeTask):
    pass
