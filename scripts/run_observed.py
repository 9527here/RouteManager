#!/usr/bin/env python3
"""记录子进程退出码/信号，不自动重启；不导入Isaac或下发动作。

python3 scripts/run_observed.py --output outputs/server_run -- bash scripts/run.sh
输出目录必须是新目录，防止覆盖上一次异常退出的证据。
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('需要在 -- 后指定启动命令')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    child = None
    with (output / 'lifecycle.jsonl').open('a', buffering=1) as events:
        def record(event, **fields):
            row = dict(event=event, time_utc=datetime.now(timezone.utc).isoformat(), **fields)
            events.write(json.dumps(row, ensure_ascii=False) + '\n')
            events.flush()
            os.fsync(events.fileno())

        def forward(signum, frame):
            record('forward_signal', signal=signum)
            if child is not None and child.poll() is None:
                try:
                    os.killpg(child.pid, signum)
                except ProcessLookupError:
                    pass

        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(signum, forward)
        env = os.environ.copy()
        # 尽早启用原生致命信号堆栈；SIGKILL本身不可捕获，由独立父进程记录退出信号。
        env.update(PYTHONFAULTHANDLER='1', PYTHONUNBUFFERED='1')
        record('supervisor_started', pid=os.getpid(), cwd=os.getcwd(), command=command,
               automatic_restart=False)
        with (output / 'process.log').open('ab', buffering=0) as log:
            try:
                child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                         stderr=subprocess.STDOUT, env=env, start_new_session=True)
            except Exception as error:
                record('launch_failed', error_type=type(error).__name__, error=str(error))
                raise
            record('child_started', pid=child.pid)
            returncode = child.wait()
            result = dict(returncode=returncode,
                          terminating_signal=-returncode if returncode < 0 else None,
                          signal_name=signal.Signals(-returncode).name if returncode < 0 else None,
                          automatic_restart=False)
            record('child_exited', pid=child.pid, **result)
            (output / 'exit.json').write_text(json.dumps(result, indent=2) + '\n')
    return returncode if returncode >= 0 else 128 - returncode


if __name__ == '__main__':
    sys.exit(main())
