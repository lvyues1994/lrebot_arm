#!/usr/bin/env python3
"""Runs the studio's scripted flow against a simulated runtime node.

Starts larm_runtime_node, runs larm_studio --script, then stops the runtime with SIGINT and requires
both to exit cleanly. Qt runs offscreen unless QT_QPA_PLATFORM is set: offscreen checks the whole flow
without a display, but its screenshots show a black viewport; QT_QPA_PLATFORM=xcb renders the scene.
"""

import argparse
import os
import signal
import subprocess
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime", required=True)
    parser.add_argument("--studio", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--screenshots")
    args = parser.parse_args()

    studio_env = dict(os.environ)
    studio_env.setdefault("QT_QPA_PLATFORM", "offscreen")
    runtime = subprocess.Popen(
        [args.runtime, "--ros-args", "-p", f"profile:={args.profile}", "-p", "real_time_factor:=2.0"],
        start_new_session=True,
    )
    studio_code = None
    try:
        time.sleep(3.0)
        if runtime.poll() is not None:
            print(f"the runtime exited early with {runtime.returncode}")
            return 1
        command = [args.studio, "--profile", args.profile, "--script"]
        if args.screenshots:
            command += ["--screenshots", args.screenshots]
        studio_code = subprocess.run(command, env=studio_env, timeout=240).returncode
        print(f"studio exited with {studio_code}")
    finally:
        runtime.send_signal(signal.SIGINT)
        try:
            runtime_code = runtime.wait(timeout=30)
        except subprocess.TimeoutExpired:
            runtime.kill()
            runtime_code = None
    print(f"runtime exited with {runtime_code}")
    return 0 if studio_code == 0 and runtime_code == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
