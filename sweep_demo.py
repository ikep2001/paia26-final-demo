#!/usr/bin/env python3
"""LLM-validated CENTER -> LEFT -> RIGHT -> CENTER, six-phase MoveIt sweep."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys

from final_demo import ollama_plan

EXPECTED_STOPS = ["CENTER", "LEFT", "RIGHT", "CENTER"]
PHASES = ["CENTER_UP", "CENTER_FOREARM_LEVEL", "LEFT", "RIGHT", "CENTER_RETURN", "HOME"]
PLAN_RE = re.compile(r"PAIA_SWEEP_PLAN_OK phase=([A-Z_]+) q1=([+-]?[\d.]+) q2=([+-]?[\d.]+) q3=([+-]?[\d.]+) q4=([+-]?[\d.]+) points=(\d+)")
POSE_RE = re.compile(r"PAIA_SWEEP_POSE phase=([A-Z_]+) q1=([+-]?[\d.]+) q2=([+-]?[\d.]+) q3=([+-]?[\d.]+) q4=([+-]?[\d.]+) max_error=([\d.]+)")


def parse_phases(output, pattern):
    entries = []
    for line in output.splitlines():
        match = pattern.search(line)
        if match:
            entries.append({"phase": match.group(1),
                            "joints_rad": [float(x) for x in match.groups()[1:5]],
                            **({"max_error_rad": float(match.group(6))} if pattern is POSE_RE
                               else {"trajectory_points": int(match.group(6))})})
    return entries


def check_phase_sequence(phase_records):
    names = [x["phase"] for x in phase_records]
    if names != PHASES:
        raise ValueError(f"Missing/out-of-order sweep phases: {names}")


def validate_sweep_plan(plan):
    if plan.get("accepted") is not True or plan.get("stops") != EXPECTED_STOPS:
        raise ValueError("LLM instruction did not produce the required CENTER/LEFT/RIGHT/CENTER order")
    return plan


def run_ros_launch(execute):
    mode = "execute" if execute else "plan"
    cmd = ["ros2", "launch", "paia26_sweep", "sweep.launch.py", f"mode:={mode}"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)
    try:
        output, _ = proc.communicate(timeout=240)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            output, _ = proc.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            output, _ = proc.communicate()
        raise RuntimeError("Sweep timed out; remaining motions aborted")
    print(output, end="", flush=True)
    if proc.returncode != 0:
        raise RuntimeError(f"ROS2 sweep exited unsuccessfully: {proc.returncode}")
    plans = parse_phases(output, PLAN_RE)
    check_phase_sequence(plans)
    if "PAIA_SWEEP_PREFLIGHT_SUCCESS phases=6" not in output:
        raise RuntimeError("Full trajectory preflight was not confirmed")
    if not execute:
        if "PAIA_SWEEP_PLAN_ONLY_DONE phases=6" not in output:
            raise RuntimeError("Planning-only completion marker absent")
        return plans, []
    poses = parse_phases(output, POSE_RE)
    check_phase_sequence(poses)
    if "PAIA_SWEEP_SUCCESS phases=6" not in output:
        raise RuntimeError("Live sweep completion marker absent")
    for item in poses:
        if not 0 <= item["max_error_rad"] <= 0.07:
            raise RuntimeError(f"Measured joint error too large: {item}")
    return plans, poses


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instruction", default="中央、左、右、中央の順に巡回して")
    parser.add_argument("--execute", action="store_true", help="Actually move the SIMULATED CRANE+ V2 arm")
    parser.add_argument("--ollama-url", default=os.getenv("PAIA_OLLAMA_URL", "http://127.0.0.1:11434"))
    parser.add_argument("--model", default=os.getenv("PAIA_OLLAMA_MODEL", "qwen3:4b"))
    parser.add_argument("--log", default="runs/sweep_v10.jsonl")
    args = parser.parse_args(argv)
    record = {"timestamp_utc": datetime.now(timezone.utc).isoformat(),
              "instruction": args.instruction, "model": args.model,
              "mode": "simulated_robot" if args.execute else "planning_only",
              "plan": None, "planned_phases": [], "measured_phases": [], "result": "not_started"}
    try:
        plan = ollama_plan(args.instruction, args.ollama_url, args.model)
        record["plan"] = plan
        print("LLM PLAN:", json.dumps(plan, ensure_ascii=False), flush=True)
        validate_sweep_plan(plan)  # Fail closed: model cannot change the demo trajectory.
        record["planned_phases"], record["measured_phases"] = run_ros_launch(args.execute)
        record["result"] = "success" if args.execute else "plan_only"
        if args.execute:
            print("=== PAIA26 SWEEP EXECUTION SUCCESS ===", flush=True)
        else:
            print("=== PAIA26 SWEEP PLAN CHECK PASS (NO MOTION) ===", flush=True)
        return 0
    except Exception as e:
        record["result"] = "error"
        record["error"] = str(e)
        print(f"ERROR: {e}", file=sys.stderr, flush=True)
        return 1
    finally:
        path = Path(args.log)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(record, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
