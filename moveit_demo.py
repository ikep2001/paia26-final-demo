#!/usr/bin/env python3
"""PAIA26 final: Japanese language -> local LLM -> bounded MoveIt sequence.

AI chooses allowed station order only. ROS 2 MoveIt plans and executes the
predefined multi-joint motions, verifying each measured pose. Simulator only.
"""
import argparse
from datetime import datetime, timezone
import json
import os
import signal
from pathlib import Path
import re
import subprocess
import sys

from final_demo import ollama_plan

SUCCESS_RE = re.compile(r"^\[.*?\] PAIA_STATION_SUCCESS (LEFT|RIGHT|CENTER)$")
SUCCESS_BARE = re.compile(r"^PAIA_STATION_SUCCESS (LEFT|RIGHT|CENTER)$")
POSE_RE = re.compile(r"PAIA_POSE station=(LEFT|RIGHT|CENTER) phase=(extended|retracted) "
                     r"yaw=([+-]?[0-9.]+) q2=([+-]?[0-9.]+) q3=([+-]?[0-9.]+) "
                     r"q4=([+-]?[0-9.]+) max_error=([0-9.]+)")


def extract_pose_lines(output, station):
    result = []
    for line in output.splitlines():
        match = POSE_RE.search(line)
        if match:
            item = {"station": match.group(1), "phase": match.group(2),
                    "joints": [float(x) for x in match.groups()[2:6]],
                    "max_error_rad": float(match.group(7))}
            if item["station"] == station:
                result.append(item)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instruction", default="右、左、中央の順に巡回して")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--ollama-url", default=os.getenv("PAIA_OLLAMA_URL", "http://127.0.0.1:11434"))
    parser.add_argument("--model", default=os.getenv("PAIA_OLLAMA_MODEL", "qwen3:4b"))
    parser.add_argument("--log", default="runs/moveit_patrol.jsonl")
    args = parser.parse_args(argv)
    record = {"timestamp_utc": datetime.now(timezone.utc).isoformat(),
              "instruction": args.instruction, "model": args.model,
              "robot_executor": "MoveIt2/CRANE+ V2", "mode": "simulated_robot",
              "plan": None, "movements": [], "result": "not_started"}
    try:
        plan = ollama_plan(args.instruction, args.ollama_url, args.model)
        record["plan"] = plan
        print("AI PLAN:", json.dumps(plan, ensure_ascii=False), flush=True)
        if not plan["accepted"]:
            record["result"] = "rejected"
            print("PLAN REJECTED; NO MOVEMENT", flush=True)
            return 0
        if not args.execute:
            record["result"] = "plan_only"
            print("PLAN ONLY; NO MOVEMENT", flush=True)
            return 0
        for station in plan["stops"]:
            print(f"MOVEIT BEGIN: {station}", flush=True)
            cmd = ["ros2", "launch", "paia26_moveit", "station.launch.py", f"station:={station}"]
            # Each station is a distinct process, and failures abort all further movement.
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, start_new_session=True)
            try:
                output, _ = p.communicate(timeout=120)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                try:
                    output, _ = p.communicate(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    output, _ = p.communicate()
                raise RuntimeError(f"MoveIt station {station} timed out (120s); motion aborted")
            print(output, end="", flush=True)
            poses = extract_pose_lines(output, station)
            station_done = bool(SUCCESS_BARE.search(output.strip().splitlines()[-1])) if output.strip() else False
            # ros2 launch adds prefixes and a clean-exit line, so use an any-line match.
            station_done = station_done or any(
                SUCCESS_BARE.match(line.strip()) or SUCCESS_RE.match(line.strip())
                or f"PAIA_STATION_SUCCESS {station}" in line
                for line in output.splitlines())
            if p.returncode != 0 or not station_done or len(poses) != 2:
                raise RuntimeError(f"Station {station} unconfirmed: exit={p.returncode}, poses={len(poses)}, success={station_done}")
            record["movements"].append({"station": station, "poses": poses})
            print(f"MOVEIT PASS: {station}", flush=True)
        record["result"] = "success"
        print("=== AI + MOVEIT MULTI-JOINT DEMO SUCCESS ===", flush=True)
        return 0
    except (Exception,) as exc:
        record["result"] = "error"
        record["error"] = str(exc)
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        out = Path(args.log)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(record, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    sys.exit(main())
