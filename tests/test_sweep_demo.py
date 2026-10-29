import unittest
from pathlib import Path
import sys
from unittest.mock import patch
import tempfile
import json
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sweep_demo

PHASES = sweep_demo.PHASES


def fake_output(execute=False):
    parts = [f"[node] PAIA_SWEEP_PLAN_OK phase={phase} q1=0.00000 q2=0.00000 q3=-1.57080 q4=0.00000 points=12" for phase in PHASES]
    parts.append("PAIA_SWEEP_PREFLIGHT_SUCCESS phases=6")
    if execute:
        parts += [f"[node] PAIA_SWEEP_POSE phase={phase} q1=0.00000 q2=0.00000 q3=-1.57080 q4=0.00000 max_error=0.00450" for phase in PHASES]
        parts.append("PAIA_SWEEP_SUCCESS phases=6 left=-1.57080 right=1.57080 center=0.00000")
    else:
        parts.append("PAIA_SWEEP_PLAN_ONLY_DONE phases=6 (NO ROBOT MOVEMENT)")
    return '\n'.join(parts)


class FakeProcess:
    def __init__(self, output):
        self.output = output
        self.returncode = 0
        self.pid = 1337
    def communicate(self, timeout=None):
        return self.output, None


class SweepTest(unittest.TestCase):
    def test_allowed_exact_plan(self):
        p = {"accepted":True,"stops":["CENTER","LEFT","RIGHT","CENTER"],"message":"ok"}
        self.assertIs(sweep_demo.validate_sweep_plan(p), p)
    def test_reject_old_order(self):
        with self.assertRaises(ValueError):
            sweep_demo.validate_sweep_plan({"accepted":True,"stops":["RIGHT","LEFT","CENTER"]})
    def test_reject_unapproved(self):
        with self.assertRaises(ValueError):
            sweep_demo.validate_sweep_plan({"accepted":False,"stops":[]})
    def test_parse_plan_lines(self):
        v = sweep_demo.parse_phases(fake_output(),sweep_demo.PLAN_RE)
        sweep_demo.check_phase_sequence(v)
        self.assertEqual(len(v),6)
        self.assertEqual(v[1]['phase'], 'CENTER_FOREARM_LEVEL')
        self.assertEqual(v[1]['joints_rad'][1:],[0.0,-1.57080,0.0])
    def test_parse_live_pose_lines(self):
        v = sweep_demo.parse_phases(fake_output(True),sweep_demo.POSE_RE)
        self.assertEqual(v[0]['max_error_rad'],0.0045)
        self.assertEqual(len(v),6)
    def test_detect_missing_pose(self):
        with self.assertRaises(ValueError):
            sweep_demo.check_phase_sequence(sweep_demo.parse_phases(fake_output(True),sweep_demo.POSE_RE)[:-1])
    def test_nonmoving_ros_command(self):
        with patch.object(sweep_demo.subprocess,'Popen',return_value=FakeProcess(fake_output())) as p:
            plans,poses=sweep_demo.run_ros_launch(False)
            self.assertEqual(len(plans),6)
            self.assertEqual(poses,[])
            self.assertEqual(p.call_args.args[0][-1],"mode:=plan")
    def test_execute_ros_command(self):
        with patch.object(sweep_demo.subprocess,'Popen',return_value=FakeProcess(fake_output(True))) as p:
            plans,poses=sweep_demo.run_ros_launch(True)
            self.assertEqual(len(poses),6)
            self.assertEqual(p.call_args.args[0][-1],"mode:=execute")
    def test_log_is_written_without_moving(self):
        p={"accepted":True,"stops":["CENTER","LEFT","RIGHT","CENTER"],"message":"ok"}
        with tempfile.TemporaryDirectory() as tmp, \
             patch.object(sweep_demo,'ollama_plan',return_value=p), \
             patch.object(sweep_demo,'run_ros_launch',return_value=(sweep_demo.parse_phases(fake_output(),sweep_demo.PLAN_RE),[])):
            out=str(Path(tmp)/'log.jsonl')
            self.assertEqual(sweep_demo.main(['--log',out]),0)
            record=json.loads(Path(out).read_text().splitlines()[-1])
            self.assertEqual(record['result'],'plan_only')
            self.assertEqual(len(record['planned_phases']),6)
    def test_no_move_if_invalid_llm(self):
        p={"accepted":True,"stops":["CENTER","LEFT","RIGHT"],"message":"ok"}
        with tempfile.TemporaryDirectory() as tmp, \
             patch.object(sweep_demo,'ollama_plan',return_value=p), \
             patch.object(sweep_demo,'run_ros_launch') as runner:
            self.assertEqual(sweep_demo.main(['--execute','--log',str(Path(tmp)/'bad.jsonl')]),1)
            runner.assert_not_called()

if __name__=='__main__': unittest.main()
