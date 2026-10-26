import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import moveit_demo

class FakeProcess:
    returncode = 0
    def __init__(self, *args, **kwargs):
        self.stdout = io.StringIO('stream object; not a string')
    def communicate(self, timeout=None):
        return ('''[station_move-1] PAIA_POSE station=RIGHT phase=extended yaw=0.40000 q2=0.00000 q3=0.00000 q4=0.00000 max_error=0.00100
[station_move-1] PAIA_POSE station=RIGHT phase=retracted yaw=0.40000 q2=-1.16000 q3=-2.01000 q4=-0.73000 max_error=0.00100
[station_move-1] PAIA_STATION_SUCCESS RIGHT
[INFO] [station_move-1]: process has finished cleanly
''', None)

class TestRuntime(unittest.TestCase):
    def test_ros_launch_completed_output_is_parsed(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = str(Path(tmp)/'run.jsonl')
            plan = {'accepted':True, 'stops':['RIGHT'], 'message':'ok'}
            with patch.object(moveit_demo,'ollama_plan',return_value=plan), \
                 patch.object(moveit_demo.subprocess,'Popen',FakeProcess):
                rc=moveit_demo.main(['--instruction','右へ','--execute','--log',output])
            self.assertEqual(rc,0)
            item=json.loads(Path(output).read_text().splitlines()[-1])
            self.assertEqual(item['result'],'success')
            self.assertEqual([p['phase'] for p in item['movements'][0]['poses']],['extended','retracted'])

if __name__ == '__main__':
    unittest.main()
