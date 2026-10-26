import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from moveit_demo import extract_pose_lines

class TestParsing(unittest.TestCase):
    def test_parse_moveit_log(self):
        content = '''[node] PAIA_POSE station=RIGHT phase=extended yaw=0.39900 q2=0.00000 q3=0.00000 q4=0.00000 max_error=0.00100
[node] PAIA_POSE station=RIGHT phase=retracted yaw=0.40000 q2=-1.16000 q3=-2.01000 q4=-0.73000 max_error=0.00001'''
        moves = extract_pose_lines(content, "RIGHT")
        self.assertEqual(len(moves), 2)
        self.assertEqual(moves[1]["joints"], [0.4, -1.16, -2.01, -0.73])

    def test_filter_wrong_station(self):
        content = "PAIA_POSE station=LEFT phase=extended yaw=-0.40000 q2=0 q3=0 q4=0 max_error=0.01"
        self.assertEqual(extract_pose_lines(content, "CENTER"), [])

if __name__ == "__main__":
    unittest.main()
