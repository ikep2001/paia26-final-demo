import json
import unittest
from unittest.mock import patch
import io
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from final_demo import PLAN_SCHEMA, POINTS, PlanError, validate_plan, ollama_plan

class TestPlanning(unittest.TestCase):
    def test_safelist(self):
        self.assertEqual(POINTS, {"LEFT": -.3, "CENTER": 0., "RIGHT": .3})
        self.assertEqual(PLAN_SCHEMA["properties"]["stops"]["maxItems"], 5)

    def test_good_plan(self):
        v={"accepted":True,"stops":["RIGHT","LEFT","CENTER"],"message":"巡回"}
        self.assertEqual(validate_plan(v),v)

    def test_refusal(self):
        self.assertFalse(validate_plan({"accepted":False,"stops":[],"message":"対象外"})["accepted"])

    def test_unsafe_station(self):
        with self.assertRaises(PlanError):
            validate_plan({"accepted":True,"stops":["ROTATE_360"],"message":"?"})

    def test_not_bool(self):
        with self.assertRaises(PlanError):
            validate_plan({"accepted":1,"stops":["LEFT"],"message":"?"})

    def test_too_many(self):
        with self.assertRaises(PlanError):
            validate_plan({"accepted":True,"stops":["LEFT"]*6,"message":"?"})

    def test_empty(self):
        with self.assertRaises(PlanError):
            validate_plan({"accepted":True,"stops":[],"message":"?"})

    def test_rejected_with_move(self):
        with self.assertRaises(PlanError):
            validate_plan({"accepted":False,"stops":["LEFT"],"message":"?"})

    def test_ollama_parse(self):
        valid={"accepted":True,"stops":["LEFT","CENTER"],"message":"OK"}
        result={"message":{"content":json.dumps(valid)}}
        class Reply:
            def __enter__(self): return io.BytesIO(json.dumps(result).encode())
            def __exit__(self,*a): pass
        with patch("final_demo.urlopen", return_value=Reply()):
            self.assertEqual(ollama_plan("左から中央", "http://localhost:11434", "model"),valid)

if __name__ == "__main__": unittest.main()
