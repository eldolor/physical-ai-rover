"""Run at a commit's worktree root: the 9 spacing surveys' outcomes and label calls."""
import json, sys, os, logging
sys.path.insert(0, os.getcwd())
logging.disable(logging.WARNING)
from tests import survey_sweep as sw
rows = [sw.run(h, st, mode="spacing") for h in sw.HOUSES for st in sw.starts(h)]
print(json.dumps({"outcomes": [r["outcome"] for r in rows],
                  "label_calls": [r["label_calls"] for r in rows]}))
