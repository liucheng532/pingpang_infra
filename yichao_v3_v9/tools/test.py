import argparse,json,sys,unittest
from pathlib import Path
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'src'),str(root/'tests')]
parser=argparse.ArgumentParser()
parser.add_argument('--report',type=Path,default=root/'output/test_results.json')
args=parser.parse_args()
import test_acceptance
suite=unittest.defaultTestLoader.loadTestsFromModule(test_acceptance)
result=unittest.TextTestRunner(verbosity=2).run(suite)
test_acceptance.RESULTS.update(tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful())
args.report.parent.mkdir(parents=True,exist_ok=True)
args.report.write_text(json.dumps(test_acceptance.RESULTS,indent=2)+'\n')
sys.exit(not result.wasSuccessful())
