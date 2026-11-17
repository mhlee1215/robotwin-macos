"""Isolate each upstream task in a process; never count a crash as a pass."""
import argparse
import ast
import json
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parent
p = argparse.ArgumentParser()
p.add_argument('--mode', default='reset', choices=['reset', 'qpos', 'simulator', 'expert'])
p.add_argument('--tasks', nargs='*')
p.add_argument('--seed', type=int, default=0)
p.add_argument('--timeout', type=int, default=180)
p.add_argument('--name', required=True)
args = p.parse_args()
tasks = args.tasks
if not tasks:
    tasks = []
    for path in sorted((root/'vendor/RoboTwin/envs').glob('*.py')):
        tree = ast.parse(path.read_text())
        if any(isinstance(n, ast.ClassDef) and any(isinstance(b, ast.Name) and b.id == 'Base_Task'
               for b in n.bases) for n in tree.body):
            tasks.append(path.stem)
out = root/'artifacts'/args.name
out.mkdir(parents=True, exist_ok=True)
results = []
for task in tasks:
    target = out/task
    target.mkdir(exist_ok=True)
    command = [sys.executable, '-u', str(root/'run_task.py'), '--task', task,
               '--mode', args.mode, '--seed', str(args.seed), '--output', str(target)]
    with (target/'stdout.log').open('w') as log:
        try:
            proc = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
            result = json.loads((target/'result.json').read_text()) if (target/'result.json').exists() else {}
            result['returncode'] = proc.returncode
            result['passed'] = bool(result.get('passed') and proc.returncode == 0)
        except subprocess.TimeoutExpired:
            result = {'passed': False, 'timeout': True}
    result['task'] = task
    results.append(result)
    (out/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
    print(f'{len(results)}/{len(tasks)} {task}: {result["passed"]} {result.get("error", "")}', flush=True)
print(f'Passed {sum(r["passed"] for r in results)}/{len(results)}')
sys.exit(0 if all(r['passed'] for r in results) else 1)
