"""Recreate the pinned, patched RoboTwin environment and official assets."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parent
COMMIT = 'ea8b21121ebb3cd201ff5b3fe361944ac94eda3f'
ASSETS = {
    'embodiments': '85ffeff55a5066def5931224a85cfa3f8abaa1fbf779bd17789a7fb3f85bc789',
    'objects': '6aa56b3cf1e1064f7c809308144da36b00815f8b137fef2d7e4de856f8becf27',
    'background_texture': '54ede0fb5b783e0faa2bc98720d3affd6ca3bb9280b225b48c1aafaf31473070',
}
p = argparse.ArgumentParser()
p.add_argument('--assets', nargs='*', choices=list(ASSETS), default=list(ASSETS))
p.add_argument('--skip-python', action='store_true')
args = p.parse_args()
assert platform.system() == 'Darwin' and platform.machine() == 'arm64', 'Native Apple Silicon required'
if not shutil.which('xcrun'):
    raise RuntimeError('Xcode Command Line Tools are required to build the Metal compute bridge')

def run(*command, **kwargs):
    subprocess.run(command, check=True, cwd=ROOT, **kwargs)

for formula in ['molten-vk', 'vulkan-loader']:
    if not Path('/opt/homebrew/opt', formula).exists():
        run('brew', 'install', formula)
if not args.skip_python:
    if not (ROOT/'.venv/bin/python').exists():
        run('uv', 'venv', '--python', '3.11', '.venv')
    run('uv', 'pip', 'sync', '--python', '.venv/bin/python', 'requirements-macos.lock')

repo = ROOT/'vendor/RoboTwin'
if not repo.exists():
    run('git', 'clone', 'https://github.com/RoboTwin-Platform/RoboTwin.git', str(repo))
    run('git', '-C', str(repo), 'checkout', COMMIT)
head = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
assert head == COMMIT, f'Expected {COMMIT}, found {head}; refusing to overwrite another checkout'
patch = ROOT/'patches/robotwin-macos.patch'
applied = subprocess.run(['git', '-C', str(repo), 'apply', '--reverse', '--check', str(patch)],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
if not applied:
    run('git', '-C', str(repo), 'apply', '--check', str(patch))
    run('git', '-C', str(repo), 'apply', str(patch))

for name in args.assets:
    archive = ROOT/'vendor'/f'{name}.zip'
    marker = ROOT/'vendor'/f'{name}.verified.json'
    if marker.exists() and (repo/'assets'/name).is_dir():
        if json.loads(marker.read_text()).get('sha256') == ASSETS[name]:
            print(f'{name}: already verified', flush=True)
            continue
    if not archive.exists():
        run('curl', '-L', '--fail', '--retry', '3', '-o', str(archive),
            f'https://huggingface.co/datasets/TianxingChen/RoboTwin2.0/resolve/main/{name}.zip')
    with archive.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    assert digest == ASSETS[name], f'{name}: incomplete or mismatched archive: {digest}'
    destination = (repo/'assets').resolve()
    with zipfile.ZipFile(archive) as z:
        for member in z.infolist():
            assert (destination/member.filename).resolve().is_relative_to(destination), member.filename
        z.extractall(destination)
        manifest = {'sha256': digest, 'files': len(z.infolist()),
                    'uncompressed_bytes': sum(x.file_size for x in z.infolist())}
    marker.write_text(json.dumps(manifest, indent=2)+'\n')
    print(f'{name}: extracted and verified', flush=True)
run(str(ROOT/'.venv/bin/python'), '-c',
    'from robotwin_macos.metal_runtime import runtime, farthest_points; '
    'import numpy as np; '
    'assert farthest_points(np.array([[0,0,0],[1,0,0],[2,0,0]],dtype=np.float32),3).tolist()==[0,2,1]; '
    'print("Metal compute verified:", runtime().device)')
print('Mac environment ready')
