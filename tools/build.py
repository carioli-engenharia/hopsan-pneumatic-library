"""Build this external library with Hopsan's matching Windows toolchain."""
import argparse
import os
from pathlib import Path
import subprocess

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--hopsan-root', type=Path, required=True)
a = p.parse_args()
root = Path(os.path.abspath(a.hopsan_root))
library = Path(__file__).resolve().parents[1] / 'Library'
compiler = root / 'mingw64/bin/g++.exe'
core = root / 'bin/hopsancore.dll'
if os.name != 'nt':
    p.error('This build script currently supports the official Windows distribution only.')
for path in [compiler, core, root / 'HopsanCore/include/ComponentEssentials.h']:
    if not path.is_file():
        p.error(f'Missing Hopsan build dependency: {path}')
command = [str(compiler), '-std=c++11', '-shared', '-O2', '-DNDEBUG', '-DHOPSAN_BUILD_TYPE_RELEASE',
           '-I' + str(root / 'HopsanCore/include'),
           '-I' + str(root / 'HopsanCore/dependencies'),
           'ValvulasPneumaticas.cpp', str(core), '-o', 'pneumaticadidatica_v04.dll']
env = os.environ.copy()
env['PATH'] = str(root / 'mingw64/bin') + os.pathsep + str(root / 'bin') + os.pathsep + env.get('PATH', '')
subprocess.run(command, cwd=library, check=True, env=env)
print('Built Library/pneumaticadidatica_v04.dll')
