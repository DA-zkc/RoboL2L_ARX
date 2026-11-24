"""Offline C++ ownership regression: never constructs a Panda or Gripper.

Run with --pybind-include pointing to the headers used to build panda_py.
"""
import argparse
import importlib
from pathlib import Path
import subprocess
import sys
import sysconfig
import tempfile
import numpy as np
import panda_py.motion

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--pybind-include',type=Path,required=True)
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as folder:
    source=Path(folder)/'trajectory_binding.cpp'
    source.write_text('''#include "motion/generators.h"
PYBIND11_MODULE(trajectory_binding, m) {
  m.def("accept", [](std::shared_ptr<motion::JointTrajectory> trajectory) {
    return bool(trajectory);
  });
}
''')
    output=Path(folder)/('trajectory_binding'+sysconfig.get_config_var('EXT_SUFFIX'))
    subprocess.run(['c++','-O2','-shared','-std=c++17','-fPIC','-fvisibility=hidden',str(source),
        '-o',str(output),'-I'+sysconfig.get_paths()['include'],'-I/usr/include/eigen3',
        '-I'+str(root/'vendor/panda-py/include'),'-I'+str(args.pybind_include)],check=True)
    sys.path.insert(0,folder)
    bridge=importlib.import_module('trajectory_binding')
    q=np.array([0.,-.3,0.,-1.8,0.,1.5,.7]);goal=q.copy();goal[0]+=.01
    trajectory=panda_py.motion.JointTrajectory([q,goal],speed_factor=.01)
    assert bridge.accept(trajectory)
    print('Shared trajectory accepted by native execution argument type; no hardware connected.')
