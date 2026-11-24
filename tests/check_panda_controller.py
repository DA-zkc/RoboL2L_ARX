"""Run the actual native controller against synthetic states, without a robot."""
import argparse
import importlib
from pathlib import Path
import subprocess
import sys
import sysconfig
import tempfile

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--pybind-include',type=Path,required=True)
args=parser.parse_args();root=Path(__file__).resolve().parents[1]/'vendor/panda-py'
with tempfile.TemporaryDirectory() as folder:
    source=Path(folder)/'controller_probe.cpp'
    source.write_text('''#include "controllers/joint_trajectory.h"
PYBIND11_MODULE(controller_probe, m) {
  m.def("probe", [](double overrun, bool at_goal, double timeout, bool coriolis) {
    Vector7d first = Vector7d::Zero(), last = first; last[0] = 0.01;
    auto trajectory = std::make_shared<motion::JointTrajectory>(std::vector<Vector7d>{first,last},0.01);
    controllers::JointTrajectory ctrl(trajectory,JointPosition::kDefaultStiffness,
        JointPosition::kDefaultDamping,0.001,1.0,timeout,coriolis);
    franka::RobotState state{};ctrl.start(state,nullptr);
    if (at_goal) state.q[0] = 0.01;
    ctrl.setTime(trajectory->getDuration()+overrun);
    franka::Duration period(1);auto torque=ctrl.step(state,period);
    py::dict result;result["finished"]=torque.motion_finished;
    result["settled"]=ctrl.settled();result["joint0_torque"]=torque.tau_J[0];return result;
  });
}
''')
    sources=['controllers/joint_position.cpp','controllers/joint_trajectory.cpp',
             'motion/generators.cpp','motion/time_optimal/path.cpp','motion/time_optimal/trajectory.cpp']
    output=Path(folder)/('controller_probe'+sysconfig.get_config_var('EXT_SUFFIX'))
    subprocess.run(['c++','-O1','-shared','-std=c++17','-fPIC','-fvisibility=hidden',str(source),
        *[str(root/'src'/name) for name in sources],'-o',str(output),'-lfranka','-pthread',
        '-I'+sysconfig.get_paths()['include'],'-I/usr/include/eigen3',
        '-I'+str(root/'include'),'-I'+str(args.pybind_include)],check=True)
    sys.path.insert(0,folder);native=importlib.import_module('controller_probe')
    early=native.probe(-.1,False,0.,False)
    assert not early['finished'],early
    late=native.probe(3.1,False,3.,False)
    assert late['finished'] and not late['settled'] and abs(late['joint0_torque']-6.)<1e-6,late
    reached=native.probe(.1,True,0.,False)
    assert reached['finished'] and abs(reached['joint0_torque'])<1e-6,reached
    assert native.probe(.1,False,10.,False)['finished']
    for timeout,coriolis in ((-1.,False),(3.,True)):
        try:native.probe(.1,False,timeout,coriolis)
        except ValueError:pass
        else:raise AssertionError('invalid controller configuration accepted')
    print('Native PD, native curve completion without endpoint threshold, and model requirement passed; no hardware connected.')
