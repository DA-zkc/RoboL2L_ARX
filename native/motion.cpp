// Project-local guarded joint-spline executor. Derived from the local FR3 guarded step implementation.
// Motion plans are checked in Python and numerically rechecked here.
// No recovery, gripper action, collision-threshold changes or homing.
#include <csignal>
constexpr double kPlanningInset=.15, kTrackingError=.003, kRuntimeInset=.01, kMaxSegment=.05;
volatile std::sig_atomic_t interrupted=0;
void on_signal(int){interrupted=1;}
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/control_types.h>
#include <array>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <sstream>

constexpr std::array<double,7> lo{-2.9007,-1.8361,-2.9007,-3.077,-2.8763,.4398,-3.0508};
constexpr std::array<double,7> hi{2.9007,1.8361,2.9007,-.1169,2.8763,4.6216,3.0508};
// Prescribed elbow redundancy must follow the generated joint references q_d.
// Cartesian impedance follows the Cartesian references instead; it does not
// establish the joint tracking contract used by this guarded path.
constexpr auto kHoverController = franka::ControllerMode::kJointImpedance;
struct Plan {std::array<double,16> start; std::array<double,3> delta; double duration,elbow_end,width; std::vector<std::array<double,7>> q; std::array<std::array<double,7>,101> second{};};
double num(std::istream& in){double v;if(!(in>>v)||!std::isfinite(v))throw std::runtime_error("Invalid plan number");return v;}
std::array<double,7> joints(const Plan& p,double f){
 double index=std::clamp(f,0.,1.)*100;int k=std::min(static_cast<int>(index),99);double b=index-k,a=1-b;
 std::array<double,7> q{};for(int j=0;j<7;j++)q[j]=a*p.q[k][j]+b*p.q[k+1][j]+((a*a*a-a)*p.second[k][j]+(b*b*b-b)*p.second[k+1][j])/60000;
 return q;
}
double progress(double u){u=std::clamp(u,0.,1.);return u*u*u*u*(35+u*(-84+u*(70-20*u)));}
Plan load(const char* file){
 std::ifstream in(file); std::string magic;in>>magic;if(magic!="FRANKA_CAP_JOINT_V1")throw std::runtime_error("Bad plan format");
 Plan p;for(auto& x:p.start)x=num(in);for(auto& x:p.delta)x=num(in);
 p.duration=num(in);p.elbow_end=num(in);p.width=num(in);double count=num(in);
 if(count!=101)throw std::runtime_error("Exactly 101 path samples required");
 int n=101;
 if(n!=101 || p.duration<8 || p.duration>30 || p.width<0 || p.width>.081)throw std::runtime_error("Plan range invalid");
 if(std::sqrt(p.delta[0]*p.delta[0]+p.delta[1]*p.delta[1]+p.delta[2]*p.delta[2])>kMaxSegment)throw std::runtime_error("Only <=5cm grasp approach/lift steps permitted");
 if(std::abs(p.start[3])+std::abs(p.start[7])+std::abs(p.start[11])+std::abs(p.start[15]-1)>1e-8)throw std::runtime_error("Invalid pose");
 for(int a=0;a<3;a++)for(int b=0;b<3;b++){
  double dot=0;for(int k=0;k<3;k++)dot+=p.start[4*a+k]*p.start[4*b+k];
  if(std::abs(dot-(a==b?1:0))>1e-5)throw std::runtime_error("Invalid rotation");}
 for(int k=0;k<n;k++){std::array<double,7> q;for(int j=0;j<7;j++){q[j]=num(in);if(q[j]<lo[j]+kPlanningInset-1e-6||q[j]>hi[j]-kPlanningInset+1e-6)throw std::runtime_error("Joint margin invalid");}
  if(k){for(int j=0;j<7;j++)if(std::abs(q[j]-p.q.back()[j])>.01)throw std::runtime_error("Discontinuous joint samples");}
  p.q.push_back(q);}
 std::string extra;if(in>>extra)throw std::runtime_error("Trailing plan data");
 if(std::abs(p.q.back()[2]-p.elbow_end)>1e-8)throw std::runtime_error("Elbow endpoint inconsistent");
 // The same natural cubic joint spline is checked offline and commanded online.
 for(int j=0;j<7;j++){
  std::array<double,101> diag{},rhs{};diag[0]=diag[100]=1;
  for(int k=1;k<100;k++){diag[k]=4;rhs[k]=6*(p.q[k+1][j]-2*p.q[k][j]+p.q[k-1][j])*10000;}
  for(int k=2;k<100;k++){double f=1/diag[k-1];diag[k]-=f;rhs[k]-=f*rhs[k-1];}
  for(int k=99;k>=1;k--)p.second[k][j]=(rhs[k]-p.second[k+1][j])/diag[k];
 }
 // Validate the interpolated curve, not just its knots. Conservative task caps.
 const double dt=.001;auto previous=joints(p,0);std::array<double,7> velocity{},acceleration{};
 for(int k=1;k<=static_cast<int>(std::ceil(p.duration/dt));k++){
  auto q=joints(p,progress(k*dt/p.duration));
  for(int j=0;j<7;j++){
   double v=(q[j]-previous[j])/dt, a=(v-velocity[j])/dt, jerk=(a-acceleration[j])/dt;
   if(q[j]<lo[j]+kPlanningInset-1e-6||q[j]>hi[j]-kPlanningInset+1e-6)throw std::runtime_error("Spline joint margin invalid");
   if(std::abs(v)>.15||std::abs(a)>.3||std::abs(jerk)>5)throw std::runtime_error("Spline dynamics exceed task caps");
   velocity[j]=v;acceleration[j]=a;
  }
  previous=q;
 }
 return p;
}
template<size_t N>void arr(const std::array<double,N>& a){std::cout<<'[';for(size_t i=0;i<N;i++){if(i)std::cout<<',';std::cout<<a[i];}std::cout<<']';}
void state(const franka::RobotState& s,double width,double measured_max=-1,double desired_max=-1,double elapsed_s=-1,bool is_grasped=false){std::cout<<"{\"q\":";arr(s.q);std::cout<<",\"q_d\":";arr(s.q_d);std::cout<<",\"dq\":";arr(s.dq);std::cout<<",\"elbow\":";arr(s.elbow);std::cout<<",\"O_T_EE\":";arr(s.O_T_EE);std::cout<<",\"O_T_EE_c\":";arr(s.O_T_EE_c);std::cout<<",\"width_m\":"<<width<<",\"max_measured_tracking_error_rad\":"<<measured_max<<",\"max_desired_path_error_rad\":"<<desired_max<<",\"elapsed_s\":"<<elapsed_s<<",\"is_grasped\":"<<(is_grasped?"true":"false")<<"}\n";}
int main(int argc,char** argv){try{
 std::signal(SIGINT,on_signal);std::signal(SIGTERM,on_signal);
 std::cout<<std::setprecision(17);

 if(argc==3 && std::string(argv[1])=="--validate"){load(argv[2]);std::cout<<"Plan numeric validation passed; no connection\n";return 0;}
 if(argc==3 && (std::string(argv[1])=="--sample" || std::string(argv[1])=="--sample-time")){auto plan=load(argv[2]);for(int k=0;k<=1000;k++){arr(joints(plan,std::string(argv[1])=="--sample-time"?progress(k/1000.):k/1000.));std::cout<<'\n';}return 0;}
 if(argc!=4 || (std::string(argv[3])!="empty" && std::string(argv[3])!="holding"))throw std::runtime_error("Usage: franka_cap_motion IP state|PLAN_FILE, or --validate PLAN_FILE");
 bool holding=std::string(argv[3])=="holding";
 std::string command=argv[2];Plan p;if(command!="state")p=load(argv[2]);
 franka::Robot robot(argv[1]);franka::Gripper gripper(argv[1]);auto s=robot.readOnce();auto gs=gripper.readOnce();
 if(command=="state"){state(s,gs.width,-1,-1,-1,gs.is_grasped);return 0;}
 if(s.robot_mode!=franka::RobotMode::kIdle || s.current_errors)throw std::runtime_error("Robot not idle/error-free");
 if(std::abs(gs.width-p.width)>.0005 || gs.is_grasped!=holding)throw std::runtime_error("Gripper state changed or holding object");
 for(int j=0;j<7;j++)if(std::abs(s.q[j]-p.q[0][j])>.001 || std::abs(s.dq[j])>.005 || std::abs(s.dq_d[j])>.005)throw std::runtime_error("Start state changed");
 for(int i=0;i<16;i++)if(std::abs(s.O_T_EE_c[i]-p.start[i])>.001 || std::abs(s.O_T_EE[i]-s.O_T_EE_c[i])>.001)throw std::runtime_error("Start Cartesian pose changed");
 if(std::abs(s.elbow[0]-p.q[0][2])>.001 || s.elbow[1]!=-1)throw std::runtime_error("Unexpected starting elbow");
 double elapsed=0,last_progress=0,measured_max=0,desired_max=0;bool first=true;
 // Joint positions remove the second, controller-side redundant IK solution.
 // Do not shift/retarget the checked spline at execution time.
 for(int j=0;j<7;j++)if(std::abs(s.q_d[j]-p.q[0][j])>.0001)throw std::runtime_error("Commanded start changed; replan required");
 robot.control([&](const franka::RobotState& current,franka::Duration period)->franka::JointPositions{
  if(interrupted)throw std::runtime_error("Operator cancellation");
  if(period.toSec()>.005)throw std::runtime_error("Control cycle exceeded 5 ms");
  if(first)first=false;else elapsed+=period.toSec();
  double f=progress(elapsed/p.duration);auto expected=joints(p,last_progress);
  for(int j=0;j<7;j++){
   measured_max=std::max(measured_max,std::abs(current.q[j]-expected[j]));
   desired_max=std::max(desired_max,std::abs(current.q_d[j]-expected[j]));
   if(current.q[j]<lo[j]+kRuntimeInset || current.q[j]>hi[j]-kRuntimeInset || std::abs(current.q[j]-expected[j])>kTrackingError || std::abs(current.q_d[j]-expected[j])>kTrackingError){
    std::ostringstream error;
    error<<std::setprecision(12)<<"Joint limit/tracking guard stopped motion: joint="<<j+1
         <<" elapsed_s="<<elapsed<<" measured="<<current.q[j]<<" desired="<<current.q_d[j]
         <<" planned="<<expected[j]<<" measured_error="<<current.q[j]-expected[j]<<" desired_error="<<current.q_d[j]-expected[j];
    throw std::runtime_error(error.str());
   }
  }
  if(current.current_errors)throw std::runtime_error("Robot error during motion");
  for(auto v:current.cartesian_contact)if(v)throw std::runtime_error("Contact detected; motion stopped");
  auto cmd=franka::JointPositions(joints(p,f));last_progress=f;
  return elapsed>=p.duration?franka::MotionFinished(cmd):cmd;
 },kHoverController,true);
 s=robot.readOnce();gs=gripper.readOnce();state(s,gs.width,measured_max,desired_max,elapsed,gs.is_grasped);return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
