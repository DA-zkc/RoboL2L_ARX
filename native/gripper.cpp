// Bounded Franka Hand commands; no homing, threshold changes, or error recovery.
#include <franka/robot.h>
#include <franka/gripper.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <future>
#include <iomanip>
#include <iostream>
#include <stdexcept>
volatile std::sig_atomic_t cancelled=0;
void on_signal(int){cancelled=1;}
double num(const char* text){size_t n=0;double x=std::stod(text,&n);if(text[n] || !std::isfinite(x))throw std::runtime_error("Invalid number");return x;}
int main(int argc,char** argv){try{
 if(argc!=5)throw std::runtime_error("Usage: franka_cap_gripper IP move|grasp WIDTH_M FORCE_N");
 const std::string action=argv[2];const double width=num(argv[3]),force=num(argv[4]);
 if((action!="move" && action!="grasp") || width<.015 || width>.08 || force<0 || force>10 || (action=="move" && force!=0) || (action=="grasp" && force<3))throw std::runtime_error("Gripper command exceeds task limits");
 std::signal(SIGINT,on_signal);std::signal(SIGTERM,on_signal);
 franka::Robot robot(argv[1]);auto state=robot.readOnce();
 if(state.robot_mode!=franka::RobotMode::kIdle || state.current_errors)throw std::runtime_error("Robot must be idle and error-free");
 for(auto v:state.dq)if(std::abs(v)>.005)throw std::runtime_error("Robot not stationary");
 franka::Gripper gripper(argv[1]);auto before=gripper.readOnce();
 if(width>before.max_width || before.max_width<.075)throw std::runtime_error("Gripper calibration/range invalid; no automatic homing");
 auto work=std::async(std::launch::async,[&]{return action=="grasp" ? gripper.grasp(width,.01,force,.003,.003) : gripper.move(width,.01);});
 const auto start=std::chrono::steady_clock::now();bool stopped=false;
 while(work.wait_for(std::chrono::milliseconds(20))!=std::future_status::ready){
  if(cancelled || std::chrono::steady_clock::now()-start>std::chrono::seconds(12)){
   gripper.stop();stopped=true;break;
  }
 }
 const bool ok=work.get();auto after=gripper.readOnce();
 std::cout<<std::setprecision(17)<<"{\"ok\":"<<(ok&&!stopped?"true":"false")<<",\"width_m\":"<<after.width<<",\"is_grasped\":"<<(after.is_grasped?"true":"false")<<",\"stopped\":"<<(stopped?"true":"false")<<"}\n";
 return ok&&!stopped?0:1;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
