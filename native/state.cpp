// This binary has no actuation command or robot.control call.
#include <franka/robot.h>
#include <franka/gripper.h>
#include <array>
#include <iostream>
#include <iomanip>
#include <stdexcept>
template<size_t N> void array(const std::array<double,N>& a) {
  std::cout << '['; for(size_t i=0;i<N;++i){if(i)std::cout<<',';std::cout<<a[i];} std::cout<<']';
}
void matrix(const std::array<double,16>& a){
  std::cout<<'[';for(size_t r=0;r<4;++r){if(r)std::cout<<',';std::cout<<'[';
  for(size_t c=0;c<4;++c){if(c)std::cout<<',';std::cout<<a[c*4+r];}std::cout<<']';}std::cout<<']';
}
int main(int argc,char** argv){try{
  if(argc!=2)throw std::runtime_error("Usage: franka_cap_state ROBOT_IP (read only)");
  franka::Robot robot(argv[1]); auto s=robot.readOnce();
  franka::Gripper gripper(argv[1]); auto g=gripper.readOnce();
  std::cout<<std::setprecision(17)<<"{\"q\":";array(s.q);
  std::cout<<",\"q_d\":";array(s.q_d);std::cout<<",\"dq_d\":";array(s.dq_d);std::cout<<",\"dq\":";array(s.dq);
  std::cout<<",\"T_base_ee\":";matrix(s.O_T_EE);
  std::cout<<",\"T_base_ee_commanded\":";matrix(s.O_T_EE_c);
  std::cout<<",\"cartesian_contact\":";array(s.cartesian_contact);
  std::cout<<",\"external_wrench\":";array(s.O_F_ext_hat_K);
  std::cout<<",\"F_T_EE\":";matrix(s.F_T_EE);
  std::cout<<",\"EE_T_K\":";matrix(s.EE_T_K);
  std::cout<<",\"mode\":"<<static_cast<int>(s.robot_mode)<<",\"has_errors\":"<<(s.current_errors?"true":"false");
  std::cout<<",\"gripper\":{\"width_m\":"<<g.width<<",\"max_width_m\":"<<g.max_width
           <<",\"is_grasped\":"<<(g.is_grasped?"true":"false")<<",\"temperature_c\":"<<g.temperature<<"}}\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
