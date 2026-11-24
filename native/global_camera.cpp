#include <libobsensor/ObSensor.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <sstream>
static volatile std::sig_atomic_t running=1;
static void stopStream(int){running=0;}
int main(int argc,char** argv){try{
  if(argc!=3)throw std::runtime_error("Usage: franka_cap_global SERIAL OUTPUT_DIRECTORY");
  const bool streaming=std::string(argv[2])=="--stream";
  std::signal(SIGTERM,stopStream);std::signal(SIGINT,stopStream);
  std::filesystem::path out(argv[2]);if(!streaming)std::filesystem::create_directories(out);
  ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_ERROR);ob::Context ctx;
  ctx.enableNetDeviceEnumeration(true);std::this_thread::sleep_for(std::chrono::seconds(3));
  auto devices=ctx.queryDeviceList();std::shared_ptr<ob::Device> dev;
  for(uint32_t i=0;i<devices->getCount();++i)if(std::string(devices->getSerialNumber(i))==argv[1])dev=devices->getDevice(i);
  if(!dev)throw std::runtime_error("Configured global camera not found");
  auto profile=dev->getSensor(OB_SENSOR_COLOR)->getStreamProfileList()->getVideoStreamProfile(1280,800,OB_FORMAT_MJPG,10);
  auto depthProfile=dev->getSensor(OB_SENSOR_DEPTH)->getStreamProfileList()->getVideoStreamProfile(1280,800,OB_FORMAT_Y16,10);
  ob::Pipeline pipe(dev);auto cfg=std::make_shared<ob::Config>();
  cfg->enableStream(profile);cfg->enableStream(depthProfile);
  cfg->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
  pipe.enableFrameSync();pipe.start(cfg);
  ob::Align align(OB_STREAM_COLOR);align.setMatchTargetResolution(true);
  std::shared_ptr<ob::ColorFrame> frame;
  std::shared_ptr<ob::DepthFrame> depth;
  bool first=true;
  do {
  frame.reset();depth.reset();
  for(int i=0;i<(streaming&&!first?3:15)&&running;++i){
    auto frames=pipe.waitForFrameset(1000);
    if(!frames || !frames->getColorFrame() || !frames->getDepthFrame())continue;
    auto aligned=align.process(frames)->as<ob::FrameSet>();
    frame=aligned->getColorFrame();depth=aligned->getDepthFrame();
    if(streaming&&frame&&depth)break;
  }
  first=false;
  if(!running)break;
  if(!frame || !depth)throw std::runtime_error("No global RGB-D frameset");
  if(depth->getFormat()!=OB_FORMAT_Y16 || depth->getWidth()!=frame->getWidth() || depth->getHeight()!=frame->getHeight()
     || depth->getDataSize()!=frame->getWidth()*frame->getHeight()*sizeof(uint16_t))
    throw std::runtime_error("Unexpected aligned depth format or dimensions");
  const double skew=std::abs(double(frame->getTimeStampUs())-double(depth->getTimeStampUs()))/1000.;
  if(skew>50.)throw std::runtime_error("Global RGB-D timestamp skew exceeds 50 ms");
  const double scale=depth->getValueScale()/1000.; // Orbbec scale is millimeters per raw unit.
  if(!std::isfinite(scale) || scale<=0)throw std::runtime_error("Invalid depth scale");
  auto dk=depth->getStreamProfile()->as<ob::VideoStreamProfile>()->getIntrinsic();
  auto ck=profile->getIntrinsic();
  if(std::abs(dk.fx-ck.fx)>.1 || std::abs(dk.fy-ck.fy)>.1 || std::abs(dk.cx-ck.cx)>.1 || std::abs(dk.cy-ck.cy)>.1)
    throw std::runtime_error("Aligned depth intrinsics differ from color");
  const auto stamp=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  if(!streaming){
  std::ofstream image(out/"rgb.jpg",std::ios::binary);image.write((const char*)frame->getData(),frame->getDataSize());image.close();
  std::ofstream depthFile(out/"depth_u16.raw",std::ios::binary);
  depthFile.write((const char*)depth->getData(),depth->getDataSize());depthFile.close();
  if(!image || !depthFile)throw std::runtime_error("Failed to save RGB-D files");
  }
  auto k=profile->getIntrinsic();auto d=profile->getDistortion();
  const char* model=(d.model==OB_DISTORTION_BROWN_CONRADY || d.model==OB_DISTORTION_BROWN_CONRADY_K6)
    ? "distortion.brown_conrady" : (d.model==OB_DISTORTION_NONE ? "distortion.none" : "unsupported");
  std::ostringstream meta;meta<<std::setprecision(17)<<"{\"serial\":\""<<argv[1]
    <<"\",\"frame_id\":"<<frame->getIndex()<<",\"host_monotonic_s\":"<<stamp<<",\"width\":"<<k.width<<",\"height\":"<<k.height
    <<",\"fx\":"<<k.fx<<",\"fy\":"<<k.fy<<",\"ppx\":"<<k.cx<<",\"ppy\":"<<k.cy
    <<",\"distortion_model_enum\":"<<int(d.model)<<",\"coeffs\":["<<d.k1<<','<<d.k2<<','<<d.p1<<','<<d.p2<<','<<d.k3<<','<<d.k4<<','<<d.k5<<','<<d.k6
    <<"],\"model\":\""<<model<<"\",\"depth_available\":true,\"alignment\":\"depth_to_color\",\"depth_scale_m\":"<<scale
    <<",\"depth_frame_id\":"<<depth->getIndex()<<",\"color_timestamp_ms\":"<<frame->getTimeStampUs()/1000.
    <<",\"depth_timestamp_ms\":"<<depth->getTimeStampUs()/1000.<<",\"rgb_depth_skew_ms\":"<<skew
    <<",\"timestamp_domain\":\"device\",\"depth_dtype\":\"uint16_native\",\"sdk_version\":\"2.10.6\"}\n";
  if(streaming){
    auto text=meta.str();
    // Fixed ASCII length header followed by JSON, JPEG, native uint16 depth.
    std::cout<<text.size()<<" "<<frame->getDataSize()<<" "<<depth->getDataSize()<<"\n";
    std::cout.write(text.data(),text.size());
    std::cout.write((const char*)frame->getData(),frame->getDataSize());
    std::cout.write((const char*)depth->getData(),depth->getDataSize());std::cout.flush();
    if(!std::cout)break;
  }else{std::ofstream file(out/"camera.json");file<<meta.str();if(!file)throw std::runtime_error("metadata write failed");}
  }while(streaming&&running);
  pipe.stop();return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
