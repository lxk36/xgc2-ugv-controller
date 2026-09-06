// Replay recorded PVA through the production flatness law. Diagnostic only:
// the optional delayed first-order plant is not a calibrated Scout tire model.
// CSV input: receipt time (s), x, y, vx, vy, ax, ay. Start at first reference,
// rest, yaw zero. Output uses the same world XY frame and radians.
#include "unicycle_ugv_controller/common/types.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
using namespace unicycle_ugv_controller;
struct Input { double t,x,y,vx,vy,ax,ay; };
struct Command { double t,v,w; };
int main(int argc,char** argv) {
 if(argc!=7) { std::cerr<<"usage: replay input.csv kp kv delay_s tau_s plant_yaw_limit\n";return 2; }
 std::ifstream file(argv[1]);std::vector<Input> input;std::string line;
 while(std::getline(file,line)) {
  std::replace(line.begin(),line.end(),',',' ');std::istringstream stream(line);Input r;
  if(stream>>r.t>>r.x>>r.y>>r.vx>>r.vy>>r.ax>>r.ay) input.push_back(r);
 }
 if(input.size()<2) return 2;
 ControllerConfig cfg;cfg.flatness_kp=std::stod(argv[2]);cfg.flatness_kv=std::stod(argv[3]);
 const double delay=std::stod(argv[4]),tau=std::stod(argv[5]),limit=std::stod(argv[6]),dt=.004;
 if(delay<0||tau<0||limit<=0) return 2;
 UgvState state;state.x=input[0].x;state.y=input[0].y;state.yaw=0;state.received=true;
 PoseVelocityEstimator filter;double body_speed=0,v=0,w=0;std::size_t index=0;
 std::deque<Command> queue;Command delayed{input[0].t,0,0};
 std::cout<<"t,x,y,target_x,target_y,cmd_v,cmd_w,body_v,body_w,error\n";
 for(double t=input.front().t;t<=input.back().t;t+=dt) {
  while(index+1<input.size()&&input[index+1].t<=t)++index;
  const Input& r=input[index];const double age=t-r.t;
  WorldPvaReference reference;reference.valid=true;reference.x=r.x+r.vx*age+.5*r.ax*age*age;
  reference.y=r.y+r.vy*age+.5*r.ay*age*age;reference.vx=r.vx+r.ax*age;
  reference.vy=r.vy+r.ay*age;reference.ax=r.ax;reference.ay=r.ay;
  updatePoseVelocityEstimator(filter,t,state.x,state.y,state.yaw,cfg);
  state.velocity_valid=filter.velocity_valid;state.vx=filter.vx;state.vy=filter.vy;
  const auto command=computeFlatnessCommand(state,reference,body_speed,dt,cfg);
  if(command.valid) body_speed=command.linear_speed;
  const double cmd_w=command.valid?command.angular_speed:0;
  queue.push_back({t,body_speed,std::max(-limit,std::min(limit,cmd_w))});
  while(!queue.empty()&&queue.front().t<=t-delay) { delayed=queue.front();queue.pop_front(); }
  const double alpha=tau>0?-std::expm1(-dt/tau):1;
  v+=alpha*(delayed.v-v);w+=alpha*(delayed.w-w);
  const double heading=state.yaw+.5*w*dt;
  state.x+=v*std::cos(heading)*dt;state.y+=v*std::sin(heading)*dt;state.yaw+=w*dt;
  const double error=std::hypot(state.x-reference.x,state.y-reference.y);
  if(!std::isfinite(error))return 3;
  std::cout<<t<<','<<state.x<<','<<state.y<<','<<reference.x<<','<<reference.y<<','<<body_speed<<','<<cmd_w<<','<<v<<','<<w<<','<<error<<'\n';
 }
}
