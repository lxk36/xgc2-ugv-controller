#!/usr/bin/env python3
"""Compile production FSM/worker sources with explicit ROS/solver boundary doubles.

This is a deterministic ownership regression, not an acados or ROS transport test.
No timing sleeps are used to arrange the worker race: a barrier controls compute.
Run from any directory with Python 3 and a C++17 compiler (CXX defaults to g++).
"""
from pathlib import Path
import os
import subprocess
import tempfile

STUB = r"""
#pragma once
#include <array>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <variant>
#include <vector>
#define ROS_WARN_THROTTLE(...) ((void)0)
#define ROS_INFO_THROTTLE(...) ((void)0)
#define ROS_WARN(...) ((void)0)
namespace ros {
struct Time {
    double t=0; Time()=default; explicit Time(double value):t(value){}
    double toSec() const {return t;}
    static Time now(){return Time(1.01);}
};
struct Publisher {template<class T> void publish(const T&) {}}
;
struct NodeHandle {template<class T> Publisher advertise(const char*,uint32_t){return {};}};
}
namespace geometry_msgs {
struct Point {double x=0,y=0,z=0;};
struct Quaternion {double x=0,y=0,z=0,w=1;};
struct Pose {Point position;Quaternion orientation;};
struct Header {ros::Time stamp;std::string frame_id;};
struct PoseStamped {Header header;Pose pose;};
struct PoseArray {Header header;std::vector<Pose> poses;};
}
namespace nav_msgs {struct Path {geometry_msgs::Header header;std::vector<geometry_msgs::PoseStamped> poses;};}
namespace state_machine {
using EventId=uint32_t;struct Status{};struct ActionResult{};
struct EventTimestamp {double value;};
enum class EventCategory {kInput,kInternal,kOutput};
struct Event {
    EventId id;double timestamp;uint64_t correlation_id=0;std::string source;
    EventCategory category=EventCategory::kInput;
    std::map<std::string,std::variant<double,bool,std::string>> payload;
    Event(EventId i,EventTimestamp t):id(i),timestamp(t.value){}
};
struct StateContext {std::vector<Event> events;void emitOutput(Event e){events.push_back(std::move(e));}};
struct State {
    virtual ~State()=default;
    virtual std::string name() const=0;
    virtual ActionResult onEnter(StateContext&)=0;
    virtual ActionResult onEvent(StateContext&,const Event&)=0;
    virtual ActionResult onTick(StateContext&)=0;
    virtual ActionResult onExit(StateContext&)=0;
};
namespace runtime {struct EventConsumer {virtual ~EventConsumer()=default;virtual std::string name() const=0;virtual bool handle(const Event&)=0;};}
}
namespace unicycle_ugv_controller {
namespace event_type {constexpr uint32_t INPUT_NMPC_SOLVE_SUCCEEDED=30,INPUT_NMPC_SOLVE_FAILED=31;}
namespace output_event_type {constexpr uint32_t REQUEST_NMPC_SOLVE=40,PUBLISH_CMD_VEL=41,PUBLISH_ZERO_CMD_VEL=42;}
enum class TrackingStrategy {NMPC,FLATNESS};
struct ControllerConfig {
    TrackingStrategy tracking_strategy=TrackingStrategy::NMPC;
    double prediction_horizon=1,solve_timeout=.05,result_timeout=.1;
    double nmpc_request_rate_hz=100,command_publish_rate_hz=30;
    double velocity_dt_min=1.0e-4;
};
struct UgvState {double speed=0,yaw=0;};
struct WorldPvaReference{};
struct ControlCommand {ros::Time stamp;double linear_speed=0,angular_speed=0;bool valid=false;};
struct FlatnessCommandOutput {bool valid=false;double linear_speed=0,angular_speed=0;};
inline FlatnessCommandOutput computeFlatnessCommand(const UgvState&,const WorldPvaReference&,double,double,const ControllerConfig&){return {};}
inline double wrapAngle(double x){return std::atan2(std::sin(x),std::cos(x));}
class PeriodicGate {
    bool initialized=false;double last=0;
public:
    void reset(){initialized=false;}
    bool due(double t,double period){if(!initialized||t-last>=period){initialized=true;last=t;return true;}return false;}
};
struct Se2Reference {struct {double yaw=0;} state;};
struct NmpcStateVector {double operator()(int)const{return 0;}};
struct UnicycleNmpcSolver {static int horizonSteps(){return 10;}};
struct ReferenceCache {bool sampleHorizon(ros::Time,double,int,std::vector<Se2Reference>& r){r.resize(11);return true;}};
class UnicycleUgvController {
    mutable std::mutex mutex;ControlCommand command_;UgvState state_;ReferenceCache cache;
public:
    double now=1;ControllerConfig cfg;
    ControllerConfig config() const{return cfg;}
    double currentTime() const{return now;}
    const UgvState& state()const{return state_;}
    UgvState controlState()const{return state_;}
    bool referenceReady()const{return true;}
    bool worldPvaReady()const{return false;}
    WorldPvaReference liftedWorldPva()const{return {};}
    ReferenceCache& referenceCache(){return cache;}
    void clearCommand(){setCommand({});}
    void setCommand(ControlCommand c){std::lock_guard<std::mutex> lock(mutex);command_=c;}
    ControlCommand command()const{std::lock_guard<std::mutex> lock(mutex);return command_;}
    bool commandReady()const{auto c=command();return c.valid&&now-c.stamp.toSec()<=cfg.result_timeout;}
};
struct ComputeBarrier {
    std::mutex mutex;std::condition_variable cv;bool entered=false,released=false;
    void waitEntered(){std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[&]{return entered;});}
    void release(){std::lock_guard<std::mutex> lock(mutex);released=true;cv.notify_all();}
};
inline ComputeBarrier barrier;
struct NmpcTrackingBackend {
    void configure(const ControllerConfig&){} bool enter(){return true;} void exit(){}
    bool compute(const UgvState&,const std::vector<Se2Reference>&,const ros::Time& t,ControlCommand& c){
        std::unique_lock<std::mutex> lock(barrier.mutex);barrier.entered=true;barrier.cv.notify_all();
        barrier.cv.wait(lock,[]{return barrier.released;});c.stamp=t;c.linear_speed=.8;c.angular_speed=.2;c.valid=true;return true;
    }
    double solveTimeMs()const{return 0;}int status()const{return 0;}
    size_t predictedStateCount()const{return 0;}
    const std::array<NmpcStateVector,11>& predictedStates()const{static const std::array<NmpcStateVector,11> a{};return a;}
};
}
"""

MAIN = r"""
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "unicycle_ugv_controller/state_machine/custom1_state.h"
#include "unicycle_ugv_controller/output/nmpc_output_consumer.h"
using namespace unicycle_ugv_controller;
namespace sm=state_machine;
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
sm::Event request(Custom1State& state,sm::StateContext& ctx){
    ctx.events.clear();state.onTick(ctx);
    for(const auto& e:ctx.events)if(e.id==output_event_type::REQUEST_NMPC_SOLVE)return e;
    throw std::runtime_error("expected solve request");
}
sm::Event success(uint64_t seq,double stamp,double speed=.7){
    sm::Event e(event_type::INPUT_NMPC_SOLVE_SUCCEEDED,sm::EventTimestamp{stamp});
    e.correlation_id=seq;e.payload["command_stamp"]=stamp;e.payload["linear_speed"]=speed;e.payload["angular_speed"]=.1;return e;
}
int main(){
    try {
        UnicycleUgvController controller;Custom1State state(controller);sm::StateContext ctx;
        ros::NodeHandle nh;std::mutex mutex;std::condition_variable cv;std::vector<sm::Event> results;
        NmpcOutputConsumer worker(nh,controller,[&](sm::Event event){
            std::lock_guard<std::mutex> lock(mutex);results.push_back(std::move(event));cv.notify_all();return sm::Status{};
        },1);
        state.onEnter(ctx);auto first=request(state,ctx);worker.handle(first);barrier.waitEntered();
        state.onExit(ctx);
        ControlCommand reset;reset.valid=true;reset.stamp=ros::Time(1);reset.linear_speed=-.25;controller.setCommand(reset);
        barrier.release();
        sm::Event old=success(0,0);
        {std::unique_lock<std::mutex> lock(mutex);check(cv.wait_for(lock,std::chrono::seconds(3),[&]{return !results.empty();}),"worker result timeout");old=results.front();}
        check(controller.command().linear_speed==-.25,"old NMPC worker overwrote Reset command");
        controller.now=1.01;state.onEnter(ctx);auto second=request(state,ctx);
        check(second.correlation_id>first.correlation_id,"request ID reused on reentry");
        state.onEvent(ctx,old);check(!controller.command().valid,"old session accepted");
        state.onEvent(ctx,success(second.correlation_id+99,1.01));check(!controller.command().valid,"future/unrequested ID accepted");
        controller.now=1.02;state.onEvent(ctx,success(second.correlation_id,1.01));check(controller.command().linear_speed==.7,"new matching result rejected");
        state.onEvent(ctx,success(second.correlation_id,1.01,.9));check(controller.command().linear_speed==.7,"duplicate result overwrote command");
        // Invalid payloads and result timestamps never become current commands.
        for(int scenario=0;scenario<5;++scenario){
            controller.now+=1;state.onExit(ctx);state.onEnter(ctx);auto req=request(state,ctx);
            auto e=success(req.correlation_id,controller.now);
            if(scenario==0)e.payload.erase("linear_speed");
            if(scenario==1)e.payload["linear_speed"]=std::numeric_limits<double>::quiet_NaN();
            if(scenario==2)e.payload["command_stamp"]=controller.now+1;
            if(scenario==3)e.payload["command_stamp"]=controller.now-1;
            if(scenario==4)controller.now+=.06;
            state.onEvent(ctx,e);check(!controller.command().valid,"invalid or late result accepted");
        }
        std::cout<<"PASS: actual worker cannot overwrite Reset; FSM rejects old, duplicate, unmatched, invalid and late results\n";
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
"""


def main():
    package = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="nmpc-ownership-") as tmp:
        root = Path(tmp)
        (root / "boundary.h").write_text(STUB)
        for name in (
            "ros/ros.h", "ros/console.h", "geometry_msgs/PoseArray.h", "nav_msgs/Path.h",
            "state_machine/state_machine.hpp", "state_machine/runtime/event_dispatcher.hpp",
            "unicycle_ugv_controller/common/types.h",
            "unicycle_ugv_controller/state_machine/periodic_gate.h",
            "unicycle_ugv_controller/unicycle_ugv_controller.h",
            "unicycle_ugv_controller/nmpc/nmpc_tracking_backend.h",
        ):
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#include "boundary.h"\n')
        (root / "test.cpp").write_text(MAIN)
        subprocess.run([
            os.environ.get("CXX", "g++"), "-std=c++17", "-pthread",
            "-Wall", "-Wextra", "-Werror", "-pedantic", "-I", str(root),
            "-I", str(package / "include"), str(root / "test.cpp"),
            str(package / "src/state_machine/custom1_state.cpp"),
            str(package / "src/output/nmpc_output_consumer.cpp"),
            "-o", str(root / "test"),
        ], check=True, timeout=60)
        subprocess.run([str(root / "test")], check=True, timeout=10)


if __name__ == "__main__":
    main()
