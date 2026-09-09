// Passive DDS -> private LCM telemetry. No LowCmd type/channel or command subscriber.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <lcm/lcm-cpp.hpp>
#include "body_control_data_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>
#include <time.h>

using LowState = unitree_hg::msg::dds_::LowState_;
using ImuState = unitree_hg::msg::dds_::IMUState_;
std::mutex lock;
body_control_data_lcmt joints{};
state_estimator_lcmt imu{};
uint64_t low_time=0, torso_time=0, received=0, bad=0;
uint64_t raw_ns() {
    timespec t{}; clock_gettime(CLOCK_MONOTONIC_RAW,&t);
    return uint64_t(t.tv_sec)*1000000000ULL+uint64_t(t.tv_nsec);
}
uint32_t crc32(uint32_t* p,uint32_t n) {
    uint32_t crc=0xffffffff;
    for(uint32_t i=0;i<n;++i) {
        uint32_t bit=1U<<31, data=p[i];
        for(uint32_t j=0;j<32;++j) {
            bool top=crc&0x80000000; crc<<=1;
            if(top)crc^=0x04c11db7;
            if(data&bit)crc^=0x04c11db7;
            bit>>=1;
        }
    }
    return crc;
}
void low(const void* value) {
    LowState s=*static_cast<const LowState*>(value);
    uint64_t now=raw_ns();
    std::lock_guard<std::mutex> g(lock);
    ++received;
    if(s.crc()!=crc32(reinterpret_cast<uint32_t*>(&s),(sizeof(LowState)>>2)-1)) {++bad;return;}
    for(size_t i=0;i<29;++i)
        if(!std::isfinite(s.motor_state()[i].q())||!std::isfinite(s.motor_state()[i].dq())){++bad;return;}
    for(auto v:s.imu_state().quaternion())if(!std::isfinite(v)){++bad;return;}
    for(auto v:s.imu_state().gyroscope())if(!std::isfinite(v)){++bad;return;}
    for(auto v:s.imu_state().accelerometer())if(!std::isfinite(v)){++bad;return;}
    for(size_t i=0;i<29;++i){joints.q[i]=s.motor_state()[i].q();joints.qd[i]=s.motor_state()[i].dq();}
    for(size_t i=0;i<4;++i)imu.quat[i]=s.imu_state().quaternion()[i];
    for(size_t i=0;i<3;++i){imu.omegaBody[i]=s.imu_state().gyroscope()[i];imu.aBody[i]=s.imu_state().accelerometer()[i];}
    low_time=now;joints.timestamp_us=now/1000;imu.timestamp_us=now/1000;
}
void torso(const void* value) {
    const auto& s=*static_cast<const ImuState*>(value);
    std::lock_guard<std::mutex> g(lock);
    for(auto v:s.rpy())if(!std::isfinite(v)){++bad;return;}
    for(size_t i=0;i<3;++i)imu.rpy[i]=s.rpy()[i];
    torso_time=raw_ns();
}
int main(int argc,char** argv) {
    if(argc!=3 || std::string(argv[1])!="--read-only-45s")return 2;
    std::string url=argv[2];
    if(url!="udpm://239.255.77.198:7767?ttl=0" && url!="udpm://239.255.77.66:7767?ttl=0")return 2;
    lcm::LCM bus(url); if(!bus.good())return 3;
    unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");
    auto a=std::make_shared<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
    auto b=std::make_shared<unitree::robot::ChannelSubscriber<ImuState>>("rt/secondary_imu");
    a->InitChannel(low,1);b->InitChannel(torso,1);
    auto end=std::chrono::steady_clock::now()+std::chrono::seconds(45);
    uint64_t published=0;
    std::cout<<"{\"started\":true,\"motor_command_publisher\":false}"<<std::endl;
    while(std::chrono::steady_clock::now()<end) {
        {
            std::lock_guard<std::mutex> g(lock);uint64_t now=raw_ns();
            if(low_time && torso_time && now-low_time<100000000 && now-torso_time<100000000) {
                imu.timestamp_us=std::min(low_time,torso_time)/1000;
                bus.publish("body_control_data",&joints);bus.publish("state_estimator_data",&imu);++published;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    a->CloseChannel();b->CloseChannel();
    std::cout<<"{\"received\":"<<received<<",\"rejected\":"<<bad<<",\"published\":"<<published
             <<",\"control_commands_sent\":0}"<<std::endl;
    // SDK static destructor asserted in a previous passive subscriber. Channels
    // are closed; avoid SDK static teardown. This is not a motor-stop operation.
    std::_Exit(published?0:4);
}
