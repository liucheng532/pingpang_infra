// Bounded read-only DDS diagnostic. This source has no command publisher.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>

using Clock = std::chrono::steady_clock;
using LowState = unitree_hg::msg::dds_::LowState_;
std::mutex guard;
uint64_t count = 0, invalid = 0, duplicate_ticks = 0, backward_ticks = 0;
uint32_t last_tick = 0;
double first = 0, last = 0, last_output = 0, max_gap = 0, max_abs_dq = 0;

double mono() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
void receive_state(const void* data) {
    const auto& state = *static_cast<const LowState*>(data);
    const double now = mono();
    std::lock_guard<std::mutex> lock(guard);
    if (count) {
        max_gap = std::max(max_gap, now-last);
        duplicate_ticks += state.tick() == last_tick;
        backward_ticks += state.tick() < last_tick;
    } else { first = now; }
    ++count; last = now; last_tick = state.tick();
    bool finite = true;
    for (size_t i=0; i<29; ++i) {
        const auto& m = state.motor_state()[i];
        finite = finite && std::isfinite(m.q()) && std::isfinite(m.dq());
        if (std::isfinite(m.dq())) max_abs_dq = std::max(max_abs_dq, double(std::abs(m.dq())));
    }
    for (auto x:state.imu_state().quaternion()) finite = finite && std::isfinite(x);
    if (!finite) { ++invalid; return; }
    if (now-last_output < .02) return;
    last_output = now;
    std::cout << "{\"kind\":\"lowstate\",\"receive_monotonic_s\":" << now
              << ",\"tick\":" << state.tick() << ",\"mode_machine_raw\":" << int(state.mode_machine())
              << ",\"mode_pr_raw\":" << int(state.mode_pr()) << ",\"q_sdk_order\":[";
    for(size_t i=0;i<29;++i) std::cout << (i?",":"") << state.motor_state()[i].q();
    std::cout << "],\"dq_sdk_order\":[";
    for(size_t i=0;i<29;++i) std::cout << (i?",":"") << state.motor_state()[i].dq();
    std::cout << "],\"imu_quaternion_raw\":[";
    for(size_t i=0;i<4;++i) std::cout << (i?",":"") << state.imu_state().quaternion()[i];
    std::cout << "],\"device_identity_verified\":false,\"real_input_accepted\":false}" << std::endl;
}
int main(int argc, char** argv) {
    if(argc != 2 || std::string(argv[1]) != "--read-only-20s") return 2;
    std::cout.precision(12);
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
    auto sub = std::make_shared<unitree::robot::ChannelSubscriber<LowState>>("rt/lowstate");
    sub->InitChannel(receive_state, 1);
    std::this_thread::sleep_for(std::chrono::seconds(20));
    sub->CloseChannel();
    std::lock_guard<std::mutex> lock(guard);
    const double hz = count>1 && last>first ? (count-1)/(last-first) : 0;
    std::cout << "{\"kind\":\"summary\",\"received\":" << count
              << ",\"callback_hz\":" << hz << ",\"max_gap_s\":" << max_gap
              << ",\"invalid\":" << invalid << ",\"duplicate_ticks\":" << duplicate_ticks
              << ",\"backward_ticks\":" << backward_ticks << ",\"max_abs_dq_rad_s\":" << max_abs_dq
              << ",\"control_commands_sent\":0,\"device_identity_verified\":false,\"real_input_accepted\":false}" << std::endl;
    // Subscription has been closed and output flushed. The deployed Cyclone DDS
    // library can assert in static EntityDelegate destruction after main returns
    // (observed on 66). For this bounded read-only process, let the OS release
    // remaining participant resources without invoking those static destructors.
    // This is NOT a motor stop procedure; this process has no command publisher.
    std::cout.flush();
    std::_Exit(count ? 0 : 3);
}
