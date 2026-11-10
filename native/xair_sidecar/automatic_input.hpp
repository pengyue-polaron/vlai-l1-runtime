#pragma once
// Shared pure spec reader: no SDK, devices, CAN or process operations.
#include "task_session_core.hpp"
#include <fstream>
namespace vlai_l1 {
struct Input { std::string interface; vlai_l1::AutoSpec spec; std::optional<vlai_l1::TaskLimits> task; };
inline Input read_input(const char* path) {
    std::ifstream f(path); Input input; std::string magic; auto& s=input.spec;
    f>>magic>>input.interface;
    const bool multi=magic=="VLAI_RIGHT_FOLLOWER_DIAGNOSTIC_V4";
    const bool task=magic=="VLAI_RIGHT_FOLLOWER_TASK_V2";
    if ((!multi && !task && magic!="VLAI_RIGHT_FOLLOWER_DIAGNOSTIC_V3") || input.interface.size()>=16)
        throw std::runtime_error("invalid diagnostic input");
    f>>s.amplitude_deg>>s.transition_s>>s.hold_s>>s.hold_test_s>>s.maximum_amplitude_deg
      >>s.maximum_reference_speed>>s.maximum_reference_acceleration>>s.cycles>>s.rate_hz>>s.capacity
      >>s.enabled_status>>s.disabled_status>>s.temperature>>s.state_timeout_ns>>s.tracking_timeout_ns
      >>s.shutdown_timeout_ns>>s.health_poll_ms>>s.priority>>s.active_motor;
    for (auto& m:s.motors) {
        unsigned slave=0;
        f>>m.feedback.name>>m.send_id>>m.feedback.response_id>>slave>>m.motor_type
         >>m.feedback.position_max_rad>>m.feedback.velocity_max_rad_s>>m.feedback.torque_max_nm
         >>m.kp>>m.kd>>m.min_deg>>m.max_deg>>m.tracking_deg>>m.speed_deg_s>>m.torque_nm>>m.excursion_deg;
        if (slave>15) throw std::runtime_error("invalid slave nibble");
        m.feedback.slave_id=static_cast<std::uint8_t>(slave);
    }
    unsigned handling=0;
    f>>handling;
    if(handling>1) throw std::runtime_error("invalid handling flag");
    if(handling) {
        s.handling.emplace(); auto& h=*s.handling;
        f>>h.speed_persistence_ns>>h.settle_ns>>h.settle_timeout_ns>>h.settle_speed_deg_s>>h.settle_span_deg;
        for(auto& speed:h.hard_speed_deg_s) f>>speed;
    }
    if(multi) {
        unsigned count=0; f>>count;
        if(!f || count<3 || count>64 || s.active_motor!=8)
            throw std::runtime_error("invalid waypoint count/selection");
        s.waypoints.resize(count);
        for(auto& point:s.waypoints) for(auto& value:point) f>>value;
    }
    if(task) {
        if(handling) throw std::runtime_error("task excludes manual handling");
        input.task.emplace();auto& t=*input.task;
        f>>t.command_age_ns>>t.observation_age_ns>>t.heartbeat_ns;
        t.validate();
    }
    std::string extra;
    if (!f || (f>>extra)) throw std::runtime_error("truncated/trailing diagnostic input");
    s.validate(); return input;
}
}
