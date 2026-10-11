#include "automatic_core.hpp"
#include <functional>
#include <iostream>
#include <limits>

void require(bool ok, const char* message) { if(!ok) throw std::runtime_error(message); }
void rejects(const std::function<void()>& operation) {
    bool caught=false; try { operation(); } catch(const std::exception&) { caught=true; }
    require(caught,"expected rejection");
}
vlai_l1::AutoSpec spec() {
    vlai_l1::AutoSpec s;
    s.amplitude_deg=.5;s.transition_s=4;s.hold_s=5;s.hold_test_s=5;
    s.maximum_amplitude_deg=2;s.maximum_reference_speed=1;s.maximum_reference_acceleration=.8;
    s.cycles=1;s.rate_hz=100;s.capacity=600000;
    s.enabled_status=1;s.disabled_status=0;s.temperature=70;
    s.state_timeout_ns=100000000;s.tracking_timeout_ns=100000000;s.shutdown_timeout_ns=2000000000;
    s.health_poll_ms=100;s.priority=20;
    for(std::size_t i=0;i<8;++i) {
        auto& m=s.motors[i];m.feedback={i<7?"joint_"+std::to_string(i+1):"gripper",static_cast<unsigned>(17+i),static_cast<std::uint8_t>(i+1),12.5,30,10};
        m.send_id=i+1;m.motor_type=0;m.kp=25;m.kd=.2;m.min_deg=-90;m.max_deg=90;
        m.tracking_deg=.8;m.speed_deg_s=15;m.torque_nm=3;m.excursion_deg=3;
    }
    return s;
}
std::array<vlai_l1::DecodedFeedback,8> states(std::uint64_t now) {
    std::array<vlai_l1::DecodedFeedback,8> a{};
    for(auto& v:a) { v.raw.host_receive_ns=now;v.status_nibble=1;v.mos_temperature_raw=30;v.rotor_temperature_raw=30; }
    return a;
}
int main() {
    try {
        auto s=spec();s.validate();std::array<double,8> q{};
        require(s.duration(false)==41,"duration");
        require(s.offset(9,false)==.5 && s.offset(18,false)==0 && s.offset(27,false)==-.5 && s.offset(36,false)==0,"return sequence");
        double previous=0;
        for(int i=0;i<=4100;++i) {
            const double v=s.offset(i*.01,false);
            require(std::abs(v)<=.5,"position envelope");
            if(i) require(std::abs(v-previous)/.01<=s.maximum_reference_speed,"sample speed");
            previous=v;
        }
        for(int kind=0;kind<9;++kind) {
            vlai_l1::AutoGuard guard(s);guard.set_reference(q);
            auto a=states(1);guard.check(1,a,q,true);
            a=states(2);
            if(kind==0) a[6].status_nibble=8;
            if(kind==1) a[6].position_rad=4/vlai_l1::kDegrees;
            if(kind==2) a[6].velocity_rad_s=1;
            if(kind==3) a[6].torque_nm=4;
            if(kind==4) a[6].mos_temperature_raw=71;
            if(kind==5) a[6].raw.host_receive_ns=3;
            if(kind==6) a[6].position_rad=std::numeric_limits<float>::quiet_NaN();
            if(kind==7) a[6].at_protocol_endpoint=true;
            if(kind==8) a[6].raw.host_receive_ns=0;
            rejects([&]{guard.check(2,a,q,true);});
            rejects([&]{guard.check(3,states(3),q,true);}); // Fault latch.
        }
        vlai_l1::AutoGuard tracking(s);tracking.set_reference(q);
        auto a=states(1);a[6].position_rad=1/vlai_l1::kDegrees;
        tracking.check(1,a,q,true);a=states(100000001);a[6].position_rad=1/vlai_l1::kDegrees;
        rejects([&]{tracking.check(100000001,a,q,true);});
        vlai_l1::AutoGuard stale(s);stale.set_reference(q);
        rejects([&]{stale.check(100000002,states(1),q,true);});
        auto bad=s;bad.maximum_reference_speed=.01;rejects([&]{bad.validate();});
        bad=s;bad.motors[1].send_id=1;rejects([&]{bad.validate();});
        bad=s;bad.capacity=8;rejects([&]{bad.validate();});
        vlai_l1::AutoGuard bounds(s);q[6]=89.9;rejects([&]{bounds.set_reference(q);});
        std::cout<<"PASS automatic pure core\n";return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
