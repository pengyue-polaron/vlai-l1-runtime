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
    s.health_poll_ms=100;s.priority=20;s.active_motor=6;
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
        // Operator-requested automatic speed cap remains a hard stop, not a bypass.
        auto faster=s;
        for(unsigned i=0;i<7;++i) faster.motors[i].speed_deg_s=45;
        vlai_l1::AutoGuard faster_guard(faster);faster_guard.set_reference(q);
        auto short_peak=states(10000000);short_peak[4].velocity_rad_s=15.530725/vlai_l1::kDegrees;
        faster_guard.check(10000000,short_peak,q,true);
        auto below_cap=states(20000000);below_cap[4].velocity_rad_s=44.9/vlai_l1::kDegrees;
        faster_guard.check(20000000,below_cap,q,true);
        auto above_cap=states(30000000);above_cap[4].velocity_rad_s=45.1/vlai_l1::kDegrees;
        rejects([&]{faster_guard.check(30000000,above_cap,q,true);});
        rejects([&]{faster_guard.check(40000000,states(40000000),q,true);});
        // J7's revised 2-degree limit tolerates the recorded lag, but still latches.
        auto j7_tracking_policy=s;j7_tracking_policy.motors[6].tracking_deg=2;
        vlai_l1::AutoGuard tracking_guard(j7_tracking_policy);tracking_guard.set_reference(q);
        auto recorded_lag=states(10000000);recorded_lag[6].position_rad=-.844009/vlai_l1::kDegrees;
        tracking_guard.check(10000000,recorded_lag,q,true);
        auto under_limit=states(110000000);under_limit[6].position_rad=-1.9/vlai_l1::kDegrees;
        tracking_guard.check(110000000,under_limit,q,true);
        auto over_limit=states(210000000);over_limit[6].position_rad=-2.1/vlai_l1::kDegrees;
        tracking_guard.check(210000000,over_limit,q,true);
        for(auto& state:over_limit) state.raw.host_receive_ns=310000000;
        rejects([&]{tracking_guard.check(310000000,over_limit,q,true);});
        rejects([&]{tracking_guard.check(320000000,states(320000000),q,true);});
        auto multi=s;
        multi.active_motor=8;multi.amplitude_deg=16;multi.maximum_amplitude_deg=16;
        multi.transition_s=40;
        for(auto& motor:multi.motors) motor.excursion_deg=18;
        multi.waypoints={
            {}, {0,10,-10,16,-10,10,10,0}, {0,10,-10,0,10,-10,-10,0},
            {0,0,-10,0,-10,10,-10,0}, {0,10,10,0,-10,-10,-10,0},
            {0,0,-10,16,-10,-10,-10,0}, {0,0,10,16,10,10,-10,0}, {}};
        multi.validate();
        require(multi.duration(false)==320,"multi duration");
        require(multi.target(q,25,false)==std::array<double,8>{0,5,-5,8,-5,5,5,0},"multi midpoint");
        require(multi.target(q,320,false)==q,"multi closure");
        require(multi.target(q,25,true)==q,"multi hold moved");
        for(unsigned leg=0;leg<7;++leg) {
            require(multi.target(q,45+45*leg,false)==multi.waypoints[leg+1],"multi waypoint order");
            const auto midpoint=multi.target(q,25+45*leg,false);
            for(unsigned i=0;i<8;++i)
                require(midpoint[i]==(multi.waypoints[leg][i]+multi.waypoints[leg+1][i])/2,"multi segment blend");
        }
        auto previous_multi=multi.target(q,0,false);
        for(unsigned tick=1;tick<=32000;++tick) {
            const auto point=multi.target(q,tick*.01,false);
            require(point[0]==0 && point[7]==0,"multi fixed base/gripper");
            for(unsigned i=0;i<8;++i) {
                require(std::abs(point[i])<=16.00000001,"multi envelope");
                require(std::abs(point[i]-previous_multi[i])/.01<=1.00000001,"multi speed");
            }
            previous_multi=point;
        }
        for(unsigned fault=0;fault<12;++fault) {
            auto bad=multi;
            if(fault==0) bad.waypoints[1][7]=1;
            if(fault==1) bad.waypoints.front()[1]=1;
            if(fault==2) bad.waypoints.back()[1]=1;
            if(fault==3) bad.waypoints[1][1]=std::numeric_limits<double>::quiet_NaN();
            if(fault==4) bad.transition_s=20; // Each offset fits; cross-posture delta does not.
            if(fault==5) bad.motors[1].excursion_deg=5;
            if(fault==6) bad.capacity=10000;
            if(fault==7) bad.active_motor=3;
            if(fault==8) bad.waypoints.resize(2);
            if(fault==9) bad.handling.emplace();
            if(fault==10) for(auto& p:bad.waypoints) for(unsigned i=2;i<8;++i) p[i]=0;
            if(fault==11) bad.waypoints.resize(65);
            rejects([&]{bad.validate();});
        }
        rejects([&]{vlai_l1::PairedSweep pair(multi);});
        vlai_l1::AutoGuard multi_guard(multi);multi_guard.set_reference(q);
        auto multi_q=multi.target(q,25,false);
        auto multi_feedback=states(10000000);
        for(unsigned i=0;i<8;++i) multi_feedback[i].position_rad=multi_q[i]/vlai_l1::kDegrees;
        multi_guard.check(10000000,multi_feedback,multi_q,true);
        multi_feedback=states(20000000);multi_feedback[2].torque_nm=4;
        rejects([&]{multi_guard.check(20000000,multi_feedback,q,true);});
        rejects([&]{multi_guard.check(30000000,states(30000000),q,true);});
        // Every future point is checked before enable, including a non-wrist joint.
        vlai_l1::AutoGuard multi_bounds(multi);
        auto near_bound=q;near_bound[1]=85;
        rejects([&]{multi_bounds.set_reference(near_bound);});
        auto manual=s;
        manual.hold_test_s=200;
        manual.handling.emplace();
        auto& h=*manual.handling;
        h.hard_speed_deg_s={15,15,15,15,45,45,45,15};
        h.speed_persistence_ns=100000000;h.settle_ns=2000000000;h.settle_timeout_ns=20000000000;
        h.settle_speed_deg_s=5;h.settle_span_deg=.05;
        manual.validate();
        // Recorded J5 trigger is tolerated only in stationary manual context.
        vlai_l1::AutoGuard brief(manual);brief.set_reference(q);
        auto disturbed=states(10000000);
        disturbed[4].position_rad=.3497108/vlai_l1::kDegrees;
        disturbed[4].velocity_rad_s=32.3207/vlai_l1::kDegrees;
        brief.check(10000000,disturbed,q,true,true);
        brief.check(20000000,states(20000000),q,true,true);
        for(int fault=0;fault<8;++fault) {
            vlai_l1::AutoGuard guarded(manual);guarded.set_reference(q);
            auto a=disturbed;
            if(fault==0) { rejects([&]{guarded.check(10000000,a,q,true);}); continue; }
            if(fault==1) a[4].velocity_rad_s=46/vlai_l1::kDegrees;
            if(fault==2) a[4].position_rad=3.1/vlai_l1::kDegrees;
            if(fault==3) a[4].torque_nm=4;
            if(fault==4) a[4].mos_temperature_raw=71;
            if(fault==5) a[4].raw.host_receive_ns=0;
            if(fault==6) a[4].status_nibble=0;
            if(fault==7) { a[0].velocity_rad_s=16/vlai_l1::kDegrees; }
            rejects([&]{guarded.check(10000000,a,q,true,true);});
            rejects([&]{guarded.check(20000000,states(20000000),q,true,true);});
        }
        vlai_l1::AutoGuard sustained(manual);sustained.set_reference(q);
        sustained.check(10000000,disturbed,q,true,true);
        auto prolonged=states(110000000);prolonged[4].velocity_rad_s=32/vlai_l1::kDegrees;
        rejects([&]{sustained.check(110000000,prolonged,q,true,true);});
        vlai_l1::AutoGuard moved_target(manual);moved_target.set_reference(q);
        auto target=q;target[6]=.01;
        rejects([&]{moved_target.check(10000000,states(10000000),target,true,true);});
        vlai_l1::AutoGuard manual_tracking(manual);manual_tracking.set_reference(q);
        auto displaced=states(10000000);displaced[6].position_rad=1/vlai_l1::kDegrees;
        manual_tracking.check(10000000,displaced,q,true,true);
        displaced[6].raw.host_receive_ns=110000000;
        for(auto& v:displaced) v.raw.host_receive_ns=110000000;
        rejects([&]{manual_tracking.check(110000000,displaced,q,true,true);});
        vlai_l1::SettlingGate gate(h);
        for(unsigned tick=1;tick<=201;++tick) {
            const auto ns=tick*10000000ULL; auto a=states(ns);
            a[4].position_rad=(tick%2 ? .02186 : 0)/vlai_l1::kDegrees;
            a[4].velocity_rad_s=2.94/vlai_l1::kDegrees;
            require(gate.update(ns,a)==(tick==201),"settling dwell/encoder noise");
        }
        gate.reset();require(!gate.update(1,states(1)),"reset retained dwell");
        auto moving=states(2000000001);moving[4].position_rad=.06/vlai_l1::kDegrees;
        require(!gate.update(2000000001,moving),"position drift accepted");
        moving=states(4000000001);moving[4].velocity_rad_s=6/vlai_l1::kDegrees;
        require(!gate.update(4000000001,moving),"velocity disturbance accepted");
        require(!gate.update(5000000001,states(5000000001)),"velocity did not reset dwell");
        require(gate.update(7000000001,states(7000000001)),"settling never completed");
        rejects([&]{gate.update(7000000001,states(7000000001));});
        vlai_l1::PairedSweep gated(manual);
        gated.step(41,false);gated.step(42,true);
        require(gated.waiting() && gated.settling(),"ack must start stationary settling");
        require(manual.target(q,gated.local(43),gated.waiting())==q,"settling reference moved");
        gated.step(44,false,true);require(gated.phase()==2 && !gated.waiting(),"settled sweep not started");
        vlai_l1::PairedSweep settling_timeout(manual);
        settling_timeout.step(41,false);settling_timeout.step(42,true);
        rejects([&]{settling_timeout.step(62,false,true);}); // Timeout wins over simultaneous stable.
        vlai_l1::PairedSweep duplicate_ack(manual);
        duplicate_ack.step(41,false);duplicate_ack.step(42,true);
        rejects([&]{duplicate_ack.step(43,true);});
        vlai_l1::PairedSweep premature_stable(manual);
        premature_stable.step(41,false);premature_stable.step(42,true);
        rejects([&]{premature_stable.step(43,false,true);});
        rejects([&]{vlai_l1::PairedSweep too_short(s);});
        auto paired_spec=s;paired_spec.hold_test_s=200;
        vlai_l1::PairedSweep paired(paired_spec);
        paired.step(0,false);paired.step(41,false);
        require(paired.waiting() && paired.phase()==1,"paired attach wait");
        require(paired_spec.target(q,paired.local(41),paired.waiting())==q,"paired waiting moved");
        paired.step(42,true);require(paired.phase()==2 && paired.local(42)==0,"paired loaded start");
        paired.step(83,false);require(paired.waiting() && paired.phase()==3,"paired remove wait");
        paired.step(84,true);paired.step(125,false);require(paired.complete(),"paired completion");
        rejects([&]{paired.step(126,false);});
        vlai_l1::PairedSweep early(paired_spec);rejects([&]{early.step(1,true);});
        vlai_l1::PairedSweep capped(paired_spec);rejects([&]{capped.step(200,false);});
        vlai_l1::PairedSweep late(paired_spec);late.step(41,false);rejects([&]{late.step(121,true);});
        vlai_l1::PairedSweep backwards(paired_spec);backwards.step(1,false);rejects([&]{backwards.step(.5,false);});
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
        for(unsigned active : {3U,5U,6U}) {
            auto selected=s;selected.active_motor=active;selected.validate();
            std::array<double,8> reference{1,2,3,4,5,6,7,8};
            for(int tick=0;tick<=4100;++tick) {
                const auto target=selected.target(reference,tick*.01,false);
                require(selected.target(reference,tick*.01,true)==reference,"hold changed reference");
                for(unsigned i=0;i<8;++i) {
                    if(i!=active) require(target[i]==reference[i],"unselected motor changed");
                    else require(std::abs(target[i]-reference[i])<=.5,"selected envelope");
                }
            }
            require(selected.target(reference,9,false)[active]==reference[active]+.5,"selected positive target");
            require(selected.target(reference,27,false)[active]==reference[active]-.5,"selected negative target");
            std::array<double,8> near_limit{};near_limit[active]=89.9;
            vlai_l1::AutoGuard selected_bounds(selected);
            rejects([&]{selected_bounds.set_reference(near_limit);});
            selected.motors[active].excursion_deg=.1;
            rejects([&]{selected.validate();});
        }
        for(unsigned active : {0U,1U,2U,4U,7U,8U}) {
            bad=s;bad.active_motor=active;rejects([&]{bad.validate();});
        }
        bad=s;bad.motors[1].send_id=1;rejects([&]{bad.validate();});
        bad=s;bad.capacity=8;rejects([&]{bad.validate();});
        vlai_l1::AutoGuard bounds(s);q[6]=89.9;rejects([&]{bounds.set_reference(q);});
        std::cout<<"PASS automatic pure core\n";return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
