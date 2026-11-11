#include "task_session_core.hpp"
#include "task_session_recorded_policy.hpp"
#include "task_command_filter.hpp"
#include <functional>
#include <iostream>
void require(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
void rejects(const std::function<void()>& operation) {bool failed=false;try{operation();}catch(const std::exception&){failed=true;}require(failed,"expected rejection");}
vlai_l1::AutoSpec make_spec() {
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
        m.tracking_deg=2;m.speed_deg_s=i<7?45:15;m.torque_nm=3;m.excursion_deg=3;
    }
    s.motors[3].max_deg=120;
    return s;
}
int main() {
    try {
        const auto spec=make_spec();
        const vlai_l1::TaskLimits limits{100000000,5000000000,500000000};
        constexpr std::uint64_t epoch=1000000000;
        {
            auto held=recorded_policy::initial;
            vlai_l1::TaskCommandFilter filter(held,epoch,100);
            auto goal=held;goal[6]=27.06936980273856;
            require(filter.advance(goal,epoch)==held,"first command retains holding reference");
            auto first=filter.advance(goal,epoch+10000000);
            require(first[6]>held[6] && first[6]-held[6]<.11,"recorded handover step smoothed");
            require(filter.advance(goal,epoch+10000000)==first,"same timestamp cannot advance twice");
            auto previous=first;
            for(unsigned tick=2;tick<=1000;++tick) {
                auto current=filter.advance(goal,epoch+tick*10000000ULL);
                require(current[6]>=previous[6] && current[6]<=goal[6],"no handover overshoot");
                require(current[6]-previous[6]<=.2000000001,"command speed bounded");
                previous=current;
            }
            require(std::abs(previous[6]-goal[6])<1e-9,"hold converges to model target");
            goal[6]=-80;
            auto reversed=filter.advance(goal,epoch+11000000000ULL);
            require(previous[6]-reversed[6]<=.2000000001,"delayed/reversed request cannot jump");
            rejects([&]{filter.advance(goal,epoch);});
        }
        auto initial=recorded_policy::initial;
        vlai_l1::TaskSession task(spec,limits,initial,epoch);
        std::uint64_t now=epoch,sequence=0;
        const auto send=[&](const char* op,double duration,std::array<double,8> q={}) {
            task.accept({++sequence,now,now,op,duration,q},now);return sequence;
        };
        send("start",.2,initial);
        for(unsigned i=0;i<20;++i) {now+=10000000;send("heartbeat",0);}
        require(!task.moving(),"start ends");
        now+=10000000;
        const auto policy_epoch=now;
        std::array<std::uint64_t,32> sequences{},applied_at{};
        sequences[0]=send("target",1.0/30,recorded_policy::actions[0]);
        applied_at[0]=now;
        require(task.reference()==recorded_policy::actions[0],"first real target is direct, not interpolated");
        task.mark_sent();
        ++now;sequences[1]=send("target",1.0/30,recorded_policy::actions[1]);
        unsigned queued=2,applied=1;
        for(unsigned tick=0;tick<120 && (task.moving() || applied<32);++tick) {
            now=policy_epoch+(tick+1)*10000000;
            task.advance(now);
            if(!task.queued() && queued<32)
                sequences[queued]=send("target",1.0/30,recorded_policy::actions[queued]),++queued;
            else send("heartbeat",0);
            task.advance(now);task.mark_sent();
            if(applied<32 && task.applied()==sequences[applied]) {
                require(task.reference()==recorded_policy::actions[applied],"unaltered recorded action");
                applied_at[applied++]=now;
            }
        }
        require(applied==32 && task.completed()==sequences[31],"all32 real targets complete");
        require(applied_at[31]-policy_epoch>=1033333333ULL &&
                applied_at[31]-policy_epoch<1043333334ULL,"absolute30Hz clock, no cumulative40ms rounding");
        for(unsigned i=1;i<32;++i)
            require(applied_at[i]-applied_at[i-1]==30000000 || applied_at[i]-applied_at[i-1]==40000000,
                    "100Hz emitter uses30/40ms spacing without dropped points");
        require(task.reference()==recorded_policy::actions[31],"entire real chunk retained");
        // Normal stream completion persists at its exact final reference.
        for(unsigned i=0;i<200;++i) {now+=10000000;send("heartbeat",0);}
        require(task.reference()==recorded_policy::actions[31] && !task.moving(),"persistent hold");
        now+=10000000;send("target",1.0/30,recorded_policy::actions[31]);
        task.mark_sent();
        require(task.policy_gap_count()==1 && task.policy_gap_ns()>1900000000,"underrun reported, owner retains target");
        for(unsigned i=0;i<4;++i) {now+=10000000;send("heartbeat",0);}
        // Return remains a smooth transit: sufficiently long return is valid,
        // while a diagnostic-speed violating return still fails.
        rejects([&]{task.accept({sequence+1,now+2,now+2,"return",.1,{}},now+2);});
        now+=10000000;send("return",30);
        for(unsigned i=0;i<300;++i) {now+=100000000;send("heartbeat",0);}
        require(task.reference()==initial && !task.moving(),"smooth exact return to immutable start");
        // Parking uses a distinct smooth target and never replaces saved_start_.
        auto parked=initial;parked[3]-=2;
        now+=10000000;
        rejects([&]{task.accept({sequence+1,now,now,"park",.1,parked},now);});
        auto outside=parked;outside[3]=121;
        rejects([&]{task.accept({sequence+1,now,now,"park",10,outside},now);});
        const auto parsed=vlai_l1::parse_task_command("VLAI_TASK_V1 1 10 10 park 5 0 0 0 0 0 0 0 0");
        require(parsed.operation=="park","park protocol parsed");
        send("park",5,parked);
        for(unsigned i=0;i<25;++i) {now+=100000000;send("heartbeat",0);}
        require(task.moving() && std::abs(task.reference()[3]-(initial[3]-1))<1e-9,
                "parking interpolates smoothly at midpoint");
        for(unsigned i=0;i<30;++i) {now+=100000000;send("heartbeat",0);}
        require(task.reference()==parked && !task.moving(),"parking persists at its own pose");
        now+=10000000;send("return",5);
        for(unsigned i=0;i<50;++i) {now+=100000000;send("heartbeat",0);}
        require(task.reference()==initial && !task.moving(),"parking preserves original start");
        auto fresh=[&] {return vlai_l1::TaskSession(spec,limits,initial,epoch);};
        {auto t=fresh();auto out=initial;out[3]=121;rejects([&]{t.accept({1,epoch,epoch,"start",240,out},epoch);});}
        {auto t=fresh();t.accept({1,epoch,epoch,"start",.01,initial},epoch);
         t.accept({2,epoch+20000000,epoch+20000000,"heartbeat",0,{}},epoch+20000000);
         rejects([&]{t.accept({3,epoch+20000001,epoch+20000001,"target",.2,initial},epoch+20000001);});}
        {auto t=fresh();rejects([&]{t.close_input();});rejects([&]{t.advance(epoch+limits.heartbeat_ns+1);});}
        // Planned tail points keep the original chunk observation timestamp;
        // only their command source must be fresh. No per-point re-expiry.
        {
            auto short_age=limits;short_age.observation_age_ns=1;
            vlai_l1::TaskSession t(spec,short_age,initial,epoch);
            t.accept({1,epoch,epoch,"start",.01,initial},epoch);
            auto stamp=epoch+20000000;
            t.accept({2,stamp,stamp,"heartbeat",0,{}},stamp);
            ++stamp;t.accept({3,stamp,epoch,"target",1.0/30,recorded_policy::actions[0]},stamp);
            t.mark_sent();
            ++stamp;t.accept({4,stamp,epoch,"target",1.0/30,recorded_policy::actions[1]},stamp);
            stamp+=70000000;t.advance(stamp);t.mark_sent();
            require(t.applied()==4 && t.reference()==recorded_policy::actions[1],"queued old observation retained");
            require(t.policy_gap_count()==1 && t.moving(),"late pending point resumes without catchup/fault");
            rejects([&]{t.accept({5,stamp-100000001,epoch,"target",1.0/30,initial},stamp);});
        }
        // Multiple drained commands around a deadline cannot retire sample N
        // before the one real SDK send at the end of that same control tick.
        {
            auto t=fresh();t.accept({1,epoch,epoch,"start",.01,initial},epoch);
            auto stamp=epoch+20000000;
            t.accept({2,stamp,stamp,"target",1.0/30,recorded_policy::actions[0]},stamp);t.mark_sent();
            t.accept({3,stamp+1,stamp+1,"target",1.0/30,recorded_policy::actions[1]},stamp+1);
            const auto near_second_end=stamp+66666000;
            t.advance(near_second_end);
            require(t.active_sequence()==3 && t.applied()==2,"unsent next sample not acknowledged");
            t.accept({4,near_second_end+1,near_second_end+1,"target",1.0/30,recorded_policy::actions[2]},near_second_end+1);
            t.accept({5,near_second_end+2000,near_second_end+2000,"heartbeat",0,{}},near_second_end+2000);
            t.advance(near_second_end+3000);
            require(t.active_sequence()==3 && t.applied()==2 && t.reference()==recorded_policy::actions[1],"drained commands cannot skip unsent sample");
            t.mark_sent();require(t.applied()==3,"actual send acknowledges sample N");
            t.advance(near_second_end+10000000);t.mark_sent();
            require(t.applied()==4 && t.reference()==recorded_policy::actions[2],"N+1 follows after N was sent");
        }
        // Full physical guards remain active in task mode. Static feedback
        // here is a guard unit fixture, not simulated motion of the real chunk.
        auto check_fault=[&](const std::function<void(vlai_l1::DecodedFeedback&)>& corrupt) {
            auto t=fresh();vlai_l1::TaskPathGuard guard(spec,initial);
            std::array<vlai_l1::DecodedFeedback,8> feedback{};
            for(unsigned i=0;i<8;++i) {
                feedback[i].position_rad=initial[i]/vlai_l1::kDegrees;
                feedback[i].raw.host_receive_ns=epoch;feedback[i].status_nibble=1;
            }
            corrupt(feedback[0]);rejects([&]{guard.check(epoch,feedback,t);});
        };
        check_fault([](auto& f){f.velocity_rad_s=45.1/vlai_l1::kDegrees;});
        check_fault([](auto& f){f.torque_nm=3.01;});
        check_fault([](auto& f){f.mos_temperature_raw=71;});
        check_fault([](auto& f){f.position_rad=91/vlai_l1::kDegrees;});
        check_fault([](auto& f){f.status_nibble=0;});
        check_fault([](auto& f){f.raw.host_receive_ns=1;});
        rejects([]{vlai_l1::parse_task_command("VLAI_TASK_V1 -1 10 10 hold 0 0 0 0 0 0 0 0 0");});
        std::cout<<"PASS: recorded32 unchanged at30Hz; hard guards; gap reporting; smooth return; persistent hold\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
