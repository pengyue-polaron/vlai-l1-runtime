// Explicit experimental right-Follower owner. Never runs beside teleoperation.
#include "automatic_core.hpp"
#include "passive_feedback_log.hpp"
#include "runtime_safety.hpp"
#include <xarm_can_sdk.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <csignal>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <cstring>

namespace {
volatile std::sig_atomic_t stopped=0;
void stop_handler(int) { stopped=1; }
std::uint64_t now_ns() {
    timespec t{}; if (clock_gettime(CLOCK_MONOTONIC,&t)) throw std::runtime_error("clock failed");
    return static_cast<std::uint64_t>(t.tv_sec)*1'000'000'000+t.tv_nsec;
}
void check(int result, const char* operation) {
    if (result!=XARM_SDK_OK) {
        char message[512]{}; xarm_sdk_get_last_error(message,sizeof(message));
        throw std::runtime_error(std::string(operation)+":"+message);
    }
}
struct Input { std::string interface; vlai_l1::AutoSpec spec; };
Input read_input(const char* path) {
    std::ifstream f(path); Input input; std::string magic; auto& s=input.spec;
    f>>magic>>input.interface;
    if (magic!="VLAI_RIGHT_FOLLOWER_DIAGNOSTIC_V1" || input.interface.size()>=IFNAMSIZ)
        throw std::runtime_error("invalid diagnostic input");
    f>>s.amplitude_deg>>s.transition_s>>s.hold_s>>s.hold_test_s>>s.maximum_amplitude_deg
      >>s.maximum_reference_speed>>s.maximum_reference_acceleration>>s.cycles>>s.rate_hz>>s.capacity
      >>s.enabled_status>>s.disabled_status>>s.temperature>>s.state_timeout_ns>>s.tracking_timeout_ns
      >>s.shutdown_timeout_ns>>s.health_poll_ms>>s.priority;
    for (auto& m:s.motors) {
        unsigned slave=0;
        f>>m.feedback.name>>m.send_id>>m.feedback.response_id>>slave>>m.motor_type
         >>m.feedback.position_max_rad>>m.feedback.velocity_max_rad_s>>m.feedback.torque_max_nm
         >>m.kp>>m.kd>>m.min_deg>>m.max_deg>>m.tracking_deg>>m.speed_deg_s>>m.torque_nm>>m.excursion_deg;
        if (slave>15) throw std::runtime_error("invalid slave nibble");
        m.feedback.slave_id=static_cast<std::uint8_t>(slave);
    }
    std::string extra;
    if (!f || (f>>extra)) throw std::runtime_error("truncated/trailing diagnostic input");
    s.validate(); return input;
}
struct Handle {
    xarm_sdk_handle_t h=nullptr;
    ~Handle() { if(h) xarm_sdk_destroy(h); }
    void open(const Input& input) {
        check(xarm_sdk_create(input.interface.c_str(),1,&h),"create");
        std::array<int,7> types{}; std::array<std::uint32_t,7> send{},receive{};
        for (std::size_t i=0;i<7;++i) {
            types[i]=input.spec.motors[i].motor_type;
            send[i]=input.spec.motors[i].send_id; receive[i]=input.spec.motors[i].feedback.response_id;
        }
        check(xarm_sdk_init_arm_motors(h,types.data(),send.data(),receive.data(),7),"init arm");
        const auto& g=input.spec.motors[7];
        check(xarm_sdk_init_gripper_motor(h,g.motor_type,g.send_id,g.feedback.response_id),"init gripper");
    }
    void refresh() {
        check(xarm_sdk_refresh_all(h),"refresh");
        check(xarm_sdk_recv_all(h,2000),"receive");
    }
    void send(const vlai_l1::AutoSpec& spec,const std::array<double,8>& q) {
        std::array<xarm_sdk_mit_param_t,8> commands{};
        for (std::size_t i=0;i<8;++i) commands[i]={static_cast<float>(q[i]/vlai_l1::kDegrees),0,
            static_cast<float>(spec.motors[i].kp),static_cast<float>(spec.motors[i].kd),0};
        check(xarm_sdk_arm_mit_control(h,commands.data(),7),"arm command");
        check(xarm_sdk_gripper_mit_control(h,&commands[7]),"gripper command");
        check(xarm_sdk_recv_all(h,2000),"receive");
    }
    void close() { if(h) { auto old=h; h=nullptr; check(xarm_sdk_destroy(old),"destroy"); } }
};
std::array<vlai_l1::DecodedFeedback,8> decode(vlai_l1::PassiveFeedbackLog& log,const vlai_l1::AutoSpec& spec) {
    if (log.faulted()) throw std::runtime_error("raw logger fault");
    const auto records=log.latest_records(); std::array<vlai_l1::DecodedFeedback,8> values{};
    for (std::size_t i=0;i<8;++i) {
        const auto& r=records[i];
        values[i]=vlai_l1::decode_feedback(spec.motors[i].feedback,
            {r.can_id,r.payload,8,r.receive_monotonic_ns,r.can_fd});
    }
    return values;
}
void output(const char* state, std::uint64_t stamp, double elapsed, const std::array<double,8>& q) {
    std::cout<<"{\"state\":\""<<state<<"\",\"monotonic_ns\":"<<stamp<<",\"elapsed_seconds\":"<<elapsed<<",\"q_deg\":[";
    for (std::size_t i=0;i<8;++i) std::cout<<(i?",":"")<<std::setprecision(10)<<q[i];
    std::cout<<"]}"<<std::endl;
}
void orphan_link_down(const std::string& interface) {
    if (getppid()!=1) return;
    const int fd=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);
    if(fd<0) return;
    ifreq req{}; std::strncpy(req.ifr_name,interface.c_str(),IFNAMSIZ-1);
    if(ioctl(fd,SIOCGIFFLAGS,&req)==0) { req.ifr_flags &= ~IFF_UP; ioctl(fd,SIOCSIFFLAGS,&req); }
    close(fd);
}
}
int main(int argc,char** argv) {
    // Paths/mode are supplied by the exclusive guarded Python supervisor.
    if(argc!=6) { std::cerr<<"usage: automatic <spec> <hold|sweep|disable> <raw.csv> <samples.csv> <owner-pid>\n"; return 2; }
    try {
        if(geteuid()!=0) throw std::runtime_error("root required");
        const std::string mode=argv[2];
        if(mode!="hold" && mode!="sweep" && mode!="disable") throw std::runtime_error("invalid mode");
        const auto owner=std::stoi(argv[5]);
        std::signal(SIGINT,stop_handler); std::signal(SIGTERM,stop_handler);
        std::signal(SIGHUP,stop_handler); std::signal(SIGPIPE,SIG_IGN);
        if(prctl(PR_SET_PDEATHSIG,SIGTERM)!=0 || getppid()!=owner) throw std::runtime_error("supervisor disappeared");
        const auto input=read_input(argv[1]); const auto& spec=input.spec;
        vlai_l1::configure_fifo_priority_cap(spec.priority);
        vlai_l1::CanHealthMonitor health({input.interface});
        Handle sdk; sdk.open(input);
        std::array<std::uint32_t,8> ids{}; for(std::size_t i=0;i<8;++i) ids[i]=spec.motors[i].feedback.response_id;
        vlai_l1::PassiveFeedbackLog log(argv[3],ids,spec.capacity,spec.state_timeout_ns,
            static_cast<int>(1000/spec.rate_hz),getuid(),getgid());
        log.start(input.interface);
        bool may_be_enabled=false, confirmed_disabled=false, success=false;
        std::string error;
        std::vector<std::array<double,26>> samples;
        samples.reserve(static_cast<std::size_t>(spec.duration(mode=="hold")*spec.rate_hz)+2);
        std::array<double,8> reference{};
        try {
            if(mode!="disable") {
            // A fresh complete refresh comes before the first enable or target.
            sdk.refresh(); std::this_thread::sleep_for(std::chrono::milliseconds(1000/spec.rate_hz));
            auto feedback=decode(log,spec);
            for(const auto& s:feedback) if(s.status_nibble!=spec.disabled_status)
                throw std::runtime_error("motor not disabled at startup");
            for(std::size_t i=0;i<8;++i) reference[i]=feedback[i].position_rad*vlai_l1::kDegrees;
            vlai_l1::AutoGuard guard(spec); guard.set_reference(reference);
            guard.check(now_ns(),feedback,reference,false);
            health.check();
            vlai_l1::set_and_require_all_threads_fifo(spec.priority,2);
            if(stopped) throw std::runtime_error("stopped before enable");
                output("arming",now_ns(),0,reference);
                may_be_enabled=true; // Includes a partially completed enable call.
                check(xarm_sdk_enable_all(sdk.h),"enable");
                sdk.send(spec,reference);
                const auto enabled_at=now_ns();
                bool ready=false;
                while(now_ns()-enabled_at<spec.state_timeout_ns && !stopped) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    feedback=decode(log,spec); ready=true;
                    for(const auto& s:feedback) if(s.status_nibble!=spec.enabled_status || s.raw.host_receive_ns<enabled_at) ready=false;
                    if(ready) break;
                    sdk.send(spec,reference);
                }
                if(!ready || stopped) throw std::runtime_error("enable feedback did not become ready");
                const auto start=now_ns(); auto deadline=start; auto last_health=start; auto last_output=start;
                const auto period=1'000'000'000ULL/spec.rate_hz;
                output("running",start,0,reference);
                while(!stopped) {
                    const auto now=now_ns();
                    if(now>deadline+spec.state_timeout_ns) throw std::runtime_error("control deadline missed");
                    const double elapsed=static_cast<double>(now-start)/1e9;
                    if(elapsed>=spec.duration(mode=="hold")) { success=true; break; }
                    auto target=reference; target[6]+=spec.offset(elapsed,mode=="hold");
                    feedback=decode(log,spec);
                    guard.check(now_ns(),feedback,target,true);
                    if(now-last_health>=spec.health_poll_ms*1'000'000ULL) { health.check(); last_health=now; }
                    sdk.send(spec,target);
                    std::array<double,26> row{}; row[0]=static_cast<double>(now); row[1]=elapsed;
                    std::array<double,8> q{};
                    for(std::size_t i=0;i<8;++i) {
                        q[i]=feedback[i].position_rad*vlai_l1::kDegrees;
                        row[2+i]=q[i]; row[10+i]=target[i]; row[18+i]=feedback[i].torque_nm;
                    }
                    samples.push_back(row);
                    if(now-last_output>=spec.health_poll_ms*1'000'000ULL) { output("running",now,elapsed,q); last_output=now; }
                    deadline+=period;
                    timespec next{static_cast<time_t>(deadline/1'000'000'000ULL),static_cast<long>(deadline%1'000'000'000ULL)};
                    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&next,nullptr);
                }
                if(stopped) error="operator_or_supervisor_stop";
            } else { may_be_enabled=true; success=true; }
        } catch(const std::exception& e) { error=e.what(); }
        // Never lower the link before attempting disable and verifying new replies.
        try {
            auto stopped_position=reference;
            if(may_be_enabled) {
                const auto disabled_at=now_ns();
                check(xarm_sdk_disable_all(sdk.h),"disable");
                const auto until=now_ns()+spec.shutdown_timeout_ns;
                while(now_ns()<until) {
                    sdk.refresh(); std::this_thread::sleep_for(std::chrono::milliseconds(1000/spec.rate_hz));
                    const auto feedback=decode(log,spec); confirmed_disabled=true;
                    for(const auto& s:feedback) if(s.raw.host_receive_ns<=disabled_at || s.status_nibble!=spec.disabled_status) confirmed_disabled=false;
                    if(confirmed_disabled) {
                        for(std::size_t i=0;i<8;++i) stopped_position[i]=feedback[i].position_rad*vlai_l1::kDegrees;
                        break;
                    }
                }
                if(!confirmed_disabled) throw std::runtime_error("disable feedback not confirmed");
            }
            sdk.close();
            output(confirmed_disabled ? "disabled" : "not_enabled",now_ns(),0,stopped_position);
            health.check();
        } catch(const std::exception& e) { success=false; error+="; cleanup:"+std::string(e.what()); }
        log.stop_capture();
        std::ofstream states(argv[4]);
        states<<"monotonic_ns,elapsed_seconds";
        for(const char* prefix:{"q_","reference_","torque_"}) for(const auto& m:spec.motors) states<<','<<prefix<<m.feedback.name;
        states<<'\n'<<std::setprecision(17);
        for(const auto& row:samples) { for(std::size_t i=0;i<row.size();++i) states<<(i?",":"")<<row[i]; states<<'\n'; }
        states.close(); if(!states) { success=false; error+="; sample output failed"; }
        try { log.finish(confirmed_disabled && error.empty() && success); } catch(const std::exception& e) { success=false; error+="; log:"+std::string(e.what()); }
        orphan_link_down(input.interface);
        if(!success || !confirmed_disabled || !error.empty()) { std::cerr<<"FAIL "<<error<<std::endl; return 1; }
        std::cout<<"{\"state\":\"completed\",\"disabled_verified\":true}"<<std::endl; return 0;
    } catch(const std::exception& e) { std::cerr<<"FAIL automatic startup:"<<e.what()<<std::endl; return 1; }
}
