// Explicit experimental right-Follower owner. Never runs beside teleoperation.
#include "automatic_core.hpp"
#include "automatic_input.hpp"
#include "task_session_io.hpp"
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
#include <optional>
#include <atomic>
#include <fcntl.h>
#include <deque>
#include <cerrno>

namespace {
volatile std::sig_atomic_t stopped=0;
volatile std::sig_atomic_t finish_requested=0;
static_assert(std::atomic<bool>::is_always_lock_free);
std::atomic<bool> advance_requested{false};
void stop_handler(int) { stopped=1; }
void finish_handler(int) { finish_requested=1; }
void advance_handler(int) { advance_requested.store(true); }
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
using vlai_l1::Input;
using vlai_l1::read_input;
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
void output(const char* state, std::uint64_t stamp, double elapsed, const std::array<double,8>& q, const char* phase=nullptr) {
    std::cout<<"{\"state\":\""<<state<<"\",\"monotonic_ns\":"<<stamp<<",\"elapsed_seconds\":"<<elapsed<<",\"q_deg\":[";
    for (std::size_t i=0;i<8;++i) std::cout<<(i?",":"")<<std::setprecision(10)<<q[i];
    std::cout<<"]";
    if(phase) std::cout<<",\"phase\":\""<<phase<<"\"";
    std::cout<<"}"<<std::endl;
}
void orphan_link_down(const std::string& interface) {
    if (getppid()!=1) return;
    const int fd=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);
    if(fd<0) return;
    ifreq req{}; std::strncpy(req.ifr_name,interface.c_str(),IFNAMSIZ-1);
    if(ioctl(fd,SIOCGIFFLAGS,&req)==0) { req.ifr_flags &= ~IFF_UP; ioctl(fd,SIOCSIFFLAGS,&req); }
    close(fd);
}
using vlai_l1::nonblocking;
using vlai_l1::task_input;
using vlai_l1::task_output;

}
int main(int argc,char** argv) {
    // Paths/mode are supplied by the exclusive guarded Python supervisor.
    if(argc!=6) { std::cerr<<"usage: automatic <spec> <hold|sweep|paired|task|disable> <raw.csv> <samples.csv> <owner-pid>\n"; return 2; }
    try {
        if(geteuid()!=0) throw std::runtime_error("root required");
        const std::string mode=argv[2];
        if(mode!="hold" && mode!="sweep" && mode!="paired" && mode!="task" && mode!="disable") throw std::runtime_error("invalid mode");
        const auto owner=std::stoi(argv[5]);
        std::signal(SIGINT,stop_handler); std::signal(SIGTERM,stop_handler);
        std::signal(SIGHUP,stop_handler); std::signal(SIGPIPE,SIG_IGN);
        std::signal(SIGUSR1,finish_handler);
        std::signal(SIGUSR2,advance_handler);
        if(prctl(PR_SET_PDEATHSIG,SIGTERM)!=0 || getppid()!=owner) throw std::runtime_error("supervisor disappeared");
        const auto input=read_input(argv[1]); const auto& spec=input.spec;
        if((mode=="task")!=input.task.has_value() && mode!="disable") throw std::runtime_error("task mode/spec mismatch");
        if(mode=="task") { nonblocking(STDIN_FILENO);nonblocking(STDOUT_FILENO); }
        std::optional<vlai_l1::PairedSweep> paired;
        std::optional<vlai_l1::SettlingGate> settling;
        if(spec.handling) {
            if(mode!="paired" && mode!="disable") throw std::runtime_error("handling requires paired mode");
            settling.emplace(*spec.handling);
        }
        if(mode=="paired") paired.emplace(spec);
        vlai_l1::configure_fifo_priority_cap(spec.priority);
        vlai_l1::CanHealthMonitor health({input.interface});
        Handle sdk; sdk.open(input);
        std::array<std::uint32_t,8> ids{}; for(std::size_t i=0;i<8;++i) ids[i]=spec.motors[i].feedback.response_id;
        vlai_l1::PassiveFeedbackLog log(argv[3],ids,spec.capacity,spec.state_timeout_ns,
            static_cast<int>(1000/spec.rate_hz),getuid(),getgid(),mode=="task");
        log.start(input.interface);
        bool may_be_enabled=false, confirmed_disabled=false, success=false;
        bool requested_completion=false;
        double completion_elapsed=0;
        std::string error;
        std::deque<std::array<double,26>> samples;
        std::uint64_t evicted_samples=0;
        std::array<double,8> reference{};
        try {
            if(mode!="disable") {
            // A fresh complete refresh comes before the first enable or target.
            sdk.refresh(); std::this_thread::sleep_for(std::chrono::milliseconds(1000/spec.rate_hz));
            auto feedback=decode(log,spec);
            for(const auto& s:feedback) if(s.status_nibble!=spec.disabled_status)
                throw std::runtime_error("motor not disabled at startup");
            for(std::size_t i=0;i<8;++i) reference[i]=feedback[i].position_rad*vlai_l1::kDegrees;
            std::optional<vlai_l1::AutoGuard> guard;
            std::optional<vlai_l1::TaskPathGuard> task_guard;
            if(mode=="task") {
                task_guard.emplace(spec,reference);
                task_guard->check_initial(now_ns(),feedback,reference);
            } else {
                guard.emplace(spec);guard->set_reference(reference);
                guard->check(now_ns(),feedback,reference,false);
            }
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
                std::optional<vlai_l1::TaskSession> task;
                std::string pending_input;
                if(mode=="task") task.emplace(spec,*input.task,reference,start);
                const auto period=1'000'000'000ULL/spec.rate_hz;
                output("running",start,0,reference);
                if(paired) output("phase",start,0,reference,paired->name());
                while(!stopped) {
                    const auto now=now_ns();
                    if(now>deadline+spec.state_timeout_ns) throw std::runtime_error("control deadline missed");
                    const double elapsed=static_cast<double>(now-start)/1e9;
                    feedback=decode(log,spec);
                    if(paired) {
                        const std::string old_phase=paired->name();
                        const bool advance=advance_requested.exchange(false);
                        const bool stable=paired->settling() && settling && settling->update(now,feedback);
                        paired->step(elapsed,advance,stable);
                        if(paired->name()!=old_phase) {
                            if(settling) settling->reset();
                            output("phase",now,elapsed,reference,paired->name());
                        }
                        if(paired->complete()) { success=true; completion_elapsed=elapsed; break; }
                    } else if(!task && (elapsed>=spec.duration(mode=="hold") ||
                       (mode=="hold" && finish_requested && elapsed>=spec.hold_s))) {
                        success=true; completion_elapsed=elapsed;
                        requested_completion=mode=="hold" && finish_requested && elapsed<spec.hold_test_s;
                        break;
                    }
                    if(task) task_input(*task,pending_input);
                    const auto target=task ? task->advance(now_ns()) :
                        spec.target(reference,paired ? paired->local(elapsed) : elapsed,
                        paired ? paired->waiting() : mode=="hold");
                    if(task) task_guard->check(now_ns(),feedback,*task);
                    else guard->check(now_ns(),feedback,target,true,paired && spec.handling && paired->waiting());
                    if(now-last_health>=spec.health_poll_ms*1'000'000ULL) { health.check(); last_health=now; }
                    sdk.send(spec,target);
                    if(task) {task->mark_sent();task_output(*task,feedback,target);}
                    std::array<double,26> row{}; row[0]=static_cast<double>(now); row[1]=elapsed;
                    std::array<double,8> q{};
                    for(std::size_t i=0;i<8;++i) {
                        q[i]=feedback[i].position_rad*vlai_l1::kDegrees;
                        row[2+i]=q[i]; row[10+i]=target[i]; row[18+i]=feedback[i].torque_nm;
                    }
                    samples.push_back(row);
                    if(task && samples.size()>spec.capacity/8) {samples.pop_front();++evicted_samples;}
                    if(!task && now-last_output>=spec.health_poll_ms*1'000'000ULL) { output("running",now,elapsed,q,paired ? paired->name() : nullptr); last_output=now; }
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
        if(input.task) std::cout<<"{\"state\":\"task_samples\",\"evicted_rows\":"<<evicted_samples<<"}"<<std::endl;
        try { log.finish(confirmed_disabled && error.empty() && success); } catch(const std::exception& e) { success=false; error+="; log:"+std::string(e.what()); }
        orphan_link_down(input.interface);
        if(!success || !confirmed_disabled || !error.empty()) { std::cerr<<"FAIL "<<error<<std::endl; return 1; }
        std::cout<<"{\"state\":\"completed\",\"disabled_verified\":true";
        if(requested_completion) std::cout<<",\"completion_kind\":\"requested_hold\",\"elapsed_seconds\":"
                                        <<std::setprecision(17)<<completion_elapsed;
        if(paired) std::cout<<",\"completion_kind\":\"paired_sweep\",\"elapsed_seconds\":"
                           <<std::setprecision(17)<<completion_elapsed;
        std::cout<<"}"<<std::endl; return 0;
    } catch(const std::exception& e) { std::cerr<<"FAIL automatic startup:"<<e.what()<<std::endl; return 1; }
}
