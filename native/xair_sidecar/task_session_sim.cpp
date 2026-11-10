// Hardware-free protocol/process simulator. No SDK or CAN implementation is
// linked. Ideal feedback is not a model of motor, USB or mechanical behavior.
#include "automatic_input.hpp"
#include "task_session_io.hpp"
#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>

namespace {
volatile std::sig_atomic_t stopped=0;
void stop_handler(int) { stopped=1; }
void emit(const std::string& line) {
    if(::write(STDOUT_FILENO,line.data(),line.size())!=static_cast<ssize_t>(line.size()))
        throw std::runtime_error("sim output blocked/failed");
}
}
int main(int argc,char** argv) {
    unsigned enabled=0,disabled=0;
    std::uint64_t cycles=0,frames=0;
    std::string error;
    bool bounded_slew=false;
    try {
        if(argc==5 && std::string(argv[3])=="--plant" && std::string(argv[4])=="bounded-slew") bounded_slew=true;
        else if(argc!=3) throw std::runtime_error("usage: task_session_sim <task-spec> <initial-eight-degrees.txt> [--plant bounded-slew]");
        const auto input=vlai_l1::read_input(argv[1]);
        if(!input.task || input.spec.rate_hz!=100) throw std::runtime_error("sim requires typed100Hz task spec");
        std::array<double,8> initial{};
        std::ifstream pose(argv[2]);for(auto& q:initial) pose>>q;
        std::string extra;if(!pose || (pose>>extra)) throw std::runtime_error("invalid sim initial pose");
        vlai_l1::TaskPathGuard guard(input.spec,initial);
        vlai_l1::nonblocking(STDIN_FILENO);vlai_l1::nonblocking(STDOUT_FILENO);
        std::signal(SIGTERM,stop_handler);std::signal(SIGINT,stop_handler);
        std::signal(SIGHUP,stop_handler);std::signal(SIGPIPE,SIG_IGN);
        auto now=vlai_l1::task_now_ns();
        vlai_l1::TaskSession task(input.spec,*input.task,initial,now);
        std::array<vlai_l1::DecodedFeedback,8> feedback{};
        for(std::size_t i=0;i<8;++i) {
            auto& f=feedback[i];f.position_rad=initial[i]/vlai_l1::kDegrees;
            f.raw.host_receive_ns=now;f.status_nibble=input.spec.disabled_status;
            f.mos_temperature_raw=25;f.rotor_temperature_raw=25;
        }
        guard.check_initial(now,feedback,initial);
        ++enabled;
        emit("{\"state\":\"running\",\"simulation_only\":true,\"owner_pid\":"+std::to_string(getpid())+"}\n");
        const auto period=1000000000ULL/input.spec.rate_hz;
        auto deadline=now;auto previous=initial;auto last_reference=initial;auto previous_ns=now;
        std::string pending;
        while(!stopped) {
            now=vlai_l1::task_now_ns();
            if(now>deadline+input.spec.state_timeout_ns) throw std::runtime_error("sim control deadline missed");
            vlai_l1::task_input(task,pending);
            now=vlai_l1::task_now_ns();const auto target=task.advance(now);
            const auto dt=static_cast<double>(now-previous_ns)/1e9;
            for(std::size_t i=0;i<8;++i) {
                // Optional test plant follows the PREVIOUS sent reference with
                // a bounded kinematic slew. This is not motor/rigid-body physics
                // and never modifies production measured-speed protection.
                double step=(bounded_slew ? last_reference[i] : target[i])-previous[i];
                if(bounded_slew) {
                    const double maximum=.8*input.spec.motors[i].speed_deg_s*dt;
                    step=std::clamp(step,-maximum,maximum);
                }
                previous[i]+=step;
                auto& f=feedback[i];f.position_rad=previous[i]/vlai_l1::kDegrees;
                f.velocity_rad_s=step/vlai_l1::kDegrees/dt;
                f.raw.host_receive_ns=now;f.status_nibble=input.spec.enabled_status;
            }
            guard.check(now,feedback,task);
            ++cycles;frames+=8;last_reference=target;previous_ns=now;
            task.mark_sent();
            vlai_l1::task_output(task,feedback,target);
            deadline+=period;
            std::this_thread::sleep_until(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(deadline)));
        }
    } catch(const std::exception& e) {error=e.what();}
    if(enabled) ++disabled;
    // This is a fake lifecycle count, never physical disable verification.
    std::cout<<"{\"state\":\"simulation_terminal\",\"simulation_only\":true,\"owner_pid\":"<<getpid()
        <<",\"simulated_plant\":\""<<(bounded_slew?"kinematic_rate_limited_not_physics":"instantaneous_ideal_not_physics")<<"\""
        <<",\"enable_calls\":"<<enabled<<",\"disable_calls\":"<<disabled
        <<",\"command_frames\":"<<frames<<",\"cycles\":"<<cycles
        <<",\"signal_stop\":"<<(stopped?"true":"false")<<",\"fault\":"<<(!error.empty()?"true":"false")<<"}\n";
    if(!error.empty()) std::cerr<<"SIM FAULT "<<error<<'\n';
    return error.empty()?0:1;
}
