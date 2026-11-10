#pragma once
// Hardware-free, bounded resident-task protocol and whole-segment validation.
#include "automatic_core.hpp"
#include <algorithm>
#include <sstream>
#include <charconv>

namespace vlai_l1 {
struct TaskLimits {
    std::uint64_t command_age_ns=0, observation_age_ns=0, heartbeat_ns=0;
    void validate() const {
        if(!command_age_ns || !observation_age_ns || !heartbeat_ns)
            throw std::invalid_argument("invalid task limits");
    }
};
struct TaskCommand {
    std::uint64_t sequence=0, source_ns=0, observation_ns=0;
    std::string operation;
    double duration_s=0;
    std::array<double,8> target{};
};
inline TaskCommand parse_task_command(const std::string& line) {
    if(line.empty() || line.size()>1024) throw std::runtime_error("task command size");
    std::istringstream in(line);std::string magic,extra,sequence,source,observation;TaskCommand c;
    in>>magic>>sequence>>source>>observation>>c.operation>>c.duration_s;
    const auto integer=[](const std::string& token) {
        std::uint64_t value=0;
        const auto parsed=std::from_chars(token.data(),token.data()+token.size(),value);
        if(parsed.ec!=std::errc{} || parsed.ptr!=token.data()+token.size() || !value)
            throw std::runtime_error("invalid task unsigned integer");
        return value;
    };
    c.sequence=integer(sequence);c.source_ns=integer(source);c.observation_ns=integer(observation);
    for(auto& q:c.target) in>>q;
    if(!in || (in>>extra) || magic!="VLAI_TASK_V1" || !c.sequence || !c.source_ns ||
       !c.observation_ns || !std::isfinite(c.duration_s) ||
       (c.operation!="start" && c.operation!="target" && c.operation!="return" &&
        c.operation!="hold" && c.operation!="heartbeat"))
        throw std::runtime_error("invalid task command");
    for(double q:c.target) if(!std::isfinite(q)) throw std::runtime_error("nonfinite task target");
    return c;
}
class TaskSession {
public:
    TaskSession(const AutoSpec& spec, TaskLimits limits, std::array<double,8> initial,
                std::uint64_t now):spec_(spec),limits_(limits),initial_(initial),q_(initial),last_now_(now),last_input_(now) {
        spec_.validate();limits_.validate();check_endpoint(initial_);
    }
    void accept(const TaskCommand& c,std::uint64_t now) {
        advance(now);
        if(c.sequence<=accepted_ || c.source_ns<=last_source_ || c.source_ns>now ||
           now-c.source_ns>limits_.command_age_ns || !c.observation_ns || c.observation_ns>c.source_ns ||
           (c.operation!="target" && now-c.observation_ns>limits_.observation_age_ns))
            throw std::runtime_error("stale/regressed task command");
        if(c.operation=="heartbeat" || c.operation=="hold") {
            if(c.duration_s!=0 || c.target!=std::array<double,8>{})
                throw std::runtime_error("task control command requires zero payload");
            if(c.operation=="hold") { pending_.reset(); hold_sequence_=c.sequence; }
        } else {
            if(hold_sequence_) throw std::runtime_error("task hold barrier pending");
            if(c.operation=="start" && (saved_start_ || moving_ || pending_))
                throw std::runtime_error("task start already defined/busy");
            if(c.operation!="start" && !saved_start_) throw std::runtime_error("task start missing");
            if(pending_) throw std::runtime_error("task queue full");
            TaskCommand planned=c;
            if(c.operation=="return") {
                if(c.target!=std::array<double,8>{}) throw std::runtime_error("return requires zero target payload");
                planned.target=*saved_start_;
            }
            validate_segment(moving_ ? destination_ : q_,planned);
            if(c.operation=="start") saved_start_=c.target;
            if(moving_) pending_=planned;
            else {
                // Normal pending samples use the absolute clock below. An
                // underrun holds the last setpoint; resume never catches up.
                auto scheduled=static_cast<long double>(now);
                if(c.operation=="target" && operation_=="target") {
                    record_gap(now,ended_);
                }
                begin(planned,scheduled);
            }
        }
        accepted_=c.sequence;last_source_=c.source_ns;last_input_=now;
        if(!moving_ && hold_sequence_) {completed_=hold_sequence_;hold_sequence_=0;operation_="hold";}
    }
    void close_input() { throw std::runtime_error("task producer EOF"); }
    const std::array<double,8>& advance(std::uint64_t now) {
        if(now<last_now_) throw std::runtime_error("regressed task time");
        last_now_=now;
        if(now-last_input_>limits_.heartbeat_ns) throw std::runtime_error("task producer heartbeat expired");
        if(moving_) {
            const auto elapsed=static_cast<long double>(now)-began_;
            const double u=std::min(1.0,static_cast<double>(elapsed/1e9L)/duration_);
            if(operation_=="target") q_=destination_;
            else {
                const double blend=u*u*u*(10-15*u+6*u*u);
                for(std::size_t i=0;i<8;++i) q_[i]=origin_[i]+(destination_[i]-origin_[i])*blend;
            }
            // Input draining can call advance several times before the one
            // SDK write of a control tick. Never retire an un-emitted sample.
            if(static_cast<long double>(now)>=std::round(ended_) &&
               (operation_!="target" || active_sent_)) {
                q_=destination_;moving_=false;completed_=active_;
                if(pending_) {
                    auto next=*pending_;pending_.reset();
                    auto scheduled=next.operation=="target" ? ended_ : static_cast<long double>(now);
                    if(next.operation=="target" && static_cast<long double>(now)>=ended_+sample_ns_) {
                        record_gap(now,ended_);scheduled=now;
                    }
                    if(next.operation=="target" ||
                       (now>=next.observation_ns && now-next.observation_ns<=limits_.observation_age_ns))
                        begin(next,scheduled);
                    else throw std::runtime_error("queued task observation expired");
                } else if(hold_sequence_) {completed_=hold_sequence_;hold_sequence_=0;operation_="hold";}
            }
        }
        return q_;
    }
    bool moving() const { return moving_; }
    bool queued() const { return pending_.has_value(); }
    std::uint64_t accepted() const { return accepted_; }
    std::uint64_t completed() const { return completed_; }
    void mark_sent() { emitted_sequence_=active_;active_sent_=true; }
    std::uint64_t applied() const { return emitted_sequence_; }
    std::uint64_t active_sequence() const { return active_; }
    std::uint64_t observation_ns() const { return active_observation_; }
    std::uint64_t policy_gap_ns() const { return gap_ns_; }
    std::uint64_t policy_gap_count() const { return gap_count_; }
    std::uint64_t pending_sequence() const { return pending_ ? pending_->sequence : 0; }
    const char* phase() const { return moving_ ? "moving" : "holding"; }
    const std::array<double,8>& reference() const { return q_; }

private:
    void record_gap(std::uint64_t now,long double scheduled) {
        if(static_cast<long double>(now)>scheduled) {
            gap_ns_=static_cast<std::uint64_t>(static_cast<long double>(now)-scheduled);
            ++gap_count_;
        }
    }
    void check_endpoint(const std::array<double,8>& q) const {
        for(std::size_t i=0;i<8;++i) {
            const auto& m=spec_.motors[i];
            if(!std::isfinite(q[i]) || q[i]<m.min_deg || q[i]>m.max_deg)
                throw std::runtime_error("task target hard bounds:"+m.feedback.name);
        }
    }
    void validate_segment(const std::array<double,8>& from,const TaskCommand& c) const {
        check_endpoint(c.target);
        if(!std::isfinite(c.duration_s) || c.duration_s<=0)
            throw std::runtime_error("task segment duration");
        if(c.operation=="target") {
            if(std::abs(c.duration_s-1.0/30)>1e-12) throw std::runtime_error("policy target requires nominal30Hz duration");
            return; // Direct PD setpoint, not a diagnostic quintic trajectory.
        }
        for(std::size_t i=0;i<8;++i) {
            const double delta=std::abs(c.target[i]-from[i]);
            if(1.875*delta/c.duration_s>spec_.maximum_reference_speed ||
               (10*std::sqrt(3.0)/3)*delta/(c.duration_s*c.duration_s)>spec_.maximum_reference_acceleration)
                throw std::runtime_error("task segment speed/acceleration:"+spec_.motors[i].feedback.name);
        }
    }
    void begin(const TaskCommand& c,long double scheduled) {
        origin_=q_;destination_=c.target;duration_=c.duration_s;began_=scheduled;active_=c.sequence;
        ended_=began_+(c.operation=="target" ? sample_ns_ : std::round(static_cast<long double>(duration_)*1e9L));
        active_observation_=c.observation_ns;moving_=true;
        operation_=c.operation;
        active_sent_=false;
        if(operation_=="target") q_=destination_;
    }
    AutoSpec spec_;TaskLimits limits_;
    std::array<double,8> initial_,q_,origin_{},destination_{};
    std::optional<std::array<double,8>> saved_start_;
    std::optional<TaskCommand> pending_;
    std::uint64_t last_now_,last_input_,last_source_=0,accepted_=0,completed_=0,active_=0,hold_sequence_=0,active_observation_=0;
    std::uint64_t gap_ns_=0,gap_count_=0;
    std::uint64_t emitted_sequence_=0;
    static constexpr long double sample_ns_=1000000000.0L/30;
    long double began_=0,ended_=0;
    double duration_=0;
    bool moving_=false;
    bool active_sent_=false;
    std::string operation_;
};
// Task motion uses original hard joint bounds. All measured protection stays
// in AutoGuard; the historical diagnostic excursion is not a policy limit.
class TaskPathGuard {
public:
    TaskPathGuard(const AutoSpec& spec,const std::array<double,8>& initial):guard_(spec) {
        guard_.initialize_reference(initial,false);
    }
    void check_initial(std::uint64_t now,const std::array<DecodedFeedback,8>& states,
                       const std::array<double,8>& initial,bool enabled=false) {
        guard_.check_impl(now,states,initial,enabled,false,true);
    }
    void check(std::uint64_t now,const std::array<DecodedFeedback,8>& states,const TaskSession& task) {
        guard_.check_impl(now,states,task.reference(),true,false,true);
    }
private:
    AutoGuard guard_;
};
}
