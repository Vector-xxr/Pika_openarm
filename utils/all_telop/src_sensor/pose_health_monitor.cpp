#include "pose_health_monitor.h"

#include <algorithm>
#include <iostream>
#include <thread>

namespace {

using SteadyClock = std::chrono::steady_clock;

SteadyClock::time_point after_seconds(SteadyClock::time_point t, double sec) {
    const auto ms = std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(std::max(0.0, sec)));
    return t + ms;
}

}  // namespace

void PoseHealthMonitor::print_static_fail_hint() {
    std::cerr
        << "静态检测不通过，请运行cd ~/pika_ros/install/pika_locator/lib && ./survive-cli"
           "或cd ~/pika_ros/install/pika_locator/lib && ./survive-cli --force-calibrate"
           "进行基站校准"
        << std::endl;
}

void PoseHealthMonitor::set_jump_thresholds(const JumpThresholds& th) { jump_th_ = th; }

void PoseHealthMonitor::enable_dynamic(bool on) {
    dynamic_enabled_.store(on);
    if (on) {
        std::lock_guard<std::mutex> lock(mu_);
        have_prev_jump_ = false;
        std::cout << "[pika] Dynamic pose jump monitor ON "
                     "(continues after 'q'; stops on Ctrl+C)."
                  << std::endl;
    }
}

bool PoseHealthMonitor::on_sample(const PoseSample& sample) {
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!static_finished_.load()) {
            const auto steady = SteadyClock::now();
            switch (phase_) {
                case StaticPhase::WaitingFirst:
                    phase_ = StaticPhase::Settling;
                    settle_deadline_ = after_seconds(steady, opt_.settle_sec);
                    std::cout << "[pika][health] First pose received; settling "
                              << opt_.settle_sec << "s (keep Tracker still)..." << std::endl;
                    break;
                case StaticPhase::Settling:
                    break;
                case StaticPhase::Collecting:
                    collect_.push_back(sample);
                    break;
                case StaticPhase::Done:
                    break;
            }
        }
    }

    if (dynamic_enabled_.load() && static_finished_.load()) {
        return accept_dynamic_sample(sample);
    }
    return true;
}

bool PoseHealthMonitor::run_static_check(std::atomic<bool>& keep_running,
                                         const StaticOptions& opt) {
    opt_ = opt;
    {
        std::lock_guard<std::mutex> lock(mu_);
        phase_ = StaticPhase::WaitingFirst;
        collect_.clear();
        first_deadline_ = after_seconds(SteadyClock::now(), opt_.wait_first_sec);
        settle_deadline_ = {};
        collect_deadline_ = {};
    }
    static_finished_.store(false);
    static_ok_.store(false);

    std::cout << "[pika][health] Waiting for first pose (timeout " << opt_.wait_first_sec
              << "s); then settle " << opt_.settle_sec << "s + stationary " << opt_.warmup_sec
              << "s..." << std::endl;

    while (keep_running.load()) {
        StaticPhase phase_snapshot;
        SteadyClock::time_point first_dl, settle_dl, collect_dl;
        {
            std::lock_guard<std::mutex> lock(mu_);
            phase_snapshot = phase_;
            first_dl = first_deadline_;
            settle_dl = settle_deadline_;
            collect_dl = collect_deadline_;
        }

        const auto now = SteadyClock::now();

        if (phase_snapshot == StaticPhase::WaitingFirst) {
            if (now >= first_dl) {
                std::cerr << "[pika][health] Timeout: no pose within " << opt_.wait_first_sec
                          << "s. Start locator first." << std::endl;
                print_static_fail_hint();
                static_ok_.store(false);
                static_finished_.store(true);
                {
                    std::lock_guard<std::mutex> lock(mu_);
                    phase_ = StaticPhase::Done;
                }
                return false;
            }
        } else if (phase_snapshot == StaticPhase::Settling) {
            if (now >= settle_dl) {
                std::lock_guard<std::mutex> lock(mu_);
                if (phase_ == StaticPhase::Settling) {
                    phase_ = StaticPhase::Collecting;
                    collect_.clear();
                    collect_deadline_ = after_seconds(SteadyClock::now(), opt_.warmup_sec);
                    std::cout << "[pika][health] Settle done; collecting " << opt_.warmup_sec
                              << "s (keep still)..." << std::endl;
                }
            }
        } else if (phase_snapshot == StaticPhase::Collecting) {
            if (now >= collect_dl) {
                std::vector<PoseSample> samples;
                {
                    std::lock_guard<std::mutex> lock(mu_);
                    samples = collect_;
                    phase_ = StaticPhase::Done;
                }
                const HealthStats st = analyze_stationary(samples, opt_.thresholds);
                std::cout << "[pika][health] n=" << st.n << "  drift=" << st.drift_m << " m / "
                          << st.drift_rad << " rad  max_v=" << st.max_speed
                          << " m/s  max_w=" << st.max_ang_speed << " rad/s  max_a=" << st.max_acc
                          << " m/s^2  => " << (st.ok ? "OK" : "FAIL") << std::endl;
                if (!st.ok) {
                    print_static_fail_hint();
                } else {
                    std::cout << "[pika][health] 静态检测通过。" << std::endl;
                }
                static_ok_.store(st.ok);
                static_finished_.store(true);
                return st.ok;
            }
        } else if (phase_snapshot == StaticPhase::Done) {
            return static_ok_.load();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    static_ok_.store(false);
    static_finished_.store(true);
    {
        std::lock_guard<std::mutex> lock(mu_);
        phase_ = StaticPhase::Done;
    }
    return false;
}

bool PoseHealthMonitor::accept_dynamic_sample(const PoseSample& sample) {
    std::lock_guard<std::mutex> lock(mu_);
    if (!have_prev_jump_) {
        prev_jump_ = sample;
        have_prev_jump_ = true;
        return true;
    }

    const JumpStats st = analyze_jump(prev_jump_, sample, jump_th_);
    prev_jump_ = sample;
    if (st.ok) {
        return true;
    }

    const auto now = SteadyClock::now();
    if (last_jump_warn_.time_since_epoch().count() != 0 &&
        now - last_jump_warn_ < std::chrono::seconds(1)) {
        return false;
    }
    last_jump_warn_ = now;

    if (st.gap) {
        std::cerr << "[pika][dynamic] 动态检测告警: 位姿间隔过大 dt=" << st.dt
                  << "s (阈值 " << jump_th_.max_gap_sec << "s)，已丢弃该控制帧。" << std::endl;
        return false;
    }
    std::cerr << "[pika][dynamic] 动态检测告警: 帧间跳变 "
              << "delta=" << st.delta_m << " m, v=" << st.speed << " m/s, w=" << st.ang_speed
              << " rad/s (阈值 delta<=" << jump_th_.max_delta_m << " m, v<=" << jump_th_.max_speed
              << " m/s, w<=" << jump_th_.max_ang_speed << " rad/s)，已丢弃该控制帧。" << std::endl;
    return false;
}
