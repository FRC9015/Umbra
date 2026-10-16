#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <utility>

namespace umbra::diagnostics {

using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;

enum class Metric : size_t {
    CaptureRead, FrameInterval, Convert, Detect, Preview, Blur,
    CornerRefine, TagPrepare,
    Pose, PublishCall, PublishInterval, RetrievalToPublish, FrameTotal, Count
};

inline constexpr std::array<const char*, static_cast<size_t>(Metric::Count)> names = {
    "capture_read", "frame_interval", "convert", "detect", "preview",
    "blur", "corner_refine", "tag_prepare", "pose", "publish_call", "publish_interval",
    "retrieval_to_publish", "frame_total"
};

struct Sample {
    uint64_t count = 0;
    double totalMs = 0;
    double maxMs = 0;
    void add(double ms) {
        ++count;
        totalMs += ms;
        if (ms > maxMs) maxMs = ms;
    }
};

inline std::mutex outputMutex;

// One instance belongs to one camera worker. All counters are local to it.
// No camera, NT, or UI behavior changes when disabled.
class Window {
public:
    explicit Window(std::string name) : name_(std::move(name)) {
        const char* value = std::getenv("UMBRA_DIAGNOSTICS");
        enabled_ = value && std::string(value) == "1";
        if (enabled_) start_ = Clock::now();
    }

    bool enabled() const { return enabled_; }
    Time stamp() const { return enabled_ ? Clock::now() : Time{}; }

    void add(Metric metric, Time from, Time to) {
        if (!enabled_) return;
        samples_[static_cast<size_t>(metric)].add(
            std::chrono::duration<double, std::milli>(to - from).count());
    }

    void readFrame(Time readAt) {
        if (!enabled_) return;
        if (lastRead_ != Time{}) add(Metric::FrameInterval, lastRead_, readAt);
        lastRead_ = readAt;
        ++sequence_;
    }

    void published(Time from, Time beforeSet, Time afterSet) {
        if (!enabled_) return;
        add(Metric::PublishCall, beforeSet, afterSet);
        add(Metric::RetrievalToPublish, from, afterSet);
        if (lastPublish_ != Time{})
            add(Metric::PublishInterval, lastPublish_, afterSet);
        lastPublish_ = afterSet;
        ++published_;
    }

    void pose(double x, double y, double yaw) {
        if (!enabled_) return;
        poseX_ = x; poseY_ = y; poseYaw_ = yaw;
        hasPose_ = true;
        status_ = "valid";
    }

    void rejected(const char* reason) {
        if (!enabled_) return;
        ++rejected_;
        status_ = reason;
    }

    void cameraUnavailable(const char* reason) {
        if (!enabled_ || cameraStatus_ == reason) return;
        cameraStatus_ = reason;
        std::lock_guard<std::mutex> lock(outputMutex);
        std::cerr << "[umbra diagnostics] camera=" << name_
                  << " status=" << reason << " (no observation published)\n";
    }

    void cameraRestored() {
        if (!enabled_ || cameraStatus_ == "open") return;
        cameraStatus_ = "open";
        std::lock_guard<std::mutex> lock(outputMutex);
        std::cerr << "[umbra diagnostics] camera=" << name_ << " status=open\n";
    }

    void frameDone(Time now) {
        if (!enabled_) return;
        ++frames_;
        if (now - start_ < std::chrono::seconds(5)) return;
        const double seconds = std::chrono::duration<double>(now - start_).count();
        std::lock_guard<std::mutex> lock(outputMutex);
        std::cerr << std::fixed << std::setprecision(2)
                  << "[umbra diagnostics] camera=" << name_
                  << " window_s=" << seconds
                  << " frames=" << frames_
                  << " processing_fps=" << frames_ / seconds
                  << " publications=" << published_
                  << " rejected=" << rejected_
                  << " status=" << status_
                  << " last_frame_seq=" << sequence_;
        if (hasPose_)
            std::cerr << " last_pose_x_m=" << poseX_
                      << " last_pose_y_m=" << poseY_
                      << " last_pose_yaw_rad=" << poseYaw_;
        if (lastPublish_ != Time{})
            std::cerr << " last_publication_age_ms="
                      << std::chrono::duration<double, std::milli>(now - lastPublish_).count();
        for (size_t i = 0; i < samples_.size(); ++i) {
            const auto& s = samples_[i];
            if (!s.count) continue;
            std::cerr << ' ' << names[i] << "_avg_ms=" << s.totalMs / s.count
                      << ' ' << names[i] << "_max_ms=" << s.maxMs
                      << ' ' << names[i] << "_n=" << s.count;
        }
        std::cerr << '\n';
        samples_ = {};
        frames_ = published_ = rejected_ = 0;
        start_ = now;
    }

private:
    std::string name_;
    std::string cameraStatus_ = "unknown";
    const char* status_ = "none";
    bool enabled_ = false;
    bool hasPose_ = false;
    double poseX_ = 0, poseY_ = 0, poseYaw_ = 0;
    Time start_{}, lastRead_{}, lastPublish_{};
    std::array<Sample, static_cast<size_t>(Metric::Count)> samples_{};
    uint64_t sequence_ = 0, frames_ = 0, published_ = 0, rejected_ = 0;
};

} // namespace umbra::diagnostics
