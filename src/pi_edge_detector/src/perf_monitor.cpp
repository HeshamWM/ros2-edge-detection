#include "pi_edge_detector/perf_monitor.hpp"

#include <sstream>
#include <iomanip>
#include <algorithm>

namespace pi_edge_detector {

PerfMonitor::PerfMonitor(std::size_t window_size)
  : window_size_(window_size),
    running_sum_(0.0),
    total_frames_(0),
    total_drops_(0)
{
}

void PerfMonitor::record_frame(double latency_ms)
{
  latencies_.push_back(latency_ms);
  running_sum_ += latency_ms;

  // Manage rolling window for latency average
  if (latencies_.size() > window_size_) {
    running_sum_ -= latencies_.front();
    latencies_.pop_front();
  }

  // Manage rolling window for FPS calculation timestamps
  frame_times_.push_back(Clock::now());
  if (frame_times_.size() > window_size_) {
    frame_times_.pop_front();
  }

  total_frames_++;
}

void PerfMonitor::record_drop()
{
  total_drops_++;
}

double PerfMonitor::avg_latency_ms() const
{
  if (latencies_.empty()) {
    return 0.0;
  }
  return running_sum_ / latencies_.size();
}

double PerfMonitor::measured_fps() const
{
  if (frame_times_.size() < 2) {
    return 0.0;
  }
  
  std::chrono::duration<double> elapsed = frame_times_.back() - frame_times_.front();
  if (elapsed.count() <= 0.0) {
    return 0.0;
  }

  return (frame_times_.size() - 1) / elapsed.count();
}

uint64_t PerfMonitor::total_frames() const
{
  return total_frames_;
}

uint64_t PerfMonitor::total_drops() const
{
  return total_drops_;
}

std::string PerfMonitor::get_summary() const
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1);
  ss << "avg=" << avg_latency_ms() << "ms";
  ss << " | fps=" << measured_fps();
  ss << " | processed=" << total_frames_;
  ss << " | dropped=" << total_drops_;
  return ss.str();
}

void PerfMonitor::reset()
{
  latencies_.clear();
  frame_times_.clear();
  running_sum_ = 0.0;
  total_frames_ = 0;
  total_drops_ = 0;
}

}  // namespace pi_edge_detector
