#ifndef PI_YOLOV26_DETECTOR__PERF_MONITOR_HPP_
#define PI_YOLOV26_DETECTOR__PERF_MONITOR_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <string>

namespace pi_yolov26_detector {

class PerfMonitor {
public:
  explicit PerfMonitor(std::size_t window_size = 100);

  void record_frame(double latency_ms);
  void record_drop();

  double avg_latency_ms() const;
  double measured_fps() const;
  uint64_t total_frames() const;
  uint64_t total_drops() const;

  std::string get_summary() const;
  void reset();

private:
  std::size_t window_size_;
  std::deque<double> latencies_;
  double running_sum_;

  uint64_t total_frames_;
  uint64_t total_drops_;

  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  std::deque<TimePoint> frame_times_;
};

}  // namespace pi_yolov26_detector

#endif  // PI_YOLOV26_DETECTOR__PERF_MONITOR_HPP_
