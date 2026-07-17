#ifndef PI_EDGE_DETECTOR__PERF_MONITOR_HPP_
#define PI_EDGE_DETECTOR__PERF_MONITOR_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <string>

namespace pi_edge_detector {

/// Lightweight performance tracker for the processing pipeline.
///
/// Tracks:
///   - Per-frame processing latency (moving average)
///   - Actual throughput (frames per second)
///   - Dropped frame count (frames skipped by the throttle)
///
/// Design notes for your implementation:
///   - Use a std::deque<double> as a sliding window for the last N latencies.
///   - Keep a running sum so avg_latency is O(1), not O(N).
///   - record_frame() is called once per processed frame.
///   - record_drop() is called each time the throttle skips a frame.
///   - get_summary() returns a human-readable one-liner for RCLCPP_INFO.
class PerfMonitor {
public:
  /// Construct with the size of the sliding window (default: 100 frames).
  explicit PerfMonitor(std::size_t window_size = 100);

  /// Call this ONCE per processed frame with the elapsed processing time.
  void record_frame(double latency_ms);

  /// Call this each time the FPS throttle decides to skip a frame.
  void record_drop();

  /// Average processing latency over the sliding window (ms).
  double avg_latency_ms() const;

  /// Measured throughput: frames_in_window / time_span_of_window.
  double measured_fps() const;

  /// Total frames successfully processed since construction.
  uint64_t total_frames() const;

  /// Total frames dropped since construction.
  uint64_t total_drops() const;

  /// Human-readable one-liner, e.g.:
  /// "avg=4.2ms | fps=14.8 | processed=1042 | dropped=58"
  std::string get_summary() const;

  /// Reset all counters and clear the window.
  void reset();

private:
  std::size_t window_size_;
  std::deque<double> latencies_;      // sliding window of latency samples
  double running_sum_;                // sum of values currently in the deque

  uint64_t total_frames_;
  uint64_t total_drops_;

  // Timestamps for FPS calculation
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  std::deque<TimePoint> frame_times_; // timestamps of recent frames
};

}  // namespace pi_edge_detector

#endif  // PI_EDGE_DETECTOR__PERF_MONITOR_HPP_
