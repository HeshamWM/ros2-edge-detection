#include "pi_edge_detector/perf_monitor.hpp"

#include <sstream>
#include <iomanip>
#include <algorithm>

namespace pi_edge_detector {

// =============================================================================
// TASK 1: Implement the constructor
// =============================================================================
// Initialize all member variables:
//   - window_size_ from the parameter
//   - running_sum_ = 0.0
//   - total_frames_ = 0
//   - total_drops_ = 0
//   - latencies_ and frame_times_ start empty (they're std::deque, default is fine)
// =============================================================================
PerfMonitor::PerfMonitor(std::size_t window_size)
  : window_size_(window_size),
    running_sum_(0.0),
    total_frames_(0),
    total_drops_(0)
{
  // Constructor body — nothing else needed if you use the initializer list above.
  // But feel free to add validation (e.g., window_size must be > 0).
}

// =============================================================================
// TASK 2: Implement record_frame()
// =============================================================================
// This is called ONCE per successfully processed frame.
//
// Steps:
//   1. Push latency_ms onto the back of latencies_
//   2. Add latency_ms to running_sum_
//   3. If latencies_.size() > window_size_:
//      - Subtract latencies_.front() from running_sum_
//      - Pop the front of latencies_
//   4. Push Clock::now() onto frame_times_ (for FPS calculation)
//   5. If frame_times_.size() > window_size_:
//      - Pop the front of frame_times_
//   6. Increment total_frames_
//
// Why a sliding window?
//   A global average would be dominated by startup frames. The sliding
//   window gives you a "recent" average that reflects current performance.
//
// Why track running_sum_ instead of computing sum each time?
//   O(1) vs O(N). When window_size=100, it doesn't matter much, but
//   this is the kind of optimization interviewers love to see.
// =============================================================================
void PerfMonitor::record_frame(double latency_ms)
{
  // YOUR CODE HERE
  latencies_.push_back(latency_ms);
  running_sum_ += latency_ms;
  if (latencies_.size() > window_size_){
    running_sum_ -= latencies_.front();
    latencies_.pop_front();
  }
  frame_times_.push_back(Clock::now());
  if (frame_times_.size() > window_size_)
  {
    frame_times_.pop_front();
  }
  total_frames_++;
}

// =============================================================================
// TASK 3: Implement record_drop()
// =============================================================================
// Just increment total_drops_. That's it. One line.
// =============================================================================
void PerfMonitor::record_drop()
{
  this->total_drops_++;
}

// =============================================================================
// TASK 4: Implement avg_latency_ms()
// =============================================================================
// Return running_sum_ / latencies_.size()
// Handle the empty case — return 0.0 if no samples yet.
// =============================================================================
double PerfMonitor::avg_latency_ms() const
{
  if (latencies_.size() < 1){
    return 0.0;
  }else {
    return running_sum_ / latencies_.size();
  }
}

// =============================================================================
// TASK 5: Implement measured_fps()
// =============================================================================
// FPS = number_of_frames_in_window / time_span_of_window
//
// Steps:
//   1. If frame_times_.size() < 2, return 0.0 (can't compute FPS from 1 sample)
//   2. Compute elapsed = frame_times_.back() - frame_times_.front()
//   3. Convert elapsed to seconds (use std::chrono::duration<double>)
//   4. Return (frame_times_.size() - 1) / elapsed_seconds
//
// Why size() - 1?
//   If you have timestamps [t0, t1, t2], that's 2 intervals, not 3.
//   Classic off-by-one that interviewers will catch.
// =============================================================================
double PerfMonitor::measured_fps() const
{
  if (frame_times_.size() < 2){
    return 0.0;
  }
  std::chrono::duration<double> elapsed = frame_times_.back() - frame_times_.front(); 
  return (frame_times_.size() - 1) / elapsed.count(); // Comopiler complaining here / can't be used with chrono duration?
}

// =============================================================================
// TASK 6: Implement total_frames() and total_drops()
// =============================================================================
// Simple getters. One line each.
// =============================================================================
uint64_t PerfMonitor::total_frames() const
{
  return total_frames_;
}

uint64_t PerfMonitor::total_drops() const
{
  return total_drops_;
}

// =============================================================================
// TASK 7: Implement get_summary()
// =============================================================================
// Return a string like:
//   "avg=4.2ms | fps=14.8 | processed=1042 | dropped=58"
//
// Use std::ostringstream and std::fixed / std::setprecision(1) for
// clean formatting. Example:
//
//   std::ostringstream ss;
//   ss << std::fixed << std::setprecision(1);
//   ss << "avg=" << avg_latency_ms() << "ms";
//   ss << " | fps=" << measured_fps();
//   ss << " | processed=" << total_frames_;
//   ss << " | dropped=" << total_drops_;
//   return ss.str();
// =============================================================================
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

// =============================================================================
// TASK 8: Implement reset()
// =============================================================================
// Clear latencies_, frame_times_, set running_sum_ = 0,
// total_frames_ = 0, total_drops_ = 0.
// =============================================================================
void PerfMonitor::reset()
{
  latencies_.clear();
  frame_times_.clear();
  running_sum_ = 0;
  total_frames_ = 0;
  total_drops_ = 0;
}

}  // namespace pi_edge_detector
