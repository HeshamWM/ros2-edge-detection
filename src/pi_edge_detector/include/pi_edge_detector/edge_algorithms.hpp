#ifndef PI_EDGE_DETECTOR__EDGE_ALGORITHMS_HPP_
#define PI_EDGE_DETECTOR__EDGE_ALGORITHMS_HPP_

#include <opencv2/opencv.hpp>
#include <string>
#include <stdexcept>

namespace pi_edge_detector {

/// Parameters that control how each edge detection algorithm behaves.
/// All of these map 1-to-1 to ROS 2 dynamic parameters.
struct EdgeParams {
  int canny_low  = 50;
  int canny_high = 150;
  int sobel_ksize = 3;        // must be 1, 3, 5, or 7
  int laplacian_ksize = 3;    // must be 1, 3, 5, or 7
};

/// Stateless utility class that wraps OpenCV edge detection calls.
///
/// Design notes for your implementation:
///   - Every method takes a **pre-allocated** output cv::Mat& so the
///     caller can reuse the same buffer across frames (no heap churn).
///   - Input `gray` is guaranteed to be single-channel (CV_8UC1).
///   - These are static methods — no instance state needed.
class EdgeAlgorithms {
public:
  /// Apply Canny edge detection.
  ///   1. Blur with a 3×3 Gaussian to suppress noise
  ///   2. cv::Canny with (low, high) thresholds
  static void canny(const cv::Mat& gray, cv::Mat& edges, const EdgeParams& p);

  /// Apply Sobel edge detection.
  ///   1. Compute gradient in X (grad_x) and Y (grad_y) with cv::Sobel
  ///   2. Combine: edges = |grad_x| + |grad_y|  (use cv::convertScaleAbs)
  static void sobel(const cv::Mat& gray, cv::Mat& edges, const EdgeParams& p);

  /// Apply Laplacian edge detection.
  ///   1. Blur with a 3×3 Gaussian to suppress noise
  ///   2. cv::Laplacian → cv::convertScaleAbs
  static void laplacian(const cv::Mat& gray, cv::Mat& edges, const EdgeParams& p);

  /// Convenience dispatcher — calls the right method based on the string name.
  /// Throws std::invalid_argument if algorithm_name is unrecognised.
  static void detect(const std::string& algorithm_name,
                     const cv::Mat& gray,
                     cv::Mat& edges,
                     const EdgeParams& p);
};

}  // namespace pi_edge_detector

#endif  // PI_EDGE_DETECTOR__EDGE_ALGORITHMS_HPP_
