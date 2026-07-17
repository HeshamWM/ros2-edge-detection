#include "pi_edge_detector/edge_algorithms.hpp"

namespace pi_edge_detector {

// =============================================================================
// TASK 1: Implement canny()
// =============================================================================
// Steps:
//   1. Apply cv::GaussianBlur on `gray` with a 3×3 kernel to reduce noise.
//      Store the result in a local cv::Mat (the blur is small, allocation is fine).
//   2. Call cv::Canny(blurred, edges, p.canny_low, p.canny_high)
//
// Hints:
//   - GaussianBlur signature: cv::GaussianBlur(src, dst, cv::Size(3,3), 0);
//   - Canny writes directly into `edges`, so it will be auto-sized.
//
// Why blur first?  //   Canny is very noise-sensitive. Without blurring, you'll get
//   thousands of spurious edges on a noisy Pi camera feed.
// =============================================================================
void EdgeAlgorithms::canny(const cv::Mat& gray, cv::Mat& edges, const EdgeParams& p)
{
  cv::GaussianBlur(gray, edges, cv::Size(3,3), 0.0);
  cv::Canny(edges, edges, p.canny_high, p.canny_low);
}

// =============================================================================
// TASK 2: Implement sobel()
// =============================================================================
// Steps:
//   1. Compute the X gradient:
//      cv::Sobel(gray, grad_x, CV_16S, 1, 0, p.sobel_ksize)
//   2. Compute the Y gradient:
//      cv::Sobel(gray, grad_y, CV_16S, 0, 1, p.sobel_ksize)
//   3. Convert both to absolute 8-bit:
//      cv::convertScaleAbs(grad_x, abs_grad_x)
//      cv::convertScaleAbs(grad_y, abs_grad_y)
//   4. Combine them:
//      cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges)
//
// Why CV_16S?
//   Sobel produces negative gradient values. CV_8U would clip them to 0,
//   losing half the edge information. CV_16S preserves the sign, and
//   convertScaleAbs takes the absolute value afterward.
//
// Interview tip:
//   Be ready to explain the difference between Sobel and Canny.
//   Sobel gives gradient *magnitude* — every pixel gets a value.
//   Canny gives binary edges — pixel is edge or not-edge.
// =============================================================================
void EdgeAlgorithms::sobel(const cv::Mat& gray, cv::Mat& edges, const EdgeParams& p)
{
  cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y;
  cv::Sobel(gray, grad_x, CV_16S, 1, 0, p.sobel_ksize);
  cv::Sobel(gray, grad_y, CV_16S, 0, 1, p.sobel_ksize);
  cv::convertScaleAbs(grad_x, abs_grad_x);
  cv::convertScaleAbs(grad_y, abs_grad_y); //abs_grad are giving error
  cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges);
}

// =============================================================================
// TASK 3: Implement laplacian()
// =============================================================================
// Steps:
//   1. Apply cv::GaussianBlur on `gray` (3×3 kernel) — same reason as Canny
//   2. cv::Laplacian(blurred, lap, CV_16S, p.laplacian_ksize)
//   3. cv::convertScaleAbs(lap, edges)
//
// Laplacian is a second-order derivative — it finds edges where the
// intensity changes *rapidly*. It's fast but very noise-sensitive,
// hence the mandatory blur step.
// =============================================================================
void EdgeAlgorithms::laplacian(const cv::Mat& gray, cv::Mat& edges, const EdgeParams& p)
{
  cv::Mat lap;
  cv::GaussianBlur(gray, edges, cv::Size(3,3), 0.0);
  cv::Laplacian(edges, lap, CV_16S, p.laplacian_ksize); // This is also probably bullshit but no other mat is allocated!
  cv::convertScaleAbs(lap, edges);
}

// =============================================================================
// TASK 4: Implement detect() dispatcher
// =============================================================================
// This is a simple routing function:
//   - If algorithm_name == "canny",     call canny(...)
//   - If algorithm_name == "sobel",     call sobel(...)
//   - If algorithm_name == "laplacian", call laplacian(...)
//   - Otherwise, throw std::invalid_argument with a helpful message
//
// This lets the node call one function and have the algorithm selected
// by a runtime ROS parameter.
// =============================================================================
void EdgeAlgorithms::detect(const std::string& algorithm_name,
                            const cv::Mat& gray,
                            cv::Mat& edges,
                            const EdgeParams& p)
{
  // YOUR CODE HERE
  if (algorithm_name == "canny")
  {
    canny(gray, edges, p);
  }
  else if (algorithm_name == "sobel")
  {
    sobel(gray, edges, p);
  }
  else if (algorithm_name == "laplacian")
  {
    laplacian(gray, edges, p);
  }
  else
  {
    throw std::invalid_argument("Choice of algorithms has to be: canny, sobel, laplacian"); }
  
}

}  // namespace pi_edge_detector
