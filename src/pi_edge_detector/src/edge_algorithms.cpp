#include "pi_edge_detector/edge_algorithms.hpp"

namespace pi_edge_detector {

void EdgeAlgorithms::canny(const cv::Mat& gray, cv::Mat& edges, const EdgeParams& p)
{
  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0.0);
  cv::Canny(blurred, edges, p.canny_low, p.canny_high);
}

void EdgeAlgorithms::sobel(const cv::Mat& gray, cv::Mat& edges, const EdgeParams& p)
{
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  // Compute X and Y derivatives
  cv::Sobel(gray, grad_x, CV_16S, 1, 0, p.sobel_ksize);
  cv::Sobel(gray, grad_y, CV_16S, 0, 1, p.sobel_ksize);

  // Convert back to absolute 8-bit values
  cv::convertScaleAbs(grad_x, abs_grad_x);
  cv::convertScaleAbs(grad_y, abs_grad_y);

  // Combine gradients
  cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0.0, edges);
}

void EdgeAlgorithms::laplacian(const cv::Mat& gray, cv::Mat& edges, const EdgeParams& p)
{
  cv::Mat blurred, lap;
  cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0.0);
  cv::Laplacian(blurred, lap, CV_16S, p.laplacian_ksize);
  cv::convertScaleAbs(lap, edges);
}

void EdgeAlgorithms::detect(const std::string& algorithm_name,
                            const cv::Mat& gray,
                            cv::Mat& edges,
                            const EdgeParams& p)
{
  if (algorithm_name == "canny") {
    canny(gray, edges, p);
  } else if (algorithm_name == "sobel") {
    sobel(gray, edges, p);
  } else if (algorithm_name == "laplacian") {
    laplacian(gray, edges, p);
  } else {
    throw std::invalid_argument("Choice of algorithm has to be: canny, sobel, laplacian");
  }
}

}  // namespace pi_edge_detector
