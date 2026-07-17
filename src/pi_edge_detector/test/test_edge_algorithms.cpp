#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "pi_edge_detector/edge_algorithms.hpp"

using namespace pi_edge_detector;

cv::Mat make_test_image(int rows = 100, int cols = 100) {
  cv::Mat img = cv::Mat::zeros(rows, cols, CV_8UC1);
  img(cv::Rect(cols / 2, 0, cols / 2, rows)) = 255;
  return img;
}

TEST(EdgeAlgorithmsTest, CannyProducesEdges) {
  cv::Mat gray = make_test_image();
  cv::Mat edges;
  EdgeParams params;
  params.canny_low = 50;
  params.canny_high = 150;

  EdgeAlgorithms::canny(gray, edges, params);

  EXPECT_EQ(edges.rows, gray.rows);
  EXPECT_EQ(edges.cols, gray.cols);
  EXPECT_GT(cv::countNonZero(edges), 0);
}

TEST(EdgeAlgorithmsTest, SobelProducesEdges) {
  cv::Mat gray = make_test_image();
  cv::Mat edges;
  EdgeParams params;
  params.sobel_ksize = 3;

  EdgeAlgorithms::sobel(gray, edges, params);

  EXPECT_EQ(edges.rows, gray.rows);
  EXPECT_EQ(edges.cols, gray.cols);
  EXPECT_GT(cv::countNonZero(edges), 0);
}

TEST(EdgeAlgorithmsTest, LaplacianProducesEdges) {
  cv::Mat gray = make_test_image();
  cv::Mat edges;
  EdgeParams params;
  params.laplacian_ksize = 3;

  EdgeAlgorithms::laplacian(gray, edges, params);

  EXPECT_EQ(edges.rows, gray.rows);
  EXPECT_EQ(edges.cols, gray.cols);
  EXPECT_GT(cv::countNonZero(edges), 0);
}

TEST(EdgeAlgorithmsTest, DetectDispatchesCanny) {
  cv::Mat gray = make_test_image();
  cv::Mat edges;
  EdgeParams params;

  EdgeAlgorithms::detect("canny", gray, edges, params);

  EXPECT_EQ(edges.rows, gray.rows);
  EXPECT_EQ(edges.cols, gray.cols);
  EXPECT_GT(cv::countNonZero(edges), 0);
}

TEST(EdgeAlgorithmsTest, DetectThrowsOnUnknownAlgorithm) {
  cv::Mat gray = make_test_image();
  cv::Mat edges;
  EdgeParams params;

  EXPECT_THROW(
    EdgeAlgorithms::detect("invalid_algo_name", gray, edges, params),
    std::invalid_argument
  );
}

TEST(EdgeAlgorithmsTest, CannyOnBlankImage) {
  cv::Mat gray = cv::Mat::zeros(100, 100, CV_8UC1);
  cv::Mat edges;
  EdgeParams params;

  EdgeAlgorithms::canny(gray, edges, params);

  EXPECT_EQ(edges.rows, gray.rows);
  EXPECT_EQ(edges.cols, gray.cols);
  EXPECT_EQ(cv::countNonZero(edges), 0);
}
