#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "pi_edge_detector/edge_algorithms.hpp"

using namespace pi_edge_detector;

// Helper: create a simple test image with a clear edge
// (left half black, right half white — a sharp vertical edge)
cv::Mat make_test_image(int rows = 100, int cols = 100) {
  cv::Mat img = cv::Mat::zeros(rows, cols, CV_8UC1);
  img(cv::Rect(cols / 2, 0, cols / 2, rows)) = 255;
  return img;
}

// =============================================================================
// YOUR TASK: Write unit tests for each edge detection algorithm
// =============================================================================
// I've provided the first test as an example. Write the remaining tests.
//
// Guidelines:
//   - Each test should create a test image, run the algorithm, and verify
//     the output is non-zero (edges were found) and has correct dimensions.
//   - Test edge cases: empty image, 1×1 image, etc.
//   - Test parameter validation where applicable.
// =============================================================================

TEST(EdgeAlgorithmsTest, CannyProducesEdges) {
  cv::Mat gray = make_test_image();
  cv::Mat edges;
  EdgeParams params;
  params.canny_low = 50;
  params.canny_high = 150;

  EdgeAlgorithms::canny(gray, edges, params);

  // Output should have the same dimensions as input
  EXPECT_EQ(edges.rows, gray.rows);
  EXPECT_EQ(edges.cols, gray.cols);

  // There should be non-zero pixels (the vertical edge was detected)
  EXPECT_GT(cv::countNonZero(edges), 0);
}

// TODO: Write TEST(EdgeAlgorithmsTest, SobelProducesEdges)
//   - Similar structure to the Canny test above

// TODO: Write TEST(EdgeAlgorithmsTest, LaplacianProducesEdges)
//   - Similar structure

// TODO: Write TEST(EdgeAlgorithmsTest, DetectDispatchesCanny)
//   - Call EdgeAlgorithms::detect("canny", ...) and verify it works

// TODO: Write TEST(EdgeAlgorithmsTest, DetectThrowsOnUnknownAlgorithm)
//   - Call EdgeAlgorithms::detect("unknown", ...) and expect std::invalid_argument
//   - Use EXPECT_THROW(EdgeAlgorithms::detect("unknown", ...), std::invalid_argument)

// TODO: Write TEST(EdgeAlgorithmsTest, CannyOnBlankImage)
//   - Create a fully black image (cv::Mat::zeros)
//   - Run Canny — output should be all zeros (no edges in a blank image)
//   - EXPECT_EQ(cv::countNonZero(edges), 0)
