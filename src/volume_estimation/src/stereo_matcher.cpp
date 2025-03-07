// src/test.cpp
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

// Include ELAS library
#include "elas.h"

class ElasTest : public rclcpp::Node
{
public:
  ElasTest() : Node("elas_test")
  {
    // Declare parameters for image paths
    this->declare_parameter("left_image_path", "left.png");
    this->declare_parameter("right_image_path", "right.png");
    this->declare_parameter("output_path", "disparity.png");
  }

  bool run()
  {
    // Get parameters
    std::string left_path = this->get_parameter("left_image_path").as_string();
    std::string right_path = this->get_parameter("right_image_path").as_string();
    std::string output_path = this->get_parameter("output_path").as_string();

    RCLCPP_INFO(this->get_logger(), "Loading images: %s and %s", left_path.c_str(), right_path.c_str());

    // Load images
    cv::Mat left_img = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
    cv::Mat right_img = cv::imread(right_path, cv::IMREAD_GRAYSCALE);

    if (left_img.empty() || right_img.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load input images!");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Processing stereo pair: %dx%d", left_img.cols, left_img.rows);

    // Get image dimensions
    const int width = left_img.cols;
    const int height = left_img.rows;

    // Allocate memory for disparity maps
    const int32_t dims[3] = {width, height, width}; // bytes per line = width
    float* D1_data = (float*)malloc(width * height * sizeof(float));
    float* D2_data = (float*)malloc(width * height * sizeof(float));

    // Set ELAS parameters
    Elas::parameters param;
    param.disp_min = 0;
    param.disp_max = 255;
    param.support_threshold = 0.85;
    param.support_texture = 10;
    param.candidate_stepsize = 5;
    param.incon_window_size = 5;
    param.incon_threshold = 5;
    param.incon_min_support = 5;
    param.add_corners = 0;
    param.grid_size = 20;
    param.beta = 0.02;
    param.gamma = 3;
    param.sigma = 1;
    param.sradius = 2;
    param.match_texture = 1;
    param.lr_threshold = 2;
    param.speckle_sim_threshold = 1;
    param.speckle_size = 200;
    param.ipol_gap_width = 3;
    param.filter_median = 0;
    param.filter_adaptive_mean = 1;
    param.postprocess_only_left = 1;
    param.subsampling = 0;

    // Create ELAS instance
    Elas elas(param);

    // Process stereo pair
    elas.process(
      left_img.data,    // Left image
      right_img.data,   // Right image
      D1_data,          // Output disparity map (left)
      D2_data,          // Output disparity map (right)
      dims              // Image dimensions
    );

    // Create OpenCV matrix from disparity data for visualization
    cv::Mat disparity(height, width, CV_32F, D1_data);

    // Convert to CV_8U for display
    cv::Mat disparity_8u;
    cv::normalize(disparity, disparity_8u, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Save disparity map in grayscale
    cv::imwrite(output_path, disparity_8u);
    RCLCPP_INFO(this->get_logger(), "Grayscale disparity map saved to: %s", output_path.c_str());

    // Free memory
    free(D1_data);
    free(D2_data);

    return true;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ElasTest>();
  
  bool success = node->run();
  
  rclcpp::shutdown();
  return success ? 0 : 1;
}