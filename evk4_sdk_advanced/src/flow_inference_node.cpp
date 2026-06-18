// Learned optical flow on the EVK4 event stream: runs the pretrained flow network
// on the GPU and overlays flow-vector arrows (vs the classical optical_flow/
// dense_flow pipelines, which use no neural network). ML-tier pipeline (needs
// Torch + the SDK ml module). The model load / preprocessing / inference loop live
// in the shared MlVisionNode base; this file adds the flow-specific output: copy
// the predicted flow field and draw arrows colored by direction.

#include "evk4_sdk_advanced/ml_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace evk4_sdk_advanced
{

class OpticalFlowInference : public MlVisionNode
{
public:
  explicit OpticalFlowInference(const rclcpp::NodeOptions & options)
  : MlVisionNode("flow_inference", "flow_inference_image", options)
  {
    // Threshold on squared flow magnitude (px^2) and arrow grid spacing (px).
    min_disp_ = static_cast<float>(declare_parameter("min_flow_disp", 3.0));
    visu_step_ = static_cast<int>(declare_parameter("visu_step", 8));
  }

  ~OpticalFlowInference() override { stopFrameThread(); }

protected:
  void onModelReady(const std::filesystem::path &, uint16_t, uint16_t) override
  {
    // Prefer the half-resolution flow output if the model exposes it, else the first.
    output_name_ = modelOutput().cbegin()->first;
    for (const auto & kv : modelOutput()) {
      if (kv.first.find("half") != std::string::npos) {output_name_ = kv.first; break;}
    }
    RCLCPP_INFO(get_logger(), "flow output: '%s'", output_name_.c_str());
  }

  // Subscription thread: copy the predicted flow field (flow_x[H*W], flow_y[H*W]).
  void extractResults(Metavision::timestamp) override
  {
    const Tensor & t = Metavision::get_tensor(modelOutput().at(output_name_));
    const auto shape = t.shape();
    if (!shape.is_valid()) {
      return;
    }
    flow_h_ = Metavision::get_dim(shape, "H");
    flow_w_ = Metavision::get_dim(shape, "W");
    const size_t n = static_cast<size_t>(flow_h_) * static_cast<size_t>(flow_w_);
    const float * ptr = t.data<float>();
    flow_buf_.assign(ptr, ptr + 2 * n);
  }

  void stageResults() override
  {
    staged_buf_ = flow_buf_;
    staged_h_ = flow_h_;
    staged_w_ = flow_w_;
  }

  void swapResults() override
  {
    work_buf_.swap(staged_buf_);
    work_h_ = staged_h_;
    work_w_ = staged_w_;
  }

  void drawResults(cv::Mat & frame) override
  {
    if (work_w_ <= 0 || work_h_ <= 0) {
      return;
    }
    const size_t n = static_cast<size_t>(work_h_) * static_cast<size_t>(work_w_);
    if (work_buf_.size() < 2 * n) {
      return;
    }
    const float * fx = work_buf_.data();
    const float * fy = work_buf_.data() + n;
    const float sx = static_cast<float>(frame.cols) / work_w_;
    const float sy = static_cast<float>(frame.rows) / work_h_;
    for (int i = visu_step_; i < work_h_; i += visu_step_) {
      for (int j = visu_step_; j < work_w_; j += visu_step_) {
        const float vx = fx[i * work_w_ + j];
        const float vy = fy[i * work_w_ + j];
        if (vx * vx + vy * vy <= min_disp_) {
          continue;
        }
        // Hue = direction; full saturation/value.
        float deg = std::atan2(vy, vx) * 180.f / static_cast<float>(CV_PI);
        if (deg < 0) {deg += 360.f;}
        cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(static_cast<uchar>(deg / 2), 255, 255));
        cv::Mat bgr;
        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
        const cv::Vec3b c = bgr.at<cv::Vec3b>(0, 0);
        cv::arrowedLine(
          frame, cv::Point(static_cast<int>(j * sx), static_cast<int>(i * sy)),
          cv::Point(static_cast<int>((j + vx) * sx), static_cast<int>((i + vy) * sy)),
          cv::Scalar(c[0], c[1], c[2]), 1, cv::LINE_AA, 0, 0.3);
      }
    }
  }

private:
  float min_disp_{3.0f};
  int visu_step_{8};
  std::string output_name_;
  std::vector<float> flow_buf_, staged_buf_, work_buf_;  // sub / staged / frame
  int flow_h_{0}, flow_w_{0}, staged_h_{0}, staged_w_{0}, work_h_{0}, work_w_{0};
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::OpticalFlowInference)
