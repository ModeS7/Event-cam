// Gesture classification (Rock-Paper-Scissors) on the EVK4 event stream: runs the
// pretrained "chifoumi" TorchScript classifier on the GPU and overlays the
// predicted label. ML-tier pipeline (needs Torch + the SDK ml module). The model
// load / preprocessing / inference loop live in the shared MlVisionNode base; this
// file is just the gesture-specific output: softmax-argmax of the class logits and
// drawing the label.

#include "evk4_sdk_advanced/ml_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/ml/utils/json_parser.h>

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace evk4_sdk_advanced
{

class GestureClassification : public MlVisionNode
{
public:
  explicit GestureClassification(const rclcpp::NodeOptions & options)
  : MlVisionNode("gesture", "gesture_image", options)
  {
    confidence_threshold_ = static_cast<float>(declare_parameter("confidence_threshold", 0.8));
  }

  ~GestureClassification() override { stopFrameThread(); }

protected:
  void onModelReady(const std::filesystem::path & json_path, uint16_t, uint16_t) override
  {
    Metavision::parse_json_labels(json_path, labels_);
    RCLCPP_INFO(get_logger(), "gesture classifier: %zu classes", labels_.size());
  }

  // Subscription thread: softmax + argmax of the class logits.
  void extractResults(Metavision::timestamp) override
  {
    const Tensor & out = Metavision::get_tensor(modelOutput().at("cls_output"));
    const auto shape = out.shape();
    if (!shape.is_valid()) {
      return;
    }
    const auto * ptr = out.data<float>();
    const int nb = Metavision::get_dim(shape, "C");
    float maxv = -1e30f;
    int maxi = 0;
    double sum = 0.0;
    for (int i = 0; i < nb; ++i) {
      sum += std::exp(static_cast<double>(ptr[i]));
      if (ptr[i] > maxv) {maxv = ptr[i]; maxi = i;}
    }
    pred_idx_ = maxi;
    pred_conf_ = static_cast<float>(std::exp(static_cast<double>(maxv)) / sum);
  }

  void stageResults() override
  {
    staged_idx_ = pred_idx_;
    staged_conf_ = pred_conf_;
  }

  void swapResults() override
  {
    work_idx_ = staged_idx_;
    work_conf_ = staged_conf_;
  }

  void drawResults(cv::Mat & frame) override
  {
    if (work_idx_ >= 0 && work_idx_ < static_cast<int>(labels_.size()) &&
      work_conf_ >= confidence_threshold_)
    {
      cv::putText(
        frame, labels_[work_idx_], cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2,
        cv::Scalar(0, 255, 0), 2);
    }
  }

private:
  float confidence_threshold_{0.8f};
  std::vector<std::string> labels_;
  int pred_idx_{-1}, staged_idx_{-1}, work_idx_{-1};
  float pred_conf_{0.f}, staged_conf_{0.f}, work_conf_{0.f};
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::GestureClassification)
