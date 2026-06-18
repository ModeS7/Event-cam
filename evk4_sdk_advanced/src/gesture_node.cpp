// Gesture classification (Rock-Paper-Scissors) on the EVK4 event stream, via the
// Metavision SDK ML module running a pretrained TorchScript model on the GPU.
//
// This is an ML-tier pipeline: unlike the model-free pipelines it loads a neural
// network (the "chifoumi" classifier) and runs inference. It is built only when
// Torch + the SDK ml module are present (the x86 full SDK build); the Pi's lean
// build skips it. Subclass of EventVisionNode.
//
// Flow (mirrors the SDK metavision_gesture_classification sample, adapted to our
// harness): an EventBufferReslicer accumulates events into the model's input
// Tensor over a fixed window (delta_t_us, default 50 ms) via the EventPreprocessor
// built from the model's .json metadata; at each window boundary the model runs
// inference on the GPU and we take the argmax of the softmaxed class output. The
// frame thread draws the event image and overlays the predicted label.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/core/algorithms/event_buffer_reslicer_algorithm.h>
#include <metavision/sdk/core/algorithms/event_rescaler_algorithm.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>
#include <metavision/sdk/core/preprocessors/event_preprocessor_factory.h>
#include <metavision/sdk/core/preprocessors/event_preprocessor_type.h>
#include <metavision/sdk/core/preprocessors/json_parser.h>
#include <metavision/sdk/core/preprocessors/tensor.h>
#include <metavision/sdk/ml/models/model.h>
#include <metavision/sdk/ml/utils/json_parser.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace evk4_sdk_advanced
{
using Metavision::EventCD;
using Metavision::Tensor;
using Metavision::Value;

class GestureClassification : public EventVisionNode
{
public:
  explicit GestureClassification(const rclcpp::NodeOptions & options)
  : EventVisionNode("gesture", "gesture_image", options)
  {
    model_path_ = declare_parameter<std::string>("model_path", "");
    gpu_id_ = static_cast<int>(declare_parameter("gpu_id", 0));
    confidence_threshold_ = static_cast<float>(declare_parameter("confidence_threshold", 0.8));
    delta_t_us_ = static_cast<Metavision::timestamp>(declare_parameter("delta_t_us", 50000));
    if (model_path_.empty()) {
      throw std::runtime_error(
        "gesture requires model_path (the chifoumi .ptjit, e.g. "
        "$MV_MODELS/classification/convRNN_chifoumi/rnn_model_classifier.ptjit)");
    }
    RCLCPP_INFO(
      get_logger(), "gesture: model=%s gpu_id=%d delta_t=%ld us",
      model_path_.c_str(), gpu_id_, static_cast<long>(delta_t_us_));
  }

  ~GestureClassification() override { stopFrameThread(); }

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    sensor_w_ = width;
    sensor_h_ = height;

    // --- load the TorchScript model on the GPU ---
    std::unordered_map<std::string, std::any> ml_params;
    ml_params["model-path"] = std::filesystem::path(model_path_);
    ml_params["use-cuda"] = (gpu_id_ != -1);
    ml_params["gpu-id"] = gpu_id_;
    ml_params["backend"] = Metavision::Backend::TORCH;
    RCLCPP_INFO(get_logger(), "loading model (this can take a few seconds)...");
    model_ = Metavision::Model::create_model(ml_params);
    if (!model_) {
      throw std::runtime_error("failed to create model for " + model_path_);
    }

    std::filesystem::path json_path = model_path_;
    json_path.replace_extension("json");
    Metavision::parse_json_labels(json_path, labels_);

    model_input_ = model_->get_input();
    model_output_ = model_->get_output();

    Tensor & evt_tensor = Metavision::get_tensor(model_input_.at("cls_input"));
    Metavision::TensorShape evt_input_shape = evt_tensor.shape();
    const int net_c = Metavision::get_dim(evt_input_shape, "C");
    net_h_ = Metavision::get_dim(evt_input_shape, "H");
    net_w_ = Metavision::get_dim(evt_input_shape, "W");

    // --- rescaler if the sensor resolution differs from the model input ---
    float width_scaling = 1.f, height_scaling = 1.f;
    if (net_h_ != sensor_h_ || net_w_ != sensor_w_) {
      width_scaling = static_cast<float>(net_w_) / sensor_w_;
      height_scaling = static_cast<float>(net_h_) / sensor_h_;
      rescaler_ = std::make_unique<Metavision::EventRescalerAlgorithm>(width_scaling, height_scaling);
    }

    // --- event preprocessor (events -> input Tensor), driven by model metadata ---
    std::vector<std::unordered_map<std::string, Metavision::PreprocessingParameters>> preprocess_maps;
    const auto root = Metavision::get_tree_from_file(json_path);
    Metavision::parse_preprocessors_params(
      root.get_child("input").get_child("preprocessing"), preprocess_maps);
    auto & pp = preprocess_maps[0];
    const auto & ptype = std::get<Metavision::EventPreprocessorType>(pp.at("type"));
    if (ptype == Metavision::EventPreprocessorType::HARDWARE_HISTO) {
      Tensor & scale = Metavision::get_tensor(model_input_.at("scale_factor"));
      const uint8_t neg = std::get<uint8_t>(pp.at("neg_saturation"));
      const uint8_t pos = std::get<uint8_t>(pp.at("pos_saturation"));
      *(scale.data<float>()) = (width_scaling * height_scaling) / std::max(neg, pos);
    } else if (ptype == Metavision::EventPreprocessorType::HARDWARE_DIFF) {
      Tensor & scale = Metavision::get_tensor(model_input_.at("scale_factor"));
      const int8_t max_val = std::get<int8_t>(pp.at("max_val"));
      *(scale.data<float>()) = (width_scaling * height_scaling) / (max_val + 1.);
    } else if (ptype == Metavision::EventPreprocessorType::HISTO ||
      ptype == Metavision::EventPreprocessorType::DIFF)
    {
      pp["scale_width"] = width_scaling;
      pp["scale_height"] = height_scaling;
    }
    event_preprocessor_ =
      Metavision::EventPreprocessorFactory::create<const EventCD *>(pp, evt_input_shape);

    evt_tensor.create(evt_input_shape, evt_tensor.type());
    processed_data_.create(
      event_preprocessor_->get_output_shape(), event_preprocessor_->get_output_type(),
      evt_tensor.data(), false);
    if (net_c != Metavision::get_dim(event_preprocessor_->get_output_shape(), "C")) {
      throw std::runtime_error("model channels != preprocessor output channels");
    }

    // --- reslicer fires inference every delta_t_us ---
    slicer_ = std::make_unique<Metavision::EventBufferReslicerAlgorithm>();
    slicer_->set_slicing_condition(
      Metavision::EventBufferReslicerAlgorithm::Condition::make_n_us(delta_t_us_));
    slicer_->set_on_new_slice_callback(
      [this](Metavision::EventBufferReslicerAlgorithm::ConditionStatus, Metavision::timestamp ts,
      std::size_t) { runInference(ts); });

    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
      width, height, accTimeUs());
    RCLCPP_INFO(
      get_logger(), "pipeline initialized for %ux%u (model input %dx%d, %zu classes)",
      width, height, net_w_, net_h_, labels_.size());
  }

  // Sub thread: accumulate events into the model input tensor; the reslicer's
  // on-new-slice callback runs inference at each window boundary.
  void processEvents(const std::vector<EventCD> & events) override
  {
    if (events.empty()) {
      return;
    }
    slicer_->process_events(
      events.data(), events.data() + events.size(),
      [this](const EventCD * cbegin, const EventCD * cend) {
        if (rescaler_) {
          rescaled_.clear();
          rescaler_->process_events(cbegin, cend, std::back_inserter(rescaled_));
          event_preprocessor_->process_events(
            win_ts_, rescaled_.data(), rescaled_.data() + rescaled_.size(), processed_data_);
        } else {
          event_preprocessor_->process_events(win_ts_, cbegin, cend, processed_data_);
        }
      });
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

  bool renderFrame(
    const std::vector<EventCD> & events, Metavision::timestamp ts, cv::Mat & frame) override
  {
    if (events.empty()) {
      return false;
    }
    frame_gen_->process_events(events.begin(), events.end());
    frame_gen_->generate(ts, frame);
    if (work_idx_ >= 0 && work_idx_ < static_cast<int>(labels_.size()) &&
      work_conf_ >= confidence_threshold_)
    {
      cv::putText(
        frame, labels_[work_idx_], cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2,
        cv::Scalar(0, 255, 0), 2);
    }
    return true;
  }

private:
  // Sub thread: run the model and take argmax of the softmaxed class output.
  void runInference(Metavision::timestamp ts)
  {
    model_->infer(model_input_, model_output_);
    const Tensor & out = Metavision::get_tensor(model_output_.at("cls_output"));
    const auto shape = out.shape();
    if (shape.is_valid()) {
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
    Metavision::get_tensor(model_input_.at("cls_input")).set_to(0.f);
    win_ts_ = ts;
  }

  // params
  std::string model_path_;
  int gpu_id_{0};
  float confidence_threshold_{0.8f};
  Metavision::timestamp delta_t_us_{50000};

  // geometry
  int sensor_w_{0}, sensor_h_{0}, net_w_{0}, net_h_{0};

  // ML
  std::vector<std::string> labels_;
  std::unique_ptr<Metavision::Model> model_;
  std::unordered_map<std::string, Value> model_input_, model_output_;
  std::unique_ptr<Metavision::EventPreprocessor<const EventCD *>> event_preprocessor_;
  std::unique_ptr<Metavision::EventRescalerAlgorithm> rescaler_;
  std::unique_ptr<Metavision::EventBufferReslicerAlgorithm> slicer_;
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;
  Tensor processed_data_;
  Metavision::timestamp win_ts_{0};
  std::vector<EventCD> rescaled_;

  // results: sub thread -> staged (mutex) -> work (frame thread)
  int pred_idx_{-1}, staged_idx_{-1}, work_idx_{-1};
  float pred_conf_{0.f}, staged_conf_{0.f}, work_conf_{0.f};
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::GestureClassification)
