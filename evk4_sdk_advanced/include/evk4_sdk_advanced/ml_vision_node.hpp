// Shared base for the neural-network (ML-tier) pipelines on the EVK4 event stream.
//
// It extends EventVisionNode with the machinery every metavision_ml inference
// pipeline needs and that is identical across them: load a TorchScript/ONNX model
// (on the GPU via gpu_id), build the EventPreprocessor (events -> input Tensor)
// from the model's .json metadata, slice the stream into fixed inference windows
// (delta_t_us), run model_->infer() at each window boundary, and render the event
// image. Each concrete pipeline (gesture/detection/flow) implements three hooks:
//   onModelReady  - set up output-specific state (labels, NMS+tracker, flow gen)
//   extractResults- read model_output_ after inference (inference thread)
//   drawResults   - overlay onto the event frame (frame thread)
// plus stageResults/swapResults (from EventVisionNode) for its own result type.
//
// Threading: a single GPU inference takes ~50 ms, far longer than the ~4 ms
// between event packets, so it CANNOT run on the subscription thread without
// stalling event ingestion (a 50 ms inference per 50 ms window saturates the
// thread -> the stream backs up, packets drop, and the renderer starves to ~0
// fps). Inference therefore runs on its OWN thread: the subscription thread only
// decodes events and hands them to a bounded queue; this inference thread drains
// the queue through the slicer + model; the frame thread renders the event image
// at fps with the latest results overlaid. The event frame is thus smooth (30
// fps) while the model output refreshes at the inference rate.
//
// Built only when Torch + the SDK ml module are present (the x86 full SDK build);
// see CMakeLists.txt and docs/sdk/install.md.

#ifndef EVK4_SDK_ADVANCED__ML_VISION_NODE_HPP_
#define EVK4_SDK_ADVANCED__ML_VISION_NODE_HPP_

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <metavision/sdk/core/algorithms/event_buffer_reslicer_algorithm.h>
#include <metavision/sdk/core/algorithms/event_rescaler_algorithm.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>
#include <metavision/sdk/core/preprocessors/event_preprocessor_factory.h>
#include <metavision/sdk/core/preprocessors/event_preprocessor_type.h>
#include <metavision/sdk/core/preprocessors/json_parser.h>
#include <metavision/sdk/core/preprocessors/tensor.h>
#include <metavision/sdk/ml/models/model.h>

#include <algorithm>
#include <any>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace evk4_sdk_advanced
{
using Metavision::EventCD;
using Metavision::Tensor;
using Metavision::Value;

class MlVisionNode : public EventVisionNode
{
public:
  MlVisionNode(
    const std::string & node_name, const std::string & image_topic,
    const rclcpp::NodeOptions & options)
  : EventVisionNode(node_name, image_topic, options)
  {
    model_path_ = declare_parameter<std::string>("model_path", "");
    gpu_id_ = static_cast<int>(declare_parameter("gpu_id", 0));
    delta_t_us_ = static_cast<Metavision::timestamp>(declare_parameter("delta_t_us", 50000));
    if (model_path_.empty()) {
      throw std::runtime_error(
        node_name + " requires model_path (the .ptjit model; gpu_id selects the GPU, -1 = CPU)");
    }
  }

  ~MlVisionNode() override { stopInferThread(); }

protected:
  // --- hooks for concrete pipelines ---
  // After the model + tensors are ready: set up output-specific state from the
  // model metadata JSON (labels, NMS, tracker, flow generator, ...).
  virtual void onModelReady(const std::filesystem::path & json_path, uint16_t w, uint16_t h) = 0;
  // Inference thread, after model_->infer(): read modelOutput() into members.
  // Called under mutex() (so it is serialized with stageResults reading those
  // members on the subscription thread); keep it cheap -- NMS/tracking only.
  virtual void extractResults(Metavision::timestamp ts) = 0;
  // Frame thread: overlay results onto the already-generated event frame.
  virtual void drawResults(cv::Mat & frame) = 0;
  // Optional: the model's event-input tensor name (default: first non-scale_factor
  // input). Override when a model has several inputs (e.g. detection's score_thresh).
  virtual std::string eventInputName() const { return ""; }

  // Stop the inference thread (and the frame thread). Derived destructors MUST
  // call this FIRST so the inference thread (which calls extractResults) and the
  // frame thread (which calls drawResults) are joined before the derived members
  // they touch are destroyed. Idempotent.
  void stopThreads()
  {
    stopInferThread();
    stopFrameThread();
  }

  // --- accessors for concrete pipelines ---
  std::unordered_map<std::string, Value> & modelInput() { return model_input_; }
  std::unordered_map<std::string, Value> & modelOutput() { return model_output_; }
  int sensorWidth() const { return sensor_w_; }
  int sensorHeight() const { return sensor_h_; }
  int netWidth() const { return net_w_; }
  int netHeight() const { return net_h_; }
  float widthScaling() const { return width_scaling_; }
  float heightScaling() const { return height_scaling_; }

  // --- EventVisionNode hooks implemented here ---
  void onInit(uint16_t width, uint16_t height) override
  {
    sensor_w_ = width;
    sensor_h_ = height;

    std::unordered_map<std::string, std::any> ml_params;
    const std::filesystem::path model_file(model_path_);
    ml_params["model-path"] = model_file;
    ml_params["use-cuda"] = (gpu_id_ != -1);
    ml_params["gpu-id"] = gpu_id_;
    ml_params["backend"] = (model_file.extension() == ".onnx")
      ? Metavision::Backend::ONNXRUNTIME : Metavision::Backend::TORCH;
    RCLCPP_INFO(
      get_logger(), "loading model %s (gpu_id=%d) -- can take a few seconds...",
      model_path_.c_str(), gpu_id_);
    model_ = Metavision::Model::create_model(ml_params);
    if (!model_) {
      throw std::runtime_error("failed to create model for " + model_path_);
    }

    model_input_ = model_->get_input();
    model_output_ = model_->get_output();

    // The event input tensor: an explicit name from the subclass, else the first
    // input that is not the optional scale_factor.
    input_name_ = eventInputName();
    if (input_name_.empty()) {
      for (const auto & kv : model_input_) {
        if (kv.first != "scale_factor") {input_name_ = kv.first; break;}
      }
    }
    if (input_name_.empty()) {
      throw std::runtime_error("model has no event input tensor");
    }

    Tensor & evt_tensor = Metavision::get_tensor(model_input_.at(input_name_));
    Metavision::TensorShape evt_shape = evt_tensor.shape();
    net_h_ = Metavision::get_dim(evt_shape, "H");
    net_w_ = Metavision::get_dim(evt_shape, "W");
    // Models with dynamic input H/W (e.g. detection) need them pinned before use.
    if (net_h_ <= 0) {net_h_ = sensor_h_; Metavision::set_dim(evt_shape, "H", net_h_);}
    if (net_w_ <= 0) {net_w_ = sensor_w_; Metavision::set_dim(evt_shape, "W", net_w_);}
    Metavision::set_dynamic_dimensions_to_one(evt_shape);
    const int net_c = Metavision::get_dim(evt_shape, "C");

    if (net_h_ != sensor_h_ || net_w_ != sensor_w_) {
      width_scaling_ = static_cast<float>(net_w_) / sensor_w_;
      height_scaling_ = static_cast<float>(net_h_) / sensor_h_;
      rescaler_ =
        std::make_unique<Metavision::EventRescalerAlgorithm>(width_scaling_, height_scaling_);
    }

    std::filesystem::path json_path = model_path_;
    json_path.replace_extension("json");
    std::vector<std::unordered_map<std::string, Metavision::PreprocessingParameters>> pp_maps;
    const auto root = Metavision::get_tree_from_file(json_path);
    Metavision::parse_preprocessors_params(
      root.get_child("input").get_child("preprocessing"), pp_maps);
    auto & pp = pp_maps[0];
    const auto & ptype = std::get<Metavision::EventPreprocessorType>(pp.at("type"));
    if (ptype == Metavision::EventPreprocessorType::HARDWARE_HISTO) {
      Tensor & scale = Metavision::get_tensor(model_input_.at("scale_factor"));
      const uint8_t neg = std::get<uint8_t>(pp.at("neg_saturation"));
      const uint8_t pos = std::get<uint8_t>(pp.at("pos_saturation"));
      *(scale.data<float>()) = (width_scaling_ * height_scaling_) / std::max(neg, pos);
    } else if (ptype == Metavision::EventPreprocessorType::HARDWARE_DIFF) {
      Tensor & scale = Metavision::get_tensor(model_input_.at("scale_factor"));
      const int8_t max_val = std::get<int8_t>(pp.at("max_val"));
      *(scale.data<float>()) = (width_scaling_ * height_scaling_) / (max_val + 1.);
    } else if (ptype == Metavision::EventPreprocessorType::HISTO ||
      ptype == Metavision::EventPreprocessorType::DIFF)
    {
      pp["scale_width"] = width_scaling_;
      pp["scale_height"] = height_scaling_;
    }
    event_preprocessor_ =
      Metavision::EventPreprocessorFactory::create<const EventCD *>(pp, evt_shape);

    evt_tensor.create(evt_shape, evt_tensor.type());
    processed_data_.create(
      event_preprocessor_->get_output_shape(), event_preprocessor_->get_output_type(),
      evt_tensor.data(), false);
    if (net_c != Metavision::get_dim(event_preprocessor_->get_output_shape(), "C")) {
      throw std::runtime_error("model channels != preprocessor output channels");
    }

    slicer_ = std::make_unique<Metavision::EventBufferReslicerAlgorithm>();
    slicer_->set_slicing_condition(
      Metavision::EventBufferReslicerAlgorithm::Condition::make_n_us(delta_t_us_));
    slicer_->set_on_new_slice_callback(
      [this](Metavision::EventBufferReslicerAlgorithm::ConditionStatus, Metavision::timestamp ts,
      std::size_t) { runWindow(ts); });

    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
      width, height, accTimeUs());

    onModelReady(json_path, width, height);

    // Warm up: a large model's FIRST inference triggers cuDNN autotuning +
    // TorchScript optimization (tens of seconds for the detector). Pay it once
    // here, on the zero-initialized input, so the first LIVE inference is not
    // stalled mid-stream.
    const auto wu = std::chrono::steady_clock::now();
    model_->infer(model_input_, model_output_);
    Metavision::get_tensor(model_input_.at(input_name_)).set_to(0.f);
    const double warmup_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wu).count();
    RCLCPP_INFO(
      get_logger(), "pipeline initialized for %ux%u (model input %dx%d, warmup %.1f s)",
      width, height, net_w_, net_h_, warmup_s);

    // Start the inference thread now that the model + slicer are ready.
    infer_running_ = true;
    infer_thread_ = std::thread([this]() { inferLoop(); });
  }

  // Subscription thread: just hand the decoded events to the inference thread.
  // Cheap (a copy + a notify), so event ingestion never stalls on inference.
  void processEvents(const std::vector<EventCD> & events) override
  {
    if (events.empty()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lk(q_mtx_);
      queued_.insert(queued_.end(), events.begin(), events.end());
      // Bound the backlog so a slow inference thread processes RECENT events
      // (fresh results) instead of an ever-growing tail: keep at most a few
      // windows of event-time, dropping the oldest under sustained overload.
      const Metavision::timestamp span = kMaxQueueWindows * delta_t_us_;
      const Metavision::timestamp cutoff = queued_.back().t - span;
      if (queued_.front().t < cutoff) {
        auto it = std::lower_bound(
          queued_.begin(), queued_.end(), cutoff,
          [](const EventCD & e, Metavision::timestamp c) { return e.t < c; });
        queued_.erase(queued_.begin(), it);
      }
    }
    q_cv_.notify_one();
  }

  bool renderFrame(
    const std::vector<EventCD> & events, Metavision::timestamp ts, cv::Mat & frame) override
  {
    if (events.empty()) {
      return false;
    }
    frame_gen_->process_events(events.begin(), events.end());
    frame_gen_->generate(ts, frame);
    drawResults(frame);
    return true;
  }

private:
  // Inference thread: drain the queue through the slicer; the slicer fires
  // runWindow() at each window boundary, where the model actually runs.
  void inferLoop()
  {
    std::vector<EventCD> batch;
    while (true) {
      {
        std::unique_lock<std::mutex> lk(q_mtx_);
        q_cv_.wait(lk, [this]() { return !infer_running_ || !queued_.empty(); });
        if (!infer_running_ && queued_.empty()) {
          break;
        }
        std::swap(queued_, batch);
      }
      if (!batch.empty()) {
        slicer_->process_events(
          batch.data(), batch.data() + batch.size(),
          [this](const EventCD * cbegin, const EventCD * cend) {
            const auto p0 = std::chrono::steady_clock::now();
            if (rescaler_) {
              rescaled_.clear();
              rescaler_->process_events(cbegin, cend, std::back_inserter(rescaled_));
              event_preprocessor_->process_events(
                win_ts_, rescaled_.data(), rescaled_.data() + rescaled_.size(), processed_data_);
            } else {
              event_preprocessor_->process_events(win_ts_, cbegin, cend, processed_data_);
            }
            preprocess_ms_ += std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - p0).count();
          });
        batch.clear();
      }
    }
  }

  // Inference thread: run inference at a window boundary, then reset the cube.
  void runWindow(Metavision::timestamp ts)
  {
    const auto t0 = std::chrono::steady_clock::now();
    model_->infer(model_input_, model_output_);
    const auto t1 = std::chrono::steady_clock::now();
    {
      // Serialize the model-output read + the subclass result members with the
      // subscription thread's stageResults() (which copies those members).
      std::lock_guard<std::mutex> lock(mutex());
      extractResults(ts);
    }
    const auto t2 = std::chrono::steady_clock::now();
    Metavision::get_tensor(model_input_.at(input_name_)).set_to(0.f);
    win_ts_ = ts;
    if (debugTiming()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ml window: preprocess=%.1f ms, infer=%.1f ms, post=%.1f ms", preprocess_ms_,
        std::chrono::duration<double, std::milli>(t1 - t0).count(),
        std::chrono::duration<double, std::milli>(t2 - t1).count());
    }
    preprocess_ms_ = 0.0;
  }

  void stopInferThread()
  {
    {
      std::lock_guard<std::mutex> lk(q_mtx_);
      if (!infer_running_) {
        return;
      }
      infer_running_ = false;
    }
    q_cv_.notify_all();
    if (infer_thread_.joinable()) {
      infer_thread_.join();
    }
  }

  std::string model_path_;
  int gpu_id_{0};
  Metavision::timestamp delta_t_us_{50000};

  int sensor_w_{0}, sensor_h_{0}, net_w_{0}, net_h_{0};
  float width_scaling_{1.f}, height_scaling_{1.f};
  std::string input_name_;

  std::unique_ptr<Metavision::Model> model_;
  std::unordered_map<std::string, Value> model_input_, model_output_;
  std::unique_ptr<Metavision::EventPreprocessor<const EventCD *>> event_preprocessor_;
  std::unique_ptr<Metavision::EventRescalerAlgorithm> rescaler_;
  std::unique_ptr<Metavision::EventBufferReslicerAlgorithm> slicer_;
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;
  Tensor processed_data_;
  Metavision::timestamp win_ts_{0};
  std::vector<EventCD> rescaled_;

  // Subscription thread -> inference thread event hand-off.
  static constexpr Metavision::timestamp kMaxQueueWindows = 4;
  std::mutex q_mtx_;
  std::condition_variable q_cv_;
  std::vector<EventCD> queued_;  // guarded by q_mtx_
  bool infer_running_{false};
  std::thread infer_thread_;

  double preprocess_ms_{0.0};  // preprocess time accumulated per window (debug_timing)
};

}  // namespace evk4_sdk_advanced

#endif  // EVK4_SDK_ADVANCED__ML_VISION_NODE_HPP_
