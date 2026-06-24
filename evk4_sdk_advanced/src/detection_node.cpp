// Object detection + tracking on the EVK4 event stream: runs the pretrained
// automotive SSD detector (red_event_cube) on the GPU and draws labeled, tracked
// bounding boxes. ML-tier pipeline (needs Torch + the SDK ml module). The model
// load / preprocessing / inference loop live in the shared MlVisionNode base; this
// file adds the detection post-processing: decode boxes from the model output,
// NMS, and DataAssociation tracking.
//
// The model is automotive (classes: cars and pedestrians, from a forward-facing
// car camera), so a desk scene yields no boxes -- point it at driving footage.

#include "evk4_sdk_advanced/ml_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/core/events/event_bbox.h>
#include <metavision/sdk/core/events/event_tracked_box.h>
#include <metavision/sdk/ml/algorithms/data_association.h>
#include <metavision/sdk/ml/algorithms/non_maximum_suppression.h>
#include <metavision/sdk/ml/utils/json_parser.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace evk4_sdk_advanced
{
using DataAssoc = Metavision::DataAssociation<Metavision::EventBbox>;

class DetectionTracking : public MlVisionNode
{
public:
  explicit DetectionTracking(const rclcpp::NodeOptions & options)
  : MlVisionNode("detection", "detection_image", options)
  {
    confidence_ = static_cast<float>(declare_parameter("confidence_threshold", 0.4));
  }

  ~DetectionTracking() override { stopThreads(); }

protected:
  std::string eventInputName() const override { return "event_cube"; }

  void onModelReady(const std::filesystem::path & json_path, uint16_t, uint16_t) override
  {
    Metavision::parse_json_labels(json_path, labels_);

    // The SSD detector takes the confidence threshold as a model input.
    Tensor & score = Metavision::get_tensor(modelInput().at("score_thresh"));
    *(score.data<float>()) = confidence_;

    float iou_threshold = 0.5f;
    int num_anchor_boxes = 0;
    Metavision::parse_json_ssd_box_decoding(json_path, iou_threshold, num_anchor_boxes);
    nms_ = std::make_unique<Metavision::NonMaximumSuppression>(
      labels_.size(), iou_threshold, widthScaling(), heightScaling());
    nms_->ignore_class_id(0);  // class 0 is background noise

    DataAssoc::Config conf(
      0.7f /*detection_update_weight*/, 100000 /*deletion_time_us*/,
      0.5f /*max_iou_inter_track*/, 0.2f /*iou_to_match*/,
      0.5f /*max_iou_one_det_many_tracks*/, 1 /*min_detections_to_confirm*/,
      false /*do_tracklet_prediction*/, 0.9f /*tracking_confidence_decay*/,
      false /*use_descriptor*/, labels_.size(), 200000 /*timesurface_memory_us*/,
      netWidth(), netHeight());
    tracker_ = std::make_unique<DataAssoc>(conf);
    tracker_->add_tracklet_consumer_cb(
      [this](const Metavision::EventTrackedBox * b, const Metavision::EventTrackedBox * e,
      Metavision::timestamp) {
        tracked_.clear();
        std::copy(b, e, std::back_inserter(tracked_));
      });
    RCLCPP_INFO(get_logger(), "detection: %zu classes", labels_.size());
  }

  // Inference thread: decode boxes -> NMS -> tracking.
  void extractResults(Metavision::timestamp ts) override
  {
    bboxes_.clear();
    valid_.clear();
    for (const auto & kv : modelOutput()) {
      const std::vector<Value> & values = Metavision::get_values(kv.second);
      if (values.empty()) {
        continue;
      }
      const Tensor & tensor = Metavision::get_tensor(values[0]);
      const auto shape = tensor.shape();
      if (!shape.is_valid()) {
        continue;
      }
      const auto * ptr = tensor.data<float>();
      const int nb = shape.dimensions[0].dim;
      for (int i = 0; i < nb; ++i) {
        Metavision::EventBbox box;
        box.t = ts;
        box.x = ptr[6 * i + 0];
        box.y = ptr[6 * i + 1];
        box.w = ptr[6 * i + 2] - box.x;
        box.h = ptr[6 * i + 3] - box.y;
        box.class_confidence = ptr[6 * i + 4];
        box.class_id = static_cast<unsigned int>(ptr[6 * i + 5]);
        bboxes_.push_back(box);
      }
    }
    nms_->process_events(bboxes_.cbegin(), bboxes_.cend(), std::back_inserter(valid_));
    tracker_->process_boxes(valid_.data(), valid_.data() + valid_.size(), ts);
  }

  void stageResults() override { staged_ = tracked_; }
  void swapResults() override { std::swap(work_, staged_); }

  void drawResults(cv::Mat & frame) override
  {
    for (const auto & b : work_) {
      const cv::Rect r(
        static_cast<int>(b.x), static_cast<int>(b.y), static_cast<int>(b.w),
        static_cast<int>(b.h));
      cv::rectangle(frame, r, cv::Scalar(0, 255, 0), 2);
      const std::string lbl =
        (b.class_id < labels_.size()) ? labels_[b.class_id] : std::to_string(b.class_id);
      cv::putText(
        frame, lbl, cv::Point(static_cast<int>(b.x), static_cast<int>(b.y) - 5),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
  }

private:
  float confidence_{0.4f};
  std::vector<std::string> labels_;
  std::unique_ptr<Metavision::NonMaximumSuppression> nms_;
  std::unique_ptr<DataAssoc> tracker_;
  std::vector<Metavision::EventBbox> bboxes_, valid_;                 // inference thread
  std::vector<Metavision::EventTrackedBox> tracked_, staged_, work_;  // infer / staged / frame
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::DetectionTracking)
