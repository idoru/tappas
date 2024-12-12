#include <iostream>
#include <iomanip>
#include "traffi.hpp"
#include "common/nms.hpp"

std::mutex TurnTracker::mutex_;

typedef unsigned int vehicle_id;

class TurnTracker::TurnTrackerPrivate
{
  public:
    vehicle_id current_id = 0;
    std::map<int, HailoDetectionPtr> hailo_unique_id_vehicles;
    std::vector<HailoDetectionPtr> vehicle_dets;
};

TurnTracker::TurnTracker() : priv(std::make_unique<TurnTrackerPrivate>()){};
TurnTracker::~TurnTracker(){};
TurnTracker &TurnTracker::GetInstance()
{
  std::lock_guard<std::mutex> lock(mutex_);
  static TurnTracker instance;
  return instance;
}

HailoDetectionPtr TurnTracker::get_vehicle_det_for_hailo_det(int hailo_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = priv->hailo_unique_id_vehicles.find(hailo_id);
  if (it != priv->hailo_unique_id_vehicles.end()) {
    return it->second;
  }
  return NULL;
}

HailoDetectionPtr TurnTracker::get_vehicle_det_matching_hailo_det_iou(HailoDetectionPtr hailo_det) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto a = hailo_det->get_bbox();
  for (auto det : priv->vehicle_dets) {
    auto b = det->get_bbox();
    if ( ((a.xmin() < b.xmin()) && (a.xmax() > b.xmax()))
       || ((b.xmin() < a.xmin()) && (b.xmax() > a.xmax())) )
       {
      if ( ((a.ymin() < b.ymin()) && (a.ymax() > b.ymax()))
        || ((b.ymin() < a.ymin()) && (b.ymax() > a.ymax())) ) {
        return det;
      }
    }
    if (common::iou_calc(a, b) > 0.50f) {
      return det;
    }
  }
  return NULL;
}

void TurnTracker::map_hailo_id_to_vehicle_det(int hailo_id, HailoDetectionPtr vehicle_det) {
  std::lock_guard<std::mutex> lock(mutex_);
  priv->hailo_unique_id_vehicles[hailo_id] = vehicle_det;
}

std::vector<HailoDetectionPtr> TurnTracker::vehicle_detections() {
  std::lock_guard<std::mutex> lock(mutex_);
  return priv->vehicle_dets;
}

void TurnTracker::add_vehicle_det(HailoDetectionPtr det) {
  std::lock_guard<std::mutex> lock(mutex_);
  det->remove_objects_typed(HAILO_UNIQUE_ID);

  priv->current_id ++;
  auto newid = std::make_shared<HailoUniqueID>(HailoUniqueID(priv->current_id));
  det->add_object(newid);
  priv->vehicle_dets.emplace_back(det);
}
// Default filter function
void filter(HailoROIPtr roi)
{
  std::cout << "----------------------- " << std::endl;

  std::map<int, HailoDetectionPtr> candidates;
  std::vector<HailoDetectionPtr> seen;

  for (auto obj : roi->get_objects_typed(HAILO_DETECTION)) {
    HailoDetectionPtr det = std::dynamic_pointer_cast<HailoDetection>(obj);
    std::string label = det->get_label();

    bool is_chosen_type = label == "car" || label == "bus" || label == "truck" || label == "train" || label == "boat";
    if (!is_chosen_type) {
      continue;
    }
    auto det_copy = std::make_shared<HailoDetection>(*det);

    for (auto detobj : det->get_objects()) {
      if (detobj->get_type() == HAILO_UNIQUE_ID) {
        HailoUniqueIDPtr id = std::dynamic_pointer_cast<HailoUniqueID>(detobj);
        candidates[id->get_id()] = det_copy; //candidates are tracked (has unique id from hailotracker) chosen types
      } else {
        std::cout << "UNHANDLED OBJ TYPE: " << detobj->get_type() << std::endl;
      }
    }
  }
  roi->remove_objects_typed(HAILO_DETECTION);

  for (const auto &pair : candidates) {
    int id = pair.first;
    auto vdet = TurnTracker::GetInstance().get_vehicle_det_for_hailo_det(id);
    if (vdet == NULL) {
      vdet = TurnTracker::GetInstance().get_vehicle_det_matching_hailo_det_iou(pair.second);
      if (vdet) {
        TurnTracker::GetInstance().map_hailo_id_to_vehicle_det(id, vdet);
      } else {
        vdet = pair.second;
        pair.second->set_label("vehicle");
        TurnTracker::GetInstance().add_vehicle_det(vdet);
        TurnTracker::GetInstance().map_hailo_id_to_vehicle_det(id, vdet);
      }
    }
    vdet->set_bbox(pair.second->get_bbox());
    seen.emplace_back(vdet);
  }
  for (auto det : seen) {
    hailo_common::add_object(roi, det);
  }
}
