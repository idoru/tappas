#include <iostream>
#include <iomanip>
#include "traffi.hpp"

std::mutex TurnTracker::mutex_;

typedef unsigned int vehicle_id;

class TurnTracker::TurnTrackerPrivate
{
  protected:
    vehicle_id current_id = 0;
    std::map<int, vehicle_id> vehicle_id_hailo_unique_ids;
    std::map<vehicle_id, HailoDetectionPtr> vehicle_id_dets;
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
  auto it = priv->vehicle_id_hailo_unique_ids.find(hailo_id);
  if (it != priv->vehicle_id_hailo_unique_ids.end()) {
    auto det_it = priv->vehicle_id_dets.find(it);
    if (it != priv->vehicle_id_dets.end()) {
      return it;
    } else {
      return NULL;
    }
  }
  return NULL;
}

HailoDetectionPtr TurnTracker::get_vehicle_det_matching_hailo_det_iou(HailoDetectionPtr hailo_det) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto bbox = hailo_det.get_bbox();
  for (auto det : priv->vehicle_dets) {
    if (common::iou_calc(bbox, det.get_bbox()) > 0.75f) {
      return det;
    }
  }
  return NULL;
}

HailoDetectionPtr TurnTracker::map_hailo_id_to_vehicle_det(int hailo_id, vehicle_id, HailoDetectionPtr vehicle_det) {
  std::lock_guard<std::mutex> lock(mutex_);
  priv->vehicle_dets.emplace_back(vehicle_det);
  priv->vehicle_id_dets[vehicle_id] = vehicle_det;
  priv-vehicle_id_hailo_unique_ids[hailo_id] = vehicle_id;
    std::map<int, vehicle_id> 
}
  //does this id have a mapping to a vehicle detection?
  //yes -> update vehicle detection bbox
    //no -> check IoU against existing vehicle detection bboxes
        //IoU match -> add mapping, update bbox
        //No IoU match -> add mapping, create detection clone
// Default filter function
void filter(HailoROIPtr roi)
{
  std::cout << "----------------------- " << std::endl;

  std::map<int, HailoDetectionPtr> candidates;

  for (auto obj : roi->get_objects_typed(HAILO_DETECTION)) {
    HailoDetectionPtr det = std::dynamic_pointer_cast<HailoDetection>(obj);
    std::string label = det->get_label();

    bool is_chosen_type = label == "car" || label == "bus" || label == "truck" || label == "train" || label == "boat";
    if (!is_chosen_type) {
      continue;
    }
    auto det_copy = std::make_shared<HailoDetection>(*det);
    det_copy->set_label("vehicle");

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
    //does this id have a mapping to vehicle_id? yes -> update vehicle bbox
    //if not yet exists, check iou against all known bboxes
      //found ? add mapping, update bbox
      //notfound ? take a new id, add mapping and new entry with bbox
    HailoDetectionPtr new_det = pair.second;
    std::cout << new_det->get_label() << " #" << id << " (" << std::fixed << std::setprecision(1) << (new_det->get_confidence() * 100) << std::setprecision(3) << "5) @ (" << new_det->get_bbox().xmin() << ", ";
    std::cout << new_det->get_bbox().ymin() << ") ";
    std::cout << new_det->get_bbox().width() << " x ";
    std::cout << new_det->get_bbox().height() << std::endl;
    //roi->add_object(det);
    hailo_common::add_object(roi, new_det);
  }
}
