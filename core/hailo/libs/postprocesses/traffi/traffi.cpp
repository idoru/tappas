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
    std::map<HailoDetectionPtr, int> vehicle_dets;
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
  for (const auto &pair : priv->vehicle_dets) {
    auto det = pair.first;
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

std::map<HailoDetectionPtr, int> TurnTracker::vehicle_detections() {
  std::lock_guard<std::mutex> lock(mutex_);
  return priv->vehicle_dets;
}

void TurnTracker::add_vehicle_det(HailoDetectionPtr det) {
  std::lock_guard<std::mutex> lock(mutex_);
  det->remove_objects_typed(HAILO_UNIQUE_ID);

  priv->current_id ++;
  auto newid = std::make_shared<HailoUniqueID>(HailoUniqueID(priv->current_id));
  det->add_object(newid);
  priv->vehicle_dets[det] = 10;
}

void TurnTracker::mark_seen(HailoDetectionPtr vehicle_det) {
  std::lock_guard<std::mutex> lock(mutex_);
  priv->vehicle_dets[vehicle_det] ++;
}

void TurnTracker::gc() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<HailoDetectionPtr> deletions;
  std::vector<int> hailo_id_deletions;
  for (const auto &pair : priv->vehicle_dets) {
    priv->vehicle_dets[pair.first] --;
    if (priv->vehicle_dets[pair.first] < 0) {
      deletions.emplace_back(pair.first);
    }
  }
  for (const auto &todelete : deletions) {
    std::cout << "Delete vehicle" << std::endl;
    priv->vehicle_dets.erase(todelete);
    for (const auto &mapping : priv->hailo_unique_id_vehicles) {
      if (mapping.second == todelete) {
        hailo_id_deletions.emplace_back(mapping.first);
      }
    }
    for (const auto &idtodelete : hailo_id_deletions) {
      std::cout << "  hid: " << idtodelete << std::endl;
      priv->hailo_unique_id_vehicles.erase(idtodelete);
    }
  }
}

// Default filter function
void filter(HailoROIPtr roi)
{
  std::map<int, HailoDetectionPtr> candidates;
  std::vector<HailoDetectionPtr> seen;

  for (auto obj : roi->get_objects_typed(HAILO_DETECTION)) {
    HailoDetectionPtr det = std::dynamic_pointer_cast<HailoDetection>(obj);
    std::string label = det->get_label();

    //dont care about non vehicles
    bool is_chosen_type = label == "car" || label == "bus" || label == "truck" || label == "train" || label == "boat";
    if (!is_chosen_type) {
      continue;
    }

    for (auto detobj : det->get_objects()) {
      if (detobj->get_type() == HAILO_UNIQUE_ID) {
        HailoUniqueIDPtr id = std::dynamic_pointer_cast<HailoUniqueID>(detobj);
        auto det_copy = std::make_shared<HailoDetection>(*det);
        candidates[id->get_id()] = det_copy; //candidates are tracked (has unique id from hailotracker) chosen types
      } else {
        std::cout << "UNHANDLED OBJ TYPE: " << detobj->get_type() << std::endl;
      }
    }
  }
  roi->remove_objects_typed(HAILO_DETECTION);

  std::cout << "-[detect]----------------------- " << std::endl;
  for (const auto &pair : candidates) {
    int id = pair.first;
    // do we already have a vehicle detection for hailo detection ID?
    auto vdet = TurnTracker::GetInstance().get_vehicle_det_for_hailo_det(id);
    if (vdet == NULL) {
      // no? how about one that matches on iou?
      vdet = TurnTracker::GetInstance().get_vehicle_det_matching_hailo_det_iou(pair.second);
      if (vdet) {
        // iou match with existing:
        // add this hailo detection ID to the list associated with the matching vehicle detection
        TurnTracker::GetInstance().map_hailo_id_to_vehicle_det(id, vdet);
        std::cout << "hid:" << id << " mapping to existing vehicle" << std::endl;
      } else {
        // no match, create a new vechile detection for this candidate
        vdet = pair.second; //take the copied detection
        pair.second->set_label("vehicle");
        TurnTracker::GetInstance().add_vehicle_det(vdet);
        TurnTracker::GetInstance().map_hailo_id_to_vehicle_det(id, vdet);
        std::cout << "hid:" << id << " seems new!" << std::endl;
      }
    } else {
      std::cout << "hid:" << id << " in existing vehicle detection" << std::endl;
    }
    vdet->set_bbox(pair.second->get_bbox());
    seen.emplace_back(vdet);
    hailo_common::add_object(roi, vdet);
    TurnTracker::GetInstance().mark_seen(vdet);
  }
  std::cout << "-[gc]----------------------- " << std::endl;
  TurnTracker::GetInstance().gc();
}
