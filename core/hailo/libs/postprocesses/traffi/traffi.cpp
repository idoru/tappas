#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>
#include <set>
#include "traffi.hpp"
#include "common/nms.hpp"

static std::string ts_prefix() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "[%Y-%m-%d %H:%M:%S] ");
    return ss.str();
}

std::mutex TurnTracker::mutex_;

typedef unsigned int vehicle_id;

class TurnTracker::TurnTrackerPrivate
{
  public:
    vehicle_id current_id = 0;
    std::map<int, HailoDetectionPtr> hailo_unique_id_vehicles;
    std::map<HailoDetectionPtr, int> vehicle_dets;
    std::set<int> illegal_turn_vehicle_ids;
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

static int unique_id(HailoDetectionPtr det) {
  for (auto detobj : det->get_objects()) {
    if (detobj->get_type() == HAILO_UNIQUE_ID) {
      HailoUniqueIDPtr id = std::dynamic_pointer_cast<HailoUniqueID>(detobj);
      return id->get_id();
    }
  }
}

void TurnTracker::illegal_turn(int hailo_id) {
  #ifdef DEBUG
  std::cout << ts_prefix() << "hid:" << hailo_id << " made an illegal turn!" << std::endl;
  #endif
  auto vdet = this->get_vehicle_det_for_hailo_det(hailo_id);
  int vehicle_id = unique_id(vdet);
  if (priv->illegal_turn_vehicle_ids.find(vehicle_id) == priv->illegal_turn_vehicle_ids.end()) {
    priv->illegal_turn_vehicle_ids.insert(vehicle_id);
    std::cout << ts_prefix() << "Vehicle ID " << vehicle_id << " made an illegal turn. New count: " << priv->illegal_turn_vehicle_ids.size() << std::endl;
  }
}

size_t TurnTracker::get_illegal_turn_count() {
  std::lock_guard<std::mutex> lock(mutex_);
  return priv->illegal_turn_vehicle_ids.size();
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
    std::cout << ts_prefix() << "Delete vehicle id: " << unique_id(todelete) << std::endl;
    priv->vehicle_dets.erase(todelete);
    for (const auto &mapping : priv->hailo_unique_id_vehicles) {
      if (mapping.second == todelete) {
        hailo_id_deletions.emplace_back(mapping.first);
      }
    }
    for (const auto &idtodelete : hailo_id_deletions) {
      std::cout << ts_prefix() << "  was hid:" << idtodelete << std::endl;
      priv->hailo_unique_id_vehicles.erase(idtodelete);
    }
  }
}

bool is_above(HailoBBox bbox, float y_intercept, float slope) {
  auto x = bbox.xmin() + (bbox.width()/2.f);
  auto y = bbox.ymin() + (bbox.height()/2.f);
  return y < (y_intercept + slope * x); //TOP LEFT is 0,0, BOTTOM RIGHT is 1.0,1.0
}

// Default filter function
void filter(HailoROIPtr roi)
{
  std::map<int, HailoDetectionPtr> candidates;
  std::vector<HailoDetectionPtr> seen;

  for (auto obj : roi->get_objects_typed(HAILO_DETECTION)) {
    HailoDetectionPtr det = std::dynamic_pointer_cast<HailoDetection>(obj);
    std::string label = det->get_label();

#ifdef DEBUG_RULERS
    for (float x = 0.0f; x <= 1.01f; x += 0.1f) {
      for (float y = 0.0f; y <= 1.01f; y += 0.1f) {
        std::ostringstream oss;
        //oss << std::fixed << std::setprecision(2) << x << "," << y;
        oss << x/0.1f/1 << "," << y/0.1f/1;
        std::string formatted = oss.str();

        auto cpy = std::make_shared<HailoDetection>(*det);
        cpy->set_bbox(HailoBBox(x-0.01f,y-0.01f, 0.02f, 0.02f));
        cpy->set_label(formatted);
        hailo_common::add_object(roi, cpy);

        auto cpytick = std::make_shared<HailoDetection>(*det);
        cpytick->set_bbox(HailoBBox(x-0.005f,y+0.045f, 0.01f, 0.01f));
        cpytick->set_label(".");
        hailo_common::add_object(roi, cpytick);

        auto cpytick2 = std::make_shared<HailoDetection>(*det);
        cpytick2->set_bbox(HailoBBox(x+0.045f,y-0.005f, 0.01f, 0.01f));
        cpytick2->set_label(".");
        hailo_common::add_object(roi, cpytick2);

        auto cpytick8 = std::make_shared<HailoDetection>(*det);
        cpytick8->set_bbox(HailoBBox(x+0.045f,y+0.045f, 0.01f, 0.01f));
        cpytick8->set_label(".");
        hailo_common::add_object(roi, cpytick8);
      }
    }
    return;
#endif

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

  #ifdef DEBUG
  std::cout << "-[detect]----------------------- " << std::endl;
  #endif
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
        std::cout << ts_prefix() << "hid:" << id << " mapping to existing vehicle id: " << unique_id(vdet) << std::endl;
      } else {
        //it's one we weren't previously tracking, check first if in the origin zone. if not, ignore
        if (is_above(pair.second->get_bbox(), 0.7f, -0.3f)) {
          continue;
        }
        //create a new vechile detection for this candidate
        vdet = pair.second; //take the copied detection
        pair.second->set_label("");
        TurnTracker::GetInstance().add_vehicle_det(vdet);
        TurnTracker::GetInstance().map_hailo_id_to_vehicle_det(id, vdet);
        std::cout << ts_prefix() << "hid:" << id << " seems new!" << std::endl;
      }
    } else {
      #ifdef DEBUG
      std::cout << ts_prefix() << "hid:" << id << " in existing vehicle detection" << std::endl;
      #endif
    }
    auto new_bbox = pair.second->get_bbox();
    //test if we crossed into the detection zone
    if (is_above(new_bbox, -0.1f, 0.8f)) {
      vdet->set_label("Oops! ;)");
      TurnTracker::GetInstance().illegal_turn(id);
    }
    vdet->set_bbox(new_bbox);
    seen.emplace_back(vdet);
    hailo_common::add_object(roi, vdet);
    TurnTracker::GetInstance().mark_seen(vdet);
  }

  HailoUserMetaPtr illegal_turn_count = std::make_shared<HailoUserMeta>(
      static_cast<int>(TurnTracker::GetInstance().get_illegal_turn_count()),
      "right-turn-count",
      0.0f
  );
  hailo_common::add_object(roi, illegal_turn_count);

  #ifdef DEBUG
  std::cout << "-[gc]----------------------- " << std::endl;
  #endif
  TurnTracker::GetInstance().gc();
}
