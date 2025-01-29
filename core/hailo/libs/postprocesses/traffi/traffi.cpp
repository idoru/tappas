#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <set>
#include "traffi.hpp"
#include "common/nms.hpp"
#include "config.hpp"
#include "event_logger.hpp"

std::mutex lastmutex_;
std::mutex TurnTracker::mutex_;

typedef unsigned int vehicle_id;

class TurnTracker::TurnTrackerPrivate
{
  public:
    vehicle_id current_id = 0;
    std::map<int, HailoDetectionPtr> hailo_unique_id_vehicles;
    std::map<HailoDetectionPtr, int> vehicle_dets;
    std::set<int> illegal_crossing_vehicle_ids;
    std::set<int> legal_crossing_vehicle_ids;
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

void TurnTracker::track_crossing(int hailo_id, std::string from, std::string to, bool islegal) {
  auto vdet = this->get_vehicle_det_for_hailo_det(hailo_id);
  int vehicle_id = unique_id(vdet);
  //did we already mark it legal or not?
  if (priv->legal_crossing_vehicle_ids.find(vehicle_id) == priv->legal_crossing_vehicle_ids.end() &&
      priv->illegal_crossing_vehicle_ids.find(vehicle_id) == priv->illegal_crossing_vehicle_ids.end()) {
    //ok not marked, based on what we are told to track now, we're marking it legal or not.
    if (islegal) {
      if (priv->legal_crossing_vehicle_ids.find(vehicle_id) == priv->legal_crossing_vehicle_ids.end()) {
        priv->legal_crossing_vehicle_ids.insert(vehicle_id);
        std::cout << "Vehicle ID " << vehicle_id << " made a legal crossing from " << from << " to " << to << ". New total: " << priv->legal_crossing_vehicle_ids.size() << std::endl;
      }
    } else {
      if (priv->illegal_crossing_vehicle_ids.find(vehicle_id) == priv->illegal_crossing_vehicle_ids.end()) {
        priv->illegal_crossing_vehicle_ids.insert(vehicle_id);
        std::cout << "Vehicle ID " << vehicle_id << " made an illegal crossing from " << from << " to " << to << ". New total: " << priv->illegal_crossing_vehicle_ids.size() << std::endl;
      }
    }
  }
}

size_t TurnTracker::get_illegal_crossing_count() {
  std::lock_guard<std::mutex> lock(mutex_);
  return priv->illegal_crossing_vehicle_ids.size();
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
    #ifdef DEBUG
    std::cout << "Delete vehicle id: " << unique_id(todelete) << std::endl;
    #endif
    priv->vehicle_dets.erase(todelete);
    for (const auto &mapping : priv->hailo_unique_id_vehicles) {
      if (mapping.second == todelete) {
        hailo_id_deletions.emplace_back(mapping.first);
      }
    }
    for (const auto &idtodelete : hailo_id_deletions) {
      #ifdef DEBUG
      std::cout << "  was hid:" << idtodelete << std::endl;
      #endif
      priv->hailo_unique_id_vehicles.erase(idtodelete);
    }
  }
}

inline bool is_below(const HailoBBox bbox, const float y_intercept, const float slope) {
  auto x = bbox.xmin() + (bbox.width()/2.f);
  auto y = bbox.ymin() + (bbox.height()/2.f);
  return y > (y_intercept + slope * x); //TOP LEFT is 0,0, BOTTOM RIGHT is 1.0,1.0
}

inline bool test_boundary(const Config::ConfigEntry& bcfg, const HailoBBox bbox) {
  bool below = is_below(bbox, bcfg.yint, bcfg.slope);
  if (bcfg.testsbelow) {
    return below;
  }
  return !below;
}

std::vector<Config::ConfigEntry> get_triggered_entries(const HailoBBox bbox) {
  std::vector<Config::ConfigEntry> matches;
  for (const auto& entry: Config::Get().GetEntries()) {
    if (test_boundary(entry, bbox)) {
      matches.emplace_back(entry);
    }
  }
  return matches;
}

static std::vector<HailoObjectPtr> lastDetections;

// Default filter function
void filter(HailoROIPtr roi)
{
  std::map<int, HailoDetectionPtr> candidates;
  std::vector<HailoDetectionPtr> seen;

  {
    std::lock_guard<std::mutex> lock(lastmutex_);
    lastDetections = roi->get_objects_typed(HAILO_DETECTION);
  }
  for (auto obj : lastDetections) {
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
        std::cout << "NEW hid:" << id << " is existing vehicle: " << unique_id(vdet) << std::endl;
      } else {
        auto trippedBoundaries = get_triggered_entries(pair.second->get_bbox());
        if (trippedBoundaries.size()==1) {
          auto boundary = trippedBoundaries.front();
          vdet = pair.second; //take the copied detection
          pair.second->set_label(boundary.label);
          //create a new vechile detection for this candidate
          TurnTracker::GetInstance().add_vehicle_det(vdet);
          TurnTracker::GetInstance().map_hailo_id_to_vehicle_det(id, vdet);
          std::cout << "NEW hid:" << id << " is new vehicle: " << unique_id(vdet) << " at " << boundary.label << std::endl;
          if (!EventLogger::getInstance().logDetection(id, boundary.label)) {
              std::cout << "ERROR posting detection event" << std::endl;
          }
        } else {
          continue;
        }
      }
    } else {
      #ifdef DEBUG
      std::cout << "hid:" << id << " in existing vehicle detection" << std::endl;
      #endif
    }

    auto new_bbox = pair.second->get_bbox();
    auto vehicle_label = vdet->get_label();
    //consider only vehicles who havent been assigned a crossing status
    if (vehicle_label!="Oops!" && vehicle_label!="OK") {
      auto trippedBoundaries = get_triggered_entries(new_bbox);
      if (trippedBoundaries.size() > 1) {
        continue;
      } else if (trippedBoundaries.size()==1) {
        auto trippedBoundary = trippedBoundaries.front();
        if (trippedBoundary.label == vehicle_label) {
          continue;
        }
        auto prohibition_match = std::find(trippedBoundary.prohibited.begin(), trippedBoundary.prohibited.end(), vdet->get_label());
        bool is_legal = (prohibition_match == trippedBoundary.prohibited.end());
        if (!EventLogger::getInstance().logCrossing(id, trippedBoundary.label, vdet->get_label(), is_legal)) {
            std::cout << "ERROR posting detection event" << std::endl;
        }
        TurnTracker::GetInstance().track_crossing(id, vehicle_label, trippedBoundary.label, is_legal);
        if (is_legal) {
          vdet->set_label("OK");
        } else {
          vdet->set_label("Oops!");
        }
      }
    }

    vdet->set_bbox(new_bbox);
    seen.emplace_back(vdet);
    hailo_common::add_object(roi, vdet);
    TurnTracker::GetInstance().mark_seen(vdet);
  }

  HailoUserMetaPtr illegal_turn_count = std::make_shared<HailoUserMeta>(
      static_cast<int>(TurnTracker::GetInstance().get_illegal_crossing_count()),
      "illegal-crossing-count",
      0.0f
  );
  hailo_common::add_object(roi, illegal_turn_count);

  #ifdef DEBUG
  std::cout << "-[gc]----------------------- " << std::endl;
  #endif
  TurnTracker::GetInstance().gc();
}

static unsigned int dumpcount;

void dump_dets(HailoROIPtr roi)
{
    std::lock_guard<std::mutex> lock(lastmutex_);
    std::cout << "taking snapshot " << dumpcount << std::endl;
    std::string filename = "/var/local/traffi/infs/result_" + std::to_string(dumpcount).insert(0, 5 - std::to_string(dumpcount).length(), '0') + ".json";
    std::ofstream outfile(filename);

    outfile << "[\n";
    bool first = true;
    for (auto obj : lastDetections) {
        HailoDetectionPtr det = std::dynamic_pointer_cast<HailoDetection>(obj);
        auto bbox = det->get_bbox();
        std::string label = det->get_label();
        float confidence = det->get_confidence();

        if (!first) {
            outfile << ",\n";
        }
        first = false;

        outfile << "  {\n";
        outfile << "    \"label\": \"" << label << "\",\n";
        outfile << "    \"confidence\": " << confidence << ",\n";
        outfile << "    \"bbox\": {\n";
        outfile << "      \"minx\": " << bbox.xmin() << ",\n";
        outfile << "      \"miny\": " << bbox.ymin() << ",\n";
        outfile << "      \"maxx\": " << bbox.xmax() << ",\n";
        outfile << "      \"maxy\": " << bbox.ymax() << "\n";
        outfile << "    }\n";
        outfile << "  }";
    }
    outfile << "\n]";

    outfile.close();

    dumpcount++;
}
