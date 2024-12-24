#pragma once
#include "hailo_objects.hpp"
#include "hailo_common.hpp"

__BEGIN_DECLS

typedef unsigned int vehicle_id;

void filter(HailoROIPtr roi);

class TurnTracker
{
private:
    class TurnTrackerPrivate;
    std::unique_ptr<TurnTrackerPrivate> priv;
    TurnTracker(const TurnTracker &) = delete;
    TurnTracker &operator=(const TurnTracker &) = delete;
    ~TurnTracker();
    TurnTracker();
    static std::mutex mutex_;

public:
    static TurnTracker &GetInstance();
    HailoDetectionPtr get_vehicle_det_for_hailo_det(int hailo_id);
    HailoDetectionPtr get_vehicle_det_matching_hailo_det_iou(HailoDetectionPtr hailo_det);
    void map_hailo_id_to_vehicle_det(int hailo_id, HailoDetectionPtr vehicle_det);
    void add_vehicle_det(HailoDetectionPtr vehicle_det);
    std::map<HailoDetectionPtr, int> vehicle_detections();
    void mark_seen(HailoDetectionPtr vehicle_det);
    void gc();
    void illegal_turn(int hailo_id);
//    void add_jde_tracker(const std::string &name, HailoTrackerParams params);
//    void add_jde_tracker(const std::string &name);
//    void remove_jde_tracker(const std::string &name);
//    std::vector<std::string> get_trackers_list();
//    std::vector<HailoDetectionPtr> update(const std::string &name, std::vector<HailoDetectionPtr> &inputs);
//    void add_object_to_track(const std::string &name, int id, HailoObjectPtr obj);
//    void remove_classifications_from_track(const std::string &name, int track_id, std::string classifier_type);
//    void remove_matrices_from_track(const std::string &name, int track_id);
//
//    // Setters for members accessible at element-property level
//    void set_kalman_distance(const std::string &name, float new_distance);
//    void set_iou_threshold(const std::string &name, float new_iou_thr);
//    void set_init_iou_threshold(const std::string &name, float new_init_iou_thr);
//    void set_keep_tracked_frames(const std::string &name, int new_keep_tracked);
//    void set_keep_new_frames(const std::string &name, int new_keep_new);
//    void set_keep_lost_frames(const std::string &name, int new_keep_lost);
//    void set_keep_past_metadata(const std::string &name, bool new_keep_past);
//    void set_std_weight_position(const std::string &name, float new_std_weight_pos);
//    void set_std_weight_position_box(const std::string &name, float new_std_weight_position_box);
//    void set_std_weight_velocity(const std::string &name, float new_std_weight_vel);
//    void set_std_weight_velocity_box(const std::string &name, float new_std_weight_velocity_box);
//    void set_debug(const std::string &name, bool new_debug);
//    void set_hailo_objects_blacklist(const std::string &name, std::vector<hailo_object_t> hailo_objects_blacklist_vec);
};
__END_DECLS

