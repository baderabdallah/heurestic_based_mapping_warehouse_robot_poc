#ifndef OBJECT_TRACKING_OBJECT_TRACKER_H
#define OBJECT_TRACKING_OBJECT_TRACKER_H

#include <vector>
#include <utility>
#include "object_tracking/data_types/pose_2d.h"
#include "object_tracking/data_types/objects_types.h"

class ObjectTracker
{
public:
    void Update(const std::vector<TimedRobotPose> &robot_poses, const std::vector<TimedDetectionPoses>& detection_poses) ;
    void ProduceDetectionPosesInGlobalCs();
    std::vector<LoadCarriers> GetLoadCarriersPosesInCsGlobal() const { return timed_load_carriers_collector_; }

private:
    std::vector<TimedRobotPose> robot_poses_{};
    std::vector<TimedDetectionPoses> detections_poses_{};
    std::vector<LoadCarriers> timed_load_carriers_collector_{};
};

#endif /* OBJECT_TRACKING_OBJECT_TRACKER_H */