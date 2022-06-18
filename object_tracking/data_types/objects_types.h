#ifndef OBJECT_TRACKING_DATA_TYPES_OBJECTS_TYPES_H
#define OBJECT_TRACKING_DATA_TYPES_OBJECTS_TYPES_H

#include "object_tracking/data_types/pose_2d.h"
#include <vector>

using TimeSec = double;
using LoadCarrierId = int;

struct TimedRobotPose {
    TimeSec time{};
    Pose2D pose_2d{};
}; 
struct TimedDetectionPoses{
    TimeSec time{};
    std::vector<Pose2D> poses{};
};

struct PotentialOrientation
{
    float orientation{0};
    int count{0};
};

struct Carrier
{
    Pose2D pose_2d{};
    std::vector<PotentialOrientation> potential_orientations{};
    int count{0};
};

struct LoadCarriers
{
TimeSec time{};
std::vector<Carrier> carriers{};
};

#endif /* OBJECT_TRACKING_DATA_TYPES_OBJECTS_TYPES_H */
