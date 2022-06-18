#ifndef MAIN_WRITERS_H
#define MAIN_WRITERS_H

#include <vector>
#include <string>
#include "object_tracking/data_types/objects_types.h"
#include "nohlman/json.hpp"
#include <fstream>

void WriteOutRobotPoses(const std::vector<TimedRobotPose> &timed_robot_poses,const std::string & path);
void WriteOutDetectionPoses(const std::vector<TimedDetectionPoses>& timed_detection_poses,const std::string & path);
void WriteOutDetectionPosesInCsGloabl(const std::vector<LoadCarriers> & timed_detection_poses_global_cs, const std::string & path);

#endif /* MAIN_WRITERS_H */