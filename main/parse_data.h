#ifndef MAIN_PARSE_DATA_H
#define MAIN_PARSE_DATA_H

#include <fstream>
#include "nohlman/json.hpp"
#include "object_tracking/data_types/pose_2d.h"
#include "object_tracking/data_types/objects_types.h"

nlohmann::json ReadJson(const std::string &path);
void ParseRobotData(std::vector<TimedRobotPose> &robotPoses, const nlohmann::json &json_file);
void ParseDetectionData(std::vector<TimedDetectionPoses> &detections, const nlohmann::json &json_file);

#endif /* MAIN_PARSE_DATA_H */