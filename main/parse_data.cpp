#include "main/parse_data.h"
#include <string.h>
#include <fstream>

using nlohmann::json;

nlohmann::json ReadJson(const std::string &path)
{
  std::ifstream input_stream(path, std::ifstream::in);
  nlohmann::json json_file;
  input_stream >> json_file;
  input_stream.close();
  return json_file;
}

void ParseRobotData(std::vector<TimedRobotPose> &robotPoses, const nlohmann::json &json_file)
{

  for (const auto &pose : json_file["robotPose"])
    robotPoses.emplace_back(TimedRobotPose{pose["time"].get<double>(), Pose2D{pose["x"].get<float>(),
                                                                              pose["y"].get<float>(),
                                                                              pose["theta"].get<float>()}});
}

void ParseDetectionData(std::vector<TimedDetectionPoses> &detections, const nlohmann::json &json_file)
{
  for (const auto &detection : json_file["detections"])
  {
    std::vector<Pose2D> poses;
    for (const auto &object : detection["poses"])
      poses.emplace_back(Pose2D{object["x"].get<float>(),
                                object["y"].get<float>(),
                                object["theta"].get<float>()});
    detections.emplace_back(TimedDetectionPoses{detection["time"].get<double>(), poses});
  }
}