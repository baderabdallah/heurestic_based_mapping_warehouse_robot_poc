#include "main/writers.h"

void WriteOutRobotPoses(const std::vector<TimedRobotPose> &timed_robot_poses, const std::string &path)
{
  nlohmann::json json_file_output_robot_poses;
  nlohmann::json pose_j;
  for (auto &pose : timed_robot_poses)
  {
    pose_j["time"] = std::to_string(pose.time);
    pose_j["x"] = std::to_string(pose.pose_2d.x);
    pose_j["y"] = std::to_string(pose.pose_2d.y);
    pose_j["theta"] = std::to_string(pose.pose_2d.orientation);
    json_file_output_robot_poses["robotPose"].push_back(pose_j);
  }

  std::ofstream file_robot(path);
  file_robot << std::setw(4) << json_file_output_robot_poses;
}

void WriteOutDetectionPoses(const std::vector<TimedDetectionPoses> &timed_detection_poses, const std::string &path)
{
  nlohmann::json json_file_output_detection;
  nlohmann::json detection_j;
  nlohmann::json poses_detection_j;
  for (auto &detection_poses : timed_detection_poses)
  {
    for (auto &pose_d : detection_poses.poses)
    {
      detection_j["x"] = std::to_string(pose_d.x);
      detection_j["y"] = std::to_string(pose_d.y);
      detection_j["theta"] = std::to_string(pose_d.orientation);
      poses_detection_j["poses"].push_back(detection_j);
      detection_j.clear();
    }
    poses_detection_j["time"].push_back(std::to_string(detection_poses.time));
    json_file_output_detection["detections"].push_back(poses_detection_j);
    poses_detection_j.clear();
  }

  std::ofstream file_detection(path);
  file_detection << std::setw(4) << json_file_output_detection;
}

void WriteOutDetectionPosesInCsGloabl(const std::vector<LoadCarriers> &timed_detection_poses_global_cs, const std::string &path)
{
  nlohmann::json json_file_output_result;
  nlohmann::json detection_j_result;
  nlohmann::json poses_detection_j_result;

  for (auto &detection_poses : timed_detection_poses_global_cs)
  {
    for (auto &carrier : detection_poses.carriers)
    {
      detection_j_result["x"] = std::to_string(carrier.pose_2d.x);
      detection_j_result["y"] = std::to_string(carrier.pose_2d.y);
      detection_j_result["theta"] = std::to_string(carrier.pose_2d.orientation);
      poses_detection_j_result["poses"].push_back(detection_j_result);
      detection_j_result.clear();
    }
    poses_detection_j_result["time"].push_back(std::to_string(detection_poses.time));
    json_file_output_result["detections"].push_back(poses_detection_j_result);
    poses_detection_j_result.clear();
  }

  std::ofstream file_detection_result(path);
  file_detection_result << std::setw(4) << json_file_output_result;
}