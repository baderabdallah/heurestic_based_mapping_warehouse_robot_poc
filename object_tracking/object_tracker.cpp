#include "object_tracking/object_tracker.h"
#include "object_tracking/utils/geometric_transformations.h"
#include <math.h>
#include <algorithm>
#include <cmath>

static constexpr double kPi{3.14159265358979323846};

static constexpr float kLoadCarrierPositionTolerance{0.2};

static const float GetAngleTolerance()
{
    return DegreeToRad(6.0F);
}

namespace
{
    double CalculateEuclideanDistance(const Pose2D &first_pose, const Pose2D &second_pose)
    {
        return sqrt(pow((first_pose.x - second_pose.x), 2) + pow((first_pose.y - second_pose.y), 2));
    }

    bool IsCarrierPositionWithinTolerance(const Pose2D &reference_position, const Pose2D &input_position)
    {
        double euclidean_distance = CalculateEuclideanDistance(reference_position, input_position);
        return euclidean_distance < kLoadCarrierPositionTolerance;
    }

    bool IsAngleWithinTolerance(float reference_angle, float input_angle)
    {
        float delta_angle = std::abs(reference_angle - input_angle);
        return delta_angle < GetAngleTolerance();
    }

    bool IsUndetectedPose(const Pose2D &pose, const std::vector<Carrier> &loading_carriers, int &index_existing_pose)
    {
        for (int i{0}; i < loading_carriers.size(); i++)
        {
            if (IsCarrierPositionWithinTolerance(pose, loading_carriers[i].pose_2d))
            {
                index_existing_pose = i;
                return false;
            }
        }
        return true;
    }

    bool IsOrientationInVector(const float potential_orientation, const std::vector<PotentialOrientation> &given_potential_orientations, int &index)
    {

        for (int i{0}; i < given_potential_orientations.size(); i++)
        {
            if (IsAngleWithinTolerance(potential_orientation, given_potential_orientations[i].orientation))
            {
                index = i;
                return true;
            }
        }
        return false;
    }

    float GetOrientationOfMaximumCount(const std::vector<PotentialOrientation> &potential_orienations)
    {

        auto orientation_with_max_count = std::max_element(potential_orienations.cbegin(), potential_orienations.cend(), [](const PotentialOrientation &pot_a, const PotentialOrientation &pot_b)
                                                           { return pot_b.count > pot_a.count; });
        return orientation_with_max_count->orientation;
    }

    void WeighInTheNewFoundPosition(Carrier &existing_carrier, const Pose2D &found_pose)
    {
        auto new_contribution_ratio = 1.0F / existing_carrier.count;
        auto cummulative_contribution_ratio = 1.0F * (existing_carrier.count - 1) / existing_carrier.count;
        existing_carrier.pose_2d.x = found_pose.x * new_contribution_ratio + cummulative_contribution_ratio * existing_carrier.pose_2d.x;
        existing_carrier.pose_2d.y = found_pose.y * new_contribution_ratio + cummulative_contribution_ratio * existing_carrier.pose_2d.y;
    }

    float GetEstimateOrientation(std::vector<PotentialOrientation> &potential_orientations, float new_potential_orientation, float current_orientation)
    {
        int index_potential_orientation{0};
        if (IsOrientationInVector(new_potential_orientation, potential_orientations, index_potential_orientation))
        {
            potential_orientations[index_potential_orientation].count++;
            auto angle_count = potential_orientations[index_potential_orientation].count;
            auto new_contribution_ratio_angle = 1.0F / angle_count;
            auto cummulative_contribution_ratio_angle = 1.0F * (angle_count - 1) / angle_count;
            potential_orientations[index_potential_orientation].orientation = new_potential_orientation * new_contribution_ratio_angle + cummulative_contribution_ratio_angle * current_orientation;
        }
        else
        {
            potential_orientations.push_back(PotentialOrientation{new_potential_orientation, 1});
        }
        return GetOrientationOfMaximumCount(potential_orientations);
    }
}

void ObjectTracker::Update(const std::vector<TimedRobotPose> &robot_poses, const std::vector<TimedDetectionPoses> &detections_poses)
{
    robot_poses_ = robot_poses;
    detections_poses_ = detections_poses;
}

void ObjectTracker::ProduceDetectionPosesInGlobalCs()
{
    for (std::size_t i{0}; i < robot_poses_.size(); i++)
    {
        std::vector<Carrier> current_load_carriers_estimated_poses{};
        Pose2D robot_pose = robot_poses_[i].pose_2d;

        TimeSec load_carrier_time_sec = detections_poses_[i].time;
        if (i > 0)
        {
            for (auto &carrier : timed_load_carriers_collector_[i - 1].carriers)
            {
                current_load_carriers_estimated_poses.push_back(carrier);
            }
        }

        for (auto &pose : detections_poses_[i].poses)
        {
            Pose2D load_carier_global = TransformRelativeToOrigin(pose, robot_pose);
            int index{-1};
            bool is_new_pose = IsUndetectedPose(load_carier_global, current_load_carriers_estimated_poses, index);
            if (is_new_pose)
            {
                current_load_carriers_estimated_poses.push_back(Carrier{Pose2D{load_carier_global.x, load_carier_global.y, GetNonAccumulateAngle(load_carier_global.orientation)},
                                                                        std::vector<PotentialOrientation>{PotentialOrientation{GetNonAccumulateAngle(load_carier_global.orientation), 1}}, 1});
            }
            else
            {
                current_load_carriers_estimated_poses[index].count++;
                WeighInTheNewFoundPosition(current_load_carriers_estimated_poses[index], load_carier_global);
                float potential_orientation = GetNonAccumulateAngle(load_carier_global.orientation);
                current_load_carriers_estimated_poses[index].pose_2d.orientation = GetEstimateOrientation(current_load_carriers_estimated_poses[index].potential_orientations, potential_orientation, current_load_carriers_estimated_poses[index].pose_2d.orientation);
            }
        }
        timed_load_carriers_collector_.push_back(LoadCarriers{load_carrier_time_sec, current_load_carriers_estimated_poses});
    }
}
