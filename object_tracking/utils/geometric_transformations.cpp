#include "object_tracking/utils/geometric_transformations.h"
#include <cmath>

#define PI 3.14159265358979323846

float DegreeToRad(float degree)
{
    return degree / (180 / PI);
}

Pose2D TransformRelativeToOrigin(Pose2D object_pose, Pose2D robot_pose)
{
    return Pose2D{object_pose.x * cos(robot_pose.orientation) - object_pose.y * sin(robot_pose.orientation) + robot_pose.x,
                  object_pose.x * sin(robot_pose.orientation) + object_pose.y * cos(robot_pose.orientation) + robot_pose.y,
                  object_pose.orientation + robot_pose.orientation};
}

float GetNonAccumulateAngle(float given_angle)
{
    float angle_less_than_pi{given_angle};
    while (angle_less_than_pi > PI)
    {
        angle_less_than_pi = angle_less_than_pi - 2 * PI;
    }

    while (angle_less_than_pi < -PI)
    {
        angle_less_than_pi = angle_less_than_pi + 2 * PI;
    }
    return angle_less_than_pi;
}