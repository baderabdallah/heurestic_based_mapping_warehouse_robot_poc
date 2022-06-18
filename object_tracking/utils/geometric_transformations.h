#ifndef OBJECT_TRACKING_UTILS_GEOMETRIC_TRANSFORMATIONS_H
#define OBJECT_TRACKING_UTILS_GEOMETRIC_TRANSFORMATIONS_H

#include "object_tracking/data_types/pose_2d.h"

float DegreeToRad(float degree);
Pose2D TransformRelativeToOrigin(Pose2D object_pose, Pose2D robot_pose);
float GetNonAccumulateAngle(float given_angle);

#endif /* OBJECT_TRACKING_UTILS_GEOMETRIC_TRANSFORMATIONS_H */