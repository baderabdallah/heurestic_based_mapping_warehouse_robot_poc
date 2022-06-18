#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "object_tracking/utils/geometric_transformations.h"
using ::testing::Eq;
using ::testing::FloatEq;

TEST(TransformRelativeToOrigin, GivenARobotPoseWithNoOrientation_ExpectCorrectTransofrmation)
{
    /// Given
    Pose2D a_robot_pose_2d_wrt_origin{10.0F, 15.0F, 0.0F};
    Pose2D an_object_pose_2d_wrt_robot{2.0F, 3.0F, DegreeToRad(160.0F)};

    /// When
    Pose2D results = TransformRelativeToOrigin(an_object_pose_2d_wrt_robot, a_robot_pose_2d_wrt_origin);

    Pose2D expected_results = Pose2D{a_robot_pose_2d_wrt_origin.x + an_object_pose_2d_wrt_robot.x,
                                     a_robot_pose_2d_wrt_origin.y + an_object_pose_2d_wrt_robot.y,
                                     a_robot_pose_2d_wrt_origin.orientation + an_object_pose_2d_wrt_robot.orientation};

    EXPECT_THAT(results.x, FloatEq(expected_results.x));
    EXPECT_THAT(results.y, FloatEq(expected_results.y));
    EXPECT_THAT(results.orientation, FloatEq(expected_results.orientation));
}

TEST(TransformRelativeToOrigin, GivenARobotPoseWithOrientation_ExpectCorrectTransofrmation)
{
    /// Given
    Pose2D a_robot_pose_2d_wrt_origin{0.0, 0.0F, DegreeToRad(90.0F)};
    Pose2D an_object_pose_2d_wrt_robot{40.0F, 50.0F, DegreeToRad(160.0F)};

    /// When
    Pose2D results = TransformRelativeToOrigin(an_object_pose_2d_wrt_robot, a_robot_pose_2d_wrt_origin);

    Pose2D expected_results = Pose2D{-50.0F,
                                     40.0F,
                                     DegreeToRad(90.0F + 160.0F)};

    EXPECT_THAT(results.x, FloatEq(expected_results.x));
    EXPECT_THAT(results.y, FloatEq(expected_results.y));
    EXPECT_THAT(results.orientation, FloatEq(expected_results.orientation));
}

TEST(TransformRelativeToOrigin, GivenAnAngleMoreThanPi_ExpectTransofmredAngleLessThanPiEquivalent)
{
    /// Given
    /// When
    float results = GetNonAccumulateAngle(DegreeToRad(250.0F));

    EXPECT_THAT(results, FloatEq(DegreeToRad(-110)));
}

TEST(TransformRelativeToOrigin, GivenAnAngleLessThanNegativePi_ExpectTransofmredAngleLessThanPiEquivalent)
{
    /// Given
    /// When
    float results = GetNonAccumulateAngle(DegreeToRad(-450.0F));

    EXPECT_THAT(results, FloatEq(DegreeToRad(-90)));
}
