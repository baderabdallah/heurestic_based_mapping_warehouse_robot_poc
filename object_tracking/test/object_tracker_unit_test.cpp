#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <object_tracking/object_tracker.h>
#include "object_tracking/utils/geometric_transformations.h"

using ::testing::Eq;
using ::testing::FloatEq;
namespace
{
  const double kPi = 3.14159265358979323846;
}

class ObjectTrackerFixture : public ::testing::Test
{
protected:
  const Pose2D a_load_carrier_pose{Pose2D{20.0F, 30.0F, DegreeToRad(0.0F)}};
  const Pose2D b_load_carrier_pose{Pose2D{50.0F, 50.0F, DegreeToRad(0.0F)}};
  const Pose2D c_load_carrier_pose{Pose2D{50.0F, 50.0F, DegreeToRad(50.0F)}};

  ObjectTracker unit_{};
};

TEST_F(ObjectTrackerFixture, GiveRobotAndDetectionPoses_ExpectGlobalDetectionsCalculatedCorrectly)
{
  const std::vector<TimedRobotPose> some_robot_poses{
      TimedRobotPose{0.0, Pose2D{1.0F, 3.0F, 0.0F}},
      TimedRobotPose{1.0, Pose2D{1.0F, 3.0F, 0.0F}},
  };

  const std::vector<TimedDetectionPoses> some_detection_poses{
      TimedDetectionPoses{0.0, {a_load_carrier_pose, b_load_carrier_pose}},
      TimedDetectionPoses{1.0, {a_load_carrier_pose, b_load_carrier_pose}},
  };

  unit_.Update(some_robot_poses, some_detection_poses);
  unit_.ProduceDetectionPosesInGlobalCs();

  auto load_carrier_A_snap_0 = unit_.GetLoadCarriersPosesInCsGlobal()[0].carriers[0].pose_2d;
  auto load_carrier_B_snap_0 = unit_.GetLoadCarriersPosesInCsGlobal()[0].carriers[1].pose_2d;

  EXPECT_THAT(20.0F + 1.0F, FloatEq(load_carrier_A_snap_0.x));
  EXPECT_THAT(30.0F + 3.0F, FloatEq(load_carrier_A_snap_0.y));

  EXPECT_THAT(50.0F + 1.0F, FloatEq(load_carrier_B_snap_0.x));
  EXPECT_THAT(50.0F + 3.0F, FloatEq(load_carrier_B_snap_0.y));

  auto load_carrier_A_snap_1 = unit_.GetLoadCarriersPosesInCsGlobal()[1].carriers[0].pose_2d;
  auto load_carrier_B_snap_1 = unit_.GetLoadCarriersPosesInCsGlobal()[1].carriers[1].pose_2d;

  auto load_carrier_B_snap_1_count = unit_.GetLoadCarriersPosesInCsGlobal()[1].carriers[1].count;

  EXPECT_THAT(20.0F + 1.0F, FloatEq(load_carrier_A_snap_1.x));
  EXPECT_THAT(30.0F + 3.0F, FloatEq(load_carrier_A_snap_1.y));

  EXPECT_THAT(50.0F + 1.0F, FloatEq(load_carrier_B_snap_1.x));
  EXPECT_THAT(50.0F + 3.0F, FloatEq(load_carrier_B_snap_1.y));
}

TEST_F(ObjectTrackerFixture, GiveDetectionsWithRepeatingPositions_ExpectOneOutputDetection)
{
  const std::vector<TimedRobotPose> some_robot_poses{
      TimedRobotPose{0.0, Pose2D{1.0F, 3.0F, 0.0F}},
  };

  const std::vector<TimedDetectionPoses> some_detection_poses{
      TimedDetectionPoses{0.0, {a_load_carrier_pose, a_load_carrier_pose, a_load_carrier_pose, a_load_carrier_pose}},
  };

  unit_.Update(some_robot_poses, some_detection_poses);
  unit_.ProduceDetectionPosesInGlobalCs();

  auto load_carrier_A_snap_0 = unit_.GetLoadCarriersPosesInCsGlobal()[0].carriers[0].pose_2d;
  auto carriers_size = unit_.GetLoadCarriersPosesInCsGlobal()[0].carriers.size();

  EXPECT_THAT(20.0F + 1.0F, FloatEq(load_carrier_A_snap_0.x));
  EXPECT_THAT(1, Eq(carriers_size));
}

TEST_F(ObjectTrackerFixture, GiveDetectionsWithRepeatingPositions_ExpectCorrectDetectionCount)
{
  const std::vector<TimedRobotPose> some_robot_poses{
      TimedRobotPose{0.0, Pose2D{1.0F, 3.0F, 0.0F}},
  };

  const std::vector<TimedDetectionPoses> some_detection_poses{
      TimedDetectionPoses{0.0, {a_load_carrier_pose, a_load_carrier_pose, a_load_carrier_pose, a_load_carrier_pose}},
  };

  unit_.Update(some_robot_poses, some_detection_poses);
  unit_.ProduceDetectionPosesInGlobalCs();

  auto load_carrier_A_snap_0 = unit_.GetLoadCarriersPosesInCsGlobal()[0].carriers[0];
  auto carrier_count = load_carrier_A_snap_0.count;

  EXPECT_THAT(4, Eq(carrier_count));
}

TEST_F(ObjectTrackerFixture, GiveDetectionsWithRepeatingPositions_ExpectResultHasAveragePosition)
{
  const std::vector<TimedRobotPose> some_robot_poses{
      TimedRobotPose{0.0, Pose2D{1.0F, 3.0F, 0.0F}},
  };
  auto c1 = a_load_carrier_pose;
  auto c2 = a_load_carrier_pose;
  c2.x = c2.x + 0.1;
  auto c3 = a_load_carrier_pose;
  c3.x = c3.x + 0.2;

  const std::vector<TimedDetectionPoses> some_detection_poses{
      TimedDetectionPoses{0.0, {c1, c2, c3}},
  };

  unit_.Update(some_robot_poses, some_detection_poses);
  unit_.ProduceDetectionPosesInGlobalCs();

  auto load_carrier_A_snap_0 = unit_.GetLoadCarriersPosesInCsGlobal()[0].carriers[0];

  EXPECT_THAT(c2.x + 1.0F, FloatEq(load_carrier_A_snap_0.pose_2d.x));
}

TEST_F(ObjectTrackerFixture, GiveDetectionsWithRepeatingPositionsOfAPreviousDetection_ExpectResultHasAverageOverallPosition)
{
  const std::vector<TimedRobotPose> some_robot_poses{
      TimedRobotPose{0.0, Pose2D{1.0F, 3.0F, 0.0F}},
      TimedRobotPose{0.0, Pose2D{1.0F, 3.0F, 0.0F}},
  };
  auto c1 = a_load_carrier_pose;
  auto c2 = a_load_carrier_pose;
  c2.x = c2.x + 0.1;
  auto c3 = a_load_carrier_pose;
  c3.x = c3.x + 0.2;

  const std::vector<TimedDetectionPoses> some_detection_poses{
      TimedDetectionPoses{0.0, {c1, c2, c3}},
      TimedDetectionPoses{0.0, {c1, c1, c1}},
  };

  unit_.Update(some_robot_poses, some_detection_poses);
  unit_.ProduceDetectionPosesInGlobalCs();

  auto load_carrier_A_snap_0 = unit_.GetLoadCarriersPosesInCsGlobal()[1].carriers[0];

  EXPECT_THAT(20.05F + 1.0F, FloatEq(load_carrier_A_snap_0.pose_2d.x));
}

TEST_F(ObjectTrackerFixture, GiveDetectionsWithOscilatingDirections_ExpectResultOrientationIsTheMostDetected)
{
  const std::vector<TimedRobotPose> some_robot_poses{
      TimedRobotPose{0.0, Pose2D{1.0F, 3.0F, 0.0F}},
  };

  auto c1 = c_load_carrier_pose;
  auto c2 = c_load_carrier_pose;
  auto c3 = c_load_carrier_pose;
  c3.orientation = c3.orientation + kPi;

  const std::vector<TimedDetectionPoses> some_detection_poses{
      TimedDetectionPoses{0.0, {c1, c2, c3}},
  };

  unit_.Update(some_robot_poses, some_detection_poses);
  unit_.ProduceDetectionPosesInGlobalCs();

  auto load_carrier_C_snap_0 = unit_.GetLoadCarriersPosesInCsGlobal()[0].carriers[0];

  EXPECT_THAT(DegreeToRad(50.0F), FloatEq(load_carrier_C_snap_0.pose_2d.orientation));
}

TEST_F(ObjectTrackerFixture, GiveDetectionsWithSlightlyNoPreciseOrientation_ExpectResultOrientationIsTheMostDetected)
{
  const std::vector<TimedRobotPose> some_robot_poses{
      TimedRobotPose{0.0, Pose2D{1.0F, 3.0F, 0.0F}},
  };

  auto c1 = c_load_carrier_pose;
  c1.orientation = c1.orientation + DegreeToRad(0.0F);
  auto c2 = c_load_carrier_pose;
  c2.orientation = c2.orientation + DegreeToRad(-5.0F);
  auto c3 = c_load_carrier_pose;

  c3.orientation = c3.orientation + kPi;

  const std::vector<TimedDetectionPoses> some_detection_poses{
      TimedDetectionPoses{0.0, {c1, c2, c3}},
  };

  unit_.Update(some_robot_poses, some_detection_poses);
  unit_.ProduceDetectionPosesInGlobalCs();

  auto load_carrier_C_snap_0 = unit_.GetLoadCarriersPosesInCsGlobal()[0].carriers[0];
  EXPECT_THAT(DegreeToRad(47.5F), FloatEq(load_carrier_C_snap_0.pose_2d.orientation));
}
