#include "gtest/gtest.h"
#include "config.h"

TEST(AvailableLanes, hasFarSideLane) {
  AvailableLanes lanes = AvailableLanes(3);
  EXPECT_EQ(true, lanes.hasFarSideLane(Lane(0)));
  EXPECT_EQ(true, lanes.hasFarSideLane(Lane(1)));
  EXPECT_EQ(false, lanes.hasFarSideLane(Lane(2)));
  EXPECT_EQ(false, lanes.hasFarSideLane(Lane(3)));
};

TEST(AvailableLanes, farSideLane) {
  AvailableLanes lanes = AvailableLanes(3);
  ASSERT_TRUE(lanes.getLane(1).laneNumberFromMiddleOfRoad == lanes.farSideLane(Lane(0)).laneNumberFromMiddleOfRoad);
  ASSERT_TRUE(lanes.getLane(2).laneNumberFromMiddleOfRoad == lanes.farSideLane(Lane(1)).laneNumberFromMiddleOfRoad);
  ASSERT_ANY_THROW(lanes.farSideLane(Lane(2)));
};

TEST(AvailableLanes, hasNearSideLane) {
  AvailableLanes lanes = AvailableLanes(3);
  EXPECT_EQ(false, lanes.hasNearSideLane(Lane(0)));
  EXPECT_EQ(true, lanes.hasNearSideLane(Lane(1)));
  EXPECT_EQ(true, lanes.hasNearSideLane(Lane(2)));
  EXPECT_EQ(false, lanes.hasNearSideLane(Lane(3)));
  EXPECT_EQ(false, lanes.hasNearSideLane(Lane(4)));
};

TEST(AvailableLanes, nearSideLane) {
  AvailableLanes lanes = AvailableLanes(3);
  ASSERT_ANY_THROW(lanes.nearSideLane(Lane(0)));
  EXPECT_EQ(lanes.getLane(0).laneNumberFromMiddleOfRoad, lanes.nearSideLane(Lane(1)).laneNumberFromMiddleOfRoad);
  EXPECT_EQ(lanes.getLane(1).laneNumberFromMiddleOfRoad, lanes.nearSideLane(Lane(2)).laneNumberFromMiddleOfRoad);
};