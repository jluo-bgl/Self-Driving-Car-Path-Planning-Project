#include "gtest/gtest.h"
#include <functional>
#include <iostream>
#include "cmath"

using namespace std;

float goal_distance_cost(int goal_lane, int intended_lane, int final_lane, float distance_to_goal) {
  /*
  The cost increases with both the distance of intended lane from the goal
  and the distance of the final lane from the goal. The cost of being out of the
  goal lane also becomes larger as vehicle approaches the goal.
  */

  int delta_d = 2.0*goal_lane - intended_lane - final_lane;
  float cost = 1 - exp(-(abs(delta_d) / distance_to_goal));
  return cost;
}

float inefficiency_cost(int target_speed, int intended_lane, int final_lane, vector<int> lane_speeds) {
  /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than target_speed.
    */
  float speed_intended = lane_speeds[intended_lane];
  float speed_final = lane_speeds[final_lane];
  float cost = (2.0*target_speed - speed_intended - speed_final)/target_speed;
  return cost;
}

TEST(PathPlanStrategyCostFunctions, toChooseALane) {
  int goal_lane = 0;

  //Test cases used for grading - do not change.
  float cost;
  cout << "Costs for (intended_lane, final_lane, goal_distance):" << endl;
  cout << "----------------------------------------------------------" << endl;
  cost = goal_distance_cost(goal_lane, 2, 2, 1.0);
  EXPECT_FLOAT_EQ(0.98168439, cost);
  cout << "The cost is " << cost << " for " << "(2, 2, 1.0)" << endl;

  cost = goal_distance_cost(goal_lane, 2, 2, 10.0);
  EXPECT_FLOAT_EQ(0.32967997, cost);
  cout << "The cost is " << cost << " for " << "(2, 2, 10.0)" << endl;

  cost = goal_distance_cost(goal_lane, 2, 2, 100.0);
  EXPECT_FLOAT_EQ(0.039210558, cost);
  cout << "The cost is " << cost << " for " << "(2, 2, 100.0)" << endl;

  cost = goal_distance_cost(goal_lane, 1, 2, 100.0);
  EXPECT_FLOAT_EQ(0.029554486, cost);
  cout << "The cost is " << cost << " for " << "(1, 2, 100.0)" << endl;

  cost = goal_distance_cost(goal_lane, 1, 1, 100.0);
  EXPECT_FLOAT_EQ(0.019801319, cost);
  cout << "The cost is " << cost << " for " << "(1, 1, 100.0)" << endl;

  cost = goal_distance_cost(goal_lane, 0, 1, 100.0);
  EXPECT_FLOAT_EQ(0.00995016, cost);
  cout << "The cost is " << cost << " for " << "(0, 1, 100.0)" << endl;

  cost = goal_distance_cost(goal_lane, 0, 0, 100.0);
  EXPECT_FLOAT_EQ(0.0, cost);
  cout << "The cost is " << cost << " for " << "(0, 0, 100.0)" << endl;

  EXPECT_EQ(1, 1);
};

TEST(PathPlanStrategyCostFunctions, toChooseALaneWithDifferentLaneSpeeds) {
  //Target speed of our vehicle
  int target_speed = 10;

  //Lane speeds for each lane
  vector<int> lane_speeds = {6, 7, 8, 9};

  //Test cases used for grading - do not change.
  float cost;
  cout << "Costs for (intended_lane, final_lane):" << endl;
  cout << "----------------------------------------------------------" << endl;
  cost = inefficiency_cost(target_speed, 3, 3, lane_speeds);
  cout << "The cost is " << cost << " for " << "(2, 2)" << endl;
  cost = inefficiency_cost(target_speed, 2, 3, lane_speeds);
  cout << "The cost is " << cost << " for " << "(2, 2)" << endl;
  cost = inefficiency_cost(target_speed, 2, 2, lane_speeds);
  cout << "The cost is " << cost << " for " << "(2, 2)" << endl;
  cost = inefficiency_cost(target_speed, 1, 2, lane_speeds);
  cout << "The cost is " << cost << " for " << "(1, 2)" << endl;
  cost = inefficiency_cost(target_speed, 1, 1, lane_speeds);
  cout << "The cost is " << cost << " for " << "(1, 1)" << endl;
  cost = inefficiency_cost(target_speed, 0, 1, lane_speeds);
  cout << "The cost is " << cost << " for " << "(0, 1)" << endl;
  cost = inefficiency_cost(target_speed, 0, 0, lane_speeds);
  cout << "The cost is " << cost << " for " << "(0, 0)" << endl;
};
