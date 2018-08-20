#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <array>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "path_plan_strategy.h"

class CostInfo {
 public:
  float targetSpeed;
  int goalLane;
  int intendedLane;
  float intendedLaneSpeed;
  int finalLane;
  float finalLanceSpeed;
  int distanceToGoal;

  CostInfo(
    float targetSpeed,
    int goalLane,
    int intendedLane,
    float intendedLaneSpeed,
    int finalLane,
    float finalLaneSpeed,
    int distanceToGoal) {
    this->targetSpeed = targetSpeed;
    this->goalLane = goalLane;
    this->intendedLane = intendedLane;
    this->intendedLaneSpeed = intendedLaneSpeed;
    this->finalLane = finalLane;
    this->finalLanceSpeed = finalLaneSpeed;
    this->distanceToGoal = distanceToGoal;
  }
};

class CostCalculator {

 public:
  float cost(CostInfo costInfo) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */

    const float costReachGoal = WEIGHT_REACH_GOAL * this->goal_distance_cost(
        costInfo.goalLane, costInfo.intendedLane, costInfo.finalLane, costInfo.distanceToGoal);
    const float costInEfficiency = WEIGHT_EFFICIENCY * this->inefficiency_cost(
        costInfo.targetSpeed, costInfo.intendedLaneSpeed, costInfo.finalLanceSpeed);

    return costReachGoal + costInEfficiency;
  }

 protected:
  const float WEIGHT_REACH_GOAL = 1000000;
  const float WEIGHT_EFFICIENCY = 100000;

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

  float inefficiency_cost(float target_speed, float speed_intended_lane, float speed_final_lane) {
    /*
      Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than target_speed.
      */
    float cost = (2.0*target_speed - speed_intended_lane - speed_final_lane)/target_speed;
    return cost;
  }

};


enum class CarState { KEEP_LANE, CHANGE_LEFT, CHANGE_RIGHT };

class CarFSM {

 public:
  CarState currentState;
  Lane currentLane;
  AvailableLanes availableLanes;
  CostCalculator costCalculator;
  float targetSpeed;
  int goalLane;
  float goalAloneS; //

  CarFSM(CarState currentState, Lane currentLane1, AvailableLanes availableLanes1,
         float targetSpeed, int goalLane, float goalAloneS):
      currentLane(currentLane1.laneNumberFromMiddleOfRoad), availableLanes(availableLanes1.howManyLanesAvailable){
    this->currentState = currentState;
    this->targetSpeed = targetSpeed;
    this->goalLane = goalLane;
    this->goalAloneS = goalAloneS;
    this->currentLane = currentLane1;
    this->availableLanes = availableLanes1;
  }

  void applyState(CarState state) {
    this->currentState = state;
    //this->currentLane = this->getIntendedLane(state);
  }

  void updateState(DriveEnvironment& driveEnvironment) {
    if (CarState::KEEP_LANE == this->currentState) {
      return;
    }

    Lane currentCarLane = availableLanes.getLaneFromD(driveEnvironment.car_d);
    Lane intendedLane = this->getIntendedLane(this->currentState);
    if (currentCarLane.isSameLane(intendedLane)) {
      this->currentState = CarState::KEEP_LANE;
      this->currentLane = currentCarLane;
    }
  }

  CarState findTheBestState(DriveEnvironment& driveEnvironment) {
    vector<CarState> states = this->successorStates(driveEnvironment);

    float lowestCost = std::numeric_limits<float>().max();
    CarState bestState = states[0];

    for (int i = 0; i < states.size(); i++) {
      CarState state = states[i];

      Lane intendedLane = this->getIntendedLane(state);
      float intendedLaneSpeed = driveEnvironment.laneSpeed(intendedLane, this->targetSpeed);
      Lane finalLane = intendedLane;
      float finalLanceSpeed = intendedLaneSpeed;

      int distanceToGoal = this->goalAloneS - driveEnvironment.car_s;

      float cost = costCalculator.cost(CostInfo(
          this->targetSpeed, this->goalLane,
          intendedLane.laneNumberFromMiddleOfRoad, intendedLaneSpeed,
          finalLane.laneNumberFromMiddleOfRoad, finalLanceSpeed, distanceToGoal
      ));
      if (cost < lowestCost) {
        bestState = state;
      }
    }

    return bestState;
  }

  Lane getIntendedLane(CarState state) {
    if (CarState::CHANGE_RIGHT == state) {
      return this->availableLanes.farSideLane(this->currentLane);
    } else if (CarState::CHANGE_LEFT == state) {
      return this->availableLanes.nearSideLane(this->currentLane);
    } else {
      return this->currentLane;
    }
  }

  vector<CarState> successorStates(DriveEnvironment& driveEnvironment) {
    vector<CarState> states;

    if (this->currentState == CarState::KEEP_LANE) {
      states.push_back(CarState::KEEP_LANE);
      if (availableLanes.hasNearSideLane(this->currentLane)) {
        if (driveEnvironment.isLaneFree(availableLanes.nearSideLane(this->currentLane))){
          states.push_back(CarState::CHANGE_LEFT);
        }
      }
      if (availableLanes.hasFarSideLane(this->currentLane)) {
        if (driveEnvironment.isLaneFree(availableLanes.farSideLane(this->currentLane))){
          states.push_back(CarState::CHANGE_RIGHT);
        }
      }
    }else if (this->currentState == CarState::CHANGE_LEFT) {
      states.push_back(CarState::CHANGE_LEFT);
    }else if (this->currentState == CarState::CHANGE_RIGHT) {
      states.push_back(CarState::CHANGE_RIGHT);
    }

    return states;
  }

};

class PathPlanningFSMAndCostFunction: public PathPlanningStrategy {
 public:
  CarFSM* fsm;
  PathPlanningFSMAndCostFunction(
      vector<double> map_waypoints_x,
      vector<double> map_waypoints_y, vector<double> map_waypoints_s,
      vector<double> map_waypoints_dx, vector<double> map_waypoints_dy,
      CarFSM* fsm
  ): PathPlanningStrategy(
      map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy) {
    this->fsm = fsm;
  }

 public:
  void planningPath(DriveEnvironment &drivingStatus, vector<double> &next_x_vals, vector<double> &next_y_vals) {
    bool too_close = drivingStatus.tooCloseToFrontCar(lane);

    if(too_close) {
      ref_velocity -= 0.224;

      CarState state = this->fsm->findTheBestState(drivingStatus);
      this->fsm->applyState(state);
      this->lane = this->fsm->getIntendedLane(state);

    }else if (ref_velocity < 49.5) {
      ref_velocity += 0.244;
    }

    this->fsm->updateState(drivingStatus);

    this->generateTrajectory(this->lane, ref_velocity, drivingStatus, next_x_vals, next_y_vals);
  }

 protected:
  Lane lane = Lane(1);
  double ref_velocity = 0.0; //mph

  void generateTrajectory(Lane lane, double ref_velocity, DriveEnvironment &drivingStatus, vector<double> &next_x_vals, vector<double> &next_y_vals) {
    vector<double> ptsx;
    vector<double> ptsy;

    XY point1;
    XY point2;
    std::tie(point1, point2) = drivingStatus.last2Point();

    int d = lane.d;
    XY next_wp0 = this->getXYInternal(drivingStatus.car_s + 30, d);
    XY next_wp1 = this->getXYInternal(drivingStatus.car_s + 60, d);
    XY next_wp2 = this->getXYInternal(drivingStatus.car_s + 90, d);

    this->addToXYVector({point1, point2, next_wp0, next_wp1, next_wp2}, ptsx, ptsy);

    XY refXY;
    double ref_yaw;
    std::tie(refXY, ref_yaw) = drivingStatus.referancePoints();
    double ref_x = refXY.x;
    double ref_y = refXY.y;

    for (int i = 0; i < ptsx.size(); i++) {
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;
      ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
      ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    tk::spline splineInterpolator;
    splineInterpolator.set_points(ptsx, ptsy);

    for (int i = 0; i < drivingStatus.previous_path_x.size(); i++) {
      next_x_vals.push_back(drivingStatus.previous_path_x[i]);
      next_y_vals.push_back(drivingStatus.previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = splineInterpolator(target_x);
    double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

    double x_add_on = 0;
    for (int i = 1; i <= 50 - drivingStatus.previous_path_x.size(); i++) {
      double N = (target_dist / (0.02 * this->ref_velocity/2.24));
      double x_point = x_add_on + target_x/N;
      double y_point = splineInterpolator(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
      y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }
  }

  std::tuple<double, double> nextXY(DriveEnvironment &drivingStatus, int currentStep) {
      return std::make_tuple(0.0, 0.0);
  };
};
