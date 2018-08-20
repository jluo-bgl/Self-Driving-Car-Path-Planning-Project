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

using namespace std;

// for convenience
using json = nlohmann::json;


struct Lane {
  int laneNumberFromMiddleOfRoad;
  int d;

  Lane(int laneNumberFromMiddleOfRoad) {
    if (laneNumberFromMiddleOfRoad < 0) {
      throw "laneNumberFromMiddleOfRoad have to been >= 0";
    }
    this->laneNumberFromMiddleOfRoad = laneNumberFromMiddleOfRoad;
    this->d = 2 + 4 * laneNumberFromMiddleOfRoad;
  }

  bool inSameLane(int dOfOtherCar) {
    // 2.2 so that we can give d a little bit room for error (sensor errors)
      if ((dOfOtherCar <= (this->d + 2.2)) && (dOfOtherCar >= (this->d - 2.2))) {
          return true;
      }else{
          return false;
      }
  }

  bool isSameLane(Lane& lane) {
    return this->laneNumberFromMiddleOfRoad == lane.laneNumberFromMiddleOfRoad;
  }
};


struct AvailableLanes {
  int howManyLanesAvailable;
  vector<Lane> lanes;

  AvailableLanes(int howManyLanesAvailable) {
    this->howManyLanesAvailable = howManyLanesAvailable;
    for (int i = 0; i < howManyLanesAvailable; i++) {
      lanes.push_back(Lane(i));
    }
  }

  Lane getLane(int laneNumber) {
    return this->lanes[laneNumber];
  }

  Lane getLaneFromD(float d) {
    for (Lane lane : this->lanes) {
      if (lane.inSameLane(d)) {
        return lane;
      }
    }
    throw "not able to found any lane based on d: " + std::to_string(d);
  }

  // has lane that far side of middle of road
  bool hasFarSideLane(Lane baseLane) {
    int laneNumber = baseLane.laneNumberFromMiddleOfRoad;
    if (laneNumber >= lanes.size() - 1) {
      return false;
    } else {
      return true;
    }
  }

  // has lane that far side of middle of road
  Lane farSideLane(Lane baseLane){
    if (this->hasFarSideLane(baseLane)) {
      int laneNumber = baseLane.laneNumberFromMiddleOfRoad;
      return lanes[laneNumber + 1];
    } else {
      throw "There are no more lane available of far side from middle of road.";
    }
  }

  // has lane that near side of middle of road
  bool hasNearSideLane(Lane baseLane) {
    int laneNumber = baseLane.laneNumberFromMiddleOfRoad;
    if (laneNumber > 0 && laneNumber < lanes.size()) {
      return true;
    } else {
      return false;
    }
  }

  // has lane that near side of middle of road
  Lane nearSideLane(Lane baseLane){
    if (this->hasNearSideLane(baseLane)) {
      int laneNumber = baseLane.laneNumberFromMiddleOfRoad;
      return lanes[laneNumber - 1];
    } else {
      throw "There are no more lane available of near side from middle of road.";
    }
  }

};