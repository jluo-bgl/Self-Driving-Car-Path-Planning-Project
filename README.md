# Self Driving Car Path Planning Project

### Lane Changes 
![lane_change_continuously.gif](./doc/lane_change_continue.gif)

### In Traffic Change With Safe Distance
![lane_change_in_traffic.gif](./doc/lane_change_in_traffic.gif)

## Full Youtube Video
[![See full videos for driving 8 miles](./doc/youtube_video_8_miles.png)](https://youtu.be/Cv-7udTWQaU)
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
Safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

##### Code to load data into 5 vectors
```c
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

if you have clion, you should able to direct open this project and run in IDE

## Testing if you have everything setup correctly

find below line in main.cpp and uncomment it

```c
// Uncomment below, you car should go straight, otherwise your setup is not right
// testAllBeenSetupAndCarShouldGoStraight(car_x, car_y, car_yaw, next_x_vals, next_y_vals);
```

## Keep the car with in lane

### Frenet Coordination

If we switch from Cartesian Coordinate System into Frenet, the question become simple, 
we just need to keep a constant d and constant delta s

![Frenet Coordination](/doc/FrenetCoordinate.png)

![Frenet Coordination](/doc/FrenetCoordinate2.png)

### Solution for keep the car with in lane

After we have Frenet Coordination, it's very straightforward to keep car with in a lane.
we increase s and make d a constant. what's all 

```c
class PathPlanningFollowLineStrategy: public PathPlanningStrategy {
 protected:
  double dist_inc = 0.4;
  Lane lane = Lane(0);

  std::tuple<double, double> nextXY(DriveEnvironment &drivingStatus, int currentStep) {
    double next_s = drivingStatus.car_s + (currentStep + 1) * dist_inc;
    double next_d = this->lane.d;
    vector<double> xy = getXY(next_s, next_d, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
    return std::make_tuple(xy[0], xy[1]);
  }
};
```


## Smooth Driving Experience

### Avoid Speed Spike with Cubic Spline Interpolation

[Cubic Spline Interpolation](https://en.wikipedia.org/wiki/Spline_interpolation) is a mathematical method commonly
used to construct new points within the boundaries of a set of known points. These new
points are function values of an interpolation function (referred to as spline), which
itself consists of multiple cubic piecewise polynomials.

for example, give below points
```
-1.5, -1.2
-.2, 0
1, 0.5
5, 1
10, 1.2
15, 2
20, 1
```
![Points](/doc/cubic_spline_interpolation_points.png)

after Spline Interpolation, we could got below Equations
![Equations](/doc/cubic_spline_interpolation_equations.png)

Based on above equations, by given any x value we could find y, 
for example, if x = 11, f(11) = 1.3714

We use this [C++ Cubic Spline Interpolation Library](http://kluge.in-chemnitz.de/opensource/spline/) to find
the equations.

## Switch to Right Lane when it's more Efficient

### FSM (Finite State Machine)

In this repo, I implemented a simple finite machine which manages state change between ```enum class CarState { KEEP_LANE, CHANGE_LEFT, CHANGE_RIGHT };```

The high level activity describer as below:

1) Start Car with state "KEEP_LANE"
2) For every car events (for example refresh interval), we see if currently too close to front car.
3) if too close, we find the best next State by given current driving environment, include sensor fusion data, ```CarState findTheBestState(DriveEnvironment& driveEnvironment)```
4) One or more new State will return, based on those State(s), we calculate COST and pick up the State who got the lowest cost
5) We apply this best State via ```fsm->applyState(CarState state)```
6) Change current lane to intended lane, for example, if best State is "CHANGE_LEFT", the intended lane will be current lane - 1
7) We monitor what's the car actual lane and compare against intended lane, if they equal to each other, we know car finished the lane change, we switch current lane to intended lane and switch current state to "KEEP_LANE" so that get ready for next lane change. ```fsm->updateState(drivingStatus)```
8) Back to step 1

See [path_plan_strategy_with_fsm_cost_function.h](./src/path_plan_strategy_with_fsm_cost_function.h)
```c
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
```

### Cost Functions

All cost functions are defined in class ```CostCalculator```

Two cost function has been implemented as below, total cost is sum weighted cost functions. we give cost "reach goal" higher weight.
* The cost increases with both the distance of intended lane from the goal and the distance of the final lane from the goal. The cost of being out of the goal lane also becomes larger as vehicle approaches the goal.
* Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than target_speed.

```c
class CostCalculator {

 public:
  float cost(CostInfo costInfo) {
    const float costReachGoal = WEIGHT_REACH_GOAL * this->goal_distance_cost(
        costInfo.goalLane, costInfo.intendedLane, costInfo.finalLane, costInfo.distanceToGoal);
    const float costInEfficiency = WEIGHT_EFFICIENCY * this->inefficiency_cost(
        costInfo.targetSpeed, costInfo.intendedLaneSpeed, costInfo.finalLanceSpeed);

    return costReachGoal + costInEfficiency;
  }

 protected:
  const float WEIGHT_REACH_GOAL = 1000000;
  const float WEIGHT_EFFICIENCY = 100000;
}
```


```c
  float goal_distance_cost(int goal_lane, int intended_lane, int final_lane, float distance_to_goal) {
    int delta_d = 2.0*goal_lane - intended_lane - final_lane;
    float cost = 1 - exp(-(abs(delta_d) / distance_to_goal));
    return cost;
  }

  float inefficiency_cost(float target_speed, float speed_intended_lane, float speed_final_lane) {
    float cost = (2.0*target_speed - speed_intended_lane - speed_final_lane)/target_speed;
    return cost;
  }
```

## Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

