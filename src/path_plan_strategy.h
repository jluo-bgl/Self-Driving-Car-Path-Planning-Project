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


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

void testAllBeenSetupAndCarShouldGoStraight(double car_x, double car_y, double car_yaw, vector<double> &next_x_vals, vector<double> &next_y_vals) {
  double dist_inc = 0.5;
  for(int i = 0; i < 50; i++)
  {
    next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
    next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
  }
}

struct XY{
  double x;
  double y;

  XY() {
      this->x = 0.0;
      this->y = 0.0;
  }

  XY(double x, double y) {
      this->x = x;
      this->y = y;
  }
};

struct Lane {
  int laneNumberFromMiddleOfRoad;
  int d;

  Lane(int laneNumberFromMiddleOfRoad) {
      this->laneNumberFromMiddleOfRoad = laneNumberFromMiddleOfRoad;
      this->d = 2 + 4 * laneNumberFromMiddleOfRoad;
  }

  bool inSameLane(int dOfOtherCar) {
      if ((dOfOtherCar < (this->d + 2)) && (dOfOtherCar > (this->d - 2))) {
          return true;
      }else{
          return false;
      }
  }
};

class DriveEnvironment {
 public:
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  // Previous path data given to the Planner
  json previous_path_x;
  json previous_path_y;
  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  json sensor_fusion;

  int prev_size;


  DriveEnvironment(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
                   double end_path_s, double end_path_d, json previous_path_x, json previous_path_y, json sensor_fusion) {
      this->car_x = car_x;
      this->car_y = car_y;
      this->car_s = car_s;
      this->car_d = car_d;
      this->car_yaw = car_yaw;
      this->car_speed = car_speed;
      this->end_path_s = end_path_s;
      this->end_path_d = end_path_d;

      this->previous_path_x = previous_path_x;
      this->previous_path_y = previous_path_y;
      this->sensor_fusion = sensor_fusion;

      this->prev_size = previous_path_x.size();
  }

  bool hasEnoughPreviewPath() {
      return this->prev_size > 2;
  }

  std::tuple<XY, XY> last2Point() {
      if (this->hasEnoughPreviewPath()) {
          return this->last2PointBasedPreviousPath();
      }else{
          return this->last2PointBasedCurrentCarPosition();
      }
  };

  std::tuple<XY, double> referancePoints() {
      if (this->hasEnoughPreviewPath()) {
          double ref_x = this->previous_path_x[this->prev_size - 1];
          double ref_y = this->previous_path_y[this->prev_size - 1];
          double ref_x_prev = this->previous_path_x[this->prev_size - 2];
          double ref_y_prev = this->previous_path_y[this->prev_size - 2];

          double ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

          return std::make_tuple(XY(ref_x, ref_y), ref_yaw);
      }else{
          XY xy = XY(this->car_x, this->car_y);
          double ref_yaw = deg2rad(this->car_yaw);
          return std::make_tuple(xy, ref_yaw);
      }
  };

  bool tooCloseToFrontCar(Lane lane) {
      bool too_close = false;

      for (int i = 0; i < this->sensor_fusion.size(); i++) {
          //car is in my lane
          float d = this->sensor_fusion[i][6];
          if (lane.inSameLane(d)) {
              double vx = this->sensor_fusion[i][3];
              double vy = this->sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = this->sensor_fusion[i][5];

              check_car_s += ((double)this->prev_size * 0.02 * check_speed);
              if ((check_car_s > this->car_s) && (check_car_s - this->car_s) < 30) {
                  too_close = true;
              }
          }
      }
      return too_close;
  }

 protected:
  std::tuple<XY, XY> last2PointBasedCurrentCarPosition() {
      double prev_car_x = this->car_x - cos(this->car_yaw);
      double prev_car_y = this->car_y - sin(this->car_yaw);
      return std::make_tuple(
          XY(prev_car_x, prev_car_y),
          XY(this->car_x, this->car_y));
  };

  std::tuple<XY, XY> last2PointBasedPreviousPath() {
      double ref_x = this->previous_path_x[this->prev_size - 1];
      double ref_y = this->previous_path_y[this->prev_size - 1];

      double ref_x_prev = this->previous_path_x[this->prev_size - 2];
      double ref_y_prev = this->previous_path_y[this->prev_size - 2];

      return std::make_tuple(
          XY(ref_x_prev, ref_y_prev),
          XY(ref_x, ref_y));
  };


};


class PathPlanningStrategy{
 public:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  PathPlanningStrategy(vector<double> map_waypoints_x,
                       vector<double> map_waypoints_y,
                       vector<double> map_waypoints_s,
                       vector<double> map_waypoints_dx,
                       vector<double> map_waypoints_dy) {
      this->map_waypoints_x = map_waypoints_x;
      this->map_waypoints_y = map_waypoints_y;
      this->map_waypoints_s = map_waypoints_s;
      this->map_waypoints_dx = map_waypoints_dx;
      this->map_waypoints_dy = map_waypoints_dy;
  }

  virtual ~PathPlanningStrategy() {};

  virtual void planningPath(DriveEnvironment &drivingStatus, vector<double> &next_x_vals, vector<double> &next_y_vals) {
      for(int i = 0; i < 50; i++)
      {
          double nextx;
          double nexty;
          std::tie(nextx, nexty) = nextXY(drivingStatus, i);
          next_x_vals.push_back(nextx);
          next_y_vals.push_back(nexty);
      }
  }

 protected:
  virtual std::tuple<double, double> nextXY(DriveEnvironment &drivingStatus, int currentStep) = 0;

  XY getXYInternal(double s, double d) {
      vector<double> xy = getXY(s, d, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
      return XY(xy[0], xy[1]);
  }

  void addToXYVector(vector<XY> points, vector<double> &ptsx, vector<double> &ptsy) {
      for(XY xy : points) {
          ptsx.push_back(xy.x);
          ptsy.push_back(xy.y);
      }
  }
};



class PathPlanningFollowLineStrategy: public PathPlanningStrategy {
 public:
  PathPlanningFollowLineStrategy(vector<double> map_waypoints_x,
                                 vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                                 vector<double> map_waypoints_dx, vector<double> map_waypoints_dy, Lane lane): PathPlanningStrategy(
      map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy
  ) {
      this->lane = lane;
  }

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


class PathPlanningGoStraightStrategy: public PathPlanningStrategy {
 public:
  PathPlanningGoStraightStrategy(vector<double> map_waypoints_x,
                                 vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                                 vector<double> map_waypoints_dx, vector<double> map_waypoints_dy): PathPlanningStrategy(
      map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy) {}

 protected:
  double dist_inc = 0.4;
  std::tuple<double, double> nextXY(DriveEnvironment &drivingStatus, int currentStep) {
      return std::make_tuple(
          drivingStatus.car_x+(dist_inc*currentStep)*cos(deg2rad(drivingStatus.car_yaw)),
          drivingStatus.car_y+(dist_inc*currentStep)*sin(deg2rad(drivingStatus.car_yaw))
      );
  }
};

class PathPlanningSlowDownWhenApprochingCar: public PathPlanningStrategy {
 public:
  PathPlanningSlowDownWhenApprochingCar(vector<double> map_waypoints_x,
                                        vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                                        vector<double> map_waypoints_dx, vector<double> map_waypoints_dy): PathPlanningStrategy(
      map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy) {}

 public:
  void planningPath(DriveEnvironment &drivingStatus, vector<double> &next_x_vals, vector<double> &next_y_vals) {
      vector<double> ptsx;
      vector<double> ptsy;

      if (drivingStatus.prev_size > 0) {
          drivingStatus.car_s = drivingStatus.end_path_s; // changed the status!!!!
      }

      bool too_close = drivingStatus.tooCloseToFrontCar(lane);

      if(too_close) {
          ref_velocity -= 0.224;
      }else if (ref_velocity < 49.5) {
          ref_velocity += 0.244;
      }

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

 protected:
  Lane lane = Lane(1);
  double ref_velocity = 0.0; //mph

  std::tuple<double, double> nextXY(DriveEnvironment &drivingStatus, int currentStep) {
      return std::make_tuple(0.0, 0.0);
  };
};

class PathPlanningChangeLaneWhenApprochingCar: public PathPlanningStrategy {
 public:
  PathPlanningChangeLaneWhenApprochingCar(vector<double> map_waypoints_x,
                                          vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                                          vector<double> map_waypoints_dx, vector<double> map_waypoints_dy): PathPlanningStrategy(
      map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy) {}

 public:
  void planningPath(DriveEnvironment &drivingStatus, vector<double> &next_x_vals, vector<double> &next_y_vals) {
      vector<double> ptsx;
      vector<double> ptsy;

      if (drivingStatus.prev_size > 0) {
          drivingStatus.car_s = drivingStatus.end_path_s; // changed the status!!!!
      }

      bool too_close = drivingStatus.tooCloseToFrontCar(lane);

      if(too_close) {
          ref_velocity -= 0.224;
          if (lane.laneNumberFromMiddleOfRoad > 0) {
              this->lane = Lane(0);
          }
      }else if (ref_velocity < 49.5) {
          ref_velocity += 0.244;
      }

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

 protected:
  Lane lane = Lane(1);
  double ref_velocity = 0.0; //mph

  std::tuple<double, double> nextXY(DriveEnvironment &drivingStatus, int currentStep) {
      return std::make_tuple(0.0, 0.0);
  };
};
