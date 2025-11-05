#ifndef OTTER_GUIDANCE_H_
#define OTTER_GUIDANCE_H_
#define THRESHOULD_D 10 //threshould to comfirm a waypoint is reached
#define DENO 15 //c_heading = std::atan2(ddy,ddx) - std::atan(-et/DENO);
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point32.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
//#include <nav_msgs/Path.h>
#include <ros/ros.h>


class Guidance
{
public:
  Guidance();
  ~Guidance();

private:
  //void newWaypoint(const geometry_msgs::PoseStamped& waypoint);
  void newPath(const geometry_msgs::PolygonStamped& path);
  void followPath(double x, double y, double psi);
  //double dist(double x0, double y0, double x1, double y1) const;

  geometry_msgs::PolygonStamped m_path;

  geometry_msgs::Point32 next_point;
  geometry_msgs::Point32 pre_point;
  ros::Publisher m_controllerPub,pubstop;

  // lookahead distance
  ///double DELTA = 0.5;

  // time-varying lookahead distance
 // double delta_max = 4.0;
 // double delta_min = 1.0;
 // double delta_k = 1.0;

  // circle of acceptance
 // double R = 1.0;
  int index =0; //index of waypoints
  int flag=0; 
  double m_turnSpeed;
  double m_navSpeed;
};


#endif
