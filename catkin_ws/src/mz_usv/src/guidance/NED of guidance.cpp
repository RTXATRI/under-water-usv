/*receive waypoints info from mapviz and stored in way[];
go to way [0] from 0,0,0 with los;
when arrived flag+1; from way[0] to way [10];
stop when get back to way[0];
edited by jianguang
subscribe /wayPoints
publish /c_heading
publish /stop
*/

#include <guidance/guidance.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>


//namespace otter_coverage

Guidance::Guidance()
{
  ros::NodeHandle nh;
  ros::NodeHandle nhP("~");
  pre_point.x=0;
  pre_point.y=0;
  pre_point.z=0;
  //m_maxSpeed = nhP.param("max_speed", 3);
//  m_turnSpeed = nhP.param("turn_speed",1);
//  m_navSpeed = nhP.param("nav_speed", 2);

  ros::Subscriber wayPoints =
      nh.subscribe("wayPoints", 5, &Guidance::newPath, this);
  ros::Subscriber odometry =
      nh.subscribe("/odometry/filtered", 5, &Guidance::odo, this);

  m_controllerPub =
      nh.advertise<std_msgs::Float64>("c_heading", 5);
  pubstop =
      nh.advertise<std_msgs::Int8>("stop", 5);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    // Get the pose of the robot in the map frame
    ROS_INFO("current,[%f,%f]",x,y);
    followPath(x, y, psi); //x,y is the position in map frame. 
    ros::spinOnce();
    rate.sleep();
  }
}

Guidance::~Guidance() {}

void Guidance::newPath(const geometry_msgs::PolygonStamped& path) //receive the plygon info published by mapviz
{ 
	m_path = path; 
}

void Guidance::odo(const nav_msgs::Odometry& odo) //receive the plygon info published by mapviz
{
	x = odo.pose.pose.position.x;
	y = odo.pose.pose.position.y;
}

void Guidance::followPath(double x, double y, double psi)

{

  int num=m_path.polygon.points.size();   //number of waypoints
  if ( num<= 1)
  {
    return;
  }
  next_point = m_path.polygon.points[index];
  double xx = next_point.y;
  double yy = next_point.x;
  next_point.x=  xx;//enu frame is changed to ned frame in mapviz
  next_point.y=  yy;
  double dist = std::sqrt(std::pow(x - next_point.x, 2) +
                            std::pow(y - next_point.y, 2));

	if (dist<THRESHOULD_D){     
		if(flag==1)                      //when back to the first waypoint, stop
		{
			  std_msgs::Int8 stop;
			  stop.data=1;
			  pubstop.publish(stop);
			  return;
		}
		
		index++;
		pre_point=next_point;
		next_point = m_path.polygon.points[index];
	}
	ROS_INFO("nextpoint,[%f,%f]", next_point.x,next_point.y);
    //ROS_INFO("num,index[%d,%d]", num,index);
	if (index ==num)
	{
		index=0;
		next_point = m_path.polygon.points[index];  
		flag=1;
	}
	//the following is LOS
  double dx=next_point.x-pre_point.x;
  double dy=next_point.y-pre_point.y;
  double ddx=next_point.x-x;
  double ddy=next_point.y-y;
  double l_ag = std::atan2(dy,dx);
  double et=-std::sin(l_ag)*ddx + std::cos(l_ag)*ddy;
  double c_heading = std::atan2(ddy,ddx) - std::atan(-et/DENO);
  c_heading = c_heading*180/3.1415;
  if(c_heading<-180)
  {
      c_heading=c_heading+360;
  }
    if(c_heading>180)
  {
      c_heading=c_heading-360;
  }
  std_msgs::Float64 msg;
  msg.data = c_heading;
  m_controllerPub.publish(msg);
  ROS_INFO("c_heading,[%f]",c_heading);
  //ROS_INFO_STREAM("psi_d: " << chi_d << " psi: " << psi);
  //ROS_INFO_STREAM("u_d: " << u);

}

