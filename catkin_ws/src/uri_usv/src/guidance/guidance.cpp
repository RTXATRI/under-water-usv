/*receive waypoints info from mapviz and stored in way[];
go to way [0] from 0,0,0 with los;
when arrived flag+1; from way[0] to way [10];
stop when get back to way[0];
edited by jianguang
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
  m_turnSpeed = nhP.param("turn_speed",1);
  m_navSpeed = nhP.param("nav_speed", 2);

  ros::Subscriber wayPoints =
      nh.subscribe("wayPoints", 5, &Guidance::newPath, this);

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
    geometry_msgs::TransformStamped tfStamped;
    try
    {
      tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0),
                                           ros::Duration(0.0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("Transform from map to base_link not found: %s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    double x = tfStamped.transform.translation.x;
    double y = tfStamped.transform.translation.y;
    double psi = tf2::getYaw(tfStamped.transform.rotation);

    followPath(x, y, psi);

    ros::spinOnce();
    rate.sleep();
  }
}

Guidance::~Guidance() {}

void Guidance::newPath(const geometry_msgs::PolygonStamped& path)
{ 
	m_path = path; 
}

void Guidance::followPath(double x, double y, double psi)
// TODO: cuts turns, how to fix?
{

  // Finished?
  //bool isTurning = false;
  int index =0;
  int flag=0;
  int num=m_path.polygon.points.size();
  if ( num<= 1)
  {
    return;
  }
  next_point = m_path.polygon.points[index];
  double dist = std::sqrt(std::pow(x - next_point.x, 2) +
                            std::pow(y - next_point.y, 2));

	if (dist<THRESHOULD_D){
		if(flag==1)
		{
			  std_msgs::Float64 stop;
			  stop.data=1;
			  pubstop.publish(stop);
			  return;
		}
		
		index++;
		pre_point=next_point;
		next_point = m_path.polygon.points[index];
	}
	if (index ==num)
	{
		index=0;
		next_point = m_path.polygon.points[index];
		flag=1;
	}
	ROS_INFO("next_point: [%f,%f]", next_point.x,next_point.y);
  // Cross-track error
  double dx=next_point.x-pre_point.x;
  double dy=next_point.y-pre_point.y;
  double ddx=next_point.x-x;
  double ddy=next_point.y-y;
  double l_ag = std::atan(dy/dx);
  double et=-std::sin(l_ag)*ddx + std::cos(l_ag)*ddy;
    // desired course angle
  double c_heading = std::atan(ddy/ddx) + std::atan(-et/DENO);
  c_heading = c_heading*180/3.1415;
  if(c_heading<-180)
  {
      c_heading=c_heading+360;
  }
    if(c_heading>180)
  {
      c_heading=c_heading-360;
  }
  // calculate error in heading
  /*
  double chi_err = chi_d - psi;
  while (chi_err > M_PI)
  {
    chi_err -= 2 * M_PI;
  }
  while (chi_err < -M_PI)
  {
    chi_err += 2 * M_PI;
  }

  // calculate desired speed
  //double u = m_maxSpeed * (1 - std::abs(y_e) / 5 - std::abs(chi_err) / M_PI_2);
  int u = m_navSpeed;
  if (isTurning)
    u = m_turnSpeed;

  // Publish speed and course to controller
  */
  std_msgs::Float64 msg;
  msg.data = c_heading;
  m_controllerPub.publish(msg);

  //ROS_INFO_STREAM("psi_d: " << chi_d << " psi: " << psi);
  //ROS_INFO_STREAM("u_d: " << u);

}

