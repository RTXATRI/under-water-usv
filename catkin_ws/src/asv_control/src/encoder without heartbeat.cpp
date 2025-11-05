extern "C" {
#include "rc_usefulincludes.h"
}
extern "C" {
#include "roboticscape.h"
}

#include <ros/ros.h>
#include <stdio.h>
#include <signal.h>
#include <rc/encoder.h>
#include <rc/time.h>
#include <std_msgs/Float64.h>
//#include <std_msgs/Float64MultiArray.h>

static int running = 0;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__((unused)) int dummy)
{
  running = 0;
  return;
}

int main(int argc, char** argv)
{
  int i;
  uint64_t ct;
  float dt;
  float left,right;
  std_msgs::Float64 leftrpm, rightrpm;
  float m_pos;
  float c_ratio = 1.7;  // ratio to adjust the gear ratio and the Encoder counts
  // initialize hardware
  if (rc_encoder_init())
  {
    fprintf(stderr, "ERROR: Failed to initializer\n");
    return -1;
  }
  //	ROS_INFO_STREAM("Hello, encoder");
  // set signal handler so the loop can exit cleanly
  signal(SIGINT, __signal_handler);
  running = 1;

  // initial ros node
  ros::init(argc, argv, "encoder");  // initial node
  ros::NodeHandle n;                 // handler
  ros::Publisher left_pub = n.advertise<std_msgs::Float64>("leftrpm", 2);
  ros::Publisher right_pub = n.advertise<std_msgs::Float64>("rightrpm", 2);
  ros::Rate loop_rate(100);
  // initialize the timer
  ct = rc_nanos_thread_time();
  rc_encoder_write(1, 0);  // reset the encoder to zero
  // ros spin
  while (ros::ok() & running)
  {
    // compute delta t between loop//
    // read encoder pose change then convert the angle to RPM
   // for (size_t i = 0; i < 5; i++) {
    ct = rc_nanos_since_boot();  // set current time to ct
    rc_encoder_write(1, 0);  // reset the encoder to zero
    rc_encoder_write(2, 0);  // reset the encoder pose to zero;
    loop_rate.sleep();
    dt = ((float)rc_nanos_since_boot() - (float)ct) / 1000000000.00;
    left=((float)rc_encoder_read(2) * c_ratio) / dt / 360.00 * 60.00;
    right=((float)rc_encoder_read(1) * c_ratio) / dt / 360.00 * 60.00;
    leftrpm.data= fabs(left);
    rightrpm.data= fabs(right);
    ROS_INFO("dt=%f,RPM=%f,RPM=%f gggggggggg", dt, leftrpm.data, rightrpm.data);
    left_pub.publish(leftrpm);  // publish to asv/rpm
	right_pub.publish(rightrpm);
    ros::spinOnce();
    //	printf("\r");
    //	printf("%f|%f|%f", (float)rc_encoder_read(1)*1.7, rpm,dt);
    fflush(stdout);
  }
  rc_encoder_cleanup();
  return 0;
}
