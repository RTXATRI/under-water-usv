
#include <sstream>
#include <ros.h>
#include <std_msgs/String.h>
char *rosSrvrIp = "192.168.5.100";

void messageCb(const std_msgs::String& received_msg){
        printf("Received subscribed chatter message: %s\n", received_msg.data);
}

int main(int argc, char** argv)
{
  ros::NodeHandle nh;
  ros::Subscriber<std_msgs::String> sub("chatter", messageCb );
  nh.initNode(rosSrvrIp);
  nh.subscribe(sub);
          while(1) {
                  sleep(1);
                  nh.spinOnce();
        }
}
