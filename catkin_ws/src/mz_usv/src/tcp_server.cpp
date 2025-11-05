/*
 *  程序名：tcp_server.cpp，此程序演示采用freecplus框架的CTcpServer类实现socket通信的服务端。
 *  作者：Louis 日期：20220408
*/
#include "usv_code/_freecplus.h"
#include "ros/ros.h"
#include "usv_code/usv_device.h"

int main(int argc,char *argv[])
{
  if (argc<3)
  {
    printf("Using:./demo48 port\nExample:./demo48 5005\n\n");
    return -1;
  }

  ros::init(argc,argv,"tcp_server");
  ros::NodeHandle nh;
  RF RF_node(nh);
  ros::Rate loop_rate(10);
  CTcpServer TcpServer;   // 创建服务端对象。

  if (TcpServer.InitServer(atoi(argv[1]))==false) // 初始化TcpServer的通信端口。
  {
    printf("TcpServer.InitServer(%s) failed.\n",argv[1]);
    return -1;
  }

  if (TcpServer.Accept()==false)   // 等待客户端连接。
  {
    printf("TcpServer.Accept() failed.\n");
    return -1;
  }

  printf("Client %s connected!\n",TcpServer.GetIP());

  char strbuffer[1024];  // 存放数据的缓冲区。

  while (ros::ok())
  {
    memset(strbuffer,0,sizeof(strbuffer));
    if (TcpServer.Read(strbuffer,300)==false)
	{
    	ROS_INFO("Reconnecting1...");
    	TcpServer.Accept();
    	ROS_INFO("Reconnected1!");
	}
    else
    {
    	if(strbuffer[1]=='J' &strbuffer[2]=='O' & strbuffer[3]=='Y')
    	{
    		cout<<strbuffer<<endl;
    		RF_node.parse_joy(strbuffer);
    	}
    }
    strcpy(strbuffer,RF_node.imu_data);
    if (TcpServer.Write(strbuffer)==false)
	{
    	ROS_INFO("Reconnecting2...");
    	TcpServer.Accept();
    	ROS_INFO("Reconnected2!");
	}
    loop_rate.sleep();
    ros::spinOnce();
  }

  printf("Client disconnected\n");    // 程序直接退出，析构函数会释放资源。
}
