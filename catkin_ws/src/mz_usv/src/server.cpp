
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<strings.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<pthread.h>
#include<signal.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include<sys/wait.h>
#define BUFSIZE   1024
#define  QUIT_STR "QUIT"
#define SERV_IP 5001
#define SERV_IP_ADDR  "192.168.7.2"
#define  BACKLOG  5
char buf[BUFSIZE];
 

 
 
void child_data_handle(int signum)
{
	if( SIGCHLD  == signum) 
	{
		waitpid(-1,NULL,WNOHANG);
	}
}
int main(int argc, char** argv)
{
	printf(" this is service!\n");
	int fd = -1;
 	ros::init(argc, argv, "server");
  	ros::NodeHandle nh;
  	ros::Publisher inet = nh.advertise<std_msgs::Int8>("inet", 2);
    ros::Rate loop_rate(50); 
	signal(SIGCHLD,child_data_handle);
	struct sockaddr_in sin;
	//1.socket
	fd=socket(AF_INET,SOCK_STREAM,0);
	
	if( fd <0)
	{
 
		perror("socket");
		exit(1);
	}
	//把结构体清零
	bzero(&sin,sizeof(sin));
	sin.sin_family=AF_INET;
 
	sin.sin_port=htons(SERV_IP);//两个字节
/*
inet_addr(),返回转换后的地址,仅适用于IPV4,出错时返回-1
将IPV4/IPV6 的地址转换成binary格式
inet_pton(AF_INET,SERV_IP_ADDR ,(void*) sin.sin_addr.s_addr);
两种方法都可以
*/
 
	sin.sin_addr.s_addr=inet_addr(SERV_IP_ADDR);
	//优化，可以在任何的服务器中运行，自动获取当前的ip地址
	//sin.sin_addr.s_addr=INADDR_ANY;
	//2.bind
	if(bind(fd,(struct sockaddr *)&sin,sizeof(sin))<0)
	{
		perror("bind");
		exit(1);
	}
	//3.listen
      if(listen(fd,BACKLOG)<0)
	{
		perror("listen");
		exit(1);
	}
	//4.acccept
 
//进程
	int newfd =  -1;
	pid_t  pid;
	struct sockaddr_in cin;
	socklen_t  addrlen =sizeof(cin);
	while(1)
	{
		 newfd=accept(fd,NULL,NULL);
		if( newfd <0)
		{
			perror("acccept");
			exit(1);
		}

		char  ipv4_addr[16];
	     if(! inet_ntop(AF_INET,(void *)&cin.sin_addr,ipv4_addr,sizeof(cin)))
	{
		perror("inet_ntop");
		exit(1);
	}
	//获取端口号和ip地址
	printf("client : (%s , %d) is connect !\n", ipv4_addr,ntohs(cin.sin_port));
	

	int ret = -1;
 char buf[BUFSIZE];
	printf(" child handle  progess : newfd =%d\n",newfd);
	while(1)
	{
		
		do
		{
			bzero(buf,BUFSIZE);
			ret=read(newfd,buf,BUFSIZE-1);
		}while(ret < 1);
		if ( ! ret)
		{
			break;
		}
   std_msgs::Int8 inet_msgs;
   if(buf[ret-1]==0x73)
   {inet_msgs.data=0;
   inet.publish(inet_msgs);}
     if(buf[ret-1]==0x77)
   {inet_msgs.data=1;
   inet.publish(inet_msgs);}
     if(buf[ret-1]==0x74)
   {inet_msgs.data=2;
   inet.publish(inet_msgs);}
		printf(" recevice data :%s\n",buf);
		//两个字符串做比较，不一样返回0，一样返回1,则执行下面的内容
		if( ! strncasecmp(buf,QUIT_STR,strlen(QUIT_STR)))	             		          {
			printf(" Client is exiting !\n");
			break;
		}
		
	}
	close(newfd);
	close(fd);
	}

		
	
		close(fd);
			
	return  0;
 
}
