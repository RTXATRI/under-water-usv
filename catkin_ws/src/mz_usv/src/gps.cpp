#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <ros/ros.h>
#include "gps_common/GPSFix.h"

	float longitude;
	float latitude;

//高度计种类 包括为连接状态 

int altimeter_fd;

int set_opt(int fd,int nSpeed,  int nBits, char nEvent, int nStop)
{

/* 五个参量 fd打开文件 speed设置波特率 bit数据位设置   neent奇偶校验位 stop停止位 */
    struct termios newtio,oldtio;
    if ( tcgetattr( fd,&oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE; 
    switch( nBits )
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

    switch( nEvent )
    {
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
    }

    switch( nSpeed )
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 38400:
            cfsetispeed(&newtio, B38400);
            cfsetospeed(&newtio, B38400);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }

    if( nStop == 1 )
        newtio.c_cflag &= ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
         perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}
int open_port(int fd,int comport)
{
/* fd 打开串口 comport表示第几个串口 */
    //char *dev[]={"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2"};
    long vdisable;
    if (comport==1)
    {
        fd = open( "/dev/ttyS0", O_RDWR|O_NOCTTY|O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
            printf("open ttyS0 .....\n");
    }
    else if(comport==2)
    {
        fd = open( "/dev/ttyS1", O_RDWR|O_NOCTTY|O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
            printf("open ttyS1 .....\n");
    }
    else if (comport==3)
    {
        fd = open( "/dev/ttyS2", O_RDWR|O_NOCTTY|O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
            printf("open ttyS2 .....\n");
    }
    else if(comport==8)
    {
        fd = open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
            printf("open ttyUSB0 .....\n");
    }
    if(fcntl(fd, F_SETFL, 0)<0)
        printf("fcntl failed!\n");
    else
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));

    if(isatty(STDIN_FILENO)==0)
        printf("standard input is not a terminal device\n");
    else
        printf("isatty success!\n");

    printf("fd-open=%d\n",fd);
    return fd;
}

void altimeter_deal(char* data)
{
	  //分割字段数量临时变量   
	  int i;
	  //分割字段指针 
	  char *splitdata;
	  //第一个分割数组保存
	  //char first_fplitdata[100] ={0};   
	  //字符串分割，逗号分割符
  	splitdata = strsep(&data, "$");
	  splitdata = strsep(&data, ",");
	  /************对比分割的首字段，识别为1007D高度计********/ 
	  if (!strcmp(splitdata, "GNGLL"))
	  {
	  	//$SDDBT,,f,2.57,M,,F*65 $SDDBT,,f,1.41,M,,F*61  $SDDBT,,f,1.04,M,,F*60  $SDDBT,,f,0.69,M,,F*6A*
		// 打印分割数据2：数值
		splitdata = strsep(&data, ",");
   	latitude = atof(splitdata);
		// printf("KONGSBERG 1007D--DATA2: %s\n", splitdata);
		// 打印分割数据3：f 
		splitdata = strsep(&data, ",");
		// printf("KONGSBERG 1007D--DATA3: %s\n", splitdata);
		//片段数据4为F，所有有效数据前的数据均匹配1007d 协议**/ 
    if (!strcmp(splitdata, "N")) 
		{
		// 打印分割数据4：高度及数值
			splitdata = strsep(&data, ",");
			longitude = atof(splitdata);

			//设置高度计类型，2为1007d 
		}
	  }
}

void altimeter_actionRun(void)
{
	char altimeterbuf[500]={0};
	int readlen;
	ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<gps_common::GPSFix>("gps",4);
    gps_common::GPSFix p;
    ros::Rate r(1);
	while(1)
	{	
		readlen=read(altimeter_fd, altimeterbuf, 100);
		if(readlen>0)
		{
			altimeterbuf[readlen]='0';
			printf("altimeter data is %s\n",altimeterbuf);
			if(readlen>2)
				altimeter_deal(altimeterbuf);
			
		}

		//std::cout<<altimeterbuf<<std::endl;
		p.longitude=longitude;
    p.latitude=latitude;
		pub.publish(p);
		//清空接收区 
		memset(altimeterbuf, '\0', 500);	
		//r.sleep();
		sleep(1);	
	}
}
int main(int argc,char* argv[])
{	
	int i;
	ros::init(argc,argv,"gps");	
	//开启声学串口  9600 8 N 1 
    if ((altimeter_fd = open_port(altimeter_fd, 3)) < 0)
    {
        perror("open_port error");
        return 0;
    }
    if((i=set_opt(altimeter_fd,38400,8,'N',1))<0)
    {
        perror("set_opt error");
        return 0;
    }
    altimeter_actionRun();
    close(altimeter_fd);	
    return 0;
}