/**
 读取电压电流温度电量（charge）等信息，电压高于一定值后电量置零。
 */
//#include <sstream>
//#include <fstream>
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
#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
using namespace std;
//ofstream pcsout;  //power consumption out
//ifstream pcsin;  //power consumption in
sensor_msgs::BatteryState batt_state;
int running =0;
void MySigintHandler(int sig)
{
	//shut things down
	//pcsout<<batt_state.charge<<"\n";
	running=0;
	return;
}
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
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

int main(int argc, char** argv)
{
    int fd;
	//double saved_kwh=0.0;
	batt_state.charge=0.0;
    int nread,i;
    unsigned char readbuf[256] = {0};
    unsigned char sendbuf[8] = {0x01, 0x03, 0x00, 0x48, 0x00, 0x08, 0xC4, 0x1A};
    unsigned char zerobuf[13] = {0x01, 0x10, 0x00, 0x4B, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0xB6, 0x2C};
    running = 1;
	//pcsin.open("/home/ubuntu/catkin_ws/src/mz_usv/files/parameters.txt");
	//pcsin>>saved_kwh;
	//pcsin.close();
	//pcsout.open("/home/ubuntu/catkin_ws/src/mz_usv/files/parameters.txt",ios::trunc);
    signal(SIGINT, MySigintHandler);
    ros::init(argc,argv,"battery_sensor");
    ros::NodeHandle nh;
    ros::Publisher batt_pub=nh.advertise<sensor_msgs::BatteryState>("/sensor/m_VI",2);
    ros::Rate loop_rate(1);
    if ((fd = open_port(fd, atoi(argv[1]))) < 0)
    {
        perror("open_port error");
        return 0;
    }
    if((i=set_opt(fd,atoi(argv[2]),8,'N',1))<0)
    {
        perror("set_opt error");
        return 0;
    }
    printf("fd=%d\n",fd);
//    fd=3;
    int count = 0;
    int zeroflag = 0;
    double voltage = 0.0;
    double current = 0.0;
    double temperature = 0.0;
	double power = 0.0;
	double kwh = 0.0;
	//printf("sendbuf=%s\n",sendbuf);
	//write(fd, zerobuf, 13);
	//sleep(2);
	//nread = read(fd, readbuf, 256);
	//memset(readbuf,0,nread);
	//sleep(2);
	//printf("nread=%d,%s\n",nread,readbuf);
    while (ros::ok()&&running == 1)
    {
        count++;
        if(count==3 &&voltage>12.2&&voltage<20)
        {
			write(fd, zerobuf, 13);
			zeroflag=1;
			//saved_kwh=0.0;
        }
        if (count == 5)
        {
        	count=1;
        	if(zeroflag==1)
        	{
        	nread = read(fd, readbuf, 256);
        	zeroflag=0;
        	}
        }
        if (count == 1)
        {
            write(fd, sendbuf, 8);
        }
        else if(count==2)
        {
            nread = read(fd, readbuf, 37);
            if(nread==37)
            {
                voltage = ((double)readbuf[3] * 0x1000000 + (double)readbuf[4] * 0x10000 + (double)readbuf[5] * 0x100 + (double)readbuf[6]) / 10000.0;
                current = ((double)readbuf[7] * 0x1000000 + (double)readbuf[8] * 0x10000 + (double)readbuf[9] * 0x100 + (double)readbuf[10]) / 10000.0;
				power = ((double)readbuf[11] * 0x1000000 + (double)readbuf[12] * 0x10000 + (double)readbuf[13] * 0x100 + (double)readbuf[14]) / 10000.0;
				kwh = ((double)readbuf[15] * 0x1000000 + (double)readbuf[16] * 0x10000 + (double)readbuf[17] * 0x100 + (double)readbuf[18]) / 10000.0;
                temperature = ((double)readbuf[27] * 0x1000000 + (double)readbuf[28] * 0x10000 + (double)readbuf[29] * 0x100 + (double)readbuf[30]) / 100.0;
				//power=10*int((po[0]<<8)|(po[1]))/1000;
				//saved_kwh=saved_kwh+power*1.00/1200.00;
				//batt_state.charge=saved_kwh;
				batt_state.charge=kwh*1000.00;
				//batt_state.design_capacity=kwh;
				batt_state.capacity=power;
                batt_state.voltage=voltage;
                batt_state.current=current;
				batt_state.percentage=temperature;
                batt_pub.publish(batt_state);
            }
            //printf("nread=%d\n",nread);
            printf("voltage:%.1lf\ncurrent:%.1lf\ntemperature:%.1lf\n", voltage, current, temperature);
            //std::cout << (int)readbuf[0] << std::endl;
        }
        //sleep(1);
        ros::spinOnce();
        loop_rate.sleep();
    }
    // nread=read(fd,buff,8);
    // printf("nread=%d,%s\n",nread,buff);
    close(fd);
    printf("exit");
    return 0;
}
