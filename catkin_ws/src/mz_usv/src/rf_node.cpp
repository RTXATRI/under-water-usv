#include <ros/ros.h>
#include <rf/rf_driver.h>

int running =0;
void MySigintHandler(int sig)
{
	//shut things down
	running=0;
	return;
}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}


/*char buffer[1024]; //�䵱������������
void *pthread_service(void* sfd,RF &RF_node)
{
	int fd=*(int *)sfd;
	int n;
	while(1)
	{
		n = readfd,buffer,1024);
		if (n < 0) error("ERROR reading from socket");
		n = write(fd,RF_node.imu_data,strlen(RF_node.imu_data));
		if (n < 0) error("ERROR writing to socket");
		write(newsockfd,"hello",5);
		bzero(buffer,1024);
		ros::spinOnce();
		sleep(1000);
	}
	close(fd);
}*/

/**
* ����-1��ʧ��
* ����>0: �ɹ�
*/
int ReadNonBlock(int fd, char* recv_buf, size_t recv_len)
{
    int readlen = 0;//�Ѿ������ĳ���
    while(readlen < recv_len)
    {
        int ret = read(fd, recv_buf+readlen, recv_len-readlen);
        if(ret == 0)//�ѵ����ļ�ĩβ
        {
            return readlen;
        }
        else if(ret > 0)
        {
            readlen += ret;
        }
        else if(errno == EINTR)
        {
            continue;
        }
        else//����EAGAINֱ���˳�
        {
            break;
        }
    }

    return readlen;
}
int main (int argc, char **argv)
{

	ros::init(argc, argv, "rf_node");
	ros::NodeHandle nh;
	RF RF_node(nh);
	running =1;
	signal(SIGINT, MySigintHandler);
	//ros::Rate loop_rate(100);  //esc need 50 Hz

	//socket

	int sockfd, newsockfd, portno; //Socket file descriptors and port number
	socklen_t clilen; //object clilen of type socklen_t
	char buffer[256]={0}; //buffer array of size 256
	char recbuffer[256]={0};
	struct sockaddr_in serv_addr, cli_addr; ///two objects to store client and server address
	std::stringstream ss;
	//std_msgs::String message;
	int n;
	if (argc < 2)
	{
		fprintf(stderr,"ERROR, no port provided\n");
		exit(1);
	}
	portno = atoi(argv[2]);
	cout << "Hello there! This node is listening on port " << portno << " for incoming connections" << endl;
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
		error("ERROR opening socket");

	int enable = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int));
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
	  error("setsockopt(SO_REUSEADDR) failed");
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	//serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_addr.s_addr =inet_addr(argv[1]);
	serv_addr.sin_port = htons(portno);
	//bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr));
	if (bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
		error("ERROR on binding");
	listen(sockfd,5);
	clilen = sizeof(cli_addr);
	newsockfd = accept(sockfd,(struct sockaddr *) &cli_addr,&clilen);
	if (newsockfd < 0)
	   error("ERROR on accept");

	float hz=50;
	ros::Rate loop_rate(hz);
	int count=0;
	while(ros::ok())
	{
		count++;
		if(count==5) count=1;
		ss.str(std::string()); //Clear contents of string stream
		bzero(buffer,256);
		bzero(recbuffer,2048);
		try
		{
//			ReadNonBlock(newsockfd,buffer,255);
//			if(buffer[1]=='J' &buffer[2]=='O' & buffer[3]=='Y')
//			{
//				RF_node.parse_joy(buffer);
//				printf("aaa");
//			}
			n = read(newsockfd,recbuffer,2047);
//			printf("%s\n",buffer);
			if (n < 0)
			{
				close(newsockfd);
				ROS_INFO("Reconnecting...11");
				newsockfd = accept(sockfd,(struct sockaddr *) &cli_addr,&clilen);
				ROS_INFO("Connected!");
			}
			else
			{
				//cout<<recbuffer<<endl;
				if(recbuffer[1]=='J' &recbuffer[2]=='O' & recbuffer[3]=='Y')
				{
					RF_node.parse_joy(recbuffer);

//					printf("%s",buffer);
//					if(RF_node.parse_joy(buffer))
//					{
//						printf("parse_joy!\n");
//					}
//					else
//					{
//						printf("parse_joy error\n");
//					}
					bzero(recbuffer,256);
				}
				if(recbuffer[1]=='P'& recbuffer[2]=='L' & recbuffer[3]=='A' & recbuffer[4]=='N')
				{
					RF_node.parse_path(recbuffer);
					bzero(recbuffer,256);
					//bzero(buffer,255);
				}
				if(recbuffer[1]=='E'& recbuffer[2]=='n' & recbuffer[3]=='d' & recbuffer[4]=='P')
				{
					RF_node.parse_pathPub(recbuffer);
					bzero(recbuffer,256);
				}



			}
			switch(count)
			{
				case 1:
					strcpy(buffer,RF_node.imu_data);
					break;
				case 2:
					strcpy(buffer,RF_node.gps_data);
					//n = write(newsockfd,"hello2",6);
					break;
				case 3:
					//n = write(newsockfd,"hello3",6);
					strcpy(buffer,RF_node.joy_data);
					break;
				case 4:
					//n = write(newsockfd,"hello4",6);
					strcpy(buffer,RF_node.sys_data);
					break;
				default:
					break;
			}

			n = write(newsockfd,buffer,strlen(buffer));
			if (n < 0)
			{
				close(newsockfd);
				ROS_INFO("Reconnecting...");
				newsockfd = accept(sockfd,(struct sockaddr *) &cli_addr,&clilen);
				ROS_INFO("Connected!");
			}
			//write(newsockfd,"hello",5);
		}
		catch(...)
		{}
		ros::spinOnce();
		loop_rate.sleep();
		//ROS_INFO("qqqq");
	}
	return 0;
}
