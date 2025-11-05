#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <fcntl.h>
#include <sstream>
#include <iomanip>
#include "mz_usv/MS5837.h"
#include <std_msgs/Bool.h>
using namespace std;

class ReceiveData {
public: 
    mz_usv::MS5837 depth_data;
    std_msgs::Bool lock_msg;
    
    ReceiveData(ros::NodeHandle &nh) : nh(nh) {
        depth_sub = nh.subscribe("/depth", 2, &ReceiveData::depth_callback, this);
        lock_sub = nh.subscribe("/lock", 2, &ReceiveData::lock_callback, this);
        ros::NodeHandle nh_priv("~");
    }
    
    void depth_callback(const mz_usv::MS5837::ConstPtr& msg) {
        depth_data.depth = msg->depth; 
        depth_data.temp = msg->temp; 
        depth_data.depth = depth_data.depth - 0.050;   
    }
    
    void lock_callback(const std_msgs::Bool::ConstPtr& msg) {
        lock_msg.data = msg->data; 
    } 

private:
    ros::NodeHandle nh;
    ros::Subscriber depth_sub;
    ros::Subscriber lock_sub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tcp_float");
    ros::NodeHandle nh;
    ReceiveData receive_Data(nh);
    bool last_lock_state;

    // 创建一个 socket
    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd == -1) {
        cerr << "socket 创建失败" << endl;
        exit(1);
    } 

    // 设置 socket 选项，解决端口占用问题
    int opt = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
        cerr << "设置 socket 选项失败" << endl;
        close(socket_fd);
        exit(1);
    }

    // 准备通讯地址
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(10006);
    addr.sin_addr.s_addr = inet_addr("192.168.31.108");

    // 绑定地址
    if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        cerr << "bind 失败" << endl;
        close(socket_fd);
        exit(1);
    }
    cout << "bind ok 等待客户端的连接" << endl;

    // 监听客户端连接
    if (listen(socket_fd, 30) == -1) {
        cerr << "listen 失败" << endl;
        close(socket_fd);
        exit(1);
    }

    ros::Rate loop_rate(50);
    int count = 0;
    while (ros::ok()) {
        struct sockaddr_in client;
        socklen_t len = sizeof(client);
        int fd = accept(socket_fd, (struct sockaddr*)&client, &len);
        if (fd == -1) {
            cerr << "accept 错误" << endl;
            continue;
        }

        // 设置为非阻塞模式
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        char *ip = inet_ntoa(client.sin_addr);
        cout << "客户：" << ip << " 连接成功" << endl;

        write(fd, "Welcome to server", 18);
        char buffer[255] = {};

        while (true) {
        
            ros::spinOnce();
            int size = read(fd, buffer, sizeof(buffer));
            if (size > 0) {
                cout << "内容：" << buffer << endl;
            } else if (size == 0) {
                cout << "客户端断开连接" << endl;
                close(fd);
                break;
            } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                cerr << "read 错误: " << strerror(errno) << endl;
                break;
            }

//            if (count % 20 == 0) {  
//                stringstream ss;
//                ss << fixed << setprecision(3) << receive_Data.depth_data.depth;
//                string depth_str = ss.str();
//                string msg = "$TCP" + depth_str + "!!";
//                write(fd, msg.c_str(), msg.length());
//            }
            

            if ((receive_Data.lock_msg.data != last_lock_state) && receive_Data.lock_msg.data ) {
                string lock = "$L1!";
                write(fd, lock.c_str(), lock.length());
                ROS_INFO("lock-----------------------------------------------------------set");
            }
            if ((receive_Data.lock_msg.data != last_lock_state) && !receive_Data.lock_msg.data ) {
                string lock = "$L0!";
                write(fd, lock.c_str(), lock.length());
                ROS_INFO("lock-----------------------------------------------------------reset");
            }
            last_lock_state = receive_Data.lock_msg.data;

            loop_rate.sleep();
//            count++;
        }
    }

    close(socket_fd);
    return 0;
}
