#include<iostream>
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<math.h>
#include<fstream>
#include<sstream>
#include<memory>
#include<vector>
#include<climits>
#include<thread>

#include<txt_to_rosmsg/cam_data.h>

using namespace std;

void extractOdo(string Path, ros::Publisher pubOdo, double(*func)(string)){
    nav_msgs::Odometry odoData;
    ifstream File(Path);
    string temp;
    vector<string> res;
    while (getline(File, temp))
    {
        stringstream input(temp);
        string out;
        while(input >> out){
            res.push_back(out);
        }
        odoData.header.stamp.sec = func(res[0]);
        odoData.header.stamp.nsec = (func(res[0]) - odoData.header.stamp.sec) * 1e9;
        odoData.twist.twist.linear.x = func(res[1]);
        odoData.twist.twist.linear.y = 0.0;
        odoData.twist.twist.linear.z = 0.0;
        odoData.twist.twist.angular.x = 0.0;
        odoData.twist.twist.angular.y = 0.0;
        odoData.twist.twist.angular.z = func(res[2]);
        pubOdo.publish(odoData);
        ROS_INFO("publish odoData");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //Queue.push_back(res);
        res.clear();
    }
    return;
}

void extractCam(string Path, ros::Publisher pubCam, double(*func)(string)){
    txt_to_rosmsg::cam_data camData;
    camData.qcl.assign(4, 0);              //对于自定义数组类型，需要预设大小，否则会越界
    camData.tlc.assign(3, 0);
    camData.q_x.assign(4, 0);
    ifstream File(Path);
    string temp;
    vector<string> res;
    while (getline(File, temp))
    {
        stringstream input(temp);
        string out;
        while(input >> out){
            res.push_back(out);
        }
        camData.start_t = func(res[0]);
        camData.end_t = func(res[1]);
        camData.tcl_length = func(res[2]);
        camData.theta_yaw = func(res[3]);
        camData.deltaTheta = func(res[4]);
        camData.qcl[0] = func(res[5]);
        camData.qcl[1] = func(res[6]);
        camData.qcl[2] = func(res[7]);
        camData.qcl[3] = func(res[8]);
        camData.tlc[0] = func(res[9]);
        camData.tlc[1] = func(res[10]);
        camData.tlc[2] = func(res[11]);
        camData.q_x[0] = func(res[12]);
        camData.q_x[1] = func(res[13]);
        camData.q_x[2] = func(res[14]);
        camData.q_x[3] = func(res[15]);
        pubCam.publish(camData);
        ROS_INFO("publish camData");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        //Queue.push_back(res);
        res.clear();
    }
    return;
}

double strTodou(string str){  //将string类型转化为double类型       atof(指针)  括号内必须为指针才能使用
    char* ch = new char[0];
    double d;
    for(int i = 0; i < str.length(); ++i){
        ch[i] = str[i];
    }
    d = atof(ch);
    delete []ch;
    ch = NULL;
    return d;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "txt_to_rosmsg");
    ros::NodeHandle nh;
    ros::Publisher pubOdo = nh.advertise<nav_msgs::Odometry>("/wheel_data", 1000 );
    ros::Publisher pubCam = nh.advertise<txt_to_rosmsg::cam_data>("/cam_data", 1000);

    ros::Rate odoRate(100);
    ros::Rate camRate(50);

    string odoDataPath = "/home/gky/odo1.txt";
    string camDataPath = "/home/gky/cam1.txt";

    vector<vector<string>> odoQueue;
    vector<vector<string>> camQueue;

    thread odo(extractOdo, odoDataPath, pubOdo, strTodou);   //由于需要odo和cam以不同的频率发布，因此采用多线程
    thread cam(extractCam, camDataPath, pubCam, strTodou);

    ros::spin();

    odo.join();
    cam.join();
    
    return 0;
}

