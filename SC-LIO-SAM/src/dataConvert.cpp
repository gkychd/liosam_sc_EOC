#include "utility.h"
//#include<ros/ros.h>

//#include<std_msgs/Header.h>
//#include<sensor_msgs/Imu.h>
//#include<sensor_msgs/NavSatFix.h>

using namespace std;

class data_convert: public ParamServer
{
private:

    ros::Publisher pubGps;
    ros::Publisher pubImu;
    ros::Subscriber subGps;
    ros::Subscriber subImu;

public:

    data_convert()
    {
        //血的教训 这里折腾了一个小时，一直接受不到消息，一切都正常，但是无法接受到消息
        //当时代码写的如下形式，导致无法接受到message
        //ros::Subscriber subGps = nh.subscribe<sensor_msgs::NavSatFix>("navsat/fix", 100, &data_convert::convertGPS, this, ros::TransportHints().tcpNoDelay());
        subGps = nh.subscribe<sensor_msgs::NavSatFix>("navsat/fix", 100, &data_convert::convertGPS, this, ros::TransportHints().tcpNoDelay());
        subImu = nh.subscribe<sensor_msgs::Imu>("imu/data", 1000, &data_convert::convertIMU, this, ros::TransportHints().tcpNoDelay());
        pubGps = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 100);
        pubImu = nh.advertise<sensor_msgs::Imu>("imu_correct", 1000);
    }

    ~data_convert(){}

    void convertGPS(const sensor_msgs::NavSatFix::ConstPtr& GPSmsg){
        sensor_msgs::NavSatFix GPS = *GPSmsg;
        GPS.header.frame_id = "base_link";
        pubGps.publish(GPS);
    }

    void convertIMU(const sensor_msgs::Imu::ConstPtr& IMUmsg){
        sensor_msgs::Imu IMU = *IMUmsg;
        IMU.header.frame_id = "base_link";
        pubImu.publish(IMU);
    }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "lio_sam");
    data_convert DC;

    ROS_INFO("\033[1;32m----> Data Convert Started.\033[0m");
    ros::spin();
    return 0;
}