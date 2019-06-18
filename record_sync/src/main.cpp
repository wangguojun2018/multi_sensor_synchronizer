#include"record_sync/collect.h"
#include<ros/ros.h>
#include<string.h>


using namespace std;

string velodyne_dir = "/home/wangguojun/dataset/velodyne_dir/";
string image_dir = "/home/wangguojun/dataset/image_dir/";
string imu_dir = "/home/wangguojun/dataset/imu_dir/";
string can_dir = "/home/wangguojun/dataset/can_dir/";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_record_node");

    collect mycollect(velodyne_dir,image_dir,imu_dir,can_dir);

    ros::spin();

    return 0;
}





