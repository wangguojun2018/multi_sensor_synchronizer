#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <string>
#include <autoware_can_msgs/CANPacket.h>
#include<kvaDbLib.h>

using namespace std;

// typedef autoware_can_msgs::CANPacketConstPtr CANPacketConstPtr;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::NavSatFix,
                                                        geometry_msgs::TwistWithCovarianceStamped, nav_msgs::Odometry, sensor_msgs::Imu, autoware_can_msgs::CANPacket,
                                                        autoware_can_msgs::CANPacket>
    MySyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image,autoware_can_msgs::CANPacket,
                                                        autoware_can_msgs::CANPacket>
    MySyncPolicy_;

class collect
{
public:
    collect(string velodyne_dir, string image_dir, string imu_dir, string can_dir);
    virtual ~collect();

    ros::NodeHandle nh;

    void CallBack(const sensor_msgs::PointCloud2ConstPtr &incloud, const sensor_msgs::ImageConstPtr &inimage, const sensor_msgs::NavSatFixConstPtr &infix,
                  const geometry_msgs::TwistWithCovarianceStampedConstPtr &invelocity, const nav_msgs::OdometryConstPtr &inodom,
                  const sensor_msgs::ImuConstPtr &inimu, const autoware_can_msgs::CANPacketConstPtr &vel_vehicle,
                  const autoware_can_msgs::CANPacketConstPtr &angle_vehicle);
    void CallBack_(const sensor_msgs::PointCloud2ConstPtr &incloud, const sensor_msgs::ImageConstPtr &inimage,const autoware_can_msgs::CANPacketConstPtr &vel_vehicle,
                  const autoware_can_msgs::CANPacketConstPtr &angle_vehicle);
    KvaDbStatus signal_parse(char *filename, unsigned int messageid, char *signalname, unsigned char *data, double *value);

private:
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> *fix_sub_;
    message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> *vel_sub_;
    message_filters::Subscriber<geometry_msgs::TwistStamped> *vel_no_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> *imu_sub_;
    message_filters::Subscriber<autoware_can_msgs::CANPacket> *vel_vehicle_sub_;
    message_filters::Subscriber<autoware_can_msgs::CANPacket> *angle_vehicle_sub_;
    message_filters::Subscriber<autoware_can_msgs::CANPacket> *acc_vehicle_sub_;
    message_filters::Synchronizer<MySyncPolicy> *sync_;
    message_filters::Synchronizer<MySyncPolicy_> *sync__;
    int seq_;

    string velodyne_dir_;
    string image_dir_;
    string imu_dir_;
    string can_dir_;
};
