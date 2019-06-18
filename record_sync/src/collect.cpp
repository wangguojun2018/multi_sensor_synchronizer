//#include "record_sync/collect.h"
#include "../include/record_sync/collect.h"
#include <fstream>
#include <boost/bind/bind.hpp>

#include <sensor_msgs/image_encodings.h> //图像编码格式
#include <opencv2/imgproc/imgproc.hpp>   //图像处理
#include <opencv2/highgui/highgui.hpp>   //opencv GUI

using namespace std;

collect::collect(string velodyne_dir, string image_dir, string imu_dir, string can_dir)
{
     cout << "class collect created a object" << endl;

     cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/velodyne_points", 1000);
     image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/image_raw", 1000);
     fix_sub_ = new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh, "/kitti/oxts/gps/fix", 1000);
     vel_sub_ = new message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>(nh, "gps/vel", 1000);
     vel_no_sub_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh, "/kitti/oxts/gps/vel", 1000);
     odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "gps/odom", 1000);
     imu_sub_ = new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/kitti/oxts/imu", 1000);
     vel_vehicle_sub_ = new message_filters::Subscriber<autoware_can_msgs::CANPacket>(nh, "/vel_vehicle", 1000);
     angle_vehicle_sub_ = new message_filters::Subscriber<autoware_can_msgs::CANPacket>(nh, "/angle_vehicle", 1000);
     // acc_vehicle_sub_ = new message_filters::Subscriber<autoware_can_msgs::CANPacket>(nh, "/acc_vehicle", 1000);
     // postype_sub_ = new message_filters::Subscriber<std_msgs::String>(nh, "gps/pos_type", 1000);

     sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *cloud_sub_, *image_sub_, *fix_sub_, *vel_sub_, *odom_sub_,
                                                             *imu_sub_, *vel_vehicle_sub_, *angle_vehicle_sub_);
     sync__ = new message_filters::Synchronizer<MySyncPolicy_>(MySyncPolicy_(10), *cloud_sub_, *image_sub_,
                                                               *vel_vehicle_sub_, *angle_vehicle_sub_);

     velodyne_dir_ = velodyne_dir;
     image_dir_ = image_dir;
     imu_dir_ = imu_dir;
     can_dir_ = can_dir;
     seq_ = 0;
     // sync_->registerCallback(boost::bind(&collect::CallBack, this, _1, _2, _3, _4, _5, _6, _7, _8));
     sync__->registerCallback(boost::bind(&collect::CallBack_, this, _1, _2, _3, _4));
}

collect::~collect()
{
}

void collect::CallBack(const sensor_msgs::PointCloud2ConstPtr &incloud, const sensor_msgs::ImageConstPtr &inimage, const sensor_msgs::NavSatFixConstPtr &infix,
                       const geometry_msgs::TwistWithCovarianceStampedConstPtr &invelocity, const nav_msgs::OdometryConstPtr &inodom,
                       const sensor_msgs::ImuConstPtr &inimu, const autoware_can_msgs::CANPacketConstPtr &vel_vehicle, const autoware_can_msgs::CANPacketConstPtr &angle_vehicle)
{
     cout << "callback:" << endl;
     cout << "pointcloud stamp is " << incloud->header.stamp<<" ";
     cout << "Image stamp is " << inimage->header.stamp<<" ";
     cout << "gps/fix stamp is " << infix->header.stamp << " ";
     cout << "gps/vel stamp is " << invelocity->header.stamp << " ";
     cout << "imu/data stamp is " << inimu->header.stamp << " "<<endl;

     //转换点云
     pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZI>);
     pcl::fromROSMsg(*incloud, *outcloud);
     std::ostringstream oss;
     oss << std::setfill('0') << std::setw(6) << seq_;
     pcl::io::savePCDFile(velodyne_dir_ + oss.str() + ".pcd", *outcloud,true);

     //写时间戳
     ofstream velodyne_stamp((velodyne_dir_ + "velodyne_stamp.txt").c_str(), ios::app);
     if (!velodyne_stamp)
     {
          velodyne_stamp.open((velodyne_dir_ + "velodyne_stamp.txt").c_str(), ios::out);
     }

     // velodyne_stamp.open(velodyne_dir_ + "velodyne_stamp.txt", ios::out);
     velodyne_stamp << incloud->header.stamp << "\n";

     velodyne_stamp.close();

     //转换图像

     cv_bridge::CvImagePtr cv_ptr;
     cv_ptr = cv_bridge::toCvCopy(inimage, sensor_msgs::image_encodings::BGR8);
     cv::imwrite(image_dir_ + oss.str() + ".png", cv_ptr->image);
     //写时间戳
     ofstream image_stamp((image_dir_ + "image_stamp.txt").c_str(), ios::app);
     if (!velodyne_stamp)
     {
          image_stamp.open((image_dir_ + "image_stamp.txt").c_str(), ios::out);
     }
     image_stamp << inimage->header.stamp << "\n";
     image_stamp.close();

     //转换imu
     ofstream imu_file((imu_dir_ + oss.str() + ".txt").c_str(), ios::out);

     //经纬度信息
     imu_file << infix->latitude << "\n"
              << infix->longitude << "\n"
              << infix->altitude << "\n";
     //速度信息
     imu_file << invelocity->twist.twist.linear.x << "\n"
              << invelocity->twist.twist.linear.y << "\n"
              << invelocity->twist.twist.linear.z << "\n";
     //位置朝向信息
     imu_file << inodom->pose.pose.position.x << "\n"
              << inodom->pose.pose.position.y << "\n"
              << inodom->pose.pose.orientation.x << "\n"
              << inodom->pose.pose.orientation.y << "\n"
              << inodom->pose.pose.orientation.z << "\n"
              << inodom->pose.pose.orientation.w << "\n"
              << inodom->twist.twist.linear.x << "\n"
              << inodom->twist.twist.linear.y << "\n"
              << inodom->twist.twist.angular.z << "\n";
     //角速度和线性加速度信息
     imu_file << inimu->linear_acceleration.x << "\n"
              << inimu->linear_acceleration.y << "\n"
              << inimu->linear_acceleration.z << "\n"
              << inimu->angular_velocity.x << "\n"
              << inimu->angular_velocity.y << "\n"
              << inimu->angular_velocity.z;

     imu_file.close();

     //写时间戳
     ofstream imu_stamp((imu_dir_ + "imu_stamp.txt").c_str(), ios::app);
     if (!velodyne_stamp)
     {
          imu_stamp.open((imu_dir_ + "imu_stamp.txt").c_str(), ios::out);
     }
     imu_stamp << inimu->header.stamp << "\n";
     imu_stamp.close();

     //解析can数据
     unsigned char data_vel[8];
     for (int i = 0; i < vel_vehicle->len; i++)
     {
          data_vel[i] = vel_vehicle->dat[i];
     }
     double velocity_;
     char *vel_signal = (char *)"BR1_Rad_kmh";
     char *dbc_file = (char *)"/home/wangguojun/test.dbc";
     signal_parse(dbc_file, vel_vehicle->id, vel_signal, data_vel, &velocity_);

     unsigned char data_angle[8];
     for (int i = 0; i < angle_vehicle->len; i++)
     {
          data_angle[i] = angle_vehicle->dat[i];
     }
     double angle_;
     char *angle_signal = (char *)"LW1_LRW";
     signal_parse(dbc_file, angle_vehicle->id, angle_signal, data_angle, &angle_);


     double angleSign_;
     char *angleSign_signal = (char *)"LW1_LRW_Sign";
     signal_parse(dbc_file, angle_vehicle->id, angleSign_signal, data_angle, &angleSign_);


     double angleSpeed_;
     char *angleSpeed_signal = (char *)"LW1_Lenk_Gesch";
     signal_parse(dbc_file, angle_vehicle->id, angleSpeed_signal, data_angle, &angleSpeed_);


     double angleSpeedSign_;
     char *angleSpeedSign_signal = (char *)"LW1_Gesch_Sign";
     signal_parse(dbc_file, angle_vehicle->id, angleSpeedSign_signal, data_angle, &angleSpeedSign_);


     ofstream can_file((can_dir_ + oss.str() + ".txt").c_str(), ios::out);
     can_file << velocity_ << "\n"
              << angle_ << "\n"
              << angleSign_ << "\n"
              << angleSpeed_ << "\n"
              << angleSpeedSign_;
     can_file.close();

     seq_ += 1;
}
KvaDbStatus collect::signal_parse(char *filename, unsigned int messageid, char *signalname, unsigned char *data, double *value)
{
     KvaDbStatus status;
     KvaDbHnd dh = 0;
     KvaDbMessageHnd mh;
     KvaDbSignalHnd sh;
     // Open a database handle
     status = kvaDbOpen(&dh);
     if (status != kvaDbOK)
     {
          printf("kvaDbOpen falied: %d\n", status);
          return status;
     }
     // Load the database file
     status = kvaDbReadFile(dh, filename);
     if (status != kvaDbOK)
     {
          printf("Could not load '%s': %d\n", filename, status);
          return status;
     }
     // Search for a message by indentifier
     status = kvaDbGetMsgById(dh, messageid, &mh);
     if (status != kvaDbOK)
     {
          printf("Could not find the message with identifier '340':%d\n", status);
          return status;
     }

     // Search for a signal in this message
     status = kvaDbGetSignalByName(mh, signalname, &sh);
     if (status != kvaDbOK)
     {
          printf("Could not find the signal with name 'T01':%d\n", status);
          return status;
     }
     status = kvaDbGetSignalValueFloat(sh, value, data, sizeof(data));
     if (status != kvaDbOK)
     {
          printf("kvaDbGetSignalValueFloat failed: %d\n", status);
          return status;
     }
     status = kvaDbClose(dh);
     if (status != kvaDbOK)
     {
          printf("kvaDbClose failed: %d\n", status);
          return status;
     }

     return status;
}
void collect::CallBack_(const sensor_msgs::PointCloud2ConstPtr &incloud, const sensor_msgs::ImageConstPtr &inimage, const autoware_can_msgs::CANPacketConstPtr &vel_vehicle,
                        const autoware_can_msgs::CANPacketConstPtr &angle_vehicle)
{
     cout << "callback:" << endl;
     cout << "pointcloud stamp is " << incloud->header.stamp << " ";
     cout << "Image stamp is " << inimage->header.stamp << " ";
     // cout << "gps/fix stamp is " << infix->header.stamp << " ";
     // cout << "gps/vel stamp is " << invelocity->header.stamp << " ";
     // cout << "imu stamp is " << inimu->header.stamp << " " << endl;

     //转换点云
     pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZI>);
     pcl::fromROSMsg<pcl::PointXYZI>(*incloud, *outcloud);
     std::ostringstream oss;
     oss << std::setfill('0') << std::setw(6) << seq_;
     pcl::io::savePCDFile<pcl::PointXYZI>(velodyne_dir_ + oss.str() + ".pcd", *outcloud);

     //写时间戳
     ofstream velodyne_stamp((velodyne_dir_ + "velodyne_stamp.txt").c_str(), ios::app);
     if (!velodyne_stamp)
     {
          velodyne_stamp.open((velodyne_dir_ + "velodyne_stamp.txt").c_str(), ios::out);
     }

     // velodyne_stamp.open(velodyne_dir_ + "velodyne_stamp.txt", ios::out);
     velodyne_stamp << incloud->header.stamp << "\n";

     velodyne_stamp.close();

     //转换图像

     cv_bridge::CvImagePtr cv_ptr;
     cv_ptr = cv_bridge::toCvCopy(inimage, sensor_msgs::image_encodings::BGR8);
     cv::imwrite(image_dir_ + oss.str() + ".png", cv_ptr->image);
     //写时间戳
     ofstream image_stamp((image_dir_ + "image_stamp.txt").c_str(), ios::app);
     if (!velodyne_stamp)
     {
          image_stamp.open((image_dir_ + "image_stamp.txt").c_str(), ios::out);
     }
     image_stamp << inimage->header.stamp << "\n";
     image_stamp.close();

     // //转换imu
     // ofstream imu_file((imu_dir_ + oss.str() + ".txt").c_str(), ios::out);
     // // if(!imu_file)
     // // {
     // //      imu_file.open((imu_dir_+oss.str()+".txt").c_str(),ios::out);
     // // }

     // imu_file << infix->latitude << "\n"
     //          << infix->longitude << "\n"
     //          << infix->altitude << "\n";
     // imu_file << invelocity->twist.linear.x << "\n"
     //          << invelocity->twist.linear.y << "\n"
     //          << invelocity->twist.linear.z << "\n";
     // //  << invelocity->twist.twist.angular << "\n";

     // imu_file << inimu->linear_acceleration.x << "\n"
     //          << inimu->linear_acceleration.y << "\n"
     //          << inimu->linear_acceleration.z << "\n"
     //          << inimu->angular_velocity.x << "\n"
     //          << inimu->angular_velocity.y << "\n"
     //          << inimu->angular_velocity.z;

     // imu_file.close();

     // //写时间戳
     // ofstream imu_stamp((imu_dir_ + "imu_stamp.txt").c_str(), ios::app);
     // if (!velodyne_stamp)
     // {
     //      imu_stamp.open((imu_dir_ + "imu_stamp.txt").c_str(), ios::out);
     // }
     // imu_stamp << inimu->header.stamp << "\n";
     // imu_stamp.close();
     //解析can数据
     unsigned char data_vel[8];
     for (int i = 0; i < vel_vehicle->len; i++)
     {
          data_vel[i] = vel_vehicle->dat[i];
     }
     double velocity_;
     char *vel_signal = (char *)"BR1_Rad_kmh";
     char *dbc_file = (char *)"/home/ainno/dataset/suteng516.dbc";
     signal_parse(dbc_file, vel_vehicle->id, vel_signal, data_vel, &velocity_);

     unsigned char data_angle[8];
     for (int i = 0; i < angle_vehicle->len; i++)
     {
          data_angle[i] = angle_vehicle->dat[i];
     }
     double angle_;
     char *angle_signal = (char *)"LW1_LRW";
     signal_parse(dbc_file, angle_vehicle->id, angle_signal, data_angle, &angle_);
     cout << "方向盘角度大小为： " << angle_ << endl;


     double angleSign_;
     char *angleSign_signal = (char *)"LW1_LRW_Sign";
     signal_parse(dbc_file, angle_vehicle->id, angleSign_signal, data_angle, &angleSign_);
     cout << "方向盘角度符号为： " << angleSign_ << endl;


     double angleSpeed_;
     char *angleSpeed_signal = (char *)"LW1_Lenk_Gesch";
     signal_parse(dbc_file, angle_vehicle->id, angleSpeed_signal, data_angle, &angleSpeed_);


     double angleSpeedSign_;
     char *angleSpeedSign_signal = (char *)"LW1_Gesch_Sign";
     signal_parse(dbc_file, angle_vehicle->id, angleSpeedSign_signal, data_angle, &angleSpeedSign_);


     ofstream can_file((can_dir_ + oss.str() + ".txt").c_str(), ios::out);
     can_file << velocity_ << "\n"
              << angle_ << "\n"
              << angleSign_ << "\n"
              << angleSpeed_ << "\n"
              << angleSpeedSign_;
     can_file.close();

     seq_ += 1;
}
