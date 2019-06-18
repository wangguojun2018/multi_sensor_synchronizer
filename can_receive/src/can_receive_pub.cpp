#include "ros/ros.h"
#include "std_msgs/String.h"
#include <can_receive/can_receive.h>
#include <autoware_can_msgs/CANPacket.h>
#include <sstream>
using namespace std;

typedef autoware_can_msgs::CANPacket CANPacket;
typedef boost::shared_ptr<CANPacket> CANPacketPtr;

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "can_decode");

	ros::NodeHandle n;

	ros::Publisher vel_pub = n.advertise<CANPacket>("vel_vehicle", 1000);
	ros::Publisher angle_pub = n.advertise<CANPacket>("angle_vehicle", 1000);
	// ros::Publisher acc_pub = n.advertise<CANPacket>("acc_vehicle", 1000);

	ros::Rate loop_rate(10);

	int channel = 1;
	int bitrate = BAUD_500K;
	can_receive mycanhandle(channel, bitrate);
	canStatus stat_vel;
	int id_vel = 0xC2;
	canStatus stat_angle;
	int id_angle = 0x1A0;
	canStatus stat_acc;
	int id_acc = 3;
	// int count = 0;
	while (ros::ok())
	{

		// mycanhandle.transCan(id_vel);
		CANPacketPtr canpacket_vel(new CANPacket);
		stat_vel = mycanhandle.receiveCan(id_vel, canpacket_vel);
		if (stat_vel == canOK)
		{
			vel_pub.publish(*canpacket_vel);
		}

		CANPacketPtr canpacket_angle(new CANPacket);
		stat_angle = mycanhandle.receiveCan(id_angle, canpacket_angle);
		if (stat_angle == canOK)
		{
			angle_pub.publish(*canpacket_angle);
		}
		// CANPacketPtr canpacket_acc(new CANPacket);
		// stat_acc = mycanhandle.receiveCan(id_acc, canpacket_acc);
		// if (stat_acc == canOK)
		// {
		// 	acc_pub.publish(*canpacket_acc);
		// }
		ros::spinOnce();
		// loop_rate.sleep();
	}

	ros::shutdown();

	return 0;
}