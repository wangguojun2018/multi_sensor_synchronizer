#include <canlib.h>
#include <stdio.h>
#include<stdlib.h>
#include<memory.h>
#include<time.h>
#include<kvaDbLib.h>
#include<autoware_can_msgs/CANPacket.h>
#include<vector>
#include<boost/shared_ptr.hpp>
#include<string>
#include<iostream>

using namespace std;

typedef autoware_can_msgs::CANPacket CANPacket;
typedef boost::shared_ptr<CANPacket> CANPacketPtr;

class can_receive
{
public:
	can_receive(int channel, int bitrate);
	virtual ~can_receive();
	void check(string id, canStatus stat);
	canStatus receiveCan(long id, CANPacketPtr candat);
	void transCan(long id, unsigned char *outmsg);
	void IncreaseId(long *id, int flags);

public:
	canHandle hnd;
	canHandle hnd1;
	// long id;
	canStatus stat;
    char* msg;
    unsigned int i, dlc, flags;
    unsigned long time;
	int chanCount;
};