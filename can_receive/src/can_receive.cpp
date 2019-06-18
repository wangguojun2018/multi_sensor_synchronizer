#include "can_receive/can_receive.h"

can_receive::can_receive(int channel, int bitrate)
{

    // canStatus stat;
    //
    // First, open a handle to the CAN circuit. Specifying
    // canOPEN_EXCLUSIVE ensures we get a circuit that noone else
    // is using.
    //初始化
    canInitializeLibrary();
    // stat = canUnloadLibrary();
    // if (stat < 0)
    // {
    //     printf("Init library failed.");
    // }
    stat = canGetNumberOfChannels(&chanCount);
    if (chanCount < 0 || chanCount > 300)
    {
        printf("ChannelCount = %d but I don't believe it.\n", chanCount);
        exit(1); // Error
    }
    else
    {
        printf("%d channels.\n", chanCount);
    }

    // printf("canOpenChannel, channel %d... ", channel);
    hnd = canOpenChannel(channel, canOPEN_ACCEPT_VIRTUAL);
    if (hnd < 0)
    {
        check("canOpenChannel", (canStatus)hnd);
        exit(1);
    }
    printf("OK.\n");
    //
    // Using our new shiny handle, we specify the baud rate
    // using one of the convenient canBITRATE_xxx constants.
    //
    // The bit layout is in depth discussed in most CAN
    // controller data sheets, and on the web at
    // http://www.kvaser.se.
    //
    // printf("Setting the bus speed...");
    // stat = canSetBusParams(hnd, bitrate, 0, 0, 0, 0, 0);
    // if (stat < 0)
    // {
    //     printf("canSetBusParams failed, stat=%d\n", stat);
    // }
    // printf("OK.\n");
    //
    // Then we start the ball rolling.
    //
    printf("Go bus-on...");
    stat = canBusOn(hnd);
    if (stat < 0)
    {
        printf("canBusOn failed, stat=%d\n", stat);
    }
    printf("OK.\n");
}

can_receive::~can_receive()
{
    // free(msg);
    canBusOff(hnd);
    // canBusOff(hnd1);
    canClose(hnd);
    // canClose(hnd1);
}

//状态检查
void can_receive::check(string id, canStatus stat)
{
    if (stat != canOK)
    {
        char buf[50];
        buf[0] = '\0';
        // Retreive informational text about the status code
        canGetErrorText(stat, buf, sizeof(buf));
        // printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
        cout << id << "failed, stat=" << (int)stat << " " << buf << endl;
    }
    else
    {
        // printf("%s: success,stat=%d \n", id, (int)stat);
        cout << id << "success, stat=" << (int)stat << endl;
    }

} // check
//打开通道
canStatus can_receive::receiveCan(long id, CANPacketPtr candat)
{
    // free(msg); //释放上一次读取占用的内存空间
    // msg = (char *)malloc(8 * sizeof(char));
    unsigned char inmsg[8];
    unsigned char msg_[8];
    // long id_;
    // stat = canReadWait(hnd, &id_, inmsg, &dlc, &flags, &time, 250);
    stat = canReadSyncSpecific(hnd, id, 500);
    if (stat == canOK)
    {
        stat = canReadSpecific(hnd, id, inmsg, &dlc, &flags, &time);
    }

    check("接收 ", stat);
    cout << "接收message: Id为" << id << " "
         << "数据为 " << inmsg << endl;
    // string string_id=std::to_string(id);
    // check(string_id, stat);

    switch (stat)
    {
    case 0:
        printf("id:%ld dlc:%d data: ", id, dlc);
        if (dlc > 8)
        {
            dlc = 8;
        }
        for (unsigned int j = 0; j < dlc; j++)
        {
            printf("%2.2x ", inmsg[j]);
            candat->dat[j] = inmsg[j];
        }
        printf(" flags:0x%x time:%ld\n", flags, time);
        candat->count = 0;
        candat->time = time;
        candat->id = id;
        candat->len = dlc;
        candat->header.stamp = ros::Time::now();
        // cout << "转换后数据为 " << candat->dat[0]<<" "<<candat->dat[1]<<" "<<candat->dat[2] << endl;
        //  unsigned char data_vel[8];
        // for (int i = 0; i < candat->len; i++)
        // {
        //     msg_[i] = candat->dat[i];
        // }
        // cout << "转换回数据为 " << msg_ << endl;
        // can_pub.publish(candat);
    case canERR_NOMSG:
        break;
    default:
        perror("canReadBlock error");
        break;
    }
    return stat;
}
void can_receive::transCan(long id, unsigned char *outmsg)
{
    // IncreaseId(&id, flags);
    // unsigned char outmsg[8] = "246";
    canStatus stat_wait = canWriteSync(hnd, -1);
    check("等待 ", stat_wait);
    if (stat_wait == canOK)
    {
        canStatus stat_write = canWrite(hnd, id, outmsg, sizeof(outmsg), 0);
        cout << "发送message: Id为" << id << " "
             << "数据为 " << outmsg << endl;
        check("发送 ", stat_write);
    }

    // Print the message we just sent.
    //
    // if (true)
    // {
    //     printf("%8ld%c%c%c  %u   ",
    //            id,
    //            (flags & canMSG_EXT) ? 'x' : ' ',
    //            (flags & canMSG_RTR) ? 'R' : ' ',
    //            (flags & canMSGERR_OVERRUN) ? 'o' : ' ',
    //            dlc);
    //     // Print the data bytes, but not for Remote Frames.
    //     if ((flags & canMSG_RTR) == 0)
    //     {
    //         for (i = 0; i < dlc; i++)
    //             printf("%3u ", msg[i]);
    //         for (; i < 8; i++)
    //             printf("    ");
    //     }
    //     // Print the time, in raw format.
    //     printf("%08lu\n", time);
    // }
}
