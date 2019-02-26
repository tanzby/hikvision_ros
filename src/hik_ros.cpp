#define  __APP_NAME__ "HIKCAM"

#include <iostream>
#include <string>

#include "hikvision_sdk/HCNetSDK.h"
#include "hikvision_sdk/PlayM4.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

LONG lPort;
unsigned int frame_col=1280,frame_row = 720;

// var for ros
image_transport::Publisher image_pub;

void CALLBACK decode_callback(int nPort, char* pBuf, int nSize, FRAME_INFO * pFrameInfo, void* nReserved1, int nReserved2)
{

    int lFrameType = pFrameInfo->nType;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "hik_cam";

    if (lFrameType == T_YV12)
    {
        
        cv::Mat picBGR;
        cv::Mat picYV12 = cv::Mat(pFrameInfo->nHeight * 3/2, pFrameInfo->nWidth, CV_8UC1, pBuf);
        cv::cvtColor(picYV12, picBGR, cv::COLOR_YUV2BGR_YV12);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", picBGR).toImageMsg();
        image_pub.publish(msg);

        ROS_INFO("[%s] Stream CallBack, Convert YV12 to sensor_msgs.",__APP_NAME__);
    }
}


void CALLBACK g_HikDataCallBack(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize,DWORD dwUser)
{
    switch (dwDataType)
    {
        case NET_DVR_SYSHEAD:    //system head
        {
            if (!PlayM4_GetPort(&lPort)) //unavailable lPort
            {
                break;
            }
            if (dwBufSize > 0)
            {
                if (!PlayM4_SetStreamOpenMode(lPort, STREAME_REALTIME))  //set strean mode
                {
                    break;
                }
                if (!PlayM4_OpenStream(lPort, pBuffer, dwBufSize, frame_col * frame_row))
                {
                    //dRet = PlayM4_GetLastError(lPort);
                    break;
                }
                //decoding callback
                if (!PlayM4_SetDecCallBack(lPort,decode_callback))
                {
                    //dRet = PlayM4_GetLastError(lPort);
                    break;
                }
                //open decoding
                if (!PlayM4_Play(lPort, 0)) // play the video stream
                {
                    //dRet = PlayM4_GetLastError(lPort);
                    break;
                }
            }
        } break;


        case NET_DVR_STREAMDATA:
        {
            if (dwBufSize > 0 && lPort != -1)
            {
                if (!PlayM4_InputData(lPort, pBuffer, dwBufSize))
                {
                    break;
                }
            }
        } break;

        default:
        {
            if (dwBufSize > 0 && lPort != -1)
            {
                if (!PlayM4_InputData(lPort, pBuffer, dwBufSize))
                {
                    break;
                }
            }
        } break;
    }
}

bool get_stream(const std::string &ip_addr, const std::string &usr_name, const std::string &password,
                int port, int channel, int link_mode)
{
    LONG lUserID;

    NET_DVR_Init();
    NET_DVR_SetLogToFile(3, "./hiklog");
    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    NET_DVR_DEVICEINFO_V40  struDeviceInfoV40 = {0};
    struLoginInfo.bUseAsynLogin = false;

    struLoginInfo.wPort = port;
    memcpy(struLoginInfo.sDeviceAddress,ip_addr.c_str(), NET_DVR_DEV_ADDRESS_MAX_LEN);
    memcpy(struLoginInfo.sUserName, usr_name.c_str(), NAME_LEN);
    memcpy(struLoginInfo.sPassword, password.c_str(), NAME_LEN);
    lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);

    if (lUserID < 0)
    {
        ROS_INFO("[%s] Login fail, error code: %d",__APP_NAME__,NET_DVR_GetLastError());
        return false;
    }

    //Set callback function of getting stream.
    LONG lRealPlayHandle;
    NET_DVR_PREVIEWINFO struPlayInfo = {0};
    struPlayInfo.hPlayWnd     = 0;  
    struPlayInfo.lChannel     = channel;  //channel NO
    struPlayInfo.dwLinkMode   = link_mode;
    struPlayInfo.bBlocked     = 1;
    struPlayInfo.dwDisplayBufNum = 1;

    lRealPlayHandle = NET_DVR_RealPlay_V40(lUserID, &struPlayInfo, NULL, NULL);

    if (lRealPlayHandle < 0)
    {
        ROS_ERROR("[%s] NET_DVR_RealPlay_V40 error, %d\n",__APP_NAME__ ,NET_DVR_GetLastError());
        NET_DVR_Logout(lUserID);
        NET_DVR_Cleanup();
        return false;
    }
    else ROS_INFO("[%s] Set Play Handler successully.",__APP_NAME__);

    //Set callback function of getting stream.
    int iRet;
    iRet = NET_DVR_SetRealDataCallBack(lRealPlayHandle, g_HikDataCallBack, 0);
    if (!iRet)
    {
        ROS_ERROR("[%s] NET_DVR_RealPlay_V40 error\n", __APP_NAME__);
        NET_DVR_StopRealPlay(lRealPlayHandle);
        NET_DVR_Logout_V30(lUserID);
        NET_DVR_Cleanup();
        return false;
    }
    else ROS_INFO("[%s] Set Data Callback successully.",__APP_NAME__);
       
    ROS_INFO("[%s] Waiting for data.",__APP_NAME__);
    ros::spin();

    NET_DVR_StopRealPlay(lRealPlayHandle);
    NET_DVR_Logout_V30(lUserID);
    NET_DVR_Cleanup();

    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, __APP_NAME__);

    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");

    std::string topic_name = "hik_cam";
    ROS_INFO("[%s] image of camera will pulish to: %s", __APP_NAME__, topic_name.c_str());
    image_transport::ImageTransport it(node);
    image_pub = it.advertise(topic_name, 2, true);


    std::string ip_addr;
    std::string usr_name;
    std::string password;

    int port;
    int channel;
    int link_mode;

    priv_node.param<std::string>("ip_addr", ip_addr,"192.168.5.100");
    ROS_INFO("[%s] ip address:\t%s", __APP_NAME__, ip_addr.c_str());

    priv_node.param<std::string>("usr_name",usr_name,"admin");
    ROS_INFO("[%s] user name: \t%s", __APP_NAME__, usr_name.c_str());

    priv_node.param<std::string>("password",password,"ht123456");
    ROS_INFO("[%s] password:  \t%s", __APP_NAME__, password.c_str());

    priv_node.param<int>("port",port, 8000);
    ROS_INFO("[%s] port:      \t%d", __APP_NAME__, port);

    priv_node.param<int>("channel",channel,1);
    ROS_INFO("[%s] channel:   \t%d", __APP_NAME__, channel);

    priv_node.param<int>("link_mode",link_mode, 0);
    if(link_mode < 0 || link_mode >5)
    {
        ROS_WARN("[%s] value %d for link_mode is illegal, set to default value 0 (tcp)",__APP_NAME__, link_mode);
    }

    std::string _mode []  = {"tcp", "udp", "multicast","rtp","rtp/rtsp", "rstp/http"};
    ROS_INFO("[%s] link mode: \t%s", __APP_NAME__, _mode[link_mode].c_str());

    get_stream(ip_addr, usr_name, password, port, channel, link_mode);

    return 0;
}
