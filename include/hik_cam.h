#ifndef HIK_CAM_H
#define HIK_CAM_H

#include <iostream>
#include <string>

#include "hikvision_sdk/HCNetSDK.h"
#include "hikvision_sdk/PlayM4.h"
#include "hikvision_sdk/error.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse.h>

class HikvisionCamera
{
private:

    /// ros parameters
    image_transport::CameraPublisher image_pub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_mgr;
    ros::ServiceServer SetCameraInfoSrv;


    /// camera parameters
    int image_width;
    int image_height;

    std::string ip_addr;
    std::string usr_name;
    std::string password;
    std::string frame_id;
    std::string camera_name;
    std::string camera_info_url;

    int port;
    int channel;
    int link_mode;

    LONG user_id;
    LONG data_play_handler;

    static void decodeCallback_(int nPort, char *pBuf, int nSize, FRAME_INFO *pFrameInfo, void *nUser, int nReserved2)
    {
        ((HikvisionCamera *) nUser)->decodeCallback(nPort, pBuf, nSize, pFrameInfo);
    }

    static void dataCallback(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *pUser)
    {
        ((HikvisionCamera *) pUser)->dataCallback(lRealHandle, dwDataType, pBuffer, dwBufSize);
    }

    std::string expandUserPath(std::string path);

    void decodeCallback(int nPort, char *pBuf, int nSize, FRAME_INFO *pFrameInfo);

    void dataCallback(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize);

    bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp);

    bool initHikSDK();

    void initROSIO(ros::NodeHandle& priv_node);


public:

    void run();

    ~HikvisionCamera();

};

#endif //HIK_CAM_H
