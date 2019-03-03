#include "hik_cam.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hik_cam_node");

    HikvisionCamera camera;
    camera.run();

    return 0;
}