/// Created by pointlaz

#include <ros/ros.h>
#include <avt_vimba_camera/multi_camera.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_camera_node");
    {
        ros::NodeHandle nh;
        ros::NodeHandle nhp("~");
        avt_vimba_camera::MultiCamera mc(nh, nhp);
        ros::spin();
    }
    return 0;
}