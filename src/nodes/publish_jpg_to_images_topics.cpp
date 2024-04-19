//
// Created by pointlaz on 4/19/24.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <opencv2/opencv.hpp>

void callbackJPGTopic(const sensor_msgs::Image::ConstPtr& imageJPG, const ros::Publisher& pub)
{
    const cv::Mat_<uchar> in(1, imageJPG->data.size(), const_cast<uchar *>(&imageJPG->data[0]));
    const cv::Mat rgb_a = cv::imdecode(in, cv::IMREAD_UNCHANGED);
    sensor_msgs::Image imageRGB;
    if(sensor_msgs::fillImage(imageRGB, sensor_msgs::image_encodings::RGB8, rgb_a.rows, rgb_a.cols, rgb_a.step, rgb_a.data))
    {
        pub.publish(imageRGB);
    }
    else
    {
        ROS_ERROR_STREAM("[callbackJPGTopic] Error while trying to fill msg for " << pub.getTopic() << " topic");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_camera_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Get topics to subscribe to
    int nbTopics;
    nhp.param<int>("nb_topics", nbTopics, 1);
    std::vector<std::string> vectorTopics;
    std::vector<ros::Subscriber> vectorSubscribers;
    std::vector<ros::Publisher> vectorPublishers;
    for(int i = 0 ; i < nbTopics ; i++)
    {
        std::string paramName = "topics/topic" + std::to_string(i);
        std::string topicName;
        nhp.param<std::string>(paramName, topicName, "");
        vectorPublishers.push_back(nhp.advertise<sensor_msgs::Image>(topicName.substr(1), 1));          // Use substr to remove first "/" to automatically add the node name as a prefix of the topic
        vectorSubscribers.push_back(nhp.subscribe<sensor_msgs::Image>(topicName, 1000, boost::bind(&callbackJPGTopic, _1, vectorPublishers[i])));
    }

    ros::spin();

    return 0;
}