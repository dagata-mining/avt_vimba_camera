//
// Test executable to check if images are saved correctly
//


#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Header.h>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <sensor_msgs/image_encodings.h>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH





