/// Created by pointlaz

#include <avt_vimba_camera/multi_camera.h>

#define DEBUG_PRINTS 1

namespace avt_vimba_camera
{
    MultiCamera::MultiCamera(ros::NodeHandle& nh, ros::NodeHandle& nhp)
            : nh_(nh), nhp_(nhp), it_(nhp)
    {
        // Start Vimba & list all available cameras
        try
        {
            api_.reset(new AvtVimbaApi);
            api_->start();
        }
        catch (std::exception &e)
        {
            ROS_ERROR("api start error because %s", e.what());
        }

        // Set the params

        nhp_.param("camera_qty", camQty_, 1);
        nhp_.param("compress_jpeg_vimba", compressJPG_, true);
        std::string topicName = "image_raw_";
        nhp_.param("compress_jpeg_quality", qualityJPG_, 90);
        nhp_.param("calculate_color_intensity", calculateColorIntensity_, true);
        nhp_.param("color_intensity_pixel_steps", colorIntensityPxSteps_, 10);
        nhp_.param("color_intensity_RGB", colorIntensityRGB_);

        guid_.resize(camQty_);
        pub_.resize(camQty_);
        camera_info_url_.resize(camQty_);
        frame_id_.resize(camQty_);
        cam_.resize(camQty_);
        name_.resize(camQty_);
        info_man_.resize(camQty_);
        if (calculateColorIntensity_)
            colorPub_.resize(camQty_);

        for (int i = 0; i < camQty_; i++)
        {
            pub_[i] = it_.advertiseCamera(topicName + std::to_string(i), 1);
            if (calculateColorIntensity_)
            {
                ROS_INFO("-------------Color Intensity");
                colorPub_[i] = nh_.advertise<std_msgs::UInt8>("/multi_camera/color_intensity_" + std::to_string(i), 1);
            }
            nhp_.param("guid_" + std::to_string(i), guid_[i], std::string(""));
            nhp_.param("camera_info_url_" + std::to_string(i), camera_info_url_[i], std::string(""));
            nhp_.param("frame_id_" + std::to_string(i), frame_id_[i], std::string(""));
            nhp_.param("name_" + std::to_string(i), name_[i], std::string(""));
            nhp_.param("print_all_features", print_all_features_, false);
            nhp_.param("use_measurement_time", use_measurement_time_, false);
            nhp_.param("ptp_offset", ptp_offset_, 0);

            try
            {
                ROS_INFO("-------------New Cam");
                std::shared_ptr<AvtVimbaCamera>
                    cam = std::shared_ptr<AvtVimbaCamera>(new AvtVimbaCamera(frame_id_[i], i));
                cam_[i] = cam;
            }
            catch (std::exception &e)
            {
                ROS_ERROR("Creating cam %d error because %s", i, e.what());
            }

        }

        // Start dynamic_reconfigur & run configure()
        try
        {
            ROS_INFO("-------------Reconfig");
            reconfigure_server_.setCallback(
                std::bind(&avt_vimba_camera::MultiCamera::configure,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2));
        }
        catch (std::exception &e)
        {
            ROS_ERROR("Reconfiguring error because %s", e.what());
        }

        for (int i = 0; i < camQty_; i++)
        {
            try
            {

                cam_[i]->startImaging();
                ROS_INFO("Setting call back cam %i is cam opened: %i", i, cam_[i]->opened_);
                cam_[i]->setCallback(std::bind(&avt_vimba_camera::MultiCamera::frameCallback,
                                               this,
                                               std::placeholders::_1,i));
            }
            catch (std::exception &e)
            {
                ROS_ERROR("Set call backe cam %d error because %s", i, e.what());
            }
        }
    }

    MultiCamera::~MultiCamera(void)
    {
        for (int i = 0 ; i < cam_.size(); i++)
        {
            cam_[i].reset();
            pub_[i].shutdown();
        }
        reconfigure_server_.clearCallback();
        cam_.clear();
        api_.reset();
    }

    void MultiCamera::frameCallback(const FramePtr& vimba_frame_ptr, const int camId)
    {
        ros::Time ros_time = ros::Time::now();
        if (pub_[camId].getNumSubscribers() >= 0)
        {
            sensor_msgs::Image img;
            sensor_msgs::CompressedImage compressed;
            if (api_->frameToImage(vimba_frame_ptr, img))
            {
                sensor_msgs::CameraInfo ci = info_man_[camId]->getCameraInfo();
                // Note: getCameraInfo() doesn't fill in header frame_id or stamp
                ci.header.frame_id = frame_id_[camId];
                if (use_measurement_time_)
                {
                    VmbUint64_t frame_timestamp;
                    try
                    {
                        vimba_frame_ptr->GetTimestamp(frame_timestamp);
                    }
                    catch (std::exception &e)
                    {
                        ROS_ERROR("Frame callback gettimestamp error cam %d error because %s",camId, e.what());
                    }
                    ci.header.stamp = ros_time;
                }
                else
                {
                    ci.header.stamp = ros_time;
                }

                img.header.stamp = ci.header.stamp;

                if (compressJPG_ || calculateColorIntensity_)
                {
                    cv_bridge::CvImagePtr cv_ptr;

                    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);

                    if (calculateColorIntensity_)
                    {
                        std_msgs::UInt8 colorIntensityMsg;
                        colorIntensityMsg.data = 0;
                        try
                        {
                            colorIntensityMsg.data = calculateColorIntensity(cv_ptr->image);
                        }
                        catch (std::exception &e)
                        {
                            ROS_INFO("-----Could not calculate color intensity cam %d because %s", camId, e.what());
                        }
                        //colorPub_[camId].publish(colorIntensityMsg);
                    }

                    if (compressJPG_)
                    {
                        // Compress the image using OpenCV
                        std::vector<int> compression_params;
                        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);  // You can use other parameters like PNG compression
                        compression_params.push_back(qualityJPG_);  // Set the desired image quality (0-100)
                        cv::imencode(".jpg", cv_ptr->image, img.data, compression_params);
                        img.encoding = "jpg";
                        pub_[camId].publish(img, ci);
                    }
                    else
                    {
                        pub_[camId].publish(img, ci);
                    }
                }
                else
                {
                    pub_[camId].publish(img, ci);
                }
            }
            else
            {
                ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
            }
        }
    }

/** Dynamic reconfigure callback
 *
 *  Called immediately when callback first defined. Called again
 *  when dynamic reconfigure starts or changes a parameter value.
 *
 *  @param newconfig new Config values
 *  @param level bit-wise OR of reconfiguration levels for all
 *               changed parameters (0xffffffff on initial call)
 **/
    void MultiCamera::configure(Config& newconfig, uint32_t level)
    {
        ROS_WARN_STREAM("-------------Configure");
        for (int i = 0 ; i < cam_.size();i++)
        {
            ROS_INFO("-------------------------------CAMERA %d", i);
            try
            {
                // The camera already stops & starts acquisition
                // so there's no problem on changing any feature.
                if (!cam_[i]->isOpened())
                {
                    ROS_WARN_STREAM("-------------START CAM " << i);
                    cam_[i]->start(ip_, guid_[i], frame_id_[i], print_all_features_);
                    ROS_WARN_STREAM("-------------STOP IMAGING CAM " << i);
                    cam_[i]->stopImaging();
                    ROS_WARN_STREAM("-------------UPDATE CONFIG CAM " << i);
                    cam_[i]->updateConfig(newconfig);
                    ROS_WARN_STREAM("-------------START IMAGING CAM " << i);
                    cam_[i]->startImaging();
                }
            }
            catch (const std::exception& e)
            {
                ROS_ERROR_STREAM("Error reconfiguring multi_camera node : " << e.what());
            }
        }
    }

    uint8_t MultiCamera::calculateColorIntensity(cv::Mat &img)
    {
        bool sumRGB = false;
        int colorId = 0;
        if (colorIntensityRGB_ == "R" || colorIntensityRGB_ == "r") colorId = 0;
        else if (colorIntensityRGB_ == "G" || colorIntensityRGB_ == "g") colorId = 1;
        else if (colorIntensityRGB_ == "B" || colorIntensityRGB_ == "b") colorId = 2;
        else sumRGB=true;

        int count = 0;
        int sum = 0;
        cv::Vec3b vecRGB;

        if (sumRGB)
        {
            for (int row = 0 ; row < img.rows; row +=  colorIntensityPxSteps_)
            {
                for (int col = 0 ; col < img.cols; col +=  colorIntensityPxSteps_)
                {
                    vecRGB = img.at<cv::Vec3b>(row,col);
                    sum += (vecRGB[0]+vecRGB[2]+vecRGB[1]);
                    count++;
                }
            }
            count *=3;
        }
        else
        {
            for (int row = 0 ; row < img.rows; row +=  colorIntensityPxSteps_)
            {
                for (int col = 0 ; col < img.cols; col +=  colorIntensityPxSteps_)
                {
                    sum += img.at<cv::Vec3b>(row,col)[colorId];
                    count++;
                }
            }
        }
        uint8_t colorIntensity = (uint8_t)(sum/count);
        return colorIntensity;
    }

};  // namespace avt_vimba_camera
