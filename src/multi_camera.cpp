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
        if (calculateColorIntensity_)
            colorPub_.resize(camQty_);

        for (int i = 0; i < camQty_; i++)
        {
            pub_[i].reset(new image_transport::CameraPublisher);
            *pub_[i] = it_.advertiseCamera(topicName + std::to_string(i), 1);
            if (calculateColorIntensity_)
            {
                ROS_INFO("-------------Color Intensity");
                colorPub_[i].reset(new ros::Publisher);
                *colorPub_[i] = nh_.advertise<std_msgs::UInt8>("/multi_camera/color_intensity_" + std::to_string(i), 1);
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
                    cam = std::shared_ptr<AvtVimbaCamera>(new AvtVimbaCamera(frame_id_[i], i ,api_,pub_[i],colorPub_[i]));
//                cam->setCallback(std::bind(&avt_vimba_camera::MultiCamera::compressCallback,
//                                           this,
//                                           std::placeholders::_1, std::placeholders::_2));
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
    }

    MultiCamera::~MultiCamera(void)
    {
        reconfigure_server_.clearCallback();
        cam_.clear();
        api_.reset();
        pub_.clear();
        colorPub_.clear();
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
                if (cam_[i]->isOpened())
                {
                    ROS_WARN_STREAM("-------------STOP IMAGING CAM " << i);
                    cam_[i]->stopImaging();
                }
                if (!cam_[i]->isOpened())
                {
                    ROS_WARN_STREAM("-------------START CAM " << i);
                    cam_[i]->start(ip_, guid_[i], frame_id_[i], print_all_features_);
                }

                ROS_WARN_STREAM("-------------UPDATE CONFIG CAM " << i);
                cam_[i]->updateConfig(newconfig);

                if (cam_[i]->connected_)
                {
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



};  // namespace avt_vimba_camera
