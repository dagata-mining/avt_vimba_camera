/// Created by pointlaz

#include <avt_vimba_camera/multi_camera.h>

#define DEBUG_PRINTS 1

namespace avt_vimba_camera
{
    MultiCamera::MultiCamera(ros::NodeHandle& nh, ros::NodeHandle& nhp)
            : nh_(nh), nhp_(nhp), it_(nhp)
    {
        // Start Vimba & list all available cameras
        api_.start();

        // Set the params

        nhp_.param("camera_qty", camQty_,1);
        guid_.resize(camQty_);
        pub_.resize(camQty_);
        camera_info_url_.resize(camQty_);
        frame_id_.resize(camQty_);
        cam_.resize(camQty_);
        name_.resize(camQty_);
        info_man_.resize(camQty_);

        for (int i = 0 ; i < camQty_; i++)
        {
            pub_[i] = it_.advertiseCamera("image_raw_"+std::to_string(i), 1);
            nhp_.param("guid_"+std::to_string(i), guid_[i], std::string(""));
            nhp_.param("camera_info_url_"+std::to_string(i), camera_info_url_[i], std::string(""));
            nhp_.param("frame_id_"+std::to_string(i), frame_id_[i], std::string(""));
            nhp_.param("name_"+std::to_string(i), name_[i], std::string(""));
            nhp_.param("print_all_features", print_all_features_, false);
            nhp_.param("use_measurement_time", use_measurement_time_, false);
            nhp_.param("ptp_offset", ptp_offset_, 0);


            std::shared_ptr<AvtVimbaCamera> cam = std::shared_ptr<AvtVimbaCamera>(new AvtVimbaCamera(frame_id_[i], i));
            cam_[i]=cam;
            cam->setCallback(std::bind(&avt_vimba_camera::MultiCamera::frameCallback, this, std::placeholders::_1, i));

            // Set camera info manager
            info_man_[i] = std::shared_ptr<camera_info_manager::CameraInfoManager>(
                    new camera_info_manager::CameraInfoManager(nhp_, name_[i], camera_info_url_[i]));
        }



        // Start dynamic_reconfigur & run configure()
        reconfigure_server_.setCallback(
                std::bind(&avt_vimba_camera::MultiCamera::configure, this, std::placeholders::_1, std::placeholders::_2));
    }

    MultiCamera::~MultiCamera(void)
    {
        for (int i = 0 ; i < cam_.size(); i++)
        {

            cam_[i]->stop();
            cam_[i].reset();
            pub_[i].shutdown();
        }


    }

    void MultiCamera::frameCallback(const FramePtr& vimba_frame_ptr, const int camId)
    {
        ROS_INFO("-------------FRAME %d", camId);
        ros::Time ros_time = ros::Time::now();
        if (pub_[camId].getNumSubscribers() > 0)
        {
            sensor_msgs::Image img;
            if (api_.frameToImage(vimba_frame_ptr, img))
            {
                sensor_msgs::CameraInfo ci = info_man_[camId]->getCameraInfo();
                // Note: getCameraInfo() doesn't fill in header frame_id or stamp
                ci.header.frame_id = frame_id_[camId];
                if (use_measurement_time_)
                {
                    VmbUint64_t frame_timestamp;
                    vimba_frame_ptr->GetTimestamp(frame_timestamp);
                    ci.header.stamp = ros::Time(cam_[camId]->getTimestampRealTime(frame_timestamp)) + ros::Duration(ptp_offset_, 0);
                }
                else
                {
                    ci.header.stamp = ros_time;
                }
                img.header.frame_id = ci.header.frame_id;
                img.header.stamp = ci.header.stamp;
                pub_[camId].publish(img, ci);
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
                    cam_[i]->start(ip_, guid_[i], frame_id_[i], print_all_features_);
                    //cam_[i]->stopImaging();
                    ROS_WARN_STREAM("-------------START");
                    Config config = newconfig;
                    cam_[i]->updateConfig(config);
                    updateCameraInfo(config,i);
                    ROS_WARN_STREAM("------------- CONFIGURED");
                    cam_[i]->startImaging();
                }

                else
                {
                    Config config = newconfig;
                    cam_[i]->updateConfig(config);
                    updateCameraInfo(config,i);
                    cam_[i]->start(ip_, guid_[i], frame_id_[i], print_all_features_);
                    ROS_WARN_STREAM("------------- CONFIGURED");
                }
            }
            catch (const std::exception& e)
            {
                ROS_ERROR_STREAM("Error reconfiguring multi_camera node : " << e.what());
            }
        }

    }

// See REP-104 for details regarding CameraInfo parameters
    void MultiCamera::updateCameraInfo(const avt_vimba_camera::AvtVimbaCameraConfig& config, const int camId)
    {
        sensor_msgs::CameraInfo ci = info_man_[camId]->getCameraInfo();

        // Set the operational parameters in CameraInfo (binning, ROI)
        int binning_or_decimation_x = std::max(config.binning_x, config.decimation_x);
        int binning_or_decimation_y = std::max(config.binning_y, config.decimation_y);

        // Set the operational parameters in CameraInfo (binning, ROI)
        int sensor_width = cam_[camId]->getSensorWidth();
        int sensor_height = cam_[camId]->getSensorHeight();

        if (sensor_width == -1 || sensor_height == -1)
        {
            ROS_ERROR("Could not determine sensor pixel dimensions, camera_info will be wrong");
        }

        ci.width = sensor_width;
        ci.height = sensor_height;
        ci.binning_x = binning_or_decimation_x;
        ci.binning_y = binning_or_decimation_y;

        // ROI is in unbinned coordinates, need to scale up
        ci.roi.width = config.width * binning_or_decimation_x;
        ci.roi.height = config.height * binning_or_decimation_y;
        ci.roi.x_offset = config.offset_x * binning_or_decimation_x;
        ci.roi.y_offset = config.offset_y * binning_or_decimation_y;

        bool roi_is_full_image = (ci.roi.width == ci.width && ci.roi.height == ci.height);
        ci.roi.do_rectify = !roi_is_full_image;

        // push the changes to manager
        info_man_[camId]->setCameraInfo(ci);
    }


};  // namespace avt_vimba_camera