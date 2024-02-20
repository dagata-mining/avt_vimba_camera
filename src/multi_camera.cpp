/// Created by pointlaz

#include <avt_vimba_camera/multi_camera.h>

#define DEBUG_PRINTS 1

namespace avt_vimba_camera
{
    MultiCamera::MultiCamera(ros::NodeHandle& nh, ros::NodeHandle& nhp)
            : nh_(nh), nhp_(nhp), it_(nhp)
    {
        api_.reset(new AvtVimbaApi);
        api_->start();
//
//        // Set the params

        nhp_.param("camera_qty", camQty_, 1);
        nhp_.param("compress_jetraw_vimba", compressJetraw_, true);
        nhp_.param("compress_jpeg_vimba", compressJPG_, false);
        std::string topicName = "image_raw_";
        nhp_.param("compress_jpeg_quality", qualityJPG_, 90);
        // Get Pixel Intensity params
        nhp_.param("calculate_pixel_intensity", calculate_pixel_intensity_,true);
        if (compressJPG_ && compressJetraw_) compressJPG_ = false;

        guid_.resize(camQty_);
        pub_.resize(camQty_);
        camera_info_url_.resize(camQty_);
        frame_id_.resize(camQty_);
        cam_.resize(camQty_);
        name_.resize(camQty_);

        if(calculate_pixel_intensity_)
        {
            int pixel_intensity_pixel_steps, pixel_intensity_saturation_value;
            double pixel_intensity_saturated_threshold;
            bool pixel_intensity_echo;
            nhp_.param("pixel_intensity_pixel_steps", pixel_intensity_pixel_steps,10);
            nhp_.param("pixel_intensity_saturated_threshold", pixel_intensity_saturated_threshold,0.1);
            nhp_.param("pixel_intensity_saturation_value", pixel_intensity_saturation_value,255);
            nhp_.param("pixel_intensity_echo", pixel_intensity_echo,false);
            api_->setPixelIntensityParameters(pixel_intensity_pixel_steps, pixel_intensity_saturated_threshold, pixel_intensity_saturation_value, pixel_intensity_echo);
            pixel_intensity_pub_.resize(camQty_);
        }

        if (compressJetraw_)
        {
            auto res = dpcore_init();
            ROS_INFO_STREAM("JETRAW------------ INIT STATUS " << std::to_string(res));
            if (res <= 1)
            {
                api_->activateJetraw();
                ROS_INFO("JETRAW------------ACTIVATED");
            }
            else
            {
                ROS_WARN("JETRAW------------INIT FAILED FALL BACK TO JPG");
                compressJPG_ = true;
                compressJetraw_ = false;
            }
        }
        if (compressJPG_) {
            api_->compressJPG_ = true;
            api_->qualityJPG_ = qualityJPG_;
        }



        for (int i = 0; i < camQty_; i++)
        {
            pub_[i].reset(new image_transport::CameraPublisher);
            *pub_[i] = it_.advertiseCamera(topicName + std::to_string(i), 1);

            nhp_.param("guid_" + std::to_string(i), guid_[i], std::string(""));
            nhp_.param("camera_info_url_" + std::to_string(i), camera_info_url_[i], std::string(""));
            nhp_.param("frame_id_" + std::to_string(i), frame_id_[i], std::string(""));
            nhp_.param("name_" + std::to_string(i), name_[i], std::string(""));
            nhp_.param("print_all_features", print_all_features_, false);
            nhp_.param("use_measurement_time", use_measurement_time_, false);
            nhp_.param("ptp_offset", ptp_offset_, 0);

            ROS_INFO("-------------New Cam");
            std::shared_ptr<AvtVimbaCamera>
                cam = std::shared_ptr<AvtVimbaCamera>(new AvtVimbaCamera(frame_id_[i], i, api_, pub_[i]));
            if (calculate_pixel_intensity_)
            {
                ROS_INFO("-------------Color Intensity");
                pixel_intensity_pub_[i].reset(new ros::Publisher);
                *pixel_intensity_pub_[i] = nh_.advertise<std_msgs::UInt8>("/multi_camera/pixel_intensity_" + std::to_string(i), 1);
                cam->setPixelIntensityPublisher(pixel_intensity_pub_[i]);
            }
            cam_[i] = cam;
        }
            //Testing JetRaw

//        std::string inFile = "/home/alex/third_party/Scanner/RosScan/Projects/avt_vimba_camera/test/raw_5.tiff";
//        cv::Mat inputImage = cv::imread(inFile, cv::IMREAD_UNCHANGED);
//        if (inputImage.empty()) {
//            ROS_ERROR("JETRAW-------------IMAGE_EMPTY");
//        }
//        // encode inputImage into encodedData
//        int32_t dstLen = 0;
//        const char* encodedData = nullptr;
//        if (!jetrawCompress::encode(inputImage, encodedData, dstLen)) {
//            ROS_ERROR("JETRAW-------------ENCODING FAILED");
//        }
//        // decode buffer encodedData and save it in cv::Mat decodedImage
//        cv::Mat decodedImage;
//        if (!jetrawCompress::decode(encodedData, inputImage.rows, inputImage.cols, decodedImage,
//                                    dstLen)) {
//            ROS_ERROR("JETRAW-------------DECODING FAILED");
//        }
////         write in disk decompressed tiff image
//        std::string outFile = "/home/alex/third_party/Scanner/RosScan/Projects/avt_vimba_camera/test/decompress.tiff";
//        cv::imwrite(outFile, decodedImage);
//        ROS_INFO("JETRAW-------------COMPLETED");


        ROS_INFO("-------------Reconfig");
        reconfigure_server_.setCallback(
            std::bind(&avt_vimba_camera::MultiCamera::configure,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));
        reconfigure_server_.clearCallback();
    }

    MultiCamera::~MultiCamera()
    {
        cam_.clear();
        if(api_)api_.reset();
        pub_.clear();
        pixel_intensity_pub_.clear();
        reconfigure_server_.clearCallback();
        std::cout<< "multi clean finish" << std::endl;
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

            if (cam_[i]->isOpened())
            {
                ROS_WARN_STREAM("-------------UPDATE CONFIG CAM " << i);
                cam_[i]->updateConfig(newconfig);
            }
            if (cam_[i]->connected_)
            {
                ROS_WARN_STREAM("-------------START IMAGING CAM " << i);
                cam_[i]->startImaging();
            }

        }
    }



}  // namespace avt_vimba_camera
