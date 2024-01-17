//
// Created by alex on 9/28/23.
//

#ifndef ODOMETRY_SRC_JETRAW_JETRAW_H
#define ODOMETRY_SRC_JETRAW_JETRAW_H


#include <jetraw/jetraw.h>
#include <jetraw/dp_status.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dpcore/dpcore.h>

void initialization();

namespace jetrawCompress
{
    /**
     *
     * @param license licence
     * @param calibration calibration absolute path .dat
     * @return success
     */
    bool encode(const cv::Mat& input,const char*& dst, int32_t& dstLen);

    /**
     *
     * @param dataIn data buffer in
     * @param rows nbr of rows
     * @param cols nbr of cols
     * @param out Image in CV
     * @return
     */
    bool decode(const char* dataIn, int rows, int cols, cv::Mat& out, int32_t inLen);
};

#endif //ODOMETRY_SRC_JETRAW_JETRAW_H
