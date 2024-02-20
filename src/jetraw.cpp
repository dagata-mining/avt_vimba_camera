//
// Created by alex on 9/28/23.
//

#include "jetraw.h"
#include <iostream>

void initialization() {
    int res = dpcore_init();
    // Handle the result or perform other initialization tasks
}

namespace jetrawCompress
{


bool encode(const cv::Mat& input,const char*& dst, int32_t& dstLen) {

    int rows = input.rows;
    int cols = input.cols;
    dstLen = cols * rows * 0.5;
    // Convert to 16-bit cv::Mat
    cv::Mat mat16bit;
    input.convertTo(mat16bit, CV_16U);
    // Create a unique_ptr for the destination buffer
    std::shared_ptr<char[]> dstBuffer(new char[dstLen]);
    uint16_t* srcBuffer = mat16bit.ptr<uint16_t>();
    // Embed dpcore header
    if (int status = dpcore_embed_meta(srcBuffer, cols * rows, "002KK-8bit") !=
        dp_success) {
        std::cerr << "[ERROR] error while embedding data: " << status
                  << std::endl;
        return false;
    }
    // Encode input buffer
    if (int status = jetraw_encode(srcBuffer, cols, rows, dstBuffer.get(),
                                   &dstLen) != dp_success) {
        std::cerr << "[ERROR] error while encoding data: " << status
                  << std::endl;
        return false;
    }
    std::unique_ptr<char[]> croppedDstBuffer(new char[dstLen]);
    std::memcpy(croppedDstBuffer.get(), dstBuffer.get(), dstLen);
    dst = croppedDstBuffer.release();
    return true;
}

bool encodeMsg(unsigned char* buffer_ptr, int rows, int cols,sensor_msgs::Image& image, int32_t& dstLen) {

    dstLen = cols * rows;
    // Create a unique_ptr for the destination buffer
    size_t pixels = cols * rows;
    std::unique_ptr<char[]> dstBuffer(new char[dstLen]);
    std::unique_ptr<uint16_t[]> srcBuffer(new uint16_t[pixels]);

//    uint16_t* srcBuffer = reinterpret_cast<uint16_t*>(buffer_ptr);
    for (size_t i = 0; i < pixels; ++i) {
        // Copy the 8-bit value into the 16-bit value, effectively padding
        // the upper 8 bits with zeros.
        srcBuffer[i] = static_cast<uint16_t>(buffer_ptr[i]);
    }


    // Embed dpcore header
    if (int status = dpcore_embed_meta(srcBuffer.get(), cols * rows, "002KK-8bit") !=
        dp_success) {
        std::cerr << "[ERROR] error while embedding data: " << status
                  << std::endl;
        return false;
    }
    // Encode input buffer
    if (int status = jetraw_encode(srcBuffer.get(), cols, rows, dstBuffer.get(),
                                   &dstLen) != dp_success) {
        std::cerr << "[ERROR] error while encoding data: " << status
                  << std::endl;
        return false;
    }

    image.encoding = "jetraw";
    image.height   = rows;
    image.width    = cols;
    image.step     = dstLen;
    image.data.resize(dstLen);

    std::memcpy(&image.data[0], dstBuffer.get(), dstLen);

    return true;
}

bool decode(const char* dataIn, int rows, int cols, cv::Mat& out, int32_t inLen) {
    // Create a unique_ptr for the image buffer
    std::unique_ptr<uint16_t[]> imageBuffer(new uint16_t[rows * cols]);
    if (int status = jetraw_decode(dataIn, inLen, imageBuffer.get(),
                                   rows * cols) != dp_success) {
        std::cerr << "[ERROR] error while decoding data: " << status
                  << std::endl;
        return false;
    }
    cv::Mat mat(rows, cols, CV_16U, imageBuffer.get());//std::move
    mat.convertTo(out, CV_8U);
    return true;
}

bool decodeMsg(const sensor_msgs::Image& src, sensor_msgs::Image& dst) {
    // Create a unique_ptr for the image buffer
    int32_t rows = src.height;
    int32_t cols = src.width;
    int32_t srcLen = src.step;
    std::unique_ptr<uint16_t[]> imageBuffer(new uint16_t[rows * cols]);
    const char *srcBuffer = reinterpret_cast<const char*>(src.data.data());

    if (int status = jetraw_decode(srcBuffer, srcLen, imageBuffer.get(),
                                   rows * cols) != dp_success) {
        std::cerr << "[ERROR] error while decoding data: " << status
                  << std::endl;
        return false;
    }

    cv::Mat mat(rows, cols, CV_16U, imageBuffer.get());//std::move
    mat.convertTo(mat, CV_8U);
    cv::cvtColor(mat,mat,CV_BayerRG2RGB);

    dst.encoding = sensor_msgs::image_encodings::BGR8;
    dst.height   = rows;
    dst.width    = cols;
    dst.step     = mat.step;
    size_t st0 = (mat.step * rows);
    dst.data.resize(st0);
    memcpy(&dst.data[0],  mat.data, st0);
    dst.is_bigendian = 0;
    return true;
}

}