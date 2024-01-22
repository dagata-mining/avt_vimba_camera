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
    std::cerr << "[INFO] compressed from " << cols * rows << " to " << dstLen
              << " (" << static_cast<float>(cols * rows) / dstLen << "x times)"
              << std::endl;
    return true;
}

bool encode(unsigned char* buffer_ptr,int rows, int cols,unsigned char*& dst, int32_t& dstLen) {

    dstLen = cols * rows * 0.5;
    // Convert to 16-bit cv::Mat
    cv::Mat mat16bit;
    // Create a unique_ptr for the destination buffer
    std::shared_ptr<char[]> dstBuffer(new char[dstLen]);
    uint16_t* srcBuffer = reinterpret_cast<uint16_t*>(buffer_ptr);
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
    char* dstChar = croppedDstBuffer.release();
    dst = reinterpret_cast<unsigned char*>(dstChar);


    std::cerr << "[INFO] compressed from " << cols * rows << " to " << dstLen
              << " (" << static_cast<float>(cols * rows) / dstLen << "x times)"
              << std::endl;
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
    cv::Mat mat(rows, cols, CV_16U, imageBuffer.get());
    mat.convertTo(out, CV_8U);
    return true;
}
}