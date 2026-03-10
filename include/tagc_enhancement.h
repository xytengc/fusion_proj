#ifndef TAGC_ENHANCEMENT_H
#define TAGC_ENHANCEMENT_H


#include <opencv2/opencv.hpp>
#include <vector>
#include "config.h"

class IM_TAGC_Enhancement {
public:
    IM_TAGC_Enhancement();
    cv::Mat process(const cv::Mat& src_rgb, double* proc_time = nullptr);
private:
    void initLUTCache(int precision = 200);
    cv::Mat makeMapping(const cv::Mat& hist, float cut_limit = 0.03f, float contrast = 1.0f);
    std::vector<cv::Mat> lut_cache;
    std::vector<float> exp_values;
    cv::Mat x_lut;
};

#endif // TAGC_ENHANCEMENT_H
