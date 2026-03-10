#include "tagc_enhancement.h"
#include <cmath>
#include <omp.h>

IM_TAGC_Enhancement::IM_TAGC_Enhancement() {
    initLUTCache();
}

void IM_TAGC_Enhancement::initLUTCache(int precision) {
    lut_cache.resize(precision + 1);
    exp_values.resize(precision + 1);
    x_lut = cv::Mat(1, 256, CV_32FC1);
    float* x_ptr = x_lut.ptr<float>();
    for (int i = 0; i < 256; i++) {
        x_ptr[i] = i / 255.0f;
    }
    #pragma omp parallel for
    for (int i = 0; i <= precision; i++) {
        float exp_val = 0.3f + (i / (float)precision) * 2.0f;
        exp_values[i] = exp_val;
        cv::Mat lut(1, 256, CV_8U);
        uchar* lut_ptr = lut.ptr<uchar>();
        for (int j = 0; j < 256; j++) {
            float val = std::pow(x_ptr[j], exp_val) * 255.0f + 0.5f;
            lut_ptr[j] = cv::saturate_cast<uchar>(val);
        }
        lut_cache[i] = lut;
    }
}

cv::Mat IM_TAGC_Enhancement::makeMapping(const cv::Mat& hist, float cut_limit, float contrast) {
    int level = 256;
    const float* h_ptr = hist.ptr<float>();
    float cum_hist[256];
    cum_hist[0] = h_ptr[0];
    for (int i = 1; i < level; i++) {
        cum_hist[i] = cum_hist[i-1] + h_ptr[i];
    }
    float total = cum_hist[255];
    float cut_amount = total * cut_limit;
    int min_b = 0, max_b = level - 1;
    for (int i = 0; i < level; i++) {
        if (cum_hist[i] >= cut_amount) {
            min_b = i;
            break;
        }
    }
    for (int i = level - 1; i >= 0; i--) {
        if (cum_hist[i] <= total - cut_amount) {
            max_b = i;
            break;
        }
    }
    int min_gray = 0, max_gray = 255;
    for (int i = 0; i < level; i++) {
        if (h_ptr[i] > 0) {
            min_gray = i;
            break;
        }
    }
    for (int i = level - 1; i >= 0; i--) {
        if (h_ptr[i] > 0) {
            max_gray = i;
            break;
        }
    }
    float delta = (max_gray - min_gray) * contrast * 0.5f;
    min_gray = std::max(min_gray - delta, 0.0f);
    max_gray = std::min(max_gray + delta, 255.0f);
    cv::Mat map_table(1, level, CV_8U);
    uchar* map_ptr = map_table.ptr<uchar>();
    if (max_b > min_b) {
        float slope = (max_gray - min_gray) / (max_b - min_b);
        for (int i = 0; i < min_b; i++) {
            map_ptr[i] = cv::saturate_cast<uchar>(min_gray);
        }
        for (int i = min_b; i <= max_b; i++) {
            map_ptr[i] = cv::saturate_cast<uchar>((i - min_b) * slope + min_gray + 0.5f);
        }
        for (int i = max_b + 1; i < level; i++) {
            map_ptr[i] = cv::saturate_cast<uchar>(max_gray);
        }
    } else {
        for (int i = 0; i < level; i++) {
            map_ptr[i] = cv::saturate_cast<uchar>(max_b);
        }
    }
    return map_table;
}

cv::Mat IM_TAGC_Enhancement::process(const cv::Mat& src_rgb, double* proc_time) {
    auto start = std::chrono::high_resolution_clock::now();
    CV_Assert(src_rgb.type() == CV_8UC3);
    int rows = src_rgb.rows;
    int cols = src_rgb.cols;
    std::vector<cv::Mat> rgb_channels(3);
    cv::split(src_rgb, rgb_channels);
    cv::Mat R = rgb_channels[0];
    cv::Mat G = rgb_channels[1];
    cv::Mat B = rgb_channels[2];
    cv::Mat Rf, Gf, Bf;
    R.convertTo(Rf, CV_32F, 1.0/255.0);
    G.convertTo(Gf, CV_32F, 1.0/255.0);
    B.convertTo(Bf, CV_32F, 1.0/255.0);
    cv::Mat L = 0.2126f * Rf + 0.7152f * Gf + 0.0722f * Bf;
    cv::Mat A = (Rf + Gf + Bf) / 3.0f;
    cv::Mat Gamma = 5.0f + (0.5f - L).mul(1.0f - A) - 2.0f * L;
    cv::Mat expVal = 2.0f / Gamma;
    cv::max(expVal, 0.3f, expVal);
    cv::Mat expIdx;
    expVal.convertTo(expIdx, CV_32S, 100.0f);
    cv::Mat result_rgb(rows, cols, CV_8UC3);
    std::vector<cv::Mat> result_channels(3);
    cv::split(result_rgb, result_channels);
    #pragma omp parallel for collapse(2)
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            int idx = expIdx.at<int>(i, j);
            idx = std::max(0, std::min(idx, (int)lut_cache.size() - 1));
            const uchar* lut = lut_cache[idx].ptr<uchar>();
            result_channels[0].at<uchar>(i, j) = lut[R.at<uchar>(i, j)];
            result_channels[1].at<uchar>(i, j) = lut[G.at<uchar>(i, j)];
            result_channels[2].at<uchar>(i, j) = lut[B.at<uchar>(i, j)];
        }
    }
    cv::merge(result_channels, result_rgb);
    cv::Mat gray;
    cv::cvtColor(result_rgb, gray, cv::COLOR_RGB2GRAY);
    cv::Mat hist;
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    cv::calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
    cv::Mat map_table = makeMapping(hist);
    cv::Mat img_out_rgb;
    cv::LUT(result_rgb, map_table, img_out_rgb);
    if (proc_time) {
        auto end = std::chrono::high_resolution_clock::now();
        *proc_time = std::chrono::duration<double, std::milli>(end - start).count();
    }
    return img_out_rgb;
}
