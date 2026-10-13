#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace hand_eye_diagnostics {

inline cv::Mat transform(const cv::Mat& rotation, const cv::Mat& translation) {
    cv::Mat h = cv::Mat::eye(4, 4, CV_64F);
    rotation.copyTo(h(cv::Rect(0, 0, 3, 3)));
    translation.copyTo(h(cv::Rect(3, 0, 1, 3)));
    return h;
}

inline bool rigid(const cv::Mat& h) {
    if (h.rows != 4 || h.cols != 4 || h.type() != CV_64F || !cv::checkRange(h))
        return false;
    const cv::Mat r = h(cv::Rect(0, 0, 3, 3));
    return cv::norm(r.t() * r - cv::Mat::eye(3, 3, CV_64F)) < 1e-6 &&
           std::abs(cv::determinant(r) - 1.0) < 1e-6 &&
           cv::norm(h.row(3) - (cv::Mat_<double>(1, 4) << 0, 0, 0, 1)) < 1e-6;
}

inline double angle(const cv::Mat& a, const cv::Mat& b) {
    cv::Mat delta = a.t() * b;
    return std::acos(std::clamp((cv::trace(delta)[0] - 1.0) / 2.0, -1.0, 1.0));
}

struct Report {
    double rotationSecondRmsRad = 0;
    double boardPositionRmsM = 0;
    double boardPositionMaxM = 0;
    double boardRotationMaxRad = 0;
    double methodTranslationDifferenceM = 0;
    double methodRotationDifferenceRad = 0;
    bool pass = false;
};

inline cv::Mat solve(const std::vector<cv::Mat>& rg, const std::vector<cv::Mat>& tg,
                     const std::vector<cv::Mat>& rc, const std::vector<cv::Mat>& tc,
                     cv::HandEyeCalibrationMethod method) {
    cv::Mat r, t;
    cv::calibrateHandEye(rg, tg, rc, tc, r, t, method);
    const cv::Mat h = transform(r, t);
    if (!rigid(h)) throw std::runtime_error("hand-eye solver returned a non-rigid transform");
    return h;
}

inline Report evaluate(const std::vector<cv::Mat>& rg, const std::vector<cv::Mat>& tg,
                       const std::vector<cv::Mat>& rc, const std::vector<cv::Mat>& tc,
                       const cv::Mat& x, const cv::Mat& other) {
    const size_t n = rg.size();
    if (n < 3 || tg.size() != n || rc.size() != n || tc.size() != n ||
        !rigid(x) || !rigid(other)) throw std::runtime_error("invalid diagnostic inputs");
    Report report;
    std::vector<cv::Mat> boards;
    cv::Mat mean = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat rotationVectors(static_cast<int>(n), 3, CV_64F);
    for (size_t i = 0; i < n; ++i) {
        // A fixed board should have the same base pose for every observation.
        boards.push_back(transform(rg[i], tg[i]) * x * transform(rc[i], tc[i]));
        mean += boards.back()(cv::Rect(3, 0, 1, 3));
        cv::Mat vector;
        cv::Rodrigues(rg[0].t() * rg[i], vector);
        for (int axis = 0; axis < 3; ++axis)
            rotationVectors.at<double>(static_cast<int>(i), axis) = vector.at<double>(axis);
    }
    mean /= static_cast<double>(n);
    cv::Mat singular;
    cv::SVD::compute(rotationVectors, singular);
    report.rotationSecondRmsRad = singular.at<double>(1) / std::sqrt(static_cast<double>(n));
    double squared = 0;
    for (size_t i = 0; i < n; ++i) {
        const double error = cv::norm(boards[i](cv::Rect(3, 0, 1, 3)) - mean);
        squared += error * error;
        report.boardPositionMaxM = std::max(report.boardPositionMaxM, error);
        for (size_t j = 0; j < i; ++j)
            report.boardRotationMaxRad = std::max(report.boardRotationMaxRad,
                angle(boards[i](cv::Rect(0, 0, 3, 3)), boards[j](cv::Rect(0, 0, 3, 3))));
    }
    report.boardPositionRmsM = std::sqrt(squared / static_cast<double>(n));
    report.methodTranslationDifferenceM = cv::norm(
        x(cv::Rect(3, 0, 1, 3)) - other(cv::Rect(3, 0, 1, 3)));
    report.methodRotationDifferenceRad = angle(
        x(cv::Rect(0, 0, 3, 3)), other(cv::Rect(0, 0, 3, 3)));
    // These are diagnostic rejection thresholds, not proof of physical accuracy.
    report.pass = n >= 15 && report.rotationSecondRmsRad >= 0.05 &&
        report.boardPositionRmsM <= 0.010 && report.boardPositionMaxM <= 0.020 &&
        report.boardRotationMaxRad <= 3.0 * CV_PI / 180.0 &&
        report.methodTranslationDifferenceM <= 0.020 &&
        report.methodRotationDifferenceRad <= 3.0 * CV_PI / 180.0;
    return report;
}

inline void writeReport(cv::FileStorage& fs, const Report& r) {
    fs << "rotation_second_rms_rad" << r.rotationSecondRmsRad;
    fs << "board_position_rms_m" << r.boardPositionRmsM;
    fs << "board_position_max_m" << r.boardPositionMaxM;
    fs << "board_rotation_max_rad" << r.boardRotationMaxRad;
    fs << "method_translation_difference_m" << r.methodTranslationDifferenceM;
    fs << "method_rotation_difference_rad" << r.methodRotationDifferenceRad;
    fs << "internal_consistency_pass" << static_cast<int>(r.pass);
    fs << "physical_validation_pass" << 0;
}

inline int selfTest() {
    auto pose = [](double x, double y, double z, double a, double b, double c) {
        cv::Mat rotation;
        cv::Rodrigues((cv::Mat_<double>(3, 1) << a, b, c), rotation);
        return transform(rotation, (cv::Mat_<double>(3, 1) << x, y, z));
    };
    const cv::Mat truth = pose(-0.10, 0.06, 0.18, 0.2, -0.1, 0.15);
    const cv::Mat board = pose(0.5, 0.1, 0.3, 0.1, 0.3, -0.2);
    std::vector<cv::Mat> rg, tg, rc, tc;
    for (int i = 0; i < 24; ++i) {
        const cv::Mat g = pose(0.3 + .08 * std::sin(i), .1 * std::cos(i),
                              .6 + .05 * std::sin(.7 * i), .5 * std::sin(i),
                              .4 * std::cos(.7 * i), .3 * std::sin(.4 * i));
        const cv::Mat c = truth.inv() * g.inv() * board;
        rg.push_back(g(cv::Rect(0, 0, 3, 3)).clone());
        tg.push_back(g(cv::Rect(3, 0, 1, 3)).clone());
        rc.push_back(c(cv::Rect(0, 0, 3, 3)).clone());
        tc.push_back(c(cv::Rect(3, 0, 1, 3)).clone());
    }
    const cv::Mat tsai = solve(rg, tg, rc, tc, cv::CALIB_HAND_EYE_TSAI);
    const cv::Mat park = solve(rg, tg, rc, tc, cv::CALIB_HAND_EYE_PARK);
    if (cv::norm(tsai - truth) > 1e-6 || !evaluate(rg, tg, rc, tc, tsai, park).pass)
        throw std::runtime_error("synthetic correct calibration test failed");
    auto badTc = tc;
    badTc[0] = tc[0] + (cv::Mat_<double>(3, 1) << .1, 0, 0);
    if (evaluate(rg, tg, rc, badTc, truth, truth).pass)
        throw std::runtime_error("outlier rejection test failed");
    auto oneAxis = rg;
    for (size_t i = 0; i < oneAxis.size(); ++i)
        oneAxis[i] = pose(0, 0, 0, .03 * i, 0, 0)(cv::Rect(0, 0, 3, 3)).clone();
    if (evaluate(oneAxis, tg, rc, tc, truth, truth).rotationSecondRmsRad >= .05)
        throw std::runtime_error("single-axis degeneracy test failed");
    cv::Mat invalid = truth.clone();
    invalid.at<double>(3, 3) = 0;
    if (rigid(invalid)) throw std::runtime_error("invalid transform test failed");
    std::cout << "[PASS] known transform, outlier, single-axis degeneracy, invalid matrix\n"
                 "[SAFE] offline tests only; no robot or camera connection\n";
    return 0;
}

}  // namespace hand_eye_diagnostics
