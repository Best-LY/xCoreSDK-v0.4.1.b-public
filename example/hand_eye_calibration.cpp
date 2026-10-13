/**
 * @file calibrate.cpp
 * @brief 机械臂手眼标定程序（眼在手上），支持双臂与并发标定，兼容RealSense相机。
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 */

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense2/rs.hpp>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "print_helper.hpp"

using namespace std;
using namespace rokae;
using namespace cv;

// ----- 参数定义 -----
constexpr double CHECKERBOARD_SQUARE_SIZE = 0.02;      // 棋盘格方块大小（米）
const cv::Size CHECKERBOARD_SIZE(7, 5);     // 棋盘格内角点数量
// ----- 数据结构 -----
struct CameraParams {
    cv::Mat cameraMatrix, distCoeffs;
    bool isCalibrated = false;
    CameraParams() {
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    }
};
struct CalibrationData {
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<cv::Mat> robotPoses;
};

// ----- 工具函数 -----
std::vector<cv::Point3f> createObjectPoints(const cv::Size& patternSize, float squareSize) {
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < patternSize.height; i++)
        for (int j = 0; j < patternSize.width; j++)
            objectPoints.emplace_back(j * squareSize, i * squareSize, 0);
    return objectPoints;
}

bool captureFromRealSense(cv::Mat& colorImage, cv::Mat& depthImage, const std::string& serial = "") {
    rs2::pipeline pipe;
    rs2::config cfg;
    bool started = false;
    try {
        if (!serial.empty()) cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        pipe.start(cfg);
        started = true;
        for (int i = 0; i < 10; i++)
            pipe.wait_for_frames();
        auto frames = pipe.wait_for_frames();
        auto color_frame = frames.get_color_frame();
        auto depth_frame = frames.get_depth_frame();
        if (!color_frame || !depth_frame) throw std::runtime_error("无法获取相机帧");
        colorImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP).clone();
        depthImage = cv::Mat(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP).clone();
        pipe.stop();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "RealSense获取图像异常: " << e.what() << std::endl;
        if (started) pipe.stop();
        return false;
    }
}

bool calibrateCamera(const std::vector<std::vector<cv::Point2f>>& imagePoints,
                     const std::vector<std::vector<cv::Point3f>>& objectPoints,
                     cv::Size imageSize, CameraParams& params) {
    if (imagePoints.size() < 3) {
        std::cerr << "至少需要3组图像进行相机标定" << std::endl;
        return false;
    }
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                     params.cameraMatrix, params.distCoeffs,
                                     rvecs, tvecs, cv::CALIB_RATIONAL_MODEL);
    std::cout << "相机标定 RMS 重投影误差: " << rms << std::endl;
    params.isCalibrated = (rms < 1.0);
    return params.isCalibrated;
}

void waitForRobotMovement(BaseRobot& robot) {
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        error_code ec;
        auto st = robot.operationState(ec);
        if (st == OperationState::idle || st == OperationState::unknown) break;
    }
}

cv::Mat robotPoseToMatrix(const CartesianPosition& pose) {
    Eigen::Matrix3d rot;
    Eigen::Vector3d euler(pose.rpy[2], pose.rpy[1], pose.rpy[0]);  // ZYX
    Utils::eulerToMatrix(euler, rot);
    cv::Mat R(3, 3, CV_64F), t(3, 1, CV_64F);
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) R.at<double>(i, j) = rot(i, j);
    t.at<double>(0, 0) = pose.trans[0]; t.at<double>(1, 0) = pose.trans[1]; t.at<double>(2, 0) = pose.trans[2];
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3))); t.copyTo(T(cv::Rect(3, 0, 1, 3)));
    return T;
}

bool performHandEyeCalibration(const CalibrationData& data, bool eyeInHand, cv::Mat& result) {
    if (data.robotPoses.size() < 3 || data.imagePoints.size() < 3) {
        std::cerr << "至少需要3组数据进行手眼标定" << std::endl;
        return false;
    }
    std::vector<cv::Mat> R_gripper2base, t_gripper2base, R_target2cam, t_target2cam;
    CameraParams cameraParams;
    if (!calibrateCamera(data.imagePoints, data.objectPoints, cv::Size(640, 480), cameraParams)) {
        std::cerr << "相机标定失败，无法进行手眼标定" << std::endl;
        return false;
    }
    for (size_t i = 0; i < data.imagePoints.size(); i++) {
        cv::Mat rvec, tvec;
        if (!cv::solvePnP(data.objectPoints[i], data.imagePoints[i],
                          cameraParams.cameraMatrix, cameraParams.distCoeffs,
                          rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
            std::cerr << "第 " << i << " 组数据PnP求解失败" << std::endl;
            continue;
        }
        cv::Mat R_tc;
        cv::Rodrigues(rvec, R_tc);
        R_target2cam.push_back(R_tc);
        t_target2cam.push_back(tvec);
        cv::Mat robotPose = data.robotPoses[i];
        R_gripper2base.push_back(robotPose(cv::Rect(0, 0, 3, 3)));
        t_gripper2base.push_back(robotPose(cv::Rect(3, 0, 1, 3)));
    }
    if (R_gripper2base.size() < 3) {
        std::cerr << "有效数据不足，无法进行手眼标定" << std::endl;
        return false;
    }
    cv::Mat R_cam2gripper, t_cam2gripper;
    cv::calibrateHandEye(R_gripper2base, t_gripper2base,
                         R_target2cam, t_target2cam,
                         R_cam2gripper, t_cam2gripper,
                         cv::CALIB_HAND_EYE_TSAI);
    result = cv::Mat::eye(4, 4, CV_64F);
    R_cam2gripper.copyTo(result(cv::Rect(0, 0, 3, 3)));
    t_cam2gripper.copyTo(result(cv::Rect(3, 0, 1, 3)));
    return true;
}

void saveCalibrationResult(const cv::Mat& result, const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) throw std::runtime_error("无法打开文件: " + filename);
    fs << "hand_eye_calibration" << result;
    fs.release();
    std::cout << "标定结果已保存至: " << filename << std::endl;
}

std::vector<CartesianPosition> generateCalibrationPoints(const CartesianPosition& center, int numPoints) {
    std::vector<CartesianPosition> points;
    double radius = 0.05;
    for (int i = 0; i < numPoints; i++) {
        double angle = 2.0 * M_PI * i / numPoints;
        CartesianPosition pose = center;
        pose.rpy[0] += 0.3 * cos(angle);
        pose.rpy[1] += 0.3 * sin(angle);
        pose.trans[0] += 0.02 * cos(angle);
        pose.trans[1] += 0.02 * sin(angle);
        points.push_back(pose);
    }
    return points;
}

// 采集单个机械臂标定数据（可多线程并行调用）
void collectCalibrationData(xMateRobot& robot, const std::vector<CartesianPosition>& calibPoints,
                            const std::vector<cv::Point3f>& objectPoints, CalibrationData& calibData, const std::string& winName) {
    error_code ec;
    for (size_t i = 0; i < calibPoints.size(); ++i) {
        std::cout << "移动到标定点 " << (i+1) << "/" << calibPoints.size() << std::endl;
        robot.executeCommand({MoveLCommand(calibPoints[i])}, ec);
        waitForRobotMovement(robot);
        CartesianPosition currPose = robot.cartPosture(CoordinateType::flangeInBase, ec);
        cv::Mat robotPoseMat = robotPoseToMatrix(currPose);
        std::cout << "当前位姿: " << currPose << std::endl;
        std::cout << "请确保标定板在相机视野内，按回车采集图像..." << std::endl;
        std::cin.get();
        cv::Mat colorImage, depthImage;
        if (!captureFromRealSense(colorImage, depthImage)) {
            std::cerr << "相机采集失败，跳过此点" << std::endl;
            continue;
        }
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(colorImage, CHECKERBOARD_SIZE, corners);
        if (found) {
            cv::Mat gray;
            cv::cvtColor(colorImage, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            cv::drawChessboardCorners(colorImage, CHECKERBOARD_SIZE, corners, found);
            cv::imshow(winName, colorImage);
            cv::waitKey(500);
            calibData.imagePoints.push_back(corners);
            calibData.objectPoints.push_back(objectPoints);
            calibData.robotPoses.push_back(robotPoseMat);
            std::cout << "采集成功" << std::endl;
        } else {
            std::cerr << "棋盘格未检测到，跳过此点" << std::endl;
        }
    }
    cv::destroyWindow(winName);
}

// 支持串行或并行双臂手眼标定
void dualRobotHandEyeCalibration(const std::string& ip1, const std::string& ip2, bool parallel = false) {
    error_code ec;
    xMateRobot robot1(ip1), robot2(ip2);
    robot1.setOperateMode(OperateMode::automatic, ec); robot1.setPowerState(true, ec); robot1.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot2.setOperateMode(OperateMode::automatic, ec); robot2.setPowerState(true, ec); robot2.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot1.setDefaultSpeed(50, ec); robot1.setDefaultZone(0, ec);
    robot2.setDefaultSpeed(50, ec); robot2.setDefaultZone(0, ec);

    CalibrationData calibData1, calibData2;
    auto objectPoints = createObjectPoints(CHECKERBOARD_SIZE, CHECKERBOARD_SQUARE_SIZE);
    CartesianPosition home({0.4, 0, 0.5, M_PI, 0, 0});
    int numPoints = 10;
    auto calibPoints1 = generateCalibrationPoints(home, numPoints);
    auto calibPoints2 = generateCalibrationPoints(home, numPoints);

    // 标定数据采集
    if (parallel) {
        std::thread t1(collectCalibrationData, std::ref(robot1), std::cref(calibPoints1), std::cref(objectPoints), std::ref(calibData1), "Robot1");
        std::thread t2(collectCalibrationData, std::ref(robot2), std::cref(calibPoints2), std::cref(objectPoints), std::ref(calibData2), "Robot2");
        t1.join(); t2.join();
    } else {
        collectCalibrationData(robot1, calibPoints1, objectPoints, calibData1, "Robot1");
        collectCalibrationData(robot2, calibPoints2, objectPoints, calibData2, "Robot2");
    }

    // 手眼标定与保存
    cv::Mat handEyeMat1, handEyeMat2;
    bool succ1 = performHandEyeCalibration(calibData1, true, handEyeMat1);
    if (succ1) saveCalibrationResult(handEyeMat1, "robot1_hand_eye_calibration.xml");
    bool succ2 = performHandEyeCalibration(calibData2, true, handEyeMat2);
    if (succ2) saveCalibrationResult(handEyeMat2, "robot2_hand_eye_calibration.xml");

    robot1.setPowerState(false, ec); robot2.setPowerState(false, ec);
    robot1.disconnectFromRobot(ec);  robot2.disconnectFromRobot(ec);
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--preflight") {
        try {
            rs2::context context;
            auto devices = context.query_devices();
            std::cout << "检测到 RealSense 相机: " << devices.size() << std::endl;
            if (devices.size() == 0) return 1;
            for (const auto& device : devices) {
                const std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                std::cout << "  " << device.get_info(RS2_CAMERA_INFO_NAME)
                          << " (序列号: " << serial << ")" << std::endl;
                cv::Mat colorImage, depthImage;
                if (!captureFromRealSense(colorImage, depthImage, serial)) return 1;
                std::cout << "    图像采集成功: 彩色 " << colorImage.cols << "x" << colorImage.rows
                          << ", 深度 " << depthImage.cols << "x" << depthImage.rows << std::endl;
            }
            return 0;
        } catch (const std::exception& e) {
            std::cerr << "预检失败: " << e.what() << std::endl;
            return 1;
        }
    }
    std::string robot1_ip = "192.168.2.160";
    std::string robot2_ip = "192.168.2.161";
    bool parallel = false;
    if (argc > 2) {
        robot1_ip = argv[1]; robot2_ip = argv[2];
        if (argc > 3 && std::string(argv[3]) == "parallel") parallel = true;
    } else if (argc > 1) {
        robot1_ip = argv[1];
    }
    std::cout << "机器人1 IP: " << robot1_ip << "\n机器人2 IP: " << robot2_ip << (parallel ? " [并行标定]" : " [串行标定]") << std::endl;
    dualRobotHandEyeCalibration(robot1_ip, robot2_ip, parallel);
    return 0;
}
