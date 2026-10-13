#define XMATEMODEL_LIB_SUPPORTED

// 基础库
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <numeric>
#include <limits>
#include <algorithm>
#include <thread>
#include <chrono>
#include <deque>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

// RealSense
#include <librealsense2/rs.hpp>

// Rokae SDK
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "rokae/data_types.h"
#include "rokae/model.h"
#include "rokae/planner.h"
#include "rokae/exception.h"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;
using namespace rokae;

/* ================= 用户配置区域 (参数已更新) ================= */

// 🤖 机器人自由度 (6 轴 xMate)
const unsigned short ROBOT_DOF = 6;

// 🤖 机械臂 IP (请根据您的配置修改)
const string IP_LEFT = "192.168.2.161";
const string IP_RIGHT = "192.168.2.160";

// 💻 本机 IP
const string LOCAL_IP = "192.168.2.6";

// 📷 相机序列号 (请根据您的配置修改)
const string SN_LEFT_CAM = "236422072262";
const string SN_RIGHT_CAM = "236422070786";

// 🏁 标定板参数
const Size BOARD_SIZE(8, 6);
const float SQUARE_SIZE = 0.020f; // 20mm

// 📐 跟随参数 (已修正: 距离 40cm, 速度 0.2)
const double Z_APPROACH_OFFSET = 0.40; // 目标上方 40cm (Z轴距离保持目标)
const double FOLLOW_SPEED_FACTOR = 0.2; // 速度系数 [0.0 - 1.0]

// 📈 滤波参数 (低通滤波/指数平滑系数)
const double FILTER_ALPHA = 0.2; 

// ⚙️ 趋势校验参数 
const int HISTORY_FRAMES = 3; 
const double MIN_MOVEMENT_THRESHOLD = 5.0; // 像素阈值

// 🏠 左右臂安全初始位姿 (独立配置)
const std::array<double, ROBOT_DOF> Q_SAFE_LEFT = {0, Utils::degToRad(15), Utils::degToRad(45), 0, Utils::degToRad(90), 0};
const std::array<double, ROBOT_DOF> Q_SAFE_RIGHT = {0, Utils::degToRad(-15), Utils::degToRad(-45), 0, Utils::degToRad(-90), 0};


/* ================= 辅助函数 (保持不变) ================= */

Mat rpyToRotationMatrix(const array<double, 3>& rpy) {
    Eigen::Matrix3d rot;
    Eigen::Vector3d euler(rpy[2], rpy[1], rpy[0]);
    Utils::eulerToMatrix(euler, rot);
    Mat R(3, 3, CV_64F);
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            R.at<double>(i,j) = rot(i,j);
    return R;
}

Mat getIntrinsics(const rs2::video_stream_profile& profile, Mat& distCoeffs) {
    auto intrinsics = profile.get_intrinsics();
    Mat K = Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = intrinsics.fx;
    K.at<double>(1, 1) = intrinsics.fy;
    K.at<double>(0, 2) = intrinsics.ppx;
    K.at<double>(1, 2) = intrinsics.ppy;
    distCoeffs = Mat::zeros(5, 1, CV_64F);
    for(int i=0; i<5; i++) distCoeffs.at<double>(i) = intrinsics.coeffs[i];
    return K;
}

Mat loadCalibrationMatrix(const string& filename) {
    Mat H;
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "❌ 无法打开标定文件: " << filename << " (请确认文件在当前目录下)" << endl;
        throw runtime_error("Calibration file not found.");
    }
    fs["H_Flange2Cam"] >> H;
    fs.release();
    if (H.empty() || H.rows != 4 || H.cols != 4) throw runtime_error("Invalid or empty matrix in file.");
    cout << "✅ 加载标定矩阵: " << filename << endl;
    return H;
}

Eigen::Isometry3d MatToEigen(const Mat& H) {
    Eigen::Matrix4d E;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            E(i, j) = H.at<double>(i, j);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.matrix() = E;
    return T;
}

void waitForIdle(xMateRobot& robot, const string& armName, error_code& ec) {
    cout << "[" << armName << "] 正在等待机械臂静止 (Idle, max 5s)..." << endl;
    auto start_time = chrono::steady_clock::now();
    const chrono::seconds timeout(5);

    do {
        this_thread::sleep_for(chrono::milliseconds(100));
        auto state = robot.operationState(ec);
        if (ec) {
            cerr << "❌ 警告：获取机器人状态失败: " << ec.message() << endl;
            return;
        }
        if (state == OperationState::idle) {
            cout << "✅ 机械臂已静止 (Idle)." << endl;
            return;
        }
        if (chrono::steady_clock::now() - start_time > timeout) {
            cerr << "❌ 警告：等待机械臂静止超时 (5s)，当前状态: " << (int)state << "。强制继续清理/退出。" << endl;
            return;
        }
    } while (true);
}

void MoveJToSafe(xMateRobot& robot, const std::array<double, ROBOT_DOF>& q_safe, const string& armName, error_code& ec) {
    auto rtCon = robot.getRtMotionController().lock();
    if (!rtCon) {
        throw runtime_error("获取实时运动控制器失败！无法执行 MoveJ。");
    }

    cout << "[" << armName << "] 正在 MoveJ 到安全位姿..." << endl;

    rtCon->MoveJ(0.5, robot.jointPos(ec), q_safe);
    if(ec) {
        throw runtime_error("MoveJ 到安全位姿失败: " + ec.message());
    }
    cout << "✅ MoveJ 运动命令已发出。" << endl;
}

/**
 * @brief 核心安全清理流程 (强制清理相机，模式复位)
 */
void safeCleanup(xMateRobot& robot, rs2::pipeline& pipe, bool is_following, rokae::FollowPosition<ROBOT_DOF>* follower_ptr, const string& armName) {
    error_code ec;

    cout << "\n\n=== 执行安全清理和模式恢复 (模式复位) ===" << endl;

    // 1. 强制停止跟随
    if (follower_ptr && is_following) {
        cout << "⚠️ 停止 FollowPosition 任务..." << endl;
        follower_ptr->stop();
    }

    // 2. 等待机械臂静止
    waitForIdle(robot, armName, ec);

    // 3. 尝试切换回非实时模式 (关闭 RCI / RtCommand)
    cout << "✅ 退出实时控制模式 (关闭 RCI/RtCommand)..." << endl;
    robot.setMotionControlMode((MotionControlMode)0, ec);
    if(ec) { cerr << "❌ 警告：退出实时控制模式失败: " << ec.message() << endl; }

    // 4. 强制下电
    cout << "执行下电操作..." << endl;
    robot.setPowerState(false, ec);
    if(ec) { cerr << "❌ 警告：强制下电失败: " << ec.message() << endl; }

    // 5. 切换到手动模式 (!!! 模式恢复 !!!)
    cout << "✅ 模式恢复：切换到手动模式..." << endl;
    robot.setOperateMode(OperateMode::manual, ec);
    if(ec) { cerr << "❌ 警告：切换到手动模式失败: " << ec.message() << endl; }

    // 6. 强制清理相机和窗口 (即使在清理模式下未启动，也要尝试执行)
    cout << "✅ 强制清理 RealSense pipe 和 OpenCV 窗口..." << endl;
    try {
        pipe.stop(); // 尝试停止 pipe，如果未启动可能会抛出异常，但我们捕获它
    } catch (const rs2::error& e) {
        // 如果 pipe 未启动，这里可能会捕获到 'device not opened' 错误，忽略它
        // cerr << "⚠️ 停止 RealSense pipe 时捕获异常: " << e.what() << endl; 
    } catch (...) {
        // 捕获其他异常
    }
    destroyAllWindows();

    cout << "✅ 安全清理完成，程序即将退出。" << endl;
}

/**
 * @brief 仅执行模式清理 (新增函数)
 */
// void runCleanupOnly(xMateRobot& robot, const string& armName) {
//     // 虚拟的 rs2::pipeline 和 FollowPosition 指针，用于调用 safeCleanup 接口
//     rs2::pipeline pipe;
//     rokae::FollowPosition<ROBOT_DOF>* follower_ptr = nullptr;
    
//     // 为了让 safeCleanup 中的 'setPowerState' 和 'setOperateMode' 能顺利执行，
//     // 机器人通常需要先切换到自动模式或保持在可操作状态。
//     error_code ec;
//     cout << "[" << armName << "] 尝试切换到自动模式以进行清理..." << endl;
//     robot.setOperateMode(OperateMode::automatic, ec);
    
//     safeCleanup(robot, pipe, false, follower_ptr, armName);
// }


void runCleanupOnly(xMateRobot& robot, const string& armName) {
    error_code ec;

    cout << "\n\n=== 强制执行安全清理和断开连接 (仅机器人模式) ===" << endl;
    
    // 1. 等待机械臂静止 (使用 5s 超时)
    waitForIdle(robot, armName, ec); 

    // 2. 尝试切换回非实时模式 (MotionControlMode::trajectory = 0)
    cout << "退出实时控制模式..." << endl;
    robot.setMotionControlMode((MotionControlMode)0, ec); 
    if(ec) { cerr << "❌ 警告：退出实时控制模式失败: " << ec.message() << endl; }

    // 3. 强制下电
    cout << "执行下电操作..." << endl;
    robot.setPowerState(false, ec);
    if(ec) { cerr << "❌ 警告：强制下电失败: " << ec.message() << endl; }
    
    // 4. 切换到手动模式
    cout << "切换到手动模式..." << endl;
    robot.setOperateMode(OperateMode::manual, ec);
    if(ec) { cerr << "❌ 警告：切换到手动模式失败: " << ec.message() << endl; }

    cout << "✅ 强制安全清理完成。" << endl;
}


/* ================= 视觉引导跟随任务 (核心功能) ================= */

void runGuidedPositioning(xMateRobot& robot, const string& cameraSN, const string& armName, const string& calibFile, const std::array<double, ROBOT_DOF>& q_safe) {
    error_code ec;
    
    rs2::pipeline pipe;
    bool is_following = false;
    rokae::FollowPosition<ROBOT_DOF>* follower_ptr = nullptr;

    Eigen::Isometry3d target_eigen_filtered = Eigen::Isometry3d::Identity();
    bool first_detection = true;

    rs2::align align_to_color(RS2_STREAM_COLOR); 
    float depth_scale = 0.0f; 

    std::deque<Point2f> history_center_pixels; 

    // --- 0. 加载标定结果 ---
    Mat H_Flange2Cam;
    try {
        H_Flange2Cam = loadCalibrationMatrix(calibFile);
    } catch (const exception& e) {
        cerr << "❌ 标定文件加载中断。" << endl;
        return;
    }

    try {
        // 1. 启动相机 (启用深度流)
        Mat distCoeffs, K;
        int cam_width = 0, cam_height = 0;
        rs2::config cfg;
        cfg.enable_device(cameraSN);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        try {
            auto profile = pipe.start(cfg);
            auto color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            cam_width = color_profile.width();
            cam_height = color_profile.height();
            K = getIntrinsics(color_profile, distCoeffs);
            depth_scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();

            cout << "✅ 相机启动成功: " << cam_width << "x" << cam_height << ". 深度比例尺: " << depth_scale << endl;
        } catch (const rs2::error& e) {
            throw runtime_error("相机启动失败: " + string(e.what()));
        }

        // 准备标定板坐标
        vector<Point3f> objPoints;
        for(int r=0; r<BOARD_SIZE.height; r++)
            for(int c=0; c<BOARD_SIZE.width; c++)
                objPoints.push_back(Point3f(c * SQUARE_SIZE, r * SQUARE_SIZE, 0));

        // 2. 机器人准备和模式切换
        robot.setOperateMode(OperateMode::automatic, ec);
        if(ec) throw runtime_error("模式切换失败: " + ec.message());

        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        if(ec) throw runtime_error("切换到实时控制模式失败: " + ec.message());

        robot.setPowerState(true, ec);
        if(ec) throw runtime_error("上电失败: " + ec.message());
        
        rokae::xMateModel<ROBOT_DOF> model = robot.model();
        rokae::FollowPosition<ROBOT_DOF> follower(robot, model);
        follower.setScale(FOLLOW_SPEED_FACTOR);
        follower_ptr = &follower;
        
        MoveJToSafe(robot, q_safe, armName, ec);
        waitForIdle(robot, armName, ec);

        // 4. 循环跟随
        while (true) {
            rs2::frameset frames_raw;
            try {
                frames_raw = pipe.wait_for_frames();
            } catch (const rs2::error& e) {
                cerr << "❌ RealSense 帧获取失败: " << e.what() << endl;
                this_thread::sleep_for(chrono::milliseconds(100));
                continue;
            }

            auto frames = align_to_color.process(frames_raw);
            rs2::video_frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();

            Mat frame(Size(cam_width, cam_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
            Mat depth_mat(Size(cam_width, cam_height), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
            Mat display = frame.clone();
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);

            vector<Point2f> corners;
            bool found = findChessboardCorners(gray, BOARD_SIZE, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

            if (found) {
                drawChessboardCorners(display, BOARD_SIZE, corners, found);
                cv::putText(display, "FOLLOWING ACTIVE", Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                
                // PnP 解算
                cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 30, 0.01));
                Mat rvec, tvec;
                solvePnP(objPoints, corners, K, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
                
                // --- 1. 获取中心点并进行趋势校验 ---
                Point2f current_center_pixel = corners[BOARD_SIZE.height * BOARD_SIZE.width / 2]; 

                bool should_update = false;

                if (history_center_pixels.size() < HISTORY_FRAMES) {
                    should_update = true;
                } else {
                    Point2f old_center_pixel = history_center_pixels.front();
                    double pixel_distance = std::sqrt(
                        std::pow(current_center_pixel.x - old_center_pixel.x, 2) + 
                        std::pow(current_center_pixel.y - old_center_pixel.y, 2)
                    );
                    
                    if (pixel_distance > MIN_MOVEMENT_THRESHOLD) {
                        should_update = true;
                    }
                }
                
                history_center_pixels.push_back(current_center_pixel);
                if (history_center_pixels.size() > HISTORY_FRAMES) {
                    history_center_pixels.pop_front();
                }
                
                // --- 2. 只有当 should_update 为 true 时，才计算和更新目标位姿 ---

                if (should_update) {
                    
                    // --- 深度反馈 Z 轴距离控制 ---
                    Point2f center_pixel = current_center_pixel;
                    int center_x = static_cast<int>(center_pixel.x);
                    int center_y = static_cast<int>(center_pixel.y);

                    uint16_t depth_raw = depth_mat.at<uint16_t>(center_y, center_x);
                    double D_cam = depth_raw * depth_scale;

                    if (D_cam == 0.0 || D_cam > 2.0) { 
                        cv::putText(display, "DEPTH INVALID/FAR", Point(20, 80), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
                        cerr << "❌ 警告：深度数据无效或太远，停止跟随。" << endl;
                        if (is_following) follower.stop(); 
                        is_following = false;
                        should_update = false;
                        continue;
                    }
                    
                    double Tx_cam = tvec.at<double>(0);
                    double Ty_cam = tvec.at<double>(1);
                    double Tz_cam = D_cam - Z_APPROACH_OFFSET; 
                    
                    if (Tz_cam < 0.05) {
                        Tz_cam = 0.05; 
                        cv::putText(display, "Z TOO CLOSE! LIMITED", Point(20, 80), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 165, 255), 2);
                    }

                    Mat R_Target2Cam;
                    Rodrigues(rvec, R_Target2Cam);
                    
                    Mat H_Target2Cam = Mat::eye(4, 4, CV_64F);
                    R_Target2Cam.copyTo(H_Target2Cam(Range(0, 3), Range(0, 3)));
                    H_Target2Cam.at<double>(0, 3) = Tx_cam;
                    H_Target2Cam.at<double>(1, 3) = Ty_cam;
                    H_Target2Cam.at<double>(2, 3) = Tz_cam;

                    // --- 坐标系变换 (Target in Base) ---
                    auto current_pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
                    if (ec) { cerr << "❌ 获取当前位姿失败: " << ec.message() << endl; continue; }
                    
                    Mat R_F2B = rpyToRotationMatrix(current_pose.rpy);
                    Mat t_F2B = (Mat_<double>(3,1) << current_pose.trans[0], current_pose.trans[1], current_pose.trans[2]);
                    Mat H_F2B = Mat::eye(4, 4, CV_64F);
                    R_F2B.copyTo(H_F2B(Range(0, 3), Range(0, 3)));
                    t_F2B.copyTo(H_F2B(Range(0, 3), Range(3, 4)));
                    
                    Mat H_Target_raw = H_F2B * H_Flange2Cam * H_Target2Cam;
                    Eigen::Isometry3d target_eigen_raw = MatToEigen(H_Target_raw);

                    // --- 应用低通滤波 ---
                    if (first_detection) {
                        target_eigen_filtered = target_eigen_raw;
                        first_detection = false;
                    } else {
                        target_eigen_filtered.matrix() = (1.0 - FILTER_ALPHA) * target_eigen_filtered.matrix() + FILTER_ALPHA * target_eigen_raw.matrix();
                    }
                } else {
                    cv::putText(display, "MINOR MOVEMENT (HOLD)", Point(20, 80), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 165, 0), 2);
                    if (!is_following) continue; 
                }

                // --- 3. 执行跟随指令 ---
                if (!is_following) {
                    if (should_update) { 
                        cout << "🚀 尝试启动跟随..." << endl;
                        follower.start(target_eigen_filtered);
                        if (ec) {
                            throw RealtimeControlException("FollowPosition 启动失败: " + ec.message());
                        } else {
                            is_following = true;
                            cout << "✅ FollowPosition 启动成功，开始跟随。" << endl;
                        }
                    } 
                } else {
                    if (should_update) {
                       follower.update(target_eigen_filtered);
                    }
                }

            } else {
                cv::putText(display, "SEARCHING TARGET...", Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
                if (is_following) {
                    cout << "⚠️ 目标丢失，停止跟随。" << endl;
                    follower.stop();
                    is_following = false;
                    first_detection = true;
                    history_center_pixels.clear(); 
                }
            }
            
            cv::putText(display, "[Q] Quit [R] Reset Pos", Point(20, 450), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
            imshow("Visual Servoing", display);

            int key = waitKey(1);
            if (key == 'q' || key == 'Q') break;

            if (key == 'r' || key == 'R') {
                cout << "\n--- 用户请求复位 ---" << endl;
                if (is_following) {
                    cout << "⚠️ 停止跟随任务..." << endl;
                    follower.stop();
                    is_following = false;
                }
                first_detection = true; 
                history_center_pixels.clear(); 
                
                try {
                    MoveJToSafe(robot, q_safe, armName, ec);
                    waitForIdle(robot, armName, ec);
                    cout << "--- 已复位，等待重新找到目标并启动跟随 ---" << endl;
                } catch (const exception& e) {
                    cerr << "❌ 复位失败，退出主循环: " << e.what() << endl;
                    break;
                }
            }
        } // end while (true)

    } catch (const exception &e) {
        cerr << "\n\n💥 异常 (任务中断): " << e.what() << endl;
        safeCleanup(robot, pipe, is_following, follower_ptr, armName);
        throw;
    }
    
    // --- 正常退出清理和模式恢复 ---
    safeCleanup(robot, pipe, is_following, follower_ptr, armName);
}


/* ================= 主函数 (最终菜单选项) ================= */

int main() try {
    cout << "=== 视觉伺服跟随/模式清理 (Visual Servoing) ===\n";
    cout << "请选择操作模式:\n";
    cout << "1. 左臂 (启动视觉跟随)\n";
    cout << "2. 右臂 (启动视觉跟随)\n";
    cout << "3. 左臂 (模式复位: 关闭实时控制, 手动模式, 断电, 清理相机)\n";
    cout << "4. 右臂 (模式复位: 关闭实时控制, 手动模式, 断电, 清理相机)\n";
    cout << "Choice: ";
    int c;
    if(!(cin >> c) || c < 1 || c > 4) {
        cerr << "无效输入或无效选项。" << endl;
        return -1;
    }
    
    string ip, sn, name, calib;
    const std::array<double, ROBOT_DOF>* Q_selected_safe_ptr = nullptr;

    bool is_cleanup_mode = (c == 3 || c == 4);
    
    // 设置 IP/SN/NAME 和安全位姿
    if (c == 1 || c == 3) { // 左臂
        ip = IP_LEFT;
        sn = SN_LEFT_CAM;
        name = "LeftArm";
        Q_selected_safe_ptr = &Q_SAFE_LEFT;
    } else { // 右臂
        ip = IP_RIGHT;
        sn = SN_RIGHT_CAM;
        name = "RightArm";
        Q_selected_safe_ptr = &Q_SAFE_RIGHT;
    }
    calib = name + "_interactive_calib.yml";
    
    xMateRobot robot(ip, LOCAL_IP);
    error_code ec_mode;
    
    robot.setRtNetworkTolerance(20, ec_mode);
    if(ec_mode) { cerr << "❌ 设置网络容忍度失败: " << ec_mode.message() << endl; }

    // 程序启动时确保切换到手动模式，作为开始前的稳定状态
    robot.setOperateMode(OperateMode::manual, ec_mode);
    if(ec_mode) { cerr << "❌ 切换到手动模式失败: " << ec_mode.message() << endl; return -1; }
    
    if (is_cleanup_mode) {
        cout << "\n--- 模式复位任务启动 ---\n";
        runCleanupOnly(robot, name);
    } else {
        cout << "\n--- 视觉跟随任务启动 ---\n";
        runGuidedPositioning(robot, sn, name, calib, *Q_selected_safe_ptr);
    }

    cout << "程序 Main 函数正常退出。" << endl;
    return 0;
}
catch (const exception &e) {
    cerr << "💥 程序主异常捕获: " << e.what() << endl;
    return -1;
}