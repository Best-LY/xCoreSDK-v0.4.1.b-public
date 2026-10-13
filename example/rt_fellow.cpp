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

/* ================= 用户配置区域 ================= */

// 🤖 机器人自由度 (6 轴 xMate)
const unsigned short ROBOT_DOF = 6; 

// 🤖 机械臂 IP (请根据您的配置修改)
const string IP_LEFT  = "192.168.2.161"; 
const string IP_RIGHT = "192.168.2.160";

// 💻 本机 IP 
const string LOCAL_IP  = "192.168.2.6"; 

// 📷 相机序列号 
const string SN_LEFT_CAM  = "236422072262"; 
const string SN_RIGHT_CAM = "236422070786"; 

// 🏁 标定板参数
const Size BOARD_SIZE(8, 6);       
const float SQUARE_SIZE = 0.020f; // 20mm

// 📐 跟随参数
const double Z_APPROACH_OFFSET = 0.10; // 目标上方 10cm 
// 目标姿态
const array<double, 3> TARGET_RPY = {Utils::degToRad(-135.0), Utils::degToRad(0.0), Utils::degToRad(-180.0)}; 
const double FOLLOW_SPEED_FACTOR = 0.3; // 速度系数 [0.0 - 1.0] - 降低至0.3以减少震荡

// ===== 抖动消除参数 (新增) =====
const double POSITION_DEADZONE = 0.002;      // 位置死区: 2mm
const double ORIENTATION_DEADZONE = 0.01;    // 姿态死区: 0.01 rad (~0.6°)
const double FILTER_ALPHA = 0.3;             // 平滑滤波系数 (0.0-1.0, 越小越平滑)
const int UPDATE_SKIP_FRAMES = 2;            // 每3帧更新一次 (降低更新频率)
const int LOSS_TOLERANCE_FRAMES = 15;        // 目标丢失容限: 15帧 (~150ms)

// ===== 距离自适应跟随参数 (新增) =====
const double TARGET_FOLLOW_DISTANCE = 0.50;  // 目标跟随距离: 50cm
const double DISTANCE_DEADZONE = 0.05;       // 距离死区: 5cm (±5cm内不调整)
const double DISTANCE_SCALE = 0.5;           // 距离补偿系数 (0.0-1.0)

// 🏠 安全初始位姿 (用户自定义)
// 左臂: {0, -M_PI/2, -M_PI/2, -M_PI, 0, M_PI*0.75}，为避免轴的角度恰好设定在硬限位临界值，故增加角度
const std::array<double, ROBOT_DOF> Q_SAFE_LEFT = {0, -M_PI/6, M_PI/3, -M_PI+0.05, -M_PI * 0.25, M_PI * 0.75};
// 右臂: {M_PI, M_PI/2, M_PI/2, -M_PI, 0, -M_PI * 1.25};
const std::array<double, ROBOT_DOF> Q_SAFE_RIGHT = {M_PI, M_PI/2, M_PI/2, -M_PI+0.05, 0, -M_PI * 1.25};


/* ================= 辅助函数 (已更新：waitForIdle 速度) ================= */

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

/**
 * @brief 计算两个位姿之间的位置差 (欧几里得距离)
 */
double calcPositionDistance(const Eigen::Isometry3d& T1, const Eigen::Isometry3d& T2) {
    return (T1.translation() - T2.translation()).norm();
}

/**
 * @brief 计算两个位姿之间的姿态差 (旋转向量的模)
 */
double calcOrientationDifference(const Eigen::Isometry3d& T1, const Eigen::Isometry3d& T2) {
    Eigen::Matrix3d dR = T1.rotation().transpose() * T2.rotation();
    Eigen::AngleAxisd aa(dR);
    return abs(aa.angle());
}

/**
 * @brief 一阶低通滤波: target = alpha * measured + (1-alpha) * target
 * @param[in/out] filtered_pose 滤波后的位姿 (会被更新)
 * @param[in] measured_pose 当前测量值
 * @param[in] alpha 滤波系数 (0.0-1.0, 越小越平滑)
 */
void lowPassFilter(Eigen::Isometry3d& filtered_pose, const Eigen::Isometry3d& measured_pose, double alpha) {
    // 对位置进行线性插值
    Eigen::Vector3d filtered_pos = alpha * measured_pose.translation() + (1.0 - alpha) * filtered_pose.translation();
    
    // 对旋转矩阵进行球面线性插值 (SLERP)
    Eigen::Quaterniond q_measured(measured_pose.rotation());
    Eigen::Quaterniond q_filtered(filtered_pose.rotation());
    Eigen::Quaterniond q_result = q_filtered.slerp(alpha, q_measured);
    Eigen::Matrix3d filtered_rot = q_result.toRotationMatrix();
    
    // 更新滤波后的位姿
    filtered_pose.translation() = filtered_pos;
    filtered_pose.linear() = filtered_rot;
}

/**
 * @brief 检查目标是否满足更新条件 (位置和姿态都在死区外)
 */
bool shouldUpdateTarget(const Eigen::Isometry3d& current_target, 
                       const Eigen::Isometry3d& last_sent_target,
                       double pos_threshold, 
                       double ori_threshold) {
    double pos_diff = calcPositionDistance(current_target, last_sent_target);
    double ori_diff = calcOrientationDifference(current_target, last_sent_target);
    
    return (pos_diff > pos_threshold) || (ori_diff > ori_threshold);
}

/**
 * @brief 计算棋盘到相机的距离 (基于PnP解算的tvec)
 * @param[in] tvec PnP解算得到的平移向量 (相机坐标系中)
 * @return 距离值，单位：米
 */
double calcBoardDistance(const Mat& tvec) {
    if (tvec.empty() || tvec.rows < 3) return -1.0;
    double x = tvec.at<double>(0);
    double y = tvec.at<double>(1);
    double z = tvec.at<double>(2);
    return std::sqrt(x*x + y*y + z*z);
}

/**
 * @brief 基于距离进行目标修正 (实现近处后退、远处跟进)
 * @param[in/out] target_pose 目标位姿 (会被修改)
 * @param[in] current_distance 当前距离 (米)
 * @param[in] target_distance 目标距离 (米)
 * 
 * 原理：
 *   当current_distance < target_distance时，机械臂后退（沿Z负方向）
 *   当current_distance > target_distance时，机械臂前进（沿Z正方向）
 *   在死区范围内不调整
 */
void adjustTargetByDistance(Eigen::Isometry3d& target_pose, 
                           double current_distance, 
                           double target_distance) {
    double distance_error = target_distance - current_distance;  // 目标距离 - 当前距离
    
    // 死区检查：如果误差在死区内，不调整
    if (std::abs(distance_error) < DISTANCE_DEADZONE) {
        return;
    }
    
    // 计算调整量：沿基坐标系Z轴方向调整
    // 当distance_error > 0 时，说明当前距离小于目标距离，需要后退（Z负方向）
    // 当distance_error < 0 时，说明当前距离大于目标距离，需要前进（Z正方向）
    double adjustment = distance_error * DISTANCE_SCALE;
    
    // 修改目标位姿的Z坐标（基坐标系）
    target_pose.translation()(2) += adjustment;
}

/**
 * @brief 阻塞等待机械臂运动静止
 * 【已更新：超时时间缩短至 5 秒】
 */
void waitForIdle(xMateRobot& robot, const string& armName, error_code& ec) {
    cout << "[" << armName << "] 正在等待机械臂静止 (Idle, max 5s)..." << endl;
    auto start_time = chrono::steady_clock::now();
    const chrono::seconds timeout(5); // <-- 缩短至 5 秒

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
            return; // 超时退出
        }
    } while (true);
}

/**
 * @brief 在 RtCommand 模式下执行 MoveJ 到安全位姿
 */
void MoveJToSafe(xMateRobot& robot, const std::array<double, ROBOT_DOF>& q_safe, const string& armName, error_code& ec) {
    auto rtCon = robot.getRtMotionController().lock();
    if (!rtCon) { 
        throw runtime_error("获取实时运动控制器失败！无法执行 MoveJ。"); 
    }
    
    cout << "[" << armName << "] 正在 MoveJ 到安全位姿..." << endl;
    
    // MoveJ 从当前位置到安全位姿 (速度系数 0.5)
    rtCon->MoveJ(0.5, robot.jointPos(ec), q_safe);
    if(ec) { 
        throw runtime_error("MoveJ 到安全位姿失败: " + ec.message()); 
    }
    cout << "✅ MoveJ 运动命令已发出。" << endl;
}

/**
 * @brief 核心安全清理流程 (运行时跟随退出时调用，包含相机停止)
 */
void safeCleanup(xMateRobot& robot, rs2::pipeline& pipe, bool is_following, rokae::FollowPosition<ROBOT_DOF>* follower_ptr, const string& armName) {
    error_code ec;

    cout << "\n\n=== 执行安全清理和断开连接 (包含相机) ===" << endl;

    // 1. 强制停止跟随 
    if (follower_ptr && is_following) {
        cout << "⚠️ 停止 FollowPosition 任务..." << endl;
        follower_ptr->stop();
    }

    // 2. 等待机械臂静止 (使用 5s 超时)
    waitForIdle(robot, armName, ec); 
    
    // 3. 尝试切换回非实时模式 (MotionControlMode::trajectory = 0)
    cout << "退出实时控制模式..." << endl;
    robot.setMotionControlMode((MotionControlMode)0, ec); 
    if(ec) { cerr << "❌ 警告：退出实时控制模式失败: " << ec.message() << endl; }

    // 4. 强制下电
    cout << "执行下电操作..." << endl;
    robot.setPowerState(false, ec);
    if(ec) { cerr << "❌ 警告：强制下电失败: " << ec.message() << endl; }
    
    // 5. 切换到手动模式
    cout << "切换到手动模式..." << endl;
    robot.setOperateMode(OperateMode::manual, ec);
    if(ec) { cerr << "❌ 警告：切换到手动模式失败: " << ec.message() << endl; }

    // 6. 关闭相机和窗口
    pipe.stop();
    destroyAllWindows();

    cout << "✅ 安全清理完成，程序即将退出。" << endl;
}

/**
 * @brief 机器人模式清理流程 (启动菜单中选项 3/4 调用)
 * 逻辑：等待静止 -> 退出实时模式 -> 下电 -> 手动模式
 */
void modeCleanupOnly(xMateRobot& robot, const string& armName) {
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


/* ================= 视觉引导跟随任务 (已更新：添加 Q_SAFE 参数) ================= */

void runGuidedPositioning(xMateRobot& robot, const string& cameraSN, const string& armName, const string& calibFile, const std::array<double, ROBOT_DOF>& q_safe) {
    error_code ec; 
    
    rs2::pipeline pipe; 
    bool is_following = false; 
    rokae::FollowPosition<ROBOT_DOF>* follower_ptr = nullptr; 

    // --- 0. 加载标定结果 ---
    Mat H_Flange2Cam;
    try {
        H_Flange2Cam = loadCalibrationMatrix(calibFile);
    } catch (const exception& e) {
        cerr << "❌ 标定文件加载中断。" << endl;
        return; 
    }

    try {
        // 1. 启动相机
        Mat distCoeffs, K;
        int cam_width = 0, cam_height = 0;
        rs2::config cfg;
        cfg.enable_device(cameraSN);
        cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8); 
        try {
            auto profile = pipe.start(cfg).get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            cam_width = profile.width();
            cam_height = profile.height();
            K = getIntrinsics(profile, distCoeffs);
            cout << "✅ 相机启动成功: " << cam_width << "x" << cam_height << endl;
        } catch (const rs2::error& e) {
            throw runtime_error("相机启动失败: " + string(e.what()));
        }

        // 准备标定板坐标
        vector<Point3f> objPoints;
        for(int r=0; r<BOARD_SIZE.height; r++)
            for(int c=0; c<BOARD_SIZE.width; c++)
                objPoints.push_back(Point3f(c * SQUARE_SIZE, r * SQUARE_SIZE, 0));

        // --- 2. 机器人准备和模式切换 ---
        cout << "[" << armName << "] 正在切换到自动模式..." << endl;
        robot.setOperateMode(OperateMode::automatic, ec);
        if(ec) throw runtime_error("模式切换失败: " + ec.message());

        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        if(ec) throw runtime_error("切换到实时控制模式失败: " + ec.message());
        cout << "✅ 机器人已切换到实时控制模式 (RtCommand)。" << endl;

        cout << "[" << armName << "] 正在上电..." << endl;
        robot.setPowerState(true, ec);
        if(ec) throw runtime_error("上电失败: " + ec.message());
        
        cout << "✅ 机器人已处于自动模式并上电。" << endl;


        // --- 3. 初始化 FollowPosition 并 MoveJ 预热 ---
        rokae::xMateModel<ROBOT_DOF> model = robot.model(); 
        
        rokae::FollowPosition<ROBOT_DOF> follower(robot, model);
        follower.setScale(FOLLOW_SPEED_FACTOR); 
        follower_ptr = &follower; // 设置指针
        
        cout << "✅ FollowPosition 初始化完成，等待目标。速度系数: " << FOLLOW_SPEED_FACTOR << endl;

        // 【 MoveJ 预热，激活实时控制链路 】
        MoveJToSafe(robot, q_safe, armName, ec); // 使用传递进来的 q_safe
        // 等待 MoveJ 完成并静止
        waitForIdle(robot, armName, ec);


        // 4. 循环跟随
        int frame_count = 0;                              // 帧计数 (用于UPDATE_SKIP_FRAMES)
        int loss_count = 0;                               // 目标丢失计数
        Eigen::Isometry3d last_valid_target;              // 上一个有效目标
        Eigen::Isometry3d filtered_target;                // 滤波后的目标
        Eigen::Isometry3d last_sent_target;               // 上次发送给机器人的目标

        while (true) {
            rs2::frameset frames;
            try {
                frames = pipe.wait_for_frames();
            } catch (const rs2::error& e) {
                cerr << "❌ RealSense 帧获取失败: " << e.what() << endl;
                this_thread::sleep_for(chrono::milliseconds(100));
                continue; 
            }

            Mat frame(Size(cam_width, cam_height), CV_8UC3, (void*)frames.get_color_frame().get_data(), Mat::AUTO_STEP);
            Mat display = frame.clone();
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);

            vector<Point2f> corners;
            bool found = findChessboardCorners(gray, BOARD_SIZE, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

            if (found) {
                loss_count = 0;  // 重置丢失计数
                drawChessboardCorners(display, BOARD_SIZE, corners, found);
                cv::putText(display, "FOLLOWING ACTIVE", Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                
                // PnP 解算 (略去中间计算过程)
                cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 30, 0.01));
                Mat rvec, tvec;
                solvePnP(objPoints, corners, K, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
                
                Mat R_Target2Cam;
                Rodrigues(rvec, R_Target2Cam);
                Mat H_Target2Cam = Mat::eye(4, 4, CV_64F);
                R_Target2Cam.copyTo(H_Target2Cam(Range(0, 3), Range(0, 3)));
                tvec.copyTo(H_Target2Cam(Range(0, 3), Range(3, 4)));

                auto current_pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
                if (ec) { cerr << "❌ 获取当前位姿失败: " << ec.message() << endl; continue; }
                
                Mat R_F2B = rpyToRotationMatrix(current_pose.rpy);
                Mat t_F2B = (Mat_<double>(3,1) << current_pose.trans[0], current_pose.trans[1], current_pose.trans[2]);
                Mat H_F2B = Mat::eye(4, 4, CV_64F);
                R_F2B.copyTo(H_F2B(Range(0, 3), Range(0, 3)));
                t_F2B.copyTo(H_F2B(Range(0, 3), Range(3, 4)));
                
                Mat H_Target = H_F2B * H_Flange2Cam * H_Target2Cam;
                
                Mat H_Offset = Mat::eye(4, 4, CV_64F);
                H_Offset.at<double>(2, 3) = -Z_APPROACH_OFFSET; 
                H_Target = H_Target * H_Offset;
                
                Mat R_Fix = rpyToRotationMatrix(TARGET_RPY);
                R_Fix.copyTo(H_Target(Range(0, 3), Range(0, 3)));

                Eigen::Isometry3d measured_target = MatToEigen(H_Target);
                
                // ===== 计算棋盘距离 (新增) =====
                double board_distance = calcBoardDistance(tvec);
                
                // ===== 基于距离的目标修正 (新增) =====
                if (board_distance > 0) {
                    adjustTargetByDistance(measured_target, board_distance, TARGET_FOLLOW_DISTANCE);
                }
                
                // ===== 平滑滤波处理 =====
                if (frame_count == 0) {
                    // 第一次初始化
                    filtered_target = measured_target;
                    last_valid_target = measured_target;
                    last_sent_target = measured_target;
                } else {
                    // 应用一阶低通滤波
                    lowPassFilter(filtered_target, measured_target, FILTER_ALPHA);
                }
                
                last_valid_target = measured_target;  // 保存最新的有效测量值

                // ===== 帧跳过和死区控制 =====
                frame_count++;
                if (frame_count % (UPDATE_SKIP_FRAMES + 1) == 0) {
                    if (shouldUpdateTarget(filtered_target, last_sent_target, POSITION_DEADZONE, ORIENTATION_DEADZONE)) {
                        // 执行跟随
                        if (!is_following) {
                            cout << "🚀 尝试启动跟随..." << endl;
                            follower.start(filtered_target); 
                            if (ec) {
                                throw RealtimeControlException("FollowPosition 启动失败: " + ec.message());
                            } else {
                                is_following = true;
                                cout << "✅ FollowPosition 启动成功，开始跟随。" << endl;
                            }
                        } else {
                            follower.update(filtered_target); 
                        }
                        last_sent_target = filtered_target;  // 更新上次发送的目标
                    }
                }
                
                // 显示滤波效果
                cv::putText(display, format("Pos:%+.3f mm  Ori:%+.3f rad", 
                    calcPositionDistance(filtered_target, last_valid_target) * 1000,
                    calcOrientationDifference(filtered_target, last_valid_target)), 
                    Point(20, 70), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100, 200, 100), 1);
                
                // ===== 距离显示 (新增) =====
                if (board_distance > 0) {
                    double distance_error = TARGET_FOLLOW_DISTANCE - board_distance;
                    string distance_status = "READY";
                    Scalar distance_color = Scalar(0, 255, 0);  // 绿色
                    
                    if (distance_error > DISTANCE_DEADZONE) {
                        distance_status = "BACKING (靠近)";
                        distance_color = Scalar(0, 165, 255);   // 橙色
                    } else if (distance_error < -DISTANCE_DEADZONE) {
                        distance_status = "FORWARD (远离)";
                        distance_color = Scalar(255, 100, 100); // 蓝色
                    }
                    
                    cv::putText(display, format("Dist: %.2fm (Target: %.2fm) - %s", 
                        board_distance, TARGET_FOLLOW_DISTANCE, distance_status.c_str()), 
                        Point(20, 100), FONT_HERSHEY_SIMPLEX, 0.6, distance_color, 2);
                }
                    
            } else {
                cv::putText(display, "SEARCHING TARGET...", Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
                loss_count++;
                
                // 如果目标丢失超过容限，停止跟随
                if (is_following && loss_count > LOSS_TOLERANCE_FRAMES) {
                    cout << "⚠️ 目标丢失 " << loss_count << " 帧，停止跟随。" << endl;
                    follower.stop();
                    is_following = false;
                } else if (loss_count <= LOSS_TOLERANCE_FRAMES && is_following) {
                    // 在容限范围内，使用上一个有效目标继续跟随（防止抖动）
                    frame_count++;
                    if (frame_count % (UPDATE_SKIP_FRAMES + 1) == 0) {
                        follower.update(last_valid_target);
                    }
                    cv::putText(display, format("Lost: %d/%d frames", loss_count, LOSS_TOLERANCE_FRAMES), 
                        Point(20, 70), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 165, 0), 1);
                }
            }
            
            // 运行时可按 Q 退出，或按 R 回到安全位姿
            cv::putText(display, "[Q] Quit [R] Reset Pos", Point(20, 450), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
            imshow("Visual Servoing", display);

            int key = waitKey(1);
            if (key == 'q' || key == 'Q') break; 

            // --- 运行时复位功能 (R 键) ---
            if (key == 'r' || key == 'R') {
                cout << "\n--- 用户请求复位 ---" << endl;
                if (is_following) {
                    cout << "⚠️ 停止跟随任务..." << endl;
                    follower.stop();
                    is_following = false;
                }
                
                // 运动到安全位姿
                try {
                    MoveJToSafe(robot, q_safe, armName, ec); // 使用传递进来的 q_safe
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
    
    // --- 正常退出清理 ---
    safeCleanup(robot, pipe, is_following, follower_ptr, armName);
}


int main() try {
    cout << "=== 视觉伺服跟随测试 (Visual Servoing) ===\n";
    cout << "请选择操作:\n";
    cout << "1. 左臂 (启动视觉跟随)\n";
    cout << "2. 右臂 (启动视觉跟随)\n";
    cout << "3. 左臂 (仅执行模式清理: 关闭实时控制, 手动模式, 断电)\n";
    cout << "4. 右臂 (仅执行模式清理: 关闭实时控制, 手动模式, 断电)\n";
    cout << "Choice: ";
    int c; 
    if(!(cin >> c) || c < 1 || c > 4) {
        cerr << "无效输入或无效选项。" << endl;
        return -1;
    }
    
    string ip = (c==1 || c==3) ? IP_LEFT : IP_RIGHT;
    string sn = (c==1 || c==3) ? SN_LEFT_CAM : SN_RIGHT_CAM;
    string name = (c==1 || c==3) ? "LeftArm" : "RightArm";
    string calib = name + "_interactive_calib.yml";

    // 根据选择确定安全位姿
    const std::array<double, ROBOT_DOF>& Q_selected_safe = (c == 1 || c == 3) ? Q_SAFE_LEFT : Q_SAFE_RIGHT;
    
    xMateRobot robot(ip, LOCAL_IP);
    error_code ec_mode; 
    
    robot.setRtNetworkTolerance(20, ec_mode); 

    // 默认切换到手动模式，确保程序开始有一个可操作的基础状态
    robot.setOperateMode(OperateMode::manual, ec_mode); 
    if(ec_mode) { cerr << "❌ 切换到手动模式失败: " << ec_mode.message() << endl; return -1; }

    if (c == 3 || c == 4) {
        // --- 选项 3 或 4: 仅执行模式清理 ---
        // 需要切换到自动模式并尝试上电才能执行模式清理
        cout << "[" << name << "] 切换到自动模式并尝试上电以执行清理操作..." << endl;
        robot.setOperateMode(OperateMode::automatic, ec_mode);
        if(!ec_mode) robot.setPowerState(true, ec_mode);
        if(ec_mode) { cerr << "❌ 警告：尝试上电失败: " << ec_mode.message() << " (将尝试继续清理)" << endl; }
        
        // 执行模式清理（等待静止 -> 退出实时模式 -> 下电 -> 手动模式）
        modeCleanupOnly(robot, name);
        return 0;
    }

    // --- 选项 1 或 2: 启动视觉跟随 ---
    
    // --- 启动时安全位姿选项 ---
    cout << "\n是否立即将机械臂 MoveJ 运动到安全位姿?\n1. 是 (推荐)\n2. 否\nChoice: ";
    int start_move_choice;
    // 确保用户输入有效
    if (cin >> start_move_choice && start_move_choice == 1) {
        try {
            // 在 MoveJ 前进行必要的模式切换和上电
            cout << "启动时运动: 切换到自动模式并上电..." << endl;
            robot.setOperateMode(OperateMode::automatic, ec_mode);
            if(ec_mode) throw runtime_error("模式切换失败: " + ec_mode.message());

            robot.setMotionControlMode(MotionControlMode::RtCommand, ec_mode);
            if(ec_mode) throw runtime_error("切换到实时控制模式失败: " + ec_mode.message());

            robot.setPowerState(true, ec_mode);
            if(ec_mode) throw runtime_error("上电失败: " + ec_mode.message());

            MoveJToSafe(robot, Q_selected_safe, name, ec_mode); // 使用选定的安全位姿
            waitForIdle(robot, name, ec_mode);
            
            // MoveJ 结束后，安全退出到手动模式，等待 runGuidedPositioning 再次设置
            cout << "启动时运动完成。执行临时安全清理..." << endl;
            robot.setMotionControlMode((MotionControlMode)0, ec_mode);
            robot.setPowerState(false, ec_mode);
            robot.setOperateMode(OperateMode::manual, ec_mode);

        } catch (const exception& e) {
            cerr << "❌ 启动时 MoveJ 失败: " << e.what() << endl;
            cerr << "将尝试继续运行主程序..." << endl;
        }
    }
    
    // 启动视觉跟随，并将选定的安全位姿传递给它
    runGuidedPositioning(robot, sn, name, calib, Q_selected_safe);

    cout << "程序 Main 函数正常退出。" << endl;
    return 0;
}
catch (const exception &e) {
    cerr << "💥 程序主异常捕获: " << e.what() << endl;
    return -1;
}

