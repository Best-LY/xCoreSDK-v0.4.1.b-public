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
#include "print_helper.hpp" 
#include "rokae/model.h"

using namespace std;
using namespace cv;
using namespace rokae;

/* ================= 用户配置区域 (与标定代码保持一致) ================= */
// 🤖 机器人自由度 (6 轴 xMate)
const unsigned short ROBOT_DOF = 6; 

// 🤖 机械臂 IP (请确保 IP 正确对应物理左臂/右臂)
const string IP_LEFT  = "192.168.2.161"; 
const string IP_RIGHT = "192.168.2.160";

// 📷 相机序列号
const string SN_LEFT_CAM  = "236422072262"; 
const string SN_RIGHT_CAM = "236422070786"; 

// 🏁 标定板参数
const Size BOARD_SIZE(8, 6);       // 内角点 (宽, 高)
const float SQUARE_SIZE = 0.020f; // 20mm, 单位：米

// 📐 运动目标配置
const double Z_APPROACH_OFFSET = 0.10; // 目标点上方 10 厘米 (0.10 米)
// 目标姿态：垂直向下 (Rx, Ry, Rz)
const array<double, 3> TARGET_RPY = {Utils::degToRad(135.0), Utils::degToRad(0.0), Utils::degToRad(180.0)}; 
const double MOVE_VELOCITY = 0.5; // 运动速度系数 [0.0 - 1.0]

/* ============================================== */

// 辅助函数：将 Rokae 的 RPY 位姿转换为 OpenCV 的旋转矩阵 Mat
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

// 辅助函数：获取相机内参 (与标定程序相同)
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

// 辅助函数：加载标定结果矩阵 H_Flange2Cam
Mat loadCalibrationMatrix(const string& filename) {
    Mat H;
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "❌ 无法打开标定文件: " << filename << "。请确保文件存在！" << endl;
        throw runtime_error("Calibration file not found.");
    }
    fs["H_Flange2Cam"] >> H;
    fs.release();
    if (H.empty() || H.rows != 4 || H.cols != 4) {
        cerr << "❌ 标定矩阵 H 读取错误或格式不正确。" << endl;
        throw runtime_error("Invalid matrix format in file.");
    }
    cout << "✅ 成功加载标定矩阵 H (Flange2Cam) from " << filename << endl;
    return H;
}

// 辅助函数：计算 4x4 齐次变换矩阵的逆矩阵
Mat inverseHomogeneousMatrix(const Mat& H) {
    Mat R = H(Range(0, 3), Range(0, 3));
    Mat t = H(Range(0, 3), Range(3, 4));
    
    Mat R_T = R.t();
    Mat t_inv = -R_T * t;
    
    Mat H_inv = Mat::eye(4, 4, CV_64F);
    R_T.copyTo(H_inv(Range(0, 3), Range(0, 3)));
    t_inv.copyTo(H_inv(Range(0, 3), Range(3, 4)));
    return H_inv;
}

// 等待运动结束
void WaitRobot(BaseRobot *robot) {
    bool running = true;
    error_code ec;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto st = robot->operationState(ec);
        if(ec) return; 
        if(st == OperationState::idle || st == OperationState::unknown){
            running = false;
        }
    }
}

/**
 * @brief 视觉引导定位任务
 */
void runGuidedPositioning(xMateRobot& robot, const string& cameraSN, const string& armName, const string& calibFile) {
    error_code ec; 
    Mat H_Flange2Cam = loadCalibrationMatrix(calibFile);
    Mat H_Cam2Flange = inverseHomogeneousMatrix(H_Flange2Cam); 
    
    rs2::pipeline pipe;
    Mat distCoeffs, K;
    int cam_width = 0;
    int cam_height = 0;

    // --- 1. 开启相机 ---
    rs2::config cfg;
    cfg.enable_device(cameraSN);
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8); 
    
    try {
        rs2::pipeline_profile pipeline_profile = pipe.start(cfg);
        auto profile = pipeline_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        cam_width = profile.width();
        cam_height = profile.height();
        K = getIntrinsics(profile, distCoeffs);
        cout << "✅ 相机启动成功，实际分辨率: " << cam_width << "x" << cam_height << endl;
    } catch (const rs2::error& e) {
        cerr << "❌ 相机启动失败: " << e.what() << endl;
        return;
    }

    // 准备标定板物理坐标
    vector<Point3f> objPoints;
    for(int r=0; r<BOARD_SIZE.height; r++)
        for(int c=0; c<BOARD_SIZE.width; c++)
            objPoints.push_back(Point3f(c * SQUARE_SIZE, r * SQUARE_SIZE, 0));

    // --- 2. 机器人上电并进入自动模式 (运动需要自动模式) ---
    cout << "[" << armName << "] 正在上电并切换到自动模式..." << endl;
    robot.setOperateMode(OperateMode::automatic, ec);
    if(ec) { cerr << "❌ 切换到自动模式失败: " << ec.message() << endl; pipe.stop(); return; }
    robot.setPowerState(true, ec);
    if(ec) { cerr << "❌ 机器人上电失败: " << ec.message() << endl; pipe.stop(); return; }
    cout << "✅ 机器人已上电，等待运动指令。" << endl;
    
    // 设置非实时运动控制模式
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    if(ec) { cerr << "❌ 设置非实时运动模式失败: " << ec.message() << endl; pipe.stop(); return; }


    // --- 3. 交互循环：定位并移动 ---
    while (true) {
        // 获取图像
        rs2::frameset frames = pipe.wait_for_frames();
        Mat frame(Size(cam_width, cam_height), CV_8UC3, (void*)frames.get_color_frame().get_data(), Mat::AUTO_STEP);
        Mat display = frame.clone();
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // 检测角点
        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, BOARD_SIZE, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

        // UI 绘制
        if (found) {
            drawChessboardCorners(display, BOARD_SIZE, corners, found);
            cv::putText(display, "READY: Press [SPACE] to move to target center!", Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        } else {
            cv::putText(display, "NO BOARD DETECTED", Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        }
        
        cv::putText(display, "Press [Q] or [ESC] to quit.", Point(20, 450), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
        imshow("Hand-Eye Validation Test - Guided Positioning", display);

        // 按键处理
        int key = waitKey(30);

        // ---> 运动逻辑 <---
        if (key == 32) { // Space 键
            if (!found) {
                cout << "⚠️ 没看到标定板，无法定位！" << endl;
                continue;
            }

            // 1. SolvePnP 计算 H_Target2Cam (标定板相对于相机)
            cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 30, 0.01));
            Mat rvec, tvec;
            solvePnP(objPoints, corners, K, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
            
            Mat R_Target2Cam;
            Rodrigues(rvec, R_Target2Cam);
            Mat H_Target2Cam = Mat::eye(4, 4, CV_64F);
            R_Target2Cam.copyTo(H_Target2Cam(Range(0, 3), Range(0, 3)));
            tvec.copyTo(H_Target2Cam(Range(0, 3), Range(3, 4)));

            // 2. 获取 H_Flange2Base (法兰盘相对于基座)
            auto current_pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
            if (ec) {
                cerr << "⚠️ 获取机械臂位姿失败: " << ec.message() << endl;
                continue;
            }

            // 3. 计算标定板中心点在 Base 坐标系下的位置 H_Target2Base
            Mat R_Flange2Base = rpyToRotationMatrix(current_pose.rpy);
            Mat t_Flange2Base = (Mat_<double>(3,1) << current_pose.trans[0], current_pose.trans[1], current_pose.trans[2]);
            Mat H_Flange2Base = Mat::eye(4, 4, CV_64F);
            R_Flange2Base.copyTo(H_Flange2Base(Range(0, 3), Range(0, 3)));
            t_Flange2Base.copyTo(H_Flange2Base(Range(0, 3), Range(3, 4)));
            
            Mat H_Target2Base = H_Flange2Base * H_Cam2Flange * H_Target2Cam;
            
            Mat target_t = H_Target2Base(Range(0, 3), Range(3, 4));
            
            // 4. 构造目标移动位姿 P_Cartesian
            CartesianPosition target_pos;
            target_pos.trans[0] = target_t.at<double>(0);
            target_pos.trans[1] = target_t.at<double>(1);
            target_pos.trans[2] = target_t.at<double>(2) + Z_APPROACH_OFFSET; 
            target_pos.rpy = TARGET_RPY; 
            target_pos.elbow = current_pose.elbow; 

            cout << "\n🎯 计算目标点在 Base 坐标系下的位姿 (10cm上方):" << endl;
            cout << target_pos << endl;
            if (target_pos.trans[2] > 1.45) {
                cout << "⚠️ 警告: 目标 Z 值偏高(" << target_pos.trans[2]
                     << "m)，可能超出机械臂可达工作空间。" << endl;
            }

            // 5. 使用逆运动学 (IK) 计算关节角度
            rokae::Model_T<ROBOT_DOF> model = robot.model(); 

            // 单一姿态可能无解：依次尝试“目标姿态/当前姿态 + 不同肘角”
            vector<CartesianPosition> ik_candidates;
            {
                CartesianPosition c1 = target_pos; // 目标姿态 + 当前肘角
                c1.elbow = current_pose.elbow;
                ik_candidates.push_back(c1);

                CartesianPosition c2 = target_pos; // 目标姿态 + 0肘角
                c2.elbow = 0.0;
                ik_candidates.push_back(c2);

                CartesianPosition c3 = target_pos; // 当前姿态 + 当前肘角
                c3.rpy = current_pose.rpy;
                c3.elbow = current_pose.elbow;
                ik_candidates.push_back(c3);

                CartesianPosition c4 = c3; // 当前姿态 + 0肘角
                c4.elbow = 0.0;
                ik_candidates.push_back(c4);
            }

            bool ik_found = false;
            array<double, ROBOT_DOF> target_joint_pos{};
            string last_ik_error;

            for (size_t i = 0; i < ik_candidates.size(); ++i) {
                error_code ik_ec;
                auto joints = model.calcIk(ik_candidates[i], ik_ec);
                if (!ik_ec) {
                    target_joint_pos = joints;
                    ik_found = true;
                    cout << "✅ 逆解成功，使用候选 #" << (i + 1) << endl;
                    break;
                }
                last_ik_error = ik_ec.message();
            }

            if (!ik_found) {
                cerr << "❌ 逆运动学计算失败: " << last_ik_error << endl;
                cerr << "   可能原因: 目标点超出工作空间、姿态过于苛刻、肘角分支不合适。" << endl;
                continue;
            }

            cout << "✅ 找到逆解。使用此解进行运动..." << endl;

            // 6. 运动到目标位姿
            // 【修正：使用 MoveAbsJCommand 执行关节运动】
            // 步骤：
            // 1. 将 array 转为 vector (JointPosition 构造函数需要 vector 或 initializer_list)
            std::vector<double> joints_vec(target_joint_pos.begin(), target_joint_pos.end());
            
            // 2. 构造 JointPosition 对象
            rokae::JointPosition joint_pos(joints_vec);
            
            // 3. 构造 MoveAbsJCommand 对象 (Move Absolute Joint)
            // MoveAbsJCommand 不是模板，直接使用类名
            rokae::MoveAbsJCommand cmd(joint_pos);
            
            // 4. 设置速度比例 (使用 jointSpeed 成员)
            cmd.jointSpeed = MOVE_VELOCITY; 
            
            std::string cmd_id;
            
            // 5. 使用 moveAppend 添加命令
            robot.moveAppend({cmd}, cmd_id, ec); 
            
            if (ec) {
                cerr << "❌ 运动指令下发失败 (moveAppend): " << ec.message() << endl;
            } else {
                // 6. 调用 moveStart 执行
                robot.moveStart(ec); 
                if (!ec) {
                    cout << "✅ 机械臂开始运动 (MoveAbsJCommand)..." << endl;
                    WaitRobot(&robot);
                    cout << "✅ 运动完成，请检查法兰盘是否对齐目标点。" << endl;
                } else {
                    cerr << "❌ 运动开始失败: " << ec.message() << endl;
                }
            }
        }
        // ---> 退出逻辑 <---
        else if (key == 'q' || key == 'Q' || key == 27) { // Q 或 ESC
            break;
        }
    }

    // --- 4. 清理 ---
    cout << "正在下电并断开连接..." << endl;
    robot.setPowerState(false, ec); 
    pipe.stop();
    destroyAllWindows();
}


int main() try {
    cout << "================================================\n";
    cout << "     Rokae 视觉引导定位测试 (Move to Target)    \n";
    cout << "================================================\n";
    cout << "1. 测试左臂 Left Arm\n";
    cout << "2. 测试右臂 Right Arm\n";
    cout << "请输入选项: ";
    
    int choice;
    if (!(cin >> choice)) return -1;

    string ip, camSN, armName, calibFile;

    if (choice == 1) {
        ip = IP_LEFT;
        camSN = SN_LEFT_CAM;
        armName = "LeftArm";
        calibFile = armName + "_interactive_calib.yml";
    } 
    else if (choice == 2) {
        ip = IP_RIGHT;
        camSN = SN_RIGHT_CAM;
        armName = "RightArm";
        calibFile = armName + "_interactive_calib.yml";
    } else {
        cout << "输入无效。" << endl;
        return -1;
    }
    
    xMateRobot robot(ip);
    error_code ec_mode; 
    // 启动前先设置为手动模式 (通常是良好的实践)
    robot.setOperateMode(OperateMode::manual, ec_mode); 
    
    runGuidedPositioning(robot, camSN, armName, calibFile);

    return 0;
}
catch (const std::exception &e) {
    cerr << "💥 发生异常: " << e.what() << endl;
    return -1;
}