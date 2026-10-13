#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

// RealSense
#include <librealsense2/rs.hpp>

// Rokae SDK
#include "rokae/robot.h"
#include "rokae/utility.h"

using namespace std;
using namespace cv;
using namespace rokae;

/* ================= 用户配置区域 (请务必修改此处) ================= */

// 🤖 1. 机械臂 IP 配置
const string IP_LEFT  = "192.168.2.160";
const string IP_RIGHT = "192.168.2.161";

// 📷 2. 相机序列号配置 (必须填写，否则无法区分左右相机)
// 请使用 realsense-viewer 查看 Device Info 获取序列号
const string SN_LEFT_CAM  = "000000000000"; // <--- 替换为左臂相机的序列号
const string SN_RIGHT_CAM = "000000000000"; // <--- 替换为右臂相机的序列号

// 🏁 3. 标定板参数
const Size BOARD_SIZE(9, 6);      // 内角点数量 (宽, 高)
const float SQUARE_SIZE = 0.020f; // 格子边长 (米)

// 📏 4. 运动参数
const double STEP_TRANS = 0.04; // 平移步长 4cm
const double STEP_ROT   = 0.10; // 旋转步长 ~6度
const double VELOCITY   = 30.0; // 运动速度 %

/* ============================================================= */

/**
 * @brief 简单的结构体用于返回标定结果
 */
struct CalibResult {
    Mat R; // 3x3
    Mat t; // 3x1
    Mat H; // 4x4
};

// 工具：等待机械臂稳定
void waitRobotIdle(BaseRobot &robot) {
    int stable_count = 0;
    while (stable_count < 5) { // 连续检测到多次空闲才算稳
        this_thread::sleep_for(chrono::milliseconds(200));
        error_code ec;
        auto st = robot.operationState(ec);
        if (st == OperationState::idle || st == OperationState::unknown) {
            stable_count++;
        } else {
            stable_count = 0;
        }
    }
}

// 工具：线性移动封装
bool moveLTo(xMateRobot& robot, const CartesianPosition& target) {
    error_code ec;
    // 使用 NrtCommand (非实时指令)
    MoveLCommand cmd(target, VELOCITY, 0); 
    robot.executeCommand({cmd}, ec);
    if (ec) {
        cerr << "❌ 移动指令发送失败: " << ec.message() << endl;
        return false;
    }
    // 等待运动结束
    waitRobotIdle(robot);
    return true;
}

// 工具：计算相对位姿
CartesianPosition applyDelta(const CartesianPosition& ref, const array<double,3>& dxyz, const array<double,3>& drpy) {
    CartesianPosition out = ref;
    // 简单叠加 (注意：对于大旋转这样直接叠加RPY不严谨，但小步长微扰做标定足够了)
    out.trans[0] += dxyz[0]; out.trans[1] += dxyz[1]; out.trans[2] += dxyz[2];
    out.rpy[0]   += drpy[0]; out.rpy[1]   += drpy[1]; out.rpy[2]   += drpy[2];
    return out;
}

// 工具：Rokae RPY (ZYX) 转 OpenCV 旋转矩阵
Mat rpyToRotationMatrix(const array<double, 3>& rpy) {
    Eigen::Matrix3d rot;
    // Rokae SDK通常定义 RPY 为 Z-Y-X 欧拉角 (alpha, beta, gamma)
    // 对应 rpy[2], rpy[1], rpy[0]
    Eigen::Vector3d euler(rpy[2], rpy[1], rpy[0]); 
    Utils::eulerToMatrix(euler, rot);
    
    Mat R(3, 3, CV_64F);
    for(int i=0; i<3; i++) 
        for(int j=0; j<3; j++) 
            R.at<double>(i,j) = rot(i,j);
    return R;
}

// 📷 获取相机内参
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

/**
 * @brief 单臂标定核心流程
 * @param robot 机械臂对象
 * @param cameraSN 该机械臂对应的相机序列号
 * @param armName 机械臂名称(用于打印和保存)
 */
void calibrateSingleArm(xMateRobot& robot, const string& cameraSN, const string& armName) {
    cout << "\n========================================" << endl;
    cout << "🚀 开始标定: " << armName << " (相机SN: " << cameraSN << ")" << endl;
    cout << "========================================" << endl;

    // --- 1. 初始化相机 ---
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_device(cameraSN); // 关键：指定开启哪台相机
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    
    try {
        pipe.start(cfg);
        cout << "📷 相机已连接，预热中..." << endl;
        std::this_thread::sleep_for(chrono::seconds(2));
    } catch (const rs2::error& e) {
        cerr << "❌ 无法打开相机 (" << cameraSN << "): " << e.what() << endl;
        return;
    }

    // --- 2. 获取初始位姿 ---
    error_code ec;
    auto ref_pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
    if(ec) { cerr << "❌ 无法获取机械臂初始位姿！" << endl; return; }

    cout << "📍 初始位姿记录成功。开始自动采集运动..." << endl;

    // --- 3. 准备数据 ---
    vector<Mat> R_gripper2base, t_gripper2base;
    vector<Mat> R_target2cam, t_target2cam;

    // 获取内参
    auto profile = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    Mat distCoeffs, K;
    K = getIntrinsics(profile, distCoeffs);

    // 标定板物理坐标
    vector<Point3f> objPoints;
    for(int r=0; r<BOARD_SIZE.height; r++)
        for(int c=0; c<BOARD_SIZE.width; c++)
            objPoints.push_back(Point3f(c * SQUARE_SIZE, r * SQUARE_SIZE, 0));

    // --- 4. 轨迹定义 (1个中心点 + 8个扰动点) ---
    vector<pair<array<double,3>, array<double,3>>> deltas = {
        {{0,0,0}, {0,0,0}},
        {{STEP_TRANS, 0, 0}, {0, STEP_ROT, 0}},
        {{-STEP_TRANS, 0, 0}, {0, -STEP_ROT, 0}},
        {{0, STEP_TRANS, 0}, {STEP_ROT, 0, 0}},
        {{0, -STEP_TRANS, 0}, {-STEP_ROT, 0, 0}},
        {{0, 0, STEP_TRANS}, {0, 0, STEP_ROT}},  // 拉远
        {{0, 0, -STEP_TRANS}, {0, 0, -STEP_ROT}}, // 拉近
        {{STEP_TRANS/2, STEP_TRANS/2, 0}, {0,0,0}},
        {{-STEP_TRANS/2, -STEP_TRANS/2, 0}, {0,0,0}}
    };

    int success_count = 0;

    // --- 5. 采集循环 ---
    for(size_t i=0; i<deltas.size(); i++) {
        cout << "\n🔄 [" << armName << "] 步骤 " << i+1 << "/" << deltas.size() << " 移动中..." << endl;
        
        CartesianPosition target = applyDelta(ref_pose, deltas[i].first, deltas[i].second);
        if(!moveLTo(robot, target)) {
            cout << "⚠️ 移动跳过" << endl;
            continue;
        }

        std::this_thread::sleep_for(chrono::milliseconds(500)); // 再次确保稳如老狗

        // 拍照并检测
        rs2::frameset frames = pipe.wait_for_frames();
        Mat image(Size(640, 480), CV_8UC3, (void*)frames.get_color_frame().get_data(), Mat::AUTO_STEP);
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);

        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, BOARD_SIZE, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

        if(found) {
            cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 30, 0.01));
            drawChessboardCorners(image, BOARD_SIZE, corners, found);

            // SolvePnP (使用 ITERATIVE 提高精度)
            Mat rvec, tvec;
            solvePnP(objPoints, corners, K, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);

            // 存储视觉位姿
            Mat R;
            Rodrigues(rvec, R);
            R_target2cam.push_back(R);
            t_target2cam.push_back(tvec);

            // 存储机械臂位姿 (实时获取更准)
            auto curr_pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
            R_gripper2base.push_back(rpyToRotationMatrix(curr_pose.rpy));
            Mat t = (Mat_<double>(3,1) << curr_pose.trans[0], curr_pose.trans[1], curr_pose.trans[2]);
            t_gripper2base.push_back(t);

            success_count++;
            cout << "✅ 检测成功!" << endl;
        } else {
            cout << "⚠️ 未检测到标定板，跳过此点。" << endl;
        }

        imshow("Robot View", image);
        waitKey(100);
    }

    // 回到原位，并在计算前关闭相机释放资源
    moveLTo(robot, ref_pose);
    pipe.stop();
    destroyAllWindows();

    // --- 6. 计算结果 ---
    if(success_count < 5) {
        cerr << "❌ 有效点数过少 (" << success_count << ")，标定失败！" << endl;
        return;
    }

    Mat R_cam2gripper, t_cam2gripper;
    // Eye-in-Hand 标定
    calibrateHandEye(R_gripper2base, t_gripper2base, 
                     R_target2cam, t_target2cam, 
                     R_cam2gripper, t_cam2gripper, 
                     CALIB_HAND_EYE_TSAI);

    // 构建 4x4 齐次矩阵
    Mat H_final = Mat::eye(4, 4, CV_64F);
    R_cam2gripper.copyTo(H_final(Range(0, 3), Range(0, 3)));
    t_cam2gripper.copyTo(H_final(Range(0, 3), Range(3, 4)));

    // --- 7. 保存与打印 ---
    string filename = armName + "_calibration.yml";
    FileStorage fs(filename, FileStorage::WRITE);
    fs << "R" << R_cam2gripper;
    fs << "t" << t_cam2gripper;
    fs << "H_Flange2Cam" << H_final;
    fs.release();

    cout << "\n🎉 [" << armName << "] 标定完成!" << endl;
    cout << "---------------------------------" << endl;
    cout << "H_Flange2Cam:\n" << H_final << endl;
    cout << "---------------------------------" << endl;
    cout << "结果已保存至: " << filename << endl;
}

// 辅助：连接并设置机械臂
void connectAndSetupRobot(xMateRobot& robot, const string& ip) {
    error_code ec;
    // 连接由构造函数完成，这里设置模式
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot.setDefaultSpeed(VELOCITY, ec); // 设置全局速度比例
}

int main() try {
    // 检查配置
    if (SN_LEFT_CAM == "000000000000" || SN_RIGHT_CAM == "000000000000") {
        cerr << "❌ 错误：请先在代码顶部的配置区域填入真实的相机序列号！" << endl;
        return -1;
    }

    cout << "========================================\n";
    cout << "   双臂双相机 Eye-in-Hand 自动标定程序   \n";
    cout << "========================================\n";
    cout << "1. 仅标定左臂 (Left Arm)\n";
    cout << "2. 仅标定右臂 (Right Arm)\n";
    cout << "3. 自动依次标定双臂 (Both)\n";
    cout << "请输入选项 (1-3): ";
    
    int choice;
    cin >> choice;

    // --- 标定左臂 ---
    if (choice == 1 || choice == 3) {
        cout << "\n🔌 正在连接左臂 (" << IP_LEFT << ")..." << endl;
        xMateRobot leftBot(IP_LEFT);
        connectAndSetupRobot(leftBot, IP_LEFT);

        cout << "\n🔴 [操作指引] 请手动将 <左臂> 移动到能看清标定板的初始位置。" << endl;
        cout << "   (确保该位置周围有 5-10cm 的安全活动空间)" << endl;
        cout << "👉 准备好后，按回车键开始..." << endl;
        cin.ignore(); cin.get();

        calibrateSingleArm(leftBot, SN_LEFT_CAM, "LeftArm");
        
        // 标定完左臂，如果是模式3，可以让它回个安全点或者下电，防止干扰
        // leftBot.setPowerState(false, ec); 
    }

    // --- 标定右臂 ---
    if (choice == 2 || choice == 3) {
        cout << "\n🔌 正在连接右臂 (" << IP_RIGHT << ")..." << endl;
        xMateRobot rightBot(IP_RIGHT);
        connectAndSetupRobot(rightBot, IP_RIGHT);

        cout << "\n🔵 [操作指引] 请手动将 <右臂> 移动到能看清标定板的初始位置。" << endl;
        cout << "   (注意：标定板不要移动！)" << endl;
        cout << "👉 准备好后，按回车键开始..." << endl;
        // 如果是模式3，cin.get可能读到上一个换行，需要处理一下
        if (choice != 2) cout << "(按回车继续...)" << endl;
        cin.get(); 

        calibrateSingleArm(rightBot, SN_RIGHT_CAM, "RightArm");
    }

    return 0;
}
catch (const rs2::error &e) {
    cerr << "❌ RealSense Error: " << e.what() << endl;
    cerr << "   (如果是 Device not connected，请检查序列号是否填错)" << endl;
    return -1;
}
catch (const std::exception &e) {
    cerr << "❌ System Error: " << e.what() << endl;
    return -1;
}