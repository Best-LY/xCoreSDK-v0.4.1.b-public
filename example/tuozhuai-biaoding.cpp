#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

// RealSense
#include <librealsense2/rs.hpp>

// Rokae SDK
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "rokae/data_types.h" 

using namespace std;
using namespace cv;
using namespace rokae;

/* ================= 用户配置区域 ================= */
// 🤖 机械臂 IP (已根据您的反馈修正：IP和SN的对应关系已对调，确保选择1时连到的是物理左臂)
// 请确认: 192.168.2.161 对应物理左臂; 192.168.2.160 对应物理右臂。
const string IP_LEFT  = "192.168.2.161"; 
const string IP_RIGHT = "192.168.2.160";

// 📷 相机序列号 (已根据您的反馈更新)
const string SN_LEFT_CAM  = "236422072262"; // 对应物理左臂相机
const string SN_RIGHT_CAM = "236422070786"; // 对应物理右臂相机

// 🏁 标定板参数
const Size BOARD_SIZE(8, 6);      // 内角点 (宽, 高)
const float SQUARE_SIZE = 0.0286f; // 28.6 mm, user-confirmed 2026-09-29
/* ============================================== */

// 辅助：Rokae RPY 转 旋转矩阵
Mat rpyToRotationMatrix(const array<double, 3>& rpy) {
    Eigen::Matrix3d rot;
    // SDK 默认 ZYX 顺序
    Eigen::Vector3d euler(rpy[2], rpy[1], rpy[0]); 
    Utils::eulerToMatrix(euler, rot);
    Mat R(3, 3, CV_64F);
    for(int i=0; i<3; i++) 
        for(int j=0; j<3; j++) 
            R.at<double>(i,j) = rot(i,j);
    return R;
}

// 辅助：获取相机内参
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
 * @brief 交互式拖拽标定核心逻辑
 */
void runInteractiveCalibration(xMateRobot& robot, const string& cameraSN, const string& armName) {
    error_code ec; 
    rs2::pipeline pipe;
    Mat distCoeffs, K;
    int cam_width = 0;
    int cam_height = 0;

    // --- 1. 开启相机 (使用默认分辨率，提高虚拟机兼容性) ---
    cout << "[" << armName << "] 正在开启相机 SN: " << cameraSN << " ..." << endl;
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
        cerr << "❌ 相机启动失败，请检查序列号是否正确或USB连接: " << e.what() << endl;
        return;
    }

    // 准备标定板物理坐标
    vector<Point3f> objPoints;
    for(int r=0; r<BOARD_SIZE.height; r++)
        for(int c=0; c<BOARD_SIZE.width; c++)
            objPoints.push_back(Point3f(c * SQUARE_SIZE, r * SQUARE_SIZE, 0));

    // --- 2. 机器人上电并开启拖拽模式 ---
    cout << "[" << armName << "] 正在上电..." << endl;
    
    robot.enableDrag(DragParameter::Space::cartesianSpace, DragParameter::Type::freely, ec); 
    
    if(ec) {
        cerr << "❌ 无法开启拖拽模式: " << ec.message() << endl;
        // 拖拽失败，尝试下电
        robot.setPowerState(false, ec); 
        pipe.stop();
        return;
    }
    
    cout << "\n✅ 拖拽模式已激活！" << endl;
    cout << "🎮 操作指南: " << endl;
    cout << "   1. 用手拖动机械臂，使相机看到标定板。" << endl;
    cout << "   2. **务必变换角度** (旋转手腕)，不要只平移。" << endl;
    cout << "   3. 按【空格键】采集当前点。" << endl;
    cout << "   4. 采集 15 个点以上后，按【Q】或【ESC】结束并计算。" << endl;

    // 数据容器
    vector<Mat> R_gripper2base, t_gripper2base;
    vector<Mat> R_target2cam, t_target2cam;
    int capture_count = 0;

    // --- 3. 交互循环 ---
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
        string status_text = "Points: " + to_string(capture_count);
        Scalar status_color = Scalar(0, 255, 0); // 绿色

        if (found) {
            drawChessboardCorners(display, BOARD_SIZE, corners, found);
            cv::putText(display, "READY TO CAPTURE", Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
        } else {
            status_color = Scalar(0, 0, 255); // 红色
            cv::putText(display, "NO BOARD DETECTED", Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
        }
        
        cv::putText(display, status_text, Point(20, 80), FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2);
        cv::putText(display, "[Space]:Capture  [Q]:Finish", Point(20, 450), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);

        imshow("Hand-Eye Calibration (Drag Mode)", display);

        // 按键处理
        int key = waitKey(30);

        // ---> 采集逻辑 <---
        if (key == 32) { // Space 键
            if (!found) {
                cout << "⚠️ 没看到标定板，无法采集！" << endl;
                continue;
            }

            // 1. 亚像素优化
            cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 30, 0.01));

            // 2. SolvePnP 计算视觉位姿
            Mat rvec, tvec;
            solvePnP(objPoints, corners, K, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
            Mat R_vis;
            Rodrigues(rvec, R_vis);

            // 3. 立即获取机械臂当前位姿
            auto current_pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
            if (ec) {
                cerr << "⚠️ 获取机械臂位姿失败: " << ec.message() << endl;
                continue;
            }

            // 4. 保存数据
            R_target2cam.push_back(R_vis);
            t_target2cam.push_back(tvec);

            R_gripper2base.push_back(rpyToRotationMatrix(current_pose.rpy));
            Mat t_robot = (Mat_<double>(3,1) << current_pose.trans[0], current_pose.trans[1], current_pose.trans[2]);
            t_gripper2base.push_back(t_robot);

            capture_count++;
            cout << "✅ 采集第 " << capture_count << " 个点成功！" << endl;

            // 视觉反馈 (闪一下白屏)
            Mat flash = Mat::ones(display.size(), display.type()) * 255;
            imshow("Hand-Eye Calibration (Drag Mode)", flash);
            waitKey(50);
        }
        // ---> 退出逻辑 <---
        else if (key == 'q' || key == 'Q' || key == 27) { // Q 或 ESC
            break;
        }
    }

    // --- 4. 停止拖拽 & 关闭资源 ---
    cout << "正在退出拖拽模式..." << endl;
    robot.disableDrag(ec); 
    
    // 下电
    robot.setPowerState(false, ec);

    pipe.stop();
    destroyAllWindows();

    // --- 5. 计算标定结果 ---
    if (capture_count < 5) {
        cerr << "❌ 采集点数太少 (" << capture_count << ")，无法计算！建议至少采集10个点。" << endl;
        return;
    }

    cout << "\n正在计算手眼矩阵..." << endl;
    Mat R_cam2gripper, t_cam2gripper;
    
    calibrateHandEye(R_gripper2base, t_gripper2base, 
                     R_target2cam, t_target2cam, 
                     R_cam2gripper, t_cam2gripper, 
                     CALIB_HAND_EYE_TSAI);

    // 组合成 4x4 矩阵
    Mat H = Mat::eye(4, 4, CV_64F);
    R_cam2gripper.copyTo(H(Range(0, 3), Range(0, 3)));
    t_cam2gripper.copyTo(H(Range(0, 3), Range(3, 4)));

    // 保存结果
    string filename = armName + "_interactive_calib.yml";
    FileStorage fs(filename, FileStorage::WRITE);
    fs << "H_Flange2Cam" << H;
    fs << "R" << R_cam2gripper;
    fs << "t" << t_cam2gripper;
    fs << "camera_serial" << cameraSN;
    fs << "board_width" << BOARD_SIZE.width;
    fs << "board_height" << BOARD_SIZE.height;
    fs << "square_size_m" << static_cast<double>(SQUARE_SIZE);
    fs.release();

    cout << "\n🎉 标定成功！" << endl;
    cout << "结果矩阵 H (Cam in Flange):\n" << H << endl;
    cout << "已保存至: " << filename << endl;
}

int main() try {
    cout << "Checkerboard: " << BOARD_SIZE.width << " x " << BOARD_SIZE.height
         << " inner corners, square size: " << SQUARE_SIZE * 1000.0f
         << " mm" << endl;
    // 检查配置
    if (SN_LEFT_CAM == "000000000000" || SN_RIGHT_CAM == "000000000000") {
        // 由于用户已提供，这个检查可以忽略，但保留作为安全检查
        // cerr << "⚠️ 警告：相机序列号未配置。如果程序找不到相机，请检查配置！" << endl;
    }

    cout << "========================================\n";
    cout << "   Rokae 交互式拖拽手眼标定 (Drag & Teach)   \n";
    cout << "========================================\n";
    cout << "1. 标定左臂 (Left Arm)\n";
    cout << "2. 标定右臂 (Right Arm)\n";
    cout << "请输入选项: ";
    
    int choice;
    if (!(cin >> choice)) return -1;

    if (choice == 1) {
        cout << "连接左臂: " << IP_LEFT << " ..." << endl;
        xMateRobot robot(IP_LEFT);
        
        error_code ec_mode; 
        // 【修正】设置模式为手动 (manual) 确保上电和拖拽能成功
        robot.setOperateMode(OperateMode::manual, ec_mode);
        if(ec_mode) { cerr << "❌ 切换到手动模式失败: " << ec_mode.message() << endl; return -1; }
        
        runInteractiveCalibration(robot, SN_LEFT_CAM, "LeftArm");
    } 
    else if (choice == 2) {
        cout << "连接右臂: " << IP_RIGHT << " ..." << endl;
        xMateRobot robot(IP_RIGHT);
        
        error_code ec_mode; 
        // 【修正】设置模式为手动 (manual) 确保上电和拖拽能成功
        robot.setOperateMode(OperateMode::manual, ec_mode);
        if(ec_mode) { cerr << "❌ 切换到手动模式失败: " << ec_mode.message() << endl; return -1; }
        
        runInteractiveCalibration(robot, SN_RIGHT_CAM, "RightArm");
    } else {
        cout << "输入无效。" << endl;
        return -1;
    }

    return 0;
}
catch (const std::exception &e) {
    cerr << "💥 发生异常: " << e.what() << endl;
    return -1;
}
