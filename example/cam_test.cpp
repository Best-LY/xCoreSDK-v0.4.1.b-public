// #include <iostream>
// #include <thread>
// #include <chrono>
// #include <atomic>
// #include <mutex>
// #include <vector>
// #include <opencv2/opencv.hpp>
// #include <librealsense2/rs.hpp>

// using namespace std;
// using namespace cv;

// /* ========== 可调参数 ========== */

// constexpr int WIDTH = 640;
// constexpr int HEIGHT = 480;
// constexpr int FPS = 30;
// const cv::Size CHECKERBOARD_SIZE(7, 5);   // ✅ 改为 const
// constexpr double SQUARE_SIZE = 0.02;      // 单位: 米

// /* ============================== */



// std::atomic<bool> g_running{true};
// std::mutex g_mtx;

// /* ---------- 棋盘格角点生成 ---------- */
// std::vector<Point3f> createObjectPoints(Size patternSize, float squareSize) {
//     std::vector<Point3f> objectPoints;
//     for (int i = 0; i < patternSize.height; i++) {
//         for (int j = 0; j < patternSize.width; j++)
//             objectPoints.emplace_back(j * squareSize, i * squareSize, 0);
//     }
//     return objectPoints;
// }

// /* ---------- 主程序 ---------- */
// int main() try {
//     // 1️⃣ 初始化相机
//     rs2::context ctx;
//     auto devices = ctx.query_devices();
//     if (devices.size() == 0) {
//         cerr << "❌ 未检测到 RealSense 设备，请检查连接。\n";
//         return -1;
//     }

//     rs2::pipeline pipe;
//     rs2::config cfg;
//     cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);
//     cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);

//     cout << "✅ 正在启动 RealSense 流..." << endl;
//     auto profile = pipe.start(cfg);
//     rs2::colorizer color_map;
//     rs2::align align_to_color(RS2_STREAM_COLOR);

//     rs2::device dev = profile.get_device();
//     cout << "✅ 相机: " << dev.get_info(RS2_CAMERA_INFO_NAME)
//          << " | SN: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
//          << endl;

//     // 2️⃣ 打印使用说明
//     cout << "---------------------------------------------\n";
//     cout << " [SPACE] 保存当前图像\n";
//     cout << " [ESC]   退出程序\n";
//     cout << "---------------------------------------------\n";

//     // 3️⃣ 创建棋盘格世界坐标模板
//     auto obj_points = createObjectPoints(CHECKERBOARD_SIZE, SQUARE_SIZE);

//     // 4️⃣ 主循环
//     int img_id = 0;
//     while (g_running.load()) {
//         rs2::frameset frames = pipe.wait_for_frames();
//         frames = align_to_color.process(frames);

//         rs2::video_frame color_frame = frames.get_color_frame();
//         rs2::depth_frame depth_frame = frames.get_depth_frame();
//         rs2::frame depth_colorized = color_map.colorize(depth_frame);

//         Mat color(Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
//         Mat depth_c(Size(WIDTH, HEIGHT), CV_8UC3, (void*)depth_colorized.get_data(), Mat::AUTO_STEP);
//         Mat gray;
//         cvtColor(color, gray, COLOR_BGR2GRAY);

//         // 5️⃣ 查找棋盘格角点
//         vector<Point2f> corners;
//         bool found = findChessboardCorners(gray, CHECKERBOARD_SIZE, corners,
//                     CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
//         if (found) {
//             cornerSubPix(gray, corners, Size(11,11), Size(-1,-1),
//                          TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 30, 0.001));
//             drawChessboardCorners(color, CHECKERBOARD_SIZE, corners, found);
//             putText(color, "Checkerboard Detected", Point(10,40),
//                     FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,255,0), 2);
//         } else {
//             putText(color, "No Checkerboard", Point(10,40),
//                     FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,0,255), 2);
//         }

//         imshow("Color Stream", color);
//         imshow("Depth Stream", depth_c);
//         int key = waitKey(1);
//         if (key == 27) break; // ESC 退出
//         if (key == 32 && found) { // SPACE 保存
//             string prefix = "capture_" + to_string(img_id);
//             imwrite(prefix + "_color.png", color);
//             imwrite(prefix + "_depth.png", Mat(Size(WIDTH, HEIGHT), CV_16UC1, (void*)depth_frame.get_data()).clone());
//             FileStorage fs(prefix + "_corners.yml", FileStorage::WRITE);
//             fs << "image_points" << corners;
//             fs << "object_points" << obj_points;
//             fs.release();
//             cout << "✅ 已保存: " << prefix << "_{color,depth,corners}\n";
//             img_id++;
//         }
//     }

//     pipe.stop();
//     destroyAllWindows();
//     cout << "✅ 程序结束\n";
//     return 0;
// }
// catch (const rs2::error &e) {
//     cerr << "[RealSense ERROR] " << e.what() << " (" 
//          << e.get_failed_function() << ")\n";
//     return -1;
// }
// catch (const std::exception &e) {
//     cerr << "[Exception] " << e.what() << "\n";
//     return -1;
// }


#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal> // 用于捕获 Ctrl+C

using namespace std;
using namespace cv;

// 全局标志位，用于控制循环退出
bool keep_running = true;

// 捕获 Ctrl+C 信号，确保程序能执行到 pipe.stop()
void signal_handler(int signal) {
    keep_running = false;
}

// === 核心功能：强制复位相机 ===
// 解决 "Device already opened" 的杀手锏
void force_reset_device() {
    rs2::context ctx;
    if (ctx.query_devices().size() == 0) return;

    cout << ">>> 检测到设备，正在执行强制硬件复位..." << endl;
    cout << ">>> (如果报错 Device already opened，这一步会自动修好它)" << endl;

    try {
        for (auto&& dev : ctx.query_devices()) {
            // 发送硬件复位指令
            dev.hardware_reset();
        }
        // 复位后，相机相当于拔掉重插，需要等待它重新上线
        // 虚拟机里可能需要更久，这里给足 5 秒
        cout << ">>> 等待设备重启 (10秒)..." << endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
    catch (...) {
        cout << ">>> 复位指令发送失败，可能设备已经掉线，跳过。" << endl;
    }
}

int main() {
    // 注册 Ctrl+C 信号
    signal(SIGINT, signal_handler);

    // 1. 程序启动先复位，清除所有僵尸状态
    force_reset_device();

    rs2::pipeline pipe;
    rs2::config cfg;

    // 2. 使用你验证过的稳定配置 (USB 2.0 兼容模式)
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 15);

    // 3. 带有自动重试的启动逻辑
    bool connected = false;
    while (keep_running && !connected) {
        try {
            cout << "正在尝试连接相机..." << endl;
            pipe.start(cfg);
            connected = true;
            cout << ">>> 相机连接成功！(按 Ctrl+C 退出)" << endl;
        }
        catch (const rs2::error& e) {
            cerr << "连接失败: " << e.what() << endl;
            cerr << "将在 5 秒后重试..." << endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            // 如果一直连不上，再次尝试复位
            force_reset_device(); 
        }
    }

    if (!connected) return -1;

    // 4. 预热，丢弃前几帧（避免刚启动时的黑屏或曝光不足）
    for(int i=0; i<10; i++) pipe.wait_for_frames();

    rs2::colorizer color_map;

    while (keep_running) {
        try {
            // 超时时间设长一点 (10000ms)，防止虚拟机卡顿导致崩毁
            rs2::frameset frames = pipe.wait_for_frames(10000);

            auto color_frame = frames.get_color_frame();
            auto depth_frame = frames.get_depth_frame();

            if (!color_frame || !depth_frame) continue;

            Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
            auto depth_vis = color_map.colorize(depth_frame);
            Mat depth(Size(640, 480), CV_8UC3, (void*)depth_vis.get_data(), Mat::AUTO_STEP);

            imshow("Color", color);
            imshow("Depth", depth);

            if (waitKey(1) == 27) keep_running = false; // ESC 退出
        }
        catch (const rs2::error& e) {
            cerr << "运行时错误: " << e.what() << endl;
            break;
        }
    }

    // 5. 确保这句一定会被执行
    cout << "正在关闭相机..." << endl;
    try {
        pipe.stop();
    } catch (...) {}
    
    cout << "程序已安全退出。" << endl;
    return 0;
}