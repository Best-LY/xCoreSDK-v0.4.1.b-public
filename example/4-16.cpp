#include <thread>
#include <iostream>
#include <vector>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include "rokae/robot.h"

using namespace rokae;

// 全局变量
std::mutex mtx;
std::condition_variable cv;
bool stop_program = false;
bool stop_robots = false;
bool waiting_for_command = false;
bool pause_display = false; // 新增：控制显示暂停

template <typename T, size_t S>
std::ostream &operator<<(std::ostream &os, const std::array<T, S> &arr) {
    os << "[ ";
    for (size_t i = 0; i < S; ++i) {
        os << arr[i];
        if (i < S - 1) os << ", ";
    }
    os << " ]";
    return os;
}

template <size_t S>
std::array<double, S> radToDeg(const std::array<double, S> &rad) {
    std::array<double, S> deg;
    for (size_t i = 0; i < S; ++i)
        deg[i] = rad[i] * (180.0 / M_PI);
    return deg;
}

void waitRobot(BaseRobot *robot) {
    bool running = true;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        error_code ec;
        auto st = robot->operationState(ec);
        if (st == OperationState::idle || st == OperationState::unknown)
            running = false;
    }
}

void moveRobotToTarget(xMateRobot &robot, const std::vector<double> &target_pos) {
    error_code ec;
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    if (ec) {
        std::cerr << "Set operate mode failed: " << ec.message() << std::endl;
        return;
    }

    robot.setPowerState(true, ec);
    if (ec) {
        std::cerr << "Power on failed: " << ec.message() << std::endl;
        return;
    }

    // 新增：周期检查停止标志
    bool execute = true;
    while (execute && !stop_robots) {
        robot.executeCommand({MoveAbsJCommand(target_pos)}, ec);
        if (ec) {
            std::cerr << "Execute command failed: " << ec.message() << std::endl;
        } else {
            waitRobot(&robot);
        }
        std::lock_guard<std::mutex> lock(mtx);
        execute = !stop_robots;
    }
}

// 修改后的显示函数，同时显示两个机械臂状态
void displayJointAngles(xMateRobot &robot1, xMateRobot &robot2, 
                       const std::string &ip1, const std::string &ip2) {
    while (!stop_program) {
        { // 检查暂停状态
            std::unique_lock<std::mutex> lk(mtx);
            cv.wait_for(lk, std::chrono::milliseconds(50), []{
                return !pause_display || stop_program;
            });
            if (stop_program) break;
            if (pause_display) continue;
        }

        error_code ec1, ec2;
        auto pos1 = robot1.jointPos(ec1);
        auto pos2 = robot2.jointPos(ec2);
        
        if (!ec1 && !ec2) {
            std::lock_guard<std::mutex> lock(mtx);
            auto deg1 = radToDeg(pos1);
            auto deg2 = radToDeg(pos2);
            std::cout << "\r" << ip1 << ": " << deg1 
                      << " | " << ip2 << ": " << deg2 << "          " << std::flush;
        }
    }
    std::cout << "\n";
}

void inputListener(xMateRobot &robot1, xMateRobot &robot2,
                  std::vector<double> &home_pos1, std::vector<double> &home_pos2) {
    std::string input;
    error_code ec;

    while (true) {
        std::cin >> input;
        if (input == "q") {
            if (!waiting_for_command) {
                { // 第一次按q：暂停显示和运动
                    std::lock_guard<std::mutex> lock(mtx);
                    stop_robots = true;
                    waiting_for_command = true;
                    pause_display = true;
                    std::cout << "\nStopping robots..." << std::endl;
                }
                cv.notify_all();

                robot1.stop(ec);
                robot2.stop(ec);
                std::cout << "Press 'q' to exit or 'h' to home" << std::endl;
            } else { // 第二次按q：退出程序
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    stop_program = true;
                    std::cout << "Exiting..." << std::endl;
                }
                cv.notify_all();
                
                robot1.setPowerState(false, ec);
                robot2.setPowerState(false, ec);
                robot1.disconnectFromRobot(ec);
                robot2.disconnectFromRobot(ec);
                break;
            }
        } else if (input == "h" && waiting_for_command) {
            { // 处理home命令
                std::lock_guard<std::mutex> lock(mtx);
                waiting_for_command = false;
                pause_display = false;
                std::cout << "\nReturning to home..." << std::endl;
            }
            cv.notify_all();

            robot1.stop(ec);
            robot2.stop(ec);
            
            // 立即执行home命令
            robot1.executeCommand({MoveAbsJCommand(home_pos1)}, ec);
            robot2.executeCommand({MoveAbsJCommand(home_pos2)}, ec);
            
            if (!ec) {
                waitRobot(&robot1);
                waitRobot(&robot2);
                std::cout << "Home position reached." << std::endl;
            }
        }
    }
}

int main() {
    xMateRobot robot1, robot2;
    try {
        robot1.connectToRobot("192.168.2.161");
        robot2.connectToRobot("192.168.2.160");
    } catch (const Exception &e) {
        std::cerr << e.what();
        return 1;
    }

    std::vector<double> target1 = {0, -M_PI_2, -M_PI_2, -M_PI, M_PI_2, M_PI*0.75};
    std::vector<double> target2 = {M_PI, M_PI_2, M_PI_2, -M_PI, -M_PI_2, M_PI*1.25};
    std::vector<double> home1 = {0, -M_PI_2, 0, -M_PI, 0, M_PI*0.75};
    std::vector<double> home2 = {M_PI, M_PI_2, 0, -M_PI, 0, -M_PI*0.75};

    std::thread input_thread(inputListener, std::ref(robot1), std::ref(robot2),
                std::ref(home1), std::ref(home2));
    std::thread display_thread(displayJointAngles, std::ref(robot1), std::ref(robot2),
                "192.168.2.161", "192.168.2.160");
    std::thread robot1_thread(moveRobotToTarget, std::ref(robot1), std::ref(target1));
    std::thread robot2_thread(moveRobotToTarget, std::ref(robot2), std::ref(target2));

    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, []{ return stop_program; });
    }

    input_thread.join();
    robot1_thread.join();
    robot2_thread.join();
    display_thread.join();

    return 0;
}