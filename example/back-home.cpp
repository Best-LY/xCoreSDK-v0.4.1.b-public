#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
void waitRobot(BaseRobot *robot);

int main() {
    using namespace rokae;
    xMateRobot robot1;
    xMateRobot robot2;
    try {
      robot1.connectToRobot("192.168.2.161");//左臂
      robot2.connectToRobot("192.168.2.160");//右臂
    } catch(const Exception &e) {
      std::cerr << e.what();
      return 1;
    }
    error_code ec;

    // 上电
    robot1.setOperateMode(rokae::OperateMode::automatic, ec);
    robot1.setPowerState(true, ec);

    robot2.setOperateMode(rokae::OperateMode::automatic, ec);
    robot2.setPowerState(true, ec);

    // 先运动到拖拽位姿, 注意选择正确的机型
    std::vector<double> drag_cr = {0, -M_PI/2, -M_PI/2, -M_PI, 0, M_PI*0.75};  //左臂
    std::vector<double> drag_er = {M_PI, M_PI/2, M_PI/2, -M_PI, 0, -M_PI*1.25};  //右臂

    // std::vector<double> drag_cr = {0, -M_PI_2, -M_PI_2, -M_PI, M_PI_2, M_PI*0.75};  //左臂参考点
    // std::vector<double> drag_er = {M_PI, M_PI_2, M_PI_2, -M_PI, -M_PI_2, -M_PI *1.25};  //右臂参考点

    robot1.executeCommand({MoveAbsJCommand(drag_cr)}, ec);
    robot2.executeCommand({MoveAbsJCommand(drag_er)}, ec);

    waitRobot(&robot1);
    waitRobot(&robot2);

    robot1.setPowerState(false, ec);
    robot1.setOperateMode(OperateMode::manual, ec);

    robot2.setPowerState(false, ec);
    robot2.setOperateMode(OperateMode::manual, ec);

    return 0;
  }

  /**
   * @brief 等待机器人静止
   * @param robot robot pointer
   */
  void waitRobot(BaseRobot *robot) {
    bool running = true;
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      error_code ec;
      auto st = robot->operationState(ec);
      if(st == OperationState::idle || st == OperationState::unknown){
        running = false;
      }
    }}

// #include <iostream>
// #include <thread>
// #include <chrono>
// #include <atomic>
// #include <vector>
// #include <mutex>

// #include "rokae/robot.h"
// #include "rokae/exception.h"
// #include "rokae/motion_control_rt.h"
// #include "rokae/utility.h"

// using namespace rokae;

// // 封装目标点位作为外部可修改的变量
// std::vector<double> targetJoints1 = {0, -M_PI_2, -M_PI_2, -M_PI, M_PI_2, M_PI * 5 / 18}; // 左臂
// std::vector<double> targetJoints2 = {M_PI, M_PI_2, M_PI_2, M_PI, -M_PI_2, M_PI * 1.25};  // 右臂

// // 整合显示位置信息和读取机械臂位置的函数
// void monitorRobotState(xMateRobot &robot, std::atomic<bool> &running, const std::string &robot_id)
// {
//   error_code ec;
//   using namespace RtSupportedFields;

//   try
//   {
//     // 设置数据发送间隔为100毫秒，接收机器人末端位姿、关节力矩和关节角度
//     std::array<double, 16> tcpPose{};
//     std::array<double, 6> arr6{};
//     std::mutex mtx;

//     // 启动状态数据接收
//     robot.startReceiveRobotState(std::chrono::milliseconds(100), {tcpPose_m, tau_m, jointPos_m});

//     // 清除旧数据
//     while (robot.updateRobotState(std::chrono::steady_clock::duration::zero()))
//       ;

//     while (running.load())
//     {
//       // 更新机器人状态数据
//       robot.updateRobotState(std::chrono::milliseconds(100));

//       // 读取机器人状态数据
//       robot.getStateData(tcpPose_m, tcpPose);
//       robot.getStateData(jointPos_m, arr6);

//       // 显示位置信息
//       std::lock_guard<std::mutex> lock(mtx);
//       std::cout << robot_id << " TCP pose: X=" << tcpPose[3] << ", Y=" << tcpPose[7] << ", Z=" << tcpPose[11]
//                 << " | Joints: ";
//       for (size_t i = 0; i < arr6.size(); ++i)
//       {
//         std::cout << arr6[i] << (i < arr6.size() - 1 ? ", " : "");
//       }
//       std::cout << std::endl;

//       std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }

//     robot.stopReceiveRobotState();
//   }
//   catch (const std::exception &e)
//   {
//     std::cerr << robot_id << " 状态监控异常: " << e.what() << std::endl;
//     running.store(false);
//   }
// }

// std::string ip1 = "192.168.2.161";
// std::string ip2 = "192.168.2.160";
// std::string localIp = "192.168.2.6";
// void contect(xMateRobot &robot, const std::string ip, std::string localIp, const std::string &robot_id)
// {
//   error_code ec;
//   robot.robotInfo(ec);
//   if (ec)
//   {
//     std::cerr << robot_id << " 机械臂未连接: " << ec.message() << std::endl;
//     return;
//   }
//   std::cout << robot_id << " 机械臂已连接" << std::endl;
// }

// void power_on(xMateRobot &robot,const std::string &robot_id)
// {
//   error_code ec;
//   PowerState powerState = robot.powerState(ec);
//   if (powerState != PowerState::on)
//   {
//     robot.setPowerState(true, ec);
//     if (ec)
//     {
//       std::cerr << robot_id << " 上电失败: " << ec.message() << std::endl;
//       return;
//     }
//     std::cout << robot_id << " 机械臂已上电" << std::endl;
//   }
//   else
//   {
//     std::cout << robot_id << " 机械臂已上电" << std::endl;
//   }
// }

// void discontect(xMateRobot &robot, const std::string ip, std::string localIp, const std::string &robot_id)
// {
//   error_code ec;
//   robot.disconnectFromRobot(ec);
//   if (ec)
//   {
//     std::cerr << robot_id << " 断开连接失败: " << ec.message() << std::endl;
//   }
//   else
//   {
//     std::cout << robot_id << " 已断开与机械臂的连接" << std::endl;
//   }
// }

// void controlSingleRobot(xMateRobot &robot, const std::vector<double> &targetJoints, std::atomic<bool> &running, const std::string &robot_id)
// {
//   try
//   {
//     error_code ec;
//     robot.robotInfo(ec);
//     if (ec)
//     {
//       std::cerr << robot_id << " 机械臂未连接: " << ec.message() << std::endl;
//       return;
//     }
//     std::cout << robot_id << " 机械臂已连接" << std::endl;

//     // 设置操作模式为自动
//     robot.setOperateMode(rokae::OperateMode::automatic, ec);
//     if (ec)
//     {
//       std::cerr << robot_id << " 设置自动模式失败: " << ec.message() << std::endl;
//       return;
//     }

//     // 上电
//     PowerState powerState = robot.powerState(ec);
//     if (powerState != PowerState::on)
//     {
//       robot.setPowerState(true, ec);
//       if (ec)
//       {
//         std::cerr << robot_id << " 上电失败: " << ec.message() << std::endl;
//         return;
//       }
//       std::cout << robot_id << " 机械臂已上电" << std::endl;
//     }
//     else
//     {
//       std::cout << robot_id << " 机械臂已上电" << std::endl;
//     }

//     // 设置非实时模式
//     robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
//     if (ec)
//     {
//       std::cerr << robot_id << " 设置非实时模式失败: " << ec.message() << std::endl;
//       return;
//     }

//     // 添加运动指令
//     MoveAbsJCommand cmd(targetJoints);
//     cmd.jointSpeed = 0.2;

//     std::string cmdID;
//     robot.moveAppend({cmd}, cmdID, ec);
//     if (ec)
//     {
//       std::cerr << robot_id << " 添加运动指令失败: " << ec.message() << std::endl;
//       return;
//     }

//     // 开始运动
//     robot.moveStart(ec);
//     if (ec)
//     {
//       std::cerr << robot_id << " 开始运动失败: " << ec.message() << std::endl;
//       return;
//     }

//     // 启动状态监控线程
//     std::thread stateThread(monitorRobotState, std::ref(robot), std::ref(running), robot_id);

//     // 等待运动完成
//     while (running.load())
//     {
//       std::this_thread::sleep_for(std::chrono::milliseconds(100));
//       if (!running.load())
//         break;
//     }

//     // 停止运动
//     robot.moveReset(ec);
//     if (ec)
//     {
//       std::cerr << robot_id << " 重置运动失败: " << ec.message() << std::endl;
//     }

//     // 设置为手动模式
//     robot.setOperateMode(rokae::OperateMode::manual, ec);

//     // 下电
//     robot.setPowerState(false, ec);
//     if (ec)
//     {
//       std::cerr << robot_id << " 下电失败: " << ec.message() << std::endl;
//     }
//     else
//     {
//       std::cout << robot_id << " 机械臂已下电" << std::endl;
//     }

//     // 断开连接
//     robot.disconnectFromRobot(ec);
//     if (ec)
//     {
//       std::cerr << robot_id << " 断开连接失败: " << ec.message() << std::endl;
//     }
//     else
//     {
//       std::cout << robot_id << " 已断开与机械臂的连接" << std::endl;
//     }

//     // 确保状态监控线程已退出
//     stateThread.join();
//   }
//   catch (const std::exception &e)
//   {
//     std::cerr << robot_id << " 控制机械臂时发生异常: " << e.what() << std::endl;
//     running.store(false);

//     error_code ec;
//     robot.setPowerState(false, ec);
//     robot.setOperateMode(rokae::OperateMode::manual, ec);
//     robot.disconnectFromRobot(ec);
//   }
// }

// int main()
// {
//   error_code ec;
//   try
//   {
//     std::string ip1 = "192.168.2.161";
//     std::string ip2 = "192.168.2.160";
//     std::string localIp = "192.168.2.6";

//     xMateRobot robot1(ip1, localIp);
//     xMateRobot robot2(ip2, localIp);

//     std::atomic<bool> running1(true);
//     std::atomic<bool> running2(true);

//     std::thread controlThread1(controlSingleRobot, std::ref(robot1), std::ref(targetJoints1), std::ref(running1), "Robot 1");
//     std::thread controlThread2(controlSingleRobot, std::ref(robot2), std::ref(targetJoints2), std::ref(running2), "Robot 2");

//     controlThread1.join();
//     controlThread2.join();

//     running1.store(false);
//     running2.store(false);

//     robot1.setPowerState(false, ec);
//     robot1.setOperateMode(rokae::OperateMode::manual, ec);
//     robot2.setPowerState(false, ec);
//     robot2.setOperateMode(rokae::OperateMode::manual, ec);
//   }
//   catch (const std::exception &e)
//   {
//     std::cerr << "主程序异常: " << e.what() << std::endl;
//   }
//   return 0;
// }