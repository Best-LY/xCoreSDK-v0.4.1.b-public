// /**
//  * @file test_rt_control_two_robots.cpp
//  * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
//  * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
//  * And may contains trade secrets that must be stored and viewed confidentially.
//  */

//  #include <iostream>
//  #include <fstream>
//  #include <atomic>
//  #include <thread>
 
//  #include "rokae/robot.h"
//  #include "rokae/utility.h"
//  #include "print_helper.hpp"
 
//  using namespace rokae;
//  int testRtTwoRobot(xMateRobot &robot1, xMateRobot &robot2);
 
//  int main() {
//    xMateRobot robot1, robot2;
//    try {
//      robot1.connectToRobot("192.168.2.160", "192.168.2.6");
//      robot2.connectToRobot("192.168.2.161", "192.168.2.6");
//    } catch(const rokae::Exception &e) {
//      std::cerr << "连接失败" << e.what();
//      return 1;
//    }
 
//    testRtTwoRobot(robot1, robot2);
 
//    return 0;
//  }
 
//  void configRt(xMateRobot &robot, std::shared_ptr<RtMotionControlCobot<6>> &rtCon) {
//    error_code ec;
//    robot.setRtNetworkTolerance(30, ec);
 
//    robot.setOperateMode(rokae::OperateMode::automatic, ec);
//    robot.setPowerState(true, ec);
//    robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
//    rtCon = robot.getRtMotionController().lock();
//  }
 
//  void deInit(xMateRobot &robot) {
//    error_code ec;
//    robot.setPowerState(false, ec);
//    robot.setMotionControlMode(MotionControlMode::Idle, ec);
//  }
 
//  int testRtTwoRobot(xMateRobot &robot1, xMateRobot &robot2) {
//    error_code ec;
//    std::shared_ptr<RtMotionControlCobot<6>> rtCon1, rtCon2;
//    configRt(robot1, rtCon1);
//    configRt(robot2, rtCon2);
 
//    std::array<double, 6> jntPos1{}, jntPos2{},
//      q_drag_er = {0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0},
//      q_drag_er2 = {-M_PI/6, M_PI/6, -M_PI_2, 0, -M_PI/3, 0},
//      jnt_read{};
 
//    std::array<double, 16> cartPos {};
 
//    std::atomic_int finished = 2;
//    double time1 = 0, time2 = 0, time_cart = 0;
 
//    std::function < JointPosition() > callback1 = [&time1, &jntPos1, &finished]() {
 
//      time1 += 0.001;
//      double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI / 2.5 * time1));
//      JointPosition cmd = {{jntPos1[0] + delta_angle, 
//                            jntPos1[1] + delta_angle,
//                            jntPos1[2] - delta_angle,
//                            jntPos1[3] + delta_angle, 
//                            jntPos1[4] - delta_angle,
//                            jntPos1[5] + delta_angle}};
 
//      if (time1 > 200) {
//        cmd.setFinished();
//        finished.fetch_sub(1);
//      }
//      return cmd;
//    };
 
//    std::function < JointPosition() > callback2 = [&time2, &jntPos2, &finished]() {
//      time2 += 0.001;
//      double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI / 2.5 * time2));
//      JointPosition cmd = {{jntPos2[0] - delta_angle,
//                            jntPos2[1] + delta_angle,
//                            jntPos2[2] - delta_angle,
//                            jntPos2[3] + delta_angle, 
//                            jntPos2[4] - delta_angle,
//                            jntPos2[5] + delta_angle}};
 
//      if (time2 > 200) {
//        cmd.setFinished();
//        finished.fetch_sub(1);
//      }
 
//      return cmd;
//    };
 
//    std::function<CartesianPosition(void)> callback_cart = [&robot2, &time_cart, &cartPos, &finished]()->CartesianPosition{
 
//      time_cart += 0.001;
//      constexpr double kRadius = 0.2;
//      double angle = M_PI / 4 * (1 - std::cos(M_PI / 2 * time_cart));
//      double delta_z = kRadius * (std::cos(angle) - 1);
 
//      CartesianPosition output{};
//      output.pos = cartPos;
//      output.pos[7] += delta_z;
 
//      if(time_cart > 20){
//        output.setFinished(); // 60秒后结束
//        finished.fetch_sub(1);
//      }
//      return output;
//    };
 
//    // 从当前位置MoveJ运动到拖拽位姿
//    try {
//      std::cout << "Robot1 start MoveJ" << std::endl;
//      rtCon1->MoveJ(0.4, robot1.jointPos(ec), q_drag_er);
//      std::cout << "Robot2 start MoveJ" << std::endl;
//      rtCon2->MoveJ(0.4, robot2.jointPos(ec), q_drag_er2);
//    } catch (const std::exception &e) {
//      std::cerr << __LINE__ << e.what() << std::endl;
//      return 1;
//    }
 
//    // 开始轴空间位置控制
//    try {
//      rtCon1->setControlLoop(callback1, 0, false);
//      jntPos1 = robot1.jointPos(ec);
//      rtCon1->startMove(RtControllerMode::jointPosition);
 
//      rtCon2->setControlLoop(callback2, 0, false);
//      jntPos2 = robot2.jointPos(ec);
//      rtCon2->startMove(RtControllerMode::jointPosition);
 
//  //    rtCon2->setControlLoop(callback_cart, 0, false);
//  //    Utils::postureToTransArray(robot2.posture(rokae::CoordinateType::flangeInBase, ec), cartPos);
//  //    rtCon2->startMove(RtControllerMode::cartesianPosition);
 
//    } catch (const std::exception &e) {
//      std::cerr << "Fail start move " << e.what() << std::endl;
//      return 1;
//    }
 
//    // 非阻塞loop
//    try {
//      rtCon1->startLoop(false);
//    } catch(const std::exception &e) {
//      std::cerr << "xmate moving error " << e.what() << std::endl;
//      finished.fetch_sub(1);
//    }
 
//    try {
//      rtCon2->startLoop(true);
//    } catch(const std::exception &e) {
//      std::cerr << "xmate moving error " << e.what() << std::endl;
//      finished.fetch_sub(1);
//    }
 
//    while (finished.load() > 0) {
//      std::this_thread::sleep_for(std::chrono::milliseconds(500));
//    }
//    std::cout << "stop loop" << std::endl;
//    try {
//      rtCon1->stopLoop();
//    } catch (const std::exception &e) {
//      std::cerr << "rtCon " << e.what() << std::endl;
//    }
 
//    deInit(robot1);
//    deInit(robot2);
 
//    return 0;
//  }



 #include <iostream>
 #include <fstream>
 #include <atomic>
 #include <thread>
 
 #include "rokae/robot.h"
 #include "rokae/utility.h"
 #include "print_helper.hpp"
 
 using namespace rokae;
 
 int testRtTwoRobot(xMateRobot &robot1, xMateRobot &robot2);
 void configRt(xMateRobot &robot, std::shared_ptr<RtMotionControlCobot<6>> &rtCon);
 void deInit(xMateRobot &robot);
 
 int main() {
   xMateRobot robot1, robot2;
   try {
     robot1.connectToRobot("192.168.2.160", "192.168.2.6");
     robot2.connectToRobot("192.168.2.161", "192.168.2.6");
   } catch(const rokae::Exception &e) {
     std::cerr << "连接失败" << e.what();
     return 1;
   }
 
   try {
     testRtTwoRobot(robot1, robot2);
   } catch(const std::exception &e) {
     std::cerr << "测试异常：" << e.what() << std::endl;
     deInit(robot1);
     deInit(robot2);
     return 1;
   }
 
   return 0;
 }
 
 void configRt(xMateRobot &robot, std::shared_ptr<RtMotionControlCobot<6>> &rtCon) {
   error_code ec;
   robot.setRtNetworkTolerance(30, ec);
   if(ec) throw std::runtime_error("设置网络容错失败: " + ec.message());
 
   robot.setOperateMode(rokae::OperateMode::automatic, ec);
   if(ec) throw std::runtime_error("设置操作模式失败: " + ec.message());
 
   robot.setPowerState(true, ec);
   if(ec) throw std::runtime_error("上电失败: " + ec.message());
 
   robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
   if(ec) throw std::runtime_error("设置运动控制模式失败: " + ec.message());
 
   rtCon = robot.getRtMotionController().lock();
   if(!rtCon) throw std::runtime_error("获取实时控制器失败");
 }
 
 void deInit(xMateRobot &robot) {
   error_code ec;
   robot.setPowerState(false, ec);
   robot.setMotionControlMode(MotionControlMode::Idle, ec);
 }
 
 int testRtTwoRobot(xMateRobot &robot1, xMateRobot &robot2) {
   error_code ec;
   std::shared_ptr<RtMotionControlCobot<6>> rtCon1, rtCon2;
   configRt(robot1, rtCon1);
   configRt(robot2, rtCon2);
 
   std::array<double, 6> jntPos1{}, jntPos2{},
     q_drag_er = {0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0},
     q_drag_er2 = {-M_PI/6, M_PI/6, -M_PI_2, 0, -M_PI/3, 0};
 
   std::array<double, 16> cartPos{};
   std::atomic_int finished = 2;
   double time1 = 0, time2 = 0;
 
   // 获取当前位置后再设置回调
   jntPos1 = robot1.jointPos(ec);
   if(ec) throw std::runtime_error("获取robot1关节位置失败: " + ec.message());
   
   jntPos2 = robot2.jointPos(ec);
   if(ec) throw std::runtime_error("获取robot2关节位置失败: " + ec.message());
 
   std::function <JointPosition()> callback1 = [&time1, &jntPos1, &finished]() {
     time1 += 0.001;
     double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI / 2.5 * time1));
     JointPosition cmd = {{
       jntPos1[0] + delta_angle, 
       jntPos1[1] + delta_angle,
       jntPos1[2] - delta_angle,
       jntPos1[3] + delta_angle, 
       jntPos1[4] - delta_angle,
       jntPos1[5] + delta_angle
     }};
 
     if(time1 > 5) { // 缩短测试时间至5秒
       cmd.setFinished();
       finished.fetch_sub(1);
     }
     return cmd;
   };
 
   std::function <JointPosition()> callback2 = [&time2, &jntPos2, &finished]() {
     time2 += 0.001;
     double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI / 2.5 * time2));
     JointPosition cmd = {{
       jntPos2[0] - delta_angle,
       jntPos2[1] + delta_angle,
       jntPos2[2] - delta_angle,
       jntPos2[3] + delta_angle, 
       jntPos2[4] - delta_angle,
       jntPos2[5] + delta_angle
     }};
 
     if(time2 > 5) { // 缩短测试时间至5秒
       cmd.setFinished();
       finished.fetch_sub(1);
     }
     return cmd;
   };
 
   try {
     std::cout << "Robot1开始MoveJ..." << std::endl;
     rtCon1->MoveJ(0.4, robot1.jointPos(ec), q_drag_er);
     if(ec) throw std::runtime_error("robot1 MoveJ失败: " + ec.message());
 
     std::cout << "Robot2开始MoveJ..." << std::endl;
     rtCon2->MoveJ(0.4, robot2.jointPos(ec), q_drag_er2);
     if(ec) throw std::runtime_error("robot2 MoveJ失败: " + ec.message());
   } catch (const std::exception &e) {
     std::cerr << e.what() << std::endl;
     deInit(robot1);
     deInit(robot2);
     return 1;
   }
 
   try {
     // 设置控制循环
     rtCon1->setControlLoop(callback1, 0, false);
     rtCon1->startMove(RtControllerMode::jointPosition);
 
     rtCon2->setControlLoop(callback2, 0, false);
     rtCon2->startMove(RtControllerMode::jointPosition);
 
     // 异步启动两个控制循环
     rtCon1->startLoop(false);
     rtCon2->startLoop(false);
 
     // 等待两个机器人完成
     while(finished.load() > 0) {
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
     }
     std::cout << "运动完成，停止控制循环" << std::endl;
 
     rtCon1->stopLoop();
     rtCon2->stopLoop();
   } catch(const std::exception &e) {
     std::cerr << "控制异常: " << e.what() << std::endl;
     deInit(robot1);
     deInit(robot2);
     return 1;
   }
 
   deInit(robot1);
   deInit(robot2);
   return 0;
 }