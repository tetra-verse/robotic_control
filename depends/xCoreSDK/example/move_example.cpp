﻿/**
 * @file move_example.cpp
 * @brief 非实时运动指令. 根据机型和坐标系的不同, 各示例中的点位不一定可达, 仅供接口使用方法的参考
 *
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <thread>
#include <cmath>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "print_helper.hpp"

using namespace std;
using namespace rokae;
std::ostream &os = std::cout; ///< print to console

namespace Predefines {
 // ******   拖拽位姿   ******
 const std::vector<double> ErDragPosture = {0, M_PI/6, M_PI/3, 0, M_PI_2, 0}; ///< xMateEr3, xMateEr7
 const std::vector<double> ErProDragPosture = {0, M_PI/6, 0, M_PI/3, 0, M_PI_2, 0}; ///< xMateEr3 Pro, xMateEr7 Pro
 const std::vector<double> CrDragPosture {0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0}; ///< xMateCR
 const std::vector<double> Cr5DragPostre = { 0, M_PI / 6, -M_PI_2, -M_PI / 3, 0}; ///< CR5轴构型

 Toolset defaultToolset; ///< 默认工具工件
}
/**
 * @brief 打印运动执行信息
 */
void printInfo(const rokae::EventInfo &info) {
  using namespace rokae::EventInfoKey::MoveExecution;
  print(std::cout, "[运动执行信息] ID:", std::any_cast<std::string>(info.at(ID)), "Index:", std::any_cast<int>(info.at(WaypointIndex)),
        "已完成: ", std::any_cast<bool>(info.at(ReachTarget)) ? "YES": "NO", std::any_cast<error_code>(info.at(Error)),
          std::any_cast<std::string>(info.at(Remark)));
  // 如果设置了自定义信息，打印这个信息
  if(info.count(CustomInfo)) {
    auto custom_info =  std::any_cast<std::string>(info.at(CustomInfo));
    if(!custom_info.empty()) print(std::cout, "自定义信息: ",custom_info);
  }
}

/**
 * @brief 等待运动结束 - 通过查询路径ID及路点序号是否已完成的方式
 */
void waitForFinish(BaseRobot &robot, const std::string &traj_id, int index){
  using namespace rokae::EventInfoKey::MoveExecution;
  error_code ec;
  while(true) {
    auto info = robot.queryEventInfo(Event::moveExecution, ec);
    auto _id = std::any_cast<std::string>(info.at(ID));
    auto _index = std::any_cast<int>(info.at(WaypointIndex));
    if(auto _ec = std::any_cast<error_code>(info.at(Error))) {
      print(std::cout, "路径", _id, ":", _index, "错误:", _ec.message());
      return;
    }
    if(_id == traj_id && _index == index) {
      if(std::any_cast<bool>(info.at(ReachTarget))) {
        print(std::cout, "路径", traj_id, ":", index, "已完成");
      }
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

/**
 * @brief 等待运动结束 - 通过查询机械臂是否处于运动中的方式
 */
void waitRobot(BaseRobot &robot, bool &running) {
  running = true;
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    error_code ec;
    auto st = robot.operationState(ec);
    if(st == OperationState::idle || st == OperationState::unknown){
      running = false;
    }
  }
}

/**
 * @brief 事件处理 - 模拟发生碰撞后等待20秒上电并继续运行
 * 发生碰撞后，机器人控制器会立即记录诊断数据，需要10秒左右，记录完毕中才能开始运动。
 */
void recoverFromCollision(BaseRobot &robot, const rokae::EventInfo &info) {
  using namespace rokae::EventInfoKey;
  bool isCollided = std::any_cast<bool>(info.at(Safety::Collided));
  print(std::cout, "Collided:", isCollided);
  if(isCollided) {
    std::this_thread::sleep_for(std::chrono::seconds(20));
    error_code ec;
    robot.setPowerState(true, ec);
    robot.moveStart(ec);
    print(std::cout, "Recovered from collision");
  }
}

/**
 * @brief 严格遵循轴配置数据(conf data)的直线运动。点位适用机型xMateER3
 */
void moveWithForcedConf(xMateRobot &robot) {
  Toolset default_toolset;
  error_code ec;
  bool running;
  std::string id;
  robot.setToolset(default_toolset, ec);
  robot.setDefaultSpeed(200,ec);
  robot.setDefaultZone(5, ec);

  print(std::cout, "运动到拖拽位姿");
  robot.executeCommand({MoveAbsJCommand(Predefines::ErDragPosture)}, ec);
  waitRobot(robot, running);

  CartesianPosition cartesian_position0({0.786, 0, 0.431, M_PI, 0.6, M_PI});
  CartesianPosition cartesian_position1({0.786, 0, 0.431, M_PI, 0.98, M_PI});


  MoveJCommand j0({cartesian_position0}), j1({cartesian_position1});
  MoveLCommand l0({cartesian_position0}), l1({cartesian_position1});

  // 不严格遵循轴配置数据， MoveL & MoveJ 逆解选取当前轴角度最近解
  print(std::cout, "开始MoveJ");
  robot.moveAppend({j0, j1}, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 1);

  // 遵循轴配置数据， 使用conf计算逆解，此时MoveL可能逆解失败
  robot.setDefaultConfOpt(true, ec);
  print(std::cerr, ec);
  cartesian_position1.confData = {-1,1,-1,0,1,0,0,2};
  l1.target = cartesian_position1;
  j1.target = cartesian_position1;

  print(std::cout, "开始MoveJ");
  robot.moveAppend({j0, j1}, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 1);

  print(std::cout, "运动到拖拽位姿");
  robot.executeCommand({MoveAbsJCommand(Predefines::ErDragPosture)}, ec);
  waitRobot(robot, running);

  print(std::cout, "开始MoveL");
  robot.moveAppend({l0, l1}, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 1);
  robot.setDefaultConfOpt(false, ec);
}

/**
 * @brief 示例 - 笛卡尔点位设置偏移 & 运动中暂停与继续; 点位适用机型xMateEr7 Pro
 */
void cartesianPointWithOffset(BaseRobot &robot) {
  error_code ec;

  std::array<double, 6> pos = {0.631, 0, 0.38, M_PI, 0, M_PI};
  std::array<double,6> offset_z = {0, 0, 0.2, 0, 0, 0};

  MoveLCommand moveL1(pos, 500, 5), moveL2(pos, 800, 0);
  // 示例：设置空间旋转速度为100°/s, 如不设置则默认为200°/s
  moveL1.rotSpeed = 100 / 180.0 * M_PI;

  // 相对工件坐标系Z+偏移0.2m
  moveL2.offset = { CartesianPosition::Offset::offs, offset_z};

  MoveJCommand moveJ1(pos, 200, 0), moveJ2(pos, 1000, 80);
  // 相对工具坐标系Z+偏移0.2m
  moveJ2.offset = {CartesianPosition::Offset::relTool, offset_z};

  // 先运动到起始位置，再执行这4个点位
  robot.executeCommand({MoveAbsJCommand(Predefines::ErProDragPosture)}, ec);
  robot.executeCommand({moveL1, moveL2}, ec);
  robot.executeCommand({moveJ1, moveJ2}, ec);

  std::thread input([&]{
    int c{};
    print(os, "[p]暂停 [c]继续 [q]退出");
    while(c != 'q') {
      c = getchar();
      switch(c) {
        case 'p':
          robot.stop(ec);
          print(std::cerr, ec); break;
        case 'c':
          robot.moveStart(ec);
          print(std::cerr, ec); break;
        default: break;
      }
    }
  });
  input.join();
  robot.moveReset(ec);
}

/**
 * @brief 螺旋线运动，适用机型: xMate3
 */
void spiralMove(rokae::xMateRobot &robot) {
  error_code ec;
  std::string id;
  rokae::Toolset default_toolset = {};
  robot.setToolset(default_toolset, ec);

  // 螺旋线终点姿态, 只用到rpy, xyz值任意
  rokae::CartesianPosition cart_target({0, 0, 0, 2.967, -0.2, 3.1415}),
  cart_target1({0, 0, 0, -2.787577,0.1639,-2.9});
  rokae::MoveAbsJCommand absjcmd({0.0,0.22150561307150393,1.4779577696969546,0.0,1.2675963456219013,0.0});

  // 螺旋线1: 初始半径0.01m, 半径变化步长0.0005m/rad, 逆时针旋转720°，速度v500
  rokae::MoveSPCommand spcmd1({cart_target, 0.01, 0.0005, M_PI * 4, false, 500}),
  // 螺旋线2: 初始半径0.05m, 半径变化步长0.001m/rad, 顺时针旋转360°，速度v100
  spcmd2({cart_target1, 0.05, 0.001, M_PI * 2, true, 100});

  std::vector<rokae::MoveSPCommand> spcmds = {spcmd1, spcmd2};
  robot.moveAppend({absjcmd}, id, ec);
  robot.moveAppend(spcmds, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, (int)spcmds.size() - 1);
}

/**
 * @brief 示例 - 七轴冗余运动 & 发生碰撞检测后恢复运动, 点位适用机型xMateER3 Pro
 */
void redundantMove(xMateErProRobot &robot) {
  error_code ec;
  std::string id;

  // 本段示例使用默认工具工件, 速度v500, 转弯区fine
  Toolset defaultToolset;
  robot.setToolset(defaultToolset, ec);
  robot.setDefaultSpeed(500, ec);
  robot.setDefaultZone(0, ec);

  // 可选: 设置碰撞检测事件回调函数
  robot.setEventWatcher(Event::safety, [&](const EventInfo &info){
    recoverFromCollision(robot, info);
  }, ec);


  MoveAbsJCommand moveAbsj({0, M_PI/6, 0, M_PI/3, 0, M_PI_2, 0});
  // ** 1) 变臂角运动 **
  MoveLCommand moveL1({0.562, 0, 0.432, M_PI, 0, -M_PI});
  moveL1.target.elbow = 1.45;
  robot.moveAppend({moveAbsj}, id, ec);
  robot.moveAppend({moveL1}, id, ec);
  moveL1.target.elbow = -1.51;
  robot.moveAppend({moveL1}, id, ec);
  robot.moveStart(ec);
  // 最后一次moveAppend()发送一条指令，故index = 0
  waitForFinish(robot, id, 0);

  // ** 2) 60°臂角圆弧 **
  CartesianPosition circle_p1({0.472, 0, 0.342, M_PI, 0, -M_PI}),
  circle_p2({0.602, 0, 0.342, M_PI, 0, -M_PI}),
  circle_a1({0.537, 0.065, 0.342, M_PI, 0, -M_PI}),
  circle_a2({0.537, -0.065, 0.342, M_PI, 0, -M_PI});
  // 臂角都是60°
  circle_p1.elbow = M_PI/3;
  circle_p2.elbow = M_PI/3;
  circle_a1.elbow = M_PI/3;
  circle_a2.elbow = M_PI/3;

  MoveLCommand moveL2(circle_p1);
  robot.moveAppend({moveL2}, id, ec);
  MoveCCommand moveC1(circle_p2, circle_a1), moveC2(circle_p1, circle_a2);
  std::vector<MoveCCommand> movec_cmds = {moveC1, moveC2};
  robot.moveAppend(movec_cmds, id, ec);
  robot.moveStart(ec);
  // 最后一次moveAppend()发送2条指令，故需要等待第二个点完成后返回，index为第二个点的下标
  waitForFinish(robot, id, (int)movec_cmds.size() - 1);
}

/**
 * @brief 示例 - 全圆运动，点位适配机型XMC20
 */
void fullCircleMove(xMateRobot &robot) {
  error_code ec;

  // 本段示例使用默认工具工件
  Toolset defaultToolset;
  robot.setToolset(defaultToolset, ec);

  // 起始角度
  std::array<double, 6> start_angle = {0, 0.557737,-1.5184888, 0,-1.3036738, 0};

  auto robot_model = robot.model();
  // 起始角度对应位姿
  auto cart_pose = robot_model.calcFk(start_angle, ec);

  MoveAbsJCommand abs_j({start_angle[0], start_angle[1], start_angle[2],
                         start_angle[3], start_angle[4], start_angle[5]}, 1000, 0);

  // 全圆指令，执行360度
  MoveCFCommand move_cf1(cart_pose, cart_pose, M_PI * 2, 100, 10);

  // 辅助点1: 起始位姿偏移Y+10mm
  move_cf1.auxOffset.type = CartesianPosition::Offset::offs;
  move_cf1.auxOffset.frame.trans[1] = 0.01;
  // 辅助点2: 起始位姿偏移X+5mm, Y-10mm
  move_cf1.targetOffset.type = CartesianPosition::Offset::offs;
  move_cf1.targetOffset.frame.trans[0] = 0.005;
  move_cf1.targetOffset.frame.trans[1] = -0.01;

  MoveCFCommand move_cf2 = move_cf1, move_cf3 = move_cf1;

  // 分别设定三种旋转姿态类型
  move_cf1.rotType = MoveCFCommand::constPose;
  move_cf2.rotType = MoveCFCommand::rotAxis;
  move_cf3.rotType = MoveCFCommand::fixedAxis;

  std::string id;
  // 执行三种全圆运动
  // 注意每次执行前先运动到起始角度，否则可能会出现关节超限位的报错
  robot.moveAppend({abs_j}, id, ec);
  robot.moveAppend({ move_cf1 }, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 0);

  robot.moveAppend({abs_j}, id, ec);
  robot.moveAppend({ move_cf2 }, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 0);

  robot.moveAppend({abs_j}, id, ec);
  robot.moveAppend({ move_cf3 }, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 0);
}

/**
 * @brief 锁定4轴奇异规避方式。示例适用机型xMateCR7
 */
void avoidSingularityMove_Lock4(rokae::xMateRobot &robot) {
  error_code ec;
  std::string id;
  bool running;
  robot.setToolset(Predefines::defaultToolset, ec);

  // 先运动到起始位姿
  robot.executeCommand({MoveAbsJCommand({0.453,0.539,-1.581,0.0,0.026,0})}, ec);
  waitRobot(robot, running);

  std::vector<rokae::MoveLCommand> cmds = {
    MoveLCommand({0.66675437164302165, -0.23850040314585069, 0.85182031,-3.1415926535897931, 1.0471975511965979, 3.01151}),
    MoveLCommand({0.66675437164302154, 0.15775146321850292, 0.464946,-3.1415926535897931, 1.0471975511965979, -2.6885547129789127})
  };

  // 不打开奇异规避模式, 会报错超出运动范围
  robot.setAvoidSingularity(AvoidSingularityMethod::lockAxis4, false, 0, ec);
  robot.moveAppend(cmds, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, (int)cmds.size() - 1);

  // 打开奇异点规避模式，点位可达
  // 注意，运动重置时会关闭所有奇异规避功能
  robot.moveReset(ec);
  robot.setAvoidSingularity(AvoidSingularityMethod::lockAxis4, true, 0, ec);
  std::cerr << ec;
  print(std::cout, "四轴锁定奇异规避功能", robot.getAvoidSingularity(AvoidSingularityMethod::lockAxis4, ec) ? "打开" : "关闭");

  robot.moveAppend(cmds, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, (int)cmds.size() - 1);
  robot.setAvoidSingularity(AvoidSingularityMethod::lockAxis4, false, 0, ec);
}

/**
 * @brief 示例 - 使用工具工件坐标系，点位适用机型xMateCR7
 */
void moveInToolsetCoordinate(BaseRobot &robot) {
  error_code ec;
  std::string id;
  // 默认的工具工件坐标系
  robot.setToolset(Predefines::defaultToolset, ec);

  MoveAbsJCommand moveAbs({0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0});
  robot.moveAppend({moveAbs}, id, ec);

  MoveLCommand movel1({0.563, 0, 0.432, M_PI, 0, M_PI}, 1000, 100);
  MoveLCommand movel2({0.33467, -0.095, 0.51, M_PI, 0, M_PI}, 1000, 100);
  robot.moveAppend({movel1, movel2}, id, ec);
  robot.moveStart(ec);
  bool moving = true;
  waitRobot(robot, moving);

  // 举例：执行完movel1和movel2, 需要切换到工具工件, 再执行后面的运动指令
  // 设置工具工件方式1: 直接设定
  Toolset toolset1;
  toolset1.ref = {{0.1, 0.1, 0}, {0, 0, 0}}; // 外部参考坐标系，X+0.1m, Y+0.1m
  toolset1.end = {{ 0, 0, 0.01}, {0, M_PI/6, 0}}; // 末端坐标，Z+0.01m, Ry+30°
#if 0
  toolset1.load.mass = 2; // 负载2kg
#endif
  robot.setToolset(toolset1, ec);

#if 0
  // 设置工具工件方式2: 使用已创建的工具工件tool1, wobj1
  robot.setToolset("tool1", "wobj1", ec);
#endif
  MoveLCommand movel3({0.5, 0, 0.4, M_PI, 0, M_PI}, 1000, 100);
  robot.moveAppend({movel3}, id, ec);
  robot.moveStart(ec);
  waitRobot(robot, moving);
}

/**
 * @brief 示例 - 运动中调整运动速率
 */
void adjustSpeed(BaseRobot &robot) {
  error_code ec;
  std::string id;
  double scale = 0.5;
  robot.adjustSpeedOnline(scale, ec); // 设置起始速度比例为50%

  // 示例用: 在cmd1和cmd2两个点位之间运动
  rokae::MoveAbsJCommand cmd1({0, 0, 0, 0, 0, 0}), cmd2({1.5, 1.5,1.5,1.5,1.5,1.5});
  robot.moveAppend({cmd1, cmd2,cmd1,cmd2,cmd1,cmd2,cmd1,cmd2}, id, ec);
  robot.moveStart(ec);
  bool running = true;

  // 读取键盘输入
  std::thread readInput([&]{
    while(running) {
      auto ch = std::getchar();
      if(ch == 'a') {
        if(scale < 0.1) { print(std::cerr, "已达到1%"); continue; }
        scale -= 1e-1;
      } else if(ch == 'd'){
        if(scale > 1) { print(std::cerr, "已达到100%"); continue; }
        scale += 1e-1;
      } else { continue; }
      robot.adjustSpeedOnline(scale, ec);
      print(os, "调整为", scale);
    }
  });
  print(os, "机器人开始运动, 请按[a]减小速度 [d]增大速度, 步长为10%");

  // 等待运动结束
  waitRobot(robot, running);
  readInput.join();
}

/**
 * @brief 示例 - 设置轴配置数据(confData)处理多逆解问题, 点位适用机型xMateCR7
 * Example - use joint configure data to get the desired IK result
 */
void multiplePosture(xMateRobot &robot) {
  error_code ec;
  std::string id;

  // 本段示例使用默认工具工件
  // use default tool and wobj frame
  Toolset defaultToolset;
  robot.setToolset(defaultToolset, ec);
  // 设置使用confdata来计算逆解
  robot.setDefaultConfOpt(true, ec);

  MoveJCommand moveJ({0.2434, -0.314, 0.591, 1.5456, 0.314, 2.173});
  // 同样的末端位姿，confData不同，轴角度也不同
  // the target posture is same, but give different joint configure data
  moveJ.target.confData =  {-67, 100, 88, -79, 90, -120, 0, 0};
  // 示例：设置关节速度百分比为10%。如不设置的话，关节速度根据末端线速度计算得出
  moveJ.jointSpeed = 0.1;
  robot.moveAppend({moveJ}, id, ec);

  moveJ.target.confData =  {-76, 8, -133, -106, 103, 108, 0, 0};
  robot.moveAppend({moveJ}, id, ec);
  moveJ.target.confData =  {-70, 8, -88, 90, -105, -25, 0, 0};
  robot.moveAppend({moveJ}, id, ec);

  robot.moveStart(ec);
  waitForFinish(robot, id, 0);
  robot.setDefaultConfOpt(false, ec);
}

/**
 * @brief 示例 - 带导轨运动。点位适配机型xMateSR4
 */
 template <WorkType wt, unsigned short dof>
void moveWithRail(Robot_T<wt, dof> *robot) {
  error_code ec;
  bool is_rail_enabled;
  robot->getRailParameter("enable", is_rail_enabled, ec);
  if(!is_rail_enabled) {
    print(os, "未开启导轨");
    return;
  }

  // 打开关闭导轨，设置导轨参数
  // 设置导轨参数和基坐标系需要重启控制器生效, 这里仅展示接口调用方法
  robot->setRailParameter("enable", true, ec); // 打开导轨功能
  robot->setRailParameter("maxSpeed", 1, ec); // 设置最大速度1m/s
  robot->setRailParameter("softLimit", std::vector<double>({-0.8, 0.8}), ec); // 设置软限位为+-0.8m
  robot->setRailParameter("reductionRatio", 1.0, ec); // 设置减速比

  auto curr = robot->BaseRobot::jointPos(ec);
  print(os, "当前轴角度", robot->BaseRobot::jointPos(ec));

  // *** Jog导轨示例 ***
  // 手动模式上电
  robot->setOperateMode(OperateMode::manual, ec);
  robot->setPowerState(true, ec);
  std::vector<double> soft_limit;
  robot->getRailParameter("softLimit", soft_limit, ec);
  // 在软限位内Jog
  double step = (curr.back() - soft_limit[0] > 0.1 ? 0.1 : (curr.back() - soft_limit[0])) * 1000.0;
  // 以六轴机型轴空间点动为例，index 0~5 代表1-6轴, index=6 代表第一个外部轴
  int ex_jnt_index = robot->robotInfo(ec).joint_num;
  // 导轨轴空间负向运动100mm
  robot->startJog(JogOpt::jointSpace, 0.6, step, ex_jnt_index, false, ec);
  // 等待Jog结束
  while(true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if(robot->operationState(ec) != OperationState::jogging) break;
  }
  robot->stop(ec);

  // *** 带导轨的运动指令示例 ***
  CartesianPosition p0({0.56, 0.136, 0.416, M_PI, 0, M_PI}), p1({0.56, 0.136, 0.3, M_PI, 0, M_PI});
  p0.external = { 0.02 }; // 导轨运动到0.02m, 下同
  p1.external = { -0.04 };
  MoveAbsJCommand abs_j_command({0, M_PI/6, -M_PI_2,0, -M_PI/3, 0 });
  abs_j_command.target.external = { 0.1 }; // 导轨运动到0.1m
  MoveJCommand j_command(p0);
  MoveLCommand l_command(p1);
  MoveCCommand c_command(p1, p0);

  // 加自定义信息，将在运动信息反馈中返回出来
  l_command.customInfo = "hello";

  std::string id;
  robot->moveAppend(abs_j_command, id, ec);
  robot->moveAppend(j_command, id, ec);
  robot->moveAppend(l_command, id, ec);
  robot->moveAppend(abs_j_command, id, ec);
  robot->moveAppend(c_command, id, ec);
  robot->moveStart(ec);
  waitForFinish(*robot, id, 0);
}

/**
 * @brief main program
 */
int main() {
  try {
    using namespace rokae;

    // *** 1. 连接机器人 ***
    // *** 1. Connect to the robot ***
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateRobot robot(ip); // ****   xMate 6-axis

    // *** 2. 切换到自动模式并上电 ***
    // *** 2. Switch to auto mode and motor on ***
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);

    // *** 3. 设置默认运动速度和转弯区 ***
    // *** 3. set default speed and turning zone ***
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot.setDefaultZone(50, ec); // 可选：设置默认转弯区
    robot.setDefaultSpeed(200, ec); // 可选：设置默认速度

    // 可选：设置运动指令执行完成和错误信息回调
    // Optional: set motion event notification
    robot.setEventWatcher(Event::moveExecution, printInfo, ec);

    // *** 4. 运动示例程序 ***
    // *** 4. demo motion program ***
    // multiplePosture(robot);

    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);
  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}