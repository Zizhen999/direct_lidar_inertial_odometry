/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/
/**
 * @file src/dlio/map_node.cc
 */

#include "dlio/map.h"

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);    // 初始化 ROS2 客户端库。必须在使用任何 ROS2 功能之前调用。
  auto node = std::make_shared<dlio::MapNode>();    // 创建 MapNode 类的共享指针。该构造函数会执行节点的初始化工作（订阅、发布、服务等）
  rclcpp::executors::MultiThreadedExecutor executor;   // 创建一个多线程执行器。MultiThreadedExecutor 允许同时处理多个回调（例如同时处理订阅回调和服务请求），提高并发性能
  executor.add_node(node);     // 将节点添加到执行器中，使其回调可以被执行。
  executor.spin();     // 启动事件循环，阻塞当前线程，不断处理传入的订阅消息、服务请求等回调，直到节点被关闭。

  rclcpp::shutdown();  // 在 spin() 返回后（通常因 Ctrl+C 或节点关闭触发），关闭 ROS2 客户端库

  return 0;

}
