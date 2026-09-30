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
 * @file src/dlio/map.cc
 */

#include "dlio/map.h"
#include "dlio/utils.h"

// 构造函数，通过初始化列表调用基类 rclcpp::Node 的构造函数，节点名称为 "dlio_map_node"
dlio::MapNode::MapNode(): Node("dlio_map_node") {

  this->getParams();  // 调用私有方法 getParams()，从 ROS 参数服务器读取配置参数

  this->keyframe_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);   // 创建一个互斥回调组（MutuallyExclusive）。同一组内的回调不会并行执行，避免数据竞争。此处用于键帧订阅回调
  auto keyframe_sub_opt = rclcpp::SubscriptionOptions();
  keyframe_sub_opt.callback_group = this->keyframe_cb_group;   // 创建订阅选项对象，并设置其回调组为刚创建的组

  this->keyframe_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("keyframes", 10,
      std::bind(&dlio::MapNode::callbackKeyframe, this, std::placeholders::_1), keyframe_sub_opt);  // 创建订阅者，订阅话题 "keyframes"，消息类型为 PointCloud2，队列大小 10。回调函数绑定为 callbackKeyframe，使用自定义订阅选项（含回调组）。当收到关键帧点云时，callbackKeyframe 将被调用

  this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 100);  // 创建发布者，话题 "map"，队列大小 100，用于发布累积的地图点云


  this->save_pcd_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);  // 创建另一个互斥回调组，用于服务回调（保存 PCD 文件），避免与订阅回调并发冲突

  this->save_pcd_srv = this->create_service<direct_lidar_inertial_odometry::srv::SavePCD>("save_pcd",
      std::bind(&dlio::MapNode::savePCD, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->save_pcd_cb_group);  // 创建服务 save_pcd，类型为自定义服务 SavePCD（请求包含保存路径和叶大小，响应包含成功标志）。回调函数为 savePCD，使用默认 QoS 配置，并指定回调组

  this->dlio_map = std::make_shared<pcl::PointCloud<PointType>>();  // 初始化 dlio_map 为空的 PCL 点云指针，用于累积所有关键帧滤波后的点云

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);   // 设置 PCL 控制台输出级别为仅错误信息，减少冗余日志。

}  // 构造函数结束


// 析构函数: 空的析构函数，无特殊资源释放（智能指针自动管理）
dlio::MapNode::~MapNode() {}


void dlio::MapNode::getParams() {
  // 声明参数（若未设置则使用默认值）。odom/odom_frame 为里程计坐标系名称，默认 "odom"；  map/sparse/leafSize 为体素滤波叶大小，默认 0.5 米
  this->declare_parameter<std::string>("odom/odom_frame", "odom");
  this->declare_parameter<double>("map/sparse/leafSize", 0.5);

  // 将参数值读取到成员变量 odom_frame 和 leaf_size_ 中
  this->get_parameter("odom/odom_frame", this->odom_frame);
  this->get_parameter("map/sparse/leafSize", this->leaf_size_);
}

void dlio::MapNode::start() {
}


// 回调函数 : 订阅回调，接收关键帧点云（常量共享指针）
void dlio::MapNode::callbackKeyframe(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe) {

  // convert scan to pcl format : 将 ROS 点云消息转换为 PCL 点云格式（PointType 通常是 pcl::PointXYZI 等）
  pcl::PointCloud<PointType>::Ptr keyframe_pcl = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter : 应用体素滤波器对点云进行降采样，叶大小由参数 leaf_size_ 决定。滤波后点云仍然保存在 keyframe_pcl 中（原地覆盖）
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save filtered keyframe to map for rviz : 将滤波后的关键帧点云累加到全局地图 dlio_map 中。+= 操作符 合并两个点云。
  *this->dlio_map += *keyframe_pcl;

  // publish full map
  if (this->dlio_map->points.size() == this->dlio_map->width * this->dlio_map->height) {
    sensor_msgs::msg::PointCloud2 map_ros;
    pcl::toROSMsg(*this->dlio_map, map_ros);   // 将累积的 PCL 地图转换为 ROS 点云消息
    map_ros.header.stamp = this->now();         // 设置消息时间戳为当前 ROS 时间
    map_ros.header.frame_id = this->odom_frame;  // 设置坐标系为 odom_frame（里程计坐标系）
    this->map_pub->publish(map_ros);    // 发布地图点云，供 Rviz 或其他节点可视化
  } 
}  // 订阅回调结束


// 回调函数 : 服务回调，接收请求和响应对象
void dlio::MapNode::savePCD(std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePCD::Request> req,
                            std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePCD::Response> res) {

  pcl::PointCloud<PointType>::Ptr m = std::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map);    // 复制当前地图点云（深拷贝）保存在 m 中，避免修改原始地图

  // 从请求中读取所需的体素叶大小和保存路径
  float leaf_size = req->leaf_size;
  std::string p = req->save_path;

  // 打印进度信息（保留 2 位小数），并立即刷新输出缓冲区（flush()）
  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlio_map.pcd"
    << " with leaf size " << to_string_with_precision(leaf_size, 2) << "... "; std::cout.flush();

  // voxelize map : 对复制的点云进行体素滤波，使用请求中指定的叶大小。滤波后结果仍保存在 m 中
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save map : 以二进制格式保存 PCD 文件。ret 为 0 表示成功，将成功标志赋给响应
  int ret = pcl::io::savePCDFileBinary(p + "/dlio_map.pcd", *m);
  res->success = ret == 0;

  // 输出保存结果
  if (res->success) {
    std::cout << "done" << std::endl;
  } else {
    std::cout << "failed" << std::endl;
  }
} // 服务回调结束
