#pragma once

#include <chrono>
#include <memory>
#include <set>
#include <mutex> 
#include <atomic>
#include <map>
#include <string>
#include <vector>

// ROS 2 消息头文件
#include <geometry_msgs/msg/twist.hpp> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread> // 需要用到线程
#include "std_msgs/msg/float32.hpp"
#include "aimrt_module_cpp_interface/module_base.h"
#include "control_module/pd_controller.h"
#include "control_module/rl_controller.h"
#include "control_module/state_machine.h"

using namespace std::chrono;

namespace xyber_x1_infer::rl_control_module {

class ControlModule : public aimrt::ModuleBase {
 public:
  ControlModule() = default;
  ~ControlModule() override = default;
  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ControlModule"};
  }
  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  bool MainLoop();
  // 自定义控制函数声明
  void realtime_control(my_ros2_proto::msg::JointCommand& cmd_msg);

 private:
  aimrt::CoreRef core_;
  aimrt::executor::ExecutorRef executor_;

  std::vector<aimrt::channel::SubscriberRef> subs_;
  aimrt::channel::PublisherRef joint_cmd_pub_;

  StateMachine state_machine_;
  std::set<std::string> trigger_topics_;
  std::map<std::string, std::shared_ptr<ControllerBase>> controller_map_;
  std::unordered_map<std::string, int> joint_state_index_map_;
  std::unordered_map<std::string, int> joint_cmd_index_map_;
  std::map<std::string, double> joint_offset_map_;

  //订阅者为 String 类型,接收 "关节名:数值"
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr my_ros2_sub_; 
  std::shared_ptr<rclcpp::Node> my_ros2_node_;
  std::thread ros2_spin_thread_;
  std::mutex cmd_vel_mutex_;

  //使用 Map 存储所有 14 个关节的目标角度
  std::map<std::string, double> all_joint_targets_;
  
  bool use_sim_handles_;
  int32_t freq_;
  std::atomic_bool run_flag_{true};
  time_point<high_resolution_clock> last_trigger_time_;
};

}  // namespace xyber_x1_infer::rl_control_module