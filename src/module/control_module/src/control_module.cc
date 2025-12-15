#include "control_module/control_module.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "control_module/global.h"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <iostream>
#include <sstream> 
namespace xyber_x1_infer::rl_control_module {

bool ControlModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;
  SetLogger(core_.GetLogger());
  subs_.clear();

  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  try {
  // ================= 纯 ROS2 实现python和C++的通讯 =================
  my_ros2_node_ = std::make_shared<rclcpp::Node>("native_cpp_listener");
  
  // 1. 初始化 14 个关节的目标角度为 0.0
  std::vector<std::string> init_joints = {
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
      "left_elbow_pitch_joint", "left_elbow_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
      "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
      "right_elbow_pitch_joint", "right_elbow_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
      "left_claw_joint", "right_claw_joint"
  };
  for(const auto& name : init_joints) {
      all_joint_targets_[name] = 0.0;
  }
  // 左臂初始姿态，与stand_mode一致
  all_joint_targets_["left_shoulder_pitch_joint"] = 0.15;
  all_joint_targets_["left_shoulder_roll_joint"]  = -0.1;
  all_joint_targets_["left_elbow_pitch_joint"]    = 0.3;
  // 右臂同理
  all_joint_targets_["right_shoulder_pitch_joint"] = 0.15;
  all_joint_targets_["right_shoulder_roll_joint"]  = -0.1;
  all_joint_targets_["right_elbow_pitch_joint"]    = 0.3;
  // 2. 创建 String 订阅者
  my_ros2_sub_ = my_ros2_node_->create_subscription<std_msgs::msg::String>(
    "py_joint_cmd", 
    10, 
    [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
        
        // 解析协议: "joint_name:delta" 
        std::string s = msg->data;
        std::string delimiter = ":";
        size_t pos = s.find(delimiter);
        
        if (pos != std::string::npos) {
            std::string joint_name = s.substr(0, pos);
            double delta = std::stod(s.substr(pos + 1));

            // 更新对应关节的目标值
            if (all_joint_targets_.find(joint_name) != all_joint_targets_.end()) {
                all_joint_targets_[joint_name] += delta;
            }
        }
    }
  );

  ros2_spin_thread_ = std::thread([this]() {
      rclcpp::spin(my_ros2_node_);
  });
  ros2_spin_thread_.detach(); 
  // ==========================================================

    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(file_path.data());
      freq_ = cfg_node["control_frequecy"].as<int32_t>();
      use_sim_handles_ = cfg_node["use_sim_handles"].as<bool>();

      // 解析状态机
      last_trigger_time_ = high_resolution_clock::now();
      state_machine_.Init(cfg_node["robot_states"]);
      for (auto iter = cfg_node["robot_states"].begin(); iter != cfg_node["robot_states"].end(); iter++) {
        auto trigger_topic = iter->second["trigger_topic"].as<std::string>();
        if (trigger_topics_.find(trigger_topic) != trigger_topics_.end()) {
          continue;
        }
        trigger_topics_.insert(trigger_topic);
        subs_.push_back(core_.GetChannelHandle().GetSubscriber(trigger_topic));
        bool ret = aimrt::channel::Subscribe<std_msgs::msg::Float32>(subs_.back(),
          [this, trigger_topic](const std::shared_ptr<const std_msgs::msg::Float32>& msg) {
            if (Throttler(high_resolution_clock::now(), last_trigger_time_, milliseconds(1000)) && state_machine_.OnEvent(trigger_topic)) {
              auto now_state = state_machine_.GetCurrentState();
              auto controller_names = state_machine_.GetCurrentControllerNames();
              for (auto name : controller_names) {
                // printf("RestartController: %s\n", name.c_str());
                controller_map_[name]->RestartController();
              }
              AIMRT_INFO("Trigger event: [{}] -> {}", trigger_topic, now_state);
            }
          });
        AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed.");
      }
      // auto controller_names = state_machine_.GetCurrentControllerNames();
      // for (auto name : controller_names) {
      //   printf("name: %s\n", name.c_str());
      // }


      // 解析控制器
      for (auto iter = cfg_node["controllers"].begin(); iter != cfg_node["controllers"].end(); iter++) {
        std::string controller_name = iter->first.as<std::string>();
        // printf("controller: %s\n", controller_name.c_str());

        if (controller_name.substr(0, 3) == "rl_") {
          controller_map_[controller_name] = std::make_shared<RLController>(use_sim_handles_);
        } else if (controller_name.substr(0, 3) == "pd_") {
          controller_map_[controller_name] = std::make_shared<PDController>(use_sim_handles_);
        } else {
          AIMRT_ERROR("Unknown controller type: {}", controller_name);
        }
        controller_map_[controller_name]->Init(iter->second);
      }

      // 设置 joint_xxx_index_map_ 的尺度
      for (const auto& joint : cfg_node["joint_list"]) {
        joint_state_index_map_[joint.as<std::string>()] = -1;
      }
      std::vector<std::string> joint_list = cfg_node["joint_list"].as<std::vector<std::string>>();
      for (size_t ii = 0; ii < joint_list.size(); ++ii) {
        joint_cmd_index_map_[joint_list[ii]] = ii;
      }
      joint_offset_map_ = cfg_node["joint_offset"].as<std::map<std::string, double>>();

      // 控制器订阅
      subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_node["sub_joy_vel_name"].as<std::string>()));
      bool ret = aimrt::channel::Subscribe<geometry_msgs::msg::Twist>(subs_.back(), 
        [this](const std::shared_ptr<const geometry_msgs::msg::Twist>& msg) {
          auto controller_names = state_machine_.GetCurrentControllerNames();
          for (const auto& name : controller_names) {
            controller_map_[name]->SetCmdData(*msg);
          }
        });

      subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_node["sub_imu_data_name"].as<std::string>()));
      ret &= aimrt::channel::Subscribe<sensor_msgs::msg::Imu>(subs_.back(), 
        [this](const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
          auto controller_names = state_machine_.GetCurrentControllerNames();
          for (const auto& name : controller_names) {
            controller_map_[name]->SetImuData(*msg);
          }
        });

      subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_node["sub_joint_state_name"].as<std::string>()));
      ret &= aimrt::channel::Subscribe<sensor_msgs::msg::JointState>(subs_.back(), 
        [this](const std::shared_ptr<const sensor_msgs::msg::JointState>& msg) {
          // 仅初始化一次 joint_state_index_map_
          if (joint_state_index_map_.begin()->second == -1) {
            for (size_t i = 0; i < msg->name.size(); i++) {
              joint_state_index_map_[msg->name[i]] = i;
            }
          }

          // 新设置的 offset
          sensor_msgs::msg::JointState temp_msg = *msg;
          for (const auto& joint : joint_offset_map_) {
            temp_msg.position[joint_state_index_map_.at(joint.first)] -= joint.second;
          }

          for (const auto& controller : controller_map_) {
            controller.second->SetJointStateData(temp_msg, joint_state_index_map_);
          }
        });
      AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed.");

      // 控制器发布
      joint_cmd_pub_ = core_.GetChannelHandle().GetPublisher(cfg_node["pub_joint_cmd_name"].as<std::string>());
      executor_ = core_.GetExecutorManager().GetExecutor("rl_control_pub_thread");
      AIMRT_CHECK_ERROR_THROW(executor_, "Can not get executor 'rl_control_pub_thread'.");
      aimrt::channel::RegisterPublishType<my_ros2_proto::msg::JointCommand>(joint_cmd_pub_);
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");
  return true;
}

/*双臂实时控制程序*/
void ControlModule::realtime_control(my_ros2_proto::msg::JointCommand& cmd_msg) {
  // 定义关节列表
  std::vector<std::string> arm_joints = {
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
      "left_elbow_pitch_joint", "left_elbow_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
      "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
      "right_elbow_pitch_joint", "right_elbow_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
      "left_claw_joint", "right_claw_joint"
  };

  double target_stiffness = 80.0;
  double target_damping = 2.0;
  double claw_effort = 1.0; // 夹爪力度 (1.0最大)

  // 获取目标数据
  std::map<std::string, double> current_targets;
  {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      current_targets = all_joint_targets_;
  }

  // 填充控制指令
  for (const auto& joint_name : arm_joints) {
    if (joint_cmd_index_map_.find(joint_name) != joint_cmd_index_map_.end()) {
      int index = joint_cmd_index_map_[joint_name];
      cmd_msg.name[index] = joint_name;
      double final_pos = current_targets[joint_name];

      // ================= 夹爪特殊处理 =================
      if (joint_name == "left_claw_joint" || joint_name == "right_claw_joint") {
          // 限位,其实底层已经限位了
          if (final_pos > 1.0) final_pos = 1.0;
          if (final_pos < 0.0) final_pos = 0.0;

          cmd_msg.position[index] = final_pos;
          cmd_msg.effort[index] = claw_effort; 
          cmd_msg.stiffness[index] = 0.0;
          cmd_msg.damping[index] = 0.0;
          cmd_msg.velocity[index] = 0.0;
      }
      // ================= 普通关节处理 =================
      else {
          if (joint_offset_map_.find(joint_name) != joint_offset_map_.end()) {
              cmd_msg.position[index] = final_pos + joint_offset_map_[joint_name];
          } else {
              cmd_msg.position[index] = final_pos;
          }
          cmd_msg.velocity[index] = 0.0;
          cmd_msg.effort[index] = 0.0;
          cmd_msg.stiffness[index] = target_stiffness;
          cmd_msg.damping[index] = target_damping;
      }
    }
  }
}

bool ControlModule::Start() {
  AIMRT_INFO("thread safe [{}]", executor_.ThreadSafe());
  try {
    executor_.Execute([this]() { MainLoop(); });
    AIMRT_INFO("Started succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }
  return true;
}

void ControlModule::Shutdown() { run_flag_.store(false); }

bool ControlModule::MainLoop() {
  try {
    AIMRT_INFO("Start MainLoop.");
    auto const period = nanoseconds(1'000'000'000 / freq_);
    time_point<high_resolution_clock, nanoseconds> next_iteration_time = high_resolution_clock::now();

    my_ros2_proto::msg::JointCommand cmd_msg;
    cmd_msg.name.resize(joint_cmd_index_map_.size(), "");
    cmd_msg.position.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.velocity.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.effort.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.damping.resize(joint_cmd_index_map_.size(), 0.0);
    cmd_msg.stiffness.resize(joint_cmd_index_map_.size(), 0.0);

    while (run_flag_) {
      next_iteration_time += period;
      std::this_thread::sleep_until(next_iteration_time);

      auto controller_names = state_machine_.GetCurrentControllerNames();
      for (const auto& name : controller_names) {
        controller_map_[name]->Update();
        my_ros2_proto::msg::JointCommand tmp_cmd = controller_map_[name]->GetJointCmdData();
        // 将 tmp_cmd 中的数据复制到 cmd_msg 中
        if (name == "pd_plan")
        {
          realtime_control(cmd_msg);//当按下RB按键，进入pd_plan模式，进入双臂控制
        }
      
        else if(name!= "pd_plan")
        {
        for (size_t ii = 0; ii < tmp_cmd.name.size(); ii++) {
          int index = joint_cmd_index_map_[tmp_cmd.name[ii].c_str()];
          cmd_msg.name[index] = tmp_cmd.name[ii];
          cmd_msg.position[index] = tmp_cmd.position[ii] + joint_offset_map_[tmp_cmd.name[ii]];
          cmd_msg.velocity[index] = tmp_cmd.velocity[ii];
          cmd_msg.effort[index] = tmp_cmd.effort[ii];
          cmd_msg.damping[index] = tmp_cmd.damping[ii];
          cmd_msg.stiffness[index] = tmp_cmd.stiffness[ii];
        }
      }
    }
    
      aimrt::channel::Publish<my_ros2_proto::msg::JointCommand>(joint_cmd_pub_, cmd_msg);
    }
    AIMRT_INFO("Exit MainLoop.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Exit MainLoop with exception, {}", e.what());
    return false;
  }
  return true;
}

}  // namespace xyber_x1_infer::rl_control_module



