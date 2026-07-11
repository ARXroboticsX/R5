#include "arx_r5_controller/R5Controller.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// using namespace std::chrono_literals;

namespace arx::r5 {
R5Controller::R5Controller() : Node("r5_controller_node") {
  RCLCPP_INFO(this->get_logger(), "机械臂开始初始化...");
  std::string arm_control_type = this->declare_parameter("arm_control_type", "normal");
  std::string package_name = "arx_r5_controller";
  std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);
  int end_type = this->declare_parameter("arm_end_type", 0);
  catch_control_mode_ = static_cast<CatchControlMode>(this->declare_parameter("catch_control_mode", 0));
  std::string urdf_path;
  if (end_type == 0)
    urdf_path = package_share_dir + "/" + "X5liteaa0.urdf";
  else
    urdf_path = package_share_dir + "/" + "R5_master.urdf";

  interfaces_ptr_ =
      std::make_shared<InterfacesThread>(urdf_path, this->declare_parameter("arm_can_id", "can0"), end_type);
  if (this->declare_parameter("catch_action_fast", false))
    interfaces_ptr_->setCatchActionFast();
  else
    interfaces_ptr_->setCatchActionSlow();
  interfaces_ptr_->arx_x(500,2000,10);
  auto pub_name = this->declare_parameter("arm_pub_topic_name", "arm_status");

  if (arm_control_type == "normal") {
    RCLCPP_INFO(this->get_logger(), "常规模式启动");
    // 创建发布器
    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(pub_name, 1);
    // 创建订阅器
    joint_state_subscriber_ = this->create_subscription<arx5_arm_msg::msg::RobotCmd>(
        this->declare_parameter("arm_sub_topic_name", "arm_cmd"),
        10,
        std::bind(&R5Controller::CmdCallback, this, std::placeholders::_1));
    // 定时器，用于发布关节信息
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&R5Controller::PubState, this));
  } else if (arm_control_type == "vr_slave") {
    RCLCPP_INFO(this->get_logger(), "vr遥操作模式启动");
    // 创建发布器
    vr_joint_state_publisher_ = this->create_publisher<arm_control::msg::PosCmd>(pub_name, 10);

    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(pub_name + "_full", 1);
    // 创建订阅器
    vr_joint_state_subscriber_ = this->create_subscription<arm_control::msg::PosCmd>(
        this->declare_parameter("arm_sub_topic_name", "ARX_VR_L"),
        10,
        std::bind(&R5Controller::VrCmdCallback, this, std::placeholders::_1));
    // 定时器，用于发布关节信息
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&R5Controller::VrPubState, this));
  } else if (arm_control_type == "remote_master") {
    RCLCPP_INFO(this->get_logger(), "remote主机模式启动");
    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(pub_name, 10);
    interfaces_ptr_->setArmStatus(InterfacesThread::state::G_COMPENSATION);
    // 定时器，用于发布关节信息
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&R5Controller::PubState, this));
  } else if (arm_control_type == "remote_slave") {
    RCLCPP_INFO(this->get_logger(), "remote从机模式启动");
    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(pub_name, 10);
    follow_joint_state_subscriber_ = this->create_subscription<arx5_arm_msg::msg::RobotStatus>(
        this->declare_parameter("arm_sub_topic_name", "followed_arm_topic"),
        10,
        std::bind(&R5Controller::FollowCmdCallback, this, std::placeholders::_1));
    // 定时器，用于发布关节信息
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&R5Controller::PubState, this));
  }
}

void R5Controller::CmdCallback(const arx5_arm_msg::msg::RobotCmd::SharedPtr msg) {
  double end_pos[6] =
      {msg->end_pos[0], msg->end_pos[1], msg->end_pos[2], msg->end_pos[3], msg->end_pos[4], msg->end_pos[5]};

  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(end_pos);

  interfaces_ptr_->setEndPose(transform);

  std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 6; i++) {
    joint_positions[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(joint_positions);

  interfaces_ptr_->setArmStatus(msg->mode);

  interfaces_ptr_->setCatch(msg->gripper);
}

void R5Controller::PubState() {
  auto message = arx5_arm_msg::msg::RobotStatus();
  message.header.stamp = this->get_clock()->now();

  Eigen::Isometry3d transform = interfaces_ptr_->getEndPose();

  // 创建长度为6的vector
  std::array<double, 6> result;

  std::vector<double> xyzrpy = {0, 0, 0, 0, 0, 0};
  xyzrpy = solve::Isometry2Xyzrpy(transform);

  // 填充vector
  result[0] = xyzrpy[0];
  result[1] = xyzrpy[1];
  result[2] = xyzrpy[2];
  result[3] = xyzrpy[3];
  result[4] = xyzrpy[4];
  result[5] = xyzrpy[5];

  message.end_pos = result;

  std::vector<double> joint_pos_vector = interfaces_ptr_->getJointPositons();
  for (std::size_t i = 0; i < message.joint_pos.size() && i < joint_pos_vector.size(); ++i) {
    message.joint_pos[i] = joint_pos_vector[i];
  }

  std::vector<double> joint_velocities_vector = interfaces_ptr_->getJointVelocities();
  for (std::size_t i = 0; i < message.joint_vel.size() && i < joint_velocities_vector.size(); ++i) {
    message.joint_vel[i] = joint_velocities_vector[i];
  }

  std::vector<double> joint_current_vector = interfaces_ptr_->getJointCurrent();
  for (std::size_t i = 0; i < message.joint_cur.size() && i < joint_current_vector.size(); ++i) {
    message.joint_cur[i] = joint_current_vector[i];
  }
  // 发布消息
  joint_state_publisher_->publish(message);
}

void R5Controller::VrCmdCallback(const arm_control::msg::PosCmd::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "接收到数据");
  double input[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(input);

  interfaces_ptr_->setEndPose(transform);

  interfaces_ptr_->setArmStatus(InterfacesThread::state::END_CONTROL);

  interfaces_ptr_->setCatch(msg->gripper);
}

void R5Controller::VrPubState() {
  // RCLCPP_INFO(this->get_logger(), "发布数据");
  auto message = arm_control::msg::PosCmd();
  // message.header.stamp = this->get_clock()->now();

  Eigen::Isometry3d transform = interfaces_ptr_->getEndPose();

  // 提取四元数和位移
  Eigen::Quaterniond quat(transform.rotation());
  Eigen::Vector3d translation = transform.translation();

  std::vector<double> xyzrpy = solve::Isometry2Xyzrpy(transform);

  // 填充vector

  message.x = xyzrpy[0];
  message.y = xyzrpy[1];
  message.z = xyzrpy[2];
  message.roll = xyzrpy[3];
  message.pitch = xyzrpy[4];
  message.yaw = xyzrpy[5];
  message.quater_x = quat.x();
  message.quater_y = quat.y();
  message.quater_z = quat.z();
  message.quater_w = quat.w();

  std::vector<double> joint_pos_vector = interfaces_ptr_->getJointPositons();
  std::vector<double> joint_velocities_vector = interfaces_ptr_->getJointVelocities();
  std::vector<double> joint_current_vector = interfaces_ptr_->getJointCurrent();

  message.gripper = joint_pos_vector[6];

  // 发布消息
  vr_joint_state_publisher_->publish(message);

  //==========================================================
  auto msg = arx5_arm_msg::msg::RobotStatus();
  msg.header.stamp = this->get_clock()->now();

  // 创建长度为6的vector
  std::array<double, 6> result;

  // 填充vector
  result[0] = xyzrpy[0];
  result[1] = xyzrpy[1];
  result[2] = xyzrpy[2];
  result[3] = xyzrpy[3];
  result[4] = xyzrpy[4];
  result[5] = xyzrpy[5];

  msg.end_pos = result;

  for (std::size_t i = 0; i < msg.joint_pos.size() && i < joint_pos_vector.size(); ++i) {
    msg.joint_pos[i] = joint_pos_vector[i];
  }

  for (std::size_t i = 0; i < msg.joint_vel.size() && i < joint_velocities_vector.size(); ++i) {
    msg.joint_vel[i] = joint_velocities_vector[i];
  }

  for (std::size_t i = 0; i < msg.joint_cur.size() && i < joint_current_vector.size(); ++i) {
    msg.joint_cur[i] = joint_current_vector[i];
  }
  // 发布消息
  joint_state_publisher_->publish(msg);
}

void R5Controller::FollowCmdCallback(const arx5_arm_msg::msg::RobotStatus::SharedPtr msg) {
  std::vector<double> target_joint_position(6, 0.0);

  for (int i = 0; i < 6; i++) {
    target_joint_position[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(target_joint_position);
  interfaces_ptr_->setArmStatus(InterfacesThread::state::POSITION_CONTROL);

  if (catch_control_mode_ == CatchControlMode::kPosition)
    interfaces_ptr_->setCatch(msg->joint_pos[6] * 5);
  else
    interfaces_ptr_->setCatchTorque(msg->joint_cur[6]);
}
}

void signalHandler(int signum) {
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  std::signal(SIGHUP, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arx::r5::R5Controller>());
  rclcpp::shutdown();
  return 0;
}
