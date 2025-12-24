#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <thread>
#include <array> // 必须包含这个头文件

#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/handangle.hpp"
#include "rm_ros_interfaces/msg/handforce.hpp"
#include "rm_ros_interfaces/msg/handspeed.hpp"
#include "rm_ros_interfaces/msg/handstatus.hpp"
// 假设 my_interfaces 包已正确生成
#include "my_interfaces/msg/header_float32.hpp" 

using namespace std::chrono_literals;

// ================= 配置区域 =================

// 【修正 1】类型改为 std::array，与 ROS 消息定义的 int16[6] 保持一致
static const std::array<int16_t, 6> HAND_INIT_ANGLE = {226, 10022, 9781, 10138, 9884, 9000};

// [Min, Max]
static const int16_t HAND_ANGLE_LIMITS[6][2] = {
    {226, 3676},
    {10022, 17837},
    {9781, 17606},
    {10138, 17654},
    {9884, 17486},
    {0, 9000} // 第6维固定
};

class Gripper2HandController : public rclcpp::Node {
public:
    Gripper2HandController() : Node("gripper_hand_controller") {
        // 1. 声明参数
        this->declare_parameter<std::string>("arm_side", "right_arm");
        this->declare_parameter<std::string>("gripper_sub_topic", "gripper_cmd");
        this->declare_parameter<double>("rate_hz", 20.0);

        // 2. 获取参数
        this->get_parameter("arm_side", arm_side_);
        std::string gripper_topic_name;
        this->get_parameter("gripper_sub_topic", gripper_topic_name);
        double rate_hz;
        this->get_parameter("rate_hz", rate_hz);

        RCLCPP_INFO(this->get_logger(), "开始启动手部controller: %s", arm_side_.c_str());

        // 3. 设置 QoS
        rclcpp::QoS qos(1);

        // 4. 创建发布者
        pub_pos_ = this->create_publisher<rm_ros_interfaces::msg::Handangle>(
            arm_side_ + "/rm_driver/set_hand_follow_pos_cmd", qos);
        pub_angle_ = this->create_publisher<rm_ros_interfaces::msg::Handangle>(
            arm_side_ + "/rm_driver/set_hand_follow_angle_cmd", qos);
        pub_speed_ = this->create_publisher<rm_ros_interfaces::msg::Handspeed>(
            arm_side_ + "/rm_driver/set_hand_speed_cmd", qos);
        pub_force_ = this->create_publisher<rm_ros_interfaces::msg::Handforce>(
            arm_side_ + "/rm_driver/set_hand_force_cmd", qos);

        // 5. 创建订阅者
        sub_status_ = this->create_subscription<rm_ros_interfaces::msg::Handstatus>(
            arm_side_ + "/rm_driver/udp_hand_status", qos,
            std::bind(&Gripper2HandController::hand_status_callback, this, std::placeholders::_1));

        sub_gripper_ = this->create_subscription<my_interfaces::msg::HeaderFloat32>(
            arm_side_ + "/" + gripper_topic_name, qos,
            std::bind(&Gripper2HandController::gripper_cmd_callback, this, std::placeholders::_1));

        // 初始化内部状态
        cmd_hand_angles_ = HAND_INIT_ANGLE;
        has_received_gripper_cmd_ = false;

        // 6. 执行硬件初始化
        init_hand_controller();

        // 7. 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz),
            std::bind(&Gripper2HandController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "ROHand 控制节点 (Angle模式) 已启动 [%s]", arm_side_.c_str());
        RCLCPP_INFO(this->get_logger(), "第6自由度固定值: 9000");
        RCLCPP_INFO(this->get_logger(), "------------------------------------------");
    }

private:
    // 成员变量
    std::string arm_side_;
    
    std::array<int16_t, 6> cmd_hand_angles_;
    
    double q_des_gripper_ = 0.0;
    bool has_received_gripper_cmd_ = false;

// Feedback buffers: 用于存储手反馈回来的真实状态
    std::array<uint16_t, 6> fb_hand_angle_; // 当前实际角度
    std::array<uint16_t, 6> fb_hand_pos_;   // 当前实际位置
    std::array<uint16_t, 6> fb_hand_state_; // 当前手指状态 (0松开, 1抓取等)
    std::array<uint16_t, 6> fb_hand_force_; // 当前电流/力
    uint16_t fb_hand_err_;                  // 系统错误码1为有错误,0为无错误

    // ROS 对象
    rclcpp::Publisher<rm_ros_interfaces::msg::Handangle>::SharedPtr pub_pos_;
    rclcpp::Publisher<rm_ros_interfaces::msg::Handangle>::SharedPtr pub_angle_;
    rclcpp::Publisher<rm_ros_interfaces::msg::Handspeed>::SharedPtr pub_speed_;
    rclcpp::Publisher<rm_ros_interfaces::msg::Handforce>::SharedPtr pub_force_;
    
    rclcpp::Subscription<rm_ros_interfaces::msg::Handstatus>::SharedPtr sub_status_;
    rclcpp::Subscription<my_interfaces::msg::HeaderFloat32>::SharedPtr sub_gripper_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    void init_hand_controller() {
        RCLCPP_INFO(this->get_logger(), "开始初始化%s的灵巧手参数...", arm_side_.c_str());

        // 1. 设置力矩
        auto force_msg = rm_ros_interfaces::msg::Handforce();
        force_msg.hand_force = 200;
        pub_force_->publish(force_msg);

        // 2. 设置速度
        auto speed_msg = rm_ros_interfaces::msg::Handspeed();
        speed_msg.hand_speed = 500;
        pub_speed_->publish(speed_msg);

        std::this_thread::sleep_for(1.0s);

        // 3. 设置初始角度
        auto angle_msg = rm_ros_interfaces::msg::Handangle();
        // 因为 msg.hand_angle 和 cmd_hand_angles_ 都是 std::array，所以可以直接赋值
        angle_msg.hand_angle = cmd_hand_angles_;
        angle_msg.block = false; 
        pub_angle_->publish(angle_msg);

        RCLCPP_INFO(this->get_logger(), "%s的灵巧手初始化完成。初始角度指令已发送。", arm_side_.c_str());
        std::this_thread::sleep_for(1.0s);
    }

// 【修改点】完整接收所有状态
    void hand_status_callback(const rm_ros_interfaces::msg::Handstatus::SharedPtr msg) {
        // 1. 赋值所有数组类型 (std::array 直接拷贝)
        fb_hand_angle_ = msg->hand_angle;
        fb_hand_pos_   = msg->hand_pos;
        fb_hand_state_ = msg->hand_state;
        fb_hand_force_ = msg->hand_force;
        
        // 2. 赋值标量类型 (错误码)
        fb_hand_err_   = msg->hand_err;

        // 3. 错误检查示例 (如果需要)
        if (fb_hand_err_ != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "[%s] 警告: 灵巧手系统报错 (Code: %u)", 
                arm_side_.c_str(), fb_hand_err_);
        }
    }

    void gripper_cmd_callback(const my_interfaces::msg::HeaderFloat32::SharedPtr msg) {
        if (std::isnan(msg->data)) {
            RCLCPP_WARN(this->get_logger(), "%s 手部收到 NaN 数据，忽略指令", arm_side_.c_str());
            return;
        }
        q_des_gripper_ = msg->data;
        has_received_gripper_cmd_ = true;
    }

    // 【修正 3】返回值改为 std::array
    std::array<int16_t, 6> gripper2handangle(double gripper_trigger) {
        // 1. 限制输入范围 [0.0, 1.0]
        double trigger = std::clamp(gripper_trigger, 0.0, 1.0);

        // 类型匹配：直接拷贝 array
        std::array<int16_t, 6> target_angles = cmd_hand_angles_;

        // 2. 仅计算前5个手指
        for (int i = 0; i < 5; ++i) {
            float p_min = static_cast<float>(HAND_ANGLE_LIMITS[i][0]);
            float p_max = static_cast<float>(HAND_ANGLE_LIMITS[i][1]);

            // 线性插值
            float angle = p_min + (p_max - p_min) * trigger;
            target_angles[i] = static_cast<int16_t>(angle);
        }

        // 3. 第6位保持不变
        return target_angles;
    }

    void control_loop() {
        if (!has_received_gripper_cmd_) {
            return;
        }

        // 【修正 4】接收变量也是 std::array
        std::array<int16_t, 6> target_angles = gripper2handangle(q_des_gripper_);
        
        auto msg = rm_ros_interfaces::msg::Handangle();
        
        // 赋值成功：两边类型一致
        msg.hand_angle = target_angles;
        msg.block = false; 
        
        pub_angle_->publish(msg);
        
        // 更新 buffer
        cmd_hand_angles_ = target_angles;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Gripper2HandController>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}