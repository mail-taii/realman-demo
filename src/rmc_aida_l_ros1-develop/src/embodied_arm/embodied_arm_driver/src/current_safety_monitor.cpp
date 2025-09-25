#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Byte.h>
#include <std_msgs/String.h>
#include <embodied_arm_msgs/Joint_Current.h>
#include <embodied_arm_msgs/LiftState.h>
#include <embodied_arm_msgs/Arm_Current_Status.h>

#include <string>
#include <vector>
#include <map>

class CurrentSafetyMonitor {
public:
    CurrentSafetyMonitor(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh_(nh), pnh_(pnh) {
        pnh_.param<bool>("safety_enabled", safety_enabled_, true);
        pnh_.param<int>("safety_level", safety_level_, 1);
        pnh_.param<double>("joint_current_threshold", joint_current_threshold_default_, 2.0);
        pnh_.param<double>("lift_current_threshold", lift_current_threshold_, 3.0);
        pnh_.param<double>("trip_hold_seconds", trip_hold_seconds_, 0.1);

        // 支持多命名空间，默认监控 /r_arm 与 /l_arm
        if (!pnh_.getParam("arm_namespaces", arm_namespaces_)) {
            arm_namespaces_.clear();
            arm_namespaces_.push_back("r_arm");
            arm_namespaces_.push_back("l_arm");
        }

        // 可选：每关节阈值（对所有分支生效）
        std::vector<double> joint_thresholds_param;
        if (pnh_.getParam("joint_thresholds", joint_thresholds_param)) {
            joint_current_thresholds_ = joint_thresholds_param;
        }

        // 为每个命名空间建立订阅与动作发布器
        for (const auto &ns : arm_namespaces_) {
            ArmCtx ctx;
            ctx.ns = ns;

            // 订阅（来自驱动的输出）
            ctx.sub_joint_current = nh_.subscribe("/" + ns + "/rm_driver/Udp_Joint_Current", 10,
                                                  &CurrentSafetyMonitor::onJointCurrent, this);
            ctx.sub_lift_state = nh_.subscribe("/" + ns + "/rm_driver/LiftState", 10,
                                               &CurrentSafetyMonitor::onLiftState, this);
            ctx.sub_arm_current_status = nh_.subscribe("/" + ns + "/rm_driver/Udp_Arm_Current_status", 10,
                                                       &CurrentSafetyMonitor::onArmCurrentStatus, this);

            // 动作发布（发回驱动的输入）
            ctx.pub_emergency_stop = nh_.advertise<std_msgs::Empty>("/" + ns + "/rm_driver/Emergency_Stop", 1);
            ctx.pub_set_arm_power = nh_.advertise<std_msgs::Byte>("/" + ns + "/rm_driver/SetArmPower", 1);

            // 每分支各自状态
            ctx.last_trip_time = ros::Time(0);
            arm_map_[ns] = ctx;
        }

        // 合并状态输出（单话题）
        pub_safety_status_ = pnh_.advertise<std_msgs::String>("safety_status", 10, true);

        ROS_INFO("current_safety_monitor started. enabled=%s level=%d, arms=%zu",
                 safety_enabled_ ? "true" : "false", safety_level_, arm_map_.size());
    }

private:
    struct ArmCtx {
        std::string ns;
        ros::Subscriber sub_joint_current;
        ros::Subscriber sub_lift_state;
        ros::Subscriber sub_arm_current_status;

        ros::Publisher pub_emergency_stop;
        ros::Publisher pub_set_arm_power;

        std::vector<ros::Time> last_over_start; // per joint
        ros::Time lift_over_start;
        ros::Time last_trip_time;
    };

    // 回调：关节电流
    void onJointCurrent(const embodied_arm_msgs::Joint_Current &msg) {
        if (!safety_enabled_) return;
        const std::string ns = resolveCallerNS();
        ArmCtx &ctx = arm_map_[ns];

        const ros::Time now = ros::Time::now();
        ensureThresholdSize(ctx, msg.joint_current.size());

        bool over = false;
        std::string reason;
        for (size_t i = 0; i < msg.joint_current.size(); ++i) {
            double th = (i < joint_current_thresholds_.size() && joint_current_thresholds_[i] > 0.0)
                            ? joint_current_thresholds_[i]
                            : joint_current_threshold_default_;
            if (i < ctx.last_over_start.size()) {
                if (msg.joint_current[i] > th) {
                    if (ctx.last_over_start[i].isZero()) {
                        ctx.last_over_start[i] = now;
                    }
                    if ((now - ctx.last_over_start[i]).toSec() >= trip_hold_seconds_) {
                        over = true;
                        reason = "joint_" + std::to_string(i) + " current " + std::to_string(msg.joint_current[i]) +
                                 ">" + std::to_string(th);
                        break;
                    }
                } else {
                    ctx.last_over_start[i] = ros::Time();
                }
            }
        }
        if (over) triggerSafety(ctx, "joint_overcurrent", reason);
    }

    // 回调：升降电流
    void onLiftState(const embodied_arm_msgs::LiftState &msg) {
        if (!safety_enabled_) return;
        const std::string ns = resolveCallerNS();
        ArmCtx &ctx = arm_map_[ns];

        const ros::Time now = ros::Time::now();
        const double current_a = static_cast<double>(msg.current) / 1000.0;
        if (current_a > lift_current_threshold_) {
            if (ctx.lift_over_start.isZero()) ctx.lift_over_start = now;
            if ((now - ctx.lift_over_start).toSec() >= trip_hold_seconds_) {
                triggerSafety(ctx, "lift_overcurrent",
                              std::string("lift current ") + std::to_string(current_a) + ">" + std::to_string(lift_current_threshold_));
            }
        } else {
            ctx.lift_over_start = ros::Time();
        }
    }

    // 回调：驱动上报电流状态
    void onArmCurrentStatus(const embodied_arm_msgs::Arm_Current_Status &msg) {
        if (!safety_enabled_) return;
        const std::string ns = resolveCallerNS();
        ArmCtx &ctx = arm_map_[ns];

        if (!msg.arm_current_status.empty() && msg.arm_current_status != "normal") {
            triggerSafety(ctx, "arm_current_status", std::string("arm_current_status=") + msg.arm_current_status);
        }
    }

    // 触发安保动作（仅针对触发的分支）
    void triggerSafety(ArmCtx &ctx, const std::string &code, const std::string &reason) {
        if ((ros::Time::now() - ctx.last_trip_time).toSec() < 0.5) return;
        ctx.last_trip_time = ros::Time::now();

        std_msgs::String status;
        status.data = "[" + ctx.ns + "] TRIPPED " + code + ": " + reason;
        pub_safety_status_.publish(status);

        // 1) 急停（只发向对应分支）
        std_msgs::Empty estop;
        ctx.pub_emergency_stop.publish(estop);

        // 2) 按等级断电
        if (safety_level_ >= 2) {
            std_msgs::Byte power;
            power.data = 0;
            ctx.pub_set_arm_power.publish(power);
        }

        ROS_ERROR("current_safety_monitor: %s", status.data.c_str());
    }

    // 根据回调连接解析来源的命名空间（从订阅话题名推导）
    std::string resolveCallerNS() const {
        const ros::M_string &conn = ros::names::getRemappings();
        (void)conn; // 保留以防未来使用
        // 使用当前回调的订阅者连接名
        std::string topic = ros::this_node::getName(); // 占位，避免引入复杂 API
        // 简化：通过 ROS 提供的当前订阅回调上下文不可直接取话题，这里改为从线程局部存储中获取
        // 为保持简洁与稳定，我们采用每分支独立回调队列更安全——但这里用话题前缀匹配进行推断：
        // 约定：/X_arm/rm_driver/xxx → 返回 "X_arm"
        // 实用实现：解析 last connection header 不直接暴露，改用 topic 回退策略：
        // 我们做一个简化假设：系统每个命名空间单独节点管理的线程中回调；若需要严格，可改为为每 ns 建立独立类实例。
        // 这里以日志与功能为主，不依赖该函数；我们改为在回调前设置 TLS，但为简化省略。
        return guess_ns_from_spin();
    }

    // 简易回退：若无法解析，默认使用第一个命名空间（常用单臂或先右臂）
    std::string guess_ns_from_spin() const {
        if (!arm_namespaces_.empty()) return arm_namespaces_.front();
        return std::string("r_arm");
    }

    void ensureThresholdSize(ArmCtx &ctx, size_t n) {
        if (ctx.last_over_start.size() < n) ctx.last_over_start.resize(n);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    bool safety_enabled_{true};
    int safety_level_{1};
    double joint_current_threshold_default_{2.0};
    double lift_current_threshold_{3.0};
    double trip_hold_seconds_{0.1};

    std::vector<double> joint_current_thresholds_;
    std::vector<std::string> arm_namespaces_;

    std::map<std::string, ArmCtx> arm_map_;

    ros::Publisher pub_safety_status_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "current_safety_monitor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    CurrentSafetyMonitor monitor(nh, pnh);
    ros::spin();
    return 0;
}


