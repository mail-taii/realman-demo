#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Byte.h>
#include <std_msgs/String.h>
#include <embodied_arm_msgs/Joint_Current.h>
#include <embodied_arm_msgs/LiftState.h>

class CurrentSafetyMonitor {
public:
    CurrentSafetyMonitor(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh_(nh), pnh_(pnh) {
        // Parameters
        pnh_.param<bool>("safety_enabled", safety_enabled_, true);
        pnh_.param<int>("safety_level", safety_level_, 1);
        pnh_.param<double>("joint_current_threshold", joint_current_threshold_default_, 2.0);
        pnh_.param<double>("lift_current_threshold", lift_current_threshold_, 3.0);
        pnh_.param<double>("trip_hold_seconds", trip_hold_seconds_, 0.1);

        // Configure per-joint thresholds (optional). If not set, use default for all joints.
        std::vector<double> joint_thresholds_param;
        if (pnh_.getParam("joint_thresholds", joint_thresholds_param)) {
            joint_current_thresholds_ = joint_thresholds_param;
        }

        // Subscribers
        sub_joint_current_ = nh_.subscribe("rm_driver/Udp_Joint_Current", 10, &CurrentSafetyMonitor::onJointCurrent, this);
        sub_lift_state_ = nh_.subscribe("rm_driver/LiftState", 10, &CurrentSafetyMonitor::onLiftState, this);
        sub_arm_current_status_ = nh_.subscribe("rm_driver/Udp_Arm_Current_status", 10, &CurrentSafetyMonitor::onArmCurrentStatus, this);

        // Publishers for safety actions
        pub_emergency_stop_ = nh_.advertise<std_msgs::Empty>("rm_driver/Emergency_Stop", 1);
        pub_set_arm_power_ = nh_.advertise<std_msgs::Byte>("rm_driver/SetArmPower", 1);

        // Safety status
        pub_safety_status_ = pnh_.advertise<std_msgs::String>("safety_status", 1, true);

        ROS_INFO("current_safety_monitor started. enabled=%s level=%d", safety_enabled_ ? "true" : "false", safety_level_);
    }

private:
    void onJointCurrent(const embodied_arm_msgs::Joint_Current &msg) {
        if (!safety_enabled_) return;
        const ros::Time now = ros::Time::now();

        double threshold_default = joint_current_threshold_default_;
        ensureThresholdSize(msg.joint_current.size());

        bool over = false;
        for (size_t i = 0; i < msg.joint_current.size(); ++i) {
            double th = (i < joint_current_thresholds_.size() && joint_current_thresholds_[i] > 0.0)
                            ? joint_current_thresholds_[i]
                            : threshold_default;
            if (i < last_over_start_.size()) {
                if (msg.joint_current[i] > th) {
                    if (last_over_start_[i].isZero()) {
                        last_over_start_[i] = now;
                    }
                    if ((now - last_over_start_[i]).toSec() >= trip_hold_seconds_) {
                        over = true;
                        over_reason_ = "joint_" + std::to_string(i) + " current " + std::to_string(msg.joint_current[i]) + 
                                       ">" + std::to_string(th);
                        break;
                    }
                } else {
                    last_over_start_[i] = ros::Time();
                }
            }
        }

        if (over) triggerSafety("joint_overcurrent");
    }

    void onLiftState(const embodied_arm_msgs::LiftState &msg) {
        if (!safety_enabled_) return;
        const ros::Time now = ros::Time::now();

        // Lift current is integer in message; convert to Amp if needed by driver spec
        const double current_a = static_cast<double>(msg.current) / 1000.0; // driver fills mA in some places
        if (current_a > lift_current_threshold_) {
            if (lift_over_start_.isZero()) lift_over_start_ = now;
            if ((now - lift_over_start_).toSec() >= trip_hold_seconds_) {
                over_reason_ = std::string("lift current ") + std::to_string(current_a) + ">" + std::to_string(lift_current_threshold_);
                triggerSafety("lift_overcurrent");
            }
        } else {
            lift_over_start_ = ros::Time();
        }
    }

    void onArmCurrentStatus(const embodied_arm_msgs::Arm_Current_Status &msg) {
        // If driver already reports abnormal current status, trip immediately
        if (!safety_enabled_) return;
        if (!msg.arm_current_status.empty() && msg.arm_current_status != "normal") {
            over_reason_ = std::string("arm_current_status=") + msg.arm_current_status;
            triggerSafety("arm_current_status");
        }
    }

    void triggerSafety(const std::string &reason_code) {
        // Debounce by not re-triggering too frequently
        if ((ros::Time::now() - last_trip_time_).toSec() < 0.5) return;
        last_trip_time_ = ros::Time::now();

        // Publish safety status
        std_msgs::String status;
        status.data = std::string("TRIPPED ") + reason_code + ": " + over_reason_;
        pub_safety_status_.publish(status);

        // 1) Emergency stop
        std_msgs::Empty estop;
        pub_emergency_stop_.publish(estop);

        // 2) Optionally cut power based on safety_level
        if (safety_level_ >= 2) {
            std_msgs::Byte power;
            power.data = 0; // 0 -> power off
            pub_set_arm_power_.publish(power);
        }

        ROS_ERROR("current_safety_monitor: %s", status.data.c_str());
    }

    void ensureThresholdSize(size_t n) {
        if (last_over_start_.size() < n) last_over_start_.resize(n);
        if (joint_current_thresholds_.size() < n) joint_current_thresholds_.resize(n, -1.0);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Params
    bool safety_enabled_ {true};
    int safety_level_ {1}; // 1: e-stop only, 2: e-stop + power off
    double joint_current_threshold_default_ {2.0};
    double lift_current_threshold_ {3.0};
    double trip_hold_seconds_ {0.1};

    // State
    std::vector<double> joint_current_thresholds_;
    std::vector<ros::Time> last_over_start_;
    ros::Time lift_over_start_;
    ros::Time last_trip_time_;
    std::string over_reason_;

    // Subs/Pubs
    ros::Subscriber sub_joint_current_;
    ros::Subscriber sub_lift_state_;
    ros::Subscriber sub_arm_current_status_;
    ros::Publisher pub_emergency_stop_;
    ros::Publisher pub_set_arm_power_;
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


