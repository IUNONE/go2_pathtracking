#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <Eigen/Dense>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/response.hpp"

#include "go2_move_client.h"


// Global variables for signal handling
std::atomic<bool> g_emergency_stop{false};

// Signal handler for Ctrl+C
void signal_handler(int signum) {
    if (signum == SIGINT) {
        g_emergency_stop.store(true);
        RCLCPP_WARN(rclcpp::get_logger("signal_handler"), "Emergency stop triggered!");
        
        // Give some time for the emergency stop command to be sent
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        rclcpp::shutdown();
    }
}

double normalize_angle(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
{
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

class Go2TrackNode : public rclcpp::Node 
{
public:
    Go2TrackNode(): Node("go2_pathtracking_node"), move_client_(this), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {   
        path_dt_ = declare_parameter<double>("path_dt", 0.1);
        control_hz_ = declare_parameter<int>("control_hz", 10);
        strategy_ = declare_parameter<std::string>("strategy", "open_loop");

        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        robot_odom_frame_ = declare_parameter<std::string>("robot_odom_frame", "odom");
        robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");

        is_ai_ctl_ = declare_parameter<bool>("is_ai_ctl", true);
        api_mode_  = declare_parameter<std::string>("api", "pos");

        kp_linear_ = declare_parameter<double>("kp_linear", 1.0);
        kd_linear_ = declare_parameter<double>("kd_linear", 0.1);

        kp_angular_ = declare_parameter<double>("kp_angular", 2.0);
        kd_angular_ = declare_parameter<double>("kd_angular", 0.2);

        if (is_ai_ctl_) {
            max_vel_x_ = 0.6;
            max_vel_y_ = 0.4;
            max_vel_yaw_ = 0.8; 
        }  else {
            max_vel_x_ = 2.5;
            max_vel_y_ = 1.0;
            max_vel_yaw_ = 4.0; 
        }

        max_pos_error_ = declare_parameter<double>("max_pos_error", 0.3);
        feedforward_ramp_time_ = declare_parameter<double>("feedforward_ramp_time", 0.2);

        pos_from_ = declare_parameter<std::string>("pos_from", "topic");
        path_topic_ = declare_parameter<std::string>("path_topic", "/planned_path");

        // --------------------------------------------------------------------------------
        using std::placeholders::_1;
        state_suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "lf/sportmodestate", 
            20,
            std::bind(&Go2TrackNode::_statecallback, this, _1));

        path_suber_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_, 
            5, 
            std::bind(&Go2TrackNode::PathCallback, this, _1)
        );

        response_suber_ = this->create_subscription<unitree_api::msg::Response>(
            "/api/sport/response",
            10,
            std::bind(&Go2TrackNode::_responsecallback, this, _1)
        );

        // --------------------------------------------------------------------------------
        if (api_mode_ == "pos") {
            RCLCPP_INFO(get_logger(), "API mode: %s, which provided by unitree official api", api_mode_.c_str());
        } else if (api_mode_ == "vel"){
            control_timer_ = create_wall_timer(
                std::chrono::duration<double>(1.0 / control_hz_),
                std::bind(&Go2TrackNode::MoveApiControlLoop, this)
            );
            RCLCPP_INFO(get_logger(), "API mode: %s. Control strategy: %s", api_mode_.c_str(), strategy_.c_str());
        } else {
            RCLCPP_FATAL(get_logger(), "Invalid api mode: %s, only support [pos, vel]", api_mode_.c_str());
            rclcpp::shutdown();
        }

        // --------------------------------------------------------------------------------
        move_client_.BalanceStand(req_);
        RCLCPP_INFO(get_logger(), "Call BalanceStand APi to be Ready");
    }

    void emergency_stop() {
        RCLCPP_WARN(get_logger(), "Emergency stop activated - sending zero velocity");
        move_client_.StopMove(req_);
        emergency_stop_active_.store(true);
    }

private:

    void _responsecallback(const unitree_api::msg::Response::SharedPtr msg) {
        auto api_id = msg->header.identity.api_id;
        auto code = msg->header.status.code;

        std::string api_name;
        switch (api_id) {
            case ROBOT_SPORT_API_ID_MOVE: 
                api_name = "MOVE"; 
                break;
            case ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW: 
                api_name = "TRAJECTORYFOLLOW"; 
                break;
            case ROBOT_SPORT_API_ID_BALANCESTAND: 
                api_name = "BALANCESTAND"; 
                break;
            case ROBOT_SPORT_API_ID_STOPMOVE: 
                api_name = "STOPMOVE"; 
                break;
            default:
                return;
        }

        std::string code_desc;
        switch (code) {
            case 0: code_desc = "success"; break;
            case -1: code_desc = "task timeout"; break;
            case -2: code_desc = "task unknown error"; break;
            default: code_desc = "unknown"; break;
        }

        RCLCPP_INFO(this->get_logger(), "[PathTracking] API Response - ID: %d (%s), Code: %d (%s)", 
                    api_id, api_name.c_str(), code, code_desc.c_str());

        if (code != 0) {
            RCLCPP_ERROR(this->get_logger(), "[PathTracking] API call failed: %s", code_desc.c_str());
        }
    }

    void _statecallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = *msg;
    }

    void update_current_pose(){
        
        if (pos_from_ == "tf") {
            try {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                    robot_odom_frame_, robot_base_frame_, 
                    tf2::TimePointZero, 
                    tf2::durationFromSec(0.1)
                );
                current_x_ = transform.transform.translation.x;
                current_y_ = transform.transform.translation.y;
                current_yaw_ = get_yaw_from_quaternion(transform.transform.rotation);
                
                return;
            } 
            catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(get_logger(), "Call StopMove Api. TF lookup failed for %s", ex.what());
                move_client_.StopMove(req_);
                return;
            }
        } else if (pos_from_ == "topic") {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_x_ = state_.position[0];
            current_y_ = state_.position[1];
            current_yaw_ = state_.imu_state.rpy[2];
        }
    }

    /** 
        - extract x, y, yaw, vx, vy, vyaw from ros Path msg 
        - vx, vy, vyaw is reference velocity for open-loop
        - reset Error
    **/ 
    void PathCallback(const nav_msgs::msg::Path::SharedPtr msg) {  
        
        std::lock_guard<std::mutex> lock(path_mutex_);
        
        size_t N = msg->poses.size();
        if (N < 2) {
            RCLCPP_WARN(get_logger(), "Received path with less than 2 points, ignoring");
            plan_path_local_.resize(0, 6);
            plan_path_global_.resize(0, 6);
            return;
        }

        // 1. get x, y, yaw with init state 0,0,0
        plan_path_local_ = Eigen::MatrixXd::Zero(N+1, 6);
        for (size_t i = 0; i < N; ++i) {
            double x = msg->poses[i].pose.position.x;
            double y = msg->poses[i].pose.position.y;
            double yaw = get_yaw_from_quaternion(msg->poses[i].pose.orientation);

            plan_path_local_(i+1, 0) = x;
            plan_path_local_(i+1, 1) = y;
            plan_path_local_(i+1, 2) = yaw;
        }
        
        // 2. compute (vx, vy, vyaw) from [i+1] - [i]
        // and the last is zero
        for (size_t i = 0; i < N; ++i) {
            double vx   = (plan_path_local_(i+1, 0) - plan_path_local_(i, 0)) / path_dt_;
            double vy   = (plan_path_local_(i+1, 1) - plan_path_local_(i, 1)) / path_dt_;
            double vyaw = normalize_angle(plan_path_local_(i+1, 2) - plan_path_local_(i, 2)) / path_dt_;

            plan_path_local_(i, 3) = vx;
            plan_path_local_(i, 4) = vy;
            plan_path_local_(i, 5) = vyaw;
        }

        // 3. path verison
        path_version_.fetch_add(1);
        path_updated_ = true;
        RCLCPP_INFO(get_logger(), "#########################################");
        RCLCPP_INFO(get_logger(), "New path received with %zu points, version: %lu, strategy: %s", 
                   N, path_version_.load(), strategy_.c_str());
        path_start_time_ = now();

        // 4. transform from local (in base_link frame) to global(in odom frame) 
        update_current_pose();
        if (api_mode_ == "pos") {
            plan_path_global_ = plan_path_local_;
            plan_path_global_.leftCols(3) = plan_path_local_.leftCols(3);

            Eigen::Matrix2d R;
            R << cos(current_yaw_), -sin(current_yaw_),
                sin(current_yaw_),  cos(current_yaw_);
            Eigen::MatrixXd XY_local =  plan_path_local_.leftCols(2).transpose();
            Eigen::MatrixXd XY_global = (R * XY_local).colwise() + Eigen::Vector2d(current_x_, current_y_);
            
            plan_path_global_.leftCols(2) = XY_global.transpose();
            plan_path_global_.col(2) = plan_path_local_.col(2).array() + current_yaw_;

            Eigen::MatrixXd Vxy_local = plan_path_local_.block(0, 3, N+1, 2).transpose();
            Eigen::MatrixXd Vxy_global = R * Vxy_local;

            plan_path_global_.block(0, 3, N+1, 2) = Vxy_global.transpose();
            plan_path_global_.col(5) = plan_path_local_.col(5).array();
            
            RCLCPP_INFO(this->get_logger(), "[PathTracking] Calling FollowTrajApi()");
            FollowTrajApi();
        } else if (strategy_ == "pd") {
            // reset for pd control states
            path_start_pos_global_ = {current_x_, current_y_, current_yaw_};
            prev_pos_error_ = {0.0, 0.0, 0.0};
            prev_time_valid_ = false;
        }
    }

    //----------------------------------------------------------
    // trajectory following api need global coord

    void FollowTrajApi() {
        
        // check path
        std::lock_guard<std::mutex> lock(path_mutex_);
        uint64_t latest_version = path_version_.load();
        if (current_path_version_ != latest_version) {
            current_path_version_ = latest_version;
            RCLCPP_INFO(get_logger(), "Switched to path version: %lu", current_path_version_.load());
        }
        if (!path_updated_ || plan_path_global_.rows() == 0) {
            move_client_.StopMove(req_);
            RCLCPP_INFO(get_logger(), "Empty Path. Call StopMove Api");
            return;
        }

        // convert to api format
        std::vector<PathPoint> path;
        for (int i = 1; i <= 30; i++) {
            PathPoint path_point;
            path_point.timeFromStart = i * path_dt_;

            if (i < plan_path_global_.rows()) {
                path_point.x = plan_path_global_(i, 0);
                path_point.y = plan_path_global_(i, 1);
                path_point.yaw = plan_path_global_(i, 2);
                path_point.vx = std::clamp(plan_path_global_(i, 3), -max_vel_x_, max_vel_x_);
                path_point.vy = std::clamp(plan_path_global_(i, 4), -max_vel_y_, max_vel_y_);
                path_point.vyaw = std::clamp(plan_path_global_(i, 5), -max_vel_yaw_, max_vel_yaw_);
            } else {
                // repeat the last one
                path_point.x = path.back().x;
                path_point.y = path.back().y;
                path_point.yaw = path.back().yaw;
                path_point.vx = 0.0;
                path_point.vy = 0.0;
                path_point.vyaw = 0.0;
            }
            path.push_back(path_point);
        }
        move_client_.TrajectoryFollow(req_, path);
        RCLCPP_INFO(this->get_logger(), "Call TrajectoryFollow API");
    }

    //----------------------------------------------------------

    void MoveApiControlLoop() {

        if (emergency_stop_active_.load() || g_emergency_stop.load()) {
            move_client_.StopMove(req_);
            RCLCPP_INFO(get_logger(), "Emergency Stop. Call StopMove Api");
            return;
        }
        if (strategy_ == "pd") {
            update_current_pose();
        }
        
        // check path
        std::lock_guard<std::mutex> lock(path_mutex_);
        uint64_t latest_version = path_version_.load();
        if (current_path_version_ != latest_version) {
            current_path_version_ = latest_version;
            RCLCPP_INFO(get_logger(), "Switched to path version: %lu", current_path_version_.load());
        }
        if (!path_updated_ || plan_path_local_.rows() == 0) {
            move_client_.StopMove(req_);
            RCLCPP_INFO(get_logger(), "Empty Path. Call StopMove Api");
            return;
        }
        auto current_time = now();
        double elapsed_time = (current_time - path_start_time_).seconds();
        bool at_path_end = (elapsed_time / path_dt_) >= (plan_path_local_.rows() - 1);
        if (at_path_end) {
            RCLCPP_INFO(get_logger(), "Path execution completed. Call StopMove Api");
            path_updated_ = false;
            move_client_.StopMove(req_);
            return;
        }

        // start control strategy
        double x_vel = 0.0;
        double y_vel = 0.0;
        double yaw_vel = 0.0;

        if (strategy_ == "open_loop") {
            std::tie(x_vel, y_vel, yaw_vel) = calculate_openloop_control(elapsed_time);
        } 
        else if (strategy_ == "pd") {
            std::tie(x_vel, y_vel, yaw_vel) = calculate_pd_control(elapsed_time);
        }   

        // Apply velocity limits
        x_vel = std::clamp(x_vel, -max_vel_x_, max_vel_x_);
        y_vel = std::clamp(y_vel, -max_vel_y_, max_vel_y_);
        yaw_vel = std::clamp(yaw_vel, -max_vel_yaw_, max_vel_yaw_);
    }

    std::tuple<double, double, double> calculate_openloop_control(double elapsed_time){
        
        double subgoal_idx = elapsed_time / path_dt_;
        size_t cur_idx = static_cast<size_t>(subgoal_idx);

        double x_vel = plan_path_local_(cur_idx,3);
        double y_vel = plan_path_local_(cur_idx,4);
        double angular_vel = plan_path_local_(cur_idx,5);

        return std::make_tuple(x_vel, y_vel, angular_vel);
    }

    std::tuple<double, double, double> calculate_pd_control(double elapsed_time){
        
        size_t subgoal_idx = static_cast<size_t>(elapsed_time / path_dt_) + 1;
        
        // error in path start frame
        double x0 = path_start_pos_global_[0];
        double y0 = path_start_pos_global_[1];
        double yaw0 = path_start_pos_global_[2];
        double current_x_local_ = cos(yaw0) * (current_x_ - x0) + sin(yaw0) * (current_y_ - y0);
        double current_y_local_ = -sin(yaw0) * (current_x_ - x0) + cos(yaw0) * (current_y_ - y0);
        double current_yaw_local_ = normalize_angle(current_yaw_ - yaw0);
        std::array<double, 3> pos_error = {
            plan_path_local_(subgoal_idx,0) - current_x_local_, 
            plan_path_local_(subgoal_idx,1) - current_y_local_, 
            normalize_angle(plan_path_local_(subgoal_idx,2)- current_yaw_local_)
        };
        double pos_err_mag = std::hypot(pos_error[0], pos_error[1]);
        if (pos_err_mag > max_pos_error_) {
            pos_error[0] *= (max_pos_error_ / pos_err_mag);
            pos_error[1] *= (max_pos_error_ / pos_err_mag);
        }

        // pd
        std::array<double, 3> desired_vel_pd;
        auto now_time = now();
        if (prev_time_valid_) {
            double dt = (now_time - prev_time_).seconds();
            desired_vel_pd = {
                kp_linear_  * pos_error[0] + kd_linear_  * (pos_error[0] - prev_pos_error_[0]) / dt,
                kp_linear_  * pos_error[1] + kd_linear_  * (pos_error[1] - prev_pos_error_[1]) / dt,
                kp_angular_ * pos_error[2] + kd_angular_ * (pos_error[2] - prev_pos_error_[2]) / dt
            };
        } else {
            desired_vel_pd = {
                kp_linear_  * pos_error[0], 
                kp_linear_  * pos_error[1],
                kp_angular_ * pos_error[2]
            };
            prev_time_valid_ = true;
        }
        
        // feedward
        std::array<double, 3> ref_vel = {
            plan_path_local_(subgoal_idx-1, 3), 
            plan_path_local_(subgoal_idx-1, 4), 
            plan_path_local_(subgoal_idx-1, 5)
        };
        double feedforward_weight = std::min(1.0, elapsed_time / feedforward_ramp_time_);
        std::array<double, 3> total_desired_vel = {
            feedforward_weight * ref_vel[0] + desired_vel_pd[0],
            feedforward_weight * ref_vel[1] + desired_vel_pd[1],
            feedforward_weight * ref_vel[2] + desired_vel_pd[2]
        };
        
        prev_time_ = now_time;
        prev_pos_error_ = pos_error;

        // transform to base_link frame
        double dyaw = current_yaw_ - yaw0;
        double x_vel =  total_desired_vel[0] * cos(dyaw) + total_desired_vel[1] * sin(dyaw);
        double y_vel = -total_desired_vel[0] * sin(dyaw) + total_desired_vel[1] * cos(dyaw);
        double angular_vel = total_desired_vel[2];

        return std::make_tuple(x_vel, y_vel, angular_vel);
    }

    //----------------------------------------------------------
    // Emergency stop state
    std::atomic<bool> emergency_stop_active_{false};

    // ros msg path
    std::string path_topic_;
    double path_dt_;
    std::mutex path_mutex_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_suber_;
    Eigen::MatrixXd plan_path_local_;       // [n, 6]  x, y, yaw, vx, vy, vyaw in local
    Eigen::MatrixXd plan_path_global_;      // [n, 6]  x, y, yaw, vx, vy, vyaw in global
    std::atomic<uint64_t> path_version_{0};
    std::atomic<uint64_t> current_path_version_{0};
    bool path_updated_ = false;
    rclcpp::Time path_start_time_;

    // ros api client
    Go2MoveClient move_client_;
    unitree_api::msg::Request req_;
    rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr response_suber_;
    double max_vel_x_ = 0.0, max_vel_y_ = 0.0, max_vel_yaw_ = 0.0; 

    // tf for close-loop control
    std::string pos_from_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string map_frame_, robot_odom_frame_, robot_base_frame_;
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
    
    // state topic for close-loop control or transform to init
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber_;
    unitree_go::msg::SportModeState state_;
    std::mutex state_mutex_;
    
    // control strategy
    std::string strategy_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    int control_hz_;

    std::string api_mode_;
    bool is_ai_ctl_;
    
    // pd + feedforward
    std::array<double,3> path_start_pos_global_ = {0.0,0.0, 0.0};
    std::array<double,3> prev_pos_error_ = {0.0,0.0, 0.0};
    double max_pos_error_;
    double kp_linear_, kd_linear_;
    double kp_angular_, kd_angular_;
    double feedforward_ramp_time_;

    rclcpp::Time prev_time_;
    bool prev_time_valid_ = false;
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2TrackNode>();

    // Register signal handler for Ctrl+C
    signal(SIGINT, signal_handler);

    RCLCPP_INFO(node->get_logger(), "Emergency stop enabled - press Ctrl+C to stop robot");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
