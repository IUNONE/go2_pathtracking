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

#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/response.hpp"

#include "go2_move_client.h"

using std::placeholders::_1;

// Custom angle utility functions for Foxy compatibility
namespace angle_utils {
    inline double shortest_angular_distance(double from, double to) {
        double diff = to - from;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        return diff;
    }
}

class Go2MoveClientNode : public rclcpp::Node 
{
public:
    explicit Go2MoveClientNode(): Node("go2_pathtracking_node"), move_client_(this) 
    {
        this->declare_parameter<std::string>("api", "pos");
        this->declare_parameter<std::string>("strategy", "open_loop");
        this->declare_parameter<int>("control_hz", 50);
        this->declare_parameter<bool>("is_ai_ctl", true);

        api_mode_ = this->get_parameter("api").as_string();
        strategy_ = this->get_parameter("strategy").as_string();
        control_hz_ = this->get_parameter("control_hz").as_int();
        is_ai_ctl_ = this->get_parameter("is_ai_ctl").as_bool();

        RCLCPP_INFO(this->get_logger(), "[PathTracking] API mode: %s, Strategy: %s, Control Hz: %d, AI Control: %s", 
                   api_mode_.c_str(), strategy_.c_str(), control_hz_, is_ai_ctl_ ? "true" : "false");
        
        // --------------------------------------------------------------------------------
        // subscribers
        state_suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "lf/sportmodestate", 
            1,
            std::bind(&Go2MoveClientNode::statecallback, this, _1)
        );

        plan_path_suber_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 
            1, 
            std::bind(&Go2MoveClientNode::plannercallback, this, _1)
        );

        response_suber_ = this->create_subscription<unitree_api::msg::Response>(
            "/api/sport/response",
            10,
            std::bind(&Go2MoveClientNode::responsecallback, this, _1)
        );
        
        // --------------------------------------------------------------------------------
        // control loop for vel control
        if (api_mode_ == "vel") {
            control_thread_ = std::thread([this] { MoveApiControlLoop(); });
        } else if (api_mode_ != "pos") {
            RCLCPP_ERROR(this->get_logger(), "[PathTracking] Unknown API mode: %s", api_mode_.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "[PathTracking] Node initialized successfully");
    }

    ~Go2MoveClientNode() {
        if (api_mode_ == "vel") {
            if (control_thread_.joinable()) {
                control_thread_.join();
            }
        }
    }

    void plannercallback(const nav_msgs::msg::Path::SharedPtr msg) {  
        
        std::lock_guard<std::mutex> lock(path_mutex_);

        size_t N = msg->poses.size();
        if (N == 0) {
            RCLCPP_WARN(this->get_logger(), "Received empty path.");
            plan_path_local_.resize(0, 3);
            plan_path_global_.resize(0, 3);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "[PathTracking] Received planned path ros2 msg with %zu points ", N);

        // extract x, y, yaw from ros msg
        plan_path_local_.resize(N, 3);
        for (size_t i = 0; i < N; ++i) {
            const auto& pose = msg->poses[i].pose;

            tf2::Quaternion q(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            plan_path_local_(i, 0) = pose.position.x;
            plan_path_local_(i, 1) = pose.position.y;
            plan_path_local_(i, 2) = yaw;
        }
        
        // Debug: print first point before transformation
        RCLCPP_INFO(this->get_logger(), "[PathTracking] First point local: x=%.3f, y=%.3f, yaw=%.3f", 
                plan_path_local_(0, 0), plan_path_local_(0, 1), plan_path_local_(0, 2));
        
        // transform planned path from local to global 
        Eigen::MatrixXd traj_global(N, 3);
        Eigen::Matrix2d R;
        R << cos(yaw0_), -sin(yaw0_),
            sin(yaw0_),  cos(yaw0_);
        Eigen::MatrixXd XY_local =  plan_path_local_.leftCols(2).transpose();
        Eigen::MatrixXd XY_global = (R * XY_local).colwise() + Eigen::Vector2d(px0_, py0_);
        traj_global.leftCols(2) = XY_global.transpose();
        traj_global.col(2) = plan_path_local_.col(2).array() + yaw0_;
        
        plan_path_global_ = traj_global;
        
        // Debug: print first point after transformation
        RCLCPP_INFO(this->get_logger(), "[PathTracking] First point global: x=%.3f, y=%.3f, yaw=%.3f", 
                plan_path_global_(0, 0), plan_path_global_(0, 1), plan_path_global_(0, 2));
        
        // apply api if needed
        if (api_mode_ == "pos") {
            RCLCPP_INFO(this->get_logger(), "[PathTracking] Calling FollowTrajApi()");
            FollowTrajApi();
        } else if (api_mode_ == "vel") {
            path_start_time_ = this->get_clock()->now();
            target_point_idx_ = -1;
        } else {
            RCLCPP_ERROR(this->get_logger(), "[PathTracking] Unknown api mode: %s", api_mode_.c_str());
        }
    }

    void statecallback(const unitree_go::msg::SportModeState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = *msg;

        RCLCPP_DEBUG(this->get_logger(), "Position: %.3f, %.3f, %.3f", 
            state_.position[0], state_.position[1], state_.position[2]);
        RCLCPP_DEBUG(this->get_logger(), "IMU rpy: %.3f, %.3f, %.3f",
            state_.imu_state.rpy[0], state_.imu_state.rpy[1], state_.imu_state.rpy[2]);
    }

    void responsecallback(const unitree_api::msg::Response::SharedPtr msg) {
        auto api_id = msg->header.identity.api_id;
        auto code = msg->header.status.code;
        
        // Only process MOVE and TRAJECTORYFOLLOW API responses
        if (api_id == ROBOT_SPORT_API_ID_MOVE || api_id == ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW) {
            std::string api_name = (api_id == ROBOT_SPORT_API_ID_MOVE) ? "MOVE" : "TRAJECTORYFOLLOW";
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
    }

    void SetInitState(const unitree_go::msg::SportModeState& msg) {
        px0_ = msg.position[0];
        py0_ = msg.position[1];
        yaw0_ = msg.imu_state.rpy[2];

        RCLCPP_INFO(this->get_logger(),
            "[PathTracking] Initial robot state set - x: %.3f, y: %.3f, yaw: %.3f",
            px0_, py0_, yaw0_);
    }

    void ClampVelocity(double& vx, double& vy, double& vyaw) {
        if (is_ai_ctl_) {
            // AI control limits
            vx = std::clamp(vx, -0.6, 0.6);
            vy = std::clamp(vy, -0.4, 0.4);
            vyaw = std::clamp(vyaw, -0.8, 0.8);
        } else {
            // Non-AI control limits
            vx = std::clamp(vx, -2.5, 3.8);
            vy = std::clamp(vy, -1.0, 1.0);
            vyaw = std::clamp(vyaw, -4.0, 4.0);
        }
    }

    // trajectory following api need global coord
    void FollowTrajApi() {
        Eigen::MatrixXd plan_path_copy;
        unitree_go::msg::SportModeState state_copy;
        {
            // Use C++17 structured binding to lock two mutexes safely
            std::scoped_lock lock(path_mutex_, state_mutex_);
            plan_path_copy = plan_path_global_;
            state_copy = state_;
        }

        int N = plan_path_copy.rows();
        
        // diff to get vel
        Eigen::RowVector3d current;
        double current_x = state_copy.position[0];
        double current_y = state_copy.position[1];
        double current_yaw = state_copy.imu_state.rpy[2];
        current << current_x, current_y, current_yaw;
        Eigen::MatrixXd traj_aug(N+1, 3);
        traj_aug << current, plan_path_copy;  
        Eigen::MatrixXd vel = (traj_aug.bottomRows(N) - traj_aug.topRows(N)) / traj_interval_s_;
        RCLCPP_INFO(this->get_logger(), "[PathTracking] Compute vel - global: vx=%.3f, vy=%.3f, vyaw=%.3f", 
                   vel(0, 0), vel(0, 1), vel(0, 2));

        // ----------------------------------------------------------
        // convert to api format
        std::vector<PathPoint> path;
        for (int i = 0; i < 30; i++) {
            PathPoint path_point;
            path_point.timeFromStart = (i + 1) * traj_interval_s_;

            if (i < N) {
                path_point.x = plan_path_copy(i, 0);
                path_point.y = plan_path_copy(i, 1);
                path_point.yaw = plan_path_copy(i, 2);
                path_point.vx = vel(i, 0);
                path_point.vy = vel(i, 1);
                path_point.vyaw = vel(i, 2);
                
                // Apply velocity limits
                ClampVelocity(path_point.vx, path_point.vy, path_point.vyaw);
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
        // ----------------------------------------------------------
        // Print first trajectory point for debugging
        if (!path.empty()) {
            RCLCPP_INFO(this->get_logger(), 
                "[PathTracking FollowTrajApi] First trajectory point - x: %.3f, y: %.3f, yaw: %.3f, vx: %.3f, vy: %.3f, vyaw: %.3f",
                path[0].x, path[0].y, path[0].yaw, path[0].vx, path[0].vy, path[0].vyaw);
        }
        
        try {
            move_client_.TrajectoryFollow(req_, path);
            RCLCPP_INFO(this->get_logger(), "[PathTracking] TrajectoryFollow api sent successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[PathTracking] TrajectoryFollow api failed: %s", e.what());
        }
    }

    // velocity following api need local coord
    void MoveApiControlLoop() {
        rclcpp::Rate rate(control_hz_);
        
        while (rclcpp::ok()) {
            Eigen::MatrixXd plan_path_copy;
            unitree_go::msg::SportModeState state_copy;
            {
                std::scoped_lock lock(path_mutex_, state_mutex_);
                plan_path_copy = plan_path_local_;
                state_copy = state_;
            }
            int n_pts = plan_path_copy.rows();

            if (n_pts > 0) {
                auto current_time = this->get_clock()->now();
                double elapsed_time = (current_time - path_start_time_).seconds();
                double x_r = state_copy.position[0];
                double y_r = state_copy.position[1];
                double yaw_r = state_copy.imu_state.rpy[2];

                // interpolate to get target in local coord
                int idx0 = std::min(static_cast<int>(elapsed_time / traj_interval_s_), n_pts - 1);
                int idx1 = std::min(idx0 + 1, n_pts - 1);
                double t0 = idx0 * traj_interval_s_;
                double t1 = idx1 * traj_interval_s_;
                double alpha = (t1 - t0 > 1e-6) ? (elapsed_time - t0) / (t1 - t0) : 0.0;

                double x0 = plan_path_copy(idx0, 0);
                double y0 = plan_path_copy(idx0, 1);
                double yaw0 = plan_path_copy(idx0, 2);
                double x1 = plan_path_copy(idx1, 0);
                double y1 = plan_path_copy(idx1, 1);
                double yaw1 = plan_path_copy(idx1, 2);
                double x_target = (1 - alpha) * x0 + alpha * x1;
                double y_target = (1 - alpha) * y0 + alpha * y1;
                double yaw_target = yaw0 + alpha * angle_utils::shortest_angular_distance(yaw0, yaw1);

                double vx, vy, vyaw;
                if (strategy_ == "open_loop") {
                    vx = (x_target - x_r) * control_hz_;
                    vy = (y_target - y_r) * control_hz_;
                    vyaw = angle_utils::shortest_angular_distance(yaw_r, yaw_target) * control_hz_;
                } else if (strategy_ == "pd") {
                    vx = kp_linear_ * (x_target - x_r);
                    vy = kp_linear_ * (y_target - y_r);
                    vyaw = kp_angular_ * angle_utils::shortest_angular_distance(yaw_r, yaw_target);
                } else {
                    vx = vy = vyaw = 0.0;
                }

                ClampVelocity(vx, vy, vyaw);

                try {
                    move_client_.Move(req_, vx, vy, vyaw);
                    RCLCPP_INFO(this->get_logger(), "[PathTracking] Move api sent successfully");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "[PathTracking] Move api failed: %s", e.what());
                }
                
                if (target_point_idx_ != idx0) {
                    target_point_idx_ = idx0;
                    RCLCPP_DEBUG(this->get_logger(),
                                "[PathTracking] t=%.2f idx=%d: x=%.3f y=%.3f yaw=%.3f, cmd vx=%.3f vy=%.3f vyaw=%.3f",
                                elapsed_time, idx0, x_target, y_target, yaw_target, vx, vy, vyaw);
                }
            }

            rate.sleep();
        }
    }

private:
    std::string api_mode_;
    std::string strategy_;
    int control_hz_;
    bool is_ai_ctl_;
    
    double kp_linear_ = 2.0;
    double kp_angular_ = 1.5;

    double px0_{}, py0_{}, yaw0_{};
    double traj_interval_s_ = 0.1;

    // Robot state
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber_;
    unitree_go::msg::SportModeState state_;
    std::mutex state_mutex_;

    // API client
    Go2MoveClient move_client_;
    unitree_api::msg::Request req_;
    rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr response_suber_;

    // plan path
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_path_suber_;
    std::mutex path_mutex_;
    Eigen::MatrixXd plan_path_local_;       // [n, 3]  x, y, yaw in local
    Eigen::MatrixXd plan_path_global_;      // [n, 3]  x, y, yaw in global

    // VEL TRACK
    int target_point_idx_ = -1;
    rclcpp::Time path_start_time_;

    // Threads
    std::thread control_thread_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Go2MoveClientNode>();
    
    // Wait for the first state message to safely get the initial pose
    RCLCPP_INFO(node->get_logger(), "Waiting for the first state message on /lf/sportmodestate...");
    
    unitree_go::msg::SportModeState::SharedPtr init_state_msg = nullptr;
    bool received_initial_state = false;
    
    // Create temporary subscription to wait for first message
    auto temp_sub = node->create_subscription<unitree_go::msg::SportModeState>(
        "/lf/sportmodestate", 1,
        [&](const unitree_go::msg::SportModeState::SharedPtr msg) {
            init_state_msg = msg;
            received_initial_state = true;
        });
    
    // Wait for initial state with timeout
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(5);
    
    while (!received_initial_state && rclcpp::ok()) {
        rclcpp::spin_some(node);
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > timeout) {
            RCLCPP_ERROR(node->get_logger(), "Timed out waiting for initial state message.");
            rclcpp::shutdown();
            return -1;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Remove temporary subscription
    temp_sub.reset();
    
    if (init_state_msg) {
        node->SetInitState(*init_state_msg);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to get initial state message.");
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
