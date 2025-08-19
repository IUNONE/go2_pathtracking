#ifndef _GO2_MOVE_CLIENT_
#define _GO2_MOVE_CLIENT_

#include <cstdint>
#include <future>
#include <iostream>
#include <utility>
#include <rclcpp/rclcpp.hpp>

#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "time_tools.hpp"
#include "ut_errror.hpp"


const int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;
const int32_t ROBOT_SPORT_API_ID_MOVE = 1008;
const int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL = 1015;
const int32_t ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW = 1018;
const int32_t ROBOT_SPORT_API_ID_SWITCHJOYSTICK = 1027;
const int32_t ROBOT_SPORT_API_ID_STATICWALK = 1061;
const int32_t ROBOT_SPORT_API_ID_TROTRUN = 1062;
const int32_t ROBOT_SPORT_API_ID_FREEAVOID = 2048;
const int32_t ROBOT_SPORT_API_ID_SWITCHAVOIDMODE = 2058;

#pragma pack(1)
struct PathPoint {
  float timeFromStart;
  float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vyaw;
};
#pragma pack()

class Go2MoveClient {
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr req_suber_;
  rclcpp::Node *node_;

 public:
  explicit Go2MoveClient(rclcpp::Node *node) : node_(node) {
    req_puber_ = node_->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", 10);
  }

  template <typename Request, typename Response>
  int32_t Call(const Request &req) {
    std::promise<typename Response::SharedPtr> response_promise;
    auto response_future = response_promise.get_future();
    auto api_id = req.header.identity.api_id;
    auto req_suber_ = node_->create_subscription<Response>(
        "/api/sport/response", 1,
        [&response_promise, api_id](const typename Response::SharedPtr data) {
          if (data->header.identity.api_id == api_id) {
            response_promise.set_value(data);
          }
        });

    req_puber_->publish(req);

    auto status = response_future.wait_for(std::chrono::seconds(5));

    if (status == std::future_status::ready) {
      auto response = *response_future.get();
      if (response.header.status.code != 0) {
        std::cout << "error code: " << response.header.status.code << std::endl;
        req_suber_.reset();
        return response.header.status.code;
      }
      req_suber_.reset();
      return UT_ROBOT_SUCCESS;
    } else if (status == std::future_status::timeout) {
      return UT_ROBOT_TASK_TIMEOUT;
    } else {
      return UT_ROBOT_TASK_UNKNOWN_ERROR;
    }
  }

  /*
   * @brief StopMove
   * @api: 1003
   */
  void StopMove(unitree_api::msg::Request &req);

  /*
   * @brief Move
   * @api: 1008
   */
  void Move(unitree_api::msg::Request &req, float vx, float vy, float vyaw);

  /*
   * @brief SpeedLevel
   * @api: 1015
   */
  void SpeedLevel(unitree_api::msg::Request &req, int level);

  /*
  * @brief TrajectoryFollow
  * @api: 1018
  */
  void TrajectoryFollow(unitree_api::msg::Request &req, std::vector<PathPoint> &path);

  /*
   * @brief SwitchJoystick
   * @api: 1027
   */
  void SwitchJoystick(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief StaticWalk
   * @api: 1061
   */
  void StaticWalk(unitree_api::msg::Request &req);

  /*
   * @brief TrotRun
   * @api: 1062
   */
  void TrotRun(unitree_api::msg::Request &req);

  /*
   * @brief FreeAvoid
   * @api: 2048
   */
  void FreeAvoid(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief SwitchAvoidMode
   * @api: 2058
   * @param flag: true to enable, false to disable
   */
  void SwitchAvoidMode(unitree_api::msg::Request &req);
};

#endif
