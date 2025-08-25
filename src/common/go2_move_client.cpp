#include "go2_move_client.h"

void SportClient::BalanceStand(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
  req_puber_->publish(req);
}

void Go2MoveClient::StopMove(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE;
  req_puber_->publish(req);
}

void Go2MoveClient::Move(unitree_api::msg::Request &req, float vx, float vy, float vyaw) {
  nlohmann::json js;
  js["x"] = vx;
  js["y"] = vy;
  js["z"] = vyaw;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
  req_puber_->publish(req);
}

void Go2MoveClient::SpeedLevel(unitree_api::msg::Request &req, int level) {
  nlohmann::json js;
  js["data"] = level;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL;
  req_puber_->publish(req);
}


void Go2MoveClient::TrajectoryFollow(unitree_api::msg::Request &req, std::vector<PathPoint> &path)
{
    nlohmann::json js_path;
    // must be 30 here. error when using path.size()
    for (size_t i = 0; i < 30; i++) 
    {
        nlohmann::json js_point;
        js_point["t_from_start"] = path[i].timeFromStart;
        js_point["x"] = path[i].x;
        js_point["y"] = path[i].y;
        js_point["yaw"] = path[i].yaw;
        js_point["vx"] = path[i].vx;
        js_point["vy"] = path[i].vy;
        js_point["vyaw"] = path[i].vyaw;
        js_path.push_back(js_point);
    }
    req.parameter =js_path.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW;
    req_puber_->publish(req);
}

void Go2MoveClient::SwitchJoystick(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHJOYSTICK;
  req_puber_->publish(req);
}


void Go2MoveClient::StaticWalk(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STATICWALK;
  req_puber_->publish(req);
}

void Go2MoveClient::TrotRun(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_TROTRUN;
  req_puber_->publish(req);
}

void Go2MoveClient::FreeAvoid(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FREEAVOID;
  req_puber_->publish(req);
}

void Go2MoveClient::SwitchAvoidMode(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHAVOIDMODE;
  req_puber_->publish(req);
}
