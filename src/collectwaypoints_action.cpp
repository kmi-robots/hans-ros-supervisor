#include "supervisor/collectwaypoints_action.h"

#include "curlpp/cURLpp.hpp"
#include "curlpp/Easy.hpp"
#include "curlpp/Options.hpp"
#include "curlpp/Exception.hpp"

CollectWaypointsAction::CollectWaypointsAction(ros::NodeHandle &nh, const std::string &name, const BT::TextParameters &params)
    : ActionNode(name), _nh(nh), _running(false) {
    assert(params.size() == 1);
    // there must be a single parameter
    _number = std::stoi(params.begin()->second);
}

void CollectWaypointsAction::collectWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (_route.size() < _number) {
        _route.push_back(msg->pose);
    }
}

std::future<std::string> CollectWaypointsAction::invoke() {
  return std::async(std::launch::async, [this]() {
      curlpp::Cleanup clean;
      curlpp::Easy request;
      request.setOpt(new curlpp::options::Url("http://137.108.116.193:5000/semanticmap-service"));
      json j;
      j["semantic_map_queries"] = {{{"name", this->ID}, {"query","getHeaterFreeAreas"}}};
      std::stringstream ss;
      ss << "json=" << j.dump();
      request.setOpt(new curlpp::options::PostFields(ss.str()));
      std::ostringstream response;
      request.setOpt(new curlpp::options::WriteStream(&response));
      request.perform();
      return std::string(response.str());
    });
}

void CollectWaypointsAction::prepareRoute(json djin) {
    ROS_INFO_STREAM(djin.dump());
    for (json::iterator it = djin["results"][ID].begin(); it != djin["results"][ID].end(); ++it) {
       json j = *it;
       _route.push_back(jsonToPose(j["pose"]));
    }    
};

geometry_msgs::Pose CollectWaypointsAction::jsonToPose(json jpose) {
    geometry_msgs::Pose p;
    p.position.x = jpose["position"]["x"];
    p.position.y = jpose["position"]["y"];
    p.position.z = jpose["position"]["z"];
    
    p.orientation.x = jpose["orientation"]["x"];
    p.orientation.y = jpose["orientation"]["y"];
    p.orientation.z = jpose["orientation"]["z"];
    p.orientation.w = jpose["orientation"]["w"];
    
    return p;
};

BT::State CollectWaypointsAction::spin() {    
    if(_number < 0) {
        std::future<std::string> response = invoke();
        response.wait();
        prepareRoute(json::parse(response.get()));
    } else {
        if (!_running) {
            _waypoints_sub  = _nh.subscribe("/waypoints", 1, &CollectWaypointsAction::collectWaypointsCallback, this);
            _running = true;
            ROS_INFO_STREAM("Subscription active");
            return BT::State::RUNNING;
        } else {
            if (_route.size() < _number) {
                ROS_INFO_STREAM("Waiting for waypoints...sleeping...");
                ros::Duration(1.0).sleep();
                return BT::State::RUNNING;
            } else {
                _waypoints_sub.shutdown();
            }
        }
    }
    _blackboard->set("route", _route);
    _running = false;
    _route.clear();
    return BT::State::SUCCESS;
}

void CollectWaypointsAction::halt() {
    if (_running) {
        _waypoints_sub.shutdown();
        _route.clear();
        _blackboard->erase("route");
        _running = false;
    }
}
