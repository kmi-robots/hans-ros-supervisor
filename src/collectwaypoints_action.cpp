#include <regex>

#include "supervisor/collectwaypoints_action.h"

#include "curlpp/cURLpp.hpp"
#include "curlpp/Easy.hpp"
#include "curlpp/Options.hpp"
#include "curlpp/Exception.hpp"
#include "curlpp/Infos.hpp"

CollectWaypointsAction::CollectWaypointsAction(const std::string &name, const BT::NodeParameters &params)
    : ActionNodeBase(name) {
    assert(params.size() == 1);
    // there must be a single parameter
    _number = std::stoi(params.at("number"));
}

void CollectWaypointsAction::collectWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (_route.size() < _number) {
        _route.push_back(msg->pose);
    }
    ROS_INFO_STREAM("Waypoint received, current size: "<<_route.size());
}

std::future<std::string> CollectWaypointsAction::invoke() {
  return std::async(std::launch::async, [this]() {
      curlpp::Cleanup clean;
      curlpp::Easy request;
      //GET /query/list/rule-waypoints?rule=hsf:heaterFreeAreaRule
//       request.setOpt(new curlpp::options::Url("http://137.108.125.184:5000/semanticmap-service"));
      request.setOpt(new curlpp::options::Url(
          "http://137.108.121.40:7070/query/list/rule-waypoints?rule=hsf:heaterFreeAreaRule"));
//       json j;
//       j["semantic_map_queries"] = {{{"name", this->ID}, {"query","getHeaterFreeAreas"}}};
//       std::stringstream ss;
//       ss << "json=" << j.dump();
//       request.setOpt(new curlpp::options::PostFields(ss.str()));
      std::ostringstream response;
      request.setOpt(new curlpp::options::WriteStream(&response));
      request.perform();
      std::cout<<curlpp::infos::ResponseCode::get(request)<<std::endl;
      return std::string(response.str());
    });
}

void CollectWaypointsAction::prepareRoute(json djin) {
    ROS_INFO_STREAM(djin.dump());
    for (json::iterator it = djin["results"].begin(); it != djin["results"].end(); ++it) {
        json j = *it;
        std::string s = j["coord"];
        std::regex rgx("\\((-?\\d+\\.?\\d*),(-?\\d+\\.?\\d*)\\),*", std::regex_constants::ECMAScript);
        auto words = std::sregex_iterator(s.begin(), s.end(), rgx);
        for (auto i = words; i != std::sregex_iterator(); ++i) {
            std::smatch match = *i;
            geometry_msgs::Pose p;
            p.position.x = std::stod(match[1]);
            p.position.y = std::stod(match[2]);;
            p.position.z = 0.0;
            p.orientation.w = 1.0;
            _route.push_back(p);
        }
    }    
};

BT::NodeStatus CollectWaypointsAction::tick() {
    ROS_INFO_STREAM(",,number is: "<<_number);
    if(_number < 0) {
        std::future<std::string> response = invoke();
        response.wait();
        prepareRoute(json::parse(response.get()));
        blackboard()->set("waypoints_number", _route.size());
    } else {
        blackboard()->set("waypoints_number", _number);
        ros::NodeHandle nh;
        _waypoints_sub  = nh.subscribe("/waypoints", 1, &CollectWaypointsAction::collectWaypointsCallback, this);
        ROS_INFO_STREAM("Waiting for waypoints...");
        ros::Rate r(1);
        while(_route.size() < _number) {
            ros::spinOnce();
            r.sleep();
        }
        _waypoints_sub.shutdown();
    }
    if(_route.size() > 0) {
        blackboard()->set("route", _route);
        _route.clear();
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
        
}

void CollectWaypointsAction::halt() {
    if(_number > 0) 
        _waypoints_sub.shutdown();
    _route.clear();
    setStatus(NodeStatus::IDLE);
}
