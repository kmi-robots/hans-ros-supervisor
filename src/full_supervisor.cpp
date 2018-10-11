#include "supervisor/full_supervisor.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "tf/tf.h"

#include "curlpp/cURLpp.hpp"
#include "curlpp/Easy.hpp"
#include "curlpp/Options.hpp"
#include "curlpp/Exception.hpp"
#include "curlpp/Infos.hpp"

#include <regex>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

FullSupervisor::FullSupervisor(int number, std::string url)
            : _number(number), _url(url), _route_initialized(false), _route_frame_id("/map"), 
            _explore("/Analyzer/explore", true), _move_base("/move_base", true) { }

BT::NodeStatus FullSupervisor::sleepOneSecond() {
    ROS_INFO_STREAM("Sleeping");
    ros::Duration(1).sleep();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FullSupervisor::popWaypoint() {
    if (_route.empty() || !_route_initialized) {
        _route_initialized = false;
        return BT::NodeStatus::FAILURE;
    }
    _next_waypoint = std::move(_route.front());
    _route.pop_front();

    ROS_INFO_STREAM("next waypoint is: (" << _next_waypoint.position.x << "," << _next_waypoint.position.y << ")");
    ROS_INFO_STREAM("there are " << _route.size() << " waypoints left");

    return BT::NodeStatus::SUCCESS;
}

void FullSupervisor::collectWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (_route.size() < _number) {
        _route.push_back(msg->pose);
    }
    ROS_INFO_STREAM("Waypoint received, current size: "<<_route.size());
}

std::future<std::string> FullSupervisor::invoke() {
  return std::async(std::launch::async, [this]() {
      curlpp::Cleanup clean;
      curlpp::Easy request;
      ROS_INFO_STREAM("request: "<<_rule);
      std::string query = curlpp::escape(_rule);
      request.setOpt(new curlpp::options::Url(_url + "/query/list/rule-waypoints?rule=" + query));
      std::ostringstream response;
      request.setOpt(new curlpp::options::WriteStream(&response));
      request.perform();
      std::cout<<curlpp::infos::ResponseCode::get(request)<<std::endl;
      return std::string(response.str());
    });
}

void FullSupervisor::prepareRoute(json djin) {
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    
    ROS_INFO_STREAM(djin.dump());
    for (json::iterator it = djin["results"].begin(); it != djin["results"].end(); ++it) {
        json j = *it;
        std::string s = j["coord"];
        point_type pt;
        if (s.find("POINT") == 0) {
            boost::geometry::read_wkt(s, pt);
        } else {
            boost::geometry::model::polygon<point_type> poly;
            boost::geometry::read_wkt(s, poly);
            boost::geometry::centroid(poly, pt);
        }        
        geometry_msgs::Pose p;
        p.position.x = pt.x();
        p.position.y = pt.y();
        p.position.z = 0.0;
        p.orientation.w = 1.0;
        _route.push_back(p);
    }    
};

BT::NodeStatus FullSupervisor::collectWaypoints() {
    ROS_INFO_STREAM("number is: "<<_number);
    if(_number < 0) {
        if(_rule=="http://data.open.ac.uk/kmi/hans#patrol") {
            std::vector<std::pair<double, double>> v = {
                {22.0, 0.3}, {-5.0, 1.63}, {7.6, 11.0}, {8.4, 16.5}, {30.8, 22.4}, {7.0, 22.0}, {-17.0, 12.0}, {-16.5, -3.75}
            };
            for(std::pair<double, double> i : v) {
                geometry_msgs::Pose p;
                p.position.x = i.first;
                p.position.y = i.second;
                p.position.z = 0.0;
                p.orientation.w = 1.0;
                _route.push_back(p);
            }
        } else {
            std::future<std::string> response = invoke();
            response.wait();
            prepareRoute(json::parse(response.get()));
        }
    } else {
        ros::NodeHandle nh;
        _waypoints_sub  = nh.subscribe("/waypoints", 1, &FullSupervisor::collectWaypointsCallback, this);
        ROS_INFO_STREAM("Waiting for waypoints...");
        ros::Rate r(1);
        while(_route.size() < _number) {
            ros::spinOnce();
            r.sleep();
        }
        _waypoints_sub.shutdown();
    }
    if(_route.size() > 0) {
        _route_initialized = true;
        return BT::NodeStatus::SUCCESS;
    } else {
        _route_initialized = false;
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus FullSupervisor::Explore() {
    if (!_explore.isServerConnected()) {
        ROS_ERROR("ExploreAction failed because server is not connected");
        return BT::NodeStatus::FAILURE;
    }

    ROS_INFO_STREAM("Starting exploration");
    _goal.classes = "Heater,FireActionNotice,FireExtinguisherLabel";
    _explore.sendGoal(_goal);
    _explore.waitForResult();

    if (_explore.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus FullSupervisor::MoveBase() {
    if (!_move_base.isServerConnected()) {
        ros::Duration(1.5).sleep();
        if (!_move_base.isServerConnected()) {
            ROS_ERROR("MoveBaseAction failed because server is not connected");
            return BT::NodeStatus::FAILURE;
        }
    }

    static int ID = 0;
    _mb_goal.target_pose.header.stamp = ros::Time::now();
    _mb_goal.target_pose.header.seq = ID++;
    _mb_goal.target_pose.header.frame_id = _route_frame_id;
    _mb_goal.target_pose.pose = _next_waypoint;

    ROS_INFO_STREAM("x: "<<_mb_goal.target_pose.pose.position.x<<
                    ", y: "<<_mb_goal.target_pose.pose.position.y);
    _move_base.sendGoal(_mb_goal);
    
    ROS_INFO_STREAM("Waiting for goal");
    _move_base.waitForResult();
    ROS_INFO_STREAM("Done");

    if (_move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

