/**
* @file mavros_fly_mission_node.cpp
* @brief Demonstration of Flying Waypoint missions using MAVROS
* @author Shakthi Prashanth M <shakthi.prashanth.m@intel.com>
* @version 0.0.1
* @date 2017-09-01
*/
#include <ros/ros.h> 
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <list>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
	bool connected = current_state.connected;
	bool armed = current_state.armed;
	ROS_INFO("%s", armed ? "" : "DisArmed");
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_set_mode");
    ros::NodeHandle nh;
    
    mavros_msgs::SetMode auto_set_mode;
    auto_set_mode.request.custom_mode = "AUTO.MISSION";
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("mavros/mission/push");
    mavros_msgs::WaypointPush wp_push_srv; // List of Waypoints
    mavros_msgs::Waypoint wp;
    /*
        uint8 FRAME_GLOBAL=0
        uint8 FRAME_LOCAL_NED=1
        uint8 FRAME_MISSION=2
        uint8 FRAME_GLOBAL_REL_ALT=3
        uint8 FRAME_LOCAL_ENU=4
        uint8 frame
        uint16 command
        bool is_current
        bool autocontinue
        float32 param1
        float32 param2
        float32 param3
        float32 param4
        float64 x_lat
        float64 y_long
        float64 z_alt
    */
    // WP 0
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
    wp.is_current     = true;
    wp.autocontinue   = true;
    wp.x_lat          = 47.3978206;
    wp.y_long         = 8.543987;
    wp.z_alt          = 10;
    wp_push_srv.request.waypoints.push_back(wp);
    // WP 1
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_LOITER_TIME;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 47.3962527;
    wp.y_long         = 8.5467917;
    wp.z_alt          = 20;
	wp.param1			= 10;
	wp.param3			= 2;
	wp.param4			= 1;
    wp_push_srv.request.waypoints.push_back(wp);
    
    // WP 2
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 47.3977783;
    wp.y_long         = 8.547906;
    wp.z_alt          = 20;
    wp_push_srv.request.waypoints.push_back(wp);

    // WP 3
    wp.frame          = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.command        = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 0;
    wp.y_long         = 0;
    wp.z_alt          = 0;
    wp_push_srv.request.waypoints.push_back(wp);

    ros::Rate rate(20);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to PX4!");


    // ARM
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    // Send WPs to Vehicle
    if (wp_client.call(wp_push_srv)) {
        ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
    }

    else
        ROS_ERROR("Send waypoints FAILED.");
    
    
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        else if ((current_state.mode != "AUTO.MISSION" && current_state.mode != "AUTO.RTL") &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(auto_set_mode) &&
                auto_set_mode.response.mode_sent){
                ROS_INFO("AUTO.MISSION enabled");
            }
            last_request = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

