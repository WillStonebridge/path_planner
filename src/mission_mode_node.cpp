/*

Main Mission node for PARTIEEE 2021-22

Behavior:
1. Connects to PX4
2. Clears previous mission
3. Uploads mission
4. (SITL USE ONLY) Arms, Sets Mission Mode
   !!Check to make sure relevant lines are
     commented out before attempting flight!!

*/
#include <ros/ros.h> 
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointList.h>
#include <list>
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>

mavros_msgs::State current_state;
sensor_msgs::NavSatFix current_position; // Use for UGV drop
geometry_msgs::TwistStamped current_vel;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
	bool connected = current_state.connected;
	bool armed = current_state.armed;
    if(!armed)
    {
       ROS_INFO("Disarmed");
    }
}

void position_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_position = *msg;
//    ROS_INFO("Current Position: (%.4f, %.4f, %.4f)", current_position.latitude, current_position.longitude, current_position.altitude);
}

void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) // Currently only using raw gps vel; GPS/FCU fused vel is not being published in SITL testing
{                                                                  // TODO Figure out how to get fused vel data 
    current_vel = *msg;
    float xvel = current_vel.twist.linear.x;
    float yvel = current_vel.twist.linear.y;
    float zvel = current_vel.twist.linear.z;
//    ROS_INFO("Current Velocity: (%.4f, %.4f, %.4f)", xvel, yvel, zvel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_mode");
    ros::NodeHandle nh;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, position_cb);
    ros::Subscriber waypoints_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/global_position/raw/gps_vel", 10, velocity_cb);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("mavros/mission/push");
    ros::ServiceClient wp_clear = nh.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");


    mavros_msgs::SetMode auto_set_mode;
    auto_set_mode.request.custom_mode = "AUTO.MISSION";

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
    FILE* fp = fopen("/root/catkin_ws/src/path_planner/routepath.json", "r"); //  Reading waypoints from routepath json file 
    char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));  
	rapidjson::Document d;
	d.ParseStream(is); 
	fclose(fp);	
	
	const rapidjson::Value& route = d["waypoints"];	 
	for(rapidjson::SizeType i = 0; i < route.Size() - 2; i++) // Create Waypoint objects 
	{
        if (i == 0) 
        {
            wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF; // Specify Takeoff Command 
            wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        }
        else 
        {
            wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT; // Specify Waypoint Command
            wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        }
        wp.is_current     = false;
        wp.autocontinue   = true;
        wp.x_lat          = route[i]["latitude"].GetFloat();
        wp.y_long         = route[i]["longitude"].GetFloat();
        wp.z_alt          = route[i]["altitude"].GetFloat();
        wp_push_srv.request.waypoints.push_back(wp);
        ROS_INFO("\n Frame: %d\n Commmand: %d \n Is_current: %d \n autocontinue: %d \n lat: %.6f \n long: %.6f \n alt: %.6f\n", wp.frame, wp.command, wp.is_current, wp.autocontinue, wp.x_lat, wp.y_long, wp.z_alt);
	}

// Specifying Start of Landing Sequence 
    wp.frame          = 2;
    wp.command        = 189;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 0;          // not using x_lat, y_long arguements
    wp.y_long         = 0;
    wp.z_alt          = 0;
    wp_push_srv.request.waypoints.push_back(wp);

// First WP of Landing Sequence (lining up w/ runway/decent route)
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_LOITER_TO_ALT;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = route[route.Size() - 2]["latitude"].GetFloat();
    wp.y_long         = route[route.Size() - 2]["longitude"].GetFloat();
    wp.z_alt          = route[route.Size() - 2]["altitude"].GetFloat();
    wp_push_srv.request.waypoints.push_back(wp);

// Second WP of Lanidng Sequence (actually landing)
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_LAND;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = route[route.Size() - 1]["latitude"].GetFloat();
    wp.y_long         = route[route.Size() - 1]["longitude"].GetFloat();
    wp.z_alt          = route[route.Size() - 1]["altitude"].GetFloat();

    wp_push_srv.request.waypoints.push_back(wp);
    ROS_INFO("Number of Waypoints: %ld", wp_push_srv.request.waypoints.size());

    ros::Rate rate(20);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to PX4!");

    // Clears WPs
    mavros_msgs::WaypointClear wp_clear_cmd;
    if (wp_clear.call(wp_clear_cmd)) {
        ROS_INFO("Clear waypoints ok: %d", wp_clear_cmd.response.success);
    }
    else
        ROS_ERROR("Clear waypoints FAILED.");

    // Send WPs to Vehicle
    if (wp_client.call(wp_push_srv)) {
        ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
    }
    else
        ROS_ERROR("Send waypoints FAILED.");
   
    ros::Time last_request = ros::Time::now();
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    while(ros::ok()){
    // SITL Testing ONLY: Comment Out for Flight
        //ARM and Sets Mode
     //   if( !current_state.armed &&
     //       (ros::Time::now() - last_request > ros::Duration(5.0))){
     //       if( arming_client.call(arm_cmd) &&
     //           arm_cmd.response.success){
     //           ROS_INFO("Vehicle armed");
     //       }
     //       last_request = ros::Time::now();
     //   }
     //   else if ((current_state.mode != "AUTO.MISSION" && current_state.mode != "AUTO.RTL") &&
     //           (ros::Time::now() - last_request > ros::Duration(5.0))) {
     //       if( set_mode_client.call(auto_set_mode) &&
     //           auto_set_mode.response.mode_sent){
     //           ROS_INFO("AUTO.MISSION enabled");
     //       }
     //       last_request = ros::Time::now();
     //   }
    // Comment to here

    //TODO Include code for UGV drop here 
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

