#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>

#include <gbeam_library/setMappingStatus.h>

int flight_phase=0;

bool gps_good = false;
bool play_press = false, last_play = false;
int flight_status = 0;	// 0: STOP, 1: GROUND, 2: AIR

void gpsCallback(const std_msgs::UInt8::ConstPtr& gps_ptr)
{
	if(gps_ptr->data >= 3)
		gps_good = true;
	else
		gps_good = false;
}

void rcCallback(const sensor_msgs::Joy::ConstPtr& rc_ptr)
{
	float play_but = rc_ptr->axes[8];
	if((play_but == 1) && (last_play == false))
	{
		play_press = true;
		last_play = true;
	}
	else
		play_press = false;
	if((play_but == 0) && (last_play == true))
		last_play = false;
}

void fsCallback(const std_msgs::UInt8::ConstPtr& fs_ptr)
{
	flight_status = fs_ptr->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "s1000_flight_manager");
	ros::NodeHandle n;

	ros::Rate loop_rate(20);

	ros::Subscriber rc_sub = n.subscribe("/dji_sdk/rc", 1, rcCallback);
	ros::Subscriber gps_h_sub = n.subscribe("/dji_sdk/gps_health", 1, gpsCallback);
	ros::Subscriber fs_sub = n.subscribe("/dji_sdk/flight_status", 1, fsCallback);

	ros::Publisher pos_ref_pub = n.advertise<geometry_msgs::PoseStamped>("s1000_interface/pos_ref", 1);

	//ros::ServiceClient arm_client = n.serviceClient<dji_sdk::DroneArmControl>("drone_arm_control");
	ros::ServiceClient pos_set_client;

	//dji_sdk::DroneArmControl arm_srv;
	dji_sdk::DroneTaskControl task_srv;
	dji_sdk::SetLocalPosRef pos_set_srv;
	dji_sdk::SDKControlAuthority auth_srv;
	gbeam_library::setMappingStatus mapping_status_srv;

	geometry_msgs::PoseStamped pos_ref;

	while(ros::ok())
	{
		if(play_press)
			if(gps_good)
				ROS_INFO("pressed, GPS good, phase: %d", flight_phase);
			else
				ROS_INFO("pressed, GPS bad");

		switch(flight_phase)
		{
			case 0:	//drone disarmed, no positioning (GPS bad)
				if(gps_good)
				{
					//set pos and update flight phase
					if(ros::service::call("/dji_sdk/set_local_pos_ref", pos_set_srv))
					{
						flight_phase = 1;
						ROS_INFO("GPS good, local pos ref set!");
					}
				}
				break;
			case 1: //drone disarmed, positioning, local position reference set
				if(play_press)
				{
					ros::service::call("/dji_sdk/set_local_pos_ref", pos_set_srv);
					//request task 4: takeoff
					task_srv.request.task = 4;
					ROS_INFO("Request takeoff");
					if(ros::service::call("/dji_sdk/drone_task_control", task_srv))
					{
						flight_phase = 2;
						ROS_INFO("Takeoff accepted");
					}
				}
				break;
			case 2: //drone taking off
				if(flight_status == 2)
				{
					ROS_INFO("Drone ariborne, taking control");
					auth_srv.request.control_enable = 1;
					if(ros::service::call("/dji_sdk/sdk_control_authority", auth_srv))
						if(auth_srv.response.result)
						{
							ROS_INFO("Control acquired");
							flight_phase = 10;
						}
				}
				break;
			case 10://drone airborne, listening to waypoints
				if(play_press)
				{
					flight_phase = 80;
				}
				break;
			case 80://mission finished, ready to land
				if(play_press)
				{
					//request task 6: landing
					task_srv.request.task = 6;
					ROS_INFO("Request landing");
					if(ros::service::call("/dji_sdk/drone_task_control", task_srv))
					{
						flight_phase = 90;
						ROS_INFO("Landing accepted");
					}
				}
				break;
			case 90://drone landing
				if(flight_status == 0)
					flight_phase = 91;
				break;
			case 91://drone landed
				if(play_press)
					flight_phase = 0;
				break;
		}

		play_press = false;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
