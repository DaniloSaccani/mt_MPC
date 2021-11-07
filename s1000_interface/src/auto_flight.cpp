#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>

int flight_phase=0;

bool gps_good = false, play_press = false, last_play = false;
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
	ros::init(argc, argv, "auto_flight_manager");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	ros::Subscriber rc_sub = n.subscribe("/dji_sdk/rc", 1, rcCallback);
	ros::Subscriber gps_h_sub = n.subscribe("/dji_sdk/gps_health", 1, gpsCallback);
	ros::Subscriber fs_sub = n.subscribe("/dji_sdk/flight_status", 1, fsCallback);

	ros::Publisher abs_ref_pub = n.advertise<geometry_msgs::PoseStamped>("/gbeam/gbeam_pos_ref", 1);

	//ros::ServiceClient arm_client = n.serviceClient<dji_sdk::DroneArmControl>("drone_arm_control");
	ros::ServiceClient pos_set_client;

	//dji_sdk::DroneArmControl arm_srv;
	dji_sdk::DroneTaskControl task_srv;
	dji_sdk::SetLocalPosRef pos_set_srv;
	dji_sdk::SDKControlAuthority auth_srv;

	geometry_msgs::PoseStamped ref_pos_out;

	ref_pos_out.pose.orientation.x = 1;
	ref_pos_out.pose.orientation.y = 0;
	ref_pos_out.pose.orientation.z = 0;
	ref_pos_out.pose.orientation.w = 0;

	int wpcount = 0;
	// float wp_x[] = {2, 3, -5, -3, 2, 4, -1, 2, -4, 0, 2, -3, 1, 4, -3, 5, -2, 1, 4, -2, 3, -5, 2, -4, 1, -2, 4, 0};
	// float wp_y[] = {2, -3, 1, 4, -3, 5, -2, 1, 4, -2, 3, -5, 2, -4, 1, -2, 4, -2, 3, -4, 2, -1, 4, -3, 1, -5, 4, 2, -3, 1, 4, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	// float wp_z[] = {10, 8, 9, 7, 9, 11, 12, 7, 9, 11, 8, 7, 9, 10, 11, 9, 8, 12, 7, 8, 12, 10, 11, 8, 9, 12, 6, 7, 9, 10, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
	float wp_x[] = {2, 3, -5, -3, 2, 4, -1, 2, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	float wp_y[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, -5, -3, 2, 4, -1, 2, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	float wp_z[] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 7, 9, 10, 8, 7, 11, 9, 6, 10, 7};
	int wp_n = sizeof(wp_x)/sizeof(wp_x[0]);

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
							ROS_INFO("Control acquired, starting mission");
							flight_phase = 10;
							ref_pos_out.pose.position.x = 0;
							ref_pos_out.pose.position.y = 0;
							ref_pos_out.pose.position.z = 10;

							abs_ref_pub.publish(ref_pos_out);
						}
				}
				break;
			case 10://drone airborne, ready to start mission
				if(play_press)
				{
					//go to next waypoint
					ref_pos_out.pose.position.x = wp_x[wpcount];
					ref_pos_out.pose.position.y = wp_y[wpcount];
					ref_pos_out.pose.position.z = wp_z[wpcount];
					abs_ref_pub.publish(ref_pos_out);

					//increase counter and check if it exceeded vector size
					if(++wpcount >= wp_n)
						flight_phase = 11;
				}
				break;
			case 11:
				if(play_press)
				{
					//go to next waypoint
					ref_pos_out.pose.position.x = 0;
					ref_pos_out.pose.position.y = 0;
					ref_pos_out.pose.position.z = 10;

					abs_ref_pub.publish(ref_pos_out);
					flight_phase = 12;
				}
				break;
			case 12:
				flight_phase = 80;
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

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
