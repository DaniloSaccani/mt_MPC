#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>


bool wp_press = false, last_wp = false;
int current_wp = 0;

void rcCallback(const sensor_msgs::Joy::ConstPtr& rc_ptr)
{
	float wp_but = rc_ptr->axes[11];
	if((wp_but == 1) && (last_wp == false))
	{
		wp_press = true;
		last_wp = true;
	}
	else
		wp_press = false;
	if((wp_but == 0) && (last_wp == true))
		last_wp = false;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_flight_manager");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	ros::Subscriber rc_sub = n.subscribe("/dji_sdk/rc", 1, rcCallback);

	ros::Publisher ref_pub = n.advertise<geometry_msgs::Point>("/mtMPC/temp_target", 1);

	while(ros::ok())
	{
		geometry_msgs::Point ref_point;

		if(wp_press)
		{
			ROS_INFO("WP button pressed");
			current_wp++;
		}
		ref_point.z = 4;
		switch(current_wp)
		{
			case 0:
				ref_point.x = 0;
				ref_point.y = 0;
				ref_point.z = 4;
				break;
			case 1:
				ref_point.x = 6.4;
				ref_point.y = 0;
				ref_point.z = 4;
				break;
			case 2:
				ref_point.x = 6.4;
				ref_point.y = 10;
				ref_point.z = 4;
				break;
			case 3:
				ref_point.x = 6.4;
				ref_point.y = 25.2;
				ref_point.z = 4;
				break;
			case 3:
				ref_point.x = 0;
				ref_point.y = 25.2;
				ref_point.z = 4;
				break;
		}


		ref_pub.publish(ref_point);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
