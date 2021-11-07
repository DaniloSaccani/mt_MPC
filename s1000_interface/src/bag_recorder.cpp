#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

bool rec_status = false;

void rcCallback(const sensor_msgs::Joy::ConstPtr& rc_ptr)
{
	float start_but = rc_ptr->axes[7];
	float stop_but = rc_ptr->axes[6];

	if (start_but == 1 && stop_but < 1 && rec_status == false)
	{
		rec_status = true;
		system("mkdir -p /root/rosbags/");
		system("rosbag record -o /root/rosbags/s1000 -a __name:=bag_recorder_s1000 &");
		/*system("rosbag record -o /root/rosbags/s1000"
			" /dji_sdk/flight_control_setpoint_generic /dji_sdk/local_position /tf /scan"
			" /gbeam/free_polytope /gbeam/reachability_graph /gbeam/gbeam_pos_ref"
			" /s1000_interface/pos_ref /s1000_interface/speed_ref"
			" __name:=bag_recorder_s1000 &");*/
		ROS_INFO("Started bag");
	}
	if (start_but < 1 && stop_but == 1 && rec_status == true)
	{
		rec_status = false;
		system("rosnode kill /bag_recorder_s1000");
		ROS_INFO("Stopped bag");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bag_rec");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	ros::Subscriber rc_sub = n.subscribe("/dji_sdk/rc", 1, rcCallback);

	ros::spin();

	return 0;
}
