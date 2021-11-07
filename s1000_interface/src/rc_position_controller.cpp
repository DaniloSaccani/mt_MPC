#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher ref_pub;

float vel_max, yaw_rate_max;

void rcCallback(const sensor_msgs::Joy::ConstPtr& rc_in)
{
	ros::param::get("/s1000_interface/position_controller/vel_max", vel_max);
	ros::param::get("/s1000_interface/position_controller/yaw_rate_max", yaw_rate_max);
	float thr_dz = 0.2;

	// compute speed references
	float x_vel = vel_max * rc_in->axes[0];
	float y_vel = vel_max * rc_in->axes[1];

	float z_vel = rc_in->axes[3] > -thr_dz && rc_in->axes[3] < thr_dz ? 0 : rc_in->axes[3] * vel_max;

	float yaw_rate = yaw_rate_max * rc_in->axes[2];

	// build and output command
	float flag = 73;
	sensor_msgs::Joy out_ref;

	out_ref.axes.push_back(x_vel);
	out_ref.axes.push_back(y_vel);
	out_ref.axes.push_back(z_vel);
	out_ref.axes.push_back(yaw_rate);
	out_ref.axes.push_back(flag);

	ref_pub.publish(out_ref);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_controller");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	ros::Subscriber rc_sub = n.subscribe("/dji_sdk/rc", 1, rcCallback);

	ref_pub = n.advertise<sensor_msgs::Joy>("/s1000_interface/speed_ref", 1);

	ros::spin();

	return 0;
}
