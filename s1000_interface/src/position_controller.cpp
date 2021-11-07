#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

geometry_msgs::Pose abs_ref;
geometry_msgs::Quaternion attitude;
geometry_msgs::Point pos;

ros::Publisher ref_pub;

float K_pos_xy, K_pos_z, K_yaw, vel_max, yaw_rate_max;
float spd_filter = 0.5;
float filtered_pos_x_prev = 0, filtered_pos_y_prev = 0;

void refCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	abs_ref = msg->pose;
}

void attCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
	attitude = msg->quaternion;
}

void posCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	pos = msg->point;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_controller");
	ros::NodeHandle n;

	ros::Rate loop_rate(20);

	ros::Subscriber ref_sub = n.subscribe("/s1000_interface/pos_ref", 1, refCallback);
	ros::Subscriber pos_sub = n.subscribe("/dji_sdk/local_position", 1, posCallback);
	ros::Subscriber att_sub = n.subscribe("/dji_sdk/attitude", 1, attCallback);

	ref_pub = n.advertise<sensor_msgs::Joy>("/s1000_interface/speed_ref", 1);

	while(n.ok())
	{
		ros::param::get("/s1000_interface/position_controller/K_pos_xy", K_pos_xy);
		ros::param::get("/s1000_interface/position_controller/K_pos_z", K_pos_z);
		ros::param::get("/s1000_interface/position_controller/K_yaw", K_yaw);
		ros::param::get("/s1000_interface/position_controller/vel_max", vel_max);
		ros::param::get("/s1000_interface/position_controller/yaw_rate_max", yaw_rate_max);

		ros::param::get("/s1000_interface/position_controller/spd_filter", spd_filter);

		filtered_pos_x_prev = (1-spd_filter) * filtered_pos_x_prev + spd_filter * abs_ref.position.x;
		filtered_pos_y_prev = (1-spd_filter) * filtered_pos_y_prev + spd_filter * abs_ref.position.y;


		// compute speed references
		float x_vel = K_pos_xy * (filtered_pos_x_prev - pos.x);
		float y_vel = K_pos_xy * (filtered_pos_y_prev - pos.y);
		float z_vel = K_pos_xy * (abs_ref.position.z - pos.z);

		// compute yaw reference and position from quaternions
		double roll_ref, pitch_ref, yaw_ref;
		tf2::Quaternion quat_ref(
				abs_ref.orientation.x,
				abs_ref.orientation.y,
				abs_ref.orientation.z,
				abs_ref.orientation.w);
		tf2::Matrix3x3 mat_ref(quat_ref);
		mat_ref.getRPY(roll_ref, pitch_ref, yaw_ref);
		double roll, pitch, yaw;
		tf2::Quaternion quat(
				attitude.x,
				attitude.y,
				attitude.z,
				attitude.w);
		tf2::Matrix3x3 mat(quat);
		mat.getRPY(roll, pitch, yaw);

		// compute yaw rate
		double yaw_rate = K_yaw * (yaw_ref - yaw);
		if(yaw_rate != yaw_rate)
			yaw_rate = 0;

		// bound output values
		if (x_vel > vel_max)
			x_vel = vel_max;
		else
			if(x_vel < -vel_max)
				x_vel = -vel_max;

		if (y_vel > vel_max)
			y_vel = vel_max;
		else
			if(y_vel < -vel_max)
				y_vel = -vel_max;

		if (z_vel > vel_max)
			z_vel = vel_max;
		else
			if(z_vel < -vel_max)
				z_vel = -vel_max;

		if (yaw_rate > yaw_rate_max)
			yaw_rate = yaw_rate_max;
		else
			if(yaw_rate < -yaw_rate_max)
				yaw_rate = -yaw_rate_max;


		// cbuild and output command
		float flag = 73; //as dji_sdk ros WIKI
		//145: hor pos, alt, yaw angle, ground_ENU frame, active brake
		//72: hor vel, ver vel, yaw rate, ground_ENU, no brake
		//73: hor vel, ver vel, yaw rate, ground_ENU, active brake

		sensor_msgs::Joy out_ref;

		out_ref.axes.push_back(x_vel);
		out_ref.axes.push_back(y_vel);
		out_ref.axes.push_back(z_vel);
		out_ref.axes.push_back(yaw_rate);
		out_ref.axes.push_back(flag);

		ref_pub.publish(out_ref);

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}
