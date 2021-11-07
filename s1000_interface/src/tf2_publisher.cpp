#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>

geometry_msgs::PointStamped local_pos;

void posCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	local_pos = *msg;
}

void attCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped trSt;

	trSt.header.stamp = ros::Time::now();
	trSt.header.frame_id = "odom";
	trSt.child_frame_id = "body_FLU";
	trSt.transform.translation.x = local_pos.point.x;
	trSt.transform.translation.y = local_pos.point.y;
	trSt.transform.translation.z = local_pos.point.z;
	trSt.transform.rotation = msg->quaternion;

	br.sendTransform(trSt);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dji_tf2_publisher");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::Subscriber posSub = n.subscribe("/dji_sdk/local_position", 1, posCallback);
	ros::Subscriber attSub = n.subscribe("/dji_sdk/attitude", 1, attCallback);

	ros::spin();
	return 0;
}
