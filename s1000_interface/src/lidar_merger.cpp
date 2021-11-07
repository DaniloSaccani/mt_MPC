#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan scan_front, scan_back;

ros::Publisher scanPub;

void frontCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scan_front = *msg;
}

void backCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scan_back = *msg;
	sensor_msgs::LaserScan scan;
	scan = scan_front;
	scan.ranges.insert(scan.ranges.end(), scan_back.ranges.begin(), scan_back.ranges.end());
	scan.angle_min = -3.14159265;
	scan.angle_max = 3.14159265;
	scan.range_min = 0.5;

	scan.header.frame_id = "base_scan";
	scan.angle_increment = 6.28318531 / (scan.ranges.size()-1);

	scanPub.publish(scan);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "2lidar_merger");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	ros::Subscriber frontSub = n.subscribe("scan_front", 1, frontCallback);
	ros::Subscriber backSub = n.subscribe("scan_back", 1, backCallback);

	scanPub = n.advertise<sensor_msgs::LaserScan>("scan", 1);

	ros::spin();
	return 0;
}
