#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/PointCloud.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define HALF_PI 1.57079632

ros::Publisher vel_pub;
ros::Publisher obst_pub;

tf2_ros::Buffer tfBuffer;

geometry_msgs::TransformStamped l2g_tf;

float safe_dist = 5, obs_spd_tol = 0.5;
float yaw_drone;
sensor_msgs::LaserScan scan;

// FUNCTIONS

// return dot product between a and b
float dotProd(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b)
{
	return a.x*b.x + a.y*b.y;// + a.z*b.z;
}

// return magnitude of vector a
float magnitude(geometry_msgs::Vector3 a)
{
	return sqrt(pow(a.x, 2) + pow(a.y, 2));// + pow(a.z, 2));
}

// return angle of vector b (in 2D!!)
float angle(geometry_msgs::Vector3 a)
{
	return atan2(a.y, a.x);
}

// return normalized vector
geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 a)
{
	geometry_msgs::Vector3 b;
	float m = magnitude(a);
	b.x = a.x/m;
	b.y = a.y/m;
	b.z = a.z/m;
	return b;
}

// return vector a scaled by factor s
geometry_msgs::Vector3 scale(geometry_msgs::Vector3 a, float s)
{
	geometry_msgs::Vector3 b;
	b.x = a.x*s;
	b.y = a.y*s;
	b.z = a.z*s;
	return b;
}

// return perpendicular vector (rotated by pi/2 ccw on xy plane)
geometry_msgs::Vector3 perpendicular(geometry_msgs::Vector3 a)
{
	geometry_msgs::Vector3 b;
	b.x = a.y;
	b.y = -a.x;
	b.z = a.z;
	return b;
}

// return projection of vector v on vector d
geometry_msgs::Vector3 project(geometry_msgs::Vector3 v, geometry_msgs::Vector3 d)
{
	d = normalize(d);
	return scale(d, dotProd(v, d));
}

// return sum of two vectors
geometry_msgs::Vector3 sum(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b)
{
	geometry_msgs::Vector3 c;
	c.x = a.x+b.x;
	c.y = a.y+b.y;
	c.z = a.z+b.z;
	return c;
}

// return difference of two vectors
geometry_msgs::Vector3 diff(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b)
{
	geometry_msgs::Vector3 c;
	c.x = a.x-b.x;
	c.y = a.y-b.y;
	c.z = a.z-b.z;
	return c;
}



// CALLBACKS AND MAIN

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scan = *msg;
}

void refCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	sensor_msgs::Joy ref_out = *msg;
	geometry_msgs::Vector3 ref_spd;
	ref_spd.x = ref_out.axes[0], ref_spd.y = ref_out.axes[1];

	sensor_msgs::PointCloud pc_out;
	pc_out.header.frame_id = "odom";

	// GET UPDATED PARAMETERS																															// RESTORE BEFORE IMPLEMENTING
	ros::param::get("/gbeam_controller/robot_param/safe_dist", safe_dist);
	ros::param::get("/gbeam_controller/obstacle_avoidance/obstacle_spd_tol", obs_spd_tol);


	// GET TRANSFORM FROM LIDAR TO GLOBAL COORDINATES		--> GET YAW
  try		//get transform from "/odom" to "/base_scan"
  {
	  l2g_tf = tfBuffer.lookupTransform("odom", "base_scan", ros::Time(0));
  }
  catch(tf2::TransformException &ex)
  {
		// if no valid transform is received, stop the drone and give warning
	  ROS_WARN("obstacle_avoidance:lookupTransform: %s", ex.what());
		ref_out.axes[0] = 0; //x vel = 0
		ref_out.axes[1] = 0; //y vel = 0
		vel_pub.publish(ref_out);
	  return;
  }
	// compute yaw position from quaternions
	double roll, pitch, yaw;
	tf2::Quaternion quat_tf;
	tf2::convert(l2g_tf.transform.rotation, quat_tf);	// convert from geometry_msgs::Quaternion l2g_tf to tf2::Quaternion quat_tf
	tf2::Matrix3x3 mat(quat_tf);
	mat.getRPY(roll, pitch, yaw);

	// MODIFY SPEED REFERENCE
	for (int i=0; i<scan.ranges.size(); i++)	//for all lidar readings
	{
		// get LiDAR measurement's polar coordinates
		float obst_angle = -(scan.angle_min + i*scan.angle_increment) + yaw;
		float range = scan.ranges[i];

		if ((range < scan.range_max) && (range > scan.range_min))	// if measurement is in range (valid)
		{

			// compute angles and magnitudes of vectors (ref speed and obstacle proximity vector)
			float delta = obst_angle - angle(ref_spd);	// delta angle between speed ref and obstacle vector
			float dist_margin = range - safe_dist;
			float dist_margin_abs = abs(dist_margin);

			geometry_msgs::Vector3 rep_vect;
			rep_vect.x = cos(obst_angle) / (dist_margin_abs);
			rep_vect.y = sin(obst_angle) / (dist_margin_abs);

			geometry_msgs::Vector3 obst_norm_dir = normalize(rep_vect);
			geometry_msgs::Vector3 obst_tan_dir = perpendicular(obst_norm_dir);

			float ref_spd_abs = magnitude(ref_spd);

			if ( dotProd(ref_spd, rep_vect) >= obs_spd_tol )
			{
				ref_spd = diff(ref_spd, scale(obst_norm_dir, dotProd(ref_spd, obst_norm_dir) - dist_margin_abs * obs_spd_tol));
			}

			// FROM HERE
			geometry_msgs::Point32 p;
			p.x = range * cos(obst_angle);
			p.y = range * sin(obst_angle);
			p.z = dotProd(ref_spd, rep_vect);
			pc_out.points.push_back(p);
			// TO HERE		only for debug

		}	// endif range valid
	}	// endfor


	if(isnan(ref_spd.x) || isnan(ref_spd.y))
	{
		ref_spd.x = 0;
		ref_spd.y = 0;
		ROS_WARN("OBSTACLE AVOIDANCE: the procedure returned NAN, stopping drone");
	}
	// OUTPUT MODIFIED REFERENCE AND END FUNCTION
	ref_out.axes[0] = ref_spd.x, ref_out.axes[1] = ref_spd.y;
	vel_pub.publish(ref_out);

	obst_pub.publish(pc_out);


	// debug
	// ROS_INFO("IN  x: %6.2f  y: %6.2f", msg->axes[0], msg->axes[1]);
	// ROS_INFO("OUT x: %6.2f  y: %6.2f", ref_out.axes[0], ref_out.axes[1]);
	return;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacle_avoidance");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	ros::Subscriber scan_sub = n.subscribe("/scan", 1, scanCallback);
	ros::Subscriber vel_sub = n.subscribe("/s1000_interface/speed_ref", 1, refCallback);

	tf2_ros::TransformListener tfListener(tfBuffer);

	vel_pub = n.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1);
	obst_pub = n.advertise<sensor_msgs::PointCloud>("/oa_obstacles", 1);

	ros::spin();
	return 0;
}
