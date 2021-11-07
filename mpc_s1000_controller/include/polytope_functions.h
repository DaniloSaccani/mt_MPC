#include "ros/ros.h"
#include "ros/console.h"

#include <stdio.h>
#include <math.h>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/PointCloud.h"

#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Return the min of 2 numbers
float min(float a, float b);

// Return the max of 2 numbers
float max(float a, float b);

// Return the distance between point a and b
float distSq(geometry_msgs::Point32 a, geometry_msgs::Point32 b);

// Given three colinear points p, q, r, the function checks if q lies between p and r
bool onSegment(geometry_msgs::Point32 p, geometry_msgs::Point32 q, geometry_msgs::Point32 r);

// Find orientation of ordered triplet (p, q, r)
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(geometry_msgs::Point32 p, geometry_msgs::Point32 q, geometry_msgs::Point32 r);

// Check intersection between segment 'p1q1' and 'p2q2'
bool doIntersect(geometry_msgs::Point32 p1, geometry_msgs::Point32 q1, geometry_msgs::Point32 p2, geometry_msgs::Point32 q2);

// check if p lies inside the convex polygon poly
bool isInsideConv(geometry_msgs::Polygon poly, geometry_msgs::Point32 p);

// check convexity of poly, true if convex
bool isConv(geometry_msgs::Polygon poly);

// count the number of points of query inside convex polygon poly
int countInsideConv(geometry_msgs::Polygon poly, sensor_msgs::PointCloud query);

// count points in query closer than d_thr to point p
int countClose(geometry_msgs::Point32 p, sensor_msgs::PointCloud query, float d_thr);

// compute angle of the normal to points in query close to p
geometry_msgs::Vector3 computeNormal(geometry_msgs::Point32 p, sensor_msgs::PointCloud query, float d_thr);

// transform point32 p with transform tf
geometry_msgs::Point32 point_transform(geometry_msgs::Point32 p, geometry_msgs::TransformStamped tf);

// transform polygonStamped p with transform tf
geometry_msgs::PolygonStamped poly_transform(geometry_msgs::PolygonStamped p, geometry_msgs::TransformStamped tf);
