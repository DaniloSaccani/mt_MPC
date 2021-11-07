#include "ros/ros.h"
#include "ros/console.h"

#include <math.h>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"

#include <mpc_s1000_controller/PolytopeMatrices.h>
#include <mpc_s1000_controller/PolytopeMatricesStamped.h>

#include "visualization_msgs/Marker.h"
#include "std_msgs/ColorRGBA.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

ros::Publisher polygon_pub;

void polyCallback(const mpc_s1000_controller::PolytopeMatricesStamped::ConstPtr& poly_ptr)
{
  geometry_msgs::PolygonStamped poly;
  poly.header = poly_ptr->header;
  poly.header.frame_id = "odom";

  for(int i=0; i<poly_ptr->polytope.n; i++)
  {
    int j=(i+1)%poly_ptr->polytope.n;
    float a1 = poly_ptr->polytope.A1[i],
      b1 = poly_ptr->polytope.A2[i],
      c1 = -poly_ptr->polytope.b[i],
      a2 = poly_ptr->polytope.A1[j],
      b2 = poly_ptr->polytope.A2[j],
      c2 = -poly_ptr->polytope.b[j];

    geometry_msgs::Point32 v;
    v.x = (b1*c2 - b2*c1) / (a1*b2 - a2*b1);
    v.y = (c1*a2 - c2*a1) / (a1*b2 - a2*b1);

    poly.polygon.points.push_back(v);
  }


  polygon_pub.publish(poly);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "polytope_matrices_viewer");
  ros::NodeHandle n;

  ros::Subscriber polytope_sub = n.subscribe("free_polytope", 1, polyCallback);
  polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("free_polytope_matrices_view", 1);

  ros::spin();

  return 0;
}
