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

#include "polytope_functions.h"

int num_vertices = 8;
float dist_step = 0.01;
float start_dist = 0.02;
float safe_dist = 3;
float update_freq = 5;

ros::Publisher poly_matrices_pub, poly_pub;

geometry_msgs::TransformStamped l2g_tf;
tf2_ros::Buffer tfBuffer;

sensor_msgs::LaserScan scan;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_ptr)
{
  scan = *scan_ptr;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "polytope_matrices_generation");
  ros::NodeHandle n;

  ros::Subscriber scan_sub = n.subscribe("scan", 1, scanCallback);
  poly_matrices_pub = n.advertise<mpc_s1000_controller::PolytopeMatricesStamped>("free_polytope", 1);
  poly_pub = n.advertise<geometry_msgs::PolygonStamped>("free_polytope_vertices", 1);

  ros::param::get("/polytope_generation_param/rate", update_freq);
  ros::Rate loop_rate(update_freq); // execute at 1 Hz

  tf2_ros::TransformListener tfListener(tfBuffer);

  while(ros::ok())
  {
    ros::spinOnce();

    int num_measurements = scan.ranges.size();

    if (num_measurements > 0)
    {
      ros::param::get("/polytope_generation_param/num_vertices", num_vertices);
      ros::param::get("/polytope_generation_param/distance_step", dist_step);
      ros::param::get("/polytope_generation_param/start_distance", start_dist);
      ros::param::get("/gbeam_controller/robot_param/safe_dist", safe_dist);


      //get transform
      try
      {
        //get transformation from "/odom" to "/base_scan" at timestamp of the polytope (same as the laser scan actually)
        l2g_tf = tfBuffer.lookupTransform("odom", "base_scan", scan.header.stamp);
      }
      catch(tf2::TransformException &ex)
      {
        ROS_WARN("polytope_matrices_generation:lookupTransform: %s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }


      // initialize vector of obstacle points
      sensor_msgs::PointCloud obstacles;
      obstacles.header = scan.header;
      //                                                                                        obstacles.header.frame_id = "base_scan";
      for(int i=0; i<num_measurements; i++)
      {
        float a = scan.angle_min + i*scan.angle_increment;
        float range = scan.ranges[i];
        // reduce range of safe_dist
        range -= safe_dist;
        if ((range >= scan.range_max) || (range < start_dist))
          range = scan.range_max;
        else
        {
          geometry_msgs::Point32 p;
          p.x = range * cos(a);
          p.y = range * sin(a);
          p.z = 0;
          obstacles.points.push_back(p);
        }
      }

      // initialize polygon vertices at start_dist
      geometry_msgs::PolygonStamped poly;
      poly.header = scan.header;
      //                                                                                        free_poly.header.frame_id = "base_scan";
      float angle_diff_vert = 2*M_PI / num_vertices;
      float vert_angle[num_vertices], vert_distance[num_vertices];
      bool move_vertex[num_vertices];
      for(int v=0; v<num_vertices; v++)
      {
        vert_angle[v] = v*angle_diff_vert + angle_diff_vert/2;
        vert_distance[v] = start_dist;
        move_vertex[v] = true;
        geometry_msgs::Point32 p;
        p.x = vert_distance[v] * cos(vert_angle[v]);
        p.y = vert_distance[v] * sin(vert_angle[v]);
        p.z = 0;
        poly.polygon.points.push_back(p);
      }

      //increase polygon until reaching obstacles
      int num_stopped = 0;
      while (num_stopped < num_vertices)
      {
        for(int v=0; v<num_vertices; v++)
        {
          // generate convex polytope
          if (move_vertex[v])
          {
            vert_distance[v] += dist_step;
            poly.polygon.points[v].x = vert_distance[v] * cos(vert_angle[v]);
            poly.polygon.points[v].y = vert_distance[v] * sin(vert_angle[v]);
            bool containsObstacles = countInsideConv(poly.polygon, obstacles) > 0;
            if (!isConv(poly.polygon) || containsObstacles || vert_distance[v] > scan.range_max)
            { //discard move
              vert_distance[v] -= dist_step;
              poly.polygon.points[v].x = vert_distance[v] * cos(vert_angle[v]);
              poly.polygon.points[v].y = vert_distance[v] * sin(vert_angle[v]);
              move_vertex[v] = false;
              num_stopped++;
            } //else keep it and go on
          }
        }
      }

      // transform polytope to bring it to static local reference frame
      poly = poly_transform(poly, l2g_tf);
      poly.header.frame_id = "odom";
      // transform drone position (0,0,0) to static local reference frame
      geometry_msgs::PointStamped p_zero;
      geometry_msgs::PointStamped p_drone;
      tf2::doTransform(p_zero, p_drone, l2g_tf);

      //compute inequalities from vertices, with following equations
      mpc_s1000_controller::PolytopeMatricesStamped poly_mat;
      poly_mat.header = scan.header;
      poly_mat.header.frame_id = "odom";
      poly_mat.polytope.n = num_vertices;

      for(int i=0; i<num_vertices; i++)
      {
        int j = (i+1)%num_vertices;
        float a1 = - (poly.polygon.points[j].y - poly.polygon.points[i].y);
        float a2 = (poly.polygon.points[j].x - poly.polygon.points[i].x);
        float b = a1 * poly.polygon.points[i].x + a2 * poly.polygon.points[i].y;
        // check if inequality is satisfied for drone position (0,0), if not invert sign
        if(a1*p_drone.point.x + a2*p_drone.point.y > b)
        {
          a1 = -a1;
          a2 = -a2;
          b = -b;
        }
        poly_mat.polytope.A1.push_back(a1);
        poly_mat.polytope.A2.push_back(a2);
        poly_mat.polytope.b.push_back(b);
      }

      //publish on topics
      poly_matrices_pub.publish(poly_mat);
      poly_pub.publish(poly);
    }

  loop_rate.sleep();

  } // while(ros::ok())

  return 0;
}
