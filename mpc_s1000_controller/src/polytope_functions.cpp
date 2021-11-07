#include "polytope_functions.h"

#define INF 100000
// infinite for raytracing

// Return the min of 2 numbers
float min(float a, float b)
{
  return a<b ? a : b;
}
// Return the max of 2 numbers
float max(float a, float b)
{
  return a>b ? a : b;
}
// Return the distance between point a and b
float distSq(geometry_msgs::Point32 a, geometry_msgs::Point32 b)
{
  return (pow(a.x-b.x,2)+pow(a.y-b.y,2)+pow(a.z-b.z,2));
}
// Given three colinear points p, q, r, the function checks if q lies between p and r
bool onSegment(geometry_msgs::Point32 p, geometry_msgs::Point32 q, geometry_msgs::Point32 r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;
    return false;
}
// Find orientation of ordered triplet (p, q, r)
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(geometry_msgs::Point32 p, geometry_msgs::Point32 q, geometry_msgs::Point32 r)
{
    float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    return (val > 0) ? 1 : ((val<0) ? 2 : 0);
}
// Check intersection between segment 'p1q1' and 'p2q2'
bool doIntersect(geometry_msgs::Point32 p1, geometry_msgs::Point32 q1, geometry_msgs::Point32 p2, geometry_msgs::Point32 q2)
{
    // compute the orientation of the 4 triplets
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4) return true;
    // Special Cases
    // p1, q1, p2 colinear and p2 lies on p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    // p1, q1, q2 colinear and q2 lies on p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    // p2, q2, p1 colinear and p1 lies on p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    // p2, q2, q1 colinear and q1 lies on p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // no intersection
}
// check if p lies inside the convex polygon poly
bool isInsideConv(geometry_msgs::Polygon poly, geometry_msgs::Point32 p)
{
  int n = poly.points.size();
  // less than 3 vertices --> no polygon
  if (n < 3)
    return false;

  bool inCW = true, inCCW = true; //check for both CW and CCW orientation of vertices
  int i = 0;
  do
  {
      int next = (i+1)%n;
      if (orientation(poly.points[i], poly.points[next], p) <= 1) //==1 -> consider also boundary
        inCW = false;
      else
        if (orientation(poly.points[i], poly.points[next], p) >= -1)
          inCCW = false;

      i = next;
  } while (i != 0);
  return inCW || inCCW;
}
// check convexity of poly, true if convex
bool isConv(geometry_msgs::Polygon poly)
{
  int n = poly.points.size();
  // less than 3 vertices --> no polygon
  if (n < 3)  return false;

  int i = 0;
  do
  {
      int next = (i+1)%n, nnext = (i+2)%n;
      if (orientation(poly.points[i], poly.points[next], poly.points[nnext]) == 1) return false;
      i = next;
  } while (i != 0);
  return true;
}
// count the number of points of query inside convex polygon poly
int countInsideConv(geometry_msgs::Polygon poly, sensor_msgs::PointCloud query)
{
  int count = 0;
  for(int i=0; i<query.points.size(); i++)
    if (isInsideConv(poly, query.points[i]))
      count++;
  return count;
}

// count points in query closer than d_thr to point p
int countClose(geometry_msgs::Point32 p, sensor_msgs::PointCloud query, float d_thr)
{
  int count = 0;
  float d_thr_sq = pow(d_thr,2);
  for (int i=0; i<query.points.size(); i++)
    if (distSq(p, query.points[i]) < d_thr_sq)
      count ++;
  return count;
}

geometry_msgs::Vector3 computeNormal(geometry_msgs::Point32 p, sensor_msgs::PointCloud query, float d_thr)
{
  sensor_msgs::PointCloud closePoints;
  float d_thr_sq = pow(d_thr,2);
  float n = 0;
  float xc = 0, yc = 0;
  for (int i=0; i<query.points.size(); i++)
    if (distSq(p, query.points[i]) < d_thr_sq)
    {
      n ++;
      xc += query.points[i].x;
      yc += query.points[i].y;
      closePoints.points.push_back(query.points[i]);
    }
  xc = xc / n;
  yc = yc / n;

  float a=0, b=0, c=0, x=0, y=0;
  for (int i=0; i<n; i++)
  {
    x = closePoints.points[i].x - xc;
    y = closePoints.points[i].y - yc;
    a += x*y;
    b += pow(y,2)-pow(x,2);
    c -= x*y;
  }
  float t = ( -b + sqrt(pow(b,2)-4*a*c) ) / (2*a);

  geometry_msgs::Vector3 v;
  v.z =0;
  v.x = sqrt(1/(1+pow(t,2)));
  v.y = -t*v.x;
  return v; // check sign
}
// transform point32 p with transform tf
geometry_msgs::Point32 point_transform(geometry_msgs::Point32 p, geometry_msgs::TransformStamped tf)
{
  geometry_msgs::PointStamped p_in;
    p_in.point.x = p.x; p_in.point.y = p.y; p_in.point.z = p.z;
  geometry_msgs::PointStamped p_out;
  tf2::doTransform(p_in, p_out, tf);
  p.x = p_out.point.x; p.y = p_out.point.y; p.z = p_out.point.z;

  return p;
}

// transform polygonStamped p with transform tf
geometry_msgs::PolygonStamped poly_transform(geometry_msgs::PolygonStamped p, geometry_msgs::TransformStamped tf)
{
  int n = p.polygon.points.size();

  for(int i=0; i<n; i++)
    p.polygon.points[i] = point_transform(p.polygon.points[i], tf);

  return p;
}
