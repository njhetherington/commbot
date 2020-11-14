#include <map>
#include <string>
#include "math.h"
#include <cmath>
#include <vector>
#include <boost/algorithm/clamp.hpp>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace geometry_msgs;
using std::string;

namespace ros_helpers{ namespace geometry{

template <class T1>
inline double_t length_xy(const T1 &xy){
    return std::hypot(xy.x, xy.y);
}

template <class T2>
inline double_t angle_xy(const T2 &xy){
    return atan2(xy.y, xy.x);
}

inline Point vector3_to_point(const Vector3 &v3){
    Point p;
    p.x = v3.x;
    p.y = v3.y;
    p.z = v3.z;
    return p;
}

inline Vector3 point_to_vector3(const Point &p){
    Vector3 v;
    v.x = p.x;
    v.y = p.y;
    v.z = p.z;
    return v;
}

inline Vector3 vector_bw_points(const Point &p0, const Point &p1){
    Vector3 result;
    result.x = p1.x - p0.x;
    result.y = p1.y - p0.y;
    result.z = p1.z - p0.z;
    return result;
}

inline double dist(const Point &p0, const Point &p1){
    const float dx = p1.x - p0.x;
    const float dy = p1.y - p0.y;
    const float dz = p1.z - p0.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

inline double angle_xy(const Point &p0, const Point &p1){
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    return (double)atan2(dy, dx);
}

inline bool is_points_equal(const Point& p0, const Point& p1){
        if(p0.x != p1.x | p0.y != p0.y | p0.z != p1.z) return false;
        else return true;
    }

template <class XYZ>
inline bool is_equal(const XYZ &a, const XYZ &b){
        if(a.x != b.x | a.y != b.y | a.z != b.z) return false;
        else return true;
}
inline bool is_equal(const Quaternion &a, const Quaternion &b){
    if(a.x != b.x || a.y != b.y || a.z != b.z || a.w != a.w) return false;
    else return true;
}

inline bool is_empty(const Quaternion &q){
    if(q.x != 0 || q.y != 0 || q.z != 0 || q.w != 0) return false;
    else return true;
}

inline bool is_empty(const Point &p){
    if(p.x != 0 || p.y != 0 || p.z != 0) return false;
    else return true;
}

inline Point tf_point(const Point &point, const TransformStamped &tf){
    Point result;
    result.x = point.x + tf.transform.translation.x;
    result.y = point.y + tf.transform.translation.y;
    result.z = point.z + tf.transform.translation.z;
    return result;
}

inline Pose pose2d_to_pose(const Pose2D &pose2d){
    Pose pose;
    pose.position.x = pose2d.x;
    pose.position.y = pose2d.y;
    tf::Quaternion q_tf = tf::createQuaternionFromYaw(pose2d.theta);
    tf::quaternionTFToMsg(q_tf, pose.orientation);
    return pose;
}

template <class T>
inline void get_param(const string &name, T& xyz, const ros::NodeHandle *nh=NULL){
    std::map<string, double> map;
    ros_helpers::get_param(name, map, nh);

    // setting fields is optional - do nothing with exceptions
    try{xyz.x = map.at("x");}catch(const std::out_of_range &e){ros_helpers::error(e, "get_param(xyz"+name+".x)");}
    try{xyz.y = map.at("y");}catch(const std::out_of_range &e){ros_helpers::error(e, "get_param(xyz"+name+".y)");}
    try{xyz.z = map.at("z");}catch(const std::out_of_range &e){ros_helpers::warn(e.what(), "get_param("+name+".z)");}
}

inline void get_param(const string &name, geometry_msgs::Quaternion& quat){
    std::map<string, double> map;
    ros::param::get(name, map);
    // setting fields is optional - do nothing with exceptions
    try{quat.x = map.at("x");}catch(std::out_of_range){}
    try{quat.y = map.at("y");}catch(std::out_of_range){}
    try{quat.z = map.at("z");}catch(std::out_of_range){}
    try{quat.w = map.at("w");}catch(std::out_of_range){}
    if(quat.x==0 && quat.y==0 && quat.z==0) quat.w = 1; // ensure proper default init (all 0s is non-sensical)
}

// Retrieves a Pose2D msg. Theta in degrees.
inline void get_param(const string &name, geometry_msgs::Pose2D &pose_2d, const ros::NodeHandle *nh=NULL){
    std::map<string, double> map;
    ros_helpers::get_param(name, map, nh);
    try{pose_2d.x = map.at("x");}catch(const std::out_of_range &){ROS_WARN("%s.x not retrieved.", name.c_str());}
    try{pose_2d.y = map.at("y");}catch(const std::out_of_range &){ROS_WARN("%s.y not retrieved.", name.c_str());}
    try{pose_2d.theta = map.at("theta")*M_PI/180;}catch(const std::out_of_range &){ROS_WARN("%s.theta not retrieved.", name.c_str());}
}

// Calculates the point in x-y at a given distance and angle from a base point
inline Point point_from_dist_angle(const Point &start, const double_t &angle, const double_t &dist){
    Point result;
    result.x = start.x + dist*cos(angle);
    result.y = start.y + dist*sin(angle);
    return result;
}

// Calculates N Points spaced evenly b/w start and end Points
std::vector<Point> linspace_xy(const Point &start, const Point &end, const int &N){

    std::vector<Point> result; // build and return

    const double_t d_start_end = ros_helpers::geometry::dist(start, end);
    const double_t d_spacing = d_start_end/N;
    const double_t theta_start_end = ros_helpers::geometry::angle_xy(start, end);
    
    result.push_back(start);
    for(int i=0; i<N; i++){ // add points to result
        if(result.empty())
            result.push_back(ros_helpers::geometry::point_from_dist_angle(*std::unique_ptr<Point>{new Point()}, theta_start_end, d_spacing)); // TODO will this cause memory leak ?
        else
            result.push_back(ros_helpers::geometry::point_from_dist_angle(result.back(), theta_start_end, d_spacing));
    }
    return result;
}

// Returns the inverse of a TransformStamped
inline TransformStamped inverse_transform_stamped(const TransformStamped &tf_s){
    
    TransformStamped result; // build and return
    
    // reverse frame_ids
    result.child_frame_id = tf_s.header.frame_id;
    result.header.frame_id = tf_s.child_frame_id;
    
    result.header.stamp = ros::Time::now(); // update timestamp

    // inverse transform
    tf2::Transform tmp;
    tf2::convert(tf_s.transform, tmp); // geometry_msgs to tf2
    tmp = tmp.inverse(); // inverse
    tf2::convert(tmp, result.transform); // tf2 to geometry_msgs

    return result;
}

// Returns first and last points if angle b/w them < max_angle, otherwise original vector
inline std::vector<Point> approx_straight_points(const std::vector<Point> &points, const double &max_angle) {
    const double angle = ros_helpers::geometry::angle_xy(points.front(), points.back()); // angle b/w first and last point in x-y
    if(angle > max_angle){ // return just first and last points
        std::vector<Point> points_out;
        points_out.push_back(points.front());
        points_out.push_back(points.back());
        return points_out;
    }
    else return points;
}

// clamps the angle of a point in x-y in its coordinate frame b/w lower and upper bounds
inline Point clamp_point_angle(const Point &point_in, const double &lower_bound, const double &upper_bound){

    Point point_out; // build and return
    
    // calc angle
    Point origin; // (0,0,0)
    double angle = ros_helpers::geometry::angle_xy(origin, point_in);
    
    // clamp angle
    angle = boost::algorithm::clamp(angle, lower_bound, upper_bound);

    // rebuild point
    const double hypot = ros_helpers::geometry::dist(origin, point_in); // hypotenuse in x-y
    point_out.x = hypot * cos(angle);
    point_out.y = hypot * sin(angle);

    return point_out;
}

// Builds a PointStamped msg using the header and position of the PoseStamped msg
inline PointStamped pointstamped_from_posestamped(const PoseStamped &pose_stamped){
    PointStamped result;
    result.header = pose_stamped.header;
    result.point = pose_stamped.pose.position;
    return result;
}

// Returns a point dist metres from Point a along angle b/w Points a and b in x-y
inline Point extrap_interp_points(const Point &a, const Point &b, const double_t &dist){
    const double_t angle = ros_helpers::geometry::angle_xy(a, b);
    return ros_helpers::geometry::point_from_dist_angle(a, angle, dist);
}

// Builds a PointStamped msg from a Header and Point (factory).
inline geometry_msgs::PointStamped PointStamped(const std_msgs::Header &header, const geometry_msgs::Point &point){
    geometry_msgs::PointStamped result;
    result.header = header;
    result.point = point;
    return result;
}

// Scales a Point or Vector3
template <class XYZ>
inline XYZ scale(const XYZ &point, const double_t &scalar){
    XYZ result;
    result.x = scalar * point.x;
    result.y = scalar * point.y;
    result.z = scalar * point.z;
    return result; 
}

// Adds two Points or two Vector3s
template <class XYZ>
inline XYZ add(const XYZ &a, const XYZ &b){
    XYZ result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

// Subtracts two Points or two Vector3s (a-b)
template <class XYZ>
inline XYZ subtract(const XYZ &a, const XYZ &b){
    XYZ result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

}} // ros_helpers::geometry
