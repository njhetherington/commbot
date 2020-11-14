#include <string>
#include <stdexcept>

#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"

using namespace geometry_msgs;
using namespace visualization_msgs;
using std::string;

namespace ros_helpers{ namespace visualization{

inline Marker transform_marker(const Marker &marker, const TransformStamped &tf){
        
    // check frame_id matches
    if(tf.header.frame_id != marker.header.frame_id)
        throw ros::Exception(ros::this_node::getName() + " transform_marker(): Mismatching frame_ids b/w "+marker.header.frame_id+" and "+tf.header.frame_id);

    Marker result = marker; // to build

    // transform each point
    auto it_point_out = result.points.begin();
    for(Point point : marker.points){
        tf2::doTransform(point, *it_point_out, tf);
        it_point_out++;
    }

    result.header.frame_id = tf.child_frame_id; // assign new frame_id to represent transform
    
    return result;
}

inline MarkerArray transform_marker_array(const MarkerArray &markers, const TransformStamped &tf){
    MarkerArray result = markers;
    result.markers.clear();
    for(Marker marker : markers.markers){
        result.markers.push_back(transform_marker(marker, tf));
    }
    return result;
}

inline void get_param(const string &name, Marker &marker) noexcept(false){
    
    std::string error_msg = "get_param(Marker) for " + name;

    // required - act on errors
    try{ros_helpers::geometry::get_param(name+"/scale", marker.scale);}catch(std::exception e){ros_helpers::error(e, error_msg + " at scale");}
    try{ros_helpers::get_param(name+"/color", marker.color);}catch(std::exception e){ros_helpers::error(e, error_msg + " at color");}
    
    // optional - do nothing with exceptions
    
    ros_helpers::get_param(name+"/header", marker.header);
   
}

// Adds current ROS time and re-assigns different IDs to each Marker in MarkerArray
inline MarkerArray tidy_marker_array(const MarkerArray &markers){
    MarkerArray markers_out = markers;
    int id = 0;
    ros::Time t = ros::Time::now();
    for(Marker marker : markers_out.markers){
        marker.id += id++; // re-assign IDs
        marker.header.stamp = t; // assign current ROS time
    }
    return markers_out;
}

// Checks if all frame ids in all markers match
inline bool frame_ids_match(const MarkerArray &array) {
    for(Marker marker : array.markers)
        if(marker.header.frame_id != array.markers.front().header.frame_id) return false;
    return true;
}

inline void transform_points(Marker &marker, const TransformStamped &tf){
    PointStamped point_stamped;
    point_stamped.header.frame_id = marker.header.frame_id;
    for(Point &point : marker.points){
        point_stamped.point = point;
        tf2::doTransform(point_stamped, point_stamped, tf);
        point = point_stamped.point;
    }
    marker.header.frame_id = tf.child_frame_id;
    marker.header.stamp = ros::Time::now();
}

inline void transform_points(MarkerArray &array, const TransformStamped &tf){
    for(Marker &marker : array.markers)
        transform_points(marker, tf);
}

inline void transform_pose(Marker &marker, const TransformStamped &tf){
    tf2::doTransform(marker.pose, marker.pose, tf);
    marker.header.frame_id = tf.child_frame_id;
    marker.header.stamp = ros::Time::now();
}

inline void transform_pose(MarkerArray &array, const TransformStamped &tf){
    for(Marker &marker : array.markers)
        transform_pose(marker, tf);
}

// Builds a MarkerArray containing one Marker with a DELETEALL action
inline MarkerArray array_delete_all(const std::string &frame_id) {
    MarkerArray result;
    Marker deleteall;
    deleteall.action = Marker::DELETEALL;
    deleteall.header.frame_id = frame_id;
    deleteall.header.stamp = ros::Time::now();
    result.markers.push_back(deleteall);
    return result;
}

inline Marker marker_delete() {
    Marker result;
    result.action = Marker::DELETE;
    return result;
}

// Adds current ROS time and re-assigns different IDs to each Marker in MarkerArray
inline void tidy(MarkerArray &array){
    int id = 0;
    ros::Time t = ros::Time::now();
    for(Marker &marker : array.markers){
        marker.id += id++; // re-assign IDs
        marker.header.stamp = t; // assign current ROS time
    }
    if(!frame_ids_match(array)) ROS_WARN("%s", "Projector::tidy() - MarkerArray with multiple frame_ids.");
}

// Recalculates Marker.points using angle
inline void re_angle_marker(Marker &marker, const double_t &angle) {
    for(Point &point : marker.points){
        if(!ros_helpers::geometry::is_equal(point, marker.points.front())){
            const double_t dist = std::hypot(point.x-marker.points.front().x, point.y-marker.points.front().y);
            point.x = marker.points.front().x + dist*std::cos(angle);
            point.y = marker.points.front().y + dist*std::sin(angle);
        }
    }
}

// Limits Marker to upper half plane
void limit_upper_half_plane(Marker &marker) {
    const double_t angle = std::atan2(marker.points.back().y - marker.points.front().y, marker.points.back().x - marker.points.front().x);
    if(angle > M_PI_2) re_angle_marker(marker, M_PI_2);
    else if(angle < -1*M_PI_2) re_angle_marker(marker, -1*M_PI_2);
}

// Limits MarkerArray to the upper half plane
void limit_upper_half_plane(MarkerArray &marker_array) {
    for(Marker &marker : marker_array.markers) limit_upper_half_plane(marker);
}

// Translates all points in every Marker in MarkerArry
void translate(MarkerArray &array, const Vector3 &translation) {
    for(Marker &marker : array.markers){
        for(Point &point : marker.points){
            point.x += translation.x;
            point.y += translation.y;
        }
    }
}

}}