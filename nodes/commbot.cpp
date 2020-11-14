#include <map>
#include <iostream>
#include <utility>
#include <cmath>
#include <boost/algorithm/clamp.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <ros_helpers/ros_helpers.h>
#include <commbot/CommbotConfig.h>
#include <Pulser.hpp>
#include <LightsFlasher.hpp>
#include <Projector.hpp>

using namespace geometry_msgs;
using namespace visualization_msgs;

namespace commbot{

class Commbot{

    public:
    
        Commbot(){
            
            ros_helpers::get_param("frame_ids/world", FRAME_WORLD_, &nh_);
            ros_helpers::get_param("frame_ids/robot", FRAME_ROBOT_, &nh_);
            
            // init path subscriber
            std::string topic_sub_path;
            ros_helpers::get_param("topics/path", topic_sub_path, &nh_);
            sub_path_ = nh_.subscribe(topic_sub_path, 1, &Commbot::cb_path, this);

            // init waypoints subscriber
            std::string topic_sub_waypoints;
            ros_helpers::get_param("topics/waypoints", topic_sub_waypoints, &nh_);
            sub_waypoints_ = nh_.subscribe(topic_sub_waypoints, 1, &Commbot::cb_waypoints, this);
           
            ros_helpers::get_param("min_prox_waypoint", projector_.min_prox_waypoint_);
            ros_helpers::get_param("min_prox_waypoint", lights_flasher_.min_prox_waypoint_);
	    
            dynamic_reconfigure::Server<CommbotConfig>::CallbackType f = boost::bind(&Commbot::cb_rcfg, this, _1, _2);
            server_rcfg_.setCallback(f);

            tf_buffer_.canTransform(FRAME_ROBOT_, FRAME_WORLD_, ros::Time(0), ros::Duration(3.0)); // wait for world->robot tf to appear
	
	        ROS_INFO("%s running.", ros::this_node::getName().c_str());
            
        }
    
    private:

        ros::NodeHandle nh_;
        ros::Subscriber sub_path_, sub_waypoints_;
        dynamic_reconfigure::Server<CommbotConfig> server_rcfg_;

        tf2_ros::Buffer tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tfl_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_, nh_));
        
        LightsFlasher lights_flasher_;
        Projector projector_;

        double_t length_traj_; // dynamically reconfigurable
        std::string FRAME_ROBOT_, FRAME_WORLD_;

	    const ros::Duration TIMEOUT_LOOKUP_TF = ros::Duration(0.5);

        // Dynamic Reconfigure callback
        void cb_rcfg(CommbotConfig &config, uint32_t level){
            
            // enable cue types
            lights_flasher_.set_cues_enabled(config.lights_traj, config.lights_goal);
            projector_.set_cues_enabled(config.projector_traj, config.projector_goal);

            // common
            length_traj_ = config.length_traj;

            // lights
            lights_flasher_.k_freq_traj_ = config.k_lights_freq_traj;
            lights_flasher_.k_freq_goal_ = config.k_freq_goal;
            lights_flasher_.max_freq_ = config.max_freq;
            lights_flasher_.max_angle_straight_ = config.lights_max_angle_straight * M_PI/180;

            // projector goal arrow
            projector_.set_markers_config_goal(config.projector_goal_arrow_length,
                config.projector_goal_ratio_arrow_length_total_length,
                config.projector_goal_ratio_dashes_width_total_length,
                config.projector_goal_ratio_arrow_width_dashes_width,
                config.projector_goal_shift_x, config.projector_goal_shift_y,
                config.k_freq_goal, config.max_freq,
                config.projector_prox_origin_min, config.projector_prox_origin_max
            );

            // projector traj arrow
            projector_.set_markers_config_traj(config.projector_traj_arrow_length,
                config.projector_traj_ratio_shaft_width_arrow_length,
                config.projector_traj_ratio_head_width_shaft_width,
                config.projector_traj_ratio_head_length_arrow_length,
                config.projector_traj_shift_x, config.projector_traj_shift_y
            );
            
        }

        // Calculate point x metres away. Pass (in robot frame) to "cues"
        void cb_path(const nav_msgs::PathConstPtr &path){

            PointStamped next_position = point_along_path(*path, length_traj_);
            const TransformStamped tf_path_robot = tf_buffer_.lookupTransform(FRAME_ROBOT_, path->header.frame_id, ros::Time(0), TIMEOUT_LOOKUP_TF); // path -> robot
            tf2::doTransform(next_position, next_position, tf_path_robot); // robot frame

            lights_flasher_.set_cue_traj(next_position);
            projector_.set_cue_traj(next_position);

        }

        // Calls Cue::set_cue_goal() with waypoints in robot frame
        void cb_waypoints(const PoseArray::ConstPtr &msg){

            PointStamped waypoint0, waypoint1, waypoint0_robot, waypoint1_robot;
            waypoint0.point = msg->poses.front().position;
            waypoint1.point = msg->poses.back().position;
            waypoint0.header = msg->header;
            waypoint1.header = msg->header;
            
            // transform waypoints from world frame to robot frame
	        const TransformStamped tf_world_robot = tf_buffer_.lookupTransform(FRAME_ROBOT_, msg->header.frame_id, ros::Time(0), TIMEOUT_LOOKUP_TF); // world -> robot
            tf2::doTransform(waypoint0, waypoint0_robot, tf_world_robot);
            tf2::doTransform(waypoint1, waypoint1_robot, tf_world_robot);

            lights_flasher_.set_cue_goal(waypoint0_robot, waypoint1_robot);
            projector_.set_cue_goal(waypoint0_robot, waypoint1_robot);

        }

        // Calculates the point at least dist metres along the Path
        PointStamped point_along_path(const nav_msgs::Path &path, const double_t &dist) const{
            PointStamped result;
            result.header = path.header;
            result.point = path.poses.back().pose.position; // set to last point in case can't find one far enough away
            
            // Find first element further than desired dist
            for(auto pose_stamped : path.poses){
                if(ros_helpers::geometry::dist(path.poses.front().pose.position, pose_stamped.pose.position) > dist){
                    result.point.x = pose_stamped.pose.position.x;
                    result.point.y = pose_stamped.pose.position.y;
                    break;
                }
            }

            return result;
        }

}; // class Commbot

} // namespace commbot

int main(int argc, char** argv){

    ros::init(argc, argv, "commbot");
    commbot::Commbot c; 
    while(ros::ok()) ros::spin();
   
    return 0;
}
