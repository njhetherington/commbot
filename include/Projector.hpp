#ifndef _PROJECTOR_HPP_
#define _PROJECTOR_HPP_

#include <cmath>
#include <limits>
#include <initializer_list>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

#include <Cue.hpp>
#include <Pulser.hpp>

using namespace geometry_msgs;
using namespace visualization_msgs;

namespace commbot{

class Projector : public Cue{

    public:

        Projector(){

            // Init publisher
            std::string topic;
            ros_helpers::get_param("topics/projector", topic);
            pub_cue_ = nh_.advertise<MarkerArray>(topic, 0);

            nh_.getParam("frame_ids/projection", FRAME_PROJECTION_);
            nh_.getParam("frame_ids/robot", FRAME_ROBOT_);
            tf_buffer_.canTransform(FRAME_PROJECTION_, FRAME_ROBOT_, ros::Time::now(), ros::Duration(3.0)); // wait for robot->projection tf to appear
            tf_robot_to_projection_ = tf_buffer_.lookupTransform(FRAME_ROBOT_, FRAME_PROJECTION_, ros::Time(0), ros::Duration(1.0));

	        ROS_INFO("%s", "Projector instantiated.");

        }

        // Configures markers_goal_
        void set_markers_config_goal(const double_t &total_length,
            const double_t &ratio_arrow_length_total_length,
            const double_t &ratio_dashes_width_total_length,
            const double_t &ratio_arrow_width_dashes_width,
            const double_t &shift_x, const double_t &shift_y,
            const double_t &k_freq_prox_waypoint, const double_t &max_freq,
            const double_t &prox_origin_min, const double_t &prox_origin_max){

            length_arrow_goal_ = total_length;
            translation_goal_.x = shift_x;
            translation_goal_.y = shift_y;
            if(k_freq_prox_waypoint > 0) k_freq_prox_waypoint_ = k_freq_prox_waypoint;
            else ROS_ERROR("%s", "Projector::set_markers_config_goal - illegal k_freq_prox_waypoint.");
            if(max_freq > 0) max_freq_ = max_freq;
            else ROS_ERROR("%s", "Projector::set_markers_config_goal - illegal max_freq.");
            PROX_ORIGIN_MIN_ = prox_origin_min;
            PROX_ORIGIN_MAX_ = prox_origin_max;

            markers_goal_.markers.clear();

            Marker line_list;
            line_list.type = Marker::LINE_LIST;
            line_list.action = Marker::MODIFY;
            line_list.color.g = 1.0;
            line_list.color.a = 1.0;
            line_list.scale.x = ratio_dashes_width_total_length * total_length;
            markers_goal_.markers.push_back(line_list);

            Marker arrow;
            arrow.type = Marker::ARROW;
            arrow.action = Marker::MODIFY;
            arrow.color.g = 1.0;
            arrow.color.a = 1.0;
            arrow.scale.x = line_list.scale.x;
            arrow.scale.y = ratio_arrow_width_dashes_width * line_list.scale.x; // head width : dashes width
            arrow.scale.z = ratio_arrow_length_total_length * total_length; // head length : total length
            markers_goal_.markers.push_back(arrow);

            pub_cue_.publish(ros_helpers::visualization::array_delete_all(FRAME_PROJECTION_));

        }
        
        // Configures markers_traj_
        void set_markers_config_traj(const double_t &length,
            const double_t &ratio_shaft_width_arrow_length,
            const double_t &ratio_head_width_shaft_width,
            const double_t &ratio_head_length_arrow_length,
            const double_t &shift_x, const double_t &shift_y){
            
            markers_traj_.markers.clear();
            Marker arrow;
            arrow.type = Marker::ARROW;
            arrow.action = Marker::MODIFY;
            arrow.color.g = 1.0;
            arrow.color.a = 1.0;

            length_arrow_traj_ = length;
            translation_traj_.x = shift_y;
            translation_traj_.y = -1*shift_x;

            // set scale using arrow length and set ratios
            arrow.scale.x = ratio_shaft_width_arrow_length * length; // shaft width
            arrow.scale.y = ratio_head_width_shaft_width * arrow.scale.x; // head width
            arrow.scale.z = ratio_head_length_arrow_length * length; // head length

            markers_traj_.markers.push_back(arrow);

            pub_cue_.publish(ros_helpers::visualization::array_delete_all(FRAME_PROJECTION_));

        }

    private:
        
        tf2_ros::Buffer tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tfl_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_, nh_));
        TransformStamped tf_robot_to_projection_;
        std::string FRAME_ROBOT_, FRAME_PROJECTION_;

        MarkerArray markers_traj_, markers_goal_;
        Pulser pulser_pub_goal_;

        const int num_dashes_goal_arrow_ = 8;

        // dynamically reconfigurable
        double_t length_arrow_traj_, length_arrow_goal_; 
        double_t prox_origin_goal_; // for placing goal marker start
        Vector3 translation_traj_, translation_goal_;
        double_t k_freq_prox_waypoint_, max_freq_;
        double_t PROX_ORIGIN_MIN_, PROX_ORIGIN_MAX_;

        // Sets markers_traj_
        void set_cue_traj_(const PointStamped &next_position) override{
            Marker arrow = markers_traj_.markers.front(); // use existing config
            arrow.header = next_position.header; // base_link
            arrow.points.clear();
            arrow.points.push_back(*std::unique_ptr<Point>(new Point)); // add empty point to start
            // calculate end point to give desired length
            Point end;
            double_t angle = std::atan2(next_position.point.y, next_position.point.x);
            end.x = length_arrow_traj_ * std::cos(angle);
            end.y = length_arrow_traj_ * std::sin(angle);
            arrow.points.push_back(end);
            markers_traj_.markers.clear();
            markers_traj_.markers.push_back(arrow);
            ros_helpers::visualization::tidy(markers_traj_);
            ros_helpers::visualization::limit_upper_half_plane(markers_traj_);
            ros_helpers::visualization::transform_points(markers_traj_, tf_robot_to_projection_);
            ros_helpers::visualization::translate(markers_traj_, translation_traj_);
        }

        // Sets markers_goal_ and activates pulser_pub_goal_.
        // Pivot about origin at angle between waypoints.
        void set_cue_goal_(const PointStamped &waypoint0, const PointStamped &waypoint1) override{
            
            int num_dashes = num_dashes_goal_arrow_;
            
            Marker line_list = markers_goal_.markers.front(); // retain marker config
            Marker arrow = markers_goal_.markers.back(); // retain marker config
            
            // calculate end point given desired length
            Point start, end;
            Point end_wrt_start = ros_helpers::geometry::subtract(waypoint1.point, waypoint0.point);
            const double_t angle_w0_to_w1 = std::atan2(end_wrt_start.y, end_wrt_start.x);

            const double_t angle_to_w0 = std::atan2(waypoint0.point.y, waypoint0.point.x);
            double_t prox_origin = std::hypot(waypoint0.point.x, waypoint0.point.y);
            ros_helpers::remap(prox_origin, 0.0, min_prox_waypoint_, PROX_ORIGIN_MIN_, PROX_ORIGIN_MAX_);
            
            start.x = prox_origin * std::cos(angle_to_w0);
            start.y = prox_origin * std::sin(angle_to_w0);
            end.x = start.x + length_arrow_goal_ * std::cos(angle_w0_to_w1);
            end.y = start.y + length_arrow_goal_ * std::sin(angle_w0_to_w1);

            // dashes
            line_list.points.clear();
            line_list.header.frame_id = waypoint0.header.frame_id;
            double_t length_dashes = length_arrow_goal_ - arrow.scale.z;
            Point end_dashes;
            end_dashes.x = start.x + length_dashes * std::cos(angle_w0_to_w1);
            end_dashes.y = start.y + length_dashes * std::sin(angle_w0_to_w1);
            line_list.points = ros_helpers::geometry::linspace_xy(start, end_dashes, num_dashes);
            line_list.points = ros_helpers::even_size(line_list.points);
            
            // arrow
            arrow.header.frame_id = waypoint0.header.frame_id;
            arrow.points.clear();
            arrow.points.push_back(end_dashes);
            arrow.points.push_back(end);

            // array
            markers_goal_.markers.clear();
            markers_goal_.markers.push_back(line_list);
            markers_goal_.markers.push_back(arrow);
            ros_helpers::visualization::transform_points(markers_goal_, tf_robot_to_projection_);
            ros_helpers::visualization::translate(markers_goal_, translation_goal_);

            // set proximity to robot
            const double_t robot_to_waypoint0 = std::hypot(waypoint0.point.x, waypoint0.point.y);
            double_t freq = k_freq_prox_waypoint_ / robot_to_waypoint0;
            freq = boost::algorithm::clamp(freq, 0, max_freq_);
            pulser_pub_goal_.activate(freq);

        }

        MarkerArray markers_;
        // Combines markers_traj_ and markers_goal_, then publishes
        void publish_() override{
            
            markers_.markers.clear();
            if(get_is_active_traj()){
		        markers_ = markers_traj_;
            }
            else if(get_is_active_goal() && pulser_pub_goal_.get_state()){
		        markers_ = markers_goal_;
            }
            
            if(markers_.markers.empty()) markers_ = ros_helpers::visualization::array_delete_all(FRAME_PROJECTION_);
            else ros_helpers::visualization::tidy(markers_);
            
            pub_cue_.publish(markers_);
            
        }

        // Makes an integer odd by decreasing if necessary
        void make_odd(int &i) const{
            if(i%2 != 0) return;
            else i -= 1;
        }

        // Does nothing
        void clear_cue_traj_() override{}
        void clear_cue_goal_() override{}

};

}

#endif
