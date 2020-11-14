#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>

#include <commbot/LightStates.h> // ROS MSG with left and right LED states
#include <ros_helpers/ros_helpers.h>

using namespace visualization_msgs;

namespace commbot{

    class LightsVisualizer{

        public:
            LightsVisualizer(){
                
                pub_marker_ = nh_.advertise<Marker>("marker_lights", 0);
                
                // light states subscriber
                std::string topic_light_states;
                ros_helpers::get_param("topics/lights", topic_light_states, &nh_);
                sub_light_states_ =  nh_.subscribe(topic_light_states, 1, &LightsVisualizer::cb_light_states, this);
                
                // build marker_
                std::string frame_projection;
                ros_helpers::get_param("frame_ids/projection", frame_projection, &nh_);
                marker_.header.frame_id = frame_projection;
                marker_.type = Marker::SPHERE_LIST;
                marker_.action = Marker::MODIFY;
                marker_.scale.x = 1; marker_.scale.y = 1; marker_.scale.z = 1;
                marker_.pose.orientation.w = 1.0;
                marker_.color.a = 1.0;
                //.points - either side of projection
                geometry_msgs::Point p_l, p_r;
                p_l.x = -5; p_l.y = -3;
                p_r.x = 5; p_r.y = -3;
                marker_.points = {p_l, p_r};
                //.colors - both orange
                std_msgs::ColorRGBA c;
                c.r = 1.0;
                c.g = 0.65;
                marker_.colors = {c, c};

                ROS_INFO("%s", "LightsVisualizer instantiated.");
            }
        
        private:

            ros::NodeHandle nh_;
            Marker marker_;

            ros::Subscriber sub_light_states_;
            ros::Publisher pub_marker_;
            
            void cb_light_states(const commbot::LightStates &light_states){
                
                // set visibility
                marker_.colors.front().a = light_states.left ? 1.0 : 0.0;
                marker_.colors.back().a = light_states.right ? 1.0 : 0.0;

                marker_.header.stamp = ros::Time::now();
                pub_marker_.publish(marker_);
            }           

    };

}

int main(int argc, char** argv){

    ros::init(argc, argv, "lights_visualizer");
    commbot::LightsVisualizer lv; 
    while(ros::ok()) ros::spin();
   
    return 0;
}