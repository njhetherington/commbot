#ifndef _CUE_HPP_
#define _CUE_HPP_

#include <string>
#include <map>
#include <utility>
#include <stdexcept>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <Pulser.hpp>

using namespace geometry_msgs;

namespace commbot{

class Cue{

    public:

        // Traj on (if enabled) unless goal active
        void set_cue_traj(const PointStamped &next_position){
            
            if(is_enabled_traj_ && !is_active_goal_){
                set_cue_traj_(next_position);
                is_active_traj_ = true;
            }
            else{
                clear_cue_traj_();
                is_active_traj_ = false;
            }
        }

        // Sets goal cue msg. If close to and facing waypoint0, then set_goal_cue_() and is_active_goal_
        void set_cue_goal(const PointStamped &waypoint0, const PointStamped &waypoint1){
            
            const double_t angle_to_waypoint0 = std::atan2(waypoint0.point.y, waypoint0.point.x);
	        const double_t prox_waypoint0 = std::hypot(waypoint0.point.x, waypoint0.point.y);
            
	        if(is_enabled_goal_
                && prox_waypoint0 < min_prox_waypoint_ // robot close enough to waypoint
                && angle_to_waypoint0 > -1*M_PI_4 && angle_to_waypoint0 < M_PI_4){
                    set_cue_goal_(waypoint0, waypoint1);
                    is_active_goal_ = true;
            }
            else{
                clear_cue_goal_();
                is_active_goal_ = false;
            }

        }

        // Sets is_enabled_X_ and is_priority_traj_. No restrictions.
        void set_cues_enabled(const bool is_traj, const bool is_goal){
            is_enabled_traj_ = is_traj;
            is_enabled_goal_ = is_goal;
        }

        bool get_is_enabled_goal(){return is_enabled_goal_;}
        bool get_is_enabled_traj(){return is_enabled_traj_;}
        bool get_is_active_goal(){return is_active_goal_;}
        bool get_is_active_traj(){return is_active_traj_;}

        double_t min_prox_waypoint_; // rcfg

    protected:

        ros::NodeHandle nh_;
        ros::Publisher pub_cue_;

        virtual void publish_() = 0;
        virtual void set_cue_traj_(const PointStamped &next_position) = 0;
        virtual void set_cue_goal_(const PointStamped &waypoint0, const PointStamped &waypoint1) = 0;
        virtual void clear_cue_traj_() = 0; // Clear trajectory cue msg
        virtual void clear_cue_goal_() = 0; // Clear goal cue msg

    private:
        
        ros::Timer timer_pub_ = nh_.createTimer(ros::Rate(10.0), &Cue::cb_timer_pub, this);

        bool is_enabled_traj_ = false; // traj cue is allowed
        bool is_enabled_goal_ = false; // goal cue is allowed
        bool is_active_traj_ = false; // traj cue is in use
        bool is_active_goal_ = false; // goal cue is in use

        // Publish cue msgs from setters
        void cb_timer_pub(const ros::TimerEvent &){publish_();}

}; // class Cue 

} // namespace commbot

#endif

