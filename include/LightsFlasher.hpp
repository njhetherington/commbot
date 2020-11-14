#ifndef _LIGHTSFLASHER_HPP_
#define _LIGHTSFLASHER_HPP_

#include <Cue.hpp>
#include <Pulser.hpp>
#include <commbot/LightStates.h>

namespace commbot{

class LightsFlasher : public Cue{

    public:
    
        LightsFlasher(){
            
	        std::string topic_lights;
            ros_helpers::get_param("topics/lights", topic_lights, &nh_);
            pub_cue_ = nh_.advertise<LightStates>(topic_lights, 0);

	        ROS_INFO("%s", "LightsFlasher instantiated.");
        }

        enum Direction{left=-1, right=1, straight=0};
        
        // parameters exposed to dynamic_reconfigure
        double_t max_angle_straight_;
        double_t k_freq_goal_; // coefficient to scale goal vector length to frequency
        double_t k_freq_traj_; // scale traj vector angle (degrees) to frequency
        double_t max_freq_;

    private:

        Direction direction_traj_, direction_goal_;
        Pulser pulser_traj_, pulser_goal_;

        // Sets direction_lights_ and activates pulser_lights_ for goal cue
        void set_cue_goal_(const PointStamped &waypoint0, const PointStamped &waypoint1) override{
            double_t freq = k_freq_goal_ / std::hypot(waypoint0.point.x, waypoint0.point.y); // freq inversely proportional to proximity to waypoint0
            freq = boost::algorithm::clamp(freq, 0, max_freq_);
            const double_t angle_future_turn = std::atan2(waypoint1.point.y-waypoint0.point.y, waypoint1.point.x-waypoint0.point.x);
            const Direction dir = angle_to_direction(angle_future_turn, max_angle_straight_);
            direction_goal_ = dir;
            pulser_goal_.activate(freq);
        }

        // Sets direction_lights_ and activates pulser_lights_ for trajectory cue
        void set_cue_traj_(const PointStamped &next_position) override{
	        double_t angle = std::atan2(next_position.point.y, next_position.point.x) * 180/M_PI;
            ros_helpers::remap(angle, -180.0, 180.0, 0.0, 360.0); // remap to [0, 180] for freq calc
            double_t freq = k_freq_traj_ * angle;
            freq = boost::algorithm::clamp(freq, 0, max_freq_);
            const Direction dir = point_to_direction(next_position.point, max_angle_straight_);
	        direction_traj_ = dir;
            pulser_traj_.activate(freq);
        }

        // Sets direction_X_ to straight and deactivates pulser_X_
        void clear_cue_traj_() override{
	        direction_traj_ = Direction::straight;
            pulser_traj_.deactivate();
        }
        void clear_cue_goal_() override{
            direction_goal_ = Direction::straight; 
            pulser_goal_.deactivate(); 
        }

        // Publishes LightStates msg using direction_lights_ and pulser_lights_.get_state()
        void publish_() override{
            
            if(get_is_active_traj() && get_is_active_goal()) ROS_WARN("%s", "LightsFlasher is both traj and goal");

            LightStates light_states;
            light_states.left = false;
            light_states.right = false;

            if(get_is_active_traj()){
		    light_states = lightstates(direction_traj_, pulser_traj_);
	    }
	    else{ light_states = lightstates(direction_goal_, pulser_goal_);
	    }
            pub_cue_.publish(light_states);
        }

        LightStates lightstates(const Direction &dir, Pulser &pulser){
            LightStates light_states; // build and return
            switch(dir){
                case(Direction::left):
		        light_states.left=pulser.get_state();
                    light_states.right=false;
                    break;
                case(Direction::right):
                    light_states.right=pulser.get_state();
                    light_states.left=false;
                    break;
                case(Direction::straight):
                    light_states.left = false;
                    light_states.right = false;
                    break;
            }
	    return light_states;
        }

        // Calculates Direction (left, right, straight) from origin to Point using max angle to be considered straight
        Direction point_to_direction(const Point &point, const double_t &max_angle_straight) const{
            const double_t angle = std::atan2(point.y, point.x);
	        return angle_to_direction(angle, max_angle_straight);
        }

        Direction angle_to_direction(const double_t &angle, const double_t &max_angle_straight) const{ 
            if(angle < -1*max_angle_straight) return Direction::right;
            else if(angle > 1*max_angle_straight) return Direction::left;
            else return Direction::straight;
        }
    
};

}

#endif
