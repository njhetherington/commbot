#include <string>
#include <vector>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

#include <ros_helpers/ros_helpers.h>

using namespace geometry_msgs;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace commbot{

class Supervisor{

    public:
        
        Supervisor() : ac_("/move_base", true){
            
            std::string topic_pub_waypoints;
            ros_helpers::get_param("topics/waypoints", topic_pub_waypoints, &nh_);
            pub_waypoints_ = nh_.advertise<PoseArray>(topic_pub_waypoints, 0);
            
            while(!ac_.waitForServer(ros::Duration(5.0))){ROS_INFO("Waiting for the move_base action server to come up");}

            ros_helpers::get_param("frame_ids/world", FRAME_WORLD_, &nh_);

            std::string goal_id;
	        ros_helpers::get_param("goal_id", goal_id);
            Pose2D junction, goal, goal1;
            ros_helpers::geometry::get_param("waypoints/junction", junction, &nh_);
            ros_helpers::geometry::get_param("waypoints/"+goal_id, goal, &nh_);
            ros_helpers::geometry::get_param("waypoints/"+goal_id+"1", goal1, &nh_);

            waypoints_.header.frame_id = FRAME_WORLD_;
            waypoints_.poses.push_back(ros_helpers::geometry::pose2d_to_pose(junction));
            waypoints_.poses.push_back(ros_helpers::geometry::pose2d_to_pose(goal));
            waypoints_.poses.push_back(ros_helpers::geometry::pose2d_to_pose(goal1));
	        it_waypoint_ = waypoints_.poses.begin();

	        // send first waypoint
	        move_base_msgs::MoveBaseGoal move_base_goal;
            move_base_goal.target_pose.header.frame_id = FRAME_WORLD_;
            move_base_goal.target_pose.header.stamp = ros::Time::now();
	        move_base_goal.target_pose.pose = *it_waypoint_;
            ac_.sendGoal(move_base_goal, 
              boost::bind(&Supervisor::cb_movebase_result, this, _1, _2),
              boost::bind(&Supervisor::cb_movebase_active, this),
              boost::bind(&Supervisor::cb_movebase_feedback, this, _1));

	    	// pause before starting
	    	ros_helpers::get_param("pause", pause_, &nh_);
		    stop_robot(pause_);

	        last_result_ = ros::Time::now();
           
            ROS_INFO("%s running. Going to %s.", ros::this_node::getName().c_str(), goal_id.c_str());
            
        }
    
    private:
    
        ros::NodeHandle nh_;
        ros::Publisher pub_waypoints_;
	    ros::Publisher pub_cmd_vel_ = nh_.advertise<Twist>("cmd_vel", 0);
        MoveBaseClient ac_;

        PoseArray waypoints_;
        std::vector<Pose>::iterator it_waypoint_;

        std::string FRAME_WORLD_;
	    ros::Duration MIN_T_TO_GOAL_ = ros::Duration(2.0); // for move_base double results
    	ros::Time last_result_;
        
        bool is_first_print_ = true;

        double_t pause_; // pause at start and each waypoint
        bool is_paused_ = false;
		    
        // Stops robot for specified duration
        void stop_robot(const double_t &duration) {
            is_paused_ = true;
            ROS_INFO("Stopping robot for %s secs", std::to_string(duration).c_str());
            const ros::Time t_start = ros::Time::now();
            const ros::Duration d = ros::Duration(duration);
            Twist twist_empty;
            while(ros::Time::now() - t_start < d){
                pub_cmd_vel_.publish(twist_empty);
                ros::Rate(10.0).sleep();
            }
            is_paused_ = false;
        }        

        // Publishes waypoints for Commbot
        void cb_movebase_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){ 

            if(it_waypoint_+1 != waypoints_.poses.end()){// at least 2 more
                PoseArray waypoints; // construct using next 2 waypoints
                waypoints.header.frame_id = waypoints_.header.frame_id;
                waypoints.header.stamp = ros::Time::now();
                auto it = it_waypoint_;
                if(is_paused_) it--; // start with previous waypoint since paused at it
                waypoints.poses.push_back(*it);
                waypoints.poses.push_back(*(it+1));
                pub_waypoints_.publish(waypoints);
            }
            else if(is_first_print_){
                is_first_print_ = false;
                ROS_WARN("%s", "On last waypoint, stopping publishing.");
            }
            
        }

        // Sends next waypoint. If no more waypoints, then shutdown.
        void cb_movebase_result(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result){
            
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
			
                // Increment waypoint only if enough time b/w results
                if(ros::Time::now() - last_result_ > MIN_T_TO_GOAL_){
                    it_waypoint_++;
                    ROS_INFO_STREAM("move_base goal succeeded. Incrementing waypoints.");
                }
                else ROS_INFO_STREAM("result too soon");

                if(it_waypoint_ != waypoints_.poses.end()){ 

                    // build and send waypoint
                    move_base_msgs::MoveBaseGoal move_base_goal;
                    move_base_goal.target_pose.header.frame_id = FRAME_WORLD_;
                    move_base_goal.target_pose.header.stamp = ros::Time::now();
                    move_base_goal.target_pose.pose = *it_waypoint_;
                    ac_.sendGoal(move_base_goal, 
                        boost::bind(&Supervisor::cb_movebase_result, this, _1, _2),
                        boost::bind(&Supervisor::cb_movebase_active, this),
                        boost::bind(&Supervisor::cb_movebase_feedback, this, _1));
                    
                    ROS_INFO("%s", "Sent waypoint to move_base.");

			        stop_robot(pause_);
		        }
                else{
                    ROS_INFO("%s", "No more waypoints, shutting down.");
                    ros::shutdown();
                }

            }

            else ROS_ERROR("move_base result: %s", state.getText().c_str());
            
	        last_result_ = ros::Time::now();

        }

        void cb_movebase_active(){ROS_INFO("%s", "move_base goal is active.");}
    
};

}

int main(int argc, char** argv){

    ros::init(argc, argv, "supervisor");
    commbot::Supervisor s; 
    while(ros::ok()) ros::spin();
   
    return 0;
}
