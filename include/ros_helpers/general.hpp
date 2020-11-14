#include <string>
#include <map>
#include <vector>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include <std_msgs/Header.h>

using std::string;
using namespace std_msgs;

namespace ros_helpers{

// Calls ROS_ERROR with node name and exception message, then re-throws error.
void error(const std::exception &e, const std::string &where) noexcept(false){
    ROS_ERROR("%s : %s : %s", ros::this_node::getName().c_str(), where.c_str(), e.what());
    throw e;
}

// Calls ROS_WARN with node name, location, and warning message.
void warn(const std::string &warning, const std::string &where) noexcept{
    ROS_WARN("%s : %s : %s", ros::this_node::getName().c_str(), where.c_str(), warning.c_str());
}

template<class T>
inline void get_param(const std::string &name, T &out, const ros::NodeHandle *nh=NULL){
    
    bool is_retrieved = false;
    if(nh==NULL) is_retrieved = ros::param::get(name, out);
    else is_retrieved = nh->getParam(name, out);
    if(!is_retrieved) warn(name + " not retrieved", "get_param()");
}

inline void get_param(const string &name, ColorRGBA &color){
    std::map<string,double> map;

    bool is_retrieved = ros::param::get(name, map);
    if(!is_retrieved){
        ROS_WARN("%s not retrieved.", name.c_str());
        return;
    }
    try{color.r = map.at("r");}
    catch(std::out_of_range){color.r = 0;}
    try{color.b = map.at("b");}
    catch(std::out_of_range){color.b = 0;}
    try{color.g = map.at("g");}
    catch(std::out_of_range ){color.g = 0; ROS_WARN("here");}
    try{color.a = map.at("a");}
    catch(std::out_of_range){color.a = 1;}
    if(color.r == 0 && color.g == 0 && color.b == 0) ROS_WARN("%s is 0", name.c_str());
}

inline void get_param(const string &name, Header &header){
    ros::param::get(name+"/frame_id", header.frame_id);
    header.stamp = ros::Time::now();
    header.seq = 0;
}

inline void get_param(const string &name, uint8_t &i){
    // cast to int32 b/c uint8_t doesn't work with ros::param::get
    int32_t i32 = i;
    ros::param::get(name, i32);

    i = i32;
}

// ros::Duration
// All components optional
// Defaults to forever
inline void get_param(const string &name, ros::Duration &duration){
    ROS_WARN("%s", "get_param(Duration)");
    duration = ros::Duration(0); // default forever
    std::map<string, int32_t> map;
    ros::param::get(name, map);
    try{duration.sec = map.at("sec");}catch(...){};
    try{duration.nsec = map.at("nsec");}catch(...){};
}

// Checks whether stamped msgs are in the same frame
template <class C2>
bool is_same_frame(const C2 &a, const C2 &b) noexcept(false){
    try{
        if(a.header.stamp == b.header.stamp) return true;
        else return false;
    }
    catch(std::exception e){
        ROS_ERROR(e.what());
        throw e;
    }
}

// Ensures vector has an even number of elements. Duplicates last element if necessary.
template <class T2>
std::vector<T2> even_size(std::vector<T2> &v_in){
    if(v_in.size()%2 != 0) v_in.push_back(v_in.back());
    return v_in;
}

// Header.msg factory
std_msgs::Header Header(const uint32_t &seq, const ros::Time &time, const std::string &frame_id) noexcept{
    std_msgs::Header result;
    result.seq = seq;
    result.stamp = time;
    result.frame_id = frame_id;
    return result;
}

template<class T3>
void remap(T3 &val, const T3 &lo1, const T3 &hi1, const T3 &lo2, const T3 &hi2){
    val = (val - lo1) / (hi1 - lo1) * (hi2 - hi1) + lo2;
}

} // ros_helpers namespace