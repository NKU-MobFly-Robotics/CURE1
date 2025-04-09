/**
 * @file robots.hpp
 * @author qingchen Bi
 * @brief 
 * @version 0.1
 * @date 2021-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef ROBOT_CLASS_H_
#define ROBOT_CLASS_H_

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "cure_planner/PointArray.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/GetPlan.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <string>
#include "nav_msgs/OccupancyGrid.h"
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "visualization_msgs/Marker.h"
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include "queue"
#include <chrono>

using namespace std;

class CandidatePoints
{
public:
    CandidatePoints(double r, geometry_msgs::Point p)
    {
        revenue_ = r;
        position_.x = p.x;
        position_.y = p.y;
    }
    ~CandidatePoints(){}
    double getX()
    {
        return position_.x;
    }
    double getY()
    {
        return position_.y;
    }
    friend bool operator < (const CandidatePoints &a, const CandidatePoints &b);

private:
    double revenue_;
    geometry_msgs::Point position_;
};
bool operator < (const CandidatePoints &a, const CandidatePoints &b)
{
    return a.revenue_ < b.revenue_;
}

class Robot
{
public:
    Robot(string name, tf::TransformListener & listener);
    ~Robot();
    Robot(const Robot& robot_copy);

    int cond = 0;
    move_base_msgs::MoveBaseGoal goal_;
    geometry_msgs::PoseStamped start_;
    geometry_msgs::PoseStamped end_;
    vector<float> timeforfree_;
    string name_;
    string name_temp_;
    string global_frame_;
    string robot_frame_;
    string plan_service_;
    tf::TransformListener *listener_;
    geometry_msgs::Point position_;
    float euler_;
    geometry_msgs::Point assigned_point_;
    ros::ServiceClient serviceClient_;
    tf::StampedTransform trans_;
    cure_planner::PointArray cen_;
    std::vector<float> last_goal_;
    geometry_msgs::Point branch_position_;
    vector<array<double, 2>> robot_pos_;
    vector<int> stop_duration_;
    bool robot_state_;
    ros::NodeHandle nh_;

    geometry_msgs::Point getPosition(tf::TransformListener & listener);
    float getAngRobot(tf::TransformListener & listener);
    int makePlan(geometry_msgs::Point start_, geometry_msgs::Point end_);
};

Robot::Robot(string name, tf::TransformListener & listener)
{
    this->listener_ = &listener;
    name_ = name;
    string ns;
    ns = ros::this_node::getName();
    ros::param::param<string>(ns + "/global_frame", global_frame_, "/map");
    ros::param::param<string>(ns + "/robot_frame", robot_frame_, "/base_link");
    ros::param::param<string>(ns + "/plan_service", plan_service_, "/move_base/GlobalPlanner/make_plan");
    ros::param::param<string>(ns + "/name_temp", name_temp_, "/move_base");

    listener.waitForTransform(this->global_frame_, this->name_  + this->robot_frame_, ros::Time(0), ros::Duration(10));

    cond = 0;
    while(cond == 0)
    {      
        try{    
            ROS_INFO("---Waiting for the robot Transform---");
            listener.lookupTransform(this->global_frame_, this->name_  + this->robot_frame_, ros::Time(0), trans_);
            cond = 1;
        }
        catch(tf::LookupException){
            ROS_INFO("1.lookupexception");
            cond = 0;
        }
        catch(tf::ConnectivityException){
            ROS_INFO("2.connectivityexception");
            cond = 0;
        }
        catch(tf::ExtrapolationException){
            ROS_INFO("3.extrapolationexception");
            cond = 0;
        }
    }

    this->position_.x = trans_.getOrigin().x();
    this->position_.y = trans_.getOrigin().y();
    this->assigned_point_ = this->position_;

    string tempp = this->name_;
    while(!ros::service::waitForService(tempp.erase(0,1) + this->plan_service_, ros::Duration(3.0)))
    {
        ROS_INFO("wait for plan");
    }
    string temppp = this->name_;
    this->serviceClient_ = nh_.serviceClient<nav_msgs::GetPlan>(temppp.erase(0,1) + this->plan_service_, true);
    Robot::start_.header.frame_id = this->global_frame_;
    Robot::end_.header.frame_id = this->global_frame_;
    branch_position_.x = 0;
    branch_position_.y = 0;
    robot_state_ = true;
    stop_duration_.clear();
    last_goal_.clear();
}

Robot::~Robot()
{
}

Robot::Robot(const Robot& robot_copy)
{
    start_ = robot_copy.start_;
    end_ = robot_copy.end_;
    timeforfree_ = robot_copy.timeforfree_;
    name_ = robot_copy.name_;
    global_frame_ = robot_copy.global_frame_;
    robot_frame_ = robot_copy.robot_frame_;
    plan_service_ = robot_copy.plan_service_;
    start_ = robot_copy.start_;      
    listener_ = robot_copy.listener_;
    position_ = robot_copy.position_;
    euler_ = robot_copy.euler_;
    assigned_point_ = robot_copy.assigned_point_;
    serviceClient_ = robot_copy.serviceClient_;   
    cen_ = robot_copy.cen_;
    branch_position_ = robot_copy.branch_position_;
    nh_ = robot_copy.nh_; 
    trans_ = robot_copy.trans_;
    robot_state_ = robot_copy.robot_state_;
    stop_duration_ = robot_copy.stop_duration_;
    robot_pos_ = robot_copy.robot_pos_;
    last_goal_ = robot_copy.last_goal_;
}

geometry_msgs::Point Robot::getPosition(tf::TransformListener & listener)
{
   cond = 0;
   this->listener_ = &listener;
    while(cond ==0)
    {
        try{
            listener.lookupTransform(this->global_frame_, this->name_  + this->robot_frame_, ros::Time(0), trans_);
            cond = 1;
        }
        catch(tf::LookupException){
            cond = 0;
        }
        catch(tf::ConnectivityException){
            cond = 0;
        }
        catch(tf::ExtrapolationException){
            cond = 0;
        }
    }
    this->position_.x = trans_.getOrigin().x();
    this->position_.y = trans_.getOrigin().y();
    return this->position_;
}

float Robot::getAngRobot(tf::TransformListener & listener)
{
    cond = 0;
    this->listener_ = &listener;
    geometry_msgs::TransformStamped TransformStamped_Yaw; 
    tf::Quaternion quat_Yaw;
    double Roll, Pitch, Yaw;

    while(cond ==0)
    {
        try{
            listener.lookupTransform(this->global_frame_, this->name_  + this->robot_frame_, ros::Time(0), trans_);
            cond = 1;
        }
        catch(tf::LookupException){
            cond = 0;
        }
        catch(tf::ConnectivityException){
            cond = 0;
        }
        catch(tf::ExtrapolationException){
            cond = 0;
        }
    }
    tf::transformStampedTFToMsg(trans_, TransformStamped_Yaw);
    tf::quaternionMsgToTF(TransformStamped_Yaw.transform.rotation, quat_Yaw);
    tf::Matrix3x3(quat_Yaw).getRPY(Roll, Pitch, Yaw);
    this->euler_ = Yaw; 
    return this->euler_;    
}

int Robot::makePlan(geometry_msgs::Point start_, geometry_msgs::Point end_)
{   string temp_name = name_;
    string temp_name2 = name_;
    nav_msgs::GetPlan srv;
    srv.request.start.header.frame_id = temp_name.erase(0,1) + "/map";
    srv.request.start.pose.position.x = start_.x;
    srv.request.start.pose.position.y = start_.y;
    srv.request.start.pose.orientation.w = 1.0;
    srv.request.goal.header.frame_id = temp_name2.erase(0,1) + "/map";
    srv.request.goal.pose.position.x = end_.x;
    srv.request.goal.pose.position.y = end_.y;
    srv.request.goal.pose.orientation.w = 1.0;
    srv.request.tolerance = 0.3;
    if(serviceClient_.call(srv))
    {
        //if(srv.response.plan.poses.size())
        return srv.response.plan.poses.size();
    }
    else
    {
        return 0;
    }
}

int indexPoint(nav_msgs::OccupancyGrid mapData, array<float, 2> Xp)
{
    float resolution = mapData.info.resolution;
    double Xstartx = mapData.info.origin.position.x;
    double Xstarty = mapData.info.origin.position.y;
    int width = mapData.info.width;
    int index = int(( floor((Xp[1] - Xstarty) / resolution) * 
                    width) + (floor((Xp[0] - Xstartx) / resolution)));
    return index;    
}
array<double, 2> pointIndex(nav_msgs::OccupancyGrid mapData, int i)
{
    array<double, 2> temp;
    temp[0] = mapData.info.origin.position.x + (i - (i / mapData.info.width) * (mapData.info.width)) * mapData.info.resolution;
    temp[1] = mapData.info.origin.position.y + (i / mapData.info.width) * mapData.info.resolution;
    return temp;
}
double informationGain(nav_msgs::OccupancyGrid mapData, array<float, 2> point, double r)
{
    bool flag_state = false;
    double infoGain = 0;
    int index = indexPoint(mapData, point);
    int r_regin = int(r/mapData.info.resolution);
    int init_index = index - r_regin *(mapData.info.width + 1);
    for(int n = 0; n < 2*r_regin + 1; n ++)
    {
        int start = n*mapData.info.width + init_index;
        int end = start + 2*r_regin;
        int limit = ((start / mapData.info.width)+2)*mapData.info.width;
        if(limit < end)
        {
           
        }
        for(int i = start; i < end + 1; i++)
        {
            if((i >= 0) && (i < limit) && i < mapData.data.size())
            {                
                int m = i / mapData.info.width; 
                int n = i % mapData.info.width; 
                int rs = index / mapData.info.width;
                int c = index % mapData.info.width;
                if(mapData.data[i] == -1 && ((((rs-m) * (rs-m)) + ((c-n) * (c-n))) <= (r_regin * r_regin)) )
                    infoGain += 1;
            }
        }
    }
    return infoGain * pow(mapData.info.resolution, 2);
}
double findInforGainR(nav_msgs::OccupancyGrid mapData, array<float, 2> point, double r1, double r2, double stepR)
{
    nav_msgs::OccupancyGrid mapData_ = mapData;
    array<float, 2> point_ = point;
    double r1_= r1;
    double r2_ = r2;
    double stepR_ = stepR;
    double infoGain = 0;
    int flag = 0;
    int index = indexPoint(mapData_, point_);
    int r_regin = int(r1_ / mapData_.info.resolution);
    int init_index = index - r_regin * (mapData_.info.width + 1); 
    for(int n = 0; n < 2 * r_regin + 1; n ++)
    {                
        int start = n * mapData_.info.width + init_index;
        int end = start + 2 * r_regin;
        int limit = ((start / mapData_.info.width)+2)*mapData_.info.width;
        for(int i = start; i < end + 1; i++)
        {
            if((i >= 0) && (i < limit) && i < mapData_.data.size())
            {
                int m = i / mapData.info.width; 
                int n = i % mapData.info.width;
                int r = index / mapData.info.width;
                int c = index % mapData.info.width;
                if((mapData_.data[i] == 100 || mapData_.data[i] == 0) && ((((r-m) * (r-m)) + ((c-n) * (c-n))) <= (r_regin * r_regin)))
                    return r1_;
            }
        } 
    }            

    r1_ = r1_ + stepR_;
    if(r1_ < r2_)
        findInforGainR(mapData_, point_,r1_, r2_, stepR_);
    if(r1_ >= r2_)
        return r2_;
   
    return r2_;
}
bool isVaild(nav_msgs::OccupancyGrid mapData, array<float, 2> point)
{
    float resolution = mapData.info.resolution;
    double Xstartx = mapData.info.origin.position.x;
    double Xstarty = mapData.info.origin.position.y;
    int width = mapData.info.width;
    int index = (floor((point[1] - Xstarty) / resolution) * width) +
                (floor((point[0] - Xstartx) / resolution));
    int index_up = index + width;
    int index_down = index - width;
    int index_left = index - 1;
    int index_right = index + 1;
    int index_leup = index_up - 1;
    int index_riup = index_up + 1;
    int index_ledo = index_down - 1;
    int index_rido = index_down + 1;
    if(int(index)< mapData.data.size())
    {
        if(mapData.data[int(index)] == 100 || mapData.data[int(index)] == 0)
            return false;
    }
    if( (int(index_up) > 0 && int(index_up)< mapData.data.size() && mapData.data[int(index_up)] == 0) ||
        (int(index_down) > 0 && int(index_down)< mapData.data.size() && mapData.data[int(index_down)] == 0) ||
        (int(index_left) > 0 && int(index_left)< mapData.data.size() && mapData.data[int(index_left)] == 0) ||
        (int(index_right) > 0 && int(index_right)< mapData.data.size() && mapData.data[int(index_right)] == 0) ||
        (int(index_leup) > 0 && int(index_leup)< mapData.data.size() && mapData.data[int(index_leup)] == 0) ||
        (int(index_riup) > 0 && int(index_riup)< mapData.data.size() && mapData.data[int(index_riup)] == 0) ||
        (int(index_ledo) > 0 && int(index_ledo)< mapData.data.size() && mapData.data[int(index_ledo)] == 0) ||
        (int(index_rido) > 0 && int(index_rido)< mapData.data.size() && mapData.data[int(index_rido)] == 0) )
    {
        return true;
    }
    if( (int(index_up) > 0 && int(index_up)< mapData.data.size() && mapData.data[int(index_up)] == 100) ||
        (int(index_down) > 0 && int(index_down)< mapData.data.size() && mapData.data[int(index_down)] == 100) ||
        (int(index_left) > 0 && int(index_left)< mapData.data.size() && mapData.data[int(index_left)] == 100) ||
        (int(index_right) > 0 && int(index_right)< mapData.data.size() && mapData.data[int(index_right)] == 100) ||
        (int(index_leup) > 0 && int(index_leup)< mapData.data.size() && mapData.data[int(index_leup)] == 100) ||
        (int(index_riup) > 0 && int(index_riup)< mapData.data.size() && mapData.data[int(index_riup)] == 100) ||
        (int(index_ledo) > 0 && int(index_ledo)< mapData.data.size() && mapData.data[int(index_ledo)] == 100) ||
        (int(index_rido) > 0 && int(index_rido)< mapData.data.size() && mapData.data[int(index_rido)] == 100) )
    {
        return false;
    }
    else
        return true;
}
bool isVaild(nav_msgs::OccupancyGrid mapData, array<float, 2> point, double d)
{
    float resolution = mapData.info.resolution;
    double Xstartx = mapData.info.origin.position.x;
    double Xstarty = mapData.info.origin.position.y;
    int width = mapData.info.width;
    int gridcount = (int)(d/resolution);
    for(int i = 1; i <= gridcount; i++)
    {
        int index = (floor((point[1] - Xstarty) / resolution) * width) +
                    (floor((point[0] - Xstartx) / resolution));
        int index_up = index + (i*width);
        int index_down = index - (i*width);
        int index_left = index - i;
        int index_right = index + i;
        int index_leup = index_up - i;
        int index_riup = index_up + i;
        int index_ledo = index_down - i;
        int index_rido = index_down + i;

        if(int(index)< mapData.data.size())
        {
            if(mapData.data[int(index)] == 100 || mapData.data[int(index)] == 0)
                return false;
        }

        if( (int(index_up) > 0 && int(index_up)< mapData.data.size() && mapData.data[int(index_up)] == 0) ||
            (int(index_down) > 0 && int(index_down)< mapData.data.size() && mapData.data[int(index_down)] == 0) ||
            (int(index_left) > 0 && int(index_left)< mapData.data.size() && mapData.data[int(index_left)] == 0) ||
            (int(index_right) > 0 && int(index_right)< mapData.data.size() && mapData.data[int(index_right)] == 0) ||
            (int(index_leup) > 0 && int(index_leup)< mapData.data.size() && mapData.data[int(index_leup)] == 0) ||
            (int(index_riup) > 0 && int(index_riup)< mapData.data.size() && mapData.data[int(index_riup)] == 0) ||
            (int(index_ledo) > 0 && int(index_ledo)< mapData.data.size() && mapData.data[int(index_ledo)] == 0) ||
            (int(index_rido) > 0 && int(index_rido)< mapData.data.size() && mapData.data[int(index_rido)] == 0) 
            )
        {
            if(i == gridcount)
                return true;
        }

        if( (int(index_up) > 0 && int(index_up)< mapData.data.size() && mapData.data[int(index_up)] == 100) ||
            (int(index_down) > 0 && int(index_down)< mapData.data.size() && mapData.data[int(index_down)] == 100) ||
            (int(index_left) > 0 && int(index_left)< mapData.data.size() && mapData.data[int(index_left)] == 100) ||
            (int(index_right) > 0 && int(index_right)< mapData.data.size() && mapData.data[int(index_right)] == 100) ||
            (int(index_leup) > 0 && int(index_leup)< mapData.data.size() && mapData.data[int(index_leup)] == 100) ||
            (int(index_riup) > 0 && int(index_riup)< mapData.data.size() && mapData.data[int(index_riup)] == 100) ||
            (int(index_ledo) > 0 && int(index_ledo)< mapData.data.size() && mapData.data[int(index_ledo)] == 100) ||
            (int(index_rido) > 0 && int(index_rido)< mapData.data.size() && mapData.data[int(index_rido)] == 100) 
            )
        {
            return false;
        }
    }
    return true;        
}
bool isInvaildCen(nav_msgs::OccupancyGrid mapData, array<float, 2> point, double d1, double d2)
{
    float resolution = mapData.info.resolution;
    double Xstartx = mapData.info.origin.position.x;
    double Xstarty = mapData.info.origin.position.y;
    int width = mapData.info.width;
    int gridcount1 = (int)(d1/resolution);
    int gridcount2 = (int)(d2/resolution);
    for(int i = gridcount1; i <= gridcount2; i++)
    {
        int index = (floor((point[1] - Xstarty) / resolution) * width) +
                    (floor((point[0] - Xstartx) / resolution));
        int index_up = index + (i*width);
        int index_down = index - (i*width);
        int index_left = index - i;
        int index_right = index + i;
        int index_leup = index_up - i;
        int index_riup = index_up + i;
        int index_ledo = index_down - i;
        int index_rido = index_down + i;
        if(int(index)< mapData.data.size())
        {
            if(mapData.data[int(index)] == 100 || mapData.data[int(index)] == 0)
                return true;
        }
        if( (int(index_up) > 0 && int(index_up)< mapData.data.size() && mapData.data[int(index_up)] == 0) ||
            (int(index_down) > 0 && int(index_down)< mapData.data.size() && mapData.data[int(index_down)] == 0) ||
            (int(index_left) > 0 && int(index_left)< mapData.data.size() && mapData.data[int(index_left)] == 0) ||
            (int(index_right) > 0 && int(index_right)< mapData.data.size() && mapData.data[int(index_right)] == 0) ||
            (int(index_leup) > 0 && int(index_leup)< mapData.data.size() && mapData.data[int(index_leup)] == 0) ||
            (int(index_riup) > 0 && int(index_riup)< mapData.data.size() && mapData.data[int(index_riup)] == 0) ||
            (int(index_ledo) > 0 && int(index_ledo)< mapData.data.size() && mapData.data[int(index_ledo)] == 0) ||
            (int(index_rido) > 0 && int(index_rido)< mapData.data.size() && mapData.data[int(index_rido)] == 0) 
            )
        {
            return false;
        }
    }
    return true;        
}
int gridValue(nav_msgs::OccupancyGrid mapData, array<float, 2> point)
{
    float resolution = mapData.info.resolution;
    double Xstartx = mapData.info.origin.position.x;
    double Xstarty = mapData.info.origin.position.y;
    int width = mapData.info.width;
    int index = (floor((point[1] - Xstarty) / resolution) * width) +
                (floor((point[0] - Xstartx) / resolution));
    if(int(index) < mapData.data.size())
        return mapData.data[int(index)];
    else
        return 100;
    
}
bool getArroundGridValue(nav_msgs::OccupancyGrid mapData, array<float, 2> point, double d)
{
    float resolution = mapData.info.resolution;
    double Xstartx = mapData.info.origin.position.x;
    double Xstarty = mapData.info.origin.position.y;
    int width = mapData.info.width;
    int gridcount = (int)(d/resolution);
    for(int i = 1; i <= gridcount; i++)
    {
        int index = (floor((point[1] - Xstarty) / resolution) * width) +
                    (floor((point[0] - Xstartx) / resolution));

        int index_up = index + (i*width);
        int index_down = index - (i*width);
        int index_left = index - i;
        int index_right = index + i;
        int index_leup = index_up - i;
        int index_riup = index_up + i;
        int index_ledo = index_down - i;
        int index_rido = index_down + i;

        if(int(index)< mapData.data.size())
        {
            if(mapData.data[int(index)] == 100 || mapData.data[int(index)] == 0)
                return false;
        }
        if( (int(index_up) > 0 && int(index_up)< mapData.data.size() && mapData.data[int(index_up)] == 0) ||
            (int(index_down) > 0 && int(index_down)< mapData.data.size() && mapData.data[int(index_down)] == 0) ||
            (int(index_left) > 0 && int(index_left)< mapData.data.size() && mapData.data[int(index_left)] == 0) ||
            (int(index_right) > 0 && int(index_right)< mapData.data.size() && mapData.data[int(index_right)] == 0) ||
            (int(index_leup) > 0 && int(index_leup)< mapData.data.size() && mapData.data[int(index_leup)] == 0) ||
            (int(index_riup) > 0 && int(index_riup)< mapData.data.size() && mapData.data[int(index_riup)] == 0) ||
            (int(index_ledo) > 0 && int(index_ledo)< mapData.data.size() && mapData.data[int(index_ledo)] == 0) ||
            (int(index_rido) > 0 && int(index_rido)< mapData.data.size() && mapData.data[int(index_rido)] == 0) 
            )
        {
            return false;
        }
        if( (int(index_up) > 0 && int(index_up)< mapData.data.size() && mapData.data[int(index_up)] == 100) ||
            (int(index_down) > 0 && int(index_down)< mapData.data.size() && mapData.data[int(index_down)] == 100) ||
            (int(index_left) > 0 && int(index_left)< mapData.data.size() && mapData.data[int(index_left)] == 100) ||
            (int(index_right) > 0 && int(index_right)< mapData.data.size() && mapData.data[int(index_right)] == 100) ||
            (int(index_leup) > 0 && int(index_leup)< mapData.data.size() && mapData.data[int(index_leup)] == 100) ||
            (int(index_riup) > 0 && int(index_riup)< mapData.data.size() && mapData.data[int(index_riup)] == 100) ||
            (int(index_ledo) > 0 && int(index_ledo)< mapData.data.size() && mapData.data[int(index_ledo)] == 100) ||
            (int(index_rido) > 0 && int(index_rido)< mapData.data.size() && mapData.data[int(index_rido)] == 100) 
            )
        {
            return false;
        }
    }
    return true;        
}
bool isReachable(geometry_msgs::Point &p, cure_planner::PointArray &ps)
{
    for(int i = 0; i < ps.points.size(); i++)
    {
        if(abs(p.x - ps.points[i].x) < 0.8 && abs(p.y - ps.points[i].y) < 0.8)
            return false;
    }
    return true;
}
bool isvaildFrontier(nav_msgs::OccupancyGrid mapData, geometry_msgs::Point pf)
{
    float resolution = mapData.info.resolution;
    double Xstartx = mapData.info.origin.position.x;
    double Xstarty = mapData.info.origin.position.y;
    int width = mapData.info.width;
    int index = (floor((pf.y - Xstarty) / resolution) * width) +
                (floor((pf.x - Xstartx) / resolution));
    int index_up = index + width;
    int index_down = index - width;
    int index_left = index - 1;
    int index_right = index + 1;
    int index_leup = index_up - 1;
    int index_riup = index_up + 1;
    int index_ledo = index_down - 1;
    int index_rido = index_down + 1;
    if(int(index)< mapData.data.size())
    {
        if(mapData.data[int(index)] == 100)
            return false;
    }
    if( (int(index_up) > 0 && int(index_up)< mapData.data.size() && mapData.data[int(index_up)] == 100) ||
        (int(index_down) > 0 && int(index_down)< mapData.data.size() && mapData.data[int(index_down)] == 100) ||
        (int(index_left) > 0 && int(index_left)< mapData.data.size() && mapData.data[int(index_left)] == 100) ||
        (int(index_right) > 0 && int(index_right)< mapData.data.size() && mapData.data[int(index_right)] == 100) ||
        (int(index_leup) > 0 && int(index_leup)< mapData.data.size() && mapData.data[int(index_leup)] == 100) ||
        (int(index_riup) > 0 && int(index_riup)< mapData.data.size() && mapData.data[int(index_riup)] == 100) ||
        (int(index_ledo) > 0 && int(index_ledo)< mapData.data.size() && mapData.data[int(index_ledo)] == 100) ||
        (int(index_rido) > 0 && int(index_rido)< mapData.data.size() && mapData.data[int(index_rido)] == 100) )
    {
        return false;
    }
    else
        return true;
}
#endif


