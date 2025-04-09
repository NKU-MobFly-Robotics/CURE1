#ifndef VISUAL_H_
#define VISUAL_H_

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "cure_planner/PointArray.h"

using namespace std;

visualization_msgs::Marker Visual_line_list(string frame_, int32_t id_, cure_planner::PointArray p_, float r, float g, float b)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = frame_;
    line_list.header.stamp = ros::Time(0);
    line_list.ns = "markers";
    line_list.id = id_;
    line_list.type = line_list.LINE_LIST;
    line_list.action = line_list.ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.scale.x = 0.25;
    line_list.scale.y = 0.01;
    //line_list.scale.z = -0.15;
    line_list.color.r = r;
    line_list.color.g = g;
    line_list.color.b = b;
    line_list.color.a = 0.5;
    line_list.lifetime = ros::Duration();
    for(int i = 0; i < p_.points.size(); i++)
    {
        line_list.points.push_back(p_.points[i]);
    }
    return line_list;
}

visualization_msgs::Marker Visual_point(string frame_, int32_t id_, float r, float g, float b, geometry_msgs::Point p_)
{
    visualization_msgs::Marker point_vis;
    point_vis.header.frame_id = frame_;
    point_vis.header.stamp = ros::Time(0);
    point_vis.ns = "markers";
    point_vis.id = id_;
    point_vis.type = point_vis.POINTS;
    point_vis.action = point_vis.ADD;
    point_vis.pose.orientation.w = 1.0;
    point_vis.scale.x = 0.8;
    point_vis.scale.y = 0.8;
    point_vis.color.g = g;
    point_vis.color.r = r;
    point_vis.color.b = b;
    point_vis.color.a = 1.0;
    point_vis.lifetime = ros::Duration();
    point_vis.points.push_back(p_);
    return point_vis;
}
visualization_msgs::Marker Visual_point(string frame_, int32_t id_, float r, float g, float b, cure_planner::PointArray visps_, float s_)
{
    visualization_msgs::Marker point_vis;
    point_vis.header.frame_id = frame_;
    point_vis.header.stamp = ros::Time(0);
    point_vis.ns = "markers";
    point_vis.id = id_;
    point_vis.type = point_vis.POINTS;
    point_vis.action = point_vis.ADD;
    point_vis.pose.orientation.w = 1.0;
    point_vis.scale.x = s_;
    point_vis.scale.y = s_;
    
    point_vis.color.g = g;
    point_vis.color.r = r;
    point_vis.color.b = b;
    point_vis.color.a = 1.0;
    point_vis.lifetime = ros::Duration();
    for(int i =0; i < visps_.points.size(); i++)
    {
        point_vis.points.push_back(visps_.points[i]);
    }
    return point_vis;
}

visualization_msgs::Marker Visual_text(string frame_, int32_t id_, float r, float g, float b, geometry_msgs::Point p_,string text_string)
{
    visualization_msgs::Marker text_vis;
    text_vis.header.frame_id = frame_;
    text_vis.header.stamp = ros::Time(0);
    text_vis.ns = "markers";
    text_vis.id = id_;
    text_vis.type = text_vis.TEXT_VIEW_FACING;
    text_vis.action = text_vis.ADD;
    text_vis.pose.orientation.w = 1.0;
    text_vis.pose.position.x = p_.x;
    text_vis.pose.position.y = p_.y;
    text_vis.scale.x = 1.5;
    text_vis.scale.y = 1.5;
    text_vis.scale.z = 1.5;
    text_vis.color.r = r;
    text_vis.color.g = g;
    text_vis.color.b = b;
    text_vis.color.a = 1.0;
    text_vis.lifetime = ros::Duration();
    text_vis.text = text_string;
    return text_vis;
}
#endif