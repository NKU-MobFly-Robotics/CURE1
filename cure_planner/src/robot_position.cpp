/**
 * @file robot_position.cpp
 * @author qingchen Bi
 * @brief 
 * @version 0.1
 * @date 2021-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "cure_planner/PointArray.h"
#include "geometry_msgs/Point.h"
#include "string"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Int8.h"

using namespace std;

int n_robot;
cure_planner::PointArray robots_positions;
geometry_msgs::Point point_temp;
geometry_msgs::Point point_temp_vis;
string robot_ = "/robot_";
visualization_msgs::Marker rp_vis1; // only vis three robot position
visualization_msgs::Marker rp_vis2;
visualization_msgs::Marker rp_vis3;

double distance(geometry_msgs::Point & p1, geometry_msgs::Point & p2)
{
    return pow((pow(abs(p1.x - p2.x), 2) + pow(abs(p1.y - p2.y), 2)), 0.5);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_position");
    ros::NodeHandle nh;
    std::string ns;
    ns=ros::this_node::getName();
    ros::param::param<int>(ns + "/n_robot", n_robot, 1);
    tf::TransformListener listener;

    ros::Publisher rp_lsdpub = nh.advertise<cure_planner::PointArray>("/robots_positions", 10);
    ros::Publisher path_vis_pub1 = nh.advertise<visualization_msgs::Marker>("path_visualization1", 10);
    ros::Publisher path_vis_pub2 = nh.advertise<visualization_msgs::Marker>("path_visualization2", 10);
    ros::Publisher path_vis_pub3 = nh.advertise<visualization_msgs::Marker>("path_visualization3", 10);
    ros::Rate rate(10);

    rp_vis1.header.frame_id = "robot_1/map";
    rp_vis1.header.stamp = ros::Time(0);
    rp_vis1.ns = "markers";
    rp_vis1.id = 0;
    rp_vis1.type = rp_vis1.POINTS;
    rp_vis1.action = rp_vis1.ADD;
    rp_vis1.pose.orientation.w = 0.0;
    rp_vis1.scale.x = 0.08;
    rp_vis1.scale.y = 0.08;
    rp_vis1.scale.z = 0.01;
    rp_vis1.color.g = 0.0/255.0;
    rp_vis1.color.r = 255.0/255.0;
    rp_vis1.color.b = 0.0/255.0;
    rp_vis1.color.a = 1.0;
    rp_vis1.lifetime = ros::Duration();

    rp_vis2.header.frame_id = "robot_1/map";
    rp_vis2.header.stamp = ros::Time(0);
    rp_vis2.ns = "markers";
    rp_vis2.id = 0;
    rp_vis2.type = rp_vis2.POINTS;
    rp_vis2.action = rp_vis2.ADD;
    rp_vis2.pose.orientation.w = 0.0;
    rp_vis2.scale.x = 0.08;
    rp_vis2.scale.y = 0.08;
    rp_vis2.scale.z = 0.01;
    rp_vis2.color.g = 0.0/255.0;
    rp_vis2.color.r = 0.0/255.0;
    rp_vis2.color.b = 255.0/255.0;
    rp_vis2.color.a = 1.0;
    rp_vis2.lifetime = ros::Duration();

    rp_vis3.header.frame_id = "robot_1/map";
    rp_vis3.header.stamp = ros::Time(0);
    rp_vis3.ns = "markers";
    rp_vis3.id = 0;
    rp_vis3.type = rp_vis3.POINTS;
    rp_vis3.action = rp_vis3.ADD;
    rp_vis3.pose.orientation.w = 0.0;
    rp_vis3.scale.x = 0.08;
    rp_vis3.scale.y = 0.08;
    rp_vis3.scale.z = 0.01;
    rp_vis3.color.g = 255.0/255.0;
    rp_vis3.color.r = 0.0/255.0;
    rp_vis3.color.b = 255.0/255.0;
    rp_vis3.color.a = 1.0;
    rp_vis3.lifetime = ros::Duration();

    while(ros::ok())
    {
        vector<geometry_msgs::Point>().swap(robots_positions.points);
        tf::StampedTransform transform1;
        geometry_msgs::Pose current_pose_ros1;
        geometry_msgs::TransformStamped transform_pose1;
        tf::Quaternion quat1; 
        double roll1,pitch1,yaw1; 
        try
        {
            for(int i = 0; i < n_robot; i++)
            {
                listener.waitForTransform(robot_ + to_string(1) + "/map", robot_ + to_string(i+1) + "/base_link", ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform(robot_ + to_string(1) + "/map", robot_ + to_string(i+1) + "/base_link", ros::Time(0), transform1);
                tf::transformStampedTFToMsg(transform1, transform_pose1);
                point_temp.x = transform1.getOrigin().x();
                point_temp.y = transform1.getOrigin().y();
                point_temp_vis.x = transform1.getOrigin().x();
                point_temp_vis.y = transform1.getOrigin().y();
                tf::quaternionMsgToTF(transform_pose1.transform.rotation,quat1); 
                tf::Matrix3x3(quat1).getRPY(roll1,pitch1,yaw1);
                point_temp.z = yaw1;
                point_temp_vis.z = 0.0;
                robots_positions.points.push_back(point_temp);
                // TODO only vis three robots positions
                if(i % 3 == 0)
                    rp_vis1.points.push_back(point_temp_vis);
                if(i % 3 == 1)
                    rp_vis2.points.push_back(point_temp_vis);
                if(i % 3 == 2)
                    rp_vis3.points.push_back(point_temp_vis);
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        if(robots_positions.points.size() == n_robot)
        {
            rp_lsdpub.publish(robots_positions);
        }

        path_vis_pub1.publish(rp_vis1);
        path_vis_pub2.publish(rp_vis2);
        path_vis_pub3.publish(rp_vis3);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}