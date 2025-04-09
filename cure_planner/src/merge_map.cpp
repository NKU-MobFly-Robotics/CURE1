/**
 * @file merge_map.cpp
 * @author qingchen Bi
 * @brief replace map merge package
 * @version 0.1
 * @date 2025-03-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "cure_planner/PointArray.h"
#include <tf/transform_listener.h>
#include <boost/bind.hpp>

using namespace std;

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

string map_topic;
int n_robot, robot_id;
double map_x, map_y;
int map_width, map_height;
std::vector<ros::Subscriber> map_subs;
nav_msgs::OccupancyGrid merged_map;
nav_msgs::OccupancyGrid tmp_map;
bool merge_init = false;
bool rece_map = false;

void findTF(const string parent_frame_id, const string children_frame_id, geometry_msgs::TransformStamped& transform_pose, const tf::TransformListener& listener)
{
    tf::StampedTransform transform_stamped;
    // geometry_msgs::TransformStamped transform_pose;
    // tf::Quaternion quat; 
    // double roll,pitch,yaw; 
    try
    {
            listener.waitForTransform(parent_frame_id, children_frame_id, ros::Time(0), ros::Duration(2.0));
            listener.lookupTransform(parent_frame_id, children_frame_id, ros::Time(0), transform_stamped);
            tf::transformStampedTFToMsg(transform_stamped, transform_pose);
            // point_temp.x = transform_stamped.getOrigin().x();
            // point_temp.y = transform_stamped.getOrigin().y();
            // tf::quaternionMsgToTF(transform_pose.transform.rotation,quat); 
            // tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}
void addAndDeleRobotObs(const tf::TransformListener& listener, nav_msgs::OccupancyGrid& planning_map)
{
    for(int k = 1; k < n_robot + 1; k++)
    {
        geometry_msgs::TransformStamped transform_rp;
        findTF("map", "robot_" + to_string(k) + "/base_link", transform_rp, listener);
        int r_global_index_x = CONTXY2DISC(transform_rp.transform.translation.x - merged_map.info.origin.position.x, merged_map.info.resolution); 
        int r_global_index_y = CONTXY2DISC(transform_rp.transform.translation.y - merged_map.info.origin.position.y, merged_map.info.resolution);
        if(k == robot_id)
        {
            for(int i = r_global_index_x - 2; i <= r_global_index_x + 2; i++)
            {
                for(int j = r_global_index_y - 2; j <= r_global_index_y + 2; j++)
                {
                    if(i < 0 || i >= merged_map.info.width || j < 0 || j >= merged_map.info.height)
                        continue;
                    else
                    {
                        planning_map.data[i + j * merged_map.info.width] = 0;
                    }
                }
            }
        }
        else
        {
            
            for(int i = r_global_index_x - 2; i <= r_global_index_x + 2; i++)
            {
                for(int j = r_global_index_y - 2; j <= r_global_index_y + 2; j++)
                {
                    if(i < 0 || i >= merged_map.info.width || j < 0 || j >= merged_map.info.height)
                        continue;
                    else
                    {
                        planning_map.data[i + j * merged_map.info.width] = 100;
                    }
                }
            }
        }
    }
}
void mergeMap(const int r_index_x, const int r_index_y, const geometry_msgs::TransformStamped transform_map)
{
                    
    double merge_range = 10.0;
    int step = merge_range / tmp_map.info.resolution;
    for(int i = r_index_x - step; i <= r_index_x + step; i++)
    {
        for(int j = r_index_y - step; j <= r_index_y + step; j++)
        {
            if(i < 0 || i >= tmp_map.info.width || j < 0 || j >= tmp_map.info.height)
            {
                continue;
            }
            else
            {              
                if(tmp_map.data[i + j * tmp_map.info.width] != -1)
                {
                    geometry_msgs::Point grid_global_position;
                    grid_global_position.x = DISCXY2CONT(i, tmp_map.info.resolution) + tmp_map.info.origin.position.x + transform_map.transform.translation.x;
                    grid_global_position.y = DISCXY2CONT(j, tmp_map.info.resolution) + tmp_map.info.origin.position.y + transform_map.transform.translation.y;
                    int g_global_index_x = CONTXY2DISC(grid_global_position.x - merged_map.info.origin.position.x, merged_map.info.resolution); 
                    int g_global_index_y = CONTXY2DISC(grid_global_position.y - merged_map.info.origin.position.y, merged_map.info.resolution);
                    if(merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] == -1)
                        merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = tmp_map.data[i + j * tmp_map.info.width];
                    else
                    {
                        merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = int(0.2 * double(merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width])) 
                                                                                                        + int(0.8 * double(tmp_map.data[i + j * tmp_map.info.width]));
                        if(merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] >= 80)
                            merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 100;
                        else
                            merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 0;
                    }
                    // else if(merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] == 0 && tmp_map.data[i + j * tmp_map.info.width] == 100)
                    // {
                    //     bool is_merge = false;
                    //     for(int m = i - 2; m <= i + 2;)
                    //     {
                    //         for(int n = j - 2; n <= j + 2;)
                    //         {
                    //             if(m < 0 || m >= tmp_map.info.width || n < 0 || n >= tmp_map.info.height)
                    //                 continue;
                    //             else
                    //             {
                    //                 if(tmp_map.data[m + n * tmp_map.info.width] == 100)
                    //                 {
                    //                     is_merge = true;
                    //                     break;
                    //                 }
                    //             }
                    //             n+=2;
                    //         }
                    //         m+=2;
                    //     }
                    //     if(is_merge)
                    //         merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 100;
                    //     else
                    //         merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 0;                            
                    // }
                    // else if(merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] == 100 && tmp_map.data[i + j * tmp_map.info.width] == 0)
                    // {
                    //     bool is_merge = false;
                    //     for(int m = g_global_index_x - 2; m <= g_global_index_x + 2;)
                    //     {
                    //         for(int n = g_global_index_y - 2; n <= g_global_index_y + 2;)
                    //         {
                    //             if(m < 0 || m >= merged_map.info.width || n < 0 || n >= merged_map.info.height)
                    //                 continue;
                    //             else
                    //             {
                    //                 if(merged_map.data[m + n * merged_map.info.width] == 100)
                    //                 {
                    //                     is_merge = true;
                    //                     break;
                    //                 }
                    //             }
                    //             n+=2;
                    //         }
                    //         m+=2;
                    //     }
                    //     if(is_merge)
                    //         merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 100;
                    //     else
                    //         merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 0;                            
                    // }
                }
            }
        }
    }
}

void initMergedMap()
{
    merged_map.header.frame_id = "map";
    merged_map.info.origin.position.x = map_x;
    merged_map.info.origin.position.y = map_y;
    merged_map.info.origin.position.z = -0.0;
    merged_map.info.resolution = 0.1;
    merged_map.info.width = map_width; 
    merged_map.info.height = map_height; 
    std::vector<int8_t> tmp(merged_map.info.width * merged_map.info.height, -1);
    merged_map.data = tmp;
    merge_init = true;    
}
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg,  const std::string& topic_name)
{
    tmp_map = *msg;
    rece_map = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "merge_map_node");
    ros::NodeHandle nh;
    std::string ns;
    ns=ros::this_node::getName();
    ros::param::param<int>(ns + "/n_robot", n_robot, 1);
    ros::param::param<int>(ns + "/robot_id", robot_id, 1);
    ros::param::param<double>(ns + "/map_x", map_x, 1);
    ros::param::param<double>(ns + "/map_y", map_y, 1);
    ros::param::param<int>(ns + "/map_width", map_width, 1);
    ros::param::param<int>(ns + "/map_height", map_height, 1);

    tf::TransformListener listener;

    for(int i = 1; i < n_robot + 1; i++)
    {
        map_topic = "/robot_" + std::to_string(i) + "/map";
        ros::Subscriber map_sub =
        nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 10, boost::bind(mapCallBack, _1, map_topic));
        map_subs.push_back(map_sub);
    }
    ros::Publisher merged_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/merged_map", 10);
    ros::Publisher merged_planning_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("planning_map", 10);
    ros::Rate rate(5);

    initMergedMap();

    while(ros::ok())
    {
        if(merge_init && rece_map)
        {
            std::string robot_ns = tmp_map.header.frame_id;
            int cur_id = std::stoi(robot_ns.substr(6));
            geometry_msgs::TransformStamped transform_map, transform_robot;
            string map_frame_id = "map";
            string r_map_frame_id = "robot_" + to_string(cur_id) + "/map";
            findTF(map_frame_id, r_map_frame_id, transform_map, listener);
            string rbaselink_frame_id = "robot_" + to_string(cur_id) + "/base_link";
            findTF(r_map_frame_id, rbaselink_frame_id, transform_robot, listener);
            int r_index_x = CONTXY2DISC(transform_robot.transform.translation.x - tmp_map.info.origin.position.x, tmp_map.info.resolution); 
            int r_index_y = CONTXY2DISC(transform_robot.transform.translation.y - tmp_map.info.origin.position.y, tmp_map.info.resolution);
            mergeMap(r_index_x, r_index_y, transform_map);
            merged_map.header.stamp = tmp_map.header.stamp;
            // ROS_ERROR("%d, %f, %f", cur_id, transform_map.transform.translation.x, transform_map.transform.translation.y);
            nav_msgs::OccupancyGrid planning_map;
            planning_map = merged_map;
            addAndDeleRobotObs(listener, planning_map);
            if(robot_id == 1)
                merged_map_pub.publish(merged_map); 
            merged_planning_map_pub.publish(planning_map);
        }
  
        ros::spinOnce();
        rate.sleep();  
    }
    return 0;
}