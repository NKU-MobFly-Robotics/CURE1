/**
 * @file frontiers.cpp
 * @author qingchen Bi
 * @brief 
 * @version 0.1
 * @date 2021-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "cure_planner/PointArray.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rosvoronoi.h"
#include "visualization_msgs/Marker.h"
#include "string"
#include "visual.h"
#include <chrono>

using namespace std;
using namespace cv;

int flag = 0;
nav_msgs::OccupancyGrid map_data; 
float re_xy[2]; 
cure_planner::PointArray  frontiers_points; 
geometry_msgs::Point frontiers_point; 
visualization_msgs::Marker frontiers_vis;
vector<Point2f> points_lsd;
string frontiers_map_topic;

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_data=*msg;
    re_xy[0] = msg->info.origin.position.x;
    re_xy[1] = msg->info.origin.position.y;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontiers");
    ros::NodeHandle nh;

    std::string ns;
    ns=ros::this_node::getName();
    ros::param::param<std::string>(ns + "/frontiers_map_topic", frontiers_map_topic, "/map_merge/map");

    ros::Subscriber map_sub = nh.subscribe(frontiers_map_topic, 1000, mapCallBack);
    ros::Publisher frontiers_pub = nh.advertise<cure_planner::PointArray>("/frontiers", 10);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("frontiers_visualization", 10);

    ros::Rate rate(10);

    while(map_data.data.size() < 1 or map_data.header.seq < 1){ 
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    frontiers_vis.header.frame_id = map_data.header.frame_id;
    frontiers_vis.header.stamp = ros::Time(0);
    frontiers_vis.ns = "markers";
    frontiers_vis.id = 0;
    frontiers_vis.type = frontiers_vis.POINTS;
    frontiers_vis.action = frontiers_vis.ADD;
    frontiers_vis.pose.orientation.w = 1.0;
    frontiers_vis.scale.x = 0.2;
    frontiers_vis.scale.y = 0.2;
    frontiers_vis.color.g = 255.0/255.0;
    frontiers_vis.color.r = 0.0/255.0;
    frontiers_vis.color.b = 0.0/255.0;
    frontiers_vis.color.a = 1.0;
    frontiers_vis.lifetime = ros::Duration();

    while(ros::ok()){
        auto decision_begin_time = std::chrono::system_clock::now();

        Mat image = getRosImage(map_data);
        Mat gray1,gray2,gray3,obs,edge,obs_INV,frontiers; 
        threshold(image,gray1,200,255,THRESH_BINARY_INV);
        // imshow("gray1",gray1);
        threshold(image,gray2,20,255,THRESH_BINARY);
        // imshow("gray2",gray2);
        threshold(image,gray3,120,255,THRESH_BINARY); // |  THRESH_OTSU); //|  THRESH_OTSU); 
        // imshow("gray3",gray3);
        bitwise_and(gray1,gray2,obs);
        // imshow("obs",obs);
        Canny(gray3,edge,80,240);
        // imshow("edge",edge);
        for(int i=0;i < obs.rows;i++)
        {
            for(int j=0;j<obs.cols;j++)
            {
                if(obs.at<uchar>(i,j) == 255)
                    circle(obs, Point2d(j,i), 2,200, -1, LINE_AA);   
                if(edge.at<uchar>(i,j) == 255)
                    circle(edge, Point2d(j,i), 2,200, -1, LINE_AA);   //  4     4      6
            }
        }  
        
        // imshow("obsed",obs);
        // imshow("edged",edge);
        threshold(obs,obs,20,255,THRESH_BINARY_INV);// |  THRESH_OTSU); //|  THRESH_OTSU); 
        // imshow("obseded",obs);
        frontiers.copySize(edge);
        bitwise_and(obs,edge,frontiers); 
        // imshow("frontiers",frontiers);
        dstandcenters frontiers_centers;
        frontiers_centers.centers.clear();
        frontiers_centers = getUnkownCenters(frontiers); 

        frontiers_points.points.clear();
        frontiers_vis.points.clear();
        for(int k=0; k<frontiers_centers.centers.size(); k++)
        {
            frontiers_point.x = (frontiers_centers.centers.at(k).x + (10*re_xy[0])) / 10;
            frontiers_point.y = (frontiers_centers.centers.at(k).y + (10*re_xy[1])) / 10;
            frontiers_points.points.push_back(frontiers_point);
            frontiers_vis.points.push_back(frontiers_point);
        }

        frontiers_pub.publish(frontiers_points);
        vis_pub.publish(frontiers_vis);
        // imshow("fc", frontiers_centers.image);     
        waitKey(30);
        ros::spinOnce();
        rate.sleep();
    }
        return 0;
}