/**
 * @file get_voronoi_centroid.cpp
 * @author qingchen Bi
 * @brief 
 * @version 0.1
 * @date 2021-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "cure_planner/PointArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PointStamped.h"
#include <vector>
#include <iostream>
#include <list>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rosvoronoi.h"
#include "cstdlib"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Int8.h"
#include "robots.hpp"
#include "visual.h"
#include <chrono>

using namespace std;
using namespace cv;

int n_robot;
string map_topic;
nav_msgs::OccupancyGrid map_data;
cure_planner::PointArray  robot_positions;
cure_planner::PointArray  center_points;
geometry_msgs::Point ce_point;
vector<Point2f> base_points;
int flag_time = 1;
clock_t dist_time;
int flag1 = 0,flag2 =0;
float re_xy[2];
visualization_msgs::Marker centroids_vis;
visualization_msgs::Marker lew_vis1,lew_vis2,lew_vis3,gew; // only vis three lew
visualization_msgs::Marker voronoi_vis;
cure_planner::PointArray vo_world_points;
int flag = 0;
geometry_msgs::Point robot_center;
vector<Point2d> vo_points;

void doneCallBack(const std_msgs::Int8& flagd)
{
    flag = flagd.data;
}
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_data=*msg;
    re_xy[0] = msg->info.origin.position.x;
    re_xy[1] = msg->info.origin.position.y;
}
void robotPositionCallBack(const cure_planner::PointArray::ConstPtr &msg)
{
    vector<geometry_msgs::Point>().swap(robot_positions.points);
    int  length =msg->points.size();
    geometry_msgs::Point point1;
    for(int i=0; i< length; i++)
    {
        point1.x = msg->points[i].x;
        point1.y = msg->points[i].y;
        robot_positions.points.push_back(point1);
    }    
    flag1 = 1;
} 
int main(int argc, char** argv)
{
    ros::init(argc, argv,"get_voronoi_centroid");
    ros::NodeHandle nh;
    std::string ns;
    ns=ros::this_node::getName();
    ros::param::param<int>(ns + "/n_robot", n_robot, 1);
    ros::param::param<string>(ns + "/map_topic", map_topic, "map");
    ros::Subscriber map_sub = nh.subscribe(map_topic, 1000, mapCallBack);
    ros::Subscriber lsd_sub = nh.subscribe("/robots_positions", 1000, robotPositionCallBack);
    ros::Publisher centers_pub = nh.advertise<cure_planner::PointArray>("/centroids", 10);
    ros::Publisher centers_vis_pub = nh.advertise<visualization_msgs::Marker>("/centroid_visual", 10);
    ros::Publisher lew1_pub = nh.advertise<visualization_msgs::Marker>("/lew1", 10);
    ros::Publisher lew2_pub = nh.advertise<visualization_msgs::Marker>("/lew2", 10);
    ros::Publisher lew3_pub = nh.advertise<visualization_msgs::Marker>("/lew3", 10);
    ros::Publisher gew_pub = nh.advertise<visualization_msgs::Marker>("/gew", 10);
    ros::Publisher voronoi_vis_pub = nh.advertise<visualization_msgs::Marker>("/vo_edge", 10);
    ros::Subscriber Done_sub = nh.subscribe("/Done", 10, doneCallBack);

    ros::Rate rate(10);
    while (map_data.header.seq<1 or map_data.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}
    
    centroids_vis.header.frame_id = map_data.header.frame_id;
    centroids_vis.header.stamp = ros::Time(0);
    centroids_vis.action = centroids_vis.ADD;
    centroids_vis.id = 0;
    centroids_vis.color.r = 0;
    centroids_vis.color.g = 0;
    centroids_vis.color.b = 255.0/255.0;
    centroids_vis.lifetime = ros::Duration();
    centroids_vis.ns = "markers";
    centroids_vis.scale.x = 0.2;
    centroids_vis.scale.y = 0.2;
    centroids_vis.type = centroids_vis.POINTS;
    centroids_vis.pose.orientation.w = 1.0;
    centroids_vis.color.a = 1.0;
    
    while(ros::ok())
    {
        auto decision_begin_time = std::chrono::system_clock::now();
        Mat tmpdata = getRosImage(map_data);
        Mat img_temp; 
        tmpdata.copyTo(img_temp);

        if(flag2 == 0){
            base_points.clear();
            for(int i = 0; i < n_robot; i++)
            {
                base_points.push_back(Point2f( tmpdata.cols/3+i, tmpdata.rows/3+i));
            }
        }
        if(flag1 == 1){
            base_points.clear();
            for(int j = 0; j< n_robot; j++)
            {
                base_points.push_back(Point2f((10*robot_positions.points[j].x)-(10*re_xy[0]),
                                            (10*robot_positions.points[j].y)-(10*re_xy[1])));
            }
            flag2 = 1;
            flag1 = 0;
        }

        robot_center.x = 0;
        robot_center.y = 0;
        for(int m = 0; m < base_points.size(); m++)
        {
            robot_center.x = robot_center.x + (base_points[m].x / base_points.size());
            robot_center.y = robot_center.y + (base_points[m].y / base_points.size());
        }

        vector<int> lminx, lminy,lmaxx,lmaxy;
        int detaR = 0;  //80   120 
        lminx.clear();
        lminy.clear();
        lmaxx.clear();
        lmaxy.clear();
        for(int m = 0; m < base_points.size(); m++ )
        {
            detaR = max(abs(base_points[m].x - robot_center.x) , abs(base_points[m].y - robot_center.y));
            while(detaR < 200)
            {
                int lx,ly,rx,ry;
                int H,W;
                detaR = detaR + 10;
                if(base_points[m].x - detaR > 0)
                    lx = base_points[m].x - detaR;
                else
                    lx = 0;
                if(base_points[m].y - detaR > 0)
                    ly = base_points[m].y - detaR;
                else
                    ly = 0;
                if(base_points[m].x + detaR > img_temp.cols)
                    rx = img_temp.cols;
                else
                    rx = base_points[m].x + detaR;
                if(base_points[m].y + detaR > img_temp.rows)
                    ry = img_temp.rows;
                else 
                    ry = base_points[m].y + detaR;

                int count = 0;
                //up
                int up1x = lx;
                int up1y = ry;
                int up5x = rx;
                int up5y = ry;
                int up3x = (int)((lx+rx)/2); 
                int up3y = ry;
                int up2x = (int)((up1x + up3x)/2);
                int up2y = ry;
                int up4x = (int)((up3x + up5x)/2);
                int up4y = ry;
                if(img_temp.at<uchar>(up1y, up1x) == 255 && 
                    img_temp.at<uchar>(up2y, up2x) == 255 &&
                    img_temp.at<uchar>(up3y, up3x) == 255 &&
                    img_temp.at<uchar>(up4y, up4x) == 255 &&
                    img_temp.at<uchar>(up5y, up5x) == 255)
                {
                    count++; 
                    if(count >= 1)
                        break;
                }
                //down
                int do1x = lx;
                int do1y = ly;
                int do5x = rx;
                int do5y = ly;
                int do3x = (int)((lx+rx)/2); 
                int do3y = ly;
                int do2x = (int)((do1x + do3x)/2);
                int do2y = ly;
                int do4x = (int)((do3x + do5x)/2);
                int do4y = ly;
                if(img_temp.at<uchar>(do1y, do1x) == 255 && 
                    img_temp.at<uchar>(do2y, do2x) == 255 &&
                    img_temp.at<uchar>(do3y, do3x) == 255 &&
                    img_temp.at<uchar>(do4y, do4x) == 255 &&
                    img_temp.at<uchar>(do5y, do5x) == 255)
                {
                    count++; 
                    if(count >= 1)
                        break;
                }
                //left
                int le1x = lx;
                int le1y = ly;
                int le5x = lx;
                int le5y = ry;
                int le3x = lx; 
                int le3y = (int)((le1y+le5y) / 2);
                int le2x = lx;
                int le2y = (int)((le1y+le3y) / 2);
                int le4x = lx;
                int le4y = (int)((le3y+le5y) / 2);
                if(img_temp.at<uchar>(le1y, le1x) == 255 && 
                    img_temp.at<uchar>(le2y, le2x) == 255 &&
                    img_temp.at<uchar>(le3y, le3x) == 255 &&
                    img_temp.at<uchar>(le4y, le4x) == 255 &&
                    img_temp.at<uchar>(le5y, le5x) == 255)
                {
                    count++; 
                    if(count >= 1)
                        break;
                }
                //right
                int ri1x = rx;
                int ri1y = ly;
                int ri5x = rx;
                int ri5y = ry;
                int ri3x = rx; 
                int ri3y = le3y;
                int ri2x = rx;
                int ri2y = le2y;
                int ri4x = rx;
                int ri4y = le4y;
                if(img_temp.at<uchar>(ri1y, ri1x) == 255 && 
                    img_temp.at<uchar>(ri2y, ri2x) == 255 &&
                    img_temp.at<uchar>(ri3y, ri3x) == 255 &&
                    img_temp.at<uchar>(ri4y, ri4x) == 255 &&
                    img_temp.at<uchar>(ri5y, ri5x) == 255)
                {
                    count++; 
                    if(count >= 1)
                        break;
                }
            }

            if(flag == 1)
                detaR = detaR - 30;

            if(base_points[m].x - detaR > 0)
                lminx.push_back(base_points[m].x - detaR);
            else 
                lminx.push_back(0 );
            if(base_points[m].y - detaR > 0)
                lminy.push_back(base_points[m].y - detaR);
            else  
                lminy.push_back(0);
            if(base_points[m].x + detaR > img_temp.cols)
                lmaxx.push_back( img_temp.cols); 
            else 
                lmaxx.push_back(base_points[m].x + detaR);
            if(base_points[m].y + detaR > img_temp.rows)
                lmaxy.push_back( img_temp.rows ); 
            else 
                lmaxy.push_back(base_points[m].y + detaR);
        }
        vector<Rect> lews;
        lews.clear();
        for(int k = 0; k < n_robot; k++)
        {
            Rect lew_tmp(lminx[k], lminy[k], lmaxx[k]- lminx[k], lmaxy[k]- lminy[k]);
            lews.push_back(lew_tmp);
        }

        // TODO only vis three lews
        cure_planner::PointArray lew1ps, lew2ps, lew3ps;
        geometry_msgs::Point lewp0, lewp1;
        for(int rk = 0; rk < n_robot; rk++)
        {
            //left
            lewp0.x = (double)(lminx[rk] * map_data.info.resolution) + re_xy[0];
            lewp0.y = (double)(lminy[rk] * map_data.info.resolution) + re_xy[1];
            lewp1.x = (double)(lminx[rk] * map_data.info.resolution) + re_xy[0];
            lewp1.y = (double)(lmaxy[rk] * map_data.info.resolution) + re_xy[1];
            if(rk % 3 == 0)
            {
                lew1ps.points.push_back(lewp0);
                lew1ps.points.push_back(lewp1);
            }
            if(rk % 3 == 1)
            {
                lew2ps.points.push_back(lewp0);
                lew2ps.points.push_back(lewp1);
            }
            if(rk % 3 == 2)
            {
                lew3ps.points.push_back(lewp0);
                lew3ps.points.push_back(lewp1);
            }
            //down
            lewp0.x = (double)(lmaxx[rk] * map_data.info.resolution) + re_xy[0];
            lewp0.y = (double)(lminy[rk] * map_data.info.resolution) + re_xy[1];
            lewp1.x = (double)(lminx[rk] * map_data.info.resolution) + re_xy[0];
            lewp1.y = (double)(lminy[rk] * map_data.info.resolution) + re_xy[1];
            if(rk == 0)
            {
                lew1ps.points.push_back(lewp0);
                lew1ps.points.push_back(lewp1);
            }
            if(rk == 1)
            {
                lew2ps.points.push_back(lewp0);
                lew2ps.points.push_back(lewp1);
            }
            if(rk == 2)
            {
                lew3ps.points.push_back(lewp0);
                lew3ps.points.push_back(lewp1);
            }
            //up
            lewp0.x = (double)(lmaxx[rk] * map_data.info.resolution) + re_xy[0];
            lewp0.y = (double)(lmaxy[rk] * map_data.info.resolution) + re_xy[1];
            lewp1.x = (double)(lminx[rk] * map_data.info.resolution) + re_xy[0];
            lewp1.y = (double)(lmaxy[rk] * map_data.info.resolution) + re_xy[1];
            if(rk == 0)
            {
                lew1ps.points.push_back(lewp0);
                lew1ps.points.push_back(lewp1);
            }
            if(rk == 1)
            {
                lew2ps.points.push_back(lewp0);
                lew2ps.points.push_back(lewp1);
            }
            if(rk == 2)
            {
                lew3ps.points.push_back(lewp0);
                lew3ps.points.push_back(lewp1);
            }
            //right
            lewp0.x = (double)(lmaxx[rk] * map_data.info.resolution) + re_xy[0];
            lewp0.y = (double)(lmaxy[rk] * map_data.info.resolution) + re_xy[1];
            lewp1.x = (double)(lmaxx[rk] * map_data.info.resolution) + re_xy[0];
            lewp1.y = (double)(lminy[rk] * map_data.info.resolution) + re_xy[1];
            if(rk == 0)
            {
                lew1ps.points.push_back(lewp0);
                lew1ps.points.push_back(lewp1);
            }
            if(rk == 1)
            {
                lew2ps.points.push_back(lewp0);
                lew2ps.points.push_back(lewp1);
            }
            if(rk == 2)
            {
                lew3ps.points.push_back(lewp0);
                lew3ps.points.push_back(lewp1);
            }
        }
        lew_vis1 = Visual_line_list(map_data.header.frame_id, 3, lew1ps, 1.0, 0.0, 0.0);
        lew_vis2 = Visual_line_list(map_data.header.frame_id, 3, lew2ps, 0.0, 0.0, 1.0);
        lew_vis3 = Visual_line_list(map_data.header.frame_id, 3, lew3ps, 0.0, 1.0, 0.0);

        int maxydeta =40; //40
        int minydeta =40;
        int maxxdeta =40;
        int minxdeta =40;
        vector<int> lsdx, lsdy;
        lsdx.clear();
        lsdy.clear();
        for(int m=0; m < base_points.size(); m++ )
        {
            lsdx.push_back(base_points[m].x);
            lsdy.push_back(base_points[m].y);
        }
        auto maxx = max_element(lsdx.begin(), lsdx.end());
        auto maxy = max_element(lsdy.begin(), lsdy.end());
        auto minx = min_element(lsdx.begin(), lsdx.end());
        auto miny = min_element(lsdy.begin(), lsdy.end());
        if(*minx - minxdeta < 0) 
            *minx = 0;
        else 
            *minx = *minx - minxdeta;
        if(*miny - minydeta < 0)
            *miny = 0;
        else 
            *miny = *miny - minydeta;
        if(*maxx+maxxdeta >img_temp.cols )
            *maxx  = img_temp.cols - *minx;
        else 
            *maxx = *maxx + maxxdeta - *minx;
        if(*maxy+maxydeta >img_temp.rows )
            *maxy  = img_temp.rows - *miny;
        else 
            *maxy = *maxy + maxydeta - *miny;
        // vis GEW
        Rect  gew_roi(*minx,*miny,*maxx,*maxy);
        cure_planner::PointArray gewps;
        geometry_msgs::Point gewp0, gewp1;
        gewp0.x = (double)((*minx) * map_data.info.resolution) + re_xy[0];
        gewp0.y = (double)((*miny) * map_data.info.resolution) + re_xy[1];
        gewp1.x = (double)((*minx) * map_data.info.resolution) + re_xy[0];
        gewp1.y = (double)((*maxy + *miny) * map_data.info.resolution) + re_xy[1];
        gewps.points.push_back(gewp0);
        gewps.points.push_back(gewp1);
        gewp0.x = (double)((*maxx + *minx) * map_data.info.resolution) + re_xy[0];
        gewp0.y = (double)((*miny) * map_data.info.resolution) + re_xy[1];
        gewp1.x = (double)((*minx) * map_data.info.resolution) + re_xy[0];
        gewp1.y = (double)((*miny) *map_data.info.resolution) + re_xy[1];
        gewps.points.push_back(gewp0);
        gewps.points.push_back(gewp1);
        gewp0.x = (double)((*maxx + *minx) * map_data.info.resolution) + re_xy[0];
        gewp0.y = (double)((*maxy + *miny) * map_data.info.resolution) + re_xy[1];
        gewp1.x = (double)((*minx) * map_data.info.resolution) + re_xy[0];
        gewp1.y = (double)((*maxy + *miny) * map_data.info.resolution) + re_xy[1];
        gewps.points.push_back(gewp0);
        gewps.points.push_back(gewp1);
        gewp0.x = (double)((*maxx + *minx) * map_data.info.resolution) + re_xy[0];
        gewp0.y = (double)((*maxy + *miny) * map_data.info.resolution) + re_xy[1];
        gewp1.x = (double)((*maxx + *minx) * map_data.info.resolution) + re_xy[0];
        gewp1.y = (double)((*miny) * map_data.info.resolution) + re_xy[1];
        gewps.points.push_back(gewp0);
        gewps.points.push_back(gewp1);
        gew = Visual_line_list(map_data.header.frame_id, 3, gewps, 1.0, 1.0, 0.0);

        vo_points.clear();
        img_temp= getVoronoi(img_temp, base_points, vo_points);
        vo_world_points.points.clear();
        for(int i = 0; i < vo_points.size(); )
        {
            geometry_msgs::Point VoWorldpoint;
            VoWorldpoint.x = vo_points[i].x * map_data.info.resolution + re_xy[0];
            VoWorldpoint.y = vo_points[i].y * map_data.info.resolution + re_xy[1];
            vo_world_points.points.push_back(VoWorldpoint);
            i = i + 2;
        }
        voronoi_vis = Visual_point(map_data.header.frame_id, 4, 1.0, 1.0, 0.0, vo_world_points, 0.12);
        voronoi_vis_pub.publish(voronoi_vis);

        Mat img_temp_gew(img_temp,gew_roi);
        // imshow("img_temp_gew", img_temp_gew);

        vector<Mat> img_temp_lews;
        img_temp_lews.clear();
        for(int l = 0; l < n_robot; l++)
        {
            Mat img_temp_lew(img_temp, lews[l]);
            // imshow("local_" + to_string(l), img_temp_lew);
            img_temp_lews.push_back(img_temp_lew);
        }

        dstandcenters dstandcenters_gew;
        dstandcenters_gew.centers.clear();
        dstandcenters_gew = getUnkownCenters(img_temp_gew);

        vector<dstandcenters> dstandcenters_lews;
        dstandcenters_lews.clear();
        for(int n = 0; n < n_robot; n++)
        {
            dstandcenters dstandcenters_lew;
            dstandcenters_lew.centers.clear();
            dstandcenters_lew = getUnkownCenters(img_temp_lews[n]);    
            dstandcenters_lews.push_back(dstandcenters_lew);
        }

        // imshow("img_temp", img_temp);
        // flip(img_temp_lews[0], img_temp_lews[0], 0);
        // imshow("Robot_1 LEW ", img_temp_lews[0]);
        // flip(dstandcenters_lews[0].image, dstandcenters_lews[0].image, 0);
        // imshow("Robot_1 LEW Contours", dstandcenters_lews[0].image);

        center_points.points.clear();
        centroids_vis.points.clear();
        for(int k =0; k < dstandcenters_gew.centers.size(); k++ )
        {
            ce_point.x = (dstandcenters_gew.centers.at(k).x + *minx+  (10*re_xy[0])) / 10;
            ce_point.y = (dstandcenters_gew.centers.at(k).y + *miny +  (10*re_xy[1])) / 10;
            array<float, 2> point;
            point[0] = ce_point.x;
            point[1] = ce_point.y;
            if(flag == 1)
            {
                if(isVaild(map_data, point, 7))
                {
                    center_points.points.push_back(ce_point);
                    centroids_vis.points.push_back(ce_point);
                }            
            }
            else
            {
                if(isVaild(map_data, point, 0.3) && !isInvaildCen(map_data, point, 0.3, 3.0))
                {
                    center_points.points.push_back(ce_point);
                    centroids_vis.points.push_back(ce_point);
                }
            }
        }
        for(int h = 0; h < n_robot; h++)
        {
            for(int k =0; k<dstandcenters_lews[h].centers.size(); k++ )
            {
                ce_point.x = (dstandcenters_lews[h].centers.at(k).x +lminx[h] +  (10*re_xy[0])) / 10;
                ce_point.y = (dstandcenters_lews[h].centers.at(k).y +lminy[h] +  (10*re_xy[1])) / 10;
                array<float, 2> point;
                point[0] = ce_point.x;
                point[1] = ce_point.y;
                if(flag == 1)
                {
                    if(isVaild(map_data, point, 7))
                    {
                        center_points.points.push_back(ce_point);
                        centroids_vis.points.push_back(ce_point);
                    }  
                }
                else
                {
                    if(isVaild(map_data, point, 0.3) && !isInvaildCen(map_data, point, 0.3, 3.0))
                    {
                        center_points.points.push_back(ce_point);
                        centroids_vis.points.push_back(ce_point);
                    }
                }
            }
        }
        centers_pub.publish(center_points);
        centers_vis_pub.publish(centroids_vis);

        lew1_pub.publish(lew_vis1);
        lew2_pub.publish(lew_vis2);
        lew3_pub.publish(lew_vis3);
        gew_pub.publish(gew);
        // flip(img_temp, img_temp, 0);
        // imshow("Ros2Opencv", img_temp);
        waitKey(30);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
 }
