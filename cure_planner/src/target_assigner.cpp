/**
 * @file target_assigner.cpp
 * @authorqingchen Bi
 * @brief 
 * @version 0.1
 * @date 2021-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ros/ros.h"
#include "cmath"
#include "nav_msgs/OccupancyGrid.h"
#include "cure_planner/PointArray.h"
#include "robots.hpp"
#include "vector"
#include "std_msgs/Int8.h"
#include "string"
#include "exception"
#include <tf/transform_listener.h>
#include "math.h"
#include "visual.h"
#include <chrono>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

nav_msgs::OccupancyGrid map_data;
cure_planner::PointArray centroids;
cure_planner::PointArray centroids_copy;
cure_planner::PointArray centroids_filtered;
cure_planner::PointArray frontiers;
cure_planner::PointArray frontiers_copy;
string map_topic;
string cen_topic;
string fron_topic;
float info_r_min;
float info_r_step;
float info_r_max;
float info_multiplier;
int n_robots;
string robot_namespace;
int namespace_init_count;
float delay_after_assignement;
int rateHz;
cure_planner::PointArray invild_points;
visualization_msgs::Marker assigned_goal_vis;

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_data = *msg;
}
void cenCallBack(const cure_planner::PointArray::ConstPtr& msg)
{
    geometry_msgs::Point temp_p;
    centroids.points.clear();
    for(int i = 0; i < msg->points.size(); i++)
    {
        temp_p.x = msg->points[i].x;
        temp_p.y = msg->points[i].y;
        centroids.points.push_back(temp_p);
    }
}
void fronCallBack(const cure_planner::PointArray::ConstPtr& msg)
{
    geometry_msgs::Point temp_p;
    frontiers.points.clear();
    for(int i = 0; i < msg->points.size(); i++)
    {
        temp_p.x = msg->points[i].x;
        temp_p.y = msg->points[i].y;
        frontiers.points.push_back(temp_p);
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_assigner");
    ros::NodeHandle nh;
    invild_points.points.clear();
    string ns = ros::this_node::getName();
    ros::param::param<std::string>(ns + "/map_topic", map_topic, "/map");
    ros::param::param<std::string>(ns + "/cen_topic", cen_topic, "/centroids");
    ros::param::param<std::string>(ns + "/fron_topic", fron_topic, "/frontiers");
    ros::param::param<float>(ns + "/info_r_min", info_r_min, 0.5);
    ros::param::param<float>(ns + "/info_r_step", info_r_step, 0.5);
    ros::param::param<float>(ns + "/info_r_max", info_r_max, 1.0);
    ros::param::param<float>(ns + "/info_multiplier", info_multiplier, 1.0);
    ros::param::param<int>(ns + "/n_robots", n_robots, 1);
    ros::param::param<string>(ns + "/robot_namespace", robot_namespace, "");
    ros::param::param<int>(ns + "/namespace_init_count", namespace_init_count, 1);
    ros::param::param<float>(ns + "/delay_after_assignement", delay_after_assignement, 0.5);
    ros::param::param<int>(ns + "/rate", rateHz, 100);

    ros::Subscriber map_sub = nh.subscribe(map_topic, 1000, mapCallBack);
    ros::Subscriber cen_sub = nh.subscribe(cen_topic, 1000, cenCallBack);
    ros::Subscriber fronp_sub = nh.subscribe(fron_topic, 1000, fronCallBack);
    ros::Publisher assigned_vis_pub = nh.advertise<visualization_msgs::Marker>("/target_assigned_vis", 10);

    ros::Rate rate(rateHz);
    
    while(centroids.points.size() < 1){ 
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    while(map_data.data.size() < 1 or map_data.header.seq < 1){
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    tf::TransformListener listener;

    vector<Robot> robots;
    robots.clear();
    if(robot_namespace.size() > 0)
    {
        for(int i = 0; i < n_robots; i++)
        {
            Robot robot_temp(robot_namespace + to_string(i + namespace_init_count),listener);
            robots.push_back(robot_temp);
        }
    }
    else if(robot_namespace.size() == 0)
    {
        Robot robot_temp(robot_namespace ,listener);
        robots.push_back(robot_temp);
    }

    for(int i =0; i < n_robots; i++)
    {     
        string temp_name = robot_namespace;    
        string temp_name2 = robot_namespace;
        MoveBaseClient *actmp = new MoveBaseClient{temp_name.erase(0,1) + to_string(i + namespace_init_count)+ "/move_base", true};
        robots[i].goal_.target_pose.pose.position.x = robots[i].getPosition(listener).x;
        robots[i].goal_.target_pose.pose.position.y = robots[i].getPosition(listener).y;
        robots[i].goal_.target_pose.pose.orientation.w = 1.0;
        // robots[i].goal_.target_pose.header.frame_id = temp_name2.erase(0,1) + to_string(i + namespace_init_count)+ "/map";
        robots[i].goal_.target_pose.header.frame_id = "map";
        actmp->waitForServer();
        actmp->sendGoal(robots[i].goal_);
    }

    while(ros::ok())
    {
        for(int ij = 0; ij < n_robots; ij++)
        {
            robots[ij].cen_.points.clear();
        }

        centroids_copy.points.clear();
        centroids_filtered.points.clear();
        centroids_copy = centroids;
        if(centroids_copy.points.size() > 0)
        {
            for(int icenp = 0; icenp < centroids_copy.points.size(); icenp++)
            {
                float mincost = 10000;
                float cost1 = 0;
                int id_record = 0;
                if(invild_points.points.size() > 0)
                {
                    if(isReachable(centroids_copy.points[icenp], invild_points))
                    {
                        for(int ij = 0; ij < n_robots; ij++)
                        {  
                            cost1 =  pow((pow((robots[ij].getPosition(listener).y - centroids_copy.points[icenp].y), 2) +
                                     pow((robots[ij].getPosition(listener).x - centroids_copy.points[icenp].x), 2)), 0.5); 
                            if(mincost > cost1)
                            {
                                mincost = cost1;
                                id_record = ij;
                            }
                        }
                        // robots[id_record].cen_.points.push_back(centroids_copy.points[icenp]);  
                        // centroids_filtered.points.push_back(centroids_copy.points[icenp]);  
                        array<float, 2> pointtemp;
                        pointtemp[0] = centroids_copy.points[icenp].x;
                        pointtemp[1] = centroids_copy.points[icenp].y;
                        if(isVaild(map_data, pointtemp))
                        {
                            robots[id_record].cen_.points.push_back(centroids_copy.points[icenp]);  
                            centroids_filtered.points.push_back(centroids_copy.points[icenp]); 
                        }                   
                    }                    
                }
                else
                {
                    for(int ij = 0; ij < n_robots; ij++)
                    {  
                        cost1 =  pow((pow((robots[ij].getPosition(listener).y - centroids_copy.points[icenp].y), 2) +
                                    pow((robots[ij].getPosition(listener).x - centroids_copy.points[icenp].x), 2)), 0.5); 
                        if(mincost > cost1)
                        {
                            mincost = cost1;
                            id_record = ij;
                        }
                    }
                    // robots[id_record].cen_.points.push_back(centroids_copy.points[icenp]);    
                    // centroids_filtered.points.push_back(centroids_copy.points[icenp]);   
                    array<float, 2> pointtemp2;
                    pointtemp2[0] = centroids_copy.points[icenp].x;
                    pointtemp2[1] = centroids_copy.points[icenp].y;
                    if(isVaild(map_data, pointtemp2) )
                    {
                        robots[id_record].cen_.points.push_back(centroids_copy.points[icenp]);  
                        centroids_filtered.points.push_back(centroids_copy.points[icenp]); 
                    }                                       
                }

               
            }
        }

        for(int irrc = 0; irrc < n_robots; irrc++)
        {
            if(robots[irrc].cen_.points.size() > 4)
            {
                robots[irrc].branch_position_.x = robots[irrc].getPosition(listener).x;
                robots[irrc].branch_position_.y = robots[irrc].getPosition(listener).y;
            }
        }

        for(int irr = 0; irr < n_robots; irr++)
        {

            string temp_name = robot_namespace;    
            MoveBaseClient *ac = new MoveBaseClient{temp_name.erase(0,1) + to_string(irr + namespace_init_count)+ "/move_base", true};

            priority_queue<CandidatePoints, vector<CandidatePoints>, less<CandidatePoints> > pri_max_canPoints;
            vector<double> infomationGain;
            infomationGain.clear();
            double cost = 0;
            double information_gain = 0;
            double revenue;
            double info_r = 0;
            auto decision_begin_time = std::chrono::system_clock::now();

            for(int ip = 0; ip < robots[irr].cen_.points.size(); ip++)
            {
                array<float, 2> temp_cen;
                temp_cen[0] = robots[irr].cen_.points[ip].x;
                temp_cen[1] = robots[irr].cen_.points[ip].y;

                info_r = findInforGainR(map_data, temp_cen,info_r_min, info_r_max, info_r_step);
                infomationGain.push_back(informationGain(map_data, temp_cen, info_r));

                cost =  pow((pow((robots[irr].getPosition(listener).y - robots[irr].cen_.points[ip].y), 2) +
                            pow((robots[irr].getPosition(listener).x - robots[irr].cen_.points[ip].x), 2)), 0.5); 

                information_gain = infomationGain[ip];

                revenue=(information_gain * info_multiplier) / cost;
        
                CandidatePoints Candidate_P(revenue, robots[irr].cen_.points[ip]);
                pri_max_canPoints.push(Candidate_P);
            }

            if(!pri_max_canPoints.empty())
            {
                if(robots[irr].robot_state_)
                {
                    CandidatePoints temp = pri_max_canPoints.top();

                    float angle_last;
                    float robot_angle_last;
                    float robot_angle_new;
                    float angle_new;
                    float robot_curr_yaw = robots[irr].getAngRobot(listener);
                    if(robots[irr].last_goal_.size())
                    {
                        angle_last = atan2((robots[irr].last_goal_[1] - robots[irr].getPosition(listener).y), (robots[irr].last_goal_[0] - robots[irr].getPosition(listener).x));
                        robot_angle_last = abs(angle_last - robot_curr_yaw);
                        
                        angle_new = atan2((temp.getY() - robots[irr].getPosition(listener).y), (temp.getX() - robots[irr].getPosition(listener).x));
                        robot_angle_new = abs(angle_new - robot_curr_yaw);

                        array<float, 2> point;
                        point[0] = robots[irr].last_goal_[0];
                        point[1] = robots[irr].last_goal_[1];
                        if((robot_angle_new > robot_angle_last) && (getArroundGridValue(map_data, point, 0.5))) 
                        {
                            robots[irr].goal_.target_pose.pose.position.x = robots[irr].last_goal_[0];
                            robots[irr].goal_.target_pose.pose.position.y = robots[irr].last_goal_[1];                    
                        }
                        else
                        {
                            robots[irr].goal_.target_pose.pose.position.x = temp.getX();
                            robots[irr].goal_.target_pose.pose.position.y = temp.getY();                    
                        }
                    }
                    else
                    {
                        robots[irr].goal_.target_pose.pose.position.x = temp.getX();
                        robots[irr].goal_.target_pose.pose.position.y = temp.getY();                    
                    }

                    string temp_namespace_2 = robot_namespace;
                    // robots[irr].goal_.target_pose.header.frame_id = temp_namespace_2.erase(0,1) + to_string(irr+1) + "/map";
                    robots[irr].goal_.target_pose.header.frame_id =  "map";
                    ac->waitForServer();
                    ac->sendGoal(robots[irr].goal_);
                }
                else
                {          
                    if(frontiers.points.size())
                    {
                        float min_dis_cost = 10000;
                        float discost;
                        int idk;
                        bool valid_fron = false;
                        for(int k =0; k < frontiers.points.size(); k ++)
                        {
                            if(isvaildFrontier(map_data, frontiers.points[k]))
                            {
                                valid_fron = true;
                                discost = pow((pow((robots[irr].getPosition(listener).y - frontiers.points[k].y), 2) +
                                                pow((robots[irr].getPosition(listener).x - frontiers.points[k].x), 2)), 0.5); 
                                if(discost < min_dis_cost)
                                {
                                    min_dis_cost = discost;
                                    idk = k;
                                }                                    
                            }

                        }
                        if(valid_fron)
                        {
                            robots[irr].goal_.target_pose.pose.position.x = frontiers.points[idk].x;
                            robots[irr].goal_.target_pose.pose.position.y = frontiers.points[idk].y;
                        }
                        else
                        {
                            robots[irr].goal_.target_pose.pose.position.x = robots[irr].branch_position_.x;
                            robots[irr].goal_.target_pose.pose.position.y = robots[irr].branch_position_.y;
                        }
                    }
                    else
                    {
                        robots[irr].goal_.target_pose.pose.position.x = robots[irr].branch_position_.x;
                        robots[irr].goal_.target_pose.pose.position.y = robots[irr].branch_position_.y;
                    }
                    string temp_namespace_4 = robot_namespace;
                    // robots[irr].goal_.target_pose.header.frame_id = temp_namespace_4.erase(0,1) + to_string(irr+1) + "/map"; 
                    robots[irr].goal_.target_pose.header.frame_id = "map";

                    ac->waitForServer();
                    ac->sendGoal(robots[irr].goal_);
                }

                ros::Duration(delay_after_assignement).sleep();
            }

            array<double,2> tmp_rp;
            tmp_rp[0] = robots[irr].getPosition(listener).x;
            tmp_rp[1] = robots[irr].getPosition(listener).y;
            robots[irr].robot_pos_.push_back(tmp_rp);
            int time_2 = ros::Time::now().toSec();
            robots[irr].stop_duration_.push_back(time_2);
            int stop_dura_thro = (int)(15 / n_robots);
            if((robots[irr].stop_duration_.end() - robots[irr].stop_duration_.begin()) > stop_dura_thro)
            {
                double max_v = 0.50;
                double tra_dis = 0.5; //double(stop_dura_thro) * max_v / 2;
                if((abs(robots[irr].robot_pos_[robots[irr].robot_pos_.size() - 1][0] - robots[irr].robot_pos_[0][0]) > tra_dis || 
                    abs(robots[irr].robot_pos_[robots[irr].robot_pos_.size() - 1][1] - robots[irr].robot_pos_[0][1]) > tra_dis))
                {
                    robots[irr].robot_state_ = true;
                    robots[irr].stop_duration_.clear();
                    robots[irr].robot_pos_.clear();
                }
                else
                {       
                    invild_points.points.push_back(robots[irr].goal_.target_pose.pose.position);
                    robots[irr].robot_state_ = false;
                }
            }
            robots[irr].last_goal_.clear();
            robots[irr].last_goal_.push_back(robots[irr].goal_.target_pose.pose.position.x);
            robots[irr].last_goal_.push_back(robots[irr].goal_.target_pose.pose.position.y);

        }
        cure_planner::PointArray visps;
        visps.points.clear();
        for(int visi = 0; visi < n_robots; visi ++)
        {
            visps.points.push_back(robots[visi].goal_.target_pose.pose.position);
        }
        assigned_goal_vis = Visual_point(map_data.header.frame_id, 1, 1.0, 0.0, 0.0, visps, 0.4);
        assigned_vis_pub.publish(assigned_goal_vis);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}











