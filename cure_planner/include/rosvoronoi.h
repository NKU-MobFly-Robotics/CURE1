/**
 * @file rosvoronoi.h
 * @author qingchen Bi
 * @brief 
 * @version 0.1
 * @date 2021-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <math.h>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

int sq2(int x, int y)
{
    return pow(x,2) + pow(y,2);
}
Mat getRosImage( nav_msgs::OccupancyGrid mapData)
{
    int H,W;
    W = mapData.info.width;
    H = mapData.info.height;
    Mat tmpdata = Mat(Size(W, H), CV_8UC1);
    for(int h = 0; h< H; h++)
    {
        for(int w = 0; w < W; w++)
        {
            if(mapData.data[h*W + w] == -1) 
            {
                tmpdata.at<uchar>(h,w) = 255;
            }
            if(mapData.data[h*W + w] == 0)
            {
                tmpdata.at<uchar>(h,w) = 0;
            }
            else if(mapData.data[h*W + w] != -1 && mapData.data[h*W + w] != 0)
            {
                circle(tmpdata, Point2d(w,h), 1, 100, -1, LINE_AA);
            }
        }
    }
    return tmpdata;
}
Mat getVoronoi( Mat & img, vector<Point2f> points, vector<Point2d> & Vopoints)
{
    int dist,d;
    int ret;
    int data_vo[img.rows][img.cols];
    for(int i = 0; i< img.rows; i++)
    {       
        for(int j =0; j< img.cols; j++)
        {  
            dist = 0;
            for(int k = 0; k< points.size(); k++)
            {
                    d = sq2((j - (int)points[k].x), (i - (int)points[k].y));
                    if( k == 0|| dist > d)
                    {
                        dist = d;
                        ret = k;
                    }
            }
            data_vo[i][j] =ret;
        }
    }   
    for(int i =1; i< img.rows - 1;i++)
    {
        for(int j =1; j< img.cols - 1; j++)
        {
            if(data_vo[i][j] != data_vo[i - 1][j]) 
            {
                circle(img, Point2d(j,i), 1, Scalar(0, 0, 0), -1, LINE_AA);
                Vopoints.push_back(Point2d(j,i));
                continue;
            }
            else  if(data_vo[i][j] != data_vo[i][j-1]) 
            {
                circle(img, Point2d(j,i), 1, Scalar(0, 0, 0), -1, LINE_AA);
                Vopoints.push_back(Point2d(j,i));
                continue;
            }
        }
    }
    return img;
}
struct dstandcenters
{
    Mat image;
    vector<Point2d> centers;
};
dstandcenters getUnkownCenters(Mat &image)
{
    vector<vector<Point> > contours; 
    vector<Point2d> centers;
    vector<vector<Point> >::iterator itr; 
    vector<Point2d>::iterator itrc; 
    vector<vector<Point> > con; 
    dstandcenters dstandcenters_res;
    dstandcenters_res.centers.clear();
    double area;
    double minarea = 80; 
    double maxarea = 0;
    Moments mom;
    Mat gray,dst;
    threshold(image,gray,150,255,THRESH_BINARY);// |  THRESH_OTSU); 
    findContours( gray, contours, 
    RETR_EXTERNAL, CHAIN_APPROX_SIMPLE ); 

    itr = contours.begin();
    while(itr != contours.end())
    {
        area = contourArea(*itr);
        if(area < minarea)
        {
            itr = contours.erase(itr); 
        }
        else
        {
            itr++;
        }
        if (area > maxarea)
        {
            maxarea = area;
        }
    }

    dst = Mat::zeros(image.rows,image.cols,CV_8UC3);
    Point2d center;
    itr = contours.begin();
    while(itr != contours.end())
    {
        area = contourArea(*itr);
        con.push_back(*itr);
        if(area == maxarea)
        drawContours(dst,con,-1,Scalar(0,0,255),2); 
        else
        drawContours(dst,con,-1,Scalar(0,0,255),2);
        con.pop_back();
        mom = moments(*itr);
        center.x = (int)(mom.m10 / mom.m00);
        center.y = (int)(mom.m01 / mom.m00);
        centers.push_back(center);
        dstandcenters_res.centers.push_back(center);
        circle(dst, center, 5, Scalar(0, 255, 0), 1, LINE_AA);
        itr++;
    }
    dstandcenters_res.image = dst;
    return dstandcenters_res;
}

