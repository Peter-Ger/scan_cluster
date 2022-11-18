/*
	DBSCAN Algorithm
	15S103182
	Ethan
*/
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <stack>
#define PI 3.1415926
using namespace std;



class point{
public:
	float x;
	float y;
	int cluster=0;
	int pointType=1;//1 noise 2 border 3 core
	int pts=0;//points in MinPts 
	vector<int> corepts;
	int visited = 0;
	point (){}
	point (float a,float b,int c){
		x = a;
		y = b;
		cluster = c;
	}
};


float squareDistance(point a,point b)
{
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

class SubscribeAndPublish
{
  private:
  ros::NodeHandle n; 
  ros::Publisher lidar_pub= n.advertise<visualization_msgs::MarkerArray>("objects", 2);
  ros::Publisher point_pub= n.advertise<visualization_msgs::MarkerArray>("points", 2);
  ros::Subscriber lidar_sub=n.subscribe("/scan",2,&SubscribeAndPublish::lidarcallback,this);
  float Eps=0.1;
  int MinPts=7;
  public:
    SubscribeAndPublish(){}
    void lidarcallback(const sensor_msgs::LaserScan msg)
    {
     vector<float> ranges = msg.ranges;
     float maxdistance=10;
     float maxtheta=PI/2*3;
     float mintheta=PI/2;
     vector<vector<float>> c;
     //转换到二维XY平面坐标系下;
     //cout << ranges.size() << endl;
     for(int i=0; i< ranges.size(); i++)
     {
       vector<float> a;
       double angle = msg.angle_min + i * msg.angle_increment;
       //if(ranges[i] <= maxdistance && angle >= mintheta && angle <= maxtheta)
        if(ranges[i] <= maxdistance)
        {
         float X = ranges[i] * cos(angle);
         float Y = ranges[i] * sin(angle);
         if (X < -0.5 || X > 5)
            continue;
         if (Y > 1 || Y < -1)
            continue;
         a.push_back(X);
         a.push_back(Y);
         c.push_back(a);
       }
     }
   
    visualization_msgs::MarkerArray points;
	for (int i=0; i<c.size(); ++i)
	{
	    visualization_msgs::Marker pt;
	
	    pt.header.frame_id = "laser_link";
        pt.header.stamp = ros::Time::now();
        pt.id = i;
        pt.ns = "points";
        pt.type = visualization_msgs::Marker::CYLINDER;
        pt.action = visualization_msgs::Marker::ADD;
        
	    pt.pose.position.x = c[i][0];
        pt.pose.position.y = c[i][1];
        pt.pose.position.z = 0;

        pt.scale.x = 0.05;
        pt.scale.y = 0.05;
        pt.scale.z = 0.2;
        
        pt.color.r = 0.0f;
        pt.color.g = 1.0f;
        pt.color.b = 0.0f;
        pt.color.a = 0.85;
        
        pt.lifetime = ros::Duration(0.1);
        points.markers.push_back(pt);
	}
	//cout << points.markers.size() << endl;
	point_pub.publish(points);
	
	
    vector<point> dataset;
    for(int i=0;i<c.size();i++)
    {
        point a;
        a.x = c[i][0];
        a.y = c[i][1];
        a.cluster = i;
        dataset.push_back(a);
    }
    
    for(int i=0;i<dataset.size();i++)
	{
		for(int j=i+1;j<dataset.size();j++)
		{
			if(squareDistance(dataset[i],dataset[j])<Eps)
				{
				dataset[i].pts++;
				dataset[j].pts++;
				}
		}
	}

	vector<point> corePoint;
	for(int i=0;i<dataset.size();i++)
	{
		if(dataset[i].pts>=MinPts)
		 {
			dataset[i].pointType = 3;
			corePoint.push_back(dataset[i]);
		}
	}

	for(int i=0;i<corePoint.size();i++)
	{
		for(int j=i+1;j<corePoint.size();j++)
		{
			if(squareDistance(corePoint[i],corePoint[j])<Eps)
			{
				corePoint[i].corepts.push_back(j);
				corePoint[j].corepts.push_back(i);
			}
		}
	}
	
	for(int i=0;i<corePoint.size();i++)
	{
		stack<point*> ps;
		if(corePoint[i].visited == 1) continue;
		ps.push(&corePoint[i]);
		point *v;
		while(!ps.empty())
		{
			v = ps.top();
			v->visited = 1;
			ps.pop();
			for(int j=0;j<v->corepts.size();j++)
			{
				if(corePoint[v->corepts[j]].visited==1) continue;
				corePoint[v->corepts[j]].cluster = corePoint[i].cluster;
				corePoint[v->corepts[j]].visited = 1;
				ps.push(&corePoint[v->corepts[j]]);				
			}
		}		
	}
	
	for(int i=0;i<dataset.size();i++)
	{
		if(dataset[i].pointType==3) continue;
		for(int j=0;j<corePoint.size();j++)
		{
			if(squareDistance(dataset[i],corePoint[j])<Eps) 
			{
				dataset[i].pointType = 2;
				dataset[i].cluster = corePoint[j].cluster;
				break;
			}
		}
	}

	visualization_msgs::MarkerArray objs;
        vector<vector<point>> cluster_res;
    
    vector<point> newcluster0;
    newcluster0.push_back(corePoint[0]);
    cluster_res.push_back(newcluster0);

    for (int i=0; i<corePoint.size(); ++i)
    {
        bool have=false;
        for (int j=0; j<cluster_res.size(); ++j)
        {
            if (corePoint[i].cluster == cluster_res[j][0].cluster)
            {
              have = true;
              cluster_res[j].push_back(corePoint[i]);
              break;
            }
        }

        if (have == true) continue;
        vector<point> newcluster;
        newcluster.push_back(corePoint[i]);
        cluster_res.push_back(newcluster);
    }
    cout<<cluster_res[0].size()<<endl;
    cout << "总类数：" << cluster_res.size() << endl;

//cluster_res每一行是这一类的所有点，统计他们算中心点和半径.
	  

    for(int i=0;i<cluster_res.size();++i)
    {
	    visualization_msgs::Marker marker;
	    float corex = 0,corey = 0;
	    float totalx = 0,totaly = 0;
	    float scalex = 0,scaley = 0;
	    float r=0;
	    
	    for(int k=0;k<cluster_res[i].size();k++)
	    {
	        totalx += cluster_res[i][k].x;
	        totaly += cluster_res[i][k].y;
	    }
	    corex = totalx / cluster_res[i].size();
	    corey = totaly / cluster_res[i].size();
	    
	    for(int k=0;k<cluster_res[i].size();k++)
	    {
	      if(sqrt((cluster_res[i][k].x-corex)*(cluster_res[i][k].x-corex)+(cluster_res[i][k].y-corey)*(cluster_res[i][k].y-corey)) > r)
	        r = sqrt((cluster_res[i][k].x-corex)*(cluster_res[i][k].x-corex)+(cluster_res[i][k].y-corey)*(cluster_res[i][k].y-corey));
	    }
	    cout << "position: " << corex << "," << corey << "   r:  " << r << endl;
	    marker.header.frame_id = "laser_link";
        marker.header.stamp = ros::Time::now();
	    marker.ns = "obstacle";
	    marker.id = i;
	    marker.type = visualization_msgs::Marker::CYLINDER;
	    marker.action = visualization_msgs::Marker::ADD;
	    
	    marker.pose.position.x = corex;
        marker.pose.position.y = corey;
        marker.pose.position.z = 0.0;
        
        scalex = r;
        scaley = r;
        
        marker.scale.x = scalex;
        marker.scale.y = scaley;
        marker.scale.z = 0.2;
        
        if (corey < 0)
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.85;
            marker.text = "red";
        }
        else
        {
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 0.85;
            marker.text = "blue";
        }
        
        
        marker.lifetime = ros::Duration(0.1);
        objs.markers.push_back(marker);
	
}

	lidar_pub.publish(objs);
    } 
};


int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "lidar_node");
    SubscribeAndPublish object;
    
    ros::spin();
    return 0;
}





