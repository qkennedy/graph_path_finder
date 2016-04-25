#include <ros/ros.h> 
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <string.h>
#include <stdio.h>  
#include <math.h>
#include <random>
#include <graph_path_finder/GNode.h>//include .h file for GNode and Graph?
#include <graph_path_finder/Graph.h>
#include <vector>
geometry_msgs::Pose g_mobot_pose; //this is the pose of the robot in the world, according to Gazebo
geometry_msgs::Pose g_noisy_mobot_pose; //added noise to x,y, and suppress orientation
geometry_msgs::Quaternion g_quat;
ros::Publisher g_pose_publisher; 
ros::Publisher g_gps_publisher; 
std::normal_distribution<double> distribution(0.0,1.0); //args: mean, std_dev
//TODO: Finish implementing Djikstra's
//TODO: Switch from double x, y to Pose 
//TODO  Create method pathfromvec that converts from the vec of indexes to a path of poses
vec<node> g_nodes;
struct node{
	double x;
	double y;
	double cost;
	std::vec<int> to;
	int bestFrom;
}

void graph_CB(const graph_path_finder_msgs::Graph& graph) 
{ 
  int num_nodes = graph.nodes.size;
  g_nodes.clear();
  g_nodes.resize(num_nodes);
  for (int i=0;i<num_nodes;i++) {
    node tmp=new node;
    tmp.x=0;
    tmp.y=0;
    tmp.bestFrom=-1;
    tmp.cost=999999999;
    tmp.to.resize(graph.nodes[i].goes_to.size());
    for(int j=0;j<graph.nodes[i].goes_to.size();j++){
    		tmp.to[i]=graph.nodes[i].goes_to[i];
    	}
    	g_nodes[i]=tmp;
    }
}

int find_nearest_node(nav_msgs::Point point){
	int minNode=-1;
 	double mindist=9999999999;
 	double x=point.x;
  	double y=point.y;
  	for(int i=0; i<g_nodes.size(); i++){
  		double currdist=sqrt(pow(x-g_nodes[i].x,2)+pow(y-g_nodes[i].y,2));
  		if(currdist<mindist){
  			minNode=i;
  		}

  	}
  	return minNode;
}

std::vec<int> solve(int start, int goal){
	clear_visited();
	std::vec<nodes> visited(g_nodes.size());
	std::vec<double> costs(g_nodes.size());
	costs[start]=0;
	costs.fill(9999999999999999);
	int index=start;
	node curr=new node;
	while(!empty(g_nodes){
		curr=&g_nodes[index];
		for(int i=0; i<curr.to.size();i++){
			curr.to[i].cost=cost(index,curr.to[i]);
		}
	}
}
double cost(int from, int to){
	node toNode= g_nodes[to];
	node fromNode=g_nodes[from];
	double deltx=toNode.x-fromNode.x;
	double delty=toNode.y-fromNode.y;
	return sqrt((pow(deltx,2))+pow(delty,2));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "graph_solver");
    ros::NodeHandle nh;

    g_path_publisher= nh.advertise<nav_msgs::Path>("/graph_path", 1); 
    ros::Subscriber graph_sub = nh.subscribe("/graph",1,graph_CB); 
    ros::spin();
}
