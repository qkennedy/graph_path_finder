#include <ros/ros.h> 
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <string.h>
#include <stdio.h>  
#include <math.h>
#include <random>
#include <graph_path_finder/GNode.h>//include .h file for GNode and Graph?
#include <graph_path_finder/Graph.h>
geometry_msgs::Pose g_mobot_pose; //this is the pose of the robot in the world, according to Gazebo
geometry_msgs::Pose g_noisy_mobot_pose; //added noise to x,y, and suppress orientation
geometry_msgs::Quaternion g_quat;
ros::Publisher g_pose_publisher; 
ros::Publisher g_gps_publisher; 
std::normal_distribution<double> distribution(0.0,1.0); //args: mean, std_dev
std::default_random_engine generator;
struct edge{
	node to;
	double dist;
}
struct node{
	int index;
	double x;
	double y;
	edge[] edges;
}

void graph_CB(const graph_path_finder_msgs::Graph& graph) 
{ 
  int num_nodes = graph.nodes.size;
  //ROS_INFO("there are %d models in the transmission",n_models);
  
  for (int i=0;i<num_nodes;i++) {
    node[num_nodes] nodes;
    nodes[i]=new node;
    }
  }
  if(found_name) {
    g_mobot_pose= model_states.pose[imodel];
    g_pose_publisher.publish(g_mobot_pose);
    g_noisy_mobot_pose = g_mobot_pose;
    g_noisy_mobot_pose.orientation = g_quat;
    g_noisy_mobot_pose.position.x += distribution(generator);
    g_noisy_mobot_pose.position.y += distribution(generator);    
    g_gps_publisher.publish(g_noisy_mobot_pose); //publish noisy values
    //double randval = distribution(generator);
    //ROS_INFO("randval =%f",randval);
    }
  else
    {
      ROS_WARN("state of mobot model not found");
    }
} 



int main(int argc, char **argv) {
    ros::init(argc, argv, "graph_solver");
    ros::NodeHandle nh;

    g_path_publisher= nh.advertise<nav_msgs::Path>("/graph_path", 1); 
    ros::Subscriber graph_sub = nh.subscribe("/graph",1,graph_CB); 
    ros::spin();
}
