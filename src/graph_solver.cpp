#include <ros/ros.h> 
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <string.h>
#include <stdio.h>  
#include <math.h>
#include <random>
#include <graph_path_finder/GNode.h>//include .h file for GNode and Graph?
#include <graph_path_finder/Graph.h>
#include <vector>

//TODO: Switch from  
//TODO  Create method pathfromvec that converts from the vec of indexes to a path of poses

struct Node{
	std::string name;
	double x;
	double y;
	double cost;
	std::vector<int> to;
	int best_from;
	Node(std:string n="",double nx=0, double ny=0,
	double c=std::numeric_limits<double>::max(),
	std::vector<int> t,int b=-1):
	name(n), x(nx), y(ny), cost(c), to(t), best_from(b){}
};

std::map<int,Node> g_nodes;
std::map<int,Node>::iterator it;
bool got_alexa_code=false;
int alexa_code=-1;

void graph_CB(const graph_path_finder::Graph::ConstPtr& graph) { 
  int num_nodes = graph->nodes.size;
  g_nodes.clear();
  for (int i=0;i<num_nodes;i++) {
    int index=i;
    Node tmp=new Node();
    tmp.name=graph->nodes[i].name.data;
    tmp.point=graph->nodes[i].point;
    tmp.best_from=-1;
    tmp.cost=std::numeric_limits<double>::max();
    tmp.to.resize(graph->nodes[i].goes_to.data.size());
    for(int j=0;j<graph->nodes[i].goes_to.size();j++){
    		tmp.to[i]=graph->nodes[i].goes_to[j];
    	}
    	g_nodes.insert(index,tmp);
    }
}

int find_nearest_node(double x, double y){
	int min_node=-1;
 	double mindist=std::numeric_limits<double>::max();

  	for (it=g_nodes.begin(); it!=g_nodes.end(); ++it){
  		double x_tmp=it->second.pose.position.x;
  		double y_tmp=it->second.position.y;
  		double currdist=sqrt(pow(x-x_tmp,2)+pow(y-y_tmp,2));
  		if(currdist<mindist){
  			min_node=i;
  		}

  	}
  	return min_node;
}

vector<int> solve(int start, int goal){
	clear_visited();
	std::map<int, Node> visited;
	std::map<int, Node> graph=g_nodes;
	int index=start;
	Node curr=new Node;
	bool all_inf=false;
	//Runs until all nodes are moved to visited;
	while(!graph.empty()&&!all_inf){
		//sets up the node being checked as a temp node
		curr=&graph.find(index)->second;
		//finds the costs for all the nodes the current node goes to;
		for(int i=0; i<curr.to.size();i++){
			double cost=cost(index,curr.to[i])+curr.cost;
			if(cost<curr.to[i].cost){
				curr.to[i].cost=cost;
				curr.best_from=curr.to[i];
			}
		}
		//moves the current node to visited, takes it out of index
		visited.add(index,curr);
		it=graph.find(index);
		graph.erase(it);
		double lowest=std::numeric_limits<double>::max();
		int lowestind=-1;
		//looks for the lowest distance node, checks that all are not max
		//if they are, ends the loop
		for (it=graph.begin(); it!=graph.end(); ++it){
			if(graph->second.cost<lowest){
				lowestind=it->first;
				lowest=it->second.cost;
			}
		}
		if(lowest==std::numeric_limits<double>::max()){
			all_inf=true;
		}
		index=lowestind;
	}	
	if(contains(goal,visited)){
		vector<int> path_keys;
		it=visited.find(goal);
		bool looped=false;
		//this loop starts at the goal node, keeps checking the best node to come from
		//until it gets to the starting index or finds it has entered a loop
		while(it!=start&&!looped){
			path_keys.push_back(it->first);
			it=visited.find(it->second.best_from);
			if(last=it->first){
				looped=true;
			}
			int last=it->first;
		 	}
		if(!looped){
		 	return path_keys;
		}else{
		 	ROS_INFO("WE'VE HIT A MINE");
		 	return new vector<int>;
		}	
	}	
}
void create_path(int start, int end){
	vector<int> keys=solve(start, end);
	while(!keys.empty()){
			//put code from pub_des_state_client here to create path from Poses in the nodes attached to the keys I have
	}
}

double cost(int from, int to){
	it=g_nodes.find(from);
	if(it!=g_nodes.end()){
		Node fromNode=it->second;
	}
	it=g_nodes.find(to);
	if(it!=g_nodes.end()){
		Node toNode=it->second;
	}
	double deltx=toNode.point.x-fromNode.point.x;
	double delty=toNode.point.y-fromNode.point.y;
	return sqrt((pow(deltx,2))+pow(delty,2));
}


bool contains(int key, std::map<int,Node> map){
	for(it=map.begin(); it!=map.end(); it++){
		if(it->first==key){
			return true;
		}
		return false;
	}
}
/*
void alexaCB(const std_msgs::UInt32& code_msg) {
    int alexa_code = code_msg.data;
    ROS_INFO("received Alexa code: %d", alexa_code);
    if (alexa_code= ALEXA_GET_COKE_CODE) {
      g_get_coke_trigger = true;
    }
}
*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "graph_solver");
    ros::NodeHandle nh;
    g_nodes=new map<int,Node>;
    g_path_publisher= nh.advertise<nav_msgs::Path>("/graph_path", 1); 
    ros::Subscriber graph_sub = nh.subscribe("/graph",1,graph_CB); 
    ros::Subscriber alexa_code = nh.subscribe("/Alexa_codes", 1, alexaCB);
    /*
    while(ros::ok()) {
    	if (!g_get_coke_trigger) {
        	ros::Duration(0.5).sleep();
        	ros::spinOnce();    
    	}else{
        	g_get_coke_trigger=false; // reset the trigger
    	}
	}
*/
}
