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
	int index;
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
    tmp.index=i;
    tmp.x=0;
    tmp.y=0;
    tmp.best_from=-1;
    tmp.cost=std::numeric_limits<double>::max();
    tmp.to.resize(graph.nodes[i].goes_to.size());
    for(int j=0;j<graph.nodes[i].goes_to.size();j++){
    		tmp.to[i]=graph.nodes[i].goes_to[i];
    	}
    	g_nodes.push_back(tmp);
    }
}

int find_nearest_node(nav_msgs::Point point){
	int minNode=-1;
 	double mindist=std::numeric_limits<double>::max();;
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

std::vec<node> solve(int start, int goal){
	clear_visited();
	std::vec<node> visited(g_nodes.size());
	int index=start;
	node curr=new node;
	bool all_inf=false;
	//Runs until all nodes are moved to visited;
	while(!empty(g_nodes&&!all_inf){
		//sets up the node being checked as a temp node
		curr=&g_nodes[index];
		//finds the costs for all the nodes the current node goes to;
		for(int i=0; i<curr.to.size();i++){
			double cost=cost(index,curr.to[i])+curr.cost;
			if(cost<curr.to[i].cost){
				curr.to[i].cost=cost;
				curr.best_from=curr.to[i];
				}
			}
			//moves the current node to visited, takes it out of index
			visited[index]=curr;
			g_nodes.erase(index);
			double lowest=std::numeric_limits<double>::max();;
			int lowestind=-1;
			//looks for the lowest distance node, checks that all are not max
			//if they are, ends the loop
			for(int j=0;j<g_nodes.size();j++){
				if(g_nodes[j].cost<lowest){
					lowestind=j;
					lowest=g_nodes[j].cost;
				}
			}
			if(lowest==std::numeric_limits<double>::max()){
				all_inf=true;
			}
			index=lowestind;
		}
		/*what I am about to do is kind of stupid.
		 Since I don't know that the index in visited will match the index of the node
		 I am looping through to find the goal node, then looping to find
		 each nodes best_from until I find the start.
		 Eventually, the solution would probably be to use something that 
		 allows for the index of the node to always be the index in the datastructure
		 would improve speed significantly.
		 I might need to do this anyways because I think I will run into problems with
		 how the initial loop works
		 I will change to this tonight 4-25-16 QCK
		*/
		bool wasFound=false;
		int i=0;
		//finds starting index
		while(!wasFound && i<visited.size()){
			if(visited[i].index==goal){
				wasFound=true;
				index=i;
			}
		}
		while(visited[index].index!=start){
			visited[index].best_from
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
