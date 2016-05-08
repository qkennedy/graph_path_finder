#include <ros/ros.h> 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt32.h>
#include <graph_path_finder/GNode.h>//include .h file for GNode and Graph?
#include <graph_path_finder/Graph.h>
#include <alpha_mobot_pub_des_state/path.h>
#include <vector>
#include <string>
#include <sstream>
#include <string.h>
#include <math.h>

/*TODO  Test that djikstra's works with a loop in the nodes, might segfault
TODO  Modify pub_des_state to work off of map_frame from amcl
TODO  Run gazebo with the map of the lab, run amcl kit, check whether 
		the output matches what I am expecting
*/
struct Node{
	std::string name;
	double x;
	double y;
	double cost;
	std::vector<int> to;
	int best_from;
	Node(std::string n="",double nx=0, double ny=0,
	double c=std::numeric_limits<double>::max(),
	std::vector<int> t=std::vector<int>(),int b=-1):
	name(n), x(nx), y(ny), cost(c), to(t), best_from(b){}
};


ros::ServiceClient client;
std::map<int,Node> g_nodes;
std::map<int, Node> visited;
std::map<int,Node>::iterator it;
bool got_alexa_code=false;
bool has_nodes;
int code=-1;
ros::Publisher g_path_publisher;
double mapx_offset;
double mapy_offset;
geometry_msgs::Pose odom_pose;
geometry_msgs::Pose map_pose;




double get_cost(int from, int to){
	it=g_nodes.find(from);
	Node toNode;
	Node fromNode;
	if(it!=g_nodes.end()){
		fromNode=it->second;
	}
	it=g_nodes.find(to);
	if(it!=g_nodes.end()){
		toNode=it->second;
	}
	double deltx=toNode.x-fromNode.x;
	double delty=toNode.y-fromNode.y;
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
void print_nodes(std::map<int,Node> map){
	ROS_INFO("Beginning Node Print");
	Node curr;
	for(it=map.begin(); it!=map.end(); ++it){
		curr=it->second;
		ROS_INFO("Index # %d Name:%s",it->first,curr.name.c_str());
		ROS_INFO("Point  ( %f , %f )",curr.x,curr.y);
		std::string to_nodes;
		std::ostringstream oss;
		oss << "Pointing to:  ";
		for(int i=0; i<curr.to.size();i++){
			oss<<curr.to[i]<<", ";
		}
		to_nodes=oss.str();
		ROS_INFO("%s",to_nodes.c_str());
		if(curr.cost==std::numeric_limits<double>::max()){
			ROS_INFO("cost=inf");	
		}else{
			ROS_INFO("cost=%f",curr.cost);
		}
		ROS_INFO("best from: %d",curr.best_from);
	}
	ROS_INFO("ENDING STREAM");
}
int find_nearest_node(double x, double y){
	int min_node=-1;
	Node best;
 	double mindist=std::numeric_limits<double>::max();

  	for (it=g_nodes.begin(); it!=g_nodes.end(); ++it){
  		double x_tmp=it->second.x;
  		double y_tmp=it->second.y;
  		double currdist=sqrt(pow(x-x_tmp,2)+pow(y-y_tmp,2));
  		if(currdist<mindist){
  			min_node=it->first;
  			mindist=currdist;
  			best=it->second;
  		}

  	}
  	ROS_INFO("starting from node= %d (%f,%f) currently at (%f,%f)",min_node, best.x,best.y,x,y);
  	return min_node;
}

std::vector<int> solve(int start, int goal){
	visited.clear();
	std::map<int, Node> graph=g_nodes;
	graph[start].cost=0;
	int index=start;
	Node curr;
	bool all_inf=false;
	//Runs until all nodes are moved to visited;

	int num_nodes=graph.size();
	while(num_nodes!=0){
				if(index==-1){
			ROS_INFO("NO PATH FOUND, SORRY DAD");
			std::vector<int> broken_arrow;
			return broken_arrow;
		}
		ROS_INFO("Looper was a great film");
		//sets up the node being checked as a temp node
		ROS_INFO("curr ind: %d",index);
		curr=graph.find(index)->second;
		//finds the costs for all the nodes the current node goes to;
		for(int i=0; i<curr.to.size();i++){
			double addcost=get_cost(index,curr.to[i]);
			double cost= addcost+curr.cost;
			double oldcost;
			it=graph.find(curr.to[i]);
			if(it!=graph.end()){

				oldcost=it->second.cost;
			}
			if(cost<oldcost){
				it->second.cost=cost;
				it->second.best_from=index;
			}
		}
		//moves the current node to visited, takes it out of index
		visited[index]=curr;
		it=graph.find(index);
		if(it!=graph.end()){
		graph.erase(it);
		num_nodes=graph.size();
		}

		double lowest=std::numeric_limits<double>::max();
		int lowestind=-1;
		//looks for the lowest distance node, checks that all are not max
		//if they are, ends the loop
		for (it=graph.begin(); it!=graph.end(); ++it){
			if(it->second.cost<lowest){
				lowestind=it->first;
				lowest=it->second.cost;
			}
		}
		if(lowest==std::numeric_limits<double>::max()){
			all_inf=true;
		}
		index=lowestind;
	}
	print_nodes(visited);
		std::vector<int> path_keys;
		it=visited.find(goal);
		bool looped=false;
		//this loop starts at the goal node, keeps checking the best node to come from
		//until it gets to the starting index or finds it has entered a loop
		int last=-1;
		while(it->first!=start){
			path_keys.push_back(it->first);
			ROS_INFO("pushed %d",it->first);
			it=visited.find(it->second.best_from);
		 	}
		if(it->first==start){
			path_keys.push_back(start);
			ROS_INFO("trying to return");
		 	return path_keys;
		}else{
		 	ROS_INFO("WE'VE HIT A MINE");
		 	std::vector<int> broken_arrow;
		 	return broken_arrow;
		}	
}


void alexaCB(const std_msgs::UInt32& code_msg) {
	ros::Duration(0.5).sleep();
    code = code_msg.data;
    ROS_INFO("received Alexa code: %d", code);
      got_alexa_code=true;
}
void create_path(int start, int end){
	std::vector<int> keys=solve(start, end);
	alpha_mobot_pub_des_state::path path_srv;
	geometry_msgs::Point point;
	geometry_msgs::PoseStamped pose;
	Node curr;
	std::vector<int>::reverse_iterator vecit;
	for ( vecit= keys.rbegin() ; vecit != keys.rend(); ++vecit){
		int ind=*vecit;
		ROS_INFO("%d",ind);
		curr = visited.find(ind)->second;
		ROS_INFO("Adding to Path: n = %s at (%f,%f)",curr.name.c_str(),curr.x,curr.y);
		point.x=curr.x;
		point.y=curr.y;
		pose.pose.position=point;
		path_srv.request.path.poses.push_back(pose);
	}
	while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    client.call(path_srv);
}

void graph_CB(const graph_path_finder::Graph::ConstPtr& graph) {
	ROS_INFO("entering Graph_CB"); 
	int num_nodes = graph->nodes.size();
  	ROS_INFO("received: %d nodes",num_nodes); 
  	g_nodes.clear();
  	for (int i=0;i<num_nodes;i++) {
    	int index=i;
    	Node tmp;
    	tmp.name=graph->nodes[i].name.data;
    	ROS_INFO("name:%s",tmp.name.c_str()); 
    	tmp.x=graph->nodes[i].point.x;
    	tmp.y=graph->nodes[i].point.y;
    	ROS_INFO("coord= (%f,%f)",tmp.x,tmp.y);
    	tmp.best_from=-1;
    	tmp.cost=std::numeric_limits<double>::max();
    	tmp.to.resize(graph->nodes[i].goes_to.data.size());
    	for(int j=0;j<graph->nodes[i].goes_to.data.size();j++){
    		tmp.to[j]=graph->nodes[i].goes_to.data[j];
    	}
    	g_nodes[i]=tmp;
    }
    print_nodes(g_nodes);
}
void odomCB(const nav_msgs::Odometry& odom) {
	odom_pose=odom.pose.pose;
}
void amclCB(const geometry_msgs::PoseWithCovarianceStamped& pose) {
	map_pose=pose.pose.pose;
	mapx_offset=map_pose.position.x;
	mapy_offset=map_pose.position.y;
	ROS_INFO("Offset Update: (%f,%f)",mapx_offset,mapy_offset);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "graph_solver");
    ros::NodeHandle nh;
    g_path_publisher= nh.advertise<nav_msgs::Path>("/graph_path", 1); 
    ros::Subscriber graph_sub = nh.subscribe("/graph",1,graph_CB); 
    ros::Subscriber alexa_code = nh.subscribe("/Alexa_codes", 1, alexaCB);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCB);
    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose",1,amclCB); 
    client = nh.serviceClient<alpha_mobot_pub_des_state::path>("append_path_queue_service");

    while(ros::ok()) {
    	if (!got_alexa_code&&!has_nodes) {
        	ros::Duration(0.5).sleep();
        	ros::spinOnce();    
    	}else{
        	got_alexa_code=false; // reset the trigger
        	double mapx=0;
        	double mapy=0;
        	mapx=mapx_offset;
        	mapy=mapy_offset;
        	int start_ind = find_nearest_node(mapx,mapy);
        	code-=10000;
        	ROS_INFO("Trying to path from %d to %d",start_ind,code);
        	create_path(start_ind,code);
        	code=-1;
    	}
	}
}
