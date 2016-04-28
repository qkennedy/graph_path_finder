#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt32.h>
#include <string.h>
#include <math.h>
#include <graph_path_finder/GNode.h>//include .h file for GNode and Graph?
#include <graph_path_finder/Graph.h>
#include <vector>
#include <iostream>
using namespace std;
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
    ros::Publisher alexa_spoof;
    graph_path_finder::Graph graph;
    graph_path_finder::GNode node;
    ros::Publisher graph_pub;
void oneNodeDummy(){
    graph.nodes.clear();
    node.name.data="Hello World";
    geometry_msgs::Point point;
    point.x=0;
    point.y=1;
    node.point=point;
    std_msgs::Int32MultiArray array;
    for(int i=0;i<10;i++){
        array.data.push_back(i);
    }
    node.goes_to=array;
    graph.nodes.push_back(node);
    graph_pub.publish(graph);
}
void threeNodeDummy(){
    //1st node
    graph.nodes.clear();
    node.name.data="Hello World";
    geometry_msgs::Point point;
    point.x=0;
    point.y=1;
    node.point=point;
    std_msgs::Int32MultiArray array;
    for(int i=0;i<10;i++){
        array.data.push_back(i);
    }
    node.goes_to=array;
    graph.nodes.push_back(node);
    //2nd node
    node.name.data="This is a node";
    node.point.x=15;
    node.point.y=15;
    array.data.clear();
    array.data.push_back(0);
    graph.nodes.push_back(node);
    node.goes_to=array;
    //3rd node
    node.name.data="Onemore";
    node.point.x=-24;
    node.point.y=10;
    array.data.clear();
    array.data.push_back(1);
    graph.nodes.push_back(node);
    node.goes_to=array;
    //publish
    int size=graph.nodes.size();
    ROS_INFO("Number of Nodes should be 3, is %d",size);
    graph_pub.publish(graph);
}
void real_graph(){
    graph.nodes.clear();
    node.name.data="Zero";
    geometry_msgs::Point point;
    point.x=0;
    point.y=0;
    node.point=point;
    std_msgs::Int32MultiArray array;
    array.data.push_back(1);
    array.data.push_back(2);
    node.goes_to=array;
    graph.nodes.push_back(node);
    node.name.data="One";
    node.point.x=1;
    node.point.y=1;
    array.data.clear();
    node.goes_to=array;
    graph.nodes.push_back(node);
    node.name.data="Two";
    node.point.x=0;
    node.point.y=3;
    array.data.clear();
    array.data.push_back(3);
    node.goes_to=array;
    graph.nodes.push_back(node);
    node.name.data="Three";
    node.point.x=4;
    node.point.y=3;
    array.data.clear();
    node.goes_to=array;
    graph.nodes.push_back(node);
    graph_pub.publish(graph);
    std_msgs::UInt32 msg;
    msg.data=10003;
    alexa_spoof.publish(msg);
}
void simple_looped(){
    graph.nodes.clear();
    node.name.data="Zero";
    geometry_msgs::Point point;
    point.x=0;
    point.y=0;
    node.point=point;
    std_msgs::Int32MultiArray array;
    array.data.push_back(1);
    array.data.push_back(2);
    node.goes_to=array;
    graph.nodes.push_back(node);
    node.name.data="One";
    node.point.x=1;
    node.point.y=1;
    array.data.clear();
    array.data.push_back(0);
    node.goes_to=array;
    graph.nodes.push_back(node);
    graph_pub.publish(graph);
    std_msgs::UInt32 msg;
    msg.data=10003;
    alexa_spoof.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "graph_publisher"); // name of this node will be "minimal_publisher"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    graph_pub = n.advertise<graph_path_finder::Graph>("/graph", 1);
    alexa_spoof = n.advertise<std_msgs::UInt32>("/Alexa_codes", 1);
    while (ros::ok()) {
        cout<<endl;
        cout << "enter a number \n 1:Rand 1 Node \n 2:Rand 3 Nodes \n 3: real 4 node graph \n 4:ShortLoop \n (x to quit): ";
        std::string in_name;
        cin>>in_name;
        if (in_name.compare("x")==0){
            return 0;
        }else if(in_name.compare("1")==0){
            oneNodeDummy();
        }else if(in_name.compare("2")==0){
            threeNodeDummy();
        }else if(in_name.compare("3")==0){
            real_graph();
        }else if(in_name.compare("4")==0){
            simple_looped();
        }
    }
}

