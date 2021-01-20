// A simple robot named: bob_ros
// Author: University of Bonn, Autonomous Intelligent Systems
//
// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <queue>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/image_encodings.h>
#include "dikstra.h"
#include <bits/stdc++.h>
#include <list>
#include "knoten.cpp"
//ME
#include <visualization_msgs/Marker.h>
#include <vector>
#include <geometry_msgs/PointStamped.h>



// Class definition
class bob_ros {
public:
	bob_ros();
	
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void amclCallback(const geometry_msgs::PoseWithCovarianceStamped pose);
	void handleNavigationNode(const geometry_msgs::Pose2D);
	void pointStampedCallback(const geometry_msgs::PointStamped);
	
	void moveToWardsNextNode();
	void drawMarkers();
	void clickToMove();

	void dest_input();
	void emergencyStop();
	void calculateCommand();
	void mainLoop();
	void readLaser();
	
protected:

	// Nodehandle for bob_ros robot
	ros::NodeHandle m_nodeHandle;

	///Publisher/////////////////

	// driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

	//Maker
	ros::Publisher m_marker_pub; 

	///Subscriber////////////////

	// laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

	// AMCL
	ros::Subscriber m_amclSubscriber;
	geometry_msgs::PoseWithCovarianceStamped amcl_pose;
	geometry_msgs::Pose2D pose_estimate;

	//PointStamped
	ros::Subscriber m_PointStamped_subscriber;
	geometry_msgs::Point pointStamped;
	//Navigation Node
	ros::Subscriber navigation_node_subscriber;
	
	// Queue of absolute positions that the robot should move to in order of the queue
	std::queue<geometry_msgs::Pose2D> navigation_node_queue;
	geometry_msgs::Pose2D *current_navigation_node;

	///GLOBAL VARIABLES ////////

	int sizeKordinanten = sizeof(koordinaten) / sizeof(koordinaten[0]);
	int sizeKantenliste = sizeof(kantenListe) / sizeof(kantenListe[0]);


//	double counter = 0;

};

// constructor
bob_ros::bob_ros() {

	// Initialising the node handle
	ros::NodeHandle m_nodeHandle("/");

	//Publisher///////
	
	//roomba command  
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);
	//marker
	m_marker_pub = m_nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	//Subscriber//////
	
	//laser
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &bob_ros::laserCallback, this);
	// pointstamped (Click in rviz)
	m_PointStamped_subscriber = m_nodeHandle.subscribe<geometry_msgs::PointStamped>("clicked_point" ,20, &bob_ros::pointStampedCallback, this);
	//NAV
	navigation_node_subscriber = m_nodeHandle.subscribe<geometry_msgs::Pose2D>("navigation_node", 20, &bob_ros::handleNavigationNode, this);
	//AMCL
	m_amclSubscriber = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("amcl_pose", 20, &bob_ros::amclCallback,this);

	
	

}// end of bob_ros constructor

///CALLBACKS///////////////////////////////////


// callback for getting laser values
void bob_ros::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {

	if(	(&m_laserscan)->ranges.size() < scanData->ranges.size()	){
		m_laserscan.ranges.resize(	(size_t)scanData->ranges.size()	);
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++){
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback

//AMCL Callback
void bob_ros::amclCallback(const geometry_msgs::PoseWithCovarianceStamped pose) {

	amcl_pose = pose;
	float	x = amcl_pose.pose.pose.position.x;
	float y = amcl_pose.pose.pose.position.y;

	const geometry_msgs::Quaternion _q = amcl_pose.pose.pose.orientation;
	const tf2::Quaternion q(_q.x, _q.y, _q.z, _q.w);
	const tf2::Matrix3x3 m(q);
	double roll;
	double pitch;
	double yaw;
	m.getRPY(roll, pitch, yaw);

	const float theta = yaw;

	pose_estimate.x = x;
	pose_estimate.y = y;
	pose_estimate.theta = theta;

}

void bob_ros::pointStampedCallback(const geometry_msgs::PointStamped point){

	pointStamped = point.point;
}

void bob_ros::handleNavigationNode(const geometry_msgs::Pose2D navigation_node) {
	navigation_node_queue.push(navigation_node);

	ROS_INFO("Queued navigation node { .x = %f, .y = %f, .theta = %f }", navigation_node.x, navigation_node.y, navigation_node.theta);
}
// http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics

//knoten auswahlen...
//Dummy-initialize
int start_knoten = 0;
int ziel_knoten = 0;

 // clear Nav_queue first and fill Nav_queue with new values
void bob_ros::dest_input() {
	//empty queue
	for(unsigned int i =0; i< navigation_node_queue.size();i++){
	navigation_node_queue.pop();
}

	list<vertex_t> path = dikstra_main(start_knoten,ziel_knoten);

	int path_size = path.size();
	for(int i=0; i<path_size; i++){
		int element = path.front();
		path.pop_front();
		geometry_msgs::Pose2D pose;
    pose.x = koordinaten[element][0];
    pose.y = koordinaten[element][1];
    pose.theta = 0;
		navigation_node_queue.push(pose);
	}
}

void bob_ros::moveToWardsNextNode(){

		// Do nothing when there is no destination
	if (navigation_node_queue.empty()){
			m_roombaCommand.linear.x = 0.0;
			m_roombaCommand.angular.z = 0.0;
			return;
		}

	const geometry_msgs::Pose2D dest = navigation_node_queue.front();

	const float factor = 0.10;
	const float d_x = fabs(dest.x - pose_estimate.x);
	const float d_y = fabs(dest.y - pose_estimate.y);
	const float factor_angle = (10.0 / 360) * 2 * M_PI;

	// check if robot in range factor
	if (d_x < factor && d_y < factor) {
		ROS_INFO("REACHED DESTINATION: 	.{.x = %f, .y = %f, .theta = %f }", dest.x, dest.y, dest.theta);
		ROS_INFO("ACTUAL COORDINATES: 	.{.x = %f, .y = %f, .theta = %f }", pose_estimate.x, pose_estimate.y, pose_estimate.theta);
		navigation_node_queue.pop();
	}

	const geometry_msgs::Pose2D dest2 = navigation_node_queue.front();

	const float diff_x = dest2.x - pose_estimate.x;
	const float diff_y = dest2.y - pose_estimate.y;

	// The angle that directly leads to the destination pose
	const float d_theta = atan2f(diff_y, diff_x) - pose_estimate.theta;
	
	//TODO in die richtung drehen die Weniger dauert 
	/*
	..
	..
	..
	*/

	m_roombaCommand.angular.z = 0.3 * tanh(d_theta* 10);
	// Only turn when we are in 45deg range of the destination direction
	m_roombaCommand.linear.x = 0.1 * tanh(10 * (pow(diff_x, 2) + pow(diff_y, 2))) * (fabs(d_theta) < M_PI / 4.0);

}

void bob_ros::drawMarkers(){

	// initialize Koardinaten
	visualization_msgs::Marker points;
	geometry_msgs::Point p;

	points.header.frame_id = "odom";
	points.header.stamp = ros::Time();
	points.ns = "nav_graphVisualisationPoints";
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.action = visualization_msgs::Marker::ADD;
	points.pose.position.z = 0.0;
	points.pose.orientation.x = 0.0;
	points.pose.orientation.y = 0.0;
	points.pose.orientation.z = 0.0;
	points.pose.orientation.w = 1.0;
	points.scale.x = 0.1;
	points.scale.y = 0.1;
	points.color.a = 1.0; // Don't forget to set the alpha!
	points.color.r = 0.0;
	points.color.g = 1.0;
	points.color.b = 0.0;

	// initialize MARKER ARROW-LINES (gerichtete Kanten zwischen Koordinaten)

	std::vector<visualization_msgs::Marker> arrow;
	arrow.resize(sizeKantenliste);

		//initalize vector of Marker (ARROW)
	for (unsigned int i = 0; i < arrow.size(); i++) {
		arrow[i].header.frame_id ="odom";
		arrow[i].header.stamp = ros::Time();
		arrow[i].ns = "nav_graphVisualisationArrow";
		arrow[i].id = i;
		arrow[i].action = visualization_msgs::Marker::ADD;
		arrow[i].type = visualization_msgs::Marker::ARROW; //ARROW
		arrow[i].scale.y = 0.03;
		arrow[i].scale.z = 0.03;
		arrow[i].color.a = 1.0; // Don't forget to set the alpha!
		arrow[i].color.r = 0.0;
		arrow[i].color.g = 0.0;
		arrow[i].color.b = 1.0;
	}

	//initalize  Marker-text (text unter Koordinaten)
	std::vector<visualization_msgs::Marker> text;

	text.resize(sizeKordinanten);

	for (unsigned int i = 0; i < text.size(); i++) {
  	
	text[i].header.frame_id = "odom";
  	text[i].header.stamp = ros::Time();
  	text[i].ns = "nav_graphVisualisationText";
  	text[i].id = i;
  	text[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  	text[i].action = visualization_msgs::Marker::ADD;
	text[i].pose.orientation.w = 1.0;
  	text[i].scale.z = 0.2;
  	text[i].color.a = 1.0; // Don't forget to set the alpha!
  	text[i].color.r = 1.0;
  	text[i].color.g = 0.0;
  	text[i].color.b = 0.0;
	}

	//DRAW POINTS // PRINT TEXT 
	for(unsigned int i =0; i<sizeKordinanten;i++){
		
		p.x = koordinaten[i][0];
		p.y = koordinaten[i][1]; 
		text[i].pose.position.x = p.x -0.15;
		text[i].pose.position.y = p.y -0.15;
		text[i].text= to_string(i);

		text[i].points.push_back(p);
 		points.points.push_back(p);	

		//PUBLISH 
		m_marker_pub.publish(text[i]);	
	}
		m_marker_pub.publish(points);

	//DRAW ARROW-LINES  (CONECTIONS) 
	tf::Quaternion q;

	for(unsigned int i=0; i<sizeKantenliste;i++){

		int kantenStart = kantenListe[i][0];
		int kantenEnde = kantenListe[i][1];

		//set start of Arrow
		arrow[i].pose.position.x =koordinaten[kantenStart][0];
		arrow[i].pose.position.y =koordinaten[kantenStart][1];
		
		//pythagoras // for lenght Arrow
		float ankatete =koordinaten[kantenEnde][0] -koordinaten[kantenStart][0];
		float gegenkatete = koordinaten[kantenEnde][1] - koordinaten[kantenStart][1];
		arrow[i].scale.x =  sqrt(pow(ankatete,2) + pow(gegenkatete,2)) ;

		//degree between hypothenuse , ankatete
		double yawArrowDegree = atan2f(gegenkatete, ankatete);	
		
		q.setRPY(0.0 , 0.0,yawArrowDegree);
		
		//set orientation of Arrow
		arrow[i].pose.orientation.w = q.getW();
		arrow[i].pose.orientation.x = q.getX();
		arrow[i].pose.orientation.y = q.getY();
		arrow[i].pose.orientation.z = q.getZ();
		
		//PUBLISH 

		m_marker_pub.publish(arrow[i]);
	}
}

float previousPointStampedX =0.0;
float previousPointStampedY =0.0;
		
bool isNewClick = false;
void bob_ros::clickToMove(){


	// check if there is a new click
	if(previousPointStampedX != pointStamped.x && previousPointStampedY != pointStamped.y){
		previousPointStampedX = pointStamped.x;
		previousPointStampedY = pointStamped.y;

		for(unsigned int i =0; i<sizeKordinanten;i++){
			// check which Point is nearest to the click
			if(pointStamped.x > koordinaten[i][0] -0.3 && pointStamped.x < koordinaten[i][0]+ 0.3 
			&& pointStamped.y > koordinaten[i][1] -0.3 && pointStamped.y < koordinaten[i][1]+ 0.3){
			
				isNewClick = true;
				ziel_knoten = i;	
			}
		}
	}
	if(isNewClick){
		// Nearest Point // max radius 10m
		bool foundNearestNode = false;
		for(unsigned int j =0;j<1000 && !foundNearestNode;j++){

			for(unsigned int i =0; i<sizeKordinanten && !foundNearestNode;i++){

				if(pose_estimate.x > koordinaten[i][0] -0.1 * j && pose_estimate.x < koordinaten[i][0]+ 0.1 * j 
				&& pose_estimate.y > koordinaten[i][1] -0.1 * j && pose_estimate.y < koordinaten[i][1]+ 0.1 * j){
					start_knoten = i;
					ROS_INFO("\n --------------------------  \n --------------------------  \n -------------------------- \n go FROM: %d TO: %d \n --------------------------  \n --------------------------  \n -------------------------- \n" , start_knoten, ziel_knoten );
					dest_input();
					isNewClick = false;
					foundNearestNode = true;
					}

				//TODO NO NODE FOUND IN 10M////////////////
				/*


				*/
			}
		}
	}
	 
}

///BASIC OPERATION FUNCTIONS/////

// have a look into the laser measurement 
void bob_ros::readLaser() {
	double sum = 0.0 ;

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	{
	   // go through all laser beams
   	   for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
	     {
		sum = sum + m_laserscan.ranges[i] ;
 	     }// end of for all laser beams
	  
	   ROS_INFO(" Sum of all measured distances:= %f \n", sum ); 
	} // end of if we have laser data
}// end of readLaser 

// robot shall stop, in case anything is closer than 0.20m

void bob_ros::emergencyStop() {
        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0)
	{
		for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
		{
			if( m_laserscan.ranges[i] <= 0.10) {
				m_roombaCommand.linear.x = 0.0;
				m_roombaCommand.angular.z = 0.0;

				// //WAS IST DAS???
				// if (!navigation_node_queue.empty()){
				// 		m_roombaCommand.linear.x = 0.0;
				// 		m_roombaCommand.angular.z = 1.5;
				// 		// work on this later to make it not stop at walls or corners
				// 		// if the next position is " behind it " 

				// 		return;
				// 	}
			}// end of if too close

		}// end of for all laser beams
	} // end of if we have laser data
}// end of emergencyStop


// here we go
// this is the place where we will generate the commands for the robot
void bob_ros::calculateCommand() {
	
	moveToWardsNextNode();
	
} // end of calculateCommands


//
void bob_ros::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);

	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok())
	{

		// if (counter == 0) {
		// 	dest_input();
		// 	counter = 1;
		// }

		calculateCommand();
		emergencyStop();
		drawMarkers();
		clickToMove();

		// send the command to the roomrider for execution
		m_commandPublisher.publish(m_roombaCommand);

		// spinOnce, just make the loop happen once
		ros::spinOnce();

		// and sleep, to be aligned with the 50ms loop rate
		loop_rate.sleep();
	}
}// end of mainLoop


int main(int argc, char** argv) {

	// initialize
	ros::init(argc, argv, "bob_ros");

	// get an object of type bob_ros and call it dude
	bob_ros dude;

	// main loop
	// make dude execute it's task
	dude.mainLoop();

	return 0;

}// end of main

// end of file: bob_ros.cpp