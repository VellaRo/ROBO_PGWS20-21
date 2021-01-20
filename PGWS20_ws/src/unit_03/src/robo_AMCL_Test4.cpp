//
// LAB: Mobile Robotics
//
// unit_03
// robo_AMCL_Test4
//
// A simple robot control named: Mystery_mover 
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
//ACML POSE
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
// quaternionen..
#include <tf/transform_datatypes.h>
//ODOM
#include <nav_msgs/Odometry.h>
//Fopen
#include <stdio.h>

#include <iostream>
//arctan2
#include<math.h>
//graph
#include "graph.h"
// Class definition
class Mystery_mover {
public:
	Mystery_mover();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void emergencyStop();
	void calculateCommand();
	void readLaser();
	void mainLoop();
	void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);
	void writeInFile();
	void initializeAdjList();
	void executePath(struct MyArray structArray);
	
protected:

	// Nodehandle for Mystery_mover robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// ínitializeAdjList()
vector< pair<int, int> > dist;
vector< vector<pair<int, int> > > adjList;
int start;
int target;

// executePath()
//Knoten
const int KnotenAnzahl = 42;
const int knotenKoordinatenAnzahl = 2;

//Kanten
const int KantenAnzahl = 72;
const int kantenKoordinatenAnzahl =2;

double relativePointToAmcl[KnotenAnzahl][knotenKoordinatenAnzahl];

double degreesTemp;


	//Variablse callback poseAMCL
double amclPositionX;
double amclPositionY;
double amclPositionZ;

//tf::Quaternion orientierungAMCL;

//tf::Matrix3x3 orientierungMatrix;
	//quaternione
double amclOrientationX , amclOrientationY ,amclOrientationZ, amclOrientationW;
//als euler
double amclRoll, amclPitch, amclYaw;
double amclYaw_degrees;
//END   Variables callback poseAMCL 
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL){
	
	//Position
	amclPositionX = msgAMCL->pose.pose.position.x;
	amclPositionY = msgAMCL->pose.pose.position.y;
	amclPositionZ = msgAMCL->pose.pose.position.z;
	
	//quaternionen ablesen aus geom_msgs
	amclOrientationX= msgAMCL->pose.pose.orientation.x;
   	amclOrientationY= msgAMCL->pose.pose.orientation.y;
   	amclOrientationZ= msgAMCL->pose.pose.orientation.z;
   	amclOrientationW= msgAMCL->pose.pose.orientation.w;
	//initialisierung quaternionen
	
	tf::Quaternion amclOrientation(amclOrientationX , amclOrientationY , amclOrientationZ , amclOrientationW);
	//initialisierung Matrix
	tf::Matrix3x3 amclOrientationMatrix(amclOrientation);
	
	//als Euler (rad bis 180° dann -rad bei nächsten 180°) (Matrix auslesen in r ,p,y schreiben)
	amclOrientationMatrix.getRPY(amclRoll, amclPitch, amclYaw);

	//convert yaw to degrees
	amclYaw_degrees = amclYaw * 180.0 / M_PI; // conversion to degrees
	if( amclYaw_degrees < 0 ) amclYaw_degrees += 360.0; // convert negative to positive angles
		
	
}
// odomCallback Variables
double odomPositionX, odomPositionY , odomPositionZ;
double odomOrientationX, odomOrientationY, odomOrientationZ, odomOrientationW;
double odomRoll, odomPitch, odomYaw;
double odomYaw_degrees;
//End odom Variables
void odomCallback(const nav_msgs::Odometry::ConstPtr &msgOdom) {
 	odomPositionX = msgOdom->pose.pose.position.x;
	odomPositionY = msgOdom->pose.pose.position.y;
	odomPositionZ = msgOdom->pose.pose.position.z;

	odomOrientationX = msgOdom->pose.pose.orientation.x;
	odomOrientationY = msgOdom->pose.pose.orientation.y;
	odomOrientationZ = msgOdom->pose.pose.orientation.z;
	odomOrientationW = msgOdom->pose.pose.orientation.w;

	
	tf::Quaternion odomOrentation(odomOrientationX,odomOrientationY,odomOrientationZ,odomOrientationW);
	tf::Matrix3x3  odomOrientationMatrix(odomOrentation);

	odomOrientationMatrix.getRPY(odomRoll,odomPitch, odomYaw);
	
	//convert Yaw to degrees
	odomYaw_degrees = odomYaw * 180.0 / M_PI; // conversion to degrees
	if( odomYaw_degrees < 0 ) odomYaw_degrees += 360.0; // convert negative to positive angles
		
}
// constructor
Mystery_mover::Mystery_mover() {

	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initializing the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Mystery_mover::laserCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

}// end of Mystery_mover constructor



// callback for getting laser values 
void Mystery_mover::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
    
	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size()); 
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback



// have a look into the laser measurement 
void Mystery_mover::readLaser() {
	double sum = 0.0 ;

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	{
	   // go through all laser beams
   	   for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
	     {
		sum = sum + m_laserscan.ranges[i] ;
 	     }// end of for all laser beams
	  
	} // end of if we have laser data
}// end of readLaser 


// robot shall stop, in case anything is closer than ... 
void Mystery_mover::emergencyStop() {

	int danger = 0 ;
        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	   for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
	      if( m_laserscan.ranges[i] <= 0.10) /////////0.40
		 danger = 1 ;		
		
	if(1==danger)
	  {
	    m_roombaCommand.linear.x = 0.0;
 	    m_roombaCommand.angular.z = 0.0;
	    ROS_INFO(" Robot halted by emergency stop" ) ;
	  } // end of if danger
	
}// end of emergencyStop
 


// here we go
// this is the place where we will generate the commands for the robot


void Mystery_mover::calculateCommand() {

	// please find out what this part of the robot controller is doing
	// watch the robot 
	// look into the source file 
	// and try to deduce the underlying idea

        // see if we have laser data available
		   	initializeAdjList();
			
			executePath(returnShortestPath(dist, start ,target));
		
		//distance to Robot
		

} // end of calculateCommands


//
void Mystery_mover::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);


	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok())
	{
	   /*  Sense */
		// get all the sensor value that might be useful for controlling the robot
		readLaser(); 

	   /*  Model */
		// still not necessary to model anything for this job 

	   /* Plan  */
		// Whatever the task is, here is the place to plan the actions
		
		
		
		calculateCommand();
	
		ROS_INFO(" \n poseAMCLx %f \n poseAMCLy %f \n poseAMCLZ %f \n\n Orientierung in Euler(AMCL) ROLL: %.3f  /n PITCH :%.3f /n YAW: %.3f \n Orientierung[AMCL] als degreeYaw: %f \n degreesDistanceTonextPoint%f \n\n" , 
			amclPositionX ,amclPositionY ,amclPositionZ,	
			amclRoll, amclPitch, amclYaw,
			amclYaw_degrees,
			degreesTemp );

		
		// double ERROR = amclYaw_degrees - odomYaw_degrees;
	
		// ROS_INFO("ERROR: %f", ERROR);

	   /* Act   */
		// once everything is planned, let's make it happen

		// last chance to stop the robot
		emergencyStop();	

		// send the command to the roomrider for execution
		m_commandPublisher.publish(m_roombaCommand);

		// spinOnce, just make the loop happen once
		ros::spinOnce();
		// and sleep, to be aligned with the 50ms loop rate
		loop_rate.sleep();

	}// end of if nodehandle O.K.

}// end of mainLoop


int main(int argc, char** argv) {

	// initialize
	ros::init(argc, argv, "Mystery_mover");

	ros::NodeHandle n;
	//subscribe to amcl
	ros::Subscriber sub_amcl = n.subscribe("amcl_pose", 100, poseAMCLCallback);
	//subscribe to odom
	ros::Subscriber sub = n.subscribe("odom", 1000, odomCallback);
	// get an object of type Mystery_mover and call it robbi
	Mystery_mover robbi;

	// make robbi run
	// main loop
	robbi.mainLoop();

	return 0;
}
///NAVIGATE  FROM ARRAY

void Mystery_mover::initializeAdjList(){
	adjList = FormAdjList();
    
    start = 0;  //startknoten
    target = 31; //endknoten   // traget 4,5,10,11,12,13,14,15 macht probleme ???ß
    
	dist =  DijkstraSP(adjList, start);
    }




//bool hasMoved =false; // forintArrayPointer = route;(test) 

double point[KnotenAnzahl][knotenKoordinatenAnzahl] = {
					{0.50,   -12.00},
					{2.00,   -11.30},
					{4.50,    -7.56},
					{5.65,    -5.80},
					{6.20,    -4.95},
					{7.65,    -3.10},
				    {10.30,    0.95},
					{11.00,    2.15},
					{12.60,    4.50},
 					{14.20,    6.80},
  					{15.65,    9.30},
  					{17.50,   11.85},
  					{17.50,   13.00},  
  					{16.75,   12.45},
  					{14.80,    9.60},  
  					{12.25,    5.70},   
  					{11.85,    4.90},  
  					{10.40,    2.80},
  					{8.85,     2.45},    
  					{8.05,    -0.40},
  					{6.25,    -2.30},
  					{5.45,    -5.04},
  					{3.95,    -7.20},
  					{1.51,   -10.85},
  					{0.00,    -12.0},
  					{-1.70,   -11.0} ,  
  					{-0.70,   -9.00},
  					{-2.35,   -7.15},
  					{-2.50,   -8.35},
  					{-1.80,   -10.50},
  					{-2.20,   -13.00},
  					{-2.60,   -14.30} ,
  					{-6.65,   -13.50},
  					{-10.0,   -12.75} , 
  					{-16.0,   -12.00},
  					{-18.6,   -11.25} ,
  					{-19.0,   -11.75}  ,
  					{-20.2,   -16.60} ,
  					{-10.25,  -13.72} ,
  					{-6.75,   -14.30},
  					{-1.50,   -15.25},
  					{-0.50,   -14.80} ,   
										};

double edgeList[72][2] ={
							{0	,1},
							{1	,2},
							{2	,3},
							{3	,4},
							{4	,5},
							{5	,6},
							{6	,7},
							{7	,8},
							{8	,9},
							{9	,10},
							{10	,11},
							{11	,12},
							{12	,13},
							{13	,11},
							{11	,13},
							{13	,14},
							{14	,15},
							{15	,16},
							{16	,8},
							{8	,16},
							{16	,17},
							{17	,7},
							{7	,17},
							{17	,18},
							{18	,17},
							{17	,6},
							{6	,17},
							{7	,18}, // neuer knoten 27
							{18	,19},
							{19	,20},
							{20	,5},
							{5	,20},
							{20	,21},
							{21	,4},
							{4	,21}, 
							{21	,22},
							{22	,3},
							{3	,22}, 
							{22	,23},
							{23	,2},
							{2	,23},
							{23	,24},
							{24	,25},
							{25	,26},
							{26	,27},
							{27	,28},
							{28	,29},
							{29	,25},
							{25	,29},
							{29	,30},
							{30	,37},
							{37	,30},	
							{30	,31},
							{31	,32},	
							{32	,33},
							{33	,34},
							{34	,35},
							{35	,36},
							{36	,35},
							{36	,37},
							{37	,36},
							{36	,38},
							{38	,33},
							{33	,38},
							{38	,39},
							{39	,32},
							{32	,39},
							{39	,40},
							{40	,41},
							{41	,30},
							{30	,41},
							{41	,2}  };
	
	int counter =0;
	void Mystery_mover::executePath(struct MyArray structArray){

			m_roombaCommand.linear.x = 0.05;

			int shortestPathArray[structArray.size];

			for(int i =0; i< structArray.size;i++){
				shortestPathArray[i] = *structArray.pointer; //
				structArray.pointer++;
			}
			

		 double relativePointToAmclX = point[shortestPathArray[counter]][0] -amclPositionX; //anka
		 double relativePointToAmclY = point[shortestPathArray[counter]][1] -amclPositionY;  //geka

		if(relativePointToAmclX <0.3 && relativePointToAmclY <0.3 ){
			if(counter < structArray.size -1){
			counter++;
			}
			else if(counter >= structArray.size -1){
				//	m_roombaCommand.linear.x  = 0.0;
					m_roombaCommand.angular.z = 0.0;
				}
			}
				
		
			
				//degrees distance to the point in RAD
												
			double degreesDistanceToPoint;
			
			//for(int i =0; i<sizeof(degreesDistanceToPoint);i++){
										
										/////RECHNE ICH RICHTIG ?????
													//geka					//anka
				degreesDistanceToPoint = atan2 (relativePointToAmclY,relativePointToAmclX);
			
				//convert Rad to degrees //degreesDistanceToPoint
				degreesDistanceToPoint = degreesDistanceToPoint * 180.0 / M_PI; // conversion to degrees
				if( degreesDistanceToPoint < 0 ) degreesDistanceToPoint += 360.0; // convert negative to positive angles
			
				// Aim degreesTemp == 0
				
		
				degreesTemp = degreesDistanceToPoint - amclYaw_degrees; 

			if(degreesTemp < 0){
				m_roombaCommand.angular.z = -0.2;
			}
			if(degreesTemp >= 0){
			
				m_roombaCommand.angular.z = 0.2;
			 }
	}



// end of file: robo_D.cpp
