//
// LAB: Mobile Robotics
//
// unit_02 
// robo_GroesteFlaeche.cpp
//
// A very simple robot control named: Test_move 
//
// Author: University of Bonn, Autonomous Intelligent Systems
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

/// MEINE INCLUDES ///

// FOR TANH
#include <math.h>
// for max
#include <algorithm>

// Class definition

class Just_move {
public:
	Just_move();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void emergencyStop();
	void calculateCommand();
	void readLaser();
	void mainLoop();
	//test
	void test();
	//calcs Area and sets direction
	void calculateArea();

protected:

	// Nodehandle for Just_move robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// constructor
Just_move::Just_move() {

	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initializing the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Just_move::laserCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

}// end of Just_move constructor



// callback for getting laser values 
void Just_move::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
    
	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size()); 
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback



// have a look into the laser measurement 
void Just_move::readLaser() {
	double sum = 0.0 ;

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	{
	   // go through all laser beams
   	   for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
	     {
		sum = sum + m_laserscan.ranges[i] ;
 	     }// end of for all laser beams
	  
	   ROS_INFO(" Sum of all measured distances:= %f \n", sum ) ; 
	} // end of if we have laser data
}// end of readLaser 


// robot shall stop, in case anything is closer than ... 
void Just_move::emergencyStop() {

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	{
		for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
		{
			if( m_laserscan.ranges[i] <= 0.20) {
				m_roombaCommand.linear.x = 0.0;
				m_roombaCommand.angular.z = 0.0;
			}// end of if too close
		}// end of for all laser beams
	} // end of if we have laser data
}// end of emergencyStop





// here we go
// this is the place where we will generate the commands for the robot
void Just_move::calculateCommand() {

	// set the roomba velocities
	// the linear velocity (front direction is x axis) is measured in m/sec
	// the angular velocity (around z axis, yaw) is measured in rad/sec
	
        // see if we have laser data
		

 	if( (&m_laserscan)->ranges.size() > 0) 
	  {
		  
		 calculateArea();


	  
	  } // end of if we have laser data
	  
} // end of calculateCommands


//
void Just_move::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);		

	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok())
	{
	  /* Sense */
		// get all the sensor value that might be useful for controlling the robot
		readLaser(); 

	  /* Model */
		// still not necessary to model anything for this job 

	  /* Plan  */
		// Whatever the task is, here is the place to plan the actions
		calculateCommand();

		ROS_INFO(" robo_GroesteFlaeche: movement commands:forward speed=%f[m/sec] and turn speed =%f[rad/sec]", 
			m_roombaCommand.linear.x, m_roombaCommand.angular.z);


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

///meine Funktionen///


//Calculates maximum area
void Just_move::calculateArea(){

	double maximumArea = 0.0;
	char * direction;
	double areaRight = 0.0;
	double areaMiddle = 0.0;
	double areaLeft = 0.0;
	
	//right Segment
	for(int i = 0;i<180;i++){
		if(m_laserscan.ranges[i] ==5.00){
			areaRight += 30.0;
		}
		areaRight += m_laserscan.ranges[i];
	}
	//middle Segments
	for(int i = 181;i<361;i++){
		if(m_laserscan.ranges[i] ==5.00){
			areaMiddle += 30.0;
		}
		areaMiddle += m_laserscan.ranges[i];
	}

	//left Segment 
	for(int i = 361;i<540;i++){
		if(m_laserscan.ranges[i] ==5.00){
			areaLeft += 30.0;
		}
		areaLeft += m_laserscan.ranges[i];
	}

	double tempMax =  std::max(areaMiddle, areaLeft);
	maximumArea = std::max(areaRight, tempMax);
	
	
	//linear geschwindigkeit
	  	m_roombaCommand.linear.x  = 0.3* tanh(0.5*( m_laserscan.ranges[250] +  m_laserscan.ranges[290])-0.80);		

	//um im Gang gerade zu bleiben
	
	// mitte am größten
	if (maximumArea == areaLeft || maximumArea == areaRight || m_laserscan.ranges[180] <= 0.80 || m_laserscan.ranges[450] <=0.80 || m_laserscan.ranges[270] <= 0.80){
		m_roombaCommand.angular.z = 0.3* tanh(areaLeft - areaRight);	
	
		}	
	else{
		m_roombaCommand.angular.z = 0.0;
	// var 2 ??ß	m_roombaCommand.angular.z = 0.3* tanh(areaLeft - areaRight);	
		
	}
	
	//Test consoleout:
	ROS_INFO(" robo_GroesteFlaeche: areaRight: %f ; areaMiddle: %f ; areaLeft: %f ; distance:%f",
				 areaRight ,areaMiddle , areaLeft, m_laserscan.ranges[270]);
	
}

int main(int argc, char** argv) {

	// initialize
	ros::init(argc, argv, "Just_move");

	// get an object of type Just_move and call it robbi
	Just_move robbi;

	// make robbi run
	// main loop
	robbi.mainLoop();

	return 0;
}

// end of file: robo_GroesteFlaeche.cpp
