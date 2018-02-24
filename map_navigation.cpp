#include <kobuki_msgs/BumperEvent.h> 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
//#include <cmvision/Blobs.h>
#include <stdio.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// ROS functionalities
ros::Publisher pub;

// *********************** State Variables ************************ //
bool active;
enum State { TO_ELEV, WAIT_ELEV, ENTER_ELEV, ON_ELEV, EXIT_ELEV, TO_TARGET, AWAIT_INPUT };
State my_state;

// ************************** Targets ************************** //

#define HOME_X 		0.398
#define HOME_Y 		0.394
#define DELIVERY_X 	50.0	// you'll want to make DELIVERY_X and DELIVER_Y not '#define's 
#define DELIVERY_Y 	50.0		// for if you're searching out a person instead of a hardcoded location
#define ELEV3_X		28.612
#define ELEV3_Y		2.641
#define ELEV2_X		100.0
#define ELEV2_Y		0.0

// ************************** Actions ************************** //
bool move_to(double x, double y){
	// Define a client for to send goal requests to the move_base server through a SimpleActionClient.
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client("move_base", true);
	ROS_INFO("Waiting for the move_base action server to come up.");
    while(!action_client.waitForServer(ros::Duration(5.0)));
    ROS_INFO("Server active.");

   move_base_msgs::MoveBaseGoal target;

   // Frame parameters.
   target.target_pose.header.frame_id = "map";	// absolute position on map
   target.target_pose.header.stamp = ros::Time::now();

   // Target location: (x,y,0.0)
   target.target_pose.pose.position.x =  x;
   target.target_pose.pose.position.y =  y;
   target.target_pose.pose.position.z =  0.0;
   // Target orientation: Heading of 0.0 (arbitrary)
   target.target_pose.pose.orientation.x = 0.0;
   target.target_pose.pose.orientation.y = 0.0;
   target.target_pose.pose.orientation.z = 0.0;
   target.target_pose.pose.orientation.w = 1.0;

   // Send target to move_base server.
   std::cout << "Sending target location...";
   action_client.sendGoal(target);
   std::cout << " Sent." << std::endl;

   // NOTE: BLOCKING!
   action_client.waitForResult();

   if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("You have reached the target.");
      my_state == WAIT_ELEV
      return true;
   }
   else{
      ROS_INFO("The robot failed to reach the target.");
      return false;
   }
}


// *************************** Main ************************** //
int main (int argc, char** argv) 
{
    // Initialize ROS.
	ros::init(argc, argv, "map_navigation_node");
    ros::NodeHandle nh;
	//ros::Subscriber pc_sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, pc_callback);
  	//ros::Publisher 	vel_sub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
  	ros::Rate loop_rate(10);
  	geometry_msgs::Twist T;

    // Setup state variables.
    //active = true; 
    my_state = TO_ELEV;
  
  	// Main loop.
  	while(ros::ok()) {
    	if (active) {
        	if (my_state == TO_ELEV ) 			{ move_to(ELEV3_X, ELEV3_Y); }
        	else if (my_state == WAIT_ELEV )	{  }
        	else if (my_state == ENTER_ELEV )	{  }
        	else if (my_state == ON_ELEV )		{}
        	else if (my_state == EXIT_ELEV )	{}
        	else if (my_state == TO_TARGET )	{}
        	else if (my_state == AWAIT_INPUT )	{}
    	}
    	ros::spinOnce();
  		loop_rate.sleep();
  	}
}

