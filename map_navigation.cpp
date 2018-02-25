#include <kobuki_msgs/BumperEvent.h> 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
//#include <cmvision/Blobs.h>
#include <stdio.h>
#include <ctype.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// ROS functionalities
//ros::Publisher pub;

//************Things for sound *********//
/*
from kobuki_msgs.msg import Sound
self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound)
sound = Sound()
sound.value = 6 
self.sound_pub.publish(sound)
*/

// *********** Global Variables ***************//
geometry_msgs::Twist T;
double Z_MIN = 0.0;

// *********************** State Variables ************************ //
//bool active;
enum State { TO_ELEV, WAIT_ELEV, ENTER_ELEV, ON_ELEV, EXIT_ELEV, TO_TARGET, AWAIT_INPUT };
State my_state;

// ************************** Targets ************************** //

#define HOME_X 		0.398
#define HOME_Y 		0.394
#define DELIVERY_X 	50.0	// you'll want to make DELIVERY_X and DELIVER_Y not '#define's 
#define DELIVERY_Y 	50.0		// for if you're searching out a person instead of a hardcoded location
#define ELEV3L_X	26.873
#define ELEV3L_Y	5.713
#define ELEV3R_X    30.043
#define ELEV3R_Y    6.352
// vv NOT MEASURED YET vv
#define ELEV2L_X	30.0
#define ELEV2L_Y	5.0
#define ELEV2R_X    25.0
#define ELEV2R_Y    5.0
#define Z_THRESH    5.0


// ************** Callbacks ******************************* //

/************************************************************
 * Function Name: PointCloud_Callback
 * Parameters: const PointCloud::ConstPtr
 * Returns: void
 *
 * Description: This is the callback function of the PointCloud
 *              topic, flags when an object is below the threshhold 
 ***********************************************************/
void pc_callback (const PointCloud::ConstPtr& cloud){

        unsigned int n = 0;
        int s,t;
        double min_z = 100, x, y;
        int y_point = 0;
        double ZTHRESH = .5; 
        std::vector<double> PCL_closest_points;
        std::vector<double> PCL_closest_points_x;
        std::vector<double> PCL_closest_points_y;
        PCL_closest_points.clear();
        PCL_closest_points_x.clear();
        PCL_closest_points_y.clear();

        Z_MIN = 100;
        //Iterate through all the points in the image
        //Convert from pcl to cm
        for(int k = 0; k < 240; k++){ // 0, 240
            for(int i = 30; i < 610; i++){ // 0, 640
                const pcl::PointXYZ & pt=cloud->points[640*(180+k)+(i)];
                if((pt.z < ZTHRESH)){
                    PCL_closest_points_x.push_back(i);
                    PCL_closest_points.push_back(pt.z);    
                    //Find min z
                    if(pt.z < Z_MIN){    
                        Z_MIN = pt.z;
                    }
                }
            }
        }
        std::cout << Z_MIN << std::endl;
}

// ************************** Actions ************************** //


// Uses SLAM to go to a (x,y) location on the provided map.
bool move_slam(double x, double y){
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
      return true;
   }
   else{
      ROS_INFO("The robot failed to reach the target.");
      return false;
   }
}

enum HRI_request {
    PRESS_DOWN,
    PRESS_UP,
    FLOOR_2,
    FLOOR_3,
    WHICH_ELEV,
    COMPLETE_DELIVERY
};

// This method is used to request assistance from human passersby for
// tasks such as pressing elevator buttons, or completing the delivery.
void request_help(HRI_request goal, char& human_input){
    //*human_input = 'o';
    switch(goal){
        case PRESS_DOWN : 
            std::cout << "Please press the 'DOWN' elevator button. Hit enter when you have pressed the button." << std::endl;
            break;
        case PRESS_UP :
            std::cout << "Please press the 'UP' elevator button. Hit enter when you have pressed the button." << std::endl;
            break;
        case WHICH_ELEV :
            std::cout << "Which elevator is coming? Please enter 'l' or 'r': ";
            break;
        default : 
            std::cout << "ERROR: Unexpected HRI_request input." << std::endl; 
    }

    // the below is blocking
    std::cin >> human_input;

    std::cout << std::endl;
    //return human_input != 'o';   // this return statement would be for waiting for a response, but std::cin >> human_input is blocking, so there's no need for it atm
}

// Method for waiting for elevator doors to open, moving onto elevator, and returns whether or not it was successful.


void rotate(ros::Publisher velocityPublisher, double magnitude){
    // turn magnitude degrees
    std::cout << "Rotating...";
    T.angular.z = magnitude;
    T.linear.x = 0.0;
    velocityPublisher.publish(T);
    std::cout << "Rotation published." << std::endl;
}


void move_forward(ros::Publisher velocityPublisher, double distance){
    std::cout << "Moving forward onto elevator" << std::endl;    
    T.angular.z = 0;
    T.linear.x = distance;
    velocityPublisher.publish(T);
}


// Rotate toward elevator, wait until door opens (BLOCKING), and then move forward into elevator
// and turn to face door
void go_through_door(ros::Publisher velocityPublisher, double distance_to_elevator){
    std::cout << "Waiting for door to open..."; 
    // wait until door is open -- BLOCKING
    while(Z_MIN < Z_THRESH); 
    std::cout << " Door open." << std::endl;
    
    std::cout << "Moving onto elevator... " << std::endl;
    // move into elevator
    move_forward(velocityPublisher, distance_to_elevator);
}





// *************************** Main ************************** //
int main (int argc, char** argv) 
{
    // Initialize ROS.
	ros::init(argc, argv, "map_navigation_node");
    ros::NodeHandle nh;
	ros::Subscriber pc_sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, pc_callback);
  	ros::Publisher 	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
  	ros::Rate loop_rate(10);


    // Setup state variables.
    //active = true; 
    my_state = WAIT_ELEV;
  
  	// Main loop.
  	while(ros::ok()) {
        char input;
        switch (my_state) {
            case TO_ELEV : 
                if(move_slam(ELEV3L_X, ELEV3L_Y)){
                    my_state = WAIT_ELEV;
                }
                break;
            case  WAIT_ELEV : 
                request_help(PRESS_DOWN, input);
                my_state = ENTER_ELEV;
                break;
            case ENTER_ELEV :
                request_help(WHICH_ELEV, input);
                // bot should already be in front of left elevator; move to right elev if needed
                if ( tolower(input) == 'r') {
                    std::cout << "Moving to right elevator..." << std::endl;
                    move_slam(ELEV3R_X, ELEV3R_Y); 
                }
                else std::cout << "At correct (left) elevator." << std::endl;
                
                // rotate to elevator
                rotate(vel_pub, 0.8);

                go_through_door(vel_pub, 5.0);

                // rotate back toward door
                rotate(vel_pub, -0.8);
                my_state = ON_ELEV;
                
                break;
            case ON_ELEV :
                break;
            case EXIT_ELEV :
                break;
            case TO_TARGET :
                break;
            case AWAIT_INPUT :
                break;
            default :
                std::cout << "Unexpected state reached." << std::endl;
                break;
        }
        ros::spinOnce();
  		loop_rate.sleep();
    }   
}
