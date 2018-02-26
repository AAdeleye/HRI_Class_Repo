
/******************************************************************
*                                                                 *
*                        HRI: CSE 276B/291F                       * 
*                               HW 9                              *
*                           Winter 2018                           *
*              Sanmi Adeleye, Andi Frank, Alyssa Kubota           *
*                                                                 *
*                       Professor: Laurel Riek                    *
*                       TA: Angelique Taylor                      *
*                                                                 *
******************************************************************/


// ************************************************************* //
// *************                                   ************* //
// *****                    INCLUDES                       ***** //
// *************                                   ************* //
// ************************************************************* //


#include <kobuki_msgs/BumperEvent.h> 
#include <kobuki_msgs/Sound.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
//#include <cmvision/Blobs.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <boost/thread/thread.hpp>

// ************************************************************* //
// *************                                   ************* //
// *****               TARGETS/OTHER MACROS                ***** //
// *************                                   ************* //
// ************************************************************* //

// vv MEASURED vv
#define HOME_X 		0.398
#define HOME_Y 		0.394
#define ELEV3L_X	25.307
#define ELEV3L_Y	0.903
#define ELEV3L_IN_X 25.367
#define ELEV3L_IN_Y 3.468
#define ELEV3R_X    27.053
#define ELEV3R_Y    1.007
#define ELEV3R_IN_X 27.192
#define ELEV3R_IN_Y 3.102
// vv NOT MEASURED YET vv
#define DELIVERY_X 	50.0	// you'll want to make DELIVERY_X and DELIVER_Y not '#define's 
#define DELIVERY_Y 	50.0		// for if you're searching out a person instead of a hardcoded location
#define ELEV2L_X	30.0
#define ELEV2L_Y	5.0
#define ELEV2R_X    25.0
#define ELEV2R_Y    5.0
#define Z_THRESH    5.0


// ************************************************************* //
// *************                                   ************* //
// *****         GLOBAL VARIABLES/TYPE DEFINITIONS         ***** //
// *************                                   ************* //
// ************************************************************* //


geometry_msgs::Twist T;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
double Z_MIN = 0.0;

//       State Variables 

// bool active;
enum State { 
    TO_ELEV, 
    WAIT_ELEV, 
    ENTER_ELEV, 
    ON_ELEV, 
    EXIT_ELEV, 
    TO_TARGET,
    AWAIT_INPUT,
    ERROR
};
State my_state;

//      Human-Robot Interaction request types.
enum HRI_request {
    PRESS_DOWN,
    PRESS_UP,
    FLOOR_2,
    FLOOR_3,
    WHICH_ELEV,
    COMPLETE_DELIVERY
};


// ************************************************************* //
// ****************                             **************** //
// *****               AUXILIARY FUNCTIONS                 ***** //
// ****************                             **************** //
// ************************************************************* //


void rotate(ros::Publisher velocityPublisher, ros::Rate rate, double magnitude){
    // turn magnitude degrees
    std::cout << "Publishing...";
    T.angular.z = magnitude;
    T.linear.x = 0.0;
    
    std::cout << " Published." << std::endl;
    for(int i = 0; i < 32; i++){
        rate.sleep();
        velocityPublisher.publish(T);
        //ros::spinOnce();

    }
    T.angular.z = 0.0;
    velocityPublisher.publish(T);
    rate.sleep();
    std::cout << " Exiting rotate() method." << std::endl;
}

// Move a specific distance straight forward.
void move_forward(ros::Publisher velocityPublisher, ros::Rate rate, double distance){
    std::cout << "Creating move forward command...";    
    T.angular.z = 0;
    T.linear.x = distance;
    for(int i = 0; i < 65; i++){
        velocityPublisher.publish(T);
        rate.sleep();
    }
    T.linear.x = 0.0;
    velocityPublisher.publish(T);
    rate.sleep();
    std::cout << " Published." << std::endl;
}

void make_sound(ros::Publisher soundPublisher){
    kobuki_msgs::Sound sound;
    sound.value = 6;
    soundPublisher.publish(sound);
}

void error_msg(std::string &msg){
    std::cout << "TLSTC has entered ERROR state with message: " << msg << std::endl;
}



// ************************************************************* //
// ****************                             **************** //
// *****                    CALLBACKS                      ***** //
// ****************                             **************** //
// ************************************************************* //

/************************************************************
 * Function Name: pc_callback
 * Parameters: const PointCloud::ConstPtr
 * Returns: void
 *
 * Description: This is the callback function of the PointCloud
 *              topic, sets Z_MIN to the distance of the closest
 *              point to the bot.
 ************************************************************/
void pc_callback (const PointCloud::ConstPtr& cloud){
        std::cout << "Entered pc_callback() method." << std::endl;
        std::cout << std::flush;
        
        
        unsigned int n = 0;
        int s,t;
        double min_z = 100, x, y;
        int y_point = 0;
        double furthest_relevant_z = .5; 
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
                if((pt.z < furthest_relevant_z)){
                    PCL_closest_points_x.push_back(i);
                    PCL_closest_points.push_back(pt.z);    
                    //Find min z
                    if(pt.z < Z_MIN){    
                        Z_MIN = pt.z;
                    }
                }
            }
        }
        std::cout << "Lowest Z: " << Z_MIN << std::endl;
}


/************************************************************
 * Function Name: io_callback
 * Parameters: const std_msgs::String::ConstPtr& msg
 * Returns: void
 *
 * Description: This is the callback function of the /stdin 
 *              topic. It hears the input from a std::cin input
 *              and updates the global std_input variable.
 ************************************************************/
// TODO: synchronous?
std::string std_input;

void io_callback(const std_msgs::String::ConstPtr& msg){
    std_input = msg->data.c_str();
    // Print to terminal (for testing).
    std::cout << "Input heard: " << std_input << std::endl;
}




// ************************************************************* //
// ****************                             **************** //
// *****                   MAIN ACTIONS                    ***** //
// ****************                             **************** //
// ************************************************************* //

//      Things for sound
/*
from kobuki_msgs.msg import Sound
sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound)
sound = Sound()
sound.value = 6 
sound_pub.publish(sound)
*/

/************************************************************
 * Function Name: io_listener
 * Parameters: int rate
 * Returns: void
 *
 * Description: Listens to std::cin for input, and publishes 
 *              message to /stdin topic.
 * ***********************************************************/
void io_listener(int rate){
    
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    ros::Publisher io_pub = nh->advertise<std_msgs::String>("/stdin", 10);
    ros::Rate loop_rate(rate);
                
    while (ros::ok()) {
        std::string temp;
        // Wait for input and save to temp variable.
        std::cin >> temp;
        // Convert to lowercase for easier comparison.
        boost::algorithm::to_lower(temp);
        // Assign to msg and publish.
        std_msgs::String msg;
        msg.data = temp;
        io_pub.publish(msg);
        // Sleep and spin.
        loop_rate.sleep();
        ros::spinOnce();
    }
}


/************************************************************
 * Function Name: move_slam
 * Parameters: double x, double y
 * Returns: bool
 *
 * Description: Moves to an (x,y) target location on the linked map
 *              using SLAM. Once the target location is sent to the 
 *              move_base server, the movement is BLOCKING.
 * ***********************************************************/
bool move_slam(double x, double y){
	// Define a client for to send goal requests to the move_base server through a SimpleActionClient
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

// This method is used to request assistance from human passersby for
// tasks such as pressing elevator buttons, or completing the delivery.
// Updates the char& human_input pointer with the entered character.
//void request_help(HRI_request goal, char& human_input) {
void request_help(HRI_request goal, std::string& response, ros::Publisher soundPublisher){
    response = std_input; 
    
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

    ros::Rate beep_rate(1); // 1 Hz
    // Beep until there is new input. BLOCKING.
    while( !response.compare(std_input) ) {
    // NOTE: std::string.compare(str2) returns 0 (false) if strings are identical, so !std::string.compare(str2) returns 1 (true) when they match
        make_sound(soundPublisher);
        beep_rate.sleep();
    }
    response = std_input;
    std::cout << std::endl << std::flush;
    // Testing sound.
    //std::thread first make_sound(soundPublisher);
    //while ( cin.get(temp,100) != '\n')
    //End of sound testing

    //return human_input != 'o';   // this return statement would be for waiting for a response, but std::cin >> human_input is blocking, so there's no need for it atm
}


// Rotate toward elevator, wait until door opens (BLOCKING), and then move forward into elevator
// and turn to face door
bool wait_and_enter(ros::Publisher velocityPublisher, ros::Rate rate, double distance_to_elevator){
    // rotate toward elevator door
    // rotate(velocityPublisher, rate, 0.8);
    
    std::cout << "Waiting for door to open..."; 
    std::cout << std::flush;
    if(move_slam(ELEV3L_IN_X, ELEV3L_IN_Y)){
        std::cout << "Inside elevator." << std::endl << std::flush;
    } else {
        std::cout << "Didn't make it inside!" << std::endl << std::flush;
    }
    // wait until door is open -- BLOCKING
    //do {
    //    std::cout << "Inside do-while loop...";
    //    ros::spinOnce();
    //    std::cout << " ros::spinOnce(); completed...";
    //    rate.sleep();
    //    std::cout << " rate.sleep(); completed." << std::endl << std::flush;
    // }
    //while(Z_MIN < Z_THRESH); 
    //std::cout << " Door open." << std::endl;
    // move into elevator
    //move_forward(velocityPublisher, rate, distance_to_elevator);
    // rotate back to face door
    //rotate(velocityPublisher, rate, 0.8);
    //rotate(velocityPublisher, rate, 0.8);
    //std::cout << " Finished spinning" << std::endl;

    return true;
}




// *************************** Main ************************** //
int main (int argc, char** argv) 
{
    // Initialize ROS.
	ros::init(argc, argv, "map_navigation_node");
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
	ros::Subscriber pc_sub = nh->subscribe<PointCloud>("/camera/depth/points", 1, pc_callback);
    ros::Subscriber io_sub = nh->subscribe<std_msgs::String>("/stdin", 1, io_callback);
    ros::Publisher 	vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
    ros::Publisher sound_pub = nh->advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound",1000);
  	ros::Rate loop_rate(10);

    // Setup state variables.
    //active = true; 
    my_state = TO_ELEV;
  
    // Spawn std::cin input listener thread.
    boost::thread io_thread(io_listener, 10);


  	// Main loop.
  	while(ros::ok()) {
        std::string response;
        switch (my_state) {
           /*******************************************************************************/  
            case TO_ELEV : 
                if(move_slam(ELEV3L_X, ELEV3L_Y)){
                    my_state = WAIT_ELEV;
                }
                break;

           /*******************************************************************************/  
            case  WAIT_ELEV : 
                //request_help(PRESS_DOWN, input);
                request_help(PRESS_DOWN, response, sound_pub);
                my_state = ENTER_ELEV;
                break;

           /*******************************************************************************/  
            case ENTER_ELEV :
                
                // Ask fpr help to determine which elevator is open.
                //request_help(WHICH_ELEV, input);
                request_help(WHICH_ELEV, response, sound_pub);
                
                // Move to correct elevator
                // bot should already be in front of left elevator; move to right elev if needed

                
                if ( !response.compare("r") )  { 
                // NOTE: std::string.compare(str2) returns 0 (false) if strings are identical, so !std::string.compare(str2) returns 1 (true) when they match
                    std::cout << "Moving to right elevator..." << std::endl;
                    //move_slam(ELEV3R_X, ELEV3R_Y); 
                    if (move_slam(ELEV3R_IN_X, ELEV3R_IN_Y)) {
                        std::cout << "Inside right elevator." << std::endl << std::flush;
                        my_state = ON_ELEV;
                    } else {
                        my_state = ERROR;
                    }
                }
                else {
                    std::cout << "At correct (left) elevator." << std::endl;
                    if (move_slam(ELEV3L_IN_X, ELEV3L_IN_Y)) {
                        std::cout << "Inside left elevator." << std::endl << std::flush;
                        my_state = ON_ELEV;
                    } else {
                        my_state = ERROR;
                    }
                }

                // Turn to elevator, wait for doors to open, then enter, and face doors again
                    // NOTE: Right now, wait_and_enter ALWAYS returns true, but it may be worth
                    // adding a case in which it returns false if it doesn't succeed in entering
                    // the elevator.
                //if ( wait_and_enter(vel_pub, loop_rate, 0.3) ) my_state = ON_ELEV;
                //else
                //                                my_state = ERROR;
                
                break;

           /*******************************************************************************/  
            case ON_ELEV :
                break;

           /*******************************************************************************/  
            case EXIT_ELEV :
                break;

           /*******************************************************************************/  
            case TO_TARGET :
                break;

           /*******************************************************************************/  
            case AWAIT_INPUT :
                break;

           /*******************************************************************************/  
            case ERROR :
                break;
            
           /*******************************************************************************/  
            default :
                std::cout << "Unexpected state reached." << std::endl;
                break;
        }
        ros::spinOnce();
  		loop_rate.sleep();
        vel_pub.publish(T);
    }   

    // Wait for io_thread() to end. 
    io_thread.join();
}
