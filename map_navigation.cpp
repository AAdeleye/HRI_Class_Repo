
/******************************************************************
*                                                                 *
*                        HRI: CSE 276B/291F                       * 
*                               HW 10                             *
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
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <vector>
//#include <pcl/point_types.h>
#include <time.h>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <boost/thread/thread.hpp>
#include "sound_play/sound_play.h"
#include <stdlib.h>

// ************************************************************* //
// *************                                   ************* //
// *****               TARGETS/OTHER MACROS                ***** //
// *************                                   ************* //
// ************************************************************* //

// Lobby
#define LOBBY_X     25.464
#define LOBBY_Y     0.2786
#define LOBBY_OZ    -0.373
#define LOBBY_OW     0.927
// Class location
#define CLASS_X     0.398
#define CLASS_Y     0.394
#define CLASS_OZ    0.0
#define CLASS_OW    0.0

// ************************************************************* //
// *************                                   ************* //
// *****         GLOBAL VARIABLES/TYPE DEFINITIONS         ***** //
// *************                                   ************* //
// ************************************************************* //

//            <<  Type Definitions  >>

//   State Variables. 
enum State { 
    TO_LOBBY, 
    WAIT_ACKNOWLEDGE,
    WAIT_FOLLOW, 
    TO_CLASS,
    AWAIT_INPUT,
    ERROR
};

//   Human-Robot Interaction request types.
enum HRI_request {
    ACKNOWLEDGE,
    ASK_FOLLOW,
    CONFIRM_RETURNED
};

//           <<   Global Variables  >>
double Z_MIN = 0.0;     // minimum distance from pointcloud data, updated in pc_callback
geometry_msgs::Twist T;
State my_state;
bool input_received;
// Whether or not to use each attention-getting modality
bool use_beeps;
bool use_voice;
bool use_rotate;

// ************************************************************* //
// ****************                             **************** //
// *****               AUXILIARY FUNCTIONS                 ***** //
// ****************                             **************** //
// ************************************************************* //

/*********************************************************************************************/
// Rotate at a desired angular_velocity for a desired duration.
void rotate(ros::Publisher velocityPublisher, double angular_vel, double duration_in_seconds){
    // turn magnitude degrees
    //std::cout << "Publishing...";
    T.angular.z = angular_vel;
    T.linear.x = 0.0;
    
    int max_count = (int) (duration_in_seconds * 10);
    ros::Rate rate(10); // publish at 10 Hz
    for(int i = 0; i < max_count; i++){
        rate.sleep();
        velocityPublisher.publish(T);
    }
    //std::cout << " Published." << std::endl;
    
    T.angular.z = 0.0;
    velocityPublisher.publish(T);
    rate.sleep();
    //std::cout << " Exiting rotate() method." << std::endl;
}

/*********************************************************************************************/
// Move a specific distance straight forward.
void move_forward(ros::Publisher velocityPublisher, double linear_vel, double duration_in_seconds){
    //std::cout << "Creating move forward command...";    
    T.angular.z = 0;
    T.linear.x = linear_vel;

    ros::Rate rate(10);
    //std::cout << " Publishing..." << std::endl << std::flush;
    int max_count = (int) (duration_in_seconds * 10);
    for(int i = 0; i < max_count; i++){
        velocityPublisher.publish(T);
        rate.sleep();
    }

    // Publish stop command.
    T.linear.x = 0.0;
    velocityPublisher.publish(T);
    rate.sleep();

}

/*********************************************************************************************/
// Cause the turtlebot to emit ascending beeps for approximately 1 second.
void make_sound(ros::Publisher soundPublisher){
    kobuki_msgs::Sound sound;
    sound.value = 6;
    soundPublisher.publish(sound);
}

/*********************************************************************************************/
// Cause the turtlebot to say the string passed
void make_voice(std::string &output_string){
    // Check that the processor is available
    //std::cout << "Checking if processor is available..." << std::endl;
    if (system(NULL)) {
        //puts ("Ok");
        // Create the command to pass
        std::string full_command_str = "rosrun sound_play say.py \"" + output_string + "\"";
        const char *full_command = full_command_str.c_str();
        // Send the command
        system(full_command);
    } else  std::cout<< "FAILURE" << std::endl;
}


/*********************************************************************************************/
// Sets the global attention-getting boolean variables.
void set_modalities(std::string &modalities) {
    // If "b" was entered, use beeps
    std::size_t found = modalities.find("b");
    use_beeps = (found != std::string::npos);
    // If "v" was entered, use voice
    found = modalities.find("v");
    use_voice = (found != std::string::npos);
    // If "r" was entered, use rotation
    found = modalities.find("r");
    use_rotate = (found != std::string::npos);
}


/*********************************************************************************************/
// Print error message from string in <msg>.
void error_msg(std::string &msg){
    std::cout << "TLSTC has entered ERROR state with message: " << msg << std::endl;
}

// ****************                             **************** //
// *****                   MAIN ACTIONS                    ***** //
// ****************                             **************** //
// ************************************************************* //

/************************************************************
 * Function Name: move_slam
 * Parameters: double x, double y
 * Returns: bool
 *
 * Description: Moves to an (x,y) target location on the linked map
 *              using SLAM. Once the target location is sent to the 
 *              move_base server, the movement is BLOCKING.
 * ***********************************************************/
bool move_slam(double x, double y, double orient_z, double orient_w){

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
    // Target orientation: Heading of (0.0, 0.0, orient_z, orient_w)
    target.target_pose.pose.orientation.x = 0.0;
    target.target_pose.pose.orientation.y = 0.0;
    target.target_pose.pose.orientation.z = orient_z;
    target.target_pose.pose.orientation.w = orient_w;

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
    else {
       ROS_INFO("The robot failed to reach the target.");
       return false;
    }
}

// Beeping thread method.
void beep_loop(ros::Publisher soundPublisher, double delay_between_beeps) {
    ros::Rate beep_rate(1.0/delay_between_beeps);
    while(ros::ok() && !input_received) {
        make_sound(soundPublisher);
        beep_rate.sleep();
        ros::spinOnce();
    }
}

// Voice thread method.
void voice_loop(std::string &output_string, double delay) {
    ros::Rate voice_rate(1.0/delay);
    while(ros::ok() && !input_received) {
        make_voice(output_string);
        voice_rate.sleep();
        ros::spinOnce();
    }
}

// Rotate thread method.
void rotate_loop(ros::Publisher velocityPublisher, double delay) {
    ros::Rate rotate_rate(1.0/delay);
    while(ros::ok() && !input_received) {
        rotate(velocityPublisher, 0.6, 1.5);
        rotate(velocityPublisher, -0.6, 1.5);
        rotate_rate.sleep();
        ros::spinOnce();    
    } 
}


// This method is used to request assistance from human passersby for
// tasks such as pressing elevator buttons, or completing the delivery.
// Updates the char& human_input pointer with the entered character.
double request_help(HRI_request goal, std::string& response, ros::Publisher soundPublisher, ros::Publisher velocityPublisher){
    std::clock_t start_time = std::clock(); 
    switch(goal){
        case ACKNOWLEDGE : 
            std::cout << "Type any character and then hit enter to acknowledge that you have seen me." << std::endl << std::flush;
            break;
        case ASK_FOLLOW :
            std::cout << "Great! Please follow me. Type any character and then hit enter to confirm." << std::endl << std::flush;
            break;
        default : 
            std::cout << "ERROR: Unexpected HRI_request input." << std::endl; 
    }
   
    // Default initialization of threads are empty
    boost::thread beep_thread;   
    boost::thread voice_thread;
    boost::thread rotate_thread; 
    // Spawn desired attention-getting threads.
    input_received = false;
    if (use_beeps)
        beep_thread = boost::thread(beep_loop, soundPublisher, 4);
    if (use_voice)
        voice_thread = boost::thread(voice_loop, (std::string) "Hello. Please check my screen.", 10);
    if (use_rotate)
        rotate_thread = boost::thread(rotate_loop, velocityPublisher, 4);

    // Wait for human input.
    std::cin >> response;
    input_received = true;
    double duration = (std::clock() - start_time) / (double) CLOCKS_PER_SEC;

    // Close threads.
    if (use_beeps)
        beep_thread.join();
    if (use_voice)
        voice_thread.join();
    if (use_rotate)
        rotate_thread.join();
    
    return duration;
    /*      CODE FOR INPUT BEING ITS OWN THREAD
    ros::Rate beep_rate(0.2); // Beep once every 5 seconds.
    // Beep until there is new input. BLOCKING.
    while( !input_received ) {
    // NOTE: std::string.compare(str2) returns 0 (false) if strings are identical,
    // so !std::string.compare(str2) returns 1 (true) when they match
        make_sound(soundPublisher);
        beep_rate.sleep();
    }
    */
}

// *************************** Main ************************** //
int main (int argc, char** argv) 
{
    // Initialize ROS.
	ros::init(argc, argv, "map_navigation_node");
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    // Publishers and subscribers.
    ros::Publisher vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
    ros::Publisher sound_pub = nh->advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound",1000);

    // Setup state variables.
    my_state = TO_LOBBY;
    // Determine which attention-getting modalities to employ
    std::string modes = argv[1];
  	set_modalities(modes);

    // For time keeping
    double duration_1;
    double duration_2;
    ros::Rate loop_rate(10);
    // Main loop.
    bool done_flag = false;
  	while(ros::ok() && !done_flag) {
        std::string response;
        switch (my_state) {
           /*******************************************************************************/  
            case TO_LOBBY : 
                if(move_slam(LOBBY_X, LOBBY_Y, LOBBY_OZ, LOBBY_OW)){
                    my_state = WAIT_ACKNOWLEDGE;
                }
                break;

           /*******************************************************************************/  
            case  WAIT_ACKNOWLEDGE : 
                duration_1 = request_help(ACKNOWLEDGE, response, sound_pub, vel_pub);
                my_state = WAIT_FOLLOW;
                break;

           /*******************************************************************************/  
            case WAIT_FOLLOW :
                duration_2 = request_help(ASK_FOLLOW, response, sound_pub, vel_pub);
                my_state = TO_CLASS;
                break;

           /*******************************************************************************/  
            case TO_CLASS :
                if(move_slam(CLASS_X, CLASS_Y, CLASS_OZ, CLASS_OW)){
                    //my_state = AWAIT_INPUT;
                    done_flag = true;
                }
                break;
           
           /*******************************************************************************/  
           /*case AWAIT_INPUT :
                request_help(CONFIRM_RETURNED, response, sound_pub);
                done_flag = true;
                break;
*/
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

    std::cout << "Duration 1: " << duration_1 << std::endl;
    std::cout << "Duration 2: " << duration_2 << std::endl;
    std::cout << "Complete. Shutting down program, goodbye!" << std::endl << std::flush;
}
