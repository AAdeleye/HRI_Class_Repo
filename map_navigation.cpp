
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
//#include <mutex>

// ************************************************************* //
// *************                                   ************* //
// *****               TARGETS/OTHER MACROS                ***** //
// *************                                   ************* //
// ************************************************************* //

//       vv MEASURED vv
// Outside left elevator
#define ELEV3L_X	    28.43
#define ELEV3L_Y	    2.17
#define ELEV3L_OZ       0.703   // NOTE: OZ and OW are heading parameters
#define ELEV3L_OW       0.710
// Outside right elevator
#define ELEV3R_X        30.97
#define ELEV3R_Y        2.38
#define ELEV3R_OZ       0.699
#define ELEV3R_OW       0.714
// Delivery location
#define DELIVERY_X 	24.986	// you'll want to make DELIVERY_X and DELIVER_Y not '#define's 
#define DELIVERY_Y 	28.211  // for if you're searching out a person instead of a hardcoded location
#define DELIVERY_OZ 0.633
#define DELIVERY_OW 0.774

//      vv NOT MEASURED YET vv
// Start location.
#define HOME_X 		    0.398
#define HOME_Y 		    0.394
// Inside left elevator
#define ELEV3L_IN_X     29.867
#define ELEV3L_IN_Y     3.468
#define ELEV3L_IN_OZ    -0.689
#define ELEV3L_IN_OW    0.724
// Inside right elevator
#define ELEV3R_IN_X     31.692
#define ELEV3R_IN_Y     3.102
#define ELEV3R_IN_OZ    -0.721
#define ELEV3R_IN_OW    0.692
// Second floor elevators
#define ELEV2L_X	30.0
#define ELEV2L_Y	5.0
#define ELEV2R_X    25.0
#define ELEV2R_Y    5.0
// Pointcloud Z threshold for elevator being considered open
#define Z_THRESH    5.0


// ************************************************************* //
// *************                                   ************* //
// *****         GLOBAL VARIABLES/TYPE DEFINITIONS         ***** //
// *************                                   ************* //
// ************************************************************* //

//            <<  Type Definitions  >>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//   State Variables. 
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

//   Human-Robot Interaction request types.
enum HRI_request {
    PRESS_DOWN,
//    PRESS_UP,
    DOOR_OPEN,
//    FLOOR_2,
//    FLOOR_3,
    WHICH_ELEV,
    CONFIRM_DELIVERY
};

//           <<   Global Variables  >>
double Z_MIN = 0.0;     // minimum distance from pointcloud data, updated in pc_callback
geometry_msgs::Twist T;
State my_state;
//std::mutex mx;
//std::string std_input;
bool input_received;

// ************************************************************* //
// ****************                             **************** //
// *****               AUXILIARY FUNCTIONS                 ***** //
// ****************                             **************** //
// ************************************************************* //

/*********************************************************************************************/
// Rotate at a desired angular_velocity for a desired duration.
void rotate(ros::Publisher velocityPublisher, double angular_vel, double duration_in_seconds){
    // turn magnitude degrees
    std::cout << "Publishing...";
    T.angular.z = angular_vel;
    T.linear.x = 0.0;
    
    int max_count = (int) (duration_in_seconds * 10);
    ros::Rate rate(10); // publish at 10 Hz
    for(int i = 0; i < max_count; i++){
        rate.sleep();
        velocityPublisher.publish(T);
        //ros::spinOnce();

    }
    std::cout << " Published." << std::endl;
    
    T.angular.z = 0.0;
    velocityPublisher.publish(T);
    rate.sleep();
    std::cout << " Exiting rotate() method." << std::endl;
}

/*********************************************************************************************/
// Move a specific distance straight forward.
void move_forward(ros::Publisher velocityPublisher, double linear_vel, double duration_in_seconds){
    std::cout << "Creating move forward command...";    
    T.angular.z = 0;
    T.linear.x = linear_vel;

    ros::Rate rate(10);
    std::cout << " Publishing..." << std::endl << std::flush;
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
// Print error message from string in <msg>.
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

/*
void io_callback(const std_msgs::String::ConstPtr& msg){
    mx.lock();
    std_input = msg->data.c_str();
    // Print to terminal (for testing).
    std::cout << "Input heard: " << std_input << std::endl;
    mx.unlock();
}

*/


// ************************************************************* //
// ****************                             **************** //
// *****                   MAIN ACTIONS                    ***** //
// ****************                             **************** //
// ************************************************************* //


/************************************************************
 * Function Name: io_listener
 * Parameters: int rate
 * Returns: void
 *
 * Description: Listens to std::cin for input, and publishes 
 *              message to /stdin topic.
 * ***********************************************************/
/*
void io_listener(int rate){
    
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    ros::Publisher io_pub = nh->advertise<std_msgs::String>("/stdin", 10);
    ros::Rate loop_rate(rate);
                
    while (ros::ok()) {
        std::string temp;
        // Wait for input and save to temp variable.
        mx.lock(); // Needed?
        std::cin >> temp;
        input_received = true;
        mx.unlock();
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
*/

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
    // Target orientation: Heading of 0.0 (arbitrary)
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
    else{
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


// This method is used to request assistance from human passersby for
// tasks such as pressing elevator buttons, or completing the delivery.
// Updates the char& human_input pointer with the entered character.
//void request_help(HRI_request goal, char& human_input) {
void request_help(HRI_request goal, std::string& response, ros::Publisher soundPublisher){
    
    switch(goal){
        case PRESS_DOWN : 
            std::cout << "Please press the 'DOWN' elevator button. Type any character and then hit enter when you have pressed the button. ";
            break;
/*        case PRESS_UP :
            std::cout << "Please press the 'UP' elevator button. Hit enter when you have pressed the button." << std::endl;
            break;
*/
        case WHICH_ELEV :
            std::cout << "Which elevator is coming? Please enter 'l' or 'r': ";
            break;
        case DOOR_OPEN :
            std::cout << "Please type any character and hit enter when the door is open. "; 
            break;
        case CONFIRM_DELIVERY :
            std::cout << "Please type 'y' and hit enter to complete the delivery. ";
            break;
        default : 
            std::cout << "ERROR: Unexpected HRI_request input." << std::endl; 
    }
    
    // Spawn beeping thread.
    input_received = false;
    boost::thread beep_thread(beep_loop, soundPublisher, 4);
    // Wait for human input.
    //response = std::cin.getline();
    std::cin >> response;
    input_received = true;
    std::cout << std::endl << std::flush;
    // Close beep_thread.
    beep_thread.join();


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
    //response = std_input;
    std::cout << std::endl << std::flush;
}


// Rotate toward elevator, wait until door opens (BLOCKING), and then move forward into elevator
// and turn to face door
bool wait_and_enter(ros::Publisher velocityPublisher, ros::Rate rate, double distance_to_elevator){
    // rotate toward elevator door
    // rotate(velocityPublisher, rate, 0.8);
    
    std::cout << "Waiting for door to open..."; 
    std::cout << std::flush;
    if(move_slam(ELEV3L_IN_X, ELEV3L_IN_Y, ELEV3L_IN_OZ, ELEV3L_IN_OW)){
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
    // double linear_vel = 0.4;
    // double duration_secs = 5.0;
    //move_forward(velocityPublisher, linear_vel, duration_secs);
    // rotate back to face door
    //rotate(velocityPublisher, 0.8, 3);
    //rotate(velocityPublisher, 0.8, 3);
    //std::cout << " Finished spinning" << std::endl;

    return true;
}

// Move into elevator and turn to face doors. In theory will eventually return true
// only if it detects it has successfully entered the elevator, buuuut for now it
// just always returns true.
bool thru_doorway(ros::Publisher velocityPublisher){
        std::cout << " Moving inside elevator.." << std::endl;
        // move into elevator
        double linear_vel = 0.5;
        double duration_secs = 10.0;
        move_forward(velocityPublisher, linear_vel, duration_secs);
        //rotate back to face door
        std::cout << "Rotating...";
        rotate(velocityPublisher, 0.8, 6);
        std::cout << " Finished rotation." << std::endl;
        std::cout << std::flush;
        return true;
}

// Move to the correct elevator using SLAM, based off human input passed in through param <response>.
// Returns true if it successfully enters elevator, and false if it does not.
bool to_correct_elev(std::string response, ros::Publisher velocityPublisher){
    if ( !response.compare("r") )  { 
     // NOTE: std::string.compare(str2) returns 0 (false) if strings are identical,
     // so !std::string.compare(str2) returns 1 (true) when they match
            
        std::cout << "Moving to right elevator..." << std::endl;
        if ( move_slam(ELEV3R_X, ELEV3R_Y, ELEV3R_OZ, ELEV3R_OW) ) {
            return true;
        }
        /*if ( move_slam(ELEV3R_X, ELEV3R_Y, ELEV3R_OZ, ELEV3R_OW) ) {
            std::cout << "Inside right elevator." << std::endl << std::flush;
            return true;
        }*/
    } 
    else {          
        std::cout << "At correct (left) elevator." << std::endl;

        return true;
        /*
        if ( move_slam(ELEV3L_X, ELEV3L_Y, ELEV3L_OZ, ELEV3L_OW) ) {
            std::cout << "Inside left elevator." << std::endl << std::flush;
            return true;
        }*/
    }
    return false;
}

// *************************** Main ************************** //
int main (int argc, char** argv) 
{
    // Initialize ROS.
	ros::init(argc, argv, "map_navigation_node");
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    // Publishers and subscribers.
    ros::Subscriber pc_sub = nh->subscribe<PointCloud>("/camera/depth/points", 1, pc_callback);
    //ros::Subscriber io_sub = nh->subscribe<std_msgs::String>("/stdin", 1, io_callback);
    ros::Publisher 	vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
    ros::Publisher sound_pub = nh->advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound",1000);

    // Setup state variables.
    my_state = TO_ELEV;
  
    // Spawn std::cin input listener thread.
    //boost::thread io_thread(io_listener, 10);

  	ros::Rate loop_rate(10);
  	// Main loop.
    bool done_flag = false;
  	while(ros::ok() && !done_flag) {
        std::string response;
        switch (my_state) {
           /*******************************************************************************/  
            case TO_ELEV : 
                if(move_slam(ELEV3L_X, ELEV3L_Y, ELEV3L_OZ, ELEV3L_OW)){
                    my_state = WAIT_ELEV;
                }
                break;

           /*******************************************************************************/  
            case  WAIT_ELEV : 
                request_help(PRESS_DOWN, response, sound_pub);
                my_state = ENTER_ELEV;
                break;

           /*******************************************************************************/  
            case ENTER_ELEV :
                
                // Ask for help to determine which elevator is open.
                request_help(WHICH_ELEV, response, sound_pub);
                
                // Move to correct elevator
                   // NOTE: bot should already be in front of left elevator, and will
                   // move to right elev if needed.
                if ( to_correct_elev(response, vel_pub) ) {
                    if ( thru_doorway(vel_pub) )      my_state = ON_ELEV;
                }
                else                                my_state = ERROR;
                
                break;

           /*******************************************************************************/  
            case ON_ELEV :
                std::cout << "Inside state ON_ELEV" << std::endl << std::flush;
                request_help(DOOR_OPEN, response, sound_pub);
                my_state = EXIT_ELEV;
                break;

           /*******************************************************************************/  
            case EXIT_ELEV :
                if( thru_doorway(vel_pub) ) my_state = TO_TARGET;
                break;

           /*******************************************************************************/  
            case TO_TARGET :
                //move_slam(DELIVERY_X, DELIVERY_Y, ELEV3L_OZ, ELEV3L_OW);
                my_state = AWAIT_INPUT;
                break;

           /*******************************************************************************/  
            case AWAIT_INPUT :
                request_help(CONFIRM_DELIVERY, response, sound_pub);
                done_flag = true;
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

    std::cout << "Delivery complete. Shutting down program, goodbye!" << std::endl << std::flush;

    // Wait for io_thread() to end. 
    //io_thread.join();
}
