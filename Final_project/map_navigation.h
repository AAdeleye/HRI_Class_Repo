#ifndef MAP_NAVIGATION_H  
#define MAP_NAVIGATION_H

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

#include <ros/ros.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <chrono>

// ************************************************************* //
// *************                                   ************* //
// *****               TARGETS/OTHER MACROS                ***** //
// *************                                   ************* //
// ************************************************************* //

// Location 1
#define L1_X     5.41 
#define L1_Y     -3.277 
#define L1_OZ    0.043
#define L1_OW    0.999
// Location 2
#define L2_X     7.73 
#define L2_Y     -9.893
#define L2_OZ    -0.678
#define L2_OW    0.7348
// Location 3
#define L3_X     8.055 
#define L3_Y     -16.669
#define L3_OZ    -0.0688
#define L3_OW    0.9976

// ************************************************************* //
// *************                                   ************* //
// *****         GLOBAL VARIABLES/TYPE DEFINITIONS         ***** //
// *************                                   ************* //
// ************************************************************* //



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

// Move to Location 1
bool move_to_L1(){
    return move_slam(L1_X, L1_Y, L1_OZ, L1_OW);
}
// Move to Location 2
bool move_to_L2(){
    return move_slam(L2_X, L2_Y, L2_OZ, L2_OW);
}
// Move to Location 3
bool move_to_L3(){
    return move_slam(L3_X, L3_Y, L3_OZ, L3_OW);
}

#endif
