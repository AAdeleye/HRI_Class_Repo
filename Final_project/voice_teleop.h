#ifndef VOICE_TELEOP_H
#define VOICE_TELEOP_H

// *********************************************** //
// **********                         ************ //
// *****        




// ******************************************* //
// **********                    ************* //
// ***          Main Function              *** //
// *********                     ************* //
// ******************************************* //

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include <string>

enum Voice_reg{
     FORWARD,
     BACKWARD,
     LEFT,
     RIGHT,
     STOP
}; 

Voice_reg voice_cmd = STOP;


void voc_RegCallBack(const std_msgs::String& command){    
    if (command.data.find("squirtlebot") != std::string::npos){
       
       if(command.data.find("forward") != std::string::npos){
            voice_cmd = FORWARD;}
       else if(command.data.find("backward") != std::string::npos){
            voice_cmd = BACKWARD;}    
       else if(command.data.find("right") != std::string::npos){
            voice_cmd = RIGHT;}       
       else if(command.data.find("left") != std::string::npos){
            voice_cmd = LEFT;}
       else if(command.data.find("stop") != std::string::npos){
            voice_cmd = STOP;}
       //std::cout << voice_cmd << std::endl;
    }
}



#endif
