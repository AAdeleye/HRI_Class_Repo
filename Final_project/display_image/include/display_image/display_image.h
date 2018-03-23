#ifndef DISPLAY_IMAGE_H
#define DISPLAY_IMAGE_H

#include <std_msgs/String.h>
#include <string.h>
#include <ctype.h>

enum BallType {
    POKEBALL,
    GREATBALL,
    ULTRABALL
};

// Creates the path then sends a system call to run the python script to display a Squirtle to the screen
void display_squirtle (std::string &emotion, int &num_pballs, int &num_gballs, int &num_uballs, 
        bool &caught_pika, bool &caught_dragon, bool &caught_mew);

// Creates the path then sends a system call to run the python script to display a Poke Ball to the screen
void display_ball (BallType ball);

void send_command (std::string full_path);

#endif
