#include "../include/display_image/display_image.h"

// Creates the path then sends a system call to run the python script to display an Squirtle to the screen.
void display_squirtle (std::string &emotion, int &num_pballs, int &num_gballs, int &num_uballs, 
        bool &caught_pika, bool &caught_dragon, bool &caught_mew)
{   
    // Make the whole path
    // Path name will have form "../images/squirtle/emotion_333_pdm.png"
    // Make the base
    std::string image_path_str = "/home/turtlebot/squirtlebot_ws/src/display_image/images/squirtle/" + emotion + "_" + std::to_string(num_pballs) +
        std::to_string(num_gballs) + std::to_string(num_uballs) + "_";
    // Add respective letters for each Pokemon caught
    if (caught_pika) image_path_str += "p";
    if (caught_dragon) image_path_str += "d";
    if (caught_mew) image_path_str += "m";
    // Add the file extention
    image_path_str += ".png";
    // Send the command
    send_command(image_path_str);
}

// Creates the path then sends a system call to run the python script to display a Poke Ball to the screen.
void display_ball (BallType ball) 
{
    std::string image_path_str = "/home/turtlebot/squirtlebot_ws/src/display_image/images/";
    switch (ball) {
        case POKEBALL:
            image_path_str += "pokeball.png";
            break;
        case GREATBALL:
            image_path_str += "greatball.png";
            break;
        case ULTRABALL:
            image_path_str += "ultraball.png";
            break;
        default:
            std::cout << "Invalid Poke Ball type." << std::endl << std::flush;
            break;
    }
    send_command(image_path_str);
}

void send_command (std::string full_path)
{
    if (system(NULL)) {
        // Concat to make the whole command
        //std::string full_command_str = "python display_image.py \"" + full_path + "\"";
        std::string full_command_str = "rosrun display_image display_image.py \"" + full_path + "\"";
        // Convert the command to a char*
        const char *full_command = full_command_str.c_str();
        // Send the command
        system(full_command);
    } else {
        std::cout << "Command not sent" << std::endl << std::flush;
    }
}

int main (int argc, char** argv) 
{
    return 0;
}
