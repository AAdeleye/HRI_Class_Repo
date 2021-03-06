
/***********************************************************
 *                                                         *
 *                  HRI: CSE 276B/291F                     *
 *                     Final project                       *
 *         Sanmi Adeleye, Andi Frank, Alyssa Kubota        *
 *                                                         *
 *                                                         *
 *                     Professor: Laurel Riek              *
 *                     TA: Angelique Taylor                *
 **********************************************************/
//TODO: we should make a launch file...
/*
 * Nodes to have running in background before starting: 
 * Note**: source squirtlebot_ws bash
 * roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/squirtlebot_ws/src/gaitech_slam_navigation/src/maps/Atk6.yaml
 * roslaunch main_pac recognizer.launch
 * roslaunch pozyx_localization pozyx_localization_node.launch
 * roslaunch sound_play soundplay_node.launch
 * roslaunch astra_launch astra_pro.launch
 * rosparam set /cmvision/color_file ~/squirtlebot_ws/src/turn_to_trainer/src/colors.txt
 * roslaunch cmvision cmvision.launch image:=/camera/rgb/image_raw (can quit after launched)
 * roslaunch researcher_input researcher_input_node.launch
 */

//TODO: The excessive couts are for debugging purposes. Remove later.

// ************************************************************* //
// ****************                             **************** //
// *****                   INCLUDES                        ***** //
// ****************                             **************** //
// ************************************************************* //

#include <display_image/display_image.h>
#include <gaitech_slam_navigation/map_navigation.h>
//#include <gaitech_slam_navigation/voice_teleop.h>
#include "dialogue.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/String.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include <string>

#include "/home/turtlebot/squirtlebot_ws/src/line_detection/src/line_detection.h"


// ************************************************************* //
// ****************                             **************** //
// *****         GlOBAL VARIABLES/TYPE DEFINITIONS         ***** //
// ****************                             **************** //
// ************************************************************* //

//      << Type Definitions >>
//   State Variables
enum State {
    INTRO,
    CONCLUSION,
    CATCH_POKEMON,
    NAVIGATE,
    PLAYER_CONFIRMATION
};
State my_state;
//  Low level state variables 
enum LL_State {
    STANDBY,
    TO_POKEBALLBOT,
    TO_POKEBALLBOT_SPIN,
    MINI_GAME,
    LOSE_BALL,
    LOSE_BALL_LEFT,
    LOSE_BALL_RIGHT,
    LOSE_BALL_SQUIRTLEBOT,
    RETURN_TO_PATH,
    CELEBRATE_LEFT,
    CELEBRATE_RIGHT,
    CELEBRATE,
    FAILURE_LEFT,
    FAILURE_RIGHT,
    FAILURE
};
LL_State my_ll_state;

enum Voice_reg{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP,
    GO,
    WAIT
}; 
Voice_reg voice_cmd = STOP;

//      << Global Variables >>
geometry_msgs::Twist T;
int pokemon_encountered = 0; 
bool caught_pikachu = false;
bool caught_dragonite = false;
bool caught_mew = false;
int num_pokeball = 0; // Increase to 3 after demo
int num_greatball = 0; // Increase to 3 after encountering Pikachu
int num_ultraball = 0; // Increase to 3 after finding UltraBalls
bool got_user_confirmation = false;


// ************************************************************* //
// ****************                             **************** //
// *****               CALLBACK FUNCTIONS                  ***** //
// ****************                             **************** //
// ************************************************************* //

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
        else if(command.data.find("go") != std::string::npos){
            got_user_confirmation = true;
            voice_cmd = GO;}
        else if(command.data.find("wait") != std::string::npos){
            got_user_confirmation = false;
            voice_cmd = WAIT;}
    }   
}

// Callback function for the Pozyx localization
// Also serves as callback for the researcher_input node 
// Changes my_ll_state to CELEBRATE_TURN when squirtlebot gets close to a pokemon
void pozyx_localization_callback(const std_msgs::String& command) {
    // Only used when we are trying to catch a pokemon
    if (my_state == CATCH_POKEMON && my_ll_state == MINI_GAME) {
        // /pozyx_localization publishes if squirtlebot is close to a specific location
        if (command.data.find("CAUGHT") != std::string::npos) {
            switch(pokemon_encountered) {
                case 0: // Demo
                    if (command.data.find("demo") != std::string::npos)
                        my_ll_state = CELEBRATE_LEFT;
                    break;
                case 1: // Pikachu
                    if (command.data.find("pikachu") != std::string::npos)
                        my_ll_state = CELEBRATE_LEFT;
                    break;
                case 2: // Dragonite
                    if (command.data.find("dragonite") != std::string::npos)
                        my_ll_state = CELEBRATE_LEFT;
                    break;
                case 3: // Ultraball
                    // Shouldn't get here
                    std::cout << "pozyx_localization_callback: in ultraball case" << std::endl << std::flush;
                    break;
                case 4: // Mew
                    if (command.data.find("mew") != std::string::npos)
                        my_ll_state = CELEBRATE_LEFT;
                    break;
                default:
                    std::cout << "pozyx_localization_callback: Invalid number of pokemon_encountered: " +
                        pokemon_encountered << std::endl << std::flush;
            }
        }
    }
}

// ************************************************************* //
// ****************                             **************** //
// *****               AUXILIARY FUNCTIONS                 ***** //
// ****************                             **************** //
// ************************************************************* //

void Slam(){
    switch(pokemon_encountered){
        case 0: // To Pikachu
            move_to_Pica();
            std::cout << "Slam case 0" << std::endl << std::flush;
            break;
        case 1: // To Dragonite
            move_to_Dragon();
            std::cout << "Slam case 1" << std::endl << std::flush;
            break;
        case 2: // To UltraBalls
            move_to_Ultra();
            break;
        case 3: // To Mew
            move_to_Meu();
            break;
        default:
            std::cout << "Error: Unexpected state in Slam reached" << std::endl;
            break;
    }  
}

void get_direction() {
    switch (voice_cmd){
        case FORWARD:
            T.angular.z = 0;
            T.linear.x = .4;
            break;
        case BACKWARD:
            T.angular.z = 0;
            T.linear.x = -.4;
            break;
        case LEFT:
            T.angular.z = 0.6;
            T.linear.x = 0;
            break;
        case RIGHT:
            T.angular.z = -0.6;
            T.linear.x = 0;
            break;
        case STOP:
            T.angular.z = 0;
            T.linear.x = 0;
            break;
    }
}

// Wrapper for the display_squirtle function in display_image
void display_squirtle(const char* emotion) {
    std::string emotion_str(emotion);
    display_squirtle(emotion_str, num_pokeball, num_greatball, num_ultraball, 
                    caught_pikachu, caught_dragonite, caught_mew);
}

// Called in NAVIGATE as SquirtleBot starts moving towards the next Pokemon
void introduce_pokemon() {
    switch(pokemon_encountered) {
        case 0: // Pikachu
            std::cout << "happy introducing pikachu" << std::endl << std::flush;
            display_squirtle("happy");
            speak("pikachu_intro");
            break;
        case 1: // Dragonite
            std::cout << "awe introducing dragonite" << std::endl << std::flush;
            display_squirtle("awe");
            speak("dragonite_intro");
            break;
        case 2: // Ultraballs
            std::cout << "very happy introducing ultraballs" << std::endl << std::flush;
            display_squirtle("veryhappy");
            speak("ultraballs");
            num_ultraball = 3;
            break;
        case 3: // Mew
            std::cout << "shine introducing mew" << std::endl << std::flush;
            display_squirtle("shine");
            speak("mew_intro");
            break;
        default:
            std::cout << "introduce_pokemon: Invalid pokemon_encountered value: " + 
                pokemon_encountered << std::endl << std::flush;
            break;
     }
}

// Called in NAVIGATE once SquirtleBot arrives at the next location.
void confirm_catch_pokemon() {
    display_squirtle("front");
    switch(pokemon_encountered) {
        case 1: // Pikachu
            speak("pikachu_ready");
            break;
        case 2: // Dragonite
            speak("dragonite_ready");
            break;
        case 3: // Ultra Balls
            break;
        case 4: // Mew
            speak("mew_ready");
            break;
        default:
            std::cout << "confirm_catch_pokemon: Invalid pokemon_encountered value: " +
                pokemon_encountered << std::endl << std::flush;
            break;
    }
}

// Called in CATCH_POKEMON: LOSE_POKEBALL if the player directs SquirtleBot off the line.
// SquirtleBot displays an unhappy SquirtleBot face and informs the player they lost a ball.
void lose_ball_script(int num_balls){
    switch (num_balls) {
        case 2:
            std::cout << "sad lose ball 1" << std::endl << std::flush;
            display_squirtle("sad");
            speak("lost_ball_1");
            break;
        case 1:
            std::cout << "grr lose ball 2" << std::endl << std::flush;
            display_squirtle("grr");
            speak("lost_ball_2");
            break;
        case 0:
            std::cout << "tch lose ball 3" << std::endl << std::flush;
            display_squirtle("tch");
            speak("lost_ball_3");
            break;
        default:
            std::cout << "lose_ball_script: invalid number of balls: " + 
                num_balls << std::endl << std::flush;
            break;
    }
    
}

// Called in CATCH_POKEMON: CELEBRATE if the player catches the Pokemon.
// SquirtleBot displays a happy face and informs the player they caught the Pokemon.
void congratulate_trainer() {
    switch (pokemon_encountered) {
        case 0: // Demo
            std::cout << "default congratulate demo" << std::endl << std::flush;
            display_squirtle("default");
            // Reset the number of PokeBalls back to 3
            num_pokeball = 3;
            speak("intro");
            my_state = NAVIGATE;
            break;
        case 1: // Pikachu
            std::cout << "veryhappy congratulate pikachu" << std::endl << std::flush;
            speak("caught_pokemon");
            caught_pikachu = true;
            display_squirtle("veryhappy");
            my_state = NAVIGATE;
            break;
        case 2: // Dragonite
            std::cout << "veryhappy congratulate dragonite" << std::endl << std::flush;
            speak("caught_pokemon");
            caught_dragonite = true;
            display_squirtle("veryhappy");
            my_state = NAVIGATE;
            break;
        case 3: // Ultraballs
            // Shouldn't get here
            std::cout << "congratulate trainer: In ultraball state" << std::endl << std::flush;
            my_state = NAVIGATE;
            break;
        case 4: // Mew
            std::cout << "veryhappy congratulate mew" << std::endl << std::flush;
            speak("caught_pokemon");
            caught_mew = true;
            display_squirtle("veryhappy");
            my_state = CONCLUSION;
            break;
        default:
            std::cout << "congratulate_trainer: Invalid number of pokemon encountered: " + 
                pokemon_encountered << std::endl << std::flush;
            my_state = CONCLUSION;
            break;
    }
}

// Called in CATCH_POKEMON: FAILURE if the player runs out of balls
// SquirtleBot displays a sad face and informs the player the Pokemon ran away.
void pokemon_escape() {
    display_squirtle("cry");
    switch(pokemon_encountered) {
        case 0: // Demo
            std::cout << "cry pokemon escape demo" << std::endl << std::flush;
            // Shouldn't get here?
            my_state = NAVIGATE;
            break;
        case 1: // Pikachu
            std::cout << "cry pikachu escape" << std::endl << std::flush;
            speak("pikachu_escape");
            my_state = NAVIGATE;
            break;
        case 2: // Dragonite
            std::cout << "cry dragonite escape" << std::endl << std::flush;
            speak("dragonite_escape");
            my_state = NAVIGATE;
            break;
        case 3: // Ultraballs
            // Shouldn't get here
            std::cout << "pokemon_escape: In ultraball state" << std::endl << std::flush;
            my_state = NAVIGATE;
            break;
        case 4: // Mew
            std::cout << "cry mew escape" << std::endl << std::flush;
            speak("mew_escape");
            my_state = CONCLUSION;
            break;
        default:
            std::cout << "pokemon_escape: Invalid number of pokemon encountered" << std::endl << std::flush;
            my_state = CONCLUSION;
            break;
    }
    display_squirtle("determined");
    speak("next_one");
}


void update_balls(BallType ball_type, int num_balls){
    switch(ball_type){
         case POKEBALL:
             num_pokeball = num_balls;
             break;
         case GREATBALL:
             num_greatball = num_balls;
             break;
         case ULTRABALL:
             num_ultraball = num_balls;
             break;
    }
}

// Makes SquirtleBot turn in the direction given times_to_spin times.
// Used for when a pokeball is lost and for spinning when entering PokeballBot mode.
int spin_count = 0;
bool done_spinning(double velocity, int times_to_spin) {
    if (spin_count >= times_to_spin) {
        T.angular.z = 0.0;
        spin_count = 0;
        return true;
    } else {
        T.angular.z = velocity;
        spin_count += 1;
        return false;
    }
}

// Used to get vocal confirmation from the trainer.
bool get_confirmation() {
    if (got_user_confirmation) {
        got_user_confirmation = false;
        return true;
    } else {
        return false;
    }
}

// ****************************************************** //
// **********                                ************ //
// ******               MAIN FUNCTION               ***** //
// **********                                ************ //
// ****************************************************** //

// Catch_Gamplay
void catch_gameplay(){
    int num_balls;
    BallType ball_type;
    switch (pokemon_encountered) {
        case 0: // Demo - Pokeball
            num_balls = num_pokeball;
            ball_type = POKEBALL;
            break;
        case 1: // Pikachu - Pokeball
            num_balls = num_pokeball;
            ball_type = POKEBALL;
            break;
        case 2: // Dragonite - Greatball
            num_balls = num_greatball;
            ball_type = GREATBALL;
            break;
        case 3: // Mew - Ultraball
            num_balls = num_ultraball;
            ball_type = ULTRABALL;
            break;
    }

    if (num_balls <= 0 && my_state != INTRO) 
        my_ll_state = FAILURE_LEFT;

    switch (my_ll_state) {
        // Transforming to PokeBallBot
        case TO_POKEBALLBOT:
            // Display the PokeBall image
            std::cout << "display ball" << std::endl << std::flush;
            display_ball(ball_type);
            speak("entering_pokeballbot");
            my_ll_state = TO_POKEBALLBOT_SPIN;
            break;
        case TO_POKEBALLBOT_SPIN:
            // Spin in a circle
            if (done_spinning(3.1, 13))
                my_ll_state = MINI_GAME;
            break;
        // Main catching mini game
        case MINI_GAME:
            //TODO: Line Follow
            //TODO: If off of line, let player know - call lose pokeball
            get_direction();
            break;
        // Losing a ball sequence
        case LOSE_BALL_LEFT:
            // Turn a little left
            if (done_spinning(0.6, 3))
                my_ll_state = LOSE_BALL_RIGHT;
            break;
        case LOSE_BALL_RIGHT:
            // Turn a little right
            if (done_spinning(-0.6, 3)) 
                my_ll_state = LOSE_BALL;
            break;
        case LOSE_BALL:
            // Play the ball sound and update the number of balls
            speak("pokeball_open");
            num_balls--;
            update_balls(ball_type, num_balls);
            my_ll_state = LOSE_BALL_SQUIRTLEBOT;
            break;
        case LOSE_BALL_SQUIRTLEBOT:
            // Spin in a circle and turn back into SquirtleBot. Inform the player they lost a ball
            if (done_spinning(-3.1, 13)) {
                lose_ball_script(num_balls);
                if (num_balls > 0)
                    my_ll_state = RETURN_TO_PATH;
                else
                    my_ll_state = FAILURE_LEFT;
            }
            break;
        case RETURN_TO_PATH:
            // Reposition SquirtleBot so it's on the line
            // TODO: Call after TO_POKEBALLBOT to make sure spinning doesn't mess up positioning?
            // return_to_path() will return true once the bot is on the path again
            
            /* Commented out to compile */
            //if(return_to_path())
                my_ll_state = TO_POKEBALLBOT; // switch back to PokeballBot and then return to MINI_GAME
            break;
        // Celebrate sequence
        case CELEBRATE_LEFT:
            // Happy dance left
            if (done_spinning(3.0, 6))
                my_ll_state = CELEBRATE_RIGHT;
            break;
        case CELEBRATE_RIGHT:
            // Happy dance right
            if (done_spinning(-3.0, 12))
                my_ll_state = CELEBRATE;
            break;
        case CELEBRATE:
            // Re-center to previous position and congratulate the trainer
            if (done_spinning(3.0, 6)) {
                congratulate_trainer(); // Note: updates my_state to NAVIGATE or CONCLUSION
                update_balls(ball_type, num_balls);
                if (pokemon_encountered == 1) { // After catching Pikachu
                    num_greatball = 3;
                    speak("greatballs");
                }
            }
            break;
        // Failure sequence
        case FAILURE_LEFT:
            // Sad dance left
            if (done_spinning(0.6, 6))
                my_ll_state = FAILURE_RIGHT;
            break;
        case FAILURE_RIGHT:
            // Sad dance right
            if (done_spinning(-0.6, 12))
                my_ll_state = FAILURE;
        case FAILURE:
            // Re-center to previous position and inform the player that the Pokemon escaped
            if (done_spinning(0.6, 6)) {
                pokemon_escape(); // Note: updates my_state to NAVIGATE or CONCLUSION
                update_balls(ball_type, num_balls);
                if (pokemon_encountered == 1) {
                    num_greatball = 3;
                    speak("greatballs");
                }
            }
            break;
     }
}


// ********************* Main *************************** //
int 
main(int argc, char** argv){
    ros::init(argc,argv, "squirtlebot_main");
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    ros::Rate rate(5);
    
    // Publishers and subscribers.
    ros::Publisher vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
    ros::Subscriber voice_cmd_sub = nh->subscribe("/recognizer/output", 5, voc_RegCallBack); 
    ros::Subscriber pozyx_loc_sub = nh->subscribe("/pozyx_localization", 1000, pozyx_localization_callback);
    // Note: researcher_input is for backup in case the Pozyx tags are unresponsive
    ros::Subscriber input_sub = nh->subscribe("/researcher_input", 1000, pozyx_localization_callback);


    my_state = INTRO;
    my_ll_state = TO_POKEBALLBOT; 

    while(ros::ok()){
        switch (my_state) {
            case INTRO:
                if (get_confirmation()) {
                    std::cout << "default intro" << std::endl << std::flush;
                    display_squirtle("default");
                    my_state = CATCH_POKEMON;
                    // Three PokeBalls for the demo
                    num_pokeball = 3;
                }
                break;
            case PLAYER_CONFIRMATION:
                if (get_confirmation()) 
                    // When pokemon_encountered == 3, we are at the ultra ball
                    // so move on to the next pokemon
                    if (pokemon_encountered == 3) {
                        my_state = NAVIGATE;
                    } else {
                        my_state = CATCH_POKEMON;
                        my_ll_state = TO_POKEBALLBOT;
                    }
                break;
            case NAVIGATE:
                introduce_pokemon();
                // Move to the next pokemon location
                Slam();
                pokemon_encountered += 1;
                // Once at the next location, say more things
                confirm_catch_pokemon();
                my_state = PLAYER_CONFIRMATION;
                break;
            case CATCH_POKEMON:
                catch_gameplay();
                break;
            case CONCLUSION:
                std::cout << "awe conclusion" << std::endl << std::flush;
                display_squirtle("awe");
                speak("conclusion");
                break;
            default:
                std::cout << "Error: unexpected state reached main" << std::endl << std::flush;
                break;
        }
        rate.sleep();
        ros::spinOnce();
        vel_pub.publish(T);
    }
}









