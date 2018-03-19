
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

/*
 * Nodes to have running in background before starting:
 * roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/squirtlebot_ws/src/gaitech_slam_navigation/src/maps/Atk6.yaml
 * roslaunch gaitech_slam_navigation recognizer.launch
 * roslaunch pozyx_localization pozyx_localization_node.launch
 * roslaunch sound_play soundplay_node.launch
 * roslaunch astra_launch astra_pro.launch
 * rosparam set /cmvision/color_file ~/squirtlebot_ws/src/turn_to_trainer/src/colors.txt
 * roslaunch cmvision cmvision.launch image:=/camera/rgb/image_raw (can quit after launched)
 */

//TODO: The excessive couts are for debugging purposes. Remove later.

// ************************************************************* //
// ****************                             **************** //
// *****                   INCLUDES                        ***** //
// ****************                             **************** //
// ************************************************************* //

#include <display_image/display_image.h>
#include <gaitech_slam_navigation/map_navigation.h>
#include <turn_to_trainer/turn_to_trainer.h>
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
    LOSE_BALL_TURN,
    TURN_BACK_TO_PATH,
    RETURN_TO_PATH,
    CELEBRATE_TURN,
    CELEBRATE,
    FAILURE_TURN,
    FAILURE
};
LL_State my_ll_state;

enum Voice_reg{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
}; 
Voice_reg voice_cmd = STOP;

//      << Global Variables >>
geometry_msgs::Twist T;
int pokemon_encountered = 0; 
bool caught_pikachu = false;
bool caught_dragonite = false;
bool caught_mew = false;
int num_pokeball = 0; // default value of pokeballs
int num_greatball = 0; // Increase to 3 after encountering pikachu
int num_ultraball = 0; // Increase to 3 after finding ultraballs
bool got_user_confirmation = false;


// ************************************************************* //
// ****************                             **************** //
// *****               CALLBACK FUNCTIONS                  ***** //
// ****************                             **************** //
// ************************************************************* //

void voc_RegCallBack(const std_msgs::String& command){    
    //std::cout << "outside " + voice_cmd << std::endl;
    std::cout << "outside " + command.data << std::endl;
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

// Callback function for the Pozyx localization
// Changes my_ll_state to CELEBRATE_TURN when squirtlebot gets close to a pokemon
void pozyx_localization_callback(const std_msgs::String& command) {
    // Only used when we are trying to catch a pokemon
    if (my_state == CATCH_POKEMON && my_ll_state == MINI_GAME) {
        // /pozyx_localization publishes if squirtlebot is close to a specific location
        if (command.data.find("CAUGHT") != std::string::npos) {
            switch(pokemon_encountered) {
                case 0: // Demo
                    if (command.data.find("demo") != std::string::npos)
                        my_ll_state = CELEBRATE_TURN;
                    break;
                case 1: // Pikachu
                    if (command.data.find("pikachu") != std::string::npos)
                        my_ll_state = CELEBRATE_TURN;
                    break;
                case 2: // Dragonite
                    if (command.data.find("dragonite") != std::string::npos)
                        my_ll_state = CELEBRATE_TURN;
                    break;
                case 3: // Ultraball
                    // Shouldn't get here
                    std::cout << "pozyx_localization_callback: in ultraball case" << std::endl << std::flush;
                    break;
                case 4: // Mew
                    if (command.data.find("mew") != std::string::npos)
                        my_ll_state = CELEBRATE_TURN;
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
            move_to_L1();
            std::cout << "Slam case 0" << std::endl << std::flush;
            break;
        case 1: // To Dragonite
            move_to_L2();
            std::cout << "Slam case 1" << std::endl << std::flush;
            break;
        case 2: // To UltraBalls
            move_to_B1();
            break;
        case 3: // To Mew
            move_to_L3();
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
//TODO: vocal confirmation from player to move to next state
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
            num_greatball = 3;
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

// TODO: Make this method.
// Returns true if SquirtleBot is on the path, or false if it is not. If it can't see
// the path, returns false by default.
// Used by return_to_path() (SquirtleBot controlling istelf) as well as for checking
// that the bot is on the path (trainer controlling SquirtleBot) throughout the minigame.
bool on_path(){
    // << SHOULD THIS JUST BE Ai FLAG SET BY blobs_callback() AND image_callback()?
        //  -> Probably not; this function should handle no_blobs counter, as well
        //      as no_blobs being less than threshold, but line being off_center anyway.
    
   /*
    if(no_blobs > NO_BLOBS_CUTOFF)
        return false;
    else if ( abs(line.bot_intercept - frame_center) > OFFCENTER_THRESHOLD )
            // note: line.bot_intercept will be different than line.intercept,
                // because bot_intercept occurs at x = frame_width, instead of
                // y = 0, as defined for line.intercept. 
        return false;
    else
        return true;
    
    * */ 
}

// TODO: Make this method.
// In this, SquirtleBot should turn towards the line's centroid, and then
// let the trainer know it is ready to receive more commands. Returns
// TRUE when it has successfully returned to path.
bool return_to_path(){
    bool facing_centroid = false; // TODO: change to be something like
                                    //  = (abs(centroid.x - window_center.x) < threshold )
    if(!facing_centroid){
        // turn to center
        // TODO: Twist messages
        return false;
    }
    else
        return true; 
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

// Used to make SquirtleBot face the trainer.
// When in LOSE_BALL_TURN state, it will update total_times_spun so SquirtleBot can
// turn back to its previous position in back_on_path().
int total_times_spun = 0;
bool facing_trainer() {
    int dir_to_turn = direction_to_turn();
    switch (my_state) {
        case LOSE_BALL_TURN:
            total_times_spun += 1;
            // Once the trainer is in SquirtleBot's sight, slow down spinning to avoid overshooting
            if (check_see_trainer()) {
                if (dir_to_turn == 0) {
                    T.angular.z = 0.0;
                    return true;
                // Turn in the direction of the trainer until they are in the center of SquirtleBot's view
                } else {
                    T.angular.z = dir_to_turn * 0.5;
                    return false;
                }
            // Spin slowly until SquirtleBot sees the trainer
            } else {
                T.angular.z = 0.5;
                return false;
            }
            break;
        case CELEBRATE_TURN:
            // Once the trainer is in SquirtleBot's sight, slow down spinning to avoid overshooting
            if (check_see_trainer()) {
                if (dir_to_turn == 0) {
                    T.angular.z = 0.0;
                    return true;
                // Turn in the direction of the trainer until they are in the center of SquirtleBot's view
                } else {
                    T.angular.z = dir_to_turn * 0.5;
                    return false;
                }
            // Spin quickly until SquirtleBot sees the trainer
            } else {
                T.angular.z = 2.5;
                return false;
            }
            break;
        case FAILURE_TURN:
            // Once the trainer is in SquirtleBot's sight, slow down spinning to avoid overshooting
            if (check_see_trainer()) {
                if (dir_to_turn == 0) {
                    T.angular.z = 0.0;
                    return true;
                // Turn in the direction of the trainer until they are in the center of SquirtleBot's view
                } else {
                T.angular.z = dir_to_turn * 0.5;
                return false;
                }
            // Spin slowly until SquirtleBot sees the trainer
            } else {
                T.angular.z = 0.5;
                return false;
            }
            break;
        default:
            std::cout << "facing trainer: invalid my_state" << std::endl << std::flush;
            return true;
    }
}

// Used to turn SquirtleBot back to its previous position on the line.
// Should probably be followed by return_to_path() so the player isn't penalized again
int times_spun = 0;
bool back_on_path() {
    if (times_spun >= total_times_spun) {
        T.angular.z = 0.0;
        times_spun = 0;
        total_times_spun = 0;
        return true;
    } 
    T.angular.z = -0.5;
    times_spun += 1;
    return false;
}

// Used to get vocal confirmation from the trainer.
bool get_confirmation() {
    if (my_state == INTRO)
        return got_user_confirmation;
    else
        return false;
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
        my_ll_state = FAILURE;

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
            //std::cout << voice_cmd << std::endl << std::flush;
            //TODO: Line Follow
            //TODO: If off of line, let player know
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
            // Change back into SquirtleBot and inform the player they lost a ball
            speak("pokeball_open");
            num_balls--;
            update_balls(ball_type, num_balls);
            my_ll_state = LOSE_BALL_TURN;
            break;
        case LOSE_BALL_TURN:
            // Turn towards the player
            if (facing_trainer()) {
                lose_ball_script(num_balls);
                if (num_balls > 0)
                    my_ll_state = TURN_BACK_TO_PATH;
                else
                    my_ll_state = FAILURE_TURN;
            }
            break;
        case TURN_BACK_TO_PATH:
            // Turn back to the previous position
            if (back_on_path()) 
                my_ll_state = RETURN_TO_PATH;
            break;
        case RETURN_TO_PATH:
            // Reposition SquirtleBot so it's on the line
            // TODO: Call after TO_POKEBALLBOT to make sure spinning doesn't mess up positioning?
            // return_to_path() will return true once the bot is on the path again
            if(return_to_path())
                my_ll_state = TO_POKEBALLBOT; // switch back to PokeballBot and then return to MINI_GAME
            break;
        // Celebrate sequence
        case CELEBRATE_TURN:
            // Spin to face the player
            if (facing_trainer())
                my_ll_state = CELEBRATE;
            break;
        case CELEBRATE:
            // Congratulate the trainer
            congratulate_trainer(); // Note: updates my_state to NAVIGATE or CONCLUSION
            update_balls(ball_type, num_balls);
            break;
        // Failure sequence
        case FAILURE_TURN:
            // Spin to face the player
            if (facing_trainer())
                my_ll_state = FAILURE;
            break;
        case FAILURE:
            // Inform the player that the Pokemon escaped
            pokemon_escape(); // Note: updates my_state to NAVIGATE or CONCLUSION
            update_balls(ball_type, num_balls);
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
    ros::Subscriber trainer_blob_sub = nh->subscribe("/blobs", 50, trainer_blobs_callback);


    // TODO: Start in PLAYER_CONFIRMATION once that's implemented?
    my_state = INTRO;
    my_ll_state = TO_POKEBALLBOT; 
    // This line for testing
    //my_state = CATCH_POKEMON;
    //my_ll_state = LOSE_BALL;

    while(ros::ok()){
        switch (my_state) {
            case INTRO:
                std::cout << "default intro" << std::endl << std::flush;
                display_squirtle("default");
                my_state = CATCH_POKEMON;
                // Three PokeBalls for the demo
                num_pokeball = 3;
                break;
            case PLAYER_CONFIRMATION:
                if (get_confirmation()) 
                    // TODO: Get vocal confirmation from player before moving on?
                    my_state = NAVIGATE;
                break;
            case NAVIGATE:
                introduce_pokemon();
                // Move to the next pokemon location
                Slam();
                pokemon_encountered += 1;
                // Once at the next location, say more things
                confirm_catch_pokemon();
                // 2 lines below for testing. 
                // Remove when we implement vocal confirmation from trainer.
                // Note: when pokemon_encounter == 3, we are at the ultra ball
                if (pokemon_encountered != 3) {
                    my_state = CATCH_POKEMON;
                    my_ll_state = TO_POKEBALLBOT; 
                } // else, remain in NAVIGATE state
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









