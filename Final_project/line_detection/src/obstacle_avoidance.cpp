 ***********************************************************/
 * Name: obstacle_avoidance.cpp
 * Author: Angelique Taylor
 * Date: 02/18/2017
 *
 * Description: This program will subscribe to the /blobs and /camera/depth/points
 *              topic, and use the blob information to find and go towards
 *              the blobs in a user-set order.
 ***********************************************************/

#include <kobuki_msgs/BumperEvent.h> 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <stdio.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#define ERROR_ALLOWANCE 30 //Avoid hysteresis

//Center of the Screen
#define CENTER 320

//Closest distance the robot will get to an object (units unidentified)
#define DIST_THRESH 0.55

//PID Constants
#define PGAIN 0.005
#define IGAIN 0.00000001
#define DGAIN 0.0000001
#define IMAX 7200000
#define IMIN 0

//Threshhold for point conversion
#define BLOBTHRESH 0.025

//Threshhold for z distance
#define ZTHRESH 0.7

//PID variables 
int iState = 0;
int dState = 0;

int state = 0;
bool got_goal_blobs = 0; //The minimum depth needed to reach the goal.
bool goal_found = 0;
int timerRight = 10;
int timerLeft = 20;

#define GOAL_DEPTH_THRESH 0.8
using namespace std;
std::vector<double> PCL_closest_points;
std::vector<double> PCL_closest_points_x;
std::vector<double> PCL_closest_points_y;

uint16_t goal_loc_x = 0;    //global to store the x coordinate of the goal.
uint16_t goal_loc_y = 0;    //global to store the y coordinate of the goal.
double goal_depth = 0;      //global storing the depth of the goal blob
double z_min = 100;         //Used to find min z value for obstacle avoidance
/************************************************************
 * Function Name: blobsCallBack
 * Parameters: const cmvision::Blobs
 * Returns: void
 *
 * Description: This is the callback function of the /blobs topic
 ***********************************************************/
void blobsCallBack (const cmvision::Blobs& blobsIn) //this gets the centroid of the color blob corresponding to the goal.
{
    if (!got_goal_blobs){
        uint16_t blob_sum, blob_centroid_sum, num_blobs;
        uint8_t cluster_dist_x = 0;
        uint8_t cluster_dist_z = 0;
        bool cluster_flag = false;
        int x_sum = 0;
        double z_sum = 0;
        uint16_t goal_sum_x = 0; //ADD TO CODE
        uint16_t goal_sum_y = 0; //ADD TO CODE
        uint16_t num_goal_blobs = 0;
        /************************************************************
        * These blobsIn.blobs[i].red, blobsIn.blobs[i].green, and blobsIn.blobs[i].blue values depend on the
        * values those are provided in the colors.txt file.
        * For example, the color file is like:
        * 
        * [Colors]
        * (255, 0, 0) 0.000000 10 RED 
        * (255, 255, 0) 0.000000 10 YELLOW 
        * [Thresholds]
        * ( 127:187, 142:161, 175:197 )
        * ( 47:99, 96:118, 162:175 )
        * 
        * Now, if a red blob is found, then the blobsIn.blobs[i].red will be 255, and the others will be 0.
        * Similarly, for yellow blob, blobsIn.blobs[i].red and blobsIn.blobs[i].green will be 255, and blobsIn.blobs[i].blue will be 0.
        ************************************************************/
        std::vector<int> blobs_centroid_vector;
        std::vector<double> cones_z;
        std::vector<int> cones_x;
        std::vector<int> cluster_cone1_x;
        std::vector<double> cluster_cone1_z;
        std::vector<int> cluster_cone2_x;
        std::vector<double> cluster_cone2_z;
        int sum_cone_1_z = 0;
        int sum_cone_2_z = 0;

        blobs_centroid_vector.clear();
        cones_z.clear();
        cones_x.clear();
        cluster_cone1_x.clear();
        cluster_cone1_z.clear();
        cluster_cone2_x.clear();
        cluster_cone2_z.clear();

        for (int i = 0; i < blobsIn.blob_count; i++){
            if (blobsIn.blobs[i].red == 0 && blobsIn.blobs[i].green == 255 && blobsIn.blobs[i].blue == 0){
                goal_sum_x += blobsIn.blobs[i].x; //stores the sum x-coordinate of the goal blobs
                goal_sum_y += blobsIn.blobs[i].y; //stores the sum y-coordinate of the goal blobs
                //ROS_INFO("We got blobs");
            }
        }

    }
}

/************************************************************
 * Function Name: PointCloud_Callback
 * Parameters: const PointCloud::ConstPtr
 * Returns: void
 *
 * Description: This is the callback function of the PointCloud
 *              topic, flags when an object is below the threshhold 
 ***********************************************************/

void PointCloud_Callback (const PointCloud::ConstPtr& cloud){

        unsigned int n = 0;
        int s,t;
        double min_z = 100, x, y;
        int y_point = 0;

        PCL_closest_points.clear();
        PCL_closest_points_x.clear();
        PCL_closest_points_y.clear();

        z_min = 100;
        //Iterate through all the points in the image
        //Convert from pcl to cm
        for(int k = 0; k < 240; k++){
            for(int i = 0; i < 640; i++){
                const pcl::PointXYZ& pt = cloud->points[640*(180+k)+(i)];
                if((pt.z < ZTHRESH)){
                    PCL_closest_points_x.push_back(i);
                    PCL_closest_points.push_back(pt.z);         
                    //Find min z
                    if(pt.z < z_min){       
                        z_min = pt.z;
                    }
                    //cout << "z: " << pt.z << endl;
                }
            }
        }
    if(got_goal_blobs){
        const pcl::PointXYZ& ptg = cloud->points[640 * (goal_loc_y - 1) + (goal_loc_x - 1)];
        goal_depth = ptg.z;
        got_goal_blobs = false; //reset the condition to allow the blob callback to get new blobs.
    }
}

/************************************************************
 * Function Name: Center_View
 * Parameters: int horz_blob_centroid
 * Returns: void
 *
 * Description: This function will control turning of the turtle
 *  bot, to keep the blob view at the center. This is also the
 *  function to handle PID control.
 ***********************************************************/ 
double Center_View(int horz_blob_centroid){
        int error = CENTER - horz_blob_centroid;

    //PID Variables
    double pTerm = abs(PGAIN*error); //Calculate the proportional control.
    double iTerm = 0.0;
    double dTerm = abs(DGAIN*(horz_blob_centroid - dState)); //Calculate the derivative control.
    double PIDgain = 0;

    //Off by too much for too long, so crank the gain to max...
    if( iState > IMAX){
        iState = IMAX;
    }
    //Reset accumulator
    else if( iState < IMIN){
        iState = IMIN;
    }
      
    iTerm = abs(IGAIN* iState); //Calculate the integral gain
    dState = horz_blob_centroid; //Update derivative state

    PIDgain = pTerm + iTerm + dTerm;

    //This means that the blob centroid is to the right of the robot's view.
    //So, turn right.
    if(error < -1*ERROR_ALLOWANCE) {
        return (-1.0*PIDgain);  
    }

    //This means that the blob centroid is to the left of the robot's view.
    //So, turn left.
    else if(error > ERROR_ALLOWANCE){
        return(PIDgain);
    }
    //The centroid is within the error allowance, so don't turn!
    else{
        return(0.0);
    }
}

/************************************************************
 * Function Name: main
 * Parameters: int argc, char **argv
 * Returns: int
 *
 * Description: This is the main function. It will subscribe
 *              to the /blobs and /PointCloud topic and go 
 *              towards specific blobs of a calibrated color,
 *              while avoiding objects. It will do this
 *              in the order specified by the user.
 ***********************************************************/ 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "blobs_test");
    bool goal_trigger = 0;
    int i =0; //counter
    bool goLeft = 0;
    bool goRight = 0;
    // Create handle that will be used for both subscribing and publishing. 
    ros::NodeHandle n;
  
    //subscribe to /PointCloud2 topic 
    ros::Subscriber PCSubscriber = n.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);  

    //subscribe to /blobs topic 
    ros::Subscriber blobsSubscriber = n.subscribe("/blobs", 100, blobsCallBack);

    // publishing the geometry message twist message
    ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

    ROS_INFO("Program started!");
    
    // Set the loop frequency in Hz.
    ros::Rate loop_rate(10);

    geometry_msgs::Twist t;
   
    //runtime loop
    while(ros::ok()){
        //STARTING SEQUENCE
        if (!goal_trigger){ 
            if (state == 0){ 
            }
            else if(state == 1){ 
            }
            else if(state == 2){ 
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
        velocityPublisher.publish(t);
    } 
    return 0;
}

