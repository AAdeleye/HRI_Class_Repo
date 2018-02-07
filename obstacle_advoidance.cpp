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
ros::Publisher pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//bool got_goal_blobs = 0;
uint16_t state;
double goal_sum_x = 0; //ADD TO CODE
int16_t goal_sum_y = 0; //ADD TO CODE
double goal_area = 0; //ADD to code
double Center = 320;
std::vector<uint16_t> goal_xs;

/************************************************************
 * Function Name: blobsCallBack
 * Parameters: const cmvision::Blobs
 * Returns: void
 *
 * Description: This is the callback function of the /blobs topic
 ***********************************************************/

void blobsCallBack (const cmvision::Blobs& blobsIn) //this gets the centroid of the color blob corresponding to the goal.
{
	//state = 1;
	//if (!got_goal_blobs){
    if (blobsIn.blob_count > 0 && state != 2 && state != 3){
		ROS_INFO("In blobsCallBack");
		uint16_t blob_sum, blob_centroid_sum, num_blobs;
		uint8_t cluster_dist_x = 0;
		uint8_t cluster_dist_z = 0;
		bool cluster_flag = false;
		int x_sum = 0;
		double z_sum = 0;
		double goal_sum_x = 0; //ADD TO CODE
		double goal_sum_y = 0; //ADD TO CODE
		uint16_t num_goal_blobs = 0;

		/************************************************************
		* These blobsIn.blobs[i].red, blobsIn.blobs[i].green, and blobsIn.blobs[i].blue values depend on the
		* values those are provided in the colos.txt file.
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
			ROS_INFO("Blob found");
//			got_goal_blobs = 1;
			goal_sum_x += blobsIn.blobs[i].x;
		    goal_sum_y += blobsIn.blobs[i].y;
		}

//		if(blobsIn.blob_count != 0){
            state = 1;
			goal_sum_x = goal_sum_x / blobsIn.blob_count;
			goal_sum_y = goal_sum_y / blobsIn.blob_count;
            goal_xs.insert(goal_xs.begin(), goal_sum_x);
            if (goal_xs.size() > 3) {
                goal_xs.pop_back();
            }
		    ROS_INFO("x: %f", goal_sum_x);
		    //ROS_INFO("y: %d", goal_sum_y);
//		}
//        else {
            // No blobs visible, so continue to turn and look for one
//            state = 0;
//        }

	}//got_goal_blobs if 
    else if (state != 2 && state != 3)  { //No blobs detected, go back to state 0
        state = 0;
    }

   if(blobsIn.blob_count > 0 && state == 2){
	   goal_area = 0;	
       for (int i = 0; i < blobsIn.blob_count; i++){
			ROS_INFO("Blob found");
			if(blobsIn.blobs[i].area > goal_area){
                goal_area = blobsIn.blobs[i].area;
            }
	    }
       ROS_INFO("goal area %f", goal_area);
       if(goal_area > 85000){
            state = 3;
       }
   }
}

double CenterView(){
    //double error = Center - goal_sum_x;
    double error = Center - goal_xs.front();
    int Pvalue = .005;
    int Ivalue = 0;
    ROS_INFO("Error: %f", error);
    // Average the last three median x's of a blob   
    for (int i=0; i < goal_xs.size(); i++){
        Ivalue += goal_xs[i];
    }
    Ivalue /= goal_xs.size();
    double output = .1;      
//    double output = (Pvalue * error) + Ivalue;
    
    if(error > 30.0 ){ 
        return output;
    }
    if(error < -30.0 ){
        return (-1.0*output);
    }
    else {
        //state = 0;
        //got_goal_blobs = 0;
        ROS_INFO("No correction");
        return 0.0;
    }
}

 void Obstacle_avoid(){
 
 
 
 }

/************************************************************
 * Function Name: PointCloud_Callback
 * Parameters: const PointCloud::ConstPtr
 * Returns: void
 *
 * Description: This is the callback function of the PointCloud
 * 				topic, flags when an object is below the threshhold 
 ***********************************************************/
void PointCloud_Callback (const PointCloud::ConstPtr& cloud){

		unsigned int n = 0;
		int s,t;
		double min_z = 100, x, y;
		int y_point = 0;
        double ZTHRESH = .5;
        std::vector<double> PCL_closest_points;
        std::vector<double> PCL_closest_points_x;
        std::vector<double> PCL_closest_points_y;
		PCL_closest_points.clear();
		PCL_closest_points_x.clear();
		PCL_closest_points_y.clear();

		double z_min = 100;
		//Iterate through all the points in the image
		//Convert from pcl to cm
		for(int k = 0; k < 240; k++){ // 0, 240
			for(int i = 30; i < 610; i++){ // 0, 640
				const pcl::PointXYZ & pt=cloud->points[640*(180+k)+(i)];
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
       // ROS_INFO("Min Z: %f", z_min);
        if(z_min < .5 && state != 3){
            state = 2;
        } 
    /*
	if(got_goal_blobs){
		const pcl::PointXYZ& ptg = cloud->points[640 * (goal_loc_y - 1) + (goal_loc_x - 1)];
		goal_depth = ptg.z;
		got_goal_blobs = false; //reset the condition to allow the blob callback to get new blobs.
	}*/
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "blob");
  ros::NodeHandle nh;
  //States variable 
  state = 0;
  
  ros::Subscriber PCSubscriber = nh.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);

  //subscribe to /blobs topic 
  ros::Subscriber blobsSubscriber = nh.subscribe("/blobs", 50, blobsCallBack);

  // publishing the geometry message twist message
  ros::Publisher velocityPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

  ros::Rate loop_rate(10);
  geometry_msgs::Twist T;// cmd(new geometry_msgs::Twist());
  int toggle = 0;
  while(ros::ok()){
    //Looking for goal
    if (state == 0){
      T.angular.z = .1;
      T.linear.x = 0;
      ROS_INFO("I am spinnning");
    }
    //Moving to goal
    else if (state == 1){
	  ROS_INFO("In State 1");
      T.linear.x = .05;
      T.angular.z = 0;
      ROS_INFO("CenterView: %f", CenterView());
      T.angular.z = CenterView();
    }
    //At goal
    else if (state == 2){
      //Obstacle_avoid(); 
      if( toggle < 5){ 
        ROS_INFO("In toggle < 5");
        T.angular.z = -.3;
        T.linear.x = 0;
      }
      else if( toggle >= 5 && toggle < 10) {
          ROS_INFO("In toggle < 10");
          T.linear.x = .1;
          T.angular.z = 0;
      }
      if (toggle >= 10) {
          ROS_INFO("In toggle > 10");
          toggle = 0;
          state = 0;
      }
      toggle += 1;
    }
    else if (state == 3){
        ROS_INFO("In state 3!");
        T.linear.x = 0;
        T.angular.z = 0;
    }
    //T.linear.x = 0.5;// (z - goal_z_) * z_scale_;
    //cmd->angular.z = 600;//-x * x_scale_;

    // Spin
    ros::spinOnce();
    loop_rate.sleep();
    velocityPublisher.publish(T);
  }
}

