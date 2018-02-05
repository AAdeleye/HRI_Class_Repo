/*#include <ros/ros.h>
#include <cmvision/Blobs.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Twist.h>
*/
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

bool got_goal_blobs = 0;
uint16_t state;
uint16_t goal_sum_x = 0; //ADD TO CODE
int16_t goal_sum_y = 0; //ADD TO CODE
uint16_t Center = 320;
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
	if (!got_goal_blobs){
		ROS_INFO("In blobsCallBack");
		uint16_t blob_sum, blob_centroid_sum, num_blobs;
		uint8_t cluster_dist_x = 0;
		uint8_t cluster_dist_z = 0;
		bool cluster_flag = false;
		int x_sum = 0;
		double z_sum = 0;
		//uint16_t goal_sum_x = 0; //ADD TO CODE
		//uint16_t goal_sum_y = 0; //ADD TO CODE
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
			if (blobsIn.blobs[i].red == 42 && blobsIn.blobs[i].green == 101 && blobsIn.blobs[i].blue == 82){
				ROS_INFO("Teal blob found");
				got_goal_blobs = 1;
				goal_sum_x += blobsIn.blobs[i].x;
				goal_sum_y += blobsIn.blobs[i].y;
			}
		}
		if(blobsIn.blob_count != 0){
            state = 1;
			goal_sum_x = goal_sum_x / blobsIn.blob_count;
			goal_sum_y = goal_sum_y / blobsIn.blob_count;
            goal_xs.push_back(goal_sum_x);
            if (goal_xs.size() > 3) {
                goal_xs.pop_back();
            }
		}
        else {
            // No blobs visible, so continue to turn and look for one
            state = 0;
        }
		ROS_INFO("x: %d", goal_sum_x);
		ROS_INFO("y: %d", goal_sum_y);
		ros::Duration(1.0).sleep();

	}
}

double CenterView(){
    int error = Center - goal_sum_x;
    if(error > 30 ){
        int Pvalue = .005;
        int Ivalue = 0;
        
        got_goal_blobs = 0;
           
        for (int i=0; i < goal_xs.size(); i++){
            Ivalue += goal_xs[i];
        }
        Ivalue /= goal_xs.size();
        

        double output = (Pvalue * error) + (Ivalue * error);
        return output;
    }
    if(error < -30 ){
        int Pvalue = .005;
        int Ivalue = 0;
        
        got_goal_blobs = 0;
           
        for (int i=0; i < goal_xs.size(); i++){
            Ivalue += goal_xs[i];
        }
        Ivalue /= goal_xs.size();
        

        double output = (Pvalue * error) + (Ivalue * error);
        return (-1.0*output);
    }
    else {
        return 0.0;
    }
}

/************************************************************
 * Function Name: PointCloud_Callback
 * Parameters: const PointCloud::ConstPtr
 * Returns: void
 *
 * Description: This is the callback function of the PointCloud
 * 				topic, flags when an object is below the threshhold 
 ***********************************************************/
/*
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
	if(got_goal_blobs){
		const pcl::PointXYZ& ptg = cloud->points[640 * (goal_loc_y - 1) + (goal_loc_x - 1)];
		goal_depth = ptg.z;
		got_goal_blobs = false; //reset the condition to allow the blob callback to get new blobs.
	}
}
*/
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "blob");
  ros::NodeHandle nh;
  //States variable 
  state = 0;
  
  //ros::Subscriber PCSubscriber = nh.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);

  //subscribe to /blobs topic 
  ros::Subscriber blobsSubscriber = nh.subscribe("/blobs", 50, blobsCallBack);

  // publishing the geometry message twist message
  ros::Publisher velocityPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

  ros::Rate loop_rate(10);
  geometry_msgs::Twist T;// cmd(new geometry_msgs::Twist());

  while(ros::ok()){
    //Looking for goal
    if (state == 0){
      T.angular.z = .05;
    }
    //Moving to goal
    else if (state == 1){
	  ROS_INFO("In State 1");
      T.linear.x = .05;
      ROS_INFO("%d", CenterView());
      T.angular.z = CenterView()*(.05);
    }
    //At goal
    else if (state == 2){
    
    }
    //T.linear.x = 0.5;// (z - goal_z_) * z_scale_;
    //cmd->angular.z = 600;//-x * x_scale_;

    // Spin
    ros::spinOnce();
    loop_rate.sleep();
    velocityPublisher.publish(T);
  }
}

