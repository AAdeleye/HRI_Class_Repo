
#ifndef line_detection_h
#define line_detection_h


// Basic includes
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <vector>
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmvision/Blobs.h>
#include <image_transport/image_transport.h>
// OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


/****************************************************************************/
/******************************            **********************************/
/**********************      MACROS / GLOBALS    ****************************/
/******************************            **********************************/
/****************************************************************************/

// allowed number of consecutive blobs_callback cycles without detecting any balls
#define NO_BLOBS_CUTOFF 5

#define _LD_SLOPE_MEM           5
#define _LD_INTERCEPT_MEM       5
#define _LD_CENTROID_MEM        5
#define _LD_R_MEM               5


/****************************************************************************/
/******************************            **********************************/
/**********************    CLASS DEFINITION     *****************************/
/******************************            **********************************/
/****************************************************************************/

class LineDetector{
    private:
        //          << FIELDS >>
        
        // ROS Functionalities
        ros::NodeHandle nh;
        
        // Blobs
        int             no_blob_count;
        ros::Subscriber blob_sub;

        // Display fields
        std::string                     window_name;
        image_transport::Subscriber     image_sub;
        image_transport::ImageTransport it; 
        bool                            display_image;
        int                             image_w, image_h;

        // Line structure and arrays to remember past line parameters for filtering
        struct Line{
            double slope;
            int intercept;
            cv::Point centroid;
            double r;
        } line;
        
        // PID Controller struct
        struct PID{
            double kp, kd, ki, imin, imax;
            int error_last, error_integral;
        } pid;

        double on_path_allowance;

        // Filters
        double      slope_filter[_LD_SLOPE_MEM];
        int         intercept_filter[_LD_INTERCEPT_MEM];
        cv::Point   centroid_filter[_LD_CENTROID_MEM];
        double      r_filter[_LD_R_MEM];
        
        //          << METHODS >>
                
        void update_filter(cv::Point* filter, int filter_size, cv::Point new_data);
        void update_filter(double* filter, int filter_size, double new_data);
        void update_filter(int* filter, int filter_size, int new_data);
        
        cv::Point   moving_average(cv::Point* filter, int filter_size);
        double      moving_average(double* filter, int filter_size);
        int         moving_average(int* filter, int filter_size);

        /***************************************************************
         * Function Name: blobs_callback
         * Parameters: const cmvision::Blobs
         * Returns: void
         *
         * Description: This is the callback function for the /blobs topic.
         *              It calculates the linear regression of the blobs.
         ****************************************************************/
         void blobs_callback(const cmvision::Blobs blobs_in);

        /***************************************************************
         * Function Name: image_callback
         * Parameters: const sensor_msgs::ImageConstPtr& msg
         * Returns: void
         *
         * Description: This is the callback function for the /webcam/image topic.
         *              It displays the image and the current linear regression.
         ****************************************************************/
         void image_callback(const sensor_msgs::ImageConstPtr& msg);

    public:
        //          << CONSTRUCTOR/DESTRUCTOR >>
         LineDetector();
         LineDetector(bool show_view);
         ~LineDetector();
        
        //          << METHODS >>
        
         void open_display();
         void close_display();

        // Returns count of how many /blobs topic updates have occurred with
        // no blobs found.
        int get_no_blob_count();
      
        cv::Point get_bot_intercept();
        int get_image_height();
        int get_image_width();
        void set_on_path_allowance(double percent_of_height);

        // Returns true if on path, false if not
        bool on_path();

        // Returns Twist message to rotate bot one "step" toward line direction. 
        geometry_msgs::Twist return_to_path();



};

#endif
