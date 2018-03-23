
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

#define _LD_FILTER_MEM      5
#define _LD_POINTS_CUTOFF   500000
#define _LD_ERROR_THRESHOLD 0.0005

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

        // Display fields
        std::string                     window_name;
        cv::Point                       image_center;
        image_transport::Subscriber     image_sub;
        image_transport::ImageTransport it; 
        bool                            display_image;

        cv::Scalar hsv_low;
        cv::Scalar hsv_high;

        // Line structure and arrays to remember past line parameters for filtering
        struct Path{
            double slope;
            cv::Point centroid;
            int num_points;
        } path;
        
        // PID Controller struct
        struct PID{
            double kp, kd, ki, imin, imax;
            int error_last, error_integral;
        } pid;

        // Filters
        double      slope_filter[_LD_FILTER_MEM];
        cv::Point   centroid_filter[_LD_FILTER_MEM];
        int         points_filter[_LD_FILTER_MEM];
        
        //          << METHODS >>
                
        cv::Mat create_mask(cv::Mat raw_image);
        void regress(cv::Mat image, bool update);

        void show_view(cv::Mat mask);
        void show_view(cv::Mat mask, cv::Mat raw_img);

        void update_filter(cv::Point* filter, int filter_size, cv::Point new_data);
        void update_filter(double* filter, int filter_size, double new_data);
        void update_filter(int* filter, int filter_size, int new_data);
        
        cv::Point   moving_average(cv::Point* filter, int filter_size);
        double      moving_average(double* filter, int filter_size);
        int         moving_average(int* filter, int filter_size);


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
         LineDetector(ros::NodeHandle* nh_in);
         LineDetector(ros::NodeHandle* nh_in, bool show_view);
         LineDetector(boost::shared_ptr<ros::NodeHandle> nh_in);
         LineDetector(boost::shared_ptr<ros::NodeHandle> nh_in, bool show_view);
         ~LineDetector();
        
        //          << METHODS >>
        
        void open_display();
        void close_display();

        // Input negative arguments for gains to keep them at previous values
        void set_PID(double kp, double kd, double ki);
        double get_PID(char term);
        
        void set_HSV(cv::Scalar low_bounds, cv::Scalar high_bounds);

        // Returns true if on path, false if not
        bool on_path();

        // Returns Twist message to rotate bot one "step" toward line direction. 
        geometry_msgs::Twist return_to_path();



};

#endif
