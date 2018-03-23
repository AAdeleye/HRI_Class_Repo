
/****************************************************************************/
/******************************            **********************************/
/**********************          INCLUDES        ****************************/
/******************************            **********************************/
/****************************************************************************/


// Basic includes
#include <stdio.h>
#include <vector>
#include <time.h>
#include <math.h>
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmvision/Blobs.h>
#include <image_transport/image_transport.h>
// OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "line_detection.h"

/****************************************************************************/
/******************************            **********************************/
/**********************      MACROS / GLOBALS        ************************/
/******************************            **********************************/
/****************************************************************************/

#define FILTER_MEM      5
#define SLOPE_MEM       5
#define INTERCEPT_MEM   5
#define CENTROID_MEM    5

// allowed number of consecutive blobs_callback cycles without detecting any balls
//#define NO_BLOBS_CUTOFF 5
// counter for the above
int no_blobs = 0;

// window width and height for use in blob_callback
int w = 500;
int h = 500;

ros::Publisher pub;
std::string window_name = "SquirtleBot View";

struct Line{
    double slope;
    int intercept;
    cv::Point centroid;
} current_line;

struct LineFilter{
    double slope_filter[SLOPE_MEM];
    int intercept_filter[INTERCEPT_MEM];
    cv::Point centroid_filter[CENTROID_MEM];
} line_filter;

double slope_filter[SLOPE_MEM];
int intercept_filter[INTERCEPT_MEM];
cv::Point centroid_filter[CENTROID_MEM];

/****************************************************************************/
/******************************            **********************************/
/**********************          METHODS             ************************/
/******************************            **********************************/
/****************************************************************************/

//                      <<< FILTERING METHODS >>>

// Update filter with new data
void update_filter(cv::Point* filter, int filter_size, cv::Point new_data){
    for(int i = filter_size-1; i > 0; i--){
        filter[i].x = filter[i-1].x;
        filter[i].y = filter[i-1].y;
    }
    filter[0].x = new_data.x;
    filter[0].y = new_data.y;
}
void update_filter(double* filter, int filter_size, double new_data){
    for(int i = filter_size-1; i > 0; i--){
        filter[i] = filter[i-1];
    }
    filter[0] = new_data;
}
void update_filter(int* filter, int filter_size, int new_data){
    for(int i = filter_size-1; i > 0; i--){
        filter[i] = filter[i-1];
    }
    filter[0] = new_data;
}
void update_filter(LineFilter filter, Line new_data){
    update_filter(filter.slope_filter, SLOPE_MEM, new_data.slope);
    update_filter(filter.intercept_filter, INTERCEPT_MEM, new_data.intercept);
    update_filter(filter.centroid_filter, CENTROID_MEM, new_data.centroid);
}

// MOVING AVERAGE OUTPUT
double moving_average(double* filter, int filter_size){
    double sum = 0;
    for(int i = 0; i < filter_size; i++){
        sum += filter[i];
    }
    return sum / (double)(filter_size);
}
int moving_average(int* filter, int filter_size){
    int sum = 0;
    for(int i = 0; i < filter_size; i++){
        sum += filter[i];
    }
    return (int) (sum / filter_size);
}
cv::Point moving_average(cv::Point* filter, int filter_size){
    int x = 0;
    int y = 0;
    for(int i = 0; i < filter_size; i++){
        x += filter[i].x;
        y += filter[i].y;
    }
    return cv::Point(x,y);
}
Line moving_average(LineFilter filter){
    Line out;
    out.slope = moving_average(filter.slope_filter, SLOPE_MEM);
    out.intercept = moving_average(filter.intercept_filter, INTERCEPT_MEM);
    out.centroid = moving_average(filter.centroid_filter, CENTROID_MEM);
    return out;
}

// WEIGHTED MOVING AVERAGE
double weighted_average(double* filter, int filter_size){
    double avg = moving_average(filter, filter_size);
    double weight[filter_size];
    double weight_sum;
    for (int i=0; i<filter_size; i++){
        weight[i] = abs(filter[i]-avg);   
        weight_sum += weight[i];
    }
    double output;
    for(int i=0; i<filter_size; i++){
        double norm_weight = weight[i] / weight_sum;
        output += norm_weight * filter[i];
    }
    return output;
}
int weighted_average(int* filter, int filter_size){
    double avg = moving_average(filter, filter_size);
    double weight[filter_size];
    double weight_sum;
    for (int i=0; i<filter_size; i++){
        weight[i] = abs(filter[i]-avg);   
        weight_sum += weight[i];
    }
    double output;
    for(int i=0; i<filter_size; i++){
        double norm_weight = weight[i] / weight_sum;
        output += norm_weight * filter[i];
    }
    return (int)(output);
}

// LOWPASS FILTER
void lowpass(double* signal_in, int signal_size, double RC){
    double alpha = 1 / RC;
    signal_in[0] *= alpha;
    for(int i=1; i<signal_size; i++){
        signal_in[i] = signal_in[i-1] + alpha * (signal_in[i]-signal_in[i-1]);
                    // RHS signal_in[i] here is unfilted, while signal_in[i-1] is already filtered
    }
}
void lowpass(int* signal_in, int signal_size, double RC){
    double alpha = 1 / RC;
    double signal_last = signal_in[0]*alpha;
    for(int i=1; i<signal_size; i++){
        signal_in[i-1] = (int)signal_last;
        signal_last = signal_last + alpha * (signal_in[i]-signal_last);
                    // RHS signal_in[i] here is unfilted, while signal_in[i-1] is already filtered
    }
    signal_in[signal_size] = (int)signal_last;
}
/***************************************************************
 * Function Name: blobs_callback
 * Parameters: const cmvision::Blobs
 * Returns: void
 *
 * Description: This is the callback function for the /blobs topic.
 *              It calculates the linear regression of the blobs.
 ****************************************************************/
void blobs_callback(const cmvision::Blobs blobs_in){
    //Line current_line;
    int n = blobs_in.blob_count; 
    // If there are blobs detected, do a linear regression to find the line of best
    // fit through the blobs.
    if(n>0){
        // set no_blobs counter back to 0
        no_blobs = 0;

        int sum_x = 0;
        int sum_y = 0;
        int sum_xy = 0;
        int sum_xx = 0;
        int sum_yy = 0;
        for(int i=0; i<n; i++){
            int x = blobs_in.blobs[i].x;
            int y = blobs_in.blobs[i].y;
            
            sum_x += x;
            sum_y += y;
            sum_xy += x*y;
            sum_xx += x*x;
            sum_yy += y*y;
            
        }
        // calculate regression slope
        double num = (n*sum_xy - sum_x*sum_y);
        double den = (n*sum_xx - sum_x*sum_x);
        if  ( abs(den) > 0.00001) {   // handle divide by approx zero
            current_line.slope = num / den;
        } else current_line.slope = 0;
        
        // intercept
        num = (sum_y*sum_xx - sum_x*sum_xy);
          // den is the same in both formulas   
        if (abs(den) >= 0.00001) { // handle divide by approx zero
            current_line.intercept = num / den;
        } else current_line.intercept = h/2;
        
        // centroid
        current_line.centroid = cv::Point(sum_x/n, sum_y/n);
    
        //update filters
        update_filter(slope_filter, SLOPE_MEM, current_line.slope);
        update_filter(intercept_filter, INTERCEPT_MEM, current_line.intercept);
        update_filter(centroid_filter, CENTROID_MEM, current_line.centroid);    
    } else {
        // If there are no blobs detected, increment no_blobs timer.
        no_blobs++;
        // if we've reached NO_BLOBS_CUTOFF, set failure flag
        if(no_blobs > NO_BLOBS_CUTOFF){
            
        }
    }
}


/***************************************************************
 * Function Name: image_callback
 * Parameters: const sensor_msgs::ImageConstPtr& msg
 * Returns: void
 *
 * Description: This is the callback function for the /webcam/image topic.
 *              It displays the image and the current linear regression.
 ****************************************************************/
void image_callback(const sensor_msgs::ImageConstPtr& msg){
    try {
        // convert to OpenCV message
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        w = frame.cols;
        h = frame.rows;

        // get filtered line
        Line my_line;
        my_line.slope = moving_average(slope_filter, SLOPE_MEM);
        my_line.intercept = moving_average(intercept_filter, INTERCEPT_MEM);
        my_line.centroid = moving_average(centroid_filter, CENTROID_MEM);

        //my_line = moving_average(line_filter);
        //Line my_line = current_line;
        int x = my_line.centroid.x;

        // If centroid x is less than 0, we assume no blobs are visible and skip the processing.
        if( x >= 0) {
        // >>>> MARK CENTROID AND AVERAGE LINE ON IMAGE
        
            // >>> Line handling.
            cv::Point pt1 = cv::Point(0, my_line.intercept); //my_line.centroid;
            cv::Point pt2 = cv::Point(w, my_line.slope * w + my_line.intercept);

            std::cout << "Points: " << pt1 << "; " << pt2 << std::endl;
            std::cout << "Slope: " << my_line.slope << "; " << 
                "Intercept: " << my_line.intercept << std::endl;
            std::cout << "Centroid: " << my_line.centroid << std::endl << std::endl;
            // Mark line on image
            cv::line(   frame,
                        pt1,
                        pt2,
                        cv::Scalar(0, 0, 200),
                        7);

 
            // >>> Centroid handling.
            // Mark blob centroid on image
            int r = 10;
            int line_thickness = -1;    // filled circle
            cv::circle( frame,
                        cv::Point(x, h/2),
                        r,
                        cv::Scalar(0, 0, 255),
                        line_thickness );

            
            
            /*
            // If centroid is outside bounds of the base (approximated as
                // error_percent% of the image width away from image center), set failure flag
            double error_percent = 0.25;
            int offcenter = abs(x - 0.5*w);
            if( offcenter > error_percent*w){
                std::cout << "Failure, centroid x at = " << x << ", "
                   << offcenter << " pixels off-center." << std::endl;
            }

            // Mark error bounds on image
            
            cv::line( frame,
                      cv::Point( 0, -(0.5-error_percent)*h),
                      cv::Point( w, -(0.5-error_percent)*h),
                      cv::Scalar (0, 255, 255),
                      3 );
            cv::line( frame,
                      cv::Point( 0, -(0.5+error_percent)*h),
                      cv::Point( 0, -(0.5+error_percent)*h),
                      cv::Scalar (0, 255, 255),
                      3 );
            */
        } else {
                std::cout << "No blobs detected." << std::endl;   
        }
        // Display image on window
        cv::imshow(window_name, frame);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}



int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "line_detector_test");
    ros::NodeHandle nh;

    cv::namedWindow(window_name);
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("webcam/image", 1, image_callback);
    ros::Subscriber blob_sub = nh.subscribe("blobs", 1, blobs_callback);

    // Spin
    ros::spin ();
    cv::destroyWindow(window_name);
}  
