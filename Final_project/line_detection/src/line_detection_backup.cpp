
#include "line_detection.h"

/****************************************************************************/
/******************************            **********************************/
/********************    CONSTRUCTOR / DESTRUCTOR      **********************/
/******************************            **********************************/
/****************************************************************************/


LineDetector::LineDetector(bool show_view) : it(nh) {
    display_image = show_view;

   // default fields
    no_blob_count = 0;
    window_name = "Line Detector View";
    image_w = 0;
    image_h = 0;
    
    // set up PID control parameters        
    pid.kp = 0.005;
    pid.kd = 0.0000001;
    pid.ki = 0.00000001;
    pid.imax = 7200000;
    pid.imin = 0;
    pid.error_last = 0;
    pid.error_integral = 0;
    
    on_path_allowance = 0.60;    

    line.slope = 0;
    line.intercept = 0;
    line.centroid = cv::Point(-1, -1);
    line.r = 0;
    
    if(display_image) {            
         open_display();
     }
    image_sub = it.subscribe("webcam/image", 1, &LineDetector::image_callback, this);
    blob_sub = nh.subscribe("blobs", 1, &LineDetector::blobs_callback, this);
} 

LineDetector::LineDetector() : it(nh) {
   // default fields
    display_image = false;
    no_blob_count = 0;
    window_name = "Line Detector View";
    image_w = 0;
    image_h = 0;

    // set up PID control parameters        
    pid.kp = 0.005;
    pid.kd = 0.0000001;
    pid.ki = 0.00000001;
    pid.imax = 7200000;
    pid.imin = 0;
    pid.error_last = 0;
    pid.error_integral = 0;
    
    on_path_allowance = 0.60;

    line.slope = 0;
    line.intercept = 0;
    line.centroid = cv::Point(-1, -1);
    line.r = 0;
    

    if(display_image) {            
         open_display();
     }
    image_sub = it.subscribe("webcam/image", 1, &LineDetector::image_callback, this);
    blob_sub = nh.subscribe("blobs", 1, &LineDetector::blobs_callback, this);
}

LineDetector::~LineDetector(){
     close_display();
}


    /****************************************************************************/
    /******************************            **********************************/
    /**********************      HELPER FUNCTIONS        ************************/
    /******************************            **********************************/
    /****************************************************************************/

    /**************************************************************************
     *                          <<< FILTERING >>>
     * ***********************************************************************/

    // Update filter with new data
    void LineDetector::update_filter(cv::Point* filter, int filter_size, cv::Point new_data){
        for(int i = filter_size-1; i > 0; i--){
            filter[i].x = filter[i-1].x;
            filter[i].y = filter[i-1].y;
        }
        filter[0].x = new_data.x;
        filter[0].y = new_data.y;
    }
    void LineDetector::update_filter(double* filter, int filter_size, double new_data){
        for(int i = filter_size-1; i > 0; i--){
            filter[i] = filter[i-1];
        }
        filter[0] = new_data;
    }
    void LineDetector::update_filter(int* filter, int filter_size, int new_data){
        for(int i = filter_size-1; i > 0; i--){
            filter[i] = filter[i-1];
        }
        filter[0] = new_data;
    }
    // MOVING AVERAGE OUTPUT
    double LineDetector::moving_average(double* filter, int filter_size){
        double sum = 0;
        for(int i = 0; i < filter_size; i++){
             sum += filter[i];
        }
        return sum / (double)(filter_size);
    }
    int LineDetector::moving_average(int* filter, int filter_size){
        int sum = 0;
        for(int i = 0; i < filter_size; i++){
            sum += filter[i];
        }
        return (int) (sum / filter_size);
    }
    cv::Point LineDetector::moving_average(cv::Point* filter, int filter_size){
        int x = 0;
        int y = 0;
        for(int i = 0; i < filter_size; i++){
            x += filter[i].x;
            y += filter[i].y;
        }
        return cv::Point(x,y);
    }


    /****************************************************************************/
    /******************************            **********************************/
    /**********************          CALLBACKS           ************************/
    /******************************            **********************************/
    /****************************************************************************/


    /***************************************************************
     * Function Name: blobs_callback
     * Parameters: const cmvision::Blobs
     * Returns: void
     *
     * Description: This is the callback function for the /blobs topic.
     *              It calculates the linear regression of the blobs.
    ****************************************************************/
    void LineDetector::blobs_callback(const cmvision::Blobs blobs_in){
        double slope, r;
        int intercept;
        cv::Point centroid;

        int n = blobs_in.blob_count; 
        // If there are blobs detected, do a linear regression to find the line of best
        // fit through the blobs.
        if(n>0){
            // set no_blobs counter back to 0
            no_blob_count = 0;

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
            if  ( abs(den) > 0.00001)    // handle divide by approx zero
                slope = num / den;
            else 
                slope = 0;
            
            // intercept
            num = (sum_y*sum_xx - sum_x*sum_xy);
              // den is the same in both formulas   
            if (abs(den) >= 0.00001)  // handle divide by approx zero
                intercept = num / den;
            else 
                intercept = image_h/2;
            
            // centroid
            centroid = cv::Point(sum_x/n, sum_y/n);
            
            // correlation coefficient
            num = (n*sum_xy - sum_x*sum_y);
            den = sqrt( (n*sum_xx - sum_x*sum_x) * (n*sum_yy - sum_y*sum_y) );
            if (abs(den) >= 0.00001) // handle divide by approx zero
                r = num / den;
            else 
                r = -1; // error value
                    
            //update filters
            update_filter(slope_filter, _LD_SLOPE_MEM, slope);
            update_filter(intercept_filter, _LD_INTERCEPT_MEM, intercept);
            update_filter(centroid_filter, _LD_CENTROID_MEM, centroid);    
            update_filter(r_filter, _LD_R_MEM, r);
        } else {
            // If there are no blobs detected, increment no_blobs timer.
            no_blob_count++;
        }
        

        // update filtered line
        line.slope =        moving_average(slope_filter, _LD_SLOPE_MEM);
        line.intercept =    moving_average(intercept_filter, _LD_INTERCEPT_MEM);
        line.centroid =     moving_average(centroid_filter, _LD_CENTROID_MEM);
        line.r =            moving_average(r_filter, _LD_R_MEM);
    }



    /***************************************************************
     * Function Name: image_callback
     * Parameters: const sensor_msgs::ImageConstPtr& msg
     * Returns: void
     *
     * Description: This is the callback function for the /webcam/image topic.
     *              It displays the image and the current linear regression.
     ****************************************************************/
    void LineDetector::image_callback(const sensor_msgs::ImageConstPtr& msg){
        try {
            // convert to OpenCV message
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            image_w = frame.cols;
            image_h = frame.rows;

            // If centroid x is less than 0, we assume no blobs are visible and skip the processing.
            if( line.centroid.x >= 0) {
            // >>>> MARK CENTROID AND AVERAGE LINE ON IMAGE

                // >>> Line handling.
                cv::Point pt1 = cv::Point(0, line.intercept); //my_line.centroid;
                cv::Point pt2 = cv::Point(image_w, line.slope * image_w + line.intercept);

                if(display_image){
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
                                cv::Point(line.centroid.x, image_h/2),
                                r,
                                cv::Scalar(0, 0, 255),
                                line_thickness );
                }

            }
            else {
                 //   std::cout << "No blobs detected." << std::endl;   
            }
            // Display image on window
            if(display_image){
                cv::imshow(window_name, frame);
                cv::waitKey(30);
            }
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }


    /****************************************************************************/
    /******************************            **********************************/
    /**********************      PUBLIC FUNCTIONS        ************************/
    /******************************            **********************************/
    /****************************************************************************/

        /***********************************************************
         *                      HELPER METHODS
         ***********************************************************/

    void LineDetector::open_display(){
        cv::namedWindow(window_name);
        cv::startWindowThread();    
    }

    void LineDetector::close_display(){
         cv::destroyWindow(window_name);
    }

    int LineDetector::get_no_blob_count(){
        return no_blob_count;
    }

    // Returns the y coordinate of the detected line when it goes out of the
    // image window by the bot (to the right).
    cv::Point LineDetector::get_bot_intercept(){
        return cv::Point(image_w, line.slope * image_w + line.intercept);
    }

    int LineDetector::get_image_width(){
        return image_w;
    }

    int LineDetector::get_image_height(){
        return image_h;
    }

    void LineDetector::set_on_path_allowance(double percent_of_height){

        // Print error message if percent_of_height isn't a valid value
        if(abs(percent_of_height) > 1){
            std::cout << "ERROR: LineDetector::on_path() argument invalid - out of 0-100% range. ";
            std::cout   << "Reverting to old on_path_allowance value: "
                        << on_path_allowance << "." << std::endl;
        } else 
            on_path_allowance = percent_of_height;
    }
        /***********************************************************
         *                      MAIN METHODS
         ***********************************************************/
    // NOTE: Percent should be a decimal value between 0.00 and 1.00
    bool LineDetector::on_path(){
        // if we haven't seen a blob in NO_BLOBS_CUTOFF updates, we're off path
        if(no_blob_count > NO_BLOBS_CUTOFF){
            return false;
        } 

        // if the intercept is outside of on_path_allowance% of the window height, we're off path
        else {
            int y = LineDetector::get_bot_intercept().y;
            if ( abs( y - image_h/2 ) > (LineDetector::on_path_allowance/2)*image_h ) {  
               return false; 
            }
        }
        // otherwise, on path
        return true;
    }


geometry_msgs::Twist LineDetector::return_to_path(){
    // PID control: turn to center line centroid
    int error = line.centroid.y - image_h;    
    double p_term = pid.kp * error;
    double d_term = pid.kd * (error - pid.error_last);

    pid.error_integral += error;
    // Make sure i_state isn't out of bounds
    if(pid.error_integral > pid.imax) pid.error_integral = pid.imax;
    if(pid.error_integral < pid.imin) pid.error_integral = pid.imin;
    double i_term = pid.ki * pid.error_integral;

    double control_signal = p_term + d_term + i_term;
    
    pid.error_last = error;

    // construct twist message 
    geometry_msgs::Twist t;
    t.linear.x = 0; t.linear.y = 0; t.linear.z = 0;
    t.angular.x = 0; t.angular.y = 0; t.angular.z = control_signal;

    return t;
}



/*****************************************************************************************/

int main(int argc, char** argv){
    ros::init(argc, argv, "line_detection");
//    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
    geometry_msgs::Twist T;

    LineDetector ld = LineDetector(true);
    
    double percent_error = 0.6;
    ld.set_on_path_allowance(percent_error);

    ros::Rate loop_rate(10);
    while(ros::ok()){
        if(ld.on_path()){
            std::cout << "On path. ";
        } else {
            std::cout << "Not on path. ";
            T = ld.return_to_path();
        }
        cv::Point bi = ld.get_bot_intercept();
        int image_h = ld.get_image_height();
        std::cout <<    "Intercept at "    << bi 
                  <<    ", bounds at "     << (0.5-percent_error/2)*image_h 
                  <<    ", and "           << (0.5+percent_error/2)*image_h << std::endl;
        
        ros::spinOnce();
        vel_pub.publish(T);
        loop_rate.sleep();
    }
}
