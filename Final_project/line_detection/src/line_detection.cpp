
//#include <line_detection/line_detection.h>
#include "/home/turtlebot/squirtlebot_ws/src/line_detection/include/line_detection/line_detection.h"

#define KP_DEFAULT       3
#define KI_DEFAULT       0.005
#define KD_DEFAULT       0.5
#define CTRL_SIG_MAX     0.5

#define SIGN(x)         (x > 0 ? 1 : (x < 0 ? -1 : 0))

/****************************************************************************/
/******************************            **********************************/
/********************    CONSTRUCTOR / DESTRUCTOR      **********************/
/******************************            **********************************/
/****************************************************************************/


LineDetector::LineDetector(ros::NodeHandle* nh_in, bool show_view) : nh(*nh_in), it(*nh_in) {
    display_image = show_view;

   // >>> Default fields
    window_name = "Path Detector View";
    image_center = cv::Point(-1, -1);

    hsv_low = cv::Scalar(100, 100, 0);
    hsv_high = cv::Scalar(150, 250, 250);

    // set up PID control parameters        
    pid.kp = KP_DEFAULT;
    pid.kd = KD_DEFAULT;
    pid.ki = KI_DEFAULT;
    pid.imax = 7200000;
    pid.imin = 0;
    pid.error_last = 0;
    pid.error_integral = 0;
    
    path.slope = 0;
    path.centroid = cv::Point(-1, -1);
    path.num_points = 0;
    
    if(display_image) {            
         open_display();
     }
    image_sub = it.subscribe("webcam/image", 1, &LineDetector::image_callback, this);
} 

LineDetector::LineDetector(ros::NodeHandle* nh_in) : nh(*nh_in), it(*nh_in) {
   // >>> Default fields
    display_image = false;
    window_name = "Path Detector View";
    image_center = cv::Point(-1, -1);
    
    hsv_low = cv::Scalar(100, 100, 0);
    hsv_high = cv::Scalar(150, 250, 250);

    // set up PID control parameters        
    pid.kp = KP_DEFAULT;
    pid.kd = KD_DEFAULT;
    pid.ki = KI_DEFAULT;
    pid.imax = 7200000;
    pid.imin = 0;
    pid.error_last = 0;
    pid.error_integral = 0;
    
    path.slope = 0;
    path.centroid = cv::Point(-1, -1);
    path.num_points = 0;    

    if(display_image) {            
         open_display();
     }
    image_sub = it.subscribe("webcam/image", 1, &LineDetector::image_callback, this);
}


LineDetector::LineDetector(boost::shared_ptr<ros::NodeHandle> nh_in) : nh(*nh_in), it(*nh_in) {
   // >>> Default fields
    display_image = false;
    window_name = "Path Detector View";
    image_center = cv::Point(-1, -1);
    
    hsv_low = cv::Scalar(100, 100, 0);
    hsv_high = cv::Scalar(150, 250, 250);

    // set up PID control parameters        
    pid.kp = KP_DEFAULT;
    pid.kd = KD_DEFAULT;
    pid.ki = KI_DEFAULT;
    pid.imax = 7200000;
    pid.imin = 0;
    pid.error_last = 0;
    pid.error_integral = 0;
    
    path.slope = 0;
    path.centroid = cv::Point(-1, -1);
    path.num_points = 0;    

    if(display_image) {            
         open_display();
     }
    image_sub = it.subscribe("webcam/image", 1, &LineDetector::image_callback, this);

}

LineDetector::LineDetector(boost::shared_ptr<ros::NodeHandle> nh_in, bool show_view) 
                                                                : nh(*nh_in), it(*nh_in) {
    display_image = show_view;
   // >>> Default fields
    window_name = "Path Detector View";
    image_center = cv::Point(-1, -1);
    
    hsv_low = cv::Scalar(100, 100, 0);
    hsv_high = cv::Scalar(150, 250, 250);

    // set up PID control parameters        
    pid.kp = KP_DEFAULT;
    pid.kd = KD_DEFAULT;
    pid.ki = KI_DEFAULT;
    pid.imax = 7200000;
    pid.imin = 0;
    pid.error_last = 0;
    pid.error_integral = 0;
    
    path.slope = 0;
    path.centroid = cv::Point(-1, -1);
    path.num_points = 0;    

    if(display_image) {            
         open_display();
     }
    image_sub = it.subscribe("webcam/image", 1, &LineDetector::image_callback, this);

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
        return cv::Point( x/filter_size, y/filter_size );
    }

    cv::Mat LineDetector::create_mask(cv::Mat raw_image){
        cv::Mat mask;
        // convert to HSV
        cv::cvtColor(raw_image, mask, cv::COLOR_BGR2HSV);
        // threshold
        cv::inRange(mask, hsv_low, hsv_high, mask);

        // morphological opening (remove small objects from foreground)
        cv::erode(mask, mask, 
                cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
        cv::dilate(mask, mask, 
                cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
        // morphological closing (fill small holes in foreground)
        cv::dilate(mask, mask, 
                cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
        cv::erode(mask, mask, 
                cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

        return mask;
    }
    

    void LineDetector::regress(cv::Mat image, bool update){
        // find centroid
        cv::Moments m = cv::moments(image);
        if(update){
            update_filter(points_filter, _LD_FILTER_MEM, m.m00);
            path.num_points = moving_average(points_filter, _LD_FILTER_MEM);
        }
        
        if( m.m00 > _LD_POINTS_CUTOFF){
            //std::cout << m.m00 << std::endl;
            // centroid
            cv::Point centroid = cv::Point( m.m10/m.m00, m.m01/m.m00 );                
            // slope
            double slope;
            double num = (m.m00*m.m11 - m.m10*m.m01)/m.m00/m.m00;
            double den = (m.m00*m.m20 - m.m10*m.m10)/m.m00/m.m00;
            if( den*SIGN(den) < 0.00001 ){
                den *= 1000;
                num *= 1000;
            }
            
            slope = num/den;


            if(update){

                update_filter(centroid_filter, _LD_FILTER_MEM, centroid);
                path.centroid = moving_average(centroid_filter, _LD_FILTER_MEM);
                                
                update_filter(slope_filter, _LD_FILTER_MEM, slope);
                path.slope = moving_average(slope_filter, _LD_FILTER_MEM);
            }
        } 
        // else {
        //    std::cout << "X No path found, n = " << m.m00 << std::endl;
        // }
    }


    void LineDetector::show_view(cv::Mat mask){
        cv::circle( mask,
                    path.centroid, 
                    5,
                    cv::Scalar(0, 0, 255),
                    -1);

        cv::imshow(window_name, mask);
        cv::waitKey(30);
    }

    void LineDetector::show_view(cv::Mat mask, cv::Mat raw_img){
        cv::Mat disp_img;
        raw_img.copyTo(disp_img, mask);

        cv::circle( disp_img,
                    path.centroid, 
                    5,
                    cv::Scalar(0, 0, 255),
                    -1);

        cv::imshow(window_name, disp_img);
        cv::waitKey(30);
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
            image_center = cv::Point(0.5*frame.cols, 0.5*frame.rows);

            cv::Mat mask = create_mask(frame);
            regress(mask, true);

            if(display_image){
                show_view(mask, frame);
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
    
    // Input negative arguments for gains to keep them at previous values
    void LineDetector::set_PID(double kp, double kd, double ki){
        if( kp >= 0 )
            pid.kp = kp;
        if( kd >= 0 )
            pid.kd = kd;
        if( kp >= 0 )
            pid.ki = ki;
    }

    double LineDetector::get_PID(char term){
        switch (term) {
            case 'p': 
                return pid.kp;
            case 'i':
                return pid.ki;
            case 'd':
                return pid.kd;
            default:
                std::cout << "ERROR: Invalid argument input to LineDetector::getPID(int term)."
                    << " term should be one of the following: 'p' for Kp, 'i' for Ki, or 'd' for Kd."
                    << std::endl;
                return 0;
        }
    }

    void LineDetector::set_HSV(cv::Scalar low_bounds, cv::Scalar high_bounds){
        hsv_low = low_bounds;
        hsv_high = high_bounds;
    }
        /***********************************************************
         *                      MAIN METHODS
         ***********************************************************/
    bool LineDetector::on_path(){
        std::cout << "On path." << std::endl;
        return (path.num_points > _LD_POINTS_CUTOFF);
    }





    geometry_msgs::Twist LineDetector::return_to_path(){
        // PID control
        geometry_msgs::Twist t;
        double control_signal = 0;
        double error = 0;

        // if we can see some of the path, turn until we make the path straight
        if(path.num_points > 8000){
            std::cout << "Slope control. ";
            error = -path.slope;
            if ( error*SIGN(error) > _LD_ERROR_THRESHOLD ) {    
                double p_term = pid.kp * error;
                double d_term = pid.kd * (error - pid.error_last);

                pid.error_integral += error;
                // Make sure i_state isn't out of bounds
                if(pid.error_integral > pid.imax) pid.error_integral = pid.imax;
                if(pid.error_integral < pid.imin) pid.error_integral = pid.imin;
                double i_term = pid.ki * pid.error_integral;

                control_signal = p_term + d_term + i_term;
                
                pid.error_last = error;
                std::cout << " p: " << p_term << "; d: " 
                    << d_term << "; i: " << i_term 
                    << "; sum: " << p_term + d_term + i_term << std::endl; 
            }
       // Otherwise, if we can't see the path, turn toward the centroid until we 
        // can see the path. Because the slope PID controller will eventually 
        // take over, we can use a less aggressive proportional controller.
        } else {
            std::cout << "Centroid control. ";
            error = path.centroid.y - image_center.y;
            std::cout << "Centroid: " << path.centroid.y << 
                "; Image center: " << image_center.y << std::endl;
            double kp = 0.05; 
            control_signal = kp*error;

        }

       // don't turn so fast that we can't update from image_callback() fast enough
        if( SIGN(control_signal)*control_signal > CTRL_SIG_MAX) {
            control_signal = SIGN(control_signal)*CTRL_SIG_MAX;
        }

        // construct twist message 
        t.linear.x = 0; t.linear.y = 0; t.linear.z = 0;
        t.angular.x = 0; t.angular.y = 0; t.angular.z = control_signal;

        std::cout << " error: " << error << ";   control_signal = " 
                  << control_signal << std::endl;

        return t;
    }



/*****************************************************************************************/

int main(int argc, char** argv){
    
    ros::init(argc, argv, "line_detection");
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    ros::Publisher vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
    geometry_msgs::Twist T;

    LineDetector ld = LineDetector(nh, true);
    
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        if ( !ld.on_path()){
            T = ld.return_to_path();
            if( T.angular.z != 0 )
                vel_pub.publish( T );
        }
        loop_rate.sleep();
    }
    
}
