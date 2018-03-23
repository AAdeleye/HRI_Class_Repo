//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

int main(int argc, char** argv){

    cv::Mat image = cv::imread("Square.jpg");

    printf("Program ran!\n");
    return 0;
}
