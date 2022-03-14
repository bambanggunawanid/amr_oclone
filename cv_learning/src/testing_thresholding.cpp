#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

image_transport::Subscriber image_sub;
image_transport::Publisher image_pub;

float minThresh = 0;
float maxThresh = 55000;
int max = 0;
int min = 0;

void image_handler(const sensor_msgs::ImageConstPtr &img)
{
    //convert ros image message to open-cv compatible BGR image
    cv::Mat image;
    image = cv_bridge::toCvShare(img)->image;

    cv::Mat mono8_img = cv::Mat(image.size(), CV_8UC1);
    cv::convertScaleAbs(image, mono8_img, 50, 0.0);

    float sumX = 0;
    float sumY = 0;
    float totalPixels = 0;

    cv::Mat newimage = cv::Mat::zeros(image.size(), CV_16UC1);

    for (int x = 0; x < mono8_img.cols; x++) {
        for (int y = 0; y < mono8_img.rows; y++) {
//          int offset = x + y * mono8_img.col;
            int d = mono8_img.at<uchar>(y,x);
            if (d>max){max = d;}
            if (d<min){min = d;}
            //std::cout << max << " " << min << std::endl;
            if (d > 35 && d < 60) {
                newimage.at<uchar>(y,x) = 100;
                sumX += x;
                sumY += y;
                totalPixels++;
            } else {
                //newimage.at<float>(y,x) = 0;
            }
        }
    }

    float avgX = sumX / totalPixels;
    float avgY = sumY / totalPixels;
    // int width = 16;
    int heigh = 4;

    for (int x = 0; x < newimage.cols; x++) {
        for (int y = avgY-(heigh/2); y < avgY+(heigh/2); y++) {
            newimage.at<uchar>(y,x) = 200;
        }
    }

    //use cv_bridge to convert opencv image to ros image message
    //create image message
    sensor_msgs::Image::Ptr output_img;

    //convert opencv image we drew on to ROS image message
    output_img = cv_bridge::CvImage(img->header, "mono8", newimage).toImageMsg();
    //publish our ros image message
    image_pub.publish(output_img);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cv_bridge_test");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_(nh);

    // Subscribe to input video feed and publish output video feed
    image_sub = it_.subscribe("/camera/depth/image", 1, image_handler);
    image_pub = it_.advertise("image_output", 1);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}