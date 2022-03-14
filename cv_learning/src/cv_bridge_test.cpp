#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

image_transport::Subscriber image_sub;
image_transport::Publisher image_pub;

void image_handler(const sensor_msgs::ImageConstPtr &img)
{
    //convert ros image message to open-cv compatible BGR image
    cv::Mat image;
    image = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8)->image;

    cv::Mat gray, blur, canny;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(3,3), 0);
    cv::Canny(blur, canny, 100, 200, 3);

    //create container for line data
    std::vector<cv::Vec4i> lines;
    //find lines in canny image and output them to lines vector
    cv::HoughLinesP(canny, lines, 2, CV_PI/180, 50, 50, 100);

    //copy original image for drawing on
    cv::Mat imcopy = image.clone();
    //make blank image of the same size for drawing on
    cv::Mat drawing = Mat::zeros(canny.size(), CV_8UC3);
    //iterate over output vector lines, drawing the lines
    for(int i=0; i<lines.size(); i++)
    {
        cv::Vec4i line = lines[i];
        int x1 = line[0];
        int y1 = line[1];
        int x2 = line[2];
        int y2 = line[3];
        cv::line(drawing, cv::Point(x1, y1), cv::Point(x2, y2),
        cv::Scalar(255,0,0), 2);
    }

    cv::Mat mask1 = Mat::zeros(canny.size(), CV_8U);
    cv::rectangle(mask1, cv::Rect(0, image.size().height*2/5, image.size().width, image.size().height*3/5),
    cv::Scalar(255,255,255), -1);
    //create polygon-shaped mask
    //create blank drawing
    cv::Mat mask2 = Mat::zeros(canny.size(), CV_8U);
    //create vector of points to hold polygon vertices
    vector<cv::Point> poly(4);
    //create trapezoid-shaped polygon in poly
    poly[0]=cv::Point(0, image.size().height);
    poly[1]=cv::Point(image.size().width/3, image.size().height*2/5);
    poly[2]=cv::Point(image.size().width*2/3, image.size().height*3/5);
    poly[3]=cv::Point(image.size().width, image.size().height);
    //fill the area in mask2 defined by poly with white
    cv::fillConvexPoly(mask2, poly, Scalar(255));
    //apply the mask to an image
    cv::Mat masked = Mat::zeros(canny.size(), CV_8U);
    cv::bitwise_and(canny, canny, masked, mask1);

    //use cv_bridge to convert opencv image to ros image message
    //create image message
    sensor_msgs::Image::Ptr output_img;

    //convert opencv image we drew on to ROS image message
    output_img = cv_bridge::CvImage(img->header, "mono8", masked).toImageMsg();
    //publish our ros image message
    image_pub.publish(output_img);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cv_bridge_test");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_(nh);

    // Subscribe to input video feed and publish output video feed
    image_sub = it_.subscribe("/camera/rgb/image_raw", 1, image_handler);
    image_pub = it_.advertise("image_output", 1);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}