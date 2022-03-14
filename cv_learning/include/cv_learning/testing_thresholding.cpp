#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <libfreenect/libfreenect.h>

Kinect kinect;

image_transport::Subscriber image_sub;
image_transport::Publisher image_pub;

void image_handler(const sensor_msgs::ImageConstPtr &img)
{
    //convert ros image message to open-cv compatible BGR image
    cv::Mat image;
    image = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8)->image;

    int depth[] = kinect.getRawDepth();

    float sumX = 0;
    float sumY = 0;
    float totalPixels = 0;

    for (int x = 0; x < kinect.depthWidth; x++) {
        for (int y = 0; y < kinect.depthHeight; y++) {
            int offset = x + y * kinect.depthWidth;
            int d = depth[offset];

            if (d > minThresh && d < maxThresh && x > 100) {
                image.pixels[offset] = color(255, 0, 150);

                sumX += x;
                sumY += y;
                totalPixels++;
            } else {
                img.pixels[offset] = color(0);
            }
        }
    }
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
    //image_sub = it_.subscribe("/camera/rgb/image_raw", 1, image_handler);
    image_pub = it_.advertise("image_output", 1);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}