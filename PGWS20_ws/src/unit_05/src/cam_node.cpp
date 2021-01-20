#include "ros/ros.h"

#include "opencv2/core.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher out_img_pub =n.advertise<sensor_msgs::Image>("image_out", 2, false);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {

    // assuming you have declared and filled cv::Mat your_mat with data in colorspace "bgr8"
  cv_bridge::CvImage lo_img;

  lo_img.encoding = "bgr8";                        // or which enconding your data has
  lo_img.header.stamp = ros::Time::now();          //  or whatever timestamp suits here;
  lo_img.header.frame_id = "whatever_frame";       // frame id as neededby you
  lo_img.image = your_mat;                          // point cv_bridge to your object

// publishing data
out_img_pub.publish( lo_img.toImageMsg() );
    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}
