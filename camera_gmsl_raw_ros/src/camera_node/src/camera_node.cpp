
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "../external/main.hpp"

#include <sstream>

class CameraNode {

private:
  int m_seq;
  ros::NodeHandle m_nh;
  ros::Publisher m_pub;


public:
  CameraNode() {
    m_seq = 1;
    m_pub = m_nh.advertise<sensor_msgs::Image>("camera/image", 1000);
  }

  void publishImage(unsigned long long timestampUs, const uint8_t* data, unsigned int width, unsigned int height) {
    int time_sec = timestampUs / 1e6;
    int time_nsec = (timestampUs - time_sec * 1e6) * 1000;
    ros::Time stamp(time_sec, time_nsec);
    
    std_msgs::Header header;
    header.stamp = stamp;
    header.seq = m_seq++;

    sensor_msgs::Image image_msg;
    image_msg.header = header;
    image_msg.encoding = sensor_msgs::image_encodings::RGB8;
    image_msg.height = height;
    image_msg.width = width;
    image_msg.step = width * 3;
    int size = width * height * 3;
    image_msg.data.resize(size);
    memcpy(image_msg.data.data(), data, size);
    m_pub.publish(image_msg);
  }

};


CameraNode* cameraNode = nullptr;

void imgCb(unsigned long long timestampUs, const unsigned char* image, unsigned int w, unsigned int h)
{
  //std::cout << "imgCb" << w << "," <<h << std::endl;
  cameraNode->publishImage(timestampUs, image, w, h);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_node");
  cameraNode = new CameraNode();

  int ret = run_main(imgCb);

  delete cameraNode;
  return ret;

  // ros::Rate loop_rate(10);
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  // return 0;
}

