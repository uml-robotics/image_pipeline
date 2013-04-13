#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <boost/format.hpp>

#include <std_srvs/Empty.h>

int g_count = 0;
boost::format g_format;
bool save_all_image, save_image_service;
std::string encoding;

bool service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  save_image_service = true;
  return true;
}

void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info)
{
  cv::Mat image;
  try
  {
    image = cv_bridge::toCvShare(image_msg, encoding)->image;
  } catch(cv_bridge::Exception)
  {
    ROS_ERROR("Unable to convert %s image to bgr8", image_msg->encoding.c_str());
    return;
  }

    if (!image.empty()) {
      std::string filename;
      try {
        filename = (g_format).str();
      } catch (...) { g_format.clear(); }
      try {
        filename = (g_format % g_count).str();
      } catch (...) { g_format.clear(); }
      try { 
        filename = (g_format % g_count % "jpg").str();
      } catch (...) { g_format.clear(); }

      if ( save_all_image || save_image_service ) {
        cv::imwrite(filename, image);
        ROS_INFO("Saved image %s", filename.c_str());
        filename = filename.replace(filename.rfind("."), filename.length(),".ini");
        camera_calibration_parsers::writeCalibration(filename, "camera", *info);

        g_count++;
        save_image_service = false;
      }
    } else {
      ROS_WARN("Couldn't save image, no data!");
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  std::string topic = nh.resolveName("image");
  image_transport::CameraSubscriber sub = it.subscribeCamera(topic, 1, &callback);

  ros::NodeHandle local_nh("~");
  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("left%04i.%s"));
  local_nh.param("encoding", encoding, std::string("bgr8"));
  local_nh.param("save_all_image", save_all_image, true);
  g_format.parse(format_string);
  ros::ServiceServer save = local_nh.advertiseService ("save", service);

  ros::spin();
}
