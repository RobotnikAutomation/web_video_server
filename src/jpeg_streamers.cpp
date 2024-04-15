#include "web_video_server/jpeg_streamers.h"
#include "async_web_server_cpp/http_reply.hpp"

namespace web_video_server
{

MjpegStreamer::MjpegStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
  ImageTransportImageStreamer(request, connection, nh), stream_(connection)
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);
  height_ = request.get_query_param_value_or_default<int>("height", -1);
  width_ = request.get_query_param_value_or_default<int>("width", -1);
  use_opencl_ = request.get_query_param_value_or_default<int>("optimization", -1);
  stream_.sendInitialHeader();
}

MjpegStreamer::~MjpegStreamer()
{
  this->inactive_ = true;
  boost::mutex::scoped_lock lock(send_mutex_); // protects sendImage.
}

void MjpegStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
{

  cv::Mat processed_img = img;
  
  cv::ocl::setUseOpenCL(use_opencl_>0 ? true: false);
  ros::Time lasttime=ros::Time::now();
  // Check if height and width are valid
  if (height_ > 0 && width_ > 0) {
    // Resize the image to the specified values if they are positive
    cv::resize(img, processed_img, cv::Size(width_, height_), 0, 0, cv::INTER_NEAREST);
  }
  // If no resizing is needed (i.e., using native resolution), processed_img remains equal to img

  std::vector<int> encode_params;
#if CV_VERSION_MAJOR >= 3
  encode_params.push_back(cv::IMWRITE_JPEG_QUALITY);
#else
  encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
#endif
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", processed_img, encoded_buffer, encode_params);
  int encoded_buffer_size = encoded_buffer.size();
  stream_.sendPartAndClear(time, "image/jpeg", encoded_buffer);
  ros::Time currtime=ros::Time::now();
  ros::Duration diff=currtime-lasttime;
  ROS_DEBUG("Buffer size: %i bytes.\t %s %lf s."
    , encoded_buffer_size
    ,(!cv::ocl::haveOpenCL() || !cv::ocl::useOpenCL() ? "Time without OpenCL:\t" : "Time with OpenCL:" )
    , diff.toSec()
  );
}

boost::shared_ptr<ImageStreamer> MjpegStreamerType::create_streamer(const async_web_server_cpp::HttpRequest &request,
                                                                    async_web_server_cpp::HttpConnectionPtr connection,
                                                                    ros::NodeHandle& nh)
{
  return boost::shared_ptr<ImageStreamer>(new MjpegStreamer(request, connection, nh));
}

std::string MjpegStreamerType::create_viewer(const async_web_server_cpp::HttpRequest &request)
{
  std::stringstream ss;
  ss << "<img src=\"/stream?";
  ss << request.query;
  ss << "\"></img>";
  return ss.str();
}

JpegSnapshotStreamer::JpegSnapshotStreamer(const async_web_server_cpp::HttpRequest &request,
                                           async_web_server_cpp::HttpConnectionPtr connection,
                                           ros::NodeHandle& nh) :
    ImageTransportImageStreamer(request, connection, nh)
{
  quality_ = request.get_query_param_value_or_default<int>("quality", 95);
  height_ = request.get_query_param_value_or_default<int>("height", -1);
  width_ = request.get_query_param_value_or_default<int>("width", -1);
}

JpegSnapshotStreamer::~JpegSnapshotStreamer()
{
  this->inactive_ = true;
  boost::mutex::scoped_lock lock(send_mutex_); // protects sendImage.
}

void JpegSnapshotStreamer::sendImage(const cv::Mat &img, const ros::Time &time)
{
  cv::Mat processed_img = img;
  // Check if height and width are valid
  if (height_ > 0 && width_ > 0) {
    // Resize the image to the specified values if they are positive
    cv::resize(img, processed_img, cv::Size(width_, height_), 0, 0, cv::INTER_NEAREST);
  }
  // If no resizing is needed (i.e., using native resolution), processed_img remains equal to img
  std::vector<int> encode_params;
#if CV_VERSION_MAJOR >= 3
  encode_params.push_back(cv::IMWRITE_JPEG_QUALITY);
#else
  encode_params.push_back(CV_IMWRITE_JPEG_QUALITY);
#endif
  encode_params.push_back(quality_);

  std::vector<uchar> encoded_buffer;
  cv::imencode(".jpeg", processed_img, encoded_buffer, encode_params);

  char stamp[20];
  sprintf(stamp, "%.06lf", time.toSec());
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
      .header("Connection", "close")
      .header("Server", "web_video_server")
      .header("Cache-Control",
              "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, "
              "max-age=0")
      .header("X-Timestamp", stamp)
      .header("Pragma", "no-cache")
      .header("Content-type", "image/jpeg")
      .header("Access-Control-Allow-Origin", "*")
      .header("Content-Length",
              boost::lexical_cast<std::string>(encoded_buffer.size()))
      .write(connection_);
  connection_->write_and_clear(encoded_buffer);
  inactive_ = true;
}

}
