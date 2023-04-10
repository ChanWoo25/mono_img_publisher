#include <string>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <CwCapture.h>

class MonoDevice
{
public:
  MonoDevice (int format_idx, 
              uint32_t resized_w=0U, 
              uint32_t resized_h=0U)
    : w_(0U), h_(0U),
      resized_w_(resized_w),
      resized_h_(resized_h)
  {
    cam_ = (CaptureDevice *) std::malloc(sizeof(CaptureDevice));

    if (cam_ == NULL) { exit(-1); }

    if (initialize(cam_, 0) < 0) { exit(-1); }

    if (set_option(cam_, format_idx) < 0) { exit(-1); }

    if (alloc_buffer(cam_) < 0) { exit(-1); }
    
    ROS_INFO("width %d, height %d, pixel_format: %d, f_idx: %d, img_bytes: %d", 
             cam_->width ,cam_->height ,cam_->pixel_format ,cam_->f_idx ,cam_->img_bytes);
    w_ = cam_->width;
    h_ = cam_->height;
    if (resized_w_ == 0U || resized_h_ == 0U) {
      resized_w_ = w_;
      resized_h_ = h_;
    }

    img_ = (uint8_t *) std::malloc(sizeof(int8_t) * w_ * h_ * 3);

    capture_stream_on();
    if (stream_state == StreamState::START)
      ROS_INFO("# Mono Camera: Streaming start!");
    else {
      ROS_ERROR("# Mono Camera: Streaming failed to start!");
      exit(-1);
    }
  }

  ~MonoDevice() 
  {
    capture_stream_off();
    if (stream_state != StreamState::STOP) {
      ROS_ERROR("# Mono Camera: Streaming failed to stop!");
      exit(-1);
    }
    else {
      ROS_INFO("# Mono Camera: Streaming off.");
    }
    for (unsigned i = 0; i < 2; i++) {
      free_buffer(cam_);
      close(cam_->fd);
      free(cam_);
      free(img_);
    }
  }

  void capture_stream_on() 
  {
    if (stream_on(cam_) < 0) 
    {
      stream_state = StreamState::PUASE;
    }
    else
    {
      stream_state =StreamState::START;
    }
  }

  void capture_stream_off() 
  {
    if (stream_off(cam_) < 0) 
    {
      stream_state = StreamState::PUASE;
    }
    else
    {
      stream_state =StreamState::STOP;
    }
  }

  int single_capture() 
  {
    capture(cam_);
    usec_ = static_cast<int64_t>(cam_->data.usec);
    return 0;
  }

  cv::Mat get_img() {
    convert(cam_, img_);
    return cv::Mat(h_, w_, CV_8UC3, img_);
  }

  int64_t usec() {
    return usec_;
  }

  enum class StreamState{
    PUASE,
    START,
    STOP
  } stream_state;

private:
  CaptureDevice * cam_;
  uint8_t * img_;
  int64_t usec_;
  uint32_t  w_;
  uint32_t  h_;
  uint32_t  resized_w_;
  uint32_t  resized_h_;
};

constexpr double img_saving_interval = 0.5; // [sec]

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "mono_img_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  image_transport::Publisher pub0 = it.advertise("/cam0/image_raw", 1);

  constexpr int FORMAT_OPTION = 0;
  MonoDevice mono_camera(FORMAT_OPTION);

  using std::chrono::system_clock;
  auto start = system_clock::now();
  auto prev = start;
  auto curr = start;
  static bool is_first = true;

  constexpr int FPS = 20;
  int compensated_fps = int(FPS * 1.1);
  ros::Rate rate(compensated_fps * 20);
  double interval = 1.0 / (double)compensated_fps;
  unsigned int count = 0;

  while (ros::ok())
  {
    if (mono_camera.single_capture() < 0)
      break;
      
    curr = system_clock::now();
    auto duration = std::chrono::duration<double>(curr - prev);
    if (is_first || duration.count() > interval)
    {
      is_first = false;
      prev = curr;
      count++;

      std_msgs::Header header;
      double timestamp = is_first ? 0.0 : std::chrono::duration<double>(curr - start).count();
      header.stamp.fromSec(timestamp);
      header.seq = count;
      auto img0 = mono_camera.get_img();
      sensor_msgs::ImagePtr msg0 = cv_bridge::CvImage(header, "bgr8", img0).toImageMsg();
      ROS_INFO("[Tiemstamp: %f] Publish(cnt-%04d): interval:%2.4f", timestamp, count, duration.count());
      pub0.publish(msg0);

      if (count % 20U == 0)
      {
        static int img0_idx = 0;
        std::string img0_filename = std::to_string(img0_idx) + ".png";
        std::string img0_filepath = "/home/nvidia/Reps/catkin_ws/mono_images/" + img0_filename;
        ROS_INFO("image %s is saved!", img0_filename.c_str());
        cv::imwrite(img0_filepath, img0);
        img0_idx++;
      }
    }
    rate.sleep();
  }
  
  return EXIT_SUCCESS;
}