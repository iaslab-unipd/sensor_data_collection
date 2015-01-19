#include <ros/ros.h>
#include <ros/topic.h>

#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_data_collection/Acquisition.h>

using sensor_data_collection::Acquisition;

namespace unipd
{

class DataCollectionNode
{
public:

  enum SaveFlags : int
  {
    SAVE_NONE = 0,
    SAVE_IMAGE = 1,
    SAVE_IR = 2,
    SAVE_DEPTH = 4,
    SAVE_POINT_CLOUD = 8,
    SAVE_IMAGE_CAMERA_INFO = 16,
    SAVE_IR_CAMERA_INFO = 32,
    SAVE_DEPTH_CAMERA_INFO = 64
  };

  enum class DepthType
  {
    DEPTH_FLOAT32,
    DEPTH_UINT16
  };

  DataCollectionNode(ros::NodeHandle & node_handle);

  bool
  initialize();

protected:

  void
  save(const sensor_msgs::CameraInfo::ConstPtr & camera_info,
       const std::string & file_name);

  void
  save(const pcl::PCLPointCloud2::ConstPtr & cloud,
       const std::string & file_name);

  void
  save(const cv::Mat & image,
       const std::string & file_name);

  void
  saveDepth(const cv::Mat & depth_image,
            const std::string & file_name);

  void
  pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg);

  void
  imageCallback(const sensor_msgs::Image::ConstPtr & msg);

  void
  irCallback(const sensor_msgs::Image::ConstPtr & msg);

  void
  depthCallback(const sensor_msgs::Image::ConstPtr & msg);

  void
  imageCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg);

  void
  irCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg);

  void
  depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg);

  void
  actionCallback(const Acquisition::ConstPtr & msg);

  bool
  waitForMessages();

  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;

  ros::Subscriber cloud_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber ir_sub_;
  image_transport::Subscriber depth_image_sub_;
  ros::Subscriber image_camera_info_sub_;
  ros::Subscriber ir_camera_info_sub_;
  ros::Subscriber depth_camera_info_sub_;

  ros::Subscriber omnicamera_info_sub_;
  ros::Subscriber action_sub_;

  pcl::PCLPointCloud2::Ptr cloud_msg_;
  sensor_msgs::Image::ConstPtr image_msg_;
  sensor_msgs::Image::ConstPtr ir_msg_;
  sensor_msgs::Image::ConstPtr depth_msg_;
  sensor_msgs::CameraInfo::ConstPtr image_camera_info_msg_;
  sensor_msgs::CameraInfo::ConstPtr ir_camera_info_msg_;
  sensor_msgs::CameraInfo::ConstPtr depth_camera_info_msg_;

  int start_index_;
  int file_index_;

  std::string device_name_;
  std::string save_folder_;

  std::string image_extension_;
  std::string image_filename_;
  std::string ir_filename_;
  std::string depth_filename_;
  std::string cloud_filename_;

  int save_flags_;
  DepthType depth_type_;

};

DataCollectionNode::DataCollectionNode(ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    image_transport_(node_handle)
{

  action_sub_ = node_handle.subscribe("action", 1, &DataCollectionNode::actionCallback, this);

  node_handle_.param("device_name", device_name_, std::string("camera"));

  if (not node_handle_.getParam("save_folder", save_folder_))
    ROS_FATAL_STREAM("[" << device_name_ << "] Missing folder!!");

  if (save_folder_.at(save_folder_.size() - 1) != '/')
    save_folder_ += "/";
  save_folder_ += device_name_ + "/";

  node_handle_.param("start_index", start_index_, 1);
  file_index_ = start_index_;

  node_handle_.param("image_extension", image_extension_, std::string("png"));

  node_handle_.param("image_filename", image_filename_, std::string("image_"));
  node_handle_.param("ir_filename",    ir_filename_,    std::string("ir_"));
  node_handle_.param("depth_filename", depth_filename_, std::string("depth_"));
  node_handle_.param("cloud_filename", cloud_filename_, std::string("cloud_"));

  bool save_image, save_image_camera_info;
  node_handle_.param("save_image", save_image, false);
  node_handle_.param("save_image_camera_info", save_image_camera_info, false);

  bool save_ir, save_ir_camera_info;
  node_handle_.param("save_ir", save_ir, false);
  node_handle_.param("save_ir_camera_info", save_ir_camera_info, false);

  bool save_depth, save_depth_camera_info;
  node_handle_.param("save_depth_image", save_depth, false);
  node_handle_.param("save_depth_camera_info", save_depth_camera_info, false);

  bool save_point_cloud;
  node_handle_.param("save_point_cloud", save_point_cloud, false);

  save_flags_ = SAVE_NONE;
  save_flags_ |= save_image ? SAVE_IMAGE : SAVE_NONE;
  save_flags_ |= save_image_camera_info ? SAVE_IMAGE_CAMERA_INFO : SAVE_NONE;
  save_flags_ |= save_ir ? SAVE_IR : SAVE_NONE;
  save_flags_ |= save_ir_camera_info ? SAVE_IR_CAMERA_INFO : SAVE_NONE;
  save_flags_ |= save_depth ? SAVE_DEPTH : SAVE_NONE;
  save_flags_ |= save_depth_camera_info ? SAVE_DEPTH_CAMERA_INFO : SAVE_NONE;
  save_flags_ |= save_point_cloud ? SAVE_POINT_CLOUD : SAVE_NONE;

  if (save_flags_ & SAVE_POINT_CLOUD)
    cloud_sub_ = node_handle.subscribe("point_cloud", 1, &DataCollectionNode::pointCloudCallback, this);

  if (save_flags_ & SAVE_IMAGE)
    image_sub_ = image_transport_.subscribe("image", 1, &DataCollectionNode::imageCallback, this);

  if (save_flags_ & SAVE_IR)
    ir_sub_ = image_transport_.subscribe("ir", 1, &DataCollectionNode::irCallback, this);

  if (save_flags_ & SAVE_DEPTH)
    depth_image_sub_ = image_transport_.subscribe("depth", 1, &DataCollectionNode::depthCallback, this);

  if (save_flags_ & SAVE_IMAGE_CAMERA_INFO)
    image_camera_info_sub_ = node_handle.subscribe("image_camera_info", 1, &DataCollectionNode::imageCameraInfoCallback, this);

  if (save_flags_ & SAVE_IR_CAMERA_INFO)
    ir_camera_info_sub_ = node_handle.subscribe("ir_camera_info", 1, &DataCollectionNode::irCameraInfoCallback, this);

  if (save_flags_ & SAVE_DEPTH_CAMERA_INFO)
    depth_camera_info_sub_ = node_handle.subscribe("depth_camera_info", 1, &DataCollectionNode::depthCameraInfoCallback, this);

  std::string depth_type_s;
  node_handle_.param("depth_type", depth_type_s, std::string("float32"));
  if (depth_type_s == std::string("float32"))
    depth_type_ = DepthType::DEPTH_FLOAT32;
  else if (depth_type_s == std::string("uint16"))
    depth_type_ = DepthType::DEPTH_UINT16;
  else
    ROS_FATAL_STREAM("[" << device_name_ << "] Wrong \"depth_type\" parameter. Use \"float32\" or \"uint16\".");

}

bool DataCollectionNode::initialize()
{
  if (not waitForMessages())
    return false;

  ros::spinOnce();

  return true;
}

void DataCollectionNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  cloud_msg_ = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(*msg, *cloud_msg_);
}

void DataCollectionNode::imageCallback(const sensor_msgs::Image::ConstPtr & msg)
{
  image_msg_ = msg;
}

void DataCollectionNode::irCallback(const sensor_msgs::Image::ConstPtr & msg)
{
  ir_msg_ = msg;
}

void DataCollectionNode::depthCallback(const sensor_msgs::Image::ConstPtr & msg)
{
  depth_msg_ = msg;
}

void DataCollectionNode::imageCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg)
{
  image_camera_info_msg_ = msg;
}

void DataCollectionNode::irCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg)
{
  ir_camera_info_msg_ = msg;
}

void DataCollectionNode::depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg)
{
  depth_camera_info_msg_ = msg;
}

void DataCollectionNode::actionCallback(const Acquisition::ConstPtr & msg)
{

  try
  {
    std::stringstream info_file_name;
    info_file_name << save_folder_ << "info.yaml";
    std::stringstream file_index_ss;
    file_index_ss << std::setw(4) << std::setfill('0') << file_index_;

    std::ofstream info_file;
    info_file.open(info_file_name.str().c_str(), file_index_ == 1 ? std::ios_base::out : std::ios_base::out | std::ios_base::app);

    if (file_index_ == 1)
    {
      info_file << "camera_info:" << std::endl;
      if (save_flags_ & SAVE_IMAGE_CAMERA_INFO)
      {
        std::stringstream camera_info_file_name;
        camera_info_file_name << save_folder_ << image_filename_ << "camera_info.yaml";
        save(image_camera_info_msg_, camera_info_file_name.str());
        info_file << "  - {type: \"image_camera_info\", "
                  << std::setprecision(19) << "timestamp: " << image_camera_info_msg_->header.stamp << ", "
                  << std::setprecision(6) << "filename: \"" << image_filename_ << "camera_info.yaml\"}" << std::endl;
      }

      if (save_flags_ & SAVE_IR_CAMERA_INFO)
      {
        std::stringstream camera_info_file_name;
        camera_info_file_name << save_folder_ << ir_filename_ << "camera_info.yaml";
        save(ir_camera_info_msg_, camera_info_file_name.str());
        info_file << "  - {type: \"ir_camera_info\", "
                  << std::setprecision(19) << "timestamp: " << ir_camera_info_msg_->header.stamp << ", "
                  << std::setprecision(6) << "filename: \"" << ir_filename_ << "camera_info.yaml\"}" << std::endl;
      }

      if (save_flags_ & SAVE_DEPTH_CAMERA_INFO)
      {
        std::stringstream camera_info_file_name;
        camera_info_file_name << save_folder_ << depth_filename_ << "camera_info.yaml";
        save(depth_camera_info_msg_, camera_info_file_name.str());
        info_file << "  - {type: \"depth_camera_info\", "
                  << std::setprecision(19) << "timestamp: " << depth_camera_info_msg_->header.stamp << ", "
                  << std::setprecision(6) << "filename: \"" << depth_filename_ << "camera_info.yaml\"}" << std::endl;
      }

      info_file << "data:" << std::endl;
    }

    info_file << "  - id: " << file_index_ss.str() << std::endl;
    if (msg->distance > 0)
      info_file << "    distance: " << msg->distance << std::endl;
    if (not msg->info.empty())
      info_file << "    info: \"" << msg->info << "\"" << std::endl;
    info_file << "    acquired:" << std::endl;

    if (save_flags_ & SAVE_IMAGE)
    {
      cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(image_msg_);
      std::stringstream image_file_name;
      image_file_name << save_folder_ << image_filename_ << file_index_ss.str() << "." << image_extension_;
      save(image_ptr->image, image_file_name.str());
      info_file << "      - {type: \"image\", "
                << std::setprecision(19) << "timestamp: " << image_msg_->header.stamp << ", "
                << std::setprecision(6) << "filename: \"" << image_filename_ << file_index_ss.str() << "." << image_extension_ << "\"}" << std::endl;
    }

    if (save_flags_ & SAVE_IR)
    {
      cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(ir_msg_);
      std::stringstream image_file_name;
      image_file_name << save_folder_ << ir_filename_ << file_index_ss.str() << "." << image_extension_;
      save(image_ptr->image, image_file_name.str());
      info_file << "      - {type: \"ir\", "
                << std::setprecision(19) << "timestamp: " << ir_msg_->header.stamp << ", "
                << std::setprecision(6) << "filename: \"" << ir_filename_ << file_index_ss.str() << "." << image_extension_ << "\"}" << std::endl;
    }

    if (save_flags_ & SAVE_DEPTH)
    {
      cv_bridge::CvImage::Ptr depth_image_ptr;
      if (depth_type_ == DepthType::DEPTH_FLOAT32)
        depth_image_ptr = cv_bridge::toCvCopy(depth_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
      else if (depth_type_ == DepthType::DEPTH_UINT16)
        depth_image_ptr = cv_bridge::toCvCopy(depth_msg_, sensor_msgs::image_encodings::TYPE_16UC1);

      std::stringstream image_file_name;
      image_file_name << save_folder_ << depth_filename_ << file_index_ss.str() << "." << image_extension_;
      saveDepth(depth_image_ptr->image, image_file_name.str());
      info_file << "      - {type: \"depth\", "
                << std::setprecision(19) << "timestamp: " << depth_msg_->header.stamp << ", "
                << std::setprecision(6) << "filename: \"" << depth_filename_ << file_index_ss.str() << "." << image_extension_ << "\"}" << std::endl;
    }

    if (save_flags_ & SAVE_POINT_CLOUD)
    {
      std::stringstream cloud_file_name;
      cloud_file_name << save_folder_ << cloud_filename_ << file_index_ss.str() << ".pcd";
      save(cloud_msg_, cloud_file_name.str());
      info_file << "      - {type: \"point_cloud\", "
                << std::setprecision(19) << "timestamp: " << pcl_conversions::fromPCL(cloud_msg_->header).stamp << ", "
                << std::setprecision(6) << "filename: \"" << cloud_filename_ << file_index_ss.str() << ".pcd\"}" << std::endl;
    }

    info_file.close();

    ROS_INFO_STREAM("[" << device_name_ << "] " << file_index_ss.str() << " saved");
    file_index_++;

  }
  catch (cv_bridge::Exception & ex)
  {
    ROS_ERROR_STREAM("[" << device_name_ << "] cv_bridge exception: " << ex.what());
  }


}

void DataCollectionNode::save(const sensor_msgs::CameraInfo::ConstPtr & camera_info,
                              const std::string & file_name)
{
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info);

  std::ofstream file;
  file.open(file_name.c_str());

  file << "frame_id: " << image_camera_info_msg_->header.frame_id << std::endl;
  file << "height: " << image_camera_info_msg_->height << std::endl;
  file << "width: " << image_camera_info_msg_->width << std::endl;
  file << "distortion_model: " << image_camera_info_msg_->distortion_model << std::endl;
  file << "D: " << model.distortionCoeffs() << std::endl;
  file << "K: " << model.intrinsicMatrix().reshape<1, 9>() << std::endl;
  file << "R: " << model.rotationMatrix().reshape<1, 9>() << std::endl;
  file << "P: " << model.projectionMatrix().reshape<1, 12>() << std::endl;
  file << "binning_x: " << image_camera_info_msg_->binning_x << std::endl;
  file << "binning_y: " << image_camera_info_msg_->binning_y << std::endl;
  file << "roi:" << std::endl;
  file << "  x_offset: " << image_camera_info_msg_->roi.x_offset << std::endl;
  file << "  y_offset: " << image_camera_info_msg_->roi.y_offset << std::endl;
  file << "  height: " << image_camera_info_msg_->roi.height << std::endl;
  file << "  width: " << image_camera_info_msg_->roi.width << std::endl;
  file << "  do_rectify: " << (image_camera_info_msg_->roi.do_rectify ? "True" : "False") << std::endl;

  file.close();
}

bool DataCollectionNode::waitForMessages()
{
  ROS_INFO_STREAM("[" << device_name_ << "] Waiting for messages...");
  bool ret = true;

  if (save_flags_ & SAVE_IMAGE)
  {
    ret = ret and ros::topic::waitForMessage<sensor_msgs::Image>("image", node_handle_);
    ROS_INFO_STREAM("[" << device_name_ << "] image message OK!");
  }

  if (save_flags_ & SAVE_IMAGE_CAMERA_INFO)
  {
    ret = ret and ros::topic::waitForMessage<sensor_msgs::CameraInfo>("image_camera_info", node_handle_);
    ROS_INFO_STREAM("[" << device_name_ << "] image camera info message OK!");
  }

  if (save_flags_ & SAVE_IR)
  {
    ret = ret and ros::topic::waitForMessage<sensor_msgs::Image>("ir", node_handle_);
    ROS_INFO_STREAM("[" << device_name_ << "] ir message OK!");
  }

  if (save_flags_ & SAVE_IR_CAMERA_INFO)
  {
    ret = ret and ros::topic::waitForMessage<sensor_msgs::CameraInfo>("ir_camera_info", node_handle_);
    ROS_INFO_STREAM("[" << device_name_ << "] ir camera info message OK!");
  }

  if (save_flags_ & SAVE_DEPTH)
  {
    ret = ret and ros::topic::waitForMessage<sensor_msgs::Image>("depth", node_handle_);
    ROS_INFO_STREAM("[" << device_name_ << "] depth message OK!");
  }

  if (save_flags_ & SAVE_DEPTH_CAMERA_INFO)
  {
    ret = ret and ros::topic::waitForMessage<sensor_msgs::CameraInfo>("depth_camera_info", node_handle_);
    ROS_INFO_STREAM("[" << device_name_ << "] depth camera info message OK!");
  }

  if (save_flags_ & SAVE_POINT_CLOUD)
  {
    ret = ret and ros::topic::waitForMessage<sensor_msgs::PointCloud2>("point_cloud", node_handle_);
    ROS_INFO_STREAM("[" << device_name_ << "] point cloud message OK!");
  }

  if (not ret)
    ROS_ERROR_STREAM("[" << device_name_ << "] not all messages received!");

  return ret;
}

void DataCollectionNode::save(const pcl::PCLPointCloud2::ConstPtr & cloud,
                              const std::string & file_name)
{
  pcl::PCDWriter pcd_writer;
  pcd_writer.writeBinary(file_name, *cloud);
}

void DataCollectionNode::save(const cv::Mat & image,
                              const std::string & file_name)
{
  cv::imwrite(file_name, image);
}

void DataCollectionNode::saveDepth(const cv::Mat & depth_image,
                                   const std::string & file_name)
{
  if (depth_type_ == DepthType::DEPTH_FLOAT32)
  {
    cv::Mat depth_image_16;
    depth_image.convertTo(depth_image_16, CV_16UC1, 1000);
    cv::imwrite(file_name, depth_image_16);
  }
  else if (depth_type_ == DepthType::DEPTH_UINT16)
  {
    cv::imwrite(file_name, depth_image);
  }
  else
  {
    // Do nothing
  }
}

} // namespace unipd

using namespace unipd;

int main(int argc,
         char ** argv)
{
  ros::init(argc, argv, "sensor_data_collection_node");
  ros::NodeHandle node_handle("~");

  try
  {
    DataCollectionNode collector_node(node_handle);
    if (not collector_node.initialize())
      return 0;
    ros::spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Calibration error: %s", error.what());
    return 1;
  }

  return 0;
}
