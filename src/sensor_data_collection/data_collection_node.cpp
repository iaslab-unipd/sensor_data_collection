#include <ros/ros.h>
#include <ros/topic.h>

#include <boost/filesystem.hpp>
#include <time.h>
#include <yaml-cpp/yaml.h>

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

namespace fs = boost::filesystem;

fs::path computeFilePath(const fs::path & original, int n)
{
  std::string new_file_path = original.string();
  auto n_pos = new_file_path.rfind("<n>");
  if (n_pos != std::string::npos)
  {
    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << n;
    new_file_path.replace(n_pos, 3, ss.str());
  }

  return fs::path(new_file_path);
}

class BaseData
{
public:

  typedef boost::shared_ptr<BaseData> Ptr;

  BaseData(ros::NodeHandle & node_handle)
  {
    this->node_handle = node_handle;
  }

  virtual bool init() = 0;
  virtual void save(int n) = 0;
  virtual bool waitForMessage() = 0;
  virtual ros::Time timestamp() = 0;

  int save_flag = 0;
  std::string topic = "";
  fs::path file_path;

  friend std::ostream & operator << (std::ostream & stream, const BaseData & data);

protected:

  ros::NodeHandle node_handle;

};

std::ostream & operator << (std::ostream & stream, const BaseData & data)
{
  return stream << "{save_flag: " << data.save_flag << ", topic: " << data.topic << ", file_path: " << data.file_path << "}";
}

template <typename MessageT_>
  class BaseData_ : public BaseData
  {
  public:

    using BaseData::BaseData;

    virtual bool waitForMessage() override
    {
      return static_cast<bool>(ros::topic::waitForMessage<MessageT_>(topic, node_handle));
    }

    virtual ros::Time timestamp() override
    {
      return msg->header.stamp;
    }

  protected:

    boost::shared_ptr<const MessageT_> msg;

  };

class ImageData : public BaseData_<sensor_msgs::Image>
{
public:

  typedef boost::shared_ptr<ImageData> Ptr;

  ImageData(ros::NodeHandle & node_handle) : BaseData_(node_handle), image_transport(node_handle) {}

  virtual bool init() override
  {
    sub = image_transport.subscribe(topic, 1, &ImageData::callback, this);
  }

  virtual void save(int n) override
  {
    std::string new_file_path = computeFilePath(file_path, n).string();
    cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(msg);
    cv::imwrite(new_file_path, image_ptr->image);
  }

private:

  image_transport::ImageTransport image_transport;
  image_transport::Subscriber sub;

  void callback(const sensor_msgs::Image::ConstPtr & msg)
  {
    this->msg = msg;
  }

};

class DepthFloat32Data : public ImageData
{
public:

  typedef boost::shared_ptr<DepthFloat32Data> Ptr;

  using ImageData::ImageData;

  virtual void save(int n) override
  {
    std::string new_file_path = computeFilePath(file_path, n).string();
    cv_bridge::CvImage::Ptr depth_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat depth_image_16;
    depth_image_ptr->image.convertTo(depth_image_16, CV_16UC1, 1000);
    cv::imwrite(new_file_path, depth_image_16);
  }

};

class DepthUint16Data : public ImageData
{
public:

  typedef boost::shared_ptr<DepthUint16Data> Ptr;

  using ImageData::ImageData;

  virtual void save(int n) override
  {
    std::string new_file_path = computeFilePath(file_path, n).string();
    cv_bridge::CvImage::Ptr depth_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::imwrite(new_file_path, depth_image_ptr->image);
  }

};

class PointCloudData : public BaseData
{
public:

  typedef boost::shared_ptr<PointCloudData> Ptr;
  typedef BaseData Base;

  using BaseData::BaseData;

  virtual bool init() override
  {
    sub = node_handle.subscribe(topic, 1, &PointCloudData::callback, this);
  }

  virtual bool waitForMessage() override
  {
    return static_cast<bool>(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, node_handle));
  }

  virtual void save(int n) override
  {
    std::string new_file_path = computeFilePath(file_path, n).string();
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(new_file_path, *msg);
  }

  virtual ros::Time timestamp() override
  {
    return pcl_conversions::fromPCL(msg->header).stamp;
  }

private:

  ros::Subscriber sub;
  pcl::PCLPointCloud2::Ptr msg;

  void callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
  {
    this->msg = boost::make_shared<pcl::PCLPointCloud2>();
    pcl_conversions::toPCL(*msg, *this->msg);
  }

};

class CameraInfoData : public BaseData_<sensor_msgs::CameraInfo>
{
public:

  typedef boost::shared_ptr<CameraInfoData> Ptr;

  using BaseData_::BaseData_;

  virtual bool init() override
  {
    sub = node_handle.subscribe(topic, 1, &CameraInfoData::callback, this);
  }

  virtual void save(int n) override
  {
    std::string new_file_path = computeFilePath(file_path, n).string();
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(msg);

    std::ofstream file;
    file.open(new_file_path.c_str());

    file << "frame_id: " << msg->header.frame_id << std::endl;
    file << "height: " << msg->height << std::endl;
    file << "width: " << msg->width << std::endl;
    file << "distortion_model: " << msg->distortion_model << std::endl;
    file << "D: " << model.distortionCoeffs() << std::endl;
    file << "K: " << model.intrinsicMatrix().reshape<1, 9>() << std::endl;
    file << "R: " << model.rotationMatrix().reshape<1, 9>() << std::endl;
    file << "P: " << model.projectionMatrix().reshape<1, 12>() << std::endl;
    file << "binning_x: " << msg->binning_x << std::endl;
    file << "binning_y: " << msg->binning_y << std::endl;
    file << "roi:" << std::endl;
    file << "  x_offset: " << msg->roi.x_offset << std::endl;
    file << "  y_offset: " << msg->roi.y_offset << std::endl;
    file << "  height: " << msg->roi.height << std::endl;
    file << "  width: " << msg->roi.width << std::endl;
    file << "  do_rectify: " << (msg->roi.do_rectify ? "True" : "False") << std::endl;

    file.close();
  }

private:

  ros::Subscriber sub;

  void callback(const sensor_msgs::CameraInfo::ConstPtr & msg)
  {
    this->msg = msg;
  }

};

class DataCollectionNode
{
public:


  enum : int
  {
    IMAGE = 0,
    IR,
    DEPTH,
    POINT_CLOUD,

    IMAGE_CAMERA_INFO,
    IR_CAMERA_INFO,
    DEPTH_CAMERA_INFO,

    N,
    N_DATA = 4,
    N_CAMERA_INFO = 3,
    START_DATA = 0,
    START_CAMERA_INFO = 4

  };

  const std::vector<std::string> TOPIC{"image", "ir", "depth", "point_cloud", "image_camera_info", "ir_camera_info", "depth_camera_info"};
  const std::vector<std::string> EXTENSION{"png", "png", "png", "pcd", "yaml", "yaml", "yaml"};

  DataCollectionNode(ros::NodeHandle & node_handle);

  bool initialize();

private:

  void actionCallback(const Acquisition::ConstPtr & msg);

  ros::NodeHandle node_handle_;
  std::vector<BaseData::Ptr> data_;

  int save_flags_;

  ros::Subscriber action_sub_;

  bool continue_;
  int continue_from_;
  int file_index_;

  std::string device_name_;
  fs::path save_directory_;

};

DataCollectionNode::DataCollectionNode(ros::NodeHandle & node_handle) : node_handle_(node_handle)
{
  action_sub_ = node_handle.subscribe("action", 1, &DataCollectionNode::actionCallback, this);

  node_handle_.param("device_name", device_name_, std::string("camera"));

  std::string save_directory;
  if (not node_handle_.getParam("save_directory", save_directory))
    ROS_FATAL_STREAM("[" << device_name_ << "] Missing \"save_directory\" parameter!!");

  auto now_pos = save_directory.rfind("<now>");
  if (now_pos != std::string::npos)
  {
    time_t now;
    std::time(&now);
    tm * timeinfo = std::localtime(&now);
    char buffer[20];
    std::strftime(buffer, 20, "%F_%H-%M-%S", timeinfo);
    save_directory.replace(now_pos, 5, buffer);
  }

  save_directory_ = fs::path(save_directory) / fs::path(device_name_);

  node_handle_.param("continue", continue_, false);
  if (continue_)
  {
    if (not node_handle_.getParam("continue_from", continue_from_))
      ROS_FATAL_STREAM("[" << device_name_ << "] Missing \"continue_from\" parameter!!");
  }

  data_.resize(N);

  data_[IMAGE] = boost::make_shared<ImageData>(node_handle_);
  data_[IMAGE_CAMERA_INFO] = boost::make_shared<CameraInfoData>(node_handle_);
  data_[IR] = boost::make_shared<ImageData>(node_handle_);
  data_[IR_CAMERA_INFO] = boost::make_shared<CameraInfoData>(node_handle_);

  std::string depth_type_s;
  node_handle_.param("depth_type", depth_type_s, std::string("uint16"));
  if (depth_type_s == std::string("float32"))
    data_[DEPTH] = boost::make_shared<DepthFloat32Data>(node_handle_);
  else if (depth_type_s == std::string("uint16"))
    data_[DEPTH] = boost::make_shared<DepthUint16Data>(node_handle_);
  else
    ROS_FATAL_STREAM("[" << device_name_ << "] Wrong \"depth_type\" parameter. Use \"float32\" or \"uint16\".");

  data_[DEPTH_CAMERA_INFO] = boost::make_shared<CameraInfoData>(node_handle_);
  data_[POINT_CLOUD] = boost::make_shared<PointCloudData>(node_handle_);

  for (int i = 0; i < N; ++i)
  {
    BaseData::Ptr & data = data_[i];
    data->topic = TOPIC[i];
    data->save_flag = 1 << i;
  }

  for (int i = 0; i < N_DATA; ++i)
  {
    BaseData::Ptr & data = data_[START_DATA + i];
    BaseData::Ptr & camera_info = data_[START_CAMERA_INFO + i];

    std::string filename;
    node_handle_.param(data->topic + "_filename", filename, data->topic + "_");

    data->file_path = save_directory_ / (filename + "<n>." + EXTENSION[START_DATA + i]);
    if (i < N_CAMERA_INFO)
    {
      if (filename[filename.size() - 1] != '_')
        filename += '_';
      camera_info->file_path = save_directory_ / (filename + "camera_info." + EXTENSION[START_CAMERA_INFO + i]);
    }
  }

  save_flags_ = 0;
  for (int i = 0; i < N; ++i)
  {
    BaseData::Ptr & data = data_[i];
    bool flag;
    node_handle_.param("save_" + data->topic, flag, false);
    save_flags_ |= flag ? data->save_flag : 0;
    if (flag)
      data->init();
  }

}

bool DataCollectionNode::initialize()
{

  // 1. Wait for sensors to be connected
  ROS_INFO_STREAM("[" << device_name_ << "] Waiting for messages...");
  bool ret = true;

  for (int i = 0; ret and i < N; ++i)
  {
    BaseData::Ptr & data = data_[i];
    if (save_flags_ & data->save_flag)
    {
      ret = ret and data->waitForMessage();
      ROS_INFO_STREAM("[" << device_name_ << "] " << data->topic << " message OK!");
    }
  }

  if (not ret)
  {
    ROS_ERROR_STREAM("[" << device_name_ << "] Not all messages received!");
    return false;
  }

  // 2. Check directories
  if (not fs::exists(save_directory_))
  {
    if (fs::create_directories(save_directory_))
    {
      ROS_INFO_STREAM("[" << device_name_ << "] Directory " << save_directory_ << " created.");
    }
    else
    {
      ROS_ERROR_STREAM("[" << device_name_ << "] Error while creating directory " << save_directory_ << ". Aborting.");
      return false;
    }
  }
  else
  {
    if (fs::is_directory(save_directory_) and fs::is_empty(save_directory_))
    {
      ROS_INFO_STREAM("[" << device_name_ << "] Directory " << save_directory_ << " already exists and is empty.");
    }
    else if (fs::is_directory(save_directory_) and not fs::is_empty(save_directory_))
    {
      if (continue_)
      {
        ROS_INFO_STREAM("[" << device_name_ << "] Directory " << save_directory_ << " exists as expected. ");
      }
      else
      {
        ROS_ERROR_STREAM("[" << device_name_ << "] Directory " << save_directory_ << " exists but is not empty. Aborting.");
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("[" << device_name_ << "] " << save_directory_ << " is not a directory. Aborting.");
      return false;
    }
  }

  // 3. Check last saved file indices (if in continue mode)
  fs::path info_yaml = save_directory_ / "info.yaml";
  if (continue_ and continue_from_ == 0 and fs::exists(info_yaml) and not info_yaml.empty())
  {
    ROS_INFO_STREAM("[" << device_name_ << "] Reading \"" << info_yaml.string() << "\"...");
    YAML::Node info_node = YAML::LoadFile(info_yaml.string());

    const YAML::Node & data_node = info_node["data"];
    int max = 0;
    for (const YAML::Node & node : data_node)
    {
      max = std::max(node["index"].as<int>(), max);
    }
    file_index_ = max + 1;
    ROS_INFO_STREAM("[" << device_name_ << "] First index is " << file_index_ << ".");
  }
  else
  {
    file_index_ = 1;
  }

  return true;
}

void DataCollectionNode::actionCallback(const Acquisition::ConstPtr & msg)
{

  try
  {
    fs::path info_file_name = save_directory_ / "info.yaml";

    std::ofstream info_file;
    info_file.open(info_file_name.string().c_str(), file_index_ == 1 ? std::ios_base::out : std::ios_base::out | std::ios_base::app);

    if (file_index_ == 1)
    {
      info_file << "camera_info:" << std::endl;
      for (int i = START_CAMERA_INFO; i < START_CAMERA_INFO + N_CAMERA_INFO; ++i)
      {
        BaseData::Ptr & data = data_[i];
        if (save_flags_ & data->save_flag)
        {
          data->save(file_index_);
          info_file << "  - {type: \"" << data->topic << "\", "
                    << std::setprecision(19) << "timestamp: " << data->timestamp() << ", "
                    << std::setprecision(6) << "filename: " << computeFilePath(data->file_path.filename(), file_index_) << "}" << std::endl;
        }
      }
      info_file << "data:" << std::endl;
    }

    info_file << "  - index: " << file_index_ << std::endl;
    if (msg->distance > 0)
      info_file << "    distance: " << msg->distance << std::endl;
    if (not msg->info.empty())
      info_file << "    info: \"" << msg->info << "\"" << std::endl;
    info_file << "    acquired:" << std::endl;

    for (int i = START_DATA; i < START_DATA + N_DATA; ++i)
    {
      BaseData::Ptr & data = data_[i];
      if (save_flags_ & data->save_flag)
      {
        data->save(file_index_);
        info_file << "      - {type: \"" << data->topic << "\", "
                  << std::setprecision(19) << "timestamp: " << data->timestamp() << ", "
                  << std::setprecision(6) << "filename: " << computeFilePath(data->file_path.filename(), file_index_) << "}" << std::endl;
      }
    }

    info_file.close();

    ROS_INFO_STREAM("[" << device_name_ << "] " << file_index_ << " saved");
    file_index_++;

  }
  catch (cv_bridge::Exception & ex)
  {
    ROS_ERROR_STREAM("[" << device_name_ << "] cv_bridge exception: " << ex.what());
  }

}

} // namespace unipd

int main(int argc,
         char ** argv)
{
  ros::init(argc, argv, "sensor_data_collection_node");
  ros::NodeHandle node_handle("~");

  try
  {
    unipd::DataCollectionNode collector_node(node_handle);
    if (not collector_node.initialize())
      return 0;
    ros::spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL_STREAM("Error: " << error.what());
    return 1;
  }

  return 0;
}
