# Sensor Data Collection
The package provides a ROS node for collecting data from one or more cameras/depth sensors.

## ROS Node

### Parameters

* `device_name` [**type** `string`, **default** `"camera"`]: the name of the device from which the data is acquired. It is the name of the directory where data are saved. 
* `save_directory` [**type** `string`, **mandatory**, **no default**]: the root directory where all the data will be saved. Actually data are saved in `save_directory/device_name/`.
* `continue` [**type** `bool`, **default** `false`]: flag that set whether to add data to a previous acquisition.
* `continue_from` [**type** `int`, **default** `0`]: if `continue` is `true`, `continue_from` is the first index that data will be saved with. As a special case, if `continue_from` is set to `0`, the index is auto-computed.
* `save_image` [**type** `bool`, **default** `false`]: flag that set whether to save the RGB image provided by the sensor or not.
* `save_image_camera_info`, [**type** `bool`, **default** `false`]: flag that set whether to save the camera info data (relative to the RGB image) provided by the sensor or not.
* `save_ir` [**type** `bool`, **default** `false`]: flag that set whether to save the IR image provided by the sensor or not.
* `save_ir_camera_info` [**type** `bool`, **default** `false`]: flag that set whether to save the camera info data (relative to the IR image) provided by the sensor or not.
* `save_depth_image` [**type** `bool`, **default** `false`]: flag that set whether to save the depth image provided by the sensor or not.
* `save_depth_camera_info` [**type** `bool`, **default** `false`]: flag that set whether to save the camera info data (relative to the depth image) provided by the sensor or not.
* `save_point_cloud` [**type** `bool`, **default** `false`]: flag that set whether to save the point cloud provided by the sensor or not.
* `depth_type` [**type** `string`, **default** `"uint16"`]: the encoding format of the depth data. Two formats are supported:
  * `"uint16"`: depth format of data provided by a Kinect.
  * `"float32"`: depth format of data provided by a SwissRanger.

### Subscribed Topics

* `~action` [**type**: `sensor_data_collection/Acquisition`]: used by the node to trigger the acquisition. I.e. data are acquired as soon as a new message is published on this topic.
* `~image` [**type**: `sensor_msgs/Image`]: topic where the RGB images are published.
* `~image_camera_info` [**type**: `sensor_msgs/CameraInfo`]: topic where the camera info data (relative to the RGB images) are published.
* `~ir` [**type**: `sensor_msgs/Image`]: topic where the IR images are published.
* `~image_camera_info` [**type**: `sensor_msgs/CameraInfo`]: topic where the camera info data (relative to the IR images) are published.
* `~depth` [**type**: `sensor_msgs/Image`]: topic where the depth images are published.
* `~image_camera_info` [**type**: `sensor_msgs/CameraInfo`]: topic where the camera info data (relative to the depth images) are published.
* `~point_cloud` [**type**: `sensor_msgs/PointCloud2`]: topic where the point clouds are published.

## How-to

### Create a Launch File

Create a file named `my_device_data_collection_node.launch` in `sensor_data_collection/launch`.
Open it an fill it as following.
Initiate the launcher.
```
<?xml version="1.0"?>
<launch>
```
Ask the user to provide the directory each time the node is launched.
```
  <arg name="save_directory" />
```
Let the user the opportunity to continue a previously interrupted acquisition.
```
  <arg name="continue"               default="false" />
  <arg name="continue_from"          default="0" />
```
Define the topics to save.
```
  <arg name="save_image"             default="true" />
  <arg name="save_image_camera_info" default="true" />
  <arg name="save_ir"                default="false" />
  <arg name="save_ir_camera_info"    default="false" />
  <arg name="save_depth_image"       default="true" />
  <arg name="save_depth_camera_info" default="true" />
  <arg name="save_point_cloud"       default="true" />
```
Define the name of the device.
```
  <arg name="device_name"            default="my_device" />
```

Create the node. Use the `device_name` argument to let multiple instances of the acquisition node to run at the same time.
```
  <node pkg="sensor_data_collection" type="data_collection_node" name="$(arg device_name)_data_collection"
        output="screen" required="true">
```
Use the previously defined arguments as parameters.
```
    <!-- Parameters -->
    <param name="device_name"             value="$(arg device_name)" />
    <param name="save_directory"          value="$(arg save_directory)" />
    
    <param name="continue"                value="$(arg continue)" />
    <param name="continue_from"           value="$(arg continue_from)" />
    
    <param name="save_image"              value="$(arg save_image)" />
    <param name="save_image_camera_info"  value="$(arg save_image_camera_info)" />
    <param name="save_ir"                 value="$(arg save_ir)" />
    <param name="save_ir_camera_info"     value="$(arg save_ir_camera_info)" />
    <param name="save_depth_image"        value="$(arg save_depth_image)" />
    <param name="save_depth_camera_info"  value="$(arg save_depth_camera_info)" />
    <param name="save_point_cloud"        value="$(arg save_point_cloud)" />
```
Set the depth type.
```
    <param name="depth_type"              value="uint16" />
```
Remap the topics to the correct ones. We use the `if="$(arg save_*)"` to avoid subscription to unused topics. 
```
    <remap from="~image"             to="/real_device/rgb/image_color"   if="$(arg save_image)" />
    <remap from="~image_camera_info" to="/real_device/rgb/camera_info"   if="$(arg save_image_camera_info)" />
    <remap from="~ir"                to="/real_device/ir/image_color"    if="$(arg save_ir)" />
    <remap from="~ir_camera_info"    to="/real_device/ir/camera_info"    if="$(arg save_ir_camera_info)" />
    <remap from="~depth"             to="/real_device/depth/image"       if="$(arg save_depth)" />
    <remap from="~depth_camera_info" to="/real_device/depth/camera_info" if="$(arg save_depth_camera_info)" />
    <remap from="~point_cloud"       to="/real_device/depth/points"      if="$(arg save_point_cloud)" />
```
Close the node and the launcher.
```
  </node>
</launch>
```

### Run the Node

You can start the acuisition by running:
```
roslaunch sensor_data_collection my_device_data_collection_node.launch save_directory:=$HOME/.ros/sensor_data_collection/red_ball
```
All the directories are automatically created if not exisiting, otherwise they are checked if they are not empty.

You can continue a previously interrupted acquisition by running the same launcher:
```
roslaunch sensor_data_collection my_device_data_collection_node.launch save_directory:=$HOME/.ros/sensor_data_collection/red_ball continue:=true
```
The program will tell you the first available index: `[my_device] First index is 3.`

### Trigger Acquisition

The acquisition is triggered by publishing an `sensor_data_collection/Acquisition` message on the defined topic:
```
rostopic pub /action sensor_data_collection/Acquisition "{}" -1
```
The `Acquisition` message has two fields:
* `distance`: distance in meters of the acquired object. *[type: `float`]*
* `info`: additional infos. *[type: `string`]** 

So, data can be acquired as following:
```
rostopic pub /action sensor_data_collection/Acquisition "{distance: 1.5, info: 'Red ball'}" -1
```

## Get a Look at the Created Files 

The node creates the following files:
* `info.yaml`: contains information about all the acquisitions performed.
* `[image|ir|depth]_camera_info.yaml`: contains the calibration parameters of the respective sensors.
* `image_<n,4>.png`: an image for each acquisition with padding zeros in the filename (e.g. `n=5` -> `image_0005.png`). The image is compressed with a lossless encoding (png).
* `ir_<n,4>.png`: an image for each acquisition. The image is compressed with a lossless encoding (png).
* `depth_<n,4>.png`: an image for each acquisition. The image is compressed with a **16bit** lossless encoding (png) .
* `point_cloud_<n,4>.pcd`: an point cloud for each acquisition.
