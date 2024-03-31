# RS to Velodyne
The package has been adapted to run on ROS2 Humble, based on the work of [HViktorTsoi](https://github.com/HViktorTsoi). This is A ROS2 tool for converting Robosense pointcloud to Velodyne pointcloud format, which can be directly used for downstream algorithm, such as LOAM, LEGO-LOAM, LIO-SAM, etc.
## Currently support:


### 1. [robosense XYZIRT] to [velodyne XYZIRT / XYZIR / XYZI]: (Recommended)
RS-16, RS-32, RS-Ruby, RS-BP and RS-Helios LiDAR point cloud.

### 2. [robosense XYZI] to [velodyne XYZIR]:
RS-16 and RS-Ruby LiDAR point cloud, More LiDAR model support is coming soon. 

 
## Usage

### 1. Config File
One of the changes made to this package is the addition of the configuration file. To correctly configure this package, the following parameters should be set in the `rs_to_velodyne/config/param.yaml` file:

    | Parameter     | Description                                       | Type    |
    |---------------|---------------------------------------------------|---------|
    | `output_type` | Type of output point cloud (e.g., XYZIRT )        | String  |
    | `input_type`  | Type of input point cloud (e.g., XYZI)            | String  |
    | `input_topic` | ROS topic for input point cloud data              | String  |
    | `output_topic`| ROS topic for output converted point cloud data   | String  |

