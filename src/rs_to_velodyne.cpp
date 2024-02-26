#include <utilities.hpp>



template<typename T>
bool has_nan(T point) {

    // remove nan point, or the feature assocaion will crash, the surf point will containing nan points
    // pcl remove nan not work normally
    // ROS_ERROR("Containing nan point!");
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
        return true;
    }
    return false;
}

template<typename T>
void publish_points(T &new_pc, const sensor_msgs::msg::PointCloud2::SharedPtr old_msg) {
    // pc properties
    new_pc->is_dense = true;

    // publish
    sensor_msgs::msg::PointCloud2 pc_new_msg;
    pcl::toROSMsg(*new_pc, pc_new_msg);
    pc_new_msg.header = old_msg->header;
    pc_new_msg.header.frame_id = "velodyne";
    pubRobosensePC->publish(pc_new_msg);
}

void rsHandler_XYZI(sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_new(new pcl::PointCloud<VelodynePointXYZIR>());
    pcl::fromROSMsg(*pc_msg, *pc);

    // to new pointcloud
    for (size_t point_id = 0; point_id < pc->points.size(); ++point_id) {
        if (has_nan(pc->points[point_id]))
            continue;

        VelodynePointXYZIR new_point;
        new_point.x = pc->points[point_id].x;
        new_point.y = pc->points[point_id].y;
        new_point.z = pc->points[point_id].z;
        new_point.intensity = pc->points[point_id].intensity;
        // remap ring id
        if (pc->height == 16) {
            new_point.ring = RING_ID_MAP_16[point_id / pc->width];
        } else if (pc->height == 128) {
            new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
        }
        pc_new->points.push_back(new_point);
    }

    publish_points(pc_new, pc_msg);
}


template<typename T_in_p, typename T_out_p>
void handle_pc_msg(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                   const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {

    // to new pointcloud
    for (size_t point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        T_out_p new_point;
//        std::copy(pc->points[point_id].data, pc->points[point_id].data + 4, new_point.data);
        new_point.x = pc_in->points[point_id].x;
        new_point.y = pc_in->points[point_id].y;
        new_point.z = pc_in->points[point_id].z;
        new_point.intensity = pc_in->points[point_id].intensity;
//        new_point.ring = pc->points[point_id].ring;
//        // 计算相对于第一个点的相对时间
//        new_point.time = float(pc->points[point_id].timestamp - pc->points[0].timestamp);
        pc_out->points.push_back(new_point);
    }
}

template<typename T_in_p, typename T_out_p>
void add_ring(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    // to new pointcloud
    int valid_point_id = 0;
    for (size_t point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        // 跳过nan点
        pc_out->points[valid_point_id++].ring = pc_in->points[point_id].ring;
    }
}

template<typename T_in_p, typename T_out_p>
void add_time(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    // to new pointcloud
    int valid_point_id = 0;
    for (size_t point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        // 跳过nan点
        pc_out->points[valid_point_id++].time = float(pc_in->points[point_id].timestamp - pc_in->points[0].timestamp);
    }
}

void rsHandler_XYZIRT(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    pcl::PointCloud<RsPointXYZIRT>::Ptr pc_in(new pcl::PointCloud<RsPointXYZIRT>());
    pcl::fromROSMsg(*pc_msg, *pc_in);

    if (output_type == "XYZIRT") {
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIRT>());
        handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        add_ring<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        add_time<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);
    } else if (output_type == "XYZIR") {
        pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIR>());
        handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
        add_ring<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);
    } else if (output_type == "XYZI") {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZI>());
        handle_pc_msg<RsPointXYZIRT, pcl::PointXYZI>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);
    }
}

int main(int argc, char **argv) {
    
    rclcpp::init(argc, argv);
     
    auto nodeHandle = rclcpp::Node::make_shared("rs_converter");
    
    
    // Declare parameters
    
    nodeHandle->declare_parameter<std::string>("output_type");
    nodeHandle->declare_parameter<std::string>("input_type");
    nodeHandle->declare_parameter<std::vector<int64_t>>("RING_ID_MAP_RUBY");
    nodeHandle->declare_parameter<std::vector<int64_t>>("RING_ID_MAP_16");
     // Retrieve parameters
    output_type = nodeHandle->get_parameter("output_type").as_string();
    input_type = nodeHandle->get_parameter("input_type").as_string();

    RING_ID_MAP_RUBY = nodeHandle->get_parameter("RING_ID_MAP_RUBY").as_integer_array();
    RING_ID_MAP_16 = nodeHandle->get_parameter("RING_ID_MAP_16").as_integer_array();
    
    
    RCLCPP_INFO(nodeHandle->get_logger(), "Output Type: %s", output_type.c_str());


    if (std::strcmp("XYZI", input_type.c_str()) == 0) {
        subRobosensePC = nodeHandle->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points", 1, rsHandler_XYZI);
    } else if (std::strcmp("XYZIRT", input_type.c_str()) == 0) {
        subRobosensePC = nodeHandle->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points", 1, rsHandler_XYZIRT);
    }
    pubRobosensePC = nodeHandle->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", 1);

    RCLCPP_INFO(nodeHandle->get_logger(),"Listening to /rslidar_points ......");
    rclcpp::spin(nodeHandle);
    rclcpp::shutdown(); 
    return 0;
}
