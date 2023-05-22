/* TODO
 * - add all observation sources
 * - set the update flag when the pcl callback is called
 * - update the layer when the update flag is set to true
 * - post process the layer when all points are added to it
 * - aggregate all layers when the update costmap service is called
 * - sort the TF tree stuff, i.e. copy the current tf between base_link and odom,
 *        and publish that till the next time the map is updated
 */

#include <cstdio>
#include <iostream>

#include "anymap.hpp"
#include "observation_source.hpp"
#include "layer_postprocessor.hpp"

#include "lane_extension.hpp"


#include "anymap_interfaces/srv/trigger_update.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"


#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"



using namespace std::chrono_literals;

static grid_map::GridMapRosConverter conv;

class AnyMapNode : public rclcpp::Node
{
public:
    AnyMapNode();
private:
    // potholes_source
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr potholes_subscription;
    std::shared_ptr<observation_source::ObservationSource> potholes_source_ptr;
    layer_postprocessor::LayerPostProcessor potholes_postprocessor;
    pcl::PointCloud<POINT_TYPE>::Ptr potholes_cloud;
    void potholes_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr potholes_processed_publisher;


    // lidar source
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription;
    std::shared_ptr<observation_source::ObservationSource> lidar_source_ptr;
    layer_postprocessor::LayerPostProcessor lidar_postprocessor;
    pcl::PointCloud<POINT_TYPE>::Ptr lidar_cloud;
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_processed_publisher;


    // path source
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lanes_subscription;
    std::shared_ptr<observation_source::ObservationSource> lanes_source_ptr;
    layer_postprocessor::LayerPostProcessor lanes_postprocessor;
    pcl::PointCloud<POINT_TYPE>::Ptr lanes_cloud;
    void lanes_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lanes_processed_publisher;
    int lane_counter = 0;



    pcl::ConditionAnd<POINT_TYPE>::Ptr anymap_box_cond;
    pcl::ConditionalRemoval<POINT_TYPE> anymap_box_filter = pcl::ConditionalRemoval<POINT_TYPE>();


    rclcpp::Service<anymap_interfaces::srv::TriggerUpdate>::SharedPtr map_update_service;
    void update_anymap_callback(const std::shared_ptr<anymap_interfaces::srv::TriggerUpdate::Request> request,
                            std::shared_ptr<anymap_interfaces::srv::TriggerUpdate::Response> response);


    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr anymap_publisher;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tf_timer;


    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_msg_ptr;
    std::shared_ptr<grid_map::GridMap> anymap_ptr;


    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


    geometry_msgs::msg::TransformStamped odom_to_map;
    geometry_msgs::msg::TransformStamped odom_to_base;

    // to publish tf?
    void timer_callback();
    void tf_publisher_callback();
};

AnyMapNode::AnyMapNode() : Node("anymap_node") {
    // initialize the anymap instance, to which layers will be added
    std::cout << "Initializing anymap\n";
    this->anymap_ptr = std::shared_ptr<grid_map::GridMap>(new grid_map::GridMap);
    *anymap_ptr.get() = anymap::init_anymap();

    // the anymap publisher
    anymap_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("anymap", 10);

    // initialize the service
    this->map_update_service = this->create_service<anymap_interfaces::srv::TriggerUpdate>
        ("~/trigger_update", std::bind(&AnyMapNode::update_anymap_callback, this, std::placeholders::_1, std::placeholders::_2));


    // variables related to lanes layer
    rmw_qos_profile_t lanes_qos_profile = rmw_qos_profile_sensor_data;
    auto lanes_qos = rclcpp::QoS(rclcpp::QoSInitialization(lanes_qos_profile.history, 15), lanes_qos_profile);
    lanes_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lane_points", lanes_qos, std::bind(&AnyMapNode::lanes_callback, this, std::placeholders::_1));
    this->lanes_source_ptr = std::shared_ptr<observation_source::ObservationSource>(
        new observation_source::ObservationSource("lanes", this->anymap_ptr));
    this->lanes_postprocessor.set_layer_name("lanes");
    this->lanes_postprocessor.set_input_grid(this->anymap_ptr);
    this->lanes_postprocessor.set_inflation(37, 37);
    pcl::PointCloud<POINT_TYPE>::Ptr temp_lanes_cloud (new pcl::PointCloud<POINT_TYPE>());
    this->lanes_cloud = temp_lanes_cloud;
    this->lanes_processed_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("lanes_processed_points", 10);



    // variables related to the potholes layer
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 15), qos_profile);
    potholes_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/potholes_points", qos, std::bind(&AnyMapNode::potholes_callback, this, std::placeholders::_1));
    this->potholes_source_ptr = std::shared_ptr<observation_source::ObservationSource>(
        new observation_source::ObservationSource("potholes", this->anymap_ptr));
    this->potholes_postprocessor.set_layer_name("potholes");
    this->potholes_postprocessor.set_input_grid(this->anymap_ptr);
    pcl::PointCloud<POINT_TYPE>::Ptr temp_potholes_cloud (new pcl::PointCloud<POINT_TYPE>());
    this->potholes_cloud = temp_potholes_cloud;
    potholes_processed_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("potholes_processed_points", 10);



    // variables related to the lidar layer
    lidar_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        // "camera/depth/color/points", 15, std::bind(&AnyMapNode::lidar_callback, this, std::placeholders::_1));
        "/scan_pcl", 15, std::bind(&AnyMapNode::lidar_callback, this, std::placeholders::_1));
    this->lidar_source_ptr = std::shared_ptr<observation_source::ObservationSource>(
        new observation_source::ObservationSource("lidar", this->anymap_ptr));
    this->lidar_postprocessor.set_layer_name("lidar");
    this->lidar_postprocessor.set_input_grid(this->anymap_ptr);
    this->lidar_postprocessor.set_inflation(34, 34);

    pcl::PointCloud<POINT_TYPE>::Ptr temp_lidar_cloud (new pcl::PointCloud<POINT_TYPE>());
    this->lidar_cloud = temp_lidar_cloud;
    this->lidar_processed_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_processed_points", 10);


    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    // anymap box filter
    pcl::ConditionAnd<POINT_TYPE>::Ptr range_cond (new pcl::ConditionAnd<POINT_TYPE>());
    anymap_box_cond = range_cond;
    anymap_box_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::LT, 1.80)));
    anymap_box_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::GT, -0.1)));
    anymap_box_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("x", pcl::ComparisonOps::LT, 7.6)));
    anymap_box_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("x", pcl::ComparisonOps::GT, 0)));
    anymap_box_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("y", pcl::ComparisonOps::LT, 3.8)));
    anymap_box_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("y", pcl::ComparisonOps::GT, -3.8)));
    anymap_box_filter.setCondition(anymap_box_cond);

    this->grid_msg_ptr = std::shared_ptr<nav_msgs::msg::OccupancyGrid>(new nav_msgs::msg::OccupancyGrid);

    // TODO replace this with an action server that updates the map everytime it is called
    this->timer_ = this->create_wall_timer(150ms, std::bind(&AnyMapNode::timer_callback, this));
    this->tf_timer = this->create_wall_timer(2ms, std::bind(&AnyMapNode::tf_publisher_callback, this));

}

void AnyMapNode::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2 lidar_msg;
    pcl::fromROSMsg(*msg, *this->lidar_cloud);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    transform.pretranslate(Eigen::Vector3f(0.15, 0, 0.55));

    pcl::PointCloud<POINT_TYPE>::Ptr transformed_cloud (new pcl::PointCloud<POINT_TYPE>());
    pcl::transformPointCloud(*this->lidar_cloud, *transformed_cloud, transform);

    anymap_box_filter.setInputCloud(transformed_cloud);
    anymap_box_filter.filter(*transformed_cloud);

    pcl::toROSMsg(*transformed_cloud, lidar_msg);
    lidar_msg.header.frame_id = "base_link";
    this->lidar_processed_publisher->publish(lidar_msg);

    if (true) {
        this->lidar_source_ptr->clear_layer();
        this->lidar_source_ptr->set_update_flag();
        this->lidar_source_ptr->set_input_cloud(transformed_cloud);
        this->lidar_source_ptr->update_layer();

        this->lidar_postprocessor.process_layer();
    }
}

void AnyMapNode::lanes_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    this->lane_counter++;
    if (lane_counter != 7) {
        return;
    } else {
        lane_counter = 0;
    }

    sensor_msgs::msg::PointCloud2 lanes_msg;
    pcl::fromROSMsg(*msg, *this->lanes_cloud);


    pcl::PointCloud<POINT_TYPE>::Ptr lanes_filtered (new pcl::PointCloud<POINT_TYPE>());
    pcl::VoxelGrid<POINT_TYPE> sor;

    sor.setInputCloud(this->lanes_cloud);
    sor.setLeafSize(0.28f, 0.28f, 1.28f);
    sor.filter(*lanes_filtered);
    this->lanes_source_ptr->set_point_weight(0.6);

    Eigen::Affine3f rs_transform = Eigen::Affine3f::Identity();

    rs_transform.pretranslate(Eigen::Vector3f(-0.475, 0, 1.3));
    rs_transform.rotate(Eigen::AngleAxisf(3.14159/2.0, Eigen::Vector3f::UnitY()));
    rs_transform.rotate(Eigen::AngleAxisf(-3.14159/2.0, Eigen::Vector3f::UnitZ()));

    rs_transform.rotate(Eigen::AngleAxisf(18.2*180.0/3.14159, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<POINT_TYPE>::Ptr lanes_transformed_cloud (new pcl::PointCloud<POINT_TYPE>());
    pcl::transformPointCloud(*lanes_filtered, *lanes_transformed_cloud, rs_transform);


    anymap_box_filter.setInputCloud(lanes_transformed_cloud);
    anymap_box_filter.filter(*lanes_transformed_cloud);


    pcl::toROSMsg(*lanes_transformed_cloud, lanes_msg);
    lanes_msg.header.frame_id = "base_link";
    this->lanes_processed_publisher->publish(lanes_msg);

    this->lanes_source_ptr->clear_layer();
    this->lanes_source_ptr->set_update_flag();
    this->lanes_source_ptr->set_input_cloud(lanes_transformed_cloud);
    this->lanes_source_ptr->update_layer();

    this->lanes_postprocessor.process_layer();
    std::cout << "the lane layer image is of size " << lanes_postprocessor.image.size() << " " << lanes_postprocessor.image.channels() << std::endl;

    cv::Mat post_processor_uint8;
    this->lanes_postprocessor.image.convertTo(this->lanes_postprocessor.image, CV_8UC1, 255.0);
    this->lanes_postprocessor.image = lane_extension::process_lane_layer(this->lanes_postprocessor.image);
    cv::imshow("extended lanes image", this->lanes_postprocessor.image);
    this->lanes_postprocessor.image.convertTo(this->lanes_postprocessor.image, CV_32FC1, 1.0/255.0);
    this->lanes_postprocessor.back_to_grid();
}


void AnyMapNode::potholes_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    sensor_msgs::msg::PointCloud2 obstacles_msg;
    pcl::fromROSMsg(*msg, *this->potholes_cloud);

    pcl::PointCloud<POINT_TYPE>::Ptr cloud_filtered (new pcl::PointCloud<POINT_TYPE>());
    pcl::VoxelGrid<POINT_TYPE> sor;
    sor.setInputCloud (this->potholes_cloud);
    sor.setLeafSize (0.23f, 0.23f, 0.23f);
    sor.filter (*cloud_filtered);
    this->potholes_source_ptr->set_point_weight(0.6);

    Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();

    transform_y.pretranslate(Eigen::Vector3f(-0.475, 0, 1.3));
    transform_y.rotate(Eigen::AngleAxisf(3.14159/2.0, Eigen::Vector3f::UnitY()));
    transform_y.rotate(Eigen::AngleAxisf(-3.14159/2.0, Eigen::Vector3f::UnitZ()));

    transform_y.rotate(Eigen::AngleAxisf(26.2*180.0/3.1415, Eigen::Vector3f::UnitY()));
    pcl::PointCloud<POINT_TYPE>::Ptr transformed_cloud (new pcl::PointCloud<POINT_TYPE>());
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transform_y);

    anymap_box_filter.setInputCloud(transformed_cloud);
    anymap_box_filter.filter(*transformed_cloud);


    pcl::toROSMsg(*transformed_cloud, obstacles_msg);
    obstacles_msg.header.frame_id = "base_link";
    this->potholes_processed_publisher->publish(obstacles_msg);

    // TODO test and see whetHer this works well in realtime
    if (true) {
        this->potholes_source_ptr->clear_layer();
        this->potholes_source_ptr->set_update_flag();
        this->potholes_source_ptr->set_input_cloud(transformed_cloud);
        this->potholes_source_ptr->update_layer();

        this->potholes_postprocessor.process_layer();
    }

}

void AnyMapNode::update_anymap_callback(const std::shared_ptr<anymap_interfaces::srv::TriggerUpdate::Request> request,
                            std::shared_ptr<anymap_interfaces::srv::TriggerUpdate::Response> response) {
    std::cout << "anymap update map service called\n";
    response->success = true;
}

void AnyMapNode::timer_callback() {
    anymap::add_layer(this->anymap_ptr, "aggregate");
    if (this->anymap_ptr->exists("lidarProcessed")) {
        this->anymap_ptr->get("aggregate") += this->anymap_ptr->get("lidarProcessed");
    }

    if (this->anymap_ptr->exists("potholesProcessed")) {
        this->anymap_ptr->get("aggregate") += this->anymap_ptr->get("potholesProcessed");
    }

    if (this->anymap_ptr->exists("lanesProcessed")) {
        this->anymap_ptr->get("aggregate") += this->anymap_ptr->get("lanesProcessed");
    }

    try {
        this->odom_to_base = this->tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        std::cout << "Could not lookup transform, is the localization thingi running?\n";
    }

    conv.toOccupancyGrid(*this->anymap_ptr.get(), "aggregate", 0, 1, *this->grid_msg_ptr.get());
    grid_msg_ptr->header.frame_id = "map_link";
    this->anymap_publisher->publish(*this->grid_msg_ptr.get());
}

void AnyMapNode::tf_publisher_callback() {
    this->odom_to_map = this->odom_to_base;
    this->odom_to_map.header.frame_id = "odom";
    this->odom_to_map.child_frame_id = "map_link";
    this->odom_to_map.header.stamp = this->get_clock()->now();
    this->tf_broadcaster_->sendTransform(this->odom_to_map);
}

int main(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    auto anymap_node = std::make_shared<AnyMapNode>();
    rclcpp::spin(anymap_node);


    return 0;
}
