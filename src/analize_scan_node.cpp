#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "analize_scan_utils.hpp"

class AnalizeScanNode : public rclcpp::Node
{
    public:
        AnalizeScanNode();
        ~AnalizeScanNode();

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obs_pub;
        void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        std::vector<Point2d> cloud;
        std::vector<cluster> clusters;
        std::vector<cluster> filtered_clusters;
        geometry_msgs::msg::PoseArray obstacles;
};


AnalizeScanNode::AnalizeScanNode(void) : Node("analize_scan_node") 
{
    using std::placeholders::_1;
    this->scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/ldlidar_node/scan", 10, std::bind(&AnalizeScanNode::scan_cb, this, _1));
    this->obs_pub  = this->create_publisher<geometry_msgs::msg::PoseArray>("obstacles", 10);
    cloud.reserve(400);
    clusters.reserve(25);
    filtered_clusters.reserve(10);
    obstacles.poses.reserve(10);
}
AnalizeScanNode::~AnalizeScanNode(void){}

void AnalizeScanNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    geometry_msgs::msg::Pose pose;
    Point2d p;
    obstacles.poses.clear();
    

    LaserRangeTo2dPoints(msg->ranges,msg->angle_increment, cloud);
    get_clusters(cloud, clusters, 0.1);
    filter_clusters_by_length(clusters, 0.03, 0.142, filtered_clusters);
    get_clusters_contour_centroids(filtered_clusters, detected_centroids);
    //track_clusters(detected_centroids, tracked_centroids);
    //pub_old_centroids(tracked_centroids);

    for (auto & cluster_ : filtered_clusters)
    {
        pose.position.x = p.x;
        pose.position.y = p.y;
        obstacles.poses.push_back(pose);
    }
    obstacles.header.frame_id = msg->header.frame_id;
    obstacles.header.stamp    = msg->header.stamp;
    obs_pub->publish(obstacles);

    // Track scan

    // Publis 

    return;
}





int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnalizeScanNode>());
  rclcpp::shutdown();
  return 0;
}