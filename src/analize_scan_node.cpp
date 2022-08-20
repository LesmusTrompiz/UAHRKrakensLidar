#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "analize_scan_utils.hpp"

class AnalizeScanNode : public rclcpp::Node
{
    public:
        AnalizeScanNode();
        ~AnalizeScanNode();

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr obs_pub;
        void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        std::vector<Point2d> cloud;
        std::vector<cluster> clusters;
        std::vector<cluster> filtered_clusters;

};


AnalizeScanNode::AnalizeScanNode(void) : Node("analize_scan_node") 
{
    using std::placeholders::_1;
    this->scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&AnalizeScanNode::scan_cb, this, _1));
    this->obs_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("obstacles", 10);
}
AnalizeScanNode::~AnalizeScanNode(void){}

void AnalizeScanNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    
    LaserRangeTo2dPoints(msg->ranges,msg->angle_increment, cloud);
    get_clusters(cloud, clusters, 0.1);
    filter_clusters_by_length(clusters, 0.03, 0.142, filtered_clusters);

    return;
}





int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnalizeScanNode>());
  rclcpp::shutdown();
  return 0;
}