#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"



class AnalizeScanNode : public rclcpp::Node
{
    public:
        AnalizeScanNode();
        ~AnalizeScanNode();

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
        void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};


AnalizeScanNode::AnalizeScanNode(void) : Node("analize_scan_node") 
{
    using std::placeholders::_1;
    this->scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&AnalizeScanNode::scan_cb, this, _1));
}
AnalizeScanNode::~AnalizeScanNode(void){}

void AnalizeScanNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{

}





int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnalizeScanNode>());
  rclcpp::shutdown();
  return 0;
}