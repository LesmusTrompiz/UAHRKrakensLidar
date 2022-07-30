#include "rclcpp/rclcpp.hpp"


class AnalizeScanNode : public rclcpp::Node
{
  public:
    AnalizeScanNode();
    ~AnalizeScanNode();
}


AnalizeScanNode::AnalizeScanNode(void) : Node("analize_scan_node")
{
    return;
}


AnalizeScanNode::~AnalizeScanNode(void)
{
    return;
}



int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnalizeScanNode>());
  rclcpp::shutdown();
  return 0;
}