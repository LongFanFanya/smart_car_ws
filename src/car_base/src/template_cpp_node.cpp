#include "rclcpp/rclcpp.hpp"



class myCustomNode : public rclcpp::Node{

public:
    myCustomNode() : Node("node_name")
    {

    }

private:



};


int main(int argc,char **argv)
{

    rclcpp::init(argc,argv);

    auto node=std::make_shared<myCustomNode>();

    rclcpp::spin(node);


    rclcpp::shutdown();
    return 0;
}