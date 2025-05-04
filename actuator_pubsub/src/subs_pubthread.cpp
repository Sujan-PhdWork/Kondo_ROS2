#include "base_splitter_node.hpp"

class SplitterNode1 : public BaseSplitterNode {
public:
  SplitterNode1() : BaseSplitterNode("splitter_node1", "position_part1", 0, 18) {}
};

class SplitterNode2 : public BaseSplitterNode {
public:
  SplitterNode2() : BaseSplitterNode("splitter_node2", "position_part2", 18, 36) {}
};

class SplitterNode3 : public BaseSplitterNode {
public:
  SplitterNode3() : BaseSplitterNode("splitter_node3", "position_part3", 36, 48) {}
};

class SplitterNode4 : public BaseSplitterNode {
public:
  SplitterNode4() : BaseSplitterNode("splitter_node4", "position_part4", 48, 60) {}
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
  
    auto node1 = std::make_shared<SplitterNode1>();
    auto node2 = std::make_shared<SplitterNode2>();
    auto node3 = std::make_shared<SplitterNode3>();
    auto node4 = std::make_shared<SplitterNode4>();
  
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.add_node(node3);
    executor.add_node(node4);
  
    executor.spin();
    rclcpp::shutdown();
    return 0;
  }
  