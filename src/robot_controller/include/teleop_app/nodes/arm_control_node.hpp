#ifndef TELEOP_APP__NODES__ARM_CONTROL_NODE_HPP_
#define TELEOP_APP__NODES__ARM_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace teleop_app {
namespace nodes {

class StartupCheckState;
class TeleopInitState;
class TeleopState;
class FaultState;
class TeleopIdleState;

class ArmControlNode : public rclcpp::Node {
public:
    explicit ArmControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmControlNode();

    friend class StartupCheckState;
    friend class TeleopInitState;
    friend class TeleopState;
    friend class FaultState;
    friend class TeleopIdleState;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace nodes
}  // namespace teleop_app

#endif  // TELEOP_APP__NODES__ARM_CONTROL_NODE_HPP_
