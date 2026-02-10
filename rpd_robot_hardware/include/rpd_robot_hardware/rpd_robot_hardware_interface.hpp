#ifndef RPD_ROBOT_HARDWARE_INTERFACE_HPP
#define RPD_ROBOT_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "SCServo.h"

namespace rpd_robot_hardware {

class RpdRobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo &info) override;
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

    hardware_interface::return_type
        read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type
        write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    SMS_STS driver_;
    std::string port_;
    int baudrate_;
    size_t num_of_joints_;

    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> acceleration_states_;
    std::vector<double> position_commands_;
    std::vector<double> velocity_commands_;
    std::vector<double> acceleration_commands_;

    std::vector<int> motor_ids_;
    std::vector<uint8_t> motor_ids_uint8_;
    uint8_t rx_packet_[4];


}; // class RpdRobotHardwareInterface

} // namespace rpd_robot_hardware

#endif // RPD_ROBOT_HARDWARE_INTERFACE_HPP