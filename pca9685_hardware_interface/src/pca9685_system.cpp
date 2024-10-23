#include "pca9685_hardware_interface/pca9685_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"



namespace pca9685_hardware_interface
{

hardware_interface::CallbackReturn Pca9685SystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  if (info_.hardware_parameters.find("pca9685_dev") != info_.hardware_parameters.end()) {
    pca9685_dev_ = info_.hardware_parameters["pca9685_dev"];
  } else {
    pca9685_dev_ = "/dev/i2c-1";
    RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "I2c device name not set, defaulting to '%s'", pca9685_dev_.c_str());
  }
  if (info_.hardware_parameters.find("pca9685_addr") != info_.hardware_parameters.end()) {
    pca9685_addr_ = std::stoi(info_.hardware_parameters["pca9685_addr"], nullptr, 16);
  } else {
    pca9685_addr_ = 0x40;
    RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "PCA9685 address not set, defaulting to '%x'", pca9685_addr_ );
  }
  if (info_.hardware_parameters.find("pca9685_hz") != info_.hardware_parameters.end()) {
    pca9685_hz_ = std::stoi(info_.hardware_parameters["pca9685_hz"]);
  } else {
    pca9685_hz_ = 50.0;
    RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "PCA9685 frequency not set, defaulting to '%f'", pca9685_hz_ );
  }

  pca_ = std::make_unique<PiPCA9685::PCA9685>(pca9685_dev_, pca9685_addr_);
  pca_->set_pwm_freq(pca9685_hz_);
  return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn Pca9685SystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (pca_) {
    RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Cleaning up PCA9685 instance.");
    pca_.reset();  
    } else {
    RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "PCA9685 instance already cleaned up.");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
    
hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_runnings_positions_.resize(info_.joints.size(), std::numeric_limits<bool>::quiet_NaN());

  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_runnings_velocities_.resize(info_.joints.size(), std::numeric_limits<bool>::quiet_NaN());

  port_id_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  max_rpm_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  max_degrees_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Pca9685System has one command interface on each output
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY &&
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %s command interfaces found. 'HW_IF_VELOCITY' or 'HW_IF_POSITION' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  for (size_t i = 0; i < info_.joints.size(); i++) {
    if (info_.joints[i].parameters.find("port_id") != info_.joints[i].parameters.end()){
      port_id_[i]=std::stoi(info_.joints[i].parameters.at("port_id"));
    }else{
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has 'port_id' not set.",
       info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (info_.joints[i].parameters.find("max_rpm") != info_.joints[i].parameters.end()){
      max_rpm_[i]=std::stoi(info_.joints[i].parameters.at("max_rpm"));
    }
    if (info_.joints[i].parameters.find("max_degrees") != info_.joints[i].parameters.end()){
      max_degrees_[i]=std::stoi(info_.joints[i].parameters.at("max_degrees"));
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type Pca9685SystemHardware::prepare_command_mode_switch(
  std::vector<std::string> const& start_interfaces, std::vector<std::string> const& stop_interfaces) {

  for (auto const& stop_interface: stop_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (stop_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        hw_runnings_positions_[i] = false;
      } else if (stop_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        hw_runnings_velocities_[i] = false;
      }
    }
  }
  for (auto const& start_interface: start_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (start_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        if (std::isnan(max_degrees_[i])) {
          RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
            "Can't claim position interface for joint '%s': max_degree is NaN!", 
            info_.joints[i].name.c_str());
          return hardware_interface::return_type::ERROR;
        }
        if (hw_runnings_velocities_[i]) {
          RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
            "Can't claim position interface for joint '%s': Velocity interface is already claimed!", 
            info_.joints[i].name.c_str());
          return hardware_interface::return_type::ERROR;
        }
        hw_runnings_positions_[i] = true;
      } else if (start_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        if (std::isnan(max_rpm_[i])) {
          RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
             "Can't claim velocity interface for joint '%s': max_rpm is NaN!", 
            info_.joints[i].name.c_str());
          return hardware_interface::return_type::ERROR;
        }
        if (hw_runnings_positions_[i]) {
          RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
            "Can't claim position interface for joint '%s': Position interface is already claimed!", 
            info_.joints[i].name.c_str());
          return hardware_interface::return_type::ERROR;
        }
        hw_runnings_velocities_[i] = true;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Pca9685SystemHardware::perform_command_mode_switch(
  std::vector<std::string> const& start_interfaces, std::vector<std::string> const& stop_interfaces) {
  for (auto const& stop_interface: stop_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (stop_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        hw_runnings_positions_[i] = false;
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
          "Stopping position interface for joint '%s'.", 
          info_.joints[i].name.c_str());
      } else if (stop_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        hw_runnings_velocities_[i] = false;
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
          "Stopping velocity interface for joint '%s'.", 
          info_.joints[i].name.c_str());
      }
    }
  }
  for (auto const& start_interface: start_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (start_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        hw_runnings_positions_[i] = true;
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
          "Starting position interface for joint '%s'.", 
          info_.joints[i].name.c_str());
      } else if (start_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        hw_runnings_velocities_[i] = true;
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
          "Starting velocity interface for joint '%s'.", 
          info_.joints[i].name.c_str());
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (std::isnan(hw_commands_positions_[i])) hw_commands_positions_[i] = 0.0;
    if (std::isnan(hw_states_positions_[i])) hw_states_positions_[i] = 0.0;
    if (!hw_runnings_positions_[i]) hw_runnings_positions_[i] = false;

    if (std::isnan(hw_commands_velocities_[i])) hw_commands_velocities_[i] = 0.0;
    if (std::isnan(hw_states_velocities_[i])) hw_states_velocities_[i] = 0.0;
    if (!hw_runnings_velocities_[i]) hw_runnings_velocities_[i] = false;

    if (std::isnan(hw_commands_[i])) hw_commands_[i] = 0;
  }
  pca_->activate();
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    for (size_t i = 0; i < info_.joints.size(); i++)
  {
    hw_commands_[i] = 0;
  }
  pca_->shutdown();
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (hw_runnings_positions_[i]) {
      hw_states_positions_[i] = (M_PI ) * (hw_commands_[i]);
    } else if (hw_runnings_velocities_[i]) {
      hw_states_velocities_[i] = (M_PI * max_rpm_[i] * hw_commands_[i]) / 60.0;
      hw_states_positions_[i] += hw_states_velocities_[i] * period.seconds();
    }
  }

  return hardware_interface::return_type::OK;
}

double Pca9685SystemHardware::command_to_duty_cycle(double command){

    double min_input = -1.0;
    double max_input = 1.0;

    double clamped_command = std::clamp(command, min_input, max_input);

    double min_duty_cycle = 0.5;
    double max_duty_cycle = 2.5;


    double slope = (max_duty_cycle-min_duty_cycle)/(max_input-min_input);
    double offset = (max_duty_cycle+min_duty_cycle)/2;

    return slope * clamped_command + offset;

}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (hw_runnings_positions_[i]) {
      hw_commands_[i] = ((hw_commands_positions_[i]) / M_PI);
    } else if (hw_runnings_velocities_[i]) {
      hw_commands_[i] = (hw_commands_velocities_[i] * 60)/(M_PI * max_rpm_[i]);
  }
    double duty_cycle = command_to_duty_cycle(hw_commands_[i]);

    // RCLCPP_INFO(
    //     rclcpp::get_logger("Pca9685SystemHardware"),
    //     "Joint '%d' has command '%f', duty_cycle: '%f'.", i, hw_commands_[i], duty_cycle);

    pca_->set_pwm_ms(port_id_[i], duty_cycle);

  }

  return hardware_interface::return_type::OK;
}

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)
