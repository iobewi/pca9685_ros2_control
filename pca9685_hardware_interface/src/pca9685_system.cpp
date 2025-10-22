#include "pca9685_hardware_interface/pca9685_system.hpp"

#include <algorithm>
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
    pca9685_hz_ = std::stod(info_.hardware_parameters["pca9685_hz"]);
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
  hw_runnings_positions_.resize(info_.joints.size(), false);

  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_runnings_velocities_.resize(info_.joints.size(), false);

  port_id_.resize(info_.joints.size(), -1);
  max_rpm_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  max_angle_rad_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  invert_signal_.resize(info_.joints.size(), false);

  has_position_command_.resize(info_.joints.size(), false);
  has_velocity_command_.resize(info_.joints.size(), false);
  has_position_state_.resize(info_.joints.size(), false);
  has_velocity_state_.resize(info_.joints.size(), false);

  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto & joint = info_.joints[i];

    if (joint.command_interfaces.empty() || joint.command_interfaces.size() > 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %zu command interfaces. Expected 1 or 2.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto & command_interface : joint.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        if (has_position_command_[i]) {
          RCLCPP_FATAL(
            rclcpp::get_logger("Pca9685SystemHardware"),
            "Joint '%s' has duplicated position command interface.", joint.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
        has_position_command_[i] = true;
      } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
        if (has_velocity_command_[i]) {
          RCLCPP_FATAL(
            rclcpp::get_logger("Pca9685SystemHardware"),
            "Joint '%s' has duplicated velocity command interface.", joint.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
        has_velocity_command_[i] = true;
      } else {
        RCLCPP_FATAL(
          rclcpp::get_logger("Pca9685SystemHardware"),
          "Joint '%s' has unsupported command interface '%s'.",
          joint.name.c_str(), command_interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (joint.state_interfaces.size() > 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %zu state interfaces. Expected at most 2.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto & state_interface : joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        if (has_position_state_[i]) {
          RCLCPP_FATAL(
            rclcpp::get_logger("Pca9685SystemHardware"),
            "Joint '%s' has duplicated position state interface.", joint.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
        has_position_state_[i] = true;
      } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        if (has_velocity_state_[i]) {
          RCLCPP_FATAL(
            rclcpp::get_logger("Pca9685SystemHardware"),
            "Joint '%s' has duplicated velocity state interface.", joint.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
        has_velocity_state_[i] = true;
      } else {
        RCLCPP_FATAL(
          rclcpp::get_logger("Pca9685SystemHardware"),
          "Joint '%s' has unsupported state interface '%s'.",
          joint.name.c_str(), state_interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (joint.parameters.find("port_id") != joint.parameters.end()){
      port_id_[i]=std::stoi(joint.parameters.at("port_id"));
    }else{
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has 'port_id' not set.",
       joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.parameters.find("reverse_command") != joint.parameters.end()) {
      invert_signal_[i] =  std::stoi(joint.parameters.at("reverse_command"));
    }
    if (joint.parameters.find("max_rpm") != joint.parameters.end()){
      max_rpm_[i]=std::stod(joint.parameters.at("max_rpm"));
    }
    if (joint.parameters.find("max_degrees") != joint.parameters.end()){
      const double max_degrees = std::stod(joint.parameters.at("max_degrees"));
      max_angle_rad_[i] = max_degrees * M_PI / 180.0;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    for (const auto & state_interface : info_.joints[i].state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
      } else if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    for (const auto & command_interface : info_.joints[i].command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
      } else if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
      }
    }
  }
  return command_interfaces;
}

hardware_interface::return_type Pca9685SystemHardware::prepare_command_mode_switch(
  std::vector<std::string> const& start_interfaces, std::vector<std::string> const& stop_interfaces) {

  for (auto const& stop_interface: stop_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (stop_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION &&
          has_position_command_[i]) {
        hw_runnings_positions_[i] = false;
      } else if (stop_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        if (!has_velocity_command_[i]) {
          continue;
        }
        hw_runnings_velocities_[i] = false;
      }
    }
  }
  for (auto const& start_interface: start_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (start_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        if (!has_position_command_[i]) {
          RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
            "Can't claim position interface for joint '%s': interface not declared.",
            info_.joints[i].name.c_str());
          return hardware_interface::return_type::ERROR;
        }
        if (std::isnan(max_angle_rad_[i])) {
          RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
            "Can't claim position interface for joint '%s': max_angle_rad is NaN!",
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
        if (!has_velocity_command_[i]) {
          RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
            "Can't claim velocity interface for joint '%s': interface not declared.",
            info_.joints[i].name.c_str());
          return hardware_interface::return_type::ERROR;
        }
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
      if (stop_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION &&
          has_position_command_[i]) {
        hw_runnings_positions_[i] = false;
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
          "Stopping position interface for joint '%s'.",
          info_.joints[i].name.c_str());
      } else if (stop_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY &&
                 has_velocity_command_[i]) {
        hw_runnings_velocities_[i] = false;
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
          "Stopping velocity interface for joint '%s'.",
          info_.joints[i].name.c_str());
      }
    }
  }
  for (auto const& start_interface: start_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (start_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION &&
          has_position_command_[i]) {
        hw_runnings_positions_[i] = true;
        RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
          "Starting position interface for joint '%s'.",
          info_.joints[i].name.c_str());
      } else if (start_interface == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY &&
                 has_velocity_command_[i]) {
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
  if (!pca_) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
      "Cannot activate hardware: PCA9685 instance is null.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (std::isnan(hw_commands_positions_[i])) hw_commands_positions_[i] = 0.0;
    if (std::isnan(hw_states_positions_[i])) hw_states_positions_[i] = 0.0;

    if (std::isnan(hw_commands_velocities_[i])) hw_commands_velocities_[i] = 0.0;
    if (std::isnan(hw_states_velocities_[i])) hw_states_velocities_[i] = 0.0;

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
  if (pca_) {
    pca_->shutdown();
  }
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (hw_runnings_positions_[i]) {
      const double max_angle = max_angle_rad_[i];
      if (std::isnan(max_angle) || max_angle <= 0.0) {
        hw_states_positions_[i] = 0.0;
      } else {
        const double normalized_command = std::clamp(hw_commands_[i], -1.0, 1.0);
        hw_states_positions_[i] = normalized_command * max_angle;
      }
      hw_states_velocities_[i] = 0.0;
    } else if (hw_runnings_velocities_[i]) {
      const double rad_per_sec = (2.0 * M_PI * max_rpm_[i] * hw_commands_[i]) / 60.0;
      hw_states_velocities_[i] = rad_per_sec;
      if (std::isnan(hw_states_positions_[i])) {
        hw_states_positions_[i] = 0.0;
      }
      hw_states_positions_[i] += rad_per_sec * period.seconds();
    } else {
      if (!std::isnan(hw_states_velocities_[i])) {
        hw_states_velocities_[i] = 0.0;
      }
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

    double duty_cycle = slope * clamped_command + offset;

    return std::clamp(duty_cycle, min_duty_cycle, max_duty_cycle);

}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!pca_) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
      "Cannot write commands: PCA9685 instance is null.");
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    double normalized_command = 0.0;
    if (hw_runnings_positions_[i]) {
      const double max_angle = max_angle_rad_[i];
      if (std::isnan(max_angle) || max_angle <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
          "Joint '%s' has invalid max_angle_rad (%f).", info_.joints[i].name.c_str(), max_angle);
        normalized_command = 0.0;
      } else {
        const double saturated_position = std::clamp(hw_commands_positions_[i], -max_angle, max_angle);
        hw_commands_positions_[i] = saturated_position;
        normalized_command = saturated_position / max_angle;
      }
    } else if (hw_runnings_velocities_[i]) {
      if (std::isnan(max_rpm_[i]) || max_rpm_[i] <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685SystemHardware"),
          "Joint '%s' has invalid max_rpm (%f).", info_.joints[i].name.c_str(), max_rpm_[i]);
        normalized_command = 0.0;
      } else {
        normalized_command = (hw_commands_velocities_[i] * 60.0) /
          (2.0 * M_PI * max_rpm_[i]);
      }
    } else {
      normalized_command = 0.0;
    }

    if (!std::isfinite(normalized_command)) {
      normalized_command = 0.0;
    }

    if (invert_signal_[i]) {
      normalized_command *= -1.0;
    }

    normalized_command = std::clamp(normalized_command, -1.0, 1.0);
    hw_commands_[i] = normalized_command;

    double duty_cycle = command_to_duty_cycle(normalized_command);

    // RCLCPP_INFO(
    //     rclcpp::get_logger("Pca9685SystemHardware"),
    //     "Joint '%ld' has command '%f', duty_cycle: '%f'.", i, hw_commands_[i], duty_cycle);

    pca_->set_pwm_ms(port_id_[i], duty_cycle);

  }

  return hardware_interface::return_type::OK;
}

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)
