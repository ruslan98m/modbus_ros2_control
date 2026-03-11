// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file master_params_parser.hpp
 * @brief Parse HardwareInfo (hardware_parameters from URDF) into modbus_master::MasterParams.
 */

#ifndef MODBUS_HW_INTERFACE__MASTER_PARAMS_PARSER_HPP_
#define MODBUS_HW_INTERFACE__MASTER_PARAMS_PARSER_HPP_

#include "hardware_interface/hardware_info.hpp"
#include "modbus_master/master_params.hpp"

namespace modbus_hw_interface {

/**
 * Parse hardware info (hardware_parameters) into MasterParams.
 * Keys: connection_type, ip_address, port, serial_port, baud_rate, parity,
 * data_bits, stop_bits, poll_rate_hz, thread_priority, cpu_affinity.
 * @return true on success, false on invalid connection_type.
 */
bool parseMasterParams(const hardware_interface::HardwareInfo& info,
                       modbus_master::MasterParams& out);

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MASTER_PARAMS_PARSER_HPP_
