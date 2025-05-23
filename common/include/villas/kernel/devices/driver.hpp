/* Interface for device drivers. OS/platform independend.
 * Implemented for Linux/Unix drivers in linux_driver.hpp
 *
 * Author: Pascal Bauer <pascal.bauer@rwth-aachen.de>
 *
 * SPDX-FileCopyrightText: 2023-2024 Pascal Bauer <pascal.bauer@rwth-aachen.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <string>

namespace villas {
namespace kernel {
namespace devices {

class Device;

class Driver {
public:
  virtual void attach(const Device &device) const = 0;
  virtual void bind(const Device &device) const = 0;
  virtual std::string name() const = 0;
  virtual void override(const Device &device) const = 0;
  virtual void unbind(const Device &device) const = 0;
};

} // namespace devices
} // namespace kernel
} // namespace villas
