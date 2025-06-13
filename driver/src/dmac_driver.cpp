/* Copyright (c) 2016, Oleksiy Kebkal <lesha@evologics.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <sstream>

#include "comm_middlemen.h"
#include "config.h"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_client.h"
#include "tcp_client.h"

namespace {

// Parameter names.
constexpr char kTypeParam[] = "dmac2/modem_config/connection_type";
constexpr char kIpParam[] = "dmac2/modem_config/tcp_config/ip";
constexpr char kPortParam[] = "dmac2/modem_config/tcp_config/port";

// Default parameter values.
constexpr char kDefaultType[] = "TCP/IP";
constexpr char kDefaultIp[] = "192.168.0.206";
constexpr int kDefaultPort = 9200;

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  dmac::config config("dmac2_node");

  // Create a node just to read configuration parameters.
  rclcpp::Node param_node("param_node");
  param_node.declare_parameter(kTypeParam, kDefaultType);
  param_node.declare_parameter(kIpParam, kDefaultIp);
  param_node.declare_parameter(kPortParam, kDefaultPort);

  boost::asio::io_service io_service;
  rclcpp::executors::MultiThreadedExecutor executor;

  const std::string type = param_node.get_parameter(kTypeParam).as_string();
  if (type == "TCP/IP") {
    const std::string IP = param_node.get_parameter(kIpParam).as_string();
    const int port = param_node.get_parameter(kPortParam).as_int();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"),
                       "Connecting to IP: " << IP << ", port: " << port);

    // Listen for TCP connections in background thread.
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(IP, std::to_string(port));
    tcp::resolver::iterator iterator = resolver.resolve(query);

    dmac::tcp_client* s = new dmac::tcp_client(io_service, iterator, config);
    if (s != nullptr && s->parser_node() != nullptr) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"),
                         "Adding the parser node for TCP/IP mode");
      executor.add_node(s->parser_node());
    }
  } else if (type == "SERIAL") {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"),
                       "Connecting to serial modem");
    dmac::serial_client* s = new dmac::serial_client(io_service, config);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("dmac2_logger"),
                        "Unsupported connection type: " << type);
    return -1;
  }

  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"), "Spinning... ");
  executor.spin();

  return 0;
}
