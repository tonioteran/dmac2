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
#ifndef DMAC_INITIALIZER_H
#define DMAC_INITIALIZER_H

#include <math.h>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <iostream>
#include <sstream>

#include "aparser.h"
#include "comm_middlemen.h"
#include "config.h"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "dmac2_interfaces/msg/dmac_async.hpp"
#include "dmac2_interfaces/msg/dmac_payload.hpp"
#include "dmac2_interfaces/msg/dmac_sync.hpp"
#include "dmac2_interfaces/msg/musbl_fix.hpp"
#include "rclcpp/rclcpp.hpp"

using boost::asio::ip::tcp;
using dmac2_interfaces::msg::DMACAsync;
using dmac2_interfaces::msg::DMACPayload;
using dmac2_interfaces::msg::DMACSync;
using dmac2_interfaces::msg::MUSBLFix;

namespace dmac {
typedef enum {
  IDLE = 0,
  REQUEST_MODE = 1,
  HANDLE_MODE = 2,
  HANDLE_YAR = 3,
  FINAL = 4
} conf_state;

typedef enum {
  EPS = 0, /* no events */
  INTERNAL = 1,
  RCV = 2,
  ERROR = 3,
  WRONG_RCV = 4,
  YAR = 5, /* yet another request */
  DONE = 6
} conf_event;

std::string conf_state_str(conf_state state) {
  switch (state) {
    case IDLE:
      return "IDLE";
    case REQUEST_MODE:
      return "REQUEST_MODE";
    case HANDLE_MODE:
      return "HANDLE_MODE";
    case HANDLE_YAR:
      return "HANDLE_YAR";
    case FINAL:
      return "FINAL";
    default:
      return "UNKNOWN";
  }
}

std::string conf_event_str(conf_event event) {
  switch (event) {
    case EPS:
      return "EPS";
    case INTERNAL:
      return "INTERNAL";
    case RCV:
      return "RCV";
    case ERROR:
      return "ERROR";
    case WRONG_RCV:
      return "WRONG_RCV";
    case YAR:
      return "YAR";
    case DONE:
      return "DONE";
    default:
      return "UNKNOWN";
  }
}

class initializer {
 public:
  initializer(dmac::abstract_parser *parser, dmac::config *config)
      : state_(IDLE), raw_buffer_("") {
    parser_ = parser;
    config_ = config;
    // block_others; /* while initializing */
  }

  bool ready(void) { return state_ == FINAL; }

  conf_state state(void) { return state_; }

  void connected(void) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"), "ini: connected");
    initializer_it = config_->initializer_begin();
    handle_event(INTERNAL);
  }

  void disconnected(void) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"), "ini: disconnected");
    state_ = IDLE;
  }

  void answer_timeout(void) {
    /* try again */
    state_ = IDLE;
    initializer_it = config_->initializer_begin();
    handle_event(INTERNAL);
  }

  void sync(DMACSync &sync_msg) {
    if (state_ == FINAL) {
      ;
    } else {
      if (sync_msg.report.compare(0, 5, "ERROR") == 0) {
        if (sync_msg.report == "ERROR NOT INITIALIZED") {  // ignore it
          ;
        } else {
          handle_event<DMACSync>(ERROR, &sync_msg);
        }
      } else {
        handle_event<DMACSync>(RCV, &sync_msg);
      }
    }
  }

  void raw(DMACPayload &raw_msg) {
    if (state_ == IDLE) {
      std::string next_raw_buffer = raw_buffer_ + raw_msg.payload;
      std::size_t match_len = sizeof("ERROR WRONG FORMAT\r\n");
      std::size_t offset = next_raw_buffer.find("ERROR WRONG FORMAT\r\n");
      if (offset == std::string::npos) {
        /* need more data */
        if (next_raw_buffer.length() > match_len) {
          raw_buffer_ =
              next_raw_buffer.substr(next_raw_buffer.length() - match_len);
        } else {
          raw_buffer_ = next_raw_buffer;
        }
      } else {
        raw_buffer_ = "";
        parser_->ctrl(dmac::WAITSYNC, dmac::WAITSYNC_NO);
        handle_event(ERROR);
      }
    } else {
      ;
    }
  }

 private:
  conf_state state_;
  std::string raw_buffer_;
  dmac::abstract_parser *parser_;
  dmac::config *config_;
  std::vector<DMACSyncPtr>::iterator initializer_it;
  rclcpp::Clock clock_;

  conf_state transition(conf_event event) {
    conf_state next_state = state_;
    switch (state_) {
      case IDLE:
        switch (event) {
          case INTERNAL:
            next_state = IDLE;
            break;
          case RCV:
            next_state = HANDLE_MODE;
            break;
          case ERROR:
            next_state = REQUEST_MODE;
            break;
        }
        break;
      case REQUEST_MODE:
        switch (event) {
          case RCV:
            next_state = HANDLE_MODE;
            break;
        }
        break;
      case HANDLE_MODE:
        switch (event) {
          case YAR:
            next_state = HANDLE_YAR;
            break;
          case WRONG_RCV:
            next_state = REQUEST_MODE;
            break;
        }
        break;
      case HANDLE_YAR:
        switch (event) {
          case YAR:
            next_state = HANDLE_YAR;
            break;
          case WRONG_RCV:
            next_state = HANDLE_YAR;
            break;
          case RCV:
            next_state = HANDLE_YAR;
            break;
          case DONE:
            next_state = FINAL;
            break;
        }
        break;
      case FINAL:
        next_state = FINAL;
        break;
        break;
      default:
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("dmac2_logger"),
                            "Alarm, unexpected state: " << state_);
        break;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"),
                       "(" << conf_state_str(state_) << ") -> "
                           << conf_event_str(event) << " -> ("
                           << conf_state_str(next_state) << ")");
    return next_state;
  }

  void handle_event(conf_event event) { handle_event<DMACSync>(event, NULL); }

  template <typename Msg>
  void handle_event(conf_event event, Msg *msg) {
    while (event != EPS) {
      state_ = transition(event);
      switch (state_) {
        case IDLE:
          event = handle_idle<Msg>(event, msg);
          break;
        case REQUEST_MODE:
          event = handle_request_mode<Msg>(event, msg);
          break;
        case HANDLE_MODE:
          event = handle_mode<Msg>(event, msg);
          break;
        case HANDLE_YAR:
          event = handle_yar<Msg>(event, msg);
          break;
        case FINAL:
          event = handle_final<Msg>(event, msg);
          break;
        default:
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("dmac2_logger"),
                              "Alarm, unexpected state: " << state_);
          break;
      }
    }
  }

  template <typename Msg>
  conf_event handle_idle(conf_event event, Msg *msg) {
    conf_event next_event = EPS;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"),
                       "Handling " << conf_event_str(event) << " in state "
                                   << conf_state_str(state_));
    switch (event) {
      case INTERNAL: {
        DMACSyncPtr sync(new DMACSync);
        sync->header.stamp = clock_.now();
        sync->command = "?MODE";
        parser_->syncCallbackWithPrivilege(sync, true);
        break;
      }
      case WRONG_RCV: {
        ;
        break;
      }
      default:
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("dmac2_logger"),
                            "Unexpected event " << conf_event_str(event)
                                                << " in state "
                                                << conf_state_str(state_));
    }
    return next_event;
  }

  template <typename Msg>
  conf_event handle_final(conf_event event, Msg *msg) {
    conf_event next_event = EPS;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"),
                       "Handling " << conf_event_str(event) << " in state "
                                   << conf_state_str(state_));
    return next_event;
  }

  template <typename Msg>
  conf_event handle_request_mode(conf_event event, Msg *msg) {
    conf_event next_event = EPS;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"),
                       "Handling " << conf_event_str(event) << " in state "
                                   << conf_state_str(state_));
    switch (event) {
      case ERROR: {
        parser_->ctrl(dmac::MODE, dmac::DMAC_COMMAND_MODE);
        DMACSyncPtr sync(new DMACSync);
        sync->header.stamp = clock_.now();
        sync->command = "?MODE";
        sync->parameters = "";
        parser_->ctrl(dmac::WAITSYNC, dmac::WAITSYNC_NO);
        parser_->syncCallbackWithPrivilege(sync, true);
        break;
      }
      default:
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("dmac2_logger"),
                            "Unexpected event " << conf_event_str(event)
                                                << " in state "
                                                << conf_state_str(state_));
    }
    return next_event;
  }

  template <typename Msg>
  conf_event handle_mode(conf_event event, Msg *msg) {
    conf_event next_event = EPS;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"),
                       "Handling " << conf_event_str(event) << " in state "
                                   << conf_state_str(state_));
    switch (event) {
      case RCV: {
        if (DMACSync *sync = dynamic_cast<DMACSync *>(msg)) {
          RCLCPP_INFO_STREAM(
              rclcpp::get_logger("dmac2_logger"),
              "command " << sync->command << ", report: " << sync->report);
          if (sync->command.compare("?MODE") == 0 &&
              sync->report.compare(0, 2, "AT") == 0) {
            DMACSyncPtr sync(new DMACSync);
            sync->header.stamp = clock_.now();
            sync->command = "O";
            sync->parameters = "";
            parser_->syncCallbackWithPrivilege(sync, true);
            parser_->ctrl(dmac::MODE, dmac::DMAC_DATA_MODE);
            next_event = YAR;
          } else if (sync->command.compare("?MODE") == 0 &&
                     sync->report.compare(0, 3, "NET") == 0) {
            parser_->ctrl(dmac::FILTER, dmac::DMAC_NET);
            next_event = YAR;
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("dmac2_logger"),
                                "WRONG_RCV: " << sync->report);
            next_event = WRONG_RCV;
          }
        } else {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("dmac2_logger"),
                              "Cast to DMACSync failed");
        }
        break;
      }
      default:
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("dmac2_logger"),
                            "Unexpected event " << conf_event_str(event)
                                                << " in state "
                                                << conf_state_str(state_));
    }
    return next_event;
  }

  template <typename Msg>
  conf_event handle_yar(conf_event event, Msg *msg) {
    conf_event next_event = EPS;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("dmac2_logger"),
                       "Handling " << conf_event_str(event) << " in state "
                                   << conf_state_str(state_));
    switch (event) {
      case YAR: {
        if (initializer_it == config_->initializer_end()) {
          next_event = DONE;
        } else {
          parser_->syncCallbackWithPrivilege(*initializer_it, true);
          initializer_it++;
        }
        break;
      }
      case RCV: {
        if (DMACSync *sync = dynamic_cast<DMACSync *>(msg)) {
          if (sync->report.compare(0, 2, "OK") == 0 ||
              sync->report.compare(0, 5, "[*]OK") == 0) {
            next_event = YAR;
          } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("dmac2_logger"),
                                "WRONG_RCV: " << sync->report);
            next_event = WRONG_RCV;
          }
        } else {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("dmac2_logger"),
                              "Failed dynamic cast of msg " << msg);
        }
        break;
      }
      default:
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("dmac2_logger"),
                            "Unexpected event " << conf_event_str(event)
                                                << " in state "
                                                << conf_state_str(state_));
    }
    return next_event;
  }
};

}  // namespace dmac

#endif /* DMAC_INITIALIZER_H */
