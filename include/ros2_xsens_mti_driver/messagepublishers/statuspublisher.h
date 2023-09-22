// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//  Copyright (c) 2003-2020 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//

#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

#include "packetcallback.h"
#include <diagnostic_msgs/msg/key_value.hpp>
#include <bitset>

using namespace std::chrono_literals;

struct RTKStatus {
    const std::pair<uint, uint> FIX = {1, 0};
    const std::pair<uint, uint> FLOATING = {0, 1};
    const std::pair<uint, uint> NONE = {0, 0};
};

struct StatusPublisher : public PacketCallback {
    rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr pub;
    std::string frame_id = DEFAULT_FRAME_ID;
    rclcpp::Node &node_handle;
    RTKStatus rtk_status;
    rclcpp::Time last_print_time;
    bool verbose;

    StatusPublisher(rclcpp::Node &node) :
            node_handle(node),
            last_print_time(0, 0, RCL_ROS_TIME) {
        int pub_queue_size = 5;

        node_handle.get_parameter("publisher_queue_size", pub_queue_size);
        pub = node_handle.create_publisher<diagnostic_msgs::msg::KeyValue>("status", pub_queue_size);
        node_handle.get_parameter("frame_id", frame_id);
        node_handle.get_parameter("verbose", verbose);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) {
        if (packet.containsStatus()) {
            diagnostic_msgs::msg::KeyValue msg;

            auto raw_bits = std::bitset<32>(packet.status());

            msg.key = "StatusWord";
            msg.value = raw_bits.to_string();

            // Only print RTK status once per second if verbose
            if (verbose && timestamp - last_print_time > 1s) {
                last_print_time = timestamp;
                std::pair<uint, uint> current_status = {raw_bits[raw_bits.size() - 4], raw_bits[raw_bits.size() - 5]};
                if (rtk_status.NONE == current_status) {
                    RCLCPP_INFO(node_handle.get_logger(), "RTK float or fix not available");
                } else if (rtk_status.FLOATING == current_status) {
                    RCLCPP_INFO(node_handle.get_logger(), "RTK float. Converging to RTK fix.");
                } else if (rtk_status.FIX == current_status) {
                    RCLCPP_INFO(node_handle.get_logger(), "RTK fix available. ");
                }
            }
            pub->publish(msg);
        }
    }
};

#endif