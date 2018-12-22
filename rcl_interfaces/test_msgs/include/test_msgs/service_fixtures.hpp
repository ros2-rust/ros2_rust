// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_MSGS__SERVICE_FIXTURES_HPP_
#define TEST_MSGS__SERVICE_FIXTURES_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "test_msgs/srv/empty.hpp"
#include "test_msgs/srv/primitives.hpp"


std::vector<
  std::pair<
    test_msgs::srv::Empty::Request::SharedPtr,
    test_msgs::srv::Empty::Response::SharedPtr
  >
>
get_services_empty()
{
  std::vector<
    std::pair<
      test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr
    >
  > services;
  auto request = std::make_shared<
    test_msgs::srv::Empty::Request>();
  auto reply = std::make_shared<
    test_msgs::srv::Empty::Response>();
  services.emplace_back(request, reply);
  return services;
}

std::vector<
  std::pair<
    test_msgs::srv::Primitives::Request::SharedPtr,
    test_msgs::srv::Primitives::Response::SharedPtr
  >
>
get_services_primitives()
{
  std::vector<
    std::pair<
      test_msgs::srv::Primitives::Request::SharedPtr,
      test_msgs::srv::Primitives::Response::SharedPtr
    >
  > services;
  {
    auto request = std::make_shared<test_msgs::srv::Primitives::Request>();
    request->bool_value = false;
    request->byte_value = 0;
    request->char_value = '\0';
    request->float32_value = 0.0f;
    request->float64_value = 0.0;
    request->int8_value = 0;
    request->uint8_value = 0;
    request->int16_value = 0;
    request->uint16_value = 0;
    request->int32_value = 0;
    request->uint32_value = 0;
    request->int64_value = 0;
    request->uint64_value = 0;
    request->string_value = "request";
    auto reply = std::make_shared<test_msgs::srv::Primitives::Response>();
    reply->bool_value = false;
    reply->byte_value = 0;
    reply->char_value = '\0';
    reply->float32_value = 0.0f;
    reply->float64_value = 0.0;
    reply->int8_value = 0;
    reply->uint8_value = 0;
    reply->int16_value = 0;
    reply->uint16_value = 0;
    reply->int32_value = 0;
    reply->uint32_value = 0;
    reply->int64_value = 0;
    reply->uint64_value = 0;
    reply->string_value = "reply";
    services.emplace_back(request, reply);
  }
  {
    auto request = std::make_shared<test_msgs::srv::Primitives::Request>();
    request->bool_value = true;
    request->byte_value = 1;
    request->char_value = 0x1;
    request->float32_value = 1.125f;
    request->float64_value = 1.11;
    request->int8_value = 1;
    request->uint8_value = 2;
    request->int16_value = 3;
    request->uint16_value = 4;
    request->int32_value = 5;
    request->uint32_value = 6;
    request->int64_value = 7;
    request->uint64_value = 8;
    // check strings longer then 256 characters
    request->string_value = "";
    for (size_t i = 0; i < 20000; ++i) {
      request->string_value += std::to_string(i % 10);
    }
    auto reply = std::make_shared<test_msgs::srv::Primitives::Response>();
    reply->bool_value = true;
    reply->byte_value = 11;
    reply->char_value = 0x11;
    reply->float32_value = 11.125f;
    reply->float64_value = 11.11;
    reply->int8_value = 11;
    reply->uint8_value = 22;
    reply->int16_value = 33;
    reply->uint16_value = 44;
    reply->int32_value = 55;
    reply->uint32_value = 66;
    reply->int64_value = 77;
    reply->uint64_value = 88;
    // check strings longer then 256 characters
    reply->string_value = "";
    for (size_t i = 0; i < 20000; ++i) {
      reply->string_value += std::to_string(i % 10);
    }
    services.emplace_back(request, reply);
  }
  return services;
}

#endif  // TEST_MSGS__SERVICE_FIXTURES_HPP_
